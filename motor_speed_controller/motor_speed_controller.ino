// Automatic Motor Speed Controller - closed-loop PID with back-EMF speed sensing
//
// The kit has no encoder, so speed is measured with the motor itself: a spinning
// DC motor is a generator, and its back-EMF is proportional to speed. Every control
// period the drive is switched off for ~1.5 ms, the inductive current dies away,
// and the voltage across the motor is measured. That voltage is "speed".
//
// Modes (button or 'mode n'):  0 open loop   1 P   2 PI   3 PID
// Tests (serial):  'step <V>' step response    'sweep' PWM-vs-speed characterisation
// Streams CSV at 115200 for ../tools/serial_logger.py and ../analysis/analyse.py.
//
// Wiring (see README.md for the diagram):
//   L293D:  EN1 (pin 1) D5, IN1 (pin 2) D4, IN2 (pin 7) GND
//           OUT1 (pin 3) -> motor -> GND        <- motor to GND, NOT to OUT2
//           VCC1 (16) 5V, VCC2 (8) motor supply (6-9 V), pins 4, 5, 12, 13 GND
//   Back-EMF sense: OUT1 -> 10k -> A1 -> 10k -> GND
//   Setpoint pot wiper -> A0          Mode button D2 -> GND
//   LCD 1602: RS D7, E D8, D4 D9, D5 D10, D6 D11, D7 D12 (RW GND, V0 -> 1k -> GND)
//   Motor supply GND must be joined to Arduino GND.

#include <LiquidCrystal.h>
#include <EEPROM.h>

// ---------- Pins ----------
const int PIN_EN = 5, PIN_IN1 = 4;
const int PIN_SENSE = A1, PIN_POT = A0;
const int PIN_BUTTON = 2;
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

// ---------- Hardware constants ----------
const float ADC_REF_V = 5.0;
const float DIVIDER_RATIO = 2.0;            // (10k + 10k) / 10k
const unsigned int BEMF_SETTLE_US = 1500;   // wait for the flyback current to decay
const byte BEMF_SAMPLES = 4;
const float SPEED_FILTER = 0.35;            // EMA smoothing (1 = none)

// ---------- Control loop ----------
const unsigned long TS_US = 20000;          // 50 Hz control loop
const float TS = TS_US / 1e6;
const int PWM_MAX = 255;
const float D_FILTER = 0.3;                 // smoothing on the derivative term

enum Mode { OPEN_LOOP, MODE_P, MODE_PI, MODE_PID, SWEEP };
const char *const MODE_NAMES[] = {"OPEN", "P", "PI", "PID", "SWP"};
Mode mode = MODE_PI;

// Tunable parameters, saved with 'save'
struct Params {
  uint16_t magic;
  float kp;        // PWM counts per volt of error
  float ki;        // PWM counts per volt-second
  float kd;        // PWM counts per volt/second
  float kf;        // feedforward: PWM counts per volt of setpoint
  float ff0;       // feedforward: PWM needed to overcome friction (deadband)
  float spMax;     // top of the setpoint range, volts of back-EMF
  float rpmPerVolt;// 0 = not calibrated
};
const uint16_t PARAMS_MAGIC = 0x3A01;
Params prm = {PARAMS_MAGIC, 40.0, 120.0, 0.8, 0.0, 0.0, 3.0, 0.0};

// ---------- State ----------
float speedV = 0;          // filtered back-EMF
float setpointV = 0;
bool setpointFromPot = true;
float integral = 0;        // PWM counts
float dTerm = 0;
float lastSpeed = 0;
int pwm = 0;
unsigned long lastStep = 0;
unsigned long stallSince = 0;
bool stalled = false;

// Test sequencer (step / sweep)
enum Test { NO_TEST, STEP_TEST, SWEEP_TEST };
Test test = NO_TEST;
unsigned long testStart = 0;
float stepLevel = 0;
float stepHold = 4.0;
Mode modeBeforeTest = MODE_PI;
const int SWEEP_STEP = 10;
const unsigned long SWEEP_HOLD_MS = 1500;

bool lastButton = HIGH;
unsigned long lastButtonChange = 0;
unsigned long lastLcd = 0;
String rxLine;

// ---------- Speed measurement ----------
float measureBackEmf() {
  analogWrite(PIN_EN, 0);                  // drive off: OUT1 goes high-impedance
  analogRead(PIN_SENSE);                   // dummy read: switch the ADC mux to this pin
  delayMicroseconds(BEMF_SETTLE_US);       // flyback current decays through the clamp diode
  unsigned int sum = 0;
  for (byte i = 0; i < BEMF_SAMPLES; i++) sum += analogRead(PIN_SENSE);
  analogWrite(PIN_EN, pwm);                // drive back on
  float v = (sum / (float)BEMF_SAMPLES) / 1023.0 * ADC_REF_V * DIVIDER_RATIO;
  return v;
}

// ---------- Controller ----------
int computeOutput() {
  float error = setpointV - speedV;

  if (mode == OPEN_LOOP) {
    integral = 0;
    return constrain((int)(setpointV / prm.spMax * PWM_MAX), 0, PWM_MAX);  // pot position straight to PWM
  }

  // Feedforward: the PWM we *expect* to need for this speed (from the sweep test)
  float ff = setpointV > 0.05 ? prm.kf * setpointV + prm.ff0 : 0;

  float p = prm.kp * error;

  // Derivative on measurement (not on error) so setpoint steps don't cause a kick,
  // low-pass filtered because differentiating a noisy signal amplifies the noise
  float rawD = -prm.kd * (speedV - lastSpeed) / TS;
  dTerm += D_FILTER * (rawD - dTerm);
  float d = (mode == MODE_PID) ? dTerm : 0;

  float u = ff + p + integral + d;

  if (mode == MODE_PI || mode == MODE_PID) {
    float candidate = integral + prm.ki * error * TS;
    float uNew = ff + p + candidate + d;
    // Anti-windup (conditional integration): stop integrating if the output is
    // saturated and the error would push it further into saturation
    bool saturatingHigh = uNew > PWM_MAX && error > 0;
    bool saturatingLow = uNew < 0 && error < 0;
    if (!saturatingHigh && !saturatingLow) {
      integral = candidate;
      u = uNew;
    }
  } else {
    integral = 0;
  }

  if (setpointV < 0.05) {  // stop means stop: don't let the integral hold the motor humming
    integral = 0;
    return 0;
  }
  return constrain((int)round(u), 0, PWM_MAX);
}

// ---------- Tests ----------
void startStep(float level, float hold) {
  modeBeforeTest = mode;
  stepLevel = constrain(level, 0, prm.spMax * 1.5);
  stepHold = hold;
  test = STEP_TEST;
  testStart = millis();
  Serial.print(F("# STEP test: 0 V for 2 s, then ")); Serial.print(stepLevel);
  Serial.print(F(" V for ")); Serial.print(stepHold); Serial.print(F(" s in mode "));
  Serial.println(MODE_NAMES[mode]);
}

void startSweep() {
  modeBeforeTest = mode;
  mode = SWEEP;
  test = SWEEP_TEST;
  testStart = millis();
  Serial.println(F("# SWEEP: open-loop PWM 0..250 in steps of 10, 1.5 s each"));
}

void endTest() {
  if (test == NO_TEST) return;
  mode = modeBeforeTest;
  test = NO_TEST;
  integral = 0;
  Serial.println(F("# test finished"));
}

// Called each control step: sets setpointV (and pwm for the sweep)
void updateSetpoint() {
  unsigned long t = millis() - testStart;
  if (test == STEP_TEST) {
    if (t < 2000) setpointV = 0;
    else if (t < 2000 + (unsigned long)(stepHold * 1000)) setpointV = stepLevel;
    else { setpointV = 0; if (t > 3000 + (unsigned long)(stepHold * 1000)) endTest(); }
  } else if (test == SWEEP_TEST) {
    int level = (t / SWEEP_HOLD_MS) * SWEEP_STEP;
    if (level > 250) { pwm = 0; endTest(); }
    else pwm = level;
    setpointV = 0;
  } else if (setpointFromPot) {
    setpointV = analogRead(PIN_POT) / 1023.0 * prm.spMax;
  }
}

// ---------- Safety ----------
void checkStall() {
  // High drive but no back-EMF for 1 s = jammed rotor (or a wiring fault). Cut power.
  if (mode == SWEEP) return;
  if (pwm > 200 && speedV < 0.15) {
    if (stallSince == 0) stallSince = millis();
    else if (millis() - stallSince > 1000 && !stalled) {
      stalled = true;
      Serial.println(F("# STALL detected - motor off. Press the button or type 'reset'."));
    }
  } else {
    stallSince = 0;
  }
}

void clearStall() {
  stalled = false;
  stallSince = 0;
  integral = 0;
  Serial.println(F("# reset"));
}

// ---------- One control step ----------
void controlStep() {
  float raw = measureBackEmf();
  lastSpeed = speedV;
  speedV += SPEED_FILTER * (raw - speedV);

  updateSetpoint();
  if (mode != SWEEP) pwm = computeOutput();
  checkStall();
  if (stalled) pwm = 0;
  analogWrite(PIN_EN, pwm);

  Serial.print(millis());        Serial.print(',');
  Serial.print((int)mode);       Serial.print(',');
  Serial.print(setpointV, 3);    Serial.print(',');
  Serial.print(speedV, 3);       Serial.print(',');
  Serial.println(pwm);
}

void sendHeader() {
  Serial.println(F("t_ms,mode,setpoint_V,speed_V,pwm"));
}

// ---------- LCD ----------
void lcdLine(int row, const char *text) {
  char padded[17];
  snprintf(padded, sizeof(padded), "%-16s", text);
  lcd.setCursor(0, row);
  lcd.print(padded);
}

void updateLcd() {
  char a[10], b[10], line[24];
  dtostrf(setpointV, 4, 2, a);
  dtostrf(speedV, 4, 2, b);
  snprintf(line, sizeof(line), "%-3s SP%s Y%s", MODE_NAMES[mode], a, b);
  lcdLine(0, line);
  if (stalled) {
    lcdLine(1, "STALL! press btn");
  } else if (prm.rpmPerVolt > 0) {
    snprintf(line, sizeof(line), "PWM%3d %5drpm", pwm, (int)(speedV * prm.rpmPerVolt));
    lcdLine(1, line);
  } else {
    snprintf(line, sizeof(line), "PWM %3d  %3d%%", pwm, (int)(pwm * 100L / 255));
    lcdLine(1, line);
  }
}

// ---------- Commands ----------
void printParams() {
  Serial.print(F("# mode=")); Serial.print(MODE_NAMES[mode]);
  Serial.print(F(" kp=")); Serial.print(prm.kp);
  Serial.print(F(" ki=")); Serial.print(prm.ki);
  Serial.print(F(" kd=")); Serial.print(prm.kd);
  Serial.print(F(" kf=")); Serial.print(prm.kf);
  Serial.print(F(" ff0=")); Serial.print(prm.ff0);
  Serial.print(F(" spmax=")); Serial.print(prm.spMax);
  Serial.print(F(" rpmpv=")); Serial.println(prm.rpmPerVolt);
}

void setMode(Mode m) {
  endTest();
  mode = m;
  integral = 0;
  dTerm = 0;
  Serial.print(F("# mode = ")); Serial.println(MODE_NAMES[mode]);
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  int s1 = cmd.indexOf(' ');
  String name = s1 < 0 ? cmd : cmd.substring(0, s1);
  String rest = s1 < 0 ? "" : cmd.substring(s1 + 1);
  float val = rest.length() ? rest.toFloat() : NAN;

  if (name == "mode" && !isnan(val))        setMode((Mode)constrain((int)val, 0, 3));
  else if (name == "kp" && !isnan(val))     prm.kp = val;
  else if (name == "ki" && !isnan(val))     { prm.ki = val; integral = 0; }
  else if (name == "kd" && !isnan(val))     prm.kd = val;
  else if (name == "kf" && !isnan(val))     prm.kf = val;
  else if (name == "ff0" && !isnan(val))    prm.ff0 = val;
  else if (name == "spmax" && val > 0)      prm.spMax = val;
  else if (name == "rpmpv" && !isnan(val))  prm.rpmPerVolt = val;
  else if (name == "sp" && !isnan(val))     { setpointV = constrain(val, 0, prm.spMax * 1.5); setpointFromPot = false; }
  else if (name == "pot")                   setpointFromPot = true;
  else if (name == "step" && !isnan(val)) {
    int s2 = rest.indexOf(' ');
    float hold = s2 < 0 ? 4.0 : constrain(rest.substring(s2 + 1).toFloat(), 1.0, 30.0);
    startStep(val, hold);
  }
  else if (name == "sweep")                 startSweep();
  else if (name == "stop")                  { endTest(); setpointFromPot = false; setpointV = 0; }
  else if (name == "reset")                 clearStall();
  else if (name == "save")                  { EEPROM.put(0, prm); Serial.println(F("# saved to EEPROM")); }
  else if (name == "status")                {}  // just print params below
  else {
    Serial.println(F("# mode <0-3> | kp/ki/kd/kf/ff0 <x> | spmax <V> | rpmpv <x> | sp <V> | pot"));
    Serial.println(F("# step <V> [hold s] | sweep | stop | reset | save | status"));
  }
  printParams();
  sendHeader();
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLine.length()) handleCommand(rxLine);
      rxLine = "";
    } else if (rxLine.length() < 40) {
      rxLine += c;
    }
  }
}

void handleButton() {
  bool b = digitalRead(PIN_BUTTON);
  if (b != lastButton && millis() - lastButtonChange > 30) {
    lastButtonChange = millis();
    lastButton = b;
    if (b == LOW) {
      if (stalled) clearStall();
      else setMode((Mode)((mode + 1) % 4));
      sendHeader();
    }
  }
}

// ---------- Setup / loop ----------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  analogWrite(PIN_EN, 0);
  digitalWrite(PIN_IN1, HIGH);   // single direction: OUT1 is driven high when EN is on
  lcd.begin(16, 2);
  lcdLine(0, "Motor speed PID");

  Params stored;
  EEPROM.get(0, stored);
  if (stored.magic == PARAMS_MAGIC) prm = stored;

  Serial.println(F("# Motor speed controller - type 'help' for commands"));
  printParams();
  sendHeader();
  lastStep = micros();
}

void loop() {
  handleButton();
  readSerial();

  if (micros() - lastStep >= TS_US) {
    lastStep += TS_US;
    if (micros() - lastStep > TS_US) lastStep = micros();  // fell behind: don't try to catch up
    controlStep();
  }

  if (millis() - lastLcd >= 250) {
    lastLcd = millis();
    updateLcd();   // ~10 ms: fits between control steps
  }
}
