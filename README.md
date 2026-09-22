# 05 · Automatic Motor Speed Controller (closed-loop PID)

A DC motor speed controller that measures its own speed and corrects itself: **PWM drive, speed feedback, P / PI / PID control, anti-windup, feedforward, stall protection**, plus Python tools to characterise the motor, fit a model, tune gains and measure step-response performance.

```
 Setpoint (pot) ──►(+)──► PID ──► PWM ──► L293D ──► Motor ──┬──► speed
                   ▲(−)                                     │
                   └──────── back-EMF measurement ◄─────────┘
```

## Sensing speed with no sensor: back-EMF

The kit has no encoder, so this project uses a technique from real sensorless drives. **A spinning DC motor is also a generator.** Its back-EMF is proportional to speed (`E = kₑ·ω`).

Every 20 ms the controller:

1. switches the drive **off** (L293D output goes high-impedance)
2. waits **1.5 ms** for the winding current to decay through the L293D's internal clamp diode
3. reads the motor terminal voltage through a 10k/10k divider. That voltage is the back-EMF, which is your speed signal
4. switches the drive back on with the new PWM value

```
      motor supply
           │
   D5 ─► EN1┤ L293D ├OUT1 ──┬──────────┐
   D4 ─► IN1│       │       │        (M) motor
            └───────┘      [10k]       │
                            ├── A1    GND
                           [10k]
                            │
                           GND
```

Speed is therefore reported in **volts of back-EMF**. To convert to RPM, see *Calibrating RPM* below.

**Things worth discussing in your write-up:**
- The measurement costs ~7.5 % of the drive time (1.5 ms of every 20 ms), so the maximum speed is slightly reduced. That's a trade-off between settle time and duty cycle.
- Brush commutation makes the signal noisy, so it is averaged over 4 ADC samples and then smoothed with an exponential filter. More filtering means less noise but more lag in the loop.
- Why the motor goes to **GND**, not to OUT2: with one side grounded, the voltage at OUT1 during the off window *is* the back-EMF.

## Control features

| Feature | Why |
|---|---|
| **P / PI / PID** selectable live | Compare them on the same hardware |
| **Derivative on measurement** | Avoids a "derivative kick" when the setpoint jumps |
| **Filtered derivative** | Differentiating a noisy signal amplifies the noise |
| **Anti-windup** (conditional integration) | Stops the integral growing while the output is saturated |
| **Feedforward** `kf·SP + ff0` | Supplies most of the required PWM straight away, including the friction **deadband**, so the PID only corrects the remainder |
| **Stop means stop** | Setpoint 0 → output 0 and integral cleared, so the motor doesn't hum |
| **Stall detection** | High PWM + no back-EMF for 1 s → motor off until reset |
| **Fixed 50 Hz loop** | Timed with `micros()`, and the LCD update fits between steps |

## Parts

| Qty | Part |
|---|---|
| 1 | L293D |
| 1 | DC motor + fan blade |
| 2 | 10 kΩ (back-EMF divider) |
| 1 | 10 kΩ potentiometer (setpoint) |
| 1 | Push button (mode) |
| 1 | LCD1602 + 220 Ω + 1 kΩ |
| 1 | Breadboard power module **or** 9 V battery for the motor |

## Wiring

| Arduino | Connects to |
|---|---|
| D5 | L293D **EN1** (pin 1), PWM |
| D4 | L293D **IN1** (pin 2), held HIGH. IN2 (pin 7) → GND |
| – | **OUT1 (pin 3) → motor → GND** (motor does *not* go to OUT2) |
| – | L293D VCC1 (16) → 5V, VCC2 (8) → motor supply, pins 4/5/12/13 → GND |
| A1 | OUT1 → 10 kΩ → **A1** → 10 kΩ → GND |
| A0 | Setpoint potentiometer wiper |
| D2 | Mode button → GND |
| D7, D8, D9–D12 | LCD RS, E, D4–D7 (RW → GND, V0 → 1 kΩ → GND) |

> ⚠️ Keep the motor supply **≤ 9 V**. With the 10k/10k divider, A1 must never see more than 5 V. **Join the motor supply ground to Arduino GND.**

## Workflow: characterise → model → tune → verify

Start the logger in one terminal. Everything below is typed into its console.

```bash
python tools/serial_logger.py COM3 --cols setpoint_V speed_V pwm
```

### 1. Static characteristic (sweep)

Type `sweep`. The motor steps through PWM 0 → 250 in steps of 10, holding 1.5 s at each. Stop the logger, then:

```bash
python analysis/analyse.py sweep logs/<file>.csv
```

You get a speed-vs-PWM plot with a linear fit, the **deadband** (PWM where the motor starts turning) and suggested `kf`, `ff0` and `spmax` values. Type them in, then `save`.

### 2. Dynamic model (open-loop step)

`mode 0`, then `step 2` (0 V for 2 s, then a step). Analyse it:

```bash
python analysis/analyse.py step logs/<file>.csv
```

This fits a first-order model **G(s) = K / (τs + 1)** and prints PI gains from the **SIMC tuning rules** for three closed-loop speeds.

### 3. Closed-loop tuning

Put the SIMC gains in (`kp …`, `ki …`), then run a step test in each mode:

```
mode 1      step 2
mode 2      step 2
mode 3      step 2
```

Save each run as a separate log, then compare them:

```bash
python analysis/analyse.py compare logs/p.csv logs/pi.csv logs/pid.csv --save
```

This prints a table of **rise time, overshoot, settling time and steady-state error**, and saves `comparison.png`. That figure is the one for the top of your GitHub README.

### 4. Disturbance rejection

Run at a steady setpoint in PI mode and gently load the fan (touch the hub with a soft cloth). Watch the PWM rise to hold the speed. Compare with open-loop mode, where the speed just drops.

## Simulation (try this first, no hardware needed)

`analysis/simulate.py` is a line-for-line Python port of the Arduino controller running against a modelled motor (first-order + deadband + measurement noise):

```bash
cd analysis
python simulate.py                    # writes sim_logs/ step_p, step_pi, step_pid, load_pi, sweep
python analyse.py compare sim_logs/step_p.csv sim_logs/step_pi.csv sim_logs/step_pid.csv
```

With the default gains and **no feedforward**, the simulation shows P-only control sitting at ~0.4 V for a 2 V setpoint, an 80 % error. That isn't a bug. Proportional control needs an error to produce any output, and the motor's deadband means a *lot* of output is needed before it moves at all. Adding the integral, or the feedforward from the sweep, fixes it. It makes a strong point to show in your write-up.

Once you've identified your real motor, run `simulate.py --K <K> --tau <tau> --deadband <d>` and plot **simulation vs measurement** on the same axes. Model validation like that is what makes a project look like engineering.

## Serial commands

| Command | Effect |
|---|---|
| `mode 0/1/2/3` | Open loop / P / PI / PID (or press the button) |
| `kp`, `ki`, `kd` `<x>` | PID gains (PWM counts per V, per V·s, per V/s) |
| `kf`, `ff0` `<x>` | Feedforward slope and deadband offset |
| `spmax <V>` | Setpoint at full pot rotation |
| `sp <V>` / `pot` | Fixed setpoint / back to the pot |
| `step <V> [hold s]` | Automated step test |
| `sweep` | Automated PWM sweep |
| `stop` | Abort test, setpoint 0 |
| `reset` | Clear a stall |
| `save` | Store the parameters in EEPROM |
| `status` | Print the parameters |

## Calibrating RPM (optional)

Run at a steady speed and measure the real RPM with a phone tachometer or strobe app, or a spectrum-analyser app listening to the fan (blade-pass frequency ÷ number of blades × 60). Then type `rpmpv <RPM ÷ speed_V>` and `save`, and the LCD will show RPM.

## Ideas to extend it

- Add an optical encoder (slotted disc + IR LED/phototransistor on interrupt pin D3) and compare it with back-EMF sensing. How accurate is the sensorless method?
- Bidirectional control using OUT2 and a full H-bridge. Measuring back-EMF gets harder, which is a good challenge.
- Model the motor electrically (measure R and L of the winding) and derive τ from first principles
- Cascade control: an inner current loop (shunt resistor) and an outer speed loop
