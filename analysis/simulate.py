"""
Simulate the motor + the exact controller in the Arduino sketch, and write logs in
the same CSV format. Use it to:
  - try gains safely before touching hardware
  - compare simulation against your real measurements (a great portfolio section)
  - test analyse.py without an Arduino

  python simulate.py                       # P, PI and PID step tests + a sweep into sim_logs/
  python simulate.py --K 0.02 --tau 0.3    # use the plant you identified
  python simulate.py --kp 60 --ki 200 --kd 0 --mode 2

Then:  python analyse.py compare sim_logs/step_p.csv sim_logs/step_pi.csv sim_logs/step_pid.csv
"""

import argparse
import csv
from pathlib import Path

import numpy as np

TS = 0.02          # control period, s (TS_US in the sketch)
PWM_MAX = 255
SPEED_FILTER = 0.35
D_FILTER = 0.3


class Motor:
    """First-order DC motor with a friction deadband: tau*dw/dt = K*max(0, pwm - dead) - w"""

    def __init__(self, K, tau, deadband, noise, rng):
        self.K, self.tau, self.dead, self.noise, self.rng = K, tau, deadband, noise, rng
        self.w = 0.0  # speed, expressed as back-EMF volts

    def step(self, pwm, dt, load=0.0):
        # The drive is off for ~1.5 ms of each 20 ms period while back-EMF is measured
        effective = pwm * (1 - 0.0015 / TS)
        target = self.K * max(0.0, effective - self.dead) - load
        target = max(target, 0.0)
        sub = 10
        for _ in range(sub):
            self.w += (target - self.w) * (dt / sub) / self.tau
        return self.w

    def measure(self):
        return max(0.0, self.w + self.rng.normal(0, self.noise))


class Controller:
    """Line-for-line port of computeOutput() in motor_speed_controller.ino"""

    def __init__(self, mode, kp, ki, kd, kf, ff0, spmax):
        self.mode, self.kp, self.ki, self.kd, self.kf, self.ff0, self.spmax = mode, kp, ki, kd, kf, ff0, spmax
        self.integral = 0.0
        self.dterm = 0.0

    def output(self, sp, y, y_last):
        error = sp - y
        if self.mode == 0:
            return int(np.clip(sp / self.spmax * PWM_MAX, 0, PWM_MAX))
        ff = self.kf * sp + self.ff0 if sp > 0.05 else 0.0
        p = self.kp * error
        raw_d = -self.kd * (y - y_last) / TS
        self.dterm += D_FILTER * (raw_d - self.dterm)
        d = self.dterm if self.mode == 3 else 0.0
        u = ff + p + self.integral + d
        if self.mode in (2, 3):
            cand = self.integral + self.ki * error * TS
            u_new = ff + p + cand + d
            if not ((u_new > PWM_MAX and error > 0) or (u_new < 0 and error < 0)):
                self.integral = cand
                u = u_new
        else:
            self.integral = 0.0
        if sp < 0.05:
            self.integral = 0.0
            return 0
        return int(np.clip(round(u), 0, PWM_MAX))


def run_step(args, mode, level, out, load_at=None):
    rng = np.random.default_rng(1)
    motor = Motor(args.K, args.tau, args.deadband, args.noise, rng)
    ctl = Controller(mode, args.kp, args.ki, args.kd, args.kf, args.ff0, args.spmax)
    speed = last = 0.0
    pwm = 0
    rows = []
    n = int((2.0 + args.hold + 1.0) / TS)
    for k in range(n):
        t = k * TS
        sp = level if 2.0 <= t < 2.0 + args.hold else 0.0
        load = args.load if (load_at and t >= load_at and t < 2.0 + args.hold) else 0.0
        motor.step(pwm, TS, load)
        last = speed
        speed += SPEED_FILTER * (motor.measure() - speed)
        pwm = ctl.output(sp, speed, last)
        rows.append((int(t * 1000), mode, round(sp, 3), round(speed, 3), pwm))
    write(out, rows)


def run_sweep(args, out):
    rng = np.random.default_rng(2)
    motor = Motor(args.K, args.tau, args.deadband, args.noise, rng)
    speed = 0.0
    rows = []
    t = 0.0
    for pwm in range(0, 251, 10):
        for _ in range(int(1.5 / TS)):
            motor.step(pwm, TS)
            speed += SPEED_FILTER * (motor.measure() - speed)
            rows.append((int(t * 1000), 4, 0.0, round(speed, 3), pwm))
            t += TS
    write(out, rows)


def write(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "mode", "setpoint_V", "speed_V", "pwm"])
        w.writerows(rows)
    print(f"wrote {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--K", type=float, default=0.025, help="plant gain, V per PWM count")
    ap.add_argument("--tau", type=float, default=0.25, help="plant time constant, s")
    ap.add_argument("--deadband", type=float, default=45, help="PWM needed before the motor turns")
    ap.add_argument("--noise", type=float, default=0.04, help="back-EMF measurement noise, V rms")
    ap.add_argument("--kp", type=float, default=40)
    ap.add_argument("--ki", type=float, default=120)
    ap.add_argument("--kd", type=float, default=0.8)
    ap.add_argument("--kf", type=float, default=0)
    ap.add_argument("--ff0", type=float, default=0)
    ap.add_argument("--spmax", type=float, default=3.0)
    ap.add_argument("--level", type=float, default=2.0, help="step size, V")
    ap.add_argument("--hold", type=float, default=4.0)
    ap.add_argument("--load", type=float, default=0.8, help="load disturbance (V of speed lost) applied mid-step")
    ap.add_argument("--mode", type=int, choices=[0, 1, 2, 3], help="only simulate this mode")
    ap.add_argument("--out", default="sim_logs")
    args = ap.parse_args()

    out = Path(args.out)
    names = {0: "open", 1: "p", 2: "pi", 3: "pid"}
    modes = [args.mode] if args.mode is not None else [0, 1, 2, 3]
    for m in modes:
        run_step(args, m, args.level, out / f"step_{names[m]}.csv")
    if args.mode is None:
        run_step(args, 2, args.level, out / "load_pi.csv", load_at=4.0)
        run_sweep(args, out / "sweep.csv")


if __name__ == "__main__":
    main()
