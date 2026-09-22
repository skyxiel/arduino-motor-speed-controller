"""
Analyse logs from the motor speed controller (CSV from tools/serial_logger.py).

  python analyse.py sweep logs/sweep.csv
      PWM -> speed characteristic. Fits the linear region and prints the
      feedforward gains (kf, ff0) and a sensible spmax.

  python analyse.py step logs/step_pi.csv
      Step-response metrics: rise time, overshoot, settling time, steady-state error.
      For an OPEN-LOOP step (mode 0) it also fits a first-order model
      K / (tau s + 1) and suggests PI gains using the SIMC tuning rules.

  python analyse.py compare logs/step_p.csv logs/step_pi.csv logs/step_pid.csv
      Overlays several step responses on one graph - the headline portfolio figure.

Add --save to write PNGs next to the CSV instead of opening windows.
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

MODE_NAMES = {0: "open loop", 1: "P", 2: "PI", 3: "PID", 4: "sweep"}


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    needed = {"t_ms", "mode", "setpoint_V", "speed_V", "pwm"}
    missing = needed - set(d.dtype.names)
    if missing:
        sys.exit(f"{path}: missing columns {missing}. Is this a motor controller log?")
    d = d[np.isfinite(d["t_ms"])]
    return d


def finish(fig, save_path):
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"Saved {save_path}")
    else:
        plt.show()


# ---------------------------------------------------------------- sweep
def analyse_sweep(path, save):
    d = load(path)
    d = d[d["mode"] == 4]
    if d.size == 0:
        sys.exit("No sweep rows (mode 4) found. Run 'sweep' on the Arduino while logging.")

    pwms, speeds = [], []
    for p in np.unique(d["pwm"]):
        rows = d[d["pwm"] == p]
        t = rows["t_ms"]
        settled = rows[t >= t.min() + 0.5 * (t.max() - t.min())]  # last half of the plateau
        pwms.append(p)
        speeds.append(settled["speed_V"].mean())
    pwms, speeds = np.array(pwms), np.array(speeds)

    vmax = speeds.max()
    lin = speeds > 0.1 * vmax
    if lin.sum() < 3:
        sys.exit("Motor barely turned during the sweep - check wiring and motor supply.")
    m, c = np.polyfit(pwms[lin], speeds[lin], 1)  # speed = m*pwm + c
    deadband = -c / m
    kf = 1.0 / m

    print(f"Max speed           : {vmax:.3f} V back-EMF at PWM {pwms[speeds.argmax()]:.0f}")
    print(f"Linear fit          : speed = {m:.5f} * PWM {c:+.3f}   (V)")
    print(f"Deadband (starts at): PWM ~ {deadband:.0f}")
    print("\nSuggested settings (type these, then 'save'):")
    print(f"  kf {kf:.1f}")
    print(f"  ff0 {max(deadband, 0):.0f}")
    print(f"  spmax {0.8 * vmax:.2f}      # 80% of max leaves headroom for the controller")

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(pwms, speeds, "o", label="measured (plateau mean)")
    x = np.linspace(max(deadband, 0), pwms.max(), 50)
    ax.plot(x, m * x + c, "--", label=f"fit: {m * 1000:.2f} mV/count, deadband {deadband:.0f}")
    ax.set_xlabel("PWM (0-255)")
    ax.set_ylabel("speed (back-EMF, V)")
    ax.set_title("Motor static characteristic")
    ax.grid(True, alpha=0.3)
    ax.legend()
    finish(fig, save and Path(path).with_suffix(".sweep.png"))


# ---------------------------------------------------------------- step
def find_step(d):
    """Return (t0 index, end index) of the last rising step in setpoint."""
    sp = d["setpoint_V"]
    jumps = np.where(np.diff(sp) > 0.1)[0]
    if jumps.size == 0:
        return None
    i0 = jumps[-1] + 1
    later = np.where(np.abs(np.diff(sp[i0:])) > 0.05)[0]
    i1 = i0 + later[0] + 1 if later.size else len(sp)
    return i0, i1


def step_metrics(t, y, y0, sp, settle_band=0.05):
    """Classic metrics for a response y(t) starting at y0 and heading to sp."""
    n_end = max(3, len(y) // 5)
    yss = y[-n_end:].mean()               # final value = mean of last 20%
    span = yss - y0
    out = {"final": yss, "ss_error": sp - yss, "ss_error_pct": 100 * (sp - yss) / sp if sp else np.nan}
    if abs(span) < 1e-6:
        return out
    frac = (y - y0) / span
    try:
        t10 = t[np.argmax(frac >= 0.1)]
        t90 = t[np.argmax(frac >= 0.9)]
        out["rise_time"] = t90 - t10 if frac.max() >= 0.9 else np.nan
    except ValueError:
        out["rise_time"] = np.nan
    out["overshoot_pct"] = max(0.0, (y.max() - yss) / span * 100)
    # Settling time: after the last sample outside the band around the final value
    outside = np.where(np.abs(y - yss) > settle_band * abs(span))[0]
    if outside.size == 0:
        out["settling_time"] = 0.0
    elif outside[-1] + 1 < len(t):
        out["settling_time"] = t[outside[-1] + 1]
    else:
        out["settling_time"] = np.nan  # never settled
    # 63.2% time = first-order time constant (meaningful for open-loop steps)
    out["t63"] = t[np.argmax(frac >= 0.632)] if frac.max() >= 0.632 else np.nan
    return out


def get_step(path):
    d = load(path)
    found = find_step(d)
    if not found:
        sys.exit(f"{path}: no setpoint step found. Use 'step <V>' on the Arduino while logging.")
    i0, i1 = found
    pre = d[max(0, i0 - 25):i0]
    seg = d[i0:i1]
    t = (seg["t_ms"] - seg["t_ms"][0]) / 1000.0
    return d, seg, t, pre


def analyse_step(path, save):
    d, seg, t, pre = get_step(path)
    mode = int(np.median(seg["mode"]))
    sp = seg["setpoint_V"][0]
    y = seg["speed_V"]
    y0 = pre["speed_V"].mean() if pre.size else y[0]
    m = step_metrics(t, y, y0, sp)

    print(f"Mode: {MODE_NAMES.get(mode, mode)}   step to {sp:.3f} V")
    print(f"  final value        : {m['final']:.3f} V")
    if mode != 0:
        print(f"  steady-state error : {m['ss_error']:+.3f} V ({m['ss_error_pct']:+.1f} %)")
    print(f"  rise time 10-90%   : {m.get('rise_time', np.nan):.3f} s")
    print(f"  overshoot          : {m.get('overshoot_pct', np.nan):.1f} %")
    print(f"  settling time (5%) : {m.get('settling_time', np.nan):.3f} s")

    if mode == 0:
        # Open loop: the input step is in PWM, the output in volts
        pwm_step = seg["pwm"].mean() - (pre["pwm"].mean() if pre.size else 0)
        K = (m["final"] - y0) / pwm_step if pwm_step else np.nan
        tau = m["t63"]
        print("\nFirst-order model  G(s) = K / (tau*s + 1)")
        print(f"  K   = {K:.5f} V per PWM count")
        print(f"  tau = {tau:.3f} s")
        if np.isfinite(K) and np.isfinite(tau) and K > 0:
            # SIMC (Skogestad) PI rules for a first-order plant with an effective delay
            # of about one sample + half the measurement period; tau_c = closed-loop target
            theta = 0.03
            for tau_c in (tau, tau / 2, tau / 4):
                kp = tau / (K * (tau_c + theta))
                ti = min(tau, 4 * (tau_c + theta))
                print(f"  SIMC PI for tau_c={tau_c:.3f}s:  kp {kp:.1f}   ki {kp / ti:.1f}")
            print("  (start with the slowest, safest one)")

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(9, 6), gridspec_kw={"height_ratios": [3, 1]})
    ax1.plot(t, seg["setpoint_V"], "k--", lw=1, label="setpoint")
    ax1.plot(t, y, label="speed")
    ax1.axhline(m["final"], color="gray", lw=0.5)
    ax1.set_ylabel("back-EMF (V)")
    ax1.set_title(f"Step response - {MODE_NAMES.get(mode, mode)}   "
                  f"rise {m.get('rise_time', np.nan):.2f}s, overshoot {m.get('overshoot_pct', np.nan):.0f}%, "
                  f"settle {m.get('settling_time', np.nan):.2f}s")
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax2.plot(t, seg["pwm"], color="tab:orange")
    ax2.set_ylabel("PWM")
    ax2.set_xlabel("time since step (s)")
    ax2.grid(True, alpha=0.3)
    finish(fig, save and Path(path).with_suffix(".step.png"))


def compare(paths, save):
    fig, ax = plt.subplots(figsize=(9, 5))
    rows = []
    for p in paths:
        d, seg, t, pre = get_step(p)
        mode = int(np.median(seg["mode"]))
        sp = seg["setpoint_V"][0]
        y0 = pre["speed_V"].mean() if pre.size else seg["speed_V"][0]
        m = step_metrics(t, seg["speed_V"], y0, sp)
        label = f"{Path(p).stem} ({MODE_NAMES.get(mode, mode)})"
        ax.plot(t, seg["speed_V"], label=label)
        rows.append((label, m))
    ax.plot(t, seg["setpoint_V"], "k--", lw=1, label="setpoint")
    ax.set_xlabel("time since step (s)")
    ax.set_ylabel("speed (back-EMF, V)")
    ax.set_title("Step response comparison")
    ax.grid(True, alpha=0.3)
    ax.legend()

    print(f"{'run':40s} {'rise s':>7s} {'OS %':>6s} {'settle s':>9s} {'ss err %':>9s}")
    for label, m in rows:
        print(f"{label:40s} {m.get('rise_time', np.nan):7.3f} {m.get('overshoot_pct', np.nan):6.1f} "
              f"{m.get('settling_time', np.nan):9.3f} {m['ss_error_pct']:9.1f}")
    finish(fig, save and Path(paths[0]).with_name("comparison.png"))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=["sweep", "step", "compare"])
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--save", action="store_true", help="save PNGs instead of showing windows")
    args = ap.parse_args()
    if args.what == "sweep":
        analyse_sweep(args.csv[0], args.save)
    elif args.what == "step":
        analyse_step(args.csv[0], args.save)
    else:
        compare(args.csv, args.save)


if __name__ == "__main__":
    main()
