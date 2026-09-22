"""
Plot a CSV saved by serial_logger.py: one subplot per column, time on the x axis.

Usage:
  python plot_log.py logs/20260922_101500.csv
  python plot_log.py logs/run.csv --cols voltage_V temp_C --save run.png
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--cols", nargs="*", help="columns to plot (default: all except time)")
    ap.add_argument("--save", help="save the figure to this file instead of showing it")
    args = ap.parse_args()

    data = np.genfromtxt(args.csv, delimiter=",", names=True)
    names = list(data.dtype.names)
    xname = names[0]
    x = data[xname]
    xlabel = xname
    if xname == "t_ms":
        x = (x - x[0]) / 1000.0
        xlabel = "time (s)"

    cols = args.cols or names[1:]
    fig, axes = plt.subplots(len(cols), 1, sharex=True, figsize=(10, 2.2 * len(cols)), squeeze=False)
    for ax, c in zip(axes[:, 0], cols):
        y = data[c]
        ax.plot(x, y, lw=1)
        ax.set_ylabel(c)
        ax.grid(True, alpha=0.3)
        finite = y[np.isfinite(y)]
        if finite.size:
            ax.set_title(f"{c}: mean {finite.mean():.4g}, std {finite.std():.3g}, "
                         f"min {finite.min():.4g}, max {finite.max():.4g}", fontsize=9, loc="left")
    axes[-1, 0].set_xlabel(xlabel)
    fig.tight_layout()
    if args.save:
        fig.savefig(args.save, dpi=150)
        print(f"Saved {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
