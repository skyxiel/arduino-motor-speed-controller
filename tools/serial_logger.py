"""
Serial logger + live plotter for every project in this folder.

All sketches speak the same simple protocol over USB serial:
  - lines starting with '#' are messages -> printed to the console
  - the first other line is a CSV header, e.g.  t_ms,voltage_V,temp_C
  - every line after that is a CSV row of numbers

Usage:
  python serial_logger.py --list                       # show serial ports
  python serial_logger.py COM3                          # log + live plot all columns
  python serial_logger.py COM3 --cols temp_C setpoint_C # plot only some columns
  python serial_logger.py COM3 --no-plot                # just log to CSV
  python serial_logger.py COM3 --send "step 2.5"        # send a command after connecting

Type a line into the console while it runs to send it to the Arduino
(e.g. 'kp 40' for the motor controller). Close the plot window or Ctrl+C to stop.
The CSV is saved in ./logs/ with a timestamped name.
"""

import argparse
import csv
import datetime as dt
import queue
import sys
import threading
import time
from collections import deque
from pathlib import Path

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial is missing:  pip install -r requirements.txt")


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found. Is the Arduino plugged in?")
    for p in ports:
        print(f"{p.device:10s} {p.description}")


class SerialReader(threading.Thread):
    """Reads lines in the background, writes CSV, queues rows for plotting."""

    def __init__(self, ser, out_path, max_points):
        super().__init__(daemon=True)
        self.ser = ser
        self.out_path = out_path
        self.header = None
        self.data = None  # dict column -> deque
        self.max_points = max_points
        self.rows = queue.Queue()
        self.running = True
        self.plotting = True
        self.header_ready = threading.Event()

    def run(self):
        with open(self.out_path, "w", newline="") as f:
            writer = csv.writer(f)
            header_written = False
            while self.running:
                try:
                    raw = self.ser.readline()
                except serial.SerialException as e:
                    print(f"Serial error: {e}")
                    break
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").strip()
                if not line:
                    continue
                if line.startswith("#"):
                    print(line)
                    continue
                parts = [p.strip() for p in line.split(",")]
                if not _is_numeric(parts):
                    # A non-numeric line is a header. Sketches repeat it after commands;
                    # only a *different* header (e.g. another sketch) starts new columns.
                    if parts != self.header:
                        self.header = parts
                        self.data = {c: deque(maxlen=self.max_points) for c in parts}
                        header_written = False
                        print("Columns:", ", ".join(parts))
                        self.header_ready.set()
                    continue
                if self.header is None or len(parts) != len(self.header):
                    continue  # partial line from a reset / garbled start
                if not header_written:
                    # Written lazily so a garbled line at start-up never lands in the CSV
                    writer.writerow(self.header)
                    header_written = True
                writer.writerow(parts)
                f.flush()
                if self.plotting:
                    self.rows.put([float(p) for p in parts])


def _is_numeric(parts):
    try:
        [float(p) for p in parts]
        return True
    except ValueError:
        return False


def stdin_sender(ser):
    """Forward whatever the user types to the Arduino."""
    for line in sys.stdin:
        try:
            ser.write((line.strip() + "\n").encode("ascii"))
        except serial.SerialException:
            break


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", help="e.g. COM3 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--cols", nargs="*", help="columns to plot (default: all except the first)")
    ap.add_argument("--window", type=int, default=600, help="points shown in the live plot")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--send", action="append", default=[], help="command to send after connecting")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--out", help="CSV output path (default logs/<timestamp>.csv)")
    args = ap.parse_args()

    if args.list or not args.port:
        list_ports()
        return

    out = Path(args.out) if args.out else Path("logs") / f"{dt.datetime.now():%Y%m%d_%H%M%S}.csv"
    out.parent.mkdir(parents=True, exist_ok=True)

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    print(f"Connected to {args.port} @ {args.baud}. Logging to {out}")
    time.sleep(2.0)  # the UNO resets when the port opens

    reader = SerialReader(ser, out, max_points=args.window)
    reader.plotting = not args.no_plot
    reader.start()
    threading.Thread(target=stdin_sender, args=(ser,), daemon=True).start()
    for cmd in args.send:
        ser.write((cmd + "\n").encode("ascii"))

    try:
        if args.no_plot:
            while reader.is_alive():
                time.sleep(0.5)
        else:
            live_plot(reader, args.cols)
    except KeyboardInterrupt:
        pass
    finally:
        reader.running = False
        time.sleep(0.3)
        ser.close()
        print(f"Saved {out}")


def live_plot(reader, wanted_cols):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    print("Waiting for CSV header...")
    while not reader.header_ready.wait(0.5):
        if not reader.is_alive():
            return

    fig, ax = plt.subplots(figsize=(10, 5))
    lines = {}
    state = {"header": None}

    def setup_lines():
        ax.clear()
        lines.clear()
        header = reader.header
        cols = wanted_cols or header[1:]
        for c in cols:
            if c in header:
                (lines[c],) = ax.plot([], [], label=c)
        ax.set_xlabel(header[0])
        ax.legend(loc="upper left")
        ax.grid(True, alpha=0.3)
        state["header"] = header

    def update(_):
        if state["header"] is not reader.header:
            setup_lines()
        header = state["header"]
        while not reader.rows.empty():
            row = reader.rows.get()
            for c, v in zip(header, row):
                reader.data[c].append(v)
        x = list(reader.data[header[0]])
        if header[0] == "t_ms":
            x = [v / 1000.0 for v in x]
            ax.set_xlabel("time (s)")
        for c, ln in lines.items():
            ln.set_data(x, list(reader.data[c]))
        if x:
            ax.relim()
            ax.autoscale_view()
        return list(lines.values())

    _anim = FuncAnimation(fig, update, interval=200, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
