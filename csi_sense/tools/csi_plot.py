#!/usr/bin/env python3
"""Live view of the csi_sense receiver: subcarrier heatmap + motion score.

    pip install pyserial numpy matplotlib
    python csi_plot.py /dev/ttyACM0                      # just watch
    python csi_plot.py /dev/ttyACM0 --record walk_01.csv # also save raw lines for the dataset

Close `idf.py monitor` first: only one program can hold the serial port.
"""

import argparse
import collections
import threading

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation

WINDOW = 500          # frames on screen (~5 s at 100 Hz)
SUBCARRIERS = 64
MOTION_WINDOW = 50    # frames per motion score (~0.5 s)
MIN_AMPLITUDE = 2.0   # subcarriers weaker than this are guard/null carriers


def parse(line):
    """'CSI,<time_ms>,<rssi>,<a0 a1 ...>' -> (time_ms, rssi, amplitudes) or None."""
    parts = line.split(",")
    if len(parts) != 4 or parts[0] != "CSI":
        return None
    try:
        amps = np.array(parts[3].split(), dtype=np.float32)
        return int(parts[1]), int(parts[2]), amps
    except ValueError:
        return None


def motion_score(frames):
    """Per frame: how much the active subcarriers fluctuate over the last MOTION_WINDOW frames.

    Uses std/mean so the score does not depend on the absolute signal strength.
    """
    scores = np.zeros(len(frames))
    if len(frames) < MOTION_WINDOW:
        return scores
    active = frames.mean(axis=0) > MIN_AMPLITUDE
    if not active.any():
        return scores
    windows = np.lib.stride_tricks.sliding_window_view(frames[:, active], MOTION_WINDOW, axis=0)
    variation = windows.std(axis=2) / (windows.mean(axis=2) + 1e-6)
    scores[MOTION_WINDOW - 1:] = variation.mean(axis=1)
    return scores


class SerialReader(threading.Thread):
    def __init__(self, port, record_path):
        super().__init__(daemon=True)
        self.port = serial.Serial(port, 115200, timeout=1)
        self.record = open(record_path, "w", buffering=1) if record_path else None
        self.lock = threading.Lock()
        self.times = collections.deque(maxlen=WINDOW)
        self.frames = collections.deque(maxlen=WINDOW)
        self.rssi = 0

    def run(self):
        while True:
            raw = self.port.readline().decode(errors="ignore").strip()
            parsed = parse(raw)
            if parsed is None:
                if raw:
                    print(raw)  # firmware log lines
                continue
            if self.record:
                self.record.write(raw + "\n")
            time_ms, rssi, amps = parsed
            amps = np.pad(amps[:SUBCARRIERS], (0, max(0, SUBCARRIERS - len(amps))))
            with self.lock:
                self.times.append(time_ms)
                self.frames.append(amps)
                self.rssi = rssi

    def snapshot(self):
        with self.lock:
            return list(self.times), np.array(self.frames), self.rssi


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("port", help="serial port of the RECEIVER board, e.g. /dev/ttyACM0")
    parser.add_argument("--record", metavar="FILE", help="append raw CSI lines to FILE")
    args = parser.parse_args()

    reader = SerialReader(args.port, args.record)
    reader.start()

    fig, (ax_heat, ax_motion) = plt.subplots(2, 1, sharex=True, figsize=(10, 7),
                                             gridspec_kw={"height_ratios": [3, 1]})
    heatmap = ax_heat.imshow(np.zeros((SUBCARRIERS, WINDOW)), aspect="auto", origin="lower",
                             cmap="viridis", vmin=0, vmax=40, extent=(0, WINDOW, 0, SUBCARRIERS))
    ax_heat.set_ylabel("subcarrier")
    fig.colorbar(heatmap, ax=ax_heat, label="amplitude")
    (motion_line,) = ax_motion.plot([], [], color="tab:red")
    ax_motion.set_xlim(0, WINDOW)
    ax_motion.set_xlabel("frame (newest on the right)")
    ax_motion.set_ylabel("motion")

    def update(_):
        times, frames, rssi = reader.snapshot()
        if len(frames) < 2:
            ax_heat.set_title("waiting for CSI frames... (is the transmitter on?)")
            return heatmap, motion_line

        heatmap.set_data(frames.T)
        heatmap.set_extent((WINDOW - len(frames), WINDOW, 0, SUBCARRIERS))
        heatmap.set_clim(0, max(1.0, np.percentile(frames, 99)))

        motion = motion_score(frames)
        motion_line.set_data(np.arange(WINDOW - len(frames), WINDOW), motion)
        ax_motion.set_ylim(0, max(0.05, motion.max() * 1.2))

        span_s = (times[-1] - times[0]) / 1000
        rate = (len(times) - 1) / span_s if span_s > 0 else 0
        ax_heat.set_title(f"{rate:.0f} frames/s   RSSI {rssi} dBm   motion {motion[-1]:.3f}")
        return heatmap, motion_line

    _anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
