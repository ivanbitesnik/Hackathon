#!/usr/bin/env python3
"""Live view of the csi_sense receiver: subcarrier heatmap + motion/presence scores.

    sudo apt install python3-serial python3-numpy python3-matplotlib
    python3 csi_plot.py /dev/ttyACM0                      # just watch
    python3 csi_plot.py /dev/ttyACM0 --record walk_01.csv # also save raw lines for the dataset

On start (and whenever you press 'c' in the plot window) it calibrates for a few seconds:
keep the space between the boards EMPTY while it does. Scores are then shown in sigmas,
i.e. multiples of the empty-room noise, so the same scale works at any distance.

Close `idf.py monitor` first: only one program can hold the serial port.
"""

import argparse
import collections
import threading

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation
from numpy.lib.stride_tricks import sliding_window_view

WINDOW = 500              # frames on screen (~5 s at 100 Hz)
SUBCARRIERS = 64
MOTION_WINDOW = 50        # frames per score (~0.5 s)
MIN_AMPLITUDE = 2.0       # subcarriers weaker than this are guard/null carriers
CALIBRATION_FRAMES = 300  # ~3 s of empty room
AMPLITUDE_MAX = 25        # heatmap colour scale (still air reads ~10-16)
SIGMA_MAX = 20            # score axis, in multiples of the empty-room noise
SIGMA_NOISE = 3           # below this line is indistinguishable from noise


def parse(line):
    """Parse CSI rows, optionally including idle and walk mean amplitudes."""
    parts = line.split(",")
    if parts[0] != "CSI" or len(parts) not in (4, 6):
        return None
    try:
        amps = np.array(parts[3].split(), dtype=np.float32)
        idle = np.array(parts[4].split(), dtype=np.float32) if len(parts) == 6 else None
        walk = np.array(parts[5].split(), dtype=np.float32) if len(parts) == 6 else None
        return int(parts[1]), int(parts[2]), amps, idle, walk
    except ValueError:
        return None


def normalize(frames, active):
    """Keep only the shape across subcarriers: dividing by each frame's mean cancels
    the receiver's automatic gain changes, which otherwise look like motion."""
    shape = frames[:, active]
    return shape / (shape.mean(axis=1, keepdims=True) + 1e-6)


def smooth(values):
    """Moving average over MOTION_WINDOW, aligned so output[i] covers frames up to i."""
    return sliding_window_view(values, MOTION_WINDOW).mean(axis=1)


class Baseline:
    """Empty-room statistics; turns raw scores into sigmas above the noise floor."""

    def __init__(self):
        self.ready = False

    def fit(self, frames):
        self.active = frames.mean(axis=0) > MIN_AMPLITUDE
        shape = normalize(frames, self.active)
        self.mean = shape.mean(axis=0)
        self.std = shape.std(axis=0) + 1e-3
        motion, presence = self._raw_scores(shape)
        self.motion_floor = (motion.mean(), motion.std() + 1e-6)
        self.presence_floor = (presence.mean(), presence.std() + 1e-6)
        self.ready = True

    def _raw_scores(self, shape):
        # motion: how much each subcarrier fluctuates within the window
        motion = sliding_window_view(shape, MOTION_WINDOW, axis=0).std(axis=2).mean(axis=1)
        # presence: how far the signal sits from the empty room, even if nothing moves
        presence = smooth((np.abs(shape - self.mean) / self.std).mean(axis=1))
        return motion, presence

    def scores(self, frames):
        """Per frame (motion, presence) in sigmas; the first MOTION_WINDOW-1 frames are 0."""
        motion = np.zeros(len(frames))
        presence = np.zeros(len(frames))
        if len(frames) >= MOTION_WINDOW:
            raw_motion, raw_presence = self._raw_scores(normalize(frames, self.active))
            motion[MOTION_WINDOW - 1:] = (raw_motion - self.motion_floor[0]) / self.motion_floor[1]
            presence[MOTION_WINDOW - 1:] = (raw_presence - self.presence_floor[0]) / self.presence_floor[1]
        return np.clip(motion, 0, None), np.clip(presence, 0, None)


class SerialReader(threading.Thread):
    def __init__(self, port, record_path):
        super().__init__(daemon=True)
        self.port = serial.Serial(port, 115200, timeout=1)
        self.record = open(record_path, "w", buffering=1) if record_path else None
        self.lock = threading.Lock()
        self.times = collections.deque(maxlen=WINDOW)
        self.frames = collections.deque(maxlen=WINDOW)
        self.idle_averages = collections.deque(maxlen=WINDOW)
        self.walk_averages = collections.deque(maxlen=WINDOW)
        self.rssi = 0
        self.total = 0  # frames received since start

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
            time_ms, rssi, amps, idle, walk = parsed
            amps = np.pad(amps[:SUBCARRIERS], (0, max(0, SUBCARRIERS - len(amps))))
            if idle is not None:
                idle = np.pad(idle[:SUBCARRIERS], (0, max(0, SUBCARRIERS - len(idle))))
                walk = np.pad(walk[:SUBCARRIERS], (0, max(0, SUBCARRIERS - len(walk))))
            with self.lock:
                self.times.append(time_ms)
                self.frames.append(amps)
                self.idle_averages.append(idle)
                self.walk_averages.append(walk)
                self.rssi = rssi
                self.total += 1

    def snapshot(self):
        with self.lock:
            return (list(self.times), np.array(self.frames), list(self.idle_averages),
                    list(self.walk_averages),
                    self.rssi, self.total)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("port", help="serial port of the RECEIVER board, e.g. /dev/ttyACM0")
    parser.add_argument("--record", metavar="FILE", help="append raw CSI lines to FILE")
    args = parser.parse_args()

    reader = SerialReader(args.port, args.record)
    reader.start()
    baseline = Baseline()
    calibration_start = {"total": 0}

    fig, (ax_heat, ax_score) = plt.subplots(2, 1, sharex=True, figsize=(10, 7),
                                            gridspec_kw={"height_ratios": [3, 2]})
    heatmap = ax_heat.imshow(np.zeros((SUBCARRIERS, WINDOW)), aspect="auto", origin="lower",
                             cmap="viridis", vmin=0, vmax=AMPLITUDE_MAX, extent=(0, WINDOW, 0, SUBCARRIERS))
    ax_heat.set_ylabel("subcarrier")
    fig.colorbar(heatmap, ax=ax_heat, label="amplitude")
    (motion_line,) = ax_score.plot([], [], color="tab:red", label="motion")
    (presence_line,) = ax_score.plot([], [], color="tab:blue", label="presence")
    ax_deviation = ax_score.twinx()
    (idle_deviation_line,) = ax_deviation.plot([], [], color="tab:green", label="from idle mean")
    (walk_deviation_line,) = ax_deviation.plot([], [], color="tab:purple", label="from walk mean")
    ax_deviation.set_ylabel("relative change from calibration mean (%)")
    ax_deviation.legend(loc="upper right")
    ax_score.axhline(SIGMA_NOISE, color="gray", linestyle=":", linewidth=1)
    ax_score.set_xlim(0, WINDOW)
    ax_score.set_ylim(0, SIGMA_MAX)
    ax_score.set_xlabel("frame (newest on the right)   ·   press 'c' to recalibrate")
    ax_score.set_ylabel("sigmas above empty room")
    ax_score.legend(loc="upper left")

    def on_key(event):
        if event.key == "c":
            baseline.ready = False
            calibration_start["total"] = reader.snapshot()[5]

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_):
        times, frames, idle_averages, walk_averages, rssi, total = reader.snapshot()
        if len(frames) < 2:
            ax_heat.set_title("waiting for CSI frames... (is the transmitter on?)")
            return heatmap, motion_line, presence_line, idle_deviation_line, walk_deviation_line

        heatmap.set_data(frames.T)
        heatmap.set_extent((WINDOW - len(frames), WINDOW, 0, SUBCARRIERS))
        span_s = (times[-1] - times[0]) / 1000
        rate = (len(times) - 1) / span_s if span_s > 0 else 0

        if not baseline.ready:
            collected = total - calibration_start["total"]
            if collected < CALIBRATION_FRAMES:
                ax_heat.set_title(f"CALIBRATING: keep the space between the boards empty "
                                  f"({collected}/{CALIBRATION_FRAMES})")
                return heatmap, motion_line, presence_line, idle_deviation_line, walk_deviation_line
            baseline.fit(frames[-CALIBRATION_FRAMES:])

        motion, presence = baseline.scores(frames)
        x = np.arange(WINDOW - len(frames), WINDOW)
        motion_line.set_data(x, motion)
        presence_line.set_data(x, presence)
        def deviation_from(averages):
            reference = next((values for values in reversed(averages)
                              if values is not None), None)
            deviation = np.zeros(len(frames))
            if reference is not None:
                active = reference > MIN_AMPLITUDE
                if np.any(active):
                    frame_shape = normalize(frames, active)
                    reference_shape = reference[active] / (reference[active].mean() + 1e-6)
                    raw = np.abs(frame_shape - reference_shape).mean(axis=1)
                    if len(frames) >= MOTION_WINDOW:
                        deviation[MOTION_WINDOW - 1:] = smooth(raw) * 100
            return deviation

        idle_deviation = deviation_from(idle_averages)
        walk_deviation = deviation_from(walk_averages)
        idle_deviation_line.set_data(x, idle_deviation)
        walk_deviation_line.set_data(x, walk_deviation)
        ax_deviation.relim()
        ax_deviation.autoscale_view(scalex=False, scaley=True)
        ax_heat.set_title(f"{rate:.0f} frames/s   RSSI {rssi} dBm   "
                          f"motion {motion[-1]:.1f}σ   presence {presence[-1]:.1f}σ   "
                          f"idle Δ {idle_deviation[-1]:.1f}%")
        return heatmap, motion_line, presence_line, idle_deviation_line, walk_deviation_line

    _anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
