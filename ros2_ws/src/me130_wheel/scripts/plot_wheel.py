#!/usr/bin/env python3
"""Wheel speed and motor command vs time, from the newest wheel_*.csv.

Record a log first:
    ros2 launch me130_wheel wheel_pi.launch.py log:=true
then, from the same directory:
    python3 ~/me130_lab/ros2_ws/src/me130_wheel/scripts/plot_wheel.py            # newest wheel_*.csv
    python3 ~/me130_lab/ros2_ws/src/me130_wheel/scripts/plot_wheel.py some.csv   # a specific log

"""
import glob
import os
import sys

import matplotlib
matplotlib.use("Agg")          # headless: save to file, never open a window
import matplotlib.pyplot as plt
import pandas as pd

LOG_GLOBS = ["./wheel_*.csv", "./build/wheel_*.csv"]
OUT_PREFIX = "wheel_response"
DPI = 150

SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
SURFACE, INK, INK_2, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9"

REQUIRED = {"t_s", "mode", "target_rad_s", "speed_rad_s", "u"}


def newest_log():
    """Most recent log, by the YYYYmmdd_HHMMSS in the filename rather than
    mtime, which a copy does not preserve."""
    hits = [f for pattern in LOG_GLOBS for f in glob.glob(pattern)]
    if not hits:
        sys.exit("No wheel log found in %s\n"
                 "Record one first:\n"
                 "  ros2 launch me130_wheel wheel_pi.launch.py log:=true" % os.getcwd())
    return max(hits, key=os.path.basename)


def load(path):
    df = pd.read_csv(path)
    missing = REQUIRED - set(df.columns)
    if missing:
        sys.exit("%s is missing column(s): %s" % (path, ", ".join(sorted(missing))))
    if df.empty:
        sys.exit("%s has no rows -- did the encoder publish?" % path)
    return df


def style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.tick_params(colors=INK_2)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else newest_log()
    df = load(path)
    t = df["t_s"]

    fig, (ax_w, ax_u) = plt.subplots(2, 1, sharex=True, figsize=(9, 6),
                                     gridspec_kw={"height_ratios": [3, 2]})
    fig.patch.set_facecolor(SURFACE)

    ax_w.plot(t, df["speed_rad_s"], color=SERIES[0], linewidth=1.4,
              label="measured")
    ax_w.step(t, df["target_rad_s"], where="post", color=SERIES[1], linewidth=1.4,
              linestyle="--", label="command")
    ax_w.set_ylabel("wheel speed (rad/s)", color=INK)
    ax_w.legend(loc="best", frameon=False, labelcolor=INK_2)
    style(ax_w)

    # Older logs have no u_unsaturated column; they just show u.
    if "u_unsaturated" in df.columns:
        ax_u.plot(t, df["u_unsaturated"], color=SERIES[3], linewidth=1.2,
                  linestyle="--", label="u unsaturated")
    ax_u.plot(t, df["u"], color=SERIES[2], linewidth=1.2, label="u saturated")
    ax_u.axhline(0.0, color=MUTED, linewidth=0.8)
    ax_u.set_ylabel("duty command u", color=INK)
    ax_u.set_xlabel("time (s)", color=INK)
    ax_u.legend(loc="best", frameon=False, labelcolor=INK_2)
    style(ax_u)

    fig.suptitle(os.path.basename(path), color=INK)
    fig.tight_layout()
    # wheel_20260929_223115.csv -> wheel_response_20260929_223115.png
    stem = os.path.splitext(os.path.basename(path))[0]
    tag = stem[len("wheel_"):] if stem.startswith("wheel_") else stem
    out = os.path.join(os.path.dirname(os.path.abspath(path)), "%s_%s.png" % (OUT_PREFIX, tag))
    fig.savefig(out, dpi=DPI, facecolor=SURFACE, bbox_inches="tight")
    print("wrote", out)


if __name__ == "__main__":
    main()
