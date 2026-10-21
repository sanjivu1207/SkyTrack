#!/usr/bin/env python3
"""Plot radar-tracker output: truth paths, detections, clutter, and confirmed tracks.

usage: python3 plot_tracks.py [csv_dir] [output.png]
Reads truth.csv, detections.csv, tracks.csv from csv_dir (default: current directory)
and writes tracks.png (or the given output path).
"""

import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

MAX_RANGE_KM = 50.0
ZOOM_SWEEPS = 15

SURFACE = "#fcfcfb"
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
BASELINE = "#c3c2b7"
TRACK = "#2a78d6"
DETECTION = "#eb6834"


def read_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def main():
    csv_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(".")
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("tracks.png")

    try:
        truth = read_rows(csv_dir / "truth.csv")
        dets = read_rows(csv_dir / "detections.csv")
        tracks = read_rows(csv_dir / "tracks.csv")
    except FileNotFoundError as e:
        sys.exit(f"Missing input: {e.filename}. Run radar_tracker first.")

    truth_by_sweep = defaultdict(dict)
    for r in truth:
        truth_by_sweep[int(r["sweep"])][r["id"]] = (float(r["x"]) / 1000.0, float(r["y"]) / 1000.0)

    truth_paths = defaultdict(lambda: ([], []))
    for r in truth:
        xs, ys = truth_paths[r["id"]]
        xs.append(float(r["x"]) / 1000.0)
        ys.append(float(r["y"]) / 1000.0)

    track_paths = defaultdict(lambda: ([], []))
    for r in tracks:
        if r["status"] != "confirmed":
            continue
        xs, ys = track_paths[r["id"]]
        xs.append(float(r["x"]) / 1000.0)
        ys.append(float(r["y"]) / 1000.0)

    real_x, real_y, clut_x, clut_y = [], [], [], []
    for r in dets:
        x, y = float(r["x"]) / 1000.0, float(r["y"]) / 1000.0
        if int(r["truth_id"]) < 0:
            clut_x.append(x)
            clut_y.append(y)
        else:
            real_x.append(x)
            real_y.append(y)

    plt.rcParams.update({"font.size": 10, "axes.titlesize": 13})
    fig, (ax, zx) = plt.subplots(1, 2, figsize=(16, 9), facecolor=SURFACE,
                                 gridspec_kw={"width_ratios": [1.6, 1.0]})

    theta = [2 * math.pi * i / 360 for i in range(361)]
    ax.plot([MAX_RANGE_KM * math.cos(t) for t in theta], [MAX_RANGE_KM * math.sin(t) for t in theta],
            color=BASELINE, lw=1.0, ls="--", zorder=1)
    ax.plot([0], [0], marker="^", ms=9, color=TEXT_PRIMARY, ls="none", zorder=6)
    ax.scatter(clut_x, clut_y, s=8, marker="x", color=MUTED, linewidths=0.8, alpha=0.6, zorder=2)
    ax.scatter(real_x, real_y, s=4, color=DETECTION, alpha=0.35, linewidths=0, zorder=3)
    for xs, ys in truth_paths.values():
        ax.plot(xs, ys, color=BASELINE, lw=4.0, solid_capstyle="round", zorder=4)
    for tid, (xs, ys) in track_paths.items():
        ax.plot(xs, ys, color=TRACK, lw=1.5, solid_capstyle="round", zorder=5)
        ax.plot(xs[-1], ys[-1], marker="o", ms=4, color=TRACK, mec=SURFACE, mew=1.0, zorder=5)
        ax.annotate(f"T{tid}", (xs[-1], ys[-1]), xytext=(4, 4), textcoords="offset points",
                    fontsize=7, color=TEXT_SECONDARY, zorder=7)

    lim = MAX_RANGE_KM * 1.12
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_title(f"{len(truth_paths)} aircraft, {len(track_paths)} confirmed tracks",
                 color=TEXT_PRIMARY, loc="left", pad=10)

    # Zoom panel: one aircraft over a short window, where the radar noise is visible.
    sweeps = sorted(truth_by_sweep)
    mid = sweeps[len(sweeps) // 2]
    window = [k for k in sweeps if mid <= k < mid + ZOOM_SWEEPS]
    in_cov = [tid for tid, (x, y) in truth_by_sweep[mid].items()
              if all(math.hypot(*truth_by_sweep[k].get(tid, (1e9, 1e9))) < MAX_RANGE_KM * 0.9
                     for k in window)]
    focus = min(in_cov, key=int) if in_cov else next(iter(truth_by_sweep[mid]))
    fx = [truth_by_sweep[k][focus][0] for k in window]
    fy = [truth_by_sweep[k][focus][1] for k in window]
    pad = 0.4
    x0, x1, y0, y1 = min(fx) - pad, max(fx) + pad, min(fy) - pad, max(fy) + pad
    half = max(x1 - x0, y1 - y0) / 2
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    x0, x1, y0, y1 = cx - half, cx + half, cy - half, cy + half

    def in_box(x, y):
        return x0 <= x <= x1 and y0 <= y <= y1

    zd = [(float(r["x"]) / 1000.0, float(r["y"]) / 1000.0) for r in dets
          if r["truth_id"] == focus and int(r["sweep"]) in window]
    zc = [(float(r["x"]) / 1000.0, float(r["y"]) / 1000.0) for r in dets
          if int(r["truth_id"]) < 0 and int(r["sweep"]) in window
          and in_box(float(r["x"]) / 1000.0, float(r["y"]) / 1000.0)]
    zt = defaultdict(lambda: ([], []))
    for r in tracks:
        x, y = float(r["x"]) / 1000.0, float(r["y"]) / 1000.0
        if r["status"] == "confirmed" and int(r["sweep"]) in window and in_box(x, y):
            zt[r["id"]][0].append(x)
            zt[r["id"]][1].append(y)

    zx.plot(fx, fy, color=BASELINE, lw=4.0, solid_capstyle="round", zorder=2)
    if zc:
        zx.scatter(*zip(*zc), s=30, marker="x", color=MUTED, linewidths=1.0, zorder=3)
    if zd:
        zx.scatter(*zip(*zd), s=36, color=DETECTION, edgecolors=SURFACE, linewidths=1.0, zorder=4)
    for tid, (xs, ys) in zt.items():
        zx.plot(xs, ys, color=TRACK, lw=2.0, marker="o", ms=4, mec=SURFACE, mew=1.0, zorder=5)
        zx.annotate(f"T{tid}", (xs[-1], ys[-1]), xytext=(6, -2), textcoords="offset points",
                    fontsize=9, color=TEXT_SECONDARY)
    zx.set_xlim(x0, x1)
    zx.set_ylim(y0, y1)
    zx.set_title(f"Zoom: aircraft {focus}, sweeps {window[0]}-{window[-1]}",
                 color=TEXT_PRIMARY, loc="left", pad=10)
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec=TEXT_SECONDARY, lw=1.0,
                           zorder=8))

    for a in (ax, zx):
        a.set_facecolor(SURFACE)
        a.set_aspect("equal")
        a.set_xlabel("East (km)", color=TEXT_SECONDARY)
        a.set_ylabel("North (km)", color=TEXT_SECONDARY)
        a.grid(color=GRID, lw=0.6)
        a.tick_params(colors=MUTED)
        for spine in a.spines.values():
            spine.set_color(BASELINE)
        a.set_axisbelow(True)

    handles = [
        Line2D([], [], color=BASELINE, lw=4.0, label="True aircraft path"),
        Line2D([], [], color=TRACK, lw=2.0, marker="o", ms=4, mec=SURFACE, label="Confirmed track"),
        Line2D([], [], color=DETECTION, ls="none", marker="o", ms=6, mec=SURFACE,
               label=f"Target detection ({len(real_x)})"),
        Line2D([], [], color=MUTED, ls="none", marker="x", ms=6, label=f"Clutter ({len(clut_x)})"),
        Line2D([], [], color=BASELINE, ls="--", lw=1.0, label="Radar coverage (50 km)"),
        Line2D([], [], color=TEXT_PRIMARY, ls="none", marker="^", ms=8, label="Radar"),
    ]
    leg = fig.legend(handles=handles, loc="lower center", ncol=6, frameon=False, fontsize=10,
                     bbox_to_anchor=(0.5, 0.0))
    for text in leg.get_texts():
        text.set_color(TEXT_SECONDARY)
    fig.suptitle("Radar multi-target tracking", x=0.02, ha="left", fontsize=15, color=TEXT_PRIMARY)

    fig.tight_layout(rect=(0, 0.05, 1, 0.97))
    fig.savefig(out_path, dpi=130, facecolor=SURFACE)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    main()
