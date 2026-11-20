#!/opt/anaconda3/bin/python3
"""Render bench CSVs as a logic-analyser style waveform PNG, or as a displacement chart.

Waveform view: stacked rows on one time axis, one signal per row: PID-tab error (robot run as dots
when given, zero line, +-0.5 in exit band), speed, drive command, D term, then digital rows drawn as
square waves: fresh/stale, close, finished. A vertical marker where `close` goes true. With two CSVs
the columns are BEFORE and AFTER, with identical scales.

    wave.py BEFORE.csv [AFTER.csv] --label "BEFORE ..." --label "AFTER ..." \
            [--robot data/tune_2026-11-19.log:6073] [--zoom 800:1600] [--err-max 15] -o out.png

Displacement chart: one panel, distance travelled (|target| - PID-tab error, from odometry as the
PID tab logs it) against time, for one or more CSVs plus the robot run.

    wave.py a.csv b.csv --label "bench, rot 20 ms" --label "bench, rot 5 ms" \
            --robot data/tune_2026-11-19.log:6073 --displacement out.png

Pose runs (tb_* --move pose): the waveform view shows distance to target, speed, the implied
lateralOut with the slip cap on the same axis, angularOut, then cap binding / close / finished.
The top view (--path) draws the true x-y path of one or more pose runs, start and target poses as
arrows, a dot every 100 ms, and a wide band under the path wherever the cap was binding.

    wave.py pose.csv [pose2.csv] --label .. -o wave.png
    wave.py pose.csv [pose2.csv ...] --label .. --path top.png [--carrot]
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import warnings

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import simlib  # noqa: E402
from simlib import STYLE, apply_axis_style  # noqa: E402

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.gridspec import GridSpec  # noqa: E402

warnings.filterwarnings("ignore", message="findfont")
plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = ["Helvetica Neue", "Helvetica", "Arial", "DejaVu Sans"]

BENCH_SLOTS = [0, 2, 3, 4, 5, 6, 7]  # series slots for bench CSVs, in order; slot 1 (orange) is the robot
ROBOT_SLOT = 1
DPI = 120
MARKER = 5.0  # points: 8.3 px at 120 dpi


def load_csv(path):
    with open(path) as fh:
        rows = list(csv.reader(fh))
    head, body = rows[0], rows[1:]
    data = np.array([[float(x) for x in r] for r in body]) if body else np.zeros((0, len(head)))
    return {h: data[:, i] for i, h in enumerate(head)}


def summarize(d):
    """Numbers for a column header, from one bench CSV."""
    s = {}
    strobe = d["tab_strobe"] == 1
    err = d["tab_err"][strobe]
    s["over"] = simlib.overshoot(err)
    it = d["iter_strobe"] == 1
    close = it & (d["close"] == 1)
    s["close_t"] = float(d["t_ms"][close][0]) if close.any() else None
    s["close_dist"] = float(d["iter_dist"][close][0]) if close.any() else None
    moving = it & ((np.abs(d["vL"]) + np.abs(d["vR"])) / 2 > 2)
    s["stale"] = float(np.mean(d["iter_fresh"][moving] == 0)) if moving.any() else float("nan")
    fin = d["finished"] == 1
    s["end_t"] = float(d["t_ms"][fin][0]) if fin.any() else None
    s["peak_v"] = float(np.max(np.abs(d["v"])))
    s["target"] = float(d["target"][0]) if "target" in d else float("nan")
    s["drive"] = not ("target" in d and np.allclose(d["v"], 0, atol=1e-9) and np.any(d["cmdL"] != d["cmdR"]))
    return s


def robot_run(spec):
    log, line = spec.rsplit(":", 1)
    if not os.path.isabs(log) and not os.path.exists(log):
        log = os.path.join(simlib.HERE, log)
    runs = simlib.parse_log(log)
    best = min(runs, key=lambda r: abs(r.line - int(line)))
    if abs(best.line - int(line)) > 40:
        sys.exit(f"no PIDLOG run near line {line} in {log}")
    return best


def digital(ax, t, sig, color, name):
    ax.fill_between(t, 0, sig, step="post", color=color, alpha=0.10, linewidth=0)
    ax.step(t, sig, where="post", color=color, lw=1.6, solid_joinstyle="miter")
    ax.set_ylim(-0.3, 1.3)
    ax.set_yticks([])
    ax.spines["left"].set_visible(False)


def row_label(ax, text, sub=None):
    ax.set_ylabel(text if sub is None else f"{text}\n{sub}", rotation=0, ha="right", va="center", fontsize=8.5,
                  color=STYLE["text"], labelpad=8, linespacing=1.25)


def waveform(args, datas, labels, rob):
    sums = [summarize(d) for d in datas]
    drive = all(s["drive"] for s in sums)
    unit = "in" if drive else "deg"
    ncol = len(datas)
    t_lo = min(d["t_ms"][0] for d in datas)
    t_hi = max(d["t_ms"][-1] for d in datas)
    if args.zoom:
        t_lo, t_hi = (float(v) for v in args.zoom.split(":"))
    win = [(d["t_ms"] >= t_lo) & (d["t_ms"] <= t_hi) for d in datas]

    # shared y limits per analog row
    err_lo = min(min(np.nanmin(d["tab_err"][w]) for d, w in zip(datas, win)), -0.5) - 0.4
    if rob is not None:
        rt, re_, _ = rob.arrays()
        rw = (rt >= t_lo) & (rt <= t_hi)
        if rw.any():
            err_lo = min(err_lo, re_[rw].min() - 0.4)
    err_hi = args.err_max
    vmax = max(np.max(np.abs(d["v"][w])) for d, w in zip(datas, win)) if drive else None
    if drive:
        spd_lim = (min(0, min(np.min(d["v"][w]) for d, w in zip(datas, win))) - 2, vmax * 1.08 + 1)
    else:
        rate = [np.gradient(d["heading_deg"], d["t_ms"] / 1000.0) for d in datas]
        spd_lim = (min(np.min(r[w]) for r, w in zip(rate, win)) * 1.08 - 5, max(np.max(r[w]) for r, w in zip(rate, win)) * 1.08 + 5)
    dvals = []
    for d, w in zip(datas, win):
        it = (d["iter_strobe"] == 1) & (d["iter_n"] > 1) & w
        dvals.append(d["iter_D"][it])
    dall = np.concatenate([x for x in dvals if len(x)]) if any(len(x) for x in dvals) else np.array([0.0])
    dpad = 0.08 * max(1.0, dall.max() - dall.min())
    d_lim = (min(dall.min(), 0) - dpad, max(dall.max(), 0) + dpad)

    heights = [2.2, 1.5, 1.5, 1.5, 0.45, 0.45, 0.45]
    fig = plt.figure(figsize=(6.2 * ncol + 1.6, sum(heights) + 1.9))  # 1.9 in: titles + x axis
    fig.patch.set_facecolor(STYLE["surface"])
    gs = GridSpec(len(heights), ncol, figure=fig, height_ratios=heights, hspace=0.18, wspace=0.10,
                  left=1.55 / (6.2 * ncol + 1.6), right=0.985, top=1 - 1.0 / (sum(heights) + 1.9),
                  bottom=0.55 / (sum(heights) + 1.9))
    for c, (d, lab, s, w) in enumerate(zip(datas, labels, sums, win)):
        color = STYLE["series"][BENCH_SLOTS[c]]
        t = d["t_ms"]
        axes = [fig.add_subplot(gs[r, c]) for r in range(len(heights))]
        for a in axes[1:]:
            a.sharex(axes[0])
        for a in axes:
            apply_axis_style(a, grid_y=True)
            a.set_xlim(t_lo, t_hi)
            if a is not axes[-1]:
                a.tick_params(labelbottom=False)
        # 1. PID tab error
        a = axes[0]
        band = 0.5 if drive else 1.0  # LemLib small exit range: lateral 0.5 in, angular 1 deg
        a.axhspan(-band, band, color=STYLE["band"], lw=0, zorder=0)
        a.axhline(0, color=STYLE["baseline"], lw=1, zorder=1)
        st = (d["tab_strobe"] == 1)
        a.plot(t[st], d["tab_err"][st], color=color, lw=2, solid_capstyle="round", zorder=3,
               label="bench: PID tab error (10 ms)")
        if rob is not None and c == 0:
            rt, re_, _ = rob.arrays()
            a.plot(rt, re_, "o", ms=MARKER, color=STYLE["series"][ROBOT_SLOT], mec=STYLE["surface"], mew=1.0,
                   zorder=4, label=f"robot: line {rob.line}, {rob.target:+.0f} in kD {rob.gains()[2]:g}")
            a.legend(loc="best" if args.zoom else "center left", fontsize=7.5, frameon=False, labelcolor=STYLE["text"], handlelength=1.6)
        a.set_ylim(err_lo, err_hi)
        a.text(t_lo + 0.01 * (t_hi - t_lo), band, f"exit band ±{band:g} {unit}", ha="left", va="bottom", fontsize=7,
               color=STYLE["text2"], zorder=5)
        # 2. speed
        a = axes[1]
        a.axhline(0, color=STYLE["baseline"], lw=1, zorder=1)
        if drive:
            a.plot(t, d["v"], color=color, lw=2, zorder=3)
        else:
            a.plot(t, np.gradient(d["heading_deg"], t / 1000.0), color=color, lw=2, zorder=3)
        a.set_ylim(*spd_lim)
        # 3. command
        a = axes[2]
        a.axhline(0, color=STYLE["baseline"], lw=1, zorder=1)
        cmd = (d["cmdL"] + d["cmdR"]) / 2 if drive else (d["cmdL"] - d["cmdR"]) / 2
        a.step(t, cmd, where="post", color=color, lw=1.6, zorder=3)
        a.set_ylim(-140, 140)
        a.set_yticks([-127, 0, 127])
        # 4. D term, held from one iteration to the next
        a = axes[3]
        a.axhline(0, color=STYLE["baseline"], lw=1, zorder=1)
        a.step(t, d["iter_D"], where="post", color=color, lw=1.4, zorder=3)
        a.set_ylim(*d_lim)
        first = (d["iter_strobe"] == 1) & (d["iter_n"] == 1)
        if first.any() and t_lo <= t[first][0] <= t_hi:
            a.text(t_lo + 0.01 * (t_hi - t_lo), d_lim[0] + 0.03 * (d_lim[1] - d_lim[0]),
                   f"1st iteration off scale: {d['iter_D'][first][0]:.0f} (kD × first error)",
                   ha="left", va="bottom", fontsize=7, color=STYLE["text2"])
        # 5-7. digital
        digital(axes[4], t, d["iter_fresh"], color, "fresh")
        digital(axes[5], t, d["close"], color, "close")
        digital(axes[6], t, d["finished"], color, "finished")
        axes[-1].set_xlabel("time since the move started, ms", fontsize=8, color=STYLE["text2"])
        # close marker across every row
        if s["close_t"] is not None and t_lo <= s["close_t"] <= t_hi:
            for a in axes:
                a.axvline(s["close_t"], color=STYLE["muted"], lw=1, zorder=2)
            what = f"close: {s['close_dist']:.2f} {unit} to go" if drive else "|error| < 20 deg"
            axes[0].text(s["close_t"], err_hi, f" {what}", ha="left", va="top", fontsize=7.5, color=STYLE["text2"])
        # column header
        bits = [f"overshoot {s['over']:.2f} {unit}"]
        if drive and s["close_t"] is not None:
            bits.append(f"close at {s['close_t']:.0f} ms")
        if np.isfinite(s["stale"]):
            bits.append(f"stale {100 * s['stale']:.0f}% of moving iterations")
        if s["end_t"] is not None:
            bits.append(f"motion ended {s['end_t']:.0f} ms")
        axes[0].set_title(lab, loc="left", fontsize=10.5, color=STYLE["text"], pad=16, fontweight="semibold")
        axes[0].text(0, 1.03, "   ".join(bits), transform=axes[0].transAxes, fontsize=8, color=STYLE["text2"],
                     ha="left", va="bottom")
        if c == 0:
            row_label(axes[0], "PID tab error", f"{unit}, + = short")
            row_label(axes[1], "speed" if drive else "turn rate", "in/s" if drive else "deg/s")
            row_label(axes[2], "drive command" if drive else "turn command", "(L+R)/2" if drive else "(L-R)/2")
            row_label(axes[3], "D term", "kD·(e - e prev)")
            row_label(axes[4], "fresh pose", "1 fresh, 0 stale")
            row_label(axes[5], "close", "7.5 in" if drive else "< 20 deg")
            row_label(axes[6], "finished")
    title = args.title or ("Drive testbench waveforms" if drive else "Turn testbench waveforms")
    sub = f"PID tab error clipped at {err_hi:g} {unit}; shared scales across columns"
    if args.zoom:
        sub += f"; window {t_lo:.0f}-{t_hi:.0f} ms"
    fig.text(0.008, 0.995, title, ha="left", va="top", fontsize=12.5, color=STYLE["text"], fontweight="semibold")
    fig.text(0.008, 0.995 - 0.30 / (sum(heights) + 1.9), sub, ha="left", va="top", fontsize=8.5, color=STYLE["text2"])
    fig.savefig(args.out, dpi=DPI, facecolor=STYLE["surface"])
    print(f"wrote {args.out}")


def spread(ys, min_gap):
    """Positions near ys (any order) with at least min_gap between neighbours; keeps the order."""
    order = np.argsort(ys)
    pos = np.array(ys, dtype=float)[order]
    for i in range(1, len(pos)):
        pos[i] = max(pos[i], pos[i - 1] + min_gap)
    shift = (np.mean(np.array(ys)[order]) - np.mean(pos))  # recentre the block on the data
    pos += shift
    out = np.empty_like(pos)
    out[order] = pos
    return out


def displacement(args, datas, labels, rob):
    targets = {abs(float(d["target"][0])) for d in datas if "target" in d}
    if rob is not None:
        targets.add(abs(rob.target))
    if len(targets) != 1:
        sys.exit(f"displacement: the CSVs and robot run need one common |target|, got {sorted(targets)}")
    target = targets.pop()
    series = []  # (label, t, disp, slot, style)
    for i, (d, lab) in enumerate(zip(datas, labels)):
        st = d["tab_strobe"] == 1
        series.append((lab, d["t_ms"][st], target - d["tab_err"][st], BENCH_SLOTS[i], "line"))
    if rob is not None:
        rt, re_, _ = rob.arrays()
        kp, ki, kd = rob.gains()
        series.insert(1 if series else 0, (f"robot (line {rob.line}, kD {kd:g})", rt, target - re_, ROBOT_SLOT, "dots"))
    t_hi = max(s[1][-1] for s in series)
    t_lo = 0.0
    if args.zoom:
        t_lo, t_hi = (float(v) for v in args.zoom.split(":"))
    ymax = max(np.max(s[2][(s[1] >= t_lo) & (s[1] <= t_hi)]) for s in series)
    ymin = min(np.min(s[2][(s[1] >= t_lo) & (s[1] <= t_hi)]) for s in series)
    y_lo = min(0.0, ymin) - 0.02 * target
    y_hi = max(target, ymax) + 0.06 * target

    fig, ax = plt.subplots(figsize=(10.5, 6.2))
    fig.patch.set_facecolor(STYLE["surface"])
    fig.subplots_adjust(left=0.075, right=0.76, top=0.86, bottom=0.10)
    apply_axis_style(ax)
    ax.axhspan(target - 7.5, target, color=STYLE["band"], lw=0, zorder=0)
    ax.axhline(target, color=STYLE["text2"], lw=1, zorder=2)
    ax.axhline(0, color=STYLE["baseline"], lw=1, zorder=1)
    ax.text(t_lo + 0.01 * (t_hi - t_lo), target - 7.5 + 0.25, "LemLib settle zone: last 7.5 in before the target",
            ha="left", va="bottom", fontsize=8, color=STYLE["text2"], zorder=5)
    for lab, t, y, slot, kind in series:
        col = STYLE["series"][slot]
        if kind == "line":
            ax.plot(t, y, color=col, lw=2, solid_capstyle="round", solid_joinstyle="round", zorder=3, label=lab)
        else:
            ax.plot(t, y, "o", ms=MARKER, color=col, mec=STYLE["surface"], mew=1.0, zorder=4, label=lab)
    ax.set_xlim(t_lo, t_hi)
    ax.set_ylim(y_lo, y_hi)
    ax.set_xlabel("time since the move started, ms", fontsize=9, color=STYLE["text2"])
    ax.set_ylabel("distance travelled, in", fontsize=9, color=STYLE["text2"])

    # right-end labels: the lines converge on the target, so labels go in a column right of the plot,
    # spread apart, each tied to its line's end with a thin leader
    ends = []
    for lab, t, y, slot, kind in series:
        k = np.nonzero((t >= t_lo) & (t <= t_hi))[0][-1]
        ends.append((t[k], y[k]))
    label_ys = spread([e[1] for e in ends], min_gap=(y_hi - y_lo) * 0.075)
    label_ys = np.clip(label_ys, y_lo, y_hi)
    x_text = t_hi + 0.04 * (t_hi - t_lo)
    for (lab, t, y, slot, kind), (xe, ye), yl in zip(series, ends, label_ys):
        over = np.max(y) - target
        txt = f"{lab}\nend {ye:.2f} in, overshoot {max(0.0, over):.2f} in"
        ax.annotate(txt, xy=(xe, ye), xytext=(x_text, yl), textcoords="data", annotation_clip=False,
                    ha="left", va="center", fontsize=8, color=STYLE["text"], linespacing=1.3,
                    arrowprops=dict(arrowstyle="-", color=STYLE["muted"], lw=0.8, shrinkA=0, shrinkB=2,
                                    connectionstyle="angle,angleA=0,angleB=90,rad=0"))
        ax.plot([x_text - 0.012 * (t_hi - t_lo)], [yl], marker="o" if kind == "dots" else "_", ms=6 if kind == "dots" else 12,
                mew=2 if kind == "line" else 1.0, color=STYLE["series"][slot],
                mec=STYLE["surface"] if kind == "dots" else STYLE["series"][slot], clip_on=False, zorder=6)
    ax.text(t_lo + 0.01 * (t_hi - t_lo), target + 0.012 * (y_hi - y_lo), f"target {target:g} in", ha="left",
            va="bottom", fontsize=8.5, color=STYLE["text"], zorder=5)
    if len(series) > 1:
        ax.legend(loc="lower right", fontsize=8, frameon=False, labelcolor=STYLE["text"], handlelength=1.8)
    title = args.title or f"Distance travelled on a {target:g} in drive: robot vs bench"
    fig.text(0.075, 0.955, title, ha="left", va="top", fontsize=13, color=STYLE["text"], fontweight="semibold")
    fig.text(0.075, 0.905, "All series are odometry as the PID tab logs it (|target| - error, every 10 ms); "
             "the robot's lost rows are simply missing", ha="left", va="top", fontsize=8.5, color=STYLE["text2"])
    out = args.displacement
    fig.savefig(out, dpi=DPI, facecolor=STYLE["surface"])
    print(f"wrote {out}")


# ------------------------------------------------------------------------------------ pose runs

def is_pose(d):
    return "p_dist" in d


def summarize_pose(d):
    s = {}
    it = d["iter_strobe"] == 1
    fin = d["finished"] == 1
    s["end_t"] = float(d["t_ms"][fin][0]) if fin.any() else None
    s["exits"] = bool(d["p_exits_met"][it][-1]) if it.any() else False
    timeout = float(d["timeout_ms"][0]) if "timeout_ms" in d else float("inf")
    s["ended"] = "the exits" if s["exits"] else ("timeout" if s["end_t"] is not None and s["end_t"] >= timeout
                                                 else "another rule")
    s["cap_ms"] = 10.0 * float(np.sum(d["p_cap_bind"][it & (d["finished"] == 0)]))
    s["err"] = float(np.hypot(d["target_x"][-1] - d["x"][-1], d["target_y"][-1] - d["y"][-1]))
    s["herr"] = round(float(np.remainder(d["target_theta"][-1] - d["heading_deg"][-1] + 180.0, 360.0) - 180.0), 1) + 0.0
    close = it & (d["close"] == 1)
    s["close_t"] = float(d["t_ms"][close][0]) if close.any() else None
    s["target"] = (float(d["target_x"][0]), float(d["target_y"][0]), float(d["target_theta"][0]))
    s["mismatch"] = int(np.sum(d["p_mirror_ok"][it] == 0))
    return s


def waveform_pose(args, datas, labels):
    sums = [summarize_pose(d) for d in datas]
    ncol = len(datas)
    t_lo = min(d["t_ms"][0] for d in datas)
    t_hi = max(d["t_ms"][-1] for d in datas)
    if args.zoom:
        t_lo, t_hi = (float(v) for v in args.zoom.split(":"))
    win = [(d["t_ms"] >= t_lo) & (d["t_ms"] <= t_hi) for d in datas]
    slip = any(np.max(np.abs(d["vside"])) > 0.01 for d in datas)

    def lim(key, pad=0.08, lo0=None):
        vals = np.concatenate([d[key][w][np.isfinite(d[key][w])] for d, w in zip(datas, win)])
        lo, hi = float(vals.min()), float(vals.max())
        if lo0 is not None:
            lo, hi = min(lo, lo0), max(hi, lo0)
        span = max(hi - lo, 1.0)
        return lo - pad * span, hi + pad * span

    dist_lim = lim("p_dist", lo0=0.0)
    spd_lim = lim("v", lo0=0.0)
    side_lim = lim("vside", lo0=0.0) if slip else None
    lat_vals = np.concatenate([d["p_lat_sent"][w][np.isfinite(d["p_lat_sent"][w])] for d, w in zip(datas, win)])
    lat_lim = (min(-10.0, float(lat_vals.min()) - 8), 140.0)
    ang_lim = lim("p_ang_sent", lo0=0.0)

    rows = [("dist", 1.7), ("speed", 1.4)] + ([("side", 1.1)] if slip else []) + \
           [("lat", 1.9), ("ang", 1.4), ("bind", 0.45), ("close", 0.45), ("fin", 0.45)]
    heights = [h for _, h in rows]
    H = sum(heights) + 1.9
    W = 6.2 * ncol + 1.6
    fig = plt.figure(figsize=(W, H))
    fig.patch.set_facecolor(STYLE["surface"])
    gs = GridSpec(len(rows), ncol, figure=fig, height_ratios=heights, hspace=0.18, wspace=0.10, left=1.55 / W,
                  right=0.985, top=1 - 1.0 / H, bottom=0.55 / H)
    for c, (d, lab, s, w) in enumerate(zip(datas, labels, sums, win)):
        color = STYLE["series"][BENCH_SLOTS[c]]
        t = d["t_ms"]
        moving = d["finished"] == 0
        axes = {}
        first = None
        for r, (name, _) in enumerate(rows):
            a = fig.add_subplot(gs[r, c], sharex=first)
            first = first or a
            apply_axis_style(a)
            a.set_xlim(t_lo, t_hi)
            if r < len(rows) - 1:
                a.tick_params(labelbottom=False)
            axes[name] = a
        a = axes["dist"]
        a.axhline(0, color=STYLE["baseline"], lw=1, zorder=1)
        if dist_lim[0] < 7.5 < dist_lim[1] - 0.08 * (dist_lim[1] - dist_lim[0]):
            a.axhline(7.5, color=STYLE["grid"], lw=1, zorder=1)
            a.text(t_lo + 0.01 * (t_hi - t_lo), 7.5, "close below 7.5 in", ha="left", va="bottom", fontsize=7,
                   color=STYLE["text2"], clip_on=True)
        a.step(t, d["p_dist"], where="post", color=color, lw=2, zorder=3)
        a.set_ylim(*dist_lim)
        a = axes["speed"]
        a.axhline(0, color=STYLE["baseline"], lw=1, zorder=1)
        a.plot(t, d["v"], color=color, lw=2, zorder=3)
        a.set_ylim(*spd_lim)
        if slip:
            a = axes["side"]
            a.axhline(0, color=STYLE["baseline"], lw=1, zorder=1)
            a.plot(t, d["vside"], color=color, lw=2, zorder=3)
            a.set_ylim(*side_lim)
        a = axes["lat"]
        a.axhline(0, color=STYLE["baseline"], lw=1, zorder=1)
        cap = np.where(moving, np.minimum(d["p_cap"], 1e4), np.nan)
        a.step(t, cap, where="post", color=STYLE["text2"], lw=1.4, zorder=2, label="cap √(drift·radius·9.8)")
        a.step(t, np.where(moving, d["p_lat_sent"], np.nan), where="post", color=color, lw=2, zorder=3,
               label="lateralOut as sent, (L+R)/2")
        a.set_ylim(*lat_lim)
        a.set_yticks([0, 60, 127])
        vis = np.nonzero(moving & (cap < lat_lim[1] - 10) & (t >= t_lo) & (t <= t_hi))[0]
        if c == 0:
            a.legend(loc="upper right", fontsize=7.5, frameon=False, labelcolor=STYLE["text"], handlelength=1.6)
        a = axes["ang"]
        a.axhline(0, color=STYLE["baseline"], lw=1, zorder=1)
        a.step(t, np.where(moving, d["p_ang_sent"], np.nan), where="post", color=color, lw=2, zorder=3)
        a.set_ylim(*ang_lim)
        digital(axes["bind"], t, np.where(moving, d["p_cap_bind"], 0), color, "bind")
        digital(axes["close"], t, d["close"], color, "close")
        digital(axes["fin"], t, d["finished"], color, "finished")
        axes["fin"].set_xlabel("time since the move started, ms", fontsize=8, color=STYLE["text2"])
        if s["close_t"] is not None and t_lo <= s["close_t"] <= t_hi:
            for a in axes.values():
                a.axvline(s["close_t"], color=STYLE["muted"], lw=1, zorder=2)
            axes["dist"].text(s["close_t"], dist_lim[1], " close" if s["close_t"] > t_lo else " close from the start",
                              ha="left", va="top", fontsize=7.5, color=STYLE["text2"], clip_on=True)
        bits = [f"ended by {s['ended']} at {s['end_t']:.0f} ms",
                f"final error {s['err']:.2f} in, {s['herr']:+.1f} deg", f"cap binding {s['cap_ms']:.0f} ms"]
        if s["mismatch"]:
            bits.append(f"mirror ≠ DUT on {s['mismatch']} iterations")
        axes["dist"].set_title(lab, loc="left", fontsize=10.5, color=STYLE["text"], pad=16, fontweight="semibold")
        axes["dist"].text(0, 1.03, "   ".join(bits), transform=axes["dist"].transAxes, fontsize=8,
                          color=STYLE["text2"], ha="left", va="bottom")
        if c == 0:
            row_label(axes["dist"], "distance to target", "in")
            row_label(axes["speed"], "forward speed", "in/s")
            if slip:
                row_label(axes["side"], "sideways speed", "in/s, + right")
            row_label(axes["lat"], "lateralOut", "motor power")
            row_label(axes["ang"], "angularOut", "(L-R)/2")
            row_label(axes["bind"], "cap binding")
            row_label(axes["close"], "close", "7.5 in")
            row_label(axes["fin"], "finished")
    tx, ty, tth = sums[0]["target"]
    title = args.title or f"moveToPose to ({tx:g}, {ty:g}, {tth:g}°): waveforms"
    sub = "signals mirrored from stock moveToPose in the bench; cap drawn only while the motion runs, clipped at 140"
    if args.zoom:
        sub += f"; window {t_lo:.0f}-{t_hi:.0f} ms"
    fig.text(0.008, 0.995, title, ha="left", va="top", fontsize=12.5, color=STYLE["text"], fontweight="semibold")
    fig.text(0.008, 0.995 - 0.30 / H, sub, ha="left", va="top", fontsize=8.5, color=STYLE["text2"])
    fig.savefig(args.out, dpi=DPI, facecolor=STYLE["surface"])
    print(f"wrote {args.out}")


def pose_arrow(ax, x, y, heading_deg, length, color, label, ha):
    hr = np.radians(heading_deg)
    dx, dy = length * np.sin(hr), length * np.cos(hr)
    ax.annotate("", xy=(x + dx, y + dy), xytext=(x, y), zorder=6,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=1.6, mutation_scale=14, shrinkA=0, shrinkB=0))
    ax.plot([x], [y], "o", ms=4, color=color, zorder=6)
    # label beside the arrow, on its left (perpendicular to the heading), clear of the path along it
    px, py = -np.cos(hr), np.sin(hr)
    off = 0.45 * length
    ha = "center" if abs(px) < 0.5 else ("right" if px < 0 else "left")
    va = "center" if abs(py) < 0.5 else ("bottom" if py > 0 else "top")
    ax.text(x + 0.5 * dx + off * px, y + 0.5 * dy + off * py, label, ha=ha, va=va, fontsize=8, color=STYLE["text"],
            zorder=7)


def path_view(args, datas, labels):
    fig, ax = plt.subplots(figsize=(8.6, 8.0))
    fig.patch.set_facecolor(STYLE["surface"])
    fig.subplots_adjust(left=0.10, right=0.96, top=0.86, bottom=0.08)
    apply_axis_style(ax, grid_y=True)
    ax.grid(axis="x", color=STYLE["grid"], linewidth=0.8, linestyle="-")
    xs = np.concatenate([d["x"] for d in datas] + [d["target_x"][:1] for d in datas] + [np.zeros(1)])
    ys = np.concatenate([d["y"] for d in datas] + [d["target_y"][:1] for d in datas] + [np.zeros(1)])
    span = max(xs.max() - xs.min(), ys.max() - ys.min(), 6.0)
    alen = 0.09 * span
    ends = []
    for i, (d, lab) in enumerate(zip(datas, labels)):
        col = STYLE["series"][BENCH_SLOTS[i]]
        t, x, y = d["t_ms"], d["x"], d["y"]
        moving = d["finished"] == 0
        bind = (d["p_cap_bind"] == 1) & moving
        # cap binding: a wide translucent band under the path (width is the encoding, not colour)
        xb, yb = np.where(bind, x, np.nan), np.where(bind, y, np.nan)
        ax.plot(xb, yb, color=col, lw=9, alpha=0.28, solid_capstyle="butt", zorder=2,
                label="cap binding (wide band)" if i == 0 else None)
        ax.plot(x, y, color=col, lw=2, solid_capstyle="round", zorder=3, label=lab)
        tick = (np.mod(t, 100) == 0) & (t >= 0)
        ax.plot(x[tick], y[tick], "o", ms=4.8, color=col, mec=STYLE["surface"], mew=1.0, zorder=4)
        if args.carrot:
            it = (d["iter_strobe"] == 1) & moving
            ax.plot(d["p_carrot_x"][it][::3], d["p_carrot_y"][it][::3], "o", ms=3.5, mfc="none", mec=STYLE["muted"],
                    mew=0.8, zorder=1, label="carrot (every 3rd iteration)" if i == 0 else None)
        ends.append((x[-1], y[-1], lab, col))
    d0 = datas[0]
    pose_arrow(ax, d0["x"][0], d0["y"][0], d0["heading_deg"][0], alen, STYLE["text2"], "start", "right")
    tx, ty, tth = float(d0["target_x"][0]), float(d0["target_y"][0]), float(d0["target_theta"][0])
    pose_arrow(ax, tx, ty, tth, alen, STYLE["text"], f"target ({tx:g}, {ty:g}, {tth:g}°)", "left")
    if len(datas) > 1:
        for (xe, ye, lab, col) in ends:
            ax.annotate(lab, xy=(xe, ye), xytext=(8, -12), textcoords="offset points", fontsize=8,
                        color=STYLE["text"], arrowprops=dict(arrowstyle="-", color=STYLE["muted"], lw=0.8))
    # equal spans on both axes, centred on the data, so equal aspect needs no limit adjustment
    half = 0.5 * span + 0.14 * span
    cx, cy = 0.5 * (xs.min() + xs.max()), 0.5 * (ys.min() + ys.max())
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x, in (+ right of the start)", fontsize=9, color=STYLE["text2"])
    ax.set_ylabel("y, in (+ ahead of the start)", fontsize=9, color=STYLE["text2"])
    ax.legend(loc="best", fontsize=8, frameon=False, labelcolor=STYLE["text"], handlelength=2.2)
    title = args.title or f"moveToPose to ({tx:g}, {ty:g}, {tth:g}°): top view"
    fig.text(0.10, 0.955, title, ha="left", va="top", fontsize=13, color=STYLE["text"], fontweight="semibold")
    fig.text(0.10, 0.915, "True path; a dot every 100 ms, so wider spacing = faster; wide band where the slip cap "
             "was binding", ha="left", va="top", fontsize=8.5, color=STYLE["text2"])
    fig.savefig(args.path, dpi=DPI, facecolor=STYLE["surface"])
    print(f"wrote {args.path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+", help="bench CSVs (waveform view: 1 or 2)")
    ap.add_argument("--label", action="append", default=[], help="one per CSV, in order")
    ap.add_argument("--robot", help="LOG:LINE of a recorded PIDLOG run (BEGIN line), drawn as dots")
    ap.add_argument("--zoom", help="T0:T1 time window, ms")
    ap.add_argument("--err-max", type=float, default=15.0, help="top of the error row (default 15: the last 15 in)")
    ap.add_argument("--title")
    ap.add_argument("-o", "--out", help="waveform PNG")
    ap.add_argument("--displacement", help="displacement-vs-time PNG")
    ap.add_argument("--path", help="top view (x-y) PNG of pose runs")
    ap.add_argument("--carrot", action="store_true", help="top view: also draw the carrot point trail")
    a = ap.parse_args()
    if not a.out and not a.displacement and not a.path:
        ap.error("give -o (waveforms), --displacement and/or --path")
    datas = [load_csv(p) for p in a.csv]
    labels = a.label + [os.path.basename(p) for p in a.csv[len(a.label):]]
    rob = robot_run(a.robot) if a.robot else None
    pose = all(is_pose(d) for d in datas)
    if a.out:
        if len(datas) > 2:
            ap.error("the waveform view takes one or two CSVs")
        if pose:
            waveform_pose(a, datas, labels)
        else:
            waveform(a, datas, labels, rob)
    if a.path:
        if not pose:
            ap.error("--path needs pose CSVs (tb_* --move pose)")
        path_view(a, datas, labels)
    if a.displacement:
        displacement(a, datas, labels, rob)


if __name__ == "__main__":
    main()
