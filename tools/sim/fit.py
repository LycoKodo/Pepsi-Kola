#!/opt/anaconda3/bin/python3
"""Fit the plant parameters in robot.json to recorded PID-tab runs.

Runs tb_baseline (the snapshot of LemLib, rotation sensors at --rot-ms, default 20 as on the robot)
for each recorded run's target and gains, and compares the simulated PID-tab error trace with the
recorded one, row by row matched on t. Rows lost in transfer are simply missing.

    /opt/anaconda3/bin/python3 tools/sim/fit.py                 # fit, print the table, write robot.json
    /opt/anaconda3/bin/python3 tools/sim/fit.py --report        # evaluate robot.json as it is
    /opt/anaconda3/bin/python3 tools/sim/fit.py --plot out/fit_runs.png

Cost per fit run (inches): 0.5 x RMS over the whole run + 1.0 x RMS over the last 15 in (rows the
robot logged with err <= 15, from the first such row on) + 1.0 x |overshoot difference|
+ 0.03 x |peak speed difference, in/s| + 0.002 x |settle difference, ms|. Speeds come from the
error trace itself (odometry over 20 ms) for both robot and bench: the logged motor rpm is too noisy.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from concurrent.futures import ThreadPoolExecutor

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import simlib  # noqa: E402

HERE = simlib.HERE
LOG = os.path.join(HERE, "data", "tune_2026-11-19.log")

# (BEGIN line in the copy, target in, kI, kD, role). All kP 12.5. Line 6909 (-48, kD 115) is left out:
# the robot hit a wall. 7190's BEGIN line was garbled in transfer; it is recovered from the hex dump.
RUNS = [
    (788, 24, 0.0, 100, "fit"),
    (5793, 12, 0.1, 100, "fit"),
    (5929, -12, 0.1, 100, "held"),
    (4587, 24, 0.1, 100, "held"),
    (4734, -24, 0.1, 100, "held"),
    (6073, 48, 0.1, 100, "fit"),
    (6227, -48, 0.1, 100, "held"),
    (6766, 48, 0.1, 115, "fit"),
    (7051, 48, 0.1, 115, "held"),
    (7190, -48, 0.1, 115, "held"),
]

# (name, lower, upper): physically sensible bounds
FIT = [
    ("C", 5.0, 80.0),          # in/s^2 per V
    ("KV", 12 / 80, 12 / 40),  # V per in/s: free speed 40-80 in/s at 12 V
    ("F", 0.0, 60.0),          # in/s^2
    ("DV_ACCEL", 0.5, 12.0),   # V
    ("DV_BRAKE", 0.5, 12.0),   # V
    ("LATENCY_MS", 0.0, 40.0), # ms
]


def odom_speed(t, err):
    """Speed from the error trace over 20 ms (rows t and t-20 both present), in/s."""
    d = dict(zip(t.tolist(), err.tolist()))
    ts, vs = [], []
    for k in t:
        if k - 20 in d:
            ts.append(k)
            vs.append((d[k - 20] - d[k]) / 0.02)
    return np.array(ts), np.array(vs)


def compare(rec: simlib.Run, ts, es, simrun=None):
    """Metrics for one run: simulated (ts, es) against the recorded rows."""
    t, e, _ = rec.arrays()
    sim = dict(zip(ts.tolist(), es.tolist()))
    last = es[-1]
    se = np.array([sim.get(k, last if k > ts[-1] else np.nan) for k in t])
    ok = ~np.isnan(se)
    diff = se[ok] - e[ok]
    rms_all = float(np.sqrt(np.mean(diff ** 2)))
    below = np.nonzero(e <= 15.0)[0]
    k15 = below[0] if len(below) else len(e)
    m15 = ok.copy()
    m15[:k15] = False
    d15 = se[m15] - e[m15]
    rms_15 = float(np.sqrt(np.mean(d15 ** 2))) if len(d15) else float("nan")
    tr, vr = odom_speed(t, e)
    tsv, vsv = odom_speed(ts, es)
    pk_r = float(np.max(np.abs(vr))) if len(vr) else float("nan")
    pk_s = float(np.max(np.abs(vsv))) if len(vsv) else float("nan")
    # where braking starts: distance to go when the drive voltage drops (see simlib.brake_by_voltage)
    bk_r = simlib.brake_by_voltage(rec)[1]
    bk_s = simlib.brake_by_voltage(simrun)[1] if simrun is not None else float("nan")
    st_r, st_s = simlib.settle_ms(t, e), simlib.settle_ms(ts, es)
    # shape only: RMS over the last 15 in after the time shift (+-150 ms, 10 ms steps) that fits best
    best = (float("inf"), 0)
    for sh in range(-150, 160, 10):
        sv = np.array([sim.get(k + sh, last if k + sh > ts[-1] else np.nan) for k in t[m15]])
        okk = ~np.isnan(sv)
        if okk.sum() >= max(5, 0.8 * m15.sum()):
            r = float(np.sqrt(np.mean((sv[okk] - e[m15][okk]) ** 2)))
            if r < best[0]:
                best = (r, sh)
    return dict(rms_all=rms_all, rms_15=rms_15, rms_15s=best[0], shift=best[1], over_r=simlib.overshoot(e), over_s=simlib.overshoot(es),
                peak_r=pk_r, peak_s=pk_s, brake_r=bk_r, brake_s=bk_s, settle_r=st_r, settle_s=st_s,
                n=int(ok.sum()), n15=int(m15.sum()))


def run_cost(m):
    ds = abs(m["settle_s"] - m["settle_r"]) if not (math.isnan(m["settle_s"]) or math.isnan(m["settle_r"])) else 1000
    return (0.5 * m["rms_all"] + 1.0 * m["rms_15"] + 1.0 * abs(m["over_s"] - m["over_r"])
            + 0.03 * abs(m["peak_s"] - m["peak_r"]) + 0.002 * ds)


class Problem:
    def __init__(self, runs, rot_ms, dut, fixed):
        self.runs = runs
        self.rot_ms = rot_ms
        self.dut = dut
        self.fixed = fixed
        self.pool = ThreadPoolExecutor(max_workers=os.cpu_count() or 8)
        self.best = (float("inf"), None)
        self.evals = 0

    def simulate(self, params, run_spec):
        rec, (line, target, ki, kd, role) = run_spec
        try:
            ts, es, _, summ = simlib.run_bench(target, (12.5, ki, kd), params, rot_ms=self.rot_ms, dut=self.dut)
        except Exception as exc:  # a parameter set the bench cannot run: score it badly, keep going
            print(f"bench failed for {params}: {exc}", file=sys.stderr)
            ts, es, summ = np.array([0.0, 10.0]), np.array([abs(target)] * 2), {}
        return ts, es, summ

    def evaluate(self, params, which=("fit",)):
        specs = [rs for rs in self.runs if rs[1][4] in which]
        sims = list(self.pool.map(lambda rs: self.simulate(params, rs), specs))
        out = []
        for (rec, spec), (ts, es, summ) in zip(specs, sims):
            m = compare(rec, ts, es, summ.get("_run"))
            m["cost"] = run_cost(m)
            out.append((rec, spec, m, ts, es, summ))
        return out

    def cost(self, x):
        params = dict(self.fixed)
        params.update({name: float(v) for (name, _, _), v in zip(FIT, x)})
        res = self.evaluate(params)
        c = float(np.mean([r[2]["cost"] for r in res]))
        if not math.isfinite(c):
            c = 1e3
        self.evals += 1
        if c < self.best[0]:
            self.best = (c, params)
        return c


def load_runs(log):
    runs = simlib.parse_log(log)
    out = []
    for line, target, ki, kd, role in RUNS:
        r = simlib.find_run(runs, line, target, kD=kd, kI=ki)
        kp, rki, rkd = r.gains()
        assert abs(kp - 12.5) < 1e-6 and abs(rki - ki) < 1e-6 and abs(rkd - kd) < 1e-6, r.label()
        out.append((r, (line, target, ki, kd, role)))
    return out


def table(results):
    hdr = (f"{'run':>5} {'target':>6} {'kD':>4} {'role':>4} {'rows':>7}  {'RMS':>5} {'RMS15':>5}  "
           f"{'shifted':>11}  {'over rec':>8} {'sim':>5}  {'settle rec':>10} {'sim':>5}  {'peak rec':>8} {'sim':>5}  "
           f"{'brake@ rec':>10} {'sim':>5}")
    lines = [hdr, "-" * len(hdr)]
    for rec, (line, target, ki, kd, role), m, *_ in results:
        lines.append(
            f"{rec.line:>5} {target:>+6.0f} {kd:>4.0f} {role:>4} {len(rec.rows):>3}/{rec.expected_rows():<3}  "
            f"{m['rms_all']:>5.2f} {m['rms_15']:>5.2f}  {m['rms_15s']:>4.2f}@{m['shift']:>+4d}ms  {m['over_r']:>8.2f} {m['over_s']:>5.2f}  "
            f"{m['settle_r']:>10.0f} {m['settle_s']:>5.0f}  {m['peak_r']:>8.1f} {m['peak_s']:>5.1f}  "
            f"{m['brake_r']:>10.1f} {m['brake_s']:>5.1f}")
    lines.append("units: RMS in (whole run / last 15 in; 'shifted' = last 15 in after the best time shift of the bench), overshoot in, settle ms (PID tab: first sample "
                 "after the last one outside +-1.0 in), peak = peak speed in/s from odometry over 20 ms, brake@ = in to go "
                 "when the drive voltage drops (the robot's first command after close)")
    return "\n".join(lines)


def plot_runs(results, path, rot_ms):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from simlib import STYLE, apply_axis_style

    n = len(results)
    cols = 2
    rows = (n + 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(12, 2.3 * rows), sharex=False)
    fig.patch.set_facecolor(STYLE["surface"])
    for ax, (rec, (line, target, ki, kd, role), m, ts, es, summ) in zip(axes.flat, results):
        t, e, _ = rec.arrays()
        apply_axis_style(ax)
        ax.axhline(0, color=STYLE["baseline"], lw=1, zorder=1)
        ax.plot(ts, es, color=STYLE["series"][0], lw=2, solid_capstyle="round", zorder=3, label="bench")
        ax.plot(t, e, "o", ms=4.6, color=STYLE["series"][1], mec=STYLE["surface"], mew=0.8, zorder=4, label="robot")
        ax.set_ylim(-2.5, 8)
        ax.set_xlim(0, max(ts[-1], t[-1]) + 20)
        ax.set_title(f"line {rec.line}: {target:+.0f} in, kI {ki:g} kD {kd:g} ({'fit' if role == 'fit' else 'held out'})"
                     f"   RMS {m['rms_all']:.2f} / last 15 in {m['rms_15']:.2f}   over {m['over_r']:.2f} vs {m['over_s']:.2f}",
                     loc="left", fontsize=8.5, color=STYLE["text"])
        ax.set_ylabel("error, in", fontsize=8, color=STYLE["text2"])
    for ax in axes.flat[n:]:
        ax.set_visible(False)
    axes.flat[0].legend(loc="upper right", fontsize=8, frameon=False, labelcolor=STYLE["text"])
    for ax in axes[-1]:
        ax.set_xlabel("time since the move started, ms", fontsize=8, color=STYLE["text2"])
    fig.suptitle(f"PID tab error: robot (dots) vs bench (line), tb_baseline --rot-ms {rot_ms}; y clipped to -2.5..8 in",
                 x=0.01, ha="left", fontsize=10, color=STYLE["text"])
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(path, dpi=120, facecolor=STYLE["surface"])
    print(f"wrote {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", default=LOG)
    ap.add_argument("--params", default=os.path.join(HERE, "robot.json"))
    ap.add_argument("--dut", default="baseline")
    ap.add_argument("--rot-ms", type=int, default=20)
    ap.add_argument("--report", action="store_true", help="evaluate the parameters in --params, no fitting")
    ap.add_argument("--maxiter", type=int, default=40)
    ap.add_argument("--popsize", type=int, default=12)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--no-write", action="store_true", help="do not update robot.json")
    ap.add_argument("--plot", help="small multiples of every run, robot vs bench (PNG)")
    a = ap.parse_args()

    if not os.path.exists(simlib.tb_path(a.dut)):
        sys.exit(f"{simlib.tb_path(a.dut)} not built: run make in tools/sim")
    runs = load_runs(a.log)
    with open(a.params) as fh:
        doc = json.load(fh)
    base = simlib.load_params(a.params)
    prob = Problem(runs, a.rot_ms, a.dut, base)

    if not a.report:
        from scipy.optimize import differential_evolution, minimize

        bounds = [(lo, hi) for _, lo, hi in FIT]
        x0 = [min(max(base[n], lo), hi) for n, lo, hi in FIT]
        print(f"fitting {', '.join(n for n, _, _ in FIT)} on runs "
              f"{', '.join(str(r.line) for r, s in runs if s[4] == 'fit')} (rot_ms {a.rot_ms})", flush=True)
        de = differential_evolution(prob.cost, bounds, x0=x0, maxiter=a.maxiter, popsize=a.popsize, seed=a.seed,
                                    tol=1e-4, polish=False, init="sobol", updating="immediate")
        print(f"DE: cost {de.fun:.4f} after {prob.evals} evaluations", flush=True)
        nm = minimize(prob.cost, de.x, method="Nelder-Mead",
                      options=dict(maxiter=400, xatol=1e-3, fatol=1e-4, initial_simplex=None))
        x = nm.x if nm.fun < de.fun else de.x
        x = [min(max(v, lo), hi) for v, (_, lo, hi) in zip(x, FIT)]
        fitted = {n: float(v) for (n, _, _), v in zip(FIT, x)}
        print(f"Nelder-Mead: cost {min(nm.fun, de.fun):.4f}; {prob.evals} evaluations in all")
        params = dict(base)
        params.update(fitted)
        if not a.no_write:
            for n, v in fitted.items():
                doc[n] = round(v, 5)
            doc["_fit"] = {"runs": [r.line for r, s in runs if s[4] == "fit"], "rot_ms": a.rot_ms,
                           "cost": round(float(min(nm.fun, de.fun)), 4), "log": os.path.relpath(a.log, HERE)}
            with open(a.params, "w") as fh:
                json.dump(doc, fh, indent=2)
                fh.write("\n")
            print(f"wrote {a.params}")
    else:
        params = base

    print("parameters: " + ", ".join(f"{n}={params[n]:.4g}" for n, _, _ in FIT)
          + f"  (free speed at 12 V {12 / params['KV']:.1f} in/s)")
    res = prob.evaluate(params, which=("fit", "held"))
    print(table(res))
    for role in ("fit", "held"):
        sel = [r[2] for r in res if r[1][4] == role]
        print(f"mean over {role:>4} runs: RMS {np.mean([m['rms_all'] for m in sel]):.2f} in, last 15 in "
              f"{np.mean([m['rms_15'] for m in sel]):.2f} in (time-aligned {np.mean([m['rms_15s'] for m in sel]):.2f}), |overshoot diff| "
              f"{np.mean([abs(m['over_s'] - m['over_r']) for m in sel]):.2f} in, cost {np.mean([m['cost'] for m in sel]):.3f}")
    if a.plot:
        plot_runs(res, a.plot, a.rot_ms)


if __name__ == "__main__":
    main()
