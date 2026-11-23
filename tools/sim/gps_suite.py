#!/opt/anaconda3/bin/python3
"""GPS fusion acceptance tests, revision 3 (docs/GPS_FUSION_DESIGN.md, "Acceptance tests, revision 2" as restated
by "Revision 3"), run in the host simulator.

    make -C tools/sim                              # build the testbenches first
    /opt/anaconda3/bin/python3 tools/sim/gps_suite.py [--seeds 50] [--jobs 12] [--no-figures] [--keep]

Every test runs the real LemLib code and the real filter (src/lemlib/chassis/gpsFusion.cpp) in the bench's route
mode (tools/sim/README.md, "Route mode"). It prints one table: the measured number, the limit, PASS or FAIL, and
INFO rows where the design says "report" or where a number is shown for information. Rows whose test name ends in
"?" are investigations of a failing row: one number changed at a time. Figures go to tools/sim/out/gps_*.png; the
route top view, error against time, blind sensor and shove figures are also copied to docs/sim/. Every number in
the table, and the per-seed results behind it, is saved in tools/sim/out/gps_suite_results.json.

The simulated GPS is as noisy as the design assumes: every noise term of the model (white, slowly varying, growing
with the distance to the wall) is scaled so that the sensor reads 1.50 in per axis (1 standard deviation) while
the robot stands still; row M measures it. One comparison row keeps the model's defaults (the first suite's
noise).

Choices the design does not make, all stated in the table and in tools/sim/README.md:
  - Odometry error sizes are fixed (A3a: IMU 1.2 %, wheels 1.5 %; A3b: 0.2 % and 0.5 %) but their signs are drawn
    per seed, so the seeds cover the ways the errors can combine. The IMU error is the same both ways round (a
    scale error). Random slips (as in the first suite) in A3a and A3b.
  - "Fused" is the pose the motions read with fusion On; "odometry alone" is a fusion-Off run with the same seed
    and errors (with fusion Off the GPS is never read, so that run is the same whatever the GPS does; A1 checks
    this). Where the design says "the pose is within X of odometry alone" (A7, A9, A10) it is measured within the
    On run: the pose against the pose minus every correction fed in, which is what the filter's guarantee is about.
  - Tests that do not name their odometry errors (A5, A6, A7, A10, A11) use A3a's on the route as written
    (full spin clockwise). A8 runs on A3b's (revision 3: the calibrated robot, the case the filter is for); A3a's
    rows are reported, not limited, because On needs a calibrated robot (its drift needs more correction than
    maxTotalCorrectionIn, so the total limit faults most of its healthy runs). A4 and A9 use perfect odometry, as the design says for A4 and as the first suite chose
    for A9 (A9 with A3a's errors is an INFO row).
  - Revision 3 restates A9 and A10 for the limits chosen by its rule (maxTotalCorrectionIn 6, relocMaxIn 6,
    relocMaxCount 1): A9's recovering shove is two thirds of relocMaxIn (4 in) and its faulting shove relocMaxIn +
    3 in (9 in); A10's limit is maxTotalCorrectionIn + 1 (7 in). The slow-liar rows of A10 (drift to the end, drift
    only while standing still, a stepped bias) run on the calibrated robot (A3b's errors), the case the filter is
    now meant for. Their "time above 4 in" is the time the fused error is more than 4 in worse than odometry alone.
"""

import argparse
import concurrent.futures
import json
import math
import os
import shutil
import statistics
import subprocess
import sys
import tempfile
import threading

import numpy as np

SIM = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(SIM))
TB_LIVE = os.path.join(SIM, "build", "live", "tb_live")
TB_PREGPS = os.path.join(SIM, "build", "pregps", "tb_pregps")
ROUTE = os.path.join(SIM, "routes", "skills60.txt")
OUT = os.path.join(SIM, "out")
DOCS_SIM = os.path.join(REPO, "docs", "sim")

# ---------------------------------------------------------------------------------------------- test settings
# The GPS model's noise terms and their defaults (tools/sim/bench/gps_model.hpp). The acceptance rows scale all
# three by NOISE_SCALE: 1.535 gives 1.50 in per axis standing still (measured by row M over 30 seeds). A11's noisy
# sensor scales them by NOISY_SCALE for 3.6 in. The model's own defaults (scale 1) are the first suite's noise
MODEL_NOISE = {"white_sd_in": 0.5, "slow_sd_in": 0.75, "wall_sd_per_100in": 0.4}
NOISE_SCALE = 1.535
NOISY_SCALE = NOISE_SCALE * 3.6 / 1.5
# what the filter is told (lemlib::gpsfusion::Config defaults, the design's numbers): used for limits only
# Revision 3: maxTotalCorrectionIn, relocMaxIn and relocMaxCount chosen by its rule on 2026-11-22 (6, 6, 1)
CFG = {"sigmaGpsIn": 1.5, "slewInPerS": 2.0, "maxStepIn": 1.0, "maxTotalCorrectionIn": 6.0, "relocMaxIn": 6.0,
       "relocMaxCount": 1, "relocSamples": 4}
ODOM_TICK_S = 0.010
# A3a / A3b: odometry errors (sizes from the design; signs drawn per seed) and random slips
A3A = {"imu": 0.012, "wheel": 0.015}
A3B = {"imu": 0.002, "wheel": 0.005}
SLIP_PER_IN = 0.03  # random slip: about one slip per 33 in each tracking wheel rolls...
SLIP_MAX_IN = 0.3  # ...losing up to 0.3 in of its travel
# A6: outliers, and the view lost for 2 s several times (two drifting, two frozen)
OUTLIER_PROB = 0.1
OUTLIER_MIN_IN, OUTLIER_MAX_IN = 6, 24
VIEW_LOSSES = ["8000:2000:1", "21000:2000:0", "36000:2000:1", "50000:2000:0"]
# A8: the true delay of the sensor (the filter is told 60 ms)
DELAYS_MS = [40, 60, 80, 100, 120, 150]
# A9: a sideways shove (to the robot's right) the wheels do not see, during the 0.5 s stop at (48, 12) after the
# long drive across the middle (route step 22), spread over 200 ms
SHOVE_STEP = 22
# A9 (revision 3): the shove that must be recovered from is about two thirds of relocMaxIn, the one that must
# fault relocMaxIn + 3 in; the sweep is reported
A9_RECOVER_IN = round(2 * CFG["relocMaxIn"] / 3)
A9_FAULT_IN = CFG["relocMaxIn"] + 3
A9_SWEEP_IN = [2, 4, 6, 8, 10, 12]
# A10: the long drifting view loss
BLIND_LOSS = "15000:20000:1:1.5"
BLIND_LOSS_START_S, BLIND_LOSS_LENGTH_S = 15, 20
# A10 (revision 3), slow liars on the calibrated robot: a blind drift from 15 s to the end of the route (the loss is
# longer than the route), drift only while standing still (bench drift_mode 2, at the revision 2 attacker's
# smallest rate), and a bias that steps 2 in every 5 s from 10 s, three times (2, 4, 6 in), in a direction per seed
SLOW_DRIFTS_IN_S = [0.1, 0.2, 0.3]
SLOW_LOSS = "15000:100000:1:{rate}"
STILL_DRIFT_IN_S = 0.5
BIAS_STEP = {"in": 2, "ms": 5000, "start_ms": 10000, "count": 3}
EXCESS_LONG_IN = 4.0  # "time above 4 in": fused more than this worse than odometry alone
# A3b (revision 3): the end error also measured this long after the last move (the route waits 4 s at its end)
END_MOMENTS_MS = [500, 1000, 2000, 4000]
# A6 (revision 3, info): the model's dead zone moved to the filter's minViewIn (20) and past it
DEAD_ZONES_IN = [20, 24]
# A7 (revision 3): start poses on the mirror lines (x = 0 for left-right, y = 0 for front-back) and the route's
# own, each at headings 0, 90 and 270. A start is "on the line" for a mirror when the mirrored frame reads the
# start pose itself: left-right on x = 0 facing 0 or 180, front-back on y = 0 facing 90 or 270 (turned 180 never:
# its heading is always 180 off)
A7_STARTS = [("route start (-60, -36)", -60, -36), ("on x = 0 (0, -36)", 0, -36), ("on y = 0 (-36, 0)", -36, 0)]
A7_HEADINGS = [0, 90, 270]
A7_MIRRORS = {1: "field turned 180", 2: "mirrored left-right", 3: "mirrored front-back"}
# the first reviewer's turn-first routine (revision 3 item 4: the heading fault judges only slow readings)
TURN_FIRST_ROUTE = "turnfirst.txt"
# A5: standard moves, each after a lead-in leg (tools/sim/routes/std_*.txt)
STD_MOVES = ["std_drive24", "std_drive48", "std_back24", "std_turn90", "std_pose"]
# A1: the bench's existing single moves
SINGLE_MOVES = [
    "--move drive --target 12", "--move drive --target 24", "--move drive --target 48",
    "--move drive --target 72", "--move drive --target -24", "--move drive --target 48 --rot-ms 20 --rot-lock",
    "--move drive --target 24 --set MU_LAT=0.3", "--move turn --target 45", "--move turn --target 90",
    "--move turn --target -90", "--move turn --target 165", "--move pose --x 24 --y 24 --theta 90",
    "--move pose --x 2 --y 4 --theta 30", "--move pose --x 12 --y 12 --theta 90",
    "--move pose --x -24 --y -24 --theta 0 --backwards",
    "--move pose --x 24 --y 24 --theta 90 --drift 8 --set MU_LAT=0.3",
]
FAULT_NAMES = ["none", "StartDisagree", "RelocTooFar", "RelocTooMany", "TotalTooLarge", "HeadingDisagree"]
# per-tick columns kept from the route CSVs (the CSVs themselves are deleted straight away)
COLS = ["t_ms", "step", "moving", "true_x", "true_y", "true_heading", "pose_heading", "cmdL", "cmdR", "err_odom",
        "err_fused"]


def noise(scale):
    out = []
    for key, value in MODEL_NOISE.items():
        out += ["--gps", f"{key}={value * scale:.6g}"]
    return out


NOISE = noise(NOISE_SCALE)


def odom_errors(sizes, slips=True):
    """--set arguments for odometry errors: the sizes are fixed, the signs come from the seed (the same draw the
    first suite made, so A3a clockwise is the first suite's A3)."""
    def args(seed):
        if sizes is None:
            return []
        rng = np.random.RandomState(10_000 + seed)
        s_imu, s_vert, s_horiz = rng.choice([-1, 1], size=3)
        out = ["--set", f"IMU_SCALE_ERR_CW={sizes['imu'] * s_imu}", "--set", f"IMU_SCALE_ERR_CCW={sizes['imu'] * s_imu}",
               "--set", f"VERT_SCALE_ERR={sizes['wheel'] * s_vert}", "--set", f"HORIZ_SCALE_ERR={sizes['wheel'] * s_horiz}"]
        if slips:
            out += ["--set", f"SLIP_PER_IN={SLIP_PER_IN}", "--set", f"SLIP_MAX_IN={SLIP_MAX_IN}"]
        return out
    return args


ERRORS = {"a3a": odom_errors(A3A), "a3b": odom_errors(A3B), "perfect": odom_errors(None)}


def a6_extras():
    args = ["--gps", f"outlier_prob={OUTLIER_PROB}", "--gps", f"outlier_min_in={OUTLIER_MIN_IN}",
            "--gps", f"outlier_max_in={OUTLIER_MAX_IN}"]
    for loss in VIEW_LOSSES:
        args += ["--gps-loss", loss]
    return args


def per_seed_direction(seed, salt, size):
    """A vector of the given length in a direction drawn from the seed: (x, y) in field inches."""
    a = np.random.RandomState(salt + seed).uniform(0, 2 * math.pi)
    return size * math.sin(a), size * math.cos(a)


# ---------------------------------------------------------------------------------------------- running the bench
TMP = None  # a scratch folder for CSVs and traces, removed at the end
ROUTES = {}  # name -> route file
_counter = [0]
_lock = threading.Lock()
ON_RUNS = []  # (test, seed, largest correction in one tick) for every fusion-On run of the suite (A5)


def tmp(name):
    return os.path.join(TMP, name)


def unique(prefix):
    with _lock:
        _counter[0] += 1
        return tmp(f"{prefix}_{_counter[0]}")


def bench(args, route="cw", tb=TB_LIVE, cols=None, trace=None, test=""):
    """Run one route through the bench. Returns its JSON summary, and the CSV columns `cols` as numpy arrays when
    asked for (the CSV is deleted again)."""
    cmd = [tb, "--route", ROUTES.get(route, route)] + [str(a) for a in args] + ["--summary-json"]
    csv = None
    if cols:
        csv = unique("run") + ".csv"
        cmd += ["--csv", csv]
    if trace:
        cmd += ["--pose-trace", trace]
    done = subprocess.run(cmd, capture_output=True, text=True)
    if done.returncode != 0:
        raise RuntimeError(f"{' '.join(cmd)}\n{done.stderr}")
    summary = json.loads(done.stdout)
    if summary.get("fusion") == "on" and "filter" in summary:
        ON_RUNS.append((test, summary["seed"], summary["filter"]["max_step_in"]))
    if not cols:
        return summary
    import pandas as pd
    frame = pd.read_csv(csv, usecols=cols)
    os.remove(csv)
    return summary, {c: frame[c].to_numpy() for c in cols}


def run_single(args, tb, csv, trace):
    cmd = [tb] + args.split() + ["--csv", csv, "--pose-trace", trace]
    done = subprocess.run(cmd, capture_output=True, text=True)
    if done.returncode != 0:
        raise RuntimeError(f"{' '.join(cmd)}\n{done.stderr}")


def same_file(a, b):
    with open(a, "rb") as fa, open(b, "rb") as fb:
        return fa.read() == fb.read()


def trace_difference(a, b):
    """Largest difference in x or y (inches) between two pose traces over the ticks both have, and whether they
    are byte-identical"""
    if same_file(a, b):
        return 0.0, True
    def load(path):
        rows = {}
        with open(path) as f:
            for line in f:
                t, x, y, th = line.split()
                rows[int(t)] = (float.fromhex(x), float.fromhex(y), float.fromhex(th))
        return rows
    ra, rb = load(a), load(b)
    worst = 0.0
    for t in set(ra) & set(rb):
        worst = max(worst, abs(ra[t][0] - rb[t][0]), abs(ra[t][1] - rb[t][1]))
    return worst, False


def parallel(fn, items, jobs):
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
        return list(pool.map(fn, items))


def median(values):
    values = [v for v in values if v is not None and not (isinstance(v, float) and math.isnan(v))]
    return statistics.median(values) if values else float("nan")


def faults_text(summaries):
    """'none' or 'TotalTooLarge 3, HeadingDisagree 1' over a list of On summaries."""
    count = {}
    for s in summaries:
        name = s["filter"]["fault_name"]
        if name != "none":
            count[name] = count.get(name, 0) + 1
    return ", ".join(f"{k} {v}" for k, v in sorted(count.items())) if count else "none"


def on_args(seed, errors="a3a", extra=(), noise_args=None):
    return ERRORS[errors](seed) + (NOISE if noise_args is None else noise_args) + list(extra) + ["--seed", seed]


# Odometry alone: a fusion-Off run per (errors, route, seed). With fusion Off the GPS is never read, so one Off
# run serves every GPS setting with the same odometry errors (A1 checks that Off is untouched by everything)
OFF = {}


def off_run(errors, route, seed):
    key = (errors, route, seed)
    if key not in OFF:
        OFF[key] = bench(ERRORS[errors](seed) + ["--seed", seed, "--fusion", "off"], route=route, cols=COLS, test="off")
    return OFF[key]


def fill_off(errors, route, seeds, jobs):
    parallel(lambda s: off_run(errors, route, s), seeds, jobs)


# ---------------------------------------------------------------------------------------------- measures
def cmd_steps(c):
    """Tick-to-tick change of each motor command (10 ms apart) while a move runs (same step, moving at both ticks).
    Both sides pooled."""
    ok = (c["moving"][1:] == 1) & (c["moving"][:-1] == 1) & (c["step"][1:] == c["step"][:-1])
    return np.concatenate([np.abs(np.diff(c["cmdL"]))[ok], np.abs(np.diff(c["cmdR"]))[ok]]).astype(np.int16)


def cmd_compare(on, off):
    """The first suite's measure: largest motor-command difference against Off while the two real robots are
    still within 0.1 in and 0.1 deg of each other and on the same step (any difference then comes from the
    correction: a move starting from a corrected pose steers differently)."""
    n = min(len(on["t_ms"]), len(off["t_ms"]))
    together = ((on["step"][:n] == off["step"][:n]) &
                (np.hypot(on["true_x"][:n] - off["true_x"][:n], on["true_y"][:n] - off["true_y"][:n]) <= 0.1) &
                (np.abs((on["true_heading"][:n] - off["true_heading"][:n] + 180) % 360 - 180) <= 0.1))
    split = int(np.argmax(~together)) if (~together).any() else n
    if split == 0:
        return 0.0
    return float(np.max(np.maximum(np.abs(on["cmdL"][:split] - off["cmdL"][:split]),
                                   np.abs(on["cmdR"][:split] - off["cmdR"][:split]))))


def stop_errors(summary, cols, column):
    """{step n: error at the last tick of that stop} for every wait step of the route."""
    out = {}
    step = cols["step"]
    for s in summary["step_list"]:
        if s["kind"] != "wait":
            continue
        idx = np.nonzero(step == s["n"])[0]
        if len(idx):
            out[s["n"]] = float(cols[column][idx[-1]])
    return out


def aligned(a_t, a_v, b_t, b_v):
    """Two per-tick series on one time base (10 ms ticks). The shorter run is held at its last value: its robot
    stands still once its route has ended."""
    t0 = min(a_t[0], b_t[0])
    n = int(max(a_t[-1], b_t[-1]) - t0) // 10 + 1
    t = t0 + 10 * np.arange(n)
    def fill(tt, vv):
        out = np.empty(n)
        idx = ((tt - t0) // 10).astype(int)
        out[:] = np.nan
        out[idx] = vv
        # hold the last value (and fill any gap with the value before it)
        for i in range(1, n):
            if np.isnan(out[i]):
                out[i] = out[i - 1]
        if np.isnan(out[0]):
            out[0] = vv[0]
        return out
    return t, fill(a_t, a_v), fill(b_t, b_v)


# ---------------------------------------------------------------------------------------------- the tests
ROWS = []  # (test, what, measured, limit, PASS/FAIL/INFO)
RESULTS = {}
ORDER = ["M", "A1", "A2", "A3a", "A3b", "A4", "A5", "A6", "A7", "A8", "A9", "A10", "A11", "S", "N"]


def row(test, what, measured, limit, ok):
    verdict = "INFO" if ok is None else ("PASS" if ok else "FAIL")
    ROWS.append((test, what, measured, limit, verdict))


def test_model(jobs, seeds):
    """The simulated GPS itself: refresh rate, delay (best fit against the truth) and noise standing still."""
    import pandas as pd

    def still_errors(job):
        scale, seed = job
        _, c = bench(["--seed", seed, "--fusion", "off"] + noise(scale), cols=[
            "t_ms", "true_x", "true_y", "true_heading", "gps_x", "gps_y", "gps_new", "gps_state", "gps_outlier"],
            test="model")
        t = c["t_ms"].astype(float)
        new = (c["gps_new"] == 1) & (c["gps_state"] == 0) & (c["gps_outlier"] == 0)
        taken = t[new] - 60.0  # each reading shows where the camera (at the robot's centre) was 60 ms before
        x0, y0, h0 = (np.interp(taken, t, c[k]) for k in ("true_x", "true_y", "true_heading"))
        x1, y1, h1 = (np.interp(taken - 100, t, c[k]) for k in ("true_x", "true_y", "true_heading"))
        # standing still: the robot moved under 0.005 in and 0.05 deg in the 100 ms before the reading was taken
        still = (np.hypot(x0 - x1, y0 - y1) < 0.005) & (np.abs(h0 - h1) < 0.05)
        return scale, c["gps_x"][new][still] - x0[still], c["gps_y"][new][still] - y0[still]

    scales = [NOISE_SCALE, 1.0, NOISY_SCALE]
    results = parallel(still_errors, [(sc, s) for sc in scales for s in seeds], jobs)
    measured = {}
    for scale in scales:
        ex = np.concatenate([r[1] for r in results if r[0] == scale])
        ey = np.concatenate([r[2] for r in results if r[0] == scale])
        measured[scale] = {"readings": int(len(ex)), "sd_x_in": float(np.std(ex)), "sd_y_in": float(np.std(ey)),
                           "per_axis_in": float(math.sqrt((np.var(ex) + np.var(ey)) / 2))}
    _, c = bench(["--seed", 1, "--fusion", "off"] + NOISE, cols=["t_ms", "true_x", "true_y", "gps_x", "gps_y", "gps_new",
                                                                 "gps_state"], test="model")
    t = c["t_ms"]
    new = (c["gps_new"] == 1) & (c["gps_state"] == 0)
    rate = c["gps_new"].sum() / (t[-1] / 1000)
    best = None
    for lag in range(0, 201, 10):
        ex = c["gps_x"][new] - np.interp(t[new] - lag, t, c["true_x"])
        ey = c["gps_y"][new] - np.interp(t[new] - lag, t, c["true_y"])
        rms = float(np.sqrt(np.mean(ex ** 2 + ey ** 2)))
        if best is None or rms < best[1]:
            best = (lag, rms)
    RESULTS["model"] = {"outputs_per_s": float(rate), "best_fit_delay_ms": best[0], "noise_scale": NOISE_SCALE,
                        "noisy_scale": NOISY_SCALE, "seeds": len(seeds),
                        "standing_still": {f"{k:.4g}": v for k, v in measured.items()}}
    m = measured[NOISE_SCALE]
    row("M", f"GPS model as the acceptance rows run it (noise terms x{NOISE_SCALE}): noise per axis standing still", f"{m['per_axis_in']:.2f} in (x {m['sd_x_in']:.2f}, "
             f"y {m['sd_y_in']:.2f}; {m['readings']} readings, {len(seeds)} seeds)", "1.5 in (design)", None)
    d = measured[1.0]
    n = measured[NOISY_SCALE]
    row("M", "  ... model defaults (the first suite's noise) / A11's noisy sensor (noise terms "
             f"x{NOISY_SCALE:.3f})", f"{d['per_axis_in']:.2f} / {n['per_axis_in']:.2f} in",
        "0.9 (first suite) / 3.6 in", None)
    row("M", "  ... outputs per second, best-fit delay", f"{rate:.1f}/s, {best[0]} ms", "25/s, 60 ms", None)


def test_a1(jobs):
    """Mode Off: every existing drive, turn and pose result, and the route, byte-identical to pre-GPS LemLib."""
    def single(i):
        out = []
        for name, tb in (("live", TB_LIVE), ("pregps", TB_PREGPS)):
            run_single(SINGLE_MOVES[i], tb, tmp(f"a1_{i}_{name}.csv"), tmp(f"a1_{i}_{name}.trace"))
        out.append(same_file(tmp(f"a1_{i}_live.csv"), tmp(f"a1_{i}_pregps.csv")))
        out.append(same_file(tmp(f"a1_{i}_live.trace"), tmp(f"a1_{i}_pregps.trace")))
        return out

    # the five route seeds of the first suite, plus the GPS model's revision 2 options (Off must ignore them all)
    routes = [(s, ERRORS["a3a"](s) + NOISE + a6_extras()) for s in range(1, 6)]
    routes += [(6, ERRORS["a3a"](6) + ["--gps", "strip_walls=1"]),
               (7, ERRORS["a3a"](7) + ["--gps", "bias_x_in=2"]),
               (8, ERRORS["a3a"](8) + ["--gps", "no_strips=2", "--gps", "no_strip_at_truth=1"]),
               (9, ERRORS["a3a"](9) + ["--gps-loss", BLIND_LOSS])]

    def route(job):
        seed, args = job
        args = args + ["--seed", seed, "--fusion", "off"]
        for name, tb in (("live", TB_LIVE), ("pregps", TB_PREGPS)):
            bench(args + ["--csv", tmp(f"a1r_{seed}_{name}.csv")], tb=tb, trace=tmp(f"a1r_{seed}_{name}.trace"),
                  test="A1")
        return [same_file(tmp(f"a1r_{seed}_live.csv"), tmp(f"a1r_{seed}_pregps.csv")),
                same_file(tmp(f"a1r_{seed}_live.trace"), tmp(f"a1r_{seed}_pregps.trace"))]

    singles = parallel(single, range(len(SINGLE_MOVES)), jobs)
    rr = parallel(route, routes, jobs)
    files = sum(len(s) for s in singles) + sum(len(r) for r in rr)
    differ = sum(not ok for s in singles for ok in s) + sum(not ok for r in rr for ok in r)
    RESULTS["A1"] = {"files_compared": files, "files_different": differ, "single_moves": SINGLE_MOVES,
                     "route_runs": [a for _, a in routes]}
    row("A1", f"Off vs pre-GPS LemLib, CSV and exact pose trace: {len(SINGLE_MOVES)} moves, {len(routes)} route runs "
              "(4 with new GPS options)", f"{differ} of {files} files differ", "0 differ", differ == 0)


def test_a2(seeds, jobs):
    """Mode On with no sensor, or a sensor that never sees a strip: the pose identical to Off."""
    variants = [("no sensor", ["--no-gps"]),
                ("never sees a strip, frozen", ["--gps", "no_strips=1"]),
                ("never sees a strip, drifting from the centre", ["--gps", "no_strips=2"])]
    for seed in seeds:  # the Off traces first, so the parallel jobs do not race to write them
        bench(ERRORS["a3a"](seed) + ["--seed", seed, "--fusion", "off"], trace=tmp(f"a2_off_{seed}.trace"), test="A2")

    def one(job):
        i, extra, seed = job
        trace = tmp(f"a2_{i}_{seed}.trace")
        summary = bench(on_args(seed, extra=extra) + ["--fusion", "on"], trace=trace, test="A2")
        diff, identical = trace_difference(tmp(f"a2_off_{seed}.trace"), trace)
        return i, seed, diff, identical, summary

    results = parallel(one, [(i, e, s) for i, (_, e) in enumerate(variants) for s in seeds], jobs)
    out = {}
    for i, (name, _) in enumerate(variants):
        mine = [r for r in results if r[0] == i]
        worst = max(r[2] for r in mine)
        identical = sum(r[3] for r in mine)
        out[name] = {"worst_pose_difference_in": worst, "identical_runs": identical, "runs": len(mine),
                     "faults": faults_text([r[4] for r in mine]),
                     "accepted": sum(r[4]["filter"]["accepted"] for r in mine)}
        row("A2", f"On, {name} ({len(mine)} seeds, A3a errors): pose vs Off",
            f"{identical}/{len(mine)} identical, worst {worst:.3g} in; faults: {out[name]['faults']}", "identical", identical == len(mine))
    RESULTS["A2"] = out


def test_a3(seeds, jobs):
    """A3a (both spins) and A3b (route as written), plus the INFO rows the task asks for."""
    out = {}
    runs = {}
    for label, errors, route in (("A3a cw", "a3a", "cw"), ("A3a ccw", "a3a", "ccw"), ("A3b cw", "a3b", "cw"),
                                 ("A3b ccw", "a3b", "ccw")):
        fill_off(errors, route, seeds, jobs)
        on = parallel(lambda s: bench(on_args(s, errors) + ["--fusion", "on"], route=route, cols=COLS, test=label),
                      seeds, jobs)
        runs[label] = {s: (OFF[(errors, route, s)], on[i]) for i, s in enumerate(seeds)}
    RUNS["a3"] = runs
    for label, r in runs.items():
        fused = [r[s][1][0]["fused"]["end"] for s in seeds]
        odom = [r[s][0][0]["odom"]["end"] for s in seeds]
        ons = [r[s][1][0] for s in seeds]
        out[label] = {
            "median_fused_end_in": median(fused), "median_odom_end_in": median(odom),
            "ratio_of_medians": median(fused) / median(odom),
            "median_paired_difference_in": median([f - o for f, o in zip(fused, odom)]),
            "median_fused_max_in": median([x["fused"]["max"] for x in ons]),
            "median_odom_max_in": median([r[s][0][0]["odom"]["max"] for s in seeds]),
            "median_fused_rms_in": median([x["fused"]["rms"] for x in ons]),
            "median_odom_rms_in": median([r[s][0][0]["odom"]["rms"] for s in seeds]),
            "faults": faults_text(ons), "fault_runs": sum(x["filter"]["fault"] != 0 for x in ons),
            "total_too_large_runs": sum(x["filter"]["fault_name"] == "TotalTooLarge" for x in ons),
            "relocalisations": sum(x["filter"]["relocalisations"] for x in ons),
            "per_seed": {s: {"fused_end": r[s][1][0]["fused"]["end"], "odom_end": r[s][0][0]["odom"]["end"],
                             "fault": r[s][1][0]["filter"]["fault_name"],
                             "fault_ms": r[s][1][0]["filter"]["fault_ms"],
                             "accepted": r[s][1][0]["filter"]["accepted"],
                             "relocalisations": r[s][1][0]["filter"]["relocalisations"],
                             "max_decided_in": r[s][1][0]["filter"]["max_decided_in"]} for s in seeds},
        }
    # Revision 3: switching the filter On needs a calibrated robot (the limits 6/6/1 were picked for it), so the
    # uncalibrated robot's rows are reported, not limited. Its drift needs more than maxTotalCorrectionIn of
    # correction, so the total limit faults most healthy runs and the pose goes back to odometry alone
    for spin in ("cw", "ccw"):
        v = out[f"A3a {spin}"]
        f, o = v["median_fused_end_in"], v["median_odom_end_in"]
        name = "clockwise (route as written)" if spin == "cw" else "counter-clockwise"
        cause = (f"outside what the filter is for (On needs a calibrated robot): an uncalibrated robot needs more than "
                 f"maxTotalCorrectionIn {CFG['maxTotalCorrectionIn']:g} in of correction; faults in "
                 f"{v['fault_runs']} of {len(seeds)} healthy runs: {v['faults']}")
        row("A3a", f"uncalibrated robot (IMU 1.2 %, wheels 1.5 %), slips, spin {name}, {len(seeds)} seeds: median "
                   f"fused end error (reported, {cause})", f"{f:.2f} in", "(was <= 2.0 in)", None)
        row("A3a", f"  ... as a fraction of odometry alone (median fused / median Off; reported, same cause)",
            f"{100 * f / o:.0f} % ({f:.2f} / {o:.2f} in)", "(was <= 60 %)", None)
    v = out["A3b cw"]
    f, o = v["median_fused_end_in"], v["median_odom_end_in"]
    row("A3b", f"calibrated (IMU 0.2 %, wheels 0.5 %), slips, route as written, {len(seeds)} seeds: median fused end "
               "error", f"{f:.2f} in", "<= 1.0 in", f <= 1.0)
    row("A3b", "  ... worse than odometry alone by (median fused - median Off)",
        f"{f - o:+.2f} in ({f:.2f} vs {o:.2f}; paired median {v['median_paired_difference_in']:+.2f})", "<= +0.2 in",
        f - o <= 0.2)
    v = out["A3b ccw"]
    row("A3b", "  ... the same with the full spin counter-clockwise (not in the design)",
        f"{v['median_fused_end_in']:.2f} vs {v['median_odom_end_in']:.2f} in "
        f"({v['median_fused_end_in'] - v['median_odom_end_in']:+.2f})", "(1.0 in / +0.2 in)", None)
    # INFO: faults in healthy runs are a cost of the limits and must be visible
    for label in runs:
        v = out[label]
        row(label.split()[0], f"  faults in healthy runs, {label}: TotalTooLarge / any fault",
            f"{v['total_too_large_runs']} / {v['fault_runs']} of {len(seeds)} ({v['faults']})", "(info)", None)
    # INFO: stops at which fused is more than 1 in worse than odometry alone (A3a, both spins)
    worse, total, worst = 0, 0, 0.0
    seeds_worse = set()
    per_stop = {}  # (label, step) -> differences over seeds
    for label in ("A3a cw", "A3a ccw"):
        for s in seeds:
            (off, offc), (on, onc) = runs[label][s]
            fo = stop_errors(off, offc, "err_odom")
            ff = stop_errors(on, onc, "err_fused")
            for n in fo:
                if n in ff:
                    total += 1
                    d = ff[n] - fo[n]
                    per_stop.setdefault((label, n), []).append(d)
                    worst = max(worst, d)
                    if d > 1.0:
                        worse += 1
                        seeds_worse.add((label, s))
    # the stop where it happens most (ties: the larger median difference)
    key = max(sorted(per_stop), key=lambda k: (sum(d > 1.0 for d in per_stop[k]), median(per_stop[k])))
    out["stops_fused_worse_by_1in"] = {"stops": total, "worse": worse, "worst_in": worst,
                                       "runs_with_one": len(seeds_worse),
                                       "worst_stop": {"run": key[0], "step": key[1],
                                                      "runs_worse_by_1in": sum(d > 1.0 for d in per_stop[key]),
                                                      "median_difference_in": median(per_stop[key])}}
    row("A3a", "  stops at which fused is > 1 in worse than odometry alone (end of each wait, both spins)",
        f"{worse} of {total} ({100 * worse / total:.1f} %), in {len(seeds_worse)} of {2 * len(seeds)} runs; "
        f"worst {worst:+.1f} in", "(info)", None)
    row("A3a", "  ... the stop where it happens most", f"{key[0]}, the wait at step {key[1]}: "
        f"{sum(d > 1.0 for d in per_stop[key])}/{len(per_stop[key])} runs, median {median(per_stop[key]):+.1f} in",
        "(info)", None)
    RESULTS["A3"] = out


def test_a3b_rev3(seeds, jobs):
    """Revision 3 rows for A3b: stops more than 1 in worse and more than 1 in better than odometry alone (both
    spins), and the end error measured 0.5, 1, 2 and 4 s after the last move (a copy of the route that waits 4 s at
    its end instead of 0.5 s; everything before that wait is the same route)."""
    runs = RUNS["a3"]
    worse = better = total = 0
    per_spin = {}
    for label in ("A3b cw", "A3b ccw"):
        w = b = n = 0
        for s in seeds:
            (off, offc), (on, onc) = runs[label][s]
            fo = stop_errors(off, offc, "err_odom")
            ff = stop_errors(on, onc, "err_fused")
            for k in fo:
                if k in ff:
                    n += 1
                    w += ff[k] - fo[k] > 1.0
                    b += ff[k] - fo[k] < -1.0
        per_spin[label] = {"stops": n, "worse_by_1in": w, "better_by_1in": b}
        worse, better, total = worse + w, better + b, total + n
    out = {"stops": per_spin}
    row("A3b", f"  stops, both spins, {len(seeds)} seeds each: fused more than 1 in worse / more than 1 in better than "
               "odometry alone", f"{worse} / {better} of {total} stops ({100 * worse / total:.1f} % / "
                                 f"{100 * better / total:.1f} %)", "report", None)
    ends = {}
    for spin in ("cw", "ccw"):
        route = spin + "_end4"
        cols = ["t_ms", "err_odom", "err_fused"]
        def one(job):
            fusion, seed = job
            errors_args = ERRORS["a3b"](seed)
            args = (errors_args + ["--seed", seed, "--fusion", "off"]) if fusion == "off" else \
                (on_args(seed, "a3b") + ["--fusion", "on"])
            summary, c = bench(args, route=route, cols=cols, test="A3b end")
            last = max(st["end_ms"] for st in summary["step_list"] if st["kind"] != "wait")
            col = "err_odom" if fusion == "off" else "err_fused"
            vals = []
            for d in END_MOMENTS_MS:
                k = int(np.searchsorted(c["t_ms"], last + d))
                vals.append(float(c[col][min(k, len(c[col]) - 1)]))
            return fusion, seed, vals
        res = parallel(one, [(f, sd) for f in ("off", "on") for sd in seeds], jobs)
        per = {}
        for i, d in enumerate(END_MOMENTS_MS):
            fused = median([r[2][i] for r in res if r[0] == "on"])
            odom = median([r[2][i] for r in res if r[0] == "off"])
            per[d] = {"median_fused_in": fused, "median_odom_in": odom}
        ends[spin] = per
        row("A3b", f"  end error {', '.join(f'{d / 1000:g}' for d in END_MOMENTS_MS)} s after the last move, spin "
                   f"{'clockwise' if spin == 'cw' else 'counter-clockwise'} (fused vs odometry alone, medians)",
            ", ".join(f"{per[d]['median_fused_in']:.2f} vs {per[d]['median_odom_in']:.2f}" for d in END_MOMENTS_MS),
            "report", None)
    out["end_moments_ms"] = ends
    RESULTS["A3b_rev3"] = out


def investigate_a3a(seeds, jobs):
    """A3a counter-clockwise fails its 60 % ratio: one number changed at a time, to find which choice causes it."""
    runs = RUNS["a3"]
    out = {}

    def ratio(label_runs, measure=lambda on: on["fused"]["end"]):
        f = median([measure(label_runs[s][1][0]) for s in seeds])
        o = median([label_runs[s][0][0]["odom"]["end"] for s in seeds])
        return f, o

    # 1. why the counter-clockwise spin is harder: the IMU's 1.2 % scale error times a larger net rotation
    def heading_error(label):
        vals = []
        for s in seeds:
            c = runs[label][s][1][1]
            late = c["t_ms"] >= 30000
            vals.append(float(np.median(np.abs((c["pose_heading"][late] - c["true_heading"][late] + 180) % 360 - 180))))
        return median(vals)
    hcw, hccw = heading_error("A3a cw"), heading_error("A3a ccw")
    out["heading_error_after_30s_deg"] = {"cw": hcw, "ccw": hccw}
    row("A3a?", "  investigation: odometry's heading error after 30 s (median |error|), clockwise / counter-clockwise",
        f"{hcw:.1f} / {hccw:.1f} deg", "", None)
    # 2. the filter's own estimate (pose + pending) against the pose the motions read
    for label in ("A3a cw", "A3a ccw"):
        f, o = ratio(runs[label], lambda on: on["est_end_err"])
        p, _ = ratio(runs[label])
        out[f"estimate_{label}"] = {"estimate": f, "pose": p, "odom": o}
    e_cw, e_ccw = out["estimate_A3a cw"], out["estimate_A3a ccw"]
    row("A3a?", "  investigation: end error of the filter's estimate (pose + pending, not yet fed in), cw / ccw",
        f"{e_cw['estimate']:.2f} ({100 * e_cw['estimate'] / e_cw['odom']:.0f} %) / {e_ccw['estimate']:.2f} "
        f"({100 * e_ccw['estimate'] / e_ccw['odom']:.0f} %); the pose: {100 * e_cw['pose'] / e_cw['odom']:.0f} / "
        f"{100 * e_ccw['pose'] / e_ccw['odom']:.0f} %", "", None)

    # 3. one number at a time, counter-clockwise
    def variant(tag, extra=(), route="ccw", cols=None):
        on = parallel(lambda s: bench(on_args(s, extra=extra) + ["--fusion", "on"], route=route, cols=cols,
                                      test=tag), seeds, jobs)
        return on
    long_end = tmp("skills60_ccw_long_end.txt")
    with open(ROUTES["ccw"]) as f:
        lines = f.read().rstrip("\n").split("\n")
    assert lines[-1].startswith("wait 500")
    lines[-1] = "wait 2000"
    with open(long_end, "w") as f:
        f.write("\n".join(lines) + "\n")
    ROUTES["ccw_long_end"] = long_end
    fill_off("a3a", "ccw_long_end", seeds, jobs)
    on = variant("A3a? long end", route="ccw_long_end")
    f = median([r["fused"]["end"] for r in on])
    o = median([OFF[("a3a", "ccw_long_end", s)][0]["odom"]["end"] for s in seeds])
    out["final_stop_2s"] = {"fused": f, "odom": o}
    row("A3a?", "  investigation: ccw with the route's last stop 2 s instead of 0.5 s (time to feed the rest in)",
        f"{f:.2f} / {o:.2f} in = {100 * f / o:.0f} %", "60 %", None)
    texts = []
    for tag, extra in (("slewInPerS 4", ["--fz", "slewInPerS=4"]), ("slewInPerS 8", ["--fz", "slewInPerS=8"]),
                       ("maxSpeedInPerS 12", ["--fz", "maxSpeedInPerS=12"]), ("maxStepIn 2", ["--fz", "maxStepIn=2"]),
                       ("sigmaMaxIn 12", ["--fz", "sigmaMaxIn=12"]), ("headingGateDeg 16", ["--fz", "headingGateDeg=16"]),
                       ("kDist 0.04", ["--fz", "kDist=0.04"]), ("gate 3", ["--fz", "gate=3"]),
                       ("minPeriodMs 50", ["--fz", "minPeriodMs=50"])):
        on = variant(tag, extra)
        f = median([r["fused"]["end"] for r in on])
        o = median([OFF[("a3a", "ccw", s)][0]["odom"]["end"] for s in seeds])
        out[tag] = {"fused": f, "odom": o, "faults": faults_text(on)}
        texts.append(f"{tag}: {100 * f / o:.0f} %")
    row("A3a?", "  investigation: ccw ratio with one filter number changed (design value: 67 % above)",
        "; ".join(texts[:2]), "60 %", None)
    row("A3a?", "  ... the other numbers tried", "; ".join(texts[2:]), "60 %", None)

    # 4. what a faster feed-in costs: slewInPerS 4 on A3a clockwise, A5's command change, A9's 6 in shove
    cw4 = variant("slew4 cw", ["--fz", "slewInPerS=4"], route="cw", cols=COLS)
    ccw4 = variant("slew4 ccw", ["--fz", "slewInPerS=4"], route="ccw", cols=COLS)
    f_cw = median([r[0]["fused"]["end"] for r in cw4])
    on_steps = np.concatenate([cmd_steps(r[1]) for r in cw4 + ccw4])
    off_steps = np.concatenate([cmd_steps(OFF[("a3a", route, s)][1]) for route in ("cw", "ccw") for s in seeds])
    p_on, p_off = float(np.percentile(on_steps, 99)), float(np.percentile(off_steps, 99))
    six = shove_runs(6, seeds[:20], jobs, cols=None, test="slew4 A9")
    rec = [six[s][0]["shoves"][0]["recover_ms"] for s in seeds[:20]]
    big_step = max(r[0]["filter"]["max_step_in"] for r in cw4 + ccw4)
    out["slewInPerS 4 costs"] = {"a3a_cw_fused": f_cw, "p99_on": p_on, "p99_off": p_off, "a9_recover_ms": rec,
                                 "largest_step_in": big_step}
    row("A3a?", "  ... slewInPerS 4 elsewhere: A3a cw end; A5 command p99 vs Off; A9 6 in back in 5 s; step per tick",
        f"{f_cw:.2f} in ({100 * f_cw / RESULTS['A3']['A3a cw']['median_odom_end_in']:.0f} %); "
        f"{p_on:.0f} vs {p_off:.0f} ({p_on - p_off:+.0f}); {sum(0 <= r <= 5000 for r in rec)}/20; {big_step:.3f} in",
        "", None)
    RESULTS["A3a_investigation"] = out


def test_a4(seeds, jobs):
    fill_off("perfect", "cw", seeds[:1], jobs)
    runs = parallel(lambda s: bench(on_args(s, "perfect") + ["--fusion", "on"], test="A4"), seeds, jobs)
    ends = [r["fused"]["end"] for r in runs]
    RESULTS["A4"] = {"median_fused_end_in": median(ends), "max_fused_end_in": max(ends),
                     "odom_alone_end_in": OFF[("perfect", "cw", seeds[0])][0]["odom"]["end"],
                     "median_fused_rms_in": median([r["fused"]["rms"] for r in runs]),
                     "faults": faults_text(runs)}
    row("A4", f"perfect odometry, GPS 1.5 in, {len(seeds)} seeds: median fused end error (faults: "
              f"{RESULTS['A4']['faults']})", f"{median(ends):.2f} in (odometry alone "
              f"{RESULTS['A4']['odom_alone_end_in']:.2f})", "<= 1.0 in", median(ends) <= 1.0)


def test_old_noise(seeds, jobs):
    """One comparison row at the model's default noise (the first suite's)."""
    old = noise(1.0)
    res = {}
    for label, errors, route in (("A3a cw", "a3a", "cw"), ("A3a ccw", "a3a", "ccw"), ("A3b", "a3b", "cw"),
                                 ("A4", "perfect", "cw")):
        on = parallel(lambda s: bench(on_args(s, errors, noise_args=old) + ["--fusion", "on"], route=route,
                                      test="N"), seeds, jobs)
        fill_off(errors, route, seeds, jobs)
        res[label] = {"median_fused_end_in": median([r["fused"]["end"] for r in on]),
                      "median_odom_end_in": median([OFF[(errors, route, s)][0]["odom"]["end"] for s in seeds]),
                      "faults": faults_text(on)}
    RESULTS["old_noise"] = res
    text = "; ".join(f"{k} {v['median_fused_end_in']:.2f}" + (f" ({100 * v['median_fused_end_in'] / v['median_odom_end_in']:.0f} %)"
                                                               if k.startswith("A3a") else "") for k, v in res.items())
    row("N", f"comparison at the model's default noise (the first suite's, row M), {len(seeds)} seeds: median fused end",
        text, "(info)", None)


def test_a5(seeds_std, jobs):
    """A5 parts (ii) and (iii) and the INFO rows; part (i) is judged over every On run at the end (finish_a5)."""
    # (iii) tick-to-tick command change during moves, On against Off, pooled over A3a's runs (both spins)
    runs = RUNS["a3"]
    on_steps, off_steps, per_run = [], [], []
    compare = []
    for label in ("A3a cw", "A3a ccw"):
        for s, ((off, offc), (on, onc)) in runs[label].items():
            a, b = cmd_steps(onc), cmd_steps(offc)
            on_steps.append(a)
            off_steps.append(b)
            per_run.append(float(np.percentile(a, 99) - np.percentile(b, 99)))
            compare.append(cmd_compare(onc, offc))
    on_all, off_all = np.concatenate(on_steps), np.concatenate(off_steps)
    p_on, p_off = float(np.percentile(on_all, 99)), float(np.percentile(off_all, 99))
    RESULTS["A5_command"] = {"p99_on": p_on, "p99_off": p_off, "ticks_on": int(len(on_all)), "ticks_off": int(len(off_all)),
                             "max_on": int(on_all.max()), "max_off": int(off_all.max()),
                             "worst_per_run_p99_difference": max(per_run),
                             "median_per_run_p99_difference": median(per_run),
                             "start_from_corrected_pose_max_dcmd": max(compare),
                             "start_from_corrected_pose_median_dcmd": median(compare)}
    row("A5", f"tick-to-tick motor command change in moves, 99th percentile, On vs Off (A3a, {len(per_run)} runs "
              "pooled)", f"{p_on:.0f} vs {p_off:.0f} of 127 ({p_on - p_off:+.0f})", "<= +5", p_on - p_off <= 5)
    row("A5", "  ... per run: largest difference of the two 99th percentiles; largest single change On / Off",
        f"{max(per_run):+.0f}; {on_all.max()} / {off_all.max()}", "(info)", None)
    row("A5", "  a move starting from a corrected pose steers differently (vs Off while the robots coincide)", f"max {max(compare):.0f}, median {median(compare):.0f} of 127", "(reported, not limited)", None)

    # (ii) the standard moves, judged on the real robot
    def one(job):
        move, seed = job
        res = []
        for mode in ("off", "on"):
            s = bench(on_args(seed) + ["--fusion", mode], route=move, test="A5 std")
            res.append([x for x in s["step_list"] if x["kind"] != "wait"][-1])
        return move, seed, res[0], res[1]
    results = parallel(one, [(m, s) for m in STD_MOVES for s in seeds_std], jobs)
    per_move = {}
    for move in STD_MOVES:
        mine = [r for r in results if r[0] == move]
        per_move[move] = {
            "max_d_swing_true_in": max(abs(r[3]["swing_true"] - r[2]["swing_true"]) for r in mine),
            "max_d_settle_true_ms": max(abs(r[3]["settle_true_ms"] - r[2]["settle_true_ms"]) for r in mine),
            "max_d_overshoot_in": max(abs(r[3]["overshoot"] - r[2]["overshoot"]) for r in mine),
            "max_d_settle_ms": max((abs(r[3]["settle_ms"] - r[2]["settle_ms"]) if r[2]["settle_ms"] >= 0 and
                                    r[3]["settle_ms"] >= 0 else float("inf")) for r in mine),
            "runs_never_settled_believed": sum(r[3]["settle_ms"] < 0 or r[2]["settle_ms"] < 0 for r in mine),
            "max_d_true_final_in": max(abs(r[3]["true_final"] - r[2]["true_final"]) for r in mine),
            "off": [{k: r[2][k] for k in ("swing_true", "settle_true_ms", "overshoot", "settle_ms", "true_final")}
                    for r in mine],
            "on": [{k: r[3][k] for k in ("swing_true", "settle_true_ms", "overshoot", "settle_ms", "true_final")}
                   for r in mine]}
    RESULTS["A5_moves"] = per_move
    sw = max(v["max_d_swing_true_in"] for v in per_move.values())
    st = max(v["max_d_settle_true_ms"] for v in per_move.values())
    worst_sw = max(per_move, key=lambda m: per_move[m]["max_d_swing_true_in"])
    worst_st = max(per_move, key=lambda m: per_move[m]["max_d_settle_true_ms"])
    row("A5", f"standard moves ({len(STD_MOVES)} x {len(seeds_std)} seeds, A3a): real robot's overshoot past its "
              "resting point vs Off", f"max change {sw:.2f} in ({worst_sw})", "<= 0.1 in",
        sw <= 0.1)
    row("A5", "  ... real robot's settle time (until it stays within 1 in / 1 deg of its resting point) vs Off",
        f"max change {st:.0f} ms ({worst_st})", "<= 100 ms", st <= 100)
    os_b = max(v["max_d_overshoot_in"] for v in per_move.values())
    se_b = max(v["max_d_settle_ms"] for v in per_move.values())
    never = sum(v["runs_never_settled_believed"] for v in per_move.values())
    fin = max(v["max_d_true_final_in"] for v in per_move.values())
    row("A5", "  ... the PID tab's measures on the believed pose (overshoot / settle), and where the robot ends",
        f"{os_b:.2f} in / " + ("never settled in %d runs" % never if se_b == float("inf") else f"{se_b:.0f} ms") +
        f"; true end moved up to {fin:.2f} in", "(reported, not limited)", None)


def finish_a5():
    """A5 part (i): the largest pose change in one tick, over every fusion-On run of the suite."""
    limit = CFG["slewInPerS"] * ODOM_TICK_S
    # runs with a changed slewInPerS (investigations) are left out: the limit is the configured rate
    judged = [r for r in ON_RUNS if not r[0].startswith("slew")]
    # ties broken by test name and seed, so the run named does not depend on which thread finished first
    worst = sorted(judged, key=lambda r: (-r[2], r[0], r[1]))[0]
    at_limit = sum(r[2] >= limit * 0.999 for r in judged)
    RESULTS["A5_step"] = {"on_runs": len(judged), "largest_in": worst[2], "test": worst[0], "seed": worst[1],
                          "runs_at_the_limit": at_limit, "limit_in": limit}
    # a float rounding allowance of 0.1 %: the bench takes the step as the difference of two float totals
    row("A5", f"largest pose change in one tick from a correction, all {len(judged)} On runs (other slewInPerS left "
              "out)",
        f"{worst[2]:.5f} in ({at_limit} runs reach it)", f"<= slewInPerS*dt = {limit:.3f} in",
        worst[2] <= limit * 1.001)


def relocalisation_book(c):
    """Every relocalisation in one run, from its CSV: how far it moved the decided correction (est - odom, the
    correction fed in plus the one still pending) in one tick, and how many of the relocSamples gate failures it
    was built on (the last ones, up to and including its own tick) came while the sensor was blind."""
    found = []
    reloc = c["reloc"]
    decided_x = c["est_x"] - c["odom_x"]
    decided_y = c["est_y"] - c["odom_y"]
    gate_ticks = np.nonzero(c["fate"] == 8)[0]
    for k in np.nonzero(np.diff(reloc) > 0)[0] + 1:
        jump = float(math.hypot(decided_x[k] - decided_x[k - 1], decided_y[k] - decided_y[k - 1]))
        used = [i for i in gate_ticks if i <= k][-CFG["relocSamples"]:]
        blind = int(sum(c["gps_state"][i] != 0 for i in used))
        found.append({"t_ms": int(c["t_ms"][k]), "jump_in": jump, "blind_readings": blind, "readings": len(used)})
    return found


def test_a6(seeds, jobs):
    a6cols = ["t_ms", "fate", "reloc", "gps_state", "est_x", "est_y", "odom_x", "odom_y"]
    both = parallel(lambda s: bench(on_args(s, extra=a6_extras()) + ["--fusion", "on"], cols=a6cols, test="A6"),
                    seeds, jobs)
    runs = [b[0] for b in both]
    books = [relocalisation_book(b[1]) for b in both]
    a3 = RESULTS["A3"]["A3a cw"]
    fused6 = [r["fused"]["end"] for r in runs]
    med6, med3 = median(fused6), a3["median_fused_end_in"]
    step = max(r["filter"]["max_reading_step_in"] for r in runs)
    acc_out = sum(r["filter"]["accepted_outliers"] for r in runs)
    biggest = max(r["filter"]["max_accepted_outlier_in"] for r in runs)
    outliers = sum(r["gps"]["outliers"] for r in runs)
    blind = sum(r["filter"]["accepted_while_blind"] for r in runs)
    blind_err = max(r["filter"]["max_accepted_blind_err_in"] for r in runs)
    blind_out = sum(r["gps"]["dead_zone_outputs"] + r["gps"]["lost_outputs"] for r in runs)
    accepted = sum(r["filter"]["accepted"] for r in runs)
    RESULTS["A6"] = {"median_fused_end_in": med6, "a3a_cw_median_fused_end_in": med3, "max_reading_step_in": step,
                     "accepted": accepted, "accepted_outliers": acc_out, "outliers_emitted": outliers,
                     "largest_accepted_outlier_in": biggest, "accepted_while_blind": blind,
                     "blind_outputs": blind_out, "largest_accepted_blind_err_in": blind_err,
                     "faults": faults_text(runs),
                     "median_paired_difference_in": median([a - RESULTS["A3"]["A3a cw"]["per_seed"][s]["fused_end"]
                                                            for a, s in zip(fused6, seeds)])}
    row("A6", f"outliers 1 in 10 (6-24 in), dead zones, 4 x 2 s losses, {len(seeds)} seeds: largest step one "
              "reading made", f"{step:.4f} in", f"<= maxStepIn {CFG['maxStepIn']:.1f} in",
        step <= CFG["maxStepIn"] + 1e-5)
    row("A6", "  ... median end error vs A3a (clockwise)", f"{med6:.2f} vs {med3:.2f} in ({med6 - med3:+.2f})",
        "within 0.5 in", abs(med6 - med3) <= 0.5)
    relocs = [r for b in books for r in b]
    on_blind = sum(r["blind_readings"] > 0 for r in relocs)
    all_blind = sum(r["blind_readings"] == r["readings"] for r in relocs)
    biggest_jump = max((r["jump_in"] for r in relocs), default=0.0)
    RESULTS["A6"]["relocalisations"] = {"count": len(relocs), "largest_jump_in": biggest_jump,
                                        "built_on_any_blind": on_blind, "built_on_all_blind": all_blind,
                                        "per_seed": {s: b for s, b in zip(seeds, books)}}
    row("A6", "  ... relocalisations; the largest jump one made; built on blind readings (at least one of its "
              f"{CFG['relocSamples']} / all of them)", f"{len(relocs)} in {len(seeds)} runs; {biggest_jump:.2f} in; "
                                                      f"{on_blind} / {all_blind}", "report", None)
    row("A6", "  ... accepted: outliers (of those injected) / blind readings (of blind outputs); largest error",
        f"{acc_out} of {outliers}, largest {biggest:.1f} in / {blind} of {blind_out}, largest {blind_err:.1f} in "
        f"({accepted} accepted in all; faults: {RESULTS['A6']['faults']})", "report", None)
    # INFO (revision 3): the model's dead zone (18 in by default) at the filter's minViewIn and past it, on the
    # calibrated robot with a healthy sensor: readings 20 to 24 in from the wall are blind but pass the filter's
    # dead zone check
    fill_off("a3b", "cw", seeds, jobs)
    dz = {}
    for zone in DEAD_ZONES_IN:
        on = parallel(lambda s: bench(on_args(s, "a3b", extra=["--gps", f"dead_zone_in={zone}"]) + ["--fusion", "on"],
                                      test="A6 dz"), seeds, jobs)
        v = {"median_fused_end_in": median([r["fused"]["end"] for r in on]),
             "median_odom_end_in": median([OFF[("a3b", "cw", s)][0]["odom"]["end"] for s in seeds]),
             "accepted_while_blind": sum(r["filter"]["accepted_while_blind"] for r in on),
             "largest_accepted_blind_err_in": max(r["filter"]["max_accepted_blind_err_in"] for r in on),
             "faults": faults_text(on)}
        dz[zone] = v
        row("A6", f"  ... (info) model dead zone {zone} in (filter minViewIn 20), calibrated robot, healthy sensor, "
                  f"{len(seeds)} seeds", f"end {v['median_fused_end_in']:.2f} vs {v['median_odom_end_in']:.2f} in; "
                                         f"blind accepted {v['accepted_while_blind']}, largest "
                                         f"{v['largest_accepted_blind_err_in']:.1f} in; faults {v['faults']}",
            "(info)", None)
    RESULTS["A6"]["dead_zone_info"] = dz


def test_a7(seeds, jobs):
    def start_off(seed):
        dx, dy = per_seed_direction(seed, 20_000, 10.0)
        return ["--start-error", f"{dx:.4f}:{dy:.4f}"]
    cases = [("start pose 10 in off (direction per seed)", start_off),
             ("frame mirrored left-right", lambda s: ["--gps", "mirror=2"]),
             ("frame mirrored front-back", lambda s: ["--gps", "mirror=3"]),
             ("field turned 180 deg", lambda s: ["--gps", "mirror=1"]),
             ("mount yaw wrong by 180: camera faces forward, filter told back", lambda s: ["--gps", "mount_yaw_deg=0"]),
             ("mount yaw wrong by 180: camera faces back, filter told forward", lambda s: ["--fz", "mountYawDeg=0"])]

    def one(job):
        i, seed = job
        return i, seed, bench(on_args(seed, extra=cases[i][1](seed)) + ["--fusion", "on"], test="A7")
    results = parallel(one, [(i, s) for i in range(len(cases)) for s in seeds], jobs)
    out = {}
    no_fault, worst_before = 0, 0.0
    for i, (name, _) in enumerate(cases):
        mine = [r[2] for r in results if r[0] == i]
        nf = sum(r["filter"]["fault"] == 0 for r in mine)
        before = max(r["filter"]["max_fed_before_fault_in"] for r in mine)
        times = [r["filter"]["fault_ms"] for r in mine if r["filter"]["fault_ms"] >= 0]
        out[name] = {"runs": len(mine), "no_fault": nf, "faults": faults_text(mine), "fault_ms": times,
                     "max_fed_before_fault_in": before,
                     "back_to_odometry_at_end": sum(r["filter"]["fed_end_in"] == 0 for r in mine)}
        no_fault += nf
        worst_before = max(worst_before, before)
    RESULTS["A7"] = out
    n = len(results)
    row("A7", f"start 10 in off; mirrored 2 ways; turned 180; mount yaw off by 180 (2 ways); {len(seeds)} seeds "
              "each: fault", f"{n - no_fault}/{n} fault", "every run", no_fault == 0)
    row("A7", "  ... before the fault, the pose's largest distance from odometry alone (corrections fed in)",
        f"{worst_before:.3f} in", "<= 1.0 in", worst_before <= 1.0)
    for name, v in out.items():
        t = v["fault_ms"]
        row("A7", f"  ... {name}", f"{v['faults']}" + (f", {min(t)}-{max(t)} ms" if t else "") +
            (f"; {v['no_fault']} never" if v["no_fault"] else ""), "(info)", None)
    test_a7_starts(seeds, jobs)
    test_turn_first(seeds, jobs)


def start_route(x, y, heading):
    """A copy of the route that starts at (x, y, heading) instead (the moves after it are the same points)."""
    name = f"start_{x}_{y}_{heading}"
    if name not in ROUTES:
        with open(ROUTE) as f:
            lines = f.read().splitlines()
        lines = [f"start {x} {y} {heading}" if ln.startswith("start ") else ln for ln in lines]
        ROUTES[name] = tmp(name + ".txt")
        with open(ROUTES[name], "w") as g:
            g.write("\n".join(lines) + "\n")
    return name


def on_mirror_line(mirror, x, y, heading):
    """Does the mirrored frame read this start pose as itself? (see A7_STARTS)"""
    if mirror == 2:
        return abs(x) < 1 and heading % 180 == 0
    if mirror == 3:
        return abs(y) < 1 and heading % 180 == 90
    return False


def test_a7_starts(seeds, jobs):
    """Revision 3: the three mirrored frames from more start poses. Off the mirror line the old limit holds (every
    run faults, under 1.0 in fed in before it); on the line the start check cannot tell, so what is fed in before
    the fault and when it comes are reported."""
    cases = [(m, label, x, y, h) for m in A7_MIRRORS for label, x, y in A7_STARTS for h in A7_HEADINGS
             if not (x == -60 and y == -36 and h == 0)]  # the route's own start at 0 deg is the case above

    names = [start_route(x, y, h) for m, label, x, y, h in cases]  # written before the parallel runs read them

    def one(job):
        i, seed = job
        return i, seed, bench(on_args(seed, extra=["--gps", f"mirror={cases[i][0]}"]) + ["--fusion", "on"],
                              route=names[i], test="A7 starts")
    results = parallel(one, [(i, s) for i in range(len(cases)) for s in seeds], jobs)
    off_runs = [r for r in results if not on_mirror_line(*[cases[r[0]][k] for k in (0, 2, 3, 4)])]
    nf = sum(r[2]["filter"]["fault"] == 0 for r in off_runs)
    before = max(r[2]["filter"]["max_fed_before_fault_in"] for r in off_runs)
    out = {"off_line": {"runs": len(off_runs), "no_fault": nf, "max_fed_before_fault_in": before,
                        "faults": faults_text([r[2] for r in off_runs])}, "on_line": {}}
    row("A7", f"  mirrored frames from start poses off the mirror line ({len(cases)} start/mirror cases, on-line ones "
              f"excluded; {len(seeds)} seeds each): fault", f"{len(off_runs) - nf}/{len(off_runs)} fault "
                                                         f"({out['off_line']['faults']})", "every run", nf == 0)
    row("A7", "  ... before the fault, the pose's largest distance from odometry alone", f"{before:.3f} in",
        "<= 1.0 in", before <= 1.0)
    for i, (m, label, x, y, h) in enumerate(cases):
        if not on_mirror_line(m, x, y, h):
            continue
        mine = [r[2] for r in results if r[0] == i]
        t = [r["filter"]["fault_ms"] for r in mine if r["filter"]["fault_ms"] >= 0]
        fed = [r["filter"]["max_fed_before_fault_in"] for r in mine]
        v = {"runs": len(mine), "faults": faults_text(mine), "fault_ms": t, "max_fed_before_fault_in": max(fed),
             "median_fed_before_fault_in": median(fed)}
        out["on_line"][f"{A7_MIRRORS[m]}, {label}, heading {h}"] = v
        row("A7", f"  ... on the line: {A7_MIRRORS[m]}, start {label} facing {h}",
            f"fed in before the fault median {v['median_fed_before_fault_in']:.2f}, worst {max(fed):.2f} in; faults "
            f"{v['faults']}" + (f" at {min(t) / 1000:.1f}-{max(t) / 1000:.1f} s" if t else "") +
            (f"; {len(mine) - len(t)} never" if len(t) < len(mine) else ""), "report", None)
    RESULTS["A7_starts"] = out


def test_turn_first(seeds, jobs):
    """Revision 3 item 4: the heading fault judges only readings taken while the robot is slow, so a routine that
    opens with a fast 180 deg turn (the first reviewer's route) does not fault with a healthy sensor."""
    out = {}
    for errors in ("perfect", "a3b"):
        on = parallel(lambda s: bench(on_args(s, errors) + ["--fusion", "on"], route=TURN_FIRST_ROUTE,
                                      test="A7 turn first"), seeds, jobs)
        out[errors] = {"faults": faults_text(on), "fault_runs": sum(r["filter"]["fault"] != 0 for r in on)}
    RESULTS["A7_turn_first"] = out
    n = out["perfect"]["fault_runs"] + out["a3b"]["fault_runs"]
    row("A7", f"  turn-first routine (routes/{TURN_FIRST_ROUTE}: 180 deg turn at once), healthy sensor, {len(seeds)} "
              "seeds each on perfect odometry and the calibrated robot: no fault",
        f"{n} of {2 * len(seeds)} fault (perfect: {out['perfect']['faults']}; calibrated: {out['a3b']['faults']})",
        "none", n == 0)


def test_a8(seeds, jobs):
    def one(job):
        delay, seed = job
        # revision 3: on the calibrated robot (A3b's errors, route as written), the case the filter is meant for
        return delay, bench(on_args(seed, "a3b", extra=["--gps", f"latency_ms={delay}"]) + ["--fusion", "on"],
                            test="A8")
    results = parallel(one, [(d, s) for d in DELAYS_MS for s in seeds], jobs)
    fill_off("a3b", "cw", seeds, jobs)
    odom_rms = median([OFF[("a3b", "cw", s)][0]["odom"]["rms"] for s in seeds])
    per_delay = {}
    for delay in DELAYS_MS:
        mine = [r[1] for r in results if r[0] == delay]
        per_delay[delay] = {"median_fused_end_in": median([r["fused"]["end"] for r in mine]),
                            "median_fused_rms_in": median([r["fused"]["rms"] for r in mine]),
                            "worst_fused_max_in": max(r["fused"]["max"] for r in mine),
                            "faults": sum(r["filter"]["fault"] != 0 for r in mine), "fault_text": faults_text(mine),
                            "relocalisations": sum(r["filter"]["relocalisations"] for r in mine),
                            "accepted": sum(r["filter"]["accepted"] for r in mine),
                            "dropped_by_gate": sum(r["filter"]["dropped_by_gate"] for r in mine)}
    # "stable" (the first suite's definition, unchanged): no fault at any delay, and the median rms error over the
    # run no more than 0.5 in worse than with the true delay equal to the configured 60 ms
    rms60 = per_delay[60]["median_fused_rms_in"]
    stable = all(per_delay[d]["faults"] == 0 and per_delay[d]["median_fused_rms_in"] - rms60 <= 0.5 for d in DELAYS_MS)
    base = per_delay[60]["median_fused_end_in"]
    RESULTS["A8"] = {"per_delay_ms": per_delay, "median_odom_rms_in": odom_rms}
    row("A8", f"true delay 40-150 ms, told 60 ({len(seeds)} seeds each, A3b's calibrated errors): median fused end error by "
               "delay (ms: in)",
        ", ".join(f"{d}: {per_delay[d]['median_fused_end_in']:.2f}" for d in DELAYS_MS), "report", None)
    worst_loss = max(per_delay[d]["median_fused_end_in"] - base for d in DELAYS_MS)
    worst_rms = max(per_delay[d]["median_fused_rms_in"] - rms60 for d in DELAYS_MS)
    faults = sum(per_delay[d]["faults"] for d in DELAYS_MS)
    row("A8", "  ... stable: no fault, and median rms over the run within 0.5 in of the 60 ms case, at every delay",
        f"faults {faults}" + (" (" + "; ".join(f"{d} ms: {per_delay[d]['fault_text']}" for d in DELAYS_MS
                                               if per_delay[d]["faults"]) + ")" if faults else "") +
        f"; rms {worst_rms:+.2f} in, end {worst_loss:+.2f} in at worst (odometry alone rms {odom_rms:.2f})",
        "stable", stable)


def shove_runs(size, seeds, jobs, errors="perfect", cols=None, test="A9"):
    def one(seed):
        r = bench(on_args(seed, errors, extra=["--shove", f"s{SHOVE_STEP}:{size}:0:200"]) + ["--fusion", "on"],
                  cols=cols, test=test)
        return r if cols else (r, None)
    return dict(zip(seeds, parallel(one, seeds, jobs)))


def test_a9(seeds, jobs):
    """Revision 3: a shove of about two thirds of relocMaxIn recovers; one of relocMaxIn + 3 in faults and the pose
    returns to odometry alone. The 2 to 12 in sweep is reported."""
    cols = ["t_ms", "err_odom", "err_fused"]
    small, big = A9_RECOVER_IN, A9_FAULT_IN
    rec_runs = shove_runs(small, seeds, jobs, cols=cols)
    rec = [rec_runs[s][0]["shoves"][0]["recover_ms"] for s in seeds]
    ok = sum(0 <= r <= 5000 for r in rec)
    finite = [r for r in rec if r >= 0]
    later = sum(0 <= r <= 5000 and rec_runs[s][0]["filter"]["fault"] != 0 for s, r in zip(seeds, rec))
    fault_runs = shove_runs(big, seeds, jobs, cols=cols)
    fb = [fault_runs[s][0]["filter"] for s in seeds]
    faulted = sum(f["fault"] != 0 for f in fb)
    unwound = sum(f["fault"] != 0 and f["fed_end_in"] == 0 for f in fb)
    back = [f["fault_to_fed_under_half_ms"] for f in fb if f["fault"] != 0]
    RESULTS["A9"] = {"recover_size_in": small, "fault_size_in": big,
                     "recover_ms": dict(zip(seeds, rec)),
                     "relocalisations": {s: rec_runs[s][0]["filter"]["relocalisations"] for s in seeds},
                     "faults_recover_size": faults_text([rec_runs[s][0] for s in seeds]),
                     "recovered_then_faulted": later,
                     "fault_size": {s: {"fault": f["fault_name"], "fault_ms": f["fault_ms"], "fed_end_in": f["fed_end_in"],
                                        "fault_to_fed_under_half_ms": f["fault_to_fed_under_half_ms"],
                                        "fault_to_unwound_ms": f["fault_to_unwound_ms"]} for s, f in zip(seeds, fb)}}
    row("A9", f"{small:g} in sideways shove (2/3 of relocMaxIn {CFG['relocMaxIn']:g}) at a stop, perfect odometry, "
              f"{len(seeds)} seeds: back within 2 in within 5 s",
        f"{ok}/{len(seeds)} (worst {max(rec)} ms, median {median(finite):.0f} ms; faults "
        f"{RESULTS['A9']['faults_recover_size']}; {later} of the recovered runs fault later)",
        f">= {len(seeds) - 1} of {len(seeds)}", ok >= len(seeds) - 1)
    row("A9", f"{big:g} in shove (relocMaxIn + 3), {len(seeds)} seeds: fault, and the pose returns to odometry alone",
        f"{faulted}/{len(seeds)} fault ({faults_text([fault_runs[s][0] for s in seeds])}), {unwound} back to exactly "
        f"odometry; fed under 0.5 in {min(back) if back else -1}-{max(back) if back else -1} ms after the fault",
        "fault, back to odometry", faulted == len(seeds) and unwound == len(seeds))
    # INFO: the recovering shove with A3a's odometry errors, and the sweep
    a3 = shove_runs(small, seeds, jobs, errors="a3a", test="A9 info")
    rec3 = [a3[s][0]["shoves"][0]["recover_ms"] for s in seeds]
    RESULTS["A9_info"] = {f"recover_ms_{small:g}in_a3a": rec3}
    row("A9", f"  ... {small:g} in shove with A3a's odometry errors (not in the design)",
        f"{sum(0 <= r <= 5000 for r in rec3)}/{len(seeds)} back within 5 s; faults {faults_text([a3[s][0] for s in seeds])}",
        "(info)", None)
    for size in A9_SWEEP_IN:
        r = rec_runs if size == small else shove_runs(size, seeds, jobs, test="A9 info")
        recs = [r[s][0]["shoves"][0]["recover_ms"] for s in seeds]
        RESULTS["A9_info"][f"recover_ms_{size}in"] = recs
        RESULTS["A9_info"][f"faults_{size}in"] = faults_text([r[s][0] for s in seeds])
        row("A9", f"  ... sweep: {size} in shove, perfect odometry",
            f"{sum(0 <= x <= 5000 for x in recs)}/{len(seeds)} back within 2 in in 5 s, "
            f"{sum(x >= 0 for x in recs)} ever; faults {faults_text([r[s][0] for s in seeds])}", "(info)", None)
    RUNS["a9"] = {"recover": rec_runs, "fault": fault_runs}


def test_a10(seeds, jobs):
    cases = [("20 s drifting view loss (1.5 in/s) from 15 s", ["--gps-loss", BLIND_LOSS]),
             ("strips on the 0 deg wall (+y) only", ["--gps", "strip_walls=1"]),
             ("strips on the 90 deg wall (+x) only", ["--gps", "strip_walls=2"]),
             ("strips on the 180 deg wall (-y) only", ["--gps", "strip_walls=4"]),
             ("strips on the 270 deg wall (-x) only", ["--gps", "strip_walls=8"]),
             ("never saw a strip, starts at the true position, drifts 1.5 in/s",
              ["--gps", "no_strips=2", "--gps", "no_strip_at_truth=1"])]
    limit = CFG["maxTotalCorrectionIn"] + 1.0
    fill_off("a3a", "cw", seeds, jobs)

    def one(job):
        i, seed = job
        on, c = bench(on_args(seed, extra=cases[i][1]) + ["--fusion", "on"], cols=["t_ms", "err_fused"], test="A10")
        off, oc = OFF[("a3a", "cw", seed)]
        t, f, o = aligned(c["t_ms"], c["err_fused"], oc["t_ms"], oc["err_odom"])
        return i, seed, on, t, f - o
    results = parallel(one, [(i, s) for i in range(len(cases)) for s in seeds], jobs)
    out = {}
    worst_all, bad_fault, late, limit_runs, faulted_runs, texts = 0.0, 0, 0, 0, 0, []
    for i, (name, _) in enumerate(cases):
        mine = [r for r in results if r[0] == i]
        excess = [float(np.max(r[4])) for r in mine]
        fl = [r[2]["filter"] for r in mine]
        hit = [f for f in fl if f["decided_over_limit_ms"] >= 0]
        # the limit was hit: a fault must be raised on that very tick (and it must be the total limit, unless an
        # earlier fault had already stopped the filter)
        wrong = [f for f in hit if not (f["fault"] != 0 and f["fault_ms"] <= f["decided_over_limit_ms"])]
        back = [f["fault_to_fed_under_half_ms"] for f in fl if f["fault"] != 0]
        slow = [b for b in back if not 0 <= b <= 7000]
        out[name] = {"worst_excess_in": max(excess), "median_excess_in": median(excess), "limit_hit_runs": len(hit),
                     "fault_runs": sum(f["fault"] != 0 for f in fl), "faults": faults_text([r[2] for r in mine]),
                     "fault_ms": [f["fault_ms"] for f in fl], "fault_to_fed_under_half_ms": back,
                     "median_fused_end_in": median([r[2]["fused"]["end"] for r in mine]),
                     "median_odom_end_in": median([OFF[("a3a", "cw", r[1])][0]["odom"]["end"] for r in mine]),
                     "max_fed_in": max(f["max_fed_in"] for f in fl),
                     "accepted_while_blind": sum(f["accepted_while_blind"] for f in fl),
                     "per_seed_excess_in": dict(zip(seeds, excess))}
        worst_all = max(worst_all, max(excess))
        bad_fault += len(wrong)
        late += len(slow)
        limit_runs += len(hit)
        faulted_runs += out[name]["fault_runs"]
        texts.append((name, out[name]))
    RESULTS["A10"] = out
    n = len(results)
    row("A10", f"blind sensor, 6 cases x {len(seeds)} seeds (below), A3a: fused error minus odometry alone's (Off run), "
               "largest", f"{worst_all:.2f} in",
        f"<= maxTotal + 1 = {limit:.0f} in", worst_all <= limit)
    row("A10", "  ... a fault is raised when the limit is hit (decided total past maxTotalCorrectionIn: that tick)",
        f"{limit_runs - bad_fault}/{limit_runs} runs that hit it faulted at once; {faulted_runs}/{n} runs faulted",
        "every time", bad_fault == 0)
    row("A10", "  ... within 7 s of the fault, the pose within 0.5 in of odometry alone (fed in under 0.5 in)",
        f"{faulted_runs - late}/{faulted_runs} faulted runs; slowest "
        f"{max(max(v['fault_to_fed_under_half_ms']) for _, v in texts if v['fault_to_fed_under_half_ms']) if faulted_runs else 0} ms",
        "<= 7000 ms", late == 0)
    for name, v in texts:
        ft = [t for t in v["fault_ms"] if t >= 0]
        row("A10", f"  ... {name}",
            f"worst {v['worst_excess_in']:+.1f} in; faults {v['faults']}" +
            (f" at {min(ft) / 1000:.1f}-{max(ft) / 1000:.1f} s" if ft else "") +
            f"; end {v['median_fused_end_in']:.1f} vs {v['median_odom_end_in']:.1f}", "(info)", None)
    RUNS["a10_blind"] = [(r[1], r[2], r[3], r[4]) for r in results if r[0] == 0]
    test_a10_slow(seeds, jobs)


def test_a10_slow(seeds, jobs):
    """Revision 3: slow liars on the calibrated robot, reported. Position readings alone cannot tell these from
    odometry drift, so the rows show how far the limits let them get."""
    def bias(seed):
        d = float(np.random.RandomState(40_000 + seed).uniform(0, 360))  # the revision 2 attacker's draw
        return ["--gps", f"bias_step_in={BIAS_STEP['in']}", "--gps", f"bias_step_ms={BIAS_STEP['ms']}",
                "--gps", f"bias_step_start_ms={BIAS_STEP['start_ms']}", "--gps", f"bias_step_count={BIAS_STEP['count']}",
                "--gps", f"bias_step_dir_deg={d:.3f}"]
    cases = [(f"blind drift {r} in/s from 15 s to the end", lambda s, r=r: ["--gps-loss", SLOW_LOSS.format(rate=r)])
             for r in SLOW_DRIFTS_IN_S]
    cases += [(f"blind drift {STILL_DRIFT_IN_S} in/s only while standing still, from 15 s",
               lambda s: ["--gps", "drift_mode=2", "--gps-loss", SLOW_LOSS.format(rate=STILL_DRIFT_IN_S)]),
              (f"bias stepping {BIAS_STEP['in']} in every {BIAS_STEP['ms'] / 1000:g} s from "
               f"{BIAS_STEP['start_ms'] / 1000:g} s to " +
               ", ".join(str(BIAS_STEP["in"] * (k + 1)) for k in range(BIAS_STEP["count"])) + " in", bias)]
    fill_off("a3b", "cw", seeds, jobs)

    def one(job):
        i, seed = job
        on, c = bench(on_args(seed, "a3b", extra=cases[i][1](seed)) + ["--fusion", "on"], cols=["t_ms", "err_fused"],
                      test="A10 slow")
        off, oc = OFF[("a3b", "cw", seed)]
        t, f, o = aligned(c["t_ms"], c["err_fused"], oc["t_ms"], oc["err_odom"])
        ex = f - o
        return i, seed, on, float(np.max(ex)), float(ex[-1]), int(np.sum(ex > EXCESS_LONG_IN)) * 10
    results = parallel(one, [(i, s) for i in range(len(cases)) for s in seeds], jobs)
    out = {}
    for i, (name, _) in enumerate(cases):
        mine = [r for r in results if r[0] == i]
        v = {"worst_excess_in": max(r[3] for r in mine), "median_worst_excess_in": median([r[3] for r in mine]),
             "worst_end_excess_in": max(r[4] for r in mine), "median_end_excess_in": median([r[4] for r in mine]),
             "ms_above_4in_worst": max(r[5] for r in mine), "ms_above_4in_median": median([r[5] for r in mine]),
             "fault_runs": sum(r[2]["filter"]["fault"] != 0 for r in mine), "faults": faults_text([r[2] for r in mine]),
             "per_seed": {r[1]: {"worst_excess_in": r[3], "end_excess_in": r[4], "ms_above_4in": r[5],
                                 "fault": r[2]["filter"]["fault_name"]} for r in mine}}
        out[name] = v
        row("A10", f"  ... calibrated robot, {name} ({len(seeds)} seeds): excess over odometry alone",
            f"worst {v['worst_excess_in']:.1f} (median {v['median_worst_excess_in']:.1f}); end worst "
            f"{v['worst_end_excess_in']:+.1f} (median {v['median_end_excess_in']:+.1f}) in; above 4 in worst "
            f"{v['ms_above_4in_worst'] / 1000:.1f} s (median {v['ms_above_4in_median'] / 1000:.1f}); faults {v['faults']}",
            "report", None)
    RESULTS["A10_slow"] = out


def test_a11(seeds, jobs):
    def bias(seed):
        dx, dy = per_seed_direction(seed, 30_000, 2.0)
        return ["--gps", f"bias_x_in={dx:.4f}", "--gps", f"bias_y_in={dy:.4f}"]
    cases = [("constant 2 in bias (direction per seed)", bias, None, ("a3a", "perfect")),
             ("GPS noise 3.6 in, filter told 1.5", lambda s: [], noise(NOISY_SCALE), ("a3a", "perfect")),
             ("camera 6 in right of centre, mount left at 0 (declared measured)", lambda s: ["--gps", "mount_right_in=6"],
              None, ("a3a",)),
             ("camera 6 in behind centre, mount left at 0 (declared measured)", lambda s: ["--gps", "mount_forward_in=-6"],
              None, ("a3a",))]
    out = {}
    for name, extra, noise_args, error_sets in cases:
        for errors in error_sets:
            fill_off(errors, "cw", seeds, jobs)
            on = parallel(lambda s: bench(on_args(s, errors, extra=extra(s), noise_args=noise_args) + ["--fusion", "on"],
                                          test="A11"), seeds, jobs)
            fused = [r["fused"]["end"] for r in on]
            odom = [OFF[(errors, "cw", s)][0]["odom"]["end"] for s in seeds]
            worse = sum(f - o > 1.0 for f, o in zip(fused, odom))
            v = {"median_fused_end_in": median(fused), "median_odom_end_in": median(odom), "worse_by_1in_at_end": worse,
                 "median_fused_max_in": median([r["fused"]["max"] for r in on]),
                 "median_odom_max_in": median([OFF[(errors, "cw", s)][0]["odom"]["max"] for s in seeds]),
                 "faults": faults_text(on), "median_nis": median([r["filter"]["nis_avg"] for r in on]),
                 "median_inn_right_in": median([r["filter"]["innovation_right_avg_in"] for r in on]),
                 "median_inn_forward_in": median([r["filter"]["innovation_forward_avg_in"] for r in on])}
            out[f"{name}, {errors}"] = v
            diag = f"; nis {v['median_nis']:.1f}" if "noise" in name else (
                f"; inn right/fwd {v['median_inn_right_in']:+.1f}/{v['median_inn_forward_in']:+.1f}" if "camera" in name else "")
            row("A11", f"{name}, {'A3a errors' if errors == 'a3a' else 'perfect odometry'}, {len(seeds)} seeds",
                f"end {v['median_fused_end_in']:.2f} vs {v['median_odom_end_in']:.2f} in; >1 in worse {worse}/{len(seeds)}; "
                f"faults {v['faults']}{diag}", "report", None)
    # for comparison: the healthy sensor's diagnostics (A3a clockwise, the same seeds)
    healthy = [RUNS["a3"]["A3a cw"][s][1][0]["filter"] for s in seeds]
    out["healthy (A3a cw)"] = {"median_nis": median([f["nis_avg"] for f in healthy]),
                               "median_inn_right_in": median([f["innovation_right_avg_in"] for f in healthy]),
                               "median_inn_forward_in": median([f["innovation_forward_avg_in"] for f in healthy])}
    h = out["healthy (A3a cw)"]
    row("A11", "  ... for comparison, the healthy sensor (A3a clockwise): nis, inn right/fwd",
        f"{h['median_nis']:.1f}, {h['median_inn_right_in']:+.1f}/{h['median_inn_forward_in']:+.1f}", "(info)", None)
    RESULTS["A11"] = out


def test_shadow(seeds, jobs):
    """Not one of the A tests, but the design's stage 2 depends on it: Shadow must leave the pose exactly as Off."""
    def one(seed):
        base = on_args(seed)
        bench(base + ["--fusion", "off"], trace=tmp(f"sh_off_{seed}.trace"), test="S")
        sh = bench(base + ["--fusion", "shadow"], trace=tmp(f"sh_{seed}.trace"), test="S")
        _, same = trace_difference(tmp(f"sh_off_{seed}.trace"), tmp(f"sh_{seed}.trace"))
        return same, sh["fused"]["end"], sh["odom"]["end"], sh["filter"]["fault_name"]
    r = parallel(one, seeds, jobs)
    same = sum(x[0] for x in r)
    RESULTS["Shadow"] = {"identical": same, "runs": len(r), "median_would_be_end_in": median([x[1] for x in r]),
                         "median_odom_end_in": median([x[2] for x in r])}
    row("S", f"Shadow, {len(r)} seeds, A3a errors: pose vs Off (exact pose trace)", f"{same}/{len(r)} identical",
        "identical", same == len(r))
    row("S", "  ... what On would have read (pose + its shadow correction): median end error",
        f"{RESULTS['Shadow']['median_would_be_end_in']:.2f} in (odometry {RESULTS['Shadow']['median_odom_end_in']:.2f})",
        "(info)", None)


RUNS = {}


# ---------------------------------------------------------------------------------------------- figures
# dataviz skill reference palette (light): validated all-pairs for the three slots used (aqua is below 3:1 against
# the surface, so it only marks the GPS dots, which the legend names)
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
SURFACE = "#fcfcfb"
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
SHADE = "#efeee9"


def style_axes(ax):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
        ax.spines[side].set_linewidth(1)
    ax.tick_params(colors=MUTED, labelcolor=INK_2, labelsize=9)
    ax.grid(True, color=GRID, linewidth=1, linestyle="-")
    ax.set_axisbelow(True)


def legend(ax, **kw):
    leg = ax.legend(frameon=False, fontsize=9, **kw)
    for text in leg.get_texts():
        text.set_color(INK_2)
    return leg


def band(series):
    """Median and 10-90 % band of per-seed series, cut to the shortest."""
    n = min(len(s) for s in series)
    m = np.vstack([s[:n] for s in series])
    return np.median(m, axis=0), np.percentile(m, 10, axis=0), np.percentile(m, 90, axis=0), n


def figures(seeds):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd
    plt.rcParams.update({"font.family": ["Helvetica", "Arial", "DejaVu Sans"], "axes.titlesize": 11,
                         "axes.labelsize": 10, "axes.labelcolor": INK_2, "text.color": INK})
    made = []
    a3 = RUNS["a3"]

    def save(fig, name):
        path = os.path.join(OUT, name)
        fig.savefig(path, facecolor=SURFACE)
        plt.close(fig)
        made.append(path)

    # 1. top view of the A3a clockwise seed whose fused end error is the median
    runs = a3["A3a cw"]
    ends = sorted(runs, key=lambda s: runs[s][1][0]["fused"]["end"])
    seed = ends[len(ends) // 2]
    csv = tmp("top.csv")
    bench(on_args(seed) + ["--fusion", "on", "--csv", csv], test="figure")
    on = pd.read_csv(csv)
    fig, ax = plt.subplots(figsize=(7.2, 7.8), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    style_axes(ax)
    ax.plot([-72, 72, 72, -72, -72], [-72, -72, 72, 72, -72], color=AXIS, linewidth=1.5)
    gps = on[(on.gps_new == 1)]
    ax.scatter(gps.gps_x, gps.gps_y, s=4, color=AQUA, alpha=0.35, linewidths=0, label="GPS readings (1.5 in noise)",
               zorder=2)
    ax.plot(on.true_x, on.true_y, color=INK, linewidth=2, label="true path", zorder=3, solid_capstyle="round")
    ax.plot(on.odom_x, on.odom_y, color=ORANGE, linewidth=2, label="odometry alone (corrections taken out)", zorder=4,
            solid_capstyle="round")
    ax.plot(on.pose_x, on.pose_y, color=BLUE, linewidth=2, label="fused (what the motions read)", zorder=5,
            solid_capstyle="round")
    last = on.iloc[-1]
    for x, y, c in ((last.true_x, last.true_y, INK), (last.odom_x, last.odom_y, ORANGE), (last.pose_x, last.pose_y, BLUE)):
        ax.scatter([x], [y], s=40, color=c, edgecolors=SURFACE, linewidths=2, zorder=6)
    ax.scatter([on.true_x.iloc[0]], [on.true_y.iloc[0]], s=40, color=INK, edgecolors=SURFACE, linewidths=2, zorder=6)
    ax.annotate("start", (on.true_x.iloc[0], on.true_y.iloc[0]), xytext=(6, -12), textcoords="offset points",
                fontsize=9, color=INK_2)
    ax.annotate(f"end: odometry alone {last.err_odom:.1f} in off,\nfused {last.err_fused:.1f} in off",
                (last.true_x, last.true_y), xytext=(22, 52), textcoords="data", fontsize=9, color=INK_2,
                arrowprops={"arrowstyle": "-", "color": MUTED, "linewidth": 1})
    ax.set_xlim(-76, 76)
    ax.set_ylim(-76, 76)
    ax.set_aspect("equal")
    ax.set_xlabel("x, in (toward the 90° wall)")
    ax.set_ylabel("y, in (toward the 0° wall)")
    ax.set_title(f"60 s Skills route, A3a odometry errors, fusion On\nseed {seed}: the median fused end error of "
                 f"{len(runs)}", loc="left", color=INK)
    legend(ax, loc="upper center", bbox_to_anchor=(0.5, -0.09), ncol=2, markerscale=3, handlelength=1.6)
    fig.tight_layout()
    save(fig, "gps_route_top.png")

    # 2. position error against time: A3a both spins and A3b, median and the 10 to 90 % band over seeds
    panels = [("A3a cw", "A3a: IMU 1.2 %, wheels 1.5 %\nfull spin clockwise"),
              ("A3a ccw", "A3a: IMU 1.2 %, wheels 1.5 %\nfull spin counter-clockwise"),
              ("A3b cw", "A3b: calibrated (IMU 0.2 %, wheels 0.5 %)\nroute as written")]
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.6), dpi=150, sharey=True)
    fig.patch.set_facecolor(SURFACE)
    top = 0
    for ax, (label, title) in zip(axes, panels):
        style_axes(ax)
        r = a3[label]
        ends = []
        for which, column, color, name in ((0, "err_odom", ORANGE, "odometry alone (fusion Off)"),
                                           (1, "err_fused", BLUE, "fused (fusion On)")):
            series = [r[s][which][1][column] for s in sorted(r)]
            t = r[sorted(r)[0]][which][1]["t_ms"] / 1000
            med, lo, hi, n = band(series)
            ax.fill_between(t[:n], lo, hi, color=color, alpha=0.13, linewidth=0)
            ax.plot(t[:n], med, color=color, linewidth=2, label=name)
            ends.append((t[n - 1], med[-1]))
            top = max(top, float(np.max(hi)))
        # the two end values, pushed apart when they would overlap (the upper one up, the lower one down)
        close = abs(ends[0][1] - ends[1][1]) < 0.6
        upper = 0 if ends[0][1] >= ends[1][1] else 1
        for i, (x, y) in enumerate(ends):
            dy = 0 if not close else (6 if i == upper else -6)
            ax.annotate(f"{y:.1f}", (x, y), xytext=(4, dy), textcoords="offset points", va="center", fontsize=9,
                        color=INK_2)
        ax.set_xlim(0, 66)
        ax.set_title(title, loc="left", color=INK, fontsize=10)
        ax.set_xlabel("time since the route started, s")
    axes[0].set_ylabel("position error, in")
    axes[0].set_ylim(0, top * 1.05)
    legend(axes[0], loc="upper left")
    fig.suptitle(f"Position error on the Skills route, {len(seeds)} seeds each: median, band 10-90 %", x=0.01,
                 ha="left", color=INK, fontsize=12)
    fig.tight_layout()
    save(fig, "gps_error_time.png")

    # 3. A10: the 20 s drifting view loss, fused error minus odometry alone's, per seed and median
    blind = RUNS["a10_blind"]
    limit = CFG["maxTotalCorrectionIn"] + 1
    fig, ax = plt.subplots(figsize=(9, 4.6), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    style_axes(ax)
    ax.axvspan(BLIND_LOSS_START_S, BLIND_LOSS_START_S + BLIND_LOSS_LENGTH_S, color=SHADE, linewidth=0, zorder=0)
    ax.annotate("view lost: the output drifts 1.5 in/s", (BLIND_LOSS_START_S, limit + 2.2), xytext=(4, 0),
                textcoords="offset points", fontsize=9, color=INK_2, va="center")
    for i, (s, on, t, ex) in enumerate(blind):
        ax.plot(t / 1000, ex, color=BLUE, linewidth=0.7, alpha=0.35, label="fused minus odometry alone, each seed"
                if i == 0 else None)
    med, lo, hi, n = band([b[3] for b in blind])
    ax.plot(blind[0][2][:n] / 1000, med, color=BLUE, linewidth=2, label="median over seeds")
    ax.axhline(limit, color=INK_2, linewidth=1, linestyle=(0, (4, 3)))
    ax.annotate(f"limit: maxTotalCorrectionIn + 1 in = {limit:.0f} in", (1, limit), xytext=(0, 4),
                textcoords="offset points", fontsize=9, color=INK_2)
    ax.axhline(0, color=AXIS, linewidth=1)
    fx, fy = [], []
    for s, on, t, ex in blind:
        fm = on["filter"]["fault_ms"]
        if fm >= 0:
            k = int(np.searchsorted(t, fm))
            fx.append(fm / 1000)
            fy.append(ex[min(k, len(ex) - 1)])
    ax.scatter(fx, fy, s=30, color=INK, edgecolors=SURFACE, linewidths=1.5, zorder=5,
               label=f"fault ({faults_text([b[1] for b in blind])})")
    ax.set_xlim(0, 66)
    ax.set_ylim(min(-4, float(np.min(lo)) - 1), limit + 4)
    ax.set_xlabel("time since the route started, s")
    ax.set_ylabel("fused error minus odometry alone's, in")
    ax.set_title(f"A10: GPS loses the strip for 20 s and drifts, A3a errors, {len(blind)} seeds", loc="left", color=INK)
    legend(ax, loc="center right", bbox_to_anchor=(1, 0.6))
    fig.tight_layout()
    save(fig, "gps_blind.png")

    # 4. A9: error around the shove, 6 in and 15 in, perfect odometry
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.4), dpi=150, sharey=True)
    fig.patch.set_facecolor(SURFACE)
    for ax, size in zip(axes, ("recover", "fault")):
        style_axes(ax)
        runs9 = RUNS["a9"][size]
        frames = {}
        for which in ("err_odom", "err_fused"):
            series = []
            for s in sorted(runs9):
                summary, c = runs9[s]
                t = c["t_ms"] - summary["shoves"][0]["start_ms"]
                keep = (t >= -2000) & (t <= 14000)
                series.append(c[which][keep])
                tt = t[keep] / 1000
            frames[which] = (tt, series)
        for which, color, name in (("err_odom", ORANGE, "odometry alone"), ("err_fused", BLUE, "fused")):
            tt, series = frames[which]
            med, lo, hi, n = band(series)
            ax.fill_between(tt[:n], lo, hi, color=color, alpha=0.13, linewidth=0)
            ax.plot(tt[:n], med, color=color, linewidth=2, label=name + " (median, band 10-90 %)")
        if size == "recover":
            ax.axhline(2, color=INK_2, linewidth=1, linestyle=(0, (4, 3)))
            ax.annotate("2 in", (14, 2), xytext=(4, 0), textcoords="offset points", va="center", fontsize=9, color=INK_2)
            ax.axvline(5, color=INK_2, linewidth=1, linestyle=(0, (4, 3)))
            ax.annotate("5 s", (5, 10), xytext=(4, 0), textcoords="offset points", fontsize=9, color=INK_2)
            ax.set_title(f"{A9_RECOVER_IN:g} in shove: relocalised ({len(runs9)} seeds)", loc="left", color=INK,
                         fontsize=10)
        else:
            ft = [runs9[s][0]["filter"]["fault_ms"] - runs9[s][0]["shoves"][0]["start_ms"] for s in runs9
                  if runs9[s][0]["filter"]["fault_ms"] >= 0]
            if ft:
                ax.axvline(median(ft) / 1000, color=INK_2, linewidth=1, linestyle=(0, (4, 3)))
                ax.annotate(f"fault (median {median(ft) / 1000:.1f} s after the shove): what was\nfed in is given back, "
                            "and fused lies on odometry alone",
                            (median(ft) / 1000, 3), xytext=(6, 0), textcoords="offset points", fontsize=9, color=INK_2)
            ax.set_title(f"{A9_FAULT_IN:g} in shove: too far to relocalise, fault ({len(runs9)} seeds)", loc="left",
                         color=INK, fontsize=10)
        ax.set_xlabel("time since the shove started, s")
        ax.set_xlim(-2, 14)
    axes[0].set_ylabel("position error, in")
    axes[0].set_ylim(0, 17)
    legend(axes[0], loc="upper right")
    fig.suptitle("A9: shoves at a stop that the wheels do not see, perfect odometry", x=0.01, ha="left", color=INK,
                 fontsize=12)
    fig.tight_layout()
    save(fig, "gps_shove.png")

    # 5. A8: accuracy against the true delay (tools/sim/out only)
    per = RESULTS["A8"]["per_delay_ms"]
    fig, ax = plt.subplots(figsize=(6.4, 3.8), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    style_axes(ax)
    d = DELAYS_MS
    ax.plot(d, [per[x]["median_fused_end_in"] for x in d], color=BLUE, linewidth=2, marker="o", markersize=6,
            markeredgecolor=SURFACE, markeredgewidth=2, label="fused, median end error")
    ax.plot(d, [per[x]["median_fused_rms_in"] for x in d], color=AQUA, linewidth=2, marker="o", markersize=6,
            markeredgecolor=SURFACE, markeredgewidth=2, label="fused, median rms over the run")
    ax.axvline(60, color=AXIS, linewidth=1)
    ax.annotate("filter told 60 ms", (60, 0), xytext=(4, 6), textcoords="offset points", fontsize=9, color=INK_2)
    ax.set_ylim(bottom=0)
    ax.set_xlabel("true delay of the GPS, ms")
    ax.set_ylabel("position error, in")
    ax.set_title("A8: accuracy against the sensor's true delay", loc="left", color=INK)
    legend(ax, loc="lower right")
    fig.tight_layout()
    save(fig, "gps_delay.png")

    os.makedirs(DOCS_SIM, exist_ok=True)
    for name in ("gps_route_top.png", "gps_error_time.png", "gps_blind.png", "gps_shove.png"):
        shutil.copy(os.path.join(OUT, name), os.path.join(DOCS_SIM, name))
    return made


# ---------------------------------------------------------------------------------------------- main
def main():
    global TMP
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seeds", type=int, default=50, help="seeds for A3a, A3b, A4, A6, A8, A11 (the design says 50); "
                                                          "A7, A9 and A10 use 20, A2, A5's moves and Shadow 10")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
    ap.add_argument("--no-figures", action="store_true")
    ap.add_argument("--keep", action="store_true", help="keep the scratch folder of CSVs and traces")
    args = ap.parse_args()
    for tb in (TB_LIVE, TB_PREGPS):
        if not os.path.exists(tb):
            sys.exit(f"{tb} is missing: run make -C tools/sim first")
    os.makedirs(OUT, exist_ok=True)
    TMP = tempfile.mkdtemp(prefix="gps_suite_", dir=os.path.join(SIM, "build"))
    ROUTES["cw"] = ROUTE
    ROUTES["ccw"] = tmp("skills60_ccw.txt")
    with open(ROUTE) as f, open(ROUTES["ccw"], "w") as g:
        text = f.read()
        assert "turn 135 cw" in text and "turn 315 cw" in text
        g.write(text.replace("turn 135 cw", "turn 135 ccw").replace("turn 315 cw", "turn 315 ccw"))
    for move in STD_MOVES:
        ROUTES[move] = os.path.join(SIM, "routes", move + ".txt")
    ROUTES[TURN_FIRST_ROUTE] = os.path.join(SIM, "routes", TURN_FIRST_ROUTE)
    # A3b's end at several moments: the same routes with a 4 s wait at the end instead of 0.5 s
    for spin in ("cw", "ccw"):
        with open(ROUTES[spin]) as f:
            lines = f.read().rstrip("\n").split("\n")
        assert lines[-1].split()[0] == "wait"
        lines[-1] = f"wait {max(END_MOMENTS_MS)}"
        ROUTES[spin + "_end4"] = tmp(f"skills60_{spin}_end4.txt")
        with open(ROUTES[spin + "_end4"], "w") as g:
            g.write("\n".join(lines) + "\n")
    seeds = list(range(1, args.seeds + 1))
    twenty = seeds[:20]
    ten = seeds[:10]
    try:
        print("M ...", flush=True)
        test_model(args.jobs, list(range(1, 31)))
        print("A1 ...", flush=True)
        test_a1(args.jobs)
        print("A2 ...", flush=True)
        test_a2(ten, args.jobs)
        print("A3a, A3b ...", flush=True)
        test_a3(seeds, args.jobs)
        print("A3b revision 3 rows ...", flush=True)
        test_a3b_rev3(seeds, args.jobs)
        if any(r[0] == "A3a" and r[4] == "FAIL" for r in ROWS):
            print("A3a investigation ...", flush=True)
            investigate_a3a(seeds, args.jobs)
        print("A4 ...", flush=True)
        test_a4(seeds, args.jobs)
        print("comparison at the old noise ...", flush=True)
        test_old_noise(seeds, args.jobs)
        print("A5 ...", flush=True)
        test_a5(ten, args.jobs)
        print("A6 ...", flush=True)
        test_a6(seeds, args.jobs)
        print("A7 ...", flush=True)
        test_a7(twenty, args.jobs)
        print("A8 ...", flush=True)
        test_a8(seeds, args.jobs)
        print("A9 ...", flush=True)
        test_a9(twenty, args.jobs)
        print("A10 ...", flush=True)
        test_a10(twenty, args.jobs)
        print("A11 ...", flush=True)
        test_a11(seeds, args.jobs)
        print("Shadow ...", flush=True)
        test_shadow(ten, args.jobs)
        made = [] if args.no_figures else figures(seeds)
        finish_a5()
    finally:
        if not args.keep:
            shutil.rmtree(TMP, ignore_errors=True)

    # the table, in the design's order (rows of one test stay in the order they were made)
    ROWS.sort(key=lambda r: ORDER.index(r[0].rstrip("?")))
    head = ("Test", "What", "Measured", "Limit", "Result")
    widths = [max(len(str(r[i])) for r in ROWS + [head]) for i in range(5)]
    widths[1] = min(widths[1], 100)
    def line(r):
        return "  ".join(str(r[i])[:widths[i]].ljust(widths[i]) for i in range(5)).rstrip()
    print()
    print(line(head))
    print("  ".join("-" * w for w in widths))
    for r in ROWS:
        print(line(r))
    fails = [r for r in ROWS if r[4] == "FAIL"]
    print(f"\n{sum(r[4] == 'PASS' for r in ROWS)} PASS, {len(fails)} FAIL, {sum(r[4] == 'INFO' for r in ROWS)} INFO")
    RESULTS["table"] = [dict(zip(("test", "what", "measured", "limit", "result"), r)) for r in ROWS]
    RESULTS["settings"] = {"route": os.path.relpath(ROUTE, REPO), "seeds": args.seeds, "noise_scale": NOISE_SCALE,
                           "noisy_scale": NOISY_SCALE, "model_noise_defaults": MODEL_NOISE, "a3a": A3A, "a3b": A3B,
                           "slip_per_in": SLIP_PER_IN, "slip_max_in": SLIP_MAX_IN, "outlier_prob": OUTLIER_PROB,
                           "view_losses": VIEW_LOSSES, "delays_ms": DELAYS_MS, "shove_step": SHOVE_STEP,
                           "blind_loss": BLIND_LOSS, "filter_numbers_used_for_limits": CFG,
                           "a9_sizes_in": [A9_RECOVER_IN, A9_FAULT_IN, A9_SWEEP_IN], "slow_drifts_in_s": SLOW_DRIFTS_IN_S,
                           "still_drift_in_s": STILL_DRIFT_IN_S, "bias_step": BIAS_STEP,
                           "end_moments_ms": END_MOMENTS_MS, "dead_zones_in": DEAD_ZONES_IN}
    with open(os.path.join(OUT, "gps_suite_results.json"), "w") as f:
        json.dump(RESULTS, f, indent=1, default=float, sort_keys=False)
    print(f"results: {os.path.join(OUT, 'gps_suite_results.json')}")
    for path in made:
        print(f"figure: {path}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
