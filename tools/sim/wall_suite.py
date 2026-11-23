#!/opt/anaconda3/bin/python3
"""Wall-fix acceptance tests (docs/WALL_FIX_DESIGN.md, "Acceptance tests" W1 to W11, its "Revision A", which
changes W2 and W4 and adds W12 and W13, and its "Revision B", which restates W8 and W13 and reports the
uncalibrated rows of W2 and W4), run in the host simulator. W1 follows the GPS filter's revision 3: Off against
pre-GPS LemLib, Off with wall sensors fitted, and On with zero wall sensors configured against none.

    make -C tools/sim                              # build the testbenches first
    /opt/anaconda3/bin/python3 tools/sim/wall_suite.py [--seeds 50] [--jobs 12] [--no-figures] [--keep] [--only W2,W4]

Every test runs the real LemLib code and the real filter (src/lemlib/chassis/gpsFusion.cpp) in the bench's route
mode with the simulated distance sensors (tools/sim/bench/dist_model.*; tools/sim/README.md, "Wall fixes"). It
prints one table in the style of gps_suite.py: the measured number, the limit, PASS or FAIL, and INFO rows where the
design says "report" or where the task asked for a number to be shown. Rows whose test name ends in "?" look into
a failing row with one thing changed at a time. Figures go to tools/sim/out/wall_*.png and are copied to docs/sim/.
Every number in the table, and the per-seed results behind it, is saved in tools/sim/out/wall_suite_results.json.

Choices the design does not make, all stated in the table and in tools/sim/README.md:
  - Route: routes/walls60.txt (58 s, 13 stops of 0.7 s with the back sensor square to a wall 9.75 to 18.75 in
    away, 5 of them with the right sensor square to the next wall too); W7 uses routes/walls_corner.txt. The
    GPS is off (--no-gps) except in W10 and W12.
  - Sensors: the back sensor (right 0, forward -8.5, yaw 180) for "one sensor", plus the right sensor (right 8.5,
    forward 0, yaw 90) for "two sensors". The filter is told exactly where they are unless a row says otherwise.
  - The sensors are noisy in every row unless the row says "noise-free": the distance model's suggested settings
    (white 3 mm + 0.3 %, slowly varying 2 mm + 0.5 %, confidence jitter 2) plus a calibration left over per
    sensor and seed (scale error up to 1 %, offset up to 3 mm, uniform, drawn from the seed). About 0.27 in at
    12 in, inside the filter's assumed 0.3 in + 1 %. Everything else in the model is at its defaults (blend cone
    of 12 degrees, 30 readings a second, confidence falling with range and angle, solid walls).
  - The filter's wall numbers are the design's first values: every wall Clear. "Solid" rows mark all four Solid.
  - Odometry errors are the GPS suite's (gps_suite.py, imported): A3a's uncalibrated robot (IMU 1.2 %, wheels
    1.5 %) and A3b's calibrated one (IMU 0.2 %, wheels 0.5 %), with random slips, signs drawn per seed. The team
    only switches fusion on with a calibrated robot (ODOMETRY_CALIBRATED), so the calibrated rows matter most.
  - "Fused" is the pose the motions read with fusion On; "odometry alone" is a fusion-Off run with the same seed
    and errors (Off never reads a sensor, so it is the same run whatever the sensors do; W1 checks that). Where the
    design says "the pose equals odometry alone" (W4, W8, W13) it is measured within the On run: nothing was ever
    fed into the pose (the bench's max_fed_in is exactly 0), so the pose is odometry alone at every tick.
  - "Error added" by a fix: the error of the filter's estimate on the fix's axis after the fix, minus before (in
    absolute value; the truth is the bench's). "Implied error": where the reading put the robot on that axis
    (estimate before the fix plus the innovation) minus the truth: the error a fix would leave at full weight.
"""

import argparse
import concurrent.futures
import json
import math
import os
import re
import shutil
import statistics
import subprocess
import sys
import tempfile
import threading

import numpy as np

SIM = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SIM)
import gps_suite as gs  # noqa: E402  the GPS suite's odometry errors, GPS noise and figure style

REPO = os.path.dirname(os.path.dirname(SIM))
TB_LIVE = os.path.join(SIM, "build", "live", "tb_live")
TB_PREGPS = os.path.join(SIM, "build", "pregps", "tb_pregps")
ROUTE = os.path.join(SIM, "routes", "walls60.txt")
CORNER_ROUTE = os.path.join(SIM, "routes", "walls_corner.txt")
OUT = os.path.join(SIM, "out")
DOCS_SIM = os.path.join(REPO, "docs", "sim")
# W1's old regression against the frozen revision 2 filter: a script and a golden binary kept outside the repository
# by the filter's engineer. No longer run (the filter changed on purpose in revision 3); the folder is left
# untouched and only named in the results. WALL_W1_DIR overrides the folder
W1_DIR = os.environ.get("WALL_W1_DIR", "/private/tmp/claude-501/-Users-arona--claude-spares-spare-GV4Lkt/"
                        "e6061d89-be15-49e0-8d04-472b7878408e/scratchpad/walls")
W1_SCRIPT = os.path.join(W1_DIR, "w1_regress.sh")
W1_GOLDEN = os.path.join(W1_DIR, "golden", "tb_gpsv2")

# ---------------------------------------------------------------------------------------------- test settings
# What the filter is told about the two sensors (--wall-sensor RIGHT:FORWARD:YAW), the same as where the distance
# model puts them by default (dist_model.hpp: s0 and s1)
BACK = (0.0, -8.5, 180.0)
RIGHT = (8.5, 0.0, 90.0)
SENSOR_SETS = {1: [BACK], 2: [BACK, RIGHT]}
# the noisy sensor (dist_model README's suggestion) and the calibration left over, drawn per sensor and seed
NOISY = {"noise_mm": 3, "noise_frac": 0.003, "slow_mm": 2, "slow_frac": 0.005, "conf_sd": 2}
CAL_SCALE_MAX = 0.01
CAL_OFFSET_MM_MAX = 3.0
# the filter's numbers used for limits only (lemlib::gpsfusion::WallConfig and Config defaults)
CFG = {"capClearIn": 1.5, "capSolidIn": 6.0, "blockedMaxIn": 2.5, "slewInPerS": 2.0,
       "maxTotalCorrectionIn": gs.CFG["maxTotalCorrectionIn"],
       "wallStartTolIn": 3.0, "planeIn": 70.25}
ODOM_TICK_S = 0.010
FAULT_NAMES = gs.FAULT_NAMES
STATE_NAMES = ["wall", "clear wall", "corner", "element", "object", "stop object", "through", "no return",
               "too close", "off"]
CLEAN_BITS = (1 << 0) | (1 << 1)  # DistState WALL and CLEAR_WALL: a reading of the wall itself
# W4: the object in the beam at every stop (dist_model: stop_obj)
W4_VARIANTS = [
    ("6-20 in from the sensor (at least 3 in in front of the wall)",
     ["stop_obj=1", "stop_obj_min_in=6", "stop_obj_max_in=20", "stop_obj_gap_in=3"], True),
    ("3-10 in in front of the wall", ["stop_obj=2", "stop_obj_min_in=3", "stop_obj_max_in=10"], True),
    ("1-3 in in front of the wall", ["stop_obj=2", "stop_obj_min_in=1", "stop_obj_max_in=3"], False),
]
# the first stop (B1) ends 4.4 s into the route; from 6 s (world time = route time + 10 ms) the objects are there,
# so the sensors' start checks happen on a clean wall and the objects have to be refused by the later checks
W4_LATE_FROM_MS = 6000
# W5: every wall clear plastic; the beam always reads through to something 2 to 40 in behind, and nothing comes
# back from beyond 28 in
W5_CLEAR = ["clear=1", "through_prob=1", "behind_in=2", "behind_spread_in=38", "dropout_in=28"]
# W7: the corner route's two placeholder elements, given to the model and to the filter as keep-outs
W7_ELEMENTS = [(3, -6, 6, 4), (0, 18, 30, 4)]
CONE_MODELS = {0: "centre ray", 1: "nearest point", 2: "blend (default)"}
# W9: standard moves after a lead-in that ends with a fix (routes/wall_std_*.txt)
STD_MOVES = ["wall_std_drive24", "wall_std_drive48", "wall_std_back24", "wall_std_turn90", "wall_std_pose"]
# W11: the -x wall is truly 0.5 in farther out than the filter is told
W11_WALL, W11_ERROR_IN = 3, 0.5
# W13 (Revision B): the total limit shown working with it lowered to these (uncalibrated robot, true walls), and
# a plane told 2 to 3 in wrong on a Solid wall reported
W13_LOW_LIMITS_IN = [1.0, 2.0]
W13_PLANE_ERRORS_IN = [2, 3, -2, -3]
# W8 (Revision B): the start error, and the margin round the start check's tolerance inside which a sensor's
# verdict is reported, not judged (a per-seed direction puts part of the error on each axis)
W8_ERROR_IN = 5.0
W8_MARGIN_IN = 1.0
# per-tick columns kept from the route CSVs (the CSVs are deleted straight away)
COLS = ["t_ms", "step", "moving", "true_x", "true_y", "true_heading", "pose_x", "pose_y", "odom_x", "odom_y", "cmdL",
        "cmdR", "err_odom", "err_fused"]


def sensor_arg(mount, scale=1.0, offset=0.0):
    r, f, y = mount
    text = f"{r:g}:{f:g}:{y:g}"
    if scale != 1.0 or offset != 0.0:
        text += f":{scale:.6g}:{offset:.6g}"
    return text


def calibration_left(seed):
    """The calibration left over for sensors 0 and 1 in this seed: (scale error, offset mm) each. Both sensors are
    always drawn, so fitting the second sensor never changes the first one's."""
    rng = np.random.RandomState(40_000 + seed)
    v = rng.uniform(-1, 1, size=4)
    return [(CAL_SCALE_MAX * v[0], CAL_OFFSET_MM_MAX * v[1]), (CAL_SCALE_MAX * v[2], CAL_OFFSET_MM_MAX * v[3])]


def dist(*keys):
    out = []
    for k in keys:
        out += ["--dist", k]
    return out


def wall_args(seed, sensors=1, noisy=True, mounts=None, dist_extra=(), wall_extra=()):
    """--wall-sensor for each sensor (what the filter is told; `mounts` overrides it as (r, f, yaw[, scale,
    offset]) tuples), the noisy model and its calibration left over, then any extra model and filter options."""
    out = []
    told = mounts if mounts is not None else SENSOR_SETS[sensors]
    for m in told:
        out += ["--wall-sensor", sensor_arg(m[:3], *(m[3:] if len(m) > 3 else ()))]
    if noisy:
        out += dist(*[f"{k}={v}" for k, v in NOISY.items()])
        for i, (scale, offset) in enumerate(calibration_left(seed)[:len(told)]):
            out += dist(f"s{i}_scale_err={scale:.6g}", f"s{i}_offset_mm={offset:.6g}")
    return out + list(dist_extra) + list(wall_extra)


def on_args(seed, errors="a3b", sensors=1, gps=False, noisy=True, extra=(), **kw):
    """A fusion-On run: odometry errors, no GPS (or the GPS suite's 1.5 in GPS), the wall sensors."""
    gps_args = gs.NOISE if gps else ["--no-gps"]
    return gs.ERRORS[errors](seed) + gps_args + wall_args(seed, sensors, noisy, **kw) + list(extra) + \
        ["--seed", seed, "--fusion", "on"]


# ---------------------------------------------------------------------------------------------- running the bench
TMP = None
_counter = [0]
_lock = threading.Lock()
ON_RUNS = []  # (test, seed, largest correction in one tick) for every fusion-On run (W9 part i)


def tmp(name):
    return os.path.join(TMP, name)


def unique(prefix):
    with _lock:
        _counter[0] += 1
        return tmp(f"{prefix}_{_counter[0]}")


def bench(args, route=ROUTE, cols=None, trace=None, tb=TB_LIVE, test="", csv=None):
    """One route through the bench. Returns its JSON summary, and the CSV columns `cols` as numpy arrays when asked
    for (the CSV is then deleted; pass csv= to keep it)."""
    cmd = [tb, "--route", route] + [str(a) for a in args] + ["--summary-json"]
    path = csv
    if cols and path is None:
        path = unique("run") + ".csv"
    if path:
        cmd += ["--csv", path]
    if trace:
        cmd += ["--pose-trace", trace]
    done = subprocess.run(cmd, capture_output=True, text=True)
    if done.returncode != 0:
        raise RuntimeError(f"{' '.join(cmd)}\n{done.stderr}")
    summary = json.loads(done.stdout)
    if summary.get("fusion") == "on" and "filter" in summary:
        with _lock:
            ON_RUNS.append((test, summary["seed"], summary["filter"]["max_step_in"]))
    if not cols:
        return summary
    import pandas as pd
    frame = pd.read_csv(path, usecols=cols)
    if csv is None:
        os.remove(path)
    return summary, {c: frame[c].to_numpy() for c in cols}


def parallel(fn, items, jobs):
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
        return list(pool.map(fn, items))


def median(values):
    values = [v for v in values if v is not None and not (isinstance(v, float) and math.isnan(v))]
    return statistics.median(values) if values else float("nan")


def same_file(a, b):
    with open(a, "rb") as fa, open(b, "rb") as fb:
        return fa.read() == fb.read()


# Odometry alone: a fusion-Off run per (errors, route, seed), with no sensor at all (Off reads none, so the pose is
# the same whatever sensors are configured; W1 checks it)
OFF = {}


def off_run(errors, route, seed):
    key = (errors, route, seed)
    if key not in OFF:
        OFF[key] = bench(gs.ERRORS[errors](seed) + ["--no-gps", "--seed", seed, "--fusion", "off"], route=route,
                         cols=COLS, test="off")
    return OFF[key]


def fill_off(errors, route, seeds, jobs):
    parallel(lambda s: off_run(errors, route, s), seeds, jobs)


# ---------------------------------------------------------------------------------------------- the route's stops
def route_stops(path):
    """{step n: name} for every wait step of a route, named by its comment (B1, BR1, K4...) or 'wait@line'."""
    stops, n = {}, 0
    with open(path) as f:
        for line_no, raw in enumerate(f, 1):
            body = raw.split("#", 1)[0].split()
            if not body or body[0] == "start":
                continue
            n += 1
            if body[0] == "wait":
                m = re.match(r"\s*([A-Z]+\d+)", raw.split("#", 1)[1]) if "#" in raw else None
                stops[n] = m.group(1) if m else "end"
    return stops


STOPS = {}


def stop_of(route, step):
    """The stop a fix at route step `step` belongs to: the first wait at or after it (a fix can come while the
    turn before the wait is settling, which LemLib still counts as part of the turn)."""
    later = [n for n in STOPS[route] if n >= step]
    return STOPS[route][min(later)] if later else "after the last stop"


def fixes_of(summary):
    """Every fix of a run, over all its wall sensors: (sensor, fix record)."""
    out = []
    for i, s in enumerate(summary["walls"]["per_sensor"]):
        out += [(i, f) for f in s["fix_list"]]
    return out


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


# ---------------------------------------------------------------------------------------------- the table
ROWS = []
RESULTS = {}
RUNS = {}
ORDER = ["W1", "W2", "W3", "W4", "W5", "W6", "W7", "W8", "W9", "W10", "W11", "W12", "W13", "E"]


def row(test, what, measured, limit, ok):
    verdict = "INFO" if ok is None else ("PASS" if ok else "FAIL")
    ROWS.append((test, what, measured, limit, verdict))


def ratio_text(f, o):
    return f"{100 * f / o:.0f} % ({f:.2f} / {o:.2f} in)" if o > 0 else f"{f:.2f} / {o:.2f} in"


# ---------------------------------------------------------------------------------------------- W1
def run_single(args, tb, csv, trace):
    cmd = [tb] + args.split() + ["--csv", csv, "--pose-trace", trace]
    done = subprocess.run(cmd, capture_output=True, text=True)
    if done.returncode != 0:
        raise RuntimeError(f"{' '.join(cmd)}\n{done.stderr}")


def csv_prefix_same(a, b, ncols):
    """The first ncols columns of two CSVs are the same text on every line (the wall columns come after them)."""
    with open(a) as fa, open(b) as fb:
        la, lb = fa.read().splitlines(), fb.read().splitlines()
    if len(la) != len(lb):
        return False
    return all(x.split(",")[:ncols] == y.split(",")[:ncols] for x, y in zip(la, lb))


def test_w1(jobs):
    """W1 (revision 3 of the GPS filter). The filter changed on purpose (heading fault rule, limits), so the
    comparison with the frozen revision 2 build (w1_regress.sh and its golden binary, kept untouched where they
    are) no longer means anything. W1 is now: (a) Off is bit-identical to pre-GPS LemLib; (b) Off with wall sensors
    fitted is bit-identical; (c) with fusion On and no wall sensor, configuring zero wall sensors through
    configureWalls() changes nothing (bit-identical pose traces)."""
    out = {}
    # (a) mode Off: every single move and route runs on both wall routes, tb_live against tb_pregps
    def single(i):
        res = []
        for name, tb in (("live", TB_LIVE), ("pregps", TB_PREGPS)):
            run_single(gs.SINGLE_MOVES[i], tb, tmp(f"w1_{i}_{name}.csv"), tmp(f"w1_{i}_{name}.trace"))
        res.append(same_file(tmp(f"w1_{i}_live.csv"), tmp(f"w1_{i}_pregps.csv")))
        res.append(same_file(tmp(f"w1_{i}_live.trace"), tmp(f"w1_{i}_pregps.trace")))
        return res

    routes = [(r, s, e) for r in (ROUTE, CORNER_ROUTE) for s, e in ((1, "a3a"), (2, "a3b"), (3, "perfect"))]

    def route(job):
        path, seed, errors = job
        tag = f"{os.path.basename(path)}_{seed}"
        args = gs.ERRORS[errors](seed) + ["--seed", seed, "--fusion", "off"]
        for name, tb in (("live", TB_LIVE), ("pregps", TB_PREGPS)):
            bench(args + ["--csv", tmp(f"w1r_{tag}_{name}.csv")], route=path, tb=tb,
                  trace=tmp(f"w1r_{tag}_{name}.trace"), test="W1")
        # (b) and Off with two wall sensors fitted and misbehaving (objects, clear walls): only columns are added
        bench(args + wall_args(seed, 2, dist_extra=dist("stop_obj=2", "wall1_clear=1", "wall1_through_prob=0.5")) +
              ["--csv", tmp(f"w1r_{tag}_walls.csv")], route=path, trace=tmp(f"w1r_{tag}_walls.trace"), test="W1")
        return [same_file(tmp(f"w1r_{tag}_live.csv"), tmp(f"w1r_{tag}_pregps.csv")),
                same_file(tmp(f"w1r_{tag}_live.trace"), tmp(f"w1r_{tag}_pregps.trace")),
                same_file(tmp(f"w1r_{tag}_live.trace"), tmp(f"w1r_{tag}_walls.trace")),
                csv_prefix_same(tmp(f"w1r_{tag}_live.csv"), tmp(f"w1r_{tag}_walls.csv"), len(gs_route_columns()))]

    gs_route_columns()
    singles = parallel(single, range(len(gs.SINGLE_MOVES)), jobs)
    rr = parallel(route, routes, jobs)
    pre = [ok for s in singles for ok in s] + [ok for r in rr for ok in r[:2]]
    walls = [ok for r in rr for ok in r[2:]]
    out["off"] = {"files_compared": len(pre), "files_different": pre.count(False),
                  "with_walls_compared": len(walls), "with_walls_different": walls.count(False)}
    RESULTS["W1"] = out
    row("W1", f"mode Off vs pre-GPS LemLib, CSV and exact pose trace: {len(gs.SINGLE_MOVES)} moves, {len(routes)} runs "
              "of the two wall routes", f"{pre.count(False)} of {len(pre)} files differ", "0 differ",
        pre.count(False) == 0)
    row("W1", "  ... Off with two wall sensors fitted (objects, a clear wall): pose trace, and the CSV's columns "
              "before the wall ones", f"{walls.count(False)} of {len(walls)} differ", "0 differ", walls.count(False) == 0)


    # (c) fusion On, no wall sensor: configureWalls() with zero sensors against not calling it at all. GPS-only
    # runs on both routes and both robots, healthy and attacked (outliers and view losses, a blind drift, a shove)
    gps_routes = [os.path.join(SIM, "routes", "skills60.txt"), ROUTE]
    zero_runs = [(r, sd, e, extra) for r in gps_routes for sd, e in ((1, "a3a"), (2, "a3b"), (3, "perfect"))
                 for extra in ([], gs.a6_extras(), ["--gps-loss", gs.BLIND_LOSS], ["--shove", f"s{gs.SHOVE_STEP}:6:0:200"])
                 if not (extra and extra[0] == "--shove" and r == ROUTE)]  # the shove's step is skills60's

    def zero(job):
        i, (path, seed, errors, extra) = job
        args = gs.ERRORS[errors](seed) + gs.NOISE + list(extra) + ["--seed", seed, "--fusion", "on"]
        bench(args, route=path, trace=tmp(f"w1z_{i}_none.trace"), test="W1")
        bench(args + ["--walls-zero"], route=path, trace=tmp(f"w1z_{i}_zero.trace"), test="W1")
        return same_file(tmp(f"w1z_{i}_none.trace"), tmp(f"w1z_{i}_zero.trace"))

    same = parallel(zero, list(enumerate(zero_runs)), jobs)
    out["zero_wall_sensors"] = {"runs": len(same), "identical": sum(same)}
    row("W1", f"fusion On, no wall sensor: configureWalls() with zero sensors vs not called, exact pose trace: "
              f"{len(same)} GPS-only runs (2 routes, 3 robots, healthy and attacked)",
        f"{len(same) - sum(same)} of {len(same)} differ", "0 differ", all(same))


_CACHE = {}


def gs_route_columns():
    """The route CSV's columns before the wall ones (the header of a run without wall sensors)."""
    if "route_cols" not in _CACHE:
        path = tmp("cols.csv")
        bench(["--no-gps", "--fusion", "off", "--csv", path], route=os.path.join(SIM, "routes", "std_turn90.txt"))
        with open(path) as f:
            _CACHE["route_cols"] = f.readline().strip().split(",")
    return _CACHE["route_cols"]


# ---------------------------------------------------------------------------------------------- W2
ROBOTS = {"a3b": "calibrated (IMU 0.2 %, wheels 0.5 %)", "a3a": "uncalibrated (IMU 1.2 %, wheels 1.5 %)"}
SHORT = {"a3b": "calibrated", "a3a": "uncalibrated"}
W2_LIMIT = {1: 0.60, 2: 0.40}


def end_stats(runs, seeds):
    f = [runs[s][1][0]["fused"]["end"] for s in seeds]
    o = [runs[s][0][0]["odom"]["end"] for s in seeds]
    return {"median_fused_end_in": median(f), "median_odom_end_in": median(o),
            "median_paired_difference_in": median([a - b for a, b in zip(f, o)]),
            "per_seed": {s: {"fused_end": a, "odom_end": b} for s, a, b in zip(seeds, f, o)}}


def whole_route(runs, seeds):
    """Whole-route error of each run (the per-tick err_fused of the On run, err_odom of the Off run, each over its
    own ticks): medians over seeds of the rms and the mean, and the seeds whose fused end error is worse."""
    def rms(x):
        return float(np.sqrt(np.mean(np.square(x))))
    f_rms = [rms(runs[s][1][1]["err_fused"]) for s in seeds]
    o_rms = [rms(runs[s][0][1]["err_odom"]) for s in seeds]
    f_mean = [float(np.mean(runs[s][1][1]["err_fused"])) for s in seeds]
    o_mean = [float(np.mean(runs[s][0][1]["err_odom"])) for s in seeds]
    worse = sum(runs[s][1][0]["fused"]["end"] > runs[s][0][0]["odom"]["end"] for s in seeds)
    return {"median_fused_rms_in": median(f_rms), "median_odom_rms_in": median(o_rms),
            "median_fused_mean_in": median(f_mean), "median_odom_mean_in": median(o_mean),
            "seeds_end_worse": int(worse)}


def fixes_by_stop(route, summaries):
    """{stop name: [fixes by sensor 0 over all runs, by sensor 1]}, and fixes per run."""
    by_stop, per_run = {}, []
    for summ in summaries:
        n = 0
        for i, f in fixes_of(summ):
            name = stop_of(route, f["step"])
            by_stop.setdefault(name, [0, 0])[i] += 1
            n += 1
        per_run.append(n)
    return by_stop, per_run


def test_w2(seeds, jobs):
    """W2 (Revision A): walls60, drifting odometry, GPS off, one sensor then two; 50 seeds. Plus the INFO rows."""
    runs = {}
    for errors in ("a3b", "a3a"):
        fill_off(errors, ROUTE, seeds, jobs)
        for sensors in (1, 2):
            on = parallel(lambda s: bench(on_args(s, errors, sensors), cols=COLS, test="W2"), seeds, jobs)
            runs[(errors, sensors)] = {s: (OFF[(errors, ROUTE, s)], on[i]) for i, s in enumerate(seeds)}
    RUNS["w2"] = runs
    out = {}
    for (errors, sensors), r in runs.items():
        v = end_stats(r, seeds)
        ons = [r[s][1][0] for s in seeds]
        v["faults"] = sorted({x["filter"]["fault_name"] for x in ons})
        v["sensor_states"] = {}
        for x in ons:
            for p in x["walls"]["per_sensor"]:
                v["sensor_states"][p["state"]] = v["sensor_states"].get(p["state"], 0) + 1
        out[f"{errors} {sensors}"] = v
        f, o = v["median_fused_end_in"], v["median_odom_end_in"]
        if errors == "a3b":
            row("W2", f"{ROBOTS[errors]}, {sensors} sensor{'s' if sensors > 1 else ''}, Clear walls, {len(seeds)} seeds: "
                      "median fused end / median odometry alone", ratio_text(f, o),
                f"<= {100 * W2_LIMIT[sensors]:.0f} %", f / o <= W2_LIMIT[sensors])
            # reported: the whole route, not only its end (per-tick errors from the CSVs; each run over its own
            # ticks, then the median over seeds), and how many seeds end worse than odometry alone
            whole = whole_route(r, seeds)
            v.update(whole)
            row("W2", f"  ... {SHORT[errors]}, {sensors}: whole-route error, fused vs odometry alone (per run over "
                      "every tick, median over seeds): rms; mean",
                f"rms {whole['median_fused_rms_in']:.2f} vs {whole['median_odom_rms_in']:.2f} in; mean "
                f"{whole['median_fused_mean_in']:.2f} vs {whole['median_odom_mean_in']:.2f} in", "(info)", None)
            row("W2", f"  ... {SHORT[errors]}, {sensors}: seeds whose fused end error is worse than odometry alone's",
                f"{whole['seeds_end_worse']} of {len(seeds)}", "(info)", None)
        else:
            # Revision B: On needs a calibrated robot, so the uncalibrated rows are reported, not limited
            row("W2", f"{ROBOTS[errors]}, {sensors} sensor{'s' if sensors > 1 else ''}, Clear walls, {len(seeds)} seeds: "
                      "median fused end / median odometry alone (reported: the 1.5 in Clear cap refuses most "
                      "corrections of an uncalibrated robot's drift; On needs a calibrated robot)", ratio_text(f, o),
                f"(was <= {100 * W2_LIMIT[sensors]:.0f} %)", None)
    RESULTS["W2"] = out

    # Revision A: stops at which a clean reading was refused as blocked or through, and the pose error then
    for (errors, sensors), r in runs.items():
        n_stops = 0
        runs_with = 0
        worst = 0.0
        per_stop = {}
        for s in seeds:
            summ = r[s][1][0]
            had = False
            for p in summ["walls"]["per_sensor"]:
                # (sorted: a set's order changes from one Python run to the next, the results file must not)
                stops = sorted({stop_of(ROUTE, n) for n in p["stops_clean_blocked"] + p["stops_clean_through"]})
                for name in stops:
                    per_stop[name] = per_stop.get(name, 0) + 1
                n_stops += len(stops)
                if stops:
                    had = True
                    worst = max(worst, p["max_clean_refused_err_in"])
            runs_with += had
        top = ", ".join(f"{k} {v}" for k, v in sorted(per_stop.items(), key=lambda kv: (-kv[1], kv[0]))[:4])
        out[f"{errors} {sensors}"]["clean_refused"] = {"sensor_stops": n_stops, "runs": runs_with,
                                                       "largest_axis_err_in": worst, "by_stop": per_stop}
        row("W2", f"  {SHORT[errors]}, {sensors}: stops where a clean reading was refused as blocked or through; "
                  "largest pose error on its axis then",
            f"{n_stops} sensor-stops in {runs_with}/{len(seeds)} runs ({top or 'none'}); {worst:.2f} in", "(report)",
            None)
    # fixes per run and at which stops
    for (errors, sensors), r in runs.items():
        by_stop, per_run = fixes_by_stop(ROUTE, [r[s][1][0] for s in seeds])
        order = [n for n in STOPS[ROUTE].values() if n in by_stop] + [n for n in by_stop if n not in STOPS[ROUTE].values()]
        text = " ".join(f"{k}:{by_stop[k][0]}" + (f"+{by_stop[k][1]}" if sensors > 1 else "") for k in order)
        out[f"{errors} {sensors}"]["fixes"] = {"median_per_run": median(per_run), "min": min(per_run),
                                               "max": max(per_run), "by_stop": by_stop}
        row("W2", f"  {SHORT[errors]}, {sensors}: fixes per run (median, range); fixes at each stop over "
                  f"{len(seeds)} runs" + (" (back+right)" if sensors > 1 else ""),
            f"{median(per_run):.0f} ({min(per_run)}-{max(per_run)}); {text}", "(report)", None)
    # stops at which fused is more than 0.5 in worse than odometry alone
    for (errors, sensors), r in runs.items():
        worse, total, worst = 0, 0, 0.0
        per = {}
        for s in seeds:
            (off, offc), (on, onc) = r[s]
            fo, ff = stop_errors(off, offc, "err_odom"), stop_errors(on, onc, "err_fused")
            for n in fo:
                if n in ff:
                    total += 1
                    d = ff[n] - fo[n]
                    worst = max(worst, d)
                    if d > 0.5:
                        worse += 1
                        per[STOPS[ROUTE].get(n, n)] = per.get(STOPS[ROUTE].get(n, n), 0) + 1
        top = ", ".join(f"{k} {v}" for k, v in sorted(per.items(), key=lambda kv: (-kv[1], kv[0]))[:4])
        out[f"{errors} {sensors}"]["stops_worse_by_half_in"] = {"stops": total, "worse": worse, "worst_in": worst,
                                                                "by_stop": per}
        row("W2", f"  {SHORT[errors]}, {sensors}: stops where fused is more than 0.5 in worse than odometry alone",
            f"{worse} of {total} ({top or 'none'}); worst {worst:+.2f} in", "(report)", None)

    # Solid walls against Clear walls
    solid = {}
    for errors in ("a3b", "a3a"):
        for sensors in (1, 2):
            on = parallel(lambda s: bench(on_args(s, errors, sensors, extra=["--wall-trust", "all:solid"]),
                                          test="W2 solid"), seeds, jobs)
            f = median([x["fused"]["end"] for x in on])
            o = out[f"{errors} {sensors}"]["median_odom_end_in"]
            c = out[f"{errors} {sensors}"]["median_fused_end_in"]
            solid[f"{errors} {sensors}"] = {"median_fused_end_in": f, "median_odom_end_in": o,
                                            "faults": sorted({x["filter"]["fault_name"] for x in on})}
            row("W2", f"  {SHORT[errors]}, {sensors}: all four walls marked Solid instead of Clear (cap 6 in, range "
                      "36 in, confidence 40)", f"{ratio_text(f, o)}; Clear {100 * c / o:.0f} %", "(info)", None)
    RESULTS["W2_solid"] = solid


def investigate_w2(seeds, jobs):
    """W2 failed: one thing changed at a time."""
    out = {}
    r2 = RESULTS["W2"]
    # (1) uncalibrated: which of the Clear wall's numbers stops the fixes
    for sensors in (1, 2):
        if r2[f"a3a {sensors}"]["median_fused_end_in"] / r2[f"a3a {sensors}"]["median_odom_end_in"] <= W2_LIMIT[sensors]:
            continue
        o = r2[f"a3a {sensors}"]["median_odom_end_in"]
        for name, extra in (("Clear cap 6 in instead of 1.5 (capClearIn)", ["--wall", "capClearIn=6"]),
                            ("blocked and through gates 6 in instead of 2.5 / 4", ["--wall", "blockedMaxIn=6",
                                                                                  "--wall", "throughMaxIn=6"]),
                            ("Clear range 36 in instead of 24 (maxRangeClearIn)", ["--wall", "maxRangeClearIn=36"]),
                            ("noise-free sensors", None)):
            if extra is None:
                on = parallel(lambda s: bench(on_args(s, "a3a", sensors, noisy=False), test="W2?"), seeds, jobs)
            else:
                on = parallel(lambda s: bench(on_args(s, "a3a", sensors, extra=extra), test="W2?"), seeds, jobs)
            f = median([x["fused"]["end"] for x in on])
            caps = sum(sum(p["dropped_by_check"][11] for p in x["walls"]["per_sensor"]) for x in on)
            out[f"a3a {sensors} {name}"] = {"median_fused_end_in": f, "cap_drops": caps}
            row("W2?", f"uncalibrated, {sensors}: {name}", f"{ratio_text(f, o)}; cap drops {caps}",
                f"<= {100 * W2_LIMIT[sensors]:.0f} %", None)
        for name, zeroed in (("IMU error only (wheels perfect)", ("VERT_SCALE_ERR", "HORIZ_SCALE_ERR")),
                             ("wheel errors only (IMU perfect)", ("IMU_SCALE_ERR",))):
            def errs(s, zeroed=zeroed):
                # A3a's errors with some of them set to 0 (the slips stay)
                return [x.split("=")[0] + "=0" if x.startswith(zeroed) else x for x in gs.ERRORS["a3a"](s)]
            off = parallel(lambda s: bench(errs(s) + ["--no-gps", "--seed", s, "--fusion", "off"], test="off"), seeds,
                           jobs)
            on = parallel(lambda s: bench(errs(s) + ["--no-gps"] + wall_args(s, sensors) + ["--seed", s, "--fusion",
                                                                                          "on"], test="W2?"),
                          seeds, jobs)
            f, oo = median([x["fused"]["end"] for x in on]), median([x["odom"]["end"] for x in off])
            out[f"a3a {sensors} {name}"] = {"median_fused_end_in": f, "median_odom_end_in": oo}
            row("W2?", f"uncalibrated, {sensors}: {name}", ratio_text(f, oo), f"<= {100 * W2_LIMIT[sensors]:.0f} %",
                None)
        on = [RUNS["w2"][("a3a", sensors)][s][1][0] for s in seeds]
        caps = sum(sum(p["dropped_by_check"][11] for p in x["walls"]["per_sensor"]) for x in on)
        fixes = sum(sum(p["fixes"] for p in x["walls"]["per_sensor"]) for x in on)
        row("W2?", f"uncalibrated, {sensors}: readings dropped by wall check 12 (the Clear cap, 1.5 in) against fixes "
                   "taken, as run", f"{caps} dropped, {fixes} fixes", "(info)", None)
    # (2) two sensors: what is left at the end is the drift of the last leg, which no wall sees
    for errors in ("a3b", "a3a"):
        if r2[f"{errors} 2"]["median_fused_end_in"] / r2[f"{errors} 2"]["median_odom_end_in"] <= W2_LIMIT[2]:
            continue
        last_stop = max(n for n, name in STOPS[ROUTE].items() if name != "end")
        legs, at_last = [], {1: [], 2: []}
        for s in seeds:
            off, offc = RUNS["w2"][(errors, 1)][s][0]
            idx = np.nonzero(offc["step"] == last_stop)[0][-1]
            ex = (offc["odom_x"][-1] - offc["true_x"][-1]) - (offc["odom_x"][idx] - offc["true_x"][idx])
            ey = (offc["odom_y"][-1] - offc["true_y"][-1]) - (offc["odom_y"][idx] - offc["true_y"][idx])
            legs.append(math.hypot(ex, ey))
            for sensors in (1, 2):
                on, onc = RUNS["w2"][(errors, sensors)][s][1]
                j = np.nonzero(onc["step"] == last_stop)[0][-1]
                at_last[sensors].append(float(onc["err_fused"][j]))
        o = r2[f"{errors} 2"]["median_odom_end_in"]
        if errors == "a3b":  # (the uncalibrated robot's noise-free row is above)
            on = parallel(lambda s: bench(on_args(s, errors, 2, noisy=False), test="W2?"), seeds, jobs)
            nf = median([x["fused"]["end"] for x in on])
            out[f"{errors} 2 noise-free"] = {"median_fused_end_in": nf}
            row("W2?", f"{SHORT[errors]}, 2: two noise-free sensors (no noise, no calibration left over; W3 measures "
                       "what the noise costs a fix)", ratio_text(nf, o), f"<= {100 * W2_LIMIT[2]:.0f} %", None)
        out[f"{errors} last leg"] = {"median_last_leg_drift_in": median(legs),
                                     "median_fused_at_last_stop_in": {k: median(v) for k, v in at_last.items()}}
        row("W2?", f"{SHORT[errors]}: odometry alone's own drift over the last leg (from the last stop, "
                   f"{STOPS[ROUTE][last_stop]}, to the end), which no fix can see",
            f"median {median(legs):.2f} in = {100 * median(legs) / o:.0f} % of odometry alone's end error", "(info)",
            None)
        row("W2?", f"{SHORT[errors]}: fused error at the end of the last stop, one sensor / two sensors",
            f"{median(at_last[1]):.2f} / {median(at_last[2]):.2f} in (end of route: "
            f"{r2[f'{errors} 1']['median_fused_end_in']:.2f} / {r2[f'{errors} 2']['median_fused_end_in']:.2f})",
            "(info)", None)
    RESULTS["W2_investigation"] = out


# ---------------------------------------------------------------------------------------------- W3
def test_w3(seeds, jobs):
    """W3. Do no harm: perfect odometry, noisy sensors: median end error at most 0.3 in above odometry alone."""
    fill_off("perfect", ROUTE, seeds, jobs)
    o = median([OFF[("perfect", ROUTE, s)][0]["odom"]["end"] for s in seeds])
    out = {"median_odom_end_in": o}
    for sensors in (1, 2):
        on = parallel(lambda s: bench(on_args(s, "perfect", sensors), test="W3"), seeds, jobs)
        f = median([x["fused"]["end"] for x in on])
        worst = max(x["fused"]["end"] for x in on)
        out[f"{sensors}"] = {"median_fused_end_in": f, "max_fused_end_in": worst,
                             "faults": sorted({x["filter"]["fault_name"] for x in on})}
        row("W3", f"perfect odometry, noisy sensors, {sensors} sensor{'s' if sensors > 1 else ''}, {len(seeds)} seeds: "
                  "median fused end error minus odometry alone's", f"{f - o:+.2f} in ({f:.2f} vs {o:.2f}; worst run "
                                                                   f"{worst:.2f})", "<= +0.3 in", f - o <= 0.3)
        # reported: the same measure on seeds 1 to 20 only
        first = [s for s in seeds if s <= 20]
        f20 = median([x["fused"]["end"] for s, x in zip(seeds, on) if s <= 20])
        o20 = median([OFF[("perfect", ROUTE, s)][0]["odom"]["end"] for s in first])
        w20 = sum(x["fused"]["end"] > OFF[("perfect", ROUTE, s)][0]["odom"]["end"] for s, x in zip(seeds, on) if s <= 20)
        out[f"{sensors}"].update({"seeds_1_20_median_fused_end_in": f20, "seeds_1_20_median_odom_end_in": o20,
                                  "seeds_1_20_end_worse": int(w20)})
        row("W3", f"  ... the same, {sensors} sensor{'s' if sensors > 1 else ''}, seeds 1 to 20 only",
            f"{f20 - o20:+.2f} in ({f20:.2f} vs {o20:.2f}); {w20} of {len(first)} seeds end worse than odometry alone",
            "(info)", None)
    for model, name in CONE_MODELS.items():
        if model == 2:
            continue
        on = parallel(lambda s: bench(on_args(s, "perfect", 2, dist_extra=dist(f"cone_model={model}")), test="W3"),
                      seeds, jobs)
        f = median([x["fused"]["end"] for x in on])
        out[f"cone {model}"] = {"median_fused_end_in": f}
        row("W3", f"  ... two sensors, the model's cone read as the {name} instead of the blend", f"{f - o:+.2f} in",
            "(info)", None)
    on = parallel(lambda s: bench(on_args(s, "perfect", 2, noisy=False), test="W3"), seeds, jobs)
    f = median([x["fused"]["end"] for x in on])
    out["noise-free"] = {"median_fused_end_in": f}
    row("W3", "  ... two noise-free sensors (no noise, no calibration left over)", f"{f - o:+.2f} in", "(info)", None)
    RESULTS["W3"] = out


# ---------------------------------------------------------------------------------------------- W4
def error_added(summaries):
    """Over every fix of these runs: the largest error added (|after| - |before| on its axis) and the largest
    |implied error|."""
    added, implied, n = -math.inf, 0.0, 0
    for summ in summaries:
        for _, f in fixes_of(summ):
            added = max(added, abs(f["err_after"]) - abs(f["err_before"]))
            implied = max(implied, abs(f["implied_err"]))
            n += 1
    return (added if n else float("nan")), (implied if n else float("nan")), n


def test_w4(seeds, jobs):
    """W4 (Revision A). An object in the beam at every stop: 3 in or more in front of the wall, no fix and the pose
    equals odometry alone; 1 to 3 in in front, report the error added per fix and at the end."""
    out = {}
    for errors in ("a3b", "a3a"):
        fill_off(errors, ROUTE, seeds, jobs)
        for label, keys, judged in W4_VARIANTS:
            for late in (False, True):
                extra = dist(*keys) + (dist(f"stop_obj_from_ms={W4_LATE_FROM_MS}") if late else [])
                on = parallel(lambda s: bench(on_args(s, errors, 2, dist_extra=extra), test="W4"), seeds, jobs)
                fixes = sum(sum(p["fixes"] for p in x["walls"]["per_sensor"]) for x in on)
                fed = sum(x["filter"]["max_fed_in"] > 0 for x in on)
                # fixes that came from a reading of the object (any reading the median was taken from), and fixes
                # taken once the objects are there (route time; the route starts 10 ms into world time)
                late_route_ms = W4_LATE_FROM_MS - on[0]["walls"]["route_start_world_ms"]
                object_fixes = sum(1 for x in on for _, f in fixes_of(x) if f["states_mask"] & (1 << 5))
                fixes_after = sum(1 for x in on for _, f in fixes_of(x) if f["t_ms"] >= late_route_ms)
                start_faults = sum(p["state"] == "START" for x in on for p in x["walls"]["per_sensor"])
                blocked = sum(p["dropped_by_check"][8] for x in on for p in x["walls"]["per_sensor"])
                objects = sum(p["sampled_by_model_state"][5] for x in on for p in x["walls"]["per_sensor"])
                added, implied, n = error_added(on)
                f = [x["fused"]["end"] for x in on]
                o = [OFF[(errors, ROUTE, s)][0]["odom"]["end"] for s in seeds]
                key = f"{errors} {label} {'from the 2nd stop' if late else 'from the start'}"
                out[key] = {"fixes": fixes, "runs_fed_in": fed, "start_faults": start_faults,
                            "fixes_from_object_readings": object_fixes, "fixes_after_objects_appear": fixes_after,
                            "dropped_blocked": blocked, "readings_of_the_object": objects,
                            "max_error_added_in": added, "max_implied_err_in": implied,
                            "median_fused_end_in": median(f), "median_odom_end_in": median(o),
                            "worst_paired_end_difference_in": max(a - b for a, b in zip(f, o))}
                when = "objects from the 2nd stop (start check on a clean wall)" if late else "objects from the start"
                what = f"{SHORT[errors]}, two sensors, object {label}, {when}, {len(seeds)} seeds"
                # Revision B: on an uncalibrated robot an object can hide inside the odometry drift, and On needs a
                # calibrated robot, so those rows are reported, not limited
                why = (" (reported: uncalibrated, an object can hide inside the odometry drift; On needs a calibrated "
                       "robot)") if errors == "a3a" else ""
                if judged and not late:
                    row("W4", what + ": fixes accepted; runs with anything fed into the pose" + why,
                        f"{fixes} fixes; {fed} runs (start faults {start_faults}, blocked drops {blocked})",
                        "0; 0 (pose = odometry alone)", None if why else (fixes == 0 and fed == 0))
                elif judged:
                    # the first stop's start check and the fix after it are on a clean wall and correct; the test is
                    # then that no reading of the object ever fixes (the pose is odometry alone plus that first fix)
                    row("W4", what + ": fixes from a reading of the object (fixes once the objects are there)" + why,
                        f"{object_fixes} ({fixes_after}; all fixes {fixes}; blocked drops {blocked})", "0",
                        None if why else object_fixes == 0)
                else:
                    row("W4", what + ": error added per fix (largest), and at the end (median fused - odometry "
                                     "alone; worst run)",
                        f"{added:+.2f} in over {n} fixes (implied {implied:.2f}); end {median(f) - median(o):+.2f} in, "
                        f"worst {out[key]['worst_paired_end_difference_in']:+.2f}", f"(report; expected < "
                                                                                    f"{CFG['blockedMaxIn']} in)", None)
    RESULTS["W4"] = out


def investigate_w4(seeds, jobs):
    """W4 failed: which fixes got through, and what changes when one thing is changed."""
    out = {}
    for errors in ("a3b", "a3a"):
        for label, keys, judged in W4_VARIANTS:
            if not judged:
                continue
            for late in (False, True):
                key = f"{errors} {label} {'from the 2nd stop' if late else 'from the start'}"
                if RESULTS["W4"][key]["fixes_from_object_readings"] == 0 and (late or RESULTS["W4"][key]["fixes"] == 0):
                    continue
                extra = dist(*keys) + (dist(f"stop_obj_from_ms={W4_LATE_FROM_MS}") if late else [])
                on = parallel(lambda s: bench(on_args(s, errors, 2, dist_extra=extra), test="W4?"), seeds, jobs)
                bad = []
                for x in on:
                    for i, f in fixes_of(x):
                        bad.append((x["seed"], i, stop_of(ROUTE, f["step"]), f["model_state"], f["states_mask"],
                                    f["err_before"], f["err_after"], f["implied_err"], f["inn"]))
                not_clean = [b for b in bad if b[4] & ~CLEAN_BITS]
                drift = [abs(b[5]) for b in not_clean]
                added = [abs(b[6]) - abs(b[5]) for b in not_clean]
                out[key] = {"fixes": [dict(zip(("seed", "sensor", "stop", "model_state", "states_mask", "err_before",
                                                "err_after", "implied_err", "inn"), b)) for b in bad]}
                row("W4?", f"{SHORT[errors]}, object {label}, {'2nd stop on' if late else 'from the start'}: fixes "
                           "with the object in the readings; the pose error on that axis just before each",
                    f"{len(not_clean)} of {len(bad)}; " + (f"{min(drift):.2f} to {max(drift):.2f} in; error added by "
                                                            f"them up to {max(added):+.2f} in" if drift else "-"),
                    "(info)", None)
                # one thing changed: the same objects with perfect odometry (no drift for an object to hide in)
                onp = parallel(lambda s: bench(on_args(s, "perfect", 2, dist_extra=extra), test="W4?"), seeds, jobs)
                fp = sum(1 for x in onp for _, f in fixes_of(x) if f["states_mask"] & (1 << 5))
                row("W4?", "  ... the same objects with perfect odometry: fixes from a reading of the object",
                    f"{fp}", "(info)", None)
    RESULTS["W4_investigation"] = out


# ---------------------------------------------------------------------------------------------- W5
def clear_walls(*keys):
    out = []
    for w in range(4):
        out += dist(*[f"wall{w}_{k}" for k in keys])
    return out


def test_w5(seeds, jobs):
    """W5. Clear walls: the beam reads through to things 2 to 40 in behind, nothing beyond 28 in. No single error
    added above 1.5 in; the health rule switches the sensor off; end error at most 1.0 in above odometry alone."""
    out = {}
    variants = [("as the model reads a clear wall (confidence 10 lower)", W5_CLEAR),
                ("with no confidence drop (so check 2 does not do the work)", W5_CLEAR + ["conf_drop=0"])]
    for errors in ("a3b", "a3a"):
        fill_off(errors, ROUTE, seeds, jobs)
        for label, keys in variants:
            for sensors in (1, 2):
                on = parallel(lambda s: bench(on_args(s, errors, sensors, dist_extra=clear_walls(*keys)), test="W5"),
                              seeds, jobs)
                added, implied, n = error_added(on)
                states = {}
                for x in on:
                    for p in x["walls"]["per_sensor"]:
                        states[p["state"]] = states.get(p["state"], 0) + 1
                health = states.get("HEALTH", 0)
                f = median([x["fused"]["end"] for x in on])
                o = median([OFF[(errors, ROUTE, s)][0]["odom"]["end"] for s in seeds])
                key = f"{errors} {sensors} {label}"
                out[key] = {"max_error_added_in": added, "max_implied_err_in": implied, "fixes": n,
                            "sensor_states": states, "median_fused_end_in": f, "median_odom_end_in": o,
                            "health_off_ms": [p["health_off_ms"] for x in on for p in x["walls"]["per_sensor"]]}
                what = f"{SHORT[errors]}, {sensors} sensor{'s' if sensors > 1 else ''}, clear walls {label}"
                row("W5", what + f", {len(seeds)} seeds: largest error added by one fix",
                    f"{added:+.2f} in over {n} fixes" if n else "no fix", f"<= {CFG['capClearIn']} in",
                    n == 0 or added <= CFG["capClearIn"])
                row("W5", "  ... sensors switched off by the health rule (end states)",
                    f"{health} of {len(on) * sensors} ({', '.join(f'{k} {v}' for k, v in sorted(states.items()))})",
                    "all", health == len(on) * sensors)
                row("W5", "  ... median end error minus odometry alone's", f"{f - o:+.2f} in ({f:.2f} vs {o:.2f})",
                    "<= +1.0 in", f - o <= 1.0)
    # the harder cases, reported: part-time read-through, and a steady thing right behind the panel
    for label, keys in (("through half the time", ["clear=1", "through_prob=0.5", "behind_in=2",
                                                    "behind_spread_in=38", "dropout_in=28", "conf_drop=0"]),
                        ("always through to something steady 2 in behind", ["clear=1", "through_prob=1", "behind_in=2",
                                                                           "conf_drop=0"]),
                        ("always through to something steady 12 in behind", ["clear=1", "through_prob=1",
                                                                            "behind_in=12", "conf_drop=0"])):
        on = parallel(lambda s: bench(on_args(s, "a3b", 1, dist_extra=clear_walls(*keys)), test="W5"), seeds, jobs)
        added, implied, n = error_added(on)
        states = {}
        for x in on:
            for p in x["walls"]["per_sensor"]:
                states[p["state"]] = states.get(p["state"], 0) + 1
        f = median([x["fused"]["end"] for x in on])
        o = median([OFF[("a3b", ROUTE, s)][0]["odom"]["end"] for s in seeds])
        out[f"info {label}"] = {"max_error_added_in": added, "fixes": n, "sensor_states": states,
                                "median_fused_end_in": f, "median_odom_end_in": o}
        row("W5", f"  calibrated, 1 sensor, {label} (no confidence drop): largest error added; end states; end vs "
                  "odometry alone", (f"{added:+.2f} in over {n} fixes" if n else "no fix") +
            f"; {', '.join(f'{k} {v}' for k, v in sorted(states.items()))}; {f - o:+.2f} in", "(info)", None)
    RESULTS["W5"] = out


# ---------------------------------------------------------------------------------------------- W6
def heading_sign(seed):
    return 1 if np.random.RandomState(50_000 + seed).uniform() < 0.5 else -1


def test_w6(seeds, jobs):
    """W6. Heading wrong by 2 degrees at a fix: error added at most 0.2 in. The robot is told a start heading 2 deg
    off (sign per seed), so every fix of the run is taken 2 deg (plus the calibrated IMU's own error) off square."""
    out = {}
    for noisy in (False, True):
        on = parallel(lambda s: bench(on_args(s, "a3b", 2, noisy=noisy,
                                              extra=["--start-error", f"0:0:{2 * heading_sign(s)}"]), test="W6"),
                      seeds, jobs)
        added, implied, n = error_added(on)
        worst_heading = 0.0
        for x in on:
            for _, f in fixes_of(x):
                # the robot's true heading against square to the wall it read (the stops are planned square)
                worst_heading = max(worst_heading, abs(((f["true_deg"] + 45) % 90) - 45))
        out["noisy" if noisy else "noise-free"] = {"max_implied_err_in": implied, "max_error_added_in": added,
                                                   "fixes": n, "largest_true_off_square_deg": worst_heading}
        if not noisy:
            row("W6", f"calibrated, two noise-free sensors, start heading told 2 deg wrong, {len(seeds)} seeds: largest "
                      "|implied error| of a fix (what it adds at full weight)",
                f"{implied:.3f} in over {n} fixes (true heading up to {worst_heading:.1f} deg off square)", "<= 0.2 in",
                implied <= 0.2)
            row("W6", "  ... largest error added by one fix (|after| - |before|)", f"{added:+.3f} in", "<= 0.2 in",
                added <= 0.2)
        else:
            row("W6", "  ... the same with the noisy sensors: largest |implied error|; largest error added",
                f"{implied:.2f} in; {added:+.2f} in over {n} fixes", "(info: noise included)", None)
    RESULTS["W6"] = out


# ---------------------------------------------------------------------------------------------- W7
def corner_args():
    """The corner route's elements, in the model and as the filter's keep-outs."""
    out = []
    for k, (wall, a, b, depth) in enumerate(W7_ELEMENTS):
        out += dist(f"elem{k}_wall={wall}", f"elem{k}_from_in={a}", f"elem{k}_to_in={b}", f"elem{k}_depth_in={depth}")
        out += ["--keep-out", f"{wall}:{a}:{b}"]
    return out


def read_elements(path):
    found = []
    with open(path) as f:
        for line in f:
            if line.startswith("#@element"):
                w, a, b, d = line.split()[1:5]
                found.append((int(w), float(a), float(b), float(d)))
    return found


def cone_bad(mask):
    """A cone mask meets a corner (two walls) or an element."""
    return bin(mask & 15).count("1") >= 2 or (mask & 16) != 0


def test_w7(seeds, jobs):
    """W7. Stops near corners and keep-outs (walls_corner.txt): no fix with the true cone across a corner or a
    keep-out. Each fix is judged from the true pose, with the model's own cone and with the filter's 18 degrees."""
    assert read_elements(CORNER_ROUTE) == [tuple(float(v) if i else v for i, v in enumerate(e)) for e in W7_ELEMENTS]
    out = {}
    for errors in ("a3b", "a3a"):
        for model, name in CONE_MODELS.items():
            on = parallel(lambda s: bench(on_args(s, errors, 2, dist_extra=dist(f"cone_model={model}"),
                                                  extra=corner_args()), route=CORNER_ROUTE, test="W7"), seeds, jobs)
            fixes = [(x["seed"], i, f) for x in on for i, f in fixes_of(x)]
            bad_model = [(s, i, stop_of(CORNER_ROUTE, f["step"])) for s, i, f in fixes if cone_bad(f["cone_mask_model"])]
            bad18 = [(s, i, stop_of(CORNER_ROUTE, f["step"])) for s, i, f in fixes if cone_bad(f["cone_mask_18"])]
            not_clean = [f for _, _, f in fixes if f["states_mask"] & ~CLEAN_BITS]
            by_stop, per_run = fixes_by_stop(CORNER_ROUTE, on)
            key = f"{errors} {name}"
            out[key] = {"fixes": len(fixes), "bad_model_cone": bad_model, "bad_18_cone": bad18,
                        "fixes_not_clean": len(not_clean), "by_stop": by_stop}
            stops = " ".join(f"{k}:{v[0]}+{v[1]}" for k, v in by_stop.items())
            row("W7", f"{SHORT[errors]}, two sensors, cone read as the {name}, {len(seeds)} seeds: fixes whose true cone "
                      "(the model's angle) met a corner or an element", f"{len(bad_model)} of {len(fixes)}", "0",
                len(bad_model) == 0)
            row("W7", "  ... judged with an 18 deg cone from the true pose; fixes with any reading not of the wall; "
                      "fixes by stop (back+right)", f"{len(bad18)}; {len(not_clean)}; {stops}", "(info)", None)
    # the model's cone wider than the filter's 18 degrees: the filter's margins against a cone it does not assume
    for half in (19, 24):
        on = parallel(lambda s: bench(on_args(s, "a3b", 2, dist_extra=dist(f"cone_half_deg={half}",
                                                                          f"cone_half_close_deg={half}"),
                                              extra=corner_args()), route=CORNER_ROUTE, test="W7"), seeds, jobs)
        fixes = [(x["seed"], i, f) for x in on for i, f in fixes_of(x)]
        bad = [f for _, _, f in fixes if cone_bad(f["cone_mask_model"])]
        not_clean = [f for _, _, f in fixes if f["states_mask"] & ~CLEAN_BITS]
        worst = max([abs(f["implied_err"]) for f in not_clean], default=0.0)
        out[f"a3b cone {half}"] = {"fixes": len(fixes), "bad_model_cone": len(bad), "fixes_not_clean": len(not_clean),
                                   "worst_implied_err_not_clean_in": worst}
        row("W7", f"  calibrated, the true cone {half} deg wide (the filter assumes 18): fixes whose cone met a corner "
                  "or an element; of them with a reading pulled off the wall, the worst implied error",
            f"{len(bad)} of {len(fixes)}; {len(not_clean)}, {worst:.2f} in", "(info)", None)
    RESULTS["W7"] = out


# ---------------------------------------------------------------------------------------------- W8
def test_w8(seeds, jobs):
    """W8 (Revision B). A sensor's start check sees only the axis of the first wall it looks at, so: with a 5 in
    start error, no fix makes the wrong axis worse, and a sensor faults when the error is on the axis it sees first.
    The axis a sensor's start check used is read from the CSV (wN_axis on the tick its wN_fate is 14 or 15)."""
    out = {}
    tol = CFG["wallStartTolIn"]
    directions = [("along x", lambda s: (W8_ERROR_IN * heading_sign(s), 0.0)),
                  ("along y", lambda s: (0.0, W8_ERROR_IN * heading_sign(s))),
                  ("in a direction drawn per seed", lambda s: gs.per_seed_direction(s, 60_000, W8_ERROR_IN))]
    for sensors in (1, 2):
        cols = ["t_ms"] + [f"w{i}_{c}" for i in range(sensors) for c in ("fate", "axis")]
        for label, fn in directions:
            def one(seed):
                dx, dy = fn(seed)
                summary, c = bench(on_args(seed, "a3b", sensors, extra=["--start-error", "%.4f:%.4f" % (dx, dy)]),
                                   cols=cols, test="W8")
                return seed, (dx, dy), summary, c
            res = parallel(one, seeds, jobs)
            must_fault = faulted = must_pass = passed = other = 0
            worse, wrong_axis_fixes = 0, 0
            worst_added = 0.0
            for seed, err, x, c in res:
                for i, p in enumerate(x["walls"]["per_sensor"]):
                    k = np.nonzero(np.isin(c[f"w{i}_fate"], (14, 15)))[0]
                    if len(k):
                        axis = int(c[f"w{i}_axis"][k[0]])
                        size = abs(err[axis])
                        fault = p["state"] == "START"
                        if size > tol + W8_MARGIN_IN:
                            must_fault += 1
                            faulted += fault
                        elif size < tol - W8_MARGIN_IN:
                            must_pass += 1
                            passed += not fault
                        else:
                            other += 1
                    # every fix on the axis that was set wrong (start error past the start check's tolerance) must
                    # not leave that axis worse. (A per-seed direction also puts up to about 1.3 in on the other
                    # axis; fixes there are ordinary corrections, where sensor noise moves a sub-inch error either
                    # way, as W3 measures)
                    for f in p["fix_list"]:
                        if abs(err[f["axis"]]) > tol:
                            wrong_axis_fixes += 1
                            added = abs(f["err_after"]) - abs(f["err_before"])
                            worst_added = max(worst_added, added)
                            worse += added > 0
            key = f"{sensors} {label}"
            out[key] = {"must_fault": must_fault, "faulted": faulted, "must_pass": must_pass, "passed": passed,
                        "near_tolerance": other, "fixes_on_wrong_axis": wrong_axis_fixes, "made_worse": worse,
                        "largest_added_in": worst_added,
                        "median_fused_end_in": median([r[2]["fused"]["end"] for r in res])}
            what = f"calibrated, {sensors} sensor{'s' if sensors > 1 else ''}, start pose {W8_ERROR_IN:g} in wrong {label}, " \
                   f"{len(seeds)} seeds"
            row("W8", what + f": fixes on the axis set wrong (start error over {tol:g} in) that made it worse (of "
                             "such fixes; largest added)",
                f"{worse} of {wrong_axis_fixes}; {worst_added:+.3f} in", "0", worse == 0)
            row("W8", f"  ... sensors whose first wall is on an axis with more than {tol + W8_MARGIN_IN:g} in of error: "
                      f"faulted (sensors on an axis with under {tol - W8_MARGIN_IN:g} in: passed; near the tolerance)",
                f"{faulted}/{must_fault} ({passed}/{must_pass}; {other})", f"{must_fault}/{must_fault}",
                faulted == must_fault)
    RESULTS["W8"] = out


def investigate_w8(seeds, jobs):
    """W8 failed: the start check of a sensor looks only along the axis of the first wall it sees."""
    out = {}
    for sensors in (1, 2):
        on = parallel(lambda s: bench(on_args(s, "a3b", sensors, extra=["--start-error", f"0:{5 * heading_sign(s)}"]),
                                      test="W8?"), seeds, jobs)
        first_axis = {}
        drops = np.zeros(13, int)
        x_fix = y_fix = 0
        for x in on:
            for i, p in enumerate(x["walls"]["per_sensor"]):
                first_axis.setdefault(i, {}).setdefault(p["state"], 0)
                first_axis[i][p["state"]] += 1
                drops += np.array(p["dropped_by_check"])
                for f in p["fix_list"]:
                    if f["axis"] == 0:
                        x_fix += 1
                    else:
                        y_fix += 1
        out[sensors] = {"end_states_by_sensor": first_axis, "drops": drops.tolist(), "x_fixes": x_fix, "y_fixes": y_fix}
        st = "; ".join(f"sensor {i}: " + ", ".join(f"{k} {v}" for k, v in sorted(d.items()))
                       for i, d in sorted(first_axis.items()))
        row("W8?", f"{sensors} sensor{'s' if sensors > 1 else ''}, 5 in along y: end states ({st}); fixes on x / on y; "
                   "readings dropped by wall checks 9 / 10 / 11",
            f"{x_fix} / {y_fix}; {drops[8]} / {drops[9]} / {drops[10]}", "(info)", None)
    RESULTS["W8_investigation"] = out


# ---------------------------------------------------------------------------------------------- W9
def test_w9(seeds_std, jobs):
    """W9, measured the way gps_suite.py's A5 is: (ii) standard moves after a fix, judged on the real robot against
    Off; (iii) the tick-to-tick change of the motor command during moves, On against Off, pooled over W2's runs.
    Part (i), the largest pose change in one tick, is judged over every On run at the end (finish_w9)."""
    out = {}
    for errors in ("a3b", "a3a"):
        on_steps, off_steps, per_run, compare = [], [], [], []
        for sensors in (1, 2):
            for s, ((off, offc), (on, onc)) in RUNS["w2"][(errors, sensors)].items():
                a, b = gs.cmd_steps(onc), gs.cmd_steps(offc)
                on_steps.append(a)
                off_steps.append(b)
                per_run.append(float(np.percentile(a, 99) - np.percentile(b, 99)))
                compare.append(gs.cmd_compare(onc, offc))
        on_all, off_all = np.concatenate(on_steps), np.concatenate(off_steps)
        p_on, p_off = float(np.percentile(on_all, 99)), float(np.percentile(off_all, 99))
        out[f"command {errors}"] = {"p99_on": p_on, "p99_off": p_off, "worst_per_run_p99_difference": max(per_run),
                                    "max_on": int(on_all.max()), "max_off": int(off_all.max()),
                                    "start_from_corrected_pose_max_dcmd": max(compare),
                                    "start_from_corrected_pose_median_dcmd": median(compare)}
        row("W9", f"{SHORT[errors]}: tick-to-tick motor command change in moves, 99th percentile, On vs Off (W2's "
                  f"{len(per_run)} runs pooled)", f"{p_on:.0f} vs {p_off:.0f} of 127 ({p_on - p_off:+.0f})", "<= +5",
            p_on - p_off <= 5)
        row("W9", "  ... per run: largest difference of the two 99th percentiles; a move starting from a corrected "
                  "pose steers differently (vs Off while the robots coincide)",
            f"{max(per_run):+.0f}; max {max(compare):.0f}, median {median(compare):.0f} of 127", "(reported)", None)

    def one(job):
        move, seed, errors = job
        res = []
        for mode in ("off", "on"):
            args = gs.ERRORS[errors](seed) + ["--no-gps"] + wall_args(seed, 2) + ["--seed", seed, "--fusion", mode]
            summ = bench(args, route=os.path.join(SIM, "routes", move + ".txt"), test="W9 std")
            res.append(summ)
        moves = [[x for x in r["step_list"] if x["kind"] != "wait"][-1] for r in res]
        fixed = sum(1 for _, f in fixes_of(res[1]) if f["t_ms"] < moves[1]["start_ms"])
        return move, seed, errors, moves[0], moves[1], fixed
    results = parallel(one, [(m, s, e) for e in ("a3b", "a3a") for m in STD_MOVES for s in seeds_std], jobs)
    for errors in ("a3b", "a3a"):
        mine = [r for r in results if r[2] == errors]
        per_move = {}
        for move in STD_MOVES:
            mm = [r for r in mine if r[0] == move]
            per_move[move] = {"max_d_swing_true_in": max(abs(r[4]["swing_true"] - r[3]["swing_true"]) for r in mm),
                              "max_d_settle_true_ms": max(abs(r[4]["settle_true_ms"] - r[3]["settle_true_ms"])
                                                          for r in mm),
                              "runs_with_a_fix": sum(r[5] > 0 for r in mm)}
        out[f"moves {errors}"] = per_move
        sw = max(v["max_d_swing_true_in"] for v in per_move.values())
        st = max(v["max_d_settle_true_ms"] for v in per_move.values())
        worst_sw = max(per_move, key=lambda m: per_move[m]["max_d_swing_true_in"])
        worst_st = max(per_move, key=lambda m: per_move[m]["max_d_settle_true_ms"])
        fixed = sum(v["runs_with_a_fix"] for v in per_move.values())
        row("W9", f"{SHORT[errors]}: standard moves after a fix ({len(STD_MOVES)} x {len(seeds_std)} seeds, {fixed} with a "
                  "fix in the stop before the move): real robot's overshoot past its resting point vs Off",
            f"max change {sw:.2f} in ({worst_sw})", "<= 0.1 in", sw <= 0.1)
        row("W9", "  ... real robot's settle time (until it stays within 1 in / 1 deg of its resting point) vs Off",
            f"max change {st:.0f} ms ({worst_st})", "<= 50 ms (wall design)", st <= 50)
    RESULTS["W9"] = out


def finish_w9():
    """W9 part (i): the largest pose change in one tick from a correction, over every fusion-On run of the suite."""
    limit = CFG["slewInPerS"] * ODOM_TICK_S
    judged = list(ON_RUNS)
    worst = sorted(judged, key=lambda r: (-r[2], r[0], r[1]))[0]
    at_limit = sum(r[2] >= limit * 0.999 for r in judged)
    RESULTS["W9_step"] = {"on_runs": len(judged), "largest_in": worst[2], "test": worst[0], "seed": worst[1],
                          "runs_at_the_limit": at_limit, "limit_in": limit}
    row("W9", f"largest pose change in one tick from a correction, all {len(judged)} On runs of this suite",
        f"{worst[2]:.5f} in ({at_limit} runs reach it)", f"<= slewInPerS*dt = {limit:.3f} in", worst[2] <= limit * 1.001)


# ---------------------------------------------------------------------------------------------- W10
def test_w10(seeds, jobs):
    """W10. GPS and walls together: end error no worse than the better of the two alone by more than 0.3 in."""
    out = {}
    for errors in ("a3b", "a3a"):
        gps = parallel(lambda s: bench(gs.ERRORS[errors](s) + gs.NOISE + ["--seed", s, "--fusion", "on"],
                                       test="W10 gps"), seeds, jobs)
        both_cols = parallel(lambda s: bench(on_args(s, errors, 1, gps=True), cols=COLS, test="W10 both"), seeds,
                             jobs)
        both = [x[0] for x in both_cols]
        walls = [RUNS["w2"][(errors, 1)][s][1][0] for s in seeds]
        # the error at the end of the last stop: what the walls left, and what the GPS made of it on the way out
        last_stop = max(n for n, name in STOPS[ROUTE].items() if name != "end")

        def at_last(c):
            return float(c["err_fused"][np.nonzero(c["step"] == last_stop)[0][-1]])
        last_both = median([at_last(c) for _, c in both_cols])
        last_walls = median([at_last(RUNS["w2"][(errors, 1)][s][1][1]) for s in seeds])
        g, w, b = (median([x["fused"]["end"] for x in r]) for r in (gps, walls, both))
        out[errors] = {"median_gps_alone_in": g, "median_walls_alone_in": w, "median_both_in": b,
                       "faults_gps_alone": sorted({x["filter"]["fault_name"] for x in gps}),
                       "faults_both": sorted({x["filter"]["fault_name"] for x in both}),
                       "fault_runs_both": sum(x["filter"]["fault"] != 0 for x in both)}
        # reported, not judged: a median within 0.3 in of the better one hides seeds where adding the GPS to the
        # walls makes the end worse, so the row says how many
        worse = sum(x["fused"]["end"] > y["fused"]["end"] for x, y in zip(both, walls))
        out[errors]["seeds_both_end_worse_than_walls"] = int(worse)
        row("W10", f"{SHORT[errors]}, GPS (1.5 in) and the back sensor together, {len(seeds)} seeds: median end error "
                   "vs the better of the two alone (reported, not judged), and seeds where together ends worse than "
                   "walls alone", f"{b:.2f} in (GPS alone {g:.2f}, walls alone {w:.2f}; faults "
                                  f"{out[errors]['fault_runs_both']}); together worse than walls alone in {worse} of "
                                  f"{len(seeds)} seeds",
            f"(was <= {min(g, w) + 0.3:.2f} in, better + 0.3)", None)
        out[errors].update({"median_at_last_stop_both_in": last_both, "median_at_last_stop_walls_in": last_walls})
        row("W10", f"  ... the same runs at the end of the last stop ({STOPS[ROUTE][last_stop]}): together vs walls "
                   "alone", f"{last_both:.2f} vs {last_walls:.2f} in", "(info)", None)
    RESULTS["W10"] = out


# ---------------------------------------------------------------------------------------------- W11
def test_w11(seeds, jobs):
    """W11. A wall plane wrong by 0.5 in: report the bias on that axis (about 0.5 in expected); it must not grow."""
    out = {}
    true_plane = CFG["planeIn"] + W11_ERROR_IN
    extra = dist(f"wall{W11_WALL}_in={true_plane}")
    axis = 0 if W11_WALL in (1, 3) else 1
    for label, errors, noisy, ss in (("perfect odometry, noise-free sensors", "perfect", False, seeds[:1]),
                                     ("calibrated odometry, noisy sensors", "a3b", True, seeds)):
        on = parallel(lambda s: bench(on_args(s, errors, 2, noisy=noisy, dist_extra=extra), test="W11"), ss, jobs)
        seqs, ends = [], []
        for x in on:
            seq = [f["err_after"] for _, f in sorted(fixes_of(x), key=lambda t: t[1]["t_ms"]) if f["wall"] == W11_WALL]
            seqs.append(seq)
            # the end error on the axis that wall fixes
            ends.append((x["pose_end"][axis] - x["true_end"][axis]))
        n = min(len(q) for q in seqs)
        per_fix = [median([q[k] for q in seqs]) for k in range(n)]
        worst = max(abs(v) for q in seqs for v in q)
        out[label] = {"x_error_after_each_fix_at_that_wall_median_in": per_fix, "largest_in": worst,
                      "median_end_axis_error_in": median(ends), "fixes_at_that_wall": [len(q) for q in seqs]}
        row("W11", f"the -x wall truly {W11_ERROR_IN} in farther out than the filter is told, {label}, two sensors"
                   f"{'' if len(ss) == 1 else f', {len(ss)} seeds (medians)'}: x error after each fix at that wall",
            " ".join(f"{v:+.2f}" for v in per_fix) + f" in; at the end {median(ends):+.2f}", "(report: about 0.5)",
            None)
        if len(ss) == 1:
            row("W11", "  ... it must not grow: the largest x error after any fix at that wall",
                f"{worst:.3f} in", f"<= {W11_ERROR_IN * 1.1:.2f} in (the plane error + 10 %)",
                worst <= W11_ERROR_IN * 1.1)
        else:
            row("W11", "  ... the largest x error after any fix at that wall, any seed (noise included)",
                f"{worst:.2f} in", "(info)", None)
    RESULTS["W11"] = out


# ---------------------------------------------------------------------------------------------- W12
def test_w12(seeds, jobs):
    """W12 (Revision A). A GPS that faults at the start (no strips on the field) with wall sensors fitted: the walls
    still work; end error within 0.3 in of the walls-only result."""
    out = {}
    for errors in ("a3b", "a3a"):
        on = parallel(lambda s: bench(on_args(s, errors, 1, gps=True, extra=["--gps", "no_strips=2"]), test="W12"),
                      seeds, jobs)
        walls = [RUNS["w2"][(errors, 1)][s][1][0] for s in seeds]
        a, w = median([x["fused"]["end"] for x in on]), median([x["fused"]["end"] for x in walls])
        faults = {}
        for x in on:
            faults[x["filter"]["fault_name"]] = faults.get(x["filter"]["fault_name"], 0) + 1
        fault_ms = [x["filter"]["fault_ms"] for x in on if x["filter"]["fault_ms"] >= 0]
        fixes = median([sum(p["fixes"] for p in x["walls"]["per_sensor"]) for x in on])
        fixes_w = median([sum(p["fixes"] for p in x["walls"]["per_sensor"]) for x in walls])
        out[errors] = {"median_end_in": a, "median_walls_only_in": w, "faults": faults,
                       "fault_ms": [min(fault_ms, default=-1), max(fault_ms, default=-1)],
                       "median_fixes": fixes, "median_fixes_walls_only": fixes_w}
        row("W12", f"{SHORT[errors]}, GPS with no strips (it reports from the centre and faults), back sensor, "
                   f"{len(seeds)} seeds: median end error vs walls only",
            f"{a:.2f} vs {w:.2f} in ({a - w:+.2f}); faults: " + ", ".join(f"{k} {v}" for k, v in sorted(faults.items())) +
            (f" at {min(fault_ms)}-{max(fault_ms)} ms" if fault_ms else "") + f"; fixes {fixes:.0f} vs {fixes_w:.0f}",
            "within 0.3 in", abs(a - w) <= 0.3)
    RESULTS["W12"] = out


# ---------------------------------------------------------------------------------------------- W13
def test_w13(seeds, jobs):
    """W13 (Revision B). A plane far wrong is refused by the start check or by checks 9 and 10 long before the total
    builds up, so: the total limit works when it is reached (shown with the limit lowered, uncalibrated robot, true
    walls), and a plane wrong by 2 to 3 in on a Solid wall is reported (it gets through and biases that axis)."""
    out = {}
    for limit in W13_LOW_LIMITS_IN:
        on = parallel(lambda s: bench(on_args(s, "a3a", 1, extra=["--fz", f"maxTotalCorrectionIn={limit}"]),
                                      test="W13"), seeds, jobs)
        fl = [x["filter"] for x in on]
        hit = [f for f in fl if f["decided_over_limit_ms"] >= 0]
        wrong = [f for f in hit if not (f["fault_name"] == "TotalTooLarge" and f["fault_ms"] <= f["decided_over_limit_ms"])]
        total = [x for x in on if x["filter"]["fault_name"] == "TotalTooLarge"]
        stopped = sum(x["walls"]["walls_stopped"] for x in total)
        back = sum(x["filter"]["fed_end_in"] == 0 for x in total)
        out[f"limit {limit}"] = {"runs_hitting_limit": len(hit), "not_faulted_at_once": len(wrong),
                                 "total_too_large": len(total), "walls_stopped": stopped, "back_to_odometry": back,
                                 "faults": sorted({f["fault_name"] for f in fl})}
        row("W13", f"maxTotalCorrectionIn lowered to {limit:g} in, uncalibrated, true walls, back sensor, {len(seeds)} "
                   "seeds: runs that hit the limit faulted TotalTooLarge at once; of those, walls stopped and the pose "
                   "back on odometry alone at the end",
            f"{len(hit) - len(wrong)}/{len(hit)}; {stopped}/{len(total)}, {back}/{len(total)}",
            "all; all, all", len(hit) > 0 and not wrong and stopped == len(total) and back == len(total))
    for err in W13_PLANE_ERRORS_IN:
        on = parallel(lambda s: bench(on_args(s, "a3b", 1, extra=["--wall-trust", "all:solid", "--wall",
                                                                   f"planeIn0={CFG['planeIn'] + err}"]), test="W13"),
                      seeds, jobs)
        y_fix = [f for x in on for _, f in fixes_of(x) if f["wall"] == 0]
        decided = max(x["filter"]["max_decided_in"] for x in on)
        worst = max([abs(f["err_after"]) for f in y_fix], default=0.0)
        faults = sorted({x["filter"]["fault_name"] for x in on})
        out[f"plane {err:+d}"] = {"fixes_at_that_wall": len(y_fix), "largest_decided_in": decided,
                                  "largest_y_error_after_in": worst, "faults": faults}
        row("W13", f"  ... the +y wall's plane told {err:+d} in off, Solid, calibrated, back sensor: fixes at that wall; "
                   "largest y error after one; faults", f"{len(y_fix)}; {worst:.2f} in; {', '.join(faults)}",
            "report (measure the field)", None)
    RESULTS["W13"] = out


def investigate_w13(seeds, jobs):
    """W13 failed: show the wrong plane never gets past the gates, and that the stage-2 fault itself works when
    the total limit is reachable (maxTotalCorrectionIn lowered, an investigation only)."""
    out = {}
    for err in (2, 3, 4, 6):
        on = parallel(lambda s: bench(on_args(s, "a3b", 1, extra=["--wall-trust", "all:solid", "--wall",
                                                                   f"planeIn0={CFG['planeIn'] + err}"]), test="W13?"),
                      seeds, jobs)
        y_fix = [f for x in on for _, f in fixes_of(x) if f["wall"] == 0]
        decided = max(x["filter"]["max_decided_in"] for x in on)
        worst = max([abs(f["err_after"]) for f in y_fix], default=0.0)
        out[f"plane +{err}"] = {"fixes_at_that_wall": len(y_fix), "largest_decided_in": decided,
                                "largest_y_error_after_in": worst}
        row("W13?", f"the +y wall told {err:+d} in off: fixes at that wall; largest total decided; largest y error "
                    "after one", f"{len(y_fix)}; {decided:.2f} in; {worst:.2f} in", "(info)", None)
    for limit in (1.0, 2.0):
        on = parallel(lambda s: bench(on_args(s, "a3a", 1, extra=["--fz", f"maxTotalCorrectionIn={limit}"]),
                                      test="W13?"), seeds, jobs)
        n = sum(x["filter"]["fault_name"] == "TotalTooLarge" for x in on)
        stopped = sum(x["walls"]["walls_stopped"] for x in on)
        back = sum(x["filter"]["fed_end_in"] == 0 for x in on)
        unwind = [x["filter"]["fault_to_unwound_ms"] for x in on if x["filter"]["fault_ms"] >= 0]
        out[f"limit {limit}"] = {"total_too_large": n, "walls_stopped": stopped, "back_to_odometry": back,
                                 "fault_to_unwound_ms": unwind}
        row("W13?", f"uncalibrated, true walls, maxTotalCorrectionIn lowered to {limit:g} in (investigation only): "
                    "TotalTooLarge; walls stopped; back on odometry alone at the end; fault to unwound",
            f"{n}; {stopped}; {back} of {len(on)}; " + (f"{min(unwind)}-{max(unwind)} ms" if unwind else "-"),
            "(info)", None)
    RESULTS["W13_investigation"] = out


# ---------------------------------------------------------------------------------------------- sensor errors
def test_sensor_errors(seeds, jobs):
    """INFO rows the task asked for: a sensor not calibrated, a mount measured wrong. Calibrated robot, back sensor
    only (where the errors go straight into the reading), against W2's run with the right numbers."""
    out = {}
    base = [RUNS["w2"][("a3b", 1)][s][1][0] for s in seeds]
    b = median([x["fused"]["end"] for x in base])
    o = RESULTS["W2"]["a3b 1"]["median_odom_end_in"]
    _, bi, bn = error_added(base)
    cases = [("sensor reads 2 % long plus 0.3 in, not calibrated out", {"dist_extra": dist("s0_scale_err=0.02",
                                                                                          "s0_offset_mm=7.62")}),
             ("mount told 0.5 in too far back (forward -9.0, truly -8.5)", {"mounts": [(0.0, -9.0, 180.0)]}),
             ("mount told 0.5 in to the side (right 0.5, truly 0)", {"mounts": [(0.5, -8.5, 180.0)]}),
             ("mount yaw told 3 deg wrong (183, truly 180)", {"mounts": [(0.0, -8.5, 183.0)]})]
    for label, kw in cases:
        on = parallel(lambda s: bench(on_args(s, "a3b", 1, **kw), test="E"), seeds, jobs)
        f = median([x["fused"]["end"] for x in on])
        _, implied, n = error_added(on)
        imp = [f_["implied_err"] for x in on for _, f_ in fixes_of(x)]
        out[label] = {"median_fused_end_in": f, "median_implied_err_in": median(imp), "max_implied_err_in": implied,
                      "fixes": n, "faults": sorted({x["filter"]["fault_name"] for x in on})}
        row("E", f"calibrated, back sensor, {label}, {len(seeds)} seeds: median end error; implied error of the fixes "
                 "(median, largest)", f"{f:.2f} in (right numbers {b:.2f}, odometry alone {o:.2f}); "
                                      f"{median(imp):+.2f}, {implied:.2f} in over {n} fixes", "(info)", None)
    RESULTS["sensor_errors"] = out


# ---------------------------------------------------------------------------------------------- figures
# the GPS suite's style and palette (dataviz reference palette, light): blue, orange and aqua are the three slots
# that pass all-pairs (scatter); everything else is ink or grey
INK, INK_2, MUTED, GRID, AXIS, SURFACE = gs.INK, gs.INK_2, gs.MUTED, gs.GRID, gs.AXIS, gs.SURFACE
BLUE, ORANGE, AQUA, SHADE = gs.BLUE, gs.ORANGE, gs.AQUA, gs.SHADE


def stop_spans(summary):
    """(start s, end s) of every wait step of a run, route time."""
    return [(x["start_ms"] / 1000, x["end_ms"] / 1000) for x in summary["step_list"] if x["kind"] == "wait"]


def figures(seeds):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd
    plt.rcParams.update({"font.family": ["Helvetica", "Arial", "DejaVu Sans"], "axes.titlesize": 11,
                         "axes.labelsize": 10, "axes.labelcolor": INK_2, "text.color": INK})
    made = []

    def save(fig, name):
        path = os.path.join(OUT, name)
        fig.savefig(path, facecolor=SURFACE)
        plt.close(fig)
        made.append(path)

    # 1. top view: the uncalibrated robot (its drift is big enough to see at field scale), one back sensor, the seed
    # whose fused end error is the median of W2's runs
    runs = RUNS["w2"][("a3a", 1)]
    order = sorted(runs, key=lambda s: (runs[s][1][0]["fused"]["end"], s))
    seed = order[len(order) // 2]
    csv = tmp("top.csv")
    summ = bench(on_args(seed, "a3a", 1) + ["--csv", csv], test="figure")
    on = pd.read_csv(csv)
    fig, ax = plt.subplots(figsize=(7.2, 8.7), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    gs.style_axes(ax)
    w = CFG["planeIn"]
    ax.plot([-w, w, w, -w, -w], [-w, -w, w, w, -w], color=INK_2, linewidth=2, solid_capstyle="butt", zorder=1)
    ax.plot(on.true_x, on.true_y, color=INK, linewidth=2, label="true path", zorder=3, solid_capstyle="round")
    ax.plot(on.odom_x, on.odom_y, color=ORANGE, linewidth=2, label="odometry alone (corrections taken out)", zorder=4,
            solid_capstyle="round")
    ax.plot(on.pose_x, on.pose_y, color=BLUE, linewidth=2, label="fused (what the motions read)", zorder=5,
            solid_capstyle="round")
    first = True
    for i, f in fixes_of(summ):
        # the beam the fix came from, from where the sensor truly was, to the wall
        r, fwd, yaw = SENSOR_SETS[1][i]
        h = math.radians(f["true_deg"])
        sx = f["true_x"] + r * math.cos(h) + fwd * math.sin(h)
        sy = f["true_y"] - r * math.sin(h) + fwd * math.cos(h)
        b = h + math.radians(yaw)
        ax.plot([sx, sx + f["clean_in"] * math.sin(b)], [sy, sy + f["clean_in"] * math.cos(b)], color=AQUA,
                linewidth=2, zorder=2, solid_capstyle="butt", label="a fix: the beam it read" if first else None)
        k = int(np.argmin(np.abs(on.t_ms.to_numpy() - f["t_ms"])))
        ax.scatter([on.pose_x[k]], [on.pose_y[k]], s=46, marker="D", color=SURFACE, edgecolors=INK, linewidths=1.5,
                   zorder=7, label="a fix: the fused pose then" if first else None)
        first = False
    last = on.iloc[-1]
    for x, y, c in ((last.true_x, last.true_y, INK), (last.odom_x, last.odom_y, ORANGE), (last.pose_x, last.pose_y, BLUE)):
        ax.scatter([x], [y], s=40, color=c, edgecolors=SURFACE, linewidths=2, zorder=6)
    ax.scatter([on.true_x.iloc[0]], [on.true_y.iloc[0]], s=40, color=INK, edgecolors=SURFACE, linewidths=2, zorder=6)
    ax.annotate("start", (on.true_x.iloc[0], on.true_y.iloc[0]), xytext=(8, -4), textcoords="offset points",
                fontsize=9, color=INK_2)
    # a short label on a patch of the surface colour, so the lines under it do not hide it; the numbers go in the
    # title (no free space on the field is sure to stay free from one seed to the next)
    ax.annotate("end", (last.true_x, last.true_y), xytext=(-24, -4), textcoords="offset points", fontsize=9,
                color=INK_2, bbox={"boxstyle": "round,pad=0.15", "fc": SURFACE, "ec": "none"}, zorder=8)
    ax.set_xlim(-76, 76)
    ax.set_ylim(-76, 76)
    ax.set_aspect("equal")
    ax.set_xlabel("x, in (toward the 90° wall)")
    ax.set_ylabel("y, in (toward the 0° wall)")
    nfix = len(fixes_of(summ))
    ax.set_title(f"Wall fixes on the 58 s wall route, uncalibrated odometry, back sensor, GPS off.\nSeed {seed}: the "
                 f"median fused end error of {len(runs)}, {nfix} fixes. At the end: odometry\nalone "
                 f"{last.err_odom:.1f} in off, fused {last.err_fused:.1f} in off", loc="left", color=INK)
    gs.legend(ax, loc="upper center", bbox_to_anchor=(0.5, -0.09), ncol=2, handlelength=1.6)
    fig.tight_layout()
    save(fig, "wall_route_top.png")

    # 2. position error against time: both robots, one and two sensors; median and the 10-90 % band over seeds
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 7.6), dpi=150, sharex=True)
    fig.patch.set_facecolor(SURFACE)
    spans = stop_spans(RUNS["w2"][("a3b", 1)][seeds[0]][0][0])
    for r_i, errors in enumerate(("a3b", "a3a")):
        top = 0
        for c_i, sensors in enumerate((1, 2)):
            ax = axes[r_i][c_i]
            gs.style_axes(ax)
            for a, b in spans:
                ax.axvspan(a, b, color=SHADE, linewidth=0, zorder=0)
            r = RUNS["w2"][(errors, sensors)]
            ends = []
            for which, column, color, name in ((0, "err_odom", ORANGE, "odometry alone (fusion Off)"),
                                               (1, "err_fused", BLUE, "fused (fusion On)")):
                series = [r[s][which][1][column] for s in sorted(r)]
                t = r[sorted(r)[0]][which][1]["t_ms"] / 1000
                med, lo, hi, n = gs.band(series)
                ax.fill_between(t[:n], lo, hi, color=color, alpha=0.13, linewidth=0)
                ax.plot(t[:n], med, color=color, linewidth=2, label=name)
                ends.append((t[n - 1], med[-1]))
                top = max(top, float(np.max(hi)))
            close = abs(ends[0][1] - ends[1][1]) < 0.25 * top / 4
            upper = 0 if ends[0][1] >= ends[1][1] else 1
            for i, (x, y) in enumerate(ends):
                dy = 0 if not close else (6 if i == upper else -6)
                ax.annotate(f"{y:.2f}", (x, y), xytext=(4, dy), textcoords="offset points", va="center", fontsize=9,
                            color=INK_2)
            ax.set_xlim(0, 64)
            ax.set_title(f"{ROBOTS[errors]}, {'back sensor' if sensors == 1 else 'back and right sensors'}", loc="left",
                         color=INK, fontsize=10)
            if r_i == 1:
                ax.set_xlabel("time since the route started, s")
        for c_i in (0, 1):
            axes[r_i][c_i].set_ylim(0, top * 1.08)
        axes[r_i][0].set_ylabel("position error, in")
    gs.legend(axes[0][0], loc="upper left")
    fig.suptitle(f"Position error on the wall route, GPS off, Clear walls, {len(seeds)} seeds each: median, band "
                 "10-90 % (shaded: the stops)", x=0.01, ha="left", color=INK, fontsize=12)
    fig.tight_layout()
    save(fig, "wall_error_time.png")

    # 3. W4 and W5: what the back sensor read against what the filter predicted, each reading it sampled at a stop
    cases = [("W4: an object 3-10 in in front of the wall at every stop from the second on",
              on_args(seeds[0], "a3b", 1, dist_extra=dist(*W4_VARIANTS[1][1], f"stop_obj_from_ms={W4_LATE_FROM_MS}"))),
             ("W5: clear walls, the beam reads through to something 2-40 in behind (no confidence drop)",
              on_args(seeds[0], "a3b", 1, dist_extra=clear_walls(*(W5_CLEAR + ["conf_drop=0"]))))]
    fig, axes = plt.subplots(2, 1, figsize=(12, 8.2), dpi=150, sharex=True)
    fig.patch.set_facecolor(SURFACE)
    for ax, (title, args) in zip(axes, cases):
        gs.style_axes(ax)
        path = unique("blocked") + ".csv"
        sm = bench(args + ["--csv", path], test="figure")
        d = pd.read_csv(path)
        for a, b in stop_spans(sm):
            ax.axvspan(a, b, color=SHADE, linewidth=0, zorder=0)
        # only the readings the filter sampled while the real robot was still (under 1 in/s and 3 deg/s over the
        # last 100 ms, the filter's own "still"): those are the ones a fix could come from
        moved = np.hypot(d.true_x.diff(10), d.true_y.diff(10)) / 0.1
        turned = ((d.true_heading.diff(10) + 180) % 360 - 180).abs() / 0.1
        still = (moved <= 1.0) & (turned <= 3.0)
        sampled = d[(d.w0_fate != 0) & still]
        t = sampled.t_ms / 1000
        diff = sampled.w0_read - sampled.w0_pnow
        ylo, yhi = -12, 46  # through readings are up to 40 in long
        invalid = sampled.w0_read.isna() | sampled.w0_pnow.isna()
        groups = [("refused as unsteady (check 4: the last 5 readings disagree)",
                   ~invalid & (sampled.w0_fate == 4), MUTED, "o", 10),
                  ("refused by another check (2 confidence, 3 moving, 5-8 geometry, 11-13)",
                   ~invalid & ~sampled.w0_fate.isin([4, 9, 10, 14, 15, 16]), MUTED, "^", 14),
                  ("refused as blocked (check 9)", sampled.w0_fate == 9, ORANGE, "o", 18),
                  ("refused as through (check 10)", sampled.w0_fate == 10, AQUA, "o", 18),
                  ("start check", sampled.w0_fate.isin([14, 15]), INK, "s", 30),
                  ("fix", sampled.w0_fate == 16, BLUE, "D", 40)]
        for name, sel, color, marker, size in groups:
            if sel.any():
                ax.scatter(t[sel], np.clip(diff[sel], ylo + 0.5, yhi - 0.5), s=size, marker=marker, color=color,
                           edgecolors=SURFACE, linewidths=0.6, zorder=4, label=name)
        if invalid.any():
            ax.scatter(t[invalid], np.full(int(invalid.sum()), yhi - 1.5), s=8, marker="|", color=MUTED, zorder=3,
                       label="no valid reading (9999, or beyond 2000 mm), drawn at the top")
        ax.axhline(0, color=AXIS, linewidth=1)
        # the widest the blocked and through gates (checks 9 and 10) can be; named in the legend, not on the plot,
        # where the readings would cover the words
        ax.axhline(-CFG["blockedMaxIn"], color=INK_2, linewidth=1, linestyle=(0, (4, 3)),
                   label=f"the widest gates: blocked {CFG['blockedMaxIn']} in short, through 4 in long")
        ax.axhline(4.0, color=INK_2, linewidth=1, linestyle=(0, (4, 3)))
        off = sm["walls"]["per_sensor"][0]["health_off_ms"]
        if off >= 0:
            ax.axvline(off / 1000, color=INK, linewidth=1.2)
            ax.annotate("health rule switches\nthe sensor off", (off / 1000, 36), xytext=(5, 0),
                        textcoords="offset points", fontsize=9, color=INK_2, va="center")
        ax.set_ylim(ylo, yhi)
        ax.set_xlim(0, 60)
        ax.set_ylabel("reading minus predicted range, in")
        ax.set_title(title + f" (calibrated, seed {seeds[0]}: {sm['walls']['per_sensor'][0]['fixes']} fixes)",
                     loc="left", color=INK, fontsize=10)
        gs.legend(ax, loc="upper left", bbox_to_anchor=(1.01, 1.0), markerscale=1.2)
    axes[1].set_xlabel("time since the route started, s (shaded: the stops)")
    fig.suptitle("What the back sensor read against what the filter predicted: every reading it sampled while the "
                 "robot stood still",
                 x=0.01, ha="left", color=INK, fontsize=12)
    fig.tight_layout()
    save(fig, "wall_blocked.png")

    os.makedirs(DOCS_SIM, exist_ok=True)
    for name in ("wall_route_top.png", "wall_error_time.png", "wall_blocked.png"):
        shutil.copy(os.path.join(OUT, name), os.path.join(DOCS_SIM, name))
    return made


# ---------------------------------------------------------------------------------------------- main
TESTS = ["W1", "W2", "W3", "W4", "W5", "W6", "W7", "W8", "W9", "W10", "W11", "W12", "W13", "E"]


def main():
    global TMP
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seeds", type=int, default=50, help="seeds for W2, W3, W10, W12 and the sensor-error rows (the "
                                                          "design says 50 for W2); W4 to W8, W11, W13 use 20, W9's "
                                                          "moves 10")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
    ap.add_argument("--no-figures", action="store_true")
    ap.add_argument("--keep", action="store_true", help="keep the scratch folder of CSVs and traces")
    ap.add_argument("--only", default="", help="comma-separated tests to run (development; W2 is run for W9, W10, "
                                               "W12, E and the figures when they need it)")
    args = ap.parse_args()
    for tb in (TB_LIVE, TB_PREGPS):
        if not os.path.exists(tb):
            sys.exit(f"{tb} is missing: run make -C tools/sim first")
    os.makedirs(OUT, exist_ok=True)
    TMP = tempfile.mkdtemp(prefix="wall_suite_", dir=os.path.join(SIM, "build"))
    for path in (ROUTE, CORNER_ROUTE):
        STOPS[path] = route_stops(path)
    seeds = list(range(1, args.seeds + 1))
    twenty = seeds[:20]
    ten = seeds[:10]
    only = set(t.strip() for t in args.only.split(",") if t.strip()) or set(TESTS)
    need_w2 = only & {"W2", "W9", "W10", "W12", "E"} or not args.no_figures
    try:
        if "W1" in only:
            print("W1 ...", flush=True)
            test_w1(args.jobs)
        if need_w2:
            print("W2 ...", flush=True)
            test_w2(seeds, args.jobs)
            if any(r[0] == "W2" and r[4] == "FAIL" for r in ROWS):
                print("W2 investigation ...", flush=True)
                investigate_w2(seeds, args.jobs)
        if "W3" in only:
            print("W3 ...", flush=True)
            test_w3(seeds, args.jobs)
        if "W4" in only:
            print("W4 ...", flush=True)
            test_w4(twenty, args.jobs)
            if any(r[0] == "W4" and r[4] == "FAIL" for r in ROWS):
                print("W4 investigation ...", flush=True)
                investigate_w4(twenty, args.jobs)
        if "W5" in only:
            print("W5 ...", flush=True)
            test_w5(twenty, args.jobs)
        if "W6" in only:
            print("W6 ...", flush=True)
            test_w6(twenty, args.jobs)
        if "W7" in only:
            print("W7 ...", flush=True)
            test_w7(twenty, args.jobs)
        if "W8" in only:
            print("W8 ...", flush=True)
            test_w8(twenty, args.jobs)
            if any(r[0] == "W8" and r[4] == "FAIL" for r in ROWS):
                print("W8 investigation ...", flush=True)
                investigate_w8(twenty, args.jobs)
        if "W9" in only:
            print("W9 ...", flush=True)
            test_w9(ten, args.jobs)
        if "W10" in only:
            print("W10 ...", flush=True)
            test_w10(seeds, args.jobs)
        if "W11" in only:
            print("W11 ...", flush=True)
            test_w11(twenty, args.jobs)
        if "W12" in only:
            print("W12 ...", flush=True)
            test_w12(seeds, args.jobs)
        if "W13" in only:
            print("W13 ...", flush=True)
            test_w13(twenty, args.jobs)
            if any(r[0] == "W13" and r[4] == "FAIL" for r in ROWS):
                print("W13 investigation ...", flush=True)
                investigate_w13(twenty, args.jobs)
        if "E" in only:
            print("sensor errors ...", flush=True)
            test_sensor_errors(seeds, args.jobs)
        made = [] if args.no_figures else figures(seeds)
        if "W9" in only:
            finish_w9()
    finally:
        if not args.keep:
            shutil.rmtree(TMP, ignore_errors=True)

    ROWS.sort(key=lambda r: ORDER.index(r[0].rstrip("?")))
    head = ("Test", "What", "Measured", "Limit", "Result")
    widths = [max(len(str(r[i])) for r in ROWS + [head]) for i in range(5)]
    widths[1] = min(widths[1], 110)
    widths[2] = min(widths[2], 90)

    def line(r):
        return "  ".join(str(r[i])[:widths[i]].ljust(widths[i]) for i in range(5)).rstrip()
    print()
    print(line(head))
    print("  ".join("-" * w for w in widths))
    for r in ROWS:
        print(line(r))
        # a row too long for its column goes on with the rest on the next lines, under the same column
        for i in (1, 2):
            rest = str(r[i])[widths[i]:]
            while rest:
                cells = [""] * 5
                cells[i] = rest[:widths[i]]
                print(line(tuple(cells)))
                rest = rest[widths[i]:]
    fails = [r for r in ROWS if r[4] == "FAIL"]
    print(f"\n{sum(r[4] == 'PASS' for r in ROWS)} PASS, {len(fails)} FAIL, {sum(r[4] == 'INFO' for r in ROWS)} INFO")
    RESULTS["table"] = [dict(zip(("test", "what", "measured", "limit", "result"), r)) for r in ROWS]
    RESULTS["settings"] = {"route": os.path.relpath(ROUTE, REPO), "corner_route": os.path.relpath(CORNER_ROUTE, REPO),
                           "seeds": args.seeds, "only": sorted(only), "sensors_told": {"back": BACK, "right": RIGHT},
                           "noisy_model": NOISY, "calibration_left_max": {"scale": CAL_SCALE_MAX,
                                                                          "offset_mm": CAL_OFFSET_MM_MAX},
                           "a3a": gs.A3A, "a3b": gs.A3B, "slip_per_in": gs.SLIP_PER_IN, "slip_max_in": gs.SLIP_MAX_IN,
                           "gps_noise_scale": gs.NOISE_SCALE, "w4_late_from_world_ms": W4_LATE_FROM_MS,
                           "w5_clear": W5_CLEAR, "w7_elements": W7_ELEMENTS, "w11": [W11_WALL, W11_ERROR_IN],
                           "w13_low_limits_in": W13_LOW_LIMITS_IN, "w13_plane_errors_in": W13_PLANE_ERRORS_IN, "w8_error_in": W8_ERROR_IN, "filter_numbers_used_for_limits": CFG,
                           "w1_script": W1_SCRIPT, "w1_golden": W1_GOLDEN}
    name = "wall_suite_results.json" if only == set(TESTS) else "wall_suite_results_partial.json"
    with open(os.path.join(OUT, name), "w") as f:
        json.dump(RESULTS, f, indent=1, default=float, sort_keys=False)
    print(f"results: {os.path.join(OUT, name)}")
    for path in made:
        print(f"figure: {path}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
