#!/opt/anaconda3/bin/python3
"""UMBmark square in the host simulator: what each odometry error does to the square, how much slip scatters it,
and how the side and speed change that.

    make -C tools/sim                                  # build the testbenches first
    /opt/anaconda3/bin/python3 tools/sim/umb_suite.py [--seeds 30] [--jobs 8] [--no-figures]

The robot drives a square under its own odometry and comes back to where it started. It believes it is back at
the start; a person measures where it really is. Real minus believed is the odometry error E (inches, in the
start frame: x to the robot's right, y forward) and H (degrees, clockwise +, both headings wrapped to +-180).
Five runs clockwise and five counter-clockwise on the robot. Here the real LemLib code (src/lemlib) drives the
same square (routes/umb_cw.txt, routes/umb_ccw.txt) against the simulated robot in bench/world.cpp, with one
error at a time, and the "person with a ruler" is the simulator's truth.

Parts (all printed as tables, every number saved in tools/sim/out/umb_results.json):
  a. Signatures: one error at a time on an otherwise perfect robot with no slip, both directions, beside the
     small-error pencil formula for the inertial rows.
  b. Slip: a perfect robot with random tracking-wheel slip at three levels, 30 seeds per direction for the
     spread (300 for where the mean goes).
  c. Sensitivity: side 24, 48, 72 in; speed 70 against 127; gps_suite.py's uncalibrated (A3a) and calibrated
     (A3b) robots with slip, five runs per direction per robot, and how well the inertial scale comes out.
  a2. A robot that spins about a point behind its tracking centre (robot-config.cpp's [ODOMCAL] note puts it
     5.8 in behind): alone, and with the inertial error. The heading is not affected; the position is.
  d. run_square(direction, seed, extra_args, spin_behind_in=0): one square, for other tools (fake ruler
     measurements).
  e. docs/sim/umb_signatures.png: where the robot really ends, relative to where it believes it is.

Choices made here, stated again in tools/sim/README.md:
  - The world's tracking wheels and the offsets LemLib is told are the robot's (src/robot-config.cpp: vertical
    0.6 in left, horizontal 2.23 in behind), not the bench's defaults (the old robot's 0.9 and 2.55). An offset
    "wrong by +0.5" means the wheel really sits 0.5 in further right (vertical) or further forward (horizontal)
    than LemLib is told.
  - The bench's turns are lemlib::Chassis::turnToHeading, not TeamChassis::turnToHeading. For this square the two
    are the same: every turn is about 90 deg, which is neither under SMALL_TURN_DEG (45) nor at least BIG_TURN_DEG
    (100), so the team's rules change nothing (the maxSpeed of 70 is under BIG_TURN_MAX_SPEED in any case).
  - The simulator's slip does not depend on speed, and it has no sideways skid (MU_LAT stays at its default), so
    speed 70 against 127 shows only the motions' own differences and the time a drift has to act, never the
    extra slip of a fast robot. The real robot will slip more at 127.
  - GPS off (--no-gps) and fusion Off: the square measures odometry alone.
  - The bench's robot spins about its drive centre, and its "true" pose is the drive centre. A tracking centre D in
    ahead of the drive centre (the robot spins about a point D behind it) is made by telling the world the
    horizontal wheel sits D further forward than LemLib is told, and by moving the ruler's reading from the drive
    centre to the tracking centre (run_square's spin_behind_in). The bench's own odometry-error figures (its
    summary) still compare against the drive centre: run_square works its numbers out from the ticks instead.
"""

import argparse
import atexit
import concurrent.futures
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import threading

import numpy as np

SIM = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(SIM))
TB = os.path.join(SIM, "build", "live", "tb_live")
ROUTE_FILES = {"CW": os.path.join(SIM, "routes", "umb_cw.txt"), "CCW": os.path.join(SIM, "routes", "umb_ccw.txt")}
OUT = os.path.join(SIM, "out")
DOCS_SIM = os.path.join(REPO, "docs", "sim")
FIGURE = "umb_signatures.png"

# ---- the square. The robot's routine, these routes and the calculator (tools/umbmark.py) must agree on these
UMB_SIDE_IN = 48
UMB_DRIVE_MAX_SPEED = 70  # out of 127: slow on purpose, so the calibration runs slip little
UMB_TURN_MAX_SPEED = 70
UMB_STOP_MS = 300  # after every move
UMB_STILL_MS = 2000  # standing still before the first move: the heading change over it gives still_dps
UMB_SETTLE_MS = 1000  # after the last turn's stop, before the pose is read
UMB_DRIVE_TIMEOUT_MS = 5000  # time limit of a leg (the routine ends the run at the first one; a route carries on)
UMB_TURN_TIMEOUT_MS = 3000  # ... of a turn

# ---- the robot's tracking wheels (src/robot-config.cpp, measured with a ruler on 2026-11-20). LemLib's signs:
# vertical + right, horizontal + front. The bench's own defaults are the old robot's 0.9 and 2.55
ROBOT_VERT_OFFSET_IN = -0.6
ROBOT_HORIZ_OFFSET_IN = -2.23

# ---- part a: the errors, one at a time
IMU_ERR = 0.012  # inertial scale error, 1.2 % (the size docs/GPS_FUSION_DESIGN.md assumes)
DRIFTS_DEG_S = [0.02, 0.05]  # steady heading drift
WHEEL_SIZE_ERR = 0.015  # a tracking wheel that reads 1.5 % long
OFFSET_ERR_IN = 0.5  # a tracking-wheel offset this far out
YAW_DEG = 2  # a tracking wheel this far off parallel
INVISIBLE_IN = 0.1  # an error whose |E| stays under this is one the closed square cannot see
NO_ERROR_ROWS = ("perfect", "imu_corrected")  # checks, not errors: their |E| must be about 0
NOTE_CLEARANCE_IN = 1.3  # the figure: a panel's note goes where no marker is closer than this to its middle

# ---- part b: slip levels, as in slip_sweep.py: (label, SLIP_PER_IN, SLIP_MAX_IN). The first is the suites' own
SLIP_LEVELS = [("0.45 %", 0.03, 0.3), ("1.5 %", 0.1, 0.3), ("5 %", 0.1, 1.0)]
SEEDS = 30
# 30 seeds put the mean only within about 0.1 in (its standard error at 0.45 % slip is 0.05 in per axis), and the
# first run of this suite read a 0.12 in shift there that 300 seeds showed to be mostly chance (0.03 in). So the
# mean's shift is measured again over this many seeds; the spread stays the 30 seeds' figure
SHIFT_SEEDS = 300

# ---- part c
SIDES_IN = [24, 48, 72]
SPEEDS = [70, 127]
SENS_DRIFT_DEG_S = 0.05
MIX_ROBOTS = 10  # robots per mix (the error signs are drawn per robot, as gps_suite.py does per seed)
RUNS_PER_DIRECTION = 5  # as on the real robot
MIX_SEED_BASE = 1000  # slip seed of run r of robot n: MIX_SEED_BASE * n + r

# Colours for the figure (a two-slot categorical palette, checked with the dataviz validator: CVD dE 24.7)
CW_COLOUR = "#2a78d6"
CCW_COLOUR = "#eb6834"
INK = "#0b0b0b"
INK_2 = "#52514e"
SURFACE = "#fcfcfb"
GRID = "#e4e3df"


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


# ------------------------------------------------------------------------------------------- running one square
_tmp = {"dir": None}
# re-entrant: route_file() holds it while _tmpdir() takes it (a plain Lock hung the suite's first full run)
_lock = threading.RLock()
_counter = [0]


def _tmpdir():
    with _lock:
        if _tmp["dir"] is None:
            # generated routes and each run's CSV; removed when Python exits, also for tools that import run_square
            _tmp["dir"] = tempfile.mkdtemp(prefix="umb_")
            atexit.register(shutil.rmtree, _tmp["dir"], True)
        return _tmp["dir"]


def _unique(name):
    d = _tmpdir()
    with _lock:
        _counter[0] += 1
        return os.path.join(d, f"{_counter[0]}_{name}")


def square_route(direction, side=UMB_SIDE_IN, drive_speed=UMB_DRIVE_MAX_SPEED, turn_speed=UMB_TURN_MAX_SPEED):
    """The square as route commands (bench/route.hpp), without comments. CW: up, right, down, left; CCW: up, left,
    down, right. Turns go the shorter way, every one 90 deg."""
    s = 1 if direction == "CW" else -1
    corners = [(0, side), (s * side, side), (s * side, 0), (0, 0)]
    headings = [90, 180, 270, 0] if s == 1 else [270, 180, 90, 0]
    lines = ["start 0 0 0", f"wait {UMB_STILL_MS}"]
    for (x, y), h in zip(corners, headings):
        lines += [f"point {x:g} {y:g} max={drive_speed:g} timeout={UMB_DRIVE_TIMEOUT_MS}", f"wait {UMB_STOP_MS}",
                  f"turn {h:g} max={turn_speed:g} timeout={UMB_TURN_TIMEOUT_MS}", f"wait {UMB_STOP_MS}"]
    lines.append(f"wait {UMB_SETTLE_MS}")
    return "\n".join(lines) + "\n"


def route_commands(path):
    """A route file's commands, comments and blank lines taken out"""
    out = []
    with open(path) as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if line:
                out.append(" ".join(line.split()))
    return out


_routes = {}


def route_file(direction, side, drive_speed, turn_speed):
    """routes/umb_*.txt for the standard square, otherwise a generated one in the scratch folder"""
    if (side, drive_speed, turn_speed) == (UMB_SIDE_IN, UMB_DRIVE_MAX_SPEED, UMB_TURN_MAX_SPEED):
        return ROUTE_FILES[direction]
    key = (direction, side, drive_speed, turn_speed)
    with _lock:
        if key not in _routes:
            path = os.path.join(_tmpdir(), f"umb_{direction.lower()}_{side:g}_{drive_speed:g}_{turn_speed:g}.txt")
            with open(path, "w") as f:
                f.write(square_route(direction, side, drive_speed, turn_speed))
            _routes[key] = path
        return _routes[key]


def geometry_args(robot=True):
    """The robot's tracking-wheel offsets, both where the world's wheels are and what LemLib is told"""
    if not robot:
        return []
    return ["--set", f"VERT_OFFSET={ROBOT_VERT_OFFSET_IN}", "--set", f"HORIZ_OFFSET={ROBOT_HORIZ_OFFSET_IN}",
            "--odom-offsets", f"{ROBOT_VERT_OFFSET_IN}:{ROBOT_HORIZ_OFFSET_IN}"]


def spin_point_args(args, spin_behind_in):
    """The world's horizontal wheel spin_behind_in further forward than LemLib is told (the last HORIZ_OFFSET in
    args wins, as the bench reads them in order): the robot then spins about a point that far behind its tracking
    centre"""
    told = ROBOT_HORIZ_OFFSET_IN
    for i, a in enumerate(args[:-1]):
        if a == "--set" and args[i + 1].startswith("HORIZ_OFFSET="):
            told = float(args[i + 1].split("=", 1)[1])
    return ["--set", f"HORIZ_OFFSET={told + spin_behind_in:g}"]


def run_square(direction, seed=1, extra_args=(), side=UMB_SIDE_IN, drive_speed=UMB_DRIVE_MAX_SPEED,
               turn_speed=UMB_TURN_MAX_SPEED, robot_geometry=True, tb=TB, spin_behind_in=0.0):
    """Drive one UMBmark square in the simulator and return what a ruler and the robot would each say.

    direction: "CW" or "CCW". seed: the bench's --seed (only the tracking-wheel slips use it). extra_args: more
    bench arguments, after the robot's geometry, so they win (e.g. ["--set", "IMU_SCALE_ERR_CW=0.012"],
    ["--imu-scale", "0.988:0.988"]). Returns a dict:
      real      {"right", "forward", "heading_change"}: where the tracking centre really is, inches from where it
                started in the start frame, and how far it really turned (deg, clockwise +, not wrapped)
      believed  {"x", "y", "theta"}: LemLib's pose at the end (what the [UMB] line prints; theta not wrapped)
      E         [right, forward] = real minus believed, inches; E_size its length
      H         real heading change minus believed theta, each wrapped to +-180, wrapped again (deg)
      duration_ms   from setPose to the pose being read (the whole run, standing still included)
      square_ms     from the first move starting to the last turn ending
      still_dps     the believed heading change over the first UMB_STILL_MS standing still, deg/s
      leg_mid_ms    the middle of each leg, ms after setPose (a drift acts like a scale error over these)
      stops         per stop after a move: [route step, |real - believed| inches] at its end
      max_err_in    the largest |real - believed| position error at any 10 ms tick
      timeouts, slips, args
    spin_behind_in: the robot spins about a point this far behind its tracking centre (0: about it). Needs the
    robot's geometry. real, E, H, the stops and max_err_in are then the tracking centre's
    """
    import pandas as pd

    route = route_file(direction, side, drive_speed, turn_speed)
    args = ["--no-gps", "--fusion", "off", "--seed", str(seed)] + geometry_args(robot_geometry) + \
        [str(a) for a in extra_args]
    if spin_behind_in:
        if not robot_geometry:
            raise ValueError("spin_behind_in needs the robot's geometry")
        args += spin_point_args(args, spin_behind_in)
    csv = _unique(f"{direction}_{seed}.csv")
    cmd = [tb, "--route", route] + args + ["--summary-json", "--csv", csv]
    done = subprocess.run(cmd, capture_output=True, text=True)
    if done.returncode != 0:
        raise RuntimeError(f"{' '.join(cmd)}\n{done.stderr}")
    summary = json.loads(done.stdout)
    frame = pd.read_csv(csv, usecols=["t_ms", "step", "true_x", "true_y", "true_heading", "pose_x", "pose_y",
                                      "pose_heading"])
    os.remove(csv)
    t = frame["t_ms"].to_numpy()
    step = frame["step"].to_numpy()
    tx, ty = frame["true_x"].to_numpy(), frame["true_y"].to_numpy()
    if spin_behind_in:
        # the tracking centre is spin_behind_in ahead of the drive centre; at the start it was at (0, D)
        th = np.radians(frame["true_heading"].to_numpy())
        tx = tx + spin_behind_in * np.sin(th)
        ty = ty + spin_behind_in * np.cos(th) - spin_behind_in
    err = np.hypot(tx - frame["pose_x"].to_numpy(), ty - frame["pose_y"].to_numpy())
    heading = frame["pose_heading"].to_numpy()
    steps = summary["step_list"]
    # the stand-still is step 1 (wait UMB_STILL_MS): heading at its last tick minus at the first tick of the run
    still = steps[0]
    end_still = np.flatnonzero(t <= still["end_ms"])[-1]
    still_dps = float(heading[end_still] - heading[0]) / ((t[end_still] - t[0]) / 1000.0)
    stops = []
    for i, s in enumerate(steps):
        if s["kind"] == "wait" and i > 0 and steps[i - 1]["kind"] != "wait":
            last = np.flatnonzero(step == s["n"])
            if len(last):
                stops.append([s["n"], float(err[last[-1]])])
    moves = [s for s in steps if s["kind"] != "wait"]
    legs = [s for s in steps if s["kind"] == "point"]
    tr, pb = list(summary["true_end"]), summary["pose_end"]
    if spin_behind_in:
        h = math.radians(tr[2])
        tr[0] += spin_behind_in * math.sin(h)
        tr[1] += spin_behind_in * math.cos(h) - spin_behind_in
    e = [tr[0] - pb[0], tr[1] - pb[1]]
    return {
        "direction": direction, "seed": seed, "side": side, "drive_speed": drive_speed, "turn_speed": turn_speed,
        "args": args,
        "real": {"right": tr[0], "forward": tr[1], "heading_change": tr[2]},
        "believed": {"x": pb[0], "y": pb[1], "theta": pb[2]},
        "E": e, "E_size": math.hypot(*e), "H": wrap180(wrap180(tr[2]) - wrap180(pb[2])),
        "duration_ms": summary["duration_ms"],
        "square_ms": moves[-1]["end_ms"] - moves[0]["start_ms"],
        "still_dps": still_dps,
        "leg_mid_ms": [(s["start_ms"] + s["end_ms"]) / 2 for s in legs],
        "stops": stops, "max_stop_err_in": max(v for _, v in stops),
        "max_err_in": float(err.max()) if spin_behind_in else summary["odom"]["max"], "spin_behind_in": spin_behind_in,
        "timeouts": summary["timeouts"], "slips": summary["world"]["slips"],
    }


def parallel(fn, items, jobs):
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
        return list(pool.map(fn, items))


# ------------------------------------------------------------------------------------------- the pencil formulas
def lead_formula(direction, e, side=UMB_SIDE_IN):
    """The small-error (first-order) formula for an inertial scale error e alone: (E right, E forward, H)"""
    a = math.pi * side * e
    if direction == "CW":
        return a, -a, -360 * e
    return -a, -a, 360 * e


def exact_formula(direction, e, side=UMB_SIDE_IN):
    """The same model without the small-e step: the real heading on leg k is k * 90 / (1 + e) degrees (mirrored
    for CCW), every leg is the full side, and the robot believes it ends exactly at the start, facing +-360"""
    s = 1 if direction == "CW" else -1
    k = np.arange(4)
    h = np.radians(s * k * 90 / (1 + e))
    return float(side * np.sin(h).sum()), float(side * np.cos(h).sum()), wrap180(s * 360 / (1 + e))


def e_from_E(direction, E, side=UMB_SIDE_IN):
    """The inertial scale error that best explains an end error E under exact_formula (least squares over a grid
    of 1e-5 from -8 % to +8 %). For checking how well the scale stands out; the team's calculator is
    tools/umbmark.py"""
    s = 1 if direction == "CW" else -1
    es = np.arange(-0.08, 0.08 + 1e-9, 1e-5)
    k = np.arange(4)
    h = np.radians(s * np.outer(1 / (1 + es), k * 90))
    mx, my = side * np.sin(h).sum(axis=1), side * np.cos(h).sum(axis=1)
    return float(es[np.argmin((mx - E[0]) ** 2 + (my - E[1]) ** 2)])


def e_from_H(H, theta):
    """The inertial scale error from the heading error: real = believed / (1 + e), so e = -H / (believed + H)"""
    return -H / (theta + H)


# ------------------------------------------------------------------------------------------- part a: signatures
def signature_list():
    """(key, label, extra bench arguments, the inertial error per direction it amounts to, if it is one)"""
    told_v, told_h = ROBOT_VERT_OFFSET_IN, ROBOT_HORIZ_OFFSET_IN
    imu = lambda cw, ccw: ["--set", f"IMU_SCALE_ERR_CW={cw}", "--set", f"IMU_SCALE_ERR_CCW={ccw}"]
    rows = [
        ("perfect", "perfect robot", [], None),
        ("imu_both_plus", f"inertial +{IMU_ERR * 100:g} % both ways", imu(IMU_ERR, IMU_ERR), (IMU_ERR, IMU_ERR)),
        ("imu_both_minus", f"inertial -{IMU_ERR * 100:g} % both ways", imu(-IMU_ERR, -IMU_ERR), (-IMU_ERR, -IMU_ERR)),
        ("imu_cw", f"inertial +{IMU_ERR * 100:g} % clockwise only", imu(IMU_ERR, 0), (IMU_ERR, 0)),
        ("imu_ccw", f"inertial +{IMU_ERR * 100:g} % counter-clockwise only", imu(0, IMU_ERR), (0, IMU_ERR)),
        # the correction rule: the value to put in IMU_SCALE_* is current / (1 + e); with it the error must go
        ("imu_corrected", f"inertial +{IMU_ERR * 100:g} %, setImuScale 1/(1+e)",
         imu(IMU_ERR, IMU_ERR) + ["--imu-scale", f"{1 / (1 + IMU_ERR):.8f}:{1 / (1 + IMU_ERR):.8f}"], None),
    ]
    for d in DRIFTS_DEG_S:
        rows.append((f"drift_{d:g}", f"heading drift +{d:g} deg/s", ["--set", f"IMU_DRIFT_DEG_S={d}"], None))
    rows += [
        ("vert_size", f"vertical wheel reads {WHEEL_SIZE_ERR * 100:g} % long", ["--set", f"VERT_SCALE_ERR={WHEEL_SIZE_ERR}"],
         None),
        ("horiz_size", f"horizontal wheel reads {WHEEL_SIZE_ERR * 100:g} % long",
         ["--set", f"HORIZ_SCALE_ERR={WHEEL_SIZE_ERR}"], None),
        ("vert_off_plus", f"vertical offset {OFFSET_ERR_IN:+g} in (wheel further right)",
         ["--set", f"VERT_OFFSET={told_v + OFFSET_ERR_IN:g}"], None),
        ("vert_off_minus", f"vertical offset {-OFFSET_ERR_IN:+g} in", ["--set", f"VERT_OFFSET={told_v - OFFSET_ERR_IN:g}"],
         None),
        ("horiz_off_plus", f"horizontal offset {OFFSET_ERR_IN:+g} in (wheel further forward)",
         ["--set", f"HORIZ_OFFSET={told_h + OFFSET_ERR_IN:g}"], None),
        ("horiz_off_minus", f"horizontal offset {-OFFSET_ERR_IN:+g} in",
         ["--set", f"HORIZ_OFFSET={told_h - OFFSET_ERR_IN:g}"], None),
        ("horiz_sign", f"horizontal offset sign wrong (told {told_h:g}, really {-told_h:+g})",
         ["--set", f"HORIZ_OFFSET={-told_h:g}"], None),
        ("vert_yaw", f"vertical wheel {YAW_DEG:g} deg off parallel", ["--set", f"VERT_YAW_DEG={YAW_DEG}"], None),
        ("horiz_yaw", f"horizontal wheel {YAW_DEG:g} deg off parallel", ["--set", f"HORIZ_YAW_DEG={YAW_DEG}"], None),
    ]
    return rows


def drift_prediction(r, drift):
    """A steady drift d acts like a scale error over the square: the believed heading runs ahead by d * t, so leg
    k is turned by d * (t_k - t_0) on top of the steady part, which is what a scale error of d * T_c / 90 does
    (T_c the time from one leg's middle to the next). Clockwise e = +d * T_c / 90, counter-clockwise -d * T_c / 90.
    H = -d * (the time from setPose to the pose being read)."""
    mids = r["leg_mid_ms"]
    tc = (mids[-1] - mids[0]) / (len(mids) - 1) / 1000.0
    e = drift * tc / 90.0 * (1 if r["direction"] == "CW" else -1)
    ex, ey, _ = exact_formula(r["direction"], e, r["side"])
    return {"T_c_s": tc, "e_equiv": e, "E": [ex, ey], "H": -drift * r["duration_ms"] / 1000.0}


def part_signatures(jobs):
    rows = signature_list()
    jobs_list = [(key, d) for key, _, _, _ in rows for d in ("CW", "CCW")]
    extra = {key: args for key, _, args, _ in rows}
    res = dict(zip(jobs_list, parallel(lambda j: run_square(j[1], 1, extra[j[0]]), jobs_list, jobs)))
    # the bench's own geometry (0.9 in right, 2.55 in front), perfect robot: the geometry alone must not matter
    old = {d: run_square(d, 1, [], robot_geometry=False) for d in ("CW", "CCW")}
    out = {"rows": [], "old_geometry_perfect": {d: _brief(old[d]) for d in old}}
    for key, label, args, imu in rows:
        entry = {"key": key, "label": label, "args": args}
        for d in ("CW", "CCW"):
            r = res[(key, d)]
            b = _brief(r)
            if imu is not None:
                e = imu[0] if d == "CW" else imu[1]
                lx, ly, lh = lead_formula(d, e)
                xx, xy, xh = exact_formula(d, e)
                b["lead"] = {"E": [lx, ly], "H": lh}
                b["exact"] = {"E": [xx, xy], "H": xh}
                b["sim_minus_lead"] = {"E": [r["E"][0] - lx, r["E"][1] - ly], "H": r["H"] - lh}
                b["e_true"] = e
                b["e_from_E"] = e_from_E(d, r["E"])
                b["e_from_H"] = e_from_H(r["H"], r["believed"]["theta"])
            if key.startswith("drift_"):
                drift = float(key.split("_")[1])
                p = drift_prediction(r, drift)
                b["drift_prediction"] = p
                b["e_from_E"] = e_from_E(d, r["E"])
                b["e_from_H"] = e_from_H(r["H"], r["believed"]["theta"])
            entry[d] = b
        biggest = max(entry["CW"]["E_size"], entry["CCW"]["E_size"])
        entry["invisible"] = key not in NO_ERROR_ROWS and biggest < INVISIBLE_IN
        out["rows"].append(entry)
    return out


def _brief(r):
    return {"E": r["E"], "E_size": r["E_size"], "H": r["H"], "real": r["real"], "believed": r["believed"],
            "duration_ms": r["duration_ms"], "square_ms": r["square_ms"], "still_dps": r["still_dps"],
            "max_stop_err_in": r["max_stop_err_in"], "max_err_in": r["max_err_in"], "stops": r["stops"],
            "timeouts": r["timeouts"], "leg_mid_ms": r["leg_mid_ms"]}


def print_signatures(sig):
    print("\n== a. Signatures: one error at a time, perfect otherwise, no slip, side 48 in, speed 70 ==")
    print("E = real minus believed end (in, x right / y forward of the start frame); H = real minus believed heading "
          "(deg); corner = largest |real - believed| at a stop on the way")
    print(f"{'error':58s} {'dir':4s} {'E x':>7s} {'E y':>7s} {'|E|':>6s} {'H':>7s} {'corner':>7s}  note")
    for entry in sig["rows"]:
        for d in ("CW", "CCW"):
            b = entry[d]
            note = ""
            if entry["invisible"] and d == "CW":
                note = f"square cannot see it (|E| < {INVISIBLE_IN} both ways)"
            if entry["key"] in NO_ERROR_ROWS and d == "CW":
                note = "a check: |E| should be about 0"
            if b["timeouts"]:
                note += f" {b['timeouts']} moves timed out"
            label = entry["label"] if d == "CW" else ""
            print(f"{label:58s} {d:4s} {b['E'][0]:7.2f} {b['E'][1]:7.2f} {b['E_size']:6.2f} {b['H']:7.2f} "
                  f"{b['max_stop_err_in']:7.2f}  {note}")
    og = sig["old_geometry_perfect"]
    print(f"(perfect robot with the bench's old geometry 0.9 / 2.55 in: |E| {og['CW']['E_size']:.2f} CW, "
          f"{og['CCW']['E_size']:.2f} CCW)")
    print("\nInertial rows against the small-error formula (CW E = (+pi L e, -pi L e), H = -360 e; CCW E = (-pi L e, "
          "-pi L e), H = +360 e):")
    print(f"{'error':40s} {'dir':4s} {'sim E':>15s} {'approx E':>15s} {'exact E':>15s} {'sim H':>7s} {'aprx H':>7s} "
          f"{'e from E':>9s} {'e from H':>9s}")
    for entry in sig["rows"]:
        for d in ("CW", "CCW"):
            b = entry[d]
            if "lead" not in b:
                continue
            print(f"{entry['label'][:40]:40s} {d:4s} ({b['E'][0]:5.2f},{b['E'][1]:6.2f}) "
                  f"({b['lead']['E'][0]:5.2f},{b['lead']['E'][1]:6.2f}) ({b['exact']['E'][0]:5.2f},{b['exact']['E'][1]:6.2f}) "
                  f"{b['H']:7.2f} {b['lead']['H']:7.2f} {100 * b['e_from_E']:8.2f}% {100 * b['e_from_H']:8.2f}%")
    print("\nDrift rows: drift acts like a scale error e = +d T_c / 90 clockwise, -d T_c / 90 counter-clockwise "
          "(T_c = time from one leg's middle to the next), H = -d T:")
    for entry in sig["rows"]:
        for d in ("CW", "CCW"):
            b = entry[d]
            if "drift_prediction" not in b:
                continue
            p = b["drift_prediction"]
            print(f"{entry['label']:28s} {d:4s} T_c {p['T_c_s']:.2f} s  e_equiv {100 * p['e_equiv']:+.3f} %  "
                  f"predicted E ({p['E'][0]:5.2f},{p['E'][1]:5.2f}) sim ({b['E'][0]:5.2f},{b['E'][1]:5.2f})  "
                  f"predicted H {p['H']:6.2f} sim {b['H']:6.2f}  still_dps {b['still_dps']:.4f}  "
                  f"e from E {100 * b['e_from_E']:+.3f} %")


# ------------------------------------------------------------------------------------------- part a2: spin point
SPIN_BEHIND_IN = [0.0, 2.24, 5.8]  # 5.8: what robot-config.cpp's [ODOMCAL] note implies; 2.24: the calculator's own test


def part_spin(jobs):
    """A robot that spins about a point D behind its tracking centre, with and without the inertial error. Neither
    slip nor any other error"""
    imu = ["--set", f"IMU_SCALE_ERR_CW={IMU_ERR}", "--set", f"IMU_SCALE_ERR_CCW={IMU_ERR}"]
    jl = [(D, e, d) for D in SPIN_BEHIND_IN for e in (0.0, IMU_ERR) for d in ("CW", "CCW")]
    res = parallel(lambda j: run_square(j[2], 1, imu if j[1] else [], spin_behind_in=j[0]), jl, jobs)
    rows = []
    for (D, e, d), r in zip(jl, res):
        row = {"spin_behind_in": D, "e_true": e, "dir": d, "E": r["E"], "E_size": r["E_size"], "H": r["H"],
               "max_err_in": r["max_err_in"], "timeouts": r["timeouts"],
               "e_from_E": e_from_E(d, r["E"]), "e_from_H": e_from_H(r["H"], r["believed"]["theta"])}
        if e:
            row["ratio_E"] = row["e_from_E"] / e
            row["predicted_ratio"] = 1 - 2.3 * D / UMB_SIDE_IN  # tools/umbmark.py's notes
        rows.append(row)
    return {"rows": rows}


def print_spin(sp):
    print("\n== a2. Turning about a point behind the tracking centre (no slip, side 48 in, speed 70) ==")
    print("e from E = the scale error the position alone gives; e from H = the heading's. The calculator's notes say "
          "the position's comes out x(1 - 2.3 D / L)")
    print(f"{'D in':>5s} {'e true':>7s} {'dir':4s} {'E x':>7s} {'E y':>7s} {'|E|':>6s} {'H':>7s} {'e from E':>9s} "
          f"{'e from H':>9s} {'ratio':>6s} {'1-2.3D/L':>8s} {'max err':>7s}")
    for r in sp["rows"]:
        ratio = f"{r['ratio_E']:6.3f} {r['predicted_ratio']:8.3f}" if "ratio_E" in r else f"{'':6s} {'':8s}"
        print(f"{r['spin_behind_in']:5.2f} {100 * r['e_true']:6.2f}% {r['dir']:4s} {r['E'][0]:7.2f} {r['E'][1]:7.2f} "
              f"{r['E_size']:6.2f} {r['H']:7.2f} {100 * r['e_from_E']:8.3f}% {100 * r['e_from_H']:8.3f}% {ratio} "
              f"{r['max_err_in']:7.2f}" + (f"  {r['timeouts']} timed out" if r["timeouts"] else ""))


# ------------------------------------------------------------------------------------------- part b: slip
def slip_args(per, mx):
    return ["--set", f"SLIP_PER_IN={per}", "--set", f"SLIP_MAX_IN={mx}"]


def scatter_stats(results, side=UMB_SIDE_IN):
    """Spread of E over runs: its mean, RMS and largest radius about the mean, and the 1-sigma error of an inertial
    scale estimated from the mean of RUNS_PER_DIRECTION runs (per-run e estimates' spread / sqrt(runs))"""
    E = np.array([r["E"] for r in results])
    mean = E.mean(axis=0)
    rad = np.hypot(*(E - mean).T)
    es = np.array([e_from_E(r["direction"], r["E"], side) for r in results])
    return {"n": len(results), "mean": mean.tolist(), "mean_size": float(np.hypot(*mean)),
            "mean_se": (E.std(axis=0, ddof=1) / math.sqrt(len(results))).tolist(),
            "rms_radius": float(np.sqrt((rad ** 2).mean())), "max_radius": float(rad.max()),
            "e_sd_one_run": float(es.std(ddof=1)), "e_sd_mean_of_runs": float(es.std(ddof=1) / math.sqrt(RUNS_PER_DIRECTION)),
            "H_sd": float(np.std([r["H"] for r in results], ddof=1)),
            "mean_slips": float(np.mean([r["slips"] for r in results])),
            "max_err_in_median": float(np.median([r["max_err_in"] for r in results]))}


def part_slip(seeds, jobs, side=UMB_SIDE_IN, speed=UMB_DRIVE_MAX_SPEED, shift_seeds=SHIFT_SEEDS):
    many = list(range(1, max(shift_seeds, len(seeds)) + 1))
    jobs_list = [(lvl, d, s) for lvl in range(len(SLIP_LEVELS)) for d in ("CW", "CCW") for s in many]

    def one(j):
        lvl, d, s = j
        _, per, mx = SLIP_LEVELS[lvl]
        return run_square(d, s, slip_args(per, mx), side=side, drive_speed=speed, turn_speed=speed)

    res = dict(zip(jobs_list, parallel(one, jobs_list, jobs)))
    noslip = {d: run_square(d, 1, [], side=side, drive_speed=speed, turn_speed=speed) for d in ("CW", "CCW")}
    out = {"side": side, "speed": speed, "seeds": list(seeds), "shift_seeds": len(many), "levels": []}
    for lvl, (label, per, mx) in enumerate(SLIP_LEVELS):
        entry = {"label": label, "slip_per_in": per, "slip_max_in": mx}
        for d in ("CW", "CCW"):
            st = scatter_stats([res[(lvl, d, s)] for s in seeds], side)
            st["mean_shift"] = float(math.hypot(st["mean"][0] - noslip[d]["E"][0], st["mean"][1] - noslip[d]["E"][1]))
            st["E_per_seed"] = [res[(lvl, d, s)]["E"] for s in seeds]
            big = scatter_stats([res[(lvl, d, s)] for s in many], side)
            st["many"] = {"n": big["n"], "mean": big["mean"], "mean_se": big["mean_se"], "rms_radius": big["rms_radius"],
                          "mean_shift": float(math.hypot(big["mean"][0] - noslip[d]["E"][0],
                                                         big["mean"][1] - noslip[d]["E"][1]))}
            entry[d] = st
        out["levels"].append(entry)
    return out


def print_slip(slip):
    print(f"\n== b. Slip: perfect calibration, {len(slip['seeds'])} seeds per direction, side {slip['side']} in, "
          f"speed {slip['speed']} ==")
    print("spread of E about its mean (RMS and largest radius), how far the mean moves from the no-slip end, and the "
          f"1-sigma error of e from a {RUNS_PER_DIRECTION}-run mean")
    print(f"{'slip':8s} {'dir':4s} {'slips':>6s} {'mean E (30)':>15s} {'shift':>6s} {'RMS r':>6s} {'max r':>6s} "
          f"{'H sd':>6s} {'e sd, 1 run':>12s} {'e sd, 5-run mean':>17s} {'mean E, ' + str(slip['shift_seeds']) + ' seeds':>16s} "
          f"{'+- se':>12s} {'shift':>6s}")
    for entry in slip["levels"]:
        for d in ("CW", "CCW"):
            st = entry[d]
            m = st["many"]
            print(f"{entry['label']:8s} {d:4s} {st['mean_slips']:6.1f} ({st['mean'][0]:6.2f},{st['mean'][1]:6.2f}) "
                  f"{st['mean_shift']:6.2f} {st['rms_radius']:6.2f} {st['max_radius']:6.2f} {st['H_sd']:6.3f} "
                  f"{100 * st['e_sd_one_run']:11.3f}% {100 * st['e_sd_mean_of_runs']:16.3f}% "
                  f"({m['mean'][0]:6.2f},{m['mean'][1]:6.2f}) ({m['mean_se'][0]:4.2f},{m['mean_se'][1]:4.2f}) "
                  f"{m['mean_shift']:6.2f}")
    print("(H sd is 0 because the simulated inertial sensor has no noise and slip never reaches it: on the robot H "
          "scatters with the sensor's noise and with how well the heading is measured)")


# ------------------------------------------------------------------------------------------- part c: sensitivity
def part_sensitivity(seeds, jobs):
    import gps_suite as gs  # the uncalibrated and calibrated mixes, so they stay the GPS suite's

    imu = ["--set", f"IMU_SCALE_ERR_CW={IMU_ERR}", "--set", f"IMU_SCALE_ERR_CCW={IMU_ERR}"]
    drift = ["--set", f"IMU_DRIFT_DEG_S={SENS_DRIFT_DEG_S}"]
    _, per, mx = SLIP_LEVELS[0]
    configs = [(side, speed) for side in SIDES_IN for speed in SPEEDS]
    jl = []
    for side, speed in configs:
        for d in ("CW", "CCW"):
            jl += [("imu", side, speed, d, 1), ("drift", side, speed, d, 1), ("clean", side, speed, d, 1)]
            jl += [("slip", side, speed, d, s) for s in seeds]
    args = {"imu": imu, "drift": drift, "clean": [], "slip": slip_args(per, mx)}
    res = dict(zip(jl, parallel(lambda j: run_square(j[3], j[4], args[j[0]], side=j[1], drive_speed=j[2],
                                                     turn_speed=j[2]), jl, jobs)))
    grid = []
    for side, speed in configs:
        entry = {"side": side, "speed": speed}
        for d in ("CW", "CCW"):
            st = scatter_stats([res[("slip", side, speed, d, s)] for s in seeds], side)
            r_imu = res[("imu", side, speed, d, 1)]
            noise = st["rms_radius"] / math.sqrt(RUNS_PER_DIRECTION)
            entry[d] = {"imu_E": r_imu["E"], "imu_E_size": r_imu["E_size"], "imu_H": r_imu["H"],
                        "lead_E_size": math.sqrt(2) * math.pi * side * IMU_ERR,
                        "drift_E_size": res[("drift", side, speed, d, 1)]["E_size"],
                        "drift_H": res[("drift", side, speed, d, 1)]["H"],
                        "clean_E_size": res[("clean", side, speed, d, 1)]["E_size"],
                        "duration_ms": res[("clean", side, speed, d, 1)]["duration_ms"],
                        "square_ms": res[("clean", side, speed, d, 1)]["square_ms"],
                        "timeouts": max(res[(k, side, speed, d, 1)]["timeouts"] for k in ("imu", "drift", "clean")),
                        "slip_rms_radius": st["rms_radius"], "slip_max_radius": st["max_radius"],
                        "slip_e_sd_mean_of_runs": st["e_sd_mean_of_runs"],
                        "imu_to_noise": r_imu["E_size"] / noise if noise > 0 else float("inf")}
        grid.append(entry)

    # the GPS suite's two robots: error signs per robot, slips per run
    mixes = {}
    for name, sizes in (("A3a uncalibrated", gs.A3A), ("A3b calibrated", gs.A3B)):
        errs = gs.odom_errors(sizes, slips=True)
        jl = [(n, d, r) for n in range(1, MIX_ROBOTS + 1) for d in ("CW", "CCW") for r in range(RUNS_PER_DIRECTION)]
        runs = dict(zip(jl, parallel(lambda j: run_square(j[1], MIX_SEED_BASE * j[0] + j[2], errs(j[0])), jl, jobs)))
        robots = []
        for n in range(1, MIX_ROBOTS + 1):
            a = errs(n)
            e_true = float(next(x for x in a if x.startswith("IMU_SCALE_ERR_CW=")).split("=")[1])
            rob = {"robot": n, "args": a, "e_true": e_true}
            for d in ("CW", "CCW"):
                rs = [runs[(n, d, r)] for r in range(RUNS_PER_DIRECTION)]
                E = np.mean([r["E"] for r in rs], axis=0)
                H = float(np.mean([r["H"] for r in rs]))
                theta = float(np.mean([r["believed"]["theta"] for r in rs]))
                model = exact_formula(d, e_true)
                rob[d] = {"E_mean": E.tolist(), "E_size": float(np.hypot(*E)), "H_mean": H,
                          "E_runs": [r["E"] for r in rs],
                          "e_from_E": e_from_E(d, E), "e_from_H": e_from_H(H, theta),
                          "residual_in": float(math.hypot(E[0] - model[0], E[1] - model[1]))}
            robots.append(rob)
        summary = {}
        for d in ("CW", "CCW"):
            errE = [abs(rb[d]["e_from_E"] - rb["e_true"]) for rb in robots]
            errH = [abs(rb[d]["e_from_H"] - rb["e_true"]) for rb in robots]
            summary[d] = {"median_E_size": float(np.median([rb[d]["E_size"] for rb in robots])),
                          "median_err_e_from_E": float(np.median(errE)), "max_err_e_from_E": float(max(errE)),
                          "median_err_e_from_H": float(np.median(errH)), "max_err_e_from_H": float(max(errH)),
                          "median_residual_in": float(np.median([rb[d]["residual_in"] for rb in robots]))}
        mixes[name] = {"sizes": sizes, "slip_per_in": gs.SLIP_PER_IN, "slip_max_in": gs.SLIP_MAX_IN,
                       "robots": robots, "summary": summary}
    return {"grid": grid, "mixes": mixes, "imu_err": IMU_ERR, "drift_deg_s": SENS_DRIFT_DEG_S,
            "slip": SLIP_LEVELS[0], "seeds": list(seeds)}


def print_sensitivity(sens):
    print(f"\n== c. Sensitivity: side and speed (inertial +{IMU_ERR * 100:g} % both ways, drift "
          f"{SENS_DRIFT_DEG_S:g} deg/s, slip {sens['slip'][0]} over {len(sens['seeds'])} seeds; no slip otherwise) ==")
    print(f"{'side':>4s} {'speed':>5s} {'dir':4s} {'square s':>8s} {'|E| imu':>8s} {'approx':>6s} {'H imu':>6s} "
          f"{'|E| drift':>9s} {'H drift':>7s} {'slip RMS r':>10s} {'slip max r':>10s} {'e sd 5-run':>10s} "
          f"{'imu/noise':>9s}")
    for g in sens["grid"]:
        for d in ("CW", "CCW"):
            v = g[d]
            print(f"{g['side']:4d} {g['speed']:5d} {d:4s} {v['square_ms'] / 1000:8.2f} {v['imu_E_size']:8.2f} "
                  f"{v['lead_E_size']:6.2f} {v['imu_H']:6.2f} {v['drift_E_size']:9.2f} {v['drift_H']:7.2f} "
                  f"{v['slip_rms_radius']:10.2f} {v['slip_max_radius']:10.2f} {100 * v['slip_e_sd_mean_of_runs']:9.3f}% "
                  f"{v['imu_to_noise']:9.1f}" + (f"  {v['timeouts']} timed out" if v["timeouts"] else ""))
    print("(imu/noise: |E| of the inertial error over the RMS radius of a 5-run mean under slip)")
    print(f"\nThe GPS suite's robots ({MIX_ROBOTS} robots each, signs per robot, {RUNS_PER_DIRECTION} runs per "
          f"direction with slip, side {UMB_SIDE_IN}, speed {UMB_DRIVE_MAX_SPEED}); e estimated from the 5-run mean")
    print(f"{'mix':18s} {'dir':4s} {'median |E|':>10s} {'e from E: median / max error':>30s} "
          f"{'e from H: median / max error':>30s} {'non-inertial part of E':>23s}")
    for name, m in sens["mixes"].items():
        for d in ("CW", "CCW"):
            s = m["summary"][d]
            print(f"{name if d == 'CW' else '':18s} {d:4s} {s['median_E_size']:10.2f} "
                  f"{100 * s['median_err_e_from_E']:20.3f} / {100 * s['max_err_e_from_E']:.3f} pp "
                  f"{100 * s['median_err_e_from_H']:20.3f} / {100 * s['max_err_e_from_H']:.3f} pp "
                  f"{s['median_residual_in']:20.2f} in")
        print(f"{'':18s}      (true inertial error {100 * m['sizes']['imu']:g} %, wheels {100 * m['sizes']['wheel']:g} %, "
              "signs per robot)")


# ------------------------------------------------------------------------------------------- part e: the figure
FIGURE_PANELS = [  # (signature key, panel title)
    ("imu_both_plus", "inertial +1.2 % both ways"), ("imu_both_minus", "inertial -1.2 % both ways"),
    ("imu_cw", "inertial +1.2 % clockwise only"), ("imu_ccw", "inertial +1.2 % counter-clockwise only"),
    ("drift_0.02", "heading drift +0.02 deg/s"), ("drift_0.05", "heading drift +0.05 deg/s"),
    ("vert_size", "vertical wheel 1.5 % long"), ("horiz_size", "horizontal wheel 1.5 % long"),
    ("vert_off_plus", "vertical offset +0.5 in"), ("vert_off_minus", "vertical offset -0.5 in"),
    ("horiz_off_plus", "horizontal offset +0.5 in"), ("horiz_off_minus", "horizontal offset -0.5 in"),
    ("horiz_sign", "horizontal offset, wrong sign"), ("vert_yaw", "vertical wheel 2 deg off parallel"),
    ("horiz_yaw", "horizontal wheel 2 deg off parallel"), ("imu_corrected", "inertial +1.2 %, corrected"),
]


def _num(v, fmt):
    """A number for a label, with -0.0 shown as 0.0"""
    text = format(v, fmt)
    return format(0.0, fmt) if float(text) == 0 else text


def _free_corner(points, lim):
    """A panel corner for the panel's note, clear of every marker: bottom left if it is free, otherwise the first
    free one, otherwise the one furthest from the markers. Returns (x, y, ha, va) in axes fractions"""
    best, where = -1.0, None
    for cx, cy in ((-1, -1), (1, -1), (-1, 1), (1, 1)):
        # the note's middle sits about a quarter of the panel in from the corner
        mx, my = cx * lim * 0.5, cy * lim * 0.7
        d = min(math.hypot(px - mx, py - my) for px, py in points)
        here = (0.03 if cx < 0 else 0.97, 0.03 if cy < 0 else 0.97, "left" if cx < 0 else "right",
                "bottom" if cy < 0 else "top")
        if d > NOTE_CLEARANCE_IN:
            return here
        if d > best:
            best, where = d, here
    return where


def draw_figure(sig, slip, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Circle

    rows = {e["key"]: e for e in sig["rows"]}
    floor = {d: slip["levels"][0][d]["rms_radius"] / math.sqrt(RUNS_PER_DIRECTION) for d in ("CW", "CCW")}
    floor_r = max(floor.values())
    lim = 3.2
    plt.rcParams.update({"font.size": 10.5, "axes.edgecolor": INK_2, "axes.labelcolor": INK_2,
                         "xtick.color": INK_2, "ytick.color": INK_2, "text.color": INK})
    fig, axes = plt.subplots(4, 4, figsize=(12, 12.9), dpi=100, facecolor=SURFACE)
    for ax, (key, title) in zip(axes.flat, FIGURE_PANELS):
        e = rows[key]
        ax.set_facecolor(SURFACE)
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_aspect("equal")
        ax.set_xticks([-3, -2, -1, 0, 1, 2, 3])
        ax.set_yticks([-3, -2, -1, 0, 1, 2, 3])
        ax.grid(True, color=GRID, linewidth=0.8, zorder=0)
        for s in ax.spines.values():
            s.set_color(GRID)
        ax.tick_params(length=0, labelsize=9)
        ax.add_patch(Circle((0, 0), floor_r, fill=False, linestyle=(0, (3, 2)), color=INK_2, linewidth=1, zorder=1))
        ax.plot([0], [0], marker="+", color=INK, markersize=12, markeredgewidth=1.6, zorder=3)
        for d, colour, marker in (("CW", CW_COLOUR, "o"), ("CCW", CCW_COLOUR, "^")):
            b = e[d]
            if "lead" in b:
                ax.plot([b["lead"]["E"][0]], [b["lead"]["E"][1]], marker=marker, markersize=11, markerfacecolor="none",
                        markeredgecolor=colour, markeredgewidth=1.2, linestyle="none", zorder=2)
            ax.plot([b["E"][0]], [b["E"][1]], marker=marker, markersize=9, color=colour, markeredgecolor=SURFACE,
                    markeredgewidth=1.5, linestyle="none", zorder=4)
        ax.set_title(title, fontsize=10.5, color=INK, pad=6)
        note = (f"|E| {_num(e['CW']['E_size'], '.2f')} / {_num(e['CCW']['E_size'], '.2f')} in\n"
                f"H {_num(e['CW']['H'], '+.1f')} / {_num(e['CCW']['H'], '+.1f')} deg")
        corner = max(e["CW"]["max_stop_err_in"], e["CCW"]["max_stop_err_in"])
        if corner >= 1:
            note += f"\ncorners off by up to {corner:.1f} in"
        if e["invisible"]:
            note += "\nthe square cannot see it"
        if key in NO_ERROR_ROWS:
            note += "\nIMU_SCALE = 1 / (1 + e): error gone"
        points = [(0, 0)] + [tuple(e[d]["E"]) for d in ("CW", "CCW")] + \
            [tuple(e[d]["lead"]["E"]) for d in ("CW", "CCW") if "lead" in e[d]]
        tx, ty, ha, va = _free_corner(points, lim)
        ax.text(tx, ty, note, transform=ax.transAxes, fontsize=8.5, color=INK_2, va=va, ha=ha,
                bbox={"facecolor": SURFACE, "edgecolor": "none", "alpha": 0.85, "pad": 1.5})
    for ax in axes[-1]:
        ax.set_xlabel("E right (in)", fontsize=9)
    for ax in axes[:, 0]:
        ax.set_ylabel("E forward (in)", fontsize=9)
    handles = [
        Line2D([], [], marker="o", color=CW_COLOUR, markeredgecolor=SURFACE, markersize=9, linestyle="none",
               label="clockwise square: where the robot really ends"),
        Line2D([], [], marker="^", color=CCW_COLOUR, markeredgecolor=SURFACE, markersize=9, linestyle="none",
               label="counter-clockwise square"),
        Line2D([], [], marker="o", markerfacecolor="none", markeredgecolor=INK_2, markersize=10, linestyle="none",
               label="hollow: the small-error pencil formula (inertial rows)"),
        Line2D([], [], marker="+", color=INK, markersize=11, markeredgewidth=1.6, linestyle="none",
               label="where the robot believes it is (the start)"),
        Line2D([], [], color=INK_2, linestyle=(0, (3, 2)), linewidth=1,
               label=f"slip scatter of a {RUNS_PER_DIRECTION}-run mean ({SLIP_LEVELS[0][0]} slip, RMS {floor_r:.2f} in)"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=2, frameon=False, fontsize=9.5, bbox_to_anchor=(0.5, 0.0))
    fig.suptitle(f"UMBmark square in the simulator: end error E = real minus believed, one error at a time "
                 f"(side {UMB_SIDE_IN} in, speed {UMB_DRIVE_MAX_SPEED}, no slip)", fontsize=12, color=INK, y=0.995)
    fig.text(0.5, 0.968, "Each note: |E| and H clockwise / counter-clockwise; \"corners\": the largest real minus "
             "believed at a stop on the way round", ha="center", fontsize=9.5, color=INK_2)
    fig.tight_layout(rect=(0, 0.06, 1, 0.965))
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


# ------------------------------------------------------------------------------------------- main
def check_routes():
    """routes/umb_*.txt must be exactly the square square_route() builds (comments aside)"""
    out = {}
    for d, path in ROUTE_FILES.items():
        out[d] = route_commands(path) == [" ".join(l.split()) for l in square_route(d).splitlines() if l.strip()]
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seeds", type=int, default=SEEDS)
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
    ap.add_argument("--no-figures", action="store_true")
    o = ap.parse_args()
    if not os.path.exists(TB):
        print(f"no {TB}: run make -C tools/sim first", file=sys.stderr)
        return 2
    seeds = list(range(1, o.seeds + 1))
    os.makedirs(OUT, exist_ok=True)
    try:
        routes_ok = check_routes()
        print("routes/umb_cw.txt and umb_ccw.txt match the square this suite builds: "
              + ", ".join(f"{d} {'yes' if ok else 'NO'}" for d, ok in routes_ok.items()))
        sig = part_signatures(o.jobs)
        print_signatures(sig)
        spin = part_spin(o.jobs)
        print_spin(spin)
        slip = part_slip(seeds, o.jobs)
        print_slip(slip)
        sens = part_sensitivity(seeds, o.jobs)
        print_sensitivity(sens)
        results = {
            "square": {"side_in": UMB_SIDE_IN, "drive_max_speed": UMB_DRIVE_MAX_SPEED,
                       "turn_max_speed": UMB_TURN_MAX_SPEED, "stop_ms": UMB_STOP_MS, "still_ms": UMB_STILL_MS,
                       "settle_ms": UMB_SETTLE_MS, "routes": {d: os.path.relpath(p, REPO) for d, p in ROUTE_FILES.items()},
                       "routes_match_suite": routes_ok},
            "robot_geometry": {"vert_offset_in": ROBOT_VERT_OFFSET_IN, "horiz_offset_in": ROBOT_HORIZ_OFFSET_IN},
            "signatures": sig, "spin_point": spin, "slip": slip, "sensitivity": sens,
        }
        path = os.path.join(OUT, "umb_results.json")
        with open(path, "w") as f:
            json.dump(results, f, indent=1)
        print(f"\nresults: {path}")
        if not o.no_figures:
            fig = os.path.join(OUT, FIGURE)
            draw_figure(sig, slip, fig)
            os.makedirs(DOCS_SIM, exist_ok=True)
            shutil.copy(fig, os.path.join(DOCS_SIM, FIGURE))
            print(f"figure: {fig} (copied to {os.path.join(DOCS_SIM, FIGURE)})")
    finally:
        if _tmp["dir"]:
            shutil.rmtree(_tmp["dir"], ignore_errors=True)
            _tmp["dir"] = None
    return 0


if __name__ == "__main__":
    sys.exit(main())
