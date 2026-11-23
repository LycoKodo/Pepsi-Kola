#!/opt/anaconda3/bin/python3
"""UMBmark square end to end: the simulator drives the squares, a fake student writes down what a ruler would say,
the team's calculator (tools/umbmark.py) works out the inertial scale, and its answer goes back into the simulator.

    make -C tools/sim                                    # build the testbenches first
    /opt/anaconda3/bin/python3 tools/sim/umb_e2e.py [--jobs 8] [--work DIR] [--robots 20] [--quick]

Nothing here runs on the robot. The robot's own routine (src/umbmark.cpp) is not run either: the simulator's routes
(routes/umb_cw.txt, umb_ccw.txt) drive the same square with the real LemLib, and this script checks that the
routine's numbers (side, speeds, waits) and its [UMB] print are the ones it copies.

For every session (five clockwise and five counter-clockwise squares, each with its own slip seed) it writes what a
student would have after a session at the robot:
  - a log, as `pros terminal --output` would capture it: the routine's [UMB] line for each run, printed with the
    routine's own printf format (read out of src/umbmark.cpp), from the pose the simulated robot believed, with
    other lines the program prints in between and one run stopped by a stick (it prints [umbmark], not [UMB], and
    uses up a run number);
  - three measurement files, from the simulator's truth (the "person with a ruler"): the simple form (where the
    centre mark ended, to the nearest 1/8 in, no heading, and which way the robot ended turned, written when it is
    SEEN_DEG or more), the simple form with a heading read to the nearest 0.5 deg (no turned column), and the
    corners form (two back corners 12.5 in apart and 7 in behind the centre, each to 1/8 in, and turned).
It then runs `python3 tools/umbmark.py FILE --log LOG --json OUT` as a student would (a separate process, nothing
imported) and checks its JSON against the truth. The calculator gives its numbers (exit status 0) or holds them back
and says why (exit status 3, "constants": null): a held-back session keeps the scales it had, as a student would.
Errors and coverage below count the sessions that gave numbers; how often each group was held back is counted too.

Scenarios (round 1, scales 1.0):
  a  inertial scale only, no slip: +1.2 %, -1.2 %, +0.3 %, and CW +1.5 % / CCW -0.5 %
  b  the same four with the suites' slip (0.45 %) and with 1.5 % slip
  c  20 random uncalibrated robots (inertial up to 1.5 % each way, wheel sizes up to 1.5 %, offsets wrong by up to
     0.5 in, 0.45 % slip) and 20 calibrated ones (0.2 %, 0.5 %, 0.1 in)
  d  heading drift 0.05 deg/s alone
  e  the horizontal wheel's offset with the wrong sign, alone
  f  slip alone at 0.45 %, 1.5 % and 5 %, 20 sessions each
  g  a robot that spins about a point 5.8 in (robot-config.cpp's [ODOMCAL] note) or 2.24 in behind its tracking
     centre, with the inertial error +1.2 % and 0.45 % slip (umb_suite.run_square's spin_behind_in)
  h  the student's mistakes (a review found the calculator gave confident wrong numbers for them): the mark measured
     from the robot, right_in and forward_in swapped, centimetres, the two corners swapped, the heading's sign
     flipped, and a round 1 file read against a log with round 2 appended. What the student SEES (turned) is right
Closing the loop, for the (c) robots and each form: the calculator's constants go into the simulator
(--imu-scale), the squares are driven again (round 2) and so is routes/skills60.txt, before and after. The calculator
then reads the round 2 squares (the log now carries the new scales), its second suggestion goes in, and round 3 and
the route are driven again. "ideal" is the scale set exactly to 1 / (1 + the sensor's error): the best any
calibration of the scale could do.

The pass/fail limits are named constants below. They were written down before the first run and are not to be moved
to make a run pass: a failure is a result, and the report says why it failed.

What the simulator does not model (tools/sim/README.md has more): inertial sensor noise (so a heading measured by
hand scatters only by its rounding here), slip that grows with speed, sideways skid, a person who misreads the tape.
A robot that does not spin about its tracking centre is emulated only in (g).

Limits added on 2026-11-22, after a review and before the run that first used them (the earlier ones are unchanged,
except that "every calculator run exits 0" became "exits 0 or 3 with its JSON", since holding the numbers back is
now an answer): clean sessions get numbers in >= 95 %; drift gets none; the heading and corners forms never give a
spin-point robot a number more than SPIN_LIMIT_PP off; no mistake gives a number that leaves the scale further off;
the appended log stops with status 2; the truth inside the ± in at most 80 % (a ± too wide once passed); round 2
must not harm (it was reported only).
"""

import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

import numpy as np

SIM = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(SIM))
sys.path.insert(0, SIM)
import umb_suite as us  # noqa: E402  (run_square, the square's numbers, the robot's geometry, the slip levels)

CALC = os.path.join(REPO, "tools", "umbmark.py")
ROUTINE = os.path.join(REPO, "src", "umbmark.cpp")
ROUTE_60 = os.path.join(SIM, "routes", "skills60.txt")
OUT_JSON = os.path.join(SIM, "out", "umb_e2e_results.json")
PY = sys.executable

# ---- the [UMB] line, as src/umbmark.cpp prints it. Checked against the routine's printf at start-up: if the
# routine changes its line, this script stops rather than test a format the robot no longer prints
UMB_FORMAT = ("[UMB] v=1 run=%d dir=%s side=%.1f drive_speed=%d turn_speed=%d x=%.3f y=%.3f theta=%.3f "
              "still_dps=%.4f scale_cw=%.6f scale_ccw=%.6f t_ms=%lu timeouts=%d\n")
STOP_FORMAT = "[umbmark] %s run %d stopped after %d of %d moves: %s. No result\n"
STOP_REASON = "a stick moved"
STOPPED_RUN = 3  # in every session the third run is stopped by a stick: the CW runs are 1, 2, 4, 5, 6
STOPPED_AFTER_MOVES = 2
MOVES = 8
# Other lines the program prints into the same capture (src/subsystems.cpp, src/pidtuner.cpp)
OTHER_LINES = ["[PID] LVGL heap 79% used", "[optical] hue 18  saturation 0.41  proximity 37"]

# ---- what the student measures
CORNER_SPACING_IN = 12.5  # the two back corners
CORNERS_BEHIND_IN = 7.0   # how far the line joining them is behind the tracking centre
RULER_STEP_IN = 1 / 8     # every distance is read to the nearest 1/8 in
PROTRACTOR_STEP_DEG = 0.5  # the heading, in the form that has one
SEEN_DEG = 1.0             # the student writes CW or CCW in the turned column from this far; 0 under it
FORMS = ("simple", "simple_hdg", "corners")
FORM_LABEL = {"simple": "simple (no heading)", "simple_hdg": "simple + heading 0.5 deg", "corners": "corners"}

# ---- scenarios
RUNS = 5  # per direction, as on the robot
IMU_CASES = [("+1.2 %", 0.012, 0.012), ("-1.2 %", -0.012, -0.012), ("+0.3 %", 0.003, 0.003),
             ("CW +1.5 % / CCW -0.5 %", 0.015, -0.005)]
SLIP_045, SLIP_15, SLIP_5 = us.SLIP_LEVELS  # (label, SLIP_PER_IN, SLIP_MAX_IN): 0.45 %, 1.5 %, 5 % of travel
ROBOTS = 20  # per set in (c)
UNCALIBRATED = {"imu": 0.015, "wheel": 0.015, "offset_in": 0.5}  # each error drawn evenly from -this to +this
CALIBRATED = {"imu": 0.002, "wheel": 0.005, "offset_in": 0.1}
ROBOT_SEED = 20261122  # the random robots, so every run of this script tests the same ones
DRIFT_DEG_S = 0.05
SESSIONS_F = 20  # sessions per slip level in (f)
ROUTE_SEEDS = 3  # slip seeds per robot on the 60 s route (the same seeds before and after)
SEED_BASE = 700000  # slip seeds of the squares count up from here, one per run, in the order sessions are made
ROUTE_SEED_BASE = 900000

# ---- pass/fail limits, written down on 2026-11-22 before the first run, and not moved after it
LIMIT_NO_SLIP_PP = 0.15   # |found e - true e|, percentage points: no slip (a, e), every direction and form
LIMIT_SLIP045_PP = 0.30   # with 0.45 % slip (b and f at 0.45 %, and all of c): every case
LIMIT_SLIP15_PP = 0.50    # with 1.5 % slip (b and f at 1.5 %): every case. The suite's 5-run spread there is
                          # 0.12 pp, so this is about 4 of those
# The calculator's +- is a 1-sigma standard error from five runs. A right one covers the truth about 63 % of the time
# (Student's t, 4 degrees of freedom) and twice it about 88 %; the calculator's own words are "usually within it,
# nearly always within twice it". Counted over every session with slip (b, c, f), per form
COVER_1X_MIN = 0.60
COVER_2X_MIN = 0.85
LEAD_COVER_1X_MIN = 0.80  # the first design's example limit. Reported, not pass/fail: a right 1-sigma +- should miss it
FALSE_SLIP_MAX = 0.05     # at 0.45 % slip the verdict "slip ... calibration will not fix this" at most this often
TRUE_SLIP_MIN = 0.80      # at 5 % slip that verdict at least this often
LOOP_MAX_RATIO = 0.5      # uncalibrated robots, one round: median square error (each run's |E|, and the UMBmark
                          # score) after at most half of before
ROUTE_MAX_RATIO = 0.7     # ...and the 60 s route's median end error at least 30 % lower. Looser than the square:
                          # the route also carries wheel-size, offset and slip errors that the square cannot fix
CALIBRATED_MAX_RATIO = 1.25  # calibrated robots: the test must not make the median square error more than 25 % worse
SECOND_REPORTED_MAX_PP = 0.30  # after one round the calculator's own |e| at most this...
SECOND_SHARE_MIN = 0.80   # ...in at least this share of uncalibrated robot-directions, and "small" left over as often
HELP_RATIO, HARM_RATIO = 0.8, 1.25  # round 2: median |true e left| after / before under 0.8 helps, over 1.25 harms
# ---- added 2026-11-22 after the review, written down before the first run that used them
GIVEN_MIN = 0.95          # clean sessions (a, b at 0.45 %, c, f at 0.45 %): the calculator gives numbers this often
COVER_1X_MAX = 0.80       # the truth inside the ± at most this often: a ± sqrt(5) too wide passed every limit before
SPIN_BEHIND = (5.8, 2.24)  # scenario g
SPIN_LIMIT_PP = 0.15      # g: the heading and corners forms never give a number further off than this (held = fine)
MISTAKE_SCALES = (0.9881, 0.9921)  # h: the scales "round 2" was driven with, in the appended log


def f32(v):
    """A constant as the robot holds it (constexpr float)"""
    return float(np.float32(v))


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


# ------------------------------------------------------------------------------------------- the routine's numbers
def c_string_literals(text):
    return [bytes(s, "utf-8").decode("unicode_escape") for s in re.findall(r'"((?:[^"\\]|\\.)*)"', text)]


def check_routine():
    """The routine's [UMB] printf format and its square's numbers must be the ones this script and the routes use.
    Returns a list of problems (empty when all agree)"""
    with open(ROUTINE) as f:
        src = f.read()
    problems = []
    m = re.search(r'std::printf\(\s*("\[UMB\](?:[^"\\]|\\.)*"(?:\s*"(?:[^"\\]|\\.)*")*)', src)
    if not m:
        problems.append("no std::printf of a \"[UMB] ...\" line found in src/umbmark.cpp")
    elif "".join(c_string_literals(m.group(1))) != UMB_FORMAT:
        problems.append("src/umbmark.cpp prints the [UMB] line as %r, but this script writes %r"
                        % ("".join(c_string_literals(m.group(1))), UMB_FORMAT))
    m = re.search(r'std::printf\(\s*("\[umbmark\] %s run %d stopped(?:[^"\\]|\\.)*")', src)
    if not m or "".join(c_string_literals(m.group(1))) != STOP_FORMAT or ('"%s"' % STOP_REASON) not in src:
        problems.append("src/umbmark.cpp's 'stopped by a stick' line is not %r" % STOP_FORMAT)
    want = {"UMB_SIDE_IN": us.UMB_SIDE_IN, "UMB_DRIVE_MAX_SPEED": us.UMB_DRIVE_MAX_SPEED,
            "UMB_TURN_MAX_SPEED": us.UMB_TURN_MAX_SPEED, "UMB_STOP_MS": us.UMB_STOP_MS,
            "UMB_STILL_MS": us.UMB_STILL_MS, "UMB_SETTLE_MS": us.UMB_SETTLE_MS, "MOVES": MOVES,
            "UMB_DRIVE_TIMEOUT_MS": us.UMB_DRIVE_TIMEOUT_MS, "UMB_TURN_TIMEOUT_MS": us.UMB_TURN_TIMEOUT_MS}
    for name, value in want.items():
        m = re.search(r"constexpr\s+\w+\s+%s\s*=\s*([-\d.]+)f?\s*;" % name, src)
        if not m:
            problems.append("src/umbmark.cpp has no constexpr %s" % name)
        elif float(m.group(1)) != float(value):
            problems.append("src/umbmark.cpp has %s = %s, the simulator's routes use %s" % (name, m.group(1), value))
    routes_ok = us.check_routes()
    for d, ok in routes_ok.items():
        if not ok:
            problems.append("routes/umb_%s.txt is not the square umb_suite.py builds" % d.lower())
    return problems


# ------------------------------------------------------------------------------------------- what the student writes
def eighths(v):
    """v to the nearest 1/8, written as a person would: '1 3/8', '-3/4', '2', '0'"""
    n = int(round(v / RULER_STEP_IN))
    sign = "-" if n < 0 else ""
    whole, rest = divmod(abs(n), 8)
    if rest == 0:
        return f"{sign}{whole}"
    g = math.gcd(rest, 8)
    frac = f"{rest // g}/{8 // g}"
    return f"{sign}{whole} {frac}" if whole else f"{sign}{frac}"


def corner_moves(right, forward, heading_deg):
    """How far each back corner (left, then right) ended from its own tape mark, as (right, forward) in the start
    frame, when the tracking centre moved (right, forward) and the robot turned heading_deg clockwise"""
    h = math.radians(heading_deg)
    out = []
    for cx in (-CORNER_SPACING_IN / 2, CORNER_SPACING_IN / 2):
        cy = -CORNERS_BEHIND_IN
        wx = cx * math.cos(h) + cy * math.sin(h)  # the corner, turned clockwise with the robot
        wy = -cx * math.sin(h) + cy * math.cos(h)
        out.append((right + wx - cx, forward + wy - cy))
    return out


def turned_seen(heading_change):
    h = wrap180(heading_change)
    return "0" if abs(h) < SEEN_DEG else ("CW" if h > 0 else "CCW")


MISTAKES = {  # (name, the forms it is made in)
    "flip": ("the mark measured from the robot (every distance's sign flipped)", ("simple", "corners")),
    "swap_cols": ("right_in and forward_in swapped", ("simple",)),
    "cm": ("centimetres", ("simple", "simple_hdg")),
    "corners_swap": ("the two corners typed in the wrong order", ("corners",)),
    "heading_flip": ("the heading's sign flipped", ("simple_hdg",)),
}


def write_measurements(path, form, runs, run_numbers, mistake=None):
    """What the student writes. mistake (scenario h): one of MISTAKES, made in every row; turned stays right"""
    k = 2.54 if mistake == "cm" else (-1.0 if mistake == "flip" else 1.0)
    lines = ["# UMBmark square, measured by tools/sim/umb_e2e.py from the simulator's truth (the person with the",
             "# ruler): distances to the nearest 1/8 in" + (", heading to the nearest 0.5 deg." if form == "simple_hdg"
                                                            else ".") + (" MISTAKE: " + mistake if mistake else "")]
    if form == "simple":
        lines.append("dir,right_in,forward_in,turned")
    elif form == "simple_hdg":
        lines.append("dir,right_in,forward_in,heading_deg")
    else:
        lines += [f"# corner_spacing_in = {CORNER_SPACING_IN:g}", f"# corners_behind_centre_in = {CORNERS_BEHIND_IN:g}",
                  "dir,run,left_right_in,left_forward_in,right_right_in,right_forward_in,turned"]
    for r, n in zip(runs, run_numbers):
        real = r["real"]
        seen = turned_seen(real["heading_change"])
        if form == "corners":
            corners = corner_moves(real["right"], real["forward"], real["heading_change"])
            if mistake == "corners_swap":
                corners = corners[::-1]
            (lx, ly), (rx, ry) = [(k * a, k * b) for a, b in corners]
            lines.append(",".join([r["direction"], str(n), eighths(lx), eighths(ly), eighths(rx), eighths(ry), seen]))
        else:
            right, forward = k * real["right"], k * real["forward"]
            if mistake == "swap_cols":
                right, forward = forward, right
            row = [r["direction"], eighths(right), eighths(forward)]
            if form == "simple_hdg":
                h = wrap180(real["heading_change"]) * (-1 if mistake == "heading_flip" else 1)
                row.append("%g" % (round(h / PROTRACTOR_STEP_DEG) * PROTRACTOR_STEP_DEG))
            else:
                row.append(seen)
            lines.append(",".join(row))
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def run_numbers_for(n_runs):
    """Run numbers since boot: one run (STOPPED_RUN) was stopped by a stick and printed no [UMB] line"""
    nums, k = [], 1
    while len(nums) < n_runs:
        if k != STOPPED_RUN:
            nums.append(k)
        k += 1
    return nums


def write_log(path, runs, run_numbers, scales):
    out = [OTHER_LINES[0]]
    for r, n in zip(runs, run_numbers):
        if n == STOPPED_RUN + 1:
            out.append((STOP_FORMAT % (runs[0]["direction"], STOPPED_RUN, STOPPED_AFTER_MOVES, MOVES,
                                       STOP_REASON)).rstrip("\n"))
        out.append(OTHER_LINES[1])
        b = r["believed"]
        out.append((UMB_FORMAT % (n, r["direction"], float(us.UMB_SIDE_IN), us.UMB_DRIVE_MAX_SPEED,
                                  us.UMB_TURN_MAX_SPEED, b["x"], b["y"], b["theta"], r["still_dps"], f32(scales[0]),
                                  f32(scales[1]), int(round(r["square_ms"])), r["timeouts"])).rstrip("\n"))
        out.append(OTHER_LINES[1])
    with open(path, "w") as f:
        f.write("\n".join(out) + "\n")


def calculate(csv_path, log_path, json_path):
    """tools/umbmark.py as a student runs it. Returns (exit status, its JSON or None, stdout, stderr)"""
    done = subprocess.run([PY, CALC, csv_path, "--log", log_path, "--json", json_path], capture_output=True,
                          text=True)
    js = None
    if done.returncode in (0, 3):  # 3: it ran, and held its numbers back
        with open(json_path) as f:
            js = json.load(f)
    return done.returncode, js, done.stdout, done.stderr


# ------------------------------------------------------------------------------------------- the simulator
def scale_args(scales):
    if scales == (1.0, 1.0):
        return []
    return ["--imu-scale", f"{scales[0]:.4f}:{scales[1]:.4f}"]


def run_route(args, seed, scales):
    """routes/skills60.txt with the robot's geometry and its errors. Returns the odometry error at the end and the
    largest on the way (inches)"""
    cmd = [us.TB, "--route", ROUTE_60, "--no-gps", "--fusion", "off", "--seed", str(seed)] + us.geometry_args() + \
        list(args) + scale_args(scales) + ["--summary-json"]
    done = subprocess.run(cmd, capture_output=True, text=True)
    if done.returncode != 0:
        raise RuntimeError(f"{' '.join(cmd)}\n{done.stderr}")
    s = json.loads(done.stdout)
    return {"end_in": s["odom"]["end"], "max_in": s["odom"]["max"], "timeouts": s["timeouts"]}


def imu_args(cw, ccw):
    return ["--set", f"IMU_SCALE_ERR_CW={cw}", "--set", f"IMU_SCALE_ERR_CCW={ccw}"]


class Seeds:
    def __init__(self, base):
        self.next = base

    def take(self, n):
        out = list(range(self.next, self.next + n))
        self.next += n
        return out


def session(key, scenario, label, args, e_sensor, scales, seeds, slip=None, robot=None, rnd=1, form=None, spin=0.0):
    return {"key": key, "scenario": scenario, "label": label, "args": list(args), "e_sensor": tuple(e_sensor),
            "scales": tuple(scales), "seeds": {"CW": seeds.take(RUNS), "CCW": seeds.take(RUNS)}, "slip": slip,
            "robot": robot, "round": rnd, "form": form, "spin": spin}


def random_robots(sizes, n, rng):
    """n robots, each error drawn evenly between -size and +size, independently"""
    robots = []
    for _ in range(n):
        e_cw, e_ccw = rng.uniform(-sizes["imu"], sizes["imu"], 2)
        ws_v, ws_h = rng.uniform(-sizes["wheel"], sizes["wheel"], 2)
        dv, dh = rng.uniform(-sizes["offset_in"], sizes["offset_in"], 2)
        robots.append({"e_cw": float(e_cw), "e_ccw": float(e_ccw), "vert_scale_err": float(ws_v),
                       "horiz_scale_err": float(ws_h), "vert_offset_err_in": float(dv), "horiz_offset_err_in": float(dh)})
    return robots


def robot_args(rb):
    # the world's wheels sit "err" further right (vertical) / forward (horizontal) than LemLib is told
    return imu_args(rb["e_cw"], rb["e_ccw"]) + [
        "--set", f"VERT_SCALE_ERR={rb['vert_scale_err']}", "--set", f"HORIZ_SCALE_ERR={rb['horiz_scale_err']}",
        "--set", f"VERT_OFFSET={us.ROBOT_VERT_OFFSET_IN + rb['vert_offset_err_in']}",
        "--set", f"HORIZ_OFFSET={us.ROBOT_HORIZ_OFFSET_IN + rb['horiz_offset_err_in']}"] + \
        us.slip_args(SLIP_045[1], SLIP_045[2])


def round1_sessions(seeds, robots_n, sessions_f, rng):
    out = []
    for label, cw, ccw in IMU_CASES:
        out.append(session(f"a {label}", "a", f"inertial {label}, no slip", imu_args(cw, ccw), (cw, ccw), (1.0, 1.0),
                           seeds))
    for slip in (SLIP_045, SLIP_15):
        for label, cw, ccw in IMU_CASES:
            out.append(session(f"b {label}, slip {slip[0]}", "b", f"inertial {label}, slip {slip[0]}",
                               imu_args(cw, ccw) + us.slip_args(slip[1], slip[2]), (cw, ccw), (1.0, 1.0), seeds,
                               slip=slip[0]))
    robots = {}
    for name, sizes in (("uncalibrated", UNCALIBRATED), ("calibrated", CALIBRATED)):
        robots[name] = random_robots(sizes, robots_n, rng)
        for i, rb in enumerate(robots[name]):
            out.append(session(f"c {name} {i + 1}", "c", f"{name} robot {i + 1}", robot_args(rb),
                               (rb["e_cw"], rb["e_ccw"]), (1.0, 1.0), seeds, slip=SLIP_045[0], robot=(name, i)))
    out.append(session("d drift", "d", f"drift {DRIFT_DEG_S:g} deg/s, no slip", ["--set", f"IMU_DRIFT_DEG_S={DRIFT_DEG_S}"],
                       (0, 0), (1.0, 1.0), seeds))
    out.append(session("e horizontal sign", "e", f"horizontal offset wrong sign (really {-us.ROBOT_HORIZ_OFFSET_IN:+g}, "
                       f"told {us.ROBOT_HORIZ_OFFSET_IN:g}), no slip",
                       ["--set", f"HORIZ_OFFSET={-us.ROBOT_HORIZ_OFFSET_IN}"], (0, 0), (1.0, 1.0), seeds))
    for slip in (SLIP_045, SLIP_15, SLIP_5):
        for k in range(sessions_f):
            out.append(session(f"f slip {slip[0]} #{k + 1}", "f", f"slip {slip[0]} alone", us.slip_args(slip[1], slip[2]),
                               (0, 0), (1.0, 1.0), seeds, slip=slip[0]))
    for D in SPIN_BEHIND:
        out.append(session(f"g spin {D:g} in", "g", f"spins {D:g} in behind, inertial +1.2 %, slip {SLIP_045[0]}",
                           imu_args(0.012, 0.012) + us.slip_args(SLIP_045[1], SLIP_045[2]), (0.012, 0.012), (1.0, 1.0),
                           seeds, slip=SLIP_045[0], spin=D))
    return out, robots


def drive(sessions, jobs):
    """Every session's squares, in parallel. Fills session["runs"] = {"CW": [...], "CCW": [...]}"""
    jl = [(i, d, s) for i, ss in enumerate(sessions) for d in ("CW", "CCW") for s in ss["seeds"][d]]

    def one(j):
        ss = sessions[j[0]]
        return us.run_square(j[1], j[2], ss["args"] + scale_args(ss["scales"]), spin_behind_in=ss["spin"])

    res = us.parallel(one, jl, jobs)
    for ss in sessions:
        ss["runs"] = {"CW": [], "CCW": []}
    for (i, d, _), r in zip(jl, res):
        sessions[i]["runs"][d].append(r)


def session_folder(work, ss):
    return os.path.join(work, re.sub(r"[^A-Za-z0-9_.+-]+", "_", ss["key"]) + f"_r{ss['round']}")


def measure_and_calculate(sessions, forms, work, jobs):
    """Writes each session's log and measurement files, runs the calculator on each form. Fills session["calc"]"""
    def one(job):
        ss, form = job
        folder = session_folder(work, ss)
        runs = ss["runs"]["CW"] + ss["runs"]["CCW"]
        log = os.path.join(folder, "tune.log")
        csv = os.path.join(folder, f"{form}.csv")
        write_measurements(csv, form, runs, run_numbers_for(len(runs)))
        code, js, out, err = calculate(csv, log, os.path.join(folder, f"{form}.json"))
        with open(os.path.join(folder, f"{form}.txt"), "w") as f:
            f.write(out + err)
        return code, js, out, err

    jl = [(ss, form) for ss in sessions for form in forms]
    # the log is written once per session, before the forms run in parallel
    for ss in sessions:
        folder = session_folder(work, ss)
        os.makedirs(folder, exist_ok=True)
        runs = ss["runs"]["CW"] + ss["runs"]["CCW"]
        write_log(os.path.join(folder, "tune.log"), runs, run_numbers_for(len(runs)), ss["scales"])
    res = us.parallel(one, jl, jobs)
    for (ss, form), (code, js, out, err) in zip(jl, res):
        ss.setdefault("calc", {})[form] = evaluate(ss, code, js, err)


def e_true(ss, d):
    """The error of the heading LemLib uses, after the scale: (1 + the sensor's error) x scale - 1"""
    k = 0 if d == "CW" else 1
    return (1 + ss["e_sensor"][k]) * f32(ss["scales"][k]) - 1


def evaluate(ss, code, js, err):
    out = {"exit": code, "stderr": err.strip()}
    if js is None:
        return out
    c = js["constants"]
    out["held"] = c is None
    out["held_back"] = js.get("held_back", [])
    out["constants"] = None if c is None else (c["IMU_SCALE_CLOCKWISE"], c["IMU_SCALE_COUNTER_CLOCKWISE"])
    out["settled"] = js.get("settled")
    out["warnings"], out["hints"] = js["warnings"], js["hints"]
    out["still_warning"] = bool(js["still"] and js["still"]["warning"])
    out["score_now_in"] = js["umbmark_score"]["now_in"]
    out["score_predicted_after_in"] = js["umbmark_score"]["predicted_after_in"]
    out["dirs"] = {}
    for d in ("CW", "CCW"):
        x = js["directions"][d]
        t = e_true(ss, d)
        se = x["e_used_se"]
        e = x["e_used"] - t
        out["dirs"][d] = {
            "e_true": t, "e_found": x["e_used"], "se": se, "err": e,
            "given": c is not None and x["constant_given"],
            "inside_1x": se is not None and abs(e) <= se, "inside_2x": se is not None and abs(e) <= 2 * se,
            "from": x["e_used_from"], "agree": x["agree"], "e_position": x["e_position"],
            "e_position_se": x["e_position_se"], "e_heading": x["e_heading"], "e_heading_se": x["e_heading_se"],
            "scatter_rms_in": x["scatter_rms_in"], "scatter_max_in": x["scatter_max_in"],
            "slip_class": x["slip_class"], "leftover_in": x["leftover_in"], "leftover_small": x["leftover_small"],
            "mean_error_in": x["mean_error_in"], "scale_new": x["scale_new"],
            # added to the calculator on 2026-11-22 after the first full run here; None from an older calculator
            "within_scatter": x.get("within_scatter")}
    return out


def truth(ss):
    """The simulator's own end errors for a session: every run's |E| and each direction's cluster centre"""
    out = {"E_runs": [], "centre": {}}
    for d in ("CW", "CCW"):
        E = np.array([r["E"] for r in ss["runs"][d]])
        out["E_runs"] += [float(v) for v in np.hypot(*E.T)]
        out["centre"][d] = float(np.hypot(*E.mean(axis=0)))
    out["score"] = max(out["centre"].values())  # the UMBmark score, from the truth
    out["timeouts"] = sum(r["timeouts"] for d in ("CW", "CCW") for r in ss["runs"][d])
    return out


# ------------------------------------------------------------------------------------------- the student's mistakes
MISTAKE_E = (0.012, 0.008)  # scenario h's robot: inertial +1.2 % clockwise, +0.8 % counter-clockwise


def run_mistakes(work, jobs, seeds, n_sessions=2):
    """Scenario h: each mistake in each form it is made in, on n_sessions sessions of one robot, plus a round 1 file
    read against a log with round 2 (other scales) appended. Returns a list of records"""
    sess = [session(f"h mistakes #{k + 1}", "h", "mistakes", imu_args(*MISTAKE_E) + us.slip_args(SLIP_045[1], SLIP_045[2]),
                    MISTAKE_E, (1.0, 1.0), seeds, slip=SLIP_045[0]) for k in range(n_sessions)]
    drive(sess, jobs)
    jl = []
    for k, ss in enumerate(sess):
        folder = session_folder(work, ss)
        os.makedirs(folder, exist_ok=True)
        runs = ss["runs"]["CW"] + ss["runs"]["CCW"]
        nums = run_numbers_for(len(runs))
        write_log(os.path.join(folder, "tune.log"), runs, nums, (1.0, 1.0))
        for m, (_, forms) in MISTAKES.items():
            for form in forms:
                jl.append((k, ss, m, form))
        # round 2 appended: the next session's runs, driven (as far as the log says) with other scales
        other = sess[(k + 1) % len(sess)]
        orun = other["runs"]["CW"] + other["runs"]["CCW"]
        write_log(os.path.join(folder, "round2.log"), orun, run_numbers_for(len(orun)), MISTAKE_SCALES)
        with open(os.path.join(folder, "appended.log"), "w") as f:
            f.write(open(os.path.join(folder, "tune.log")).read() + open(os.path.join(folder, "round2.log")).read())
        jl.append((k, ss, "appended_log", "simple"))

    def one(job):
        k, ss, m, form = job
        folder = session_folder(work, ss)
        runs = ss["runs"]["CW"] + ss["runs"]["CCW"]
        csv = os.path.join(folder, f"{m}_{form}.csv")
        write_measurements(csv, form, runs, run_numbers_for(len(runs)), None if m == "appended_log" else m)
        log = os.path.join(folder, "appended.log" if m == "appended_log" else "tune.log")
        code, js, out, err = calculate(csv, log, os.path.join(folder, f"{m}_{form}.json"))
        with open(os.path.join(folder, f"{m}_{form}.txt"), "w") as f:
            f.write(out + err)
        rec = {"session": k + 1, "mistake": m, "form": form, "exit": code, "stderr": err.strip(), "held": None,
               "constants": None, "before_pp": [100 * abs(e) for e in MISTAKE_E], "after_pp": None, "worse": False}
        if js is not None:
            rec["held"] = js["constants"] is None
            rec["held_back"] = js["held_back"]
            if js["constants"] is not None:
                c = (js["constants"]["IMU_SCALE_CLOCKWISE"], js["constants"]["IMU_SCALE_COUNTER_CLOCKWISE"])
                rec["constants"] = c
                rec["after_pp"] = [100 * abs((1 + e) * f32(sc) - 1) for e, sc in zip(MISTAKE_E, c)]
                rec["worse"] = any(a > b for a, b in zip(rec["after_pp"], rec["before_pp"]))
        return rec

    return us.parallel(one, jl, jobs)


# ------------------------------------------------------------------------------------------- closing the loop
def loop_sessions(robot_sets, prev, seeds, rnd, forms):
    """For every (c) robot and form, a session driven with the constants the calculator gave on `prev` (a dict
    (set, robot, form) -> session). rnd 2 or 3"""
    out = {}
    for name, robots in robot_sets.items():
        for i, rb in enumerate(robots):
            for form in forms:
                p = prev[(name, i, form)]
                if "dirs" not in p["calc"][form]:
                    continue
                # numbers held back: the student keeps the scales the robot has
                c = p["calc"][form].get("constants") or p["scales"]
                out[(name, i, form)] = session(f"c {name} {i + 1} {form}", "c", f"{name} robot {i + 1}, round {rnd}",
                                               robot_args(rb), (rb["e_cw"], rb["e_ccw"]), (c[0], c[1]), seeds,
                                               slip=SLIP_045[0], robot=(name, i), rnd=rnd, form=form)
    return out


def ideal_scales(rb):
    return (float("%.4f" % (1 / (1 + rb["e_cw"]))), float("%.4f" % (1 / (1 + rb["e_ccw"]))))


# ------------------------------------------------------------------------------------------- summaries
def pct(v, nd=2):
    return f"{100 * v:+.{nd}f}"


def share(xs):
    xs = list(xs)
    return sum(1 for x in xs if x) / len(xs) if xs else float("nan")


def cases(sessions, form, pick, given_only=True):
    """Every (session, direction) record of one form for the sessions `pick` accepts. given_only: only those the
    calculator gave a number for (a held-back one gives the student nothing to type in)"""
    out = []
    for ss in sessions:
        if pick(ss) and "dirs" in ss["calc"].get(form, {}):
            for d in ("CW", "CCW"):
                x = ss["calc"][form]["dirs"][d]
                if x["given"] or not given_only:
                    out.append((ss, d, x))
    return out


def held_share(sessions, form, pick):
    """Share of the sessions `pick` accepts whose numbers the calculator held back, and how many sessions"""
    cs = [ss["calc"][form] for ss in sessions if pick(ss) and "dirs" in ss["calc"].get(form, {})]
    return (share(c["held"] for c in cs) if cs else float("nan")), len(cs)


def scenario_table(sessions, forms):
    lines = ["E = real minus believed. 'e true' is the error of the heading LemLib uses (the sensor's error, scale 1.0);",
             "'found' is what tools/umbmark.py used for its constants, 'error' = found - true, all in percent;",
             "'+-' its stated 1-sigma; 'in' = truth inside +- (1x) / inside 2x; 'from' = position, heading or both;",
             "'scatter' its RMS scatter (in); 'left' its leftover (in), * = it called it small.",
             f"{'session':44s} {'form':10s} {'dir':4s} {'e true':>7s} {'found':>7s} {'error':>7s} {'+-':>5s} "
             f"{'in':>4s} {'from':9s} {'scatter':>7s} {'verdict':11s} {'left':>6s}  notes"]
    for ss in sessions:
        for form in forms:
            c = ss["calc"][form]
            if "dirs" not in c:
                lines.append(f"{ss['label'][:44]:44s} {form:10s} calculator failed (exit {c['exit']}): {c['stderr']}")
                continue
            for d in ("CW", "CCW"):
                x = c["dirs"][d]
                inside = ("1x" if x["inside_1x"] else ("2x" if x["inside_2x"] else "no"))
                notes = []
                if d == "CW":
                    if c["still_warning"]:
                        notes.append("drift warning")
                    if c["warnings"]:
                        notes.append(f"{len(c['warnings'])} warning(s)")
                    if c["hints"]:
                        notes.append(f"{len(c['hints'])} hint(s)")
                if x["from"] == "heading":
                    notes.append("DISAGREE")
                if d == "CW" and c["held"]:
                    notes.append("HELD BACK (no numbers)")
                lines.append(f"{(ss['label'] if d == 'CW' and form == forms[0] else '')[:44]:44s} "
                             f"{(form if d == 'CW' else ''):10s} {d:4s} {pct(x['e_true']):>7s} {pct(x['e_found']):>7s} "
                             f"{pct(x['err']):>7s} {100 * (x['se'] or 0):5.2f} {inside:>4s} {x['from']:9s} "
                             f"{x['scatter_rms_in']:7.2f} {x['slip_class']:11s} {x['leftover_in']:5.2f}"
                             f"{'*' if x['leftover_small'] else ' '}  {', '.join(notes)}")
    return lines


def group_stats(recs, held=float("nan")):
    errs = np.array([abs(x["err"]) for _, _, x in recs])
    if not len(errs):
        return None
    return {"n": len(recs), "held": held, "median_abs_err_pp": float(100 * np.median(errs)), "p95_abs_err_pp":
            float(100 * np.percentile(errs, 95)), "max_abs_err_pp": float(100 * errs.max()),
            "cover_1x": share(x["inside_1x"] for _, _, x in recs), "cover_2x": share(x["inside_2x"] for _, _, x in recs),
            "median_se_pp": float(100 * np.median([x["se"] for _, _, x in recs])),
            "median_scatter_in": float(np.median([x["scatter_rms_in"] for _, _, x in recs])),
            "verdicts": {k: sum(1 for _, _, x in recs if x["slip_class"] == k)
                         for k in ("repeatable", "borderline", "slip", "unknown")},
            "median_leftover_in": float(np.median([x["leftover_in"] for _, _, x in recs])),
            "leftover_small": share(x["leftover_small"] for _, _, x in recs),
            "from": {k: sum(1 for _, _, x in recs if x["from"] == k) for k in ("position", "heading", "both")}}


def stats_line(name, st):
    v = st["verdicts"]
    return (f"{name:34s} {st['n']:4d} {st['median_abs_err_pp']:7.3f} {st['p95_abs_err_pp']:7.3f} "
            f"{st['max_abs_err_pp']:7.3f} {100 * st['cover_1x']:5.0f}% {100 * st['cover_2x']:5.0f}% "
            f"{st['median_se_pp']:6.3f} {st['median_scatter_in']:7.2f}  {v['repeatable']:3d}/{v['borderline']:3d}/"
            f"{v['slip']:3d}  {st['median_leftover_in']:5.2f} {100 * st['leftover_small']:4.0f}%  "
            f"{st['from']['position']}/{st['from']['heading']}/{st['from']['both']}  {100 * st['held']:5.0f}%")


STATS_HEADER = (f"{'group':34s} {'n':>4s} {'med|err|':>7s} {'95%':>7s} {'max':>7s} {'in 1x':>6s} {'in 2x':>6s} "
                f"{'med+-':>6s} {'scatter':>7s}  rep/bord/slip  left  small  pos/hdg/both  held")


# ------------------------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
    ap.add_argument("--work", help="keep every generated log, measurement file and calculator output here")
    ap.add_argument("--robots", type=int, default=ROBOTS, help="random robots per set in (c)")
    ap.add_argument("--sessions-f", type=int, default=SESSIONS_F, help="sessions per slip level in (f)")
    ap.add_argument("--route-seeds", type=int, default=ROUTE_SEEDS)
    ap.add_argument("--quick", action="store_true", help="4 robots per set, 4 sessions per slip level, 1 route seed")
    ap.add_argument("--out", default=OUT_JSON, help="results JSON (default tools/sim/out/umb_e2e_results.json)")
    o = ap.parse_args()
    if o.quick:
        o.robots, o.sessions_f, o.route_seeds = 4, 4, 1
    if not os.path.exists(us.TB):
        print(f"no {us.TB}: run make -C tools/sim first", file=sys.stderr)
        return 2
    t0 = time.time()
    problems = check_routine()
    if problems:
        print("The robot's routine, the routes and this script do not agree:\n  " + "\n  ".join(problems),
              file=sys.stderr)
        return 2
    print("src/umbmark.cpp: the [UMB] format, the stopped-run line and the square's numbers match this script and "
          "routes/umb_*.txt")
    work = o.work or tempfile.mkdtemp(prefix="umb_e2e_")
    os.makedirs(work, exist_ok=True)
    try:
        return run_all(o, work, t0)
    finally:
        if not o.work:
            shutil.rmtree(work, ignore_errors=True)


def run_all(o, work, t0):
    seeds = Seeds(SEED_BASE)
    rng = np.random.RandomState(ROBOT_SEED)
    r1, robot_sets = round1_sessions(seeds, o.robots, o.sessions_f, rng)
    drive(r1, o.jobs)
    measure_and_calculate(r1, FORMS, work, o.jobs)
    print(f"round 1: {len(r1)} sessions, {sum(len(s['runs']['CW']) + len(s['runs']['CCW']) for s in r1)} squares, "
          f"{len(r1) * len(FORMS)} calculator runs ({time.time() - t0:.0f} s)")

    # ---- closing the loop on the (c) robots
    first = {(ss["robot"][0], ss["robot"][1], form): ss for ss in r1 if ss["scenario"] == "c" for form in FORMS}
    r2 = loop_sessions(robot_sets, first, seeds, 2, FORMS)
    drive(list(r2.values()), o.jobs)
    for form in FORMS:  # each round 2 session is read with the form it was calibrated from
        measure_and_calculate([s for k, s in r2.items() if k[2] == form], (form,), work, o.jobs)
    r3 = loop_sessions(robot_sets, r2, seeds, 3, FORMS)
    drive(list(r3.values()), o.jobs)
    ideal = {}
    for name, robots in robot_sets.items():
        for i, rb in enumerate(robots):
            ideal[(name, i)] = session(f"c {name} {i + 1} ideal", "c", f"{name} robot {i + 1}, ideal scale",
                                       robot_args(rb), (rb["e_cw"], rb["e_ccw"]), ideal_scales(rb), seeds,
                                       slip=SLIP_045[0], robot=(name, i), rnd=0)
    drive(list(ideal.values()), o.jobs)
    # the 60 s route: before, ideal, after round 1 and after round 2 of each form, the same slip seeds each time
    route_jobs = []
    for name, robots in robot_sets.items():
        for i, rb in enumerate(robots):
            variants = [("before", (1.0, 1.0)), ("ideal", ideal_scales(rb))]
            for form in FORMS:
                if (name, i, form) in r2:
                    variants.append((f"r1 {form}", r2[(name, i, form)]["scales"]))
                if (name, i, form) in r3:
                    variants.append((f"r2 {form}", r3[(name, i, form)]["scales"]))
            for label, sc in variants:
                for k in range(o.route_seeds):
                    route_jobs.append((name, i, label, k, robot_args(rb), ROUTE_SEED_BASE + 100 * i + k, sc))
    route_res = us.parallel(lambda j: run_route(j[4], j[5], j[6]), route_jobs, o.jobs)
    routes = {}
    for j, r in zip(route_jobs, route_res):
        routes.setdefault((j[0], j[1], j[2]), []).append(r)
    print(f"closing the loop: {len(r2) + len(r3) + len(ideal)} more sessions, {len(route_jobs)} runs of the 60 s route "
          f"({time.time() - t0:.0f} s)")

    mistakes = run_mistakes(work, o.jobs, seeds)
    results = report(o, r1, r2, r3, ideal, robot_sets, routes, mistakes)
    results["elapsed_s"] = time.time() - t0
    results["work"] = o.work
    os.makedirs(os.path.dirname(o.out), exist_ok=True)
    with open(o.out, "w") as f:
        json.dump(results, f, indent=1, default=str)
    print(f"\nresults: {o.out}" + (f"\nfiles: {o.work}" if o.work else ""))
    failed = [lim for lim in results["limits"] if lim["pass"] is False]
    return 1 if failed else 0


def report(o, r1, r2, r3, ideal, robot_sets, routes, mistakes):
    res = {"settings": {"runs_per_direction": RUNS, "robots_per_set": o.robots, "sessions_f": o.sessions_f,
                        "route_seeds": o.route_seeds, "uncalibrated": UNCALIBRATED, "calibrated": CALIBRATED,
                        "robot_seed": ROBOT_SEED, "corner_spacing_in": CORNER_SPACING_IN,
                        "corners_behind_in": CORNERS_BEHIND_IN, "ruler_step_in": RULER_STEP_IN,
                        "protractor_step_deg": PROTRACTOR_STEP_DEG},
           "robots": robot_sets, "limits": []}
    limits = res["limits"]

    def limit(name, value, ok, note="", reported_only=False):
        limits.append({"name": name, "value": value, "pass": None if reported_only else bool(ok),
                       "met": bool(ok), "note": note})

    # ---- every calculator run must work on valid input
    calc_runs = [(ss, f) for ss in r1 + list(r2.values()) for f in ss.get("calc", {})]
    bad = [(ss["key"], f, ss["calc"][f]["exit"], ss["calc"][f]["stderr"]) for ss, f in calc_runs
           if ss["calc"][f]["exit"] not in (0, 3) or "dirs" not in ss["calc"][f]]
    limit("every calculator run gives its JSON, status 0 (numbers) or 3 (held back)",
          f"{len(calc_runs) - len(bad)} of {len(calc_runs)}", not bad,
          "; ".join(f"{k} {f}: {e}" for k, f, _, e in bad[:3]))
    res["calculator_failures"] = bad

    print("\n== 1. Scenarios a, b, d, e, and the first session of each slip level in f (round 1, scales 1.0) ==")
    shown = [ss for ss in r1 if ss["scenario"] in "abdeg" or (ss["scenario"] == "f" and ss["key"].endswith("#1"))]
    for line in scenario_table(shown, FORMS):
        print(line)
    for ss in shown:
        t = truth(ss)
        ss["truth"] = t
    res["scenarios"] = [{"key": ss["key"], "label": ss["label"], "e_sensor": ss["e_sensor"], "truth": truth(ss),
                         "calc": ss["calc"]} for ss in shown]

    # ---- the student's mistakes (h)
    print("\n== 1b. Scenario h: the student's mistakes (inertial +1.2 % CW, +0.8 % CCW, slip 0.45 %; turned seen "
          "right) ==")
    print(f"{'mistake':58s} {'form':10s} {'run':>3s} {'exit':>4s}  result")
    for m in mistakes:
        label = MISTAKES[m["mistake"]][0] if m["mistake"] in MISTAKES else "round 1 file, log with round 2 appended"
        if m["constants"] is not None:
            result = (f"NUMBERS {m['constants'][0]:.4f} / {m['constants'][1]:.4f}: scale error "
                      f"{m['before_pp'][0]:.2f}/{m['before_pp'][1]:.2f} -> {m['after_pp'][0]:.2f}/{m['after_pp'][1]:.2f} pp"
                      + ("  WORSE" if m["worse"] else ""))
        elif m["held"]:
            result = "held back: " + (m["held_back"][0][:90] if m["held_back"] else "?")
        else:
            result = "stopped: " + m["stderr"][:100]
        print(f"{label[:58]:58s} {m['form']:10s} {m['session']:3d} {m['exit']:4d}  {result}")
    res["mistakes"] = mistakes
    made = [m for m in mistakes if m["mistake"] != "appended_log"]
    worse = [m for m in made if m["worse"] or m["exit"] not in (0, 3)]
    limit("mistakes (h): no number that leaves the scale error further from 0",
          f"{len(made) - len(worse)} of {len(made)} fine ({sum(1 for m in made if m['held'])} held back, "
          f"{sum(1 for m in made if m['constants'] is not None)} gave numbers)", not worse,
          "; ".join(f"{m['mistake']} {m['form']}" for m in worse[:4]))
    app = [m for m in mistakes if m["mistake"] == "appended_log"]
    ok_app = [m for m in app if m["exit"] == 2 and "--session" in m["stderr"]]
    limit("mistakes (h): a round 1 file against a log with round 2 appended stops (status 2, asks for --session)",
          f"{len(ok_app)} of {len(app)}", len(ok_app) == len(app))

    # ---- groups
    def pick(scen=None, slip=None, robot_set=None):
        def f(ss):
            return ((scen is None or ss["scenario"] in scen) and (slip is None or ss["slip"] == slip) and
                    (robot_set is None or (ss["robot"] and ss["robot"][0] == robot_set)))
        return f

    groups = [("a  inertial only, no slip", pick("a")), ("e  horizontal sign, no slip", pick("e")),
              ("d  drift 0.05 deg/s (true e = 0)", pick("d")),
              ("b  inertial, slip 0.45 %", pick("b", SLIP_045[0])), ("b  inertial, slip 1.5 %", pick("b", SLIP_15[0])),
              ("c  uncalibrated robots", pick("c", robot_set="uncalibrated")),
              ("c  calibrated robots", pick("c", robot_set="calibrated")),
              ("f  slip 0.45 % alone", pick("f", SLIP_045[0])), ("f  slip 1.5 % alone", pick("f", SLIP_15[0])),
              ("f  slip 5 % alone", pick("f", SLIP_5[0])), ("g  spin point, inertial +1.2 %", pick("g"))]
    r1c = [ss for ss in r1]
    res["groups"] = {}
    print("\n== 2. Every round 1 group, per form: |found - true| in percentage points, coverage of the stated +-, "
          "verdicts ==")
    for form in FORMS:
        print(f"\n-- form: {FORM_LABEL[form]}")
        print(STATS_HEADER)
        for name, p in groups:
            recs = cases(r1c, form, p)
            held, n_sessions = held_share(r1c, form, p)
            st = group_stats(recs, held)
            if st:
                res["groups"][f"{form} | {name}"] = st
                print(stats_line(name, st))
            elif n_sessions:
                res["groups"][f"{form} | {name}"] = {"n": 0, "held": held}
                print(f"{name:34s}    0  (every session held back: {n_sessions})")

    # ---- why a heading read by hand can be over-trusted here: in the simulator the robot's real heading at the end
    # hardly changes from run to run (no inertial noise, and slip never touches the heading), so five readings to
    # the nearest PROTRACTOR_STEP_DEG are often all the same and the calculator sees no scatter in them
    spreads, same = [], []
    for ss in r1:
        if ss["slip"] is None:
            continue
        for d in ("CW", "CCW"):
            h = [wrap180(r["real"]["heading_change"]) for r in ss["runs"][d]]
            spreads.append(float(np.std(h, ddof=1)))
            same.append(len({round(v / PROTRACTOR_STEP_DEG) for v in h}) == 1)
    res["heading_spread"] = {"median_sd_deg": float(np.median(spreads)), "max_sd_deg": float(max(spreads)),
                             "all_five_read_the_same": share(same), "n": len(same)}
    print(f"\nReal heading at the end, spread between the 5 runs of a session with slip: median sd "
          f"{np.median(spreads):.3f} deg (largest {max(spreads):.3f}); all five read the same to "
          f"{PROTRACTOR_STEP_DEG:g} deg in {100 * share(same):.0f} % of {len(same)} session-directions")

    # ---- (c) robot by robot, simple form, so the team can see single cases
    print("\n== 3. The (c) robots, one by one (round 1): true e and found e per direction, % ==")
    print(f"{'robot':16s} {'e_cw':>6s} {'e_ccw':>6s} {'wheel v/h %':>12s} {'offset v/h in':>14s}   "
          + "   ".join(f"{FORM_LABEL[f][:10]:>10s} cw / ccw err" for f in FORMS))
    for name, robots in robot_sets.items():
        for i, rb in enumerate(robots):
            ss = next(s for s in r1 if s["robot"] == (name, i))
            cols = []
            for form in FORMS:
                c = ss["calc"][form]
                cols.append(f"{pct(c['dirs']['CW']['err']):>7s} / {pct(c['dirs']['CCW']['err']):>6s}" if "dirs" in c
                            else "failed")
            print(f"{name[:5]} {i + 1:2d}         {pct(rb['e_cw']):>6s} {pct(rb['e_ccw']):>6s} "
                  f"{pct(rb['vert_scale_err'], 1):>5s}/{pct(rb['horiz_scale_err'], 1):>5s} "
                  f"{rb['vert_offset_err_in']:+6.2f}/{rb['horiz_offset_err_in']:+6.2f}   " + "   ".join(
                      f"{c:>23s}" for c in cols))

    # ---- limits on round 1. The error and coverage limits count the sessions that gave numbers; the verdict
    # limits count every session (the verdict is printed either way)
    def worst_of(recs):
        return (max(abs(x["err"]) for _, _, x in recs), len(recs)) if recs else (0.0, 0)

    bcf = "bcf"  # the limits below were defined over these scenarios; g and h are not in them
    clean = lambda ss: ss["scenario"] == "a" or (ss["scenario"] in bcf and ss["slip"] == SLIP_045[0])
    for form in FORMS:
        worst, n = worst_of(cases(r1, form, pick("ae")))
        limit(f"[{form}] no slip (a, e): every |error| <= {LIMIT_NO_SLIP_PP} pp", f"worst {100 * worst:.3f} pp of {n}",
              100 * worst <= LIMIT_NO_SLIP_PP)
        worst, n = worst_of(cases(r1, form, lambda ss: ss["slip"] == SLIP_045[0] and ss["scenario"] in bcf))
        limit(f"[{form}] 0.45 % slip (b, c, f): every |error| <= {LIMIT_SLIP045_PP} pp",
              f"worst {100 * worst:.3f} pp of {n}", 100 * worst <= LIMIT_SLIP045_PP)
        worst, n = worst_of(cases(r1, form, lambda ss: ss["slip"] == SLIP_15[0] and ss["scenario"] in "bf"))
        limit(f"[{form}] 1.5 % slip (b, f): every |error| <= {LIMIT_SLIP15_PP} pp",
              f"worst {100 * worst:.3f} pp of {n}", 100 * worst <= LIMIT_SLIP15_PP)
        recs = cases(r1, form, lambda ss: ss["slip"] is not None and ss["scenario"] in bcf)
        c1, c2 = share(x["inside_1x"] for _, _, x in recs), share(x["inside_2x"] for _, _, x in recs)
        limit(f"[{form}] slip cases (b, c, f): truth inside +- in >= {100 * COVER_1X_MIN:.0f} %", f"{100 * c1:.0f} % "
              f"of {len(recs)}", c1 >= COVER_1X_MIN)
        limit(f"[{form}] slip cases (b, c, f): truth inside 2x +- in >= {100 * COVER_2X_MIN:.0f} %",
              f"{100 * c2:.0f} % of {len(recs)}", c2 >= COVER_2X_MIN)
        limit(f"[{form}] slip cases (b, c, f): truth inside +- in <= {100 * COVER_1X_MAX:.0f} % (a +- too wide fails)",
              f"{100 * c1:.0f} % of {len(recs)}", c1 <= COVER_1X_MAX)
        limit(f"[{form}] first design's example: truth inside +- in >= {100 * LEAD_COVER_1X_MIN:.0f} %", f"{100 * c1:.0f} %",
              c1 >= LEAD_COVER_1X_MIN, "reported only: a right 1-sigma from 5 runs covers about 63 %",
              reported_only=True)
        recs = cases(r1, form, lambda ss: ss["slip"] == SLIP_045[0] and ss["scenario"] in bcf, given_only=False)
        fs = share(x["slip_class"] == "slip" for _, _, x in recs)
        limit(f"[{form}] 0.45 % slip: verdict 'slip' in <= {100 * FALSE_SLIP_MAX:.0f} %", f"{100 * fs:.1f} % of "
              f"{len(recs)}", fs <= FALSE_SLIP_MAX)
        recs = cases(r1, form, lambda ss: ss["slip"] == SLIP_5[0], given_only=False)
        ts = share(x["slip_class"] == "slip" for _, _, x in recs)
        limit(f"[{form}] 5 % slip: verdict 'slip' in >= {100 * TRUE_SLIP_MIN:.0f} %", f"{100 * ts:.0f} % of "
              f"{len(recs)}", ts >= TRUE_SLIP_MIN)
        dss = [ss for ss in r1 if ss["scenario"] == "d"]
        limit(f"[{form}] drift 0.05 deg/s: the calculator warns", "yes" if all(
            ss["calc"][form].get("still_warning") for ss in dss) else "no",
            all(ss["calc"][form].get("still_warning") for ss in dss))
        held, n = held_share(r1, form, pick("d"))
        limit(f"[{form}] drift 0.05 deg/s: numbers held back", f"{100 * held:.0f} % of {n}", held == 1.0)
        held, n = held_share(r1, form, clean)
        limit(f"[{form}] clean sessions (a, b/c/f at 0.45 %): numbers given in >= {100 * GIVEN_MIN:.0f} %",
              f"{100 * (1 - held):.1f} % of {n}", 1 - held >= GIVEN_MIN)
        recs = cases(r1, form, pick("g"))
        worst, n = worst_of(recs)
        held, n_g = held_share(r1, form, pick("g"))
        found = ", ".join(f"{ss['spin']:g} in {d}: {pct(x['e_found'])} (x{x['e_found'] / x['e_true']:.2f})"
                          for ss, d, x in cases(r1, form, pick("g"), given_only=False))
        if form == "simple":
            limit(f"[{form}] spin point (g): position only, a known limit", f"held {100 * held:.0f} %; found {found}",
                  True, "reported only: the position comes out x(1 - 2.3 D / L) and nothing in it shows that",
                  reported_only=True)
        else:
            limit(f"[{form}] spin point (g): no number more than {SPIN_LIMIT_PP} pp off (held back counts as none)",
                  f"held {100 * held:.0f} % of {n_g}; worst given {100 * worst:.3f} pp of {n}; found {found}",
                  100 * worst <= SPIN_LIMIT_PP)

    # ---- closing the loop
    print("\n== 4. Closing the loop on the (c) robots: the calculator's constants back into the simulator ==")
    print("square: each run's |E| (truth), median over robots and runs; score: the UMBmark score (larger cluster-centre "
          "distance, truth), median over robots;")
    print("route: routes/skills60.txt odometry error at the end / largest on the way, median over robots x "
          f"{o.route_seeds} slip seeds; |e left|: |true heading error after the scale|, median over robot-directions")
    res["loop"] = {}
    print(f"{'set':13s} {'form':10s} {'stage':16s} {'square |E|':>10s} {'score':>6s} {'route end':>9s} "
          f"{'route max':>9s} {'|e left| %':>10s} {'max |e left|':>12s}")

    def e_left(sc, rb):
        return [abs((1 + rb["e_cw"]) * f32(sc[0]) - 1), abs((1 + rb["e_ccw"]) * f32(sc[1]) - 1)]

    for name, robots in robot_sets.items():
        res["loop"][name] = {}
        for form in FORMS:
            rows = {}
            stages = [("before", lambda i: next(s for s in r1 if s["robot"] == (name, i)), "before"),
                      ("after round 1", lambda i: r2.get((name, i, form)), f"r1 {form}"),
                      ("after round 2", lambda i: r3.get((name, i, form)), f"r2 {form}"),
                      ("ideal scale", lambda i: ideal[(name, i)], "ideal")]
            for stage, get, rkey in stages:
                sq, sc, rend, rmax, el, ok = [], [], [], [], [], True
                for i, rb in enumerate(robots):
                    ss = get(i)
                    if ss is None:
                        ok = False
                        continue
                    t = truth(ss)
                    sq += t["E_runs"]
                    sc.append(t["score"])
                    rend += [r["end_in"] for r in routes[(name, i, rkey)]]
                    rmax += [r["max_in"] for r in routes[(name, i, rkey)]]
                    el += e_left(ss["scales"], rb)
                rows[stage] = {"square_median_in": float(np.median(sq)), "score_median_in": float(np.median(sc)),
                               "route_end_median_in": float(np.median(rend)),
                               "route_max_median_in": float(np.median(rmax)),
                               "e_left_median_pp": float(100 * np.median(el)), "e_left_max_pp": float(100 * max(el)),
                               "complete": ok}
                v = rows[stage]
                print(f"{(name if form == FORMS[0] and stage == 'before' else ''):13s} "
                      f"{(form if stage == 'before' else ''):10s} {stage:16s} {v['square_median_in']:10.2f} "
                      f"{v['score_median_in']:6.2f} {v['route_end_median_in']:9.2f} {v['route_max_median_in']:9.2f} "
                      f"{v['e_left_median_pp']:10.3f} {v['e_left_max_pp']:12.3f}")
            # the second pass: what the calculator says about the round 2 squares
            recs = [(r2[(name, i, form)], d, r2[(name, i, form)]["calc"][form]["dirs"][d])
                    for i in range(len(robots)) if (name, i, form) in r2 and "dirs" in r2[(name, i, form)]["calc"][form]
                    for d in ("CW", "CCW")]
            second = {"n": len(recs),
                      "reported_abs_e_median_pp": float(100 * np.median([abs(x["e_found"]) for _, _, x in recs])),
                      "reported_within_limit": share(abs(x["e_found"]) <= SECOND_REPORTED_MAX_PP / 100
                                                     for _, _, x in recs),
                      "leftover_small": share(x["leftover_small"] for _, _, x in recs),
                      "cover_1x": share(x["inside_1x"] for _, _, x in recs),
                      "cover_2x": share(x["inside_2x"] for _, _, x in recs),
                      "abs_err_median_pp": float(100 * np.median([abs(x["err"]) for _, _, x in recs])),
                      "verdicts": {k: sum(1 for _, _, x in recs if x["slip_class"] == k)
                                   for k in ("repeatable", "borderline", "slip")}}
            # does it say "this test is done" (every direction within twice its +-)? Round 1 for comparison
            r1_calcs = [next(s for s in r1 if s["robot"] == (name, i))["calc"][form] for i in range(len(robots))]
            r2_calcs = [r2[(name, i, form)]["calc"][form] for i in range(len(robots)) if (name, i, form) in r2]
            for tag, calcs in (("round1", r1_calcs), ("round2", r2_calcs)):
                ws = [bool(c.get("settled")) for c in calcs if "dirs" in c]
                second[f"says_done_{tag}"] = share(ws) if ws else None
                second[f"held_{tag}"] = share(c["held"] for c in calcs if "dirs" in c)
            # the score it predicts after its constants, against the score the truth gives on the next round
            second["score_predicted_after_median_in"] = float(np.median([c["score_predicted_after_in"]
                                                                         for c in r1_calcs if "dirs" in c]))
            print(f"{'':13s} {'':10s} held back: round 1 {100 * second['held_round1']:.0f} %, round 2 "
                  f"{100 * second['held_round2']:.0f} %")
            second["score_next_round_median_in"] = rows["after round 1"]["score_median_in"]
            ratio = rows["after round 2"]["e_left_median_pp"] / rows["after round 1"]["e_left_median_pp"]
            second["e_left_ratio"] = ratio
            second["verdict"] = "helps" if ratio < HELP_RATIO else ("harms" if ratio > HARM_RATIO else "noise")
            rows["second_pass"] = second
            res["loop"][name][form] = rows
            print(f"{'':13s} {'':10s} second pass: calculator's own |e| median {second['reported_abs_e_median_pp']:.3f} "
                  f"%, within {SECOND_REPORTED_MAX_PP} pp in {100 * second['reported_within_limit']:.0f} %, leftover "
                  f"small in {100 * second['leftover_small']:.0f} %, truth inside +- {100 * second['cover_1x']:.0f} % "
                  f"(2x {100 * second['cover_2x']:.0f} %); round 2 {second['verdict']} (|e left| x{ratio:.2f})")
            done1, done2 = second["says_done_round1"], second["says_done_round2"]
            print(f"{'':13s} {'':10s} 'this test is done': round 1 {'-' if done1 is None else f'{100 * done1:.0f} %'}"
                  f", round 2 {'-' if done2 is None else f'{100 * done2:.0f} %'}; score it predicted after round 1 "
                  f"{second['score_predicted_after_median_in']:.2f} in, the truth gave "
                  f"{second['score_next_round_median_in']:.2f} in (medians)")

    for form in FORMS:
        u = res["loop"]["uncalibrated"][form]
        b, a = u["before"], u["after round 1"]
        limit(f"[{form}] loop, uncalibrated: median square |E| after <= {LOOP_MAX_RATIO} x before",
              f"{b['square_median_in']:.2f} -> {a['square_median_in']:.2f} in (x{a['square_median_in'] / b['square_median_in']:.2f})",
              a["square_median_in"] <= LOOP_MAX_RATIO * b["square_median_in"])
        limit(f"[{form}] loop, uncalibrated: median UMBmark score after <= {LOOP_MAX_RATIO} x before",
              f"{b['score_median_in']:.2f} -> {a['score_median_in']:.2f} in (x{a['score_median_in'] / b['score_median_in']:.2f})",
              a["score_median_in"] <= LOOP_MAX_RATIO * b["score_median_in"])
        limit(f"[{form}] loop, uncalibrated: median 60 s route end error after <= {ROUTE_MAX_RATIO} x before",
              f"{b['route_end_median_in']:.2f} -> {a['route_end_median_in']:.2f} in "
              f"(x{a['route_end_median_in'] / b['route_end_median_in']:.2f})",
              a["route_end_median_in"] <= ROUTE_MAX_RATIO * b["route_end_median_in"])
        c = res["loop"]["calibrated"][form]
        limit(f"[{form}] loop, calibrated: median square |E| after <= {CALIBRATED_MAX_RATIO} x before",
              f"{c['before']['square_median_in']:.2f} -> {c['after round 1']['square_median_in']:.2f} in",
              c["after round 1"]["square_median_in"] <= CALIBRATED_MAX_RATIO * c["before"]["square_median_in"])
        s = u["second_pass"]
        limit(f"[{form}] second pass, uncalibrated: calculator's |e| <= {SECOND_REPORTED_MAX_PP} pp in >= "
              f"{100 * SECOND_SHARE_MIN:.0f} %", f"{100 * s['reported_within_limit']:.0f} %",
              s["reported_within_limit"] >= SECOND_SHARE_MIN)
        limit(f"[{form}] second pass, uncalibrated: leftover 'small' in >= {100 * SECOND_SHARE_MIN:.0f} %",
              f"{100 * s['leftover_small']:.0f} %", s["leftover_small"] >= SECOND_SHARE_MIN)
        limit(f"[{form}] round 2 (second suggestion applied), uncalibrated: not 'harms'",
              s["verdict"] + f" (x{s['e_left_ratio']:.2f})", s["verdict"] != "harms",
              "helps < 0.8 < noise < 1.25 < harms; reported only until 2026-11-22")

    print("\n== 5. Limits (written down before the first run) ==")
    for lim in limits:
        mark = "PASS" if lim["pass"] else ("FAIL" if lim["pass"] is False else ("info: met" if lim["met"] else
                                                                                 "info: not met"))
        print(f"{mark:13s} {lim['name']}: {lim['value']}" + (f"  ({lim['note']})" if lim["note"] else ""))
    n_fail = sum(1 for lim in limits if lim["pass"] is False)
    print(f"{sum(1 for lim in limits if lim['pass'])} passed, {n_fail} failed, "
          f"{sum(1 for lim in limits if lim['pass'] is None)} reported only")
    return res


if __name__ == "__main__":
    sys.exit(main())
