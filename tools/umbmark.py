#!/usr/bin/env python3
"""umbmark - the inertial sensor's scale error, from the UMBmark square test (docs/UMBMARK.md).

    python3 tools/umbmark.py measurements.csv [--log tune.log] [--session N] [--use-heading] [--side 48]
                             [--scale-cw 1] [--scale-ccw 1] [--json FILE] [--plot FILE.png]
    python3 tools/umbmark.py --template simple      (or corners): print a file to fill in

THE TEST. The robot drives a 48 in square under its own odometry (legs forwards with
moveToPoint, turns on the spot with turnToHeading) and comes back to its start. It believes it
is back at the start; a person measures with a ruler where it really is. Five runs clockwise
(CW), five counter-clockwise (CCW). At the end of each run the robot prints one line:

    [UMB] v=1 run=3 dir=CW side=48 drive_speed=70 turn_speed=70 x=0.12 y=-0.31 theta=359.4
          still_dps=0.004 scale_cw=1 scale_ccw=1 t_ms=21430                    (one line)

The frame is the robot's start (setPose(0, 0, 0)): x to its right, y forward, heading in degrees,
clockwise positive, as LemLib has it.

THE MEASUREMENTS FILE is a CSV ('#' lines are comments). Two forms, chosen by the header:

  simple:  dir,right_in,forward_in,heading_deg,turned
      How far the robot's centre mark really ended from its start mark: right + / left -,
      forward + / back -. heading_deg (clockwise +) may be blank or left out.
  corners: dir,left_right_in,left_forward_in,right_right_in,right_forward_in,turned
      The same two numbers for two marked points on the robot (its two back corners, say), each
      measured from its own tape mark. Needs two lines in the file:
          # corner_spacing_in = 14.0            distance between the two points
          # corners_behind_centre_in = 7.0      how far the line joining them is behind the
                                                tracking centre (negative if in front)
      and, if their midpoint is not straight behind the centre, # corners_mid_right_in = ...

turned (either form): which way the robot ended up turned compared with how it started, seen
from above: CW, CCW, or 0 if it is not visibly turned (blank: not looked). It is the check on the signs: a mark
measured from the wrong end, or two columns swapped, give numbers that fit a scale error of the
other sign perfectly, and only something seen another way can tell.
Either form may add believed_x_in, believed_y_in, believed_theta_deg (where the robot thought it
was) and run (checked against the log). Numbers can be written 1.375, 11/8 or 1 3/8.

WHAT IS FITTED. Say the heading the odometry uses (the sensor's reading after setImuScale)
changes by (1 + e) times every real clockwise change (e_cw) and every real counter-clockwise
change (e_ccw). On a clockwise square the robot turns until it believes it faces 90, 180, 270
and 360, so on leg k (k = 0..3) it really faces k*psi, psi = 90/(1 + e) degrees. It drives each
leg straight, so relative to where it believes it is it really ends at

    L * sum over k of (sin, cos)(k*psi)  =  L * sin(2 psi) / sin(psi/2) * (sin 1.5psi, cos 1.5psi)

(x mirrored for counter-clockwise), and its real heading change is 360/(1 + e). For small e that
is about (+pi L e, -pi L e) and -360 e degrees clockwise, (-pi L e, -pi L e) and +360 e
counter-clockwise. The tool fits e with the exact formula, from position and (when measured)
from heading. The test sees the heading after the current scale, so the scale to use is
current / (1 + e).

WHEN IT GIVES NO NUMBERS. It prints the two constants only when nothing says they could be
wrong. It holds them back (exit status 3, the reasons at the top of the report) when: position and
heading disagree; the turned column says the robot ended turned the other way from what the
numbers say, or is missing while the error is big enough to see; a sign or column mix-up would
explain the error far better; the error is over 2 % with no heading to back it up (V5 sensors are
usually 1 to 1.4 % off: check the numbers are inches); the heading crept while the robot stood
still; or a run ran out of time. A direction with fewer than 3 runs gets no number. --use-heading
takes the heading's number when the two disagree, for a robot that does not spin about its
tracking centre (see below), once the heading has been checked.

WHAT IT CANNOT SEE. Tracking-wheel size: every leg scales the same and the square still closes
(check it with a tape). Wrong wheel offsets and a wheel not quite parallel each cancel round a
closed square with on-the-spot turns (under 0.1 in each in umbmark_selftest.py's own little
simulator and in tools/sim). A robot that turns about a point away from its tracking centre is
different: with a scale error it makes the position fit come out too SMALL, by about 2.3 x D / L
of itself for a point D in behind (a quarter at 5.8 in, the spin point robot-config.cpp's
[ODOMCAL] note suggests; a tenth at 2.24 in), and it leaves nothing over to warn you. The heading
is not affected. So measure the heading too (heading_deg, or the corners form): when position and
heading disagree this way, the tool says so. When every direction comes out under twice its ±
and nothing is left over, the tool says the test is done: it cannot tell the scale from right,
and another round only swaps noise for noise (tools/sim/umb_e2e.py runs the whole loop in the
LemLib simulator). A steady heading drift looks like a scale error, + clockwise and -
counter-clockwise: keep still_dps small. Slip shows as scatter between identical runs.

Exit status: 0 numbers printed, 3 numbers held back (the report says why), 2 a problem with the
input (the message says where), 1 a bug in this tool.

Standard library only, plus matplotlib for the plot.
"""

import argparse
import csv
import difflib
import json
import math
import os
import re
import sys

# ---------------------------------------------------------------------------
# Settings. Every number that decides a verdict lives here.
# ---------------------------------------------------------------------------
DEFAULT_SIDE_IN = 48.0          # UMB_SIDE_IN in the robot routine
STOP_TOL_IN = 0.5               # linearController small error range (robot-config.cpp): how far from the
STOP_TOL_DEG = 1.0              # point a move may stop; angularController's, for turns. Only used in warnings
SLIP_REPEATABLE_IN = 0.5        # scatter (RMS distance from the cluster centre) under this: repeatable
SLIP_BAD_IN = 1.0               # over this: slip or a lifting wheel
MIN_RUNS_SCATTER = 3            # fewer runs than this in a direction: the scatter cannot be judged
RECOMMENDED_RUNS = 5
STILL_DPS_WARN = 0.02           # heading drift standing still (deg/s) worth a warning
AGREE_SIGMAS = 2.0              # position and heading agree if within this many combined standard errors...
AGREE_FLOOR = 0.0015            # ...plus this (0.15 %), which covers ruler and protractor rounding
SE_FLOOR = 0.0002               # smallest standard error used (0.02 %). Five headings rounded to the same
                                # half degree have no scatter at all, which would let them take over the weighting
LEFTOVER_SMALL_IN = 0.25        # leftover repeatable error under this counts as "small"
SETTLED_SIGMAS = 2.0            # a scale error under this many times its ± cannot be told from 0: the test is done.
                                # Without this the tool asked for one more round every time. In tools/sim/umb_e2e.py
                                # (40 robots, three forms) applying a second round's suggestion changed the median
                                # scale error left by x0.7 to x1.2, and the square and route errors not at all:
                                # once the scale is right, each round only swaps one bit of slip noise for another
FIT_LIMIT = 0.2                 # the fit searches +-20 %...
FIT_GRID = 0.002                # ...in steps of this, then refines
MAX_BELIEVABLE_E = 0.10         # a sensor 10 % off would miss a 90 deg turn by 9 deg: the measurement is wrong.
                                # A 40 in error once fitted -16.7 % with 30 in left over, so the search edge
                                # alone did not catch it
SUSPICIOUS_IN = 12.0            # a run that ended further than this from where it thought: check it
SUSPICIOUS_DEG = 30.0           # likewise for a heading
CORNER_SPACING_TOL_IN = 0.25    # corners form: the two points' spacing must come out within this
TYPED_VS_LOG_IN = 0.1           # typed believed_* columns and the log should agree within these
TYPED_VS_LOG_DEG = 0.5
SIGN_HINT_MIN_IN = 1.0          # leftover over this, and a sign flip would cut it to under
SIGN_HINT_RATIO = 0.3           # this fraction of itself: suggest the student check the signs
# The checks that hold the constants back (added 2026-11-22 after a review found confident wrong-sign numbers
# for a mark measured from the robot, two columns swapped, centimetres, and two corners typed in the wrong order)
TURN_VISIBLE_DEG = 1.0          # a heading this far off its start can be seen against a tape line (18 in robot:
                                # 0.3 in from back to front). A direction whose scale error turns the robot at least
                                # this far needs its sign confirmed: by heading_deg agreeing, or by the turned column
PLAUSIBLE_E = 0.02              # V5 inertial sensors are usually 1 to 1.4 % off. Above 2 % (centimetres read as
                                # inches gave 3 %) the number needs heading_deg to back it up
MIN_RUNS_CONSTANT = 3           # fewer runs than this in a direction: no constant for it (no scatter, no ±)
HEADING_STEPS_DEG = (1.0, 0.5, 0.25, 0.1)  # the steps a heading is read to, coarsest first: the readings tell which
# Five headings read to the same half degree carry the same rounding error, so their mean is no better than one
# reading. The ± of e from heading_deg is never under (step / sqrt(12)) / 360: 0.04 % for 0.5 deg. With only
# SE_FLOOR the truth was inside the ± in 12 % of tools/sim/umb_e2e.py's cases (31 % inside twice it)
MAX_INPUT_IN = 100.0            # a distance over this is a typing mistake (a 1e308 once crashed the fit)
MAX_INPUT_DEG = 180.0           # a heading is between -180 and +180
JUNK_SHARE = 0.05               # more unreadable characters than this: the file is not text
# Coverage of a 1-sigma and a 2-sigma standard error from n runs (Student's t, n - 1 degrees of freedom), for the
# words under the ±. From 11 degrees of freedom on, the last entry
COVERAGE_PCT = {1: (50, 70), 2: (58, 82), 3: (61, 86), 4: (63, 88), 5: (64, 90), 6: (64, 91), 7: (65, 91),
                8: (65, 92), 9: (66, 92), 10: (66, 93)}
EXIT_HELD_BACK = 3
PLOT_DPI = 130

# Colour-blind-safe categorical palette, the same as pidlog.py.
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
C_SURFACE, C_INK, C_INK2, C_MUTED = "#fcfcfb", "#0b0b0b", "#52514e", "#898781"
C_GRID, C_AXIS = "#e1e0d9", "#c3c2b7"

DIRS = ("CW", "CCW")
DIR_WORD = {"CW": "clockwise", "CCW": "counter-clockwise"}
UMB_MARKER = "[UMB]"
ANSI_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]|\x1b[@-Z\\-_]")
CTRL_RE = re.compile(r"[\x00-\x08\x0b-\x1f\x7f]")
FRAC_RE = re.compile(r"^([+-]?)(?:(\d+)\s+)?(\d+)\s*/\s*(\d+)$")
UNIT_RE = re.compile(r"\s*(in|inch|inches|\"|deg|degrees|°)$", re.IGNORECASE)
KEY_RE = re.compile(r"^#\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*?)\s*$")

SIMPLE_COLS = ["dir", "right_in", "forward_in"]
CORNER_COLS = ["dir", "left_right_in", "left_forward_in", "right_right_in", "right_forward_in"]
OPTIONAL_COLS = ["heading_deg", "turned", "believed_x_in", "believed_y_in", "believed_theta_deg", "run"]
TURNED_WORDS = {"cw": "CW", "clockwise": "CW", "ccw": "CCW", "acw": "CCW", "anticlockwise": "CCW",
                "anti-clockwise": "CCW", "counterclockwise": "CCW", "counter-clockwise": "CCW",
                "0": "0", "none": "0",          # looked, and it was not visibly turned
                "": None, "?": None, "-": None}  # not written down
CORNER_KEYS = {"corner_spacing_in": "corner_spacing_in",
               "corners_behind_centre_in": "corners_behind_centre_in",
               "corners_behind_center_in": "corners_behind_centre_in",
               "corners_mid_right_in": "corners_mid_right_in"}

TEMPLATES = {
    "simple": """\
# UMBmark square: one row per run, in the order the runs were driven.
# dir: CW or CCW.
# right_in: how far the robot's centre mark ended to the RIGHT of its start mark (left = negative).
# forward_in: how far FORWARD of its start mark (back = negative).
# Measure FROM the start mark TO where the robot's mark is now.
# heading_deg: how far it ended turned, clockwise +. Leave it blank if you did not measure it.
# turned: which way it ended turned compared with the tape along its side, seen from above:
#   CW, CCW, or 0 if it is not visibly turned (leave it blank only if you did not look).
# Right, left, forward and back are as the robot faced at the start. Numbers: 1.375, 11/8 or 1 3/8.
# Delete the '# ' in front of the example rows and put your own numbers in.
dir,right_in,forward_in,heading_deg,turned
# CW,1 7/8,-1 3/4,-4.5,CCW
# CCW,-1 1/8,-1 1/4,,CW
""",
    "corners": """\
# UMBmark square, measured at two marked points on the robot (its two back corners, say).
# Each point is measured from its own tape mark, put down at the start: FROM the mark TO the point.
# right: how far it ended to the RIGHT of its mark (left = negative); forward: FORWARD (back = negative).
# turned: which way it ended turned compared with how it started, seen from above: CW, CCW, or 0 (not turned).
# Right, left, forward and back are as the robot faced at the start. Numbers: 1.375, 11/8 or 1 3/8.
# Distance between the two points:
# corner_spacing_in = 14.0
# How far the line joining them is behind the tracking centre (negative if it is in front):
# corners_behind_centre_in = 7.0
# Delete the '# ' in front of the example rows and put your own numbers in.
dir,left_right_in,left_forward_in,right_right_in,right_forward_in,turned
# CW,1 3/8,-2 1/4,2 1/2,-1 1/8,CCW
""",
}


class InputError(Exception):
    """Something in the input a student can fix. The message says where and what."""


# ---------------------------------------------------------------------------
# Geometry and the model
# ---------------------------------------------------------------------------
def wrap180(a):
    a = math.fmod(a + 180.0, 360.0)
    if a < 0:
        a += 360.0
    return a - 180.0


def turn_sign(direction):
    return 1.0 if direction == "CW" else -1.0


def rot_cw(p, deg):
    """Turn the vector p = (right, forward) clockwise by deg degrees."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return (p[0] * c + p[1] * s, -p[0] * s + p[1] * c)


def square_end(e, side, direction):
    """Where the robot really ends, as (right, forward) in the start frame, relative to where it
    believes it is, after a square whose heading reads (1 + e) times every real change."""
    psi = (math.pi / 2.0) / (1.0 + e)
    r = side * math.sin(2.0 * psi) / math.sin(psi / 2.0)
    return (turn_sign(direction) * r * math.sin(1.5 * psi), r * math.cos(1.5 * psi))


def square_end_by_legs(e, side, direction):
    """The same as square_end, leg by leg. Kept as the plain definition the closed form is checked against."""
    psi = (math.pi / 2.0) / (1.0 + e)
    sx = sum(math.sin(k * psi) for k in range(4))
    sy = sum(math.cos(k * psi) for k in range(4))
    return (turn_sign(direction) * side * sx, side * sy)


def heading_error_model(e, direction):
    """Real minus believed heading at the end of a square (degrees, clockwise +, wrapped)."""
    turn = 360.0 * turn_sign(direction)
    return wrap180(turn / (1.0 + e) - turn)


def _golden(f, a, b, iters=60):
    g = (math.sqrt(5.0) - 1.0) / 2.0
    c, d = b - g * (b - a), a + g * (b - a)
    fc, fd = f(c), f(d)
    for _ in range(iters):
        if fc < fd:
            b, d, fd = d, c, fc
            c = b - g * (b - a)
            fc = f(c)
        else:
            a, c, fc = c, d, fd
            d = a + g * (b - a)
            fd = f(d)
    return (a + b) / 2.0


def fit_position(ex, ey, side, direction):
    """The e whose square_end comes closest to the error (ex, ey). Returns (e, leftover vector)."""
    def cost(e):
        x, y = square_end(e, side, direction)
        return (x - ex) ** 2 + (y - ey) ** 2
    n = int(round(FIT_LIMIT / FIT_GRID))
    best = min((k * FIT_GRID for k in range(-n, n + 1)), key=cost)
    e = _golden(cost, max(-FIT_LIMIT, best - FIT_GRID), min(FIT_LIMIT, best + FIT_GRID))
    x, y = square_end(e, side, direction)
    return e, (ex - x, ey - y)


def e_from_heading(believed_theta, real_heading, direction):
    """Exact: the odometry turned believed degrees while the robot really turned real degrees.
    Both are taken as the full turn nearest to +-360, so a wrapped or unwrapped theta both work."""
    turn = 360.0 * turn_sign(direction)
    tb = turn + wrap180(believed_theta - turn)
    tr = turn + wrap180(real_heading)
    return tb / tr - 1.0


def corners_to_pose(dl, dr, spacing, behind, mid_right):
    """Two points measured from their own start marks -> (real move of the tracking centre, real heading
    change, the points' spacing as measured). The points start at (mid_right -+ spacing/2, -behind) in the
    robot's frame; after the run each is at T + rot_cw(start, phi)."""
    vx = spacing + dr[0] - dl[0]
    vy = dr[1] - dl[1]
    phi = math.degrees(math.atan2(-vy, vx))
    m0 = (mid_right, -behind)
    mid_now = (m0[0] + (dl[0] + dr[0]) / 2.0, m0[1] + (dl[1] + dr[1]) / 2.0)
    rm = rot_cw(m0, phi)
    return (mid_now[0] - rm[0], mid_now[1] - rm[1]), phi, math.hypot(vx, vy)


def floor_se(se):
    return None if se is None else max(se, SE_FLOOR)


def mean(xs):
    xs = list(xs)
    return sum(xs) / len(xs) if xs else None


def std_error(xs):
    """Standard error of the mean (sample standard deviation / sqrt(n)); None for fewer than 2."""
    xs = list(xs)
    n = len(xs)
    if n < 2:
        return None
    m = sum(xs) / n
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (n - 1)) / math.sqrt(n)


# ---------------------------------------------------------------------------
# Reading the measurements file
# ---------------------------------------------------------------------------
def parse_number(text):
    """'1.375', '11/8', '1 3/8', '-1 3/8', '2 in' -> float; '' -> None; anything else -> ValueError."""
    t = text.strip().replace("−", "-")
    t = UNIT_RE.sub("", t).strip()
    if not t:
        return None
    m = FRAC_RE.match(t)
    if m:
        sign, whole, num, den = m.groups()
        if int(den) == 0:
            raise ValueError("divide by zero")
        v = (int(whole) if whole else 0) + int(num) / int(den)
        return -v if sign == "-" else v
    v = float(t)
    if not math.isfinite(v):
        raise ValueError("not finite")
    return v


def _hint(word, choices):
    close = difflib.get_close_matches(word, choices, n=1, cutoff=0.6)
    return (" Did you mean '%s'?" % close[0]) if close else ""


def read_measurements(path):
    try:
        with open(path, "r", encoding="utf-8-sig", errors="replace", newline="") as fh:
            text = fh.read()
    except OSError as exc:
        raise InputError("cannot read the measurements file %s: %s" % (path, exc.strerror or exc))
    name = os.path.basename(path)
    # A spreadsheet saved as .xlsx, or a photo, read as text: say so, rather than blame the separators
    junk = sum(1 for ch in text if ch == "\ufffd" or (ord(ch) < 32 and ch not in "\t\r\n"))
    if "\x00" in text or (text and junk > JUNK_SHARE * len(text)):
        raise InputError("%s does not look like a text CSV file (it has %d unreadable characters). Save it from "
                         "the spreadsheet as CSV, or start from:  python3 tools/umbmark.py --template simple"
                         % (name, junk))
    keys, unknown_keys = {}, []
    header, header_ln, data = None, None, []
    for ln, raw in enumerate(text.replace("\r\n", "\n").replace("\r", "\n").split("\n"), start=1):
        s = raw.strip()
        if not s:
            continue
        if s.startswith("#"):
            m = KEY_RE.match(s)
            if m:
                k = m.group(1).lower()
                if k in CORNER_KEYS:
                    try:
                        v = parse_number(m.group(2))
                    except ValueError:
                        v = None
                    if v is None:
                        raise InputError("%s line %d: '%s' needs a number after the '=', for example "
                                         "'# %s = 14.0'." % (name, ln, s, k))
                    if abs(v) > MAX_INPUT_IN:
                        raise InputError("%s line %d: %s = %g is more than %g in. Check the number (inches)."
                                         % (name, ln, k, v, MAX_INPUT_IN))
                    keys[CORNER_KEYS[k]] = (v, ln)
                else:
                    unknown_keys.append((k, ln))
            continue
        cells = next(csv.reader([raw]))
        if header is None:
            if ";" in raw and "," not in raw:
                raise InputError("%s line %d: the columns are separated by ';'. Save the file with commas "
                                 "(CSV), or replace each ';' with ','." % (name, ln))
            header = [c.strip().lower() for c in cells]
            while header and header[-1] == "":
                header.pop()        # a spreadsheet's trailing comma
            header_ln = ln
        elif any(c.strip() for c in cells):
            data.append((ln, cells))   # a spreadsheet's empty ",,," rows are skipped
    if header is None:
        raise InputError("%s has no header line. The first line that is not a '#' comment must name the "
                         "columns, for example:  dir,right_in,forward_in,heading_deg   "
                         "(python3 tools/umbmark.py --template simple prints a file to start from)." % name)

    known = set(SIMPLE_COLS + CORNER_COLS + OPTIONAL_COLS)
    for c in header:
        if c == "":
            raise InputError("%s line %d: one of the column names is empty (two commas in a row, or a comma "
                             "at the end of the header)." % (name, header_ln))
        if c not in known:
            raise InputError("%s line %d: '%s' is not a column this tool knows.%s The columns are: %s."
                             % (name, header_ln, c, _hint(c, sorted(known)), ", ".join(
                                 SIMPLE_COLS + ["heading_deg"] + CORNER_COLS[1:] + OPTIONAL_COLS[1:])))
    dup = sorted({c for c in header if header.count(c) > 1})
    if dup:
        raise InputError("%s line %d: the column '%s' appears twice." % (name, header_ln, dup[0]))
    has_simple = any(c in header for c in SIMPLE_COLS[1:])
    has_corner = any(c in header for c in CORNER_COLS[1:])
    if has_simple and has_corner:
        raise InputError("%s line %d: the header mixes the simple form (right_in, forward_in) with the corners "
                         "form (left_right_in, ...). Use one or the other." % (name, header_ln))
    form = "corners" if has_corner else "simple"
    need = CORNER_COLS if form == "corners" else SIMPLE_COLS
    missing = [c for c in need if c not in header]
    if missing:
        raise InputError("%s line %d: the %s form needs the column(s) %s. The header should be:  %s"
                         % (name, header_ln, form, ", ".join(missing), ",".join(
                             need + (["heading_deg"] if form == "simple" else []) + ["turned"])))
    if form == "corners" and "heading_deg" in header:
        raise InputError("%s line %d: the corners form works the heading out from the two points, so it has no "
                         "heading_deg column. Delete that column." % (name, header_ln))
    if form == "corners":
        lacking = [k for k in ("corner_spacing_in", "corners_behind_centre_in") if k not in keys]
        if lacking:
            msg = ("%s: the corners form needs these lines in the file: %s. Add, for example:  "
                   "# corner_spacing_in = 14.0   and   # corners_behind_centre_in = 7.0"
                   % (name, ", ".join("'# %s = ...'" % k for k in lacking)))
            for k, kln in unknown_keys:
                h = _hint(k, list(CORNER_KEYS))
                if h:
                    msg += ". Line %d has '# %s = ...':%s" % (kln, k, h)
            raise InputError(msg)
        if keys["corner_spacing_in"][0] <= 0:
            raise InputError("%s line %d: corner_spacing_in must be more than 0." % (name, keys["corner_spacing_in"][1]))
    if not data:
        raise InputError("%s has a header but no rows of measurements (rows starting with '#' are comments)."
                         % name)

    rows, warnings = [], []
    col = {c: i for i, c in enumerate(header)}
    for idx, (ln, cells) in enumerate(data, start=1):
        while len(cells) > len(header) and not cells[-1].strip():
            cells = cells[:-1]
        if len(cells) > len(header):
            raise InputError("%s line %d has %d cells but the header has %d columns (%s)."
                             % (name, ln, len(cells), len(header), ",".join(header)))
        cells = [c.strip() for c in cells] + [""] * (len(header) - len(cells))

        def num(c, required):
            if c not in col:
                return None
            txt = cells[col[c]]
            try:
                v = parse_number(txt)
            except ValueError:
                raise InputError("%s line %d, column %s: '%s' is not a number. Write it like 1.5, 3/8 or 1 3/8 "
                                 "(inches or degrees, no other units)." % (name, ln, c, txt))
            if v is None and required:
                raise InputError("%s line %d: %s is empty. Every row needs it." % (name, ln, c))
            if v is not None and c.endswith("_in") and abs(v) > MAX_INPUT_IN:
                raise InputError("%s line %d, column %s: %g is more than %g in. Check the number (inches, not "
                                 "centimetres or millimetres)." % (name, ln, c, v, MAX_INPUT_IN))
            if v is not None and c == "heading_deg" and abs(v) > MAX_INPUT_DEG:
                raise InputError("%s line %d, column heading_deg: %g is outside -180 to +180. Write how far the "
                                 "robot ended turned from its start, clockwise +." % (name, ln, v))
            return v

        d = cells[col["dir"]].strip().upper()
        if d not in DIRS:
            raise InputError("%s line %d: dir is '%s'. It must be CW (clockwise) or CCW (counter-clockwise)."
                             % (name, ln, cells[col["dir"]]))
        row = {"row": idx, "line": ln, "dir": d, "heading": None, "corner_spacing_measured": None}
        if form == "simple":
            row["right"] = num("right_in", True)
            row["forward"] = num("forward_in", True)
            row["heading"] = num("heading_deg", False)
        else:
            dl = (num("left_right_in", True), num("left_forward_in", True))
            dr = (num("right_right_in", True), num("right_forward_in", True))
            sp = keys["corner_spacing_in"][0]
            behind = keys["corners_behind_centre_in"][0]
            mid = keys.get("corners_mid_right_in", (0.0, None))[0]
            (tx, ty), phi, sep = corners_to_pose(dl, dr, sp, behind, mid)
            row.update(right=tx, forward=ty, heading=phi, corner_spacing_measured=sep, corners=(dl, dr))
            if abs(sep - sp) > CORNER_SPACING_TOL_IN:
                warnings.append("%s line %d: the four numbers put the two points %.2f in apart, but "
                                "corner_spacing_in is %.2f. One of them is probably wrong (a sign, or a "
                                "point measured from the wrong mark)." % (name, ln, sep, sp))
        row["turned"] = None
        if "turned" in col:
            t = cells[col["turned"]].strip().lower()
            if t not in TURNED_WORDS:
                raise InputError("%s line %d, column turned: '%s' is not CW, CCW or 0. Write which way the robot "
                                 "ended up turned compared with how it started, seen from above (0 if it is not "
                                 "visibly turned)." % (name, ln, cells[col["turned"]]))
            row["turned"] = TURNED_WORDS[t]
        row["believed_typed"] = {"x": num("believed_x_in", False), "y": num("believed_y_in", False),
                                 "theta": num("believed_theta_deg", False)}
        run_txt = cells[col["run"]].strip() if "run" in col else ""
        row["run"] = run_txt or None
        rows.append(row)
    return {"form": form, "rows": rows, "keys": {k: v for k, (v, _) in keys.items()},
            "warnings": warnings, "name": name}


# ---------------------------------------------------------------------------
# Reading the robot's log
# ---------------------------------------------------------------------------
def clean_lines(raw):
    text = raw.decode("utf-8", errors="replace") if isinstance(raw, bytes) else raw
    text = text.replace("\r\n", "\n").replace("\r", "\n")
    for line in text.split("\n"):
        yield CTRL_RE.sub("", ANSI_RE.sub("", line))


def fval(kv, key):
    try:
        v = float(kv[key])
    except (KeyError, TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def read_log(path):
    try:
        with open(path, "rb") as fh:
            raw = fh.read()
    except OSError as exc:
        raise InputError("cannot read the log %s: %s" % (path, exc.strerror or exc))
    out = []
    for ln, line in enumerate(clean_lines(raw), start=1):
        pos = line.find(UMB_MARKER)
        if pos < 0:
            continue
        kv = {}
        for tok in line[pos + len(UMB_MARKER):].split():
            k, eq, v = tok.partition("=")
            if eq and k:
                kv[k] = v
        ent = {"line": ln, "kv": kv, "problems": [], "dir": kv.get("dir", "").upper(), "run": kv.get("run")}
        if ent["dir"] not in DIRS:
            ent["problems"].append("no dir=CW or dir=CCW")
        for k in ("x", "y", "theta"):
            ent[k] = fval(kv, k)
            if ent[k] is None:
                ent["problems"].append("no usable %s=" % k)
        for k in ("side", "still_dps", "scale_cw", "scale_ccw", "t_ms", "drive_speed", "turn_speed", "v", "timeouts"):
            ent[k] = fval(kv, k)
        out.append(ent)
    return out


BLOCK_KEYS = ("side", "scale_cw", "scale_ccw", "drive_speed", "turn_speed")


def split_blocks(entries):
    """The log's [UMB] lines in sets: a new set starts wherever the side, the scales or the speeds change (a new
    upload, usually). pros terminal --output appends, so one tune.log often holds round 1 and round 2. A damaged
    line stays in the set it is in."""
    blocks = []
    for ent in entries:
        key = tuple(ent[k] for k in BLOCK_KEYS)
        if blocks and (ent["problems"] or key == blocks[-1]["key"]):
            blocks[-1]["entries"].append(ent)
        else:
            blocks.append({"key": key, "entries": [ent]})
    for k, b in enumerate(blocks, start=1):
        b["n"] = k
    return blocks


def block_text(b):
    es = b["entries"]

    def g(v, fmt):
        return fmt % v if v is not None else "?"
    side, cw, ccw, ds, ts = b["key"]
    return ("set %d: lines %d-%d, %d run(s), side %s, scales CW %s, CCW %s, speeds %s/%s"
            % (b["n"], es[0]["line"], es[-1]["line"], len(es), g(side, "%g"), g(cw, "%.4f"), g(ccw, "%.4f"),
               g(ds, "%g"), g(ts, "%g")))


def fit_problem(rows, block, n, lname, meas_name):
    """Why the rows cannot be matched to the last n lines of this set, or None if they can"""
    ents = block["entries"]
    if len(ents) < n:
        return ("%s has %d rows but %s has only %d [UMB] line(s). Each row is matched to the line its run printed, so "
                "the log must hold every run in the file." % (meas_name, n, block["where"], len(ents)))
    used = ents[-n:]
    for row, ent in zip(rows, used):
        if ent["problems"]:
            return ("%s line %d is an [UMB] line, but it is damaged (%s), and it is one of the last %d lines, which "
                    "are matched to your %d rows. Capture the runs again, or type where the robot thought it was "
                    "into believed_x_in, believed_y_in and believed_theta_deg and run without --log."
                    % (lname, ent["line"], "; ".join(ent["problems"]), n, n))
        if ent["dir"] != row["dir"]:
            return ("Row %d of %s (line %d) says %s, but the [UMB] line it is matched with (%s line %d, run=%s) says "
                    "%s. Rows are matched to the LAST %d [UMB] lines of %s, in order. Check the rows are in the "
                    "order the runs were driven and that the log ends with these runs (cut any later runs out of a "
                    "copy of the log)." % (row["row"], meas_name, row["line"], row["dir"], lname, ent["line"],
                                          ent["run"], ent["dir"], n, block["where"]))
        if row["run"] is not None and ent["run"] is not None and str(row["run"]) != str(ent["run"]):
            return ("Row %d of %s (line %d) says run %s, but the [UMB] line it is matched with (%s line %d) is run=%s. "
                    "Rows are matched to the LAST %d [UMB] lines of %s, in order."
                    % (row["row"], meas_name, row["line"], row["run"], lname, ent["line"], ent["run"], n,
                       block["where"]))
    return None


def match_log(rows, entries, log_path, meas_name, warnings, session=None):
    """The log lines the rows are matched with: the last len(rows) lines of one set of runs (split_blocks). With
    one set that is the end of the log, as before. With several, the rows must fit the last set and no other, or
    --session must say which; a file from round 1 read against a log with round 2 appended took round 2's scales
    and doubled the correction, with no warning (review, 2026-11-22)."""
    lname = os.path.basename(log_path)
    n = len(rows)
    if not entries:
        raise InputError("%s has no [UMB] lines. Is it the capture from the square runs (pros terminal "
                         "--output), and did each run finish? Without a log, add believed_x_in, believed_y_in "
                         "and believed_theta_deg columns, or run without --log." % lname)
    blocks = split_blocks(entries)
    for b in blocks:
        b["where"] = lname if len(blocks) == 1 else "%s's set %d (lines %d-%d)" % (
            lname, b["n"], b["entries"][0]["line"], b["entries"][-1]["line"])
    listing = "; ".join(block_text(b) for b in blocks)
    if session is not None:
        if not 1 <= session <= len(blocks):
            raise InputError("--session %d: %s holds %d set(s) of runs (%s)." % (session, lname, len(blocks), listing))
        chosen = blocks[session - 1]
        problem = fit_problem(rows, chosen, n, lname, meas_name)
        if problem:
            raise InputError(problem)
    else:
        fits = [b for b in blocks if fit_problem(rows, b, n, lname, meas_name) is None]
        last = blocks[-1]
        if len(blocks) > 1 and len(fits) > 1:
            raise InputError("Your %d rows fit more than one set of runs in %s, and each set was made with different "
                             "scales, so the answer depends on which: %s. Say which set you measured: run again with "
                             "--session %d for the last one (or another number). Better: capture each round into "
                             "its own log (pros terminal --output umb_round2.log)."
                             % (n, lname, "; ".join(block_text(b) for b in fits), last["n"]))
        if last in fits:
            chosen = last
        elif fits:
            raise InputError("Your rows do not fit the last set of runs in %s (%s). They fit %s. If that is the set "
                             "you measured, run again with --session %d."
                             % (lname, fit_problem(rows, last, n, lname, meas_name), block_text(fits[0]), fits[0]["n"]))
        elif len(blocks) > 1 and len(entries) >= n:
            raise InputError("The last %d [UMB] lines of %s were not all driven with the same side, scales and speeds "
                             "(%s). Analyse each set of runs on its own: put each in its own measurements file and "
                             "log." % (n, lname, listing))
        else:
            raise InputError(fit_problem(rows, last, n, lname, meas_name))
    used = chosen["entries"][-n:]
    if len(blocks) > 1:
        warnings.append("%s holds %d sets of runs made with different scales (%s). Your rows were matched with set %d."
                        % (lname, len(blocks), listing, chosen["n"]))
    for row, ent in zip(rows, used):
        if ent["v"] is not None and ent["v"] != 1:
            warnings.append("%s line %d has v=%g; this tool was written for v=1. Read it anyway."
                            % (lname, ent["line"], ent["v"]))
    skipped = [e for b in blocks for e in b["entries"] if e["problems"] and e not in used]
    if skipped:
        warnings.append("%s: %d damaged [UMB] line(s) outside the ones used were ignored (first at line %d)."
                        % (lname, len(skipped), skipped[0]["line"]))
    return used


def common_value(used, key, what, lname):
    vals = [e[key] for e in used if e[key] is not None]
    if not vals:
        return None
    if max(vals) - min(vals) > 1e-6:
        raise InputError("The runs matched in %s were driven with different %s (%s). Analyse each set of runs "
                         "on its own (put each in its own measurements file and log)."
                         % (lname, what, ", ".join(sorted({"%g" % v for v in vals}))))
    return vals[0]


# ---------------------------------------------------------------------------
# The analysis
# ---------------------------------------------------------------------------
def heading_step(values):
    """The step the headings were read to: the coarsest of HEADING_STEPS_DEG that every reading is a whole number of"""
    for step in HEADING_STEPS_DEG:
        if all(abs(v / step - round(v / step)) < 1e-6 for v in values):
            return step
    return HEADING_STEPS_DEG[-1]


def predicted_heading(e, believed_theta, direction):
    """Where the robot really ends turned (deg from its start, clockwise +, wrapped) if the heading reads (1 + e)
    times every real change: the inverse of e_from_heading"""
    turn = 360.0 * turn_sign(direction)
    tb = turn + wrap180(believed_theta - turn)
    return wrap180(tb / (1.0 + e))


def turned_check(d, rs, e):
    """The turned column against the way the scale error e says the robot ended turned. A run counts only if it was
    seen turned (CW or CCW) and e turns it at least TURN_VISIBLE_DEG"""
    agree = disagree = unseen = 0
    preds = [predicted_heading(e, r["believed_theta_deg"], d) for r in rs]
    for r, p in zip(rs, preds):
        if r["turned"] is None or abs(p) < TURN_VISIBLE_DEG:
            continue
        if r["turned"] == "0":
            unseen += 1          # looked, saw no turn, where the numbers say one should show
        elif (p > 0) == (r["turned"] == "CW"):
            agree += 1
        else:
            disagree += 1
    return {"agree": agree, "disagree": disagree, "unseen": unseen,
            "given": sum(1 for r in rs if r["turned"] is not None),
            "predicted_deg": mean(preds), "visible": mean(abs(p) for p in preds) >= TURN_VISIBLE_DEG}


def analyse(meas, used, args):
    warnings = list(meas["warnings"])
    hints = []
    rows = meas["rows"]
    lname = os.path.basename(args.log) if args.log else None

    # Side length and current scales: the log first, then the command line, then the defaults.
    side, side_from = None, None
    if used:
        side = common_value(used, "side", "side lengths", lname)
        if side is not None:
            side_from = "log"
            if args.side is not None and abs(args.side - side) > 1e-6:
                warnings.append("--side %g ignored: the log says side=%g." % (args.side, side))
    if side is None:
        side = args.side if args.side is not None else DEFAULT_SIDE_IN
        side_from = "--side" if args.side is not None else "default"
    if side <= 0:
        raise InputError("The side length must be more than 0 (got %g)." % side)
    scale, scale_from = {}, {}
    for d, key, opt in (("CW", "scale_cw", args.scale_cw), ("CCW", "scale_ccw", args.scale_ccw)):
        v = common_value(used, key, "IMU scales (%s)" % key, lname) if used else None
        if v is not None:
            scale[d], scale_from[d] = v, "log"
            if opt is not None and abs(opt - v) > 1e-6:
                warnings.append("--%s %g ignored: the log says %s=%g." % (key.replace("_", "-"), opt, key, v))
        else:
            scale[d] = opt if opt is not None else 1.0
            scale_from[d] = ("--" + key.replace("_", "-")) if opt is not None else "default"
        if scale[d] <= 0:
            raise InputError("The current %s scale must be more than 0 (got %g)." % (DIR_WORD[d], scale[d]))
    if used:
        for key, what in (("drive_speed", "drive speeds"), ("turn_speed", "turn speeds")):
            vals = {e[key] for e in used if e[key] is not None}
            if len(vals) > 1:
                warnings.append("The runs used different %s (%s): the scatter includes the effect of speed."
                                % (what, ", ".join("%g" % v for v in sorted(vals))))

    # Each run.
    runs, assumed = [], 0
    for i, row in enumerate(rows):
        d = row["dir"]
        sgn = turn_sign(d)
        typed = row["believed_typed"]
        src = {}
        if used:
            ent = used[i]
            bx, by, bt = ent["x"], ent["y"], ent["theta"]
            src = {"x": "log", "y": "log", "theta": "log"}
            for k, v, tol in (("x", bx, TYPED_VS_LOG_IN), ("y", by, TYPED_VS_LOG_IN)):
                if typed[k] is not None and abs(typed[k] - v) > tol:
                    warnings.append("Row %d: believed_%s_in %.2f does not match the log's %s=%.2f (line %d). "
                                    "Used the log." % (row["row"], k, typed[k], k, v, ent["line"]))
            if typed["theta"] is not None and abs(wrap180(typed["theta"] - bt)) > TYPED_VS_LOG_DEG:
                warnings.append("Row %d: believed_theta_deg %.1f does not match the log's theta=%.1f (line %d). "
                                "Used the log." % (row["row"], typed["theta"], bt, ent["line"]))
        else:
            ent = None
            bx = typed["x"] if typed["x"] is not None else 0.0
            by = typed["y"] if typed["y"] is not None else 0.0
            bt = typed["theta"] if typed["theta"] is not None else 360.0 * sgn
            src = {k: ("typed" if typed[k] is not None else "assumed") for k in ("x", "y", "theta")}
            if "assumed" in (src["x"], src["y"]) or (row["heading"] is not None and src["theta"] == "assumed"):
                assumed += 1
        ex, ey = row["right"] - bx, row["forward"] - by
        r = {"row": row["row"], "line": row["line"], "dir": d,
             "log_line": ent["line"] if ent else None, "log_run": ent["run"] if ent else None,
             "timeouts": ent["timeouts"] if ent else None,
             "real_right_in": row["right"], "real_forward_in": row["forward"], "real_heading_deg": row["heading"],
             "turned": row["turned"],
             "believed_x_in": bx, "believed_y_in": by, "believed_theta_deg": bt, "believed_from": src,
             "error_right_in": ex, "error_forward_in": ey, "error_in": math.hypot(ex, ey),
             "heading_error_deg": wrap180(row["heading"] - bt) if row["heading"] is not None else None,
             "corner_spacing_measured_in": row["corner_spacing_measured"]}
        r["e_position"] = fit_position(ex, ey, side, d)[0]
        r["e_heading"] = e_from_heading(bt, row["heading"], d) if row["heading"] is not None else None
        if math.hypot(row["right"], row["forward"]) > SUSPICIOUS_IN:
            warnings.append("Row %d (line %d): the robot ended %.1f in from its start mark. That is a lot: check "
                            "the measurement is in inches and from the right mark."
                            % (row["row"], row["line"], math.hypot(row["right"], row["forward"])))
        if row["heading"] is not None and abs(wrap180(row["heading"])) > SUSPICIOUS_DEG:
            warnings.append("Row %d (line %d): a heading of %.1f deg is a lot for this test: check it."
                            % (row["row"], row["line"], wrap180(row["heading"])))
        runs.append(r)

    # heading_deg read by hand: the step it was read to sets the smallest ± its e can have (HEADING_STEPS_DEG)
    step, head_floor = None, None
    if meas["form"] == "simple":
        hv = [r["real_heading_deg"] for r in runs if r["real_heading_deg"] is not None]
        if hv:
            step = heading_step(hv)
            head_floor = step / math.sqrt(12.0) / 360.0

    # Each direction.
    res = {}
    for d in DIRS:
        rs = [r for r in runs if r["dir"] == d]
        res[d] = summarise(d, rs, side, scale[d], hints, meas["form"], head_floor, args.use_heading) if rs else None

    for d in DIRS:
        x = res[d]
        if x is not None and (abs(x["e_position"]) > MAX_BELIEVABLE_E or
                              (x["e_heading"] is not None and abs(x["e_heading"]) > MAX_BELIEVABLE_E)):
            raise InputError("The %s runs ended %.1f in from where the robot thought it was. To explain that the "
                             "inertial sensor would have to be %.0f%% off, and no inertial scale error can explain "
                             "something that size. Check the measurements are in inches, and the signs (right +, "
                             "forward +, heading clockwise +)."
                             % (DIR_WORD[d], x["mean_error_in"],
                                100 * max(abs(x["e_position"]), abs(x["e_heading"] or 0.0))))

    # The classic UMBmark score: the larger of the two cluster-centre distances. After the new scales, what is
    # left over plus the scatter of a new set of runs' centre (rms / sqrt(n)): without the scatter the prediction
    # came out low (tools/sim/umb_e2e.py: 0.17 in predicted, 0.25 in found)
    have = [res[d] for d in DIRS if res[d] is not None]
    score = {"now_in": max(x["mean_error_in"] for x in have),
             "predicted_after_in": max(x["predicted_after_in"] for x in have),
             "directions": [x["dir"] for x in have]}

    # Heading drift standing still.
    still = None
    if used:
        vals = [e["still_dps"] for e in used if e["still_dps"] is not None]
        tms = [e["t_ms"] for e in used if e["t_ms"] is not None]
        if vals:
            sd = mean(vals)
            t_s = mean(tms) / 1000.0 if tms else None
            still = {"still_dps": sd, "worst_dps": max(vals, key=abs), "warning": abs(sd) > STILL_DPS_WARN
                     or any(abs(v) > STILL_DPS_WARN for v in vals), "square_s": t_s}
            if t_s:
                drift = sd * t_s
                still["drift_deg"] = drift
                still["equivalent_e_cw"] = drift / 360.0
                still["equivalent_e_ccw"] = -drift / 360.0

    # Everything that holds the constants back, worst first. Each says what to do about it
    held = []
    for d in DIRS:
        if res[d] is not None:
            held += res[d]["holds"]
    held += ["Hint: " + h for h in hints]
    # src/umbmark.cpp ended a run at its first timed-out move from 2026-11-22; a line from an older build can still
    # say timeouts > 0. Such a run carried on from wherever the move stopped: it is not the square the others are
    for r in runs:
        if r["timeouts"] is not None and r["timeouts"] > 0:
            held.append("Row %d is run=%s (log line %d), where %d move(s) ran out of time ('!' on the controller). "
                        "That run is not a proper square: drive it again and put the new run in its place."
                        % (r["row"], r["log_run"], r["log_line"], r["timeouts"]))
    # The drift: it reads as a scale error, + clockwise and - counter-clockwise, and is not taken out (still_dps
    # comes from 2 s standing still: taking it out would multiply its noise by about 12)
    if still is not None and still["warning"]:
        msg = ("The heading crept %.3f deg/s while the robot stood still (worst run %.3f; over %.2f is too much)."
               % (still["still_dps"], still["worst_dps"], STILL_DPS_WARN))
        if still.get("drift_deg") is not None:
            msg += (" Over a %.0f s square that is about %.1f deg, which this test reads as %s clockwise and %s "
                    "counter-clockwise." % (still["square_s"], still["drift_deg"], pct(still["equivalent_e_cw"]),
                                            pct(still["equivalent_e_ccw"])))
        msg += (" Restart the robot and let the inertial sensor calibrate with the robot completely still, then "
                "drive the squares again.")
        held.append(msg)

    believed_from = "log" if used else ("assumed zero" if assumed == len(runs) else
                                         ("typed" if assumed == 0 else "typed, some assumed zero"))
    measured = [d for d in DIRS if res[d] is not None]
    settled = (not held and all(res[d]["constant_given"] and res[d]["within_scatter"] and res[d]["leftover_small"]
                                for d in measured))
    return {"side": side, "side_from": side_from, "scale": scale, "scale_from": scale_from, "runs": runs,
            "dirs": res, "score": score, "still": still, "warnings": warnings, "hints": hints,
            "assumed_rows": assumed, "believed_from": believed_from, "held_back": held, "settled": settled,
            "heading_step_deg": step}


def summarise(d, rs, side, scale, hints, form, head_floor, use_heading):
    word = DIR_WORD[d].capitalize()
    n = len(rs)
    ex = mean(r["error_right_in"] for r in rs)
    ey = mean(r["error_forward_in"] for r in rs)
    dist = [math.hypot(r["error_right_in"] - ex, r["error_forward_in"] - ey) for r in rs]
    # Plain RMS (divided by n). Dividing by n - 1 was tried: with two runs the "RMS" came out bigger than the
    # largest distance, which reads as a mistake. The slip verdict needs 3 runs or more anyway
    rms = math.sqrt(sum(x * x for x in dist) / n) if n >= 2 else None
    out = {"dir": d, "n": n, "mean_error_right_in": ex, "mean_error_forward_in": ey,
           "mean_error_in": math.hypot(ex, ey), "scatter_rms_in": rms,
           "scatter_max_in": max(dist) if n >= 2 else None, "scale_current": scale, "holds": []}
    holds = out["holds"]
    e_pos, left_pos = fit_position(ex, ey, side, d)
    out["e_position"], out["e_position_se"] = e_pos, floor_se(std_error(r["e_position"] for r in rs))
    out["heading_error_predicted_deg"] = heading_error_model(e_pos, d)
    hs = [r for r in rs if r["e_heading"] is not None]
    out["heading_n"] = len(hs)
    if hs:
        out["mean_heading_error_deg"] = mean(r["heading_error_deg"] for r in hs)
        out["e_heading"] = mean(r["e_heading"] for r in hs)
        out["e_heading_se"] = floor_se(std_error(r["e_heading"] for r in hs))
        if head_floor is not None and out["e_heading_se"] is not None:
            out["e_heading_se"] = max(out["e_heading_se"], head_floor)
    else:
        out["mean_heading_error_deg"] = out["e_heading"] = out["e_heading_se"] = None

    # Which e to use. Both agree: weight each by how scattered it is. They disagree: no number, unless
    # --use-heading. Until 2026-11-22 the heading won, but in a review it was the broken input as often as not
    # (two corners typed in the wrong order, a comma typed for a decimal point), and the tool cannot tell which
    sp, sh = out["e_position_se"], out["e_heading_se"]
    out["disagree"] = False
    if out["e_heading"] is None:
        out["agree"], e, se, used_from = None, e_pos, sp, "position"
    else:
        eh = out["e_heading"]
        lim = AGREE_FLOOR + AGREE_SIGMAS * math.sqrt((sp or 0.0) ** 2 + (sh or 0.0) ** 2)
        out["agree"] = abs(e_pos - eh) <= lim
        if out["agree"]:
            if sp is not None and sh is not None:
                wp, wh = 1.0 / sp ** 2, 1.0 / sh ** 2
                e = (wp * e_pos + wh * eh) / (wp + wh)
                se = 1.0 / math.sqrt(wp + wh)
            else:
                e, se = (e_pos + eh) / 2.0, None
            used_from = "both"
        else:
            out["disagree"] = True
            e, se, used_from = eh, sh, "heading"
            flipped = mean(e_from_heading(r["believed_theta_deg"], -r["real_heading_deg"], d) for r in hs)
            if form == "simple" and abs(flipped - e_pos) <= lim:
                hints.append("%s: the heading and the position agree if heading_deg had the other sign. "
                             "Clockwise is +: check how the heading was measured." % word)
            elif not use_heading and n >= MIN_RUNS_CONSTANT:
                why = ("%s: the position says %s and the heading %s: they disagree. Either something was measured "
                       "wrong (the heading's sign, clockwise +; the two corners typed in the wrong order; a comma "
                       "typed for a decimal point; a column swapped), or the robot does not spin about its tracking "
                       "centre" % (word, pct_pm(e_pos, sp), pct_pm(eh, sh)))
                if e_pos * eh > 0 and abs(e_pos) < abs(eh):
                    why += (": then the position comes out smaller than the heading, as here (robot-config.cpp's "
                            "[ODOMCAL] note puts the spin point 5.8 in behind the centre)")
                elif e_pos * eh > 0:
                    why += ": then the position comes out bigger than the heading if the spin point is in front"
                holds.append(why + ". Measure again, or, once the heading is checked, run again with --use-heading.")
    out["e_used"], out["e_used_se"], out["e_used_from"] = e, se, used_from
    out["within_scatter"] = se is not None and abs(e) <= SETTLED_SIGMAS * se
    out["scale_new"] = scale / (1.0 + e)
    out["sensor_error"] = (1.0 + e) / scale - 1.0
    mx, my = square_end(e, side, d)
    lx, ly = ex - mx, ey - my
    out["leftover_right_in"], out["leftover_forward_in"], out["leftover_in"] = lx, ly, math.hypot(lx, ly)
    small = max(LEFTOVER_SMALL_IN, 2.0 * rms / math.sqrt(n)) if rms is not None else LEFTOVER_SMALL_IN
    out["leftover_small"] = out["leftover_in"] <= small
    out["predicted_after_in"] = math.sqrt(out["leftover_in"] ** 2 + (rms * rms / n if rms is not None else 0.0))

    # Signs swapped by the person measuring?
    if out["leftover_in"] > SIGN_HINT_MIN_IN:
        for label, (vx, vy) in (("right and left swapped", (-ex, ey)), ("forward and back swapped", (ex, -ey)),
                                ("right_in and forward_in swapped", (ey, ex))):
            _, lv = fit_position(vx, vy, side, d)
            if math.hypot(*lv) < SIGN_HINT_RATIO * out["leftover_in"]:
                hints.append("%s: the error would fit an inertial scale error far better with %s. Check how "
                             "the runs were measured (right +, forward +, as the robot faced at the start)."
                             % (word, label))
                break

    # Which way did it end turned? A global sign flip (the mark measured from the robot) or right_in and forward_in
    # swapped give numbers that fit a scale error of the other sign perfectly: nothing in them can tell. heading_deg
    # measured with a protractor can (when it agrees with the position), and so can the turned column, in any form.
    # The corners form's heading comes from the same four numbers, so only turned checks it
    tc = turned_check(d, rs, e)
    out["turned"] = tc
    # --use-heading: the student says the heading was checked, so it counts as seen
    confirmed_by_heading = form == "simple" and (out["agree"] is True or (out["disagree"] and use_heading))
    # a disagreement already holds the numbers back (or names the heading's sign): the checks below would only
    # repeat it, worded from the heading's number
    quiet = out["disagree"] and not use_heading
    if tc["disagree"] > 0 and tc["disagree"] >= tc["agree"]:
        holds.append("%s: on %d run(s) the robot was seen turned the other way from what the numbers say (they say "
                     "it ended about %.1f deg %s; the turned column says %s). The signs are the wrong way round "
                     "somewhere: measure FROM the start mark TO the robot's mark, right +, forward +, and check "
                     "right_in and forward_in are in the right columns (corners form: left corner first)."
                     % (word, tc["disagree"], abs(tc["predicted_deg"]),
                        "clockwise" if tc["predicted_deg"] > 0 else "counter-clockwise",
                        "CCW" if tc["predicted_deg"] > 0 else "CW"))
    elif tc["visible"] and not confirmed_by_heading and tc["agree"] == 0 and n >= MIN_RUNS_CONSTANT and not quiet:
        if tc["unseen"]:
            holds.append("%s: a scale error of %s would leave the robot turned about %.1f deg %s by the end, but the "
                         "turned column says it was not visibly turned. Look again against a tape line along its side "
                         "(1 deg is 0.3 in over an 18 in robot) and write CW or CCW; if it really is straight, the "
                         "positions are off (see Slip) and the sign cannot be checked%s."
                         % (word, pct(e), abs(tc["predicted_deg"]),
                            "clockwise" if tc["predicted_deg"] > 0 else "counter-clockwise",
                            "; heading_deg would settle it" if form == "simple" else ""))
        else:
            holds.append("%s: a scale error of %s turns the robot about %.1f deg by the end, which you can see, but "
                         "nothing in the file says which way it really turned, so the sign cannot be checked. Fill in "
                         "the turned column (CW, CCW, or 0 if it is not visibly turned) for each run%s."
                         % (word, pct(e), abs(tc["predicted_deg"]),
                            ", or measure heading_deg" if form == "simple" else ""))
    # Too big to believe without a heading: centimetres read as inches gave 3 % in the review
    if abs(e) > PLAUSIBLE_E and not confirmed_by_heading and n >= MIN_RUNS_CONSTANT and not quiet:
        holds.append("%s: %s is more than %g %%, and V5 inertial sensors are usually 1 to 1.4 %% off. Check every "
                     "distance is in inches (centimetres give numbers 2.5 times too big). If they are, measure "
                     "heading_deg as well (simple form): the number is printed when the heading agrees."
                     % (word, pct(e), 100 * PLAUSIBLE_E))
    out["constant_given"] = n >= MIN_RUNS_CONSTANT

    if n < MIN_RUNS_SCATTER:
        out["slip"] = ("only %d run(s): the scatter cannot be judged (drive at least %d, %d is better)"
                       % (n, MIN_RUNS_SCATTER, RECOMMENDED_RUNS))
        out["slip_class"] = "unknown"
    else:
        # What the scale explains, against what it does not (left over) and the scatter. "Calibration will fix most
        # of it" was said from the cluster's size alone, also when the scale explained none of it (review: right and
        # left swapped, 2.6 in left over)
        explained = math.hypot(mx, my)
        if explained > out["leftover_in"] and explained > rms:
            fix = "the scale explains most of the repeatable error: calibration will fix most of it"
        elif out["leftover_in"] > rms:
            fix = "repeatable, but the scale does not explain it: see 'Left over'"
        else:
            fix = "what is left is mostly scatter, which calibration cannot fix"
        if rms < SLIP_REPEATABLE_IN:
            out["slip"], out["slip_class"] = "little slip; " + fix, "repeatable"
        elif rms > SLIP_BAD_IN:
            out["slip"], out["slip_class"] = "slip or a lifting wheel: calibration will not fix this", "slip"
        else:
            out["slip"] = "borderline: some slip, %.1f in or so from run to run; %s" % (rms, fix)
            out["slip_class"] = "borderline"
    return out


# ---------------------------------------------------------------------------
# Text output
# ---------------------------------------------------------------------------
def pct(e):
    return "%+.2f %%" % (e * 100.0)


def pct_pm(e, se):
    """'+1.19 ± 0.03 %', or '+1.19 % (no ±: one run)'"""
    if se is None:
        return "%+.2f %% (no ±: one run)" % (e * 100.0)
    return "%+.2f ± %.2f %%" % (e * 100.0, se * 100.0)


def coverage_words(n):
    """How often the truth is within the ± and twice it, for a ± from n runs"""
    if n < 2:
        return None
    c1, c2 = COVERAGE_PCT.get(n - 1, COVERAGE_PCT[max(COVERAGE_PCT)])
    return c1, c2


def sgnf(x, nd=2):
    return ("%+." + str(nd) + "f") % x if x is not None else "-"


def lr(x):
    return "%.2f in %s" % (abs(x), "right" if x >= 0 else "left")


def fb(y):
    return "%.2f in %s" % (abs(y), "forward" if y >= 0 else "back")


def run_table(res, form):
    have_log = any(r["log_run"] is not None for r in res["runs"])
    have_turned = any(r["turned"] is not None for r in res["runs"])
    hdr = ["row", "dir"] + (["run"] if have_log else []) + ["real right", "fwd", "hdg"] + \
        (["seen"] if have_turned else []) + ["thought x", "y", "hdg", "error right", "fwd", "hdg"]
    lines = []
    for r in res["runs"]:
        bt = r["believed_theta_deg"]
        row = [str(r["row"]), r["dir"]] + ([str(r["log_run"])] if have_log else []) + [
            sgnf(r["real_right_in"]), sgnf(r["real_forward_in"]), sgnf(r["real_heading_deg"], 1)] + \
            ([r["turned"] or "-"] if have_turned else []) + [
            sgnf(r["believed_x_in"]), sgnf(r["believed_y_in"]), sgnf(wrap180(bt), 1),
            sgnf(r["error_right_in"]), sgnf(r["error_forward_in"]), sgnf(r["heading_error_deg"], 1)]
        lines.append(row)
    w = [max(len(hdr[i]), *(len(x[i]) for x in lines)) for i in range(len(hdr))]
    left = {0, 1}

    def f(cols):
        return "  " + "  ".join(c.ljust(w[i]) if i in left else c.rjust(w[i]) for i, c in enumerate(cols)).rstrip()
    return [f(hdr)] + [f(x) for x in lines]


def report(meas, res, args):
    out = []
    n_cw = sum(1 for r in res["runs"] if r["dir"] == "CW")
    n_ccw = len(res["runs"]) - n_cw
    out.append("umbmark: %s  (%s form, %d run(s): %d clockwise, %d counter-clockwise)"
               % (meas["name"], meas["form"], len(res["runs"]), n_cw, n_ccw))
    src = {"log": "from the log", "default": "default", "--side": "from --side"}
    out.append("Side %g in (%s). The runs were made with scales CW %.4f, CCW %.4f (%s)."
               % (res["side"], src.get(res["side_from"], res["side_from"]), res["scale"]["CW"],
                  res["scale"]["CCW"], "from the log" if "log" in res["scale_from"].values()
                  else ("from the command line" if any(v.startswith("--") for v in res["scale_from"].values())
                        else "default: give --scale-cw and --scale-ccw if they were not 1")))
    if args.log:
        runs = [r["log_run"] for r in res["runs"]]
        lines = [r["log_line"] for r in res["runs"]]
        out.append("Where the robot thought it was: %s, %d [UMB] lines (run=%s ... run=%s, lines %d-%d)."
                   % (os.path.basename(args.log), len(runs), runs[0], runs[-1], min(lines), max(lines)))
    elif res["believed_from"] == "typed":
        out.append("Where the robot thought it was: the believed_* columns.")
    if res["assumed_rows"]:
        out.append("")
        out.append("!! No log and no believed_x_in / believed_y_in / believed_theta_deg for %d row(s): assumed the"
                   % res["assumed_rows"])
        out.append("!! robot thought it ended exactly at (0, 0) facing 0. Moves stop up to about %.1f in and turns"
                   % STOP_TOL_IN)
        out.append("!! about %g deg from their targets, so every number below can be off by that much."
                   % STOP_TOL_DEG)
        out.append("!! Run with --log tune.log (the capture of these runs) to fix it.")
    if res["held_back"]:
        out.append("")
        out.append("!! NO NEW NUMBERS THIS TIME. Fix these first, then run this again:")
        for k, h in enumerate(res["held_back"], start=1):
            out.append("!! %d. %s" % (k, h))
    if res["warnings"]:
        out.append("")
        out.append("Check these first:")
        for w in res["warnings"]:
            out.append("  - " + w)

    out.append("")
    out.append("Each run, in inches and degrees (error = where it really ended minus where it thought it was):")
    out += run_table(res, meas["form"])

    for d in DIRS:
        x = res["dirs"][d]
        out.append("")
        if x is None:
            out.append("%s: no runs." % DIR_WORD[d].capitalize())
            continue
        out.append("%s, %d run(s)" % (DIR_WORD[d].capitalize(), x["n"]))
        out.append("  Repeatable error (cluster centre): %.2f in  (%s, %s)"
                   % (x["mean_error_in"], lr(x["mean_error_right_in"]), fb(x["mean_error_forward_in"])))
        if x["scatter_rms_in"] is not None:
            out.append("  Scatter about it (slip): RMS %.2f in, largest %.2f in"
                       % (x["scatter_rms_in"], x["scatter_max_in"]))
        else:
            out.append("  Scatter about it (slip): needs 2 or more runs")
        if x["mean_heading_error_deg"] is not None:
            out.append("  Heading error: %+.1f deg on average (measured on %d of %d runs)"
                       % (x["mean_heading_error_deg"], x["heading_n"], x["n"]))
        else:
            out.append("  Heading error: not measured. If the scale is the whole story it is about %+.1f deg."
                       % x["heading_error_predicted_deg"])

    out.append("")
    out.append("Inertial scale error (+ = the robot thinks it turned more than it really did):")
    for d in DIRS:
        x = res["dirs"][d]
        if x is None:
            out.append("  %-18s not measured" % (DIR_WORD[d] + ":"))
            continue
        line = "  %-18s position %s" % (DIR_WORD[d] + ":", pct_pm(x["e_position"], x["e_position_se"]))
        if x["e_heading"] is not None:
            line += "   heading %s   %s" % (pct_pm(x["e_heading"], x["e_heading_se"]),
                                           "agree" if x["agree"] else "DISAGREE")
        out.append(line)
    ns = sorted({x["n"] for x in res["dirs"].values() if x is not None and x["n"] >= 2})
    cov = coverage_words(ns[0]) if ns else None
    if cov:
        out.append("  (± comes from the scatter between runs. With %d runs the true value is within it about %d %% of"
                   % (ns[0], cov[0]))
        out.append("  the time and within twice it about %d %%.)" % cov[1])
    if res["heading_step_deg"] is not None:
        out.append("  (heading read to %g deg: the heading's ± is never under %.2f %%, the rounding of one reading)"
                   % (res["heading_step_deg"], 100 * res["heading_step_deg"] / math.sqrt(12.0) / 360.0))
    for d in DIRS:
        x = res["dirs"][d]
        if x is None or x["e_used_from"] == "position":
            continue
        if x["e_used_from"] == "both":
            out.append("  %s: used %s, both together." % (DIR_WORD[d].capitalize(), pct_pm(x["e_used"], x["e_used_se"])))
        elif args.use_heading:
            out.append("  %s: they disagree by %.2f %%. Used the heading (%s), as --use-heading asks."
                       % (DIR_WORD[d].capitalize(), abs(x["e_position"] - x["e_heading"]) * 100, pct(x["e_used"])))
    for d in DIRS:
        x = res["dirs"][d]
        if x is not None and abs(x["scale_current"] - 1.0) > 1e-9:
            out.append("  %s: with the current scale %.4f, the sensor itself reads %s."
                       % (DIR_WORD[d].capitalize(), x["scale_current"], pct(x["sensor_error"])))

    out.append("")
    measured = [d for d in DIRS if res["dirs"][d] is not None]
    if res["held_back"]:
        out.append("No numbers to type in this time: see the top. Keep the scales you have.")
    else:
        out.append("Type these into include/robot-config.hpp, in place of the scales these runs were made with")
        out.append("(CW %.4f, CCW %.4f: check the file still says that, or these numbers are wrong):"
                   % (res["scale"]["CW"], res["scale"]["CCW"]))
        for d, name in (("CW", "IMU_SCALE_CLOCKWISE"), ("CCW", "IMU_SCALE_COUNTER_CLOCKWISE")):
            x = res["dirs"][d]
            if x is None:
                out.append("  constexpr float %s = %.4ff; // not measured: no %s runs, left as it is"
                           % (name, res["scale"][d], DIR_WORD[d]))
            elif not x["constant_given"]:
                out.append("  constexpr float %s = %.4ff; // left as it is: only %d %s run(s), it needs %d"
                           % (name, res["scale"][d], x["n"], DIR_WORD[d], MIN_RUNS_CONSTANT))
            else:
                out.append("  constexpr float %s = %.4ff;" % (name, x["scale_new"]))
        if res["settled"]:
            out.append("Every direction is within the scatter between runs (under %g times its ±) and nothing is left"
                       % SETTLED_SIGMAS)
            out.append("over: this test cannot tell the scales from right. Keep the ones you have, or type in the")
            out.append("numbers above: either way the difference is too small for this test to see. This test is done.")
        else:
            out.append("Then drive the squares again with the new scales: the error should come out close to 0. A")
            out.append("second round also catches what other small errors hid in the first.")
            for d in measured:
                x = res["dirs"][d]
                if x["within_scatter"] and x["constant_given"]:
                    out.append("%s is already within the scatter between runs: its new number is optional."
                               % DIR_WORD[d].capitalize())

    out.append("")
    out.append("Left over (repeatable error the scale does not explain):")
    big = False
    for d in DIRS:
        x = res["dirs"][d]
        if x is None:
            continue
        out.append("  %-18s %.2f in  (%s, %s)%s" % (DIR_WORD[d] + ":", x["leftover_in"], lr(x["leftover_right_in"]),
                                                   fb(x["leftover_forward_in"]),
                                                   "  small" if x["leftover_small"] else ""))
        big = big or not x["leftover_small"]
    if big:
        out.append("  Not small: something besides the scale. Wrong wheel offsets and a wheel not quite parallel")
        out.append("  mostly cancel round this square. More than a few tenths, look at: the signs and columns (a")
        out.append("  hint at the top says if a mix-up fits), the mark measured not being over the tracking centre,")
        out.append("  the floor (a seam or slope on the route), slip that repeats in the same place, or no log.")
    else:
        out.append("  The scale explains nearly all of the repeatable error.")
    out.append("  A robot that spins about a point away from its tracking centre leaves NOTHING over here: it makes")
    out.append("  the position's scale error smaller (a quarter smaller for 5.8 in). Only the heading shows it.")
    out.append("  Wheel SIZE cannot show up in this test (it cancels round a closed square): check it with a tape.")

    sc = res["score"]
    out.append("")
    if len(sc["directions"]) == 2:
        out.append("UMBmark score (the larger cluster-centre distance): %.2f in now, about %.2f in predicted"
                   % (sc["now_in"], sc["predicted_after_in"]))
        out.append("after the new scales (what is left over plus the scatter of a new set of runs).")
    else:
        out.append("UMBmark score: needs both directions; %s only: %.2f in now, about %.2f in after."
                   % (DIR_WORD[sc["directions"][0]], sc["now_in"], sc["predicted_after_in"]))

    out.append("")
    out.append("Slip:")
    for d in DIRS:
        x = res["dirs"][d]
        if x is None:
            continue
        rms = (" RMS %.2f in:" % x["scatter_rms_in"]) if x["scatter_rms_in"] is not None else ""
        out.append("  %-18s%s %s" % (DIR_WORD[d] + ":", rms, x["slip"]))

    st = res["still"]
    if st is not None:
        out.append("")
        if st["warning"]:
            out.append("Heading drift standing still: %.3f deg/s, too much (see the top)." % st["still_dps"])
        else:
            extra = ""
            if st.get("drift_deg") is not None:
                extra = " (about %.2f deg over a %.0f s square: up to %.2f %% on the scale, which is in the numbers" \
                        " above)" % (st["drift_deg"], st["square_s"], 100 * abs(st["equivalent_e_cw"]))
            out.append("Heading drift standing still: %.3f deg/s, fine%s." % (st["still_dps"], extra))
    elif args.log:
        out.append("")
        out.append("Heading drift standing still: the log lines have no still_dps.")
    return out


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------
def make_plot(plt, res, path):
    fig, ax = plt.subplots(figsize=(7.6, 7.6))
    fig.patch.set_facecolor(C_SURFACE)
    ax.set_facecolor(C_SURFACE)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(C_AXIS)
    ax.tick_params(colors=C_INK2, labelsize=9)
    ax.grid(True, color=C_GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    xs, ys = [0.0], [0.0]
    emax = max([abs(res["dirs"][d]["e_used"]) for d in DIRS if res["dirs"][d] is not None] + [0.005])
    span = min(FIT_LIMIT / 2, 1.3 * emax + 0.003)
    for d, color, marker in (("CW", SERIES[0], "o"), ("CCW", SERIES[1], "^")):
        rs = [r for r in res["runs"] if r["dir"] == d]
        x = res["dirs"][d]
        if not rs:
            continue
        px = [r["error_right_in"] for r in rs]
        py = [r["error_forward_in"] for r in rs]
        xs += px
        ys += py
        ax.scatter(px, py, s=46, color=color, marker=marker, edgecolors=C_SURFACE, linewidths=0.8, zorder=4,
                   label="%s runs" % DIR_WORD[d])
        # where a pure scale error would put this direction's cluster
        es = [-span + 2 * span * k / 60.0 for k in range(61)]
        curve = [square_end(e, res["side"], d) for e in es]
        ax.plot([c[0] for c in curve], [c[1] for c in curve], color=color, lw=1.0, ls=(0, (4, 3)), alpha=0.7,
                zorder=2, label="where a %s scale error puts it" % DIR_WORD[d])
        for k in range(-int(span * 100), int(span * 100) + 1):
            if k == 0:
                continue
            cx, cy = square_end(k / 100.0, res["side"], d)
            ax.plot([cx], [cy], marker="o", ms=3, color=color, alpha=0.7, zorder=2)
            if abs(k) == int(span * 100):
                ax.annotate("%+d%%" % k, (cx, cy), xytext=(4, 2), textcoords="offset points", fontsize=7.5,
                            color=color)
        # centres and the origin are named in the legend: a label next to them always ended up on a run
        ax.scatter([x["mean_error_right_in"]], [x["mean_error_forward_in"]], s=260, color=color, marker=marker,
                   edgecolors=C_INK, linewidths=1.4, zorder=5,
                   label="%s centre, %.2f in from no error" % (DIR_WORD[d], x["mean_error_in"]))
        xs.append(x["mean_error_right_in"])
        ys.append(x["mean_error_forward_in"])
    ax.plot([0], [0], marker="+", ms=16, mew=1.6, color=C_INK, ls="none", zorder=6, label="no error")
    pad = 0.6
    lo_x, hi_x, lo_y, hi_y = min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad
    half = max(hi_x - lo_x, hi_y - lo_y) / 2.0
    cxm, cym = (lo_x + hi_x) / 2.0, (lo_y + hi_y) / 2.0
    ax.set_xlim(cxm - half, cxm + half)
    ax.set_ylim(cym - half, cym + half)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("error to the right (in)", color=C_INK2)
    ax.set_ylabel("error forward (in)", color=C_INK2)
    ax.legend(loc="best", frameon=False, fontsize=8, labelspacing=0.9)
    sub = []
    for d, name in (("CW", "CW"), ("CCW", "CCW")):
        x = res["dirs"][d]
        if x is not None:
            sub.append("%s scale error %s" % (name, pct(x["e_used"])) +
                       ("" if res["held_back"] else " -> %.4f" % x["scale_new"]))
    if res["held_back"]:
        sub.append("(no new numbers: see the report)")
    fig.text(0.08, 0.975, "UMBmark square: where the robot really ended minus where it thought",
             fontsize=11.5, color=C_INK, weight="bold", va="top")
    fig.text(0.08, 0.945, "   ".join(sub) + "   (dots on the dashed lines: every 1%)", fontsize=8.5,
             color=C_INK2, va="top")
    fig.subplots_adjust(left=0.11, right=0.97, top=0.91, bottom=0.08)
    fig.savefig(path, dpi=PLOT_DPI, facecolor=C_SURFACE)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def constants_of(res):
    out = {}
    for d, name in (("CW", "IMU_SCALE_CLOCKWISE"), ("CCW", "IMU_SCALE_COUNTER_CLOCKWISE")):
        x = res["dirs"][d]
        out[name] = round(x["scale_new"], 4) if x is not None and x["constant_given"] else res["scale"][d]
    return out


def to_json(meas, res, args, plot_path, plot_msg):
    held = bool(res["held_back"])
    return {"tool": "umbmark", "version": 2, "measurements": args.measurements, "form": meas["form"],
            "form_keys": meas["keys"], "log": args.log, "session": args.session, "use_heading": args.use_heading,
            "side_in": res["side"], "side_from": res["side_from"],
            "scale_current": res["scale"], "scale_from": res["scale_from"], "believed_from": res["believed_from"],
            "assumed_rows": res["assumed_rows"], "runs": res["runs"], "directions": res["dirs"],
            # None while anything holds them back; what they would have been is kept apart, for scripts only
            "constants": None if held else constants_of(res),
            "constants_held_back": constants_of(res) if held else None,
            "held_back": res["held_back"], "settled": res["settled"], "heading_step_deg": res["heading_step_deg"],
            "exit_status": EXIT_HELD_BACK if held else 0,
            "umbmark_score": res["score"], "still": res["still"], "warnings": res["warnings"],
            "hints": res["hints"], "plot": plot_path, "plot_skipped": plot_msg}


def main(argv=None):
    ap = argparse.ArgumentParser(description="Work out the inertial sensor's scale error from UMBmark square runs.")
    ap.add_argument("measurements", nargs="?", help="CSV of what was measured with the ruler, one row per run")
    ap.add_argument("--log", help="capture of the runs (pros terminal --output tune.log), for the [UMB] lines")
    ap.add_argument("--side", type=float, default=None,
                    help="side of the square in inches, if the log does not say (default %g)" % DEFAULT_SIDE_IN)
    ap.add_argument("--scale-cw", type=float, default=None,
                    help="IMU_SCALE_CLOCKWISE the runs were made with, if the log does not say (default 1)")
    ap.add_argument("--scale-ccw", type=float, default=None,
                    help="IMU_SCALE_COUNTER_CLOCKWISE the runs were made with, if the log does not say (default 1)")
    ap.add_argument("--session", type=int, default=None, metavar="N",
                    help="the set of runs in the log to match (sets are numbered from 1 in log order; a new set starts "
                         "where the scales change). Needed when your rows fit more than one")
    ap.add_argument("--use-heading", action="store_true",
                    help="when position and heading disagree, take the heading's number (for a robot that does not "
                         "spin about its tracking centre; check the heading first)")
    ap.add_argument("--json", metavar="FILE", help="also write the results as JSON")
    ap.add_argument("--plot", metavar="FILE.png", help="draw where each run ended (needs matplotlib)")
    ap.add_argument("--template", choices=sorted(TEMPLATES), help="print a measurements file to fill in, and stop")
    ap.add_argument("--debug", action="store_true", help="show Python's own error report if something breaks")
    args = ap.parse_args(argv)
    try:
        sys.stdout.reconfigure(errors="replace")
    except Exception:
        pass
    if args.template:
        sys.stdout.write(TEMPLATES[args.template])
        return 0
    if not args.measurements:
        ap.error("give the measurements file (or --template simple to get one to fill in)")
    try:
        meas = read_measurements(args.measurements)
        used = None
        if args.log:
            used = match_log(meas["rows"], read_log(args.log), args.log, meas["name"], meas["warnings"],
                             args.session)
        elif args.session is not None:
            raise InputError("--session picks a set of runs in the log: give the log with --log.")
        res = analyse(meas, used, args)
        plot_path, plot_msg = None, None
        if args.plot:
            try:
                import matplotlib
                matplotlib.use("Agg")
                import matplotlib.pyplot as plt
            except Exception as exc:
                plt = None
                plot_msg = "plot skipped: matplotlib is not available (%s)" % exc
            if plt is not None:
                try:
                    make_plot(plt, res, args.plot)
                    plot_path = args.plot
                except OSError as exc:
                    raise InputError("cannot write the plot %s: %s" % (args.plot, exc.strerror or exc))
        lines = report(meas, res, args)
        if plot_path:
            lines += ["", "Plot: " + plot_path]
        elif plot_msg:
            lines += ["", plot_msg]
        print("\n".join(lines))
        if args.json:
            try:
                with open(args.json, "w", encoding="utf-8") as fh:
                    json.dump(to_json(meas, res, args, plot_path, plot_msg), fh, indent=2)
            except OSError as exc:
                raise InputError("cannot write %s: %s" % (args.json, exc.strerror or exc))
            print("JSON: " + args.json)
        if res["held_back"]:
            return EXIT_HELD_BACK
    except InputError as exc:
        print("umbmark: " + str(exc), file=sys.stderr)
        return 2
    except Exception as exc:  # a bug in this tool, not the student's input
        if args.debug:
            raise
        print("umbmark: something went wrong inside the tool (%s: %s). Run again with --debug and show the "
              "programmers." % (type(exc).__name__, exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
