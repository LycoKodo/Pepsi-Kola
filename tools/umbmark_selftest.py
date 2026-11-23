#!/usr/bin/env python3
"""Self-test for umbmark.py.

    python3 tools/umbmark_selftest.py [--out DIR] [--verbose]

Three parts:

  1. The model. The closed form against the plain leg-by-leg sum, against the small-angle
     formulas the design started from, and every fit run backwards.
  2. A small simulator of the square, written here and nowhere else (it does not use tools/sim):
     a robot stepped every 5 ms with its real pose and the pose its odometry believes, the
     odometry worked out as LemLib does it (arc formula, inertial heading with setImuScale applied
     per step as in odom.cpp, a vertical and a horizontal tracking wheel with offsets). The robot
     turns on the spot until its believed heading is within a random part of 1 deg of the target,
     and drives to each believed point, stopping a random part of 0.5 in short. It checks that the
     exact model matches what the simulated robot does, and the claims in umbmark.py's notes:
     wheel size, wrong offsets, a wheel not parallel and an off-centre turning point cancel round
     the square; a steady drift reads as + clockwise and - counter-clockwise.
  3. End to end. Writes measurement files (both forms) and captures with [UMB] lines (noise,
     CRLF, colour codes, more lines than rows), runs umbmark.py on them as a subprocess, and checks
     the recovered errors, the constants it suggests, its verdicts, and its error messages. Also
     the mistakes a review found it gave confident wrong numbers for (a mark measured from the
     robot, columns swapped, centimetres, corners swapped, an old file against an appended log):
     each must now hold the numbers back (exit status 3) or stop with a message (exit status 2).
     The ± and the weighting of position against heading are checked exactly, so a ± that is too
     wide, or a heading that always wins, cannot pass.

Ends with 'selftest passed: N checks'. Exit status 0 = all passed.
"""

import argparse
import json
import statistics
import math
import os
import random
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, "umbmark.py")
sys.dont_write_bytecode = True      # importing umbmark must not leave a .pyc in the repository
sys.path.insert(0, HERE)
import umbmark as U  # noqa: E402

SIDE = 48.0
SEEN_DEG = 1.0          # a person can tell which way the robot ended turned from this far (the turned column)

# Simulator settings. Slow and simple on purpose: the claims are about geometry, not about the controllers.
DT_MOVE = 0.005         # s per step while moving
DT_STILL = 0.05         # s per step while standing still (only drift happens)
DRIVE_IPS = 25.0        # top speed on a leg, in/s
TURN_DPS = 150.0        # top turn rate, deg/s
K_TURN = 6.0            # turn rate = K_TURN x believed heading error (1/s), capped
MIN_TURN_DPS = 8.0      # ...but never slower than this, so a turn always finishes
K_STEER = 4.0           # on a leg, rotation rate = K_STEER x believed bearing error (1/s)
STEER_OFF_IN = 4.0      # stop steering this close to the point, as moveToPoint settles
K_DRIVE = 3.0           # leg speed = K_DRIVE x distance still to go + MIN_IPS, capped
MIN_IPS = 2.0
STILL_S, STOP_S, SETTLE_S = 2.0, 0.3, 1.0   # UMB_STILL_MS, UMB_STOP_MS, UMB_SETTLE_MS
MAX_STEPS = 200000


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------
class Checks:
    def __init__(self, verbose=False):
        self.n = 0
        self.failures = []
        self.verbose = verbose

    def ok(self, cond, msg):
        self.n += 1
        if not cond:
            self.failures.append(msg)
            print("FAIL: " + msg)
        elif self.verbose:
            print("ok: " + msg)
        return cond


# ---------------------------------------------------------------------------
# Part 2: the simulator
# ---------------------------------------------------------------------------
def wrap_rad(a):
    return math.atan2(math.sin(a), math.cos(a))


class Sim:
    """One robot. Real pose (X, Y, TH) and believed pose (x, y, th): x right, y forward at the start,
    headings in radians clockwise +. Offsets: the vertical wheel sits vert_x to the right of the tracking
    centre, the horizontal wheel horiz_y in front of it (negative = left / behind, as robot-config.cpp).
    *_odom are the offsets the odometry is told; centre is the point the robot turns about."""

    def __init__(self, e_cw=0.0, e_ccw=0.0, scale_cw=1.0, scale_ccw=1.0, drift_dps=0.0,
                 vert_x=-0.6, vert_x_odom=None, horiz_y=-2.23, horiz_y_odom=None,
                 vert_size=0.0, horiz_size=0.0, vert_yaw_deg=0.0, horiz_yaw_deg=0.0, centre=(0.0, 0.0),
                 slip=0.0, stop_in=0.5, stop_deg=1.0, seed=1, side=SIDE):
        self.e_cw, self.e_ccw, self.s_cw, self.s_ccw = e_cw, e_ccw, scale_cw, scale_ccw
        self.drift = math.radians(drift_dps)
        self.vx_real, self.hy_real = vert_x, horiz_y
        self.vx_odom = vert_x if vert_x_odom is None else vert_x_odom
        self.hy_odom = horiz_y if horiz_y_odom is None else horiz_y_odom
        self.vk, self.hk = vert_size, horiz_size
        self.va, self.ha = math.radians(vert_yaw_deg), math.radians(horiz_yaw_deg)
        self.centre = centre
        self.slip = slip
        self.stop_in, self.stop_deg = stop_in, stop_deg
        self.rng = random.Random(seed)
        self.side = side
        self.reset()

    def reset(self):
        self.X = self.Y = self.TH = 0.0
        self.x = self.y = self.th = 0.0
        self.t = 0.0

    def step(self, u, w, dt):
        """Drive centre moves forward at u (in/s) and the robot turns at w (rad/s, clockwise +) for dt."""
        cx, cy = self.centre
        vx, vy = -w * cy, u + w * cx                        # the tracking centre's velocity, robot frame
        dth = w * dt
        # tracking wheels: velocity at the wheel, along its rolling direction, times its size error
        vv = (vx, vy - w * self.vx_real)
        hv = (vx + w * self.hy_real, vy)
        dV = dt * (vv[0] * math.sin(self.va) + vv[1] * math.cos(self.va)) * (1.0 + self.vk)
        dH = dt * (hv[0] * math.cos(self.ha) - hv[1] * math.sin(self.ha)) * (1.0 + self.hk)
        if self.slip:
            dV += self.rng.gauss(0.0, self.slip * math.sqrt(abs(dV)))
            dH += self.rng.gauss(0.0, self.slip * math.sqrt(abs(dH) + 0.2 * abs(dV)))
        # real pose: exact for a constant twist over dt
        c = 2.0 * math.sin(dth / 2.0) / dth if dth else 1.0
        a = self.TH + dth / 2.0
        lx, ly = c * vx * dt, c * vy * dt
        self.X += lx * math.cos(a) + ly * math.sin(a)
        self.Y += -lx * math.sin(a) + ly * math.cos(a)
        self.TH += dth
        # inertial sensor, then setImuScale on the sign of each step's change (odom.cpp)
        raw = dth * (1.0 + (self.e_cw if dth > 0 else self.e_ccw)) + self.drift * dt
        if raw > 0:
            raw *= self.s_cw
        elif raw < 0:
            raw *= self.s_ccw
        # LemLib's arc formula
        if raw == 0:
            lx, ly = dH, dV
        else:
            s = 2.0 * math.sin(raw / 2.0)
            ly = s * (dV / raw + self.vx_odom)
            lx = s * (dH / raw - self.hy_odom)
        a = self.th + raw / 2.0
        self.x += lx * math.cos(a) + ly * math.sin(a)
        self.y += -lx * math.sin(a) + ly * math.cos(a)
        self.th += raw
        self.t += dt

    def pause(self, secs):
        n = max(1, int(round(secs / DT_STILL)))
        for _ in range(n):
            self.step(0.0, 0.0, secs / n)

    def turn_to(self, target_deg):
        now = math.degrees(self.th)
        tgt = now + U.wrap180(target_deg - now)
        thr = self.rng.uniform(0.15, self.stop_deg)
        for _ in range(MAX_STEPS):
            err = tgt - math.degrees(self.th)
            if abs(err) <= thr:
                return
            rate = max(-TURN_DPS, min(TURN_DPS, K_TURN * err))
            if abs(rate) < MIN_TURN_DPS:
                rate = math.copysign(MIN_TURN_DPS, err)
            self.step(0.0, math.radians(rate), DT_MOVE)
        raise RuntimeError("turn never finished")

    def move_to(self, tx, ty):
        thr = self.rng.uniform(0.05, self.stop_in)
        for _ in range(MAX_STEPS):
            dx, dy = tx - self.x, ty - self.y
            ahead = dx * math.sin(self.th) + dy * math.cos(self.th)
            if ahead <= thr:
                return
            w = 0.0
            if math.hypot(dx, dy) > STEER_OFF_IN:
                err = wrap_rad(math.atan2(dx, dy) - self.th)
                w = max(-math.radians(TURN_DPS), min(math.radians(TURN_DPS), K_STEER * err))
            self.step(min(DRIVE_IPS, K_DRIVE * ahead + MIN_IPS), w, DT_MOVE)
        raise RuntimeError("move never finished")

    def run(self, direction):
        """One square, as the robot routine drives it. Returns what the [UMB] line and the ruler would give."""
        self.reset()
        self.pause(STILL_S)
        still_dps = math.degrees(self.th) / STILL_S
        t0 = self.t
        L, s = self.side, U.turn_sign(direction)
        pts = [(0.0, L), (s * L, L), (s * L, 0.0), (0.0, 0.0)]
        heads = [90, 180, 270, 0] if direction == "CW" else [270, 180, 90, 0]
        for (px, py), h in zip(pts, heads):
            self.move_to(px, py)
            self.pause(STOP_S)
            self.turn_to(h)
            self.pause(STOP_S)
        t_ms = (self.t - t0) * 1000.0
        self.pause(SETTLE_S - STOP_S)
        h = U.wrap180(math.degrees(self.TH))
        return {"dir": direction, "right": self.X, "forward": self.Y, "heading": h, "turned": seen(h),
                "x": self.x, "y": self.y, "theta": math.degrees(self.th), "still_dps": still_dps, "t_ms": t_ms}


def run_errors(r):
    return r["right"] - r["x"], r["forward"] - r["y"]


def sim_fit(sim_kwargs, seed=3):
    """One CW and one CCW square; the position and heading fits for each."""
    out = {}
    for k, d in enumerate(("CW", "CCW")):
        r = Sim(seed=seed + k, **sim_kwargs).run(d)
        ex, ey = run_errors(r)
        e_pos, left = U.fit_position(ex, ey, SIDE, d)
        out[d] = {"run": r, "E": (ex, ey), "e_pos": e_pos, "left": math.hypot(*left),
                  "e_head": U.e_from_heading(r["theta"], r["heading"], d),
                  "H": U.wrap180(r["heading"] - r["theta"])}
    return out


# ---------------------------------------------------------------------------
# Part 3 helpers: files and the tool
# ---------------------------------------------------------------------------
def eighth(x):
    return round(x * 8.0) / 8.0


def ruler(x, style):
    """A ruler reading written the way a student might: '1 3/8', '-11/8' or '1.375'."""
    v = eighth(x)
    if style == 0:
        return "%.3f" % v
    sign = "-" if v < 0 else ""
    whole, frac = divmod(int(round(abs(v) * 8)), 8)
    if frac == 0:
        return "%s%d" % (sign, whole)
    g = math.gcd(frac, 8)
    f = "%d/%d" % (frac // g, 8 // g)
    if style == 1:
        return "%s%d %s" % (sign, whole, f) if whole else sign + f
    return "%s%d/%d" % (sign, whole * (8 // g) + frac // g, 8 // g)


def synth_run(rng, d, e_eff, scatter_in=0.0, scatter_deg=0.0, stop=True, ring=None, k=0, n=5):
    """One run from the exact geometry: believed end within the stopping tolerance, real end = believed end +
    the square's leg-by-leg error + scatter. ring=R puts the scatter exactly on a circle of radius R."""
    sgn = U.turn_sign(d)
    if stop:
        a, rr = rng.uniform(0, 2 * math.pi), 0.5 * math.sqrt(rng.random())
        bx, by = rr * math.cos(a), rr * math.sin(a)
        bt = sgn * 360.0 + rng.uniform(-1.0, 1.0)
    else:
        bx = by = 0.0
        bt = sgn * 360.0
    ex, ey = U.square_end_by_legs(e_eff, SIDE, d)
    if ring is not None:
        ang = 2 * math.pi * k / n + 0.3
        ex += ring * math.cos(ang)
        ey += ring * math.sin(ang)
    else:
        ex += rng.gauss(0, scatter_in)
        ey += rng.gauss(0, scatter_in)
    real_turn = bt / (1.0 + e_eff) + rng.gauss(0, scatter_deg)
    h = U.wrap180(real_turn)
    return {"dir": d, "right": bx + ex, "forward": by + ey, "heading": h, "x": bx, "y": by, "theta": bt,
            "turned": seen(h)}


def seen(h):
    """What a person sees against the tape: the way the robot ended turned, if it is a degree or more"""
    return None if abs(h) < SEEN_DEG else ("CW" if h > 0 else "CCW")


def umb_line(n, r, scale_cw=1.0, scale_ccw=1.0, still=0.004, t_ms=21000, side=SIDE, speed=70, timeouts=None):
    # timeouts= is the routine's last key (src/umbmark.cpp); left out by default, as the first version printed
    return ("[UMB] v=1 run=%d dir=%s side=%g drive_speed=%d turn_speed=70 x=%.2f y=%.2f theta=%.2f "
            "still_dps=%.4f scale_cw=%.4f scale_ccw=%.4f t_ms=%d"
            % (n, r["dir"], side, speed, r["x"], r["y"], r["theta"], still, scale_cw, scale_ccw, t_ms)) + \
        ("" if timeouts is None else " timeouts=%d" % timeouts)


NOISE = ["\x1b[32m[INFO]\x1b[0m odom: x=12.40 y=-3.10 theta=91.2", "[optical] hue=212 red=0",
         "[PIDLOG] BEGIN v=1 run=4 mode=TURN target=90", "~10,9000,0,0,0,0,0", "Battery: 87%", ""]


def write_log(path, runs, earlier=(), seed=1, **kw):
    """A capture: some earlier [UMB] lines, noise, then one line per run. CRLF and colour codes, as pros terminal."""
    rng = random.Random(seed)
    lines = ["PROS Terminal", "Waiting for device..."]
    n = 1
    for r in list(earlier) + list(runs):
        for _ in range(rng.randint(0, 3)):
            lines.append(rng.choice(NOISE))
        ln = umb_line(n, r, **kw)
        if n % 3 == 0:
            ln = "\x1b[36m" + ln + "\x1b[0m"
        if n % 4 == 0:
            ln = "[optical] hue=40 " + ln       # another task's print glued in front
        lines.append(ln)
        n += 1
    lines.append("[optical] hue=200 blue=1")
    with open(path, "wb") as fh:
        fh.write(("\r\n".join(lines) + "\r\n").encode("utf-8"))


TURNED_TEXT = {"CW": ("CW", "cw", "clockwise"), "CCW": ("CCW", "acw", "anticlockwise"), None: ("0", "", "?")}


def write_simple(path, runs, heading=True, believed=False, blank_heading_every=0, turned=True, wrap_theta=False):
    hdr = ["dir", "right_in", "forward_in"] + (["heading_deg"] if heading else []) + (["turned"] if turned else []) + \
          (["believed_x_in", "believed_y_in", "believed_theta_deg"] if believed else [])
    lines = ["# UMBmark square, simple form (selftest)", ",".join(hdr)]
    for i, r in enumerate(runs):
        cells = [r["dir"].lower() if i % 4 == 1 else r["dir"], ruler(r["right"], i % 3), ruler(r["forward"], (i + 1) % 3)]
        if heading:
            blank = blank_heading_every and i % blank_heading_every == 0
            cells.append("" if blank else "%.1f" % (round(r["heading"] * 2) / 2))
        if turned:
            cells.append(TURNED_TEXT[r["turned"]][i % 3])
        if believed:
            th = U.wrap180(r["theta"]) if wrap_theta else r["theta"]
            cells += ["%.2f" % r["x"], "%.2f" % r["y"], "%.1f" % th]
        lines.append(",".join(cells))
        if i == 2:
            lines.append("# a comment in the middle")
            lines.append("")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")


CORNER_SPACING, CORNER_BEHIND, CORNER_MID = 14.0, 7.0, 0.3


def write_corners(path, runs, turned=True, swap=False):
    lines = ["# UMBmark square, corners form (selftest)",
             "# corner_spacing_in = %g" % CORNER_SPACING,
             "# corners_behind_centre_in = %g" % CORNER_BEHIND,
             "# corners_mid_right_in = %g" % CORNER_MID,
             "dir,left_right_in,left_forward_in,right_right_in,right_forward_in" + (",turned" if turned else "")]
    for i, r in enumerate(runs):
        T, phi = (r["right"], r["forward"]), r["heading"]
        pts = []
        for sx in (-1, 1):
            p0 = (CORNER_MID + sx * CORNER_SPACING / 2.0, -CORNER_BEHIND)
            rp = U.rot_cw(p0, phi)
            pts.append([ruler(T[0] + rp[0] - p0[0], i % 3), ruler(T[1] + rp[1] - p0[1], (i + 2) % 3)])
        if swap:
            pts.reverse()       # the two corners typed in the wrong order
        cells = [r["dir"]] + pts[0] + pts[1] + ([TURNED_TEXT[r["turned"]][0]] if turned else [])
        lines.append(",".join(cells))
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")


def run_tool(args):
    p = subprocess.run([sys.executable, "-B", TOOL] + args, capture_output=True, text=True, encoding="utf-8")
    return p.returncode, p.stdout, p.stderr


def alternate(rng, e_cw, e_ccw, n=5, **kw):
    runs = []
    for k in range(n):
        runs.append(synth_run(rng, "CW", e_cw, k=k, n=n, **kw))
        runs.append(synth_run(rng, "CCW", e_ccw, k=k, n=n, **kw))
    return runs


# ---------------------------------------------------------------------------
# The test
# ---------------------------------------------------------------------------
def part_model(C):
    for e in (-0.05, -0.012, 0.0, 0.004, 0.012, 0.05):
        for d in ("CW", "CCW"):
            a, b = U.square_end(e, SIDE, d), U.square_end_by_legs(e, SIDE, d)
            C.ok(math.hypot(a[0] - b[0], a[1] - b[1]) < 1e-9, "closed form = leg sum (e=%g %s)" % (e, d))
            back = U.fit_position(a[0], a[1], SIDE, d)[0]
            C.ok(abs(back - e) < 1e-7, "position fit run backwards gives e=%g %s (got %.8f)" % (e, d, back))
            th = 360.0 * U.turn_sign(d)
            hb = U.e_from_heading(th, U.wrap180(th / (1.0 + e)), d)
            C.ok(abs(hb - e) < 1e-9, "heading fit run backwards gives e=%g %s" % (e, d))
            C.ok(abs(U.heading_error_model(e, d) - (-U.turn_sign(d) * 360.0 * e / (1.0 + e))) < 1e-9,
                 "H = -+360 e/(1+e) exactly (e=%g %s)" % (e, d))
    # The design's small-angle formulas: CW E ~ (+pi L e, -pi L e), CCW (-pi L e, -pi L e), H ~ -+360 e.
    e = 0.012
    for d, sx in (("CW", 1.0), ("CCW", -1.0)):
        x, y = U.square_end(e, SIDE, d)
        lead = (sx * math.pi * SIDE * e, -math.pi * SIDE * e)
        C.ok(abs(x - lead[0]) < 0.05 * abs(lead[0]) and abs(y - lead[1]) < 0.05 * abs(lead[1]),
             "small-angle formula within 5%% at e=1.2%% %s: exact (%.3f, %.3f) vs (%.3f, %.3f)" % (d, x, y, *lead))
        C.ok(abs(U.heading_error_model(e, d) + U.turn_sign(d) * 360 * e) < 0.02 * 360 * e,
             "small-angle heading within 2%% at e=1.2%% %s" % d)
    C.ok(abs(math.hypot(*U.square_end(0.012, SIDE, "CW")) - 2.55) < 0.01, "e=1.2%, L=48 gives 2.55 in")
    # Corners: a pose -> two points -> the pose again.
    for (tx, ty, phi) in ((1.3, -2.1, -4.3), (-0.4, 0.9, 3.0), (0.0, 0.0, 0.0)):
        dl, dr = [], []
        for sx, acc in ((-1, dl), (1, dr)):
            p0 = (0.5 + sx * 7.0, -6.0)
            rp = U.rot_cw(p0, phi)
            acc += [tx + rp[0] - p0[0], ty + rp[1] - p0[1]]
        (gx, gy), gphi, sep = U.corners_to_pose(dl, dr, 14.0, 6.0, 0.5)
        C.ok(abs(gx - tx) < 1e-9 and abs(gy - ty) < 1e-9 and abs(gphi - phi) < 1e-9 and abs(sep - 14.0) < 1e-9,
             "corners -> pose round trip (%g, %g, %g deg)" % (tx, ty, phi))
    for txt, v in (("1 3/8", 1.375), ("-1 3/8", -1.375), ("11/8", 1.375), ("-3/8", -0.375), ("2.5", 2.5),
                   ("2 in", 2.0), ("−1.5", -1.5), ("", None)):
        C.ok(U.parse_number(txt) == v, "number '%s' reads as %s" % (txt, v))


def part_sim(C, findings):
    # A perfect robot: the simulator's odometry must agree with its own physics.
    f = sim_fit({})
    for d in ("CW", "CCW"):
        C.ok(math.hypot(*f[d]["E"]) < 1e-6 and abs(f[d]["H"]) < 1e-6,
             "simulator: a perfect robot's odometry is exact (%s, E=%.2g in)" % (d, math.hypot(*f[d]["E"])))

    # Inertial scale error only: the simulated robot against the exact model.
    e_true = {"CW": 0.012, "CCW": 0.007}
    f = sim_fit({"e_cw": 0.012, "e_ccw": 0.007})
    for d in ("CW", "CCW"):
        m = U.square_end(e_true[d], SIDE, d)
        diff = math.hypot(f[d]["E"][0] - m[0], f[d]["E"][1] - m[1])
        findings.append("sim, scale error only, %s: E=(%.3f, %.3f), exact model (%.3f, %.3f), %.3f in apart"
                        % (d, f[d]["E"][0], f[d]["E"][1], m[0], m[1], diff))
        C.ok(diff < 0.06, "simulated %s square matches the exact model within 0.06 in (%.3f)" % (d, diff))
        C.ok(abs(f[d]["e_pos"] - e_true[d]) < 0.0003, "simulated %s: position fit %.4f%% vs %.4f%%"
             % (d, f[d]["e_pos"] * 100, e_true[d] * 100))
        C.ok(abs(f[d]["e_head"] - e_true[d]) < 0.0001, "simulated %s: heading fit %.4f%% vs %.4f%%"
             % (d, f[d]["e_head"] * 100, e_true[d] * 100))

    # The claims: these errors cancel round the square (no inertial error).
    cases = [
        ("tracking wheel sizes +3% / -2%", {"vert_size": 0.03, "horiz_size": -0.02}, 0.06),
        ("wheel offsets wrong by 1 in (both)", {"vert_x_odom": -0.6 + 1.0, "horiz_y_odom": -2.23 - 1.0}, 0.06),
        ("horizontal wheel 2 deg off parallel", {"horiz_yaw_deg": 2.0}, 0.1),
        ("vertical wheel 2 deg off parallel", {"vert_yaw_deg": 2.0}, 0.06),
        ("turning point 2 in ahead, 1 in right of the tracking centre", {"centre": (1.0, 2.0)}, 0.06),
    ]
    for name, kw, tol in cases:
        f = sim_fit(kw)
        for d in ("CW", "CCW"):
            E = math.hypot(*f[d]["E"])
            findings.append("sim, %s, %s: |E| = %.3f in, fitted scale error %+.3f%%"
                            % (name, d, E, f[d]["e_pos"] * 100))
            C.ok(E < tol, "%s cancels round the %s square (|E| = %.3f in < %.2f)" % (name, d, E, tol))

    # The same errors WITH a scale error do not cancel completely: each couples with it (second order, e times
    # the other error). Found here, 2026-11-22: a turning point 2.24 in from the tracking centre moved the fitted
    # scale by +0.1 %, wheel sizes +3/-2 % by -0.04 %; all together left up to 0.3 in on one direction.
    for name, kw, tol in cases:
        k = dict(kw, e_cw=0.012, e_ccw=0.007)
        f = sim_fit(k)
        for d in ("CW", "CCW"):
            findings.append("sim, %s plus scale error, %s: fitted %+.3f%% (true %+.3f%%), leftover %.3f in"
                            % (name, d, f[d]["e_pos"] * 100, e_true[d] * 100, f[d]["left"]))
            C.ok(abs(f[d]["e_pos"] - e_true[d]) < 0.0015, "%s with a scale error: %s scale within 0.15%% "
                 "(%.3f%% vs %.3f%%)" % (name, d, f[d]["e_pos"] * 100, e_true[d] * 100))
    allk = {"e_cw": 0.012, "e_ccw": 0.007, "vert_size": 0.03, "horiz_size": -0.02, "vert_x_odom": 0.4,
            "horiz_y_odom": -3.23, "horiz_yaw_deg": 2.0, "centre": (1.0, 2.0)}
    f = sim_fit(allk)
    scales = {}
    for d in ("CW", "CCW"):
        findings.append("sim, all of the above plus scale error, %s: fitted %+.3f%% (true %+.3f%%), leftover %.3f in"
                        % (d, f[d]["e_pos"] * 100, e_true[d] * 100, f[d]["left"]))
        C.ok(abs(f[d]["e_pos"] - e_true[d]) < 0.001, "%s scale within 0.1%% with every other error present "
             "(%.3f%% vs %.3f%%)" % (d, f[d]["e_pos"] * 100, e_true[d] * 100))
        scales[d] = 1.0 / (1.0 + f[d]["e_pos"])
    # A second round, with the scales the first round gave typed in: what is left of the scale error is tiny.
    f2 = sim_fit(dict(allk, scale_cw=scales["CW"], scale_ccw=scales["CCW"]))
    for d in ("CW", "CCW"):
        left_true = (1.0 + e_true[d]) * scales[d] - 1.0
        findings.append("sim, all errors, second round with the new scales, %s: fitted %+.3f%% (scale error left "
                        "%+.3f%%), leftover %.3f in" % (d, f2[d]["e_pos"] * 100, left_true * 100, f2[d]["left"]))
        C.ok(abs(left_true) < 0.0007 and abs(f2[d]["e_pos"]) < 0.0005,
             "second round converges (%s: scale error left %.3f%%, fitted %.3f%%)"
             % (d, left_true * 100, f2[d]["e_pos"] * 100))

    # A turning point away from the tracking centre WITH a scale error: the position fit comes out smaller by about
    # 2.3 x D / L of itself (D behind; bigger if in front), the heading fit is right, and almost nothing is left
    # over to warn. This is a limit of the position-only form, stated in umbmark.py's notes (a review found the
    # notes' old "up to 0.1 %" read as harmless): the checks below hold it to those words
    for D in (2.24, 5.8, -2.24):
        f = sim_fit({"e_cw": 0.012, "e_ccw": 0.012, "centre": (0.0, -D)})
        for d in ("CW", "CCW"):
            ratio = f[d]["e_pos"] / 0.012
            findings.append("sim, turning point %+.2f in behind, scale error +1.2 %%, %s: position fit %+.3f %% (x%.3f, "
                            "1 - 2.3 D / L = %.3f), heading fit %+.3f %%, leftover %.2f in"
                            % (D, d, f[d]["e_pos"] * 100, ratio, 1 - 2.3 * D / SIDE, f[d]["e_head"] * 100, f[d]["left"]))
            C.ok(abs(ratio - (1 - 2.3 * D / SIDE)) < 0.02 and abs(f[d]["e_head"] - 0.012) < 0.0001 and f[d]["left"] < 0.15,
                 "turning point %+.2f in behind, %s: position x%.3f (about 1 - 2.3 D / L), heading right, little left"
                 % (D, d, ratio))

    # Steady drift: reads as + clockwise, - counter-clockwise, about drift x time / 360 each.
    drift = 0.05
    f = sim_fit({"drift_dps": drift})
    for d in ("CW", "CCW"):
        r = f[d]["run"]
        total = drift * (r["t_ms"] / 1000.0 + STILL_S + SETTLE_S)
        expect = U.turn_sign(d) * total / 360.0
        findings.append("sim, drift %.2f deg/s, %s: square took %.1f s, fitted position %+.3f%%, heading %+.3f%%, "
                        "drift x time / 360 = %+.3f%%, leftover %.3f in"
                        % (drift, d, r["t_ms"] / 1000, f[d]["e_pos"] * 100, f[d]["e_head"] * 100, expect * 100,
                           f[d]["left"]))
        C.ok(f[d]["e_pos"] * U.turn_sign(d) > 0 and f[d]["e_head"] * U.turn_sign(d) > 0,
             "drift reads as %s on the %s square" % ("+" if d == "CW" else "-", d))
        C.ok(abs(f[d]["e_head"] - expect) < 0.3 * abs(expect), "drift's heading effect is drift x time / 360 (%s)" % d)
        C.ok(abs(r["still_dps"] - drift) < 1e-6, "still_dps measures the drift (%s)" % d)
    return f


def check_err(C, tag, rc, out, err, *needles):
    C.ok(rc == 2, "%s: exit status 2 (got %d)" % (tag, rc))
    C.ok("Traceback" not in out + err, "%s: no traceback" % tag)
    for nd in needles:
        C.ok(nd in err, "%s: message mentions '%s'  [%s]" % (tag, nd, err.strip()[:300]))


def part_end_to_end(C, out, findings):
    j = os.path.join(out, "res.json")

    def load():
        with open(j, encoding="utf-8") as fh:
            return json.load(fh)

    def e_ok(tag, got, se, true, tol_abs, k=3.0, floor=0.0005):
        lim = min(tol_abs, k * (se or 0.0) + floor) if se else tol_abs
        C.ok(abs(got - true) <= max(lim, floor), "%s: %.3f%% vs true %.3f%% (allowed %.3f%%)"
             % (tag, got * 100, true * 100, max(lim, floor) * 100))

    # A. Simple form with a log that has more [UMB] lines than rows; unequal errors; headings measured.
    rng = random.Random(11)
    runs = alternate(rng, 0.012, 0.007, scatter_in=0.15, scatter_deg=0.3)
    earlier = [synth_run(rng, "CCW", 0.0), synth_run(rng, "CCW", 0.0), synth_run(rng, "CW", 0.0)]
    meas, log = os.path.join(out, "a.csv"), os.path.join(out, "a.log")
    write_simple(meas, runs)
    write_log(log, runs, earlier=earlier)
    plot = os.path.join(out, "a.png")
    rc, so, se_ = run_tool([meas, "--log", log, "--json", j, "--plot", plot])
    C.ok(rc == 0, "A: tool ran (status %d) %s" % (rc, se_.strip()[:300]))
    if rc == 0:
        res = load()
        for d, t in (("CW", 0.012), ("CCW", 0.007)):
            x = res["directions"][d]
            e_ok("A %s position" % d, x["e_position"], x["e_position_se"], t, 0.002)
            e_ok("A %s heading" % d, x["e_heading"], x["e_heading_se"], t, 0.002)
            C.ok(x["agree"] is True, "A %s: position and heading agree" % d)
            C.ok(x["slip_class"] == "repeatable", "A %s: slip verdict repeatable (%s)" % (d, x["slip"]))
            C.ok(0.3 < x["e_position_se"] / (0.15 / (math.sqrt(2) * math.pi * SIDE) / math.sqrt(5)) < 3.0,
                 "A %s: the ± is the size the scatter says (%.4f%%)" % (d, x["e_position_se"] * 100))
        c = res["constants"]
        C.ok(abs(c["IMU_SCALE_CLOCKWISE"] - 1 / 1.012) < 0.0012, "A: IMU_SCALE_CLOCKWISE %.4f vs %.4f"
             % (c["IMU_SCALE_CLOCKWISE"], 1 / 1.012))
        C.ok(abs(c["IMU_SCALE_COUNTER_CLOCKWISE"] - 1 / 1.007) < 0.0012, "A: IMU_SCALE_COUNTER_CLOCKWISE %.4f vs "
             "%.4f" % (c["IMU_SCALE_COUNTER_CLOCKWISE"], 1 / 1.007))
        C.ok(res["side_from"] == "log" and res["believed_from"] == "log", "A: side and believed pose from the log")
        C.ok([r["log_run"] for r in res["runs"]] == [str(k) for k in range(4, 14)],
             "A: rows matched to the last 10 [UMB] lines (runs 4-13)")
        C.ok(res["umbmark_score"]["predicted_after_in"] < 0.3 * res["umbmark_score"]["now_in"],
             "A: UMBmark score predicted to drop (%.2f -> %.2f in)" % (res["umbmark_score"]["now_in"],
                                                                     res["umbmark_score"]["predicted_after_in"]))
        C.ok("IMU_SCALE_CLOCKWISE = %.4ff;" % c["IMU_SCALE_CLOCKWISE"] in so, "A: constant printed for robot-config.hpp")
        C.ok(not any(res["directions"][d]["within_scatter"] for d in ("CW", "CCW")) and
             "drive the squares again" in so and "This test is done" not in so,
             "A: a 1.2 % error is not 'within the scatter': it asks for another round")
        C.ok("!! No log" not in so, "A: no 'assumed zero' warning when the log is there")
        C.ok(rc == 0 and res["held_back"] == [] and res["constants"] is not None, "A: good data: numbers given")
        # The ± exactly: the spread of the per-run e over sqrt(n), floored (a ± sqrt(5) too wide once passed every
        # check). The heading's floor is its reading step / sqrt(12) / 360
        step = res["heading_step_deg"]
        C.ok(step in (0.5, 1.0), "A: headings written to 0.5 deg read as step %s" % step)
        for d in ("CW", "CCW"):
            x = res["directions"][d]
            ep = [r["e_position"] for r in res["runs"] if r["dir"] == d]
            eh = [r["e_heading"] for r in res["runs"] if r["dir"] == d and r["e_heading"] is not None]
            sp = max(U.SE_FLOOR, statistics.stdev(ep) / math.sqrt(len(ep)))
            sh = max(U.SE_FLOOR, step / math.sqrt(12.0) / 360.0, statistics.stdev(eh) / math.sqrt(len(eh)))
            C.ok(abs(x["e_position_se"] - sp) < 1e-12 and abs(x["e_heading_se"] - sh) < 1e-12,
                 "A %s: ± exactly spread / sqrt(n), floored (%.5f%% / %.5f%%)" % (d, sp * 100, sh * 100))
            wp, wh = 1.0 / sp ** 2, 1.0 / sh ** 2
            comb = (wp * x["e_position"] + wh * x["e_heading"]) / (wp + wh)
            C.ok(abs(x["e_used"] - comb) < 1e-12 and abs(x["e_used_se"] - 1.0 / math.sqrt(wp + wh)) < 1e-12 and
                 min(x["e_position"], x["e_heading"]) + 1e-7 < x["e_used"] < max(x["e_position"], x["e_heading"]) - 1e-7,
                 "A %s: agreeing position and heading are weighted by 1/±^2, the result strictly between" % d)
        # the score predicted after the new scales: what is left over plus a new set of runs' scatter (rms / sqrt n)
        pred = max(math.hypot(res["directions"][d]["leftover_in"],
                              res["directions"][d]["scatter_rms_in"] / math.sqrt(res["directions"][d]["n"]))
                   for d in ("CW", "CCW"))
        C.ok(abs(res["umbmark_score"]["predicted_after_in"] - pred) < 1e-12,
             "A: predicted UMBmark score = leftover and scatter of a new set's centre (%.3f in)" % pred)
        C.ok(res["plot"] == plot and os.path.getsize(plot) > 20000 if res["plot"] else "skipped" in so,
             "A: plot written (or skipped with a note)")
        findings.append("end-to-end A report:\n" + so)

    # B. Corners form, current scales not 1, effective errors negative and about zero.
    rng = random.Random(12)
    s_cw, s_ccw, es_cw, es_ccw = 0.9881, 1.0050, 0.004, -0.005
    eff_cw, eff_ccw = s_cw * (1 + es_cw) - 1, s_ccw * (1 + es_ccw) - 1
    runs = alternate(rng, eff_cw, eff_ccw, scatter_in=0.12, scatter_deg=0.2)
    meas, log = os.path.join(out, "b.csv"), os.path.join(out, "b.log")
    write_corners(meas, runs)
    write_log(log, runs, scale_cw=s_cw, scale_ccw=s_ccw)
    rc, so, se_ = run_tool([meas, "--log", log, "--json", j])
    C.ok(rc == 0, "B: tool ran (status %d) %s" % (rc, se_.strip()[:300]))
    if rc == 0:
        res = load()
        C.ok(res["form"] == "corners" and res["scale_from"]["CW"] == "log", "B: corners form, scales from the log")
        for d, t in (("CW", eff_cw), ("CCW", eff_ccw)):
            x = res["directions"][d]
            e_ok("B %s position" % d, x["e_position"], x["e_position_se"], t, 0.002)
            e_ok("B %s heading (from corners)" % d, x["e_heading"], x["e_heading_se"], t, 0.002)
        for name, true in (("IMU_SCALE_CLOCKWISE", 1 / (1 + es_cw)), ("IMU_SCALE_COUNTER_CLOCKWISE", 1 / (1 + es_ccw))):
            C.ok(abs(res["constants"][name] - true) < 0.0012, "B: %s %.4f vs %.4f (= 1/(1 + sensor error), whatever "
                 "the current scale)" % (name, res["constants"][name], true))
        C.ok("the sensor itself reads" in so, "B: says what the sensor itself reads when the scale is not 1")
        C.ok(not any("apart" in w for w in res["warnings"]), "B: corner spacing check quiet on good data")
        findings.append("end-to-end B (corners, scales 0.9881/1.0050) constants: CW %.4f (true %.4f), CCW %.4f "
                        "(true %.4f)" % (res["constants"]["IMU_SCALE_CLOCKWISE"], 1 / (1 + es_cw),
                                         res["constants"]["IMU_SCALE_COUNTER_CLOCKWISE"], 1 / (1 + es_ccw)))

    # L. The second round, after a good calibration: no scale error left, only slip scatter, the new scales in the
    # log. The tool must say the test is done, not ask for yet another round, and must not say calibration will
    # fix the scatter. One run timed out on the robot: that must be pointed out, not used without a word.
    rng = random.Random(22)
    runs = alternate(rng, 0.0, 0.0, scatter_in=0.3)
    meas, log = os.path.join(out, "l.csv"), os.path.join(out, "l.log")
    write_simple(meas, runs, heading=False)
    write_log(log, runs, scale_cw=0.9882, scale_ccw=0.9932, timeouts=0)
    rc, so, se_ = run_tool([meas, "--log", log, "--json", j])
    C.ok(rc == 0, "L: tool ran (status %d) %s" % (rc, se_.strip()[:300]))
    if rc == 0:
        res = load()
        C.ok(all(res["directions"][d]["within_scatter"] for d in ("CW", "CCW")) and res["settled"] and
             "This test is done" in so and "drive the squares again" not in so,
             "L: no scale error left: 'within the scatter', test done")
        C.ok("calibration will fix most of it" not in so and "calibration cannot fix" in so,
             "L: scatter bigger than the repeatable error is not promised to calibration")
        C.ok("(CW 0.9882, CCW 0.9932: check the file still says that" in so,
             "L: the scales the runs were made with are printed next to the constants")
    # L2. One run (row 8 of 10) timed out on an older build: no numbers until it is driven again
    with open(log, "rb") as fh:
        lines = fh.read().decode("utf-8").split("\r\n")
    umb = [i for i, ln in enumerate(lines) if "[UMB]" in ln]
    lines[umb[-3]] = lines[umb[-3]].replace("timeouts=0", "timeouts=1")
    with open(log, "wb") as fh:
        fh.write("\r\n".join(lines).encode("utf-8"))
    rc, so, se_ = run_tool([meas, "--log", log, "--json", j])
    res = load() if rc in (0, 3) else None
    C.ok(rc == 3 and res["constants"] is None and sum("ran out of time" in h for h in res["held_back"]) == 1 and
         any("Row 8 " in h for h in res["held_back"]), "L2: a run with timeouts=1 holds the numbers back, row 8 named")

    # L3. No scale error left, but a repeatable error the scale does not explain (0.7 in, across each direction's
    # scale line, under the 1 in where a sign hint starts): numbers, but NOT "this test is done"
    rng = random.Random(28)
    runs = alternate(rng, 0.0, 0.0, scatter_in=0.15, stop=False)
    for r in runs:
        r["right"] += 0.5
        r["forward"] += 0.5 if r["dir"] == "CW" else -0.5
    meas = os.path.join(out, "l3.csv")
    write_simple(meas, runs, heading=False, believed=True)
    rc, so, se_ = run_tool([meas, "--json", j])
    res = load() if rc in (0, 3) else None
    C.ok(rc == 0 and all(res["directions"][d]["within_scatter"] and not res["directions"][d]["leftover_small"]
                         for d in ("CW", "CCW")) and not res["settled"] and "This test is done" not in so
         and "Not small" in so, "L3: within the scatter but 0.7 in left over: numbers, not 'done'")

    # C. No log, no believed columns, no headings: the warning, and looser numbers.
    rng = random.Random(13)
    runs = alternate(rng, 0.015, -0.005, scatter_in=0.1)
    meas = os.path.join(out, "c.csv")
    write_simple(meas, runs, heading=False)
    rc, so, se_ = run_tool([meas, "--json", j])
    C.ok(rc == 0, "C: tool ran (status %d) %s" % (rc, se_.strip()[:300]))
    if rc == 0:
        res = load()
        C.ok("!! No log" in so and "about 0.5 in" in so, "C: prominent warning that (0, 0, 0) was assumed")
        C.ok(res["believed_from"] == "assumed zero" and res["assumed_rows"] == 10, "C: all rows assumed")
        for d, t in (("CW", 0.015), ("CCW", -0.005)):
            x = res["directions"][d]
            e_ok("C %s position, believed pose unknown" % d, x["e_position"], None, t, 0.003)
            C.ok(x["e_heading"] is None and x["e_used_from"] == "position", "C %s: position only" % d)
        C.ok("Heading error: not measured" in so, "C: says the heading was not measured")
        C.ok("Heading drift" not in so, "C: no drift line without a log")

    # D. Believed pose typed in, no log, some headings blank.
    rng = random.Random(14)
    runs = alternate(rng, -0.01, 0.0, scatter_in=0.1, scatter_deg=0.3)
    meas = os.path.join(out, "d.csv")
    write_simple(meas, runs, believed=True, blank_heading_every=3)
    rc, so, se_ = run_tool([meas, "--json", j])
    C.ok(rc == 0, "D: tool ran (status %d) %s" % (rc, se_.strip()[:300]))
    if rc == 0:
        res = load()
        C.ok(res["believed_from"] == "typed" and "!! No log" not in so, "D: typed believed pose used, no warning")
        n_head = sum(1 for k in range(10) if k % 3 != 0)
        C.ok(res["directions"]["CW"]["heading_n"] + res["directions"]["CCW"]["heading_n"] == n_head,
             "D: blank headings left out (%d measured)" % n_head)
        for d, t in (("CW", -0.01), ("CCW", 0.0)):
            x = res["directions"][d]
            e_ok("D %s used" % d, x["e_used"], x["e_used_se"], t, 0.002)

    # E. Too few runs; F. scatter verdicts (scatter placed exactly on a ring, so the RMS is known).
    rng = random.Random(15)
    runs = [synth_run(rng, "CW", 0.01), synth_run(rng, "CW", 0.01), synth_run(rng, "CCW", 0.01)]
    meas = os.path.join(out, "e.csv")
    write_simple(meas, runs)
    rc, so, se_ = run_tool([meas, "--json", j])
    C.ok(rc == 0 and "cannot be judged" in so, "E: fewer than 3 runs: the scatter cannot be judged")
    if rc == 0:
        res = load()
        C.ok(res["directions"]["CCW"]["e_position_se"] is None and "no ±: one run" in so,
             "E: one run gives no ±, and says so")
        C.ok(res["constants"] == {"IMU_SCALE_CLOCKWISE": 1.0, "IMU_SCALE_COUNTER_CLOCKWISE": 1.0} and
             so.count("left as it is: only") == 2, "E: 2 and 1 runs: no new constant for either direction")
        C.ok("With 1 runs" not in so and "With 2 runs the true value is within it about 50 %" in so,
             "E: the ± wording is tied to the number of runs (2 runs: 50 %%)")
    for name, ring, cls in (("repeatable", 0.2, "repeatable"), ("borderline", 0.6, "borderline"),
                            ("slip", 1.2, "slip")):
        rng = random.Random(16)
        runs = alternate(rng, 0.01, 0.01, ring=ring, stop=False)
        meas = os.path.join(out, "f_%s.csv" % name)
        write_simple(meas, runs, heading=False, believed=True)
        rc, so, se_ = run_tool([meas, "--json", j])
        res = load() if rc == 0 else None
        C.ok(res is not None and all(res["directions"][d]["slip_class"] == cls for d in ("CW", "CCW")),
             "F: scatter ring %.1f in gives '%s'" % (ring, cls))

    # G. Clockwise runs only: the counter-clockwise constant is left alone.
    rng = random.Random(17)
    runs = [synth_run(rng, "CW", 0.01, scatter_in=0.1) for _ in range(4)]
    meas = os.path.join(out, "g.csv")
    write_simple(meas, runs, believed=True)
    rc, so, se_ = run_tool([meas, "--json", j, "--scale-ccw", "0.9950"])
    C.ok(rc == 0, "G: tool ran with one direction (status %d)" % rc)
    if rc == 0:
        res = load()
        C.ok(res["constants"]["IMU_SCALE_COUNTER_CLOCKWISE"] == 0.995 and "left as it is" in so,
             "G: counter-clockwise constant kept at the current 0.9950")
        C.ok("needs both directions" in so, "G: UMBmark score says it needs both directions")

    # H. Drift standing still over the limit.
    rng = random.Random(18)
    runs = alternate(rng, 0.01, 0.01, scatter_in=0.1)
    meas, log = os.path.join(out, "h.csv"), os.path.join(out, "h.log")
    write_simple(meas, runs)
    write_log(log, runs, still=0.05, t_ms=22000)
    rc, so, se_ = run_tool([meas, "--log", log, "--json", j])
    C.ok(rc == 3 and "NO NEW NUMBERS" in so and "crept 0.050 deg/s" in so and "Type these into" not in so,
         "H: drift over 0.02 deg/s holds the numbers back, at the top")
    if rc == 3:
        res = load()
        st = res["still"]
        C.ok(st["warning"] and abs(st["equivalent_e_cw"] - 0.05 * 22 / 360) < 1e-6 and res["constants"] is None,
             "H: drift's effect on the scale; no constants")
    write_log(log, runs, still=0.004)
    rc, so, se_ = run_tool([meas, "--log", log])
    C.ok(rc == 0 and "deg/s, fine" in so and "!! Heading drift" not in so, "H: small drift reported as fine")

    # I. Right and left swapped by the person measuring: a hint, not silence.
    rng = random.Random(19)
    runs = alternate(rng, 0.015, 0.015, scatter_in=0.1)
    for r in runs:
        r["right"] = 2 * r["x"] - r["right"]          # error right flipped, believed pose kept
    meas = os.path.join(out, "i.csv")
    write_simple(meas, runs, heading=False, believed=True)
    rc, so, se_ = run_tool([meas])
    C.ok(rc == 3 and "right and left swapped" in so and so.index("right and left swapped") < so.index("Each run")
         and "This test is done" not in so, "I: swapped right/left: a hint at the top, no numbers, not 'done'")

    # K. Heading measured with the wrong sign: they disagree, and the hint says why.
    rng = random.Random(20)
    runs = alternate(rng, 0.012, 0.012, scatter_in=0.1, scatter_deg=0.2)
    for r in runs:
        r["heading"] = -r["heading"]
    meas = os.path.join(out, "k.csv")
    write_simple(meas, runs, believed=True)
    rc, so, se_ = run_tool([meas])
    C.ok(rc == 3 and "DISAGREE" in so and "heading_deg had the other sign" in so,
         "K: heading with the wrong sign: DISAGREE, a hint, no numbers")
    # K2. The same with no turned column: the hint is the reason given, not a check worded from the flipped heading
    write_simple(meas, runs, believed=True, turned=False)
    rc, so, se_ = run_tool([meas])
    C.ok(rc == 3 and "heading_deg had the other sign" in so and "nothing in the file says" not in so,
         "K2: heading with the wrong sign, no turned: the hint, and no sign check worded from the heading")

    # J. End to end from the simulator: scale error, wrong offsets, a wheel off parallel, slip.
    kw = {"e_cw": 0.012, "e_ccw": 0.007, "vert_x_odom": 0.0, "horiz_y_odom": -3.0, "horiz_yaw_deg": 1.0,
          "slip": 0.02}
    runs = []
    for k in range(5):
        runs.append(Sim(seed=100 + k, **kw).run("CW"))
        runs.append(Sim(seed=200 + k, **kw).run("CCW"))
    meas, log = os.path.join(out, "j.csv"), os.path.join(out, "j.log")
    write_simple(meas, runs)
    write_log(log, runs, still=0.0, t_ms=int(runs[0]["t_ms"]))
    rc, so, se_ = run_tool([meas, "--log", log, "--json", j])
    C.ok(rc == 0, "J: tool ran on simulated runs (status %d)" % rc)
    if rc == 0:
        res = load()
        for d, t in (("CW", 0.012), ("CCW", 0.007)):
            x = res["directions"][d]
            e_ok("J %s used (simulated robot, slip)" % d, x["e_used"], x["e_used_se"], t, 0.0015)
            findings.append("end-to-end J (simulator, slip 0.02): %s used %+.3f%% ± %.3f (true %+.3f%%), scatter RMS "
                            "%.2f in, leftover %.2f in" % (d, x["e_used"] * 100, (x["e_used_se"] or 0) * 100, t * 100,
                                                          x["scatter_rms_in"], x["leftover_in"]))

    # The mistakes a review found the tool gave confident wrong numbers for. Truth: +1.2 % CW, +0.8 % CCW, a little
    # scatter, believed pose typed. The person measuring makes the mistake; what they SEE (turned) is right
    def mistake(tag, runs_m, writer=write_simple, expect="NO NEW NUMBERS", needle=None, **kw):
        meas = os.path.join(out, "m_%s.csv" % tag)
        writer(meas, runs_m, **kw)
        rc, so, se_ = run_tool([meas, "--json", j])
        res = load() if rc in (0, 3) else None
        C.ok(rc == 3 and res["constants"] is None and expect in so and (needle is None or needle in so),
             "%s: no numbers (status %d)%s" % (tag, rc, "" if needle is None else ", says '%s'" % needle))
        return meas

    rng = random.Random(23)
    base = alternate(rng, 0.012, 0.008, scatter_in=0.1, scatter_deg=0.2)
    flip = [dict(r, right=2 * r["x"] - r["right"], forward=2 * r["y"] - r["forward"]) for r in base]
    mistake("N mark measured from the robot (simple, turned seen)", flip, heading=False, believed=True,
            needle="seen turned the other way")
    mistake("N2 the same, heading also flipped", [dict(r, heading=-r["heading"]) for r in flip], believed=True,
            needle="seen turned the other way")
    swap = [dict(r, right=r["x"] + (r["forward"] - r["y"]), forward=r["y"] + (r["right"] - r["x"])) for r in base]
    mistake("O right_in and forward_in swapped", swap, heading=False, believed=True, needle="seen turned the other way")
    cm = [dict(r, right=r["x"] + 2.54 * (r["right"] - r["x"]), forward=r["y"] + 2.54 * (r["forward"] - r["y"]))
          for r in base]
    mistake("P centimetres, no heading", cm, heading=False, believed=True, needle="more than 2 %")
    mistake("P2 centimetres, heading measured", cm, believed=True, needle="DISAGREE")
    mistake("Q corners typed in the wrong order", base, writer=write_corners, swap=True)
    mistake("R no turned, no heading", base, heading=False, believed=True, turned=False,
            needle="nothing in the file says which way")
    mistake("R2 turned written 0 where the numbers say 4 deg", [dict(r, turned=None) for r in base], heading=False,
            believed=True, needle="says it was not visibly turned")
    # the reverse: the same runs measured right give numbers
    meas = os.path.join(out, "m_ok.csv")
    write_simple(meas, base, heading=False, believed=True)
    rc, so, se_ = run_tool([meas, "--json", j])
    C.ok(rc == 0 and load()["constants"] is not None, "N-R reference: the same runs measured right give numbers")

    # S. A robot that spins about a point behind its tracking centre: the position's e comes out x0.72 (5.8 in),
    # the heading's is right. They disagree: no numbers, and the words say why. --use-heading takes the heading's
    rng = random.Random(24)
    spin = alternate(rng, 0.012, 0.012, scatter_in=0.05, scatter_deg=0.15)
    for r in spin:
        ex, ey = U.square_end_by_legs(0.012 * 0.72, SIDE, r["dir"])
        r["right"], r["forward"] = r["x"] + ex + rng.gauss(0, 0.05), r["y"] + ey + rng.gauss(0, 0.05)
    meas = mistake("S spin point behind (heading right)", spin, believed=True, needle="smaller than the heading")
    rc, so, se_ = run_tool([meas, "--json", j, "--use-heading"])
    res = load() if rc in (0, 3) else None
    C.ok(rc == 0 and abs(res["constants"]["IMU_SCALE_CLOCKWISE"] - 1 / 1.012) < 0.0008 and
         "as --use-heading asks" in so, "S --use-heading: the heading's number (%s)"
         % (res["constants"] if res else rc))

    # T. A log with round 1 and round 2 (different scales) appended: rows that fit both need --session
    rng = random.Random(25)
    r1 = alternate(rng, 0.012, 0.008, scatter_in=0.1)
    r2 = alternate(rng, 0.0, 0.0, scatter_in=0.1)
    log1, log2, both = (os.path.join(out, x) for x in ("t1.log", "t2.log", "t_both.log"))
    write_log(log1, r1)
    write_log(log2, r2, scale_cw=0.9881, scale_ccw=0.9921, seed=2)
    with open(both, "wb") as fh:
        fh.write(open(log1, "rb").read() + open(log2, "rb").read())
    meas = os.path.join(out, "t_round1.csv")
    write_simple(meas, r1, heading=False)
    check_err(C, "T round 1 file, log with round 2 appended", *run_tool([meas, "--log", both]), "fit more than one set",
              "--session 2")
    rc, so, se_ = run_tool([meas, "--log", both, "--session", "1", "--json", j])
    res = load() if rc in (0, 3) else None
    C.ok(rc == 0 and res["scale_current"]["CW"] == 1.0 and abs(res["constants"]["IMU_SCALE_CLOCKWISE"] - 1 / 1.012) < 0.0012
         and any("2 sets of runs" in w for w in res["warnings"]), "T --session 1: round 1's scales, a warning about the sets")
    meas2 = os.path.join(out, "t_round2.csv")
    write_simple(meas2, r2, heading=False)
    rc, so, se_ = run_tool([meas2, "--log", both, "--session", "2", "--json", j])
    res = load() if rc in (0, 3) else None
    C.ok(rc == 0 and res["scale_current"]["CW"] == 0.9881, "T --session 2: round 2's scales")

    # U. A believed heading typed wrapped (-0.4 for 359.6) gives the same heading fit as unwrapped
    rng = random.Random(26)
    runs = alternate(rng, 0.01, 0.01, scatter_in=0.1, scatter_deg=0.2)
    got = []
    for wrapped in (False, True):
        meas = os.path.join(out, "u_%d.csv" % wrapped)
        write_simple(meas, runs, believed=True, wrap_theta=wrapped)
        rc, so, se_ = run_tool([meas, "--json", j])
        got.append([load()["directions"][d]["e_heading"] for d in ("CW", "CCW")] if rc in (0, 3) else None)
    C.ok(got[0] is not None and got[1] is not None and all(abs(a - b) < 1e-9 for a, b in zip(*got)),
         "U: believed_theta_deg typed wrapped or not: the same heading fit")

    # V. Five headings read to the same half degree: the heading's ± is at least the rounding of one reading
    rng = random.Random(27)
    runs = alternate(rng, 0.012, 0.012, scatter_in=0.1, scatter_deg=0.0)
    meas = os.path.join(out, "v.csv")
    write_simple(meas, runs, believed=True)
    rc, so, se_ = run_tool([meas, "--json", j])
    res = load() if rc in (0, 3) else None
    floor = 0.5 / math.sqrt(12) / 360
    C.ok(res is not None and res["heading_step_deg"] == 0.5 and
         all(res["directions"][d]["e_heading_se"] >= floor - 1e-15 for d in ("CW", "CCW")),
         "V: identical half-degree headings: ± no smaller than %.3f %%" % (floor * 100))

    # Bad input: a message a student can act on, never a traceback.
    rng = random.Random(21)
    runs = alternate(rng, 0.01, 0.01, n=3)
    good = os.path.join(out, "good.csv")
    write_simple(good, runs)
    log = os.path.join(out, "bad.log")
    flipped = [dict(r) for r in runs]
    flipped[3]["dir"] = "CW" if flipped[3]["dir"] == "CCW" else "CCW"
    write_log(log, flipped, earlier=[runs[0]])
    check_err(C, "dir mismatch", *run_tool([good, "--log", log]), "Row 4", "says CCW", "LAST 6")
    write_log(log, runs[:4])
    check_err(C, "short log", *run_tool([good, "--log", log]), "has 6 rows", "only 4 [UMB]")
    with open(log, "w") as fh:
        fh.write("nothing here\n")
    check_err(C, "log without [UMB]", *run_tool([good, "--log", log]), "no [UMB] lines")
    write_log(log, runs)
    with open(log, encoding="utf-8") as fh:
        txt = fh.read()
    with open(log, "w", encoding="utf-8") as fh:
        fh.write(txt.replace(" theta=", " thet", 1))     # the first run's line lost a few characters
    check_err(C, "damaged log line", *run_tool([good, "--log", log]), "damaged", "theta")
    write_log(log, runs, side=36)
    with open(log, encoding="utf-8") as fh:
        txt = fh.read()
    with open(log, "w", encoding="utf-8") as fh:
        fh.write(txt.replace("side=36", "side=48", 2))
    check_err(C, "mixed sides", *run_tool([good, "--log", log]), "not all driven with the same side", "side 36")

    def bad(tag, text, *needles, extra=()):
        p = os.path.join(out, "bad_%s.csv" % tag.replace(" ", "_"))
        with open(p, "w", encoding="utf-8") as fh:
            fh.write(text)
        check_err(C, tag, *run_tool([p] + list(extra)), *needles)

    bad("bad number", "dir,right_in,forward_in\nCW,1.5,-2\nCW,l.5,-2\n", "line 3", "right_in", "'l.5' is not a number")
    bad("unknown column", "dir,right_inches,forward_in\nCW,1,2\n", "right_inches", "Did you mean 'right_in'")
    bad("corner keys missing", "# corner_spacing = 14\ndir,left_right_in,left_forward_in,right_right_in,right_forward_in\n"
        "CW,1,2,3,4\n", "corner_spacing_in", "corners_behind_centre_in", "Did you mean 'corner_spacing_in'")
    bad("bad dir", "dir,right_in,forward_in\nCW,1,2\nC W,1,2\n", "line 3", "must be CW")
    bad("empty file", "# only a comment\n", "no header")
    bad("header only", "dir,right_in,forward_in\n# CW,1,2\n", "no rows")
    bad("mixed forms", "dir,right_in,forward_in,left_right_in\nCW,1,2,3\n", "mixes the simple form")
    bad("semicolons", "dir;right_in;forward_in\nCW;1;2\n", "separated by ';'")
    bad("empty cell", "dir,right_in,forward_in\nCW,1,\n", "line 2", "forward_in is empty")
    bad("too many cells", "dir,right_in,forward_in\nCW,1,2,3\n", "line 2", "4 cells")
    bad("missing column", "dir,right_in,heading_deg\nCW,1,2\n", "forward_in")
    bad("absurd error", "dir,right_in,forward_in\nCW,-40,40\nCW,-40,41\n", "no inertial scale error can explain")
    write_log(os.path.join(out, "two.log"), [synth_run(rng, "CW", 0.01), synth_run(rng, "CCW", 0.01)])
    bad("run mismatch", "dir,right_in,forward_in,run\nCW,1,-1,1\nCCW,-1,-1,7\n", "says run 7", extra=(
        "--log", os.path.join(out, "two.log")))
    check_err(C, "missing file", *run_tool([os.path.join(out, "no_such.csv")]), "cannot read")
    bad("absurd number", "dir,right_in,forward_in\nCW,1e308,1e308\n", "more than 100 in")
    bad("absurd heading", "dir,right_in,forward_in,heading_deg\nCW,1,1,400\n", "outside -180 to +180")
    bad("bad turned", "dir,right_in,forward_in,turned\nCW,1,1,left\n", "is not CW, CCW or 0")
    p = os.path.join(out, "junk.csv")
    with open(p, "wb") as fh:
        fh.write(bytes(random.Random(5).randrange(256) for _ in range(3000)))
    check_err(C, "binary junk", *run_tool([p]), "does not look like a text CSV")
    # A file as a spreadsheet saves it: byte-order mark, CRLF, spaces in the header, a trailing comma, an empty row.
    p = os.path.join(out, "spreadsheet.csv")
    with open(p, "wb") as fh:
        fh.write(b"\xef\xbb\xbfdir, right_in , forward_in,heading_deg,\r\nCW,1 7/8,-1 3/4,-4.5,\r\n,,,,\r\n"
                 b"CCW,-1 1/8,-1 1/4,,\r\nCW,2,-1.5,-4,\r\nCCW,-1,-1,4,\r\n")
    rc, so, se_ = run_tool([p, "--json", j])
    C.ok(rc in (0, 3), "spreadsheet-style CSV accepted (status %d) %s" % (rc, se_.strip()[:200]))
    if rc in (0, 3):
        res = load()
        C.ok(len(res["runs"]) == 4 and res["runs"][0]["real_right_in"] == 1.875 and
             res["runs"][1]["real_heading_deg"] is None, "spreadsheet-style CSV: 4 runs, '1 7/8' read, blank heading")
    rc, so, se_ = run_tool(["--template", "corners"])
    C.ok(rc == 0 and "corner_spacing_in = " in so and "dir,left_right_in" in so, "--template corners prints a file")
    # the corner spacing check: one corner measured with the wrong sign
    p = os.path.join(out, "spacing.csv")
    with open(p, "w") as fh:
        fh.write("# corner_spacing_in = 14\n# corners_behind_centre_in = 7\n"
                 "dir,left_right_in,left_forward_in,right_right_in,right_forward_in\nCW,1,-2,-3,-1\nCW,1,-2,1.2,-1\n")
    rc, so, se_ = run_tool([p])
    C.ok(rc in (0, 3) and "apart, but corner_spacing_in is 14.00" in so, "corner spacing check catches a wrong number")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None, help="folder for the files it writes (default: a temporary folder)")
    ap.add_argument("--verbose", action="store_true", help="print every check, and what the simulator found")
    args = ap.parse_args()
    out = args.out or tempfile.mkdtemp(prefix="umbmark_selftest_")
    os.makedirs(out, exist_ok=True)
    C = Checks(args.verbose)
    findings = []
    part_model(C)
    part_sim(C, findings)
    part_end_to_end(C, out, findings)
    if args.verbose:
        print("\n".join(findings))
    print("=" * 72)
    if C.failures:
        print("%d of %d check(s) failed. Files in %s" % (len(C.failures), C.n, out))
        return 1
    print("Files in " + out)
    print("selftest passed: %d checks" % C.n)
    return 0


if __name__ == "__main__":
    sys.exit(main())
