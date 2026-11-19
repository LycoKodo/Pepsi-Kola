#!/usr/bin/env python3
"""End-to-end self-test for pidlog.py.

    python3 tools/pidlog_selftest.py [--out DIR] [--keep]

Simulates a VEX drivetrain driven by a LemLib-style PID controller, writes a
synthetic `pros terminal` capture in the exact [PIDLOG] format (with noise lines,
CRLF endings, ANSI colour codes and deliberately damaged runs), runs pidlog.py on
it as a subprocess and checks every run's verdict, the direction of every
suggested change, and the side flags. Exit status 0 = all checks passed.

The simulator is deliberately simple:
  * each drive side is a first-order voltage -> speed response with static and
    kinetic friction and a +-12 V limit; turns use their own constants (more
    scrub friction);
  * the controller runs every 10 ms: out = kP*e + kI*I + kD*(e - e_prev), e in
    inches or degrees, out in motor power (+-127 = +-12 V), clamped,
    slew-limited on increases, the integral only accumulating inside the windup
    range and reset when the error changes sign;
  * LemLib's small and large exit conditions and the timeout end the move;
    the motors then brake and logging continues for 300 ms.
"""

import argparse
import json
import math
import os
import random
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, "pidlog.py")

CTRL_MS = 10          # controller period
PHYS_MS = 1           # physics step
COAST_MS = 300        # rows logged after the move ends

DRIVE_PLANT = dict(K=48.0, tau=0.12, v_static=1.0, v_kinetic=0.6, brake_tau=0.06)
TURN_PLANT = dict(K=40.0, tau=0.10, v_static=0.8, v_kinetic=0.5, brake_tau=0.05)
IPS_PER_RPM = 0.1080
TRACK = 11.61


class Side:
    def __init__(self, K, tau, v_static, v_kinetic, brake_tau):
        self.K, self.tau, self.vs, self.vk, self.bt = K, tau, v_static, v_kinetic, brake_tau
        self.v = 0.0  # rpm

    def step(self, volts, dt, brake=False):
        if brake:
            self.v += -self.v * dt / self.bt
            if abs(self.v) < 0.5:
                self.v = 0.0
            return
        if abs(volts) <= self.vs and abs(self.v) < 12.0:
            # stiction: slow and not pushed past breakaway -> it grinds to a halt
            self.v -= self.v * dt / 0.03
            if abs(self.v) < 0.5:
                self.v = 0.0
            return
        d = self.v if abs(self.v) >= 0.5 else volts
        eff = volts - math.copysign(self.vk, d)
        self.v += (self.K * eff - self.v) * dt / self.tau


def simulate(mode="DRIVE", target=24.0, kP=12.0, kI=0.0, kD=60.0, windup=3.0, slew=15.0,
             small=(0.5, 200), large=(3.0, 500), timeout=4000, batt_pct=90, dt_ms=20,
             asym=0.0, kA=3.0, noise=None, delay_ticks=1, abort_at=None, plant=None,
             friction_scale=1.0, seed=1):
    """Run one move; return (begin_kv, rows, end_kv)."""
    rng = random.Random(seed)
    P = dict(DRIVE_PLANT if mode == "DRIVE" else TURN_PLANT)
    if plant:
        P.update(plant)
    P["v_static"] *= friction_scale
    P["v_kinetic"] *= friction_scale
    L = Side(P["K"] * (1 + asym / 2), P["tau"], P["v_static"], P["v_kinetic"], P["brake_tau"])
    R = Side(P["K"] * (1 - asym / 2), P["tau"], P["v_static"], P["v_kinetic"], P["brake_tau"])
    batt_factor = 0.86 + 0.14 * batt_pct / 100.0
    sgn = 1.0 if target >= 0 else -1.0
    if noise is None:
        noise = 0.004 if mode == "DRIVE" else 0.01
    dist = 0.0       # drive: distance travelled (in), signed
    heading = 0.0    # deg, clockwise +
    cx = cy = 0.0    # centre position (in)
    integral, prev_e, prev_out = 0.0, None, 0.0
    small_start = large_start = -1
    pending = [(0.0, 0.0)] * delay_ticks     # command pipeline (motor latency)
    applied = (0.0, 0.0)
    cmd = (0.0, 0.0)
    end_ms, result = None, "ok"
    rows = []
    t = 0
    t_stop = None
    max_overshoot = 0.0
    settle_mark = -1

    def true_err():
        if mode == "DRIVE":
            return (target - dist) * sgn
        return (target - heading) * sgn

    while True:
        if t % CTRL_MS == 0:
            moving = end_ms is None
            e_meas = true_err() + rng.gauss(0, noise)
            if moving:
                if prev_e is not None and (e_meas > 0) != (prev_e > 0):
                    integral = 0.0
                if abs(e_meas) < windup:
                    integral += e_meas
                deriv = 0.0 if prev_e is None else e_meas - prev_e
                out = kP * e_meas + kI * integral + kD * deriv
                out = max(-127.0, min(127.0, out))
                if abs(out) > abs(prev_out) and abs(out - prev_out) > slew:
                    out = prev_out + math.copysign(slew, out - prev_out)
                prev_e, prev_out = e_meas, out
                if mode == "DRIVE":
                    ang = kA * (0.0 - heading)
                    pl, pr = out * sgn + ang, out * sgn - ang
                else:
                    pl, pr = out * sgn, -out * sgn
                cmd = (max(-127, min(127, pl)) / 127.0 * 12000.0, max(-127, min(127, pr)) / 127.0 * 12000.0)
                # exit conditions (LemLib ExitCondition semantics)
                for which in ("small", "large"):
                    rng_, ms = small if which == "small" else large
                    st = small_start if which == "small" else large_start
                    if abs(e_meas) > rng_:
                        st = -1
                    elif st == -1:
                        st = t
                    elif t - st >= ms:
                        end_ms = t
                    if which == "small":
                        small_start = st
                    else:
                        large_start = st
                if small_start != -1 and settle_mark == -1:
                    settle_mark = small_start
                if small_start == -1:
                    settle_mark = -1
                if end_ms is None and t >= timeout:
                    end_ms = t
                if end_ms is None and abort_at is not None and t >= abort_at:
                    end_ms, result = t, "aborted:stick"
                if end_ms is not None:
                    cmd = (0.0, 0.0)
                    t_stop = end_ms + COAST_MS
            pending.append(cmd)
            applied = pending.pop(0)
            if t % dt_ms == 0:
                vl = L.v + rng.gauss(0, 1.5)
                vr = R.v + rng.gauss(0, 1.5)
                dev = heading if mode == "DRIVE" else math.hypot(cx, cy)
                rows.append((t, round(e_meas * 100), round(dev * 100), round(vl), round(vr),
                             round(cmd[0]), round(cmd[1])))
            max_overshoot = max(max_overshoot, -true_err())
            if t_stop is not None and t >= t_stop:
                break
        dt = PHYS_MS / 1000.0
        brake = end_ms is not None
        L.step(applied[0] / 1000.0 * batt_factor, dt, brake)
        R.step(applied[1] / 1000.0 * batt_factor, dt, brake)
        vl_ips, vr_ips = L.v * IPS_PER_RPM, R.v * IPS_PER_RPM
        vc = (vl_ips + vr_ips) / 2
        heading += math.degrees((vl_ips - vr_ips) / TRACK * dt)
        if mode == "DRIVE":
            dist += vc * dt
        cx += vc * math.sin(math.radians(heading)) * dt
        cy += vc * math.cos(math.radians(heading)) * dt
        t += PHYS_MS

    begin = dict(v=1, mode=mode, target="%.1f" % target, unit="in" if mode == "DRIVE" else "deg",
                 kP="%.3f" % kP, kI="%.4f" % kI, kD="%.3f" % kD, windup="%.2f" % windup,
                 slew="%.1f" % slew, small="%.2f/%d" % small, large="%.2f/%d" % large,
                 timeout=timeout, batt_mV=int(11000 + 18 * batt_pct), batt_pct=batt_pct, dt_ms=dt_ms,
                 ips_per_rpm="%.4f" % IPS_PER_RPM, track="%.2f" % TRACK, cols="t,err,dev,vL,vR,uL,uR")
    e_end = next(r[1] for r in reversed(rows) if r[0] <= end_ms) / 100.0
    end = dict(rows=len(rows), sum=sum(sum(r) for r in rows) % (1 << 32), end_ms=end_ms, result=result,
               final="%.2f" % e_end, over="%.2f" % max_overshoot, settle_ms=settle_mark)
    return begin, rows, end


def kv(d):
    return " ".join("%s=%s" % (k, v) for k, v in d.items())


def run_lines(run_no, begin, rows, end, extra_begin="", extra_end=""):
    b = "[PIDLOG] BEGIN v=1 run=%d %s%s" % (run_no, kv({k: v for k, v in begin.items() if k != "v"}), extra_begin)
    e = "[PIDLOG] END run=%d %s%s" % (run_no, kv(end), extra_end)
    return b, ["~" + ",".join(str(x) for x in r) for r in rows], e


NOISE = [
    "\x1b[32m[INFO]\x1b[0m odom: x=12.40 y=-3.10 theta=91.2",
    "\x1b[33m[WARN] intake jam detected, reversing\x1b[0m",
    "Battery: 87%",
    "~~~ auton selector: SKILLS ~~~",
    "[lemlib] chassis calibrated in 2430 ms",
    "\x1b[1;31mE: motor port 7 disconnected\x1b[0m",
    "",
]


# ---------------------------------------------------------------------------
# Scenarios: (name, sim kwargs, expected verdict, expected (param, direction) or None, expected flags, tamper)
#   direction: "up", "down", "set" (from 0), None = no tuning advice expected
#   tamper: None | "truncate" | "drop_row" | "corrupt" | "malformed" | "buffer" | "end_mismatch" | "never_ended"
# ---------------------------------------------------------------------------
SCENARIOS = [
    ("drive good",            dict(mode="DRIVE", target=24, kP=12, kD=60), "GOOD", None, [], None),
    ("drive overshoot",       dict(mode="DRIVE", target=24, kP=12, kD=25), "OVERSHOOT", ("kD", "up"), [], None),
    ("drive overshoot kD=0",  dict(mode="DRIVE", target=24, kP=20, kD=0), "OVERSHOOT", ("kD", "set"), [], None),
    ("drive oscillating kP",  dict(mode="DRIVE", target=24, kP=60, kD=10, slew=127, delay_ticks=2, timeout=6000),
     "OSCILLATING", ("kP", "down"), [], None),
    ("drive ended early",     dict(mode="DRIVE", target=24, kP=12, kD=260), "ENDED_EARLY", ("kD", "down"), [], None),
    ("drive stops short",     dict(mode="DRIVE", target=24, kP=8, kD=40, friction_scale=1.5),
     "STOPS_SHORT", ("kP", "up"), [], None),
    ("drive slow",            dict(mode="DRIVE", target=36, kP=24, kD=900, large=(3.0, 3000), timeout=6000),
     "SLOW", ("kD", "down"), [], None),
    ("drive timeout moving",  dict(mode="DRIVE", target=72, kP=12, kD=60, timeout=1200),
     "TIMEOUT", ("timeout", "up"), [], None),
    ("drive timeout creeping", dict(mode="DRIVE", target=24, kP=1.5, kD=4, friction_scale=2.4, timeout=2500),
     "TIMEOUT", ("kP", "up"), [], None),
    ("drive timeout stopped", dict(mode="DRIVE", target=24, kP=3, kD=10, friction_scale=3.0, timeout=2500),
     "TIMEOUT", ("kP", "up"), [], None),
    ("drive aborted",         dict(mode="DRIVE", target=24, kP=12, kD=60, abort_at=480), "ABORTED", None, [], None),
    ("drive low battery",     dict(mode="DRIVE", target=-24, kP=12, kD=60, batt_pct=41), "GOOD", None,
     ["battery"], None),
    ("drive drift",           dict(mode="DRIVE", target=36, kP=12, kD=60, asym=0.08, kA=1.0), "GOOD", None,
     ["heading", "faster"], None),
    ("turn good",             dict(mode="TURN", target=90, kP=4, kD=25, small=(1.0, 200)), "GOOD", None, [], None),
    ("turn overshoot",        dict(mode="TURN", target=-90, kP=5, kD=8, small=(1.0, 200)),
     "OVERSHOOT", ("kD", "up"), [], None),
    ("turn chatter",          dict(mode="TURN", target=90, kP=5, kD=200, small=(0.5, 200), noise=0.05,
                                   delay_ticks=2, slew=127, timeout=3000), "OSCILLATING", ("kD", "down"), [], None),
    # Chatter with only 2 big crossings: must NOT be called OVERSHOOT (raising kD would make it worse).
    ("turn chatter 2 swings", dict(mode="TURN", target=90, kP=5, kD=200, small=(1.0, 200), noise=0.05,
                                   delay_ticks=2, slew=127, timeout=3000), "OSCILLATING", ("kD", "down"), [], None),
    # Chatter that never arrives: must NOT be told "raise the timeout".
    ("turn chatter timeout",  dict(mode="TURN", target=90, kP=5, kD=350, small=(1.0, 200), noise=0.05,
                                   delay_ticks=2, slew=127, timeout=3000), "TIMEOUT", ("kD", "down"), [], None),
    ("turn wander",           dict(mode="TURN", target=180, kP=4, kD=25, small=(1.0, 200), asym=0.25),
     "GOOD", None, ["centre wandered"], None),
    ("drive truncated",       dict(mode="DRIVE", target=24, kP=12, kD=60), "DAMAGED", None, [], "truncate"),
    ("drive dropped row",     dict(mode="DRIVE", target=24, kP=12, kD=60), "DAMAGED", None, [], "drop_row"),
    ("drive bad checksum",    dict(mode="DRIVE", target=24, kP=12, kD=60), "DAMAGED", None, [], "corrupt"),
    ("drive malformed row",   dict(mode="DRIVE", target=24, kP=12, kD=60), "DAMAGED", None, [], "malformed"),
    ("drive buffer full",     dict(mode="DRIVE", target=24, kP=12, kD=60), "TIMEOUT", ("kP", "up"), [], "buffer"),
    ("drive END mismatch",    dict(mode="DRIVE", target=24, kP=12, kD=60), "GOOD", None,
     ["END line disagrees"], "end_mismatch"),
    ("drive never ended",     dict(mode="DRIVE", target=24, kP=3, kD=10, friction_scale=3.0, timeout=2500),
     "TIMEOUT", ("kP", "up"), [], "never_ended"),
]


def build_capture(path, seed=7):
    rng = random.Random(seed)
    lines = ["\x1b[0mPROS Terminal", "Waiting for device..."]
    expected = []
    run_no = 100
    for k, (name, kw, verdict, sug, flags, tamper) in enumerate(SCENARIOS):
        run_no += 1
        begin, rows, end = simulate(seed=k + 1, **kw)
        rows = list(rows)
        if tamper == "drop_row":
            del rows[len(rows) // 3]
        elif tamper == "corrupt":
            r = list(rows[len(rows) // 2])
            r[1] += 7          # one flipped digit: still parses, checksum catches it
            rows[len(rows) // 2] = tuple(r)
        elif tamper == "buffer":
            rows = rows[:40]
            end["rows"] = len(rows)
            end["sum"] = sum(sum(r) for r in rows) % (1 << 32)
            end["result"] = "buffer"
            end["end_ms"] = -1  # the firmware sends -1: the move never ended inside the recording
        elif tamper == "end_mismatch":
            end["over"] = "1.75"
        elif tamper == "never_ended":
            end["end_ms"] = -1
        b, data, e = run_lines(run_no, begin, rows, end,
                               extra_begin=" robot=pepsi fw=0.9.1" if k % 3 == 0 else "",
                               extra_end=" heap=81234" if k % 4 == 0 else "")
        if tamper == "malformed":
            data[len(data) // 2] = "~240,1203,4,odom: x=1"   # another task printed mid-row
        if tamper == "truncate":
            data = data[: len(data) // 2]
            e = None
        if k % 2 == 0:
            b = "\x1b[36m" + b + "\x1b[0m"
        lines.append(b)
        for i, d in enumerate(data):
            if rng.random() < 0.04:
                lines.append(rng.choice(NOISE))
            if i == 5 and k % 5 == 1:
                d = "[odom] tick" + d       # glued onto the end of someone else's line: recoverable
            lines.append(d)
        if e is not None:
            lines.append(e)
        lines.append(rng.choice(NOISE))
        expected.append((run_no, name, verdict, sug, flags, tamper))
    # stray rows and END outside any run must be ignored
    lines.append("~0,1,2,3,4,5,6")
    lines.append("[PIDLOG] END run=999 rows=0 sum=0 end_ms=0 result=ok")
    with open(path, "wb") as fh:
        fh.write(("\r\n".join(lines) + "\r\n").encode("utf-8"))
    return expected


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None, help="output directory (default: a temp dir)")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()
    out = args.out
    if out is None:
        import tempfile
        out = tempfile.mkdtemp(prefix="pidlog_selftest_")
    os.makedirs(out, exist_ok=True)
    cap = os.path.join(out, "selftest_capture.log")
    expected = build_capture(cap)
    cmd = [sys.executable, TOOL, cap, "--out", out, "--json"] + (["--no-plots"] if args.no_plots else [])
    proc = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8")
    print(proc.stdout)
    if proc.returncode != 0:
        print(proc.stderr)
        print("FAIL: pidlog.py exited with %d" % proc.returncode)
        return 1
    with open(os.path.join(out, "metrics.json"), encoding="utf-8") as fh:
        data = json.load(fh)
    runs = data["runs"]
    failures = []

    def check(cond, msg):
        if not cond:
            failures.append(msg)

    check(len(runs) == len(expected), "expected %d runs, tool found %d" % (len(expected), len(runs)))
    for (run_no, name, verdict, sug, flags, tamper), r in zip(expected, runs):
        tag = "run %d (%s)" % (run_no, name)
        check(str(r["run"]) == str(run_no), "%s: run number parsed as %s" % (tag, r["run"]))
        check(r["verdict"] == verdict, "%s: verdict %s, expected %s  [%s]" % (tag, r["verdict"], verdict, r["diagnosis"]))
        s = r["suggestion"]
        if verdict in ("DAMAGED", "ABORTED"):
            check(s is None, "%s: gave tuning advice on a %s run" % (tag, verdict))
        elif sug is None:
            check(s is None or s.get("param") is None, "%s: unexpected change %s" % (tag, s and s["text"]))
        else:
            param, direction = sug
            if s is None or s.get("param") != param:
                check(False, "%s: expected a %s change, got %s" % (tag, param, s and s["text"]))
            else:
                if direction == "up":
                    check(s["new"] > s["old"], "%s: %s should go up: %s" % (tag, param, s["text"]))
                elif direction == "down":
                    check(s["new"] < s["old"], "%s: %s should go down: %s" % (tag, param, s["text"]))
                elif direction == "set":
                    check(s["old"] == 0 and s["new"] > 0, "%s: %s should be set from 0: %s" % (tag, param, s["text"]))
        alltext = " ".join(r["flags"])
        for f in flags:
            check(f in alltext, "%s: missing flag containing '%s' (flags: %s)" % (tag, f, r["flags"]))
        if "battery" not in flags:
            check("battery" not in alltext, "%s: unexpected battery flag" % tag)
        if "heading" not in flags and r["mode"] == "DRIVE" and verdict not in ("DAMAGED",):
            check("heading drifted" not in alltext, "%s: unexpected heading flag" % tag)
        if tamper == "truncate":
            check(any("no END" in d for d in r["damage"]), "%s: truncation not reported" % tag)
        if tamper == "drop_row":
            check(any("rows=" in d for d in r["damage"]), "%s: dropped row not reported" % tag)
            check(any("checksum" in d for d in r["damage"]), "%s: checksum mismatch not reported" % tag)
        if tamper == "corrupt":
            check(any("checksum" in d for d in r["damage"]) and not any("rows=" in d for d in r["damage"]),
                  "%s: corruption should be a checksum-only failure: %s" % (tag, r["damage"]))
        if tamper == "malformed":
            check(any("malformed" in d for d in r["damage"]), "%s: malformed row not reported" % tag)
        if tamper == "buffer":
            check(any("buffer" in d for d in r["notes"]), "%s: buffer result not reported" % tag)
        if tamper in (None, "end_mismatch", "never_ended", "buffer"):
            check(not r["damage"], "%s: clean run reported damaged: %s" % (tag, r["damage"]))
    check(data["stats"]["orphan_rows"] == 1, "stray data row outside a run not counted")
    check(data["stats"]["orphan_ends"] == 1, "stray END outside a run not counted")
    if not args.no_plots and not data.get("plots_skipped"):
        for r in runs:
            p = data["plots"].get(str(r["index"]))
            check(p and os.path.getsize(p) > 10000, "run %s: plot missing" % r["run"])
        for mode in ("DRIVE", "TURN"):
            check(os.path.exists(os.path.join(out, "overview_%s.png" % mode.lower())), "overview %s missing" % mode)
    elif data.get("plots_skipped"):
        check("skipped" in proc.stdout, "plots skipped but the text output does not say so")

    print("=" * 72)
    if failures:
        for f in failures:
            print("FAIL: " + f)
        print("%d check(s) failed. Output in %s" % (len(failures), out))
        return 1
    print("selftest passed: %d runs, all verdicts, suggestion directions and flags as expected." % len(runs))
    print("Output in " + out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
