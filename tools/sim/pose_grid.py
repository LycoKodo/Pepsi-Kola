#!/opt/anaconda3/bin/python3
"""Run a set of moveToPose targets on one bench binary and print one row per pose.

    pose_grid.py [--dut fix1] [--drift 8] [--lead 0.6] [--max-speed 127] [--min-speed 0] [--mu-lat 0.3]
                 [--timeout 4000] [--csv table.csv] [--save-runs DIR] [--poses "x,y,th[,b];..."] [-- extra tb args]

Every move starts at (0, 0, heading 0). Columns:
  time     ms until moveToPose returned
  ended    exits (stock loop condition met, mirrored), timeout, or other (stopped earlier for a reason
           the stock mirror does not see: an early exit with minSpeed, or a reworked exit rule)
  err      final position error, in (true pose, 300 ms after the motion ended)
  along    error along the target heading, in, + = short of the target
  across   error across the target heading, in, + = robot ended right of the target line
  head     final heading error, deg, + = robot should still turn clockwise
  peak     peak forward speed, in/s
  cap_ms   time with the slip cap binding (|lateralOut as sent| within 1 of the cap, cap < maxSpeed)
  minLat   lowest non-zero |lateralOut as sent| while close (inside 7.5 in)
  side     peak sideways speed, in/s (0 unless --mu-lat lets the wheels slide)
  mirror   iterations where the stock-moveToPose mirror disagrees with the DUT's commands (shown if any)
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))

# (x, y, theta, backwards, what it exercises)
DEFAULT_POSES = [
    (0, 24, 0, False, "straight"),
    (12, 24, 0, False, "offset, same heading"),
    (-12, 24, 0, False, "offset, same heading"),
    (24, 24, 90, False, "quarter arc"),
    (-24, 24, -90, False, "quarter arc"),
    (0, 36, 45, False, "end at 45"),
    (24, 48, 0, False, "S-shape"),
    (12, 12, 90, False, "tight"),
    (36, 12, 90, False, "wide"),
    (0, 24, 180, False, "arrive reversed"),
    (0, -24, 0, True, "backwards"),
    (4, 6, 0, False, "short"),
    (2, 4, 30, False, "short"),
]


def parse_poses(text):
    out = []
    for item in text.split(";"):
        f = [v.strip() for v in item.split(",") if v.strip()]
        if not f:
            continue
        back = len(f) > 3 and f[3].lower() in ("b", "back", "backwards", "1", "true")
        out.append((float(f[0]), float(f[1]), float(f[2]), back, ""))
    return out


def run_pose(tb, pose, a, extra):
    x, y, th, back, _ = pose
    cmd = [tb, "--move", "pose", "--x", f"{x:g}", "--y", f"{y:g}", "--theta", f"{th:g}", "--summary-json"]
    if back:
        cmd.append("--backwards")
    for opt, val in (("--drift", a.drift), ("--lead", a.lead), ("--max-speed", a.max_speed),
                     ("--min-speed", a.min_speed), ("--timeout", a.timeout)):
        if val is not None:
            cmd += [opt, f"{val:g}"]
    if a.mu_lat is not None:
        cmd += ["--set", f"MU_LAT={a.mu_lat:g}"]
    if a.save_runs:
        os.makedirs(a.save_runs, exist_ok=True)
        name = f"pose_{x:g}_{y:g}_{th:g}{'_b' if back else ''}.csv".replace("-", "m")
        cmd += ["--csv", os.path.join(a.save_runs, name)]
    cmd += extra
    out = subprocess.run(cmd, capture_output=True, text=True, check=True).stdout
    return json.loads(out.strip().splitlines()[-1])


def fmt(v, spec):
    return "-" if v is None else format(v, spec)


def main():
    argv = sys.argv[1:]
    extra = []
    if "--" in argv:
        k = argv.index("--")
        argv, extra = argv[:k], argv[k + 1:]
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dut", default="fix1")
    ap.add_argument("--drift", type=float)
    ap.add_argument("--lead", type=float)
    ap.add_argument("--max-speed", type=float)
    ap.add_argument("--min-speed", type=float)
    ap.add_argument("--mu-lat", type=float)
    ap.add_argument("--timeout", type=float)
    ap.add_argument("--poses", help='override the pose set: "x,y,theta[,b];..." (b = backwards)')
    ap.add_argument("--csv", help="also write the table as CSV")
    ap.add_argument("--save-runs", help="directory for each pose's full bench CSV")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 8)
    a = ap.parse_args(argv)
    tb = os.path.join(HERE, "build", a.dut, f"tb_{a.dut}")
    if not os.path.exists(tb):
        sys.exit(f"{tb} not built: make -C {HERE} DUT={a.dut}")
    poses = parse_poses(a.poses) if a.poses else DEFAULT_POSES
    with ThreadPoolExecutor(a.jobs) as pool:
        results = list(pool.map(lambda p: run_pose(tb, p, a, extra), poses))

    first = results[0]["pose"]
    print(f"tb_{a.dut}: drift {first['drift']:g}, lead {first['lead']:g}, maxSpeed {first['max_speed']:g}, "
          f"minSpeed {first['min_speed']:g}, MU_LAT {results[0]['mu_lat']:g}, timeout {first['timeout']} ms")
    hdr = (f"{'pose (x, y, th)':<22} {'time':>5} {'ended':>7} {'err':>5} {'along':>6} {'across':>6} {'head':>6} "
           f"{'peak':>5} {'cap_ms':>6} {'minLat':>6} {'side':>5}")
    print(hdr)
    print("-" * len(hdr))
    rows = []
    for (x, y, th, back, what), r in zip(poses, results):
        p = r["pose"]
        name = f"({x:g}, {y:g}, {th:g}){' back' if back else ''}"
        line = (f"{name:<22} {r['motion_end_ms']:>5} {p['ended_by']:>7} {fmt(p['pos_err'], '5.2f')} "
                f"{fmt(p['along'], '+6.2f')} {fmt(p['across'], '+6.2f')} {fmt(p['heading_err'], '+6.1f')} "
                f"{fmt(p['peak_fwd'], '5.1f')} {fmt(p['cap_bind_ms'], '6.0f')} {fmt(p['min_lat_close'], '6.2f')} "
                f"{fmt(r['peak_side'], '5.1f')}")
        if p["mirror_mismatches"]:
            line += f"  mirror differs on {p['mirror_mismatches']} iterations"
        print(line)
        rows.append({"x": x, "y": y, "theta": th, "backwards": back, "what": what, "time_ms": r["motion_end_ms"],
                     "ended_by": p["ended_by"], "err_in": p["pos_err"], "along_in": p["along"], "across_in": p["across"],
                     "heading_err_deg": p["heading_err"], "peak_fwd_ips": p["peak_fwd"], "cap_bind_ms": p["cap_bind_ms"],
                     "min_lat_close": p["min_lat_close"], "peak_side_ips": r["peak_side"],
                     "side_dist_in": r["side_dist"], "odom_err_in": p["odom_err"],
                     "mirror_mismatches": p["mirror_mismatches"], "drift": p["drift"], "lead": p["lead"],
                     "max_speed": p["max_speed"], "min_speed": p["min_speed"], "mu_lat": r["mu_lat"], "dut": a.dut})
    if a.csv:
        with open(a.csv, "w", newline="") as fh:
            wr = csv.DictWriter(fh, fieldnames=list(rows[0]))
            wr.writeheader()
            wr.writerows(rows)
        print(f"wrote {a.csv}")


if __name__ == "__main__":
    main()
