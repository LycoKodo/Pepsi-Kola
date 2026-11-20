#!/opt/anaconda3/bin/python3
"""Print motion-loop iterations from a bench CSV.

Drive CSV: the iterations around `close`, and the stale fraction.
    iters.py out/drive48_before.csv [--before 5 --after 5]
Pose CSV (--move pose): the mirrored moveToPose stages per iteration, the last N by default,
or a window around close, or a time range.
    iters.py pose.csv [--last 12 | --around-close | --from MS --to MS] [--every K]
"""
import argparse
import csv
import math

LIMIT = ["none", "maxSpeed", "slew", "cap", "overturn", "direction", "minSpeed"]

ap = argparse.ArgumentParser()
ap.add_argument("csv")
ap.add_argument("--before", type=int, default=5)
ap.add_argument("--after", type=int, default=5)
ap.add_argument("--last", type=int, default=12)
ap.add_argument("--around-close", action="store_true")
ap.add_argument("--from", dest="t0", type=float)
ap.add_argument("--to", dest="t1", type=float)
ap.add_argument("--every", type=int, default=1, help="print every K-th iteration")
a = ap.parse_args()
with open(a.csv) as fh:
    rows = [r for r in csv.DictReader(fh) if r["iter_strobe"] == "1"]
f = lambda r, k: float(r[k])
k_close = next((i for i, r in enumerate(rows) if r["close"] == "1"), None)

if "p_dist" in rows[0]:
    if a.t0 is not None or a.t1 is not None:
        sel = [r for r in rows if (a.t0 is None or f(r, "t_ms") >= a.t0) and (a.t1 is None or f(r, "t_ms") <= a.t1)]
    elif a.around_close and k_close is not None:
        sel = rows[max(0, k_close - a.before):k_close + a.after]
    else:
        sel = rows[-a.last:]
    sel = sel[::a.every]
    bad = sum(r["p_mirror_ok"] == "0" for r in rows)
    print(f"{a.csv}: {len(rows)} iterations; close at iteration {None if k_close is None else k_close + 1}; "
          f"mirror differs from the DUT on {bad} iterations")
    print(f"{'iter':>4} {'t_ms':>5} {'dist':>6} {'latErr':>6} {'angErr':>7} {'latPID':>7} {'angOut':>7} {'maxSp':>5} "
          f"{'radius':>7} {'cap':>6} {'ovrtrn':>6} {'latOut':>6} {'L':>4} {'R':>4} {'latSent':>7} {'limit':>9} bind "
          f"exits(LS LL AS AL set) close")
    for r in sel:
        rad = f(r, "p_radius")
        print(f"{int(f(r, 'iter_n')):>4} {int(f(r, 't_ms')):>5} {f(r, 'p_dist'):>6.2f} {f(r, 'p_lat_err'):>6.2f} "
              f"{f(r, 'p_ang_err'):>7.2f} {f(r, 'p_lat_pid'):>7.1f} {f(r, 'p_ang_out'):>7.1f} {f(r, 'p_max_speed'):>5.0f} "
              f"{('inf' if math.isinf(rad) or rad > 1e6 else f'{rad:.2f}'):>7} {min(f(r, 'p_cap'), 9999):>6.2f} "
              f"{f(r, 'p_overturn'):>6.1f} {f(r, 'p_lat_out'):>6.2f} {r['cmdL']:>4} {r['cmdR']:>4} {f(r, 'p_lat_sent'):>7.1f} "
              f"{LIMIT[int(f(r, 'p_limit'))]:>9} {r['p_cap_bind']:>4}    {r['p_ex_lat_small']}  {r['p_ex_lat_large']}  "
              f"{r['p_ex_ang_small']}  {r['p_ex_ang_large']}  {r['p_lat_settled']}    {r['close']}")
else:
    moving = [r for r in rows if (abs(f(r, "vL")) + abs(f(r, "vR"))) / 2 > 2]
    print(f"{a.csv}: {len(rows)} iterations; stale {sum(r['iter_fresh'] == '0' for r in rows) / len(rows):.0%} of all, "
          f"{sum(r['iter_fresh'] == '0' for r in moving) / max(1, len(moving)):.0%} of the {len(moving)} while moving (wheels > 2 in/s)")
    if k_close is None:
        print("close never went true")
    else:
        print(f"{'iter':>4} {'t_ms':>5} {'dist_in':>7} {'err_in':>7} {'fresh':>5} {'P':>7} {'D':>7} {'P+I+D':>7} {'cmdL':>5} {'cmdR':>5} {'v_in/s':>6} close")
        for r in rows[max(0, k_close - a.before):k_close + a.after]:
            print(f"{int(f(r, 'iter_n')):>4} {int(f(r, 't_ms')):>5} {f(r, 'iter_dist'):>7.2f} {f(r, 'iter_err'):>7.2f} "
                  f"{r['iter_fresh']:>5} {f(r, 'iter_P'):>7.1f} {f(r, 'iter_D'):>7.1f} {f(r, 'iter_pid'):>7.1f} "
                  f"{r['cmdL']:>5} {r['cmdR']:>5} {f(r, 'v'):>6.1f} {r['close']}")
