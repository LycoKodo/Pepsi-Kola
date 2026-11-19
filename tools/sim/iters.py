#!/opt/anaconda3/bin/python3
"""Print the motion-loop iterations around `close` from a bench CSV, and the stale fraction.

    iters.py out/drive48_before.csv [--before 5 --after 5]
"""
import argparse
import csv

ap = argparse.ArgumentParser()
ap.add_argument("csv")
ap.add_argument("--before", type=int, default=5)
ap.add_argument("--after", type=int, default=5)
a = ap.parse_args()
with open(a.csv) as fh:
    rows = [r for r in csv.DictReader(fh) if r["iter_strobe"] == "1"]
f = lambda r, k: float(r[k])
k_close = next((i for i, r in enumerate(rows) if r["close"] == "1"), None)
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
