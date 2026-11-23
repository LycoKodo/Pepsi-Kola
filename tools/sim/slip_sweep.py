"""Slip sweep: a calibrated robot with heavier tracking-wheel slip than the suites use, filters off and on.

/opt/anaconda3/bin/python3 tools/sim/slip_sweep.py      (needs `make -C tools/sim` first; about 3 s)

The suites use one slip level (SLIP_PER_IN 0.03, SLIP_MAX_IN 0.3: about 0.45 % of travel lost). This runs the GPS
route (GPS On against odometry alone) and the wall route (one back sensor against odometry alone) at five levels,
30 seeds each, and prints the median end error, whole-route RMS and faults. Results: docs/OVERNIGHT_REPORT_2026-11-22.md.
"""
import sys, os, json, subprocess, statistics
from concurrent.futures import ThreadPoolExecutor
SIM = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SIM)
import gps_suite as gs
import wall_suite as ws
TB = gs.TB_LIVE
SEEDS = list(range(1, 31))
# (label, SLIP_PER_IN, SLIP_MAX_IN)
LEVELS = [("none", 0, 0), ("as modelled 0.45%", 0.03, 0.3), ("1.5%", 0.1, 0.3), ("5%", 0.1, 1.0), ("10%", 0.2, 1.0)]
base = gs.odom_errors(gs.A3B, slips=False)

def run(route, args):
    cmd = [TB, "--route", route] + [str(a) for a in args] + ["--summary-json"]
    d = subprocess.run(cmd, capture_output=True, text=True)
    if d.returncode != 0:
        raise RuntimeError(" ".join(cmd) + "\n" + d.stderr)
    return json.loads(d.stdout)

def slip(per, mx):
    return ["--set", f"SLIP_PER_IN={per}", "--set", f"SLIP_MAX_IN={mx}"] if per else []

def job(j):
    kind, lvl, seed = j
    _, per, mx = LEVELS[lvl]
    e = base(seed) + slip(per, mx)
    if kind == "gps_off":
        return j, run(gs.ROUTE, e + ["--seed", seed, "--fusion", "off"])
    if kind == "gps_on":
        return j, run(gs.ROUTE, e + gs.NOISE + ["--seed", seed, "--fusion", "on"])
    if kind == "wall_off":
        return j, run(ws.ROUTE, e + ["--no-gps", "--seed", seed, "--fusion", "off"])
    if kind == "wall_on":
        return j, run(ws.ROUTE, e + ["--no-gps"] + ws.wall_args(seed, 1) + ["--seed", seed, "--fusion", "on"])

jobs = [(k, l, s) for k in ("gps_off", "gps_on", "wall_off", "wall_on") for l in range(len(LEVELS)) for s in SEEDS]
with ThreadPoolExecutor(8) as ex:
    res = dict(ex.map(job, jobs))
json.dump({f"{k}|{l}|{s}": v for (k, l, s), v in res.items()}, open(os.path.join(SIM, "out", "slip_sweep.json"), "w"))
med = statistics.median
one = res[("gps_on", 1, 1)]
print("filter keys:", list(one.get("filter", {}).keys()))
for off, on, name in (("gps_off", "gps_on", "GPS, skills60"), ("wall_off", "wall_on", "one back wall sensor, walls60")):
    print(f"\n== {name}: calibrated robot, {len(SEEDS)} seeds ==")
    print(f"{'slip':22s} {'slips':>6s} {'odo end':>8s} {'fus end':>8s} {'odo rms':>8s} {'fus rms':>8s} {'odo max':>8s} {'worse':>6s} {'faults':>6s}")
    for l, (label, _, _) in enumerate(LEVELS):
        o = [res[(off, l, s)] for s in SEEDS]; f = [res[(on, l, s)] for s in SEEDS]
        worse = sum(1 for a, b in zip(o, f) if b["fused"]["end"] > a["odom"]["end"])
        faults = sum(1 for b in f if b.get("filter", {}).get("fault", 0) not in (0, "none", None))
        print(f"{label:22s} {med([a['world']['slips'] for a in o]):6.0f} {med([a['odom']['end'] for a in o]):8.2f} "
              f"{med([b['fused']['end'] for b in f]):8.2f} {med([a['odom']['rms'] for a in o]):8.2f} "
              f"{med([b['fused']['rms'] for b in f]):8.2f} {med([a['odom']['max'] for a in o]):8.2f} {worse:4d}/{len(SEEDS)} {faults:6d}")
