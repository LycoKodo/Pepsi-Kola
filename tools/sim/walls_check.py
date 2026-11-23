#!/usr/bin/env python3
"""Geometric check of the wall-test routes (walls60.txt, walls_corner.txt).

Reads a route in the language of tools/sim/bench/route.cpp (start / point / turn / pose / wait) and, for the
planned poses:
  - checks that the robot's outline (18 in square, centred on the tracking centre) never leaves the field, along
    a simple version of the path: turn in place toward the target, then drive straight (moveToPoint really
    curves a little; the bench's true path can be checked with --csv);
  - lists every stop (wait) with, for the back and the right sensor: the range along the centre ray to the wall
    it meets, how far off square it is, where the beam lands along the wall and how far that is from the nearest
    corner, whether the filter's check 8 would pass (cone of 18 degrees, footprint widened by 2 in, kept 6 in from
    corners and off keep-outs), and what the true cone (12 and 18 degrees) meets;
  - says which stops meet the route's targets: back sensor within 24 in, within 6 degrees of square, beam at
    least 20 in from any corner, stop at least 0.6 s; and the same for the right sensor.

With --csv TICKS.csv (a tb_live --route run of the same route), the same table is made from the TRUE pose at the
end of each stop, and the outline is checked at every tick.

Wall-mounted elements for the corner route are declared in the route file as comments the bench ignores:
    #@element WALL FROM TO DEPTH      (wall 0 = +y, 1 = +x, 2 = -y, 3 = -x; FROM/TO along the wall, field inches)

    /opt/anaconda3/bin/python3 walls_check.py walls60.txt [--csv ticks.csv] [--quiet]
"""
import argparse
import csv
import math
import sys

WALL = [70.25, 70.25, 70.25, 70.25]  # inside faces: 0 = +y, 1 = +x, 2 = -y, 3 = -x
HALF_SIZE = 9.0  # the robot is 18 in square
SENSORS = [("back", 0.0, -8.5, 180.0), ("right", 8.5, 0.0, 90.0)]  # name, right, forward, yaw (deg)
# the route's targets for a stop that counts
TARGET_RANGE_IN = 24.0
TARGET_SQUARE_DEG = 6.0
TARGET_CORNER_IN = 20.0
TARGET_STOP_MS = 600
# the filter's check 8 (docs/WALL_FIX_DESIGN.md, first values)
FILTER_CONE_DEG = 18.0
FOOTPRINT_MARGIN_IN = 2.0
CORNER_MARGIN_IN = 6.0
# the model's true cone (dist_model.hpp defaults: 12 degrees, 18 below 200 mm)
MODEL_CONE_DEG = 12.0
MODEL_CONE_CLOSE_DEG = 18.0
MODEL_CLOSE_IN = 200 / 25.4
EVEN_RAYS = 49


def wrap180(a):
    a = math.fmod(a + 180.0, 360.0)
    if a < 0:
        a += 360.0
    return a - 180.0


# ------------------------------------------------------------------------------------------------ the route
def parse_route(path):
    start, steps, elements = None, [], []
    with open(path) as f:
        for line_no, raw in enumerate(f, 1):
            if raw.startswith("#@element"):
                w = raw.split()
                elements.append((int(w[1]), float(w[2]), float(w[3]), float(w[4])))
                continue
            line = raw.split("#")[0].split()
            if not line:
                continue
            cmd, rest = line[0], line[1:]
            nums = [float(x) for x in rest if _is_num(x)]
            words = [x for x in rest if not _is_num(x)]
            if cmd == "start":
                start = tuple(nums)
                continue
            step = {"cmd": cmd, "nums": nums, "back": "back" in words, "line": line_no, "text": raw.split("#")[0].strip(),
                    "comment": raw.split("#", 1)[1].strip() if "#" in raw else "",
                    "dir": 1 if "cw" in words else -1 if "ccw" in words else 0}
            steps.append(step)
    return start, steps, elements


def _is_num(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


def plan(start, steps):
    """Planned poses: one entry per step with the pose after it (heading unwrapped), and the path samples."""
    x, y, h = start
    samples = [(x, y, h, 0)]
    out = []
    for i, s in enumerate(steps, 1):
        if s["cmd"] in ("point", "pose"):
            tx, ty = s["nums"][0], s["nums"][1]
            travel = math.degrees(math.atan2(tx - x, ty - y)) + (180.0 if s["back"] else 0.0)
            h1 = h + wrap180(travel - h)  # it turns toward the target the shorter way
            for k in range(1, 21):
                samples.append((x, y, h + (h1 - h) * k / 20, i))
            dist = math.hypot(tx - x, ty - y)
            n = max(1, int(dist / 0.5))
            for k in range(1, n + 1):
                samples.append((x + (tx - x) * k / n, y + (ty - y) * k / n, h1, i))
            x, y, h = tx, ty, h1
            if s["cmd"] == "pose":
                h2 = h + wrap180(s["nums"][2] - h)
                for k in range(1, 21):
                    samples.append((x, y, h + (h2 - h) * k / 20, i))
                h = h2
        elif s["cmd"] == "turn":
            target = s["nums"][0]
            if s["dir"] == 0:
                d = wrap180(target - h)
            elif s["dir"] > 0:
                d = (target - h) % 360.0
            else:
                d = -((h - target) % 360.0)
            for k in range(1, 41):
                samples.append((x, y, h + d * k / 40, i))
            h += d
        out.append((i, s, x, y, h))
    return out, samples


# ------------------------------------------------------------------------------------------------ geometry
def outline_clearance(x, y, hdeg):
    """Smallest distance from a corner of the robot to a wall (negative = outside the field)."""
    hr = math.radians(hdeg)
    s, c = math.sin(hr), math.cos(hr)
    worst = 1e9
    for r, f in ((HALF_SIZE, HALF_SIZE), (HALF_SIZE, -HALF_SIZE), (-HALF_SIZE, HALF_SIZE), (-HALF_SIZE, -HALF_SIZE)):
        cx = x + r * c + f * s
        cy = y - r * s + f * c
        worst = min(worst, WALL[1] - cx, cx + WALL[3], WALL[0] - cy, cy + WALL[2])
    return worst


def boxes_of(elements):
    out = []
    for wall, a, b, d in elements:
        lo, hi = min(a, b), max(a, b)
        W = WALL[wall]
        if wall == 0:
            out.append((lo, hi, W - d, W))
        elif wall == 1:
            out.append((W - d, W, lo, hi))
        elif wall == 2:
            out.append((lo, hi, -W, -W + d))
        else:
            out.append((-W, -W + d, lo, hi))
    return out


def cast(sx, sy, hdeg, boxes=()):
    """First hit: (range, kind, index) with kind 'wall' or 'elem'."""
    hr = math.radians(hdeg)
    dx, dy = math.sin(hr), math.cos(hr)
    best = (math.inf, None, -1)
    if dy > 1e-12:
        best = min(best, ((WALL[0] - sy) / dy, "wall", 0))
    if dx > 1e-12:
        best = min(best, ((WALL[1] - sx) / dx, "wall", 1))
    if dy < -1e-12:
        best = min(best, ((-WALL[2] - sy) / dy, "wall", 2))
    if dx < -1e-12:
        best = min(best, ((-WALL[3] - sx) / dx, "wall", 3))
    for k, (x0, x1, y0, y1) in enumerate(boxes):
        te, tx = -math.inf, math.inf
        miss = False
        for s0, d, lo, hi in ((sx, dx, x0, x1), (sy, dy, y0, y1)):
            if abs(d) < 1e-12:
                if s0 < lo or s0 > hi:
                    miss = True
            else:
                t1, t2 = (lo - s0) / d, (hi - s0) / d
                te, tx = max(te, min(t1, t2)), min(tx, max(t1, t2))
        if not miss and te <= tx and tx >= 0:
            best = min(best, (max(0.0, te), "elem", k))
    return best


def along_of(wall, px, py):
    return px if wall in (0, 2) else py


def corner_distance(wall, along):
    lo, hi = (-WALL[3], WALL[1]) if wall in (0, 2) else (-WALL[2], WALL[0])
    return min(along - lo, hi - along)


def cone_targets(sx, sy, phi, half, boxes):
    """Set of targets ('wall', i) / ('elem', k) met first by any ray in the cone."""
    dirs = [phi - half + 2 * half * k / (EVEN_RAYS - 1) for k in range(EVEN_RAYS)]
    for a in (0, 90, 180, 270):
        if abs(wrap180(a - phi)) <= half:
            dirs.append(a)
    for (x0, x1, y0, y1) in boxes:
        for cx, cy in ((x0, y0), (x0, y1), (x1, y0), (x1, y1)):
            a = math.degrees(math.atan2(cx - sx, cy - sy))
            if abs(wrap180(a - phi)) <= half:
                dirs.append(a)
    return {cast(sx, sy, a, boxes)[1:] for a in dirs}


def sensor_view(x, y, hdeg, sensor, elements):
    name, r, f, yaw = sensor
    hr = math.radians(hdeg)
    sx = x + r * math.cos(hr) + f * math.sin(hr)
    sy = y - r * math.sin(hr) + f * math.cos(hr)
    phi = hdeg + yaw
    boxes = boxes_of(elements)
    rng, kind, idx = cast(sx, sy, phi)  # walls only: what the filter predicts
    wall = idx
    hr2 = math.radians(phi)
    dx, dy = math.sin(hr2), math.cos(hr2)
    hit_x, hit_y = sx + rng * dx, sy + rng * dy
    along = along_of(wall, hit_x, hit_y)
    normal_comp = abs(dy) if wall in (0, 2) else abs(dx)
    off_square = math.degrees(math.acos(min(1.0, normal_comp)))
    perp = rng * normal_comp
    # the filter's check 8: both edge rays on the same wall; footprint + margin clear of corners and keep-outs
    edges = []
    same = True
    for a in (phi - FILTER_CONE_DEG, phi + FILTER_CONE_DEG):
        t, _, w2 = cast(sx, sy, a)
        if w2 != wall:
            same = False
        ha = math.radians(a)
        edges.append(along_of(wall, sx + t * math.sin(ha), sy + t * math.cos(ha)))
    lo, hi = min(edges) - FOOTPRINT_MARGIN_IN, max(edges) + FOOTPRINT_MARGIN_IN
    c_lo, c_hi = (-WALL[3], WALL[1]) if wall in (0, 2) else (-WALL[2], WALL[0])
    corner_ok = lo >= c_lo + CORNER_MARGIN_IN and hi <= c_hi - CORNER_MARGIN_IN
    keep_ok = all(not (ew == wall and min(a, b) < hi and max(a, b) > lo) for ew, a, b, _ in elements)
    check8 = same and corner_ok and keep_ok
    # the true cone
    true_half = MODEL_CONE_CLOSE_DEG if cast(sx, sy, phi, boxes)[0] < MODEL_CLOSE_IN else MODEL_CONE_DEG
    t12 = cone_targets(sx, sy, phi, true_half, boxes)
    t18 = cone_targets(sx, sy, phi, 18.0, boxes)

    def describe(ts):
        # "w23" = walls 2 and 3 (a corner); "+E" = an element; "E" alone = only the element
        walls = sorted(i for k, i in ts if k == "wall")
        s = ("w" + "".join(str(i) for i in walls)) if walls else ""
        if any(k == "elem" for k, _ in ts):
            s += "+E" if walls else "E"
        return s

    return {"name": name, "range": rng, "perp": perp, "wall": wall, "off": off_square, "along": along,
            "corner": corner_distance(wall, along), "check8": check8, "same": same, "corner_ok": corner_ok,
            "keep_ok": keep_ok, "cone12": describe(t12), "cone18": describe(t18),
            "counts": rng <= TARGET_RANGE_IN and off_square <= TARGET_SQUARE_DEG and corner_distance(wall, along) >= TARGET_CORNER_IN}


# ------------------------------------------------------------------------------------------------ report
WALL_NAME = ["+y", "+x", "-y", "-x"]


def stop_table(stops, elements, title):
    print(title)
    print("  step  line  wait  pose (x, y, heading)     | sensor  wall  range  perp   off   along  corner chk8  cone12 cone18 counts")
    n_back = n_both = 0
    walls_back = set()
    for (i, s, x, y, h, wait) in stops:
        views = [sensor_view(x, y, h, sen, elements) for sen in SENSORS]
        long_enough = wait >= TARGET_STOP_MS
        for k, v in enumerate(views):
            lead = f"  {i:4d}  {s['line']:4d}  {wait:4d}  ({x:7.2f}, {y:7.2f}, {h % 360:6.1f}) |" if k == 0 else " " * 50 + "|"
            flag = "yes" if v["counts"] and long_enough else "-"
            reason = "" if v["check8"] else " (" + ",".join(
                n for n, ok in (("2nd wall", v["same"]), ("corner", v["corner_ok"]), ("keep-out", v["keep_ok"])) if not ok) + ")"
            print(f"{lead} {v['name']:6s}  {WALL_NAME[v['wall']]:>3s}  {v['range']:6.2f} {v['perp']:6.2f} {v['off']:5.2f} {v['along']:7.2f} {v['corner']:6.2f}"
                  f"  {'pass' if v['check8'] else 'FAIL':4s}  {v['cone12']:6s} {v['cone18']:6s} {flag}{reason}")
        b, r = views[0], views[1]
        if b["counts"] and long_enough:
            n_back += 1
            walls_back.add(WALL_NAME[b["wall"]])
            if r["counts"]:
                n_both += 1
    print(f"  stops that count: back sensor {n_back} (walls {', '.join(sorted(walls_back))}); back and right {n_both}")
    return n_back, n_both, walls_back


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("route")
    ap.add_argument("--csv", help="a tb_live --route CSV of this route: check the TRUE poses too")
    ap.add_argument("--quiet", action="store_true", help="with --csv: only the summary line")
    ap.add_argument("--stops-out", help="write the planned stop poses (x y heading_deg) to this file")
    a = ap.parse_args()

    start, steps, elements = parse_route(a.route)
    poses, samples = plan(start, steps)
    if elements:
        print("elements (keep-outs):", ", ".join(f"wall {w} ({WALL_NAME[w]}) {lo:g} to {hi:g}, {d:g} in deep" for w, lo, hi, d in elements))
    worst = min(samples, key=lambda p: outline_clearance(p[0], p[1], p[2]))
    clr = outline_clearance(worst[0], worst[1], worst[2])
    print(f"planned path: {len(samples)} samples; smallest outline clearance {clr:.2f} in at ({worst[0]:.1f}, {worst[1]:.1f}, "
          f"{worst[2] % 360:.0f} deg), step {worst[3]}: {'inside the field' if clr >= 0 else 'LEAVES THE FIELD'}")
    hs = [p[2] for p in samples]
    print(f"planned net turning: from {min(hs):.0f} to {max(hs):.0f} deg (unwrapped, start {start[2]:.0f})")
    # the largest single turn (a turn step, or the turn a point/pose move makes toward its target): one near 180
    # degrees could go either way, depending on noise
    prev_h, biggest = start[2], (0.0, 0)
    for (i, s, x, y, h) in poses:
        if s["cmd"] in ("turn", "point", "pose"):
            biggest = max(biggest, (abs(h - prev_h), i))
        prev_h = h
    print(f"largest single planned turn: {biggest[0]:.0f} deg (step {biggest[1]})")

    stops = []
    for (i, s, x, y, h) in poses:
        if s["cmd"] == "wait":
            stops.append((i, s, x, y, h, int(s["nums"][0])))
    if a.stops_out:
        with open(a.stops_out, "w") as f:
            for (i, s, x, y, h, w) in stops:
                f.write(f"{i} {x:.6f} {y:.6f} {h % 360:.6f}\n")
    if not (a.csv and a.quiet):
        stop_table(stops, elements, "planned stops:")

    if a.csv:
        rows = list(csv.DictReader(open(a.csv)))
        worst_row = min(rows, key=lambda r: outline_clearance(float(r["true_x"]), float(r["true_y"]), float(r["true_heading"])))
        clr = outline_clearance(float(worst_row["true_x"]), float(worst_row["true_y"]), float(worst_row["true_heading"]))
        end_t = int(rows[-1]["t_ms"])
        true_stops = []
        for (i, s, x, y, h, w) in stops:
            mine = [r for r in rows if int(r["step"]) == i]
            if not mine:
                continue
            r = mine[-1]
            true_stops.append((i, s, float(r["true_x"]), float(r["true_y"]), float(r["true_heading"]), w))
        if a.quiet:
            nb = nboth = 0
            worst_range = worst_off = 0.0
            worst_corner = 1e9
            walls = set()
            for (i, s, x, y, h, w) in true_stops:
                b = sensor_view(x, y, h, SENSORS[0], elements)
                rr = sensor_view(x, y, h, SENSORS[1], elements)
                if not s["comment"].startswith("B"):
                    continue  # only the stops the route marks as wall stops (B1.., BR1..)
                planned_both = s["comment"].startswith("BR")
                if b["counts"]:
                    nb += 1
                    walls.add(WALL_NAME[b["wall"]])
                    if rr["counts"]:
                        nboth += 1
                worst_range = max(worst_range, b["range"], rr["range"] if planned_both else 0)
                worst_off = max(worst_off, b["off"], rr["off"] if planned_both else 0)
                worst_corner = min(worst_corner, b["corner"], rr["corner"] if planned_both else 1e9)
            print(f"{a.csv}: {end_t / 1000:.1f} s, outline clearance >= {clr:.2f} in, counting stops back {nb} "
                  f"({''.join(sorted(walls))}), both {nboth}; worst planned-sensor range {worst_range:.2f} in, "
                  f"off square {worst_off:.2f} deg, corner {worst_corner:.2f} in")
        else:
            print(f"\ntrue path ({a.csv}): {len(rows)} ticks, {end_t / 1000:.1f} s; smallest outline clearance {clr:.2f} in at "
                  f"t = {worst_row['t_ms']} ms: {'inside the field' if clr >= 0 else 'LEAVES THE FIELD'}")
            stop_table(true_stops, elements, "true poses at the end of each stop:")


if __name__ == "__main__":
    main()
