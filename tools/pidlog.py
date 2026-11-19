#!/usr/bin/env python3
"""pidlog - summarise LemLib PID-tuning logs captured from the V5 serial terminal.

    python3 tools/pidlog.py tune.log [--out DIR] [--last N] [--json] [--no-plots]

Reads a `pros terminal --output tune.log` capture (noise lines, CRLF and ANSI
colour codes are fine), finds every `[PIDLOG] BEGIN ... END` run, checks it for
lost/corrupted lines, computes metrics from the raw rows and prints:

  1. a one-line-per-run table,
  2. a plain-English diagnosis and one suggested gain change per run,
  3. PNG plots (one per run + one overview per mode) if matplotlib is installed.

Everything printed is also written to DIR/report.txt; --json writes DIR/metrics.json.
Suggestions are heuristics: treat them as "try this next", not as answers.
Standard library only, plus matplotlib for the plots.
"""

import argparse
import json
import math
import os
import re
import sys
from statistics import median

# ---------------------------------------------------------------------------
# Thresholds. Every number that decides a verdict lives here so it can be
# adjusted after looking at real data. Distances are in the run's unit (inches
# for DRIVE, degrees for TURN) unless the name says otherwise.
# ---------------------------------------------------------------------------
DEFAULT_TOL = 1.0               # used only if the BEGIN line has no usable small=range/ms
FINAL_WINDOW_MS = 100           # "final error" = mean error over the last this-many ms of the capture
STALL_WINDOW_MS = 150           # robot counts as stalled if speed ~0 over this window before end_ms
STALL_RPM = 5.0                 # mean |motor rpm| below this = "not moving"
END_VOLTAGE_WINDOW_MS = 60      # voltage "at the moment the move ended" = mean over this window
SAT_MV = 11000                  # |u| at or above this counts as voltage saturation
ZC_ARM_FACTOR = 2.0             # count zero crossings only after |err| first comes within 2x tolerance
ZC_DEADBAND_FRAC = 0.2          # a zero crossing must swing beyond +-0.2 x tolerance (ignores sensor jitter)
OSC_MIN_BIG_CROSSINGS = 3       # OSCILLATING: this many crossings with swings beyond +-tolerance
CHATTER_FLIP_MAX_MS = 80        # voltage "chatter": median time between voltage sign flips at or below this...
CHATTER_MIN_FLIPS = 6           # ...with at least this many flips during the move. Chatter alone counts as
                                # OSCILLATING (derivative amplifying noise), even if the error swings are small.
VOLT_FLIP_DEADBAND_MV = 300     # voltage within +-this is treated as zero when counting sign flips
TIMEOUT_MARGIN_MS = 60          # end_ms within this of the timeout = the timeout ended the move
STOPS_SHORT_MIN_MV = 150        # below this the controller wasn't really pushing
BLOCKED_MV = 6000               # stalled with more than this applied = probably physically blocked
SLOW_TAIL_FRACTION = 0.10       # SLOW compares the time to cover the first 90% of the distance...
SLOW_TAIL_RATIO = 0.75          # ...with the last 10% -> tolerance; tail > ratio x head = SLOW
TIMEOUT_SAT_FRAC = 0.6          # timed out while saturated this much = the move is physically too long
LOW_BATTERY_PCT = 50
HEADING_DEV_DEG = 2.0           # DRIVE: max heading deviation worth flagging
LR_MISMATCH_FRAC = 0.05         # DRIVE: persistent left/right speed difference worth flagging
LR_MISMATCH_CONSISTENCY = 0.8   # ...in the same direction in at least this fraction of cruise samples
LR_CRUISE_FRAC = 0.3            # cruise samples = speed above this fraction of peak, before arrival
TURN_WANDER_IN = 1.0            # TURN: centre wandering further than this is worth flagging
END_MISMATCH_FRAC_TOL = 0.5     # END-line final/over disagree if off by more than this x tolerance...
END_MISMATCH_MIN = 0.05         # ...and by at least this much in absolute terms
GAP_FACTOR = 1.5                # a time step above 1.5 x dt_ms is reported as a gap

# Suggested-change sizes.
OSC_NOISE_KD_FACTOR = 0.80      # derivative amplifying noise: kD -20%
OSC_KP_FACTOR = 0.85            # slow oscillation: kP -15%
OVERSHOOT_KD_MIN_PCT = 15       # overshoot just above tolerance: kD +15%
OVERSHOOT_KD_MAX_PCT = 40       # overshoot at OVERSHOOT_FULL_SCALE x tolerance or more: kD +40%
OVERSHOOT_FULL_SCALE = 4.0
KD_START_RATIO = {"DRIVE": 4.0, "TURN": 4.0}  # kD == 0: suggest kD = ratio x kP as a starting point
ENDED_EARLY_KD_FACTOR = 0.85
ENDED_EARLY_KP_FACTOR = 1.10
STOPS_SHORT_KP_FACTOR = 1.12
KI_START_FRACTION = 0.02        # kI starting value = this x kP (LemLib integral = sum of error per 10 ms tick)
SLOW_KD_FACTOR = 0.85
TIMEOUT_KP_FACTOR = 1.15

PLOT_DPI = 130
MAX_OVERVIEW_DEFAULT = 6

# Colour-blind-safe categorical palette (validated order; never cycle it).
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
LINESTYLES = ["-", "--", "-.", ":", (0, (5, 1, 1, 1)), (0, (1, 1))]
C_SURFACE, C_INK, C_INK2, C_MUTED = "#fcfcfb", "#0b0b0b", "#52514e", "#898781"
C_GRID, C_AXIS, C_BAND, C_CRIT = "#e1e0d9", "#c3c2b7", "#dcdad2", "#d03b3b"

MARKER = "[PIDLOG]"
ANSI_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]|\x1b[@-Z\\-_]")
CTRL_RE = re.compile(r"[\x00-\x08\x0b-\x1f\x7f]")
NUM_RE = re.compile(r"^[+-]?\d+$")
ROW_START_RE = re.compile(r"^~\s*[+-]?\d")  # "~" then a number = an attempted data row

VERDICT_ORDER = ["DAMAGED", "ABORTED", "TIMEOUT", "OSCILLATING", "OVERSHOOT",
                 "ENDED_EARLY", "STOPS_SHORT", "SLOW", "GOOD"]


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------
class Run:
    def __init__(self, index, begin, line_no):
        self.index = index          # position in the file (1-based), unique even if run= repeats
        self.begin = begin          # dict of BEGIN key=value
        self.end = None             # dict of END key=value, or None
        self.rows = []              # list of 7-int tuples
        self.malformed = []         # (line_no, text)
        self.line_no = line_no
        self.end_line_no = None
        self.recovered = 0          # rows rescued from lines with other text glued in front
        self.damage = []            # reasons this run is DAMAGED
        self.notes = []             # non-fatal remarks
        self.close_reason = None
        self.gaps = []              # (t_before_gap, step_ms)

    @property
    def run_id(self):
        return self.begin.get("run", "?")

    @property
    def mode(self):
        return str(self.begin.get("mode", "?")).upper()


def parse_kv(tokens):
    out = {}
    for tok in tokens:
        if "=" in tok:
            k, _, v = tok.partition("=")
            if k:
                out[k] = v
    return out


def fnum(d, key, default=None):
    if not d or key not in d:
        return default
    try:
        x = float(d[key])
    except (TypeError, ValueError):
        return default
    return x if math.isfinite(x) else default


def parse_exit(s):
    """'0.50/200' -> (0.5, 200.0); anything unusable -> (None, None)."""
    if not s or "/" not in s:
        return None, None
    a, _, b = s.partition("/")
    try:
        return float(a), float(b)
    except ValueError:
        return None, None


def parse_row(text):
    parts = text.split(",")
    if len(parts) != 7:
        return None
    vals = []
    for p in parts:
        p = p.strip()
        if not NUM_RE.match(p):
            return None
        vals.append(int(p))
    return tuple(vals)


def clean_lines(raw):
    text = raw.decode("utf-8", errors="replace") if isinstance(raw, bytes) else raw
    text = text.replace("\r\n", "\n").replace("\r", "\n")
    for line in text.split("\n"):
        line = ANSI_RE.sub("", line)
        line = CTRL_RE.sub("", line)
        yield line


def parse_log(raw):
    """Return (runs, stats). Never raises on bad content."""
    runs, cur = [], None
    stats = {"lines": 0, "other_lines": 0, "orphan_rows": 0, "orphan_ends": 0}

    def close(reason):
        nonlocal cur
        if cur is not None:
            cur.close_reason = reason
            finalize(cur)
            runs.append(cur)
            cur = None

    for ln, line in enumerate(clean_lines(raw), start=1):
        stats["lines"] += 1
        pos = line.find(MARKER)
        if pos >= 0:
            tokens = line[pos + len(MARKER):].split()
            kind = tokens[0].upper() if tokens else ""
            kv = parse_kv(tokens[1:])
            if kind == "BEGIN":
                if cur is not None:
                    close("a new BEGIN at line %d arrived before this run's END" % ln)
                cur = Run(len(runs) + 1, kv, ln)
            elif kind == "END":
                if cur is None:
                    stats["orphan_ends"] += 1
                    continue
                cur.end = kv
                cur.end_line_no = ln
                close(None)
            else:
                stats["other_lines"] += 1
            continue
        s = line.strip()
        if "~" in s:
            looks_like_row = ROW_START_RE.match(s) is not None
            if cur is None:
                stats["orphan_rows" if looks_like_row else "other_lines"] += 1
                continue
            if looks_like_row:
                row = parse_row(s[1:])
                if row is None:
                    cur.malformed.append((ln, s))
                else:
                    cur.rows.append(row)
            else:
                # Another task printed without a newline and our row got glued on.
                row = parse_row(s.rsplit("~", 1)[1])
                if row is not None:
                    cur.rows.append(row)
                    cur.recovered += 1
                else:
                    stats["other_lines"] += 1
            continue
        if s:
            stats["other_lines"] += 1
    close("the capture ended before this run's END line")
    return runs, stats


def finalize(run):
    h, e = run.begin, run.end
    if h.get("v", "1") != "1":
        run.notes.append("log format version v=%s (tool written for v=1); parsed anyway" % h.get("v"))
    for key in ("mode", "target", "small"):
        if key not in h:
            run.damage.append("BEGIN line has no %s=" % key)
    if run.mode not in ("DRIVE", "TURN") and "mode" in h:
        run.notes.append("unknown mode=%s; analysed like DRIVE" % h.get("mode"))
    if run.malformed:
        ln, txt = run.malformed[0]
        snippet = txt if len(txt) <= 40 else txt[:37] + "..."
        run.damage.append("%d malformed data row(s), first at line %d: '%s'"
                          % (len(run.malformed), ln, snippet))
    if run.recovered:
        run.notes.append("%d row(s) recovered from lines with other output glued in front"
                         % run.recovered)

    # Time gaps: where a dropped row would show up.
    dt = fnum(h, "dt_ms")
    gaps, backwards = [], 0
    for a, b in zip(run.rows, run.rows[1:]):
        step = b[0] - a[0]
        if step < 0:
            backwards += 1
        elif dt and step > GAP_FACTOR * dt:
            gaps.append((a[0], step))
    if backwards:
        run.damage.append("time goes backwards %d time(s) (rows out of order or mixed runs)" % backwards)
    gap_txt = ""
    run.gaps = gaps
    if gaps:
        gap_txt = "; time gaps at " + ", ".join("t=%d ms (+%d)" % g for g in gaps[:4])
        if len(gaps) > 4:
            gap_txt += " and %d more" % (len(gaps) - 4)

    if e is None:
        run.damage.append("no END line (%s)" % (run.close_reason or "unknown"))
    else:
        if "run" in e and "run" in h and e["run"] != h["run"]:
            run.damage.append("END run=%s does not match BEGIN run=%s" % (e["run"], h["run"]))
        n_sent = fnum(e, "rows")
        if n_sent is not None and int(n_sent) != len(run.rows):
            run.damage.append("END says rows=%d but %d intact rows arrived%s"
                              % (int(n_sent), len(run.rows), gap_txt))
            gap_txt = ""
        elif n_sent is None:
            run.notes.append("END has no rows=, row count not checked")
        s_sent = fnum(e, "sum")
        if s_sent is not None:
            got = sum(sum(r) for r in run.rows) % (1 << 32)
            if int(s_sent) % (1 << 32) != got:
                run.damage.append("checksum mismatch (END sum=%d, rows add up to %d): "
                                  "at least one line was corrupted or lost" % (int(s_sent), got))
        else:
            run.notes.append("END has no sum=, checksum not checked")
        res = e.get("result", "")
        if res == "buffer":
            # not damage: every recorded row was sent, the move just outlasted the recording
            run.notes.append("result=buffer: the move was still running when the robot's 5 s recording "
                             "ran out, so the end of it is not in the log")
    if gap_txt:
        run.notes.append(gap_txt.lstrip("; "))
    if not run.rows:
        run.damage.append("no intact data rows")


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------
def _mean(xs):
    xs = list(xs)
    return sum(xs) / len(xs) if xs else None


def count_crossings(vals, times, h):
    """Sign changes of vals that swing beyond +-h. Starts on the + side (the approach)."""
    state, n, at = 1, 0, []
    for v, t in zip(vals, times):
        s = 1 if v > h else (-1 if v < -h else 0)
        if s and s != state:
            n += 1
            at.append(t)
        if s:
            state = s
    return n, at


def analyse(run):
    h, e = run.begin, run.end or {}
    mode = run.mode if run.mode in ("DRIVE", "TURN") else "DRIVE"
    unit = h.get("unit") or ("in" if mode == "DRIVE" else "deg")
    target = fnum(h, "target", 0.0)
    tol, tol_ms = parse_exit(h.get("small"))
    large, large_ms = parse_exit(h.get("large"))
    if not tol or tol <= 0:
        tol = DEFAULT_TOL
        run.notes.append("no usable small= exit; tolerance assumed %.2f" % tol)
    timeout = fnum(h, "timeout")
    ips = fnum(h, "ips_per_rpm")
    track = fnum(h, "track")
    end_ms = fnum(e, "end_ms") if run.end else None
    m = {
        "index": run.index, "run": run.run_id, "mode": mode, "unit": unit, "target": target,
        "kP": fnum(h, "kP"), "kI": fnum(h, "kI"), "kD": fnum(h, "kD"),
        "windup": fnum(h, "windup"), "slew": fnum(h, "slew"),
        "tol": tol, "tol_ms": tol_ms, "large": large, "large_ms": large_ms, "timeout": timeout,
        "batt_pct": fnum(h, "batt_pct"), "batt_mV": fnum(h, "batt_mV"),
        "result": e.get("result") if run.end else None, "end_ms": end_ms,
        "rows": len(run.rows), "damaged": bool(run.damage),
    }
    rows = run.rows
    if not rows:
        return m
    t = [r[0] for r in rows]
    err = [r[1] / 100.0 for r in rows]
    dev = [r[2] / 100.0 for r in rows]
    vL = [r[3] for r in rows]
    vR = [r[4] for r in rows]
    uL = [r[5] for r in rows]
    uR = [r[6] for r in rows]
    sgn = -1 if target < 0 else 1
    if mode == "TURN":
        ucmd = [(a - b) / 2.0 * sgn for a, b in zip(uL, uR)]
    else:
        ucmd = [(a + b) / 2.0 * sgn for a, b in zip(uL, uR)]
    vabs = [(abs(a) + abs(b)) / 2.0 for a, b in zip(vL, vR)]
    dt = fnum(h, "dt_ms") or (median([b - a for a, b in zip(t, t[1:])]) if len(t) > 1 else 20)
    m["dt_ms"] = dt

    if end_ms is not None and end_ms >= 0:
        move_end = end_ms
    else:
        # never ended (or no END): the move lasted as long as the controller was commanding
        cmd_idx = [i for i in range(len(t)) if abs(ucmd[i]) > VOLT_FLIP_DEADBAND_MV]
        move_end = t[cmd_idx[-1]] if cmd_idx else t[-1]
    move = [i for i in range(len(t)) if t[i] <= move_end] or list(range(len(t)))
    i_end = move[-1]
    m["move_end_ms"] = move_end

    fin = [err[i] for i in range(len(t)) if t[i] >= t[-1] - FINAL_WINDOW_MS]
    m["final"] = _mean(fin)
    m["err_at_end"] = err[i_end]
    m["overshoot"] = max(0.0, -min(err))

    rise_i = next((i for i in range(len(t)) if abs(err[i]) <= tol), None)
    m["rise_ms"] = t[rise_i] if rise_i is not None else None
    last_out = max((i for i in range(len(t)) if abs(err[i]) > tol), default=None)
    if last_out is None:
        m["settle_ms"] = t[0]
    elif last_out == len(t) - 1:
        m["settle_ms"] = None
    else:
        m["settle_ms"] = t[last_out + 1]
    t90_i = next((i for i in range(len(t)) if abs(err[i]) <= SLOW_TAIL_FRACTION * abs(target)), None)
    m["t90_ms"] = t[t90_i] if t90_i is not None else None

    arm = next((i for i in range(len(t)) if abs(err[i]) <= ZC_ARM_FACTOR * tol), None)
    if arm is not None:
        zc, _ = count_crossings(err[arm:], t[arm:], ZC_DEADBAND_FRAC * tol)
        bc, bat = count_crossings(err[arm:], t[arm:], tol)
    else:
        zc, bc, bat = 0, 0, []
    m["zero_crossings"], m["big_crossings"] = zc, bc
    m["half_period_ms"] = median([b - a for a, b in zip(bat, bat[1:])]) if len(bat) >= 2 else None

    # Voltage sign flips during the move (derivative noise shows up as chatter).
    flips_at, last_s = [], 0
    for k in move:
        u = ucmd[k]
        s = 1 if u > VOLT_FLIP_DEADBAND_MV else (-1 if u < -VOLT_FLIP_DEADBAND_MV else 0)
        if s and last_s and s != last_s:
            flips_at.append(k)
        if s:
            last_s = s
    m["volt_flips"] = len(flips_at)
    m["volt_flip_median_ms"] = (median([t[b] - t[a] for a, b in zip(flips_at, flips_at[1:])])
                                if len(flips_at) >= 2 else None)
    m["chatter"] = (m["volt_flips"] >= CHATTER_MIN_FLIPS and m["volt_flip_median_ms"] is not None
                    and m["volt_flip_median_ms"] <= CHATTER_FLIP_MAX_MS)

    if mode == "TURN":
        if ips and track:
            m["peak_speed"] = max(abs(vL[i] - vR[i]) for i in move) * ips / track * 180.0 / math.pi
            m["speed_unit"] = "deg/s"
        else:
            m["peak_speed"], m["speed_unit"] = max(vabs[i] for i in move), "rpm"
    else:
        if ips:
            m["peak_speed"], m["speed_unit"] = max(vabs[i] for i in move) * ips, "in/s"
        else:
            m["peak_speed"], m["speed_unit"] = max(vabs[i] for i in move), "rpm"

    pre = [i for i in move if rise_i is None or i < rise_i] or move
    m["sat_frac"] = sum(1 for i in pre if max(abs(uL[i]), abs(uR[i])) >= SAT_MV) / len(pre)

    win = [i for i in move if t[i] >= move_end - STALL_WINDOW_MS]
    m["speed_end_rpm"] = _mean(vabs[i] for i in win)
    m["stalled_end"] = len(win) >= 2 and m["speed_end_rpm"] < STALL_RPM
    k = i_end
    while k > 0 and vabs[k] < STALL_RPM:
        k -= 1
    m["stopped_for_ms"] = move_end - t[k] if vabs[i_end] < STALL_RPM else 0
    vw = [i for i in move if t[i] >= move_end - END_VOLTAGE_WINDOW_MS]
    m["u_end_mV"] = _mean(abs(ucmd[i]) for i in vw)
    m["u_at_end_mV"] = ucmd[i_end]

    timed_out = None
    if run.end is not None and end_ms is not None:
        timed_out = end_ms < 0 or (timeout is not None and end_ms >= timeout - TIMEOUT_MARGIN_MS)
    m["timed_out"] = timed_out

    if mode == "DRIVE":
        m["heading_dev_max"] = max(abs(x) for x in dev)
        peak_v = max(vabs[i] for i in move)
        # cruise = the first half of the distance at decent speed (before braking starts)
        cruise = [i for i in pre if vabs[i] > LR_CRUISE_FRAC * peak_v and vabs[i] > 0
                  and abs(err[i]) > 0.5 * abs(target)]
        if len(cruise) >= 5:
            rel = [(abs(vL[i]) - abs(vR[i])) / vabs[i] for i in cruise]
            med = median(rel)
            same = sum(1 for r in rel if (r > 0) == (med > 0) and r != 0) / len(rel)
            m["lr_mismatch"] = med
            m["lr_mismatch_persistent"] = abs(med) > LR_MISMATCH_FRAC and same >= LR_MISMATCH_CONSISTENCY
        else:
            m["lr_mismatch"], m["lr_mismatch_persistent"] = None, False
    else:
        m["wander_max"] = max(abs(x) for x in dev)

    # Compare against what the firmware claimed.
    mism = []
    lim = max(END_MISMATCH_FRAC_TOL * tol, END_MISMATCH_MIN)
    ef = fnum(e, "final")
    if ef is not None and min(abs(ef - m["err_at_end"]), abs(ef - m["final"])) > lim:
        mism.append("final=%.2f but rows show %.2f at end_ms / %.2f at rest" % (ef, m["err_at_end"], m["final"]))
    eo = fnum(e, "over")
    if eo is not None and abs(eo - m["overshoot"]) > lim:
        mism.append("over=%.2f but rows show %.2f" % (eo, m["overshoot"]))
    # END's settle_ms is not compared: the brain's on-screen readout uses its own, wider tolerance
    # (1 in / 1.5 deg), so it legitimately differs from the settle time computed here
    m["end_line_mismatch"] = mism
    m["_series"] = {"t": t, "err": err, "dev": dev, "vL": vL, "vR": vR, "uL": uL, "uR": uR}
    return m


# ---------------------------------------------------------------------------
# Verdicts and advice
# ---------------------------------------------------------------------------
def nice(x):
    if not x:
        return 0.0
    mag = 10 ** (math.floor(math.log10(abs(x))) - 2)
    return round(round(x / mag) * mag, 6)


def fmt(x, nd=2):
    if x is None:
        return "-"
    s = ("%." + str(nd) + "f") % x
    return s


def gnum(x):
    """Gain formatting: up to 4 decimals, no trailing zeros."""
    if x is None:
        return "?"
    s = "%.4f" % x
    s = s.rstrip("0").rstrip(".")
    return s if s not in ("-0", "") else "0"


def change(param, old, factor, why):
    new = nice(old * factor)
    if new == old:
        new = old * factor
    pct = (new / old - 1) * 100 if old else 0
    return {"param": param, "old": old, "new": new, "pct": round(pct),
            "text": "%s %s → %s (%+d%%) %s" % (param, gnum(old), gnum(new), round(pct), why)}


def setval(param, old, new, why):
    new = nice(new)
    return {"param": param, "old": old, "new": new, "pct": None,
            "text": "%s %s → %s %s" % (param, gnum(old), gnum(new), why)}


def classify(run, m):
    """Return dict(verdict, diagnosis, suggestion, alt, flags)."""
    u = m["unit"]
    tol = m["tol"]
    kP, kI, kD = m["kP"] or 0.0, m["kI"] or 0.0, m["kD"] or 0.0
    out = {"verdict": None, "diagnosis": "", "suggestion": None, "alt": None, "flags": []}
    flags = out["flags"]

    if m.get("batt_pct") is not None and m["batt_pct"] < LOW_BATTERY_PCT:
        flags.append("battery %d%%: results will shift on a full battery" % m["batt_pct"])
    if m["rows"]:
        if m["mode"] == "DRIVE":
            if m["heading_dev_max"] > HEADING_DEV_DEG:
                flags.append("heading drifted up to %.1f deg while driving straight: mechanical drift "
                             "(or heading correction), not a lateral-gain problem" % m["heading_dev_max"])
            if m.get("lr_mismatch_persistent"):
                side = "left" if m["lr_mismatch"] > 0 else "right"
                flags.append("%s side ran %.0f%% faster than the other through the cruise: check friction, "
                             "motors and wiring on the drive, not the gains" % (side, abs(m["lr_mismatch"]) * 100))
        elif m.get("wander_max", 0) > TURN_WANDER_IN:
            flags.append("centre wandered %.1f in during the turn: wheels scrubbing or a drive side "
                         "slipping, not an angular-gain problem" % m["wander_max"])
        for s in m.get("end_line_mismatch", []):
            flags.append("END line disagrees with the rows: " + s)
    for n in run.notes:
        flags.append("note: " + n)

    def done(verdict, diag, sug=None, alt=None):
        out.update(verdict=verdict, diagnosis=diag, suggestion=sug, alt=alt)
        return out

    if run.damage:
        what = "; ".join(run.damage)
        partial = ("Metrics below come from the %d intact rows and may be wrong." % m["rows"]
                   if m["rows"] else "Nothing to analyse.")
        return done("DAMAGED", "Log damaged: %s. %s Re-run this move before tuning from it." % (what, partial))
    res = m.get("result") or ""
    if res.startswith("aborted"):
        why = {"aborted:stick": "driver stick input", "aborted:stop": "a stop request"}.get(res, res)
        return done("ABORTED", "Move was aborted by %s at %s ms; no tuning advice from this run."
                    % (why, gnum(m["end_ms"])))
    if not m["rows"]:
        return done("DAMAGED", "No rows to analyse.")

    final, eae = m["final"], m["err_at_end"]
    over = m["overshoot"]
    short = abs(eae) > tol and abs(final) > tol
    osc_fast = m["chatter"]
    oscillating = m["big_crossings"] >= OSC_MIN_BIG_CROSSINGS or osc_fast

    def osc_advice(prefix=""):
        if osc_fast:
            sw = ("%d crossing(s) beyond tolerance" % m["big_crossings"]) if m["big_crossings"] else \
                "error swings mostly inside tolerance"
            d = (prefix + "Voltage chatter: it flipped sign %d times, every ~%d ms (%s); the derivative term is "
                 "amplifying sensor noise and the motors are fighting themselves."
                 % (m["volt_flips"], m["volt_flip_median_ms"], sw))
            return d, change("kD", kD, OSC_NOISE_KD_FACTOR, "to stop the derivative chasing noise"), None
        hp = (" about %d ms per swing" % m["half_period_ms"]) if m["half_period_ms"] else ""
        d = (prefix + "Oscillates around the target (%d crossings beyond the %.2f %s tolerance,%s): "
             "too much proportional gain for the damping available." % (m["big_crossings"], tol, u, hp))
        return d, change("kP", kP, OSC_KP_FACTOR, "to calm the oscillation"), \
            "or raise kD instead if the approach speed is already where you want it"

    def short_advice(prefix=""):
        ue = m["u_end_mV"] or 0
        if ue > BLOCKED_MV:
            d = (prefix + "Stopped %.2f %s short while still applying %.1f V: the robot is probably blocked "
                 "or touching something. Check mechanically before changing gains." % (abs(final), u, ue / 1000))
            return d, None, None
        d = (prefix + "Stopped %.2f %s short of the target and stayed there with only %.2f V applied: "
             "not enough push to beat friction near the end." % (abs(final), u, ue / 1000))
        sug = change("kP", kP, STOPS_SHORT_KP_FACTOR, "for more push near the target")
        alt = None
        w = m["windup"] or 0
        if kI == 0 and w > abs(final) and kP:
            alt = setval("kI", 0.0, KI_START_FRACTION * kP,
                         "(starting value; integral accumulates inside windup=%s)" % gnum(w))["text"]
            alt = "or add a small integral: " + alt
        elif kI == 0 and w > 0 and kP:
            alt = ("kI alone would not help: it stopped %.2f %s away, outside windup=%s where the integral "
                   "accumulates; fix kP first" % (abs(final), u, gnum(w)))
        elif kI == 0:
            alt = "kI would not help yet: windup=0 means the integral never accumulates"
        return d, sug, alt

    # TIMEOUT
    if m["timed_out"]:
        when = "never ended" if (m["end_ms"] is not None and m["end_ms"] < 0) else \
            "hit the %s ms timeout" % gnum(m["timeout"])
        pre = "The move %s. " % when
        if oscillating:
            d, s, a = osc_advice(pre)
            return done("TIMEOUT", d, s, a)
        if m["stalled_end"] and short:
            d, s, a = short_advice(pre)
            return done("TIMEOUT", d, s, a)
        if not m["stalled_end"]:
            if m["sat_frac"] >= TIMEOUT_SAT_FRAC and m["timeout"]:
                est = m["timeout"] * max(1.5, abs(m["target"]) / max(abs(m["target"]) - abs(eae), 1e-6))
                d = (pre + "It was still moving (%.2f %s to go) after spending %.0f%% of the move at full voltage: "
                     "the move is physically longer than the timeout allows." % (abs(eae), u, m["sat_frac"] * 100))
                return done("TIMEOUT", d, setval("timeout", m["timeout"], math.ceil(est / 100) * 100,
                                                  "ms (or use a shorter target)"), None)
            d = (pre + "It was still creeping toward the target (%.2f %s to go) without much power: "
                 "too slow near the end." % (abs(eae), u))
            return done("TIMEOUT", d, change("kP", kP, TIMEOUT_KP_FACTOR, "for a quicker approach"),
                        "or lower kD ~15% if the robot slows down long before the target")
        d = (pre + "The robot was within tolerance and stopped, but the exit conditions never completed "
             "before the timeout.")
        return done("TIMEOUT", d, None, "check small=%s/%s against the timeout" % (gnum(tol), gnum(m["tol_ms"])))

    if oscillating:
        d, s, a = osc_advice()
        return done("OSCILLATING", d, s, a)

    if over > tol:
        ratio = over / tol
        if kD == 0:
            start = KD_START_RATIO.get(m["mode"], 4.0) * kP
            s = setval("kD", 0.0, start, "(starting value, about %g x kP)" % KD_START_RATIO.get(m["mode"], 4.0))
        else:
            frac = min(1.0, max(0.0, (ratio - 1) / (OVERSHOOT_FULL_SCALE - 1)))
            pct = OVERSHOOT_KD_MIN_PCT + frac * (OVERSHOOT_KD_MAX_PCT - OVERSHOOT_KD_MIN_PCT)
            s = change("kD", kD, 1 + pct / 100.0, "to brake harder on the approach")
        if m["settle_ms"] is not None:
            st = "came back and settled at %.2f s" % (m["settle_ms"] / 1000)
        elif final < -tol:
            st = "stayed %.2f %s past it" % (-final, u)
        else:
            st = "came back but ended %.2f %s short" % (final, u)
        d = ("Overshot the target by %.2f %s (%.1fx the tolerance), %s; %d crossing(s) beyond tolerance."
             % (over, u, ratio, st, m["big_crossings"]))
        return done("OVERSHOOT", d, s, None)

    if short and not m["stalled_end"]:
        d = ("LemLib ended the move %.2f %s short while the robot was still creeping (%.0f rpm): the "
             "large=%s/%s exit fired before it reached the %.2f %s tolerance. The approach is too slow at the end."
             % (abs(eae), u, m["speed_end_rpm"], gnum(m["large"]), gnum(m["large_ms"]), tol, u))
        if kD:
            s = change("kD", kD, ENDED_EARLY_KD_FACTOR, "to approach faster")
            a = "kP %s → %s (+10%%); or make the large exit stricter (smaller range or longer time than %s/%s)" % (
                gnum(kP), gnum(nice(kP * ENDED_EARLY_KP_FACTOR)), gnum(m["large"]), gnum(m["large_ms"]))
        else:
            s = change("kP", kP, ENDED_EARLY_KP_FACTOR, "to approach faster")
            a = "make the large exit stricter (smaller range or longer time than %s/%s)" % (
                gnum(m["large"]), gnum(m["large_ms"]))
        return done("ENDED_EARLY", d, s, a)

    if short and m["stalled_end"]:
        d, s, a = short_advice()
        return done("STOPS_SHORT", d, s, a)

    if (m["t90_ms"] is not None and m["rise_ms"] is not None and tol < SLOW_TAIL_FRACTION * abs(m["target"])
            and m["t90_ms"] > 0):
        head = m["t90_ms"]
        tail = m["rise_ms"] - m["t90_ms"]
        if tail > SLOW_TAIL_RATIO * head:
            d = ("No overshoot, but the last %d%% of the distance took %.2f s versus %.2f s for the first %d%%: "
                 "overdamped, it brakes too early." % (round(SLOW_TAIL_FRACTION * 100), tail / 1000, head / 1000,
                                                      round(100 - SLOW_TAIL_FRACTION * 100)))
            return done("SLOW", d, change("kD", kD, SLOW_KD_FACTOR, "to brake later") if kD else
                        change("kP", kP, TIMEOUT_KP_FACTOR, "for a quicker approach"), None)

    tgt = abs(m["target"])
    shorter, longer = nice(tgt / 2), nice(min(tgt * 2, 180.0) if m["mode"] == "TURN" else tgt * 2)
    st = ("settled in %.2f s" % (m["settle_ms"] / 1000)) if m["settle_ms"] is not None else "ended"
    d = ("Reached the target with %.2f %s final error, %.2f %s overshoot, %s. Looks well tuned for this distance."
         % (abs(final), u, over, u, st))
    if longer > tgt:
        txt = "keep these gains; confirm at a shorter (%s %s) and a longer (%s %s) target" % (
            gnum(shorter), u, gnum(longer), u)
    else:
        txt = "keep these gains; confirm at shorter targets (%s and %s %s)" % (gnum(shorter), gnum(nice(tgt / 4)), u)
    s = {"param": None, "old": None, "new": None, "pct": None, "text": txt}
    return done("GOOD", d, s, None)


# ---------------------------------------------------------------------------
# Text output
# ---------------------------------------------------------------------------
def end_str(m):
    if m.get("result") is None:
        return "no END"
    r = m["result"]
    if m.get("end_ms") is not None and m["end_ms"] < 0:
        return "%s/never ended" % r
    if m.get("timed_out"):
        return "%s/timeout" % r
    return "%s@%ss" % (r, fmt(m["end_ms"] / 1000, 2)) if m.get("end_ms") is not None and m["end_ms"] >= 0 else r


def table(results):
    hdr = ["run", "mode", "target", "kP/kI/kD", "batt", "final", "over", "settle", "rise",
           "peak", "sat", "zx", "ended", "verdict"]
    lines = []
    for run, m, v in results:
        unit = "in" if m["mode"] == "DRIVE" else "deg"
        rid = str(m["run"]) + ("*" if m["damaged"] else "")
        has = m["rows"] > 0
        peak = ("%.0f%s" % (m["peak_speed"], m["speed_unit"])) if has else "-"
        row = [rid, m["mode"], "%s%s" % (gnum(m["target"]), unit),
               "%s/%s/%s" % (gnum(m["kP"]), gnum(m["kI"]), gnum(m["kD"])),
               ("%d%%" % m["batt_pct"]) if m["batt_pct"] is not None else "-",
               fmt(m.get("final")) if has else "-", fmt(m.get("overshoot")) if has else "-",
               (fmt(m["settle_ms"] / 1000) + "s") if has and m["settle_ms"] is not None else ("never" if has else "-"),
               (fmt(m["rise_ms"] / 1000) + "s") if has and m["rise_ms"] is not None else ("never" if has else "-"),
               peak, ("%d%%" % round(m["sat_frac"] * 100)) if has else "-",
               ("%d/%d" % (m["zero_crossings"], m["big_crossings"])) if has else "-",
               end_str(m), v["verdict"]]
        lines.append(row)
    w = [max(len(hdr[i]), *(len(r[i]) for r in lines)) if lines else len(hdr[i]) for i in range(len(hdr))]
    right = {2, 4, 5, 6, 7, 8, 9, 10, 11}
    def f(r):
        return "  ".join(c.rjust(w[i]) if i in right else c.ljust(w[i]) for i, c in enumerate(r)).rstrip()
    out = [f(hdr), "  ".join("-" * x for x in w)]
    out += [f(r) for r in lines]
    out.append("final/over in the run's unit (in or deg); final = mean of the last %d ms of the capture; "
               "zx = zero crossings / those swinging beyond tolerance; sat = share of the approach at |u| >= %.0f V; "
               "* = damaged log, metrics from the intact rows only." % (FINAL_WINDOW_MS, SAT_MV / 1000))
    return out


def details(run, m, v, plot_path):
    unit = "in" if m["mode"] == "DRIVE" else "deg"
    head = "run %s (#%d in file, line %d)  %s %s %s  kP %s kI %s kD %s  -> %s" % (
        m["run"], m["index"], run.line_no, m["mode"], gnum(m["target"]), unit,
        gnum(m["kP"]), gnum(m["kI"]), gnum(m["kD"]), v["verdict"])
    out = [head, "  " + v["diagnosis"]]
    if v["suggestion"]:
        lab = "Next" if v["verdict"] == "GOOD" else "Suggestion"
        out.append("  %s: %s." % (lab, v["suggestion"]["text"].strip()))
    if v["alt"]:
        out.append("  Alternative: %s." % v["alt"])
    for fl in v["flags"]:
        out.append("  Flag: " + fl)
    if plot_path:
        out.append("  Plot: " + plot_path)
    return out


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------
def style_axes(ax):
    ax.set_facecolor(C_SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(C_AXIS)
    ax.tick_params(colors=C_INK2, labelsize=9)
    ax.grid(True, color=C_GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    ax.yaxis.label.set_color(C_INK2)
    ax.xaxis.label.set_color(C_INK2)


def box_empty(ax, xs, ys, box, pad=0.02):
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    bx, by, bw, bh = box
    for x, y in zip(xs, ys):
        ax_x = (x - x0) / (x1 - x0)
        ax_y = (y - y0) / (y1 - y0)
        if bx - pad <= ax_x <= bx + bw + pad and by - pad <= ax_y <= by + bh + pad:
            return False
    return True


def zoom_range(m):
    tol = m["tol"]
    z = max(4 * tol, 1.3 * m.get("overshoot", 0), 1.5 * abs(m.get("final", 0) or 0),
            1.3 * abs(m.get("err_at_end", 0) or 0))
    if m.get("large") and m["large"] <= 1.6 * z:
        z = max(z, 1.15 * m["large"])
    return z


def draw_error(ax, m, ts, err, lw=1.8, show_large=True):
    tol = m["tol"]
    ax.axhspan(-tol, tol, color=C_BAND, alpha=0.7, lw=0, zorder=0)
    ax.axhline(0, color=C_INK2, lw=0.8, zorder=1)
    if show_large and m.get("large"):
        for s in (1, -1):
            ax.axhline(s * m["large"], color=C_MUTED, lw=0.8, ls=(0, (4, 3)), zorder=1)
    ax.plot(ts, err, color=SERIES[0], lw=lw, zorder=3)


def plot_run(plt, run, m, v, path):
    s = m["_series"]
    ts = [x / 1000 for x in s["t"]]
    unit_lbl = "in" if m["mode"] == "DRIVE" else "deg"
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10.5, 8.6),
                             gridspec_kw={"height_ratios": [1.5, 1, 1], "hspace": 0.16})
    fig.patch.set_facecolor(C_SURFACE)
    for ax in axes:
        style_axes(ax)
    ax_e, ax_v, ax_u = axes
    end_s = m["end_ms"] / 1000 if m.get("end_ms") is not None and m["end_ms"] >= 0 else None

    draw_error(ax_e, m, ts, s["err"])
    ax_e.set_ylabel("error (%s)\n+ = short of target" % unit_lbl)
    for ax in axes:
        if end_s is not None:
            ax.axvline(end_s, color=C_INK, lw=1.0, ls="--", zorder=2)
            ax.axvspan(end_s, ts[-1], color=C_GRID, alpha=0.35, lw=0, zorder=0)
    hL, = ax_v.plot(ts, s["vL"], color=SERIES[0], lw=1.5, label="left")
    hR, = ax_v.plot(ts, s["vR"], color=SERIES[1], lw=1.5, ls="--", label="right")
    ax_v.axhline(0, color=C_INK2, lw=0.8)
    for g0, step in run.gaps:
        for ax in axes:
            ax.axvspan(g0 / 1000, (g0 + step) / 1000, color=C_CRIT, alpha=0.25, lw=0)
    if run.gaps:
        ax_v.annotate("red = missing rows", xy=(run.gaps[0][0] / 1000, 1.0), xycoords=("data", "axes fraction"),
                      xytext=(4, -10), textcoords="offset points", fontsize=8, color=C_CRIT)
    ax_v.set_ylabel("motor speed (rpm)")
    ax_u.plot(ts, [x / 1000 for x in s["uL"]], color=SERIES[0], lw=1.5, label="left")
    ax_u.plot(ts, [x / 1000 for x in s["uR"]], color=SERIES[1], lw=1.5, ls="--", label="right")
    ax_u.axhline(0, color=C_INK2, lw=0.8)
    for sgn in (1, -1):
        ax_u.axhline(sgn * SAT_MV / 1000, color=C_MUTED, lw=0.8, ls=":")
    ax_u.set_ylim(-13, 13)
    ax_u.set_yticks([-12, -6, 0, 6, 12])
    ax_u.set_ylabel("motor voltage (V)")
    ax_u.set_xlabel("time since move start (s)")
    fig.legend([hL, hR], ["left side", "right side"], loc="upper right", bbox_to_anchor=(0.985, 0.95),
               ncol=2, frameon=False, fontsize=9, handlelength=2.5)
    ax_e.set_xlim(ts[0], ts[-1] if ts[-1] > ts[0] else ts[0] + 1)

    # Key labels directly on the error panel.
    if end_s is not None:
        lab = "LemLib ended the move at %.2f s" % end_s
        right_room = (ts[-1] - end_s) / max(ts[-1] - ts[0], 1e-6)
        ax_u.annotate(lab, xy=(end_s, 0.04), xycoords=("data", "axes fraction"),
                      xytext=(-5 if right_room < 0.3 else 5, 0), textcoords="offset points",
                      ha="right" if right_room < 0.3 else "left", fontsize=8.5, color=C_INK,
                      bbox=dict(boxstyle="round,pad=0.2", fc=C_SURFACE, ec="none", alpha=0.9))
    elif m.get("result") is not None:
        ax_u.annotate("LemLib never ended the move (end_ms=-1)", xy=(0.99, 0.04), xycoords="axes fraction",
                      ha="right", fontsize=8.5, color=C_INK,
                      bbox=dict(boxstyle="round,pad=0.2", fc=C_SURFACE, ec="none", alpha=0.9))
    ax_u.annotate("dotted = ±%.0f V saturation" % (SAT_MV / 1000), xy=(1.0, 1.0), xycoords="axes fraction",
                  xytext=(0, 2), textcoords="offset points", ha="right", va="bottom", fontsize=8, color=C_MUTED)

    # Zoomed inset of the end-game, placed where it hides no data.
    z = zoom_range(m)
    ylo, yhi = ax_e.get_ylim()
    placed = None
    if yhi - ylo > 3 * z:
        for box in ([0.47, 0.36, 0.51, 0.52], [0.30, 0.36, 0.45, 0.52], [0.47, 0.05, 0.51, 0.45]):
            if box_empty(ax_e, ts, s["err"], box):
                placed = box
                break
        if placed:
            ins = ax_e.inset_axes(placed)
            style_axes(ins)
            ins.patch.set_alpha(0.97)
            for sp in ins.spines.values():
                sp.set_visible(True)
                sp.set_color(C_AXIS)
            draw_error(ins, m, ts, s["err"], lw=1.6)
            start = next((ts[i] for i, e in enumerate(s["err"]) if abs(e) <= 1.4 * z), ts[0])
            ins.set_xlim(max(ts[0], start - 0.05), ts[-1])
            ins.set_ylim(-z, z)
            if end_s is not None:
                ins.axvline(end_s, color=C_INK, lw=1.0, ls="--")
            ins.tick_params(labelsize=7.5)
            ins.set_title("zoom: ±%.2f %s   grey band = tolerance ±%s%s" % (
                z, unit_lbl, gnum(m["tol"]),
                ("   dashed = large exit ±%s" % gnum(m["large"])) if m.get("large") and m["large"] <= z else ""),
                fontsize=8, color=C_INK2, loc="left")
            ax_e.indicate_inset_zoom(ins, edgecolor=C_MUTED, alpha=0.6)
    if not (yhi - ylo > 3 * z and placed):
        ax_e.text(0.995, 0.03, "grey band = tolerance ±%s %s" % (gnum(m["tol"]), unit_lbl),
                  transform=ax_e.transAxes, ha="right", fontsize=8, color=C_INK2)

    # Title: what was run, how it went.
    batt = ""
    if m.get("batt_pct") is not None:
        batt = "battery %d%%" % m["batt_pct"]
        if m.get("batt_mV"):
            batt += " (%.2f V)" % (m["batt_mV"] / 1000)
    l1 = "Run %s  ·  %s %s %s  ·  kP %s  kI %s  kD %s  ·  %s" % (
        m["run"], m["mode"], gnum(m["target"]), unit_lbl, gnum(m["kP"]), gnum(m["kI"]), gnum(m["kD"]), batt)
    sett = ("%.2f s" % (m["settle_ms"] / 1000)) if m["settle_ms"] is not None else "never"
    rise = ("%.2f s" % (m["rise_ms"] / 1000)) if m["rise_ms"] is not None else "never"
    l2 = "final %.2f %s  ·  overshoot %.2f %s  ·  rise %s  ·  settle %s  ·  peak %.0f %s  ·  %.0f%% saturated" % (
        m["final"], unit_lbl, m["overshoot"], unit_lbl, rise, sett, m["peak_speed"], m["speed_unit"], m["sat_frac"] * 100)
    fig.text(0.07, 0.975, l1, fontsize=12, color=C_INK, weight="bold", va="top")
    fig.text(0.07, 0.945, l2, fontsize=9.5, color=C_INK2, va="top")
    badge_c = C_CRIT if v["verdict"] in ("DAMAGED", "ABORTED") else C_INK
    fig.text(0.985, 0.975, v["verdict"], fontsize=12, color=badge_c, weight="bold", va="top", ha="right")
    if v["verdict"] == "DAMAGED":
        reason = "; ".join(run.damage)
        if len(reason) > 120:
            reason = reason[:117] + "..."
        fig.text(0.07, 0.92, "DAMAGED LOG, partial data: " + reason, fontsize=9, color=C_CRIT, va="top")
    fig.subplots_adjust(left=0.09, right=0.985, top=0.895, bottom=0.07)
    fig.savefig(path, dpi=PLOT_DPI, facecolor=C_SURFACE)
    plt.close(fig)


def plot_overview(plt, mode, items, path):
    unit_lbl = "in" if mode == "DRIVE" else "deg"
    fig, (ax_f, ax_z) = plt.subplots(2, 1, sharex=True, figsize=(10.5, 7.6),
                                     gridspec_kw={"height_ratios": [1, 1.2], "hspace": 0.1})
    fig.patch.set_facecolor(C_SURFACE)
    style_axes(ax_f)
    style_axes(ax_z)
    tol = max(m["tol"] for _, m, _ in items)
    z = max(zoom_range(m) for _, m, _ in items)
    z = min(z, max(6 * tol, 1.3 * max(m["overshoot"] for _, m, _ in items)))
    tmax = 0
    for ax in (ax_f, ax_z):
        ax.axhspan(-tol, tol, color=C_BAND, alpha=0.7, lw=0, zorder=0)
        ax.axhline(0, color=C_INK2, lw=0.8)
    for k, (run, m, v) in enumerate(items):
        s = m["_series"]
        ts = [x / 1000 for x in s["t"]]
        tmax = max(tmax, ts[-1])
        dmg = m["damaged"]
        lab = "run %s: kP %s  kI %s  kD %s  ·  %s %s  ·  %s" % (
            m["run"], gnum(m["kP"]), gnum(m["kI"]), gnum(m["kD"]), gnum(m["target"]), unit_lbl,
            v["verdict"].lower() + (" (partial)" if dmg else ""))
        kw = dict(color=SERIES[k % len(SERIES)], ls=LINESTYLES[k % len(LINESTYLES)],
                  lw=1.2 if dmg else 1.8, alpha=0.6 if dmg else 1.0)
        ax_f.plot(ts, s["err"], label=lab, **kw)
        ax_z.plot(ts, s["err"], **kw)
        if m.get("end_ms") is not None and m["end_ms"] >= 0:
            i = max(j for j, t in enumerate(s["t"]) if t <= m["end_ms"]) if s["t"][0] <= m["end_ms"] else 0
            if abs(s["err"][i]) <= z:   # off-scale end points are visible in the top panel instead
                ax_z.plot([ts[i]], [s["err"][i]], marker="o", ms=7, color=kw["color"],
                          mec=C_SURFACE, mew=1.5, ls="none", zorder=5)
            else:
                ax_f.plot([ts[i]], [s["err"][i]], marker="o", ms=7, color=kw["color"],
                          mec=C_SURFACE, mew=1.5, ls="none", zorder=5)
    ax_z.set_ylim(-z, z)
    ax_f.set_xlim(0, tmax)
    ax_f.set_ylabel("error (%s)" % unit_lbl)
    ax_z.set_ylabel("error near target (%s)" % unit_lbl)
    ax_z.set_xlabel("time since move start (s)")
    ax_f.legend(loc="upper right", frameon=False, fontsize=8.5, title="oldest → newest",
                title_fontsize=8.5)
    ax_z.text(0.995, 0.03, "grey band = tolerance ±%s %s   ● = where LemLib ended the move" % (gnum(tol), unit_lbl),
              transform=ax_z.transAxes, ha="right", fontsize=8, color=C_INK2)
    fig.text(0.08, 0.975, "%s: error of the last %d run(s)" % (mode, len(items)),
             fontsize=12, color=C_INK, weight="bold", va="top")
    fig.text(0.08, 0.947, "top: whole move   bottom: zoomed to ±%.2f %s around the target   "
             "(damaged and aborted runs left out)" % (z, unit_lbl),
             fontsize=9.5, color=C_INK2, va="top")
    fig.subplots_adjust(left=0.09, right=0.985, top=0.915, bottom=0.08)
    fig.savefig(path, dpi=PLOT_DPI, facecolor=C_SURFACE)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def jsonable(m, v, run):
    d = {k: val for k, val in m.items() if not k.startswith("_")}
    d.update(verdict=v["verdict"], diagnosis=v["diagnosis"], suggestion=v["suggestion"],
             alternative=v["alt"], flags=v["flags"], damage=run.damage, notes=run.notes,
             begin=run.begin, end=run.end)
    return d


def main(argv=None):
    ap = argparse.ArgumentParser(description="Summarise [PIDLOG] runs from a V5 terminal capture.")
    ap.add_argument("log", help="capture file from `pros terminal --output tune.log`")
    ap.add_argument("--out", default="pidlog_out", help="output directory for PNGs/report (default ./pidlog_out)")
    ap.add_argument("--last", type=int, default=MAX_OVERVIEW_DEFAULT,
                    help="runs per mode in the overview plot (default %d)" % MAX_OVERVIEW_DEFAULT)
    ap.add_argument("--json", action="store_true", help="also write metrics.json")
    ap.add_argument("--no-plots", action="store_true", help="skip PNGs")
    args = ap.parse_args(argv)
    try:
        sys.stdout.reconfigure(errors="replace")
    except Exception:
        pass

    with open(args.log, "rb") as fh:
        raw = fh.read()
    runs, stats = parse_log(raw)
    os.makedirs(args.out, exist_ok=True)

    results = []
    for run in runs:
        m = analyse(run)
        v = classify(run, m)
        results.append((run, m, v))

    plt = None
    plot_msg = None
    if args.no_plots:
        plot_msg = "plots skipped (--no-plots)"
    else:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt  # noqa: F811
        except Exception as exc:  # ImportError or a broken install
            plot_msg = "plots skipped: matplotlib not available (%s). Install it with: pip install matplotlib" % exc

    paths = {}
    if plt is not None:
        for run, m, v in results:
            if not m["rows"]:
                continue
            p = os.path.join(args.out, "%02d_run%s_%s.png" % (run.index, re.sub(r"\W", "", str(m["run"])) or "x",
                                                            m["mode"].lower()))
            try:
                plot_run(plt, run, m, v, p)
                paths[run.index] = p
            except Exception as exc:
                print("warning: plot for run %s failed: %s" % (m["run"], exc), file=sys.stderr)
        for mode in ("DRIVE", "TURN"):
            items = [r for r in results if r[1]["mode"] == mode and r[1]["rows"]
                     and r[2]["verdict"] not in ("DAMAGED", "ABORTED")][-max(1, args.last):]
            if items:
                p = os.path.join(args.out, "overview_%s.png" % mode.lower())
                try:
                    plot_overview(plt, mode, items, p)
                    paths[mode] = p
                except Exception as exc:
                    print("warning: overview plot for %s failed: %s" % (mode, exc), file=sys.stderr)

    out = []
    out.append("pidlog: %s  -  %d run(s); %d other line(s) ignored%s" % (
        args.log, len(runs), stats["other_lines"],
        "".join([", %d data row(s) outside any run" % stats["orphan_rows"] if stats["orphan_rows"] else "",
                 ", %d END line(s) without a BEGIN" % stats["orphan_ends"] if stats["orphan_ends"] else ""])))
    if not runs:
        out.append("No [PIDLOG] BEGIN lines found. Is the firmware logging enabled, and is this the right file?")
    else:
        out.append("")
        out += table(results)
        out.append("")
        counts = {}
        for _, _, v in results:
            counts[v["verdict"]] = counts.get(v["verdict"], 0) + 1
        out.append("verdicts: " + ", ".join("%s %d" % (k, counts[k]) for k in VERDICT_ORDER if k in counts))
        out.append("")
        for run, m, v in results:
            out += details(run, m, v, paths.get(run.index))
            out.append("")
        for mode in ("DRIVE", "TURN"):
            if mode in paths:
                out.append("Overview %s: %s" % (mode, paths[mode]))
    if plot_msg:
        out.append(plot_msg)
    out.append("Suggestions are heuristics from one run each: change one gain at a time and re-run.")
    text = "\n".join(out)
    print(text)
    with open(os.path.join(args.out, "report.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    if args.json:
        jp = os.path.join(args.out, "metrics.json")
        with open(jp, "w", encoding="utf-8") as fh:
            json.dump({"log": args.log, "stats": stats,
                       "runs": [jsonable(m, v, run) for run, m, v in results],
                       "plots": {str(k): p for k, p in paths.items()},
                       "plots_skipped": plot_msg}, fh, indent=2)
        print("JSON: " + jp)
    return 0


if __name__ == "__main__":
    sys.exit(main())
