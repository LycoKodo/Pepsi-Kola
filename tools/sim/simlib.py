"""Shared helpers for the drive testbench: PIDLOG reader, bench runner, run metrics.

PIDLOG format (see CLAUDE.md): a `[PIDLOG] BEGIN k=v ...` line, rows `~t,err,dev,vL,vR,uL,uR`
(err in hundredths of an inch, positive = short of the target), then `[PIDLOG] END ...`.
Serial transfer drops bytes; pros-cli then prints `WARNING ... Could not decode bytes: <hex>`.
This reader also recovers a BEGIN line or complete rows from that hex when they are intact.
"""
from __future__ import annotations

import json
import os
import re
import subprocess
import tempfile
from dataclasses import dataclass, field

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
IPS_PER_RPM = 0.1080  # 2.75" wheel at 450 rpm on a 600 rpm motor, as in the log header

_ROW = re.compile(r"^~(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+)\s*$")
_HEX = re.compile(r"Could not decode bytes: ([0-9a-fA-F]+)")
_KV = re.compile(r"(\w+)=(\S+)")


@dataclass
class Run:
    line: int                      # 1-based line of the BEGIN line (or of the warning that held it)
    header: dict
    rows: dict = field(default_factory=dict)  # t_ms -> (err_in, dev, vL_rpm, vR_rpm, uL_mV, uR_mV)
    end: dict = field(default_factory=dict)
    recovered: int = 0             # rows taken from hex in decode warnings
    header_from_hex: bool = False

    @property
    def target(self) -> float:
        return float(self.header.get("target", "nan"))

    @property
    def mode(self) -> str:
        return self.header.get("mode", "?")

    def gains(self):
        return tuple(float(self.header.get(k, "nan")) for k in ("kP", "kI", "kD"))

    def arrays(self):
        """t (ms), err (in), speed (in/s, mean of both sides), all sorted by t."""
        ts = sorted(self.rows)
        t = np.array(ts, dtype=float)
        err = np.array([self.rows[k][0] for k in ts])
        spd = np.array([(self.rows[k][2] + self.rows[k][3]) / 2 * IPS_PER_RPM for k in ts])
        return t, err, spd

    def expected_rows(self) -> int:
        try:
            return int(self.end.get("rows", -1))
        except ValueError:
            return -1

    def label(self) -> str:
        kp, ki, kd = self.gains()
        return f"L{self.line} {self.target:+.0f}in kI{ki:g} kD{kd:g}"


def _parse_header(text: str) -> dict:
    return {k: v for k, v in _KV.findall(text.split("BEGIN", 1)[1])}


def _row_from(text: str):
    m = _ROW.match(text.strip())
    if not m:
        return None
    v = [int(g) for g in m.groups()]
    return v[0], (v[1] / 100.0, v[2], v[3], v[4], v[5], v[6])


def parse_log(path: str) -> list[Run]:
    runs: list[Run] = []
    cur: Run | None = None
    pending_hex_rows: list = []
    with open(path, errors="replace") as fh:
        lines = fh.readlines()
    for i, raw in enumerate(lines, 1):
        line = raw.rstrip("\n")
        decoded = None
        m = _HEX.search(line)
        if m:
            try:
                decoded = bytes.fromhex(m.group(1)).decode("latin1")
            except ValueError:
                decoded = None
        if "[PIDLOG] BEGIN" in line or (decoded and "[PIDLOG] BEGIN" in decoded):
            src = line if "[PIDLOG] BEGIN" in line else decoded[decoded.index("[PIDLOG] BEGIN"):]
            cur = Run(line=i, header=_parse_header(src), header_from_hex=decoded is not None and src is not line)
            runs.append(cur)
            pending_hex_rows = []
            continue
        if cur is None:
            continue
        if line.startswith("[PIDLOG] END"):
            cur.end = {k: v for k, v in _KV.findall(line.split("END", 1)[1])}
            _merge_hex_rows(cur, pending_hex_rows)
            cur = None
            continue
        r = _row_from(line)
        if r is not None:
            cur.rows[r[0]] = r[1]
            continue
        if decoded:
            # a complete row is '~' + 7 integers + newline; anything cut by lost bytes is dropped
            for frag in re.findall(r"~(-?\d+(?:,-?\d+){6})\n", decoded):
                rr = _row_from("~" + frag)
                if rr is not None:
                    pending_hex_rows.append(rr)
    return runs


def _merge_hex_rows(run: Run, cand):
    """Accept a hex-recovered row only if it is plausible next to the rows we do have."""
    if not cand or not run.rows:
        return
    ts = np.array(sorted(run.rows))
    errs = np.array([run.rows[t][0] for t in ts])
    for t, vals in cand:
        if t in run.rows or t % 10 or t < 0 or t > ts[-1] + 10:
            continue
        k = np.searchsorted(ts, t)
        nb = errs[max(0, k - 1):k + 1]
        # the error must lie between its neighbours (it is monotone over short spans), with slack
        if len(nb) and nb.min() - 1.5 <= vals[0] <= nb.max() + 1.5:
            run.rows[t] = vals
            run.recovered += 1


def find_run(runs: list[Run], near_line: int, target: float, kD: float | None = None, kI: float | None = None,
             tol_lines: int = 40) -> Run:
    """The run whose BEGIN is within tol_lines of near_line and whose header matches."""
    best = None
    for r in runs:
        if abs(r.line - near_line) > tol_lines or r.mode != "DRIVE" or abs(r.target - target) > 1e-6:
            continue
        kp, ki, kd = r.gains()
        if kD is not None and abs(kd - kD) > 1e-6:
            continue
        if kI is not None and abs(ki - kI) > 1e-6:
            continue
        if best is None or abs(r.line - near_line) < abs(best.line - near_line):
            best = r
    if best is None:
        raise LookupError(f"no DRIVE run target={target} kD={kD} kI={kI} near line {near_line}")
    return best


# --------------------------------------------------------------------------------------------
# metrics, the same way the PID tab computes them (src/pidtuner.cpp testTask)
# --------------------------------------------------------------------------------------------

def overshoot(err: np.ndarray) -> float:
    return float(max(0.0, -np.min(err))) if len(err) else float("nan")


def settle_ms(t: np.ndarray, err: np.ndarray, tol: float = 1.0) -> float:
    """PID tab definition: time of the first sample after the last one outside +-tol (1.0 in)."""
    out = np.nonzero(np.abs(err) > tol)[0]
    if len(out) == 0:
        return 0.0
    k = out[-1]
    if k == len(err) - 1:
        return float("nan")
    return float(t[k + 1])


def brake_by_voltage(run: "Run"):
    """(t, distance to go) where braking starts: the mean drive voltage, averaged over 20 ms (two
    samples, so the D-term alternation cancels), first falls below 40% of its running peak after the
    peak has passed 6 V. Works the same on robot logs and on the bench's pidlog."""
    ts = sorted(run.rows)
    sgn = -1.0 if run.target < 0 else 1.0
    u = np.array([(run.rows[k][4] + run.rows[k][5]) / 2000.0 * sgn for k in ts])
    e = np.array([run.rows[k][0] for k in ts])
    peak = 0.0
    for i in range(1, len(ts)):
        if ts[i] - ts[i - 1] != 10:
            continue
        um = (u[i] + u[i - 1]) / 2
        peak = max(peak, um)
        if peak > 6.0 and um < 0.4 * peak:
            return float(ts[i]), float(e[i])
    return float("nan"), float("nan")


def brake_start(t: np.ndarray, err: np.ndarray, spd: np.ndarray):
    """(t, distance to go) where the speed peaks: the start of braking."""
    if len(spd) == 0:
        return float("nan"), float("nan"), float("nan")
    k = int(np.argmax(np.abs(spd)))
    return float(t[k]), float(err[k]), float(abs(spd[k]))


# --------------------------------------------------------------------------------------------
# running the bench
# --------------------------------------------------------------------------------------------

def tb_path(dut: str = "baseline") -> str:
    return os.path.join(HERE, "build", dut, f"tb_{dut}")


def load_params(path: str | None = None) -> dict:
    with open(path or os.path.join(HERE, "robot.json")) as fh:
        d = json.load(fh)
    return {k: v for k, v in d.items() if isinstance(v, (int, float)) and not isinstance(v, bool)}


def run_bench(target: float, gains=(12.5, 0.1, 100.0), params: dict | None = None, rot_ms: int = 20,
              dut: str = "baseline", move: str = "drive", extra: list[str] | None = None,
              csv: str | None = None):
    """Run the bench; returns (t_ms, err_in, speed_ips, summary dict) of the PID-tab trace."""
    fd, plog = tempfile.mkstemp(suffix=".pidlog")
    os.close(fd)
    cmd = [tb_path(dut), "--move", move, "--target", f"{target:g}", "--kP", f"{gains[0]:g}",
           "--kI", f"{gains[1]:g}", "--kD", f"{gains[2]:g}", "--rot-ms", str(rot_ms), "--pidlog", plog,
           "--summary-json"]
    for k, v in (params or {}).items():
        cmd += ["--set", f"{k}={v:.10g}"]
    if csv:
        cmd += ["--csv", csv]
    cmd += extra or []
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, check=True).stdout
        runs = parse_log(plog)
    finally:
        os.unlink(plog)
    summ = json.loads(out.strip().splitlines()[-1])
    t, err, spd = runs[0].arrays()
    summ["_run"] = runs[0]
    return t, err, spd, summ


# --------------------------------------------------------------------------------------------
# chart style (dataviz skill reference palette, light surface; validated with validate_palette.js:
# blue/orange/aqua pass all-pairs CVD and normal-vision floors; aqua is < 3:1 on the surface, so
# every aqua line carries a direct label)
# --------------------------------------------------------------------------------------------

STYLE = {
    "surface": "#fcfcfb",
    "text": "#0b0b0b",       # primary ink
    "text2": "#52514e",      # secondary ink
    "muted": "#898781",      # axis ticks, annotations
    "grid": "#e1e0d9",       # hairline grid
    "baseline": "#c3c2b7",   # zero line / axis
    "band": "#f0efec",       # neutral wash (exit band, settle zone)
    "series": ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"],
    "font": ["system-ui", "-apple-system", "Helvetica Neue", "Arial", "DejaVu Sans"],
}


def apply_axis_style(ax, grid_y=True):
    import matplotlib as mpl
    ax.set_facecolor(STYLE["surface"])
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(STYLE["baseline"])
        ax.spines[side].set_linewidth(0.8)
    ax.tick_params(colors=STYLE["muted"], labelsize=8, length=3, width=0.8)
    for lbl in ax.get_xticklabels() + ax.get_yticklabels():
        lbl.set_color(STYLE["muted"])
    if grid_y:
        ax.grid(axis="y", color=STYLE["grid"], linewidth=0.8, linestyle="-")
        ax.set_axisbelow(True)
    mpl.rcParams["font.family"] = "sans-serif"
