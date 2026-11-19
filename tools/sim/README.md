# Drive testbench (host simulation of LemLib's motions)

The real LemLib code (moveToPoint, turnToHeading, moveToPose, odometry, PID, exit conditions) is the DUT, compiled
unmodified for macOS against stub PROS headers (`stubs/`), run in virtual time against a simulated robot
(`bench/world.cpp`), with every millisecond dumped to CSV/VCD.

**Build:** `make -C tools/sim` builds `build/baseline/tb_baseline` (from `dut_baseline/`, `src/lemlib` as at git HEAD)
and `build/live/tb_live` (from `src/lemlib` now); `make -C tools/sim DUT=live` builds one. Headers always come from
`include/lemlib`. The Makefile picks a macOS SDK that links (this laptop's default SDK is newer than its linker).

**Run:** `tb_baseline --move drive --target 48 [--kP 12.5 --kI 0.1 --kD 100] [--rot-ms 20] [--rot-lock]
[--set KEY=VAL] --csv out.csv [--vcd out.vcd] [--pidlog run.log]`, or `--move turn --target 90`. It mirrors the PID
tab's test (`src/pidtuner.cpp`): BRAKE mode, target straight ahead, samples every 10 ms until 300 ms after the end.
A DUT `set_data_rate()` call overrides `--rot-ms` unless `--rot-lock`; the summary prints each sensor's final period.
DUT printf goes to /dev/null (`--dut-stdout FILE` keeps it). `--pidlog` writes the robot's PIDLOG format. Columns are
listed in `COLS[]` in `bench/tb_main.cpp`; `iter_*` are taken when the DUT's loop calls `pros::delay`, then held.
Each ms, events run before DUT code: IMU (10 ms) and rotation (`--rot-ms`) refresh, `lemlib::update()` every 10 ms
(the odometry task itself never runs), the PID-tab sampler. Motions start on a 10 ms boundary.

**Fit:** `/opt/anaconda3/bin/python3 tools/sim/fit.py` fits C, KV, F, DV_ACCEL, DV_BRAKE, LATENCY_MS in `robot.json`
(bounded differential evolution, then Nelder-Mead) on runs 788, 5793, 6073, 6766 of `data/tune_2026-11-19.log` and
reports all ten runs, held-out ones included. `--report` evaluates only; `--plot out/fit_runs.png` draws every run.

**Render:** `tools/sim/figures.sh` regenerates `out/before_after_{48,24}.png` and `out/displacement_{48,24}.png`
(BEFORE `tb_baseline --rot-ms 20`, AFTER `--rot-ms 5`; `AFTER_DUT=live AFTER_ARGS="" tools/sim/figures.sh` puts tb_live
in AFTER). Directly: `wave.py A.csv [B.csv] --label .. [--robot LOG:LINE] [--zoom T0:T1] -o out.png`, or
`--displacement out.png` for distance against time. `iters.py out.csv` prints the iterations around `close`.

**Modelled:** differential drive at 1 ms. Per side: command latency, back-EMF (KV), net-voltage limits (DV_ACCEL
with the motion, DV_BRAKE against it, mirrored in reverse), torque gain C, Coulomb friction F with stick-at-zero,
BRAKE/COAST at command 0 (HOLD = BRAKE). Heading rate (vL-vR)/TRACK_EFF. Tracking wheels at +0.9 in right and
+2.55 in front (2.125 in), integer centidegrees refreshed every `--rot-ms`; IMU true heading every 10 ms. Per-side
C scales (`C_DRIVE_L`...) default to 1. Plant units are odometry inches, as the PID tab logs them.

**Not modelled:** slip, sideways drift, left/right asymmetry (the robot drifts about 6 deg over 48 in), backlash,
viscous friction, battery sag, heat, sensor noise, the V5 motor's own control loops beyond the voltage limits, RTOS
jitter and task phase drift (phases are fixed, runs deterministic), TeamChassis' turn rules (stock turnToHeading is
called). TRACK_EFF is not fitted: turn numbers are only good for comparisons. The recorded runs came from the prebuilt
`firmware/LemLib.a`; the fit assumes it behaves like `src/lemlib` at HEAD for moveToPoint, PID and odometry.
