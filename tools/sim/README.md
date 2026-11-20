# Drive testbench (host simulation of LemLib's motions)

The real LemLib code (moveToPoint, turnToHeading, moveToPose, odometry, PID, exit conditions) is the DUT, compiled
unmodified for macOS against stub PROS headers (`stubs/`), run in virtual time against a simulated robot
(`bench/world.cpp`), with every millisecond dumped to CSV/VCD.

**Build:** `make -C tools/sim` builds every DUT: `live` (`src/lemlib` now) and each `dut_<name>/` laid out like
`src/lemlib` (`baseline` = first session's HEAD, `fix1` = HEAD with the PID/5 ms fix and stock moveToPose).
`make -C tools/sim DUT=<name>` builds `build/<name>/tb_<name>`. Headers always come from `include/lemlib`, so a DUT
snapshot must match today's headers. The Makefile picks a macOS SDK that links (the default one is newer than
the linker).

**Run:** `tb_fix1 --move drive --target 48 [--kP .. --kI .. --kD ..] [--rot-ms 20] [--rot-lock]`, `--move turn
--target 90`, or `--move pose --x 24 --y 24 --theta 90 [--lead 0.6] [--max-speed 127] [--min-speed 0] [--drift D]
[--backwards] [--timeout 4000]` (real `Chassis::moveToPose` from (0, 0, 0)). Add `[--set KEY=VAL] --csv out.csv
[--vcd out.vcd] [--pidlog run.log]`. Every move mirrors the PID tab: BRAKE mode, samples every 10 ms until 300 ms after
the end. A DUT `set_data_rate()` overrides `--rot-ms` unless `--rot-lock`. DUT printf goes to /dev/null. Columns are
listed in `COLS[]` in `bench/tb_main.cpp`; `iter_*` are taken when the DUT's loop calls `pros::delay`, then held.
Each ms, events run before DUT code: IMU and rotation refresh, `lemlib::update()` every 10 ms, the PID-tab sampler.

**Pose signals are mirrored, not read out of the DUT.** `bench/pose_mirror.cpp` re-does stock moveToPose's arithmetic
(carrot, close, errors, maxSpeed, slew, the `maxSlipSpeed` cap, overturn, direction, minSpeed, ratio) from the pose the
DUT saw, with the DUT's own `PID`, `ExitCondition`, `getCurvature` and `Pose`. Every iteration it checks its left/right
commands against the DUT's (`p_mirror_ok`; the summary counts mismatches): 0 on fix1, so the `p_*` stages are exact
there. After moveToPose is reworked they describe stock behaviour only. In drive mode `iter_D` is the bench's
one-sample `kD*(e - e_prev)`, not the two-sample derivative fix1's PID uses.

**Scripts** (`/opt/anaconda3/bin/python3`): `fit.py` fits C, KV, F, DV_ACCEL, DV_BRAKE, LATENCY_MS to the PID-tab logs
in `data/` (`--report` evaluates only). `figures.sh` renders the drive before/after and displacement PNGs.
`pose_grid.py [--dut fix1] [--drift 8] [--mu-lat 0.3] [--csv t.csv] [--save-runs DIR]` runs 13 target poses, one row
each. `pose_figures.sh [fix1 [live]]` renders top view and waveforms for (24,24,90) and (2,4,30). `wave.py` draws
waveforms (`-o`, drive or pose CSVs, one or two side by side), `--displacement`, and `--path` (top view, `--carrot`).
`iters.py CSV` prints iterations: around `close` for drives, the mirrored stages for poses.

**Modelled:** differential drive at 1 ms. Per side: command latency, back-EMF (KV), net-voltage limits (DV_ACCEL
with the motion, DV_BRAKE against it), torque gain C, Coulomb friction F with stick-at-zero (commands below about 3.5
do not move a stopped robot), BRAKE/COAST at command 0. Heading rate (vL-vR)/TRACK_EFF. Sideways slip: without
sideways force the robot keeps its world velocity (sideways speed changes at -omega*v), wheel scrub holds it up to
MU_LAT*g; the default MU_LAT 100 never lets go (drives and turns are bit-for-bit as before). Tracking wheels at +0.9 in
right and +2.55 in front read turning and slip; the IMU reads true heading every 10 ms. Units are odometry inches.

**Not modelled:** left/right asymmetry (the robot drifts about 6 deg over 48 in), tracking-wheel slip, backlash,
viscous friction, battery sag, heat, sensor noise, RTOS jitter, TeamChassis' turn rules. TRACK_EFF (15) is not fitted,
so turning dynamics in turns and moveToPose are a guess. Use those results to compare library versions; don't read
them as predictions of the robot's own times.
