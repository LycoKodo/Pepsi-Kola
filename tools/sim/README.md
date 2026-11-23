# Drive testbench (host simulation of LemLib's motions)

The real LemLib code (moveToPoint, turnToHeading, moveToPose, odometry, PID, exit conditions) is the DUT, compiled
unmodified for macOS against stub PROS headers (`stubs/`), run in virtual time against a simulated robot
(`bench/world.cpp`), with every millisecond dumped to CSV/VCD.

**Build:** `make -C tools/sim` builds every DUT: `live` (`src/lemlib` now) and each `dut_<name>/` laid out like
`src/lemlib` (`baseline` = first session's HEAD, `fix1` = HEAD with the PID/5 ms fix and stock moveToPose,
`pregps` = `src/lemlib` just before GPS fusion was added, for Off-equivalence checks).
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

**GPS fusion** (`docs/GPS_FUSION_DESIGN.md`): `make -C tools/sim unit` builds and runs `tests/gps_fusion_unit.cpp`
(the filter, `GpsFusionCore`, compiled with `GPSFUSION_CORE_ONLY` and no PROS headers at all) and
`tests/gps_fusion_layer.cpp` (its robot half against `stubs/`, whose `pros/gps.hpp` is an unplugged sensor unless a
test installs `pros::sim_detail::gpsReadHook`, and whose `pros::Mutex` can be made busy with
`pros::sim_detail::mutexBusy` to check that nothing waits long for the fusion lock; the bench never sets it). Wall fixes (`docs/WALL_FIX_DESIGN.md`, distance sensors as one-axis measurements in the same filter) are tested by `tests/gps_fusion_walls.cpp` (built like the unit test, no PROS) and in `tests/gps_fusion_layer.cpp`; `stubs/pros/distance.hpp` is an unplugged sensor (`PROS_ERR`) unless `pros::sim_detail::distanceReadHook` is set, and whatever links it defines that hook once (the bench in `bench/stubs_impl.cpp`; route mode points it at the simulated distance sensor, below). `make unit` also builds and runs `tests/dist_model_test.cpp`, the simulated distance sensor's own checks (198, against hand-computed numbers). `make unit GPSFUSION_SRC=FILE` runs the three filter tests against another copy of the filter. DUTs
that have `chassis/gpsFusion.cpp` compile it (the Makefile passes `DUT_HAS_GPSFUSION`); the single-move modes never
switch fusion on, route mode can (below). `--pose-trace FILE` writes the exact pose (hex floats) after every
`lemlib::update()`: with the same arguments, `tb_live` and `tb_pregps` must give identical traces and CSVs.

**Route mode, the simulated GPS and the fusion acceptance tests.** `tb_live --route routes/skills60.txt [--seed N]
[--fusion off|shadow|on] [--csv ticks.csv] [--summary-json]` drives a list of moves in field coordinates (inches from
the field centre, +y toward the 0 degree wall, heading clockwise) with the real `moveToPoint` / `turnToHeading` /
`moveToPose`, blocking, one after another; the file format is at the top of `bench/route.hpp` (`start`, `point`,
`turn [cw|ccw]`, `pose`, `wait`). `routes/skills60.txt` is a 60 s Skills-like route (about 900 in, 2100 deg turned,
25 stops of 0.3-0.5 s, near walls and through the middle, |x|, |y| <= 60); `routes/std_*.txt` are the A5 standard
moves, each after a lead-in. *Odometry errors* are world parameters, all 0 by default, set with `--set`:
`IMU_SCALE_ERR_CW/CCW` (0.012 = 1.2 %), `IMU_DRIFT_DEG_S`, `VERT_SCALE_ERR`, `HORIZ_SCALE_ERR`, `SLIP_PER_IN` and
`SLIP_MAX_IN` (random slips of each tracking wheel, from `--seed`), and in route mode `--shove T_MS:RIGHT:FORWARD[:MS]`
or `--shove sSTEP:...` (the robot is pushed without its wheels or IMU noticing). With all of them 0 every existing
single-move output (CSV, VCD, PIDLOG, summary, pose trace) of all four DUTs is byte-identical to the bench before they
were added (checked on 16 moves x 4 DUTs). *The GPS* (`bench/gps_model.*`, behind `stubs/pros/gps.hpp`, whose
unplugged default `tests/gps_fusion_layer.cpp` still uses) is fed by the world: camera mount and facing, a new output
every 40 ms, 60 ms delay, white + slowly varying noise + noise growing with the distance to the wall it faces, a dead
zone where it dead-reckons and drifts with no error flag, outliers, view losses that freeze or drift, "no strips"
(frozen, or dead-reckoning from where it thinks it started), heading noise plus the mount yaw, `get_error()` loosely
tied to the real error, and a mirrored frame. For the revision 2 tests, all off by default: `strip_walls` (which walls
have a strip, added up: 1 = +y, 2 = +x, 4 = -y, 8 = -x, 15 = all; facing a wall with no strip the camera is blind and
dead-reckons as in the dead zone, `gps_state` 4), `bias_x_in` / `bias_y_in` (a constant error on every output that
follows the camera; `get_error()` does not see it), `no_strip_at_truth` (with `no_strips` 1 or 2 the sensor believes it
booted exactly where the camera was, facing the right way: with 2 it then looks like a working sensor that slowly
drifts), and a drift rate per view loss (`--gps-loss START:LENGTH:1:RATE`, in/s). For revision 3's slow-liar rows,
ported from the revision 2 attacker's bench and off by default (then the model and its JSON are byte-identical to
before): `drift_mode` 2 (a blind drift that runs only while the camera's true speed is under 0.5 in/s) and
`bias_step_in` / `bias_step_ms` / `bias_step_start_ms` / `bias_step_count` / `bias_step_dir_deg` (a bias that grows in
steps from a route time, in a field direction). A camera that is blind from boot (dead
zone, or a wall with no strip) dead-reckons from its true position, the worst case. Every number: `--gps KEY=VALUE` or
`--gps-json FILE` (keys in `GpsModelParams`, `bench/gps_model.hpp`; a JSON file may also hold `"view_losses": [start,
length, ...]`), `--gps-loss START:LENGTH[:1 drift|0 freeze[:RATE]]`, `--no-gps`. Its random numbers have their own seed
and a fixed count per output, so turning on outliers does not change the noise. With the new options at their defaults
every output is byte-identical to the bench before they were added (16 single moves x 4 DUTs in every format, 20 route
runs and 5 standard moves with fusion Off, Shadow and On: 455 of 487 files identical, the 31 route JSON summaries
identical once the keys added to them are taken out, and one CSV differs only where check 5b, switched on in that run,
now shows `fate` 12 instead of 0). *Fusion*: `--fz KEY=VALUE` / `--fz-json FILE` set any
`lemlib::gpsfusion::Config` number by its C++ name; `--start-error DX:DY[:DEG]` tells `setPose` the wrong start;
`--start-wait MS` stands still first; `--compare-with OFF.csv` (a fusion-Off run, same seed) compares motor commands
tick by tick while the two real robots are within 0.1 in and 0.1 deg. *Outputs*: the CSV has one row per odometry
tick (columns listed in `COLUMNS[]`, `bench/route.cpp`: truth, the pose the motions read, odometry alone = pose minus
the corrections fed in, fused, the filter's estimate, sigma, pending, the correction this tick, the GPS output and its
state, `fate` = what the filter did with a new reading (1-7 the check that dropped it, 8 gate, 9 no history, 10 start
check, 11 accepted, 12 dropped by check 5b, which is off by default), `fault` (1 StartDisagree, 2 RelocTooFar, 3
RelocTooMany, 4 TotalTooLarge, 5 HeadingDisagree), motor commands, errors). The JSON summary has end / max / rms
position error for odometry alone and fused, counts per check, the fault and when, accepted outliers and blind
readings (with the largest error among them), the largest correction in one tick, the command comparison, shove
recovery times, per move the PID tab's overshoot and settle time (from the pose the motions read) plus `swing_true` /
`settle_true_ms` (the real robot against its own resting point), and revision 2's numbers: `fault_name`, the largest
step one reading made (`max_reading_step_in`), the decided total and when it first went past `maxTotalCorrectionIn`
(`max_decided_in`, `decided_over_limit_ms`), the most fed in before the fault and at all, how long after the fault until
less than 0.5 in was fed in (`fault_to_fed_under_half_ms`, -1 if never) and until nothing was (`fault_to_unwound_ms`),
the heading-fault counts, the average innovation in the robot's frame and the average NIS.

`/opt/anaconda3/bin/python3 tools/sim/gps_suite.py [--seeds 50] [--jobs N] [--no-figures] [--keep]` runs the design's
acceptance tests (A1-A11, "Acceptance tests, revision 2" as restated by "Revision 3") over their seeds (about 8 minutes
on this laptop, about 5000 bench runs), prints one table (measured, limit, PASS / FAIL, and INFO where the design says "report" or a number is
only shown; rows marked `?` investigate a failing row with one number changed at a time), saves every number in
`out/gps_suite_results.json` and draws `out/gps_*.png`; the route top view, error against time (A3a both spins, A3b),
the 20 s blind loss (A10) and the shove figure (A9) are copied to `docs/sim/`. The GPS is as noisy as the design
assumes: every noise term of the model is scaled by 1.535, which row M measures as 1.50 in per axis standing still (the
model's defaults, the first suite's noise, measure 0.98 in; one row compares at that). Choices it makes where the design
is silent: odometry error signs are drawn per seed, with random slips; "fused" is the pose the motions read with fusion
On, "odometry alone" a fusion-Off run with the same seed (the GPS is never read with fusion Off, so one Off run serves
every GPS setting), except where the design says "the pose is within X of odometry alone" (A7, A9, A10): that is
measured within the On run, as the pose minus the corrections fed in; tests that name no odometry errors use A3a's on
the route as written, A4 and A9 perfect odometry, A8 A3b's (the calibrated robot). Since revision 3 (limits 6/6/1, On
needs a calibrated robot) A3a's end-error rows are INFO: they are outside what the filter is for, and their text gives
the cause (an uncalibrated robot needs more than `maxTotalCorrectionIn` of correction; the faults by name). A3a runs the route's full spin both ways (the counter-clockwise route
is made by the suite). A5: the largest correction in one tick over every On run of the suite; the standard moves judged
on the real robot (`swing_true`, `settle_true_ms`) against Off; the 99th percentile of the tick-to-tick change of each
motor command while a move runs, pooled over A3a's runs, On against Off. A6 compares with A3a clockwise. A7: the start
error's direction is drawn per seed; the mount yaw is wrong both ways. A8's "stable" means no fault and the run's rms
error within 0.5 in of the 60 ms case. A9's shove is to the robot's right over 200 ms in the stop after the long drive
(step 22); recovery means within 2 in and staying there 1 s. Revision 3 sizes A9 from `relocMaxIn` (6): a 4 in shove
(two thirds) must recover in 19 of 20, a 9 in shove (`relocMaxIn` + 3) must fault and return to odometry alone, and a 2
to 12 in sweep is reported. A10: the loss starts at 15 s; the excess is fused error minus odometry-alone error at the
same tick, the shorter run held at its last value; "a fault when the limit is hit" means a fault on the tick the
decided total first passed `maxTotalCorrectionIn`; the limit is `maxTotalCorrectionIn` + 1 (7 in). Revision 3's slow
liars (reported, calibrated robot): a blind drift of 0.1, 0.2, 0.3 in/s from 15 s to the end, a 0.5 in/s drift only
while standing still, a bias stepping 2 in every 5 s from 10 s to 6 in; each with the worst and end excess and the
time the excess is above 4 in. A3b also reports stops more than 1 in worse and better than odometry alone, and the end
error 0.5, 1, 2 and 4 s after the last move (the suite makes a copy of the route that waits 4 s at its end). A6 also
reports relocalisations (read from the CSV: the jump is the change of `est - odom` on the tick `reloc` grows; "built on
blind readings" counts the last 4 gate failures up to it with `gps_state` not 0) and, as INFO, the model's dead zone
at 20 and 24 in. A7 also runs the three mirrored frames from start poses on x = 0 and y = 0 and the route's own, at
headings 0, 90 and 270: a start whose mirrored reading is the start pose itself (left-right on x = 0 facing 0 or 180,
front-back on y = 0 facing 90 or 270) is "on the line" and reported; every other one keeps A7's limits. A7 also runs
`routes/turnfirst.txt` (the first revision 2 reviewer's route: a 180 deg turn at once) with a healthy sensor, which
must not fault. A11's bias direction is drawn per seed. Extra INFO rows: faults in healthy runs (A3a, A3b), stops
where fused is more than 1 in worse than odometry alone, A9 with A3a's errors, and Shadow left exactly as Off.

**Wall fixes in route mode** (`docs/WALL_FIX_DESIGN.md`, its Revision A winning). *The simulated distance sensor*
(`bench/dist_model.*`, `sim::DistModel`; written from community reports, not measurements: every number in it is a
guess) is given the time and the TRUE pose every ms (the world's `deviceRefresh`, after the GPS model) and answers
`{mm, confidence, objectSize}` as PROS does for up to two sensors: a new reading every 33.3 ms, the beam a cone (12
deg, 18 below 200 mm) read as its centre ray, its nearest point or a blend of the two (default), the 20 and 2000 mm
limits, noise (new every reading and slowly varying), a scale error and an offset per sensor, confidence falling with
range and angle, the true wall planes, clear walls (read through to something behind, dropouts beyond a range, lower
confidence), wall-mounted elements, circles, and an object put in the beam whenever the robot is nearly stopped
(`stop_obj`, test W4; `stop_obj_from_ms`, added when it came into the bench, keeps it away before a world time so a
first stop can be clean). Each sensor has its own random generator (seeded from `--seed * 2 + 3`) and every reading
draws the same numbers whatever the options, so switching an effect on never changes another. `--dist help` lists
every option with its default. *Options* (route mode, DUTs with fusion only): `--wall-sensor RIGHT:FORWARD:YAW[:SCALE[:
OFFSET_IN]]` (up to two, in order: sensor 0 on port 6, sensor 1 on port 21) is what the FILTER is told (its
`WallSensorMount`); configuring sensor N also fits the model's sensor N (`sN_enabled=1`), whose true mount is the
model's own `sN_right_in`, `sN_forward_in`, `sN_yaw_deg` (defaults: back sensor 0, -8.5, 180; right sensor 8.5, 0, 90),
so a test can tell the filter a wrong mount; `--wall KEY=VALUE` / `--wall-json FILE` set any
`lemlib::gpsfusion::WallConfig` number by its C++ name (`planeIn0`..`planeIn3`, `trust0`..`trust3` as 0 Ignore, 1 Clear,
2 Solid); `--wall-trust WALL|all:ignore|clear|solid`; `--keep-out WALL:FROM_IN:TO_IN` (up to 8); `--dist KEY=VALUE` /
`--dist-json FILE` set the model (applied after the sensors are fitted, so `--dist s1_enabled=0` unplugs one); the
GPS is present or not as before (`--no-gps`). `configureWalls()` is called right after `configure()`, before the
mode is set. *Outputs, only with `--wall-sensor`* (with none, every output is byte-identical to the bench before the
model existed: 487 files compared, 16 single moves x 4 DUTs in every format, 20 GPS route runs, 5 standard moves Off
and On, one pre-GPS route run; the only differences are the output folder's own name inside 5 `cmd_compare`
summaries): the CSV gains, after its columns, `sigma_x`,
`sigma_y`, `walls_waiting`, `walls_stopped` and per sensor N `wN_mm`, `wN_conf`, `wN_new`, `wN_mstate` (what the reading
really is: 0 wall, 1 clear wall, 2 corner, 3 element, 4 object, 5 stop object, 6 through, 7 no return, 8 too close, 9
not fitted), `wN_clean` (the range a clean reading would give, from the truth), `wN_read` (the reading calibrated as
the filter does it), `wN_pnow` (the range predicted from the filter's estimate, the bench's own cast with the told
mount and planes), `wN_state` (0 unused, 1 waiting for its start check, 2 working, 3 start fault, 4 health off),
`wN_fate` (this tick: 0 not sampled, 1-13 dropped by wall check N, 14 start check passed, 15 start check failed, 16
fix, 17 switched off by the health rule after passing check 10), `wN_fixes`, and with an innovation `wN_axis`,
`wN_pred` (p), `wN_meas` (m), `wN_inn`, `wN_corr` (the fix's correction); the fate is read from which of the filter's
per-sensor counters moved. The JSON summary gains `walls`: the mounts told, the `WallConfig`, the model's options,
`route_start_world_ms` (the model runs on world time; route time 0 is 10 ms), the doubts at the end, and per sensor
its state, samples, fixes, drops per wall check, health counts, start innovation, when its start check and any
health switch-off happened, clean readings (every reading its median came from of the wall itself, by the model)
refused as blocked or through, at which steps and the largest estimate error on that axis then, readings sampled by
model state, fixes whose readings were not all of the wall, the largest error any fix added, and every fix with the
truth beside it (`err_before`, `err_after`: the estimate minus the truth on the fix's axis; `implied_err`: where the
reading put the robot minus the truth; `states_mask`; `cone_mask_model` / `cone_mask_18`: what the true cone meets from
the true pose at the model's angle and at 18 deg, bits as in `dist_model.hpp`). The text summary gains a line per
sensor. *Routes*: `routes/walls60.txt` (58 s, 13 stops of 0.7 s with the back sensor square to a wall 9.75 to 18.75 in
away on all four walls, 5 with the right sensor square to the next wall too, all in the first 32 s; second lap the
other way round), `routes/walls_corner.txt` (corner and keep-out stops; its two placeholder elements are `#@element
WALL FROM TO DEPTH` comment lines), `routes/wall_std_*.txt` (W9: a 40 in lead-in, a 400 ms stop with the right
sensor square to the +x wall, then the measured move); `walls_check.py ROUTE [--csv TICKS.csv]` checks a route's
geometry (outline inside the field, each stop's range, angle and corner distance for both sensors, check 8 as its
author reads the design). Not modelled: the bench has no walls (a robot that drives into one carries on, and the
sensor reads TOO_CLOSE), the cone's vertical spread, colour and shine, other robots except as circles.

`/opt/anaconda3/bin/python3 tools/sim/wall_suite.py [--seeds 50] [--jobs N] [--no-figures] [--keep] [--only W2,W4]`
runs the wall design's acceptance tests W1-W13 (about 3 minutes, about 3500 bench runs) and prints one table in the
GPS suite's style (PASS / FAIL / INFO; rows marked `?` investigate a failing row with one thing changed at a time),
saves every number in `out/wall_suite_results.json` and draws `out/wall_route_top.png` (a median uncalibrated run from
above: walls, true path, odometry alone, fused, each fix's beam), `out/wall_error_time.png` (error against time, both
robots, one and two sensors) and `out/wall_blocked.png` (W4 and W5: each sampled reading minus the prediction, by what
the filter did with it), all copied to `docs/sim/`. Choices it makes where the design is silent: GPS off except W10
and W12; the back sensor for one sensor, back and right for two, the filter told where they truly are; the sensors
noisy in every row unless it says noise-free (white 3 mm + 0.3 %, slowly varying 2 mm + 0.5 %, confidence jitter 2,
and a calibration left over per sensor and seed of up to 1 % scale and 3 mm offset; about 0.27 in at 12 in); every
wall Clear unless a row says Solid; odometry errors imported from `gps_suite.py` (A3a uncalibrated, A3b calibrated,
both with slips, signs per seed), with the calibrated rows first because the team only switches fusion on with a
calibrated robot; "odometry alone" a fusion-Off run with the same seed and no sensor (W1 checks that Off is the same
with sensors fitted); "the pose equals odometry alone" measured within the On run as nothing ever fed in; "error
added" by a fix the change in the size of the estimate's error on its axis, "implied error" where the reading put
the robot minus the truth. W1 (since the GPS filter's revision 3, which changed the filter on purpose): (a) Off
against pre-GPS LemLib, (b) Off with two misbehaving wall sensors fitted, (c) fusion On with no wall sensor, run with
and without `--walls-zero` (the bench then calls `configureWalls()` with zero sensors), 21 GPS-only runs, pose traces
byte-identical. The old comparison with the frozen revision 2 binary (`w1_regress.sh`, `golden/tb_gpsv2` in the wall
work's scratch folder, `W1_DIR`) is no longer run; that folder is left as it was. W4's objects come from the start (the start check sees them) and, as a second row, from
6 s world time (after the first stop): then the first stop's start check and fix are on a clean wall and the test is
that no reading of the object fixes; on the uncalibrated robot those rows are reported (Revision B). W2's
uncalibrated rows are reported too. Next to W2's calibrated end-error rows, INFO rows give the whole-route rms and
mean error (per run over every tick of the CSV, median over seeds), fused against odometry alone, and how many seeds end
worse than odometry alone. W10 (GPS and walls together) is reported, not judged, with the number of seeds in which
together ends worse than walls alone; W3 also reports its measure on seeds 1 to 20 only. W8 (Revision B) shifts the start 5 in along x, along y and in a direction per
seed and judges two things: no fix on the axis set wrong (more than the 3 in start tolerance of error) leaves it worse, and a sensor whose start check
looked at an axis with more than 4 in of error faults (the axis is `wN_axis` on the tick `wN_fate` is 14 or 15;
sensors within 1 in of the 3 in tolerance are only counted); W9 is measured
as A5 (largest pose change in one tick over every On run, the standard moves on the real robot against Off with the
wall design's 50 ms, the 99th percentile of the tick-to-tick command change pooled over W2's runs); W11 moves the true
-x wall 0.5 in out and judges "must not grow" on a noise-free deterministic run as never more than the plane error
plus 10 percent; W12's GPS has no strips (`no_strips=2`: it faults, HeadingDisagree, 0.7 s in); W13 (Revision B)
lowers `maxTotalCorrectionIn` to 1 and 2 in on the uncalibrated robot with true walls (every run that passes the limit
must fault TotalTooLarge on that tick, stop the walls and end on odometry alone) and reports the +y wall's plane told
2 or 3 in wrong on a Solid wall.

**The UMBmark square** (odometry calibration: the robot drives a square under its own odometry, believes it is back
at the start, and a person measures where it really is; real minus believed is the error). *Routes*:
`routes/umb_cw.txt` and `routes/umb_ccw.txt`, the square the robot's routine drives: `setPose(0, 0, 0)`, stand still
2000 ms, then (CW) `point 0 48`, `turn 90`, `point 48 48`, `turn 180`, `point 48 0`, `turn 270`, `point 0 0`, `turn 0`
(CCW mirrored: 270, 180, 90, 0), every move at `max=70` with the routine's time limits (`timeout=5000` a leg, 3000 a
turn; the routine ends the run at the first one, a route carries on), a 300 ms stop after each, then 1000 ms more
before the pose is read (the route format already had `max=`). Turns go the shorter way. The route files do not
carry the robot's tracking-wheel offsets: `umb_suite.py` passes them (`--set VERT_OFFSET=-0.6 --set
HORIZ_OFFSET=-2.23 --odom-offsets -0.6:-2.23`); run bare, they get the bench's old 0.9 / 2.55. Also not in them: the
routine sets the pose twice, 20 ms apart (a route has one `start`), and since 2026-11-22 it runs each move in LemLib's
own task and checks the sticks every 10 ms, so each stop can be up to 10 ms longer (in a scratch host build of the
routine on this world: 13730 ms against 13650 ms for the square, and without slip the same end pose to 0.001 in; with
slip the random slips fall at other moments, so single runs differ by about as much as slip scatters them anyway). The bench calls `lemlib::Chassis::turnToHeading`,
not `TeamChassis::turnToHeading`; for these 90 deg turns they are the same (the team's rules only act under 45 deg or
from 100 deg). *Bench options* (both modes; unused, every output is byte-identical to the bench before them, checked
on 18 single moves x 4 DUTs in every format and 15 route runs, 492 files): `--imu-scale CW:CCW` calls
`lemlib::setImuScale` (only DUTs that have it: the Makefile passes `DUT_HAS_IMUSCALE`; today only `live`);
`--odom-offsets VERT:HORIZ` sets the tracking-wheel offsets LemLib is told (default 0.9:2.55, the old robot's, which
is also where the world's wheels are unless `--set VERT_OFFSET` / `HORIZ_OFFSET` move them; the robot itself has
-0.6:-2.23). *World parameters*: `VERT_YAW_DEG`, `HORIZ_YAW_DEG` (0 by default): a tracking wheel turned that many
degrees clockwise from parallel, which then reads part of the motion across it. *Suite*:
`/opt/anaconda3/bin/python3 tools/sim/umb_suite.py [--seeds 30] [--jobs N] [--no-figures]` (about 25 s, about 2400
runs) prints four tables and saves every number in `out/umb_results.json`: (a) one error at a time on a perfect robot
without slip, both directions, E (x right, y forward of the start frame), H, and the largest error at a stop on the
way, with the small-error formula and its exact form beside the inertial rows and a drift-as-scale prediction
beside the drift rows; (b) slip at 0.45, 1.5 and 5 % (slip_sweep.py's pairs), 30 seeds per direction for the
spread, 300 for the mean; (c) side 24 / 48 / 72 in against speed 70 / 127, and gps_suite.py's A3a and A3b robots (10
robots, 5 runs per direction, e estimated from each 5-run mean); (a2) a robot that spins about a point 2.24 or 5.8 in
behind its tracking centre, alone and with the inertial error. `run_square(direction, seed, extra_args,
spin_behind_in=0)` is the importable helper: one square, returning the real end, the believed end, E, H, the duration
and still_dps. It draws `docs/sim/umb_signatures.png`. Choices: every run puts both the world's wheels and what LemLib
is told at the robot's offsets (-0.6, -2.23); GPS and fusion off. The bench's robot spins about its drive centre and its
true pose is that point, so `spin_behind_in=D` tells the world the horizontal wheel sits D further forward than LemLib
is told (the tracking centre is then D ahead of the spin point) and moves the ruler's reading to the tracking centre;
the bench's own odometry-error summary still uses the drive centre, so `run_square` works its numbers out from the
ticks. Not shown by this bench: slip that grows with speed (the model's slip is per inch, whatever the speed), IMU
noise (so H has no scatter here).

*End to end*: `/opt/anaconda3/bin/python3 tools/sim/umb_e2e.py [--jobs N] [--work DIR] [--quick]` drives whole
sessions (5 squares each way), writes what a student would have (the log, with the routine's own printf format read
out of `src/umbmark.cpp`, and three measurement files: position and turned, position and a heading to 0.5 deg, two
corners and turned), runs `tools/umbmark.py` on them as a separate process, checks its answers against the truth,
puts its constants back with `--imu-scale` for a second and third round and the 60 s route, and tries the student
mistakes a review found (scenario h). It stops at start-up if the routine's numbers, its [UMB] and [umbmark] lines or
the route files no longer match. Its pass/fail limits are named constants, written down before the runs that use
them; a failure is reported, not hidden (exit status 1). Results in `out/umb_e2e_results.json`.

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

**Not modelled:** left/right asymmetry (the robot drifts about 6 deg over 48 in), backlash, viscous friction, battery
sag, heat, rotation-sensor and IMU noise, RTOS jitter, TeamChassis' turn rules, walls (a route that drives into one
carries on through it). Tracking-wheel slip, scale errors, IMU scale and drift exist only as the optional error sources
above. The GPS model is built from community reports, not measurements of this sensor. TRACK_EFF (15) is not fitted,
so turning dynamics in turns and moveToPose are a guess. Use those results to compare library versions; don't read
them as predictions of the robot's own times.
