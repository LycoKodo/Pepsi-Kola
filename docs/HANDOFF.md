# Handoff: state of the work

Written by the cloud session that did the work below, so another session can take
over with the user directly. `CLAUDE.md` has the durable repo knowledge;
`docs/TUNING_RUNBOOK.md` has the step-by-step for a tuning session. This file is
the rest: what was done, what is known, what is undecided, and what was planned.

## Update: first calibration session on the robot, 2026-11-23 (read this first)

Everything in this section ran on the robot unless it says otherwise.

- **Horizontal tracking sensor counted the wrong way.** Now constructed reversed (`-ports::HORIZONTAL_ODOM`).
  This was the old `[ODOMCAL]` puzzle; values logged before this date have the horizontal sign flipped.
- **Tracking wheels are 1.982 in (vertical) and 1.987 in (horizontal)**, from 48 in / 24 in tape pushes. LemLib's
  `NEW_2` (2.125) made the odometry read 6 % long, so every move drove 6 % short. Existing autons tuned by trial
  will now overshoot: re-check them.
- **The tracking centre is now the robot's spin point**, 1.55 in ahead and 0.18 in left of the geometric centre
  (offsets -0.42 / -3.78), fitted from `[UMBLEG]` lines over 8 turns and re-checked over 16. Before, each 90 deg
  turn swung the tracked point about 2 in and UMBmark squares ended 2.4-3.3 in from the start mark; after, 0.4,
  0.6, 0.8 and 1.8 in. Poses in routines now mean the spin point.
- **Inertial scale left at 1.0.** UMBmark, 3 runs each way: clockwise -0.04 +- 0.10 %, counter-clockwise
  +0.40 +- 0.09 % (position only), and the calculator predicted no gain from changing it.
- **Selector crash fixed.** `tabWatcher` was a task calling `lv_*`; it died with DATA ABORT three times. It is an
  `lv_timer` now (50 ms). Compiled and uploaded; no crash since, over about ten runs.
- **UMBmark** prints `[UMBLEG]` (pose after each of the 8 moves) and refuses to start when the heading creeps
  more than 0.05 deg/s standing still (a bad IMU boot gave -3.18 deg/s and a square 45 deg crooked).
- **Y button** (driver control, no competition switch): tap prints a `[MARK]` line with pose and raw sensor
  degrees since the last zero; hold 1 s zeroes the pose.

Open: every turn still moves the pose about 0.64 in "backward" at mid-turn heading, the same for both
directions, so no offset removes it. Not known whether the robot really moves or the vertical wheel reads it.
Test: five PID-tab TURN +90 / -90 pairs on a tape cross, then compare the robot with the Y-button pose. Drives
end 0.25 in past their target. `tools/sim/umb_suite.py` still uses the ruler offsets (-0.6 / -2.23) as the
robot's geometry. The last build (drift refusal) was uploaded; the drift refusal itself has not been triggered
on the robot.

## Update: UMBmark square test, 2026-11-22 evening 

Built and tested in the simulator; the robot code is only compiled. Nothing ran
on the robot. Uncommitted. The how-to for the user is `docs/UMBMARK.md`.

- **What it is.** The robot drives a 48 in square under odometry, clockwise and
  counter-clockwise, five runs each; a person measures where it really ends; a
  laptop tool turns that into `IMU_SCALE_CLOCKWISE` / `_COUNTER_CLOCKWISE`.
  It replaces "ten turns each way against a floor mark" as the way to get the
  inertial scale, and its run-to-run scatter is the slip check.
- **Pieces.** `src/umbmark.cpp` (Skills buttons "UMB CW" 101 and "UMB CCW" 102,
  cases in `runAuton()`; `imuCalibratedAtBoot()` in `main.cpp`);
  `tools/umbmark.py` + `tools/umbmark_selftest.py` (289 checks);
  `tools/sim/routes/umb_*.txt`, `tools/sim/umb_suite.py` (what each error looks
  like, figure `docs/sim/umb_signatures.png`), `tools/sim/umb_e2e.py` (simulator
  -> fake ruler measurements -> calculator -> apply -> re-drive; 59 pass, 0
  fail, 4 reported only in the full run; `--quick` fails 2 limits by chance with
  only 4 robots). Bench options added: `--imu-scale CW:CCW`, `--odom-offsets
  V:H`, world keys `VERT_YAW_DEG`, `HORIZ_YAW_DEG`. Off-pose equivalence with
  `dut_pregps` re-checked after the bench changes.
- **What the simulator says.** Scale found to within 0.18 points (centre mark)
  or 0.08 (two corners) at the usual slip. One round takes 20 uncalibrated
  robots from 2.07 to about 1.04 in at the end of the 60 s route, the same as
  a perfect scale. A closed square cannot see wheel size, wheel offsets (even
  the wrong sign) or a wheel off parallel: under 0.1 in at the end, though up
  to 8.9 in at the corners on the way round. Heading drift reads as a scale
  error of opposite sign each way; the tool refuses above 0.02 deg/s.
- **The limit that matters for this robot.** If it really spins about a point
  5.8 in behind its tracking centre (the open `[ODOMCAL]` question), the scale
  from position alone comes out about 27 % low with no warning. The heading is
  unaffected, so measure the heading too (corners form, or `heading_deg`).
- **History worth knowing.** The first build ran its moves blocking; the review
  found a stick push during a move did nothing and a mid-run sensor dropout
  drove blind. The routine was rewritten to run moves async and watch every
  10 ms. That rewrite was run on a scratch host build and read by the lead
  session, but not independently reviewed, and never on the robot: do the
  wheels-off-the-ground check in `docs/UMBMARK.md` section 7 first.
- **Not done.** `pidtuner.cpp` does not refuse RUN while a UMB run is going
  (the UMB run stops itself instead). The long Skills descriptions have not
  been seen on the brain's text box. Hot package is now 36.5 KB (was 32.7).

## Update: GPS fusion and wall sensors, 2026-11-22

Built overnight, in the simulator only. Nothing ran on the robot. Uncommitted
on `claude/serene-curie-yvp8mp` until the user says to commit. The short
version for the user is `docs/OVERNIGHT_REPORT_2026-11-22.md`.

- **What exists.** `src/lemlib/chassis/gpsFusion.cpp` (+ header): a filter that
  nudges x and y (never the heading) toward GPS readings, and, in the same
  filter, one-axis fixes from wall-facing distance sensors. Hook at the end of
  `lemlib::update()` in `odom.cpp`; `lemlib::setPose` restarts it;
  `lemlib::setImuScale`. Designs with every decision: `docs/GPS_FUSION_DESIGN.md`
  (through "Revision 3 results") and `docs/WALL_FIX_DESIGN.md` (through
  "Revision C"). Movement code was not touched.
- **Switches** (`include/robot-config.hpp`): `GPS_FUSION_MODE = Off`,
  `GPS_MOUNT_MEASURED = false`, `ODOMETRY_CALIBRATED = false`. With Off the
  pose is bit-for-bit the pre-GPS one (checked against `tools/sim/dut_pregps`).
  On gives Shadow until both flags are true. Routines call `startGpsFusion()`
  after `chassis.setPose(...)`. No wall sensor is fitted or configured.
- **Verdict.** GPS: Shadow (log only) at most; do not switch On. A healthy GPS
  adds nothing measurable on a calibrated robot, and a GPS drifting 0.1 in/s
  is followed up to 5.4 in with a fault in only 4 of 20 runs. Walls: the design
  is not ready (Revision C lists eight open problems); needs a real sensor
  measured first. **Calibrating odometry is the gain**: 0.9 in against 2.6 in
  at the end of a 60 s route in the simulator.
- **Tests.** `make -C tools/sim unit` (444 + 310 + 102 + 198 checks),
  `python3 tools/sim/gps_suite.py` (about 5.5 min), `python3 tools/sim/wall_suite.py`
  (about 3 min). Figures in `docs/sim/gps_*.png` and `wall_*.png`. The sensor
  models are built from other teams' reports, not from measurements.
- **Next upload re-sends the cold package** (3.7 MB, over the cable):
  `src/lemlib` changed.
- **Only a robot can check:** the filter's logging task (8 KB stack, priority
  below default, started from `initialize()` by `gpsfusion::configure`); what
  the GPS's `get_error()` does when the camera is covered (decides whether On
  is ever safe); GPS noise, delay and dead zone; the mount offsets.
- **Found in passing, not ours, not fixed:** in PROS 4.1 a competition task
  deleted in the middle of a `printf` leaves the serial lock held, and every
  later `printf` then waits for ever. Keep prints out of `autonomous()`; the
  driver loop's `[optical]` print would hang `opcontrol()` after that.
- **Still open from before:** odometry calibration (IMU scale by ten turns
  each way, 48 in tape test per tracking wheel), the horizontal wheel's offset
  sign, curve sway (turn kI), the PID tab's ODOMCAL using the raw IMU (it will
  not match a non-1 IMU scale), the selector's `tabWatcher` race.

## Update: first robot session, 2026-11-19

This section is newer than everything below it and wins where they disagree.

**Verified on the robot**
- Boots; `[PID] LVGL heap 49% used, biggest free block 16756 bytes` (not the 79% estimate).
- Left-side motor speed and voltage are sign-corrected (both sides positive on a straight drive).
- Horizontal tracking wheel is plugged in on port 3 and enabled; pushing the robot right raises X.
  Pushing 27 in forward moved X by 0.15 in; a full hand spin returned X, Y to the start. The wheel
  offsets (0.9 in, 2.55 in) are still the old robot's numbers and were not measured with a ruler.
- GPS sensor is on port 7 (recorded in `ports.hpp`, not read by any code yet).
- Uploads: the two screen images are in the cold package, a normal upload is about 32 KB.
- Log transfer: fine over the cable to the brain; through the controller's radio most runs lose
  rows (`Could not decode bytes` in the terminal) and the reader marks them DAMAGED. The brain's
  one-line `[PID] ...` summary still arrives.

**Gains saved in `robot-config.cpp` (tuned with the OLD library behaviour, see below)**
- Drive 12.5 / 0.1 / 100: 12 and 24 in within 0.25 in; 48 in overshot 0.6 to 2.0 in.
- Turn 4.8 / 0.25 / 42: 45, 90 and 165 deg all ended within about 1 deg, overshoot 0.6 deg or less.
  The + way ends about 0.5 deg shorter than the - way every time.

**The LemLib finding and fix (NOT yet run on the robot)**
- Recorded runs showed full drive voltage until 7.4 in from the target at any speed, then a step
  to braking; kD 115 was no better than 100. Cause: the rotation sensors refresh every 20 ms, the
  loops run every 10 ms, so `PID::update`'s one-sample derivative read 0, double, 0, double, and
  `moveToPoint`'s slew limit (active only outside its 7.5 in settle zone) flattened it.
- Fix, both in the library, no new parameters: `TrackingWheel::reset()` sets rotation sensors to
  5 ms; `PID::update` takes the derivative over two samples.
- LemLib now builds from `src/lemlib` into the cold package (see `CLAUDE.md`).
- `tools/sim/` is a host testbench that runs the real LemLib code against a robot model fitted to
  this day's logs (`tools/sim/README.md`). It reproduces the 48 in overshoot (1.48 in, robot 1.46)
  and with the fix gives 0.04 to 0.21 in from 12 to 72 in; turns change by under 0.1 deg. Graphs:
  `docs/sim/`. It does not model yaw, left/right asymmetry or the smaller overshoot in reverse.
- First thing to do with the robot: upload over the cable (cold package changed), run DRIVE 24 and
  48 in each way and one TURN 90 pair. Expect the reader's "stale position" flag to be gone and
  48 in to overshoot well under 0.5 in; if not, try kD 115. The drive gains may want a retune.

**Still open**
- During hard braking from a 48 in drive the robot yawed 3 to 6 deg; LemLib does not steer inside
  7.5 in. Not addressed. The fix should make braking gentler, which may shrink it.
- Tracking-wheel speed reads about 1.2x the speed computed from motor rpm with the configured
  2.75 in wheels at 450 rpm. One of those constants is probably wrong (harmless to LemLib while
  the tracking wheel and IMU are used). Tape test still wanted: push until Y reads 48, measure.
- Claw: with the claw open the sensor read 72 to 77 mm and every L1 press "caught" 0.3 s later
  (`CLAW_CATCH_MM` is 100). Nobody has said what was in the claw.
- `moveToPose`: speed cap `sqrt(horizontalDrift * radius * 9.8)` with drift 2 limits a 50 in
  radius arc to about 25% power and can stall short of the target; it only exits within 0.5 in.
  Read from the code, not tested. Raising `horizontalDrift` to about 8 is the first thing to try.
- Dormant LemLib bugs: `angleError` sanitises `target` twice and never `position` (forced-direction
  turns can over-rotate past 360 deg); async motions capture arguments by reference.
- The "no auton picked" controller rumble is switched off by a flag in `selector.cpp`.
- How the user works at the robot: they run `pros mu` and `pros terminal --output tune.log`
  themselves in their own terminal; do not open terminals or hold the serial port.

## What was done

1. **Reset.** The repo was returned to the start-of-season state. An earlier
   on-brain PID editor was reverted. The cleanup of the `boom` merge (which had
   been committed with conflict markers) was kept.
2. **New-robot config and driver controls**, ported from the user's local folder
   "Pepsi-Kola copy v2 2": `ports.hpp`, `robot-config`, `subsystems`, `macros`,
   `controls` are copies of that folder's files apart from reworded comments and
   removed dependencies. Last season's 19 autons and the old colour-sort stub were
   deleted (git history has them). The selector got that folder's red/blue
   alliance button. Each selector tab holds a "NONE" placeholder.
3. **Deliberately left out** of that folder, by the user's choice: its autons
   (`1ring`, `james67`, `tank` and mirrors), `autotune` (a relay auto-tuner),
   `tuning` (measured test routes), `diagnostics`, `devcmd`, `motion.cpp`, its own
   D-pad PID editor, its docs and tools, its LemLib edits. They still exist in
   that local folder if the user wants any of them back.
4. **PID tab** (`src/pidtuner.cpp`), **run logging** and the **log reader**
   (`tools/pidlog.py`): see `CLAUDE.md`.

## What has and has not been verified

| | Status |
|---|---|
| Builds clean | Yes |
| PID tab logic | 41 host checks against a simulated chassis, including the log blocks' row counts and checksums |
| Log reader | 26 synthetic runs, every verdict and every kind of damaged capture |
| Upload and boot on the robot | Yes (first power-on today) |
| Driver controls on the robot | Claw and colour sensor seen working in the terminal; the rest not yet reported |
| PID tab on the robot | Not yet run |
| `[PID] LVGL heap` boot line | Not yet seen (the terminal was attached after boot) |
| Log transfer over the serial link | Not yet seen. The first capture was through the controller's radio, the route most likely to drop lines |
| Sign of `get_voltage()` / `get_actual_velocity()` for the reversed left side | Unknown. If the left columns are mirrored on a straight drive, negate them in `testTask` |
| Reader thresholds and suggestion sizes | Calibrated on a simulated drivetrain only |

## Seen at first power-on

- With the claw open and apparently empty the claw sensor read 72-73 mm, and
  every L1 press reported "caught" at 76-84 mm about 0.3 s later. `CLAW_CATCH_MM`
  is 100. If nothing was in the claw, the threshold is above the empty reading
  and needs to come down. Ask the user what was in the claw.

## How to read a tuning run

The controller is LemLib's: `output = kP*error + kI*integral + kD*(error - previous error)`
per 10 ms tick, error in inches or degrees, output in motor power out of 127,
slew-limited, integral only inside the windup range and reset when the error
changes sign. A move ends when the error has stayed within `small` for its time,
or within `large` for its time, or at the timeout.

- **Overshoot, one swing:** more kD (15-40% depending on size).
- **Several swings:** less kP. If instead the voltage flips sign every few
  samples while the error barely moves, kD is amplifying sensor noise: less kD.
- **Stops short while a small voltage is still applied:** not enough push to
  beat friction. More kP, or a small kI with a windup range (the drive's is 3 in,
  the turn's 5 deg).
- **Ends while still creeping, more than the small range away:** the `large`
  exit fired (3 in / 500 ms). The approach is too slow at the end: less kD or
  more kP.
- **Heading drifts on a straight drive, or one side is consistently faster:**
  mechanical, not a gain problem.
- **Turns:** `TeamChassis::turnToHeading` adds a minimum speed of 51 to turns
  under 45 deg (LemLib then ends the turn the moment it crosses the target) and
  caps turns of 100 deg or more at speed 80. The angular gains were tuned with
  these rules in place, so a 30 deg test and a 90 deg test are different regimes.
- Judge a setting from two runs, one in each direction, and note the battery.

Why `moveToPose` is slow and stops short (the user asked): forward power is
capped each tick at `sqrt(horizontalDrift * radius * 9.8)`, and `horizontalDrift`
is 2, so a 50 in arc radius allows 31 out of 127; turning takes priority over
driving when the two saturate; and near the target the same cap can leave too
little power to move. Raising `horizontalDrift`, lowering `lead`, or using
`moveToPoint` then `turnToHeading` all help.

## Open questions for the user

1. Delete `firmware/LemLib.a` so `src/lemlib` is the code that runs? Recommended,
   together with removing the never-live edits in there so behaviour does not
   change. Not decided.
2. Did the old relay auto-tuner (`autotune.cpp` in the local folder) give good
   gains? If yes, bringing it back is cheaper than writing an automatic search.
3. Do the team's competition fields have GPS strips? (They own a GPS sensor.)
4. How far off is the robot at the end of a skills run?
5. Which undershoot do they see most with `moveToPose`: still creeping when the
   next step starts, stopped dead short, or stopped off to one side?
6. Can the horizontal tracking wheel be plugged in? It is switched off in
   `robot-config.cpp`; without it the robot cannot see sideways motion.

## Measurements still wanted from the robot

- The `[PID] LVGL heap` boot line.
- Y on the Developer tab after pushing the robot exactly 48 in (tracking wheel diameter).
- Theta after 10 full turns by hand (should be 3600; IMU scale).
- The vertical tracking wheel's offset from the centre of rotation, by ruler
  (the 0.9 in in the config came from the old robot).
- Repeatability: how far from the tape after 10 out-and-back 48 in drives and
  after 10 left-right 90 deg turns.

## What was planned next, in the order discussed

1. **Today:** tune TURN then DRIVE on the PID tab, using the logs.
2. **An AUTO button on the PID tab:** an automatic search over kP and kD using
   the tab's own measurements. Proposed, not started; depends on question 2.
3. **A motion layer on top of LemLib's odometry** (new functions beside the
   LemLib ones, so autons can mix them): planned speed curves with feedforward
   for drives and turns first, then curve-following pose moves, with the
   feedforward constants coming from a short self-characterisation routine. The
   user agreed in principle; nothing is written.
4. **Localization:** calibrate first (the measurements above), then fuse
   odometry with an absolute reference in a Kalman filter. GPS if fields have
   strips; distance sensors against the walls work everywhere. Nothing is written.
5. **A simulator fitted from logs:** the PID tab logs (voltage in, speed out,
   every 10 ms) are the first data for it. `tools/pidlog_selftest.py` already
   holds a small drivetrain-plus-LemLib-PID simulation that can be the seed.
6. **A neural-network gain adapter trained in simulation:** the user's idea.
   Parked until steps 3-5 show how much variation is left; a fitted lightweight
   simulator was recommended over Isaac Sim for training.

## Things the user has said about how they want to work

- Cost matters: use the strongest model for the hardest, most critical code and
  cheaper models or subagents for routine implementation.
- Simplicity of use beats features for anything used at the robot.
- They capture robot output with `pros terminal`.
