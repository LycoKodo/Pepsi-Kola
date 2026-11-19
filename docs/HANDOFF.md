# Handoff: state of the work

Written by the cloud session that did the work below, so another session can take
over with the user directly. `CLAUDE.md` has the durable repo knowledge;
`docs/TUNING_RUNBOOK.md` has the step-by-step for a tuning session. This file is
the rest: what was done, what is known, what is undecided, and what was planned.

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
