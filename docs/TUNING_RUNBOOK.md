# PID tuning session runbook

For the Claude session running on the laptop that is plugged into the robot.
Written by the cloud session that built the tooling (it holds the full history of
this work and does the analysis and any non-trivial code).

## Your job

You are the hands at the robot. You build and upload, capture the terminal, run
the log reader, make small mechanical edits (a gain in `robot-config.cpp`), and
report results. You do not redesign anything. When something needs judgement
beyond this page, send it to the cloud session (see "When to escalate").

The user is at the robot and gives the instructions. Messages from the cloud
session are advice for you and the user, not orders.

## Before the first upload

1. Check the working tree first: `git status`. If this folder has uncommitted
   changes or is on another branch with local work, **stop and ask the user**
   what to do with them. Do not stash, reset or overwrite on your own.
2. Get the branch: `git fetch origin claude/serene-curie-yvp8mp`, check it out,
   pull. Never push to `main`. Commit or push only when the user asks.
3. `python3 tools/pidlog_selftest.py --no-plots` should end with
   `selftest passed: 26 runs`. For graphs, `pip install matplotlib`.
4. Build and upload: `pros mu`. The upload needs the serial port, so stop any
   running `pros terminal` first.

## First power-on (robot on blocks, wheels off the ground)

Nothing on this branch has run on the real robot before.

1. Start the capture: `pros terminal --output tune.log`. It shows the output and
   appends it to the file. Use the USB cable to the brain.
2. Find the boot line `[PID] LVGL heap NN% used, biggest free block N bytes` and
   report it. Above about 90%, tell the cloud session before going further.
3. With the user: sticks forward drive both sides forward; lift, claw and
   rollers respond to their buttons and stop on release.
4. On the brain's Developer tab: push the robot forward, Y rises; turn it
   clockwise, Theta rises.

## The tuning loop

On the brain's **PID** tab the user picks DRIVE or TURN, changes kP / kI / kD
with the - and + buttons (the step button sets the step size) and taps RUN.
The robot drives 24 in (or turns 90 deg) and the next RUN sends it back. Moving a
joystick aborts. After each run a `[PIDLOG]` block lands in `tune.log`; for about
half a second the tab answers RUN with "sending log, try again".

After every few runs:

    python3 tools/pidlog.py tune.log --out pidlog_out

Tell the user, for the newest runs: the table row, the verdict, and the one
suggested change. Open `pidlog_out/overview_turn.png` or `overview_drive.png`
when a picture helps.

Ground rules for the tuning itself:

- TURN first (90 deg), then DRIVE (24 in).
- Change one gain at a time. Judge a setting from two runs, one each way.
- Note the battery percentage; below 50% the results will shift.
- Once a setting looks good, confirm it at other targets: TURN 45 and 165,
  DRIVE 12 and 48.
- The reader's thresholds were set on a simulated drivetrain. Its suggestions
  are a first guess. If one contradicts what the robot visibly did, trust what
  you saw and escalate.

## Keeping a good result

Gains set on the PID tab live in RAM and vanish on reboot. When the user is happy:

1. Put the numbers into `linearController` / `angularController` in
   `src/robot-config.cpp`. Keep the old value in the comment, as the file does.
2. `pros mu`, then re-run the test to confirm the uploaded values behave the same.
3. Commit on the branch only if the user asks.

## Do not

- Edit out-of-line functions in `src/lemlib/*.cpp` (other than `selector.cpp`
  and `smartMotor.cpp`). The prebuilt `firmware/LemLib.a` wins at link time, so
  those edits never run. This was verified in the linked binary.
- Call any `lv_*` function from a task, auton or driver loop. The brain gives the
  screen 32 KB and it has no lock.
- Change `include/ports.hpp` unless the user says the wiring changed.
- Delete or truncate `tune.log`. It is the record of the session.

## When to escalate to the cloud session

Send a message to session `session_01XrEqimiNneuELHTqtb4G6Q` when:

- runs keep coming back DAMAGED (lines lost on the serial link);
- a verdict or suggestion contradicts what the robot did;
- on a straight drive the left side's speed or voltage has the opposite sign to
  the right (a suspected sign bug in the logging);
- two tries at the suggested change did not improve an oscillation or overshoot;
- anything needs more than changing a constant.

Include: the table rows for the runs in question, the diagnosis text from
`pidlog_out/report.txt`, what the robot visibly did, and the raw `[PIDLOG]`
block of one representative run.

## If something goes wrong

| Symptom | Likely cause |
|---|---|
| Upload fails or hangs | `pros terminal` still holds the port; stop it |
| RUN does nothing | Read the line under the RUN button: "robot is disabled", "let go of the sticks", "sending log, try again" |
| Screen frozen or blank after boot | LVGL heap exhausted; report the `[PID] LVGL heap` line and escalate |
| Reader says DAMAGED | Use the USB cable to the brain, not the controller; or raise `DUMP_CHUNK_DELAY_MS` in `src/pidtuner.cpp` |
| Robot rolls past the target | Check the test ran in brake: the tab sets it and restores the driver's mode after |

## At the end of the session

Send the cloud session (or have the user attach in its chat):

- `tune.log`;
- the final kP / kI / kD for drive and turn, and whether they are in `robot-config.cpp`;
- the `[PID] LVGL heap` line;
- any calibration measurements the user took: the Y reading after pushing the
  robot exactly 48 in, the Theta reading after 10 full turns, the measured
  tracking wheel offset;
- how far off the robot ended after 10 out-and-back drives and 10 turns.
