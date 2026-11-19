# CLAUDE.md

Guidance for Claude sessions working in this repository. Read `docs/HANDOFF.md`
for the current state of the work and what is still open.

## What this is

Competition code for the team's VEX V5 robot: PROS kernel 4.1.0, LemLib 0.5.2
(vendored as source), LVGL 8.3 for the brain screen. C++20, cross-compiled for
ARM. There is no way to test on hardware from a laptop or the cloud: `make`
compiling is the only automatic check, and everything that moves has to be
confirmed on the robot.

## Commands

    make -j8                              # build
    pros mu                               # build + upload (stop `pros terminal` first: same serial port)
    pros terminal --output tune.log       # show the robot's output and append it to a file
    python3 tools/pidlog.py tune.log --out pidlog_out   # read PID tab logs (plots need matplotlib)
    python3 tools/pidlog_selftest.py      # the reader's own check: "selftest passed: 26 runs"

`pros` with no subcommand crashes on this laptop (a rich_click version clash) and
prints SSL/analytics warnings after every command. Both are harmless.

## Where things live

| Layer | Files |
|---|---|
| Wiring: every port number, with a compile-time duplicate check | `include/ports.hpp` |
| Devices, drivetrain geometry, odometry, PID gains, exit timings, `TeamChassis` | `src/robot-config.cpp`, `include/robot-config.hpp` |
| Mechanism commands and named positions, colour toggle | `src/subsystems.cpp` |
| Multi-step driver macros (one task, one macro at a time) | `src/macros.cpp` |
| Driver-control loop and button map | `src/controls.cpp` |
| Start-up order, `runAuton()`, competition callbacks | `src/main.cpp` |
| Auton selector and Developer tab (team-written, lives under lemlib/) | `src/lemlib/selector.cpp` |
| PID tab: live gain editing, test moves, run logging | `src/pidtuner.cpp` |
| Laptop tools | `tools/pidlog.py`, `tools/pidlog_selftest.py` |

## Traps that have already cost time

**The prebuilt LemLib wins at link time.** `firmware/LemLib.a` is linked alongside
`src/lemlib`. For every function present in both, the linker uses the prebuilt
one, so edits to out-of-line functions in `src/lemlib/**/*.cpp` never run
(checked in the linked binary: the symbols resolve to the cold image and the
edited code's strings are absent). `selector.cpp` and `smartMotor.cpp` are not in
the archive and are live. Change LemLib behaviour only through headers (inline
code), or in team files: that is why `PID::setGains` is inline in
`include/lemlib/pid.hpp` and the turn rules live in `TeamChassis`. Do not change
the layout of a LemLib class the prebuilt code reads. Deleting `firmware/LemLib.a`
makes `src/lemlib` live and builds cleanly, but it also switches on old edits in
there that have never run (a `leftPower*0.9` in `moveToPose` and `arcade`, debug
prints in the motion loops). The user has not decided on this yet.

**The screen has 32 KB and no lock.** LVGL's heap on the brain is 32 KB and the
selector plus the PID tab use most of it (estimated 79%; the real figure prints at
boot as `[PID] LVGL heap NN% used`). Running out hangs the display. Prefer one
`lv_btnmatrix` over many buttons, static arrays over heap ones, shared styles.
PROS runs LVGL on its own task without a lock: call `lv_*` only from LVGL event
callbacks, an `lv_timer`, or `initialize()`. The PID tab follows this. The
selector's `tabWatcher` does not (it is a separate task that calls `lv_*`), which
is an existing race nobody has fixed.

**Selector numbering.** `auton` is 1.. for Qual, -1.. for AI, 101.. for Skills,
999 for none. The button handlers in `selector.cpp` map button text to a number
with an `i > 3` adjustment that assumes a `"\n"` row break after the third
button; adding buttons in another shape needs that code checked. While `auton` is
999 the controller rumbles every 10 ms.

**Driver loop rules.** `driverControl()` runs in the opcontrol task and never
blocks. Anything with a wait is a macro. Do not spawn loops from `opcontrol()` or
`autonomous()`: PROS kills those tasks on a mode change but not their children.
Anything that drives the drivetrain during driver control (the PID tab's test
move) needs `drive()` to yield, as it does for `pidtuner::isRunning()`.

**Printing.** A `printf` in a 10 ms loop floods the terminal. The colour toggle
already prints `[optical]` twice a second.

## The PID tab and its log

The tab edits `chassis.lateralPID` / `chassis.angularPID` live (RAM only) and
mirrors the values into `linearController` / `angularController` for the
Developer tab. RUN starts a test task: it sets the drive to BRAKE, runs
`moveToPoint` or `TeamChassis::turnToHeading` asynchronously, samples every 10 ms
until the move ends plus 300 ms, restores the brake mode, and aborts if a stick
moves. Then it prints the run:

    [PIDLOG] BEGIN v=1 run=N mode=DRIVE|TURN target=<signed> unit=in|deg kP= kI= kD= windup= slew=
             small=<range>/<ms> large=<range>/<ms> timeout= batt_mV= batt_pct= dt_ms=10
             ips_per_rpm= track= cols=t,err,dev,vL,vR,uL,uR        (one line)
    ~t,err,dev,vL,vR,uL,uR                                         (one row per sample)
    [PIDLOG] END run=N rows= sum= end_ms= result=ok|aborted:stick|aborted:stop|buffer final= over= settle_ms=

Rows are integers: `t` ms; `err` hundredths of in or deg, positive = short of the
target; `dev` hundredths (DRIVE: heading off the start heading in deg; TURN: how
far the robot's centre wandered, in); `vL`,`vR` motor rpm; `uL`,`uR` motor mV.
`sum` is every integer in every row added up, wrapped to 32 bits. `end_ms` is
when LemLib ended the move (-1 with `result=buffer`: the move outlasted the 5 s
recording). `tools/pidlog.py` checks rows and checksum and marks a run DAMAGED if
lines were lost.

## Working rules

- Work on `claude/serene-curie-yvp8mp`. Never push to `main`. Commit or push when
  the user asks.
- Build after every edit. Say plainly what was only compiled and what ran on the
  robot.
- Port numbers go in `ports.hpp` only; tuning numbers are named constants.
- Keep the team's comments that record what was tried and why.
