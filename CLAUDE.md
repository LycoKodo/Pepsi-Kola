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
    python3 tools/umbmark.py FILE.csv --log umb_round1.log   # UMBmark square: IMU scales from ruler measurements
    python3 tools/umbmark_selftest.py     # its own check: "selftest passed: 289 checks"

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
| GPS and wall-sensor fusion (Off by default; hook at the end of `lemlib::update()`), IMU scale | `src/lemlib/chassis/gpsFusion.cpp`, `include/lemlib/chassis/gpsFusion.hpp`, `src/lemlib/chassis/odom.cpp`; designs in `docs/GPS_FUSION_DESIGN.md`, `docs/WALL_FIX_DESIGN.md` |
| UMBmark square, the odometry test: Tests tab buttons "UMB CW" (-1) and "UMB CCW" (-2); how-to in `docs/UMBMARK.md` | `src/umbmark.cpp` |
| Inertial sensor drift watch (boot check, standing-still warning, refuses test runs) and the guided sensor test: Tests tab button "IMU TEST" (-3); how-to in `docs/IMU_TEST.md` | `src/imuwatch.cpp`, `src/imutest.cpp` |
| SHELVED, switched off by `ctrlselect::ENABLED = false` in `include/autons.hpp` (never run on the robot; needs a field controller to try): picking the routine on the controller (hold A + LEFT/RIGHT; LEFT/RIGHT alone while disabled); uses the selector's routine list | `src/ctrlselect.cpp`, `selectRoutine()` in `src/lemlib/selector.cpp` |
| Laptop tools | `tools/pidlog.py`, `tools/pidlog_selftest.py`, `tools/umbmark.py`, `tools/umbmark_selftest.py` |
| Host simulator (real LemLib on the laptop), fusion unit tests and suites | `tools/sim/` (`make -C tools/sim unit`, `gps_suite.py`, `wall_suite.py`, `slip_sweep.py`, `umb_suite.py`, `umb_e2e.py`; use `/opt/anaconda3/bin/python3`, the system one has no numpy) |
| Screen images (boot logo, selector picture), linked into the COLD package so normal uploads stay ~32 KB; see the bottom of `Makefile` | `cold/image.c`, `cold/dorito.c` |

## Traps that have already cost time

**LemLib runs from source, in the cold package (since 2026-11-19).** Until then
the prebuilt `firmware/LemLib.a` was linked alongside `src/lemlib` and won for
every function in both, so edits in `src/lemlib` never ran. It is no longer
linked (kept, unused, as `firmware/prebuilt/LemLib.a.unused`). The bottom of
`Makefile` compiles `src/lemlib/**` into `firmware/liblemlib.a` and links it into
the cold package; `selector.cpp` and `smartMotor.cpp` are team code and stay in
the hot program. Consequences: an edit under `src/lemlib` or `include/lemlib` now
takes effect, and the next upload re-sends the 3.7 MB cold package (do it over the
cable to the brain; normal uploads are about 32 KB). When the switch was made,
the team edits that had never run were removed from the paths that run (debug
prints in `moveToPoint`/`turnToHeading`, `leftPower*0.9` and `distTarget < 1.5`
in `moveToPose`, the "AAA" controller print in `calibrate`); `arcade`'s
`leftPower*0.9`, `curvature`'s `turn *= 0.95` and `variableCurvature` are still
there and would run if called (the driver loop uses `chassis.tank`). Two library
changes were made the same day: `TrackingWheel::reset()` sets rotation sensors to
a 5 ms refresh, and `PID::update` takes its derivative over two samples. On
2026-11-20: the first `PID::update` after a reset has no derivative (it used to
read a jump from 0, a full-power twitch at the start of motions), and
`moveToPose` resets its angular PID and ramps the angular power (the angular
slew) once inside its 7.5 in settle zone (the heading target jumps there and the
robot stopped and lurched). Test library changes in `tools/sim/` before the robot.

**The screen has 32 KB and no lock.** LVGL's heap on the brain is 32 KB and the
selector plus the PID tab use most of it (estimated 79%; the real figure prints at
boot as `[PID] LVGL heap NN% used`). Running out hangs the display. Prefer one
`lv_btnmatrix` over many buttons, static arrays over heap ones, shared styles.
PROS runs LVGL on its own task without a lock: call `lv_*` only from LVGL event
callbacks, an `lv_timer`, or `initialize()`. The PID tab follows this, and so
does the selector's `tabWatcher`, a 50 ms `lv_timer` since 2026-11-20/23. As its own
task it rewrote the Developer tab's text every 10 ms with no lock: the "prefetch
error" crashes, and DATA ABORT with CURRENT TASK: tabWatcher.

**Selector numbering.** `auton` is 1.. for Qual, -1.. for the Tests tab (the old AI tab; the code
still calls it AI), 101.. for Skills,
999 for none. The button handlers in `selector.cpp` map button text to a number
with an `i > 3` adjustment that assumes a `"\n"` row break after the third
button; adding buttons in another shape needs that code checked. While `auton` is
999 the controller rumbles every 10 ms.

**Driver loop rules.** `driverControl()` runs in the opcontrol task and never
blocks. Anything with a wait is a macro. Do not spawn loops from `opcontrol()` or
`autonomous()`: PROS kills those tasks on a mode change but not their children.
Anything that drives the drivetrain during driver control (the PID tab's test
move) needs `drive()` to yield, as it does for `pidtuner::isRunning()`.

**Fusion stays Off unless the user asks.** `GPS_FUSION_MODE` is Off and must
keep the pose bit-for-bit what `tools/sim/dut_pregps` gives
(`make -C tools/sim` then compare with `--pose-trace`). On is refused (Shadow
instead) until `GPS_MOUNT_MEASURED` and `ODOMETRY_CALIBRATED` are true. The
odometry task never prints: the fusion code's own logging task does. Run both
suites after any change to `gpsFusion.cpp`.

**The UMBmark square (`src/umbmark.cpp`) is a test run only.** It refuses under
field control or a competition switch, with the lift up, or with the inertial
sensor failed. Its moves run async in LemLib's own task while the routine
watches the sticks, the sensor and the PID tab every 10 ms and cancels on any of
them; it prints one line per run. Its constants (side 48, speeds 70, waits) are
mirrored in `tools/sim/routes/umb_*.txt`, `umb_suite.py` and `umb_e2e.py`, which
check they match: change them together. `tools/umbmark.py` exits 3 and prints no
constants when its checks do not agree (reasons at the top of the report); that
is by design, not a failure. A closed square cannot see tracking wheel size or
offsets, and reads the scale about a quarter low from position alone if the
robot spins about a point behind its tracking centre.

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
