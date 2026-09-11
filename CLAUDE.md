# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

VEX V5 competition robot code for team 1831D (2025-26 Push Back), built on PROS 4.1.0 with
LemLib 0.5.2 and LVGL 8.3.8. C++20, cross-compiled for ARM Cortex-A9.

There is no test suite and there cannot be a meaningful one — every code path ends at
hardware. `make` compiling is the only automated check. Everything else is verified by
downloading to a brain and driving the robot.

## Commands

```bash
make                       # build (default goal `quick`); ALWAYS run this after editing
make clean                 # wipe bin/ and .d/
pros mu                    # make + upload to a connected brain
pros mut                   # make + upload + open the terminal (printf output lands here)
pros terminal              # attach to a running brain's stdout
pros build-compile-commands  # regenerate compile_commands.json for clangd (gitignored, local-only)
```

The link step always emits `warning: ... has a LOAD segment with RWX permissions`. That is
normal for PROS hot/cold linking and is not something to fix. The `pros` CLI also spams
`SSLError ... urlopen` retries — that is its telemetry failing to phone home; ignore it.
(`pros --help` itself is broken on this machine's install, a `rich_click` version clash. The
subcommands work.)

`USE_PACKAGE:=1` in the Makefile enables hot/cold linking: LemLib/libc/liblvgl/libpros go in
the cold image, your code in the hot image. Only the hot image re-uploads on a normal change,
which is why uploads are fast. Header dependencies are tracked in `.d/`, so a plain `make`
after editing a header is enough — but note that touching a widely-included header like
`include/lemlib/pid.hpp` rebuilds ~40 files.

## Architecture

### Hardware is declared in exactly one place

`src/robot-config.cpp` defines every motor, pneumatic, and sensor as a global; `include/robot-config.hpp`
externs them. Every auton and every control loop reaches these globals directly — there is no
dependency injection, no subsystem classes. Changing a port is a one-line edit there.

The current robot: a 4-motor drive (left 6, 7 reversed; right 8, 9), blue cartridges geared to
450 rpm on 2.75" omnis; an IMU on port 2 and no tracking wheels, so LemLib measures distance off
the drive motor encoders; and `intakeTop` 16, `intakeBottom` 15, `cascade` 10 + 21 (21 reversed),
`rollerTurn` 5, `rollerMain` 4, all green. The ports came from the team's VEXcode config. The
drivetrain's track width is still a placeholder until it is measured.

Declare each physical motor exactly once. Last season's config put the same ports in both a
`MotorGroup` and individual `Motor`s with conflicting directions and gearsets, and the two
drove the hardware differently depending on which name the calling code happened to use.

### LemLib is vendored, not a black box

`src/lemlib/` and `include/lemlib/` hold full LemLib source, and this team has modified it —
`pid.hpp`/`pid.cpp` gained gain accessors for the on-brain tuner, and `selector.cpp` is
substantially custom. Edits there take effect: the hot-image objects override the symbols in
`firmware/LemLib.a`. Treat `src/lemlib/selector.cpp` as first-party code. Do not "update LemLib"
without accounting for these local changes.

### Auton dispatch: a four-file contract

Selecting and running an auton spans four places, and they are coupled by an index scheme that
is easy to break silently.

1. `src/main.cpp` `initialize()` declares the button labels (`qualBtnmMap`, `AIBtnmMap`,
   `skillsBtnmMap`) and their descriptions, then hands them to `lemlib::selector::init()`.
2. `src/lemlib/selector.cpp` builds the LVGL tabs and, on a button press, resolves the label
   back to an integer stored in `lemlib::selector::auton`.
3. `src/main.cpp` `autonomous()` switches on that integer.
4. `src/autons/*.cpp` hold one routine per file, declared in `include/autons.hpp`.

The integer ranges are: **1..100 qual, -1..-100 AI, 101..201 skills, 999 = none** (falls through
to `auton1ring()`).

The index math has two traps:

- **The map array includes `"\n"` row separators; the description array does not.** The routine
  number comes from the position in the *map* (`auton = i > 3 ? i : i + 1`), while the on-screen
  description is indexed by LVGL's *button id*, which skips newlines. The `i > 3` fudge only
  produces 1..6 because there is exactly one `"\n"` and it sits at index 3. Change the row
  layout and the wrong routine runs, with no compile error. Adjust the offset to match.
- **Buttons are matched by `strcmp` on their label.** Two buttons with the same text in one tab
  will always run the first one's routine.

Adding an auton therefore means: new file in `src/autons/`, declaration in `autons.hpp`, `case`
in `autonomous()`, *and* a label plus a description entry in `initialize()` — with the array
positions checked against the offset rule above.

### Teleop runs as two independent tasks

`opcontrol()` starts `drivetrain_control` and `teleop_loop` (`src/controls.cpp`) as separate
`pros::Task`s, both looping at 10ms. The first owns the joysticks and the arcade/tank toggle
(all four shoulder buttons at once); the second owns the mechanisms. They never communicate.
Keep them non-blocking. `teleop_loop` has no mechanism bindings yet — last season's were
retired along with its robot.

`drivetrain_control` stops writing the drive while `autotune::isRelayActive()` — the relay drives
the motors itself, and `chassis.arcade` writes them on every call, even `move(0)` at centered
sticks. Anything else that needs to own the drivetrain during opcontrol needs the same yield.

### The Developer tab owns the D-pad

`src/pidtuner.cpp` (gain editor) and `src/autotune.cpp` (relay auto-tuner wizard) both read the
D-pad. `tabWatcher` in `selector.cpp` arbitrates: it activates the tuner only while tab 0 is
showing and gives the autotune wizard priority over it. `teleop_loop` guards each of its own
D-pad branches with `!pidtuner::isActive() && !autotune::isRunning()`. **Any new D-pad binding
in teleop needs the same guard**, or it will fire while someone is editing gains.

Tuned gains live in RAM only. They reset to the constants in `robot-config.cpp` on reboot, so a
good tune has to be transcribed back into source by hand. (Persisting them to the SD card is a
known wishlist item.)

### Screen: LVGL, never LLEMU

`pros::lcd::*` is unusable here — LVGL owns the display. Use LVGL widgets, or `printf` to the
terminal for debugging. Note that `printf` in a 10ms loop floods `pros terminal`; gate it.

## Conventions

- Autons call LemLib motions with `async = false` (the final argument) so they run in sequence.
  Passing `true` without a matching `chassis.waitUntilDone()` is almost always a bug.
- Every auton starts with `chassis.setPose(...)` and `chassis.setBrakeMode(...)`.
- Last season's autons, teleop bindings and colour sorter were deleted when the new robot's
  config went in. They are in git history (before the commit that added the VEXcode ports) if a
  routine is worth porting. `auton1ring()` is the only routine left: the empty fallback.
