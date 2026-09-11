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

Two robots share this file via commented-out blocks ("Gavin's Robot" / "Quinn's Robot"). Swapping
robots means uncommenting one block and commenting the other, in several places. Verify all of
them are consistent before building for a different chassis.

**Ports 9, 17, and 19 are currently declared twice** — once inside the `intake` MotorGroup and
once as the individual `ramp`/`top1`/`top2` motors, with a *conflicting direction on port 17 and
a conflicting gearset on port 9*. Teleop drives the individual motors; many autons drive the
`intake` group. They disagree. Do not add new call sites against `intake` — use
`ramp`/`top1`/`top2`, and treat consolidating the two onto one representation as a pending fix.

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
(all four shoulder buttons at once); the second owns the intake and pneumatics. They never
communicate. Keep them non-blocking — note that the L1/R1 handlers deliberately spin-wait on
button release to make lift and score distinct presses.

`teleop_loop` carries one piece of state, `is_heaven_up`. Any code path that moves the `heaven`
pneumatic must update it or the lift/score toggle desynchronizes from the hardware.

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
- `include/colorsort.hpp` / `src/colorsort.cpp` are a dormant skeleton from a previous season —
  the optical sensor is commented out and `get_hue()` is stubbed to a constant. Nothing
  instantiates it. Either wire it up properly or leave it alone; don't half-use it.
