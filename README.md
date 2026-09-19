<div align="center">

# 1831E — TimmyTammies

**VEX V5 competition code · 2026–27 Override**

PROS 4.1.0 · LemLib 0.5.2 (vendored) · LVGL 8.3.8 · C++20 · ARM Cortex-A9

</div>

---

## Quick start

```bash
make            # build (default goal: quick)
pros mu         # build + upload to a connected brain
pros mut        # build + upload + open the terminal (printf lands here)
pros terminal   # attach to a running brain's stdout
```

There is no test suite, and there cannot be a meaningful one — every code path
ends at hardware. `make` compiling is the only automated check. Everything else
is verified by downloading to a brain and driving the robot.

Two bits of noise are expected and are **not** bugs:

| You'll see | Why |
| --- | --- |
| `warning: ... has a LOAD segment with RWX permissions` | normal for PROS hot/cold linking |
| `SSLError ... urlopen` retries from the `pros` CLI | its telemetry failing to phone home |

Hot/cold linking (`USE_PACKAGE:=1`) puts LemLib, libc, liblvgl and libpros in the
cold image and your code in the hot image, so only the hot image re-uploads on a
normal change. Header deps are tracked in `.d/` — a plain `make` after editing a
header is enough, but touching something widely included like
`include/lemlib/pid.hpp` rebuilds ~40 files.

---

## Layout

```
src/
  robot-config.cpp    every motor, pneumatic and sensor — the only place hardware is declared
  main.cpp            initialize / autonomous / opcontrol, selector button maps
  controls.cpp        the two teleop tasks: drivetrain_control + teleop_loop
  pidtuner.cpp        on-brain PID gain editor (Developer tab)
  autotune.cpp        relay auto-tuner wizard (Åström–Hägglund)
  autons/             one routine per file
  lemlib/             vendored LemLib — modified, see below
include/              mirrors src/
```

---

## Driver controls

Bound in [`src/controls.cpp`](src/controls.cpp), ported from the team's VEXcode
program. The drive is **tank only**; an arcade version sits commented out in
`drivetrain_control()` as a drop-in backup.

| Input | Action |
| --- | --- |
| Left + right stick Y | drive (tank) |
| **A** | cascade up one level, wrapping 1→2→3→1 |
| **B** | cascade *and* `rollerTurn` back to resting |
| **X** | swing `rollerTurn` out (and creep `rollerMain` forward at 20%), or bring it home |
| **Y** (hold) | run the cascade forward |
| **R1** / **L1** (hold) | intake in |
| **L2** (hold) | intake out |
| **R2** (hold) | intake out; on release, burp the cascade forward for 1 s |
| **D-pad ↑** (hold) | raise the cascade, up to +2900° from resting |
| **D-pad ↓** (hold) | lower the cascade, down to resting |
| **D-pad ←** | toggle the intake lift pneumatic |
| **D-pad →** | send `rollerTurn` and the cascade home |

Cascade and `rollerTurn` targets are offsets from wherever those mechanisms were
resting at boot — `mechanisms_init()` captures that in `initialize()`.

---

## Things that will bite you

### Hardware is declared in exactly one place

`src/robot-config.cpp` defines every motor, pneumatic and sensor as a global;
`include/robot-config.hpp` externs them. Every auton and control loop reaches
these globals directly — no dependency injection, no subsystem classes. Changing
a port is a one-line edit.

**Declare each physical motor exactly once.** Last season's config put the same
ports in both a `MotorGroup` and individual `Motor`s with conflicting directions
and gearsets, and the two drove the hardware differently depending on which name
the calling code happened to use.

### LemLib is vendored and modified

`src/lemlib/` and `include/lemlib/` hold full LemLib source, and this team has
changed it: `pid.hpp`/`pid.cpp` gained gain accessors for the on-brain tuner, and
`selector.cpp` is substantially custom. Those edits take effect — the hot-image
objects override the symbols in `firmware/LemLib.a`. Treat
`src/lemlib/selector.cpp` as first-party code, and don't "update LemLib" without
accounting for the local changes.

### Adding an auton touches four files

Selection and dispatch are coupled by an index scheme that breaks silently:

1. `initialize()` in `main.cpp` — button labels + descriptions → `selector::init()`
2. `src/lemlib/selector.cpp` — builds the tabs, resolves a label back to an int
3. `autonomous()` in `main.cpp` — switches on that int
4. `src/autons/*.cpp` — one routine per file, declared in `include/autons.hpp`

Ranges: **1..100 qual · −1..−100 AI · 101..201 skills · 999 = none** (falls
through to `auton1ring()`, which does nothing).

Right now there is one real routine: qual 1, `autonVexcode()` in
[`src/autons/vexcode.cpp`](src/autons/vexcode.cpp) — the routine ported from the
VEXcode program. LemLib has no relative `drive_for`, so that file carries a
`driveFor()` helper that projects a distance along the current heading.

Two traps:

- **The map array includes `"\n"` row separators; the description array does
  not.** The routine number comes from the position in the *map*
  (`auton = i > 3 ? i : i + 1`), while the description is indexed by LVGL's
  *button id*, which skips newlines. The `i > 3` fudge only yields 1..6 because
  there is exactly one `"\n"` and it sits at index 3. Change the row layout and
  the wrong routine runs, with no compile error.
- **Buttons are matched by `strcmp` on their label.** Two buttons with the same
  text in one tab will always run the first one's routine.

### Teleop is two independent tasks

`opcontrol()` starts `drivetrain_control` and `teleop_loop` as separate
`pros::Task`s, both at 10 ms. The first owns the joysticks, the second owns the
mechanisms. They never communicate. Keep both
non-blocking.

`drivetrain_control` stops writing the drive while `autotune::isRelayActive()` —
the relay drives the motors itself, and `chassis.tank` writes them on *every*
call, even `move(0)` at centered sticks. Anything else that wants the drivetrain
during opcontrol needs the same yield.

### The Developer tab owns the D-pad

`pidtuner.cpp` and `autotune.cpp` both read the D-pad; `tabWatcher` in
`selector.cpp` arbitrates, activating the tuner only while tab 0 is showing and
giving the autotune wizard priority. **Any new D-pad binding in teleop needs**

```cpp
!pidtuner::isActive() && !autotune::isRunning()
```

or it fires while someone is editing gains.

Tuned gains live in RAM only. They reset to the constants in `robot-config.cpp`
on reboot, so a good tune has to be transcribed back into source by hand.

### Screen: LVGL, never LLEMU

`pros::lcd::*` is unusable here — LVGL owns the display. Use LVGL widgets, or
`printf` to the terminal. Note that `printf` in a 10 ms loop floods
`pros terminal`; gate it.

---

## Conventions

- Autons call LemLib motions with `async = false` (the final argument) so they
  run in sequence. Passing `true` without a matching `chassis.waitUntilDone()`
  is almost always a bug.
- Every auton starts with `chassis.setPose(...)` and `chassis.setBrakeMode(...)`.
- Last season's autons, teleop bindings and colour sorter were deleted when the
  new robot's config went in. They're in git history (before the commit that
  added the VEXcode ports) if a routine is worth porting.

---

<div align="center">
<sub>Powered by a project manager and two Dorito fans.</sub>
</div>
