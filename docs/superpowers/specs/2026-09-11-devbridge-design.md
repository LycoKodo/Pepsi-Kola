# devbridge — host↔brain development bridge

**Status:** approved design, revised after adversarial review, not yet implemented
**Date:** 2026-09-11

## Problem

Every development loop against this robot currently runs through the brain screen and a
rebuild. Three specific costs:

1. **Gains do not round-trip.** `pidtuner` and `autotune` produce gains that live in RAM only.
   A good tune is read off a 480×240 screen and hand-typed back into `robot-config.cpp`.
   Transcription happens at a competition, under time pressure, and a wrong digit is silent.
2. **Port discovery is manual.** Bringing up a new chassis means reading port assignments off
   the brain's device screen and typing them into `robot-config.cpp`.
3. **No observability.** Diagnosing a motion that overshoots means inferring the error curve
   from watching the robot, because the only channel out is `printf` into a terminal nobody
   is reading.

An assistant working on this codebase cannot see the robot at all. Everything it knows is
inferred from source.

## Goals

A tethered, bidirectional command channel over the existing USB serial link, usable by both a
human at a terminal and an assistant via shell commands. It must:

- answer queries about live robot state (pose, ports, per-motor telemetry, gains, motion state)
- set PID gains live and write the result back into source
- trigger bench motions, behind interlocks, with a `stop` that actually stops
- stream periodic telemetry on demand, in a window the robot itself enforces

## Non-goals

- **Running during a match.** Motion is refused whenever field control is connected, and a
  motion already running is aborted the moment it becomes connected.
- **Untethered operation.** USB or controller tether only. No VEXlink, no custom radio.
- **Persisting gains on the robot.** Explicitly rejected — see Decision 2.
- **A background daemon.** Explicitly rejected — see Decision 1.
- **Replacing the on-brain tuner.** `pidtuner`/`autotune` stay. All three writers of gains go
  through one shared helper (see Integration points), so they cannot drift apart.

## Decisions

### Decision 1 — one-shot CLI, no daemon

The V5 user serial port is exclusive: whoever holds it blocks `pros mu`. A persistent broker
would have to be started, stopped, and released around every upload. Instead each
`tools/brain <cmd>` invocation opens the port, sends one line, reads until the reply is complete,
and exits. The port is free the moment the command returns.

"Complete" depends on the command. For queries it is `[end]`. For motion it is the motion's
terminal `[evt]` — `tools/brain drive 24` holds the port until the drive finishes, so its exit
code reports how the motion ended, not merely that it was accepted (see Host design). For
streams it is the robot-enforced window closing.

The only thing given up is always-on background logging, which is not worth a daemon.

### Decision 2 — source is always the truth for gains

The robot never loads gains from anywhere but its compiled-in constants. There is no SD card in
this design. The serial link already carries gains off the robot in the same session that
produced them, so an SD file would be write-only. What the robot does in a match is fully
determined by the flashed binary, which is fully determined by git.

### Decision 3 — the robot does not format C++

`dump` and `sync-gains` are *host* commands. The robot exposes gains as values via `gains`; the
host formats them. Changing the output format never requires a reflash.

### Decision 4 — motion runs on its own task

Motion commands do not execute on the reader task. If they did, the reader would be blocked for
the duration of a motion and could not receive `stop`. The reader dispatches to a dedicated
motion task and stays responsive throughout.

### Decision 5 — the bridge owns the robot while it moves, and the controller can take it back

While a bridge motion is active, the two teleop tasks stop writing to any actuator. Without
this, `drivetrain_control` rewrites the drive every 10 ms (`chassis.arcade` calls `move()`
unconditionally, even at zero stick — `src/lemlib/chassis/opcontrol.cpp:43-44`) and
`teleop_loop` brakes `ramp`/`top1`/`top2` in its idle branch every 10 ms, so bridge motion
would be fought to a standstill.

Handing the robot to the bridge takes it away from the person holding the controller, so the
controller gets a deadman: **any stick deflection past 30 or any button press during a bridge
motion is a `stop`.** This path involves no serial link and no host at all — it is the one
stop mechanism that still works if the laptop is closed, the cable is yanked, or the host
process is wedged.

### Decision 6 — `stop` is a latch, not a one-shot

`chassis.cancelAllMotions()` only clears two flags (`src/lemlib/chassis/chassis.cpp:147-151`).
It ends the LemLib motion in flight and nothing else. A `run <name>` auton is a straight-line
C++ function; after the cancelled motion returns, the routine proceeds to its next statement —
the next `moveToPoint`, the next `ramp.move(127)` — and the robot drives off. A one-shot `stop`
is therefore a no-op against the command where it matters most.

`stop` instead sets a latch that holds until the motion task is idle again. While latched:

- every new chassis motion returns immediately without driving (via the LemLib patch below);
- a guard task brakes the drivetrain and every named motor every 10 ms.

The motion task is never killed. `pros::Task::remove()` on it is forbidden: it may be holding
`Chassis::mutex` (taken in `requestMotionStart()`, `chassis.cpp:126`, released only in
`endMotion()`), and a stranded mutex deadlocks every later chassis motion — including the real
`autonomous()` at the next match. `src/autotune.cpp:302-310` made the same "join, don't remove"
choice for the same reason.

What `stop` honestly cannot do during `run <name>`, stated so nobody is surprised on the bench:

- **Pneumatic writes still fire.** The routine keeps executing; braking cannot undo a
  `set_value`. Keep hands clear of the pistons until `motion` reports idle.
- **Raw motor writes pulse for up to 10 ms.** A routine's `ramp.move(127)` runs until the guard's
  next brake tick overwrites it.
- **The routine takes its natural duration to unwind.** `pros::delay` calls are not
  interruptible. A skills routine stopped at 5 s stays `state=stopping` until its remaining
  ~55 s of delays have elapsed. No other motion can be started until then.

For `drive`, `turn` and `moveto` none of these apply — they are one LemLib motion each, and a
latched stop ends them within one 10 ms tick.

## Architecture

```
  tools/brain ──(1 line)──▶ /dev/cu.usbmodem* ──▶ reader ──▶ devcmd::parse()
       ▲                                             │
       │                                             ▼
       └──([ok]/[err]/[end] from reader) ◀─────  dispatch ──┬──▶ query / gain handlers (inline)
       └──([evt] from motion)                               ├──▶ motion task
       └──([tlm] from stream)                               └──▶ stream task

                          guard task: never writes serial; brakes, cancels, polls
```

Four tasks, all started from `initialize()` after `chassis.calibrate()`, so the bridge is alive
during disabled and autonomous as well as opcontrol:

| Task | Priority | Role |
|---|---|---|
| `devbridge_guard` | default + 1 | 10 ms loop: holds the `stop` latch (cancel + brake), polls field control and the controller deadman while motion is active. **Never writes to serial.** |
| `devbridge_reader` | default | blocking line read, parse, dispatch, reply |
| `devbridge_motion` | default | executes one motion job at a time; emits its terminal `[evt]` |
| `devbridge_stream` | default − 2 | emits `[tlm]` lines at the configured period until the window closes |

The guard never writes to serial because serial writes block (next section). A guard stalled on
a full TX buffer would be a safety mechanism that stops working exactly when the host is gone.
It acts through flags and actuators only; the motion task reports what happened.

### Serial output constraints

These are properties of the PROS 4.1.0 runtime this repo links, not choices:

- stdout writes block, with an infinite timeout, behind a ~2 KB TX buffer drained at the USB
  rate. Any task that writes can stall if nothing is reading.
- `fdctl(..., SERCTL_NOBLKWRITE, ...)` is a no-op in the shipped `libpros.a` and must not be
  relied on.
- newlib is built reentrant per task, so each task's `\n`-terminated `printf` arrives as a
  contiguous line. A write with **no** trailing newline is the one way output splices: the next
  task's line lands on the end of it.

Consequences for the design: every bridge line is formatted into a local buffer and written with
a single `fputs` + `fflush`; `stop` performs its cancel and brake *before* emitting `[ok] stop`,
so it takes effect even if the write then stalls; the stream window is enforced on the robot so
an abandoned stream cannot fill the buffer forever; and the per-iteration LemLib `printf`s are
removed (Integration points).

## Wire protocol

### Host → robot

One command per line, `\n`-terminated (a preceding `\r` is stripped), ASCII, **128 bytes
maximum** including the newline. Tokens are separated by runs of spaces. The command word is
case-insensitive; arguments are not. A longer line is discarded up to the next newline and
answered with `[err] line too long` / `[end] ?`.

### Robot → host

Every bridge line carries a bracket tag:

| Tag | Emitted by | Meaning |
|---|---|---|
| `[ok]` | reader | a successful reply line; a command may emit several |
| `[err]` | reader | the command was refused or failed; the text after the tag is the reason |
| `[end] <cmd>` | reader | terminator — always the last reply line, including on error |
| `[evt]` | motion | terminal outcome of a motion job, or stream expiry |
| `[tlm]` | stream | telemetry sample |

Ordering rules:

1. `[err]` is a reply line and is always emitted **before** the `[end]` of the command that
   produced it. Every interlock is evaluated on the reader at dispatch, before handoff, so a
   refusal can always be reported this way.
2. `[evt]` and `[tlm]` are the only tags that may appear after an `[end]`, or between a
   command and its `[end]`. They are never part of a reply.
3. Anything that goes wrong after dispatch — a `stop`, a controller override, field control
   connecting mid-motion — is reported as the motion's terminal `[evt]`, never as `[err]`.

`<cmd>` in `[end]` is the first token of the raw line as received, lower-cased, or `?` when the
line was empty or discarded.

**Host parsing rule:** for each received line, find the first match of
`\[(ok|err|end|evt|tlm)\]` at *any* offset, discard everything before it, and parse from there.
Only a line containing no tag at all is ignored as untagged `printf`. This keeps a stray
unterminated `printf` from any future edit — in an auton or in vendored LemLib — from swallowing
a terminator.

## Command reference

Floats are printed with a fixed number of decimals. Fields are `key=value` with no spaces in
values.

### Query — always permitted, including under field control

| Command | Reply |
|---|---|
| `pose` | `[ok] pose x=12.40 y=31.08 th=-89.70` |
| `ports` | one `[ok] port <n>=<TYPE>` per occupied port |
| `motors` | one `[ok] motor <name> port=<signed n> temp=<c> cur=<mA> pos=<deg> eff=<pct>` per **physical** motor |
| `gains` | `[ok] gains lat kp=14.2500 ki=0.0000 kd=110.0000`, then the same for `ang` |
| `motion` | `[ok] motion state=<idle\|running\|stopping> ...` — see below |
| `sel` | `[ok] sel auton=3 state=QUAL started=0` |
| `bat` | `[ok] bat mv=12340 ma=1520 temp=31.0` |
| `ver` | `[ok] ver built=Sep_11_2026_14:22:03` |
| `help` | one `[ok] help <cmd>` per command |

**`ports`** iterates registry index `i = 0..20` and calls `registry_get_plugged_type(i)`, which
is zero-indexed (`include/pros/apix.h:719-740`: "The V5 port number from 0-20"). It reports
`port <i+1>`, so wire output uses the same 1-based numbering as `pros::Motor` constructors and
`robot-config.cpp`. Indices reading `E_DEVICE_NONE`, or returning an error, are omitted. Types
are the `v5_device_e_t` names: `MOTOR`, `ROTATION`, `IMU`, `DISTANCE`, `RADIO`, `VISION`, `ADI`,
`OPTICAL`, `GPS`, `SERIAL`, `UNDEFINED`.

**`motors`** reports every physical motor, not every named object, so a dead motor in a group
cannot hide behind a healthy average. The name table in `devbridge.cpp` holds
`pros::AbstractMotor&` — the common base of `pros::Motor` and `pros::MotorGroup` — and iterates
each entry by index with `get_port(i)`, `get_temperature(i)`, `get_current_draw(i)`,
`get_position(i)`, `get_efficiency(i)` for `i < size()`:

```
[ok] motor leftMotors[0] port=-11 temp=41.0 cur=820 pos=1520.3 eff=64.0
[ok] motor leftMotors[1] port=-14 temp=42.0 cur=790 pos=1519.8 eff=63.0
[ok] motor leftMotors[2] port=15 temp=47.0 cur=1310 pos=1521.1 eff=41.0
  ... rightMotors[0..2] ...
[ok] motor ramp port=9 temp=39.0 cur=0 pos=0.0 eff=0.0
[ok] motor top1 port=-17 ...
[ok] motor top2 port=19 ...
```

Names match the identifiers in `robot-config.hpp`. `intake` is deliberately absent: CLAUDE.md
deprecates it, and its three ports are the same physical motors as `ramp`/`top1`/`top2`.

**`motion`** is backed by the motion task's own state, never by `chassis.isInMotion()` (which
reads false between the chained motions inside a `run` routine):

```
[ok] motion state=running verb=drive t=812
[ok] motion state=stopping verb=run name=skills3 t=5210 reason=host
[ok] motion state=idle last=drive result=stopped reason=controller x=3.10 y=11.92 th=0.40
```

**`ver`** reads `_PROS_COMPILE_TIMESTAMP`, which the PROS link step already regenerates on every
link (`common.mk:283-297`), declared `extern "C" char const* const _PROS_COMPILE_TIMESTAMP;`.
Spaces are replaced with `_`. This answers "is the robot running the build I just made?", which
is the question that matters. A git hash was dropped: a compile-time define goes stale because
nothing rebuilds the object when `HEAD` moves, and it lies anyway whenever the tree is dirty.

### Tune — always permitted

| Command | Reply |
|---|---|
| `set <lat\|ang>.<kp\|ki\|kd> <value>` | `[ok] set lat kd 110.0000 -> 118.0000` |

Rejected with `[err]` on: an unknown axis or gain name, a value that does not parse as a
finite float, a negative value (matching `clampGain` in `pidtuner.cpp`), or a value above 1000.

Writes go through `pidtuner::applyGains()`, the shared helper that `pidtuner` and `autotune`
also use. It calls `setGains()` on the live `lemlib::PID` *and* mirrors into the global
`linearController`/`angularController` settings, exactly as both callers do today
(`pidtuner.cpp:99-106`, `autotune.cpp:330-337`). The Developer tab reflects a serial `set`
immediately.

### Motion — interlocked

| Command | On success | On refusal |
|---|---|---|
| `arm` | `[ok] arm 30s` | never refused |
| `disarm` | `[ok] disarm` | never refused |
| `drive <inches>` | `[ok] drive 24.00 accepted`, later `[evt] drive <result> ...` | `[err] motion: <reason>` |
| `turn <degrees>` | `[ok] turn 90.00 accepted`, later `[evt] turn <result> ...` | `[err] motion: <reason>` |
| `moveto <x> <y> <th>` | `[ok] moveto accepted`, later `[evt] moveto <result> ...` | `[err] motion: <reason>` |
| `run <name>` | `[ok] run qual3 accepted`, later `[evt] run <result> ...` | `[err] motion: <reason>` |
| `stop` | `[ok] stop` | never refused |

A refused motion emits `[err]`, then `[end]`, and no `[evt]` ever follows.

The terminal event is `[evt] <verb> <done|stopped> reason=<r> x=.. y=.. th=..`, where `reason`
is `none` for `done`, and `host`, `controller`, or `competition` for `stopped`. For `run`, the
name follows the verb: `[evt] run qual3 stopped reason=host ...`.

**Interlocks.** All four are evaluated on the reader at dispatch; all must pass:

1. **Field control.** `[err] motion: competition control connected` when
   `competition_is_connected()`. Additionally, while a motion is active the guard polls it every
   10 ms and, on a transition to connected, latches `stop` with `reason=competition`. The
   in-flight motion cannot check this itself — it is blocked inside a LemLib loop, and of the
   vendored motions only `pursuit.cpp:242` re-reads competition state.
2. **Arming.** `[err] motion: not armed` unless `arm` was issued within the last 30 seconds.
   This gates *acceptance*. It does not bound the motion's run time — the motion's own timeout
   and `stop` do that.
3. **Single-flight.** `[err] motion: busy` while the motion task is `running` or `stopping`.
4. **Auto-tuner.** `[err] motion: autotune open` while `autotune::isRunning()`. The relay writes
   the drive motors directly (`autotune.cpp:115-126`), so the two must never overlap. The
   converse is enforced in `autotune`: its ARMED step refuses to start the relay while
   `devbridge::isMotionActive()`.

**Bounds.** Checked by the parser, so they are unit-tested: `drive` |in| ≤ 72; `moveto` |x|, |y|
≤ 72 and |th| ≤ 360; `turn` |deg| ≤ 360. Every bridge motion runs with a fixed LemLib timeout —
5000 ms for `drive`/`moveto`, 3000 ms for `turn` — so a motion ends on the brain's own clock
even if every other mechanism fails.

**`stop`.** Exempt from all interlocks, never refused, idempotent. It latches with
`reason=host`, calls `chassis.cancelAllMotions()`, brakes the drivetrain and every named motor,
clears the arm window (an implicit `disarm`), and only then emits `[ok] stop`. With no motion
active it still brakes, and still replies `[ok] stop`.

**`run <name>`** accepts the routine names in `autons.hpp` — `qual1`…`qual6`, `AI1`…`AI6`,
`skills1`…`skills6`, `1ring` — through a name→function table in `devbridge.cpp`. It does not
accept the selector's integer encoding; that encoding is a display-layer artifact with known
traps (see CLAUDE.md). Its stop semantics are the limitations listed under Decision 6.

### Stream

| Command | Reply |
|---|---|
| `stream <pose\|pid\|motors> <period_ms> <duration_ms>` | `[ok] stream pose 50 8000` |
| `stream off` | `[ok] stream off` |

`period_ms` is 10–1000. `duration_ms` is **mandatory**, 1–60000. The stream task stops itself
when the window closes and emits `[evt] stream expired`. `stream off` is an early stop, not the
only stop, so a Ctrl-C'd host cannot leave a stream filling the TX buffer. A second `stream`
replaces the first.

```
[tlm] pose t=4120 x=12.40 y=31.08 th=-89.70
[tlm] pid t=4120 lat.err=2.31 lat.out=54.02 ang.err=0.40 ang.out=3.10
[tlm] motors t=4120 leftMotors[0]=41.0 leftMotors[1]=42.0 ... top2=44.0
```

The tag after `[tlm]` is the same topic token the host asked for. `motors` reports temperature
per physical motor.

`pid` reads `lemlib::PID::getLastError()` and `getLastOutput()` (Integration points). `out` is
the raw PID output **before** LemLib's clamp, slew, and direction limit
(`moveToPoint.cpp:83-95`), so it can differ from what actually reached the motors — which is
exactly what makes saturation visible. There is no target field: the target is a local inside
each motion body and never leaves it.

## On-robot design

Two file pairs, split so the fiddly half needs no hardware.

### `include/devcmd.hpp` / `src/devcmd.cpp` — pure parsing

**No PROS includes. No LemLib includes.** Depends only on `<string_view>`, `<cstdint>`,
`<cstdlib>`, `<cmath>`. Compiles unmodified with host `c++`.

```cpp
namespace devcmd {

enum class Verb { Pose, Ports, Motors, Gains, Motion, Sel, Bat, Ver, Help,
                  Set, Arm, Disarm, Drive, Turn, MoveTo, Run, Stop, Stream,
                  Invalid };

enum class Axis  { Lateral, Angular };
enum class Gain  { KP, KI, KD };
enum class Topic { Pose, Pid, Motors, Off };

struct Command {
    Verb  verb = Verb::Invalid;
    Axis  axis{};
    Gain  gain{};
    Topic topic{};
    float args[3]{};        // drive/turn/moveto operands, or the set value
    int   argc = 0;
    int   period = 0;       // stream period, ms
    int   duration = 0;     // stream window, ms
    char  name[24]{};       // run <name>: validated for length and charset only
    char  error[64]{};      // populated when verb == Invalid
};

Command parse(std::string_view line);

} // namespace devcmd
```

`parse` never allocates, never throws, and never writes outside the returned struct. Every
rejection sets `Verb::Invalid` with a human-readable `error`, which the reader emits after
`[err] `. Whether a `run` name refers to a real routine is decided by `devbridge.cpp`'s table,
not the parser — keeping the parser free of robot knowledge.

### `include/devbridge.hpp` / `src/devbridge.cpp` — the PROS-facing half

```cpp
namespace devbridge {
void start();              // called from initialize(); spawns the four tasks
bool isMotionActive();     // running or stopping; teleop and autotune yield while true
}
```

`start()` calls `serctl(SERCTL_DISABLE_COBS, NULL)` before spawning, so the link carries plain
bytes. (See Open Questions for its unconfirmed effect on input.)

The reader emits `[end] <cmd>` last on every path. Query and gain handlers run inline — they
take microseconds. Motion handlers evaluate interlocks, hand the job to the motion task, reply
`accepted`, and return.

This file also holds the only robot-specific knowledge in the bridge: the motor name table and
the auton name table.

## Integration points

| File | Change | Why |
|---|---|---|
| `src/main.cpp` | `devbridge::start();` in `initialize()`, after `chassis.calibrate()` | starts the bridge |
| `src/controls.cpp` | `drivetrain_control` skips its `arcade`/`tank` call while `devbridge::isMotionActive() \|\| autotune::isRelayActive()`; `teleop_loop` skips every actuator write while `devbridge::isMotionActive()` | Decision 5 |
| `src/autotune.cpp`, `include/autotune.hpp` | add `isRelayActive()` (true between relay start and join); ARMED refuses to start while `devbridge::isMotionActive()`; apply results via `pidtuner::applyGains()` | interlock 4; one gain-write path |
| `src/pidtuner.cpp`, `include/pidtuner.hpp` | extract the existing `setGains` + settings-mirror sequence into public `applyGains(bool lateral, float kP, float kI, float kD)` | one gain-write path |
| `src/lemlib/chassis/chassis.cpp`, `include/lemlib/chassis/chassis.hpp` | `requestMotionStart()` returns `bool`; after `mutex.take`, if `!motionRunning \|\| motionsInhibited` it clears `motionRunning`, calls `mutex.give()`, returns `false`. Add public `bool motionsInhibited` | fixes a latent LemLib 0.5.2 mutex leak that `stop` makes reachable; enables the latch |
| `src/lemlib/chassis/motions/*.cpp` (all seven) | `if (!this->requestMotionStart()) return;` replaces the call plus its `if (!motionRunning) return;` | today that early return exits **holding** `Chassis::mutex`, which is non-recursive; the next motion then self-deadlocks forever |
| `src/lemlib/pid.cpp`, `include/lemlib/pid.hpp` | add `getLastError()` (returns the existing `prevError`) and a `lastOutput` member set in `update()`, with `getLastOutput()` | data source for `stream pid` |
| `src/lemlib/chassis/motions/moveToPoint.cpp:108`, `moveToPose.cpp:136`, `turnToHeading.cpp:38` | delete these per-iteration `printf`s | 100 Hz debug noise; `moveToPose.cpp:136` has no newline and splices into bridge lines; `stream pid` replaces all three on demand. The one-shot exit-reason `printf`s in `turnToHeading.cpp:44-56` stay |
| `Makefile` | add a `test-host` target | host-native tests |
| `CLAUDE.md` | extend "LemLib is vendored" with the chassis, pid and printf changes; add the bridge to the actuator-ownership notes | keep the repo map true |

Touching `include/lemlib/pid.hpp` and `chassis.hpp` rebuilds roughly 40 files. That is expected.

### Phase 0 — ships before the bridge, independently (done: see git log for `isRelayActive`)

The `drivetrain_control` half of the `controls.cpp` change fixes an **existing bug** in
`eb6b083`, and should land first on its own. Today the auto-tuner's relay task and
`drivetrain_control` both write the drive motors every 10 ms — the relay writes ±d,
`chassis.arcade` writes 0 at centered sticks. Whichever runs last in a tick wins, so the
effective relay amplitude is below the `d` used in `Ku = 4d / (πa)` and the computed gains are
skewed. The guard keys on `isRelayActive()`, not `isRunning()`, so the driver can still drive
the robot into position during the wizard's PLACEMENT step. Gains produced by the auto-tuner
before this fix should be re-measured.

## Host design

### `tools/brain`

A POSIX `sh` launcher plus `tools/brain.py` (Python 3, `pyserial`). The launcher execs
`brain.py` with the first interpreter that can `import serial`, trying in order: `$V5_PYTHON`;
each `python3` on `PATH`; the interpreter named in the shebang of the `pros` executable (on this
machine, the only one with `pyserial` 3.5). Nothing is hardcoded; if none succeeds it prints the
`pip install pyserial` line to run and exits 3.

```
tools/brain pose                       # query; prints the reply
tools/brain set lat.kp 14.5
tools/brain drive 24                   # holds the port until [evt] drive ...
tools/brain drive 24 --no-wait         # exit on "accepted"; poll with `tools/brain motion`
tools/brain stream pid 20 --for 8s     # sends duration 8000; reads until [evt] stream expired
tools/brain dump                       # local: gains as a ControllerSettings block
tools/brain sync-gains                 # local: rewrite robot-config.cpp, show diff
tools/brain --mock pose                # canned responses, no hardware
```

**Port selection:** glob `/dev/cu.usbmodem*`, send `ver` to each candidate, and keep the one that
answers with `[ok] ver`. Overridable with `--port` or `$V5_PORT`. This settles Open Question 3
in code instead of documenting a magic device name.

**Motion waits by default.** After `[end]`, a motion invocation keeps reading until its terminal
`[evt]`, or until motion timeout + 2 s. **Ctrl-C while waiting sends `stop` on the port it
already holds** before closing — since the port is exclusive, no other invocation could deliver
it.

**Exit codes:** `0` done / success; `1` robot replied `[err]`, or the motion ended `stopped`;
`2` timeout or no device; `3` usage or environment error. `--no-wait` motions exit `0` on
`accepted`, which then means only that.

### `tools/brain sync-gains`

```
$ tools/brain sync-gains
--- src/robot-config.cpp
+++ src/robot-config.cpp
-lemlib::ControllerSettings linearController(  14.25, // proportional gain (kP) // k_cr = 48
+lemlib::ControllerSettings linearController(  16.8, // proportional gain (kP) // k_cr = 48
-                                              110, // derivative gain (kD) 13.5
+                                              118, // derivative gain (kD) 13.5
2 controllers checked, 2 gains changed. run `make`.
```

It sends `gains`, then in `src/robot-config.cpp` finds each of `linearController(` and
`angularController(` and replaces only the numeric token of the first three constructor
arguments. The real layout has one argument per line, the first on the same line as the paren,
each followed by a `//` comment carrying historical values (`robot-config.cpp:96-116`); comments,
indentation and every other argument are left byte-identical. Values are printed in shortest
form with up to 4 decimals.

It refuses to run if either block has uncommitted changes, shows a unified diff, and writes only
on confirmation (or `--yes` when stdin is not a TTY). If either block cannot be located it
changes nothing and exits 3.

## Testing

### Host-native unit tests — runnable today, no hardware

`make test-host` compiles `src/devcmd.cpp` and `tools/test-devcmd.cpp` with the system `c++`
(not the ARM toolchain) and runs the result. The test file lives in `tools/`, outside the
`src/` tree that `common.mk` globs, so it can never be swept into the robot build. **This is the
first automated test in the repo.**

Coverage:

- every verb with representative valid arguments
- `set` with all six axis × gain combinations
- unknown verb, axis, gain; non-numeric, `nan`, `inf`, negative, >1000 and empty values
- missing and surplus arguments for every verb
- each motion bound, at the limit and one past it
- `stream` with missing duration, duration 0, duration over 60000, period out of range, `off`
- `run` with a valid-looking name, an empty name, a name longer than 23 bytes, bad characters
- 128-byte line, 129-byte line, empty line, whitespace-only line, `\r\n`, mixed-case verb,
  repeated inner spaces

### Host CLI tests

`tools/brain --mock` answers from a fixture table, exercising argument handling, reply parsing,
`[end]` and `[evt]` detection, timeouts, exit codes, and Ctrl-C-sends-`stop`. Fixtures include
spliced lines — `lateralOut: 5.0, angularOut: 1.0[end] moveto` and the same with `[evt]` — to
prove the any-offset tag rule. `sync-gains` is tested against a copy of the real
`robot-config.cpp` in a temp directory, asserting the diff touches exactly the gain tokens.

### Hardware bring-up checklist — first contact with a brain

In order; each step depends on the ones before.

1. **Phase 0 first.** With the `controls.cpp` guard flashed, run the auto-tuner once and
   confirm the driver can still drive during PLACEMENT but not during RUNNING.
2. Flash the bridge. Open `pros terminal`; confirm existing `printf` output still appears.
3. `tools/brain ver` — proves port auto-detect, the reader task, stdin delivery, and that the
   `built=` stamp matches the build you just made.
4. `tools/brain ports` — confirm the MOTOR ports match `robot-config.cpp` **exactly** (drive
   today: 1, 3, 4, 11, 14, 15). This is the check on the 0-indexed registry mapping; a
   plausible table of wrong numbers is the failure it exists to catch.
5. `tools/brain motors` — six drive rows plus `ramp`/`top1`/`top2`, signed ports matching the
   source.
6. `gains`, `set lat.kp 15`, `gains`; confirm the Developer tab shows 15.
7. `stream pose 50 --for 5s`; confirm it ends on its own with `[evt] stream expired`. Repeat and
   Ctrl-C at 2 s; confirm `pros terminal` shows the stream stopping at 5 s anyway.
8. **Robot on blocks, wheels clear, from here on.** `drive 6` without `arm` → refused. `arm`,
   `drive 6` → exits 0. `arm`, `drive 48`, Ctrl-C mid-motion → halts, exit 1, `result=stopped
   reason=host`.
9. **Controller deadman.** `arm`, `drive 48`, nudge a stick → halts, `reason=controller`.
10. **The latch.** `arm`, `run qual2`, `stop` during the first `moveToPoint`. Confirm no wheel
    motion for the rest of the routine, `motion` shows `state=stopping` until it unwinds, then
    `[evt] run qual2 stopped`. Then `arm`, `drive 6` → it runs, proving `Chassis::mutex` was
    released.
11. **Field control.** Plug in a competition switch: every motion refused, queries still answer.
    Then, with the switch unplugged, `arm`, `drive 48`, and plug it in mid-motion → halts,
    `reason=competition`.
12. `stream pid 20 --for 8s` in one terminal while `arm; drive 48` runs from another — every
    `[tlm]` line parses and `lat.err` visibly converges.
13. `drive 48 --no-wait`, then close the host immediately; time the motion against a waited run
    (Open Question 5).
14. `tools/brain sync-gains`; inspect the diff before committing.

## Open questions — require hardware

None blocks implementation; all block the checklist.

1. **Does `stdin` reliably deliver to a PROS task, and does a blocking read yield the CPU?**
   `apix.h:793` documents `open("serial", O_RDWR)`. If the read busy-waits, fall back to a
   non-blocking read plus a 20 ms poll in the reader loop.
2. **How does `SERCTL_DISABLE_COBS` affect input?** Documented for output framing only. Fallback:
   leave COBS enabled and decode it in `brain.py`.
3. **Which enumerated CDC device is the user port?** Resolved in code by `ver`-probing each
   candidate; the checklist confirms the probe works.
4. **Bandwidth over a controller tether.** A 20 ms `pid` stream may exceed what the radio
   carries. If so, raise the minimum period when tethered, or use `pose`.
5. **Does the TX buffer drain while USB is attached but no host has the port open?** If not,
   any task writing to stdout — including remaining LemLib exit-reason `printf`s — stalls once
   the buffer fills. Waiting by default makes this rare; step 13 measures it.

## Out of scope for v1, deliberately

- Background telemetry logging — needs the daemon Decision 1 rejects.
- Writing anything to the SD card — Decision 2.
- Editing odometry or drivetrain geometry live — those change `Drivetrain` construction, not a
  mutable controller, and a wrong value there is far more dangerous than a wrong gain.
- Generating `robot-config.cpp` port assignments from `ports`. `ports` supplies the data and
  step 4 makes it trustworthy; deciding which motor is `left` versus `right` needs a human.
- Interruptible `run`. Making `stop` end a routine immediately, rather than letting it unwind
  inert, needs routines that poll a cancellation flag between statements. Worth revisiting if
  the unwind time proves painful on the bench.

## Review record

An adversarial review ran five independent lenses over the first draft (PROS runtime, protocol,
safety, implementability, completeness). Each finding then went to a separate verifier told to
refute it. 30 findings were raised; 18 survived, collapsing to 12 distinct issues. Every one is
addressed above:

| Issue | Resolution |
|---|---|
| `stop` does not stop `run <name>` (raised independently by 3 lenses) | Decision 6 latch |
| `stop` racing a queued motion leaks `Chassis::mutex` → permanent deadlock | LemLib `requestMotionStart()` patch |
| Teleop tasks rewrite actuators every 10 ms, fighting bridge motion (3 lenses) | Decision 5 yield + deadman |
| Bridge motion and the auto-tuner relay can drive simultaneously | interlock 4 + the converse in autotune |
| Unterminated LemLib `printf` splices into and swallows bridge lines | printfs removed; any-offset tag rule |
| `[evt]` unobservable under one-shot exit | motion waits by default; `motion` query verb |
| Abandoned stream fills the TX buffer forever | mandatory robot-enforced `duration_ms` |
| Field control checked only at dispatch | guard polls during motion |
| `stream pid` had no data source | `getLastError`/`getLastOutput` LemLib patch; target field dropped |
| `ports` off by one (registry is 0-indexed) | explicit `i+1` mapping + checklist step 4 |
| `motors` hid dead group members and named nonexistent objects | one row per physical motor via `AbstractMotor&` |
| `-DDEVBRIDGE_GIT` define unquoted, never-falling-back, and stale | replaced by the link-time `_PROS_COMPILE_TIMESTAMP` |

Separately, a manual check while the review ran found the Phase 0 bug in the existing
auto-tuner, which the review's autotune finding had only touched from the bridge's side.

Findings refuted with evidence and deliberately not adopted include: newlib `printf`
interleaving mid-line (the linked runtime is reentrant per task — only the unterminated `printf`
is a problem), a reader/`tabWatcher` race on gain writes (impossible at the stated priorities),
and a demand for a host heartbeat deadman (every LemLib motion already carries a mandatory
timeout, and the controller deadman now covers the rest).
