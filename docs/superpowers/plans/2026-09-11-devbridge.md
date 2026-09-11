# DevBridge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the tethered host↔V5 brain command bridge specified in `2026-09-11-devbridge-design.md`, including safe queries, gain editing, interlocked motion, bounded telemetry, and a one-shot host CLI.

**Architecture:** Keep command parsing in a host-native, allocation-free C++ unit; keep robot state and PROS APIs in `devbridge.cpp`; and keep serial discovery, protocol parsing, source rewriting, and user-facing exit codes in `tools/brain.py`. The bridge uses separate reader, motion, stream, and safety-guard tasks so serial I/O cannot block stopping behavior.

**Tech Stack:** C++20, PROS 4.1.0, vendored LemLib 0.5.2, POSIX `sh`, Python 3, pyserial 3.5, standard-library `unittest`.

**Spec:** `docs/superpowers/specs/2026-09-11-devbridge-design.md`

## Global Constraints

- Host commands are ASCII, one `\n`-terminated line, at most 128 bytes including newline.
- Robot replies use `[ok]`, `[err]`, `[end]`, `[evt]`, and `[tlm]`; the host finds the first recognized tag at any line offset.
- Queries and `set` remain available under field control; motion never does.
- Motion requires a 30-second arm window, one active job, no competition connection, and no running auto-tuner relay.
- `drive`/`moveto` timeout at 5000 ms; `turn` at 3000 ms; bounds are ±72 inches and ±360 degrees.
- `stop` is idempotent, clears arming, brakes all named actuators before replying, and inhibits every subsequent LemLib motion until the motion task is idle.
- The guard task never writes serial.
- Controller deadman is any analog magnitude over 30 or any digital button during bridge motion.
- Streams require 10–1000 ms periods and 1–60000 ms robot-enforced durations.
- Gains are finite values from 0 through 1000 and always flow through `pidtuner::applyGains`.
- Do not use the deprecated duplicate `intake` MotorGroup in new bridge code; use `ramp`, `top1`, and `top2`.
- Keep every teleop loop non-blocking beyond its existing button-release waits.
- Every production edit must end with `make`; parser and host CLI behavior must also pass `make test-host`.

---

### Task 1: Allocation-free command parser

**Files:**
- Create: `include/devcmd.hpp`
- Create: `src/devcmd.cpp`
- Create: `tools/test-devcmd.cpp`
- Modify: `Makefile`

**Interfaces:**
- Produces:

```cpp
namespace devcmd {
enum class Verb { Pose, Ports, Motors, Gains, Motion, Sel, Bat, Ver, Help,
                  Set, Arm, Disarm, Drive, Turn, MoveTo, Run, Stop, Stream,
                  Invalid };
enum class Axis { Lateral, Angular };
enum class Gain { KP, KI, KD };
enum class Topic { Pose, Pid, Motors, Off };
struct Command {
    Verb verb = Verb::Invalid;
    Axis axis{};
    Gain gain{};
    Topic topic{};
    float args[3]{};
    int argc = 0;
    int period = 0;
    int duration = 0;
    char name[24]{};
    char error[64]{};
};
Command parse(std::string_view line);
}
```

- Consumes only `<string_view>`, `<cstdint>`, `<cstdlib>`, `<cmath>`, and fixed-size C/C++ utilities; no PROS, LemLib, heap allocation, or exceptions.

- [ ] **Step 1: Add the host test target and failing parser tests**

Add a phony target that uses the system compiler rather than the ARM toolchain:

```make
.PHONY: test-host
test-host:
	@mkdir -p /tmp/pepsi-kola-host-tests
	c++ -std=c++20 -Wall -Wextra -Werror -Iinclude src/devcmd.cpp tools/test-devcmd.cpp -o /tmp/pepsi-kola-host-tests/test-devcmd
	/tmp/pepsi-kola-host-tests/test-devcmd
	python3 tools/test_brain.py
```

Until Task 2 creates `tools/test_brain.py`, gate that final command with `test ! -f tools/test_brain.py || python3 tools/test_brain.py`.

Write a small assertion-based test runner covering every zero-argument verb and representative valid `set`, `drive`, `turn`, `moveto`, `run`, and `stream` commands. Include all six axis/gain combinations.

- [ ] **Step 2: Run the parser tests and verify RED**

Run:

```bash
make test-host
```

Expected: compilation fails because `devcmd.hpp`/`devcmd.cpp` and `devcmd::parse` do not exist yet.

- [ ] **Step 3: Implement tokenization and valid-command parsing**

Implement case-insensitive matching for command words only, stripping a trailing `\r`, splitting on runs of spaces, rejecting surplus tokens, and parsing floats with full-token consumption plus `std::isfinite`.

Exact valid forms:

```text
pose | ports | motors | gains | motion | sel | bat | ver | help
set <lat|ang>.<kp|ki|kd> <0..1000>
arm | disarm | stop
drive <-72..72>
turn <-360..360>
moveto <-72..72> <-72..72> <-360..360>
run <1..23 ASCII letters/digits/_/->
stream <pose|pid|motors> <10..1000> <1..60000>
stream off
```

- [ ] **Step 4: Run valid-command tests and verify GREEN**

Run `make test-host`. Expected: all current parser assertions pass.

- [ ] **Step 5: Add failing malformed-input and boundary tests**

Cover: unknown command/axis/gain/topic; empty and whitespace-only lines; missing and surplus arguments for every shape; non-numeric, partial numeric, `nan`, `inf`, negative gain, gain over 1000; motion values exactly at each bound and one past; stream period 9/10/1000/1001 and duration 0/1/60000/60001; empty/24-byte/bad-character run names; CRLF; mixed-case command; repeated inner spaces; a 128-byte line and a 129-byte line.

- [ ] **Step 6: Run malformed-input tests and verify RED**

Run `make test-host`. Expected: new malformed or boundary assertions fail.

- [ ] **Step 7: Complete validation and stable error messages**

Every rejection must return `Verb::Invalid` and populate `error` with a concise message that can be emitted directly after `[err] `. A line over 128 bytes must report `line too long`.

- [ ] **Step 8: Run host tests and robot build**

Run:

```bash
make test-host
make
```

Expected: parser suite passes and PROS build exits 0.

### Task 2: One-shot host CLI

**Files:**
- Create: `tools/brain`
- Create: `tools/brain.py`
- Create: `tools/test_brain.py`
- Modify: `Makefile`

**Interfaces:**
- `tools/brain <command>` sends exactly one robot command and exits after `[end]`, the relevant motion `[evt]`, or stream expiry.
- `tools/brain dump` formats current gains locally.
- `tools/brain sync-gains [--yes]` rewrites only the first three numeric constructor arguments of `linearController` and `angularController`.
- `tools/brain --mock ...` uses deterministic fixture responses without opening hardware.
- Port precedence is `--port`, `$V5_PORT`, then `/dev/cu.usbmodem*` candidates probed with `ver`.

- [ ] **Step 1: Write failing CLI tests**

Use `unittest` plus subprocesses to cover:

```text
--mock pose
--mock gains
--mock drive 24
--mock drive 24 --no-wait
--mock stream pid 20 --for 8s
unknown command and malformed --for
[err] => exit 1
timeout/no device => exit 2
usage/environment error => exit 3
```

Test tag extraction from normal lines and spliced lines such as `lateralOut: 5.0[end] moveto`. Test `sync-gains` against a temporary copy of the real `robot-config.cpp`, asserting only the six gain number tokens change and confirmation is required unless `--yes` is supplied.

- [ ] **Step 2: Run CLI tests and verify RED**

Run `python3 tools/test_brain.py`. Expected: failure because the launcher and module do not exist.

- [ ] **Step 3: Implement the launcher**

`tools/brain` must try `$V5_PYTHON`, each `python3` on `PATH`, then the Python interpreter in the `pros` executable shebang. Select the first interpreter for which `import serial` succeeds and `exec` `tools/brain.py`; print an actionable `pip install pyserial` message and exit 3 if none works.

- [ ] **Step 4: Implement argument normalization and mock transport**

Convert `--for 8s` to duration `8000`, preserve `--no-wait`, and make fixture responses exercise `[ok]`, `[err]`, `[end]`, `[evt]`, and `[tlm]`. Keep serial imported lazily so tests and `--mock` work under a Python without pyserial.

- [ ] **Step 5: Run CLI tests and verify partial GREEN**

Run `python3 tools/test_brain.py`. Expected: mock argument, parsing, and exit-code tests pass; real transport/source-sync tests still fail.

- [ ] **Step 6: Implement serial transport and reply parsing**

Open ports at 115200 baud with bounded timeouts, send `command + "\n"`, ignore lines with no recognized tag, and retain content starting at the first recognized tag. Auto-detect by probing candidates with `ver`. Query commands stop at matching `[end]`; waited motions continue to their verb’s `[evt]`; streams continue through `[evt] stream expired`. On Ctrl-C during a waited motion, send `stop\n` on the already-open port before closing.

- [ ] **Step 7: Implement `dump` and transactional `sync-gains`**

Parse:

```text
[ok] gains lat kp=<f> ki=<f> kd=<f>
[ok] gains ang kp=<f> ki=<f> kd=<f>
```

Locate both exact `ControllerSettings` constructor blocks, compute all six replacements before writing, preserve comments/whitespace, show a unified diff, refuse if either block cannot be parsed, and write only after confirmation or `--yes`.

- [ ] **Step 8: Run complete host tests**

Remove the temporary Makefile guard for `tools/test_brain.py`, then run:

```bash
make test-host
```

Expected: parser and CLI suites pass.

### Task 3: Shared gains and LemLib safety/telemetry prerequisites

**Files:**
- Modify: `include/pidtuner.hpp`
- Modify: `src/pidtuner.cpp`
- Modify: `include/autotune.hpp`
- Modify: `src/autotune.cpp`
- Modify: `include/lemlib/pid.hpp`
- Modify: `src/lemlib/pid.cpp`
- Modify: `include/lemlib/chassis/chassis.hpp`
- Modify: `src/lemlib/chassis/chassis.cpp`
- Modify: all seven `src/lemlib/chassis/motions/*.cpp`

**Interfaces:**
- Produces `pidtuner::applyGains(bool lateral, float kP, float kI, float kD)`.
- Produces `PID::getLastError() const`, `PID::getLastOutput() const`, and stores the output returned by `PID::update`.
- Changes protected `Chassis::requestMotionStart()` to `bool`.
- Adds public `bool Chassis::motionsInhibited = false`.

- [ ] **Step 1: Extract the single gain-write path**

Implement:

```cpp
void applyGains(bool lateral, float kP, float kI, float kD) {
    lemlib::PID& pid = lateral ? chassis.lateralPID : chassis.angularPID;
    pid.setGains(kP, kI, kD);
    lemlib::ControllerSettings& settings =
        lateral ? linearController : angularController;
    settings.kP = kP;
    settings.kI = kI;
    settings.kD = kD;
}
```

Replace the duplicated writes in PID editor adjustment and autotune result application with this helper.

- [ ] **Step 2: Add PID telemetry accessors**

Store the exact value returned by `update()` in `lastOutput`; expose it and `prevError`; reset both to zero in `reset()`.

- [ ] **Step 3: Make motion acquisition inhibition-safe**

After taking `mutex`, `requestMotionStart()` must check `!motionRunning || motionsInhibited`; on refusal clear `motionRunning`, give the mutex, and return false. On success return true.

- [ ] **Step 4: Update every motion entry point**

In `follow`, `moveToPose`, `moveToPoint`, `turnToHeading`, `turnToPoint`, `swingToHeading`, and `swingToPoint`, replace the old start plus `if (!motionRunning) return` pair with:

```cpp
if (!this->requestMotionStart()) return;
```

- [ ] **Step 5: Remove high-rate serial noise**

Delete only the per-iteration `printf` calls from `moveToPoint.cpp`, `moveToPose.cpp`, and `turnToHeading.cpp`. Keep one-shot turn exit-reason messages.

- [ ] **Step 6: Build after the vendored LemLib changes**

Run `make`. Expected: all affected source files compile and link successfully; the known RWX link warning is acceptable.

### Task 4: Robot bridge runtime

**Files:**
- Create: `include/devbridge.hpp`
- Create: `src/devbridge.cpp`

**Interfaces:**
- Produces:

```cpp
namespace devbridge {
void start();
bool isMotionActive();
}
```

- Consumes `devcmd::parse`, robot globals, `pidtuner::applyGains`, `autotune::isRunning`, auton functions, LemLib inhibition, and `_PROS_COMPILE_TIMESTAMP`.

- [ ] **Step 1: Define bridge state and single-line writer**

Use fixed-size job/state buffers and PROS synchronization; no dynamic strings in task loops. Every emitted bridge record must be assembled in a local buffer and written once with a trailing newline plus flush. Call `serctl(SERCTL_DISABLE_COBS, nullptr)` before tasks start.

- [ ] **Step 2: Implement reader framing and terminators**

Read from `stdin`/`serial` without busy-spinning, enforce 128 bytes including newline, discard an overlong remainder, strip CR, parse, dispatch, and emit `[end] <lowercased-first-token-or-?>` last on every inline response path. Motion acceptance emits `[end]` before the later terminal event.

- [ ] **Step 3: Implement always-permitted queries**

Implement exact wire forms for `pose`, `ports`, `motors`, `gains`, `motion`, `sel`, `bat`, `ver`, and `help`. Iterate registry indices 0 through 20 and report `i + 1`. Expand `leftMotors` and `rightMotors` by physical index and report `ramp`, `top1`, `top2`; omit `intake`.

- [ ] **Step 4: Implement gain and arming commands**

`set` changes one current PID gain through `pidtuner::applyGains`; `arm` stores a 30-second deadline; `disarm` clears it; `stop` performs cancellation, inhibition, braking, and arm clearing before replying.

- [ ] **Step 5: Implement motion task and interlocks**

Use a fixed name→function table for `qual1`…`qual6`, `AI1`…`AI6`, `skills1`…`skills6`, and `1ring`. Reject motion if competition control is connected, arm expired/missing, another job is active/stopping, or autotune is open. Execute `drive`, `turn`, `moveto`, or `run` synchronously inside the dedicated task, then emit:

```text
[evt] <verb> done reason=none x=<f> y=<f> th=<f>
[evt] <verb> stopped reason=<host|controller|competition> x=<f> y=<f> th=<f>
```

For `drive`, compute the target from the current pose and heading and call `moveToPoint(..., 5000, ..., false)`; `turn` is relative to current heading and uses 3000 ms; `moveto` uses `moveToPose(..., 5000, ..., false)`.

- [ ] **Step 6: Implement the guard task**

At 10 ms, while motion is active: latch competition connection; latch any controller analog magnitude over 30 or any digital button; while stopping, repeatedly set `motionsInhibited`, cancel motions, brake both drive groups plus `ramp`/`top1`/`top2`. Never write serial. Clear inhibition and the stop latch only after the motion task reaches idle.

- [ ] **Step 7: Implement bounded stream task**

Support replacement and `stream off`. Enforce the robot-side deadline even if the host disconnects. Emit periodic `pose`, PID raw error/output, or per-physical-motor temperatures, followed by `[evt] stream expired` at natural expiry.

- [ ] **Step 8: Compile the runtime**

Run `make`. Expected: PROS build exits 0.

### Task 5: Ownership integration and documentation

**Files:**
- Modify: `src/main.cpp`
- Modify: `src/controls.cpp`
- Modify: `src/autotune.cpp`
- Modify: `CLAUDE.md`

**Interfaces:**
- `initialize()` starts DevBridge after `chassis.calibrate()`.
- Teleop and autotune consume `devbridge::isMotionActive()`.

- [ ] **Step 1: Start the bridge after calibration**

Include `devbridge.hpp` and call `devbridge::start()` immediately after `chassis.calibrate()`, before selector setup.

- [ ] **Step 2: Yield all teleop actuators**

`drivetrain_control` must skip drive writes while either bridge motion or autotune relay is active. `teleop_loop` must put a top-level `if (devbridge::isMotionActive()) { pros::delay(10); continue; }` before any motor or pneumatic write, preserving `is_heaven_up`.

- [ ] **Step 3: Make autotune mutually exclusive**

In the ARMED step, do not start the relay while `devbridge::isMotionActive()`. Keep the wizard on ARMED and render a clear refusal/status instead of moving motors.

- [ ] **Step 4: Document the first-party LemLib and ownership changes**

Update `CLAUDE.md` with DevBridge commands, source files, task ownership, gain helper, PID telemetry accessors, motion inhibition/mutex fix, and removed loop printfs.

- [ ] **Step 5: Run all automated verification**

Run:

```bash
make test-host
make clean
make
git diff --check
```

Expected: all host tests pass, clean PROS rebuild exits 0, and no whitespace errors are reported.

### Task 6: Safe first-contact hardware verification

**Files:**
- No source changes unless a reproducible hardware defect is found; any defect starts with the smallest available failing host test or diagnostic evidence.

**Interfaces:**
- Uses the connected controller-tethered brain and `tools/brain`.

- [ ] **Step 1: Upload only after confirming the bench is non-driving**

Because the connected brain is not mounted to the robot, upload the compiled firmware with `pros mu` only after confirming no unexpected actuator hardware is attached.

- [ ] **Step 2: Verify version and input path**

Run `tools/brain ver`. Confirm `[ok] ver built=...`, matching `[end] ver`, and exit 0.

- [ ] **Step 3: Verify safe query set**

Run:

```bash
tools/brain ports
tools/brain motors
tools/brain gains
tools/brain pose
tools/brain bat
tools/brain motion
```

Confirm each response terminates, no motion command is sent, and motor/port rows reflect the devices actually attached to this bench brain.

- [ ] **Step 4: Verify non-persistent gain round trip**

Read current gains, set one gain to its existing value, then read gains again. Do not alter source or use a new tuning value during this connection-only check.

- [ ] **Step 5: Record deferred robot-only checks**

Do not run `drive`, `turn`, `moveto`, `run`, relay autotune, controller deadman, field-control, or PID convergence checks until the complete robot is on blocks with wheels clear. Report these as explicitly unverified rather than passing.
