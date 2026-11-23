# GPS-aided odometry: design

Decided 2026-11-21 after a research pass (official VEX and PROS sources, the game
manuals, community reports) and three independent designs. This file is the
specification the code and the simulator tests are written against.

## What the research says (short)

- The V5 GPS sensor is a rear-facing camera that reads the black-and-white code
  strip on the field walls, plus its own IMU. It reports x, y in metres from the
  field centre (+x toward the 90 degree wall, +y toward the 0 degree wall) and a
  heading in degrees, 0 at the 0 degree wall, clockwise. That is the same sense
  of rotation as LemLib's heading.
- Strips are **required only on Autonomous Coding Skills fields**. On match
  fields they are the event partner's choice. So this must fall back to plain
  odometry with no strips, and its sure payoff is in Skills.
- Nobody publishes accuracy. Community reports: about 1 to 2 in of jitter
  standing still, worse while moving and near walls, a dead zone of roughly
  13 to 20 in from the wall the camera looks at, about one bad reading in ten,
  heading unreliable, about 25 updates per second, delay not published.
- The sensor already filters internally and keeps reporting a position from its
  own IMU when it cannot see the strip. **PROS returns no error in that case**,
  and PROS 4.1 has no `quality()` call, only `get_error()` (RMS metres, meaning
  not documented).
- PROS has no mounting-angle setting: a rear-facing sensor reports the camera's
  heading (robot heading + 180). What frame `set_offset` uses is undocumented.
- No team was found reporting a GPS Kalman filter that worked in real matches.
  The public top teams localise with distance sensors against the walls.
- The V5 inertial sensor has a consistent 1 to 1.4 percent heading scale error
  that calibration does not remove. That is a likely source of this robot's
  drift and costs one measured constant to fix.

## Decision

Build a deliberately cautious, position-only filter inside the odometry update.
Odometry stays the estimate from moment to moment; the GPS is a slow, heavily
checked correction. Heading stays with the inertial sensor (scale-corrected).

Reasons: the GPS is noisy, delayed and sometimes wrong in believable ways, so a
wrong reading must be able to do almost nothing; heading is its least trusted
output and the one that disturbs turns most; the team has to be able to
understand and tune every number; and it must be provable on the robot without
letting it move the robot first.

Not doing in this version: heading correction from GPS, scale states inside the
filter, velocity states, use of the GPS's own gyro or accelerometer, starting
the pose from GPS, any use while the driver is driving or on the PID tab.

## Where it lives

- `lemlib::update()` in `src/lemlib/chassis/odom.cpp` computes the dead-reckoned
  pose exactly as today, then (only when the mode is not Off) calls the fusion
  step. Every motion keeps calling `getPose()`. No motion file, parameter struct
  or call changes.
- New library files: `include/lemlib/chassis/gpsFusion.hpp`,
  `src/lemlib/chassis/gpsFusion.cpp`. The maths is in a class with no PROS
  dependency (`GpsFusionCore`) so it runs in the host simulator and in unit
  tests; a thin layer reads `pros::Gps`.
- With the mode Off (the default) the pose must be bit-for-bit what it is today.

## Frames

- Routines that want fusion are written in field coordinates: inches, origin at
  the field centre, +x toward the 90 degree wall, +y toward the 0 degree wall,
  heading 0 facing the 0 degree wall, clockwise. The routine calls `setPose`
  with its measured start pose, then fusion is switched on.
- The sensor's PROS offsets are set to zero. Our code converts metres to inches
  (x 39.3701) and removes the lever arm: robot centre = sensor position minus
  the mount vector (right, forward, inches) rotated by the robot's heading from
  odometry. The mount yaw (180 for a rear-facing camera) is used only to compare
  the GPS heading with ours as a sanity check.

## The filter

State kept by the filter, besides the pose odometry already has:

- `sigma` (in): how unsure odometry is. It grows as the robot moves:
  `sigma += kDist * |distance this tick| + kTurn * |angle this tick (rad)|`,
  capped at `sigmaMax`. Linear growth in the standard deviation is deliberate:
  odometry error is mostly systematic (scale, heading), not random.
- `pending` (in, a vector): correction decided but not yet applied.
- a ring buffer of the last 64 ticks: pose, and the total correction applied so
  far at that tick.

Each tick, after dead reckoning:

1. Read the GPS. A reading is **new** when x, y or heading differ from the
   previous read. If nothing has been new for `staleMs`, the sensor is treated
   as frozen and nothing is used.
2. Checks, in order. A reading failing any of them is dropped and counted:
   1. finite numbers (PROS error values are infinity);
   2. inside the field (|x|, |y| under 72 in);
   3. not stale;
   4. not in the camera's dead zone: cast a ray from the sensor along the
      direction the camera faces to the wall; drop if the wall is closer than
      `minViewIn`;
   5. heading agrees: `|wrap(gpsHeading - mountYaw - ourHeading)|` at most
      `headingGateDeg`;
   6. the robot is slow enough: speed at most `maxSpeed` and turn rate at most
      `maxTurnRate` (both from odometry);
   7. at most one reading every `minPeriodMs` is used (the sensor's output is
      already filtered, so consecutive readings are not independent).
3. Delay: the reading is compared with where we believed the robot was
   `latencyMs` ago: `past = history.pose + (appliedNow - history.applied) + pending`.
   `innovation = gpsRobotCentre - past`.
4. Statistical gate: drop if `|innovation|^2 > gate^2 * (sigma^2 + sigmaGps^2)`.
5. Update: `K = sigma^2 / (sigma^2 + sigmaGps^2)`;
   `pending += K * innovation`; `sigma = sqrt(1 - K) * sigma`.
6. Feed-in: each tick move at most `slewInPerS * dt` of `pending` into the pose
   that the motions read. Never a jump.

Safeguards:

- **Start check.** After switching on, the first `startSamples` readings that
  pass checks 1 to 7 are averaged. If they disagree with the pose by more than
  `startTolIn`, fusion faults ("start pose and GPS disagree": wrong start pose,
  wrong alliance, sensor mounted the other way) and does nothing for the rest of
  the run.
- **Relocalise or give up.** If `relocSamples` consecutive readings fail only
  the statistical gate and agree with each other (spread under `relocSpreadIn`),
  odometry has probably slipped (a shove). If their mean is within `relocMaxIn`
  of the pose, set `sigma` to that distance so the next reading is accepted;
  otherwise fault.
- **Shadow mode.** Everything runs and is logged, but step 6 feeds nothing in,
  so the pose is exactly today's odometry. The correction it would have applied
  is tracked separately. This is how the team proves it on the robot before
  trusting it.
- Faults latch until fusion is switched on again. A fault or Off leaves the
  pose alone; corrections already applied stay applied.

## Inertial sensor scale

`lemlib::setImuScale(clockwise, counterClockwise)` multiplies each heading
change from the IMU. Default 1.0, which is bit-for-bit today's behaviour. The
team measures it: turn the robot ten full turns each way and divide 3600 by the
reading.

## Numbers and their first values

| Name | First value | Meaning | How to measure |
|---|---|---|---|
| `mountRightIn`, `mountForwardIn` | 0, 0 | where the sensor is from the robot's centre | ruler |
| `mountYawDeg` | 180 | camera faces backward | look at the robot |
| `sigmaGpsIn` | 1.5 | GPS position noise (1 standard deviation) | log it standing still at several spots |
| `kDist` | 0.02 | odometry doubt gained per inch driven | drift measured on a taped route |
| `kTurn` | 0.3 | inches of doubt per radian turned | same |
| `sigmaMaxIn` | 12 | cap on doubt | none needed |
| `latencyMs` | 60 | delay of a reading | compare GPS and odometry on a drive |
| `maxSpeedInPerS` | 6 | use GPS only below this speed | raise once the delay is measured |
| `maxTurnRateDegPerS` | 30 | and below this turn rate | same |
| `minPeriodMs` | 100 | one reading used per this time | none needed |
| `staleMs` | 200 | unchanged this long = frozen | none needed |
| `minViewIn` | 20 | camera closer than this to the wall it faces = dead zone | log near a wall |
| `headingGateDeg` | 8 | GPS heading must agree with ours this well | log it |
| `gate` | 3 | readings more than 3 standard deviations off are dropped | none needed |
| `slewInPerS` | 2 | fastest a correction is fed in | simulator: command disturbance |
| `startSamples`, `startTolIn` | 5, 8 | start check | none needed |
| `relocSamples`, `relocSpreadIn`, `relocMaxIn` | 20, 2, 18 | shove recovery | simulator |

## Acceptance tests (simulator)

- A1. Mode Off: every existing drive, turn and pose result is byte-identical.
- A2. Mode On with no sensor, or a sensor that never sees a strip (frozen or
  drifting output): pose identical to Off.
- A3. A 60 second Skills-like route with odometry errors injected (IMU scale
  1.2 percent, wheel scale 1.5 percent, random slip), 50 seeds: fused median end
  error at most 2.0 in and at most 40 percent of odometry alone.
- A4. Do no harm: with perfect odometry and a noisy GPS, fused median end error
  at most 0.75 in.
- A5. Disturbance: corrections change the motor command by at most 5 of 127,
  and standard moves keep their overshoot within 0.1 in and settle time within
  50 ms of Off.
- A6. One reading in ten is an outlier of 6 to 24 in, dead zones are enforced by
  the model, and the view is lost for 2 s several times: result within 0.5 in of
  A3, and no outlier is ever accepted.
- A7. Start pose wrong by 10 in, or the frame mirrored: fault within 1 s, pose
  equals odometry.
- A8. True delay anywhere from 40 to 150 ms with 60 configured: stable; report
  how much accuracy is lost.
- A9. A 6 in sideways shove the wheels do not see: back within 2 in in 5 s.

## Stages on the robot

0. Calibrate odometry first: IMU scale, the tape test for wheel size, and the
   unresolved horizontal wheel sign. A filter hides these, it does not fix them.
1. Log only: read the GPS, print it beside odometry, measure noise, delay, the
   dead zone and which way the frame points. Nothing changes in the pose.
2. Shadow mode on a taped route: compare odometry, the fused estimate and the
   tape at the end. Go or no-go.
3. On, for Skills routines first.

## Revision 2 (2026-11-22): what the first build and its reviews changed

The first build matched this design (an independent model agreed with it over
19 million ticks) and Off and Shadow were proven identical to plain odometry.
On mode failed A5, A6, A7 and A9, and three reviewers found that a GPS which is
wrong in a believable way was limited only by the feed-in rate: a sensor that
lost the strip and drifted dragged the pose 24 to 44 in with no fault. The
changes below are decisions, in the order they matter. Where this section
disagrees with the text above, this section wins.

### R1. Hard limits on what the GPS can do

- **Step limit.** One reading may change the decided correction by at most
  `maxStepIn`. If `K * innovation` is longer than that, scale `K` down so it is
  exactly that long, and use the scaled `K` in `sigma = sqrt(1 - K) * sigma`.
  Readings are used at most every 100 ms and fed in at 2 in/s, so this costs no
  speed; it means one wrong reading can do at most 1 in.
- **Total limit.** If the total correction decided since switching on
  (the length of the `decided` vector) goes over `maxTotalCorrectionIn`, fault
  `TotalTooLarge`. A calibrated robot does not drift that far in a run, so the
  GPS is the likelier liar.
- **Faults give everything back.** On any fault, the correction already fed in
  is fed back out at the same rate (`slewInPerS`), so within a few seconds the
  pose is plain odometry again; nothing still pending is used. Shadow tracks
  the same thing in its would-be total. This gives the guarantee the team can
  reason about: with fusion On the pose is never further than
  `maxTotalCorrectionIn` from what odometry alone would say, and after a fault
  it is odometry alone.
- `sigmaMaxIn` 12 -> 5, so the gate can never grow wider than the
  relocalisation limit. `gate` 3 -> 2.5.

### R2. Relocalisation that can actually happen

The first numbers (20 readings within 2 in) could never be met with 1.5 in of
noise, so the shove recovery and its fault were unreachable. Now:

- A run is `relocSamples` (4) gate failures in a row (a reading that passes the
  gate ends the run; readings dropped by the checks neither add nor end it),
  all within `relocWindowMs` (2000) of each other.
- They agree if each is within `relocSpreadIn` (4.5 = 3 x `sigmaGpsIn`) of
  their mean.
- If the mean innovation is at most `relocMaxIn` (12): add the mean innovation
  to `pending` in one go (it is an average, so the step limit does not apply),
  set `sigma = sigmaGpsIn`, clear the run. Otherwise fault `RelocTooFar`.
- At most `relocMaxCount` (2) relocalisations per run; a third faults
  `RelocTooMany`. A robot that keeps getting "shoved" has a bad GPS.
- The total limit applies to relocalisations too.

### R3. Start check

- Nothing is ever corrected before the start check passes (unchanged).
- When it passes, `sigma` is set to `startSigmaIn` (1.0: how well the team can
  place the robot) instead of 0, so a start pose a few inches off is corrected
  at the first stop rather than after 5 s of driving.
- **Heading fault.** Of the first `startHeadingSamples` (10) readings that pass
  checks 1 to 4, if 8 or more fail the heading check, fault `HeadingDisagree`
  (field turned round, mirrored frame, wrong mount yaw). This is judged once.
  Before, those cases dropped every reading for the whole run in silence.
- No time limit on the start check: a robot that starts with its camera
  against the wall is in the dead zone and legitimately has no readings yet.
- The mount yaw is used for the heading check **and** for the dead-zone ray
  (the text above said "only" the heading check; that was wrong).

### R4. Smaller changes

- Check 6 also looks at the moment the reading was taken: the robot must have
  been slow over the 50 ms ending now **and** over the 50 ms ending `latencyMs`
  ago. Otherwise the first reading after every stop was taken at full speed.
- Check 5b (off by default): drop a reading if the sensor's own `get_error()`
  (converted to inches) is above `maxSensorErrorIn`. 0 = not used. PROS does
  not document what the number means; if stage 1 logs show it rises when the
  camera is covered, this becomes the blind-sensor check.
- The PROS layer never waits without limit for its lock. `setMode` stores the
  mode (an atomic) first, so Off takes effect even if the lock cannot be had;
  lock waits are a few ms, then give up. A competition task deleted while
  holding the lock must not be able to hang `opcontrol()`.
- `lemlib::update()` adds the correction only when it is not zero and the mode
  is still not Off.
- `setImuScale` is called before `chassis.calibrate()`; the result of
  `gps.set_offset(0, 0)` is checked and a failure printed once.
- **Mount guard.** `GPS_MOUNT_MEASURED` (robot-config.hpp, false). While it is
  false, asking for On gives Shadow and prints why. A camera 6 in off-centre
  with the mount left at 0 made fusion 4 in worse than odometry.
- The `[GPSFUSE]` line also prints the start check's progress, the average
  innovation in the robot's own frame (right, forward: a steady non-zero value
  there is a mount offset error and reads off directly) and the average of
  `innovation^2 / (sigma^2 + sigmaGps^2)` (about 2 when the noise number is
  right; much more means `sigmaGpsIn` is too small).

### What is still not solved

A GPS that cannot see the strip keeps reporting from its own motion sensors,
with no flag. Position alone cannot tell a slow drift of that kind from real
odometry drift within a few seconds: the noise (1.5 in) is as large as the
drift. The limits above bound the damage (at most 12 in, then a fault and back
to odometry); they do not detect it early. What would: `get_error()` if it
turns out to mean something (check 5b), or knowing which walls have strips.
Stage 1 must therefore include covering the camera for 10 s, standing still and
driving, and logging what the sensor reports.

The filter also cannot see: a constant GPS bias (it follows it), a GPS much
noisier than `sigmaGpsIn` says (at 3.6 in it is worse than odometry), or a
wrong mount offset below the start tolerance. Those are stage 1 and 2
measurements, and the stage 2 go/no-go is: Shadow's would-be error must be
clearly below odometry's on the taped route, or it stays off.

### Numbers changed or added

| Name | Value | Meaning |
|---|---|---|
| `sigmaMaxIn` | 5 (was 12) | cap on doubt |
| `gate` | 2.5 (was 3) | statistical gate |
| `maxStepIn` | 1.0 | most one reading may move the decided correction |
| `maxTotalCorrectionIn` | 12 | total correction above this = fault |
| `startSigmaIn` | 1.0 | doubt in the start pose once the start check passes |
| `startHeadingSamples` | 10 | readings judged for the heading fault (8 of them failing = fault) |
| `relocSamples` | 4 (was 20) | gate failures in a row |
| `relocWindowMs` | 2000 | all within this time |
| `relocSpreadIn` | 4.5 (was 2) | each within this of their mean |
| `relocMaxIn` | 12 (was 18) | mean further than this = fault |
| `relocMaxCount` | 2 | relocalisations allowed per run |
| `maxSensorErrorIn` | 0 (off) | check 5b |

### Acceptance tests, revision 2

The simulated GPS must be as noisy as the design assumes: 1.5 in per axis
standing still (the first suite's was 0.9). Limits that changed are marked.

- A1, A2, A8: unchanged.
- A3a. IMU 1.2 percent, wheels 1.5 percent, slips; the route's full spin run
  both clockwise and counter-clockwise; 50 seeds each. Fused median end error
  at most 2.0 in and at most 60 percent of odometry alone, both ways.
  (Changed: 40 percent held only for one spin direction. The gain depends on
  how wrong odometry is.)
- A3b. Calibrated robot (IMU 0.2 percent, wheels 0.5 percent): fused median end
  error at most 1.0 in and at most 0.2 in worse than odometry alone.
- A4. Perfect odometry: fused median end error at most 1.0 in. (Changed from
  0.75, which the honest noise level cannot meet.)
- A5. Judged on what the robot does, not on the believed pose:
  the largest pose change in one tick is `slewInPerS * dt`; on the standard
  moves the real robot's overshoot changes by at most 0.1 in and its settle
  time by at most 100 ms against Off; the tick-to-tick change in motor command
  during moves (99th percentile) is at most 5 of 127 above Off's. A move that
  starts from a corrected pose steers differently from the start; that is the
  correction working, and is reported, not limited.
- A6. One reading in ten an outlier of 6 to 24 in, dead zones, 2 s view losses:
  no reading moves the decided correction by more than `maxStepIn`; median end
  error within 0.5 in of A3a. Report how many outliers and blind readings were
  accepted. (Changed from "no outlier accepted", which a 1.5 in noise level
  makes impossible for a 6 in outlier.)
- A7. Start pose 10 in off; frame mirrored left-right; mirrored front-back;
  field turned 180; mount yaw wrong by 180. 20 seeds each. Every run faults,
  and before the fault the pose is within 1.0 in of odometry alone.
- A9. A 6 in shove: back within 2 in in 5 s in at least 19 of 20 seeds. A 15 in
  shove: fault, and the pose returns to odometry alone.
- A10. Blind sensor: a 20 s drifting view loss (1.5 in/s); strips on one wall
  only (four cases); a sensor that never saw a strip but starts at the right
  position. In every run the fused error never exceeds odometry alone's by more
  than `maxTotalCorrectionIn` + 1 in, a fault is raised when the limit is hit,
  and within 7 s of the fault the pose is within 0.5 in of odometry alone.
- A11 (reported, no limit). Constant 2 in bias; GPS noise 3.6 in with 1.5
  configured; camera 6 in off-centre with the mount declared measured.

## Revision 2 results and what was decided from them (2026-11-22)

Simulator only. Suite: `tools/sim/gps_suite.py`, 25 PASS, 1 FAIL, 45 reported.
Off is still bit-for-bit the pose from before the GPS work.

What passed: A1, A2, A3a clockwise (0.79 in against 2.67 for odometry alone),
A3b, A4 (0.82 in), A5, A6, A7 (120 of 120 wrong set-ups fault, nothing fed in
first), A8, A9 (19 of 20), A10 (worst 12.3 in, every run faults and returns to
odometry within 5.7 s).

What failed: A3a with the route's full spin counter-clockwise: 1.73 in against
2.60 (67 percent, limit 60). Cause, from runs with one number changed at a time:

- With the inertial sensor 1.2 percent wrong, a counter-clockwise spin leaves
  the heading 3.2 degrees off after 30 s (1.1 degrees clockwise). The filter
  corrects position, not heading, so every later leg rebuilds position error.
- Odometry alone partly cancels its own error on the way back across the field.
  A filter that corrected position at the earlier stops loses that
  cancellation: at one stop it is worse than odometry alone in 50 of 50 runs
  (7.9 in against 3.1). With a calibrated inertial sensor (A3b) this does not
  happen.
- The route's end is measured 0.5 s after the last move, with 1 to 1.5 in of
  decided correction still being fed in.

Decisions:

- No number is changed to make A3a pass. `slewInPerS` 4 helps one spin
  direction and hurts the other.
- **A calibrated inertial sensor is a condition for On, not advice.**
  `ODOMETRY_CALIBRATED` (robot-config.hpp, false) joins `GPS_MOUNT_MEASURED`:
  On gives Shadow until both are true. It is set once the IMU scale and the
  tracking wheel size have been measured (stage 0).
- Correcting heading from the GPS stays out. It would fix the cause, but it
  changes what every turn reads, and stage 0 fixes the same cause for free.
- What to expect, honestly: on a calibrated robot the filter gains about
  0.1 in at the end of a 60 s route (A3b) and costs about 0.8 in when odometry
  is perfect (A4). Its value is as a net under slips, bumps and whatever
  calibration misses. Shadow mode on the real robot shows which case this
  robot is in before anything is switched on.
- The `nis=` number on the `[GPSFUSE]` line reads about 1.4 for a healthy
  sensor in the simulator, not 2, and 6 for one that is noisier than
  configured. Take the healthy value from stage 1 logs.
- Costs of the limits, accepted: a shove of 11 in or more faults instead of
  recovering; with uncalibrated odometry a 6 in shove recovers in 13 of 20
  runs and faults in 6.

## Revision 3 (2026-11-22): what the reviews of revision 2 changed

Three independent reviews of revision 2 (a second model written from this
document alone, an attacker in the simulator, and a reading of the robot side
against the PROS kernel). No coding error in the filter: the second model
agreed with it over 4.17 million ticks. What they did find:

1. **A slowly wrong GPS is followed with no fault.** A blind sensor drifting
   0.2 in/s leaves a calibrated robot a median 9 in off at the end of the
   route (odometry alone: 0.95 in), 0 of 20 runs fault. Stepped bias and drift
   only while standing still do the same. Position readings alone cannot tell
   a slow liar from odometry drift; only a second witness can (the sensor's own
   error number if stage 1 shows it works, or wall sensors).
2. **Relocalisation is the fastest way in for a lie.** In the outlier test, 5 of
   9 relocalisations were built on blind readings and moved the pose 5 to 9 in
   on one reading.
3. **Printing from the odometry task can stop odometry.** PROS's serial write
   waits without a time limit, and a competition task deleted in the middle of
   a print leaves the serial lock held for good.
4. **`setPose` while fusion is on** leaves the filter believing the pose still
   holds its old corrections; a later fault then takes them out of a pose that
   no longer has them (5 in in the test).
5. **The guards could be walked round** by calling `setMode(On)` directly.
6. A start on a mirror line of the field passes the start check with a mirrored
   frame; about 3 in is fed in before the fault at about 9 s.
7. On a calibrated robot the filter is a wash, not a gain: at the end it is
   within 0.2 in of odometry alone either way depending on when the end is
   measured, and at stops it is more than 1 in worse about as often as it is
   more than 1 in better (6 percent each).

Decisions:

- **What the filter is for is narrowed**: a calibrated robot, on a field with
  strips on every wall, alone (Skills). There a real jump of the robot is rare
  and a lying GPS is the main danger, so every limit is set to give up early.
  The total limit, the relocalisation distance and the relocalisation count
  are chosen from three candidates (12 in / 12 in / 2 as now; 8 / 8 / 1;
  6 / 6 / 1) by one rule fixed before the runs: the smallest for which healthy
  runs on a calibrated robot and on perfect odometry fault in at most 1 of 150.
  The shove test (A9) and the blind test (A10) are restated for the numbers
  chosen, and slow drift, stepped bias and standing drift join A10 as reported
  rows with the worst excess stated.
- The odometry task never prints or formats text. A small logging task, started
  in `configure()` at a lower priority, copies the status without waiting and
  prints `[GPSFUSE]` and `[WALLFIX]`. Messages from `setMode` go through it too.
- `lemlib::setPose` tells the filter when fusion is not Off. The filter starts
  again from the new pose, as `setMode` does: nothing is fed back out, the
  start check runs again. Corrections fed in before a `setPose` or a second
  `setMode` stay in the pose; R1's promise is per switch-on.
- The guards move into the library: `configure()` is told whether On is
  allowed (`GPS_MOUNT_MEASURED && ODOMETRY_CALIBRATED`); `setMode(On)` gives
  Shadow otherwise and says why. Routines use `startGpsFusion()`.
- The heading fault judges only readings taken while the robot is slow
  (check 6), so a routine that opens with a turn cannot trip it.
- `configure()` checks the numbers and warns once per number outside a sane
  range; a `latencyMs` the history cannot reach is refused. `nohist=` goes
  back on the line.
- Mirror-line starts (6) get no new mechanism: about 3 in for about 9 s, then a
  fault, and Shadow on the field shows it beforehand. A7 runs more start poses
  and reports the on-line ones.
- A6 reports relocalisations and what they were built on. A3b gets a row for
  stops and its end is measured at several moments.
- **Recommendation that follows:** Shadow only at nationals. On needs stage 1
  to show that the sensor's own error number rises when the camera is covered
  (check 5b), or wall sensors as the second witness.

### Numbers after revision 3

Chosen on 2026-11-22 by the rule fixed above (simulator only): the smallest candidate whose healthy runs
fault at most once in 150 (calibrated robot, both spins, 50 seeds each; perfect odometry, 50 seeds).

| Name | Value | Was |
|---|---|---|
| `maxTotalCorrectionIn` | 6 | 12 |
| `relocMaxIn` | 6 | 12 |
| `relocMaxCount` | 1 | 2 |

| Candidate (total / reloc / count) | Healthy runs that faulted (of 150) | A10 worst excess (6 blind cases) | 0.2 in/s drift from 15 s, calibrated: median end (odometry 0.82), faults of 20 | Shoves 2 / 4 / 6 / 8 in: back within 2 in in 5 s (of 20), faults |
|---|---|---|---|---|
| 12 / 12 / 2 | 0 | 12.3 in | 9.00 in, 0 | 20 / 20 / 19 / 20; none |
| 8 / 8 / 1 | 0 | 8.4 in | 1.98 in, 13 | 20 / 20 / 19 / 2; 0 / 0 / 5 / 20 |
| 6 / 6 / 1 | 0 | 5.9 in | 0.98 in, 20 | 20 / 20 / 10 / 0; 0 / 6 / 20 / 20 |

### Revision 3 results (simulator, 2026-11-22)

Built and run. Off is still bit-for-bit the pre-GPS pose. Unit tests: 444 + 310
+ 102 + 198 checks pass. ARM build compiles (hot 32752 bytes, cold 3.7 MB).

- Healthy sensor, calibrated robot: 0 faults in 150 runs; end error 0.70 in
  against 0.82 (0.65 to 0.83 depending on when the end is measured); at stops
  more than 1 in worse 6.1 percent of the time, more than 1 in better 6.2.
- Blind sensor (the six A10 cases): worst 5.9 in worse than odometry alone,
  every run faults, back to odometry within 2.7 s.
- Slow liars on a calibrated robot: 0.2 and 0.3 in/s drift, drift only while
  standing, stepped bias: all 20 of 20 fault, after reaching about 6 in.
  **0.1 in/s drift: only 4 of 20 fault; the pose ends up to 5.4 in worse
  (median 2.6).** This is the case no number fixes.
- Shoves: 4 in or less recovers (20 of 20). 6 in recovers in 10 of 20. 8 in or
  more faults and goes back to odometry alone.
- Relocalisation on blind readings still happens: 3 in 50 outlier runs, the
  largest moving the pose 5.2 in in one tick.
- Uncalibrated robot: the 6 in limit faults in 41 to 48 of 50 healthy runs.
  That is the limit doing its job outside what the filter is for; those rows
  are now reported, not limited.
- The heading fault now comes at the first stop (about 1.6 s on the test
  route), not at 0.3 s, because it only judges readings taken while slow.

Not checked anywhere but on a robot: the logging task's stack (8 KB) and
priority, and that `configure()` starting a task from `initialize()` behaves.
