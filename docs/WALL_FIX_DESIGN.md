# Wall fixes from distance sensors: design

Decided 2026-11-21 after a research pass and three independent designs (an
explicit reset the routine calls, automatic one-axis fixes inside the filter, a
particle filter). This is the specification the code and simulator tests are
written against. It builds on `docs/GPS_FUSION_DESIGN.md`: same frame, same
filter, same modes.

## What the research says (short)

- The V5 distance sensor is a laser rangefinder rated 20 to 2000 mm, about
  15 mm accurate under 200 mm and about 5 percent beyond. Its beam is a cone
  (about 12 to 18 degrees either side), not a dot. About 30 readings a second.
- **The walls decide whether it works.** On the older steel perimeter with clear
  plastic panels, teams report the beam passing through, reading things behind
  the wall, or dropping out beyond about 700 mm. On the portable perimeter the
  same sensor read to about 1800 mm. Reports conflict. The venue's wall type is
  unknown.
- What teams actually use is a fix taken while stopped, square to a wall and
  close to it, correcting one coordinate. No team has published measured
  accuracy from matches for this or for particle filters. The best-instrumented
  public particle filter lists failures where it settled tens of inches off.
- This robot has no wall-facing sensor. Its two distance sensors look at game
  objects. Smart ports 6 and 21 are free: two wall sensors at most.

## Decision

Wall readings become a second kind of measurement in the same filter as the
GPS. The filter takes a fix on its own, with no call from the routine, but only
when a sensor plainly has a clean, close, square view of one flat wall while the
robot is standing still. A flat wall pins down one coordinate, so a fix corrects
one axis. Heading stays with the inertial sensor.

Why this one: it needs no change to routines or movement code; near a wall,
square and still is exactly where the reading is trustworthy and where the GPS
is at its worst, so the two cover each other; and with the heading known and
four straight walls, a one-axis Kalman update is the exact answer that a
particle filter would only approximate.

Not doing: a particle filter (two ports, unknown walls, believable wrong
answers, about ten coupled tuning numbers); heading correction from walls;
fixes while moving; learning wall positions; starting the pose from walls; using
the claw or pickup sensors; any use in driver control or on the PID tab.

## What changes in the filter

- The single doubt `sigma` becomes `sigmaX` and `sigmaY`. Both grow by the same
  rule as before. The GPS update and gate work per axis:
  `Kx = sigmaX^2 / (sigmaX^2 + sigmaGps^2)`, the same for y, and the gate is
  `ix^2 / (sigmaX^2 + sigmaGps^2) + iy^2 / (sigmaY^2 + sigmaGps^2) <= gate^2`.
  While the two doubts are equal this is the filter as it was. With no wall
  sensor configured, GPS-only results must match the previous filter to within
  0.0001 in.
- A doubt never goes below `sigmaFloorIn`, so repeated fixes cannot make the
  filter overconfident.
- Wall corrections go into the same `pending` and are fed in at the same rate
  limit, so the motions never see a jump and nothing is counted twice.
- Faults are kept per source: a wall sensor can fault without stopping the GPS,
  and the other way round.

## Geometry

Frame as in the GPS design: inches from the field centre, +x toward the 90
degree wall, +y toward the 0 degree wall, heading `t` clockwise from +y.
Forward is `(sin t, cos t)`, right is `(cos t, -sin t)`.

- Walls are four planes: `x = +wallPlusX`, `x = -wallMinusX`, `y = +wallPlusY`,
  `y = -wallMinusY`. First value 70.25 in each (140.5 in inside width). An
  error in a plane is a direct bias in that coordinate, so measure the field.
- Sensor: mount `rightIn`, `forwardIn` (to the sensor's face), `yawDeg` (0
  forward, 90 right, 180 back, 270 left), a scale and an offset from bench
  calibration (first values 1 and 0).
- Reading: `m = scale * mm / 25.4 + offset` inches.
- Sensor position: `S = (x + r cos t + f sin t, y - r sin t + f cos t)`.
  Beam heading `phi = t + yaw`, direction `d = (sin phi, cos phi)`.
- Cast the ray from `S` along `d` to the four planes; the nearest hit is the
  wall, at predicted range `p`.
- A wall `x = Xw` implies `x = Xw - m * d.x - (S.x - x)`; a wall `y = Yw`
  implies `y = Yw - m * d.y - (S.y - y)`. The innovation on that axis is
  `(p - m) * d.x` (or `d.y`). The other axis is not touched.
- Noise on that axis: `R = (sigmaWall * |d.along|)^2 + (m * |d.across| * sigmaHeading)^2`
  where `sigmaWall = wallNoiseIn + wallNoiseFrac * m`, `d.along` is the
  component toward the wall and `d.across` the component along it. The second
  term is what a heading error costs when the beam is not square.

## Checks, in order

A reading failing any check is dropped and counted under that check.

1. Valid: not an error value, not 0, not 9999, between 20 and 2000 mm.
2. Confident: above 200 mm, `get_confidence()` at least `minConfidence`
   (`minConfidenceClear` on a wall marked Clear).
3. Still: speed at most `wallMaxSpeed` and turn rate at most `wallMaxTurnRate`
   over the last 100 ms, from the pose history. Because the robot is still, no
   delay handling is needed.
4. Steady: the last `steadyCount` valid readings, taken at least 35 ms apart,
   span at most the larger of 0.5 in and 2 percent of their median. The median
   is the reading used.
5. One wall: the ray hits a wall whose trust is not Ignore.
6. Square enough: the angle between the beam and the wall's normal is at most
   `maxIncidenceDeg`.
7. In range: both `p` and `m` between `minRangeIn` and the wall's maximum range
   (`maxRangeClearIn` or `maxRangeSolidIn`).
8. Plain wall: rays at `phi` plus and minus `coneHalfDeg` hit the same wall,
   and the cone's footprint on the wall, widened by `footprintMarginIn`, does
   not overlap any keep-out segment listed for that wall (this season's
   wall-mounted elements) or come within `cornerMarginIn` of a corner.
9. Not blocked: if `m < p - shortGate`, something is in the way. Dropped. This
   one-sided rule is the main defence against a robot or game object in the
   beam. `shortGate` is the larger of `blockedMinIn` and twice the noise.
10. Not through: if `m > p + longGate`, the beam went through or past the wall.
    Dropped.
11. Statistical gate: `innovation^2 <= gate^2 * (sigmaAxis^2 + R)`.
12. Cap: the correction `K * innovation` is at most the wall's cap
    (`capClearIn` or `capSolidIn`). Larger ones are dropped, not clipped.
13. Once per stop: after a sensor gives a fix, it gives no other until the
    robot has travelled `refixTravelIn` or turned `refixTurnDeg`. Repeated
    readings from a still robot share the same error and must not be counted
    as independent.

Then `K = sigmaAxis^2 / (sigmaAxis^2 + R)`; `pending += K * innovation` on that
axis; `sigmaAxis = max(sigmaFloorIn, sqrt(1 - K) * sigmaAxis)`.

Safeguards:

- **Wall trust.** Each wall is Solid, Clear or Ignore. Every wall starts Clear
  (short range, strict confidence, small cap) until the team has proven on that
  kind of wall that the sensor reads it. A steady read-through to something
  just behind a clear panel cannot be told from a good reading; the Clear cap
  bounds what it can do.
- **Start check, per sensor.** The first reading that passes checks 1 to 8 is
  compared with the pose. If it disagrees by more than `wallStartTolIn`, that
  sensor faults for the run.
- **Health, per sensor.** If, over at least `healthMinTries` readings that
  passed the geometry checks (5 to 8), half or more were invalid, low
  confidence or through, that sensor is switched off for the run. This is how
  it finds out the walls are clear.
- **Walls never relocalise.** Only the GPS path can recover from a shove.
- Shadow, Off and faults behave as in the GPS design.

## Numbers and their first values

| Name | First value | Meaning | How to measure |
|---|---|---|---|
| wall planes | 70.25 each | inside face of each wall from the field centre | tape, three places per wall |
| mount right, forward, yaw | per sensor | where the sensor is and which way it points | ruler |
| scale, offset | 1, 0 | bench calibration of the reading | readings against a tape at 6, 12, 24 in |
| `wallNoiseIn`, `wallNoiseFrac` | 0.3, 0.01 | reading noise: fixed part and per-inch part | log it standing still |
| `sigmaHeadingDeg` | 1 | how wrong the heading may be at a fix | drift on a taped route |
| `minConfidence`, `minConfidenceClear` | 40, 50 | of 63 | log it |
| `wallMaxSpeed`, `wallMaxTurnRate` | 1 in/s, 3 deg/s | "still" | none needed |
| `steadyCount` | 5 | readings that must agree | none needed |
| `maxIncidenceDeg` | 10 | how far off square | bench: error against angle |
| `minRangeIn` | 1.5 | | none needed |
| `maxRangeClearIn`, `maxRangeSolidIn` | 24, 36 | farthest wall used | wall test at the venue |
| `coneHalfDeg`, `footprintMarginIn`, `cornerMarginIn` | 18, 2, 6 | beam width and safety margins | bench |
| `blockedMinIn` | 1.0 | shorter than predicted by this = blocked | none needed |
| `longGate` | 3 standard deviations | longer than predicted by this = through | none needed |
| `capClearIn`, `capSolidIn` | 1.5, 6 | largest single correction | none needed |
| `refixTravelIn`, `refixTurnDeg` | 6, 20 | movement needed before another fix | none needed |
| `sigmaFloorIn` | 0.3 | doubt never goes below this | none needed |
| `wallStartTolIn` | 3 | start check | none needed |
| `healthMinTries` | 6 | readings before the health rule can act | none needed |

## Acceptance tests (simulator)

- W1. No wall sensor configured: GPS-only results A1 to A9 match the previous
  filter within 0.0001 in, and mode Off is byte-identical.
- W2. Skills-like route with stops near walls and drifting odometry, GPS off.
  One back sensor: median end error at most 60 percent of odometry alone. Two
  sensors (back and one side): at most 40 percent. 50 seeds.
- W3. Do no harm: perfect odometry, noisy sensors: median end error at most
  0.3 in above odometry alone.
- W4. An object in the beam at every fix (6 to 20 in from the sensor): no fix
  accepted; the pose equals odometry alone.
- W5. Clear walls: the beam reads through to objects 2 to 40 in behind, and
  drops out beyond 28 in. No single error added above 1.5 in; the health rule
  switches the sensor off; end error at most 1.0 in above odometry alone.
- W6. Heading wrong by 2 degrees at a fix: error added at most 0.2 in.
- W7. Stops near corners and keep-outs: no fix with the cone across a corner
  or a keep-out.
- W8. Start pose wrong by 5 in: the sensor faults; pose equals odometry alone.
- W9. Disturbance: corrections change the motor command by at most 5 of 127;
  standard moves keep overshoot within 0.1 in and settle time within 50 ms.
- W10. GPS and walls together: end error no worse than the better of the two
  alone by more than 0.3 in.
- W11. A wall plane wrong by 0.5 in: report the bias (expected about 0.5 in on
  that axis); it must not grow.

## Hardware and stages on the robot

0. As for the GPS: calibrate odometry first.
1. Buy and mount one or two distance sensors on ports 6 and 21: rigid, level,
   beam 5 to 7 in above the tiles. First choice: one facing back. Second: one
   side, chosen by where the routines stop.
2. Bench: scale and offset against a tape; error against angle; a clear panel
   and a portable-perimeter panel with and without something 12 in behind.
3. Log only on the practice field, then Shadow on a taped route.
4. On, Skills first. At each event, check the walls in Shadow on the practice
   field before trusting them; mark walls Solid only when proven.

## Revision A (2026-11-22): fitting the walls into revision 2 of the GPS filter

Decided after the GPS filter's second build. Where this disagrees with the text
above, this wins.

### Doubt, limits and faults

- `sigmaX` and `sigmaY` both start at `startSigmaIn` when fusion is switched
  on, and are both set to `startSigmaIn` when the GPS start check passes (as
  revision 2 does for its one doubt). Both grow by the same rule and are capped
  at `sigmaMaxIn`. A relocalisation sets both to `sigmaGpsIn`.
- GPS step limit: the step is the vector `(Kx*ix, Ky*iy)`. If it is longer than
  `maxStepIn`, both gains are scaled by the same factor so that it is exactly
  `maxStepIn`, and the scaled gains are the ones used to shrink the doubts.
  With equal doubts this is revision 2 exactly.
- A wall fix is not subject to `maxStepIn`. Its own cap (check 12) applies.
- `sigmaFloorIn` belongs to the wall numbers and is applied only when at least
  one wall sensor is configured, so GPS-only results do not change.
- The total limit (`maxTotalCorrectionIn`) counts every correction decided,
  from either source.
- Faults come in two stages.
  1. While the GPS is in use, any fault of revision 2 (start, heading,
     relocalisation, total) takes the GPS out for the run and feeds everything
     fed in so far back out, as now. Wall fixes pause while that happens. When
     the pose is plain odometry again, wall sensors that are healthy carry on
     alone: both doubts are set to `sigmaMaxIn`, the total starts again from
     zero, and each sensor may fix again at once.
  2. A total-limit fault with the GPS already out (or never configured) stops
     the walls as well, and feeds their corrections back out.
- A wall sensor that fails its start check or its health rule stops for the
  run. The GPS and the other wall sensor carry on. Nothing is fed back out:
  each of its fixes was gated and capped.

### Blocked and through (checks 9 and 10), restated

The text above set these gates from the reading's noise alone. That refuses any
real correction above about an inch: the fix a drifted robot needs looks the
same as a blocked or passed-through beam. The gates now follow how unsure the
pose is, between fixed limits:

- `s = sqrt(sigmaAxis^2 + R)`: one standard deviation of the innovation.
- Blocked (check 9): dropped if the reading is shorter than predicted by more
  than `shortGate = min(blockedMaxIn, max(blockedMinIn, 2*s))` along the axis.
- Through (check 10): dropped if longer than predicted by more than
  `longGate = min(throughMaxIn, 3*s)`.
- First values: `blockedMinIn` 1.0, `blockedMaxIn` 2.5, `throughMaxIn` 4.0.

So an object less than about 2.5 in in front of the wall cannot be told from
drift and is bounded by the cap, and a sensor whose pose has drifted further
than the gates from the truth can no longer fix that axis (the GPS or the next
wall on the other side has to). Both are measured by the tests below.

### Sampling

Each sensor is sampled at most once every 35 ms. Check 3 (still) uses the pose
history with the corrections fed in taken out, like the GPS speed check. The
prediction uses the filter's estimate (pose plus what is decided but not yet
fed in).

### Walls and keep-outs in code

Walls are numbered 0 = +y (the 0 degree wall), 1 = +x (90), 2 = -y (180),
3 = -x (270). A keep-out is a wall number and a from/to position along that
wall, in field inches. At most 2 wall sensors and 8 keep-outs, in fixed arrays.

### Tests changed

- W4. Objects at least 3 in in front of the wall: no fix accepted, pose equals
  odometry alone. Objects 1 to 3 in in front of the wall: report the error
  added per fix and at the end (no limit; expected under `blockedMaxIn`).
- W2 also reports how many stops a clean reading was refused as blocked or
  through, and the largest pose error at such a stop.
- W12 (new). GPS that faults at the start (no strips on the field) with wall
  sensors fitted: the walls still work; end error within 0.3 in of the
  walls-only result.
- W13 (new). Total limit with walls only: a wall plane wrong by 14 in on a
  Solid wall. The filter faults and the pose returns to odometry alone.

### On the robot

No wall sensor is fitted, so the robot's configuration has none and the wall
code never runs there. The simulator is the only place it runs for now.

## Revision B (2026-11-22): after the first wall build and suite

Suite: `tools/sim/wall_suite.py`. The sensor model is built from reports, not
from measurements, so the numbers compare filter behaviour, not real accuracy.

- Calibrated robot, Clear walls, one back sensor: end error 0.41 in against
  0.92 for odometry alone (45 percent). A second sensor adds nothing on this
  route (46 percent): it has no stop to use after 32 s.
- Uncalibrated robot: no gain with Clear walls (90 and 107 percent); the 1.5 in
  cap refuses four corrections for each one it takes. With Solid walls 50
  percent. On needs a calibrated robot (as for the GPS), so uncalibrated rows
  are reported, not limited.
- W8 as written cannot be met: a sensor's start check sees only the axis of the
  first wall it looks at. What happens instead is harmless: readings on the
  axis that was set wrong are refused by checks 9 and 10, and the other axis is
  still fixed. W8 becomes: with a 5 in start error, no fix makes the wrong axis
  worse, and the sensor faults when the error is on the axis it sees first.
- W13 as written cannot happen: a plane 14 in wrong is refused by the start
  check or by checks 9 and 10 long before the total builds up. W13 becomes: the
  total limit works when it is reached (shown with the limit lowered), and a
  plane wrong by 2 to 3 in is reported: it gets through and biases that axis by
  up to 3.6 in with no fault. Measure the field.
- W4 with objects appearing later: on a calibrated robot no fix came from a
  reading of an object. On an uncalibrated one an object can hide inside the
  drift (reported).
- Known thin spots: W10 (GPS and walls together) passes by 0.0007 in, and the
  GPS makes the walls-only result worse at the last stop (0.66 against 0.36).

## Revision C (2026-11-22): what the review of the wall build found

Two reviews on a frozen copy (the code against this design; an attacker in the
simulator). No coding error in the geometry or the checks, and with no wall
sensor configured the wall code cannot change the pose. The design itself is
not ready. **Nothing below is fixed yet.**

What the simulator number really is:

- 0.41 in against 0.92 is the error at one moment, 2 s after the last fix.
  Averaged over the whole route the gain is about a quarter (RMS 0.55 against
  0.75). Moving the route's last point gives 45, 70 or 88 percent. In 10 of 50
  seeds the fused pose ends worse than odometry alone.

What gets through every check and makes a calibrated robot 2 to 3 times worse
than odometry alone, with no fault and no sensor switched off:

- an object resting 1 to 2 in in front of the wall at the stops (capped fixes
  add up from stop to stop: the cap bounds one fix, not the total);
- the field 1 to 2 in smaller than the filter was told;
- the sensor's mount told 1 in too far forward, or an offset of -1 in;
- a clear panel with something steady 2 in behind it, if the panel does not
  lower the sensor's confidence (the model guesses that it does);
- a GPS biased 2 in: it overrides the walls completely, and the honest wall
  sensor is then switched off by its own health rule.

So wall sensors are not the "second witness" against a lying GPS that GPS
revision 3 hoped for, not as designed here. With the GPS on, walls add almost
nothing (worse than walls alone in 35 of 50 seeds).

Open design problems, in the order they matter:

1. Bound what wall fixes add together on one axis, not only per fix; or need
   agreement from a second wall before repeated same-direction fixes.
2. Decide who wins near a wall: a fresh wall fix should limit or veto a GPS
   relocalisation on that axis, and the health rule must not read a GPS error
   as a clear wall.
3. Watch the mean innovation per sensor and per wall, and fault a sensor whose
   mean stays above about 0.5 in: that is what a wrong plane, mount or offset
   looks like.
4. The health rule: count valid readings as good tries whether or not they
   complete a steady row (a wall with 15 percent dropouts switches the sensor
   off now); do not count "through" as bad while the axis doubt is large.
5. Check 13: re-arm on distance from the pose of the last fix, not on path
   length, which jitter at rest adds up.
6. Apply the doubt floor only in the wall update, so that merely configuring a
   wall sensor does not make a biased GPS pull harder.
7. Refuse `configureWalls` unless the mode is Off.
8. An unplugged sensor shows as HEALTH, the same as clear walls: give it its
   own state.

Before any of this is worth doing, measure a real sensor: offset and scale on
the bench at 6 to 24 in; readings and confidence square to the venue's kind of
wall with something 1 to 4 in behind it; readings with a game object against
the wall; the field's wall planes at three places per wall.
