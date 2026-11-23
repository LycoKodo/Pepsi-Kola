# The UMBmark square: checking the odometry

The robot drives a 48 in square (two tiles a side) on its own odometry and comes back to where it started.
It *believes* it is back at the start. You measure where it *really* is. The difference is the odometry's
error. Five runs clockwise (CW), five counter-clockwise (CCW), then a laptop tool works out the
inertial sensor's scale error and the two numbers to type into `include/robot-config.hpp`.

**Nothing here has run on the real robot yet.** Everything below was checked in the simulator
(`tools/sim`, the real LemLib code) and by building the program. The first session at the robot is
also the first real test: do the checks in section 7 first.

## 1. What it can and cannot tell you

| Error | Does the square see it? | If not, what does |
|---|---|---|
| Inertial sensor scale (reads 1.2 % too much) | Yes: 2.55 in off and 4.3 deg turned at the end | |
| Heading drift while standing still | Yes, but it looks like a scale error, so the tool refuses to give numbers above 0.02 deg/s | Restart with the robot still |
| Wheel slip | Yes, as scatter between runs that should be the same | Drive slower, clean the wheels |
| Tracking wheel size | No (every leg is wrong by the same amount, the square still closes) | Drive along a tape measure |
| Tracking wheel offsets, even the wrong sign | No (under 0.1 in at the end) | The PID tab's turn test and `[ODOMCAL]` lines |
| A tracking wheel not quite straight | No | Look at it with a square |
| Robot spinning about a point that is not its tracking centre | Only if you measure the heading too (see section 5) | `[ODOMCAL]` and the hand-turn check in `robot-config.cpp` |

## 2. What you need

- Floor space. The robot starts centred on a tile. Clockwise needs 3 clear tiles ahead and 3 to the
  right, and 1 behind and 1 to the left. Counter-clockwise: 3 ahead and 3 to the LEFT, 1 behind and
  1 to the right. (An 18 in robot's corners reach about 61 in ahead and to the side of the start
  centre, and about 13 in behind.)
- Masking tape, a pen, a tape measure or ruler that reads 1/8 in, a protractor if you have one.
- The laptop on the USB cable or the radio, to capture the robot's log. Without it you can copy the
  numbers off the controller screen, but the tool is less accurate.
- The lift fully down, no competition switch or field cable plugged in.
- Each run drives for about 17 s (13.7 s of square). Measuring takes longer than driving.

## 3. Steps at the robot

1. On the laptop: `pros terminal --output umb_round1.log`. **One log file per round**
   (`umb_round2.log` next time). If rounds share a file, the tool will ask which set you mean.
2. Put the robot centred on a tile, square to the tile lines.
3. Mark the start:
   - **Centre mark (simple form):** tape a small pointer to the robot so its tip sits over the
     tracking centre (the point the wheel offsets were measured from: the centre of the robot).
     Put a tape cross on the floor exactly under the tip.
   - **Two corners (corners form):** a tape cross under each back corner of the robot. Measure the
     distance between the two corners, and how far the line joining them is behind the centre.
   - **Either way:** a long strip of tape on the floor along one side of the robot. It shows which
     way the robot ended up turned.
4. On the brain, Skills tab: tap **UMB CW** (or **UMB CCW**).
5. On the controller: **hold A, press B.** The robot stands still for 2 s (do not touch it), then
   drives the square. Keep your thumb near a stick: **pushing any stick stops it at once** (it brakes,
   then gives you control when you let go). It also stops by itself if the inertial sensor stops
   working, a move takes too long, or someone starts a PID tab test. A stopped run has no result:
   just do it again.
6. When it stops for good, the controller shows `X+0.12 Y-0.31` and `H+359.62 CW #3`: where the robot
   thinks it is, and the run number. Do not move the robot yet.
7. Measure FROM the start mark TO where the mark is now, as the robot faced at the start:
   - `right_in`: to the right is +, to the left is -.
   - `forward_in`: forward is +, back is -.
   - Corners form: the same two numbers for each corner, from its own cross.
   - `turned`: look along the tape strip. Is the robot turned clockwise (CW) or counter-clockwise (CCW)
     from it, seen from above? Write `0` if it is straight. One degree shows as 0.3 in over the robot.
   - `heading_deg` (simple form, if you have a protractor): how far it is turned, clockwise +.
8. Put the robot back on its start marks and do the next run. Five CW, five CCW, in any order, but
   write the rows in the order you drove them.

## 4. The file and the command

`python3 tools/umbmark.py --template simple` (or `corners`) prints a file to start from. A filled-in
simple file looks like this (this one was written by the simulator for a sensor 1.2 % off):

    dir,right_in,forward_in,heading_deg,turned
    CW,1 3/8,-1 7/8,,CCW
    CW,1 5/8,-1 3/4,,CCW
    CW,1 5/8,-1 7/8,,CCW
    CW,1 7/8,-1 7/8,,CCW
    CW,1 3/8,-2 1/4,,CCW
    CCW,-1 3/4,-1 5/8,,CW
    CCW,-1 3/4,-1 1/2,,CW
    CCW,-1 3/8,-2,,CW
    CCW,-2 1/4,-1 3/4,,CW
    CCW,-2 1/8,-5/8,,CW

Then run (the plot needs matplotlib; without it the tool still works):

    python3 tools/umbmark.py umb_round1.csv --log umb_round1.log --plot umb_round1.png

The report for that file (shortened):

    Inertial scale error (+ = the robot thinks it turned more than it really did):
      clockwise:         position +1.17 ± 0.03 %
      counter-clockwise: position +1.14 ± 0.06 %
    Type these into include/robot-config.hpp, in place of the scales these runs were made with
    (CW 1.0000, CCW 1.0000: check the file still says that, or these numbers are wrong):
      constexpr float IMU_SCALE_CLOCKWISE = 0.9884f;
      constexpr float IMU_SCALE_COUNTER_CLOCKWISE = 0.9887f;
    Then drive the squares again with the new scales: the error should come out close to 0.

## 5. Reading the result

- **Numbers printed:** check `include/robot-config.hpp` still has the scales the report says the runs
  were made with. Change `IMU_SCALE_CLOCKWISE` and `IMU_SCALE_COUNTER_CLOCKWISE` to the new numbers,
  build, upload, and drive another 5 + 5 into `umb_round2.log`. The new error should be small. When
  the report says **"This test is done"**, stop: more rounds only swap one bit of slip for another.
- **"NO NEW NUMBERS THIS TIME":** the reasons are at the top, each with what to do. The common ones:
  - *seen turned the other way*: a sign or a column is the wrong way round. Measure from the start
    mark to the robot's mark, right +, forward +.
  - *nothing in the file says which way it turned*: fill in the `turned` column.
  - *says it was not visibly turned*: the numbers say it should be turned a degree or more. Look again
    against the tape strip; if it really is straight, the positions are off (slip).
  - *more than 2 %*: check every number is in inches, not centimetres.
  - *position and heading disagree*: re-check the heading's sign (clockwise +) and the corners' order
    (left corner first). If the position comes out **smaller** than the heading, the robot may be
    spinning about a point behind its centre (see below). Once you trust the heading, run again with
    `--use-heading`.
  - *the heading crept*: restart the robot and keep it completely still while it calibrates.
- **Spinning about a point behind the centre.** `robot-config.cpp`'s `[ODOMCAL]` note suggests this
  robot spins about a point 5.8 in behind its centre. If so, the position alone makes the scale error
  look about a quarter smaller, and nothing in the position shows it. The heading does not care. So
  for this robot, measure the heading too: use the corners form (with `turned`), or the simple form
  with `heading_deg`.
- **Left over:** what the scale does not explain. A few tenths of an inch is normal.
- **Slip:** scatter between runs. Over 1 in RMS, the numbers are not worth using: clean the wheels or
  drive slower.

## 6. What the simulator says to expect

All from `tools/sim` (the real LemLib code against a simulated robot with this robot's wheel
positions), side 48 in, speed 70. Five runs each way per session. Slip is "0.45 %" unless it says
otherwise (the simulator's usual amount). Distances read to 1/8 in, the heading to 0.5 deg.

| | simple (centre + turned) | simple + heading | corners + turned |
|---|---|---|---|
| Error in the scale found, worst over every session with 0.45 % slip, % | 0.18 | 0.13 | 0.08 |
| True value inside the ± (inside twice it), sessions with slip | 66 % (91 %) | 70 % (95 %) | 71 % (88 %) |
| Normal sessions given numbers (the rest: "look again") | 96 % | 100 % | 97 % |
| 20 badly calibrated robots, square error per run, median, before -> after one round, in | 1.38 -> 0.38 | 1.38 -> 0.32 | 1.38 -> 0.35 |
| The same robots on the 60 s skills route, error at the end, in | 2.07 -> 1.03 | 2.07 -> 1.07 | 2.07 -> 1.04 |
| Robot spinning 5.8 in behind its centre, sensor +1.2 %: scale found | +0.89 / +0.86 %, no warning | no numbers (disagree) | no numbers (disagree) |
| Student mistakes (sign flipped, columns swapped, cm, corners swapped, heading sign) | no numbers in 14 of 14 | | |

Other results: a drift of 0.05 deg/s reads as +0.19 % clockwise and -0.19 % counter-clockwise, and
the tool gives no numbers for it. A round 1 file read against a log with round 2 added stops and asks
for `--session` (2 of 2). The square cannot fix the route's other half (wheel size, offsets, slip):
with a perfect scale the route still ends about 1 in out. In the robot's own routine (a host build of
`src/umbmark.cpp` on the simulator), a stick pushed in the middle of a leg had the drive at rest
0.41 s later, 4.45 in further on; with the inertial sensor dead part way, it cancelled the move within
20 ms and braked.

What the simulator leaves out: inertial sensor noise (so its headings agree run to run far better
than the real robot's will), slip that grows with speed, sideways skid, and a person misreading the
tape. Expect more scatter on the real robot. Raising both speeds to 127 in `src/umbmark.cpp` measures
the slip at match speed; the simulator cannot show that.

## 7. First time on the real robot

1. Boot with the robot still. Check the controller does not say `IMU FAILED`.
2. **Wheels off the ground first** (robot on a stand), competition cable unplugged, lift down. Pick
   UMB CW, hold A, press B. The drive wheels should spin forward. Push a stick: they should stop at
   once. Start it again and leave it: with the tracking wheels still, it believes it has not moved,
   so after 5 s it should stop by itself and show `timed out: redo`.
3. Clear the floor (section 2). Nobody stands inside the square or on the turning side. The person
   with the controller keeps a thumb near a stick for every run.
4. First real run: watch the whole square. Check the controller numbers match the `[UMB]` line in the
   log.
5. If you can, settle the spin point question before calibrating: on the Developer tab, turn the
   robot one full turn by hand about its centre; X and Y should stay within about half an inch.
