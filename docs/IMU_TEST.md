# Inertial sensor test

Finds out why the heading sometimes climbs by itself (the robot then spins or drives into a wall). It tells
apart a sensor that is fine but was handled while the program started, a bad cable or port, a loose mount, and a
bad sensor. About 2.5 minutes. **Nothing drives.**

**Written 2026-11-24 and only compiled: it has never run on the robot.** The first run is also its own test.

## Before you start

- Robot on the floor, wheels down, nobody leaning on it. No competition switch.
- Upload over the cable (this build re-sends the 3.7 MB library package).
- On the laptop, to keep the results: `pros terminal --output imu_test.log`
- Start the program with the robot still. At start-up the terminal already prints one line, for example
  `[imu] boot check: heading creeps 0.004 deg/s standing still, fine`.

## Run it

1. On the brain: **Tests** tab, tap **IMU TEST**.
2. On the controller: **hold A, press B.**
3. Do what the controller's top line says. Each step waits 4 s first so you can read it.

| Controller says | What you do | How long |
|---|---|---|
| `1/4 HANDS OFF` | Nothing. Do not touch the robot. It calibrates ten times. | about 35 s |
| `2/4 HANDS OFF 60s` | Nothing. Do not touch the robot. | 60 s |
| `3/4 WIGGLE CABLE` | Wiggle the inertial sensor's cable at the sensor end and at the brain end. Do not move the robot. | 20 s |
| `4/4 TAP THE FRAME` | Tap the frame firmly near the sensor, several times. Stop when it says `HANDS OFF`. | 15 s + 3 s |

Pushing any stick stops the test. The controller rumbles once at the start of steps 3 and 4, twice at the end.

## The result

The controller's top line is the verdict, and the terminal says the same in full
(`[IMUTEST] WHAT TO DO: ...`).

| Top line | Meaning | What to do |
|---|---|---|
| `SENSOR OK` | Sensor, cable and mount are fine. The fault was the robot being handled while the program started. | Start the program with hands off. The boot check now catches it and calibrates again. |
| `CABLE OR PORT` | The reading jumped or dropped out when the cable was moved. | New cable. If it still fails, another brain port (change `IMU` in `include/ports.hpp`). |
| `CHECK CABLE/PORT` | The sensor did not calibrate, or dropped out while untouched. | Cable and port first, then the sensor. |
| `REPLACE SENSOR` | It creeps after a clean calibration with a steady cable. | Try one other port. If the same, replace the sensor. |
| `FIX THE MOUNT` | It did not come back after the knocks. | Screw it down rigid and flat, away from motors. Run the test again. |
| `ROBOT MOVED: REDO` | The robot rolled during a step, so that step cannot be judged. | Run it again without moving the robot. |

Lines 2 and 3 of the controller show each step: `cal`, `sit`, `wig`, `knk`, each PASS, FAIL or MOVED.

## What counts as passing

| Step | Passes when |
|---|---|
| Calibrate x10 | every calibration creeps under 0.05 deg/s (a good sensor reads under 0.012; the fault was 3.18) |
| Sit 60 s | the heading wanders under 1 deg, with no jumps or dropouts |
| Wiggle | no dropout, no jump over 0.5 deg, under 1 deg in total |
| Knock | within 1.5 deg of where it was, and not creeping afterwards |

If you change a cable, port or sensor, run the test again: it should come out `SENSOR OK`.

Send `imu_test.log` to whoever is debugging. The lines that matter start with `[imu]` and `[IMUTEST]`.
