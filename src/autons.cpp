#include <cmath>
#include <functional>
#include "main.h"
#include "autons.hpp"
#include "macros.hpp"
#include "robot-config.hpp"
#include "subsystems.hpp"

// Autonomous routines. Each one is a plain blocking function; runAuton() (src/main.cpp) picks one by the
// selector's number. Set the alliance on the selector's Qual tab before the match: routines read it

// --- james_67 numbers -------------------------------------------------------------------------------
// Inches and degrees; the robot starts at 0, 0 facing 0 deg (= +y), clockwise positive
namespace {
// at the start: toggleB turns this far, motor degrees
constexpr double START_TOGGLE_DEG = 360;
// 67james wiggles first: forward, back, forward again at WIGGLE_POWER, WIGGLE_MS each, the last forward
// WIGGLE_LAST_MS so it comes back in (each part first has to stop the last one). Straight motor power, not a
// LemLib drive: a 5 in moveToPoint only got about 1.2 in in 0.8 s and the drive back then counted as already there
// (2026-11-26). Driving each part by distance (5 in, back 3, forward to 5) was tried and taken out
constexpr int WIGGLE_POWER = 100; // of 127
constexpr int WIGGLE_MS = 200;
constexpr int WIGGLE_LAST_MS = 400; // was 200, then 300
// 67james after the wiggle: straight back this far, then to its first goal lifting to J67_GOAL1_LIFT_DEG on the
// way, and score there
constexpr float J67_BACK_IN = 10;
constexpr int J67_BACK_TIMEOUT_MS = 1000;
constexpr float J67_GOAL1_X = 14.5; // 2 in forward of 12.5 (facing 90: forward is +x)
constexpr float J67_GOAL1_Y = -14.2;
constexpr float J67_GOAL1_THETA = 90;
constexpr double J67_GOAL1_LIFT_DEG = 300;
constexpr int TO_J67_GOAL1_TIMEOUT_MS = 2000;
// after the score: straight back this far (the back-up after the wiggle stays J67_BACK_IN)
constexpr float J67_SCORE_BACK_IN = 9; // was 10 (J67_BACK_IN), 11, then 8
// then turn on the spot to this heading, and L1's grab at the first pin (closes once there)
constexpr float J67_TURN1_THETA = 45; // was 45, 40, 37.5, 35, 40, then 35
constexpr int J67_TURN1_TIMEOUT_MS = 800;
constexpr float J67_PIN1_X = 17; // was 13.8: the odometry only got to about 10.5 there
constexpr float J67_PIN1_Y = 0;
constexpr float J67_PIN1_THETA = 13;
// between the turn and the pin drive: straight forward this far at J67_FORWARD_RPM on the motors' own speed control
// (move_velocity, percent of the blue motors' 600 rpm), both sides the same, until odometry says it got there.
// Tried before: a 5 in LemLib drive (about 1.2 in in 0.8 s); 50% straight power then a step down to a slow speed
// (twitched at the switch); a ramp up, easing from 50% down to 20% and holding the heading with the inertial
// sensor (too fast); straight power under about 28 of 127 stalled
constexpr float J67_FORWARD_IN = 16; // was 5, 10, 8, 12, 16, then 18
constexpr int J67_FORWARD_RPM = 600 * 20 / 100; // 20%, about 13 in/s
// after the drive (L1's grab running): grab, then turn to J67_TURN3_THETA
// the nudge after the drive, before the grab: only the right side drives forward (the left holds), at
// J67_NUDGE_POWER for J67_NUDGE_MS, which swings the robot to the left. (Stopping once the heading had dropped
// 5 deg was tried first: 40 of 127 turned it 1 deg in 0.5 s, 90 swung 5 deg in 0.22 s)
constexpr int J67_NUDGE_POWER = 64; // of 127: 50%
constexpr int J67_NUDGE_MS = 1000;
constexpr float J67_TURN3_THETA = 40; // was 45
constexpr int J67_TURN3_TIMEOUT_MS = 1500; // was 800 for the turn on the spot
constexpr int J67_TURN3_POWER = 64; // of 127: 50%, the right side only (backward)
constexpr int J67_FORWARD_MAX_MS = 4500; // 9 in at 3.2 in/s is about 2.8 s; was 3500; // 2500 ran out at 28 power (17.4 of 18 in); // was 2000 before the longer slow part; 600 (for 5 in), 1000 (for 10), 1200 (for 12), 1500 (16 at 80)

// first goal: drive here, lifting on the way, and score
constexpr float GOAL_X = -13.27;
constexpr float GOAL_Y = 20.6;
constexpr float GOAL_THETA = 0;
constexpr int TO_GOAL_TIMEOUT_MS = 2000; // was 3000
// lift height on the way to the goal, motor degrees (tried: level 1, 440; half of level 1, 220)
constexpr double TO_GOAL_LIFT_DEG = 300;
constexpr int LIFT_DELAY_MS = 500; // the lift starts this long into the drive (was 100)
// (a min speed of 50 was tried and taken out: it also makes LemLib end the move still moving)
// (a horizontalDrift of 8 on this drive, then 6 on every pose drive, to go faster round curves: both
// tried and taken out)
// (a 5 in drive forward between the goal and scoring, 200 ms timeout, was tried and taken out)

// after each score: straight back this far, opposite to the way the robot faces
constexpr float BACK_IN = 10;
constexpr int BACK_TIMEOUT_MS = 1000;
// the drive back ends (HOLD stops it) as soon as the robot is this close to its point
// (chaining it into the pin drive, still moving, with a min speed of 50 was tried twice and taken out)
constexpr float BACK_EXIT_WITHIN_IN = 2;

// first pin: L1's grab (claw open + motors 14/16 in) while driving here, then close on it
// 1 in to the robot's right and 2 in back (facing -45 deg) of -36, 21 (tried: -36, 20; -34, 21; -36, 21;
// -34.59, 22.41; -35.29, 21.71)
constexpr float PIN_X = -33.88;
constexpr float PIN_Y = 20.30;
constexpr float PIN_THETA = -45;

// second goal: lift to level 1 alliance height while driving here, and score
constexpr float GOAL2_X = -7.3; // 2 in right (+x) of -9.3 (was -13.3, then 4 in forward to -9.3)
constexpr float GOAL2_Y = 22.5; // 1 in to the robot's right (facing 90 deg: left is +y) of 23.5 (tried 23.5, 21.5)
constexpr float GOAL2_THETA = 90;
constexpr int TO_GOAL2_TIMEOUT_MS = 2000; // was 3000
constexpr double GOAL2_LIFT_DEG = subsystems::CASCADE_ALLIANCE[0]; // first pin's score: level 1 alliance, 500 (400 tried; 440 was level 1)

// second pin: grab, the same way as the first
// 6 in back (facing 47.5 deg) of -9.71, 47.13 (tried: -10, -12, -14 at 45.9; -12.53, 47.25; -11.06, 48.6; -9.71, 47.13;
// -13.40, 43.75; -17.09, 40.37; -15.62, 41.72)
constexpr float PIN2_X = -14.15;
constexpr float PIN2_Y = 43.07;
constexpr float PIN2_THETA = 47.5;

// third goal: lift to level 2 alliance height while driving here, and score
constexpr float GOAL3_X = -10; // 5 in left of -15 (robot faces 180 here: its right is -x). Tried -10, -12, -15, -10, -15, -5.5, -0.5, -4.5, -7.5, -15
constexpr float GOAL3_Y = 26; // (facing 180: forward is -y). Tried 24, 22, 20, 28.9, 23.9, 20.9, 17.9
constexpr float GOAL3_THETA = 180;
constexpr int TO_GOAL3_TIMEOUT_MS = 2000; // was 3000
constexpr double GOAL3_LIFT_DEG = subsystems::CASCADE_ALLIANCE[1]; // second pin's score: level 2 alliance, 700 (640 was level 2)

// after the third goal: B's reset (lift to the bottom and re-zeroed, roller homed) while driving straight to the
// third pin. (Backing up to -7, 46 facing 180 first was tried and taken out.) Before that drive, straight back
// this far (+y: the robot faces 180 there)
constexpr float BACK3_IN = 25; // was 20; 30 tried and taken back
constexpr int BACK3_TIMEOUT_MS = 1800; // was 1500 (for 20 in), 2100 (for 30 in)
// B's reset starts once the robot has backed this far from the goal. B finds the bottom by driving the lift down
// until it stops; started at the goal, the lift is still sitting on it, so B "found the bottom" there, zeroed the
// lift at goal height and never went down (2026-11-20)
constexpr float RESET_AFTER_BACK_IN = 6;
// then turn on the spot to this heading before driving to the third pin
constexpr float PIN3_TURN_THETA = 90;
constexpr int PIN3_TURN_TIMEOUT_MS = 1000;

// third pin: grab, the same way as the others
constexpr float PIN3_X = 12;
constexpr float PIN3_Y = 55; // 2 in to the robot's right (facing 90 deg: right is -y) of 57
constexpr float PIN3_THETA = GOAL2_THETA; // the same heading as the goal scored after the first pin (90)
// the fourth pin (the second third-pin round): grabbed 2 in further forward (facing 90: forward is +x), same heading
constexpr float PIN4_X = PIN3_X + 2;
constexpr float PIN4_Y = PIN3_Y;
// and scored 1 in to the robot's left and 2 in forward of the third goal's point (facing 180: left is +x,
// forward is -y), same heading (was 2 in left)
constexpr float GOAL4_X = GOAL3_X + 1;
constexpr float GOAL4_Y = GOAL3_Y - 2;
// the fifth pin (L1's grab): 2 in to the robot's right of the fourth pin's point (facing 90: right
// is -y), same heading (2 in left was tried)
constexpr float PIN5_X = PIN4_X;
constexpr float PIN5_Y = PIN4_Y - 1; // 1 in to the robot's right of the fourth pin (2 in right tried, 2 in left)
// then straight back this far, and to the fifth goal lifting to GOAL5_LIFT_DEG on the way, and score there
constexpr float PIN5_BACK_IN = 10;
constexpr float GOAL5_X = -31; // 2 in forward of -29 (facing 270: forward is -x)
constexpr float GOAL5_Y = 42; // 3 in to the robot's left of 45 (facing 270: left is -y). Tried 45, 43
constexpr float GOAL5_THETA = 270;
constexpr double GOAL5_LIFT_DEG = 400; // was 300
constexpr int TO_GOAL5_TIMEOUT_MS = 2500; // about 43 in with a 180 deg turn
// after the fifth pin's score: back to the third-pin spot for a sixth pin. The robot faces 270 at the goal and the
// pin is behind it (+x), so: straight back PIN6_BACK_IN (B's reset once clear of the goal, as after the third goal),
// turn round on the spot to face 90, then the usual L1 grab (early catch) with its pose drive to the pin
constexpr float PIN6_X = PIN5_X; // the fifth pin's point, same heading (90)
constexpr float PIN6_Y = PIN5_Y;
constexpr float PIN6_BACK_IN = 10;
constexpr int PIN6_TURN_TIMEOUT_MS = 1000; // about 180 deg on the spot, to face 90
constexpr int TO_PIN6_TIMEOUT_MS = 3000; // about 40 in
// then straight back this far (-x: it faces 90 there), and back to the third goal's point (GOAL3_*, facing 180),
// lifting to level 3 alliance height (900 deg) on the way, and score there
constexpr float BACK4_IN = 11;
constexpr int BACK4_TIMEOUT_MS = 1000;
constexpr int RETURN_TO_GOAL3_TIMEOUT_MS = 3500; // about 30 in (was 2500: ran out, 2026-11-25)
// before that drive, turn on the spot to face the goal's point, so the drive is a straight line. One curved
// moveToPose from facing 90 to the goal facing 180 arced wide to the right (LemLib slows tight curves down) and ran
// out of time 18 and 24 in short (2026-11-25, pins 3 and 4)
constexpr int RETURN_TURN_TIMEOUT_MS = 800;
// then the whole third-pin cycle again (back up + B, turn, grab at the third pin, back up, to the third goal's
// point), this time lifting to level 4 alliance height (1020 deg). Driver control's anti-tip (controls.cpp) caps
// the drive at 80% with the lift at 1020, so this drive gets the same cap
constexpr float LEVEL4_MAX_SPEED = 127 * 0.8;
// the lift's speed (percent) every time it lifts up to a goal (was 100; 80 on pins 3 and 4 first, then all)
constexpr int LIFT_PERCENT = 80;

// after the fourth pin's score: the same back-up + B and turn to 90, then to the fifth pin's point (PIN5_*) with
// L1's grab (grabPin), motors 14/16 intaking through the PIN5_BACK_IN back-up after it. (L2's grab, claw closed
// and intaking all the way, was tried first and swapped for L1's)

// every drive to a pin
constexpr int TO_PIN_TIMEOUT_MS = 2000; // was 3000
// within this far of the pin point, finish the drive slower
constexpr float PIN_SLOW_WITHIN_IN = 3; // was 5 (pose drives end at 2 in, so about the last inch is slow)
// 50% of 127. NOTE: a moveToPose that starts inside 7.5 in of its target replaces maxSpeed with 60 (LemLib,
// moveToPose.cpp), so the robot actually gets 60 (47%) whatever this says
constexpr float PIN_SLOW_SPEED = 127 * 0.5;
constexpr int PIN_SLOW_TIMEOUT_MS = 1500;
// at the pin: wait up to this long for the claw sensor to see it (within CLAW_CATCH_MM), then close anyway
constexpr int PIN_CATCH_WAIT_MS = 200; // was 500
// pins 3, 4 and 5: from this long after the claw opens, close it as soon as the sensor sees
// something (within CLAW_CATCH_MM), even mid-drive, and end the drive there. The other pins wait for the point
constexpr std::uint32_t PIN3_CATCH_AFTER_MS = 250;
// pins 1 and 2: the same early catch (claw closes the moment the sensor sees the pin), but the drive carries on to
// the point
constexpr std::uint32_t PIN12_CATCH_AFTER_MS = 250;

// When a pose drive counts as arrived (waitPose). Same ranges as the drive's own exit settings in
// robot-config.cpp, but EITHER one ends the drive, as in moveToPoint: LemLib's moveToPose wants BOTH, so a robot
// stopped 0.6 in off (against a goal, say) sat there until the timeout
constexpr float ARRIVED_SMALL_IN = 0.5;
constexpr int ARRIVED_SMALL_MS = 200;
constexpr float ARRIVED_LARGE_IN = 3;
constexpr int ARRIVED_LARGE_MS = 500;
constexpr float ARRIVED_HEADING_DEG = 3; // and the heading within this of the target

// Every pose drive ends as soon as the robot is this close to its point (was 1, on the second pin only).
// Done in waitPose, not with LemLib's earlyExitRange: that only works together with a minSpeed, which made the
// robot miss the point. The heading may still be turning when it ends
constexpr float POSE_EXIT_WITHIN_IN = 2;

// Start R1's scoring motion (lift down until it lands, open the claw, outtake) once the robot is this close to a
// goal, while still driving, instead of after the drive. The lift takes ~0.3-0.4 s to come down from scoring
// height, so the robot is normally there before it lands. Lower it if the lift catches the edge of a goal; 0 = start
// it after the drive, as before
constexpr float SCORE_EARLY_IN = 4;

// --- steps used more than once ------------------------------------------------------------------------

// Wait for the pose drive to (x, y, theta) that is running now, and end it as soon as the robot has arrived
// (see ARRIVED_*; theta NAN = any heading, for a moveToPoint), or is within `exitWithin` of the point (0 = no early exit). `eachTick` runs every 10 ms
// meanwhile. Prints how long the drive took and why it ended
void waitPose(float x, float y, float theta, float exitWithin = POSE_EXIT_WITHIN_IN, const std::function<void()>& eachTick = nullptr) {
    const std::uint32_t start = pros::millis();
    const lemlib::Pose target(x, y);
    const char* why = "LemLib ended it (its own exit or the timeout)";
    int smallMs = 0, largeMs = 0;
    while (chassis.isInMotion()) {
        const lemlib::Pose pose = chassis.getPose();
        const float dist = pose.distance(target);
        const bool headingOk =
            std::isnan(theta) || std::fabs(std::remainder(pose.theta - theta, 360.0)) <= ARRIVED_HEADING_DEG;
        smallMs = dist <= ARRIVED_SMALL_IN && headingOk ? smallMs + 10 : 0;
        largeMs = dist <= ARRIVED_LARGE_IN && headingOk ? largeMs + 10 : 0;
        if (exitWithin > 0 && dist <= exitWithin) why = "within the early exit range";
        else if (smallMs >= ARRIVED_SMALL_MS) why = "arrived (small range)";
        else if (largeMs >= ARRIVED_LARGE_MS) why = "arrived (large range)";
        else {
            if (eachTick) eachTick();
            pros::delay(10);
            continue;
        }
        chassis.cancelMotion(); // HOLD stops it here
        break;
    }
    chassis.waitUntilDone();
    const lemlib::Pose end = chassis.getPose();
    std::printf("[auton] to (%.1f, %.1f, %.0f): %lu ms, %s; ended %.2f in off, heading %.1f deg off\n", x, y, theta,
                static_cast<unsigned long>(pros::millis() - start), why, end.distance(target),
                std::remainder(end.theta - theta, 360.0));
}

// R1's motion (lift down onto the goal, open the claw, outtake 0.3 s), the same code as the driver's R1. Waits
// only until the claw opens: the outtake carries on in the macro task while the robot drives away. (If the
// lift never lands, the macro ends without opening the claw, and this waits for that.)
// waitOuttake: wait for the 0.3 s outtake too, as before
void finishScore(bool waitOuttake = false) {
    while (macros::isBusy() && (waitOuttake || !subsystems::clawOpen())) pros::delay(10);
}

// Start R1's motion in the macro task (finishScore() waits for it). Motors 14/16 stop here: the motion owns them
// from now on (it outtakes with them once the claw opens), so nothing else may drive them until it is over
void startScore() {
    xMotors.brake();
    macros::setPlaceDownPercent(100); // full speed down in the auton (driver control's R1 uses 80)
    macros::request(macros::Macro::PlaceRelease, true);
}

// Wait for the pose drive to the goal (x, y, theta) that is running now, starting the scoring motion once within
// SCORE_EARLY_IN of the point (or when the drive ends, if it never got that close). `eachTick` runs every 10 ms
// until scoring starts. finishScore() after it
void waitPoseThenScore(float x, float y, float theta, const std::function<void()>& eachTick = nullptr) {
    const lemlib::Pose goal(x, y);
    bool started = false;
    waitPose(x, y, theta, POSE_EXIT_WITHIN_IN, [&] {
        // the scoring motion opened the claw: the pin is on, end the drive. Against a goal the robot can stop short
        // of its point (3.8 in at the fifth goal) and used to push there until the timeout, a second each time
        // (2026-11-25)
        if (started && subsystems::clawOpen()) {
            chassis.cancelMotion();
            return;
        }
        if (started) return;
        if (chassis.getPose().distance(goal) <= SCORE_EARLY_IN) {
            startScore();
            started = true;
            std::printf("[auton] scoring started %.1f in from the goal\n", chassis.getPose().distance(goal));
            return;
        }
        if (eachTick) eachTick();
    });
    if (!started) startScore();
}

// straight back `inches`, opposite to the way the robot faces now. Waits. `once` (if given) runs once, when the
// robot has backed `onceAfterIn` (or at the end, if the drive stopped short of that)
void driveBack(float inches, int timeoutMs, const std::function<void()>& once = nullptr, float onceAfterIn = 0) {
    const lemlib::Pose now = chassis.getPose();
    const float heading = lemlib::degToRad(now.theta);
    const lemlib::Pose target(now.x - inches * std::sin(heading), now.y - inches * std::cos(heading));
    chassis.moveToPoint(target.x, target.y, timeoutMs, {.forwards = false});
    bool done = once == nullptr;
    while (chassis.isInMotion() && chassis.getPose().distance(target) > BACK_EXIT_WITHIN_IN) {
        if (!done && chassis.getPose().distance(now) >= onceAfterIn) {
            once();
            done = true;
        }
        pros::delay(10);
    }
    if (!done) once();
    chassis.cancelMotion(); // (nothing to cancel if it already ended) HOLD stops it here
    chassis.waitUntilDone();
}

// L1's grab at a pin: claw open and motors 14/16 in while driving to (x, y, theta), full speed until within
// PIN_SLOW_WITHIN_IN, then slower; at the point close the claw as soon as the sensor sees the pin (or after
// PIN_CATCH_WAIT_MS anyway). Motors 14/16 are left intaking: L1 keeps them on CLAW_INTAKE_AFTER_MS more.
// exitWithin > 0: end the drive as soon as the robot is that close to the point (HOLD then stops it)
// @return millis() when the claw closed, for stopping them
// theta NAN: no final heading, the drives are moveToPoint instead of moveToPose
// catchAfterMs > 0: from that long after the claw opens, close as soon as the sensor sees something; with
// stopOnCatch, end the drive there too (pins 3-5), otherwise carry on to the point (pins 1 and 2)
std::uint32_t grabPin(float x, float y, float theta, float exitWithin = POSE_EXIT_WITHIN_IN,
                      std::uint32_t catchAfterMs = 0, bool stopOnCatch = true, int timeoutMs = TO_PIN_TIMEOUT_MS) {
    // finishScore() returns before its outtake is over, and the outtake ends by stopping motors 14/16: let it finish
    // first, or it would stop the intake started below (normally already over by the end of the drive back).
    // Other macros (B's reset) don't touch motors 14/16 and the claw, so the grab doesn't wait for them
    while (macros::isRunning(macros::Macro::PlaceRelease)) pros::delay(10);
    subsystems::setClaw(true);
    const std::uint32_t openedAt = pros::millis();
    xMotors.move_velocity(subsystems::rpm(100));
    bool caught = false;
    std::uint32_t closedAt = 0;
    auto sees = [] {
        const std::int32_t mm = clawSensor.get_distance();
        return mm > 0 && mm <= subsystems::CLAW_CATCH_MM;
    };
    // the early catch: closes the claw the moment it sees the pin (and ends the drive, with stopOnCatch)
    auto watch = [&] {
        if (catchAfterMs == 0 || caught || pros::millis() - openedAt < catchAfterMs || !sees()) return;
        caught = true;
        subsystems::setClaw(false);
        closedAt = pros::millis();
        if (stopOnCatch) chassis.cancelMotion();
    };
    auto stopped = [&] { return caught && stopOnCatch; };

    const bool anyHeading = std::isnan(theta);
    auto drive = [&](int timeoutMs, float maxSpeed) {
        if (anyHeading) chassis.moveToPoint(x, y, timeoutMs, {.forwards = true, .maxSpeed = maxSpeed});
        else chassis.moveToPose(x, y, theta, timeoutMs, {.forwards = true, .maxSpeed = maxSpeed});
    };
    drive(timeoutMs, 127);
    // LemLib can't change a move's speed part way, so the first move is cancelled and a slow one started
    const lemlib::Pose pin(x, y);
    while (chassis.isInMotion() && chassis.getPose().distance(pin) > PIN_SLOW_WITHIN_IN) {
        watch();
        pros::delay(10);
    }
    if (chassis.isInMotion() && !stopped()) {
        // a cancelled move sets the drive to 0 for ~10 ms before the next starts: coast through that
        // instead of HOLD braking hard at speed
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
        chassis.cancelMotion();
        chassis.waitUntilDone(); // the first move has fully ended, so the next isn't dropped
        drive(PIN_SLOW_TIMEOUT_MS, PIN_SLOW_SPEED);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    }
    if (!stopped()) waitPose(x, y, theta, exitWithin, watch);
    chassis.waitUntilDone();

    int waited = 0;
    for (; !caught && waited < PIN_CATCH_WAIT_MS; waited += 10) {
        if (sees() && pros::millis() - openedAt >= catchAfterMs) break;
        pros::delay(10);
    }
    if (!caught) {
        subsystems::setClaw(false);
        closedAt = pros::millis();
    }
    std::printf("[auton] pin (%.1f, %.1f): claw closed (%s, sensor %d mm, %.1f in from the point)\n", x, y,
                caught ? "caught on the way" : waited < PIN_CATCH_WAIT_MS ? "caught" : "timed out",
                static_cast<int>(clawSensor.get_distance()), chassis.getPose().distance(pin));
    return closedAt;
}

// after a grab: lift to `liftDeg` while driving to the goal (x, y, theta), scoring on the way in (finishScore()
// after it). Motors 14/16 keep
// intaking CLAW_INTAKE_AFTER_MS after the grab (`closedAt`, from grabPin), as L1 does, during the drive
void liftDrive(double liftDeg, float x, float y, float theta, int timeoutMs, std::uint32_t closedAt,
               float maxSpeed = 127, int liftPercent = LIFT_PERCENT) {
    // B's reset must be over before lifting, or the two fight over the lift (normally long over by now)
    while (macros::isRunning(macros::Macro::ResetAll)) pros::delay(10);
    cascade.move_absolute(liftDeg, subsystems::rpm(liftPercent));
    chassis.moveToPose(x, y, theta, timeoutMs, {.forwards = true, .maxSpeed = maxSpeed});
    waitPoseThenScore(x, y, theta, [closedAt] {
        if (pros::millis() - closedAt >= subsystems::CLAW_INTAKE_AFTER_MS) xMotors.brake();
    });
    // (if scoring started before the 2 s were up, startScore() stopped the intake: the outtake comes next anyway)
}

// From just after scoring at the third goal: back away (B's reset once clear of the goal), then turn to face the
// third pin. The scoring motion is completely over first (its outtake too): starting a macro cancels the one
// running. B carries on in the macro task during the next drive
void backUpAndTurn() {
    while (macros::isBusy()) pros::delay(10);
    driveBack(BACK3_IN, BACK3_TIMEOUT_MS, [] { macros::request(macros::Macro::ResetAll, true); }, RESET_AFTER_BACK_IN);
    chassis.turnToHeading(PIN3_TURN_THETA, PIN3_TURN_TIMEOUT_MS);
    chassis.waitUntilDone();
}

// From just after scoring at the third goal: back away (B's reset once clear of the goal), turn, grab the third
// pin, back away from it, then lift to `liftDeg` while driving back to the third goal's point, and score there.
// Waits until the claw opens on the goal
void thirdPinCycle(double liftDeg, float pinX, float pinY, float goalX, float goalY, float maxSpeed = 127) {
    // B's reset once clear of the goal (RESET_AFTER_BACK_IN). The scoring motion is completely over first (its
    // outtake too): starting a macro cancels the one running. B carries on in the macro task during the drive to
    // the pin
    backUpAndTurn();

    // the pin. The reset may still be running: it uses the lift and roller, the grab doesn't
    const std::uint32_t closedAt = grabPin(pinX, pinY, PIN3_THETA, POSE_EXIT_WITHIN_IN, PIN3_CATCH_AFTER_MS);
    // back away from the pin (motors 14/16 keep intaking their 2 s after the grab, as L1; liftDrive stops them)
    driveBack(BACK4_IN, BACK4_TIMEOUT_MS);

    // back to the third goal's point, lifting on the way, and score there (scoring starts SCORE_EARLY_IN out, as
    // at the other goals). B's reset is long over by now; wait for it if not, so it doesn't fight the lift
    while (macros::isRunning(macros::Macro::ResetAll)) pros::delay(10);
    chassis.turnToPoint(goalX, goalY, RETURN_TURN_TIMEOUT_MS);
    chassis.waitUntilDone();
    liftDrive(liftDeg, goalX, goalY, GOAL3_THETA, RETURN_TO_GOAL3_TIMEOUT_MS, closedAt, maxSpeed);
    finishScore();
}
} // namespace

// Qual routine 2
void james_67() {
    chassis.setPose(0, 0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD); // driver control sets it back to COAST when it starts

    // toggleB one full turn. The colour toggle is paused meanwhile so it doesn't fight it. (Spinning until the
    // optical sensor saw our alliance colour, subsystems::spinToAllianceColour(), was tried and swapped for this;
    // turning it during the first drive instead of before was tried and taken out)
    subsystems::pauseColourToggle(true);
    subsystems::spinTo(toggleB, toggleB.get_position() + START_TOGGLE_DEG, subsystems::rpm(100));
    subsystems::pauseColourToggle(false);

    // first goal. moveToPose doesn't wait, so the robot is already moving during the lift delay
    chassis.moveToPose(GOAL_X, GOAL_Y, GOAL_THETA, TO_GOAL_TIMEOUT_MS, {.forwards = true});
    pros::delay(LIFT_DELAY_MS);
    cascade.move_absolute(TO_GOAL_LIFT_DEG, subsystems::rpm(LIFT_PERCENT));
    waitPoseThenScore(GOAL_X, GOAL_Y, GOAL_THETA);
    finishScore(true); // the first goal keeps the full 0.3 s outtake before driving away
    // B's reset on the way back, once clear of the goal (RESET_AFTER_BACK_IN), before the first pin's grab. It
    // carries on during the grab (it doesn't use the claw); the lift to the second goal waits for it (liftDrive)
    driveBack(BACK_IN, BACK_TIMEOUT_MS, [] { macros::request(macros::Macro::ResetAll, true); }, RESET_AFTER_BACK_IN);

    // first pin
    const std::uint32_t closedAt = grabPin(PIN_X, PIN_Y, PIN_THETA, POSE_EXIT_WITHIN_IN, PIN12_CATCH_AFTER_MS, false);

    // second goal: GOAL2_LIFT_DEG (500 deg; was level 1 alliance height, 440)
    liftDrive(GOAL2_LIFT_DEG, GOAL2_X, GOAL2_Y, GOAL2_THETA, TO_GOAL2_TIMEOUT_MS, closedAt);
    finishScore();
    driveBack(BACK_IN, BACK_TIMEOUT_MS);

    // second pin
    const std::uint32_t closed2At = grabPin(PIN2_X, PIN2_Y, PIN2_THETA, POSE_EXIT_WITHIN_IN, PIN12_CATCH_AFTER_MS, false);

    // turn on the spot to face the goal first, so the drive there is a straight line: the one curved drive from
    // facing 47.5 to facing 180 was too tight a turn and ended in the wrong place (2026-11-25)
    chassis.turnToPoint(GOAL3_X, GOAL3_Y, RETURN_TURN_TIMEOUT_MS);
    chassis.waitUntilDone();

    // third goal: GOAL3_LIFT_DEG (700 deg; was level 2 alliance height, 640)
    liftDrive(GOAL3_LIFT_DEG, GOAL3_X, GOAL3_Y, GOAL3_THETA, TO_GOAL3_TIMEOUT_MS, closed2At);
    finishScore();

    // third pin, scored at the third goal's point at level 3 alliance height (900 deg)
    thirdPinCycle(subsystems::CASCADE_ALLIANCE[2], PIN3_X, PIN3_Y, GOAL3_X, GOAL3_Y);

    // the same again, at level 4 alliance height (1020 deg)
    thirdPinCycle(subsystems::CASCADE_ALLIANCE[3], PIN4_X, PIN4_Y, GOAL4_X, GOAL4_Y, LEVEL4_MAX_SPEED);

    // back up + B and turn again, then the fifth pin with L1's grab
    backUpAndTurn();
    const std::uint32_t closed5At = grabPin(PIN5_X, PIN5_Y, PIN3_THETA, POSE_EXIT_WITHIN_IN, PIN3_CATCH_AFTER_MS);

    // back off still intaking, then to the fifth goal lifting to 300 on the way, and score there. The claw stays
    // closed from the grab until the scoring motion opens it
    driveBack(PIN5_BACK_IN, BACK_TIMEOUT_MS);
    xMotors.brake();
    // turn on the spot to face the goal first, so the drive there is a straight line (as for pins 3 and 4)
    chassis.turnToPoint(GOAL5_X, GOAL5_Y, RETURN_TURN_TIMEOUT_MS);
    chassis.waitUntilDone();
    liftDrive(GOAL5_LIFT_DEG, GOAL5_X, GOAL5_Y, GOAL5_THETA, TO_GOAL5_TIMEOUT_MS, closed5At);
    finishScore();

    // sixth pin, at the third-pin spot: back off the goal (B once clear), face the pin, grab it
    while (macros::isBusy()) pros::delay(10); // the score's outtake is over first: B would cancel it
    driveBack(PIN6_BACK_IN, BACK_TIMEOUT_MS, [] { macros::request(macros::Macro::ResetAll, true); },
              RESET_AFTER_BACK_IN);
    // turn round on the spot to face 90, then the grab's pose drive to the pin. (Tried first, all slower or worse:
    // a point 12 in short of the pin, in line with it, reached by turn + straight drive + turn to 90; by one pose
    // drive from facing 270, which drove into the wall turning round; by a turn + pose drive, 2026-11-25)
    chassis.turnToHeading(PIN3_THETA, PIN6_TURN_TIMEOUT_MS);
    chassis.waitUntilDone();
    const std::uint32_t closed6At = grabPin(PIN6_X, PIN6_Y, PIN3_THETA, POSE_EXIT_WITHIN_IN, PIN3_CATCH_AFTER_MS, true,
                                            TO_PIN6_TIMEOUT_MS);

    // and score it the way pin 5 was: back off still intaking, face the fifth goal, straight there lifting to the
    // first pin's score height (GOAL2_LIFT_DEG), score
    driveBack(PIN5_BACK_IN, BACK_TIMEOUT_MS);
    xMotors.brake();
    chassis.turnToPoint(GOAL5_X, GOAL5_Y, RETURN_TURN_TIMEOUT_MS);
    chassis.waitUntilDone();
    liftDrive(GOAL2_LIFT_DEG, GOAL5_X, GOAL5_Y, GOAL5_THETA, TO_GOAL5_TIMEOUT_MS, closed6At);
    finishScore();

    // rest of the routine goes here
}

// Qual routine 3, "67james" on the selector (the default)
void james67b() {
    chassis.setPose(0, 0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD); // driver control sets it back to COAST when it starts

    // the wiggle: forward, back, forward again (the last one longer). (toggleA's 720 deg spin after it was tried
    // and taken out)
    const int powers[] = {WIGGLE_POWER, -WIGGLE_POWER, WIGGLE_POWER};
    const int times[] = {WIGGLE_MS, WIGGLE_MS, WIGGLE_LAST_MS};
    for (int i = 0; i < 3; ++i) {
        leftMotors.move(powers[i]);
        rightMotors.move(powers[i]);
        pros::delay(times[i]);
    }
    leftMotors.brake(); // HOLD
    rightMotors.brake();

    // back off, then to the first goal lifting to 300 on the way, and score there
    driveBack(J67_BACK_IN, J67_BACK_TIMEOUT_MS);
    liftDrive(J67_GOAL1_LIFT_DEG, J67_GOAL1_X, J67_GOAL1_Y, J67_GOAL1_THETA, TO_J67_GOAL1_TIMEOUT_MS, pros::millis());
    finishScore(true); // the full 0.3 s outtake before backing off, as james_67's first goal
    driveBack(J67_SCORE_BACK_IN, J67_BACK_TIMEOUT_MS); // straight back off the goal

    // turn to J67_TURN1_THETA (45), then drive forward with L1's grab
    chassis.turnToHeading(J67_TURN1_THETA, J67_TURN1_TIMEOUT_MS);
    chassis.waitUntilDone();
    while (macros::isRunning(macros::Macro::PlaceRelease)) pros::delay(10); // its outtake would stop the intake
    // the grab: claw open, motors 14/16 OUTTAKING (X's way) through the drive and the nudge; the claw closes after the
    // nudge, and from then the motors intake CLAW_INTAKE_AFTER_MS, as L1 after a catch. (Tried: intaking all the
    // way, as L1; L2's grab, claw closed all the way; closing as soon as the sensor saw the pin)
    subsystems::setClaw(true);
    xMotors.move_velocity(-subsystems::rpm(100));
    {
        const lemlib::Pose from = chassis.getPose();
        // (only for the log: how far the heading wanders off what the turn aimed for)
        const float holdHeading = J67_TURN1_THETA;
        float worstOff = 0;
        const std::uint32_t start = pros::millis();
        while (pros::millis() - start < J67_FORWARD_MAX_MS && chassis.getPose().distance(from) < J67_FORWARD_IN) {
            // stop as soon as the claw sensor sees the pin, so the robot doesn't drive into it and knock it over (it
            // used to push on at the slow power: 16.8 of 18 in after the full 3.5 s, 2026-11-26). The claw closes
            // after the drive
            const std::int32_t mm = clawSensor.get_distance();
            if (mm > 0 && mm <= subsystems::CLAW_CATCH_MM) break;
            // both sides at J67_FORWARD_RPM, no heading correction
            const float off = lemlib::angleError(chassis.getPose().theta, holdHeading, false);
            if (std::fabs(off) > std::fabs(worstOff)) worstOff = off;
            leftMotors.move_velocity(J67_FORWARD_RPM);
            rightMotors.move_velocity(J67_FORWARD_RPM);
            pros::delay(10);
        }
        leftMotors.brake(); // HOLD
        rightMotors.brake();
        std::printf("[auton] forward %.1f in: %.1f after %u ms (sensor %d mm); heading %.1f at the start, %.1f at the "
                    "end, worst %.1f off %.0f\n",
                    J67_FORWARD_IN, chassis.getPose().distance(from), static_cast<unsigned>(pros::millis() - start),
                    static_cast<int>(clawSensor.get_distance()), from.theta, chassis.getPose().theta, worstOff,
                    holdHeading);
    }
    // grabPin(J67_PIN1_X, J67_PIN1_Y, J67_PIN1_THETA); // the pose drive to the first pin, off for now

    // grab, then turn to J67_TURN3_THETA (40), still intaking, then stop the intake. (A right-side-only nudge before
    // the grab was tried and taken out.)
    // (Tried and taken out: the early catch during the drive, closing straight after the drive, turning to 35 after
    // the grab, L1's grab at the 0 / 25 deg turn)
    // the right-side nudge
    {
        const float startHeading = chassis.getPose().theta;
        const std::uint32_t start = pros::millis();
        leftMotors.brake(); // HOLD: the left side stays put
        rightMotors.move(J67_NUDGE_POWER);
        pros::delay(J67_NUDGE_MS);
        rightMotors.brake();
        std::printf("[auton] nudge: heading %.1f -> %.1f after %u ms\n", startHeading, chassis.getPose().theta,
                    static_cast<unsigned>(pros::millis() - start));
    }
    // the grab: close, and intake from here
    subsystems::setClaw(false);
    xMotors.move_velocity(subsystems::rpm(100));
    const std::uint32_t closedAt = pros::millis();
    std::printf("[auton] L1 grab after the nudge: claw closed (sensor %d mm)\n",
                static_cast<int>(clawSensor.get_distance()));
    // back to J67_TURN3_THETA with the right side only (the left holds). The nudge left the robot turned left of
    // it (about 20 deg), so the right side drives BACKWARD, which swings the robot clockwise, until the heading
    // reaches the target or J67_TURN3_TIMEOUT_MS is up. (A turn on the spot with both sides was tried first)
    {
        const float startHeading = chassis.getPose().theta;
        const std::uint32_t start = pros::millis();
        leftMotors.brake(); // HOLD
        rightMotors.move(-J67_TURN3_POWER);
        while (pros::millis() - start < J67_TURN3_TIMEOUT_MS &&
               lemlib::angleError(J67_TURN3_THETA, chassis.getPose().theta, false) > 0)
            pros::delay(10);
        rightMotors.brake();
        std::printf("[auton] right side back to %.0f: heading %.1f -> %.1f after %u ms\n", J67_TURN3_THETA,
                    startHeading, chassis.getPose().theta, static_cast<unsigned>(pros::millis() - start));
    }
    if (pros::millis() - closedAt < subsystems::CLAW_INTAKE_AFTER_MS)
        pros::delay(subsystems::CLAW_INTAKE_AFTER_MS - (pros::millis() - closedAt));
    xMotors.brake();

    // rest of the routine goes here
}
