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

// second pin: grab, the same way as the first
// 8 in back (facing 47.5 deg) of -9.71, 47.13 (tried: -10, -12, -14 at 45.9; -12.53, 47.25; -11.06, 48.6; -9.71, 47.13;
// -13.40, 43.75; -17.09, 40.37)
constexpr float PIN2_X = -15.62;
constexpr float PIN2_Y = 41.72;
constexpr float PIN2_THETA = 47.5;

// third goal: lift to level 2 alliance height while driving here, and score
constexpr float GOAL3_X = -10; // 5 in left of -15 (robot faces 180 here: its right is -x). Tried -10, -12, -15, -10, -15, -5.5, -0.5, -4.5, -7.5, -15
constexpr float GOAL3_Y = 26; // (facing 180: forward is -y). Tried 24, 22, 20, 28.9, 23.9, 20.9, 17.9
constexpr float GOAL3_THETA = 180;
constexpr int TO_GOAL3_TIMEOUT_MS = 2000; // was 3000

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
constexpr float GOAL5_X = -29;
constexpr float GOAL5_Y = 43; // 2 in to the robot's left of 45 (facing 270: left is -y)
constexpr float GOAL5_THETA = 270;
constexpr double GOAL5_LIFT_DEG = 300;
constexpr int TO_GOAL5_TIMEOUT_MS = 2500; // about 43 in with a 180 deg turn
// then straight back this far (-x: it faces 90 there), and back to the third goal's point (GOAL3_*, facing 180),
// lifting to level 3 alliance height (900 deg) on the way, and score there
constexpr float BACK4_IN = 11;
constexpr int BACK4_TIMEOUT_MS = 1000;
constexpr int RETURN_TO_GOAL3_TIMEOUT_MS = 2500; // about 40 in with a 90 deg turn
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
    macros::request(macros::Macro::PlaceRelease, true);
}

// Wait for the pose drive to the goal (x, y, theta) that is running now, starting the scoring motion once within
// SCORE_EARLY_IN of the point (or when the drive ends, if it never got that close). `eachTick` runs every 10 ms
// until scoring starts. finishScore() after it
void waitPoseThenScore(float x, float y, float theta, const std::function<void()>& eachTick = nullptr) {
    const lemlib::Pose goal(x, y);
    bool started = false;
    waitPose(x, y, theta, POSE_EXIT_WITHIN_IN, [&] {
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
// catchAfterMs > 0: from that long after the claw opens, close as soon as the sensor sees something, and end the
// drive there (third-pin rounds)
std::uint32_t grabPin(float x, float y, float theta, float exitWithin = POSE_EXIT_WITHIN_IN,
                      std::uint32_t catchAfterMs = 0) {
    // finishScore() returns before its outtake is over, and the outtake ends by stopping motors 14/16: let it finish
    // first, or it would stop the intake started below (normally already over by the end of the drive back).
    // Other macros (B's reset) don't touch motors 14/16 and the claw, so the grab doesn't wait for them
    while (macros::isRunning(macros::Macro::PlaceRelease)) pros::delay(10);
    subsystems::setClaw(true);
    const std::uint32_t openedAt = pros::millis();
    xMotors.move_velocity(subsystems::rpm(100));
    bool caught = false;
    auto sees = [] {
        const std::int32_t mm = clawSensor.get_distance();
        return mm > 0 && mm <= subsystems::CLAW_CATCH_MM;
    };
    // the early catch: ends the running drive when it sees the pin
    auto watch = [&] {
        if (catchAfterMs == 0 || caught || pros::millis() - openedAt < catchAfterMs || !sees()) return;
        caught = true;
        chassis.cancelMotion();
    };

    const bool anyHeading = std::isnan(theta);
    auto drive = [&](int timeoutMs, float maxSpeed) {
        if (anyHeading) chassis.moveToPoint(x, y, timeoutMs, {.forwards = true, .maxSpeed = maxSpeed});
        else chassis.moveToPose(x, y, theta, timeoutMs, {.forwards = true, .maxSpeed = maxSpeed});
    };
    drive(TO_PIN_TIMEOUT_MS, 127);
    // LemLib can't change a move's speed part way, so the first move is cancelled and a slow one started
    const lemlib::Pose pin(x, y);
    while (chassis.isInMotion() && chassis.getPose().distance(pin) > PIN_SLOW_WITHIN_IN) {
        watch();
        pros::delay(10);
    }
    if (chassis.isInMotion() && !caught) {
        // a cancelled move sets the drive to 0 for ~10 ms before the next starts: coast through that
        // instead of HOLD braking hard at speed
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
        chassis.cancelMotion();
        chassis.waitUntilDone(); // the first move has fully ended, so the next isn't dropped
        drive(PIN_SLOW_TIMEOUT_MS, PIN_SLOW_SPEED);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    }
    if (!caught) waitPose(x, y, theta, exitWithin, watch);
    chassis.waitUntilDone();

    int waited = 0;
    for (; !caught && waited < PIN_CATCH_WAIT_MS; waited += 10) {
        if (sees() && pros::millis() - openedAt >= catchAfterMs) break;
        pros::delay(10);
    }
    subsystems::setClaw(false);
    std::printf("[auton] pin (%.1f, %.1f): claw closed (%s, sensor %d mm, %.1f in from the point)\n", x, y,
                caught ? "caught on the way" : waited < PIN_CATCH_WAIT_MS ? "caught" : "timed out",
                static_cast<int>(clawSensor.get_distance()), chassis.getPose().distance(pin));
    return pros::millis();
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
    const std::uint32_t closedAt = grabPin(PIN_X, PIN_Y, PIN_THETA);

    // second goal: level 1 alliance height (440 deg)
    liftDrive(subsystems::CASCADE_ALLIANCE[0], GOAL2_X, GOAL2_Y, GOAL2_THETA, TO_GOAL2_TIMEOUT_MS, closedAt);
    finishScore();
    driveBack(BACK_IN, BACK_TIMEOUT_MS);

    // second pin
    const std::uint32_t closed2At = grabPin(PIN2_X, PIN2_Y, PIN2_THETA);

    // third goal: level 2 alliance height (640 deg)
    liftDrive(subsystems::CASCADE_ALLIANCE[1], GOAL3_X, GOAL3_Y, GOAL3_THETA, TO_GOAL3_TIMEOUT_MS, closed2At);
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
    liftDrive(GOAL5_LIFT_DEG, GOAL5_X, GOAL5_Y, GOAL5_THETA, TO_GOAL5_TIMEOUT_MS, closed5At);
    finishScore();

    // rest of the routine goes here
}
