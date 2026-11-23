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
constexpr float PIN_X = -36; // 2 in left of -34 (tried: -36, 20 then -34, 21)
constexpr float PIN_Y = 21;
constexpr float PIN_THETA = -45;

// second goal: lift to level 1 alliance height while driving here, and score
constexpr float GOAL2_X = -7.3; // 2 in right (+x) of -9.3 (was -13.3, then 4 in forward to -9.3)
constexpr float GOAL2_Y = 23.5;
constexpr float GOAL2_THETA = 90;
constexpr int TO_GOAL2_TIMEOUT_MS = 2000; // was 3000

// second pin: grab, the same way as the first
// 2 in to the robot's right (facing 47.5 deg) of -11.06, 48.6 (tried: -10, -12, -14 at 45.9; -12.53, 47.25; -11.06, 48.6)
constexpr float PIN2_X = -9.71;
constexpr float PIN2_Y = 47.13;
constexpr float PIN2_THETA = 47.5;

// third goal: lift to level 2 alliance height while driving here, and score
constexpr float GOAL3_X = -7.5; // 3 in right of -4.5 (robot faces 180 here: its right is -x). Tried -10, -12, -15, -10, -15, -5.5, -0.5, -4.5
constexpr float GOAL3_Y = 17.9; // 3 in forward of 20.9 (facing 180: forward is -y). Tried 24, 22, 20, 28.9, 23.9, 20.9
constexpr float GOAL3_THETA = 180;
constexpr int TO_GOAL3_TIMEOUT_MS = 2000; // was 3000

// after the third goal: B's reset (lift to the bottom and re-zeroed, roller homed) while driving straight to the
// third pin. (Backing up to -7, 46 facing 180 first was tried and taken out.) Before that drive, straight back
// this far (+y: the robot faces 180 there)
constexpr float BACK3_IN = 25; // was 20
constexpr int BACK3_TIMEOUT_MS = 1800; // was 1500 (for 20 in)
// B's reset starts once the robot has backed this far from the goal. B finds the bottom by driving the lift down
// until it stops; started at the goal, the lift is still sitting on it, so B "found the bottom" there, zeroed the
// lift at goal height and never went down (2026-11-20)
constexpr float RESET_AFTER_BACK_IN = 6;
// then turn on the spot to this heading before driving to the third pin
constexpr float PIN3_TURN_THETA = 90;
constexpr int PIN3_TURN_TIMEOUT_MS = 1000;

// third pin: grab, the same way as the others
constexpr float PIN3_X = 12;
constexpr float PIN3_Y = 57;
constexpr float PIN3_THETA = GOAL2_THETA; // the same heading as the goal scored after the first pin (90)
// then straight back this far (-x: it faces 90 there), and back to the third goal's point (GOAL3_*, facing 180),
// lifting to level 3 alliance height (900 deg) on the way, and score there
constexpr float BACK4_IN = 11;
constexpr int BACK4_TIMEOUT_MS = 1000;
constexpr int RETURN_TO_GOAL3_TIMEOUT_MS = 2500; // about 40 in with a 90 deg turn
// then the whole third-pin cycle again (back up + B, turn, grab at the third pin, back up, to the third goal's
// point), this time lifting to level 4 alliance height (1020 deg). Driver control's anti-tip (controls.cpp) caps
// the drive at 80% with the lift at 1020, so this drive gets the same cap
constexpr float LEVEL4_MAX_SPEED = 127 * 0.8;

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
std::uint32_t grabPin(float x, float y, float theta, float exitWithin = POSE_EXIT_WITHIN_IN) {
    // finishScore() returns before its outtake is over, and the outtake ends by stopping motors 14/16: let it finish
    // first, or it would stop the intake started below (normally already over by the end of the drive back).
    // Other macros (B's reset) don't touch motors 14/16 and the claw, so the grab doesn't wait for them
    while (macros::isRunning(macros::Macro::PlaceRelease)) pros::delay(10);
    subsystems::setClaw(true);
    xMotors.move_velocity(subsystems::rpm(100));

    const bool anyHeading = std::isnan(theta);
    auto drive = [&](int timeoutMs, float maxSpeed) {
        if (anyHeading) chassis.moveToPoint(x, y, timeoutMs, {.forwards = true, .maxSpeed = maxSpeed});
        else chassis.moveToPose(x, y, theta, timeoutMs, {.forwards = true, .maxSpeed = maxSpeed});
    };
    drive(TO_PIN_TIMEOUT_MS, 127);
    // LemLib can't change a move's speed part way, so the first move is cancelled and a slow one started
    const lemlib::Pose pin(x, y);
    while (chassis.isInMotion() && chassis.getPose().distance(pin) > PIN_SLOW_WITHIN_IN) pros::delay(10);
    if (chassis.isInMotion()) {
        // a cancelled move sets the drive to 0 for ~10 ms before the next starts: coast through that
        // instead of HOLD braking hard at speed
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
        chassis.cancelMotion();
        chassis.waitUntilDone(); // the first move has fully ended, so the next isn't dropped
        drive(PIN_SLOW_TIMEOUT_MS, PIN_SLOW_SPEED);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    }
    waitPose(x, y, theta, exitWithin);

    int waited = 0;
    for (; waited < PIN_CATCH_WAIT_MS; waited += 10) {
        const std::int32_t mm = clawSensor.get_distance();
        if (mm > 0 && mm <= subsystems::CLAW_CATCH_MM) break;
        pros::delay(10);
    }
    subsystems::setClaw(false);
    std::printf("[auton] pin (%.1f, %.1f): claw closed (%s, sensor %d mm)\n", x, y,
                waited < PIN_CATCH_WAIT_MS ? "caught" : "timed out", static_cast<int>(clawSensor.get_distance()));
    return pros::millis();
}

// after a grab: lift to `liftDeg` while driving to the goal (x, y, theta), scoring on the way in (finishScore()
// after it). Motors 14/16 keep
// intaking CLAW_INTAKE_AFTER_MS after the grab (`closedAt`, from grabPin), as L1 does, during the drive
void liftDrive(double liftDeg, float x, float y, float theta, int timeoutMs, std::uint32_t closedAt,
               float maxSpeed = 127) {
    cascade.move_absolute(liftDeg, subsystems::rpm(100));
    chassis.moveToPose(x, y, theta, timeoutMs, {.forwards = true, .maxSpeed = maxSpeed});
    waitPoseThenScore(x, y, theta, [closedAt] {
        if (pros::millis() - closedAt >= subsystems::CLAW_INTAKE_AFTER_MS) xMotors.brake();
    });
    // (if scoring started before the 2 s were up, startScore() stopped the intake: the outtake comes next anyway)
}

// From just after scoring at the third goal: back away (B's reset once clear of the goal), turn, grab the third
// pin, back away from it, then lift to `liftDeg` while driving back to the third goal's point, and score there.
// Waits until the claw opens on the goal
void thirdPinCycle(double liftDeg, float maxSpeed = 127) {
    // B's reset once clear of the goal (RESET_AFTER_BACK_IN). The scoring motion is completely over first (its
    // outtake too): starting a macro cancels the one running. B carries on in the macro task during the drive to
    // the pin
    while (macros::isBusy()) pros::delay(10);
    driveBack(BACK3_IN, BACK3_TIMEOUT_MS, [] { macros::request(macros::Macro::ResetAll, true); }, RESET_AFTER_BACK_IN);
    chassis.turnToHeading(PIN3_TURN_THETA, PIN3_TURN_TIMEOUT_MS);
    chassis.waitUntilDone();

    // the pin. The reset may still be running: it uses the lift and roller, the grab doesn't
    const std::uint32_t closedAt = grabPin(PIN3_X, PIN3_Y, PIN3_THETA);
    // back away from the pin (motors 14/16 keep intaking their 2 s after the grab, as L1; liftDrive stops them)
    driveBack(BACK4_IN, BACK4_TIMEOUT_MS);

    // back to the third goal's point, lifting on the way, and score there (scoring starts SCORE_EARLY_IN out, as
    // at the other goals). B's reset is long over by now; wait for it if not, so it doesn't fight the lift
    while (macros::isRunning(macros::Macro::ResetAll)) pros::delay(10);
    liftDrive(liftDeg, GOAL3_X, GOAL3_Y, GOAL3_THETA, RETURN_TO_GOAL3_TIMEOUT_MS, closedAt, maxSpeed);
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
    cascade.move_absolute(TO_GOAL_LIFT_DEG, subsystems::rpm(100));
    waitPoseThenScore(GOAL_X, GOAL_Y, GOAL_THETA);
    finishScore(true); // the first goal keeps the full 0.3 s outtake before driving away
    driveBack(BACK_IN, BACK_TIMEOUT_MS);

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
    thirdPinCycle(subsystems::CASCADE_ALLIANCE[2]);

    // the same again, at level 4 alliance height (1020 deg)
    thirdPinCycle(subsystems::CASCADE_ALLIANCE[3], LEVEL4_MAX_SPEED);

    // rest of the routine goes here
}
