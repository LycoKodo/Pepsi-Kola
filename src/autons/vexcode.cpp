#include "main.h"
#include <cmath>

#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/util.hpp"
#include "lemlib/chassis/chassis.hpp"

// ----------------------------------------------- //
// Ported VEXcode autonomous (onauton_autonomous_0) //
// ----------------------------------------------- //
//
// VEXcode drove with SmartDrive.drive_for(direction, mm), which is relative to
// wherever the robot is. LemLib's motions are absolute, so driveFor() below
// projects the requested distance along the current heading and moves to that
// point. Distances converted from mm at 25.4 mm/in.
//
// VEXcode percentages become LemLib's 0..127 maxSpeed: 80% -> 102, 50% -> 64,
// 40% -> 51.

namespace {

/**
 * @brief Drive a distance along the current heading, blocking until done
 *
 * @param inches distance to travel; negative drives in reverse
 * @param maxSpeed 0..127
 * @param timeout milliseconds before the motion gives up
 */
void driveFor(float inches, float maxSpeed, int timeout) {
    const lemlib::Pose pose = chassis.getPose();
    // LemLib uses compass headings: 0 is +Y, increasing clockwise
    const float rad = lemlib::degToRad(pose.theta);
    chassis.moveToPoint(pose.x + inches * std::sin(rad), pose.y + inches * std::cos(rad), timeout,
                        {.forwards = inches >= 0, .maxSpeed = maxSpeed}, false);
}

} // namespace

int autonVexcode() {
    // VEXcode: drivetrain.set_heading(0, DEGREES)
    chassis.setPose(0, 0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    // the auton repeated when_started's motor setup; mechanisms_init() already
    // did the brake modes, so only the ones it changes are worth restating
    cascade.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    rollerTurn.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // five pushes against the wall at 80% drive speed
    driveFor(-11.81, 102, 1500);  // 300mm reverse
    pros::delay(500);
    driveFor(21.65, 102, 2000);   // 550mm forward
    pros::delay(500);
    driveFor(-13.78, 102, 1500);  // 350mm reverse
    pros::delay(500);
    driveFor(19.69, 102, 2000);   // 500mm forward
    pros::delay(500);
    driveFor(-15.75, 102, 1500);  // 400mm reverse

    // VEXcode issued turn_to_heading(-90) four times with 1.2s settles between,
    // because drive_for/turn_to_heading return before the robot has stopped
    // moving. LemLib's turnToHeading runs to its own exit conditions, so one
    // call replaces all four
    chassis.turnToHeading(-90, 2500, {.maxSpeed = 51}, false);
    pros::delay(500);

    // swing the roller out, creeping the main roller forward to hold the ball
    rollerMain.move_velocity(40); // set_velocity(20, PERCENT)
    rollerTurn.move_absolute(getRollerTurnHome() + 345, 200);

    // back into the goal at 40% drive speed, then nudge off it
    driveFor(-19.69, 51, 2500);   // 500mm reverse
    driveFor(-3.15, 51, 1000);    // 80mm reverse
    driveFor(1.57, 51, 1000);     // 40mm forward

    // score: main roller to full reverse for a second and a half
    rollerMain.brake();
    rollerMain.move_velocity(-200); // set_velocity(100, PERCENT), spin(REVERSE)
    pros::delay(1500);

    driveFor(19.69, 51, 2500);    // 500mm forward
    rollerMain.brake();
    rollerTurn.move_absolute(getRollerTurnHome(), 200);

    return 0;
}
