#include "main.h"

#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"



// ------------------------- //
// Skills Autonomous Routine //
// ------------------------- //

// e.g. idk whatever you want

int skills4() {
    chassis.setPose(0,0,0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    
    chassis.moveToPoint(0, 33, 1200, {.forwards=true, .maxSpeed=127, .minSpeed=0, .earlyExitRange=8}, false);
    chassis.moveToPose(0, 33, 0, 1200, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);

    tongue.set_value(true);

    ramp.move(127);
    top1.move(-127);
    top2.move(-127);

    chassis.turnToHeading(-90, 1500);
    chassis.moveToPoint(-10, 33, 1200, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);
    
    pros::delay(200);

    leftMotors.move(50);
    rightMotors.move(50);
    pros::delay(1500);
    leftMotors.move(0);
    rightMotors.move(0);

    chassis.moveToPoint(9, 32.5, 1200, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);
    chassis.turnToHeading(180, 1200, {}, false);

    tongue.set_value(false);

    chassis.moveToPoint(9, -66.5, 1200, {.forwards=true, .maxSpeed=127, .minSpeed=60, .earlyExitRange=24}, false);
    chassis.moveToPose(9, -66.5, 180, 1200, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);

    chassis.turnToHeading(-90, 1200, {}, false);
    heaven.set_value(true);
    pros::delay(600);

    chassis.moveToPoint(30, -66.5, 1200, {.forwards=false, .maxSpeed=127, .minSpeed=0, .earlyExitRange=0}, false);
    intake.move(127);

    tongue.set_value(true);
    pros::delay(2000);

    ramp.move(127);
    top1.move(-127);
    top2.move(-127);

    chassis.moveToPoint(0, -66.5, 1500, {.maxSpeed=100, .minSpeed=45, .earlyExitRange=5}, false);
    chassis.moveToPoint(-10, -66.5, 1500, {.maxSpeed=80, .minSpeed=0}, false);

    pros::delay(200);

    leftMotors.move(50);
    rightMotors.move(50);
    pros::delay(1500);
    leftMotors.move(0);
    rightMotors.move(0);

    // intake.move(127);


    
    return 0;
}


