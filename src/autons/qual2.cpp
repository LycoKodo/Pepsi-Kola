#include "main.h"

#include "pros/motors.h"
#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"



// ------------------------- //
// RIGHT  Autonomous Routine //
// ------------------------- //

// e.g. red left

int qual2() {
    chassis.setPose(0, 0, 0); 
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    // -- Approach Loader -- //

    chassis.moveToPoint(0, 32, 1000, {.forwards=true, .maxSpeed=90}, false);

    chassis.turnToHeading(90, 700, {}, false);

    tongue.set_value(true);

    ramp.move(127);

    pros::delay(300);

    chassis.moveToPose(10, 32.5, 90, 1000, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=60}, true);

    pros::delay(1000);

    chassis.cancelAllMotions();
    ramp.move(0);

    // -- Approach Long Goal -- //
    heaven.set_value(true);

    chassis.moveToPose(-25, 32.5, 90, 1100, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=60}, false);

    intake.move(-127);

    pros::delay(1000);

    intake.move(80);

    pros::delay(50);

    intake.move(-127);

    pros::delay(2000);

    tongue.set_value(false);
    heaven.set_value(false);

    // -- Approach Mid Goal -- //

    // TODO - FOLLOWING is motion that NEEDS TUNING 

    chassis.moveToPose(-10, 32, 90, 800, {.forwards=true, .lead=0, .maxSpeed=90, .minSpeed=0}, false);

    intake.move(0);

    chassis.turnToHeading(-140, 700);

    ramp.move(127);

    chassis.moveToPose(-33, -36, -180, 2500, {.forwards=true, .lead=0.6, .maxSpeed=100, .minSpeed=0,}, false);

    chassis.turnToHeading(130, 700);

    top1.move(-50);
    top2.move(-50);

    chassis.moveToPose(-43.5, -27.5, 130, 1000, {.forwards=false, .lead=0, .maxSpeed=90, .minSpeed=0,}, false);

    intake.move(-90);

    pros::delay(1600);

    intake.move(0);

    chassis.moveToPose(-12.2, -64, 137, 900, {.forwards=true, .lead=0, .maxSpeed=90, .minSpeed=0,}, false);

    chassis.turnToHeading(90, 700);

    heaven.set_value(true);

    chassis.moveToPose(-28.3, -64, 90, 1000, {.forwards=false, .lead=0, .maxSpeed=90, .minSpeed=0,}, false);

    intake.move(-127);

    return 0;
}


