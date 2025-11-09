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

    chassis.moveToPoint(0, 30, 900, {.forwards=true, .maxSpeed=120}, false);

    chassis.turnToHeading(90, 500, {}, false);

    tongue.set_value(true);

    ramp.move(127);

    pros::delay(500);

    chassis.moveToPose(10, 31.5, 90, 700, {.forwards=true, .lead=0, .maxSpeed=90, .minSpeed=50}, true);

    pros::delay(1100);

    chassis.cancelAllMotions();

    // -- Approach Long Goal -- //
    heaven.set_value(true);

    chassis.moveToPose(-25, 31.5, 90, 900, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=60}, false);

    intake.move(-127);

    pros::delay(1000);

    intake.move(80);

    pros::delay(10);

    intake.move(-127);

    pros::delay(1200);

    tongue.set_value(false);
    heaven.set_value(false);

    // -- Approach Mid Goal -- //

    // TODO - FOLLOWING is motion that NEEDS TUNING 

    chassis.moveToPose(-11, 31, 90, 600, {.forwards=true, .lead=0, .maxSpeed=120, .minSpeed=50}, false);

    intake.move(0);

    chassis.turnToHeading(-140, 600);

    ramp.move(127);

    chassis.moveToPose(-29, -40, -180, 1800, {.forwards=true, .lead=0.6, .maxSpeed=127, .minSpeed=80}, false);

    chassis.turnToHeading(134, 600);

    ramp.move(60);
    top1.move(-30);
    top2.move(20);

    chassis.moveToPose(-38.5, -29, 134, 700, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=70,}, false);


    ramp.move(90);
    top1.move(-80);
    top2.move(80);

    pros::delay(550);

    top1.move(0);
    top2.move(0);

    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    // pros::delay(10000);

    chassis.moveToPose(-10.56, -63.5, 137, 800, {.forwards=true, .lead=0, .maxSpeed=120, .minSpeed=60,}, false);

    chassis.turnToHeading(90, 600);

    heaven.set_value(true);

    chassis.moveToPose(-25.44, -63.5, 90, 780, {.forwards=false, .lead=0, .maxSpeed=120, .minSpeed=60,}, false);

    intake.move(-127);

    return 0;
}


