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

    chassis.moveToPoint(0, 32, 950, {.forwards=true, .maxSpeed=100}, false);

    chassis.turnToHeading(90, 500, {}, false);

    tongue.set_value(true);

    ramp.move(127);

    pros::delay(500);

    chassis.moveToPose(10, 32.5, 90, 950, {.forwards=true, .lead=0, .maxSpeed=80, .minSpeed=20}, true);

    pros::delay(1100);

    chassis.cancelAllMotions();

    // -- Approach Long Goal -- //
    heaven.set_value(true);

    chassis.moveToPose(-25, 32.5, 90, 800, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=60}, false);

    intake.move(-127);

    pros::delay(1000);

    intake.move(80);

    pros::delay(10);

    intake.move(-127);

    pros::delay(1800);

    tongue.set_value(false);
    heaven.set_value(false);

    // -- Approach Mid Goal -- //

    // TODO - FOLLOWING is motion that NEEDS TUNING 

    chassis.moveToPose(-10, 32, 90, 700, {.forwards=true, .lead=0, .maxSpeed=90, .minSpeed=10}, false);

    intake.move(0);

    chassis.turnToHeading(-140, 600);

    ramp.move(127);

    chassis.moveToPose(-27, -36, -180, 2250, {.forwards=true, .lead=0.6, .maxSpeed=120, .minSpeed=30}, false);

    chassis.turnToHeading(134, 600);

    ramp.move(60);
    top1.move(-30);
    top2.move(20);

    chassis.moveToPose(-36.5, -28, 134, 900, {.forwards=false, .lead=0, .maxSpeed=110, .minSpeed=30,}, false);


    ramp.move(90);
    top1.move(-80);
    top2.move(80);

    pros::delay(800);

    intake.move(0);

    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    // pros::delay(10000);

    chassis.moveToPose(-5.56, -61, 137, 950, {.forwards=true, .lead=0, .maxSpeed=100, .minSpeed=30,}, false);

    chassis.turnToHeading(90, 600);

    heaven.set_value(true);

    chassis.moveToPose(-21.44, -61.8, 90, 950, {.forwards=false, .lead=0, .maxSpeed=100, .minSpeed=30,}, false);

    intake.move(-127);

    return 0;
}


