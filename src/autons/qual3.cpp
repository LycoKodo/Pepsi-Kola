#include "main.h"

#include "pros/motors.h"
#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"



// ------------------------- //
// Solo Mid + Low, Start Left //
// ------------------------- //

// e.g. goal rush

int qual3() {
    chassis.setPose(0, 0, 0); 
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    // -- Approach Loader -- //

    chassis.moveToPoint(0, 31, 900, {.forwards=true, .maxSpeed=120}, false);

    chassis.turnToHeading(90, 500, {}, false);

    tongue.set_value(true);

    ramp.move(127);

    pros::delay(500);

    chassis.moveToPose(10, 32.5, 90, 700, {.forwards=true, .lead=0, .maxSpeed=90, .minSpeed=50}, true);

    pros::delay(1100);

    chassis.cancelAllMotions();

    // -- Approach Long Goal -- //
    heaven.set_value(true);

    chassis.moveToPose(-25, 32.5, 90, 900, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=60}, false);

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

    chassis.moveToPose(-11, 32, 90, 600, {.forwards=true, .lead=0, .maxSpeed=120, .minSpeed=50}, false);

    intake.move(0);

    chassis.turnToHeading(-140, 600);

    pros::delay(500);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    return 0;
}


