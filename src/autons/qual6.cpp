#include "main.h"

#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"



// ------------------------- //
// Qual 6  Autonomous Routine //
// ------------------------- //

// e.g. red right awp

int qual6() {
    chassis.setPose(0, 0, 0); 
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    // -- Approach Loader -- //

    chassis.moveToPoint(0, 30, 1000, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);

    chassis.turnToHeading(-90, 750, {.maxSpeed=127}, false);

    tongue.set_value(true);

    chassis.moveToPoint(-10, 31, 1000, {.forwards=true}, false);
    
    pros::delay(500);

    ramp.move(127);

    heaven.set_value(true);

    pros::delay(300);

    chassis.moveToPoint(19, 31, 900, {.forwards=false, .maxSpeed=127, .minSpeed=45, .earlyExitRange=14}, false);

    intake.move(127);

    chassis.moveToPose(20, 31, -90, 500, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);

    pros::delay(2000);

    tongue.set_value(false);

    chassis.moveToPoint(5, 29, 900, {.forwards=true}, false);

    intake.move(0);

    heaven.set_value(false);

    chassis.turnToHeading(-215, 1000, {}, false);

    ramp.move(127);

    chassis.moveToPoint(24.5, 10.2, 900, {.forwards=true, .maxSpeed=127, .minSpeed=45}, false);

    chassis.turnToHeading(-315, 900, {}, false);

    chassis.moveToPoint(40.9, 27.3, 900, {.forwards=true, .maxSpeed=127, .minSpeed=25}, false);

    chassis.moveToPoint(25.5, 10.2, 900, {.forwards=false, .maxSpeed=127, .minSpeed=45, .earlyExitRange=10}, false);

    chassis.moveToPose(25.5, 10.2, -288.6, 700, {.forwards=false, .lead=0.05,.maxSpeed=127, .minSpeed=0}, false);

    chassis.moveToPoint(14, 32.0, 900, {.forwards=false, .maxSpeed=127, .minSpeed=25, .earlyExitRange=15}, false);

    chassis.moveToPose(14, 32, -135, 700, {.forwards=false, .lead=0.05,.maxSpeed=127, .minSpeed=0}, false);

    ramp.move(127);

    heaven.set_value(true);

    chassis.turnToHeading(-90, 700, {}, false);

    chassis.moveToPoint(20, 31, 900, {.forwards=false}, false);

    intake.move(127);

    pros::delay(500);

    return 0;
}


