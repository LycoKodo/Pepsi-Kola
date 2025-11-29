#include "main.h"

#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"



// ------------------------- //
// Qual 5  Autonomous Routine //
// ------------------------- //

// e.g. red left awp

int qual5() {
    
    chassis.setPose(0, 0, 0); 
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    // -- Approach Loader -- //

    chassis.moveToPoint(0, 31, 1000, {.forwards=true, .maxSpeed=127, .minSpeed=20}, false);

    chassis.turnToHeading(-90, 1000);

    tongue.set_value(true);

    chassis.moveToPoint(-10, 32, 1000, {.forwards=true}, false);
    
    pros::delay(500);

    ramp.move(127);

    heaven.set_value(true);

    pros::delay(300);

    chassis.moveToPoint(18, 32, 1000, {.forwards=false, .maxSpeed=127, .minSpeed=45, .earlyExitRange=14}, false);

    chassis.moveToPose(19, 32, -90, 500, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);

    intake.move(127);

    pros::delay(2000);

    tongue.set_value(false);

    chassis.moveToPoint(6, 32, 900, {.forwards=true}, false);

    intake.move(0);

    heaven.set_value(false);

    chassis.turnToHeading(-215, 1000);

    ramp.move(127);

    chassis.moveToPoint(24.5, 10.2, 1000, {.forwards=true, .maxSpeed=127, .minSpeed=45}, false);

    chassis.turnToHeading(-315, 900);

    chassis.moveToPoint(41.9, 28.3, 1000, {.forwards=true, .maxSpeed=127, .minSpeed=45, .earlyExitRange=15}, false);

    chassis.moveToPose(41.9, 28.3, -298.6, 700, {.forwards=true, .lead=0.05,.maxSpeed=100, .minSpeed=0}, false);

    chassis.moveToPoint(25.5, 10.2, 1000, {.forwards=false, .maxSpeed=127, .minSpeed=45, .earlyExitRange=10}, false);

    chassis.moveToPose(25.5, 10.2, -288.6, 700, {.forwards=false, .lead=0.05,.maxSpeed=127, .minSpeed=0}, false);

    chassis.turnToHeading(-403, 900);

    chassis.moveToPoint(36.1, -1.4, 1000, {.forwards=false, .maxSpeed=127, .minSpeed=20}, false);

    intake.move(127);

    // pros::delay(500);

    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    return 0;
}


