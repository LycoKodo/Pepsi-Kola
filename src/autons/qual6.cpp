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

    chassis.moveToPoint(0, 32, 4000, {.forwards=true}, false);

    chassis.turnToHeading(-90, 4000);

    tongue.set_value(true);

    chassis.moveToPoint(-10, 32, 4000, {.forwards=true}, false);

    ramp.move(127);

    heaven.set_value(true);

    pros::delay(500);

    chassis.moveToPose(19, 32, -90, 2000, {.forwards=false, .maxSpeed=127, .minSpeed=20}, false);

    intake.move(127);

    tongue.set_value(false);

    chassis.moveToPoint(8, 32, 4000, {.forwards=true}, false);

    intake.move(0);

    heaven.set_value(false);

    chassis.turnToHeading(-215, 4000);

    ramp.move(127);

    chassis.moveToPose(23.4, 9.0, -224, 4000, {.forwards=true, .maxSpeed=127, .minSpeed=20}, false);

    chassis.turnToHeading(-315, 2000);

    chassis.moveToPose(41.9, 27.9, -298.6, 4000, {.forwards=true, .lead=0.05,.maxSpeed=100, .minSpeed=20}, false);

    chassis.moveToPose(25.5, 10.2, -288.6, 4000, {.forwards=false, .lead=0.05,.maxSpeed=127, .minSpeed=20}, false);

    chassis.moveToPose(9.35, 31.0, -176.0, 4000, {.forwards=false, .lead=0.05,.maxSpeed=127, .minSpeed=20}, false);

    ramp.move(0);

    chassis.turnToHeading(-90, 2000);

    heaven.set_value(true);

    chassis.moveToPose(19, 32, -90, 4000, {.forwards=false, .maxSpeed=127, .minSpeed=20}, false);

    intake.move(127);

    pros::delay(500);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    return 0;
}


