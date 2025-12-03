#include "main.h"

#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"



// ------------------------- //
// Qual 4  Autonomous Routine //
// ------------------------- //

// e.g. normal rush

int qual4() {

    chassis.setPose(0, 0, 0); 
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    // -- Approach Loader -- //

    chassis.moveToPoint(0, 28.8, 1000, {.forwards=true, .maxSpeed=127, .minSpeed=20}, false);

    chassis.turnToHeading(90, 750, {}, false);

    tongue.set_value(true);

    ramp.move(127);

    chassis.moveToPoint(13, 29.2, 1000, {.forwards=true}, false);
    
    pros::delay(250);

    heaven.set_value(true);

    pros::delay(100);

    chassis.moveToPoint(-21, 35, 1000, {.forwards=false, .maxSpeed=127, .minSpeed=45}, false);

    intake.move(127);

    pros::delay(1100);

    tongue.set_value(false);

    chassis.moveToPoint(-8, 31, 900, {.forwards=true}, false);

    intake.move(0);

    heaven.set_value(false);

    chassis.turnToHeading(215, 1000, {}, false);

    ramp.move(127);

    chassis.moveToPoint(-25.5, 10.8, 900, {.forwards=true, .maxSpeed=127, .minSpeed=45,  .earlyExitRange=2}, true);

    pros::delay(400);

    tongue.set_value(true);

    chassis.turnToHeading(315, 900, {}, false);

    tongue.set_value(false);

    chassis.moveToPose(-41, 29.3, 270, 1000, {.forwards=true, .lead=0.05,.maxSpeed=127, .minSpeed=20}, false);

    pros::delay(500);

    chassis.moveToPoint(-25.5, 10.2, 900, {.forwards=false, .maxSpeed=127, .minSpeed=45, .earlyExitRange=10}, false);

    chassis.moveToPose(-25.5, 10.2, 288.6, 700, {.forwards=false, .lead=0.05,.maxSpeed=127, .minSpeed=0}, false);

    chassis.moveToPoint(-14, 35.0, 900, {.forwards=false, .maxSpeed=127, .minSpeed=25, .earlyExitRange=15}, false);

    chassis.moveToPose(-14, 35, 135, 500, {.forwards=false, .lead=0.05,.maxSpeed=127, .minSpeed=0}, false);

    ramp.move(0);

    heaven.set_value(true);

    chassis.turnToHeading(90, 700, {}, false);

    chassis.moveToPoint(-22, 35, 900, {.forwards=false}, false);

    intake.move(127);

    odomLift.set_value(0);

    return 0;
}


