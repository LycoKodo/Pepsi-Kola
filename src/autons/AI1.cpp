#include "main.h"

#include "pros/motors.h"
#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"



// ------------------------- //
// AI 1 Autonomous Routine //
// ------------------------- //

// e.g. normal blue left

int AI1() {
    chassis.setPose(0, 0, 0); 
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    // -- Approach Loader -- //

    chassis.moveToPoint(0, 28.8, 1000, {.forwards=true, .maxSpeed=127, .minSpeed=20}, false);

    chassis.turnToHeading(-90, 750, {}, false);

    tongue.set_value(true);

    ramp.move(127);

    pros::delay(200);

    chassis.moveToPoint(-13, 29.2, 1000, {.forwards=true}, false);
    
    pros::delay(250);

    heaven.set_value(true);

    pros::delay(100);

    chassis.moveToPoint(21, 32, 1000, {.forwards=false, .maxSpeed=127, .minSpeed=45}, false);

    intake.move(127);

    pros::delay(1100);

    tongue.set_value(false);

    chassis.moveToPoint(8, 31, 900, {.forwards=true}, false);

    intake.move(0);

    heaven.set_value(false);

    chassis.turnToHeading(-215, 1000, {}, false);

    ramp.move(127);

    chassis.moveToPoint(24.5, 11.8, 900, {.forwards=true, .maxSpeed=127, .minSpeed=45,  .earlyExitRange=2}, true);

    pros::delay(500);

    tongue.set_value(true);

    chassis.turnToHeading(-411.8, 900, {}, false);

    ramp.move(127);
    
    chassis.moveToPoint(35, -3.8, 1500, {.forwards=false, .maxSpeed=75, .minSpeed=25}, false);

    top1.move(-55);
    top2.move(55);
    pros::delay(20000);



    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    return 0; // success, 1 for fail
}


