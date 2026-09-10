#include "main.h"

#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"



// ------------------------- //
// AI 5 Autonomous Routine //
// ------------------------- //

// e.g. blue left awp

int AI5() {

    chassis.setPose(0, 1, 0); 
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    // -- Approach 3 Stack -- //

    ramp.move(127);
    
    chassis.moveToPose(-0.5, 21.83, 0, 1200, {.forwards=true, .lead=0, .maxSpeed=60, .minSpeed=0}, false);
    
    chassis.moveToPose(-1, 31.62, 0, 800, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=0}, false);
    
    chassis.turnToHeading(-37.25, 800);
    
    chassis.moveToPose(-13.96, 50.33, -34.84, 1100, {.forwards=true, .lead=0, .maxSpeed=100, .minSpeed=0}, false);

    chassis.moveToPoint(-1, 31.62, 1000, {.forwards=false, .maxSpeed=127, .minSpeed=40, .earlyExitRange=15}, false);

    chassis.moveToPose(-32.41, 26.32, 205, 2000, {.forwards=false, .lead=0.05, .maxSpeed=127, .minSpeed=0}, false);

    chassis.turnToHeading(196, 800);

    heaven.set_value(1);

    chassis.moveToPoint(-26.3, 34.11, 1000, {.forwards=false, .maxSpeed=127, .minSpeed=40}, false);

    intake.move(127);

    pros::delay(1500);

    intake.move(0);

    ramp.move(127);

    tongue.set_value(1);

    heaven.set_value(0);

    chassis.moveToPose(-35.31, 2.348, 196.5, 1500, {.forwards=true, .lead=0, .maxSpeed=70, .minSpeed=0}, false);

    pros::delay(300);

    heaven.set_value(1);

    chassis.moveToPose(-25.46, 34.57, 196.5, 1500, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=0}, false);

    intake.move(127);

    tongue.set_value(0);

    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::delay(20000);
    
    return 0;
}


