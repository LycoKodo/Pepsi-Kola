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

    // -- Approach 3 Stack -- //

    ramp.move(127);
    chassis.moveToPose(0.5, 21.83, 0, 2500, {.forwards=true, .lead=0, .maxSpeed=45, .minSpeed=0}, false);
    pros::delay(800); // pickup first row
    chassis.moveToPose(1, 31.62, 0, 2500, {.forwards=true, .lead=0, .maxSpeed=45, .minSpeed=0}, false);
    chassis.turnToHeading(119, 1200);
    chassis.moveToPose(-12.0, 37.0, 119, 2500, {.forwards=false, .lead=0, .maxSpeed=45, .minSpeed=0}, false);
    top1.move(-42);
    top2.move(42);
    ramp.move(127);
    pros::delay(20000);
    
    return 0;
}


