#include "main.h"

#include "pros/motors.h"
#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"



// ------------------------- //
// LEFT Autonomous routine.  //
// ------------------------- //

// e.g. 

int qual1() {
    
    chassis.setPose(0, 0, 0);

    // chassis.moveToPose(0, 24, 0, 3000, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=0, .earlyExitRange=0, }, false);
    // pros::delay(800);
    // chassis.moveToPose(0, 0, 0, 3000, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=0, .earlyExitRange=0}, false);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    return 0;
}


