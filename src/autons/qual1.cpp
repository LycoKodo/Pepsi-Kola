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

int qual1() {
    chassis.setPose(0, 0, 0); 
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    // TODO - Starter Struct:
        // chassis.moveToPose(0, 24, 0, 2000, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=0 }, false);


    return 0;
}


