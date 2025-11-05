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

    chassis.moveToPose(0, 32, 1200, {.forwards= true, .lead = 0, .maxSpeeed=85}, false);



    return 0;
}


