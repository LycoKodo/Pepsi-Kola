#include "main.h"

#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"



// ------------------------- //
// Qual 2  Autonomous Routine //
// ------------------------- //

// e.g. red left

int qual2() {
    
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(0, -24, 0, 10000, {}, true);
    pros::delay(5000);
    chassis.turnToHeading(-90, 100000);

    return 0;
}


