#include "main.h"

#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"



// ------------------------- //
// Skills Autonomous Routine //
// ------------------------- //

// e.g. 100% success run

int skills1() {

    top1.move(127);
    pros::delay(2000);
    top1.brake();
    
    return 0;
}


