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
    chassis.moveToPoint(0, 12, 2000, {}, false);
    return 0;
}


