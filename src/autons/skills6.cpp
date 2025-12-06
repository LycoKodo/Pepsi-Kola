#include "main.h"

#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"



// ------------------------- //
// Skills Autonomous Routine //
// ------------------------- //

// e.g. idk whatever you want

int skills6() {
    ramp.move(-127);
    top1.move(127);
    top2.move(-127);
    chassis.moveToPose(0, 48, 0, 1500, {.forwards=true}, false);
    chassis.moveToPose(0, 0, 0, 1500, {.forwards=false}, false);

    ramp.move(127);
    top1.move(-127);
    top2.move(-127);
    return 0;
}


