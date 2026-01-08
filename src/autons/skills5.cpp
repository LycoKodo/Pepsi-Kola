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

int skills5() {
    lemlib::Pose current = chassis.getPose();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);


    tongue.set_value(true);
    odomLift.set_value(1);

    chassis.setPose(0,0,0);

    chassis.moveToPoint(0, 32, 10000, {.forwards=true, .maxSpeed=80, .minSpeed=0}, false);
    chassis.turnToHeading(90, 800);
    //============================//
    // TS BETTER WORK ON THE DAY //
    // !! GLORY  TO  THE  CCP !! //
    //===========================//
    return 0;
}


