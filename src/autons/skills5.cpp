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


    // -------------------------------------------------- //
    // USED TO TEST FRAGMENTS OF SKILLS AUTON FOR SKILLS6 //
    // -------------------------------------------------- //
    chassis.setPose(75,32,90);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    heaven.set_value(true);
    ::pros ::delay(3000);
    chassis.moveToPose(110, 2, 180, 5000, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);
    tongue.set_value(true);
    heaven.set_value(false);

    pros::delay(1000);

    odomLift.set_value(0);
    float x_pose = chassis.getPose().x;  
    float y_pose = chassis.getPose().y;  
    chassis.moveToPoint(x_pose, y_pose - 100, 3000, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);


    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}


