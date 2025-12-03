#include "main.h"

#include "pros/motors.h"
#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"



// ------------------------- //
// Solo Mid + Low, Start Left //
// ------------------------- //

// e.g. goal rush

int qual3() {
    chassis.setPose(0, 0, 0); 
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    // -- Approach Loader -- //

    chassis.moveToPose(0, 33, 0, 2000, {.forwards=true, .maxSpeed=90, .minSpeed=0}, false);

    chassis.turnToHeading(-90, 1000, {.maxSpeed=90}, false);

    tongue.set_value(true);

    ramp.move(127);

    pros::delay(700);

    chassis.moveToPoint(-10, 33, 1000, {.forwards=true, .maxSpeed=100}, false);
    
    pros::delay(300); // @NOTE - loader delay

    heaven.set_value(true);

    pros::delay(300);

    chassis.moveToPose(25, 33, -90, 2000, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);

    tongue.set_value(false);

    intake.move(127);

    pros::delay(1500);

    intake.move(-30);

    pros::delay(500);

    intake.move(127);

    odomLift.set_value(0);

    pros::delay(50000);
    return 0;
}


