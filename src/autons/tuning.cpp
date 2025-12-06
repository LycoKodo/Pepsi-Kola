#include "main.h"

#include "robot-config.hpp"
#include "lemlib/chassis/chassis.hpp"

// Simple linear routine for PID tuning: drive forward and back.
int tuningLinear() {
    chassis.setPose(0, 0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.moveToPoint(0, 24, 2000, {.forwards = true, .maxSpeed = 127}, false);
    pros::delay(250);
    chassis.moveToPoint(0, 0, 2000, {.forwards = false, .maxSpeed = 127}, false);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    return 0;
}

// Simple angular routine for PID tuning: turn out and back.
int tuningAngular() {
    chassis.setPose(0, 0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.turnToHeading(180, 1500, {}, false);
    pros::delay(250);
    chassis.turnToHeading(0, 1500, {}, false);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    return 0;
}
