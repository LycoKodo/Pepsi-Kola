#include "main.h"

#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"


// ------------------------- //
// Skills Autonomous Routine //
// ------------------------- //

// e.g. spin intake/score preload

int skills2() {
        chassis.setPose(0, 0, 0); 
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.moveToPose(0, 30, 0, 1500, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=60}, false);

    chassis.turnToHeading(-90, 900, {}, false);

    tongue.set_value(true);

    pros::delay(500);

    ramp.move(127);

    chassis.moveToPose(-12, 31.5, -90, 1000, {.forwards=true, .lead=0, .maxSpeed=80, .minSpeed=20}, false);

    chassis.moveToPose(-3, 31.5, -90, 950, {.forwards=false, .lead=0, .maxSpeed=80, .minSpeed=20}, false);

    chassis.moveToPose(-12, 31.5, -90, 1000, {.forwards=true, .lead=0, .maxSpeed=80, .minSpeed=20}, false);

    chassis.cancelAllMotions();
    ramp.move(0);

    heaven.set_value(true);

    chassis.moveToPose(27, 32.5, -90, 900, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=60}, false);

    intake.move(-127);

    pros::delay(3000);

    intake.move(0);

    // second run

    chassis.moveToPose(-12, 31.5, -90, 1500, {.forwards=true, .lead=0, .maxSpeed=110, .minSpeed=60}, false);
    ramp.move(127);

    pros::delay(400);

    chassis.moveToPose(-3, 31.5, -90, 950, {.forwards=false, .lead=0, .maxSpeed=80, .minSpeed=20}, true);

    chassis.moveToPose(-12, 31.5, -90, 800, {.forwards=true, .lead=0, .maxSpeed=80, .minSpeed=20}, true);

    pros::delay(1500);

    chassis.cancelAllMotions();
    ramp.move(0);

    heaven.set_value(true);

    chassis.moveToPose(27, 32, -90, 1500, {.forwards=false, .lead=0, .maxSpeed=110, .minSpeed=60}, false);

    intake.move(-127);

    pros::delay(2000);

    // Second Part

    chassis.moveToPose(10, 30, -90, 1200, {.forwards=true, .lead=0, .maxSpeed=110, .minSpeed=0}, false);

    tongue.set_value(false);

    intake.move(0);

    heaven.set_value(false);

    chassis.turnToHeading(180, 2000);

    chassis.moveToPose(10, -61, 180, 2500, {.forwards=true, .lead=0, .maxSpeed=120, .minSpeed=0}, false);

    chassis.turnToHeading(270, 2000);

    tongue.set_value(true);

    ramp.move(127);

    chassis.moveToPose(-13, -61, 270, 1500, {.forwards=true, .lead=0, .maxSpeed=90, .minSpeed=0}, false);

    pros::delay(1000);

    heaven.set_value(true);

    chassis.moveToPose(27, -61, 270, 1500, {.forwards=false, .lead=0, .maxSpeed=120, .minSpeed=20}, false);

    intake.move(-127);

    pros::delay(3000);

    intake.move(0);

    ramp.move(127);

    chassis.moveToPose(-13, -61, 270, 1500, {.forwards=true, .lead=0, .maxSpeed=100, .minSpeed=0}, false);

    heaven.set_value(false);

    pros::delay(1500);

    heaven.set_value(true);

    chassis.moveToPose(27, -61.4, 270, 1500, {.forwards=false, .lead=0, .maxSpeed=120, .minSpeed=20}, false);

    intake.move(-127);

    pros::delay(3000);

    intake.move(-127);

    chassis.moveToPose(10, -61.4, 270, 1500, {.forwards=true, .lead=0, .maxSpeed=120, .minSpeed=0}, false);

    heaven.set_value(false);

    tongue.set_value(false);

    chassis.turnToHeading(0, 1500);

    chassis.moveToPose(10, -14.5, 0, 1500, {.forwards=true, .lead=0, .maxSpeed=120, .minSpeed=0}, false);

    intake.move(0);

    chassis.turnToHeading(270, 1500);

    chassis.moveToPose(20, -14.5, 270, 1500, {.forwards=false, .lead=0, .maxSpeed=120, .minSpeed=0}, false);

    chassis.moveToPose(-30, -14.5, 270, 5000, {.forwards=true, .lead=0, .maxSpeed=120, .minSpeed=0}, false);

    // chassis.turnToHeading(0, 2000);

    // chassis.turnToHeading(-315, 2000);

    // top1.move(0);
    // top2.move(0);

    // chassis.moveToPose(19.59, -2.69, -315, 2500, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=0}, false);

    // chassis.moveToPose(-10, -15, -90, 2500, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=127}, false);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    return 0;
}


