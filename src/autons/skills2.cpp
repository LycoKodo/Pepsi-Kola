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

    chassis.moveToPose(0, 30, 0, 1300, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=60}, false);

    chassis.turnToHeading(-90, 900, {}, false);

    pros::delay(800);

    tongue.set_value(true);

    pros::delay(800);

    ramp.move(127);

    chassis.moveToPose(-4.3, 30, -90, 800, {.forwards=true, .lead=0, .maxSpeed=15, .minSpeed=60}, true);

    chassis.moveToPose(-3, 30, -90, 800, {.forwards=false, .lead=0, .maxSpeed=15, .minSpeed=60}, true);

    chassis.moveToPose(-4.3, 30, -90, 800, {.forwards=true, .lead=0, .maxSpeed=15, .minSpeed=60}, true);

    pros::delay(1500);

    chassis.cancelAllMotions();
    ramp.move(0);

    heaven.set_value(true);

    chassis.moveToPose(25, 32, -90, 900, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=60}, false);

    intake.move(-127);

    pros::delay(2000);

    intake.move(0);

    // second run

    chassis.moveToPose(-4.3, 30, -90, 1500, {.forwards=true, .lead=0, .maxSpeed=110, .minSpeed=60}, false);
    ramp.move(127);

    pros::delay(400);
    chassis.moveToPose(-3, 30, -90, 800, {.forwards=false, .lead=0, .maxSpeed=15, .minSpeed=60}, true);

    chassis.moveToPose(-4.3, 30, -90, 800, {.forwards=true, .lead=0, .maxSpeed=15, .minSpeed=60}, true);

    pros::delay(1500);

    chassis.cancelAllMotions();
    ramp.move(0);

    heaven.set_value(true);

    chassis.moveToPose(25, 32, -90, 900, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=60}, false);

    intake.move(-127);

    pros::delay(2000);

    // Second Part

    chassis.moveToPose(13, 30, -90, 1200, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=0}, false);

    tongue.set_value(false);

    chassis.turnToHeading(180, 2000);

    chassis.moveToPose(13, -15, 180, 2500, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=0}, false);

    chassis.turnToHeading(-90, 2000);

    chassis.moveToPose(23, -15, -90, 2500, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=0}, false);

    chassis.moveToPose(-10, -15, -90, 2500, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=127}, false);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    return 0;
}


