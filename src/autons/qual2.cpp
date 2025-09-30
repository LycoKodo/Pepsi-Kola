#include "main.h"

#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"



// ------------------------- //
// RIGHT  Autonomous Routine //
// ------------------------- //

// e.g. red left

int qual2() {
    chassis.setPose(0, 0, 0); 
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    intake.move(127); // 17.55

    // Intake
    top_intake.move(-127);
    front_intake.move(127);
    middle_intake.move(-100);
    back_intake.move(127);

    chassis.moveToPose(2.7, 18.5, -2.6, 2000, {.forwards=true, .lead=0.01, .maxSpeed=127, .minSpeed=0, .earlyExitRange=0, }, false);
    pros::delay(1000);
    chassis.moveToPose(2.7, 25.3, -2.6, 2000, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=0, .earlyExitRange=0, }, false);
    pros::delay(500);
    intake.move(0);
    chassis.turnToHeading(-60, 1200);

    // chassis.moveToPoint(12, 29.6, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=0, .earlyExitRange=0, }, false);
    chassis.moveToPose(-11.6, 32.55, -60, 3000, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=0, .earlyExitRange=0, }, false);
    pros::delay(500);

    // Score Middle
    back_intake.move(127);
    top_intake.move(-127);
    middle_intake.move(127);
    front_intake.move(-127);

    pros::delay(1500);

    chassis.moveToPose(-15, 32.8, -60, 1500, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=40, .earlyExitRange=0, }, false);
    pros::delay(1000);
    chassis.moveToPose(0, 24, -60, 2000, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=0, .earlyExitRange=0, }, false);

    return 0;
}


