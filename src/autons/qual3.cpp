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
    intake.move(127);

    // Intake
    top_intake.move(-127);
    front_intake.move(127);
    middle_intake.move(-100);
    back_intake.move(127);


    chassis.moveToPose(-2.7, 18.5, 2.6, 1400, {.forwards=true, .lead=0.01, .maxSpeed=127, .minSpeed=0, .earlyExitRange=0, }, false);
    chassis.moveToPose(-2.7, 25.3, 2.6, 1200, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=0, .earlyExitRange=0, }, false);
    chassis.turnToHeading(60, 1000);
    intake.move(0);

    // chassis.moveToPoint(12, 29.6, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=0, .earlyExitRange=0, }, false);
    chassis.moveToPose(11.6, 33, 60, 1200, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=0, .earlyExitRange=0, }, false);

    // Score Middle
    back_intake.move(100);
    top_intake.move(-100);
    middle_intake.move(100);
    front_intake.move(100);

    pros::delay(2000);

    chassis.moveToPose(15, 32.8, 60, 800, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=40, .earlyExitRange=0, }, false);
    pros::delay(1000);
    chassis.moveToPose(0, 24, 60, 2000, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=0, .earlyExitRange=0, }, false);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    return 0;
}


