#include "main.h"

#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"



// ------------------------- //
// AI 3 Autonomous Routine //
// ------------------------- //

// e.g. goal rush

int AI3() {
    
    chassis.setPose(0, 0.8, 0); 
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    // -- Approach 3 Stack -- //

    ramp.move(127);
    
    chassis.moveToPose(-0.5, 21.83, 0, 1200, {.forwards=true, .lead=0, .maxSpeed=60, .minSpeed=0}, false);
    
    chassis.moveToPose(-1, 31.62, 0, 800, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=0}, false);
    
    chassis.turnToHeading(-37.25, 800);
    
    chassis.moveToPose(-12.96, 48.63, -34.84, 1000, {.forwards=true, .lead=0, .maxSpeed=100, .minSpeed=0}, false);

    chassis.moveToPose(-14.219, 50.563, -26.1, 1200, {.forwards=true, .lead=0, .maxSpeed=110, .minSpeed=0}, false);

    chassis.moveToPose(14.24, 41.18, -120.65, 2500, {.forwards=false, .lead=0.28, .maxSpeed=127, .minSpeed=40}, false);

    top1.move(-65);
    top2.move(65);
    ramp.move(127);

    pros::delay(700);

    top1.move(0);
    top2.move(0);

    chassis.moveToPose(-32, 9.26, -160.84, 1800, {.forwards=true, .lead=0.02, .maxSpeed=127, .minSpeed=0}, false);

    tongue.set_value(1);

    pros::delay(200);

    // chassis.moveToPose(-35.249, -3.255, -160.50, 1000, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=20}, false);

    // pros::delay(400);

    leftMotors.move(50);
    rightMotors.move(50);
    pros::delay(800);
    leftMotors.move(0);
    rightMotors.move(0);

    pros::delay(400);


    heaven.set_value(1);

    chassis.moveToPose(-23.68, 37.34, -160.50, 1500, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=0}, false);

    intake.move(127);

    tongue.set_value(0);

    pros::delay(1000);

    ramp.move(-30);

    pros::delay(300);

    ramp.move(127);

    //================================================================================

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::delay(20000);

    return 0;
}


