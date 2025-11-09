#include "main.h"

#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"



// ------------------------- //
// Qual 4  Autonomous Routine //
// ------------------------- //

// e.g. normal rush

int qual4() {
    

    chassis.setPose(0, 0, 0); 
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    // -- Approach Loader -- //

    chassis.moveToPoint(0, 31, 1000, {.forwards=true, .maxSpeed=120}, false);

    chassis.turnToHeading(90, 500, {}, false);

    tongue.set_value(true);

    ramp.move(127);

    pros::delay(500);

    chassis.moveToPose(10, 32, 90, 1000, {.forwards=true, .lead=0, .maxSpeed=90, .minSpeed=30}, true);

    pros::delay(1100);

    chassis.cancelAllMotions();

    // -- Approach Long Goal -- //
    heaven.set_value(true);

    chassis.moveToPose(-25, 32, 90, 1000, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=30}, false);

    intake.move(-127);

    pros::delay(2000);

    intake.move(80);

    pros::delay(500);

    intake.move(-127);

    pros::delay(2000);

    tongue.set_value(false);
    heaven.set_value(false);

    // -- Approach Mid Goal -- //

    // TODO - FOLLOWING is motion that NEEDS TUNING 

    chassis.moveToPose(-11, 32, 90, 1000, {.forwards=true, .lead=0, .maxSpeed=120, .minSpeed=30}, false);

    intake.move(0);

    chassis.turnToHeading(-140, 600);

    ramp.move(127);

    chassis.moveToPose(-29, -40, -180, 2000, {.forwards=true, .lead=0.6, .maxSpeed=127, .minSpeed=70}, false);

    chassis.turnToHeading(134, 600);

    ramp.move(60);
    top1.move(-30);
    top2.move(20);

    chassis.moveToPose(-40, -30.5, 134, 1000, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=40,}, false);


    ramp.move(90);
    top1.move(-80);
    top2.move(80);

    // pros::delay(550);

    // top1.move(0);
    // top2.move(0);

    // // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    // // pros::delay(10000);

    // chassis.moveToPose(-10.56, -63.5, 137, 780, {.forwards=true, .lead=0, .maxSpeed=120, .minSpeed=60,}, false);

    // chassis.turnToHeading(90, 600);

    // heaven.set_value(true);

    // chassis.moveToPose(-25.44, -63.5, 90, 780, {.forwards=false, .lead=0, .maxSpeed=120, .minSpeed=60,}, false);

    // intake.move(-127);



    return 0;
}


