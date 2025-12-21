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

int skills6() {

    //intial setup
    chassis.setPose(0,0,0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    

    //move to loader
    chassis.moveToPoint(0, 32, 1200, {.forwards=true, .maxSpeed=127, .minSpeed=0, .earlyExitRange=8}, false);
    chassis.moveToPose(0, 32, 0, 1200, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);


    //clear loader NO1
    tongue.set_value(true);
    ramp.move(127);
    top1.move(-127);
    top2.move(-127);
    chassis.turnToHeading(-87, 1500);
    chassis.moveToPoint(-14, 32, 1200, {.forwards=true, .maxSpeed=80, .minSpeed=0}, false);
        //ram into loader to clear
    for (int i = 0; i < 5; i++) {
        leftMotors.move(40);
        rightMotors.move(40);
        pros::delay(200);
        leftMotors.brake();
        rightMotors.brake();
        pros::delay(100);
    }
        //reset pos at loader
    chassis.setPose(-7,32,-90);
    ::pros ::delay(200);



    //turn into the funnel
    chassis.moveToPoint(2, 32, 1200, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);
    tongue.set_value(false);
    chassis.turnToHeading(0, 1000);
    chassis.moveToPose(22, 48, 87, 3000, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);


    //move to the other side of the field
    chassis.moveToPoint(90, 48, 3000, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);
    chassis.moveToPose(95, 33, 86, 2500, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);
    heaven.set_value(true);
    ::pros::delay(400);


    //move to long goal for scoring
    chassis.moveToPoint(75, 33, 1200, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);
        //reset pose at long goal
    chassis.setPose(75,32,90);
        //score long goal
    ramp.move(127);
    top1.move(-127);
    top2.move(127);
    ::pros::delay(3000);


    //move away from long goal
    chassis.moveToPoint(90, 33, 1200, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);
    top1.move(-127);
    top2.move(-127);
    heaven.set_value(false);
    tongue.set_value(true);


    //clear loader NO2
    chassis.moveToPoint(104, 32, 1200, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);
        //ram into loader to clear
    for (int i = 0; i < 5; i++) {
        leftMotors.move(40);
        rightMotors.move(40);
        pros::delay(200);
        leftMotors.brake();
        rightMotors.brake();
        pros::delay(100);
    }
    ::pros ::delay(200);


    //enter scoring position before reaching goal
    chassis.moveToPoint(90, 32, 1200, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);
    tongue.set_value(false);
    heaven.set_value(true);
    ::pros ::delay(400);


    //reach and score on long goal
    chassis.moveToPoint(72, 32, 1200, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);
        //reset pose at long goal
    chassis.setPose(75,32,90);
    ramp.move(127);
    top1.move(-127);
    top2.move(127);
    ::pros::delay(3000);


    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}


