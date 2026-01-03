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
    odomLift.set_value(1);
    chassis.setPose(0,0,0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    

    //move to loader
    chassis.moveToPoint(0, 32, 800, {.forwards=true, .maxSpeed=80, .minSpeed=0, .earlyExitRange=8}, false);
    chassis.moveToPose(0, 32, 0, 800, {.forwards=true, .maxSpeed=80, .minSpeed=0}, false);


    //clear loader NO1
    tongue.set_value(true);
    ramp.move(127);
    top1.move(-127);
    top2.move(-127);
    chassis.turnToHeading(-87, 800);
    chassis.moveToPoint(-14, 30.5, 900, {.forwards=true, .maxSpeed=70, .minSpeed=0}, false);
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
    lemlib::Pose current = chassis.getPose();
    chassis.setPose(-7, 32, current.theta);
    ::pros ::delay(160);


    //turn into the funnel
    chassis.moveToPoint(2, 32, 900, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);
    tongue.set_value(false);
    chassis.turnToHeading(0, 900);
    chassis.moveToPose(22, 48, 90, 3000, {.forwards=true, .maxSpeed=127, .minSpeed=30}, false);


    //move to the other side of the field 2450
    chassis.moveToPoint(91.5, 48, 3000, {.forwards=true, .maxSpeed=100, .minSpeed=20}, false);
    ::pros::delay(240);
    chassis.moveToPose(99, 31, 90, 3000, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);
    heaven.set_value(true);
    ::pros::delay(240);


    //move to long goal for scoring
    chassis.moveToPoint(75, 31, 1000, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);
        //reset pose at long goal
    current = chassis.getPose();
    chassis.setPose(75,32,current.theta);
        //score long goal
    ramp.move(127);
    top1.move(-127);
    top2.move(127);
    ::pros::delay(2650);


    //move away from long goal
    chassis.moveToPoint(90, 33, 900, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);
    top1.move(-127);
    top2.move(-127);
    heaven.set_value(false);
    tongue.set_value(true);


    //clear loader NO2
    chassis.moveToPoint(104, 32, 1100, {.forwards=true, .maxSpeed=80, .minSpeed=0}, false);
        //ram into loader to clear
    for (int i = 0; i < 5; i++) {
        leftMotors.move(40);
        rightMotors.move(40);
        pros::delay(200);
        leftMotors.brake();
        rightMotors.brake();
        pros::delay(100);
    }
    ::pros ::delay(160);

    //enter scoring position before reaching goal
    chassis.moveToPoint(90, 32, 900, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);
    tongue.set_value(false);
    heaven.set_value(true);
    ::pros ::delay(240);

    //reach and score on long goal
    chassis.moveToPoint(72, 32.5, 900, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);
        //reset pose at long goal
    current = chassis.getPose();
    chassis.setPose(75,32,current.theta);
        //score long goal
    ramp.move(127);
    top1.move(-127);
    top2.move(127);
    ::pros::delay(2650);

    //move away form goal
    chassis.moveToPose(110, 3, 180, 5000, {.forwards=true, .lead=0.3, .maxSpeed=127, .minSpeed=50}, false);
    chassis.moveToPose(110, 3, 180, 2000, {.forwards=true, .lead=0.3, .maxSpeed=127, .minSpeed=0}, false);
    ::pros::delay(1000);
    heaven.set_value(false);
    tongue.set_value(true);
    ramp.brake();
    top1.brake();
    top2.brake();
    ::pros::delay(200);

    //ready to clear park
    odomLift.set_value(0);

    ::pros::delay(100);

    //clear park
    leftMotors.move(80);
    rightMotors.move(80);
    ramp.move(-40);
    pros::delay(1500);
    leftMotors.brake();
    rightMotors.brake();
    ramp.brake();

    //reset position at side of park
    leftMotors.move(-33);
    rightMotors.move(-33);
    ::pros::delay(800);
    leftMotors.brake();
    rightMotors.brake();
    ::pros::delay(150);
    odomLift.set_value(1);
    chassis.setPose(0,0,0);

    ::pros::delay(100);

    //turn to loader NO3
    chassis.moveToPose(15, 32, 0, 3000, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);
    chassis.turnToHeading(-90, 1000);
    
    //clear loader NO3
    ramp.move(127);
    top1.move(-127);
    top2.move(-127);
    chassis.moveToPoint(4, 32, 1200, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);

    for (int i = 0; i < 5; i++) {
        leftMotors.move(40);
        rightMotors.move(40);
        pros::delay(200);
        leftMotors.brake();
        rightMotors.brake();
        pros::delay(100);
    }
    ::pros ::delay(200);

    //move away form goal
    /* chassis.moveToPoint(85, 32, 1100, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);
    heaven.set_value(false);
    ::pros::delay(240);

    ramp.brake();
    top1.brake();
    top2.brake();

    //move to loader NO3
    chassis.turnToHeading(180, 900);
    ::pros::delay(160);
    chassis.moveToPoint(85, -64.5, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);
    ::pros::delay(240);
    chassis.turnToHeading(90, 900);
    ::pros::delay(160);

    tongue.set_value(true);
    ramp.move(127);
    top1.move(-127);
    top2.move(-127);

    ::pros::delay(160);

    //move into laoder

    chassis.moveToPoint(105, -64.5, 1200, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);
    chassis.moveToPoint(120, -64.5, 800, {.forwards=true, .maxSpeed=60, .minSpeed=0}, false);
        //ram into loader to clear
    for (int i = 0; i < 5; i++) {
        leftMotors.move(40);
        rightMotors.move(40);
        pros::delay(200);
        leftMotors.brake();
        rightMotors.brake();
        pros::delay(100);
    }
    ::pros ::delay(160);

/* 
    //ready to clear park
    chassis.moveToPose(110, 3, 180, 5000, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);
    tongue.set_value(true);
    heaven.set_value(false);
    odomLift.set_value(0);

    ::pros::delay(1000);

    //clear park
    leftMotors.move(80);
    rightMotors.move(80);
    ramp.move(-40);
    pros::delay(1500);
    leftMotors.brake();
    rightMotors.brake();
    ramp.brake();

    //reset position at side of park
    leftMotors.move(-40);
    rightMotors.move(-40);
    ::pros::delay(500);
    leftMotors.brake();
    rightMotors.brake();
    chassis.setPose(0,0,0);

    //turn to loader NO3
    chassis.moveToPose(15, 32, 0, 1200, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);
    chassis.turnToHeading(-90, 1000);
    
    //clear loader NO3
    ramp.move(127);
    top1.move(-127);
    top2.move(-127);
    chassis.moveToPoint(14, 32, 1200, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);

    for (int i = 0; i < 5; i++) {
        leftMotors.move(40);
        rightMotors.move(40);
        pros::delay(200);
        leftMotors.brake();
        rightMotors.brake();
        pros::delay(100);
    }
    ::pros ::delay(200);
 */


    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    return 0;
}


