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
    chassis.moveToPoint(0, 32.5, 800, {.forwards=true, .maxSpeed=80, .minSpeed=0, .earlyExitRange=8}, false);
    chassis.moveToPose(0, 32.5, 0, 600, {.forwards=true, .maxSpeed=80, .minSpeed=0}, false);


    //clear loader NO1
    tongue.set_value(true);
    ramp.move(127);
    top1.move(-100);
    top2.move(-127);
    chassis.turnToHeading(-90, 800);
    chassis.moveToPoint(-10, 32.5, 900, {.forwards=true, .maxSpeed=100, .minSpeed=30}, false);
        //ram into loader to clear
    for (int i = 0; i < 5; i++) {
        leftMotors.move(50);
        rightMotors.move(50);
        pros::delay(200);
        leftMotors.brake();
        rightMotors.brake();
        pros::delay(100);
    }
        //reset pos at loader
    lemlib::Pose current = chassis.getPose();
    ///--================--///
    ///    CHECK POINT    ///
    ///--===============--///
    chassis.setPose(-7, 32, current.theta);
    ::pros ::delay(160);


    //turn into the funnel
    chassis.moveToPoint(2, 32, 900, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);
    tongue.set_value(false);
    chassis.turnToHeading(0, 900);
    chassis.moveToPose(22, 48, 90, 3000, {.forwards=true, .maxSpeed=127, .minSpeed=30}, false);


    //move to the other side of the field 2450
    chassis.moveToPoint(92, 48, 2500, {.forwards=true, .maxSpeed=110, .minSpeed=20}, false);
    ::pros::delay(240);
    //!!!!changed from 2600
    chassis.moveToPose(92, 31.5, 90, 4000, {.forwards=true, .maxSpeed=127, .minSpeed=10}, false);
    heaven.set_value(true);
    ::pros::delay(400);

    //!!!!delayed extra second
    ::pros ::delay(500);


    //move to long goal for scoring

    chassis.turnToHeading(90, 900);

    //!!!!changed from 1000
    chassis.moveToPoint(75, 33, 1200, {.forwards=false, .maxSpeed=127, .minSpeed=40}, false);
        //reset pose at long goal
        //score long goal
    ramp.move(127);
    top1.move(-127);
    top2.move(127);
    ::pros::delay(1400);
    current = chassis.getPose();
    ///--================--///
    ///    CHECK POINT    ///
    ///--===============--///
    chassis.setPose(75,32,current.theta);
    ::pros::delay(1400);


    //move away from long goal
    chassis.moveToPoint(90, 32, 700, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);
    top1.move(-100);
    top2.move(-127);
    heaven.set_value(false);
    tongue.set_value(true);


    //clear loader NO2
    chassis.moveToPoint(104, 32, 1100, {.forwards=true, .maxSpeed=70, .minSpeed=0}, false);
        //ram into loader to clear
    for (int i = 0; i < 5; i++) {
        leftMotors.move(30);
        rightMotors.move(30);
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
    ::pros ::delay(400);

    //reach and score on long goal
    chassis.moveToPoint(72, 32, 900, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);

        //score long goal
    ramp.move(127);
    top1.move(-127);
    top2.move(127);
    ::pros::delay(1400);
        //reset pose at long goal
    current = chassis.getPose();
    ///--================--///
    ///    CHECK POINT    ///
    ///--===============--///
    chassis.setPose(75,32,current.theta);
    ::pros::delay(1400);


    //move away form goal
    chassis.moveToPose(110, 2, 180, 3500, {.forwards=true, .lead=0.3, .maxSpeed=127, .minSpeed=0}, false);

    //prepare to clear park
    heaven.set_value(false);
    tongue.set_value(true);
    top1.brake();
    top2.brake();
    odomLift.set_value(0);
    ramp.move(127);

    ::pros::delay(100);


    //clear park
    leftMotors.move(80);
    rightMotors.move(80);
    ::pros::delay(1500);
    
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
    ///--================--///
    ///    CHECK POINT    ///
    ///--===============--///
    chassis.setPose(0,0,0);


    ::pros::delay(100);
    //turn to loader NO3
    chassis.moveToPose(15, 32.5, 0, 3000, {.forwards=true, .maxSpeed=127, .minSpeed=30}, false);

        //turn to loader NO3
    chassis.turnToHeading(-90, 1000);
    ::pros::delay(160);
    
    //clear loader NO3
    ramp.move(127);
    top1.move(-100);
    top2.move(-127);
    chassis.moveToPose(2, 32.5 , -90, 1200, {.forwards=true, .maxSpeed=110, .minSpeed=70}, false);

    for (int i = 0; i < 5; i++) {
        leftMotors.move(30);
        rightMotors.move(30);
        pros::delay(200);
        leftMotors.brake();
        rightMotors.brake();
        pros::delay(100);
    }
    ::pros ::delay(200);
    tongue.set_value(false);


    //move across field
        //into the channel
    chassis.moveToPose(34, 47.5, -90, 2500, {.forwards=false, .maxSpeed=127, .minSpeed=50}, false);
    ::pros::delay(160);
        //across the channel
    chassis.moveToPose(90, 47.5, -90, 2000, {.forwards=false, .maxSpeed=127, .minSpeed=20}, false);
    ::pros::delay(160);
        //approach long in two movements to adjust pos
    chassis.moveToPose(105, 34.8, 0, 2000, {.forwards=false, .maxSpeed=127, .minSpeed=60}, false);
    chassis.moveToPoint(105, 34.8, 1000, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);
        //turn to long
    chassis.turnToHeading(90, 1000, {.maxSpeed=127, .minSpeed=50}, false);
    ::pros::delay(100);
    heaven.set_value(true);
    ::pros ::delay(400);

    //move to long and score
    chassis.moveToPoint(88, 34.8, 1000, {.forwards=false, .maxSpeed=127, .minSpeed=50}, false);
    ramp.move(127);
    top1.move(-127);
    top2.move(127);
        //reset position mid scoring
    ::pros::delay(1325);
    current = chassis.getPose();
    ///--================--///
    ///    CHECK POINT    ///
    ///--===============--///
    chassis.setPose(88, 32, current.theta);
    ::pros::delay(1325);


    //move to loader NO4
    chassis.moveToPoint(100, 32.5, 1000, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);
    heaven.set_value(false);
    tongue.set_value(true);
    ramp.move(127);
    top1.move(-100);
    top2.move(-127);
        //reach loader NO4
    chassis.moveToPoint(116, 32.5, 1300, {.forwards=true, .maxSpeed=70, .minSpeed=0}, false);

    for (int i = 0; i < 5; i++) {
        leftMotors.move(30);
        rightMotors.move(30);
        pros::delay(200);
        leftMotors.brake();
        rightMotors.brake();
        pros::delay(100);
    }
    ::pros ::delay(200);

    //move to long and score
    chassis.moveToPoint(100, 32.5, 600, {.forwards=false, .maxSpeed=127, .minSpeed=50}, false);
        //enter scoring position
    heaven.set_value(true);
    tongue.set_value(false);
    ::pros::delay(400);
        //reach long and score
    chassis.moveToPoint(85, 32.5, 600, {.forwards=false, .maxSpeed=127, .minSpeed=50}, false);
    ramp.move(127);
    top1.move(-127);
    top2.move(127);
        //reset position while scoring
    ::pros::delay(1325);
    ///--================--///
    ///    CHECK POINT    ///
    ///--===============--///
    chassis.setPose(88, 32.5, 90);
    ::pros::delay(1325);


    //move to park
    chassis.moveToPose(123, 2.5, 180, 3000, {.forwards=true, .lead=0.3, .maxSpeed=127, .minSpeed=50}, false);
    chassis.moveToPose(123, 2.5, 180, 1000, {.forwards=true, .lead=0.3, .maxSpeed=127, .minSpeed=0}, false);


    //prepare to park
    tongue.set_value(true);
    heaven.set_value(false);
    top1.brake();
    top2.brake();
    odomLift.set_value(0);
    ramp.move(127);

    ::pros::delay(100);


    //Grand Finale
    leftMotors.move(95);
    rightMotors.move(95);
    pros::delay(800);
    leftMotors.brake();
    rightMotors.brake();
    ramp.brake();
    tongue.set_value(false);
    chassis.setPose(67, 67, 67);


    //============================//
    // TS BETTER WORK ON THE DAY //
    // !! GLORY  TO  THE  CCP !! //
    //===========================//
    return 0;
}


