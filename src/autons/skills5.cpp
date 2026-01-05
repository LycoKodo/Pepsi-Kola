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

int skills5() {
    lemlib::Pose current = chassis.getPose();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);


    tongue.set_value(true);
    odomLift.set_value(1);

    chassis.setPose(0,0,0);

    ::pros::delay(100);

    //turn to loader NO3
    chassis.moveToPose(15, 33, 0, 3000, {.forwards=true, .maxSpeed=127, .minSpeed=50}, true);
        //outtake while moving with delay to prevent jamming at loader
    ::pros::delay(1000);
    ramp.move(127);
    top1.move(-127);
    top2.move(127);
    chassis.waitUntilDone();

        //turn to loader NO3
    chassis.turnToHeading(-90, 1000);
    ::pros::delay(200);
    
    //clear loader NO3
    ramp.move(127);
    top1.move(-127);
    top2.move(-127);
    chassis.moveToPose(3, 32 , -90, 1200, {.forwards=true, .maxSpeed=85, .minSpeed=75}, false);

    for (int i = 0; i < 5; i++) {
        leftMotors.move(40);
        rightMotors.move(40);
        pros::delay(200);
        leftMotors.brake();
        rightMotors.brake();
        pros::delay(100);
    }
    ::pros ::delay(200);
    tongue.set_value(false);


    //move across field
        //into the channel
    chassis.moveToPose(34, 46, -90, 2500, {.forwards=false, .maxSpeed=127, .minSpeed=50}, false);
    ::pros::delay(160);
        //across the channel
    chassis.moveToPose(90, 46, -90, 2000, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);
    ::pros::delay(160);
        //approach long in two movements to adjust pos
    chassis.moveToPose(105, 32, 0, 2000, {.forwards=false, .maxSpeed=127, .minSpeed=60}, false);
    chassis.moveToPoint(105, 32, 1000, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);
        //turn to long
    chassis.turnToHeading(90, 1000, {.maxSpeed=127, .minSpeed=50}, false);
    ::pros::delay(100);
    heaven.set_value(true);
    ::pros ::delay(240);

    //move to long and score
    chassis.moveToPoint(88, 32, 1000, {.forwards=false, .maxSpeed=127, .minSpeed=50}, false);
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
    chassis.moveToPoint(100, 32, 1000, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);
    heaven.set_value(false);
    tongue.set_value(true);
    ramp.move(127);
    top1.move(-127);
    top2.move(-127);
        //reach loader NO4
    chassis.moveToPoint(116, 32, 1300, {.forwards=true, .maxSpeed=70, .minSpeed=0}, false);

    for (int i = 0; i < 5; i++) {
        leftMotors.move(40);
        rightMotors.move(40);
        pros::delay(200);
        leftMotors.brake();
        rightMotors.brake();
        pros::delay(100);
    }
    ::pros ::delay(200);

    //move to long and score
    chassis.moveToPoint(100, 32, 600, {.forwards=false, .maxSpeed=127, .minSpeed=50}, false);
        //enter scoring position
    heaven.set_value(true);
    tongue.set_value(false);
    ::pros::delay(240);
        //reach long and score
    chassis.moveToPoint(85, 32, 600, {.forwards=false, .maxSpeed=127, .minSpeed=50}, false);
    ramp.move(127);
    top1.move(-127);
    top2.move(127);
        //reset position while scoring
    ::pros::delay(1325);
    ///--================--///
    ///    CHECK POINT    ///
    ///--===============--///
    chassis.setPose(88, 32, 90);
    ::pros::delay(1325);


    //move to park
    chassis.moveToPose(123, 2, 180, 3000, {.forwards=true, .lead=0.3, .maxSpeed=127, .minSpeed=50}, false);
    chassis.moveToPose(123, 2, 180, 1000, {.forwards=true, .lead=0.3, .maxSpeed=127, .minSpeed=0}, false);


    //prepare to park
    heaven.set_value(false);
    tongue.set_value(true);
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


