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
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    lemlib::Pose current = chassis.getPose();
    chassis.setPose(75,32,90);

    heaven.set_value(true);

    ::pros::delay(3000);

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
    chassis.moveToPose(15, 33, 0, 3000, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);
    chassis.turnToHeading(-90, 1000);
    
    //clear loader NO3
    ramp.move(127);
    top1.move(-127);
    top2.move(-127);
    chassis.moveToPoint(4, 33.7, 1200, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);

    for (int i = 0; i < 5; i++) {
        leftMotors.move(40);
        rightMotors.move(40);
        pros::delay(200);
        leftMotors.brake();
        rightMotors.brake();
        pros::delay(100);
    }
    ::pros ::delay(200);

    //move across field
    chassis.moveToPose(34, 46, -90, 3000, {.forwards=false, .maxSpeed=127, .minSpeed=50}, false);
    ::pros::delay(100);
    chassis.moveToPoint(90, 46, 2000, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);
    ::pros::delay(100);
    chassis.moveToPose(105, 33.7, 0, 3000, {.forwards=false, .maxSpeed=127, .minSpeed=50}, false);
    ::pros::delay(100);
    chassis.turnToHeading(90, 1000, {.maxSpeed=127, .minSpeed=50}, false);
    ::pros::delay(200);
    chassis.moveToPoint(88, 33.7, 1000, {.forwards=false, .maxSpeed=127, .minSpeed=50}, false);


    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    return 0;
}


