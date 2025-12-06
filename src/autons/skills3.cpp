#include "main.h"

#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"



// ------------------------- //
// Skills Autonomous Routine //
// ------------------------- //

// e.g. most points

int skills3() {

    chassis.setPose(0, 0, 0); 
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);


    chassis.moveToPoint(0, 31.5, 1500, {.forwards=true, .maxSpeed=127, .minSpeed=60, .earlyExitRange=8}, false);
    chassis.moveToPose(0, 31.5, 0, 1500, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);

    chassis.turnToHeading(90, 1200);
    // chassis.turnToHeading(90, 1200, {}, false);

    tongue.set_value(1);

    ramp.move(127);

    pros::delay(200);

    chassis.moveToPoint(11, 31.2, 1500, {.forwards=true, .maxSpeed=110}, false);
    
    pros::delay(1000);

    ramp.move(-30);

    pros::delay(200);

    ramp.move(127);

    chassis.moveToPoint(11, 31.2, 1500, {.forwards=true}, false);

    pros::delay(800);

    chassis.moveToPoint(-0.63, 33.2, 1500, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);

    chassis.turnToHeading(-62, 1000);

    tongue.set_value(0);

    chassis.moveToPoint(-22, 46.5, 1500, {.forwards=true, .maxSpeed=100, .minSpeed=45, .earlyExitRange=8}, false);

    chassis.moveToPoint(-70, 46.5, 1500, {.forwards=true, .maxSpeed=100, .minSpeed=45, .earlyExitRange=8}, false);

    chassis.moveToPoint(-86, 39, 1500,  {.forwards=true, .maxSpeed=90, .minSpeed=35, .earlyExitRange=10}, false);

    chassis.moveToPose(-93, 31, -90, 1300, {.forwards=true, .maxSpeed=120, .minSpeed=0}, false);

    chassis.turnToHeading(-90, 1000);

    heaven.set_value(1);

    chassis.moveToPoint(-77, 31, 1000, {.forwards=false, .maxSpeed=100, .minSpeed=0}, false);

    pros::delay(500);

    intake.move(127);

    pros::delay(2000);

    ramp.move(-40);

    pros::delay(500);

    ramp.move(127);

    pros::delay(2500);

    intake.move(0);

    ramp.move(127);

    chassis.moveToPoint(-79, 31, 1000, {.forwards=false, .maxSpeed=100, .minSpeed=0}, false);

    lemlib::Pose current = chassis.getPose();
    
    chassis.setPose(0, 0, current.theta);

    tongue.set_value(1);

    // move to loader on other side

    ramp.move(127);

    pros::delay(200);

    heaven.set_value(0);

    chassis.moveToPoint(-28.7, 0, 1500, {.forwards=true, .maxSpeed=100}, false);
    
    pros::delay(1000);

    ramp.move(-30);

    pros::delay(200);

    ramp.move(127);

    chassis.moveToPoint(-28.7, 0, 1500, {.forwards=true}, false);

    heaven.set_value(1);

    pros::delay(1000);

    chassis.moveToPose(8, 0, -90, 2000, {.forwards=false}, false);

    intake.move(127);

    pros::delay(2000);

    ramp.move(-40);

    pros::delay(500);

    ramp.move(127);

    pros::delay(2500);

    intake.move(0);

    ramp.move(127);

    chassis.moveToPose(-10, 0, -90, 1500, {.forwards=true}, false);

    chassis.turnToHeading(-180, 1000);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    pros::delay(20000);
    
    return 0;
}


