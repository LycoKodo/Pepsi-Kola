#include "main.h"

#include "pros/motors.h"
#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"



// ------------------------- //
// LEFT Autonomous routine.  //
// ------------------------- //

ASSET(LeftMiddle_txt);

// @NOTES:
    // - [] use motion chaining for continuous movements
    // - [] use maxspeed for long moveToPoints that are drifting
    // - [] use path.jerryio for long complex movements

int qual1() {
    chassis.setPose(0, 0, 0); 
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    // -- Approach Loader -- //

    chassis.moveToPoint(0, 32, 1200, {.forwards=true, .maxSpeed=85}, false);

    chassis.turnToHeading(-90, 900);

    tongue.set_value(true);

    ramp.move(127);

    chassis.moveToPose(-10, 32.5, -90, 800, {.forwards=true, .lead=0, .maxSpeed=80, .minSpeed=20}, true);

    pros::delay(1100);

    chassis.cancelAllMotions();
    ramp.move(0);

    // -- Approach Long Goal -- //
    heaven.set_value(true);

    chassis.moveToPose(25, 32.5, -90, 900, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=60}, false);

    intake.move(-127);

    pros::delay(2500);

    tongue.set_value(false);
    heaven.set_value(false);

    // -- Approach Mid Goal -- //

    // TODO - FOLLOWING is motion that NEEDS TUNING 

    chassis.moveToPose(12.5, 32, -90, 900, {.forwards=true, .lead=0, .maxSpeed=120, .minSpeed=30}, false);

    intake.move(0);

    chassis.turnToHeading(-212, 800);
    
    ramp.move(127);

    chassis.moveToPose(23, 0, -180, 1200, {.forwards=true, .lead=0.35, .maxSpeed=110, .minSpeed=0}, false);

    chassis.turnToHeading(-35, 900);

    chassis.moveToPose(35.5, -3.5, -48, 1200, {.forwards=false, .lead=0, .maxSpeed=110, .minSpeed=0}, false);

    intake.move(-127);

    pros::delay(2000);

    intake.move(0);

    chassis.moveToPose(12.5, 32, 0, 1200, {.forwards=true, .lead=0, .maxSpeed=120, .minSpeed=0}, false);

    chassis.turnToHeading(-90, 900);

    heaven.set_value(true);

    chassis.moveToPose(-14, 30, -90, 800, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=0}, true);

    tongue.set_value(true);

    heaven.set_value(false);

    

    // chassis.moveToPose(32, -32, -316, 4000, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=0,}, false);

    // intake.move(127);

    // chassis.moveToPose(10.6, 1, -180, 900, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=60}, false);

    // chassis.turnToHeading(-270, 800);

    // intake.move(0);
    
    // ramp.move(127);

    // chassis.moveToPose(25.0, 2.4, -310.3, 1000, {.forwards=true, .lead=0, .maxSpeed=100, .minSpeed=60}, false);

    // tongue.set_value(true);

    // chassis.turnToHeading(-405, 900);

    // chassis.moveToPose(36.9, -3.47, -407, 1000, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=60}, false);
    
    // -------------------- // 
    // TODO - @TIMMY this is a PATH.JERRYIO EXAMPLE!!!!
    // chassis.setPose(-44.261, 47.146, 135); // [!] - make sure this is your starting point in path.jerryio!!!!!
    // intake.move(-127);
    // chassis.follow(
    //     LeftMiddle_txt,
    //     15, 
    //     10000,
    //     true,
    //     false
    // );
    // -------------------- // 
    // pros::delay(1000);
    
    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    return 0;
}


