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

int qual1() {
    chassis.setPose(0, 0, 0); 
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    

    // TODO - Starter Struct:
        // chassis.moveToPose(0, 24, 0, 2000, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=0 }, false);
    
    top1.move(127);
    pros::delay(1000);
    ramp.move(127);
    chassis.moveToPose(0, 20, 0, 1100, {.forwards=true, .lead=0, .maxSpeed=90, .minSpeed=0 }, false);
    top1.brake();
    pros::delay(200);  
    chassis.moveToPose(0, 26, 0, 1100, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=0 }, false);
    ramp.move(0);
    chassis.moveToPose(0, 29, 0, 800, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=0 }, false);
    chassis.turnToHeading(-118, 1000, {.maxSpeed=90, .minSpeed=0}, false);
    pros::delay(200);  
    chassis.moveToPose(11, 34, -118, 1000, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=0 }, false);
    intake.move(-127);
    


    return 0;
}


