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


    // -------------------------------------------------- //
    // USED TO TEST FRAGMENTS OF SKILLS AUTON FOR SKILLS6 //
    // -------------------------------------------------- //


    //set preloaded position for location checking
    chassis.setPose(75,32,90);
    heaven.set_value(true);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    ::pros::delay(3000);
    //::pros::delay(5000);
    //set preloaded position for location checking

        //move away from long goal
    chassis.moveToPoint(90, 32, 1200, {.forwards=true, .maxSpeed=127, .minSpeed=0}, false);
    ramp.move(127);
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

    //move to long goal for scoring
    chassis.moveToPoint(90, 32, 1200, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);
    tongue.set_value(false);
    heaven.set_value(true);
    ::pros ::delay(400);

        //ram into long goal to score
    chassis.moveToPoint(72, 32, 1200, {.forwards=false, .maxSpeed=127, .minSpeed=0}, false);
        //reset pose at long goal
    chassis.setPose(75,32,90);
        //score long goal
    ramp.move(127);
    top1.move(-127);
    top2.move(127);
    ::pros::delay(3000);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}


