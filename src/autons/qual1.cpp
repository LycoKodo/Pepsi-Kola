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

    chassis.moveToPose(0, 30, 0, 1300, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=60}, false);

    chassis.turnToHeading(-90, 900);

    tongue.set_value(true);

    ramp.move(127);

    chassis.moveToPose(-4.3, 30, -90, 800, {.forwards=true, .lead=0, .maxSpeed=15, .minSpeed=60}, true);

    chassis.moveToPose(-3, 30, -90, 800, {.forwards=false, .lead=0, .maxSpeed=15, .minSpeed=60}, true);

    chassis.moveToPose(-4.3, 30, -90, 800, {.forwards=true, .lead=0, .maxSpeed=15, .minSpeed=60}, true);

    pros::delay(1500);

    chassis.cancelAllMotions();
    ramp.move(0);

    heaven.set_value(true);

    chassis.moveToPose(25, 32, -90, 900, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=60}, false);

    intake.move(-127);

    pros::delay(2000);

    tongue.set_value(false);
    heaven.set_value(false);

    chassis.moveToPose(15, 32, -90, 900, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=60}, false);

    chassis.turnToHeading(-180, 800);

    chassis.moveToPose(10.6, 1, -180, 900, {.forwards=true, .lead=0, .maxSpeed=127, .minSpeed=60}, false);

    chassis.turnToHeading(-270, 800);

    intake.move(0);
    
    ramp.move(127);

    chassis.moveToPose(25.0, 2.4, -310.3, 1000, {.forwards=true, .lead=0, .maxSpeed=100, .minSpeed=60}, false);

    tongue.set_value(true);

    chassis.turnToHeading(-405, 900);

    chassis.moveToPose(35.9, -2.27, -407, 1000, {.forwards=false, .lead=0, .maxSpeed=127, .minSpeed=60}, false);

    intake.move(-127);
    


    return 0;
}


