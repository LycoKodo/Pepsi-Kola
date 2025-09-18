#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <sys/wait.h>

#include "robot-config.hpp"

void drivetrain_control() {
    bool toggle = true;
    bool latch = false;

    while (true) {
        bool b_button = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (b_button && !latch) {
            // MODE - Variable Arcade
            // move the chassis with curvature drive
            chassis.variableCurvature(leftY, rightX, 1, false);                     // Regular tank
            toggle = !toggle;             // Flip the toggle state
            latch = true;                 // Engage latch to prevent repeated toggles
            
        } else if (!b_button) {
            // MODE - Tank
            // move the chassis with curvature drive
            chassis.tank(leftY, rightX);                     // Regular tank
            latch = false;                // Reset latch when button is released
        }
        
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
        pros::delay(10);
    }
}

void score_goal() {
    while (true) {
        if (master.get_digital(DIGITAL_R2)) {
            front_intake.move(127);
            middle_intake.move(127);
            top_intake.move(127);
            back_intake.move(127);
        }
        else if (master.get_digital(DIGITAL_L2)) {
            front_intake.move(127);
            middle_intake.move(127);
            top_intake.move(-127);
            back_intake.move(127);
        } else {
            front_intake.brake();
            middle_intake.brake();
            top_intake.brake();
            back_intake.brake();
        }
        pros::delay(10);
    }
}

void intake_loop() {
    while(true) {
        if (master.get_digital(DIGITAL_R1)) {
            front_intake.move(127);
            middle_intake.move(-127);
            top_intake.move(-127);
            back_intake.move(127);
        }
        else if (master.get_digital(DIGITAL_L1)) {
            front_intake.move(-127);
            middle_intake.move(127);
            top_intake.move(127);
            back_intake.move(-127);
        } else {
            front_intake.brake();
            middle_intake.brake();
            top_intake.brake();
            back_intake.brake();
        }
        pros::delay(10);
    }
}

void endgame_ctl() {
    bool toggle = true;
    bool latch = false;

    while (true) {
        bool b_button = master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);

        if (b_button && !latch) {
            endgame.set_value(1);
            toggle = !toggle;             // Flip the toggle state
            latch = true;                 // Engage latch to prevent repeated toggles
            
        } else if (!b_button) {
            endgame.set_value(0);
            latch = false;                // Reset latch when button is released
        }
        pros::delay(10);
    }
}

void tongue_aligner_toggle() {
    bool toggle = true;
    bool latch = false;

    while (true) {
        // TODO - potential PROS bug? dual reference to a controller button does not produce warning. 
        bool b_button = master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);

        if (b_button && !latch) {
            tongue.set_value(1);
            aligner.set_value(0);
            toggle = !toggle;             // Flip the toggle state
            latch = true;                 // Engage latch to prevent repeated toggles
            
        } else if (!b_button) {
            tongue.set_value(0);
            aligner.set_value(1);
            latch = false;                // Reset latch when button is released
        }
        pros::delay(10);
    }
}
