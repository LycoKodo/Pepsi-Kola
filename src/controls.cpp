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
#include "colorsort.hpp"

char alliance = 'N';
bool allianceConfirmed = false;

void drivetrain_control() {
    bool toggle = true;
    bool latch = false;

    while (true) {
        bool b_button = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);

        if (b_button && !latch) {
            // MODE - Variable Arcade
            int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            // move the chassis with curvature drive
            leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            
            chassis.variableCurvature(leftY, rightX, 1, false);                     // Regular tank
            toggle = !toggle;             // Flip the toggle state
            latch = true;                 // Engage latch to prevent repeated toggles
            
        } else if (!b_button) {
            // MODE - Tank
            int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            // move the chassis with curvature drive
            leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            
            chassis.tank(leftY, rightX);                     // Regular tank
            latch = false;                // Reset latch when button is released
        }
        
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
        pros::delay(10);
    }
}
