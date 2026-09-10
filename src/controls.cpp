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
#include "pidtuner.hpp"

void drivetrain_control() {
    bool arcadeMode = true;   // default: ARCADE
    bool latch = false;

    while (true) {
        // 1. Read toggle combo
        bool combo_pressed =
            master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
            master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) &&
            master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) &&
            master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

        // 2. On *new* press, flip mode + buzz pattern
        if (combo_pressed && !latch) {
            // flip mode
            arcadeMode = !arcadeMode;

            // rumble feedback based on NEW mode
            if (arcadeMode) {
                // switched TO ARCADE → 1 buzz
                master.rumble(".");      // short buzz
            } else {
                // switched TO TANK → 2 buzzes
                master.rumble("..");     // two short buzzes
            }

            latch = true;  // prevent retrigger while held
        } else if (!combo_pressed) {
            latch = false; // allow next press to toggle
        }

        // 3. Read joysticks
        int leftY  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // 4. Drive based on current mode
        if (arcadeMode) {
            printf("Control: Arcade\n");
            chassis.arcade(leftY, rightX);
        } else {
            printf("Control: Tank\n");
            chassis.tank(leftY, rightY);
        }

        pros::delay(10);
    }
}



// void teleop_loop() {
//     while(true) {
//         if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // long goal 
//             aligner.set_value(1);
//             tongue.set_value(0);
//             front_intake.move(127);
//             middle_intake.move(127);
//             top_intake.move(127);
//             back_intake.move(127);
//         }
//         else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // intake to bucket
//             front_intake.move(127);
//             middle_intake.move(-127);
//             top_intake.move(-127);
//             back_intake.move(127);
//         } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // middle goal
//             aligner.set_value(0);
//             front_intake.move(127);
//             middle_intake.move(127);
//             top_intake.move(-127);
//             back_intake.move(127);
//         } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { // outtake
//             front_intake.move(-127);
//             middle_intake.move(127);
//             top_intake.move(127);
//             back_intake.move(-127);
//         } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) { // Endgame down
//             endgame.set_value(1);
//         } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) { // Endgame up
//             endgame.set_value(0);
//         } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) { // Aligner
//             aligner.set_value(1);
//             tongue.set_value(0);
//         } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) { // Tongue
//             aligner.set_value(0);
//             tongue.set_value(1);
//         } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) { // Disable both
//             aligner.set_value(0);
//             tongue.set_value(0);
//         } else {
//             front_intake.brake();
//             middle_intake.brake();
//             top_intake.brake();
//             back_intake.brake();
//         }
//         pros::delay(10);
//     }
// }

void teleop_loop() {
    // STATE VARIABLE: Tracks if the lift/heaven is currently UP (true) or DOWN (false)
    // Adjust 'false' to 'true' if your robot starts the match with the lift up.
    bool is_heaven_up = false; 
    int i = 0;

    while (true) {
        
        // --- L1 Logic (Score Long / Raise) ---
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {

            descore.set_value(0); // Ensure descorer is retracted when scoring long

            if (!is_heaven_up) {
                // If robot is currently LOWERED:
                // Only move robot UP, do NOT spin intake
                is_heaven_up = true;
                heaven.set_value(true);
                tongue.set_value(0);
                // Ensure motors are stopped while lifting
                ramp.brake();
                top1.brake();
                top2.brake();

                while (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                    pros::delay(10);
                }
            } else {
                // If robot is ALREADY UP:
                // ONLY spin intake (Score Long)
                ramp.move(127);
                top1.move(-60); // Scoring direction
                top2.move(127);  // Scoring direction
            }
        }
        // --- R1 Logic (Score Mid / Lower) ---
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {

            descore.set_value(1); // Ensure descorer is retracted when scoring mid

            if (is_heaven_up) {
                // If robot is currently UP:
                // Only move robot DOWN, do NOT spin intake
                is_heaven_up = false;
                heaven.set_value(false);

                // Ensure motors are stopped while lowering
                ramp.brake();
                top1.brake();
                top2.brake();
 
                while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                    pros::delay(10);
                }
            } else {
                // If robot is ALREADY DOWN:
                // ONLY spin intake (Score Mid)
                ramp.move(127);
                top1.move(-55);
                top2.move(127);
            }
        }
        // --- L2 Logic (Intake) ---
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            // Intake to Ramp, Top Forward, Bottom Reverse
            ramp.move(127);
            top1.move(-127);
            top2.move(-127);
        } 
        // --- R2 Logic (Outtake) ---
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            ramp.move(-127);
            top1.move(127);
            top2.move(-127);
        }

        // --- D-pad ---
        // suppressed while the Developer tab is open, where the D-pad is driving
        // the PID editor instead. any other tab and these behave as normal
        else if (!pidtuner::isActive() && master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            descore.set_value(1);
        } 

        // DOWN ARROW Logic (Macro Aligner for Mid)
        else if (!pidtuner::isActive() && master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            descore.set_value(0);
        } 
        // Left for odom lift up
        else if (!pidtuner::isActive() && master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            odomLift.set_value(0);
        }
        // right for odom lift down
        else if (!pidtuner::isActive() && master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            odomLift.set_value(1);
        }
        // --- Idle State ---
        else {
            ramp.brake();
            top1.brake();
            top2.brake();
        }
        
        // --- Pneumatics (Wings/Tongue) ---
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            descore.set_value(0); // Ensure descorer is retracted when scoring long
            tongue.set_value(1);
            // Original code set heaven false here. 
            // We must update the state variable to match!
            heaven.set_value(false);
            is_heaven_up = false; 
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            tongue.set_value(0);
        }

        pros::delay(10);
    }
}

// TODO - 
// If you press L1 and robot is lowered (heaven is false), it only moves robot up without intake | else if robot is already up, it ONLY spins intake
// If you press R1 and robot is Up (heaven is false), it only moves robot DOWN (heaven false) without intake | else if robot is already down, it ONLY spins intake

// if up arrow pressed, lower robot (hevaen), spinning ramp 127, and top1 top2 at scoring direction, but at 50%