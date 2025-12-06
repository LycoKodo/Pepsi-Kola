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
    while (true) {
        // // --- Read inputs once ---
        // const bool btnL2   = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);   // long goal
        // const bool btnL1   = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);   // intake (color sort)
        // const bool btnR1   = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);   // outtake
        // const bool btnR2   = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);   // middle goal
        // const bool btnDown = master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN); // endgame down
        // const bool btnUp   = master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);   // endgame up
        // const bool btnA    = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);    // aligner on
        // const bool btnB    = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);    // tongue on
        // const bool btnX    = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);    // both off

        // // --- Sensors / derived state ---
        // const bool passed = true; // colorsort.ringInspect();

        // // --- Actuators to compute ---
        // int frontSpd  = 0;
        // int middleSpd = 0;
        // int topSpd    = 0;
        // int backSpd   = 0;

        // // Default aux states
        // int  alignerVal = -1;  // -1 = leave as-is this cycle; 0/1 = force
        // int  tongueVal  = -1;
        // int  endgameVal = -1;
        // int  redirect   = 0;   // intake_redirect default (0 normal, 1 redirect)

        // // --- Endgame is independent ---
        // if (btnDown)      endgameVal = 1;
        // else if (btnUp)   endgameVal = 0;

        // // --- Aligner / Tongue are independent manual toggles ---
        // if (btnA) { alignerVal = 1; tongueVal = 0; }
        // else if (btnB) { alignerVal = 0; tongueVal = 1; }
        // else if (btnX) { alignerVal = 0; tongueVal = 0; }

        // // --- Build motor speeds by priority ---
        // // Highest priority: OUTTAKE (R1)
        // if (btnR1) {
        //     redirect   = 0;
        //     frontSpd   = -127;
        //     middleSpd  =  127;
        //     topSpd     =  127;
        //     backSpd    = -127;
        // }
        // // Next: LONG GOAL (L2)
        // else if (btnL2) {
        //     // Also force aligner/tongue as per your original L2 behavior
        //     if (passed) {
        //         redirect   = 0;
        //         frontSpd   = 127;
        //         middleSpd  = 127;
        //         topSpd     = 127;
        //         backSpd    = 127;
        //     } else {
        //         // Wrong color: redirect on; choose behavior for motors.
        //         // If you want them stopped while redirecting, keep 0s.
        //         redirect   = 1;
        //         // Optionally: set a specific eject/redirect pattern here.
        //     }
        // }
        // // Next: MIDDLE GOAL (R2)
        // else if (btnR2) {
        //     redirect   = 0;
        //     frontSpd   = 127;
        //     middleSpd  = 127;
        //     topSpd     = -127;
        //     backSpd    = 127;
        // }
        // // Next: INTAKE (L1) with color sort
        // else if (btnL1) {
        //     redirect   = 0;
        //     frontSpd   =  127;
        //     middleSpd  = -127;
        //     topSpd     = -127;
        //     backSpd    =  127;
        // }
        // // else: all speeds remain 0

        // // --- Apply outputs in one place ---
        // if (alignerVal != -1) aligner.set_value(alignerVal);
        // if (tongueVal  != -1) tongue.set_value(tongueVal);
        // if (endgameVal != -1) endgame.set_value(endgameVal);

        // intake_redirect.set_value(redirect);

        // front_intake.move(frontSpd);
        // middle_intake.move(middleSpd);
        // top_intake.move(topSpd);
        // back_intake.move(backSpd);


        //Gavin's Robot Controller Config
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            // Score Long
            ramp.move(127);
            // TODO - experimental
            top1.move(-127);
            // top1.move_velocity(-66.6666); // YES!
            top2.move(127);
            heaven.set_value(true);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            // Score Mid
            ramp.move(127);
            top1.move(-127);
            top2.move(127);
            heaven.set_value(false);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            // Intake to Ramp, Top Forward, Bottom Reverse
            ramp.move(127);
            top1.move(-127);
            top2.move(-127);
        } 
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            // 
            ramp.move(-127);
            top1.move(127);
            top2.move(-127);
        } 
        else {
            ramp.brake();
            top1.brake();
            top2.brake();
        }
        
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            tongue.set_value(1);
            heaven.set_value(false);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            tongue.set_value(0);
        }
        

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            heaven.set_value(true);
            descore.set_value(1);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            descore.set_value(0);
        }


        //Quinn's Robot Controller Config
        // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        //     intake.move(127);
        // }
        // else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        //     intake.move(-127);
        // }
        // else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        //     intake1.move(127);
        //     intake2.move(-127);
        //     intake3.move(127);
        //     intake4.move(127);
        // } 
        // // else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        // //     ramp.move(-127);
        // //     top1.move(127);
        // //     top2.move(-127);
        // // } 
        // else {
        //     intake.brake();
        // }
        // 
        // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        //     tongue.set_value(1);
        //     heaven.set_value(false);
        // }
        // else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
        //     tongue.set_value(0);
        // }
        // else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        //     heaven.set_value(1);
        // }
        // else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        //     heaven.set_value(0);
        // }

        pros::delay(10);
    }
}
