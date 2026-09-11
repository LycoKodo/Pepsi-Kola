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
#include "autotune.hpp"

void drivetrain_control() {
    bool arcadeMode = true;   // default: ARCADE
    bool latch = false;
    bool testLatch = false;   // TEMP: A+B test motion

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

        // TEMP test motion: A + B drives to (0, 24) and back. runs in this
        // task, not teleop_loop, so the arcade/tank writes below are blocked for
        // the whole motion instead of fighting it. off the D-pad on purpose, so
        // it works on the Developer tab without nudging a gain. still yields to
        // the auto-tuner, whose relay drives the motors itself
        bool test_pressed =
            master.get_digital(pros::E_CONTROLLER_DIGITAL_A) &&
            master.get_digital(pros::E_CONTROLLER_DIGITAL_B);

        if (test_pressed && !testLatch && !autotune::isRunning()) {
            testLatch = true;  // one run per press, even if still held after
            chassis.moveToPoint(0, 24, 2000, {}, false);
            chassis.moveToPoint(0, 0, 2000, {.forwards=false}, false);
        } else if (!test_pressed) {
            testLatch = false;
        }

        // 3. Read joysticks
        int leftY  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // 4. Drive based on current mode, unless the auto-tuner's relay has the
        // drivetrain. chassis.arcade writes the motors on every call, even 0 at
        // centered sticks, so both tasks would write the drive every 10ms and
        // the relay's effective amplitude -- and the gains computed from it --
        // would come out wrong
        if (autotune::isRelayActive()) {
            // hands off
        } else if (arcadeMode) {
            chassis.arcade(leftY, rightX);
        } else {
            chassis.tank(leftY, rightY);
        }

        pros::delay(10);
    }
}



void teleop_loop() {
    // new robot: intakeTop, intakeBottom, cascade, rollerTurn and rollerMain are
    // configured but not bound to buttons yet. put the bindings in this loop.
    //
    // A/B/X/Y and L1/L2/R1/R2 are free, but holding all four shoulder buttons
    // together is drivetrain_control's arcade/tank toggle, so a binding there
    // fires alongside it. The D-pad is free too, but the Developer tab's PID
    // editor reads it: guard any D-pad binding with
    //   !pidtuner::isActive() && !autotune::isRunning()
    // so it cannot fire while someone is editing gains
    while (true) {
        pros::delay(10);
    }
}
