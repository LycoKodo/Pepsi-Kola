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

namespace {

// VEXcode's set_velocity(100, PERCENT) on a green cartridge
constexpr int GREEN_RPM = 200;

// X's "swing out" branch and R2's feed branch both drop rollerMain to
// set_velocity(40, PERCENT)
constexpr int ROLLER_FEED_RPM = 80;

// one cascade level, in degrees (VEXcode: 958 * Height)
constexpr double CASCADE_STEP = 958;

// Height cycled 1, 2, 3 and reset when it hit 4
constexpr int CASCADE_LEVELS = 3;

// D-pad UP refuses to raise past this many degrees above the resting position
constexpr double CASCADE_CEILING = 2900;

// rollerTurn's travel between home and the swung-out position, in degrees
constexpr double ROLLER_FLIP = 355;

// ondriver_drivercontrol_0 waited 90 seconds, then set_stopping(BRAKE)
constexpr std::uint32_t DRIVER_BRAKE_MS = 90000;

// R1's scoring macro (VEXcode: onevent_controller_1buttonR1_pressed_1), as
// milliseconds from the press. VEXcode ran it as a blocking 2.1s handler
// thread; teleop_loop has to stay non-blocking, so it runs as a timed state
// machine instead
constexpr std::uint32_t MACRO_CASCADE_ON_MS = 1000;  // after 1s of rollerMain reversing
constexpr std::uint32_t MACRO_TURN_ON_MS = 1100;     // +0.1s
constexpr std::uint32_t MACRO_TURN_OFF_MS = 1300;    // +0.2s
constexpr std::uint32_t MACRO_END_MS = 2100;         // +0.8s, cascade stops

// Captured once at program start, as VEXcode's when_started did, so every
// cascade and rollerTurn target is relative to wherever the mechanism was
// resting at boot. (VEXcode: CascadeSet and CascadeUpperLimit. Its
// CascadeLowerLimit was set to 0 and never read, so it is not carried over.)
double cascadeHomePos = 0;
double rollerTurnHomePos = 0;

int cascadeLevel = 1;       // VEXcode: Height
bool sideExtended = false;  // VEXcode: IntakeUp, but it drives SideToggle

// VEXcode: myVariable. 0 -> the next X press sends rollerTurn home and stops
// rollerMain; 1 -> the next X press swings it out and feeds rollerMain at 40%.
// R2 only runs its feed branch while this is 1
int rollerTurnState = 0;

// VEXcode's sticky RollerMain.set_velocity(). when_started leaves it at 100%;
// X and R2's feed branches drop it to 40%, and L1, L2 and R1's macro raise it
// back to 100% -- so whatever ran last decides how fast the next press spins it
int rollerMainRpm = GREEN_RPM;

// What rollerMain does when no hold-to-run button owns it. VEXcode left the
// motor spinning whenever a handler called spin() without a matching stop():
// X's swing-out branch creeps it forward, and R1's macro leaves it reversing
// indefinitely once the macro ends
int rollerIdleVel = 0;

// which hold-to-run button currently owns the cascade
enum class CascadeHold { NONE, UP, DOWN, Y };

} // namespace

void drivetrain_control() {
    // VEXcode's auton left the drivetrain in COAST and ondriver_drivercontrol_0
    // switched it to BRAKE 90 seconds into the driver period -- i.e. right at
    // the end of a match, so a bumped robot stops rolling
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    const std::uint32_t brakeAt = pros::millis() + DRIVER_BRAKE_MS;
    bool braked = false;

    while (true) {
        if (!braked && pros::millis() >= brakeAt) {
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
            braked = true;
        }

        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        chassis.tank(leftY, rightY);
        pros::delay(10);
    }
}

// void drivetrain_control() {
//   while (true) {
//     chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

//     int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
//     int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

//     chassis.arcade(leftY, rightX);
//     pros::delay(10);
//   }
// }

double getCascadeHome() { return cascadeHomePos; }

double getRollerTurnHome() { return rollerTurnHomePos; }

void mechanisms_init() {
    // VEXcode: RollerTurn.set_stopping(HOLD) / Cascade.set_stopping(HOLD).
    // Everything else keeps the PROS default (coast), matching VEXcode's
    // default stopping mode for the intake and main roller
    cascade.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    rollerTurn.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    cascadeHomePos = cascade.get_position();
    rollerTurnHomePos = rollerTurn.get_position();

    cascadeLevel = 1;
    rollerTurnState = 0;
    rollerMainRpm = GREEN_RPM;
    rollerIdleVel = 0;
    sideExtended = false;

    intakeLift.set_value(false);
    toggleLift.set_value(false);
    sideToggle.set_value(false);
}

void teleop_loop() {
    CascadeHold cascadeHold = CascadeHold::NONE;
    bool rollersDriving = false;   // something is commanding the intake/rollers
    std::uint32_t macroStart = 0;  // 0 = R1's macro is not running
    bool macroCascadeOn = false;
    bool macroTurnOn = false;

    while (true) {
        // The Developer tab's PID editor and the auto-tune wizard both read the
        // D-pad. Gating the get_digital_new_press calls themselves -- not just
        // the actions -- is what keeps this loop from stealing edges out from
        // under pidtuner::handleInput()
        const bool devTab = pidtuner::isActive() || autotune::isRunning();

        // =================== R1: scoring macro ===================
        // VEXcode registered two handlers on R1: pressed_0 is an empty `pass`,
        // pressed_1 is this. It owns the cascade, rollerTurn and rollerMain for
        // its 2.1 seconds -- in VEXcode another button pressed mid-macro would
        // fight it motor-for-motor; here the macro simply wins until it ends.
        // (Its trailing wait polls buttonR2, not R1, which looks like a
        // copy-paste slip in the original. Edge-triggering makes it moot.)
        if (macroStart == 0 && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            macroStart = pros::millis();
            macroCascadeOn = false;
            macroTurnOn = false;
            cascadeHold = CascadeHold::NONE;
            rollerMainRpm = GREEN_RPM; // set_velocity(100, PERCENT)
            rollerMain.move_velocity(-rollerMainRpm);
            rollersDriving = false;
        }

        if (macroStart != 0) {
            const std::uint32_t elapsed = pros::millis() - macroStart;
            if (!macroCascadeOn && elapsed >= MACRO_CASCADE_ON_MS) {
                cascade.move_velocity(GREEN_RPM);
                macroCascadeOn = true;
            }
            if (!macroTurnOn && elapsed >= MACRO_TURN_ON_MS) {
                rollerTurn.move_velocity(GREEN_RPM);
                macroTurnOn = true;
            }
            if (macroTurnOn && elapsed >= MACRO_TURN_OFF_MS) {
                rollerTurn.brake();
                macroTurnOn = false;
            }
            if (elapsed >= MACRO_END_MS) {
                cascade.brake();
                // VEXcode never stopped RollerMain here, so it keeps reversing
                // until the next binding takes it
                rollerIdleVel = -rollerMainRpm;
                macroStart = 0;
            }
            pros::delay(10);
            continue;
        }

        // =================== cascade ===================
        // B, R2 and D-pad RIGHT command an absolute position; UP, DOWN and Y run
        // the cascade open-loop for as long as they are held. An absolute move
        // cancels any hold so the two cannot fight over the motor

        // --- R2: step up one level, wrapping 1..3 ---
        // and, when rollerTurnState says the roller is home, swing it out and
        // start feeding so the ball is delivered at the new height
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            cascade.move_absolute(cascadeHomePos + CASCADE_STEP * cascadeLevel, GREEN_RPM);
            if (++cascadeLevel > CASCADE_LEVELS) cascadeLevel = 1;
            cascadeHold = CascadeHold::NONE;

            if (rollerTurnState == 1) {
                rollerMainRpm = ROLLER_FEED_RPM; // sticky, as in VEXcode
                rollerIdleVel = rollerMainRpm;
                rollerTurn.move_absolute(rollerTurnHomePos + ROLLER_FLIP, GREEN_RPM);
            }
        }

        // --- B: everything back to its resting position ---
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            cascadeLevel = 1;
            rollerTurnState = 1;
            rollerTurn.move_absolute(rollerTurnHomePos, GREEN_RPM);
            cascade.move_absolute(cascadeHomePos, GREEN_RPM);
            cascadeHold = CascadeHold::NONE;
        }

        // --- Y: run the cascade forward for as long as it is held ---
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            cascadeLevel = 1;
            cascadeHold = CascadeHold::Y;
        }

        // --- D-pad UP / DOWN: raise and lower against soft limits ---
        // VEXcode checked the limit once, when the button went down, and simply
        // did nothing for that whole press if it failed. Same here
        if (!devTab) {
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) &&
                cascade.get_position() <= cascadeHomePos + CASCADE_CEILING) {
                cascadeHold = CascadeHold::UP;
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) &&
                cascade.get_position() >= cascadeHomePos) {
                cascadeHold = CascadeHold::DOWN;
            }
        } else if (cascadeHold == CascadeHold::UP || cascadeHold == CascadeHold::DOWN) {
            // the tuner took the D-pad mid-hold; let go of the cascade
            cascade.brake();
            cascadeHold = CascadeHold::NONE;
        }

        switch (cascadeHold) {
            case CascadeHold::UP:
                if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) cascade.move_velocity(GREEN_RPM);
                else { cascade.brake(); cascadeHold = CascadeHold::NONE; }
                break;
            case CascadeHold::DOWN:
                if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) cascade.move_velocity(-GREEN_RPM);
                else { cascade.brake(); cascadeHold = CascadeHold::NONE; }
                break;
            case CascadeHold::Y:
                if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) cascade.move_velocity(GREEN_RPM);
                else { cascade.brake(); cascadeHold = CascadeHold::NONE; }
                break;
            case CascadeHold::NONE: break;
        }

        // =================== rollerTurn ===================
        // --- X: swing rollerTurn out, or bring it home ---
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            if (rollerTurnState == 0) {
                rollerMain.brake();
                rollerIdleVel = 0;
                rollerTurn.move_absolute(rollerTurnHomePos, GREEN_RPM);
                rollerTurnState = 1;
            } else {
                rollerMainRpm = ROLLER_FEED_RPM; // sticky, as in VEXcode
                rollerIdleVel = rollerMainRpm;
                rollerTurn.move_absolute(rollerTurnHomePos + ROLLER_FLIP, GREEN_RPM);
                rollerTurnState = 0;
            }
        }

        // --- D-pad LEFT: toggle the side pneumatic ---
        // --- D-pad RIGHT: send rollerTurn and the cascade home together ---
        if (!devTab) {
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                sideExtended = !sideExtended;
                sideToggle.set_value(sideExtended);
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                rollerTurn.move_absolute(rollerTurnHomePos, GREEN_RPM);
                cascade.move_absolute(cascadeHomePos, GREEN_RPM);
                cascadeHold = CascadeHold::NONE;
                // VEXcode did not reset Height or myVariable here, so the next
                // R2 press resumes from the level the cascade was on
            }
        }

        // =================== intake and rollers ===================
        // L1 runs the intake and main roller in, L2 runs them out. Both restate
        // set_velocity(100, PERCENT) on every pass, which is what resets the
        // sticky rollerMain speed after X or R2 dropped it to 40%. Holding both
        // resolves through the chain order rather than racing, which is what
        // VEXcode's separate handler threads did
        const bool intakeIn = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        const bool intakeOut = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        if (intakeIn || intakeOut) {
            rollerMainRpm = GREEN_RPM;
            const int dir = intakeIn ? 1 : -1;
            intakeTop.move_velocity(dir * GREEN_RPM);
            intakeBottom.move_velocity(dir * GREEN_RPM);
            rollerMain.move_velocity(dir * rollerMainRpm);
            rollersDriving = true;
            // both handlers stop all three motors when the button comes up
            rollerIdleVel = 0;
        } else if (rollerIdleVel != 0) {
            // X left the main roller creeping forward to hold a ball against the
            // swung-out roller, or R1's macro left it reversing; either way the
            // intake itself stays stopped
            intakeTop.brake();
            intakeBottom.brake();
            rollerMain.move_velocity(rollerIdleVel);
            rollersDriving = true;
        } else if (rollersDriving) {
            intakeTop.brake();
            intakeBottom.brake();
            rollerMain.brake();
            rollersDriving = false;
        }

        pros::delay(10);
    }
}
