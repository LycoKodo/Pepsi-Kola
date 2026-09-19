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
    while (true) {
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

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
namespace {

// VEXcode's set_velocity(100, PERCENT) on a green cartridge
constexpr int GREEN_RPM = 200;

// X's "swing out" branch drops rollerMain to set_velocity(20, PERCENT)
constexpr int ROLLER_FEED_RPM = 40;

// one cascade level, in degrees (VEXcode: 958 * Height)
constexpr double CASCADE_STEP = 958;

// Height cycled 1, 2, 3 and reset when it hit 4
constexpr int CASCADE_LEVELS = 3;

// D-pad UP refuses to raise past this many degrees above the resting position
constexpr double CASCADE_CEILING = 2900;

// rollerTurn's travel between home and the swung-out position, in degrees
constexpr double ROLLER_FLIP = 350;

// R2 release: how long the cascade runs forward before everything stops
constexpr int EJECT_MS = 1000;

// Captured once at program start, as VEXcode's when_started did, so every
// cascade and rollerTurn target is relative to wherever the mechanism was
// resting at boot. (VEXcode: CascadeSet and CascadeUpperLimit. Its
// CascadeLowerLimit was set to 0 and never read, so it is not carried over.)
double cascadeHomePos = 0;
double rollerTurnHomePos = 0;

int cascadeLevel = 1;    // VEXcode: Height
bool intakeLifted = false; // VEXcode: IntakeUp

// VEXcode: myVariable. 0 -> the next X press sends rollerTurn home and stops
// rollerMain; 1 -> the next X press swings it out and feeds rollerMain at 20%
int rollerTurnState = 0;

// VEXcode's sticky RollerMain.set_velocity(). when_started leaves it at 100%,
// and X's swing-out branch drops it to 20% -- where it stays, so every later
// R1/L1/L2/R2 press also runs the main roller at 20% until something raises it
// again. Faithful to the original; almost certainly not what was intended
int rollerMainRpm = GREEN_RPM;

// X's swing-out branch leaves rollerMain spinning forward indefinitely
bool rollerFeeding = false;

// which hold-to-run button currently owns the cascade
enum class CascadeHold { NONE, UP, DOWN, Y };

} // namespace

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
    rollerFeeding = false;
    intakeLifted = false;

    intakeLift.set_value(false);
    toggleLift.set_value(false);
    sideToggle.set_value(false);
}

void teleop_loop() {
    CascadeHold cascadeHold = CascadeHold::NONE;
    bool r2WasHeld = false;
    bool rollersDriving = false;     // something is commanding the intake/rollers
    std::uint32_t ejectDeadline = 0; // 0 = not in R2's release sequence

    while (true) {
        // The Developer tab's PID editor and the auto-tune wizard both read the
        // D-pad. Gating the get_digital_new_press calls themselves -- not just
        // the actions -- is what keeps this loop from stealing edges out from
        // under pidtuner::handleInput()
        const bool devTab = pidtuner::isActive() || autotune::isRunning();

        // =================== cascade ===================
        // A, B and D-pad RIGHT command an absolute position; UP, DOWN and Y run
        // the cascade open-loop for as long as they are held. An absolute move
        // cancels any hold so the two cannot fight over the motor

        // --- A: step up one level, wrapping 1..3 ---
        // NOTE: drivetrain_control's TEMP debug motion fires on A+B together, so
        // that combo also runs this and the B binding below
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            cascade.move_absolute(cascadeHomePos + CASCADE_STEP * cascadeLevel, GREEN_RPM);
            if (++cascadeLevel > CASCADE_LEVELS) cascadeLevel = 1;
            cascadeHold = CascadeHold::NONE;
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
                rollerFeeding = false;
                rollerTurn.move_absolute(rollerTurnHomePos, GREEN_RPM);
                rollerTurnState = 1;
            } else {
                rollerMainRpm = ROLLER_FEED_RPM; // sticky, as in VEXcode
                rollerFeeding = true;
                rollerTurn.move_absolute(rollerTurnHomePos + ROLLER_FLIP, GREEN_RPM);
                rollerTurnState = 0;
            }
        }

        // --- D-pad LEFT: toggle the intake lift pneumatic ---
        // --- D-pad RIGHT: send rollerTurn and the cascade home together ---
        if (!devTab) {
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                intakeLifted = !intakeLifted;
                intakeLift.set_value(intakeLifted);
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                rollerTurn.move_absolute(rollerTurnHomePos, GREEN_RPM);
                cascade.move_absolute(cascadeHomePos, GREEN_RPM);
                cascadeHold = CascadeHold::NONE;
                // VEXcode did not reset Height or myVariable here, so the next A
                // press resumes from the level the cascade was on
            }
        }

        // =================== intake and rollers ===================
        // R1 and L1 were identical handlers in VEXcode; both run the intake in.
        // L2 and R2 both run it out -- R2 adds the release burp below. Holding
        // an in and an out button together resolves through the chain order
        // rather than racing, which is what VEXcode's separate threads did
        const bool intakeIn =
            master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) ||
            master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        const bool r2Held = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        const bool intakeOut = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) || r2Held;

        // R2 release: the intake stops, the cascade runs forward for a second
        // with rollerMain still reversing, then both stop
        if (r2WasHeld && !r2Held) {
            intakeTop.brake();
            intakeBottom.brake();
            rollerMain.move_velocity(-rollerMainRpm);
            cascade.move_velocity(GREEN_RPM);
            cascadeHold = CascadeHold::NONE;
            ejectDeadline = pros::millis() + EJECT_MS;
        }
        r2WasHeld = r2Held;

        if (ejectDeadline != 0) {
            if (pros::millis() >= ejectDeadline) {
                if (cascadeHold == CascadeHold::NONE) cascade.brake();
                rollerMain.brake();
                rollerFeeding = false;
                ejectDeadline = 0;
                rollersDriving = false;
            }
        } else if (intakeIn) {
            intakeTop.move_velocity(GREEN_RPM);
            intakeBottom.move_velocity(GREEN_RPM);
            rollerMain.move_velocity(rollerMainRpm);
            rollersDriving = true;
        } else if (intakeOut) {
            intakeTop.move_velocity(-GREEN_RPM);
            intakeBottom.move_velocity(-GREEN_RPM);
            rollerMain.move_velocity(-rollerMainRpm);
            rollersDriving = true;
        } else if (rollerFeeding) {
            // X left the main roller creeping forward to hold a ball against the
            // swung-out roller; the intake itself stays stopped
            intakeTop.brake();
            intakeBottom.brake();
            rollerMain.move_velocity(rollerMainRpm);
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
