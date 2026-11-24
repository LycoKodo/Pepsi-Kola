#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"

#include "lemlib/selector.hpp"

#include "autons.hpp"
#include "controls.hpp"
#include "macros.hpp"
#include "pidtuner.hpp"
#include "robot-config.hpp"
#include "subsystems.hpp"

using namespace subsystems;

namespace {

// When the drive switches from coast to brake, as in the Blocks project's
// "when driver control: wait 90s, set drive stopping to brake" (driver period
// is 105s, so this is the last 15s, for parking/holding position)
constexpr int BRAKE_AFTER_MS = 90000;

// Edge detection kept here rather than get_digital_new_press(): PROS tracks
// "new press" per controller, so any second reader of the buttons would steal
// presses from this loop
struct Button {
    pros::controller_digital_e_t id;
    bool now = false;
    bool before = false;

    void update() {
        before = now;
        now = master.get_digital(id);
    }
    bool pressed() const { return now && !before; }
    bool released() const { return !now && before; }
};

Button L1{pros::E_CONTROLLER_DIGITAL_L1}, L2{pros::E_CONTROLLER_DIGITAL_L2};
Button R1{pros::E_CONTROLLER_DIGITAL_R1}, R2{pros::E_CONTROLLER_DIGITAL_R2};
Button A{pros::E_CONTROLLER_DIGITAL_A}, B{pros::E_CONTROLLER_DIGITAL_B};
Button X{pros::E_CONTROLLER_DIGITAL_X}, Y{pros::E_CONTROLLER_DIGITAL_Y};
Button UP{pros::E_CONTROLLER_DIGITAL_UP}, DOWN{pros::E_CONTROLLER_DIGITAL_DOWN};
Button LEFT{pros::E_CONTROLLER_DIGITAL_LEFT}, RIGHT{pros::E_CONTROLLER_DIGITAL_RIGHT};

Button* const ALL[] = {&L1, &L2, &R1, &R2, &A, &B, &X, &Y, &UP, &DOWN, &LEFT, &RIGHT};

// Anti-tip: the higher the lift, the lower the drive's top speed. Full speed up to
// ANTI_TIP_FROM_DEG (R2's level 3), then evenly down to ANTI_TIP_MIN_SCALE at ANTI_TIP_TO_DEG
// (level 5) and above. It scales the sticks, so steering and slow driving feel the same
constexpr double ANTI_TIP_FROM_DEG = CASCADE_ALLIANCE[2]; // 900: level 3
constexpr double ANTI_TIP_TO_DEG = CASCADE_ALLIANCE[4];   // 1200: level 5
constexpr double ANTI_TIP_MIN_SCALE = 0.5;                // top speed with the lift that high or higher

double antiTipScale() {
    const double lift = cascade.get_position();
    if (!std::isfinite(lift) || lift <= ANTI_TIP_FROM_DEG) return 1.0;
    const double t = std::min(1.0, (lift - ANTI_TIP_FROM_DEG) / (ANTI_TIP_TO_DEG - ANTI_TIP_FROM_DEG));
    return 1.0 - t * (1.0 - ANTI_TIP_MIN_SCALE);
}

// tank drive: left stick = left side, right stick = right side (Blocks used split arcade)
void drive() {
    const int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // the PID tab's test move drives the motors itself; chassis.tank writes
    // them on every call, even 0 at centred sticks, so it must stay hands off
    if (pidtuner::isRunning()) return;

    // anti-tip: lower top speed with the lift up
    const double scale = antiTipScale();
    chassis.tank(static_cast<int>(leftY * scale), static_cast<int>(rightY * scale));
}

void intakeControls(bool dpadFree) {
    // D-pad RIGHT intakes, LEFT outtakes (moved from L1/L2). Hold to run, stop on release. Writes
    // nothing while they are untouched, so it does not stomp a macro that is using rollerMain
    if (dpadFree && RIGHT.now) {
        intake(100);
    } else if (dpadFree && LEFT.now) {
        intake(-100);
    } else if (RIGHT.released() || LEFT.released()) {
        intakeStop();
    }
}

// The claw. Hold L1: the claw opens and motors 14 and 16 spin in. Once the claw's sensor sees
// something within CLAW_CATCH_MM (10 cm), the claw closes and the motors keep intaking for
// CLAW_INTAKE_AFTER_MS more, then stop (even with L1 still held, or let go meanwhile). Letting go of
// L1 before it catches anything closes it and stops them straight away
// CLAW_CATCH_MM, CLAW_IGNORE_MS and CLAW_INTAKE_AFTER_MS are in subsystems.hpp (autons grab the same way)
enum class Claw { Idle, Grabbing, Caught } claw = Claw::Idle;
std::uint32_t clawOpenedAt = 0;
std::uint32_t xMotorsStopAt = 0; // millis() to stop motors 14 and 16 at; 0 = no timer


void clawClose() {
    setClaw(false);
    xMotors.brake();
}

void clawControls() {
    if (L1.pressed()) {
        xMotorsStopAt = 0; // a new grab cancels a timer left from the last one
        setClaw(true);
        xMotors.move_velocity(rpm(100));
        claw = Claw::Grabbing;
        clawOpenedAt = pros::millis();
        std::printf("[claw] open (sensor reads %d mm now, closes at %d mm)\n",
                    static_cast<int>(clawSensor.get_distance()), static_cast<int>(CLAW_CATCH_MM));
    }
    if (claw == Claw::Grabbing && pros::millis() - clawOpenedAt >= CLAW_IGNORE_MS &&
        clawSensor.get_distance() > 0 && clawSensor.get_distance() <= CLAW_CATCH_MM) {
        std::printf("[claw] caught (%d mm): close, intake 2 s more\n", static_cast<int>(clawSensor.get_distance()));
        setClaw(false); // caught something: close, but keep intaking a moment
        xMotorsStopAt = pros::millis() + CLAW_INTAKE_AFTER_MS;
        claw = Claw::Caught;
    }
    if (xMotorsStopAt != 0 && pros::millis() >= xMotorsStopAt) {
        xMotors.brake();
        xMotorsStopAt = 0;
    }
    if (L1.released()) {
        if (claw == Claw::Grabbing) clawClose(); // let go before it caught anything
        claw = Claw::Idle;
    }

}

// Y sends the roller to its position through here. When it cancels a macro that is
// moving the roller (a stow mid-push), that macro stops the motor on its way out, which would
// wipe out a move sent the same tick; so the move waits until the macro has let go (a few ms).
// A move still waiting after 300 ms is dropped rather than fight whatever started since
constexpr std::uint32_t ROLLER_PENDING_MS = 300;
double pendingRoller = 0;
std::uint32_t pendingRollerAt = 0; // 0 = nothing waiting

void sendRoller(double target) {
    pendingRoller = target;
    pendingRollerAt = std::max<std::uint32_t>(1, pros::millis());
}

void applyPendingRoller() {
    if (pendingRollerAt == 0) return;
    if (pros::millis() - pendingRollerAt > ROLLER_PENDING_MS) {
        std::printf("[roller] move to %.0f dropped: a macro was still running\n", pendingRoller);
        pendingRollerAt = 0;
    } else if (!macros::isBusy()) {
        rollerTurn.move_absolute(pendingRoller, rpm(ROLLER_SPEED_PCT));
        pendingRollerAt = 0;
    }
}

// "Already at Y": where the roller was last sent OR where it physically is (within
// ROLLER_AT_DEG), so it still counts after something else last moved it close by, e.g.
// R1's L4 score leaving it at 600
constexpr double ROLLER_AT_DEG = 25;

// Y and LEFT: a press sends the roller to the button's position and starts intaking; holding
// on, once the roller is there, watches the distance sensor, and something within
// the button's distance (Y_PICKUP_MM / LEFT_PICKUP_MM) runs PickUp (which keeps intaking), once per hold. Letting go first stops the intake
struct PickupHold {
    const char* name;
    double rollerTo;
    int pickupMm;
    macros::Macro pickup; // what runs when the sensor fires
    bool watching = false;
    int closeTicks = 0;
    std::uint32_t reportAt = 0; // 0 = nothing to report

    // now: move the roller straight away (B's reset is running and won't touch it until it ends,
    // which would make the usual wait-for-macros move time out)
    void start(bool now = false) {
        // debugging "doesn't go all the way up": where the roller starts, and (in update) where it got to
        std::printf("[%s] press: roller at %.0f, going to %.0f\n", name, rollerTurn.get_position(), rollerTo);
        reportAt = std::max<std::uint32_t>(1, pros::millis() + 1500);
        if (now) rollerTurn.move_absolute(rollerTo, rpm(ROLLER_SPEED_PCT));
        else sendRoller(rollerTo);
        intake(100);
        watching = true;
        closeTicks = 0;
    }

    void update(bool held) {
        if (reportAt != 0 && pros::millis() >= reportAt) {
            reportAt = 0;
            std::printf("[%s] 1.5 s later: roller at %.0f (target %.0f), %d mA of %d, %.0f C\n", name,
                        rollerTurn.get_position(), rollerTurn.get_target_position(), rollerTurn.get_current_draw(),
                        ROLLER_CURRENT_LIMIT_MA, rollerTurn.get_temperature());
        }
        if (watching && !held) {
            watching = false;
            intakeStop(); // let go before anything came close
        }
        // only once the roller has got there: before that, a sensor reading would start PickUp,
        // which would swallow the roller move (it waits for macros to finish)
        if (!watching || std::fabs(rollerTurn.get_position() - rollerTo) >= ROLLER_AT_DEG) return;
        // 3 readings in a row (30 ms), so one noisy reading can't fire it. 0 = no reading
        const int mm = distanceSensor.get_distance();
        closeTicks = (mm > 0 && mm <= pickupMm) ? closeTicks + 1 : 0;
        if (closeTicks >= 3) {
            std::printf("[%s] distance %d mm: PickUp\n", name, mm); // once per hold
            watching = false;
            macros::request(pickup, true);
        }
    }
};
PickupHold yHold{"Y", ROLLER_Y, Y_PICKUP_MM, macros::Macro::PickUpY}; // ends at alliance L1, then L2
PickupHold leftHold{"LEFT", ROLLER_LEFT, LEFT_PICKUP_MM, macros::Macro::PickUp};

void cascadeControls(bool dpadFree) {
    const bool up = dpadFree && UP.now;
    const bool down = dpadFree && DOWN.now;
    const bool grabbed = dpadFree && (UP.pressed() || DOWN.pressed());
    const bool letGo = dpadFree && (UP.released() || DOWN.released());

    // the driver grabbing the cascade always beats a macro
    if (grabbed) macros::cancel();


    // UP also does X's deploy: roller out to its deployed position, rollerMain in at 40%.
    // With the cascade up and the roller already out (at X or higher), the roller holds where it is
    if (dpadFree && UP.pressed()) {
        rollerMainFor(40, ROLLER_SPIN_MS); // stops by itself after 2 s
        if (!rollerStaysPut()) rollerTurn.move_absolute(ROLLER_DEPLOYED_DRIVER, rpm(ROLLER_SPEED_PCT));
        macros::setXDeploysNext(false); // keeps X in step: its next press stows
    }

    if (up) {
        cascade.move_velocity(rpm(100)); // no top limit: the driver stops it
    } else if (down) {
        cascade.move_velocity(-rpm(CASCADE_DOWN_PCT)); // no bottom limit either: the driver stops it
    } else if (letGo) {
        cascade.brake();
    }
}

void macroControls(bool dpadFree) {
    // R1: lift down onto the pin/cup, then open the claw (the Unload macro is off the buttons now)
    if (R1.pressed()) macros::request(macros::Macro::PlaceRelease);
    // R2 cuts in on a running R2 (so quick presses step straight up), but not on other macros
    if (R2.pressed()) {
        macros::request(macros::Macro::CycleHeight, macros::isRunning(macros::Macro::CycleHeight));
    }
    // L2 held: motors 14 and 16 intake (spin in, the way L1 grabs), and the press closes the claw.
    // X held: they outtake (spin out), and the press opens the claw.
    // (L2 was the claw toggle; Y, the old roller toggle / pickup, and X's roller toggle are off the buttons)
    if (L2.pressed()) setClaw(false);
    if (X.pressed()) setClaw(true);
    if (L2.now) xMotors.move_velocity(rpm(100));
    else if (X.now) xMotors.move_velocity(-rpm(100));
    else if (L2.released() || X.released()) xMotors.brake();
    // resets win over whatever is running: they are the "get me out of this" buttons
    if (B.pressed() && !A.now) macros::request(macros::Macro::ResetAll, true); // A+B is the test run
}

// A held 1 s swaps R2's cascade heights (alliance goal <-> neutral goal) and rumbles once.
// A tap does nothing (it used to toggle the roller to ROLLER_A_DRIVER). A+B is the test run
bool aUsed = false; // this A press was part of A+B, or a 1 s hold
std::uint32_t aDownAt = 0;
constexpr std::uint32_t A_HOLD_MS = 1000;

void rollerAControls() {
    if (A.pressed()) {
        aUsed = false;
        aDownAt = pros::millis();
    }
    if (A.now && B.now) aUsed = true; // A+B, even when the test run is blocked by field control

    // held 1 s (and not A+B): swap R2's heights between alliance and neutral goal, one rumble
    if (A.now && !aUsed && pros::millis() - aDownAt >= A_HOLD_MS) {
        aUsed = true;
        macros::toggleNeutralHeights();
        master.rumble(".");
    }
}

// CALIBRATION MARKS (Y, added 2026-11-23 for the tape pushes and hand turns): tap Y to print the pose and the
// raw sensors to the terminal (captured by `pros terminal --output FILE`) and show it on controller line 2;
// hold Y for CAL_ZERO_HOLD_MS to zero the pose and start a new set. Raw values are counted from the last zero,
// so wheel size and inertial scale can be worked out without trusting the config. Not under field control or
// a competition switch. Y was free (its old roller toggle was taken off the buttons)
constexpr std::uint32_t CAL_ZERO_HOLD_MS = 1000;
std::uint32_t yDownAt = 0;
bool yHoldDone = false;   // this press already zeroed
int calSet = 0, calMark = 0;
int calZeroTicks = 0;     // setPose again on the next tick, in case the odometry task wrote over the first
double calVert0 = 0, calHoriz0 = 0, calImu0 = 0;

void calPrintMark() {
    const lemlib::Pose p = chassis.getPose();
    const double vert = verticalEncoder.get_position() / 100.0 - calVert0;   // centidegrees -> degrees
    const double horiz = horizontalEncoder.get_position() / 100.0 - calHoriz0;
    const double rot = imu.get_rotation() - calImu0;
    ++calMark;
    std::printf("[MARK] set=%d mark=%d x=%.3f y=%.3f theta=%.3f imu_deg=%.3f vert_deg=%.2f horiz_deg=%.2f\n",
                calSet, calMark, p.x, p.y, p.theta, rot, vert, horiz);
    char line[20];
    std::snprintf(line, sizeof(line), "%d.%d %.2f %.2f   ", calSet, calMark, p.x, p.y);
    master.set_text(2, 0, line);
}

void calZero() {
    chassis.setPose(0, 0, 0);
    calVert0 = verticalEncoder.get_position() / 100.0;
    calHoriz0 = horizontalEncoder.get_position() / 100.0;
    calImu0 = imu.get_rotation();
    ++calSet;
    calMark = 0;
    calZeroTicks = 1;
    std::printf("[MARK] set=%d zeroed\n", calSet);
    master.set_text(2, 0, "ZERO              ");
    master.rumble(".");
}

void calControls() {
    if (calZeroTicks > 0 && --calZeroTicks == 0) chassis.setPose(0, 0, 0);
    if (pros::competition::is_connected()) return;
    if (Y.pressed()) {
        yDownAt = pros::millis();
        yHoldDone = false;
    }
    if (Y.now && !yHoldDone && pros::millis() - yDownAt >= CAL_ZERO_HOLD_MS) {
        yHoldDone = true;
        calZero();
    }
    if (Y.released() && !yHoldDone) calPrintMark();
}

} // namespace

void driverControl() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    bool braking = false;
    const std::uint32_t start = pros::millis();

    // seed twice so now == before: a button already held when driver control
    // starts must not count as a fresh press
    for (Button* b : ALL) b->update();
    for (Button* b : ALL) b->update();

    while (true) {
        for (Button* b : ALL) b->update();

        // true while nothing else claims the D-pad. Nothing does at the moment (the PID tab is
        // edited on the touchscreen), but the D-pad handlers keep the parameter so one can again
        // Hold A: LEFT / RIGHT step through the routines on the controller (src/ctrlselect.cpp), and the D-pad
        // does nothing else meanwhile
        // Shelved (ctrlselect::ENABLED in autons.hpp is false): the D-pad is then always free, as before
        if (ctrlselect::ENABLED && A.now && (LEFT.pressed() || RIGHT.pressed())) {
            ctrlselect::step(RIGHT.pressed() ? 1 : -1);
            aUsed = true; // holding A on must not swap the heights
        }
        if (ctrlselect::ENABLED) ctrlselect::pump();
        const bool dpadFree = !(ctrlselect::ENABLED && A.now);

        // TEST RUN: hold A, press B -> run the auton picked on the brain.
        // Never with field control or a competition switch attached: there,
        // autonomous() is how autons run. Blocks this loop until the routine ends
        if (A.now && B.pressed() && !pros::competition::is_connected()) {
            const int routine = lemlib::selector::auton;
            char line[20];
            if (routine == 999) {
                // nothing picked on the brain: say so instead of silently doing nothing
                master.set_text(0, 0, "NO ROUTINE PICKED ");
                master.rumble("-");
            } else if (imuwatch::drifting() && routine != -3) { // -3 is the inertial sensor test itself
                // the heading is sliding by itself: the routine would spin or drive off (src/imuwatch.cpp)
                std::printf("[test] routine %d not run: the heading creeps %.2f deg/s standing still. Restart the "
                            "program with the robot completely still\n", routine, imuwatch::driftDps());
                master.set_text(0, 0, "IMU DRIFT: RESTART");
                master.rumble("---");
            } else {
                macros::cancel();
                std::printf("[test] running routine %d\n", routine);
                std::snprintf(line, sizeof(line), "RUN #%-13d", routine);
                master.set_text(0, 0, line);

                const std::uint32_t t0 = pros::millis();
                runAuton(routine);
                const std::uint32_t ms = pros::millis() - t0;
                // skills routines (101..200) get the 60s skills period, the rest a match's 15s
                const bool skills = routine >= 101 && routine <= 200;
                // a test run (negative number) has no time limit
                const std::uint32_t limitMs = routine < 0 ? UINT32_MAX : skills ? 60000 : 15000;

                // stop the cascade and roller, but leave the intake as the routine left it
                // (an auton that ends intaking keeps intaking; LEFT/RIGHT stop it)
                cascade.brake();
                rollerTurn.brake();
                std::printf("[test] routine %d done in %.1fs%s\n", routine, ms / 1000.0,
                            ms <= limitMs ? "" : skills ? "  (OVER the 60s skills period)" : "  (OVER the 15s autonomous period)");
                pros::delay(60); // the controller drops updates under 50ms apart
                std::snprintf(line, sizeof(line), "#%d %.1fs%-10s", routine, ms / 1000.0, ms <= limitMs ? "" : skills ? " >60s!" : " >15s!");
                master.set_text(0, 0, line);
            }
            chassis.setBrakeMode(braking ? pros::E_MOTOR_BRAKE_BRAKE : pros::E_MOTOR_BRAKE_COAST);
            for (Button* b : ALL) b->update(); // presses during the run are stale
            for (Button* b : ALL) b->update();
            aUsed = true; // holding A on after a test run must not swap the heights
            continue;
        }

        drive();
        intakeControls(dpadFree);
        clawControls();
        cascadeControls(dpadFree);
        macroControls(dpadFree);
        rollerAControls();
        calControls();
        applyPendingRoller(); // after Y, so an unblocked move goes out the same tick
        rollerMainTimeout(); // ends the X / UP deploy spin after ROLLER_SPIN_MS

        if (!braking && pros::millis() - start >= BRAKE_AFTER_MS) {
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
            braking = true;
        }

        pros::delay(10);
    }
}
