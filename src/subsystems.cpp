#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>

#include "main.h"
#include "robot-config.hpp"
#include "subsystems.hpp"

namespace subsystems {

namespace {
bool sideState = false; // only ever touched from the opcontrol loop
std::atomic<std::uint32_t> rollerMainStopAt{0}; // millis() to stop rollerMain at; 0 = no timer
constexpr double ARRIVED_DEG = 2; // close enough to call a move done
} // namespace

void init() {
    cascade.tare_position_all();
    rollerTurn.tare_position();
    toggleA.tare_position(); // zero = where they are at startup (the go-back-to-zero moves use it)
    toggleB.tare_position();
    rollerTurn.set_current_limit(ROLLER_CURRENT_LIMIT_MA);

    // hold: these two carry load and must not sag when stopped (as in Blocks)
    cascade.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    rollerTurn.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    toggleA.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    toggleB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // stops dead on blue
    rollerMain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    sideToggle.set_value(false);
    setClaw(false); // the claw starts closed
    xMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
}

void intake(int percent) {
    rollerMainStopAt = 0;
    toggleA.move_velocity(rpm(percent));
    rollerMain.move_velocity(rpm(percent));
}

void intakeStop() {
    rollerMainStopAt = 0;
    toggleA.brake();
    rollerMain.brake();
}

void intakeFor(int percent, int ms) {
    intake(percent);
    pros::delay(ms);
    intakeStop();
}

bool rollerStaysPut() {
    constexpr double NEAR_DEG = 25;
    const double roller = std::max(rollerTurn.get_target_position(), rollerTurn.get_position());
    return cascade.get_position() > CASCADE_UP_DEG && roller >= ROLLER_DEPLOYED_DRIVER - NEAR_DEG;
}

void rollerMainFor(int percent, int ms) {
    rollerMain.move_velocity(rpm(percent));
    rollerMainStopAt = std::max<std::uint32_t>(1, pros::millis() + ms);
}

void rollerMainTimeout() {

    const std::uint32_t stopAt = rollerMainStopAt.load();
    if (stopAt != 0 && pros::millis() >= stopAt) {
        rollerMainStopAt = 0;
        rollerMain.brake();
    }
}

void stopAll() {
    intakeStop();
    cascade.brake();
    rollerTurn.brake();
}

namespace {
std::atomic<bool> blueAlliance{false}; // set from the selector's Qual tab (display task); red to start

// Ranges are a first guess: check the hue the brain's Devices screen shows for each colour and adjust.
// Red wraps round 0/360 on the hue wheel
constexpr double BLUE_HUE_MIN = 180, BLUE_HUE_MAX = 260;
constexpr double RED_HUE_BELOW = 25, RED_HUE_ABOVE = 335; // red: hue < 25 or > 335
constexpr double YELLOW_HUE_MIN = 40, YELLOW_HUE_MAX = 75;
// "Something within 5 cm". The optical sensor has no cm reading, only proximity (0-255, higher =
// closer), so 150 is a first guess at 5 cm: hold a block 5 cm away, read the [optical] line in the
// terminal, and set this to that proximity. It also keeps room light from counting as a colour
constexpr std::int32_t NEAR_PROXIMITY = 150;
constexpr double MIN_SATURATION = 0.2; // 0-1; room light and glare read washed out, near 0 (0.4 missed blue blocks, which read 0.26-0.30)

bool somethingClose() { return opticalSensor.get_proximity() >= NEAR_PROXIMITY; }

enum class Colour { None, Red, Blue, Yellow };

Colour seenColour() {
    if (!somethingClose()) return Colour::None;
    if (opticalSensor.get_saturation() < MIN_SATURATION) return Colour::None; // washed out: room light
    const double hue = opticalSensor.get_hue();
    if (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX) return Colour::Blue;
    if (hue < RED_HUE_BELOW || hue > RED_HUE_ABOVE) return Colour::Red;
    if (hue >= YELLOW_HUE_MIN && hue <= YELLOW_HUE_MAX) return Colour::Yellow;
    return Colour::None;
}

const char* name(Colour c) {
    switch (c) {
    case Colour::Red: return "red";
    case Colour::Blue: return "blue";
    case Colour::Yellow: return "yellow";
    default: return "nothing";
    }
}

// Twice a second to the terminal: what the sensor reads, to set the numbers above from
void printOptical() {
    static std::uint32_t lastPrint = 0;
    if (pros::millis() - lastPrint < 500) return;
    lastPrint = pros::millis();
    std::printf("[optical] hue %.0f  saturation %.2f  proximity %d\n", opticalSensor.get_hue(),
                opticalSensor.get_saturation(), static_cast<int>(opticalSensor.get_proximity()));
}
} // namespace

void setBlueAlliance(bool blue) { blueAlliance = blue; }
bool isBlueAlliance() { return blueAlliance; }

void startColourToggle() {
    opticalSensor.set_led_pwm(100); // its own light, so the hue doesn't depend on the room
    static pros::Task task([] {
        constexpr int SPEED_PCT = 100; // full speed (70, 60 and 55 were tried)
        constexpr int RESUME_MS = 750; // after L1/L2 is let go, wait this long before spinning again
        bool spinning = false;
        std::uint32_t lastHeld = 0;
        bool everHeld = false;
        bool homing = false; // sent back to zero for this L1/L2 hold
        while (true) {
            const std::uint32_t now = pros::millis();
            printOptical();

            // while the driver holds LEFT or RIGHT (intake), and for RESUME_MS after letting go, the toggle stays still
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) ||
                master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                lastHeld = now;
                everHeld = true;
            }
            const bool paused = everHeld && now - lastHeld < RESUME_MS;

            // LEFT/RIGHT: back to zero degrees (the nearest full turn, so it doesn't unwind every turn it
            // has spun), and hold there until it may spin again
            if (paused) {
                if (!homing) {
                    const double home = std::round(toggleB.get_position() / 360.0) * 360.0;
                    toggleB.move_absolute(home, rpm(SPEED_PCT));
                    homing = true;
                    spinning = false;
                    std::printf("[toggle] LEFT/RIGHT: back to zero\n");
                }
                pros::delay(5);
                continue;
            }
            homing = false;

            // red alliance: spins on yellow or blue. Blue alliance: spins on yellow or red.
            // Anything else (its own colour, nothing close) stops it at once
            const Colour c = seenColour();
            const Colour other = blueAlliance ? Colour::Red : Colour::Blue;
            const bool want = !paused && (c == Colour::Yellow || c == other);

            if (want != spinning) { // only command on a change
                if (want) {
                    toggleB.move_velocity(rpm(SPEED_PCT));
                    // toggleA (port 10) back to zero degrees, the nearest full turn, as LEFT/RIGHT does for toggleB.
                    // NOTE: this stops toggleA if intake() was running it (a macro or an auton)
                    toggleA.move_absolute(std::round(toggleA.get_position() / 360.0) * 360.0, rpm(SPEED_PCT));
                } else {
                    toggleB.brake(); // hold brake: stops dead
                }
                spinning = want;
                std::printf("[toggle] %s alliance, sees %s: %s\n", blueAlliance ? "blue" : "red", name(c),
                            want ? "spin" : "stop");
            }
            pros::delay(5); // check often, so a passing blue block isn't missed between checks
        }
    }, "colourToggle");
}

namespace {
std::atomic<bool> clawIsOpen{false};
} // namespace

void setClaw(bool open) {
    clawIsOpen = open;
    pistonH.set_value(open ? CLAW_OPEN : CLAW_CLOSED);
}

bool clawOpen() { return clawIsOpen; }

void toggleSide() {
    sideState = !sideState;
    sideToggle.set_value(sideState);
}

bool spinTo(pros::AbstractMotor& motor, double target, int velocity,
            const std::function<bool()>& abort) {
    const double travel = std::fabs(target - motor.get_position()); // degrees
    // degrees / (rpm * 6 deg/s per rpm) = seconds; allow 1.5x plus 500ms to settle
    const int timeoutMs =
        static_cast<int>(travel / (std::max(1, std::abs(velocity)) * 6.0) * 1500.0) + 500;

    motor.move_absolute(target, std::abs(velocity));

    for (int t = 0; t < timeoutMs; t += 10) {
        if (abort && abort()) return false;
        if (std::fabs(target - motor.get_position()) < ARRIVED_DEG) return true;
        pros::delay(10);
    }
    motor.brake(); // jammed or too slow: stop pushing rather than cook the motor
    return false;
}

namespace {
constexpr double HOMING_APPROACH = 20; // degrees above stowed where the gentle push starts
constexpr int HOMING_MV = -3000;       // push toward the stop: 3 of 12 volts, so it can't slam
constexpr int HOMING_SETTLE_MS = 150;  // let it get moving before looking for the stall
constexpr double HOMING_STILL_DEG = 1; // moved less than this...
constexpr int HOMING_STALL_MS = 200;   // ...in this long = against the stop (by position, not the noisy velocity)
constexpr int HOMING_MAX_MS = 1500;    // give up pushing after this, and zero anyway
} // namespace

bool homeRoller(const std::function<bool()>& abort) {
    if (!spinTo(rollerTurn, ROLLER_STOWED + HOMING_APPROACH, rpm(ROLLER_SPEED_PCT), abort) && abort && abort()) return false;

    rollerTurn.move_voltage(HOMING_MV);
    const double startPos = rollerTurn.get_position();
    double windowPos = startPos; // where it was at the start of the current still window
    int windowMs = 0;
    int t = 0;
    bool stalled = false;
    for (; t < HOMING_MAX_MS; t += 10) {
        if (abort && abort()) {
            rollerTurn.brake();
            return false;
        }
        pros::delay(10);
        const double pos = rollerTurn.get_position();
        if (t < HOMING_SETTLE_MS || std::fabs(pos - windowPos) >= HOMING_STILL_DEG) {
            windowPos = pos; // still moving: start the window again from here
            windowMs = 0;
        } else if ((windowMs += 10) >= HOMING_STALL_MS) {
            stalled = true;
            break;
        }
    }
    std::printf("[home] roller %s after %d ms, pushed from %.0f to %.0f: zeroed there\n",
                stalled ? "hit the stop" : "GAVE UP (never stalled)", t, startPos, rollerTurn.get_position());
    rollerTurn.tare_position(); // against the stop: this is stowed
    // hold at the new zero with a position move (not brake()), so the motor's target is in the
    // new frame too and the next move starts clean
    rollerTurn.move_absolute(ROLLER_STOWED, rpm(ROLLER_SPEED_PCT));
    return true;
}

namespace {
// spinTo() for the pickup: done once within NEAR_DEG of the target OR once the motor has stopped
// moving (pressing on the pin, or the cascade sitting on its stop a little off 0), instead of
// waiting for 2 deg and timing out. @return false only if aborted
bool spinToNear(pros::AbstractMotor& motor, double target, int velocity, const std::function<bool()>& abort) {
    constexpr double NEAR_DEG = 10;
    constexpr int SETTLE_MS = 100;        // let it get moving before looking for a stall
    constexpr double STALL_RPM = 5;       // slower than this...
    constexpr int STALL_MS = 60;          // ...for this long = it's as far as it will go
    const double travel = std::fabs(target - motor.get_position());
    const int timeoutMs = static_cast<int>(travel / (std::max(1, std::abs(velocity)) * 6.0) * 1500.0) + 500;

    motor.move_absolute(target, std::abs(velocity));
    int stillMs = 0;
    for (int t = 0; t < timeoutMs; t += 10) {
        if (abort && abort()) return false;
        if (std::fabs(target - motor.get_position()) < NEAR_DEG) return true;
        const bool still = std::fabs(motor.get_actual_velocity()) < STALL_RPM;
        stillMs = (t >= SETTLE_MS && still) ? stillMs + 10 : 0;
        if (stillMs >= STALL_MS) return true;
        pros::delay(10);
    }
    return true;
}
} // namespace

bool pickUpMotion(const std::function<bool()>& abort) {
    constexpr double ROLLER_DOWN = ROLLER_A_POSITION - 55;
    constexpr int INTAKE_MS = 50; // at the bottom, roller tilted down (was 300, then 150)

    intake(100);
    // "close or stopped", not "within 2 deg": the bottom is where it used to sit out the time limit
    if (!spinToNear(cascade, CASCADE_MIN, rpm(100), abort)) return intakeStop(), false; // full speed (was 80%)
    if (!spinToNear(rollerTurn, ROLLER_DOWN, rpm(ROLLER_SPEED_PCT), abort)) return intakeStop(), false;
    for (int t = 0; t < INTAKE_MS; t += 10) {
        if (abort && abort()) return intakeStop(), false;
        pros::delay(10);
    }
    // back up to X's position without waiting for it to get there: whatever comes next (a drive,
    // the cascade lift) starts straight away while the roller finishes
    rollerTurn.move_absolute(ROLLER_DEPLOYED_DRIVER, rpm(ROLLER_SPEED_PCT));
    return true; // intake left running on purpose
}

void yStart() {
    intake(100);
    cascade.move_absolute(CASCADE_MIN, rpm(CASCADE_DOWN_PCT));
    rollerTurn.move_absolute(ROLLER_Y, rpm(ROLLER_SPEED_PCT));
}

void yFinish(double cascadeTo, bool waitForLift, int sensorWaitMs) {
    // no waiting for the roller to reach ROLLER_Y first: the pickup sends it straight down anyway
    if (sensorWaitMs > 0) {
        // like the driver's Y: 3 close readings in a row (30 ms). 0 = no reading
        int closeTicks = 0;
        for (int t = 0; t < sensorWaitMs && closeTicks < 3; t += 10) {
            const int mm = distanceSensor.get_distance();
            closeTicks = (mm > 0 && mm <= Y_PICKUP_MM) ? closeTicks + 1 : 0;
            pros::delay(10);
        }
        if (closeTicks < 3)
            std::printf("[auton] Y pickup: nothing within %d mm in %d ms, picking up anyway\n", Y_PICKUP_MM, sensorWaitMs);
    }
    pickUpMotion(); // waits for the cascade to reach the bottom first
    if (waitForLift) spinTo(cascade, cascadeTo);
    else cascade.move_absolute(cascadeTo, rpm(100)); // doesn't wait: the auton drives on while it lifts
}

void unloadMotion(const std::function<bool()>& abort, double cascadeUp, int firstMs) {
    // up cascadeUp, but no higher than CASCADE_MAX, and never down (UP can take it past the max)
    const double cascadeAt = cascade.get_position();
    const double cascadeTo = std::max(cascadeAt, std::min(CASCADE_MAX, cascadeAt + cascadeUp));

    intake(-100);
    for (int t = 0; t < firstMs; t += 10) {
        if (abort && abort()) return intakeStop();
        pros::delay(10);
    }
    rollerTurn.move_absolute(R1_ROLLER_TO, rpm(ROLLER_SPEED_PCT)); // starts; moves while the cascade lifts
    if (cascadeUp > 0 && !spinTo(cascade, cascadeTo, rpm(100), abort) && abort && abort()) return intakeStop();
    if (!spinTo(rollerTurn, R1_ROLLER_TO, rpm(ROLLER_SPEED_PCT), abort) && abort && abort()) return intakeStop();
    intakeStop();
}

} // namespace subsystems
