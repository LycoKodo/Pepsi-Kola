#include "autotune.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "liblvgl/lvgl.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"

#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "lemlib/util.hpp"
#include "robot-config.hpp"

namespace autotune {

namespace {

// ---------------------------------------------------------------------------
// Tuning constants
// ---------------------------------------------------------------------------

/** @brief LemLib runs every motion loop at 10ms. The gain conversion needs it */
constexpr float DT = 0.010f;

/** @brief Flips to collect before stopping. The first few are discarded */
constexpr int TARGET_FLIPS = 11;
constexpr int DISCARDED_FLIPS = 3;

/** @brief Give up if the oscillation never establishes */
constexpr int TIMEOUT_MS = 15000;

/** @brief Bail out if the robot runs away: degrees for turns, inches for drives */
constexpr float ANGULAR_RUNAWAY = 100.0f;
constexpr float LATERAL_RUNAWAY = 30.0f;

/**
 * @brief Relay deadband, in degrees or inches
 *
 * Without it, sensor noise near the setpoint flips the relay continuously and
 * the measured period is noise rather than the plant.
 */
constexpr float ANGULAR_HYSTERESIS = 1.0f;
constexpr float LATERAL_HYSTERESIS = 0.25f;

enum Axis { ANGULAR = 0, LATERAL = 1 };

enum Step { INTRO = 0, AXIS_SELECT, PLACEMENT, AMPLITUDE, ARMED, RUNNING, RESULTS, FAILED };

enum Preset { SAFE = 0, ZIEGLER_NICHOLS = 1 };

// ---------------------------------------------------------------------------
// Wizard state
// ---------------------------------------------------------------------------

lv_obj_t* overlay = nullptr;
lv_obj_t* body = nullptr;

bool wizardOpen = false;
Step step = INTRO;
Axis axis = ANGULAR;
Preset preset = SAFE;

int amplitude = 60; // relay drive, out of 127

pros::Task* relayTask = nullptr;

/** @brief Written by the relay task, read by the UI. Guarded by resultMutex */
struct Measurement {
        bool done = false;
        bool ok = false;
        float ku = 0;
        float tu = 0;
        float amplitudeMeasured = 0;
        int flips = 0;
        const char* failure = "";
};

Measurement measurement;
pros::Mutex resultMutex;

/** @brief Set by the UI to ask the relay task to stop early */
bool abortRequested = false;

// ---------------------------------------------------------------------------
// Measurement
// ---------------------------------------------------------------------------

/**
 * @brief Signed error from the setpoint the relay oscillates around
 *
 * Angular error is in degrees, matching what angularPID.update() is fed in
 * turnToHeading. Lateral error is in inches, matching moveToPoint. Keeping the
 * units identical to the real motions is what makes the resulting Ku directly
 * usable as a kP.
 */
float measureError(const lemlib::Pose& start) {
    const lemlib::Pose now = chassis.getPose();
    if (axis == ANGULAR) return lemlib::angleError(start.theta, now.theta, false);

    // project displacement onto the heading the robot started on. LemLib's theta
    // is 0 at +Y and grows clockwise, so the forward unit vector is (sin, cos)
    const float rad = lemlib::degToRad(start.theta);
    const float dx = now.x - start.x;
    const float dy = now.y - start.y;
    return -(dx * std::sin(rad) + dy * std::cos(rad));
}

/** @brief Put the relay output on the drivetrain for the chosen axis */
void driveRelay(float output) {
    if (axis == ANGULAR) {
        // same convention as turnToHeading: positive output increases theta
        leftMotors.move(output);
        rightMotors.move(-output);
    } else {
        // same convention as moveToPoint: lateral output goes to both sides
        leftMotors.move(output);
        rightMotors.move(output);
    }
}

void stopDrive() {
    leftMotors.move(0);
    rightMotors.move(0);
}

void publish(const Measurement& result) {
    resultMutex.take(TIMEOUT_MAX);
    measurement = result;
    resultMutex.give();
}

Measurement snapshot() {
    resultMutex.take(TIMEOUT_MAX);
    const Measurement copy = measurement;
    resultMutex.give();
    return copy;
}

/**
 * @brief The relay experiment itself
 *
 * Bypasses every LemLib motion deliberately. Their exit conditions and slew
 * limiting would distort the limit cycle, and the whole point is to see the
 * plant's raw response to a square wave.
 */
void relayLoop(void*) {
    Measurement result;

    const lemlib::Pose start = chassis.getPose();
    const float hysteresis = (axis == ANGULAR) ? ANGULAR_HYSTERESIS : LATERAL_HYSTERESIS;
    const float runaway = (axis == ANGULAR) ? ANGULAR_RUNAWAY : LATERAL_RUNAWAY;
    const float d = static_cast<float>(amplitude);

    // kick the relay off, otherwise it sits at zero error and never starts
    float output = d;

    float halfPeriodSum = 0;
    float peakPositiveSum = 0;
    float peakNegativeSum = 0;
    int halfPeriodCount = 0;
    int peakPositiveCount = 0;
    int peakNegativeCount = 0;

    float runningPeak = 0;
    int flips = 0;

    const uint32_t startTime = pros::millis();
    uint32_t lastFlip = startTime;

    while (true) {
        const uint32_t now = pros::millis();
        const float error = measureError(start);

        if (abortRequested) {
            result.failure = "Aborted";
            break;
        }
        if (now - startTime > static_cast<uint32_t>(TIMEOUT_MS)) {
            result.failure = (flips <= DISCARDED_FLIPS) ? "No oscillation. Raise amplitude." : "Timed out";
            break;
        }
        if (std::fabs(error) > runaway) {
            result.failure = "Runaway. Lower amplitude.";
            break;
        }

        // track the extreme reached during this half cycle
        if (std::fabs(error) > std::fabs(runningPeak)) runningPeak = error;

        const float previous = output;
        if (error > hysteresis) output = d;
        else if (error < -hysteresis) output = -d;

        if (output != previous) {
            flips++;
            // the first few half cycles are the transient, not the limit cycle
            if (flips > DISCARDED_FLIPS) {
                halfPeriodSum += static_cast<float>(now - lastFlip);
                halfPeriodCount++;
                if (runningPeak > 0) {
                    peakPositiveSum += runningPeak;
                    peakPositiveCount++;
                } else {
                    peakNegativeSum += runningPeak;
                    peakNegativeCount++;
                }
            }
            lastFlip = now;
            runningPeak = 0;

            if (flips >= TARGET_FLIPS && peakPositiveCount > 0 && peakNegativeCount > 0) {
                result.ok = true;
                break;
            }

            // publish progress so the RUNNING screen's swing counter actually
            // moves. only on a flip, which is a few times a second at most
            Measurement progress;
            progress.flips = flips;
            publish(progress);
        }

        driveRelay(output);
        pros::delay(10);
    }

    stopDrive();
    result.flips = flips;

    if (result.ok) {
        const float meanHalfPeriod = halfPeriodSum / static_cast<float>(halfPeriodCount);
        const float meanPositive = peakPositiveSum / static_cast<float>(peakPositiveCount);
        const float meanNegative = peakNegativeSum / static_cast<float>(peakNegativeCount);

        // a is the half peak-to-peak swing of the limit cycle
        const float a = (meanPositive - meanNegative) / 2.0f;
        result.tu = 2.0f * meanHalfPeriod / 1000.0f; // ms to seconds
        result.amplitudeMeasured = a;

        if (a <= 0.0001f || result.tu <= 0.0001f) {
            result.ok = false;
            result.failure = "Oscillation too small";
        } else {
            // describing function approximation of the ultimate gain
            result.ku = (4.0f * d) / (static_cast<float>(M_PI) * a);
        }
    }

    result.done = true;
    publish(result);
}

// ---------------------------------------------------------------------------
// Gain conversion
// ---------------------------------------------------------------------------

/**
 * @brief Turn (Ku, Tu) into LemLib gains
 *
 * The textbook rules are written for u = Kp*e + Ki*integral(e dt) + Kd*de/dt.
 * LemLib instead accumulates raw error and uses a raw difference:
 *
 *   u = kP*e + kI*sum(e) + kD*(e - ePrev)
 *
 * so the integral gain has to be scaled by the loop period and the derivative
 * gain divided by it, or the numbers come out wrong by a factor of 100.
 */
void computeGains(float ku, float tu, Preset which, float& kP, float& kI, float& kD) {
    // SAFE deliberately undershoots classic Ziegler-Nichols, which targets about
    // 25% overshoot. That is fine for a chemical process and awful for a robot
    // that has to stop on a line.
    const float factor = (which == SAFE) ? 0.33f : 0.60f;

    kP = factor * ku;

    const float integralTime = tu / 2.0f;
    const float derivativeTime = tu / 8.0f;

    kI = (which == SAFE) ? 0.0f : (kP / integralTime) * DT;
    kD = (kP * derivativeTime) / DT;
}

// ---------------------------------------------------------------------------
// Wizard flow
// ---------------------------------------------------------------------------

void beginRun() {
    abortRequested = false;
    publish(Measurement {});
    step = RUNNING;
    if (relayTask != nullptr) {
        delete relayTask;
        relayTask = nullptr;
    }
    relayTask = new pros::Task(relayLoop, nullptr, "pidRelay");
}

void closeWizard() {
    abortRequested = true;
    if (relayTask != nullptr) {
        // join rather than remove. relayLoop polls abortRequested every 10ms and
        // stops the motors on its way out, and once it returns PROS has already
        // freed the underlying task, so calling remove() on the handle here
        // would be operating on a dead task
        for (int i = 0; i < 40 && !snapshot().done; i++) pros::delay(10);
        delete relayTask;
        relayTask = nullptr;
    }
    stopDrive();

    if (overlay != nullptr) {
        lv_obj_del(overlay);
        overlay = nullptr;
        body = nullptr;
    }
    wizardOpen = false;
    step = INTRO;
}

void applyResult() {
    const Measurement result = snapshot();
    if (!result.ok) return;

    float kP, kI, kD;
    computeGains(result.ku, result.tu, preset, kP, kI, kD);

    lemlib::PID& pid = (axis == ANGULAR) ? chassis.angularPID : chassis.lateralPID;
    pid.setGains(kP, kI, kD);

    lemlib::ControllerSettings& settings = (axis == ANGULAR) ? angularController : linearController;
    settings.kP = kP;
    settings.kI = kI;
    settings.kD = kD;
}

} // namespace

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void start() {
    if (wizardOpen) return;
    wizardOpen = true;
    step = INTRO;
    abortRequested = false;
    publish(Measurement {});

    // lv_layer_top sits above the tabview, so the wizard covers the whole screen
    // without disturbing the selector underneath
    overlay = lv_obj_create(lv_layer_top());
    lv_obj_set_size(overlay, 480, 272);
    lv_obj_align(overlay, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_pad_all(overlay, 6, 0);
    lv_obj_set_style_radius(overlay, 0, 0);
    lv_obj_clear_flag(overlay, LV_OBJ_FLAG_SCROLLABLE);

    body = lv_label_create(overlay);
    lv_obj_set_width(body, 466);
    lv_label_set_long_mode(body, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_font(body, &lv_font_montserrat_12, 0);
    lv_obj_align(body, LV_ALIGN_TOP_LEFT, 0, 0);

    render();
}

bool isRunning() { return wizardOpen; }

void handleInput() {
    if (!wizardOpen) return;

    const bool up = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP);
    const bool left = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT);
    const bool right = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT);

    if (!up && !left && !right) return;

    switch (step) {
        case INTRO:
            if (right) step = AXIS_SELECT;
            else if (left) closeWizard();
            break;

        case AXIS_SELECT:
            if (left) axis = ANGULAR;
            else if (right) axis = LATERAL;
            else if (up) step = PLACEMENT;
            break;

        case PLACEMENT:
            if (right) step = AMPLITUDE;
            else if (left) step = AXIS_SELECT;
            break;

        case AMPLITUDE:
            if (left) amplitude -= 5;
            else if (right) amplitude += 5;
            else if (up) step = ARMED;
            if (amplitude < 20) amplitude = 20;
            if (amplitude > 110) amplitude = 110;
            break;

        case ARMED:
            if (right) beginRun();
            else if (left) step = AMPLITUDE;
            break;

        case RUNNING:
            // only an abort is accepted while the robot is moving
            if (left) abortRequested = true;
            break;

        case RESULTS:
            if (left) preset = SAFE;
            else if (right) preset = ZIEGLER_NICHOLS;
            else if (up) {
                applyResult();
                closeWizard();
            }
            break;

        case FAILED:
            if (right) step = AMPLITUDE;
            else if (left) closeWizard();
            break;
    }
}

void render() {
    if (!wizardOpen || body == nullptr) return;

    // the relay task finishing moves the wizard on by itself
    if (step == RUNNING) {
        const Measurement result = snapshot();
        if (result.done) step = result.ok ? RESULTS : FAILED;
    }

    const char* axisName = (axis == ANGULAR) ? "ANGULAR (turn)" : "LATERAL (drive)";
    char buffer[900];

    switch (step) {
        case INTRO:
            snprintf(buffer, sizeof(buffer),
                     "PID AUTO-TUNE\n\n"
                     "This measures your drivetrain instead of guessing at it.\n\n"
                     "It swaps the PID for a hard on/off relay for about ten\n"
                     "seconds. The robot WILL oscillate on purpose, back and\n"
                     "forth, and will not stop until the measurement is done.\n\n"
                     "How far and how fast it swings gives the gain at which it\n"
                     "goes unstable (Ku) and how fast it rings (Tu). Those two\n"
                     "numbers are enough to compute kP, kI and kD.\n\n"
                     "Clear the area. Keep a hand near the field switch.\n\n"
                     "RIGHT = continue        LEFT = cancel");
            break;

        case AXIS_SELECT:
            snprintf(buffer, sizeof(buffer),
                     "STEP 1 of 4 - WHICH CONTROLLER\n\n"
                     "Selected:  %s\n\n"
                     "ANGULAR spins the robot in place around its heading. Needs\n"
                     "about a 2ft square, reads the IMU, and is the one to do\n"
                     "first. Tune turns before you tune drives.\n\n"
                     "LATERAL rocks the robot forward and back along the heading\n"
                     "it currently faces. Needs roughly 6ft of clear lane and\n"
                     "leans on your odometry being trustworthy.\n\n"
                     "LEFT = angular   RIGHT = lateral   UP = confirm",
                     axisName);
            break;

        case PLACEMENT:
            if (axis == ANGULAR) {
                snprintf(buffer, sizeof(buffer),
                         "STEP 2 of 4 - PLACE THE ROBOT\n\n"
                         "Mode: %s\n\n"
                         "Put the robot on the field with at least 2ft clear on\n"
                         "every side. It will spin back and forth in place.\n\n"
                         "Point it somewhere you can eyeball, so you can see\n"
                         "whether it is swinging evenly.\n\n"
                         "Battery matters. These gains scale with voltage, so\n"
                         "tune on a charged battery or you retune every session.\n\n"
                         "RIGHT = continue        LEFT = back",
                         axisName);
            } else {
                snprintf(buffer, sizeof(buffer),
                         "STEP 2 of 4 - PLACE THE ROBOT\n\n"
                         "Mode: %s\n\n"
                         "Give it a clear lane, about 6ft, along the direction it\n"
                         "is currently facing. It will drive forward and back.\n\n"
                         "Do the angular tune first if you have not. A robot that\n"
                         "cannot hold a heading will not give a clean drive\n"
                         "oscillation either.\n\n"
                         "Battery matters. Tune on a charged battery.\n\n"
                         "RIGHT = continue        LEFT = back",
                         axisName);
            }
            break;

        case AMPLITUDE:
            snprintf(buffer, sizeof(buffer),
                     "STEP 3 of 4 - RELAY STRENGTH\n\n"
                     "Amplitude:  %d / 127\n\n"
                     "How hard the relay drives the motors each time it flips.\n"
                     "It barely changes the answer, it just has to produce a\n"
                     "clean swing.\n\n"
                     "Too low and static friction wins: the robot judders and\n"
                     "the reading is garbage. Too high and it swings violently.\n\n"
                     "60 is a sane start. Raise it if nothing moves.\n\n"
                     "LEFT/RIGHT = adjust by 5        UP = confirm",
                     amplitude);
            break;

        case ARMED:
            snprintf(buffer, sizeof(buffer),
                     "STEP 4 of 4 - READY TO RUN\n\n"
                     "Mode:       %s\n"
                     "Amplitude:  %d / 127\n\n"
                     "The robot is about to start moving by itself and will keep\n"
                     "moving for up to 15 seconds.\n\n"
                     "It stops on its own once it has counted enough swings, or\n"
                     "if it drifts too far, or if you abort.\n\n"
                     "Hands clear.\n\n"
                     "RIGHT = START        LEFT = back",
                     axisName, amplitude);
            break;

        case RUNNING: {
            const Measurement result = snapshot();
            snprintf(buffer, sizeof(buffer),
                     "RUNNING - %s\n\n"
                     "Swings counted:  %d of %d\n\n"
                     "Let it settle into an even back-and-forth rhythm. The\n"
                     "first few swings are thrown away as transient.\n\n"
                     "If it is barely moving, abort and raise the amplitude.\n"
                     "If it is thrashing, abort and lower it.\n\n"
                     "LEFT = ABORT",
                     axisName, result.flips, TARGET_FLIPS);
            break;
        }

        case RESULTS: {
            const Measurement result = snapshot();

            float kP, kI, kD;
            computeGains(result.ku, result.tu, preset, kP, kI, kD);

            lemlib::PID& pid = (axis == ANGULAR) ? chassis.angularPID : chassis.lateralPID;

            snprintf(buffer, sizeof(buffer),
                     "RESULT - %s\n\n"
                     "Ku %.2f    Tu %.3f s    swing %.2f\n\n"
                     "Preset: %s\n"
                     "  proposed  kP %.2f   kI %.4f   kD %.2f\n"
                     "  current   kP %.2f   kI %.4f   kD %.2f\n\n"
                     "SAFE is a third of the unstable gain with no integral. It\n"
                     "lands soft, which is what a drivetrain wants. ZN is the\n"
                     "textbook rule and deliberately overshoots about 25%%.\n\n"
                     "Applying only changes RAM. Copy it into robot-config.cpp.\n\n"
                     "LEFT = SAFE   RIGHT = ZN   UP = apply and close",
                     axisName, result.ku, result.tu, result.amplitudeMeasured,
                     preset == SAFE ? "SAFE" : "ZIEGLER-NICHOLS", kP, kI, kD, pid.getKP(), pid.getKI(), pid.getKD());
            break;
        }

        case FAILED: {
            const Measurement result = snapshot();
            snprintf(buffer, sizeof(buffer),
                     "TUNE FAILED\n\n"
                     "Reason:  %s\n"
                     "Swings counted:  %d\n\n"
                     "No oscillation usually means the relay is too weak to\n"
                     "break static friction. Raise the amplitude.\n\n"
                     "Runaway means it never came back to the setpoint. Lower\n"
                     "the amplitude, and on lateral check your odometry.\n\n"
                     "Nothing was changed.\n\n"
                     "RIGHT = try again        LEFT = close",
                     result.failure, result.flips);
            break;
        }
    }

    // render() runs every 10ms off the tab watcher. rebuilding a 900 character
    // label that often is pointless churn for the LVGL daemon, and most ticks
    // produce byte-identical text
    static char previous[sizeof(buffer)] = {0};
    if (std::strcmp(previous, buffer) != 0) {
        std::snprintf(previous, sizeof(previous), "%s", buffer);
        lv_label_set_text(body, buffer);
    }
}

} // namespace autotune
