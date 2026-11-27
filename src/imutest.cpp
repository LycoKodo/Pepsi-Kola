// Inertial sensor test: Tests tab button "IMU TEST" (routine -3), started like any test run (hold A, press B).
// Nothing drives. It tells the tester what to do on the controller screen, prints one [IMUTEST] line per step and
// a verdict at the end, and takes about 2.5 minutes. How-to: docs/IMU_TEST.md.
//
// Why (2026-11-24): a teammate's routine sometimes spun or drove into a wall, and the heading on the Developer
// tab was already climbing by itself before the run. That is an inertial sensor with a wrong zero for its turn
// rate. This test tells apart the three things that cause it: the robot being handled while the sensor
// calibrates (the sensor is fine), a bad cable or port (the reading jumps or drops out when the cable is moved),
// and a bad sensor or mount (it creeps after a clean calibration, or stays off after a knock).
//
// Steps:
//  1. CALIBRATE x10: calibrates ten times with the robot standing still, and measures the heading creep after
//     each. This is what happens at program start, ten times over.
//  2. SIT: 60 s untouched. How far the heading wanders.
//  3. WIGGLE: 20 s while the tester wiggles the sensor's cable at both ends. Counts dropouts and jumps.
//  4. KNOCK: 15 s while the tester taps the frame near the sensor, then 3 s to settle. Whether it comes back.
// "Standing still" is checked with the two tracking sensors. A step during which the robot rolled is marked
// MOVED and has to be done again. Any stick ends the test.
// It refuses under field control or a competition switch.

#include <cmath>
#include <cstdint>
#include <cstdio>

#include "main.h"
#include "autons.hpp"
#include "robot-config.hpp"

namespace {

constexpr int CAL_RUNS = 10;
constexpr int CAL_WATCH_MS = 1000;            // creep measured over this long after each calibration
constexpr double CAL_GOOD_DPS = 0.05;         // a good calibration creeps less than this (good ones: under 0.012)
constexpr double CAL_BAD_DPS = 0.5;           // worse than this is the fault the team saw
constexpr int SIT_MS = 60000;
constexpr double SIT_MAX_DEG = 1.0;           // heading may wander this far in SIT_MS
constexpr int WIGGLE_MS = 20000;
constexpr double JUMP_DEG = 0.5;              // a step this big between two 10 ms reads, standing still, is a jump
constexpr double WIGGLE_MAX_DEG = 1.0;
constexpr int KNOCK_MS = 15000;
constexpr int KNOCK_SETTLE_MS = 3000;
constexpr double KNOCK_MAX_DEG = 1.5;         // off by more than this after the knocks: it did not come back
constexpr std::int32_t STILL_CDEG = 100;      // tracking sensor moved under 1 deg (0.017 in): the robot stood still
constexpr std::int32_t KNOCK_STILL_CDEG = 600; // knocks shake the wheels a little: allow 0.1 in
constexpr int GET_READY_MS = 4000;            // time to read the controller before a hands-on step
constexpr int STICK_LIMIT = 15;
constexpr int SAMPLE_MS = 10;
constexpr int CONTROLLER_GAP_MS = 60;         // the controller takes one message per 50 ms
constexpr int CONTROLLER_COLS = 18;

void show(std::uint8_t row, const char* text) {
    char padded[CONTROLLER_COLS + 1];
    std::snprintf(padded, sizeof(padded), "%-*s", CONTROLLER_COLS, text);
    pros::delay(CONTROLLER_GAP_MS);
    master.set_text(row, 0, padded);
}

bool stickMoved() {
    return std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) > STICK_LIMIT ||
           std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > STICK_LIMIT ||
           std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > STICK_LIMIT ||
           std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) > STICK_LIMIT;
}

struct Watch {
    double driftDeg = 0;   // heading at the end minus at the start
    double maxStepDeg = 0; // the largest change between two reads
    int jumps = 0;         // reads that stepped more than JUMP_DEG
    int dropouts = 0;      // reads that were not a number (the sensor dropped out or restarted)
    bool moved = false;    // a tracking sensor rolled: the robot did not stand still
    bool stopped = false;  // a stick was pushed
    bool noStart = false;  // the sensor did not read at the start
};

// Watch the heading for `ms`. `countdownRow`: controller row for the seconds left, or -1 for none
Watch watch(int ms, std::int32_t stillCdeg, int countdownRow = -1) {
    Watch w;
    const double h0 = imu.get_rotation();
    const std::int32_t v0 = verticalEncoder.get_position();
    const std::int32_t x0 = horizontalEncoder.get_position();
    if (!std::isfinite(h0)) {
        w.noStart = true;
        w.dropouts = 1;
        return w;
    }
    double last = h0;
    int shownS = -1;
    for (int t = 0; t < ms; t += SAMPLE_MS) {
        pros::delay(SAMPLE_MS);
        if (stickMoved()) {
            w.stopped = true;
            return w;
        }
        const double h = imu.get_rotation();
        if (!std::isfinite(h)) {
            w.dropouts++;
        } else {
            const double step = std::fabs(h - last);
            if (step > w.maxStepDeg) w.maxStepDeg = step;
            if (step > JUMP_DEG) w.jumps++;
            last = h;
        }
        const std::int32_t v = verticalEncoder.get_position();
        const std::int32_t x = horizontalEncoder.get_position();
        if (v != PROS_ERR && x != PROS_ERR && (std::abs(v - v0) > stillCdeg || std::abs(x - x0) > stillCdeg))
            w.moved = true;
        const int leftS = (ms - t) / 1000;
        if (countdownRow >= 0 && leftS != shownS && (ms - t) % 1000 < SAMPLE_MS) {
            shownS = leftS;
            char text[CONTROLLER_COLS + 1];
            std::snprintf(text, sizeof(text), "%d s left", leftS);
            char padded[CONTROLLER_COLS + 1];
            std::snprintf(padded, sizeof(padded), "%-*s", CONTROLLER_COLS, text);
            master.set_text(static_cast<std::uint8_t>(countdownRow), 0, padded); // once a second: over the 50 ms gap
        }
    }
    w.driftDeg = last - h0;
    return w;
}

// "GET READY" with a countdown, so the tester can read what to do. false = a stick ended the test
bool getReady(const char* what) {
    show(0, what);
    show(1, "starts in 4 s");
    for (int t = 0; t < GET_READY_MS; t += SAMPLE_MS) {
        pros::delay(SAMPLE_MS);
        if (stickMoved()) return false;
    }
    return true;
}

void stoppedByStick() {
    std::printf("[IMUTEST] stopped: a stick moved. No verdict\n");
    show(0, "IMU TEST STOPPED");
    show(1, "stick moved");
    show(2, "");
}

const char* word(int verdict) { return verdict == 0 ? "PASS" : verdict == 1 ? "FAIL" : "MOVED"; }

} // namespace

void imuTest() {
    if (pros::competition::is_connected()) {
        std::printf("[IMUTEST] not run: field control or a competition switch is plugged in\n");
        show(1, "IMU TEST NOT RUN");
        show(2, "unplug comp switch");
        return;
    }
    std::printf("[IMUTEST] start. Robot on the floor, nobody touching it. Any stick stops the test\n");
    show(2, "");

    // 1. ten calibrations, standing still
    int calBad = 0, calMarginal = 0, calMoved = 0, calFailed = 0;
    double calWorst = 0;
    if (!getReady("1/4 HANDS OFF")) return stoppedByStick();
    for (int n = 1; n <= CAL_RUNS; n++) {
        char text[CONTROLLER_COLS + 1];
        std::snprintf(text, sizeof(text), "calibrating %d/%d", n, CAL_RUNS);
        show(1, text);
        // blocks about 2 s. The odometry task holds the heading while the sensor reads nothing
        const bool calOk = imu.reset(true) != PROS_ERR && std::isfinite(imu.get_rotation());
        if (stickMoved()) return stoppedByStick();
        if (!calOk) {
            calFailed++;
            std::printf("[IMUTEST] cal n=%d FAILED: the sensor did not calibrate (errno %d)\n", n, errno);
            continue;
        }
        const Watch w = watch(CAL_WATCH_MS, STILL_CDEG);
        if (w.stopped) return stoppedByStick();
        const double dps = w.driftDeg * 1000.0 / CAL_WATCH_MS;
        const char* verdict = "good";
        if (w.moved) {
            calMoved++;
            verdict = "MOVED (robot rolled: does not count)";
        } else if (w.dropouts > 0) {
            calFailed++;
            verdict = "DROPPED OUT";
        } else if (std::fabs(dps) > CAL_BAD_DPS) {
            calBad++;
            verdict = "BAD";
        } else if (std::fabs(dps) > CAL_GOOD_DPS) {
            calMarginal++;
            verdict = "marginal";
        }
        if (!w.moved && std::fabs(dps) > std::fabs(calWorst)) calWorst = dps;
        std::printf("[IMUTEST] cal n=%d creep=%+.4f deg/s dropouts=%d %s\n", n, dps, w.dropouts, verdict);
    }
    const int calCounted = CAL_RUNS - calMoved;
    const int calVerdict = calCounted < CAL_RUNS / 2 ? 2 : (calBad + calFailed + calMarginal > 0 ? 1 : 0);

    // 2. sit still
    if (!getReady("2/4 HANDS OFF 60s")) return stoppedByStick();
    show(1, "sitting still");
    const Watch sit = watch(SIT_MS, STILL_CDEG, 2);
    if (sit.stopped) return stoppedByStick();
    const int sitVerdict =
        sit.moved ? 2 : (std::fabs(sit.driftDeg) > SIT_MAX_DEG || sit.dropouts > 0 || sit.jumps > 0 ? 1 : 0);
    std::printf("[IMUTEST] sit ms=%d drift=%+.3f deg (%.4f deg/s) max_step=%.3f jumps=%d dropouts=%d %s\n", SIT_MS,
                sit.driftDeg, sit.driftDeg * 1000.0 / SIT_MS, sit.maxStepDeg, sit.jumps, sit.dropouts, word(sitVerdict));

    // 3. wiggle the cable
    master.rumble(".");
    if (!getReady("3/4 WIGGLE CABLE")) return stoppedByStick();
    show(1, "both ends, 20 s");
    const Watch wig = watch(WIGGLE_MS, STILL_CDEG, 2);
    if (wig.stopped) return stoppedByStick();
    const int wigVerdict =
        wig.moved ? 2 : (wig.dropouts > 0 || wig.jumps > 0 || std::fabs(wig.driftDeg) > WIGGLE_MAX_DEG ? 1 : 0);
    std::printf("[IMUTEST] wiggle ms=%d drift=%+.3f deg max_step=%.3f jumps=%d dropouts=%d %s\n", WIGGLE_MS,
                wig.driftDeg, wig.maxStepDeg, wig.jumps, wig.dropouts, word(wigVerdict));

    // 4. knock the frame, then let it settle
    master.rumble(".");
    if (!getReady("4/4 TAP THE FRAME")) return stoppedByStick();
    show(1, "near sensor, 15 s");
    const double beforeKnock = imu.get_rotation();
    const Watch knock = watch(KNOCK_MS, KNOCK_STILL_CDEG, 2);
    if (knock.stopped) return stoppedByStick();
    master.rumble(".");
    show(0, "4/4 HANDS OFF");
    show(1, "settling");
    const Watch settle = watch(KNOCK_SETTLE_MS, STILL_CDEG, 2);
    if (settle.stopped) return stoppedByStick();
    const double afterKnock = imu.get_rotation();
    const double knockOff = afterKnock - beforeKnock;
    const double settleDps = settle.driftDeg * 1000.0 / KNOCK_SETTLE_MS;
    const bool knockBad = !std::isfinite(knockOff) || std::fabs(knockOff) > KNOCK_MAX_DEG || knock.dropouts > 0 ||
                          settle.dropouts > 0 || std::fabs(settleDps) > CAL_BAD_DPS;
    const int knockVerdict = knock.moved || settle.moved ? 2 : (knockBad ? 1 : 0);
    std::printf("[IMUTEST] knock ms=%d off_after=%+.3f deg max_step=%.3f dropouts=%d creep_after=%+.4f deg/s %s\n",
                KNOCK_MS, knockOff, knock.maxStepDeg, knock.dropouts + settle.dropouts, settleDps, word(knockVerdict));

    // verdict: what to do about it
    const char* advice;
    const char* shortAdvice;
    if (calVerdict == 2 || sitVerdict == 2 || wigVerdict == 2 || knockVerdict == 2) {
        advice = "the robot rolled during a step marked MOVED: run the test again without moving the robot";
        shortAdvice = "ROBOT MOVED: REDO";
    } else if (calFailed > 0) {
        advice = "the sensor did not calibrate or dropped out: check its cable and port, then swap the sensor";
        shortAdvice = "CHECK CABLE/PORT";
    } else if (wigVerdict == 1) {
        advice = "the reading changed when the cable was moved: replace the cable, then try another brain port";
        shortAdvice = "CABLE OR PORT";
    } else if (calBad > 0 || calMarginal > 0 || sitVerdict == 1) {
        advice = "it creeps after a clean calibration with a good cable: try another port once, then replace the sensor";
        shortAdvice = "REPLACE SENSOR";
    } else if (knockVerdict == 1) {
        advice = "it does not come back after a knock: fix the mount (rigid, flat, away from motors), then test again";
        shortAdvice = "FIX THE MOUNT";
    } else {
        advice = "the sensor, cable and mount are fine: the fault was the robot being handled while the program "
                 "started. The boot check (src/imuwatch.cpp) covers that";
        shortAdvice = "SENSOR OK";
    }
    std::printf("[IMUTEST] RESULT cal=%s (good %d, marginal %d, bad %d, failed %d, moved %d of %d; worst %+.4f deg/s) "
                "sit=%s wiggle=%s knock=%s\n",
                word(calVerdict), calCounted - calBad - calMarginal - calFailed, calMarginal, calBad, calFailed,
                calMoved, CAL_RUNS, calWorst, word(sitVerdict), word(wigVerdict), word(knockVerdict));
    std::printf("[IMUTEST] WHAT TO DO: %s\n", advice);
    char text[CONTROLLER_COLS + 1];
    show(0, shortAdvice);
    std::snprintf(text, sizeof(text), "cal %s sit %s", word(calVerdict), word(sitVerdict));
    show(1, text);
    std::snprintf(text, sizeof(text), "wig %s knk %s", word(wigVerdict), word(knockVerdict));
    show(2, text);
    master.rumble("..");
}
