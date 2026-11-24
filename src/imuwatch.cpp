// Watches for an inertial sensor whose heading creeps while the robot stands still.
//
// Why (2026-11-24): a teammate reported that sometimes the heading starts at 0, climbs by itself, and the robot
// then spins or drives into a wall. The day before, a UMBmark run started after the program was restarted with
// the robot being handled read -3.18 deg/s standing still, and its square came out about 45 deg crooked. A V5
// inertial sensor that calibrates while it is being moved (or takes a knock or a static shock) comes out with a
// wrong zero for its turn rate; every heading after that slides, and every move follows the slide.
//
// What this does:
//  - checkAtBoot(), right after chassis.calibrate(): if the robot is standing still and the heading creeps, the
//    sensor is calibrated once more (about 2 s). If it still creeps, it says so.
//  - start(): a slow task that keeps checking whenever the robot stands still and nothing is driving it. While
//    the heading creeps it says so on the controller every few seconds, so the driver can restart the program
//    BEFORE a match. It never touches the pose and never recalibrates by itself after boot.
//  - drifting(): the A+B test run (controls.cpp) refuses to start a routine while this is true.
// "Standing still" is judged by the two tracking wheels, which the inertial sensor cannot fool: the horizontal
// wheel sits 3.8 in from the spin point, so it rolls whenever the robot really turns.
// A good sensor reads under 0.012 deg/s standing still (2026-11-23, [UMB] still_dps); the limits are far above that.

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "main.h"
#include "autons.hpp"
#include "robot-config.hpp"

namespace imuwatch {
namespace {

constexpr double BOOT_DRIFT_DPS = 0.5;     // creep at boot above this: calibrate again
constexpr int BOOT_WINDOW_MS = 500;        // how long the boot check watches
constexpr double WATCH_DRIFT_DPS = 0.3;    // creep while standing above this, WATCH_WINDOWS times running: "drifting"
constexpr int WATCH_PERIOD_MS = 50;        // the watcher's tick
constexpr int WATCH_WINDOW_TICKS = 20;     // one window: 20 ticks = 1 s
constexpr int WATCH_WINDOWS = 2;           // windows in a row over the limit before it counts
constexpr std::int32_t STILL_CDEG = 100;   // a tracking sensor that moved under 1 deg (0.017 in): standing still
constexpr std::uint32_t SAY_EVERY_MS = 3000; // controller message and rumble while drifting

std::atomic<bool> isDrifting{false};
std::atomic<float> lastDps{0};
std::atomic<bool> paused{false}; // the inertial sensor test (src/imutest.cpp) is running: stay quiet

// heading creep over `ms`, in deg/s. ok = false if a sensor read failed or the robot moved meanwhile
double creepOver(int ms, bool& ok) {
    const double h0 = imu.get_rotation();
    const std::int32_t v0 = verticalEncoder.get_position();
    const std::int32_t x0 = horizontalEncoder.get_position();
    pros::delay(ms);
    const double h1 = imu.get_rotation();
    const std::int32_t v1 = verticalEncoder.get_position();
    const std::int32_t x1 = horizontalEncoder.get_position();
    ok = std::isfinite(h0) && std::isfinite(h1) && v0 != PROS_ERR && v1 != PROS_ERR && x0 != PROS_ERR &&
         x1 != PROS_ERR && std::abs(v1 - v0) < STILL_CDEG && std::abs(x1 - x0) < STILL_CDEG;
    return (h1 - h0) * 1000.0 / ms;
}

void watcher(void*) {
    int over = 0;
    std::uint32_t saidAt = 0;
    while (true) {
        if (chassis.isInMotion() || paused) { // a routine or a test move is driving: nothing to judge
            pros::delay(WATCH_PERIOD_MS);
            continue;
        }
        bool ok = false;
        const double dps = creepOver(WATCH_PERIOD_MS * WATCH_WINDOW_TICKS, ok);
        if (!ok || chassis.isInMotion() || paused) continue; // the robot moved, or a move started: keep the last verdict
        lastDps = static_cast<float>(dps);
        if (std::fabs(dps) > WATCH_DRIFT_DPS) {
            if (over < WATCH_WINDOWS) over++;
        } else {
            over = 0;
        }
        isDrifting = over >= WATCH_WINDOWS;
        if (isDrifting && pros::millis() - saidAt >= SAY_EVERY_MS &&
            !pros::competition::is_autonomous()) {
            saidAt = pros::millis();
            std::printf("[imu] heading creeps %.2f deg/s with the robot standing still: restart the program with "
                        "the robot completely still\n", dps);
            char line[20];
            std::snprintf(line, sizeof(line), "IMU DRIFT %+.1f/s   ", dps);
            master.set_text(0, 0, line);
            master.rumble("-");
        }
    }
}

} // namespace

void pause(bool on) { paused = on; }
bool drifting() { return isDrifting; }
float driftDps() { return lastDps; }

void checkAtBoot() {
    bool ok = false;
    double dps = creepOver(BOOT_WINDOW_MS, ok);
    if (!ok) {
        std::printf("[imu] boot check skipped: the robot was moving or a sensor did not read\n");
        return;
    }
    if (std::fabs(dps) <= BOOT_DRIFT_DPS) {
        std::printf("[imu] boot check: heading creeps %.3f deg/s standing still, fine\n", dps);
        return;
    }
    std::printf("[imu] boot check: heading creeps %.2f deg/s standing still. Calibrating again\n", dps);
    master.set_text(0, 0, "IMU RECALIBRATING ");
    imu.reset(true); // blocks until the sensor has calibrated, about 2 s. The odometry holds the heading meanwhile
    dps = creepOver(BOOT_WINDOW_MS, ok);
    if (ok && std::fabs(dps) <= BOOT_DRIFT_DPS) {
        std::printf("[imu] after calibrating again: %.3f deg/s, fine\n", dps);
        pros::delay(60); // the controller takes one message per 50 ms
        master.set_text(0, 0, "IMU OK (2nd try)  ");
        return;
    }
    isDrifting = true;
    lastDps = static_cast<float>(dps);
    std::printf("[imu] STILL creeping %.2f deg/s after calibrating again (or the robot moved). Restart the program "
                "with the robot completely still\n", dps);
    pros::delay(60);
    master.set_text(0, 0, "IMU DRIFT: RESTART");
    pros::delay(60);
    master.rumble("---");
}

void start() {
    static pros::Task task(watcher, nullptr, "imuWatch");
}

} // namespace imuwatch
