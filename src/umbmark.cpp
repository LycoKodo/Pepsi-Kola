#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "main.h"
#include "autons.hpp"
#include "pidtuner.hpp"
#include "ports.hpp"
#include "robot-config.hpp"
#include "subsystems.hpp"

// UMBmark square: an odometry test (after the University of Michigan's UMBmark, adapted to this robot).
// The robot drives a square under its own odometry and comes back to its start. It BELIEVES it is back at the
// start; a person measures with a ruler where it really is. Real minus believed is the odometry error.
// Five runs clockwise ("UMB CW", routine -1), five counter-clockwise ("UMB CCW", routine -2); both on the Tests tab.
// How to run it and read the result: docs/UMBMARK.md.
//
// Frame: setPose(0, 0, 0) at the start. x = to the robot's right at the start, y = forward at the start, heading
// in degrees, clockwise positive (LemLib's way). The robot's centre (LemLib's tracking centre) drives the square:
// 48 in forward, then 48 in to the RIGHT (clockwise) or to the LEFT (counter-clockwise), and back.
//
// It runs only as a test run (hold A, press B, no competition switch or field control plugged in). Pushing a
// stick stops it at once, at any point. So does the inertial sensor stopping, a move running out of time, or a
// PID tab test starting. A stopped run prints one [umbmark] line and has no result.
//
// At the very end of a finished run it prints ONE line (the laptop tool parses it by key, in any order):
//   [UMB] v=1 run= dir=CW|CCW side= drive_speed= turn_speed= x= y= theta= still_dps= scale_cw= scale_ccw= t_ms=
//         timeouts=
// x, y in inches and theta in degrees are the believed pose, read once after the settle wait. theta is as LemLib
// has it, not wrapped: about +360 after a clockwise square, -360 after a counter-clockwise one. still_dps is how
// fast the heading crept (deg/s, clockwise +) while the robot stood still before the first move; it creeps for
// t_ms + 3300 ms in all before the pose is read (the 2 s still, the square, the last stop and the settle). scale_cw and
// scale_ccw are the IMU scale this program was built with (robot-config.hpp). t_ms is from the start of the first
// move to the end of the last turn. timeouts is always 0 now: a move that runs out of time ends the run (until
// 2026-11-22 the run carried on and counted it here). It stays in the line so the format is still v=1.
// The same pose, rounded to 2 decimals, goes on the controller screen, for when no laptop is connected.

namespace {

// ---------------------------------------
// The test's numbers (the simulator's routes and the laptop calculator assume the same ones)
// ---------------------------------------
constexpr float UMB_SIDE_IN = 48; // side of the square, inches, for the robot's centre (two field tiles)
// Top speeds, out of 127. Slow on purpose: the calibration wants as little wheel slip as it can get. Raise both
// to 127 to measure the slip at match speed. 90 deg turns are under BIG_TURN_DEG (100), so TeamChassis does not cap
// them, even at 127
constexpr int UMB_DRIVE_MAX_SPEED = 70;
constexpr int UMB_TURN_MAX_SPEED = 70;
constexpr int UMB_STOP_MS = 300;    // standing still after every move, so the next one starts from rest
constexpr int UMB_STILL_MS = 2000;  // standing still before the first move; the heading change over it gives still_dps
constexpr int UMB_SETTLE_MS = 1000; // after the last turn (and its stop), before the pose is read
// Time limits. Generous: they are there for a robot that is stuck, not to cut a slow move short. In the simulator
// (not yet timed on the robot) a 48 in leg at speed 70 took about 1.9 s and a 90 deg turn about 0.95 s: the
// square 13.7 s, the whole run 17.1 s. A move that runs out of time ends the run (see the moves below)
constexpr int UMB_DRIVE_TIMEOUT_MS = 5000;
constexpr int UMB_TURN_TIMEOUT_MS = 3000;
// The drive brakes during the run, as the PID tab's test moves do: driver control's COAST would let a move roll on
// past its target where a routine would not. The mode it had is put back at the end
constexpr pros::motor_brake_mode_e_t UMB_BRAKE_MODE = pros::E_MOTOR_BRAKE_BRAKE;
constexpr float UMB_MAX_STILL_DPS = 0.05f; // heading creep while standing above this: the run is refused
constexpr int UMB_STICK_LIMIT = 15; // a stick pushed further than this (of 127) stops the run (the PID tab's limit)
// How often the sticks, the inertial sensor and the PID tab are checked while the robot moves or stands. Every
// move runs in LemLib's own task and this one watches it (see the moves below)
constexpr int UMB_WATCH_MS = 10;
// A run that ends early keeps the drive braking this long before the brake mode is put back. In the simulator a
// stick pushed in the middle of a leg had the drive at rest about 0.4 s later, 4.5 in further on: after 300 ms it
// was still rolling, and driver control's COAST let it roll on
constexpr int UMB_HALT_MS = 1000;
// After a stick stops the run, the drive keeps braking until every stick is let go, for up to this long. Without
// it, driver control took over at once and the robot drove off the way the stick was still pushed
constexpr int UMB_RELEASE_WAIT_MS = 3000;
// The lift must be low. Driver control lowers the drive's top speed once the lift is above level 3 (controls.cpp's
// anti-tip); this test drives at 70 and brakes hard at the end of every move, so it does not start above that
constexpr double UMB_LIFT_MAX_DEG = subsystems::CASCADE_ALLIANCE[2]; // 900: where anti-tip starts
// LemLib's odometry task can overwrite a setPose that lands in the middle of its update: it writes back the
// heading it worked out from the old pose. So the routine sets the pose twice, this far apart (two updates)
constexpr int UMB_SETPOSE_GAP_MS = 20;
// The controller drops screen messages that come less than 50 ms apart; controls.cpp waits 60 ms between them
constexpr int CONTROLLER_GAP_MS = 60;
constexpr int CONTROLLER_COLS = 18; // the width controls.cpp pads its lines to, so old text is wiped

// The corners, in sides of the square, and the heading to turn to at each, in the order they are driven. Every
// leg is driven forwards with moveToPoint and every turn is on the spot with TeamChassis::turnToHeading. LemLib
// turns the short way, so each turn is 90 deg the same way round (360 deg in all)
struct Corner {
    float x, y, heading;
};
constexpr Corner CW_SQUARE[4] = {{0, 1, 90}, {1, 1, 180}, {1, 0, 270}, {0, 0, 0}};
constexpr Corner CCW_SQUARE[4] = {{0, 1, 270}, {-1, 1, 180}, {-1, 0, 90}, {0, 0, 0}};
constexpr int MOVES = 8; // 4 legs and 4 turns

// Why a run ended before its result. Each gives one [umbmark] line and two controller lines
enum class Stop { None, Stick, ImuLost, PidTab, TimedOut };

// LemLib has no brake mode getter: read it off the drive motors, as the PID tab does (setBrakeMode sets both
// sides). If the motor can't be read (unplugged), go back to COAST, driver control's mode
pros::motor_brake_mode_e_t driveBrakeMode() {
    const pros::MotorBrake current = leftMotors.get_brake_mode();
    return current == pros::MotorBrake::invalid ? pros::E_MOTOR_BRAKE_COAST
                                                : static_cast<pros::motor_brake_mode_e_t>(current);
}

bool sticksMoved() {
    return std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) > UMB_STICK_LIMIT ||
           std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > UMB_STICK_LIMIT ||
           std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > UMB_STICK_LIMIT ||
           std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) > UMB_STICK_LIMIT;
}

// A dead or unplugged inertial sensor reads infinity. LemLib then holds the heading still, so every turn would
// spin until it ran out of time and every leg would steer blind
bool imuReading() { return std::isfinite(imu.get_rotation()); }

// What ends the run early, checked every UMB_WATCH_MS. A PID tab test (RUN on the brain) would queue its own
// move behind this run's and drive in the middle of the square, so it ends the run too
Stop checkStops() {
    if (sticksMoved()) return Stop::Stick;
    if (!imuReading()) return Stop::ImuLost;
    if (pidtuner::isRunning()) return Stop::PidTab;
    return Stop::None;
}

// Stands still for ms, checking every UMB_WATCH_MS. Returns why the run must end, or Stop::None
Stop watchedWait(int ms) {
    const std::uint32_t start = pros::millis();
    std::uint32_t wake = start;
    while (pros::millis() - start < static_cast<std::uint32_t>(ms)) {
        const Stop s = checkStops();
        if (s != Stop::None) return s;
        pros::Task::delay_until(&wake, UMB_WATCH_MS);
    }
    return checkStops();
}

// One move. startMove starts it with async = true: it runs in LemLib's own task and this one checks every
// UMB_WATCH_MS until it ends. Anything that ends the run cancels the move at once; the move's own code then stops
// the motors, which brake (UMB_BRAKE_MODE). Until 2026-11-22 the moves ran with async = false, in this task, so
// the sticks could only be read between moves: in the simulator a stick held for a whole second in the middle of a
// leg did nothing, and with the inertial sensor dead part way the robot spun 3000-4900 deg and drove up to 75 in
// from its start. Running the move in its own task also means PROS can never delete this task with LemLib's
// motion lock held (a test run has no mode change; if it did, the move would finish and hand the lock back)
template <typename StartMove> Stop watchedMove(StartMove startMove, int timeoutMs) {
    const std::uint32_t start = pros::millis();
    startMove(); // returns once LemLib's task has started the move (it waits 10 ms for that)
    std::uint32_t wake = pros::millis();
    while (chassis.isInMotion()) {
        const Stop s = checkStops();
        if (s != Stop::None) {
            chassis.cancelAllMotions();
            return s;
        }
        pros::Task::delay_until(&wake, UMB_WATCH_MS);
    }
    // LemLib's timer ends a move at elapsed >= its limit. This task notices up to UMB_WATCH_MS later, so a move
    // that ends within that much of its limit also counts: either way the robot was stuck or lost
    if (pros::millis() - start >= static_cast<std::uint32_t>(timeoutMs)) return Stop::TimedOut;
    return checkStops();
}

// One controller line, padded with spaces so the last message's text is wiped. Waits first, so it never comes
// less than CONTROLLER_GAP_MS after the message before it (the driver loop writes line 0 just before a test run,
// and again 60 ms after it). Nothing else in the program writes lines 1 and 2, so they stay until the next run
void showOnController(std::uint8_t row, const char* text) {
    char padded[CONTROLLER_COLS + 1];
    std::snprintf(padded, sizeof(padded), "%-*s", CONTROLLER_COLS, text);
    pros::delay(CONTROLLER_GAP_MS);
    master.set_text(row, 0, padded);
}

// Says why the routine did not start: one [umbmark] line and two controller lines. Nothing has moved
void refuse(const char* dir, const char* why, const char* shortWhy) {
    std::printf("[umbmark] %s not run: %s\n", dir, why);
    showOnController(1, "UMB NOT RUN");
    showOnController(2, shortWhy);
}

// Ends a run early, with no result (the robot is not back at the start). The drive brakes to a stop first; after a
// stick it keeps braking until the sticks are let go. Then the brake mode the drive had is put back and one line
// is printed: the only print of a stopped run
void endEarly(const char* dir, int run, int movesDone, Stop why, pros::motor_brake_mode_e_t savedBrakeMode) {
    chassis.cancelAllMotions(); // nothing queued or running, whatever ended the run
    pros::delay(UMB_HALT_MS);   // still braking (UMB_BRAKE_MODE), so the robot comes to rest
    if (why == Stop::Stick) {
        const std::uint32_t start = pros::millis();
        while (sticksMoved() && pros::millis() - start < static_cast<std::uint32_t>(UMB_RELEASE_WAIT_MS))
            pros::delay(UMB_WATCH_MS);
    }
    chassis.setBrakeMode(savedBrakeMode);

    char reason[48];
    const char* shortWhy = "";
    switch (why) {
        case Stop::Stick:
            std::snprintf(reason, sizeof(reason), "a stick moved");
            shortWhy = "stick moved";
            break;
        case Stop::ImuLost:
            std::snprintf(reason, sizeof(reason), "the inertial sensor stopped reading");
            shortWhy = "IMU lost: restart";
            break;
        case Stop::PidTab:
            std::snprintf(reason, sizeof(reason), "the PID tab started a test move");
            shortWhy = "PID tab test";
            break;
        case Stop::TimedOut:
        case Stop::None: // not used: there is always a reason
            std::snprintf(reason, sizeof(reason), "move %d ran out of time", movesDone + 1);
            shortWhy = "timed out: redo";
            break;
    }
    std::printf("[umbmark] %s run %d stopped after %d of %d moves: %s. No result\n", dir, run, movesDone, MOVES,
                reason);
    char text[CONTROLLER_COLS + 1];
    std::snprintf(text, sizeof(text), "UMB #%d STOPPED", run);
    showOnController(1, text);
    showOnController(2, shortWhy);
}

} // namespace

void umbmarkSquare(bool clockwise) {
    static int runNumber = 0; // counts up from 1 since boot; a refused run does not use a number
    const char* dir = clockwise ? "CW" : "CCW";
    char text[CONTROLLER_COLS + 1];

    // PROS hazard (docs/HANDOFF.md): a task deleted in the middle of a printf leaves the serial lock held, and
    // every later printf then waits for ever. So this routine prints exactly once per run, at the end, with the
    // robot stopped (or, if it refuses or is stopped, one line saying so instead). Nothing else in here prints.
    // The only task it starts is LemLib's own, one per move. No LVGL calls are made.

    // Refuse before anything moves. Under field control or a competition switch the routine would run as an
    // autonomous: a match autonomous (15 s) ends in the middle of the square, and a skills run with this still
    // picked would drive it. It is a test, so it runs only from driver control (hold A, press B)
    if (pros::competition::is_connected()) {
        refuse(dir, "a competition switch or field control is plugged in. Unplug it and use hold A, press B",
               "unplug comp switch");
        return;
    }
    // If the inertial sensor failed to calibrate at boot, LemLib took its heading from the wheels from then on,
    // even if the sensor reads again now
    if (!imuCalibratedAtBoot()) {
        refuse(dir, "the inertial sensor did not calibrate at boot. Restart with the robot still", "IMU bad: restart");
        return;
    }
    if (!imuReading()) {
        refuse(dir, "the inertial sensor is not reading. Check its cable, restart with the robot still",
               "IMU not reading");
        return;
    }
    if (pidtuner::isRunning() || chassis.isInMotion()) {
        refuse(dir, "another move is running (a PID tab test?). Wait for it to end", "wait: robot moving");
        return;
    }
    const double lift = cascade.get_position();
    if (std::isfinite(lift) && lift > UMB_LIFT_MAX_DEG) { // unreadable: allowed, as driver control's anti-tip does
        refuse(dir, "the lift is up. Lower it fully first", "lower the lift");
        return;
    }

    const int run = ++runNumber;
    const pros::motor_brake_mode_e_t savedBrakeMode = driveBrakeMode();
    chassis.setBrakeMode(UMB_BRAKE_MODE);

    // GPS fusion stays Off: this measures plain odometry. Never call startGpsFusion() in here
    chassis.setPose(0, 0, 0);
    pros::delay(UMB_SETPOSE_GAP_MS);
    chassis.setPose(0, 0, 0);

    // Stand still. The heading change over this time is the inertial sensor's drift. The controller messages go
    // out first (they wipe the last run's numbers, so nobody copies those by mistake); the wait that follows makes
    // the still time up to UMB_STILL_MS
    const std::uint32_t stillStart = pros::millis();
    const float stillTheta0 = chassis.getPose().theta;
    std::snprintf(text, sizeof(text), "UMB %s #%d going", dir, run);
    showOnController(1, text);
    showOnController(2, "");
    const int stillLeftMs = UMB_STILL_MS - static_cast<int>(pros::millis() - stillStart);
    Stop stop = watchedWait(stillLeftMs > 0 ? stillLeftMs : 0);
    const std::uint32_t stillMs = pros::millis() - stillStart;
    const float stillDps = (chassis.getPose().theta - stillTheta0) * 1000.0f / static_cast<float>(stillMs);
    if (stop != Stop::None) {
        endEarly(dir, run, 0, stop, savedBrakeMode);
        return;
    }
    // The heading must hold still while the robot stands. On 2026-11-23 a run started after the program was
    // restarted with the robot being moved read -3.18 deg/s here: the inertial sensor had calibrated badly, and
    // the square came out turned about 45 deg and nearly reached a wall. Good runs read under 0.012 deg/s
    if (std::fabs(stillDps) > UMB_MAX_STILL_DPS) {
        chassis.setBrakeMode(savedBrakeMode);
        std::printf("[umbmark] %s not run: the heading crept %.3f deg/s standing still (limit %.2f). Restart the "
                    "program with the robot completely still, and keep hands off it for the first 2 s\n",
                    dir, stillDps, UMB_MAX_STILL_DPS);
        showOnController(1, "UMB NOT RUN");
        showOnController(2, "IMU drift: restart");
        return;
    }

    // The square: a leg then a turn at each corner, UMB_STOP_MS standing after every move
    const Corner* square = clockwise ? CW_SQUARE : CCW_SQUARE;
    int movesDone = 0;
    // The believed pose at rest after each move, printed after the [UMB] line as [UMBLEG] lines (added
    // 2026-11-23). A drive's line shows how far from its corner the move ended; the change across a turn shows
    // how far the tracking centre swung, which is where the robot's real spin point is relative to it
    struct LegPose { float x, y, theta; } legPose[MOVES] = {};
    std::uint32_t squareMs = 0;
    const std::uint32_t squareStart = pros::millis();
    for (int k = 0; k < 4; k++) {
        const float x = square[k].x * UMB_SIDE_IN;
        const float y = square[k].y * UMB_SIDE_IN;
        stop = watchedMove(
            [x, y] {
                chassis.moveToPoint(x, y, UMB_DRIVE_TIMEOUT_MS, {.forwards = true, .maxSpeed = UMB_DRIVE_MAX_SPEED},
                                    true);
            },
            UMB_DRIVE_TIMEOUT_MS);
        if (stop == Stop::None) stop = watchedWait(UMB_STOP_MS);
        if (stop != Stop::None) {
            endEarly(dir, run, movesDone, stop, savedBrakeMode);
            return;
        }
        { const lemlib::Pose p = chassis.getPose(); legPose[movesDone] = {p.x, p.y, p.theta}; }
        movesDone++;

        const float heading = square[k].heading;
        stop = watchedMove(
            [heading] { chassis.turnToHeading(heading, UMB_TURN_TIMEOUT_MS, {.maxSpeed = UMB_TURN_MAX_SPEED}, true); },
            UMB_TURN_TIMEOUT_MS);
        if (stop == Stop::None) {
            if (movesDone + 1 == MOVES) squareMs = pros::millis() - squareStart;
            // after the last turn the square is done and nothing will move again: the stop after it is not watched
            if (movesDone + 1 < MOVES) stop = watchedWait(UMB_STOP_MS);
            else pros::delay(UMB_STOP_MS);
        }
        if (stop != Stop::None) {
            endEarly(dir, run, movesDone, stop, savedBrakeMode);
            return;
        }
        { const lemlib::Pose p = chassis.getPose(); legPose[movesDone] = {p.x, p.y, p.theta}; }
        movesDone++;
    }

    pros::delay(UMB_SETTLE_MS);
    const lemlib::Pose pose = chassis.getPose(); // read ONCE: the line and the controller show the same pose
    chassis.setBrakeMode(savedBrakeMode);

    // the one print of the run (see the hazard above); the robot has stopped. timeouts is always 0 (see the top)
    const int timeouts = 0;
    std::printf("[UMB] v=1 run=%d dir=%s side=%.1f drive_speed=%d turn_speed=%d x=%.3f y=%.3f theta=%.3f "
                "still_dps=%.4f scale_cw=%.6f scale_ccw=%.6f t_ms=%lu timeouts=%d\n",
                run, dir, UMB_SIDE_IN, UMB_DRIVE_MAX_SPEED, UMB_TURN_MAX_SPEED, pose.x, pose.y, pose.theta, stillDps,
                IMU_SCALE_CLOCKWISE, IMU_SCALE_COUNTER_CLOCKWISE, static_cast<unsigned long>(squareMs), timeouts);

    for (int m = 0; m < MOVES; m++) {
        const Corner& c = square[m / 2];
        std::printf("[UMBLEG] run=%d dir=%s move=%d kind=%s to_x=%.1f to_y=%.1f to_theta=%.0f x=%.3f y=%.3f theta=%.3f\n",
                    run, dir, m + 1, m % 2 == 0 ? "drive" : "turn", c.x * UMB_SIDE_IN, c.y * UMB_SIDE_IN, c.heading,
                    legPose[m].x, legPose[m].y, legPose[m].theta);
    }

    // For copying onto paper: line 1 "X-0.42 Y+0.31", line 2 "H+359.62 CW #3" (H = theta, as in the line above).
    // In a test run the driver loop then writes the routine number and time on line 0
    std::snprintf(text, sizeof(text), "X%+.2f Y%+.2f", pose.x, pose.y);
    showOnController(1, text);
    std::snprintf(text, sizeof(text), "H%+.2f %s #%d", pose.theta, dir, run);
    showOnController(2, text);
}
