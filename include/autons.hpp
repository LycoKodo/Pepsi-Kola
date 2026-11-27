#ifndef AUTONS_HPP
#define AUTONS_HPP

// Autonomous routines (src/autons.cpp); the Skills tab has the odometry test
// below (src/umbmark.cpp). Adding one: write it there, declare it here, add a
// button + description to its selector tab in initialize(), and a case in
// runAuton() (src/main.cpp)

/** @brief Qual routine 2 (selector: Qual tab, "james_67") */
void james_67();

/** @brief Qual routine 3 (selector: Qual tab, "67james"; a C++ name cannot start with a digit) */
void james67b();

/**
 * @brief Run routine number `routine` (the selector's numbering). Blocking.
 * autonomous() calls it in a match; driverControl() calls it for a test run
 */
void runAuton(int routine);

/**
 * @brief UMBmark square, the odometry test (src/umbmark.cpp, docs/UMBMARK.md). Blocking, about 17 s in the
 * simulator (not yet timed on the robot).
 * From wherever the robot stands: stands still 2 s, then drives a 48 in square under odometry and comes back to
 * its start, then prints one [UMB] line with where it believes it is and shows that on the controller. A person
 * measures where it really is. Clockwise goes 48 in forward and 48 in to the right; counter-clockwise to the left.
 * Selector: Tests tab, "UMB CW" (routine -1) and "UMB CCW" (-2). Never switches GPS fusion on.
 * Test runs only (hold A, press B): it refuses under field control or a competition switch, with the lift up, with
 * the inertial sensor failed, or while another move runs. A stick pushed stops it at once, as does the inertial
 * sensor stopping, a move running out of time or a PID tab test starting: one [umbmark] line, no result
 */
void umbmarkSquare(bool clockwise);

/**
 * @brief true if the inertial sensor read a heading right after chassis.calibrate() at boot (src/main.cpp). If
 * it did not, LemLib takes its heading from the wheels until the program restarts, even if the sensor comes back
 */
/**
 * Inertial sensor test (src/imutest.cpp, docs/IMU_TEST.md): Tests tab "IMU TEST" (routine -3), a test run only.
 * Nothing drives. Calibrates ten times, sits 60 s, then asks for the cable to be wiggled and the frame tapped,
 * and prints [IMUTEST] lines with a verdict: sensor fine, cable or port, mount, or replace the sensor.
 */
void imuTest();

bool imuCalibratedAtBoot();

/**
 * Inertial sensor drift watch (src/imuwatch.cpp). A sensor that calibrated badly makes the heading creep while
 * the robot stands still, and every move then follows the sliding heading.
 * checkAtBoot(): call once in initialize(), after chassis.calibrate(); calibrates once more if the heading creeps.
 * start(): starts the task that keeps checking while the robot stands still and warns on the controller.
 * drifting(): true while the heading creeps standing still; driftDps(): the last rate measured, deg/s.
 */
namespace imuwatch {
void checkAtBoot();
void start();
bool drifting();
float driftDps();
void pause(bool on); // true while the inertial sensor test runs: no checks, no messages
} // namespace imuwatch

/**
 * @brief Switch GPS fusion to GPS_FUSION_MODE (robot-config.hpp; Off by default, and then this does nothing).
 * The only way a routine switches fusion on. A routine that wants fusion calls it right after
 * chassis.setPose(<its measured start pose, in field coordinates>), because switching on starts the filter from
 * the pose odometry has at that moment. On gives Shadow until GPS_MOUNT_MEASURED and ODOMETRY_CALIBRATED are both
 * true (the library enforces it). runAuton() switches fusion Off again when the routine ends
 * (docs/GPS_FUSION_DESIGN.md).
 *
 * What one switch-on promises (Revision 3): after a fault, the corrections fed in since THIS switch-on are fed
 * back out and the pose is plain odometry again. A later chassis.setPose restarts the filter from the new pose by
 * itself (no need to call this again): from then on it is a new switch-on. Corrections fed in before a setPose or
 * before a second startGpsFusion() stay in the pose for good, and the total limit (maxTotalCorrectionIn) counts
 * from zero again.
 * If fusion is switched Off while a fault is still feeding a correction back out, the rest stays in the pose
 */
void startGpsFusion();

/** @brief True if runAuton(routine) runs a real routine (not "none" or a NONE placeholder) */
bool routineExists(int routine);

/**
 * Test runs (hold A, press B or X; controls.cpp): while setTestRun(true), pressing Y stops the routine at once,
 * wherever it is (the routines' wait loops check for it). testRunWasStopped() says whether that happened, so the
 * driver loop can start recording the path as the robot is pushed on by hand. Never set in a match.
 */
void setTestRun(bool on);
bool testRunWasStopped();
/**
 * The teaching run: a test run that also stops by itself after the turn numbered `stopAfterTurn` (autons.cpp lists
 * them; 0 = an ordinary test run). Hold A + X gives 2, hold A + Y gives 3.
 */
void setTeachRun(int stopAfterTurn);

/**
 * Picking the routine on the controller (src/ctrlselect.cpp). The pick is the same one the brain's selector
 * shows: either can change it.
 * Driver control: hold A, press LEFT / RIGHT to step through the routines (hold A, press B still runs it).
 * Disabled by field control: LEFT / RIGHT alone.
 */
namespace ctrlselect {
// SHELVED 2026-11-24: written and compiled, never run on the robot. Picking while disabled needs a field
// controller or competition switch to try out, so the whole picker is switched off until the team has one.
// With this false nothing calls into src/ctrlselect.cpp: the D-pad, A and disabled() behave as they did before.
// To bring it back: set this to true, then test A + LEFT/RIGHT in driver control and LEFT/RIGHT while disabled.
// Mockup of the intended behaviour: https://claude.ai/artifact/5hWaizLscDuFEdnq7sLVeR
constexpr bool ENABLED = false;
void step(int direction);  // -1 previous, +1 next; wraps, and includes "none"
void show();               // queue the three controller lines (sent by pump)
void pump();               // call every loop tick: sends one queued line per 60 ms, never blocks
void showNow();            // write the three lines now, blocking about 200 ms (for initialize())
void disabledLoop();       // for disabled(): never returns
} // namespace ctrlselect

#endif // AUTONS_HPP
