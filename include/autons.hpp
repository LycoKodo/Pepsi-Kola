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
 * Selector: Skills tab, "UMB CW" (routine 101) and "UMB CCW" (102). Never switches GPS fusion on.
 * Test runs only (hold A, press B): it refuses under field control or a competition switch, with the lift up, with
 * the inertial sensor failed, or while another move runs. A stick pushed stops it at once, as does the inertial
 * sensor stopping, a move running out of time or a PID tab test starting: one [umbmark] line, no result
 */
void umbmarkSquare(bool clockwise);

/**
 * @brief true if the inertial sensor read a heading right after chassis.calibrate() at boot (src/main.cpp). If
 * it did not, LemLib takes its heading from the wheels until the program restarts, even if the sensor comes back
 */
bool imuCalibratedAtBoot();

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

#endif // AUTONS_HPP
