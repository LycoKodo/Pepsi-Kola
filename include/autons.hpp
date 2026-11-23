#ifndef AUTONS_HPP
#define AUTONS_HPP

// Autonomous routines (src/autons.cpp). Adding one: write it there, declare it
// here, add a button + description to its selector tab in initialize(), and a
// case in runAuton() (src/main.cpp)

/** @brief Qual routine 2 (selector: Qual tab, "james_67") */
void james_67();

/**
 * @brief Run routine number `routine` (the selector's numbering). Blocking.
 * autonomous() calls it in a match; driverControl() calls it for a test run
 */
void runAuton(int routine);

/** @brief True if runAuton(routine) runs a real routine (not "none" or a NONE placeholder) */
bool routineExists(int routine);

#endif // AUTONS_HPP
