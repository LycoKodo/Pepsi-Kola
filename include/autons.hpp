#ifndef AUTONS_HPP
#define AUTONS_HPP

// Autonomous routines. None exist for the new robot yet. Adding one: write it,
// add a button + description to its selector tab in initialize(), and a case
// in runAuton() (src/main.cpp)

/**
 * @brief Run routine number `routine` (the selector's numbering). Blocking.
 * autonomous() calls it in a match; driverControl() calls it for a test run
 */
void runAuton(int routine);

#endif // AUTONS_HPP
