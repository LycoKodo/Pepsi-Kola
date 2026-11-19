#ifndef CONTROLS_HPP
#define CONTROLS_HPP

/**
 * @file controls.hpp
 * @brief Driver control. The button map is in the comments in src/controls.cpp and
 *        the macro table in macros.hpp.
 */

/**
 * @brief The driver-control loop. Never returns; call it from opcontrol().
 *
 * Drive, intake and manual cascade run here every 10ms and never block.
 * Anything with a wait in it is a macro (macros.hpp) and runs on its own task.
 */
void driverControl();

#endif // CONTROLS_HPP
