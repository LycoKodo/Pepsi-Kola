#ifndef CONTROLS_HPP
#define CONTROLS_HPP

/**
 * @file controls.hpp
 * @brief Header file for robot control mechanisms.
 *
 * This file declares the global variables and functions used to manage various subsystems
 * of the robot, such as alliance selection, drivetrain, and intake control.
 */

#include "main.h"

/**
 * @brief Provides joystick-based drivetrain control with toggleable braking modes.
 */
void drivetrain_control();
void teleop_loop();

#endif // CONTROLS_HPP