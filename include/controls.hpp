#ifndef CONTROLS_HPP
#define CONTROLS_HPP

/**
 * @file controls.hpp
 * @brief Header file for robot control mechanisms.
 *
 * This file declares the global variables and functions used to manage various subsystems
 * of the robot, such as alliance selection, drivetrain, and intake control.
 *
 * ## Global Variables
 * - `char alliance`: Tracks the selected alliance ('R' for Red, 'B' for Blue, 'N' for None).
 * - `bool allianceConfirmed`: Indicates if the alliance selection has been finalized.
 */

#include "main.h"

// Variables to keep track of alliance and selection status
extern char alliance;
extern bool allianceConfirmed;

/**
 * @brief Provides joystick-based drivetrain control with toggleable braking modes.
 */
void drivetrain_control();

#endif // CONTROLS_HPP