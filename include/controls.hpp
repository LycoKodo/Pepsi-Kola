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

/**
 * @brief Capture the cascade/rollerTurn resting positions and set brake modes.
 *
 * The VEXcode `when_started` block. Call once from initialize(); every cascade
 * and rollerTurn target in teleop_loop() is an offset from what this records.
 */
void mechanisms_init();

/** @brief The cascade position recorded at boot (VEXcode: CascadeSet) */
double getCascadeHome();

/** @brief The rollerTurn position recorded at boot (VEXcode: CascadeUpperLimit) */
double getRollerTurnHome();

void teleop_loop();

#endif // CONTROLS_HPP