#ifndef AUTOTUNE_HPP
#define AUTOTUNE_HPP

/**
 * @file autotune.hpp
 * @brief Guided relay (Astrom-Hagglund) auto-tuner for the chassis PIDs.
 *
 * Replaces the PID with a bang-bang relay for a few seconds, which drives the
 * robot into a sustained oscillation, then measures that oscillation to get the
 * ultimate gain and ultimate period:
 *
 *   Ku = 4d / (pi * a)     d = relay amplitude, a = oscillation amplitude
 *   Tu = oscillation period
 *
 * Standard tuning rules then turn (Ku, Tu) into kP/kI/kD. This is a measurement,
 * not a search: it runs once, for about ten seconds, and does not need dozens of
 * trial motions the way a cost-function optimiser would.
 *
 * The wizard is a full screen overlay driven entirely from the D-pad, launched
 * from the AUTO field of the PID editor on the Developer tab. It walks through
 * safety, axis choice, placement, relay amplitude, the run itself, and finally
 * applying the result.
 *
 * Everything it produces is a starting point. Trim by hand in the PID editor
 * afterwards, then transcribe into robot-config.cpp.
 */

namespace autotune {

/** @brief Open the wizard. Ignored if it is already open */
void start();

/** @brief Whether the wizard overlay is showing and owns the D-pad */
bool isRunning();

/**
 * @brief Whether the relay is driving the drivetrain right now
 *
 * Narrower than isRunning(): false on the wizard's other pages, so the driver
 * can still drive the robot into position during PLACEMENT. drivetrain_control
 * checks this and stops writing the drive motors while it is true.
 */
bool isRelayActive();

/** @brief Poll the D-pad and advance the wizard. Safe to call every loop */
void handleInput();

/** @brief Redraw the wizard. Safe to call every loop */
void render();

} // namespace autotune

#endif // AUTOTUNE_HPP
