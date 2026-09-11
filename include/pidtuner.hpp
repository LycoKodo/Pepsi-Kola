#ifndef PIDTUNER_HPP
#define PIDTUNER_HPP

/**
 * @file pidtuner.hpp
 * @brief On-brain editor for the chassis PID constants.
 *
 * Lets kP/kI/kD for both the lateral (drive) and angular (turn) controllers be
 * adjusted from the controller D-pad while the Developer tab is open, so gains
 * can be tuned without rebuilding and re-downloading the program.
 *
 * Values live in RAM only. They reset to the constants in robot-config.cpp on
 * every reboot, so a tune that works has to be transcribed back into the source.
 *
 * Controls (Developer tab only). The D-pad moves around the on-screen grid:
 *   RIGHT - move the cursor to the next field
 *   LEFT  - move the cursor to the previous field
 *   UP    - increase the selected field by one step
 *   DOWN  - decrease the selected field by one step
 *
 * Fields run in reading order, LAT P > LAT I > LAT D > ANG P > ANG I > ANG D >
 * STEP > AUTO, and wrap at both ends. STEP is the increment UP/DOWN apply, so a
 * coarse sweep and a fine trim are both reachable without extra buttons. UP on
 * AUTO opens the auto-tuner wizard; DOWN there does nothing.
 */

#include "liblvgl/lvgl.h"

namespace pidtuner {

/**
 * @brief Build the editor widget inside the given parent object
 *
 * @param parent LVGL object to place the editor in (the Developer tab)
 * @param x x offset from the parent's top-left corner
 * @param y y offset from the parent's top-left corner
 * @param w width in pixels
 * @param h height in pixels
 */
void init(lv_obj_t* parent, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h);

/**
 * @brief Poll the controller and apply any edits
 *
 * Does nothing unless the editor is active. Safe to call every loop.
 */
void handleInput();

/**
 * @brief Redraw the editor with the gains the chassis is currently running
 */
void render();

/**
 * @brief Set whether the editor owns the D-pad
 *
 * Driven by the tab watcher: true while the Developer tab is showing.
 */
void setActive(bool state);

/**
 * @brief Whether the editor currently owns the D-pad
 *
 * teleop_loop() checks this so the D-pad does not drive both the pneumatics and
 * the editor at the same time.
 */
bool isActive();

} // namespace pidtuner

#endif // PIDTUNER_HPP
