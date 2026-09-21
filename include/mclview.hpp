#ifndef MCLVIEW_HPP
#define MCLVIEW_HPP

/**
 * @file mclview.hpp
 * @brief Live view of the particle filter on the brain screen.
 *
 * Draws the field, the particle cloud, the odometry pose and the filter's
 * estimate into an LVGL canvas, with the numbers beside it. Lives on its own
 * selector tab, so it can be swapped in and out with a touch while an auton is
 * running -- the tab watcher stops forcing the Developer tab once the auton
 * starts, and just renders whichever of the two is showing.
 *
 * Reading it: the scattered dots are the particles, amber is where odometry
 * thinks the robot is, green is where the filter thinks it is, and the four
 * dim spokes off the green marker are the distance readings the sensors
 * actually returned. When the spokes land on the walls, the filter agrees with
 * the sensors; when they stop short or overshoot, it does not.
 */

#include "liblvgl/lvgl.h"

namespace mclview {

/**
 * @brief Build the canvas and readout inside the given parent object
 *
 * @param parent LVGL object to place the view in (the MCL tab)
 */
void init(lv_obj_t* parent);

/**
 * @brief Redraw from the filter's current state
 *
 * Throttled internally to ~20Hz, so calling it from a 10ms loop is fine.
 */
void render();

} // namespace mclview

#endif // MCLVIEW_HPP
