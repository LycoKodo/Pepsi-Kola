#ifndef MCLFIELD_HPP
#define MCLFIELD_HPP

/**
 * @file mclfield.hpp
 * @brief The world model the particle filter weighs its hypotheses against.
 *
 * Split out from the filter itself because this is the half that changes when
 * the robot does: where the four distance sensors are bolted on, which way they
 * look, and what shape the field they are looking at is. The filter in mcl.cpp
 * never needs editing to follow a hardware change; this does.
 *
 * The field is modelled as the bare perimeter wall -- an axis-aligned square,
 * nothing inside it. Field elements are deliberately absent: a goal that a
 * sensor happens to catch produces a reading far shorter than any wall
 * prediction, which the Gaussian in the sensor model treats as a very unlikely
 * observation rather than as evidence for some other pose. That is the right
 * behaviour, and adding obstacles to the model without also modelling where
 * they get moved to during a match would make things worse, not better.
 */

#include "mcl.hpp"

namespace mcl::field {

/**
 * @brief Where each sensor's face sits and which way it looks
 *
 * PLACEHOLDER -- measure these on the real robot, the same way the drivetrain
 * track width still has to be measured. x is inches right of the tracking
 * centre, y is inches forward, theta is the direction the sensor points in
 * degrees clockwise from robot-forward. Until they are right the ray-cast
 * expectations are wrong, and a wrong world model makes the filter confidently
 * converge on the wrong pose rather than admit it does not know.
 */
extern SensorMount mounts[SENSOR_COUNT];

/**
 * @brief Distance from a point inside the field to the first wall along a ray
 *
 * @param ox,oy the ray's origin, field frame inches
 * @param headingDeg compass heading of the ray
 */
float castToWall(float ox, float oy, float headingDeg);

/** @brief Range a sensor would read if the robot really were at this particle */
float expectedRange(const Particle& p, const SensorMount& m);

/**
 * @brief Read sensor i in inches
 *
 * @return the range, or -1 if the reading cannot be trusted (sensor missing,
 *         low confidence, or outside the sensor's usable band)
 */
float readSensor(int i);

} // namespace mcl::field

#endif // MCLFIELD_HPP
