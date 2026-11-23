#pragma once

#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"

namespace lemlib {
/**
 * @brief Set the sensors to be used for odometry
 *
 * @param sensors the sensors to be used
 * @param drivetrain drivetrain to be used
 */
void setSensors(lemlib::OdomSensors sensors, lemlib::Drivetrain drivetrain);
/**
 * @brief Get the pose of the robot
 *
 * @param radians true for theta in radians, false for degrees. False by default
 * @return Pose
 */
Pose getPose(bool radians = false);
/**
 * @brief Set the Pose of the robot
 *
 * @param pose the new pose
 * @param radians true if theta is in radians, false if in degrees. False by default
 */
void setPose(Pose pose, bool radians = false);
/**
 * @brief Get the speed of the robot
 *
 * @param radians true for theta in radians, false for degrees. False by default
 * @return lemlib::Pose
 */
Pose getSpeed(bool radians = false);
/**
 * @brief Get the local speed of the robot
 *
 * @param radians true for theta in radians, false for degrees. False by default
 * @return lemlib::Pose
 */
Pose getLocalSpeed(bool radians = false);
/**
 * @brief Estimate the pose of the robot after a certain amount of time
 *
 * @param time time in seconds
 * @param radians False for degrees, true for radians. False by default
 * @return lemlib::Pose
 */
Pose estimatePose(float time, bool radians = false);
/**
 * @brief Correct the inertial sensor's heading scale
 *
 * Every heading change the IMU reports is multiplied by `clockwise` when the robot turned clockwise and by
 * `counterClockwise` when it turned the other way. 1.0 and 1.0 (the default) change nothing at all.
 * To measure: turn the robot ten full turns one way and divide 3600 by how many degrees it reads; repeat the
 * other way (docs/GPS_FUSION_DESIGN.md, "Inertial sensor scale")
 *
 * @param clockwise scale for clockwise turns
 * @param counterClockwise scale for counter-clockwise turns
 */
void setImuScale(float clockwise, float counterClockwise);
/**
 * @brief Update the pose of the robot
 *
 */
void update();
/**
 * @brief Initialize the odometry system
 *
 */
void init();
} // namespace lemlib
