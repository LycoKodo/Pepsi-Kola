#ifndef MCL_HPP
#define MCL_HPP

/**
 * @file mcl.hpp
 * @brief Monte Carlo localisation (particle filter) running off four distance
 *        sensors and the LemLib odometry pose.
 *
 * PROTOTYPE. The filter runs in its own task and, by default, only *observes*:
 * it never writes back to the chassis unless setCorrectionEnabled(true) is
 * called. That keeps it safe to leave running underneath an auton that has not
 * been validated against it.
 *
 * How it works, in the same shape as the published VEX implementations
 * (2654E Echo's MCL, 5225A's and 1010S' writeups):
 *
 *   1. MOTION UPDATE. Every tick the filter takes the change in the chassis
 *      pose since the last tick, converts it to a robot-relative
 *      forward/strafe/turn, and applies it to all PARTICLE_COUNT particles with
 *      Gaussian noise proportional to how far the robot moved.
 *   2. SENSOR UPDATE. For every particle, each distance sensor is ray-cast
 *      against the field walls to get the range that sensor *would* read if the
 *      robot really were at that particle. The particle's weight is multiplied
 *      by a Gaussian likelihood of the actual reading given that expectation.
 *   3. RESAMPLE. When the effective sample size drops below half the particle
 *      count, the set is resampled (low-variance / systematic resampling) so
 *      particles concentrate where the sensors agree.
 *
 * COORDINATE FRAME. Everything here is in the *field* frame: inches, origin at
 * the centre of the field, and LemLib's compass heading convention (0 degrees
 * faces +Y, increasing clockwise). The filter is only meaningful if the auton's
 * chassis.setPose() puts the robot at its real field coordinates -- a routine
 * that starts with setPose(0, 0, 0) is claiming to start dead centre facing the
 * far wall. The filter re-seeds itself whenever it sees the odometry pose jump
 * further than one tick of driving could explain, so an auton calling setPose()
 * mid-routine does not have to tell the filter anything.
 *
 * WHAT STILL NEEDS MEASURING. SENSOR_MOUNTS in mcl.cpp is a placeholder, like
 * the drivetrain track width. Each entry is where that sensor's *face* sits
 * relative to the tracking centre and which way it points. Until those are
 * measured off the real robot the ray-cast expectations are wrong and the
 * filter will confidently converge on the wrong pose.
 */

#include <cstdint>
#include "pros/distance.hpp"

namespace mcl {

/** @brief Particles in the filter. 300 costs ~2ms/tick on the V5 */
constexpr int PARTICLE_COUNT = 300;

/** @brief Distance sensors feeding the filter */
constexpr int SENSOR_COUNT = 4;

/** @brief Half the field's wall-to-wall interior, inches (140.5" / 2) */
constexpr float FIELD_HALF = 70.25f;

/** @brief One hypothesis about where the robot is */
struct Particle {
    float x;      // inches, field frame
    float y;      // inches, field frame
    float theta;  // degrees, compass
    float weight; // normalised, sums to 1 across the set
};

/** @brief Where a distance sensor sits on the robot and which way it looks */
struct SensorMount {
    pros::Distance* sensor;
    float x;     // inches right of the tracking centre (negative = left)
    float y;     // inches forward of the tracking centre (negative = back)
    float theta; // degrees the sensor faces, clockwise from robot-forward
};

/** @brief The filter's answer */
struct Estimate {
    float x;
    float y;
    float theta;
    float spread;     // 1-sigma radius of the particle cloud, inches
    float confidence; // 0..1, derived from spread: 1 = tight cloud, 0 = lost
    bool valid;       // false until the filter has run a sensor update
};

/**
 * @brief A consistent copy of the filter's state, for the visualiser
 *
 * Taken under the filter's mutex so the particle cloud, the estimate and the
 * sensor readings all describe the same tick.
 */
struct Snapshot {
    Particle particles[PARTICLE_COUNT];
    Estimate estimate;
    float odomX;
    float odomY;
    float odomTheta;
    float measured[SENSOR_COUNT]; // inches; negative means the reading was rejected
    float expected[SENSOR_COUNT]; // inches the estimate pose predicts for that sensor
    SensorMount mounts[SENSOR_COUNT]; // so the visualiser draws the rays from the
                                      // same geometry the filter weighted against
    bool locked;                  // confidence has held long enough that the
                                  // filter is allowed to correct the pose
    std::uint32_t updates;        // ticks completed since init()
    std::uint32_t usPerUpdate;    // wall time of the last tick, microseconds
};

/**
 * @brief Seed the particle set around the chassis' current pose
 *
 * Call once, after chassis.calibrate(). Does not start the task.
 */
void init();

/** @brief Start the background task. Safe to call twice */
void start();

/** @brief Stop and delete the background task */
void stop();

bool isRunning();

/**
 * @brief Re-seed every particle around a known pose
 *
 * @param x,y,theta the pose to seed around, field frame
 * @param xySigma spread of the seed cloud in inches
 * @param thetaSigma spread of the seed cloud in degrees
 */
void setPose(float x, float y, float theta, float xySigma = 2.0f, float thetaSigma = 3.0f);

/**
 * @brief Scatter particles uniformly over the whole field
 *
 * The "kidnapped robot" reset: throws away all position belief but keeps the
 * heading, which the IMU knows far better than the filter ever will.
 */
void scatter();

Estimate getEstimate();

/** @brief Copy the whole filter state out for rendering */
void snapshot(Snapshot& out);

/**
 * @brief Let the filter write its estimate back into the chassis pose
 *
 * Off by default. When on, each tick nudges the chassis pose a fraction of the
 * way toward the estimate rather than snapping to it, so a LemLib motion that
 * is mid-flight does not see the target teleport.
 */
void setCorrectionEnabled(bool state);
bool isCorrectionEnabled();

/**
 * @brief Also correct heading, not just x/y
 *
 * Off by default, and normally should stay off: a calibrated IMU drifts less
 * over a 15 second auton than the filter's heading estimate does.
 */
void setHeadingCorrectionEnabled(bool state);
bool isHeadingCorrectionEnabled();

} // namespace mcl

#endif // MCL_HPP
