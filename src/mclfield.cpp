#include "mclfield.hpp"

#include <cmath>

#include "pros/misc.h"
#include "robot-config.hpp"

namespace mcl::field {

namespace {
constexpr float DEG_TO_RAD = 0.017453292519943295f;

// A V5 distance sensor reads 20mm..2000mm. The whole band has to stay open:
// from the middle of the field every wall is 63 inches away, so a tighter
// cutoff blinds all four sensors at once, exactly where a routine spends most
// of its time
constexpr float MIN_RANGE_IN = 1.5f;
constexpr float MAX_RANGE_IN = 78.0f;   // ~2000mm
constexpr int MIN_CONFIDENCE = 30;      // sensor reports 0..63
} // namespace

// Ports 16..19, declared in robot-config.cpp. Which physical face each sensor
// is on has to match the offsets here
SensorMount mounts[SENSOR_COUNT] = {
    {&distFront, 0.0f, 7.0f, 0.0f},
    {&distRight, 7.0f, 0.0f, 90.0f},
    {&distBack, 0.0f, -7.0f, 180.0f},
    {&distLeft, -7.0f, 0.0f, 270.0f},
};

/**
 * @brief Distance from a point inside the field to the first wall along a ray
 *
 * The field is an axis-aligned box, so this is the slab method: work out where
 * the ray crosses each of the two walls it is heading toward and take the
 * nearer crossing. No per-wall segment intersection needed.
 */
float castToWall(float ox, float oy, float headingDeg) {
    const float rad = headingDeg * DEG_TO_RAD;
    // LemLib compass convention: 0 degrees is +Y, increasing clockwise
    const float dx = std::sin(rad);
    const float dy = std::cos(rad);

    float t = MAX_RANGE_IN * 2.0f;
    if (std::fabs(dx) > 1e-6f) {
        const float tx = ((dx > 0 ? FIELD_HALF : -FIELD_HALF) - ox) / dx;
        if (tx > 0.0f && tx < t) t = tx;
    }
    if (std::fabs(dy) > 1e-6f) {
        const float ty = ((dy > 0 ? FIELD_HALF : -FIELD_HALF) - oy) / dy;
        if (ty > 0.0f && ty < t) t = ty;
    }
    return t;
}

/** @brief Range a sensor would read if the robot were at this particle */
float expectedRange(const Particle& p, const SensorMount& m) {
    const float rad = p.theta * DEG_TO_RAD;
    const float sn = std::sin(rad);
    const float cs = std::cos(rad);
    // robot forward is (sin, cos), robot right is (cos, -sin)
    const float ox = p.x + m.y * sn + m.x * cs;
    const float oy = p.y + m.y * cs - m.x * sn;
    return castToWall(ox, oy, p.theta + m.theta);
}

/** @brief Read one sensor in inches, or -1 if the reading cannot be trusted */
float readSensor(int i) {
    pros::Distance* d = mounts[i].sensor;
    if (d == nullptr) return -1.0f;
    const int mm = d->get();
    if (mm == PROS_ERR || mm <= 0) return -1.0f;
    if (d->get_confidence() < MIN_CONFIDENCE) return -1.0f;
    const float inches = static_cast<float>(mm) / 25.4f;
    if (inches < MIN_RANGE_IN || inches > MAX_RANGE_IN) return -1.0f;
    return inches;
}

} // namespace mcl::field
