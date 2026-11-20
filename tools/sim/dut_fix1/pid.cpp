#include "pid.hpp"
#include "util.hpp"

namespace lemlib {
PID::PID(float kP, float kI, float kD, float windupRange, bool signFlipReset)
    : kP(kP),
      kI(kI),
      kD(kD),
      windupRange(windupRange),
      signFlipReset(signFlipReset) {}

float PID::update(const float error) {
    // calculate integral
    integral += error;
    if (sgn(error) != sgn((prevError)) && signFlipReset) integral = 0;
    if (fabs(error) > windupRange && windupRange != 0) integral = 0;

    // calculate derivative: the average change per loop over the last TWO loops, not just the last one.
    // Why: a sensor that refreshes slower than the loop (the V5 rotation sensor's default is 20 ms, the
    // loops run every 10 ms) hands the same position to two loops in a row. A one-sample difference then
    // reads 0, double, 0, double..., and the motion code's slew limit turns that into "no braking at
    // all" until its 7.5 in settle zone (seen on the robot 2026-11-19: full power until 7.4 in from the
    // target at any speed, and kD had no effect). Over two samples every difference spans one refresh,
    // so the value is steady. With fresh data every loop it is the same slope, 5 ms later
    const float derivative = (error - prevPrevError) / 2;
    prevPrevError = prevError;
    prevError = error;

    // calculate output
    return error * kP + integral * kI + derivative * kD;
}

void PID::reset() {
    integral = 0;
    prevError = 0;
    prevPrevError = 0;
}
} // namespace lemlib