// Testbench stub of PROS abstract_motor.hpp: the C++ motor enums.
#pragma once
#include <cstdint>
#include <climits>
#include "pros/motors.h"
#include "pros/rtos.hpp"

namespace pros {
inline namespace v5 {
enum class MotorBrake { coast = 0, brake = 1, hold = 2, invalid = INT32_MAX };

enum class MotorEncoderUnits { degrees = 0, deg = 0, rotations = 1, counts = 2, invalid = INT32_MAX };
using MotorUnits = MotorEncoderUnits;

enum class MotorGears {
    ratio_36_to_1 = 0,
    red = ratio_36_to_1,
    rpm_100 = ratio_36_to_1,
    ratio_18_to_1 = 1,
    green = ratio_18_to_1,
    rpm_200 = ratio_18_to_1,
    ratio_6_to_1 = 2,
    blue = ratio_6_to_1,
    rpm_600 = ratio_6_to_1,
    invalid = INT32_MAX
};
using MotorGearset = MotorGears;
using MotorCart = MotorGears;
using MotorCartridge = MotorGears;
using MotorGear = MotorGears;
} // namespace v5
} // namespace pros
