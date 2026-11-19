// Testbench stub of PROS motors.h: just the C enums LemLib names.
#pragma once
#include <cstdint>
#include <climits>

namespace pros {
typedef enum motor_brake_mode_e {
    E_MOTOR_BRAKE_COAST = 0,
    E_MOTOR_BRAKE_BRAKE = 1,
    E_MOTOR_BRAKE_HOLD = 2,
    E_MOTOR_BRAKE_INVALID = INT32_MAX
} motor_brake_mode_e_t;

typedef enum motor_encoder_units_e {
    E_MOTOR_ENCODER_DEGREES = 0,
    E_MOTOR_ENCODER_ROTATIONS = 1,
    E_MOTOR_ENCODER_COUNTS = 2,
    E_MOTOR_ENCODER_INVALID = INT32_MAX
} motor_encoder_units_e_t;

typedef enum motor_gearset_e {
    E_MOTOR_GEARSET_36 = 0,
    E_MOTOR_GEARSET_18 = 1,
    E_MOTOR_GEARSET_06 = 2,
    E_MOTOR_GEARSET_INVALID = INT32_MAX
} motor_gearset_e_t;
} // namespace pros
