// Testbench stub of PROS motor_group.hpp. Device state lives in the simulated world, keyed by port
// (like the real brain), so the methods are const as in PROS. A negative port means reversed; the
// stub treats every group as already direction-corrected (positive = forward), as LemLib assumes.
#pragma once
#include <cstdint>
#include <initializer_list>
#include <vector>
#include "pros/abstract_motor.hpp"
#include "pros/motors.h"

namespace pros {
inline namespace v5 {
class MotorGroup {
    public:
        MotorGroup(const std::initializer_list<std::int8_t> ports, const MotorGears gearset = MotorGears::invalid,
                   const MotorUnits encoder_units = MotorUnits::invalid);
        MotorGroup(const std::vector<std::int8_t>& ports, const MotorGears gearset = MotorGears::invalid,
                   const MotorUnits encoder_units = MotorUnits::invalid);

        // -127..127, stored as the side's command (float arguments truncate toward zero, as in PROS)
        std::int32_t move(std::int32_t voltage) const;
        std::int32_t move_voltage(std::int32_t millivolts) const;

        double get_actual_velocity(const std::uint8_t index = 0) const; // motor rpm
        std::vector<double> get_actual_velocity_all(void) const;
        std::int32_t get_voltage(const std::uint8_t index = 0) const; // applied mV
        double get_position(const std::uint8_t index = 0) const;       // in the set encoder units
        std::vector<double> get_position_all(void) const;
        std::int32_t tare_position(const std::uint8_t index = 0) const;
        std::int32_t tare_position_all(void) const;

        std::int32_t set_brake_mode(const MotorBrake mode, const std::uint8_t index = 0) const;
        std::int32_t set_brake_mode(const pros::motor_brake_mode_e_t mode, const std::uint8_t index = 0) const;
        std::int32_t set_brake_mode_all(const MotorBrake mode) const;
        std::int32_t set_brake_mode_all(const pros::motor_brake_mode_e_t mode) const;
        MotorBrake get_brake_mode(const std::uint8_t index = 0) const;

        std::int32_t set_encoder_units(const MotorUnits units, const std::uint8_t index = 0) const;
        std::int32_t set_encoder_units(const pros::motor_encoder_units_e_t units, const std::uint8_t index = 0) const;
        std::int32_t set_encoder_units_all(const MotorUnits units) const;
        std::int32_t set_encoder_units_all(const pros::motor_encoder_units_e_t units) const;

        std::int32_t set_gearing_all(const MotorGears gearset) const;
        std::vector<MotorGears> get_gearing_all(void) const;

        std::int8_t size(void) const;
        std::vector<std::int8_t> get_port_all(void) const { return ports_; }

    private:
        std::vector<std::int8_t> ports_;
};
} // namespace v5
} // namespace pros
