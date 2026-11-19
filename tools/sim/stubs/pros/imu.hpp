// Testbench stub of PROS imu.hpp: reads the simulated heading (clockwise positive, degrees),
// refreshed every imu period (default 10 ms). Never calibrating.
#pragma once
#include <cmath>
#include <cstdint>

// LemLib's chassis.cpp calls isnanf (newlib/glibc); macOS libm does not declare it
#ifndef isnanf
inline int isnanf(float x) { return std::isnan(x); }
#endif

namespace pros {
enum class ImuStatus { ready = 0, calibrating = 19, error = 0xFF };

inline namespace v5 {
class Imu {
    public:
        explicit Imu(const std::uint8_t port);
        virtual ~Imu() = default;
        virtual std::int32_t reset(bool blocking = false) const;
        virtual double get_rotation() const;
        virtual double get_heading() const;
        virtual std::int32_t set_rotation(double target) const;
        virtual std::int32_t set_heading(double target) const;
        virtual std::int32_t tare() const;
        virtual pros::ImuStatus get_status() const;
        virtual bool is_calibrating() const;
        std::uint8_t get_port() const { return port_; }

    private:
        std::uint8_t port_;
};
} // namespace v5
} // namespace pros
