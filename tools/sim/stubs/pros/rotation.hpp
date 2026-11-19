// Testbench stub of PROS rotation.hpp. get_position() returns centidegrees of the simulated tracking
// wheel, refreshed only every data-rate ms (default --rot-ms, 20). set_data_rate() from the DUT takes
// effect unless the bench runs with --rot-lock. A negative port means reversed.
#pragma once
#include <cstdint>

namespace pros {
inline namespace v5 {
class Rotation {
    public:
        Rotation(const std::int8_t port);
        virtual ~Rotation() = default;
        virtual std::int32_t reset(void) const;
        virtual std::int32_t set_data_rate(std::uint32_t rate) const;
        virtual std::int32_t set_position(std::uint32_t position) const;
        virtual std::int32_t reset_position(void) const;
        virtual std::int32_t get_position() const;
        virtual std::int32_t get_velocity() const;
        virtual std::int32_t get_angle() const;
        virtual std::int32_t set_reversed(bool value) const;
        virtual std::int32_t reverse() const;
        virtual std::int32_t get_reversed() const;
        std::uint8_t get_port() const { return port_; }

    private:
        std::uint8_t port_;
};
} // namespace v5
} // namespace pros
