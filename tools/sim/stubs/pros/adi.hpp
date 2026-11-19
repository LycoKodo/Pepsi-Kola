// Testbench stub of PROS adi.hpp: LemLib's TrackingWheel can hold an ADI encoder; the robot uses
// rotation sensors, so this encoder always reads 0.
#pragma once
#include <cstdint>

namespace pros {
inline namespace v5 {
namespace adi {
class Encoder {
    public:
        Encoder(std::uint8_t adi_port_top, std::uint8_t adi_port_bottom, bool reversed = false) {
            (void)adi_port_top;
            (void)adi_port_bottom;
            (void)reversed;
        }
        std::int32_t reset() const { return 1; }
        std::int32_t get_value() const { return 0; }
};
} // namespace adi
} // namespace v5
} // namespace pros
