// Testbench stub of PROS misc.hpp: competition status is always 0 (enabled, driver, not connected).
#pragma once
#include <cstdint>
#include "pros/misc.h"

namespace pros {
namespace competition {
inline std::uint8_t get_status(void) { return 0; }
inline std::uint8_t is_autonomous(void) { return 0; }
inline std::uint8_t is_connected(void) { return 0; }
inline std::uint8_t is_disabled(void) { return 0; }
} // namespace competition
} // namespace pros
