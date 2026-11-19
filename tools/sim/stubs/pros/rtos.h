// Testbench stub of PROS rtos.h: virtual time. See tools/sim/bench/world.hpp.
#pragma once
#include <cstdint>

#define TIMEOUT_MAX ((uint32_t)0xffffffffUL)
#define TASK_PRIORITY_DEFAULT 8
#define TASK_STACK_DEPTH_DEFAULT 0x2000

namespace pros {
namespace c {
// advances the simulated world by `milliseconds` and returns
void delay(const uint32_t milliseconds);
// simulated time in ms
uint32_t millis(void);
} // namespace c
} // namespace pros
