// Testbench stub of PROS rtos.hpp: single thread, virtual time.
// pros::Task does NOT run its function: the only task LemLib creates on the synchronous path is the
// odometry loop in lemlib::init(), and the world scheduler calls lemlib::update() in its place.
#pragma once
#include <cstdint>
#include <utility>
#include "pros/rtos.h"

namespace pros {
inline void delay(const std::uint32_t milliseconds) { c::delay(milliseconds); }
inline std::uint32_t millis(void) { return c::millis(); }

namespace sim_detail {
void task_created(const char* name); // bench hook: the first task created is the odometry task
}

class Task {
    public:
        template <class F> Task(F&& function, const char* name = "") {
            (void)function;
            sim_detail::task_created(name);
        }
        template <class F>
        Task(F&& function, std::uint32_t prio, std::uint16_t stack_depth = TASK_STACK_DEPTH_DEFAULT,
             const char* name = "") {
            (void)function;
            (void)prio;
            (void)stack_depth;
            sim_detail::task_created(name);
        }
        static void delay(const std::uint32_t milliseconds) { c::delay(milliseconds); }
        static void delay_until(std::uint32_t* const prev_time, const std::uint32_t delta) {
            const std::uint32_t wake = *prev_time + delta;
            const std::uint32_t now = c::millis();
            if (wake > now) c::delay(wake - now);
            *prev_time = wake;
        }
};

class Mutex {
    public:
        bool take(std::uint32_t timeout = TIMEOUT_MAX) {
            (void)timeout;
            return true;
        }
        bool give() { return true; }
};
} // namespace pros
