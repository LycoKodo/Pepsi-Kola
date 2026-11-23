// Testbench stub of PROS distance.hpp. Only what src/lemlib/chassis/gpsFusion.cpp uses is declared, plus
// get_distance() and get_object_size() for a sensor model's own checks.
//
// Every read goes through pros::sim_detail::distanceReadHook. With no hook installed (or a null one) this is a
// sensor that is not there: every call fails the way PROS 4.1 reports a failed read (PROS_ERR, which is
// INT32_MAX, with errno set to ENODEV), which is what it returns for an unplugged port.
// The hook is declared here (extern) and must be defined exactly once by whatever program links this stub:
//     pros::sim_detail::DistanceRead (*pros::sim_detail::distanceReadHook)(int port) = nullptr;
// tests/gps_fusion_layer.cpp defines it for that test, and bench/stubs_impl.cpp for the testbench, where route mode
// points it at the simulated distance sensor (bench/dist_model.*) when it is given wall sensors (--wall-sensor).
// Whatever installs the hook must give the same answer to every call within one odometry tick (get() and
// get_confidence() each call it); the bench's model changes its reading only between ticks.
#pragma once
#include <cerrno>
#include <cstdint>

#ifndef PROS_ERR
#define PROS_ERR (INT32_MAX) // as in PROS's error.h
#endif

namespace pros {
namespace sim_detail {
// What the simulated sensor reports, in PROS's units, exactly as the three PROS calls would return them:
// mm (9999 = nothing seen, PROS_ERR = failed read), confidence (0 to 63; PROS returns 10 at or below 200 mm),
// objectSize (0 to 400, -1 = cannot tell)
struct DistanceRead {
        std::int32_t mm;
        std::int32_t confidence;
        std::int32_t objectSize;
};
// The reading of the sensor on `port` at this moment. nullptr = every distance sensor is unplugged
extern DistanceRead (*distanceReadHook)(int port);
} // namespace sim_detail

inline namespace v5 {
class Distance {
    public:
        explicit Distance(const std::uint8_t port)
            : port_(port) {}
        virtual ~Distance() = default;
        virtual std::int32_t get() { return read().mm; }
        virtual std::int32_t get_distance() { return get(); }
        virtual std::int32_t get_confidence() { return read().confidence; }
        virtual std::int32_t get_object_size() { return read().objectSize; }
        std::uint8_t get_port() const { return port_; }

    private:
        sim_detail::DistanceRead read() const {
            if (sim_detail::distanceReadHook != nullptr) return sim_detail::distanceReadHook(port_);
            errno = ENODEV;
            return {PROS_ERR, PROS_ERR, PROS_ERR};
        }
        std::uint8_t port_;
};
} // namespace v5
} // namespace pros
