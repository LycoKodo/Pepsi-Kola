// Testbench stub of PROS gps.hpp. Only what src/lemlib/chassis/gpsFusion.cpp uses is declared.
//
// With nothing installed, this is a GPS sensor that is not there: every read fails the way PROS reports a failed
// read (PROS_ERR_F, which is infinity). tests/gps_fusion_layer.cpp relies on that.
// The testbench's route mode installs pros::sim_detail::gpsReadHook, and then every read returns what the
// simulated sensor (bench/gps_model.cpp) is reporting at that moment.
#pragma once
#include <cmath>
#include <cstdint>

namespace pros {
// as in PROS's gps.h (packed there too)
typedef struct __attribute__((__packed__)) gps_position_s {
        double x;
        double y;
} gps_position_s_t;

namespace sim_detail {
// What the simulated sensor reports, in PROS's units: metres from the field centre, heading in degrees
struct GpsReading {
        double xMetres;
        double yMetres;
        double headingDeg;
        double errorMetres;
};
// Fills `out` and returns true, or returns false for "no sensor on this port"
using GpsReadHook = bool (*)(std::uint8_t port, GpsReading& out);
inline GpsReadHook gpsReadHook = nullptr;
} // namespace sim_detail

inline namespace v5 {
class Gps {
    public:
        explicit Gps(const std::uint8_t port)
            : port_(port) {}
        virtual ~Gps() = default;
        virtual std::int32_t set_offset(double, double) const { return 1; }
        virtual pros::gps_position_s_t get_position() const {
            sim_detail::GpsReading r;
            if (!read(r)) return {INFINITY, INFINITY};
            return {r.xMetres, r.yMetres};
        }
        virtual double get_heading() const {
            sim_detail::GpsReading r;
            return read(r) ? r.headingDeg : INFINITY;
        }
        virtual double get_error() const {
            sim_detail::GpsReading r;
            return read(r) ? r.errorMetres : INFINITY;
        }
        std::uint8_t get_port() const { return port_; }

    private:
        bool read(sim_detail::GpsReading& r) const {
            return sim_detail::gpsReadHook != nullptr && sim_detail::gpsReadHook(port_, r);
        }
        std::uint8_t port_;
};
} // namespace v5
} // namespace pros
