// Implementations of the stub PROS devices declared in tools/sim/stubs/pros/*.
// All device state lives in sim::world(), keyed by port.
#include <cstring>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "world.hpp"

using sim::world;

// ---------------------------------------------------------------- time
namespace pros {
namespace c {
void delay(const uint32_t milliseconds) {
    sim::World& w = world();
    if (w.onDelay) w.onDelay(w.now);
    w.advance(milliseconds);
}
uint32_t millis(void) { return world().now; }
} // namespace c

namespace sim_detail {
// The distance sensor stub's hook (stubs/pros/distance.hpp), defined once here for the whole bench. It stays
// nullptr (every distance sensor unplugged) unless route mode is given wall sensors (--wall-sensor): then
// bench/route.cpp points it at the simulated sensor (bench/dist_model.*)
DistanceRead (*distanceReadHook)(int port) = nullptr;

void task_created(const char* name) {
    // GPS fusion's logging task (gpsFusion.cpp, started by configure()) only prints the [GPSFUSE] lines; the bench
    // reads status() itself, so it is not run, not counted and not warned about
    if (name && std::strcmp(name, "gpsfuse log") == 0) return;
    sim::World& w = world();
    w.tasksCreated++;
    if (w.tasksCreated == 1) {
        // lemlib::init()'s odometry loop: the scheduler calls lemlib::update() instead
        w.odomRunning = true;
    } else {
        std::fprintf(stderr, "[tb] warning: the DUT created another task (%s); it does not run in the bench\n",
                     name && *name ? name : "unnamed");
    }
}
} // namespace sim_detail

// ---------------------------------------------------------------- motors
inline namespace v5 {
static int sideOf(const std::vector<std::int8_t>& ports) {
    for (auto p : ports) {
        auto it = world().motors.find(std::abs(p));
        if (it != world().motors.end() && it->second.side >= 0) return it->second.side;
    }
    return -1;
}

static void registerPorts(const std::vector<std::int8_t>& ports, MotorGears g, MotorUnits u) {
    for (auto p : ports) {
        sim::MotorPort& m = world().motors[std::abs(p)];
        if (g != MotorGears::invalid) m.gearing = int(g);
        if (u != MotorUnits::invalid) m.units = int(u);
    }
}

MotorGroup::MotorGroup(const std::initializer_list<std::int8_t> ports, const MotorGears gearset,
                       const MotorUnits encoder_units)
    : ports_(ports) {
    registerPorts(ports_, gearset, encoder_units);
}

MotorGroup::MotorGroup(const std::vector<std::int8_t>& ports, const MotorGears gearset, const MotorUnits encoder_units)
    : ports_(ports) {
    registerPorts(ports_, gearset, encoder_units);
}

std::int32_t MotorGroup::move(std::int32_t voltage) const {
    const int s = sideOf(ports_);
    if (s >= 0) world().cmd[s] = std::clamp<std::int32_t>(voltage, -127, 127);
    return 1;
}

std::int32_t MotorGroup::move_voltage(std::int32_t mv) const {
    return move(std::int32_t(std::clamp<std::int32_t>(mv, -12000, 12000) * 127 / 12000));
}

double MotorGroup::get_actual_velocity(const std::uint8_t) const {
    const int s = sideOf(ports_);
    return s >= 0 ? world().motorRpm(s) : 0.0;
}

std::vector<double> MotorGroup::get_actual_velocity_all(void) const {
    return std::vector<double>(ports_.size(), get_actual_velocity(0));
}

std::int32_t MotorGroup::get_voltage(const std::uint8_t) const {
    const int s = sideOf(ports_);
    return s >= 0 ? std::int32_t(std::lround(world().applied[s] * 1000.0)) : 0;
}

double MotorGroup::get_position(const std::uint8_t index) const {
    if (index >= ports_.size()) return 0;
    return world().motorPositionRaw(world().motors[std::abs(ports_[index])]);
}

std::vector<double> MotorGroup::get_position_all(void) const {
    std::vector<double> out;
    for (size_t i = 0; i < ports_.size(); i++) out.push_back(get_position(std::uint8_t(i)));
    return out;
}

std::int32_t MotorGroup::tare_position(const std::uint8_t index) const {
    if (index >= ports_.size()) return 0;
    sim::MotorPort& m = world().motors[std::abs(ports_[index])];
    if (m.side >= 0) m.tare = world().travel[m.side];
    return 1;
}

std::int32_t MotorGroup::tare_position_all(void) const {
    for (size_t i = 0; i < ports_.size(); i++) tare_position(std::uint8_t(i));
    return 1;
}

std::int32_t MotorGroup::set_brake_mode(const MotorBrake mode, const std::uint8_t index) const {
    if (index < ports_.size()) world().motors[std::abs(ports_[index])].brake = int(mode);
    return 1;
}
std::int32_t MotorGroup::set_brake_mode(const pros::motor_brake_mode_e_t mode, const std::uint8_t index) const {
    return set_brake_mode(MotorBrake(int(mode)), index);
}
std::int32_t MotorGroup::set_brake_mode_all(const MotorBrake mode) const {
    for (auto p : ports_) world().motors[std::abs(p)].brake = int(mode);
    return 1;
}
std::int32_t MotorGroup::set_brake_mode_all(const pros::motor_brake_mode_e_t mode) const {
    return set_brake_mode_all(MotorBrake(int(mode)));
}
MotorBrake MotorGroup::get_brake_mode(const std::uint8_t index) const {
    if (index >= ports_.size()) return MotorBrake::invalid;
    return MotorBrake(world().motors[std::abs(ports_[index])].brake);
}

std::int32_t MotorGroup::set_encoder_units(const MotorUnits units, const std::uint8_t index) const {
    if (index < ports_.size()) world().motors[std::abs(ports_[index])].units = int(units);
    return 1;
}
std::int32_t MotorGroup::set_encoder_units(const pros::motor_encoder_units_e_t units, const std::uint8_t index) const {
    return set_encoder_units(MotorUnits(int(units)), index);
}
std::int32_t MotorGroup::set_encoder_units_all(const MotorUnits units) const {
    for (auto p : ports_) world().motors[std::abs(p)].units = int(units);
    return 1;
}
std::int32_t MotorGroup::set_encoder_units_all(const pros::motor_encoder_units_e_t units) const {
    return set_encoder_units_all(MotorUnits(int(units)));
}

std::int32_t MotorGroup::set_gearing_all(const MotorGears gearset) const {
    for (auto p : ports_) world().motors[std::abs(p)].gearing = int(gearset);
    return 1;
}
std::vector<MotorGears> MotorGroup::get_gearing_all(void) const {
    std::vector<MotorGears> out;
    for (auto p : ports_) out.push_back(MotorGears(world().motors[std::abs(p)].gearing));
    return out;
}

std::int8_t MotorGroup::size(void) const { return std::int8_t(ports_.size()); }

// ---------------------------------------------------------------- IMU
Imu::Imu(const std::uint8_t port)
    : port_(port) {
    world().imu[port].bound = true;
}
std::int32_t Imu::reset(bool) const {
    sim::ImuDev& d = world().imu[port_];
    d.zeroDeg = d.sampleDeg;
    return 1;
}
double Imu::get_rotation() const {
    const sim::ImuDev& d = world().imu[port_];
    return d.sampleDeg - d.zeroDeg;
}
double Imu::get_heading() const {
    const double r = std::fmod(get_rotation(), 360.0);
    return r < 0 ? r + 360.0 : r;
}
std::int32_t Imu::set_rotation(double target) const {
    sim::ImuDev& d = world().imu[port_];
    d.zeroDeg = d.sampleDeg - target;
    return 1;
}
std::int32_t Imu::set_heading(double target) const { return set_rotation(target); }
std::int32_t Imu::tare() const { return reset(false); }
pros::ImuStatus Imu::get_status() const { return pros::ImuStatus::ready; }
bool Imu::is_calibrating() const { return false; }

// ---------------------------------------------------------------- rotation sensors
Rotation::Rotation(const std::int8_t port)
    : port_(std::uint8_t(std::abs(port))) {
    sim::RotationDev& r = world().rot[port_];
    r.reversed = port < 0;
    r.periodMs = world().p.rotMs;
}
std::int32_t Rotation::reset(void) const { return reset_position(); }
std::int32_t Rotation::set_data_rate(std::uint32_t rate) const {
    sim::RotationDev& r = world().rot[port_];
    r.requestedRates.push_back(int(rate));
    if (!world().p.rotLock) r.periodMs = std::max(5, int(rate) / 5 * 5);
    return 1;
}
std::int32_t Rotation::set_position(std::uint32_t position) const {
    sim::RotationDev& r = world().rot[port_];
    world().refreshRotation(r);
    r.zero = r.sample - std::int32_t(position) * (r.reversed ? -1 : 1);
    return 1;
}
std::int32_t Rotation::reset_position(void) const { return set_position(0); }
std::int32_t Rotation::get_position() const {
    const sim::RotationDev& r = world().rot[port_];
    const std::int32_t v = r.sample - r.zero;
    return r.reversed ? -v : v;
}
std::int32_t Rotation::get_velocity() const { return 0; }
std::int32_t Rotation::get_angle() const {
    const std::int32_t a = get_position() % 36000;
    return a < 0 ? a + 36000 : a;
}
std::int32_t Rotation::set_reversed(bool value) const {
    world().rot[port_].reversed = value;
    return 1;
}
std::int32_t Rotation::reverse() const { return set_reversed(!world().rot[port_].reversed); }
std::int32_t Rotation::get_reversed() const { return world().rot[port_].reversed; }
} // namespace v5
} // namespace pros
