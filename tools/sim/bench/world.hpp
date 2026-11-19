// The simulated robot ("world") and the virtual-time scheduler.
//
// Time: integer ms. The DUT runs in the bench's only thread; pros::delay(n) calls World::advance(n),
// which runs n ticks. One tick from t to t+1:
//   1. preTick(t) hook (the bench records the row for time t: the DUT's code at t has already run)
//   2. 1 ms of physics
//   3. now = t+1, then the events due at t+1, in this order: sensor refreshes (IMU, rotation
//      sensors), lemlib::update() when odometry is running and (t+1 - odomPhase) % 10 == 0, then
//      the postEvents(t+1) hook (the bench's PID-tab sampler).
// So anything scheduled at a millisecond happens before DUT code that runs in that millisecond.
#pragma once
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace sim {

enum Side { LEFT = 0, RIGHT = 1 };

struct Params {
    // plant (robot.json)
    double C = 29.4;            // in/s^2 per volt of net voltage
    double KV = 12.0 / 76.0;    // volts per in/s (back-EMF)
    double F = 25.0;            // in/s^2 of Coulomb friction
    double DV_ACCEL = 3.4;      // net-voltage limit while driving with the motion (or from rest), V
    double DV_BRAKE = 6.0;      // net-voltage limit while opposing the motion, V
    double TRACK_EFF = 15.0;    // effective track width for heading rate, in
    double LATENCY_MS = 10.0;   // command latency, ms (fractional: interpolated)
    double C_DRIVE_L = 1.0, C_DRIVE_R = 1.0; // per-side scale on C while driving
    double C_BRAKE_L = 1.0, C_BRAKE_R = 1.0; // per-side scale on C while braking
    double V_MAX = 12.0;        // volts at command 127
    double STICK_EPS = 1e-3;    // in/s: below this a side counts as stopped
    // geometry
    double VERT_OFFSET = 0.9;   // vertical tracking wheel, in right of centre
    double HORIZ_OFFSET = 2.55; // horizontal tracking wheel, in in front of centre
    double TRACK_WHEEL_D = 2.125;
    double DRIVE_WHEEL_D = 2.75;
    double DRIVE_RPM = 450;     // wheel rpm at motor free speed (600 rpm cartridge)
    // timing (command line)
    int rotMs = 20;      // rotation sensor refresh period until the DUT changes it
    bool rotLock = false; // ignore the DUT's set_data_rate()
    int rotPhase = 0;    // rotation sensors refresh when (t - rotPhase) % period == 0
    int imuMs = 10;
    int imuPhase = 0;
    int odomPhase = 0;   // lemlib::update() when (t - odomPhase) % 10 == 0

    bool set(const std::string& key, double value); // false if unknown key
};

struct RotationDev {
    bool bound = false;
    bool isVertical = true;   // which tracking wheel this sensor is on
    bool reversed = false;
    int periodMs = 20;
    std::vector<int> requestedRates; // every set_data_rate() the DUT made
    int32_t sample = 0;       // last refreshed reading, centidegrees, before reverse/zero
    int32_t zero = 0;         // raw reading at the last reset_position()
    uint32_t seq = 0;         // refresh counter
    uint32_t lastRefresh = 0;
};

struct ImuDev {
    bool bound = false;
    double sampleDeg = 0;     // last refreshed heading, deg, clockwise positive, unwrapped
    double zeroDeg = 0;
};

struct MotorPort {
    int side = -1;            // LEFT/RIGHT if part of the drivetrain
    int brake = 0;            // 0 coast, 1 brake, 2 hold
    int units = 0;            // 0 degrees, 1 rotations, 2 counts
    int gearing = 2;          // 0 red, 1 green, 2 blue
    double tare = 0;          // side travel (in) at the last tare
};

struct World {
    Params p;
    uint32_t now = 0;

    // true state (in, rad, in/s); heading 0 = +y, clockwise positive (LemLib convention)
    double x = 0, y = 0, heading = 0;
    double v[2] = {0, 0};
    double travel[2] = {0, 0};      // per-side ground travel, in
    double vertTravel = 0;          // vertical tracking wheel: + forward
    double horizTravelLeft = 0;     // horizontal tracking wheel: + to the robot's left (LemLib)

    // drive commands and actuator outputs
    int cmd[2] = {0, 0};            // last value the DUT passed to move(), -127..127
    std::vector<int> cmdHist[2];    // cmd at every ms (for the latency)
    double vcmd[2] = {0, 0};        // command voltage after the latency, V
    double net[2] = {0, 0};         // net voltage actually applied, V
    double applied[2] = {0, 0};     // terminal voltage: back-EMF + net (what get_voltage reports)
    bool coasting[2] = {false, false};

    std::map<int, RotationDev> rot; // by port
    std::map<int, ImuDev> imu;
    std::map<int, MotorPort> motors;

    // odometry scheduling
    bool odomRunning = false;
    int tasksCreated = 0;
    uint32_t odomUpdates = 0;
    uint32_t odomFreshUpdates = 0;  // updates that consumed a new vertical-sensor sample
    uint32_t lastVertSeqSeen = 0;
    bool odomStrobe = false;        // lemlib::update() ran at `now`

    // hooks
    std::function<void(uint32_t)> preTick;
    std::function<void(uint32_t)> postEvents;
    std::function<void(uint32_t)> onDelay; // DUT called pros::delay at `now`

    void reset();
    void advance(uint32_t ms);
    void tick();
    void physics(double dt);
    void events();
    void refreshRotation(RotationDev& r);
    double motorPositionRaw(const MotorPort& m) const; // in the port's encoder units, since tare
    double motorRpm(int side) const;
    int vertPort() const;
};

World& world();

} // namespace sim
