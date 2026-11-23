// The simulated robot ("world") and the virtual-time scheduler.
//
// Time: integer ms. The DUT runs in the bench's only thread; pros::delay(n) calls World::advance(n),
// which runs n ticks. One tick from t to t+1:
//   1. preTick(t) hook (the bench records the row for time t: the DUT's code at t has already run)
//   2. 1 ms of physics
//   3. now = t+1, then the events due at t+1, in this order: sensor refreshes (IMU, rotation
//      sensors, then the deviceRefresh hook: the GPS model in route mode), lemlib::update() when
//      odometry is running and (t+1 - odomPhase) % 10 == 0, then the postEvents(t+1) hook (the
//      bench's PID-tab sampler, or route mode's logger).
// Shoves (route mode) move the robot during step 2, after the physics.
// So anything scheduled at a millisecond happens before DUT code that runs in that millisecond.
#pragma once
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>
#include "rng.hpp"

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
    double MU_LAT = 100.0;      // sideways (scrub) friction coefficient; sideways grip = MU_LAT * 386.09 in/s^2.
                                // The default never lets go, so drives and turns are unchanged
    double V_MAX = 12.0;        // volts at command 127
    double STICK_EPS = 1e-3;    // in/s: below this a side counts as stopped
    // geometry
    double VERT_OFFSET = 0.9;   // vertical tracking wheel, in right of centre
    double HORIZ_OFFSET = 2.55; // horizontal tracking wheel, in in front of centre
    double TRACK_WHEEL_D = 2.125;
    double DRIVE_WHEEL_D = 2.75;
    double DRIVE_RPM = 450;     // wheel rpm at motor free speed (600 rpm cartridge)
    // odometry error sources (docs/GPS_FUSION_DESIGN.md, acceptance tests). All zero by default: perfect sensors,
    // and then not one existing result changes
    double IMU_SCALE_ERR_CW = 0;  // the IMU reads (1 + this) times every clockwise heading change (0.012 = 1.2 %)
    double IMU_SCALE_ERR_CCW = 0; // ... and every counter-clockwise one
    double IMU_DRIFT_DEG_S = 0;   // the IMU's heading creeps this many degrees per second (clockwise +)
    double VERT_SCALE_ERR = 0;    // the vertical tracking wheel reads (1 + this) times its real travel
    double HORIZ_SCALE_ERR = 0;   // the horizontal one
    double SLIP_PER_IN = 0;       // chance, per inch a tracking wheel rolls, that it slips (each wheel separately)
    double SLIP_MAX_IN = 0;       // a slip loses between 0 and this many inches of the wheel's travel
    // A tracking wheel that is not parallel to the robot (the UMBmark square, tools/sim/umb_suite.py): it rolls
    // along a line turned this many degrees clockwise (seen from above) from the one LemLib assumes, so it reads
    // part of the motion across it. 0 by default, and then the wheels read exactly what they did before
    double VERT_YAW_DEG = 0;      // the vertical wheel: + reads some rightward motion as forward
    double HORIZ_YAW_DEG = 0;     // the horizontal wheel: + reads some forward motion as leftward
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

// A shove: the robot is pushed without its tracking wheels turning (and without the IMU seeing a turn)
struct Shove {
    uint32_t startMs = 0;      // world time
    int lengthMs = 200;        // spread evenly over this long
    double rightIn = 0;        // how far, in the robot's frame at the moment it starts: + to the robot's right
    double forwardIn = 0;      // + forward
    // filled in when it starts: the same push in field coordinates
    bool started = false;
    double dxIn = 0, dyIn = 0;
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
    double vSide = 0;               // sideways speed in the robot frame, in/s, + to the robot's right
    double sideDist = 0;            // integral of |vSide|, in
    double peakSide = 0;            // max |vSide|, in/s
    double travel[2] = {0, 0};      // per-side ground travel, in
    double vertTravel = 0;          // vertical tracking wheel: + forward
    double horizTravelLeft = 0;     // horizontal tracking wheel: + to the robot's left (LemLib)

    // odometry error sources (Params IMU_* / *_SCALE_ERR / SLIP_*; all off by default)
    double imuHeading = 0;          // what the IMU believes the heading is, rad (= heading with a perfect IMU)
    double vertSlip = 0;            // travel the vertical wheel lost to slips, in (signed like vertTravel)
    double horizSlip = 0;           // the horizontal wheel's
    uint32_t slips = 0;             // how many slips so far
    std::vector<Shove> shoves;
    Rng rng;                        // slips draw from this (seeded by the bench's --seed)

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
    std::function<void(uint32_t)> deviceRefresh; // more sensors (the GPS model), every ms, just before odometry

    void reset();
    void placeAt(double xIn, double yIn, double headingRad); // put the robot somewhere (route mode), at rest
    bool imuHasErrors() const;
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
