// Route mode: the bench drives a list of moves in field coordinates with the real LemLib motions, against a
// world whose odometry can drift and a simulated GPS (bench/gps_model.*) that can misbehave, with GPS fusion
// (src/lemlib/chassis/gpsFusion.cpp) Off, in Shadow or On. Optionally up to two wall-facing distance sensors
// (bench/dist_model.*) feed the same filter's wall fixes (docs/WALL_FIX_DESIGN.md). See tools/sim/README.md,
// "Route mode" and "Wall fixes".
//
// Route file: one command per line, # starts a comment. Field coordinates: inches from the field centre,
// +x toward the 90 degree wall, +y toward the 0 degree wall, heading 0 facing the 0 degree wall, clockwise.
//   start X Y HEADING                         where the robot truly starts (first line; setPose is told this)
//   point X Y [back] [timeout=MS] [max=N] [min=N] [early=IN]          Chassis::moveToPoint
//   turn HEADING [cw|ccw] [timeout=MS] [max=N]     Chassis::turnToHeading (cw/ccw: that way round; default the
//                                                  shorter way, which for exactly 180 deg depends on noise)
//   pose X Y HEADING [back] [timeout=MS] [lead=L] [max=N] [drift=D]   Chassis::moveToPose
//   wait MS                                                            motors stopped (BRAKE)
// Every move is blocking (async false), one after another, as a routine runs them.
#pragma once
#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

namespace lemlib {
class Chassis;
}

namespace sim {

// A shove (World::shoves) given on the command line: at a route time, or when a route step starts
struct ShoveSpec {
        bool atStep = false; // true: when step `step` (1 = the first command after start) begins
        int step = 0;
        std::uint32_t routeMs = 0; // otherwise: this long after the route started
        int lengthMs = 200;
        double rightIn = 0, forwardIn = 0;
};

// A view loss of the GPS, in route time
struct LossSpec {
        std::uint32_t routeMs = 0;
        std::uint32_t lengthMs = 0;
        int drift = -1; // 1 the output drifts, 0 it freezes, -1 as the model's loss_drift says
        double driftInS = -1; // drift rate, in/s; below 0: the model's blind_drift_in_s
};

// A wall sensor as the filter is told about it (--wall-sensor): docs/WALL_FIX_DESIGN.md, "Geometry". Where the
// simulated sensor TRULY is comes from the distance model's own options (--dist sN_right_in=... and so on), so a
// test can tell the filter a wrong mount
struct WallSensorSpec {
        double rightIn = 0, forwardIn = 0, yawDeg = 0; // inches right, inches forward (to the face), degrees
        double scale = 1, offsetIn = 0; // the bench calibration the filter applies: m = scale * mm / 25.4 + offset
};

// A keep-out given on the command line (--keep-out WALL:FROM:TO)
struct KeepOutSpec {
        int wall = -1;
        double fromIn = 0, toIn = 0;
};

struct RouteOptions {
        std::string routeFile;
        std::uint64_t seed = 1;
        std::string fusion = "off"; // off, shadow or on
        bool noGps = false; // no sensor at all: fusion is configured with no sensor (nullptr)
        std::vector<std::pair<std::string, double>> gpsSets; // --gps KEY=VALUE
        std::vector<std::string> gpsJsonFiles; // --gps-json FILE
        std::vector<std::pair<std::string, double>> fusionSets; // --fz KEY=VALUE (gpsfusion::Config names)
        std::vector<std::string> fusionJsonFiles; // --fz-json FILE
        std::vector<LossSpec> losses; // --gps-loss START_MS:LENGTH_MS[:1 drift | 0 freeze[:RATE_IN_PER_S]]
        std::vector<ShoveSpec> shoves; // --shove
        double startErrXIn = 0, startErrYIn = 0, startErrDeg = 0; // setPose is told the truth plus this
        int startWaitMs = 0; // stand still this long after switching fusion on, before the first step
        std::string csv; // one row per odometry tick (10 ms)
        std::string poseTrace; // as in the other modes: t and the exact pose after every lemlib::update()
        std::string compareWith; // the CSV of a fusion-Off run with the same seed: compare motor commands
        bool summaryJson = false;
        double recoverTolIn = 2; // shove recovery: back within this many inches
        // --walls-zero: with no --wall-sensor, call configureWalls() anyway with zero sensors (wall test W1: that
        // must change nothing). Off by default, and then the filter is set up exactly as before
        bool configureZeroWalls = false;
        // Wall fixes (docs/WALL_FIX_DESIGN.md). With no --wall-sensor none of this is used and every output is
        // exactly what it was before wall sensors existed in the bench
        std::vector<WallSensorSpec> wallSensors; // --wall-sensor, in order: sensor 0 (port 6), sensor 1 (port 21)
        std::vector<std::pair<std::string, double>> wallSets; // --wall KEY=VALUE (gpsfusion::WallConfig names)
        std::vector<std::string> wallJsonFiles; // --wall-json FILE
        std::vector<std::pair<int, int>> wallTrusts; // --wall-trust WALL|all:ignore|clear|solid (wall -1 = all)
        std::vector<KeepOutSpec> keepOuts; // --keep-out WALL:FROM:TO
        std::vector<std::pair<std::string, double>> distSets; // --dist KEY=VALUE (the distance model's options)
        std::vector<std::string> distJsonFiles; // --dist-json FILE
};

// Runs the route on the robot the bench built, writes the outputs, returns the process exit code
int runRoute(const RouteOptions& options, lemlib::Chassis& chassis, FILE* out);

// Read numeric arrays from a flat JSON object, e.g. "view_losses": [1000, 2000, 5000, 2000]
std::vector<double> jsonNumberArray(const std::string& text, const std::string& key);

} // namespace sim
