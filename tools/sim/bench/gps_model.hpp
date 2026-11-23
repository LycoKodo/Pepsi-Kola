// The simulated V5 GPS sensor, driven by the simulated world (bench/world.cpp). Route mode only.
//
// What it models (docs/GPS_FUSION_DESIGN.md, "What the research says"):
//   - a camera at a set place on the robot, facing a set direction (default: rear-facing, at the centre)
//   - a new output every period_ms, showing where the camera was latency_ms ago
//   - white noise plus slowly varying noise (first order, time constant slow_tau_s), and extra noise that grows
//     with the distance from the camera to the wall it faces
//   - a dead zone: closer than dead_zone_in to the wall it faces, the camera cannot read the strip. The output
//     then stops following the truth and dead-reckons with a drift, with NO error flag (as the real sensor does).
//     When it can see again the output comes back to the truth over recover_tau_s
//   - outliers: with probability outlier_prob an output is off by outlier_min_in to outlier_max_in
//   - scheduled view losses (start, length): the output freezes or drifts, also with no error flag
//   - "no strips at all": the output never follows the truth (frozen, or dead-reckoning from where the sensor
//     thinks it is at boot)
//   - a heading with its own noise, plus the mount yaw (a rear-facing camera reports the robot's heading + 180)
//   - get_error(): a number loosely related to the real error
//   - a mirrored field frame, for the wrong-alliance test
// Added for the revision 2 tests (docs/GPS_FUSION_DESIGN.md, "Acceptance tests, revision 2"), all off by default:
//   - strips on only some walls (strip_walls): the camera is blind unless the wall it faces has a strip
//   - a constant bias on the reported position (bias_x_in, bias_y_in)
//   - a sensor that never saw a strip but starts at the true position (no_strip_at_truth)
//   - view losses with their own drift rate (GpsViewLoss::driftInS, --gps-loss START:LENGTH:1:RATE)
//
// Every number is set with --gps KEY=VALUE or --gps-json FILE. Random numbers come from its own seeded generator,
// and every output uses the same count of them whatever the settings, so turning one effect on does not
// reshuffle the others (an A6 run sees the same noise as the A3 run with the same seed).
#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include "rng.hpp"

namespace sim {

struct World;

struct GpsModelParams {
        // where the camera truly is on the robot (the filter has its own idea of this in its Config)
        double mount_right_in = 0; // inches right of the robot's centre
        double mount_forward_in = 0; // inches in front of the centre
        double mount_yaw_deg = 180; // which way the camera faces, degrees clockwise from the robot's front
        // timing
        double period_ms = 40; // a new output every this many ms (about 25 a second)
        double phase_ms = 0; // outputs happen when (t - phase_ms) is a multiple of period_ms
        double latency_ms = 60; // each output shows where the camera was this long ago
        // noise on x and y (each axis), inches
        double white_sd_in = 0.5; // new every output
        double slow_sd_in = 0.75; // slowly varying part: settles to this standard deviation
        double slow_tau_s = 0.5; // and changes over about this long
        double wall_sd_per_100in = 0.4; // extra white noise per 100 in between the camera and the wall it faces
        double heading_sd_deg = 1.0; // noise on the reported heading
        // dead zone (camera too close to the wall it faces)
        double dead_zone_in = 18; // research: roughly 13 to 20 in
        double blind_drift_in_s = 1.5; // how fast the output drifts away while blind, random direction each time
        double recover_tau_s = 0.25; // after being blind, the output returns to the truth with this time constant
        // outliers
        double outlier_prob = 0; // chance that one output is an outlier (research: about 1 in 10)
        double outlier_min_in = 6; // its size, uniform between these, random direction
        double outlier_max_in = 24;
        // scheduled view losses (the list itself is set with --gps-loss): 1 = the output drifts, 0 = it freezes.
        // A loss can also say for itself (--gps-loss START:LENGTH:MODE)
        double loss_drift = 1;
        // no strips at all: 0 = normal, 1 = the output is frozen, 2 = it dead-reckons and drifts from where the
        // sensor thinks it is at boot (it never saw a strip, so it does not know)
        double no_strips = 0;
        double no_strip_x_in = 0; // where a sensor that never saw a strip thinks it is: PROS's default, the centre
        double no_strip_y_in = 0;
        double no_strip_heading_deg = 0;
        // field frame: 0 = right, 1 = turned 180 deg (x, y -> -x, -y), 2 = mirrored left-right (x -> -x),
        // 3 = mirrored front-back (y -> -y). For the wrong-alliance test
        double mirror = 0;
        // get_error(): the real error times a random factor from 0.5 to 1.5, plus this, in metres
        double error_floor_m = 0.005;
        // Which walls have a code strip, added up: 1 = the 0 degree wall (+y), 2 = the 90 degree wall (+x),
        // 4 = the 180 degree wall (-y), 8 = the 270 degree wall (-x). 15 = all four, as on a Skills field. The
        // camera can read only the wall it faces (the wall its centre line meets): facing a wall with no strip it
        // is blind and dead-reckons with a drift, as in the dead zone, with no error flag
        double strip_walls = 15;
        // a constant error on the reported position, inches, field frame (strips put up a little off, say). It is
        // added to every output that follows the camera (seeing or blind), not to a no-strips sensor's, and
        // get_error() does not know about it
        double bias_x_in = 0;
        double bias_y_in = 0;
        // no_strips 1 or 2: 1 = the sensor believes it booted exactly where the camera really was, facing the way
        // it really faced (someone told it the start pose), instead of at no_strip_x_in / _y_in / _heading_deg.
        // With no_strips 2 it then follows the robot and drifts: a working sensor in every way but the drift
        double no_strip_at_truth = 0;
        // ---- attack options for the revision 3 A10 rows, ported from the revision 2 attacker's bench
        // (rev2_review/breaker2). All 0 = off, and then the model and its JSON are exactly as before.
        // Blind drift shape: 0 = the drift runs all the time (as before); 2 = it runs only while the robot stands
        // still (the camera's true speed under 0.5 in/s over the 50 ms before the reading): a sensor that lies
        // only when odometry cannot be checked against anything. (The attacker's modes 1 and 3 are not ported.)
        double drift_mode = 0;
        // a bias that grows in steps: bias_step_in more every bias_step_ms from route time bias_step_start_ms, at
        // most bias_step_count steps, in field direction bias_step_dir_deg (clockwise from +y). Added like
        // bias_x_in: to every output that follows the camera, after get_error (which does not know about it)
        double bias_step_in = 0, bias_step_ms = 0, bias_step_start_ms = 0, bias_step_count = 0, bias_step_dir_deg = 0;

        bool set(const std::string& key, double value); // false if there is no such key
        std::string toJson() const;
};

// One scheduled loss of view, in world time (ms)
struct GpsViewLoss {
        std::uint32_t startMs = 0;
        std::uint32_t lengthMs = 0;
        int drift = -1; // 1 the output drifts, 0 it freezes, -1 as loss_drift says
        double driftInS = -1; // how fast it drifts, in/s, in a random direction; below 0: blind_drift_in_s (and then
                              // a loss that starts while the camera is already blind keeps that drift)
};

// What the sensor sees, for each output
enum GpsState {
    GPS_SEES = 0, // reading the strips: the output follows the truth (with noise)
    GPS_DEAD_ZONE = 1, // too close to the wall it faces: dead-reckoning
    GPS_VIEW_LOST = 2, // a scheduled view loss
    GPS_NO_STRIPS = 3, // never sees a strip at all
    GPS_STRIPLESS_WALL = 4 // faces a wall with no strip (strip_walls): dead-reckoning, as in the dead zone
};

class GpsModel {
    public:
        GpsModelParams p;
        std::vector<GpsViewLoss> losses;
        Rng rng;

        // Called by the world every ms, before odometry: records the truth and makes a new output when one is due
        void refresh(const World& w);

        // what the sensor reports right now, in PROS's units
        double outXMetres() const;
        double outYMetres() const;
        double outHeadingDeg() const { return outHeading; }
        double outErrorMetres() const { return outError; }

        // for the logs. Inches and degrees, field frame
        double outXIn() const { return outX; }
        double outYIn() const { return outY; }
        bool hasOutput() const { return outputs > 0; }
        std::uint32_t outputCount() const { return outputs; }
        bool outlierNow() const { return outlier; }
        double outlierSizeNow() const { return outlierSize; }
        GpsState stateNow() const { return state; }
        // the reported position minus where the camera was when the reading was taken (latency_ms ago), inches:
        // the sensor's own error, which a filter that knows the delay exactly would see
        double errorVsDelayedIn() const { return errDelayed; }
        // the reported position minus where the camera is now, inches
        double errorVsNowIn(const World& w) const;
        // how far the camera is from the wall it faces now, inches
        double wallDistanceNowIn(const World& w) const;

        // counts over the run
        std::uint32_t outlierCount = 0;
        std::uint32_t deadZoneOutputs = 0;
        std::uint32_t lostOutputs = 0;
        std::uint32_t striplessOutputs = 0;
        double sumSquaredErrorSeeing = 0; // errorVsDelayed^2 summed over outputs while seeing, not outliers
        std::uint32_t seeingOutputs = 0;

    private:
        struct Truth {
                double x, y, heading; // robot centre, inches; heading radians clockwise from +y
        };
        std::vector<Truth> history; // one per ms, index = world time
        bool started = false;

        // noise and blindness state
        double slowX = 0, slowY = 0;
        double blindX = 0, blindY = 0; // offset built up while blind, decays once the camera sees again
        double heldNoiseX = 0, heldNoiseY = 0; // while blind the output is smooth: the noise is held
        double driftX = 0, driftY = 0; // this blind period's drift, in/s
        bool wasBlind = false;
        int lossIndex = -1; // the view loss the last output was in (index into losses), or -1
        double noStripX = 0, noStripY = 0; // no_strips 2: the dead-reckoned position
        Truth bootTruth {0, 0, 0};

    public:
        std::uint32_t t0Ms = 0; // world time of route time 0 (set by route.cpp), for bias_step_start_ms

    private:

        // the current output
        std::uint32_t outputs = 0;
        double outX = 0, outY = 0, outHeading = 0, outError = 0;
        bool outlier = false;
        double outlierSize = 0;
        double errDelayed = 0;
        GpsState state = GPS_SEES;
        std::uint32_t lastOutputMs = 0;

        Truth truthAt(long ms) const;
        void makeOutput(std::uint32_t now);
        const GpsViewLoss* viewLostAt(std::uint32_t ms) const; // the loss going on at ms, or nullptr
};

// The one simulated GPS of the bench
GpsModel& gpsModel();

// Distance, inches, from (x, y) to the wall of the 144 in square field along headingRad (clockwise from +y).
// Zero on or outside the wall. Written separately from the filter's own ray cast on purpose
double wallDistanceIn(double xIn, double yIn, double headingRad);

// Which wall that ray meets, as a strip_walls number: 1 = +y, 2 = +x, 4 = -y, 8 = -x (0 on or outside the wall).
// A ray straight into a corner counts as meeting the y wall
int wallFacedBit(double xIn, double yIn, double headingRad);

} // namespace sim
