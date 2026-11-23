// The simulated V5 distance sensor looking at the field walls, for the host testbench (tools/sim). It is what the
// wall fixes of docs/WALL_FIX_DESIGN.md are tested against.
//
// It is given the time and the robot's TRUE pose (inches from the field centre, +x toward the 90 degree wall,
// +y toward the 0 degree wall, heading in radians clockwise from +y: the frame of the design) and says what each
// sensor reads, in PROS's units: { mm, confidence 0 to 63, object size }.
//
// What it models. Every effect has an option (DistModelParams, KEY=VALUE, see helpText()) and is off, or does no
// harm, by default:
//   - up to 2 sensors, each with its own mount (right, forward, yaw) on the robot
//   - the sensor's own update rate (about 30 a second): a read between two updates repeats the last value
//   - a delay between where the robot was and what the reading shows (latency_ms, off by default)
//   - the four TRUE wall planes, settable per wall, so the real field can differ from what the filter is told
//   - a beam that is a cone, not a line: "centre ray", "nearest point in the cone", or a blend of the two (the
//     default; dist_model.cpp says why). A corner, the neighbouring wall, an object or a wall-mounted element
//     inside the cone changes the reading in the nearest and blend models
//   - noise that grows with range (new every reading, and a slowly varying part), a scale error and an offset
//     per sensor, readings rounded to whole mm
//   - the 20 mm lower limit, 9999 when nothing is in range (2000 mm), a random dropout
//   - a confidence that falls with range and with the angle to the surface, and means nothing below 200 mm
//   - an object size that grows with how much of the cone the target fills
//   - objects: circles fixed on the field, circles there only between two times, and an object put in the beam
//     whenever the robot is nearly stopped, at a chosen distance in front of the sensor or of the wall (test W4)
//   - clear walls: the beam sometimes or always passes through and reads something behind the wall, reads the wall
//     with lower confidence otherwise, and gets no return beyond a set range (test W5)
//   - wall-mounted elements: a stretch of a wall that sticks out, so it reads closer (keep-out test W7)
//
// Random numbers: each sensor has its own generator, and every reading draws the same count of numbers whatever
// the options are. So switching one effect on never changes what another effect does, and switching the second
// sensor on never changes the first sensor's readings. A seed always gives exactly the same readings.
//
// Not modelled (README.md says more): the vertical spread of the cone (floor, top of the wall), the colour or
// shine of a surface, other robots as anything but circles, object velocity, the sensor's own filtering.
//
// Everything here is a guess built from community reports, not from measurements of this sensor.
#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include "rng.hpp"

namespace sim {

constexpr int DIST_MAX_SENSORS = 2; // the robot has two free smart ports (6 and 21): two wall sensors at most
constexpr int DIST_WALL_COUNT = 4; // 0 = +y (the 0 degree wall), 1 = +x (90), 2 = -y (180), 3 = -x (270)
constexpr int DIST_MAX_ELEMENTS = 8; // wall-mounted elements; the filter keeps at most 8 keep-outs
constexpr int DIST_MAX_OBJECTS = 16; // circles on the field
constexpr std::int32_t DIST_PROS_ERR = 2147483647; // PROS_ERR (INT32_MAX): what an unplugged port returns
constexpr std::int32_t DIST_NOTHING_MM = 9999; // what PROS returns when the sensor detects nothing

// One sensor: where it is on the robot and how wrong its readings are
struct DistSensorParams {
        double enabled = 0; // 1 = this sensor is fitted. 0 = not fitted: it reads PROS_ERR, as an empty port does
        double right_in = 0; // the sensor's face, inches right of the robot's tracking centre
        double forward_in = 0; // ... and inches in front of it
        double yaw_deg = 0; // which way it points, degrees clockwise from the robot's front (180 = back, 90 = right)
        double scale_err = 0; // the sensor reads (1 + this) times the true range (0.02 = reads 2 percent long)
        double offset_mm = 0; // ... plus this many mm
        double phase_ms = 0; // its readings come when (t - phase_ms) is a multiple of period_ms
};

// One wall: where it truly is, and whether it is clear plastic
struct DistWallParams {
        double in = 70.25; // the wall's inside face, inches from the field centre (140.5 in wall to wall)
        double clear = 0; // 1 = a clear plastic panel: the effects below apply. 0 = solid
        double through_prob = 0; // clear: the chance, each reading, that the beam passes through (1 = always)
        double behind_in = 12; // clear, passing through: it reads something this many inches behind the wall...
        double behind_spread_in = 0; // ...plus a random 0 to this many inches, new each reading (0 = steady)
        double conf_drop = 10; // clear: confidence is this much lower, whether it reads the wall or through it
        double dropout_in = 0; // clear: farther than this (inches) from the sensor, no return at all (0 = never)
};

// A wall-mounted element: a stretch of a wall that sticks out into the field (a box against the wall)
struct DistElementParams {
        double wall = -1; // which wall (0 to 3, numbered as above); -1 = no element
        double from_in = 0; // where it starts and ends along the wall, field inches: x for walls 0 and 2, y for
        double to_in = 0; //   walls 1 and 3
        double depth_in = 0; // how far it sticks out from the wall's face, inches: it reads this much closer
};

// A circle on the field (a game object, a goal post, a robot seen from above)
struct DistObjectParams {
        double x = 0; // centre, field inches
        double y = 0;
        double r = 0; // radius, inches; 0 = no object
        double from_ms = 0; // there from this time (the time given to update())...
        double to_ms = -1; // ...until this time; below 0 = forever
};

struct DistModelParams {
        // ---- timing
        double period_ms = 1000.0 / 30.0; // a new reading every this many ms (about 30 a second); 0 = every update()
        double latency_ms = 0; // a reading shows where the robot was this long ago (0 = no delay; at most 500)
        // ---- the cone
        double cone_model = 2; // 0 = centre ray, 1 = nearest point in the cone, 2 = a blend of the two
        double cone_blend = 0.5; // blend: 0 = nearest point, 1 = centre ray, 0.5 = halfway
        double cone_half_deg = 12; // half-angle of the cone (research: about 12 degrees at long range)...
        double cone_half_close_deg = 18; // ...and about 18 degrees at short range, where the sensor switches...
        double cone_close_mm = 200; // ...below this range (along the centre ray), mm
        // ---- range limits
        double min_mm = 20; // closer than this the sensor cannot measure...
        double below_min_value = 0; // ...and reads this (0: VEXos's "no valid reading"; a guess)
        double max_mm = 2000; // farther than this it reads 9999
        // ---- noise, mm, 1 standard deviation
        double noise_mm = 0; // new every reading: this fixed part...
        double noise_frac = 0; // ...plus this fraction of the range (0.01 = 1 percent)
        double slow_mm = 0; // slowly varying part, settles to this fixed part...
        double slow_frac = 0; // ...plus this fraction of the range...
        double slow_tau_s = 2; // ...and changes over about this long (it does not average out over one stop)
        double dropout_prob = 0; // the chance, each reading, that it reads 9999 for no reason
        double dropout_incidence_deg = 0; // the centre ray more than this off square to the surface: 9999 (0 = never)
        // ---- confidence (0 to 63; PROS: meaningful only above 200 mm)
        double conf_model = 1; // 1 = falls with range and angle as below; 0 = always 63 above 200 mm
        double conf_per_m = 10; // confidence lost per metre of range
        double conf_per_deg = 0.5; // confidence lost per degree off square
        double conf_sd = 0; // random part, new every reading
        double conf_below_200 = 10; // what it reads at 200 mm or closer (the PROS C header says 10)
        // ---- object size (0 to 400; -1 when nothing is detected)
        double size_full = 100; // the size of a target that fills the whole cone (a wall); a guess
        double size_sd = 0; // random part, new every reading
        // ---- an object in the beam whenever the robot is nearly stopped (test W4)
        double stop_obj = 0; // 0 = off, 1 = a distance in front of the SENSOR, 2 = a distance in front of the WALL
        double stop_obj_min_in = 6; // the distance is drawn between these at each stop, then held for the stop
        double stop_obj_max_in = 20; //   (min = max for a fixed distance); measured along the centre ray
        double stop_obj_gap_in = 3; // mode 1: the object stays at least this far in front of the wall (the top of the
                                    // draw is lowered to fit; none if even min does not fit). Revision A's W4 says
                                    // objects 3 in or more in front of the wall must never give a fix
        double stop_obj_r_in = 2; // the object is a circle of this radius, its near side on the centre ray
        double stop_obj_sensors = 3; // which sensors, added up: 1 = sensor 0, 2 = sensor 1, 3 = both
        double stop_obj_from_ms = 0; // no stop object before this time (the time given to update()); 0 = from the
                                     // start. Lets a test keep the first stop clean (added in the bench)
        double stop_speed_in_s = 2; // "nearly stopped": slower than this...
        double stop_turn_deg_s = 6; // ...turning slower than this...
        double stop_window_ms = 50; // ...for this long. Looser than the filter's "still": the object is there first

        DistSensorParams sensor[DIST_MAX_SENSORS] = {
            // sensor 0: facing back, on the centre line, its face 0.5 in inside the rear of an 18 in robot
            {1, 0.0, -8.5, 180, 0, 0, 0},
            // sensor 1 (not fitted unless switched on): facing right, at the right side, on the centre line
            {0, 8.5, 0.0, 90, 0, 0, 0},
        };
        DistWallParams wall[DIST_WALL_COUNT];
        DistElementParams elem[DIST_MAX_ELEMENTS];
        DistObjectParams obj[DIST_MAX_OBJECTS];

        // KEY=VALUE. Plain keys are the fields above by name; the rest are numbered: sN_<field> (N = 0, 1),
        // wallN_<field> (N = 0 to 3), elemN_<field> (N = 0 to 7), objN_<field> (N = 0 to 15). false = no such key
        bool set(const std::string& key, double value);
        bool get(const std::string& key, double& value) const; // false = no such key
        std::string toJson() const; // every key with its value (elements and objects only when used)
        static std::string helpText(); // one line per option, with its default
        static std::vector<std::string> allKeys(); // every key set() accepts
};

// What one read returns, as PROS gives it
struct DistReading {
        std::int32_t mm = 0; // distance, mm (9999 nothing in range, PROS_ERR not fitted)
        std::int32_t confidence = 0; // 0 to 63
        std::int32_t objectSize = -1; // 0 to 400, -1 when it cannot tell
};

// What a reading really is, for the logs. When several apply, the first in this order wins: OFF; a random
// dropout (NO_RETURN); the sensor's face on or past a wall or inside an object (TOO_CLOSE); the other NO_RETURN
// reasons; closer than min_mm (TOO_CLOSE); THROUGH, STOP_OBJECT, OBJECT, ELEMENT, CORNER, CLEAR_WALL, WALL.
// "Comes from" means the target of the centre ray (cone_model 0) or of the nearest point in the cone (1 and 2)
enum DistState {
    DIST_WALL = 0, // a plain wall: the true wall geometry (plus noise and the sensor's own errors)
    DIST_CLEAR_WALL = 1, // a clear wall, read as the wall (lower confidence)
    DIST_CORNER = 2, // the reading is pulled by a wall other than the one the centre ray meets (a corner in the cone)
    DIST_ELEMENT = 3, // the reading comes from a wall-mounted element
    DIST_OBJECT = 4, // ... from a circle object
    DIST_STOP_OBJECT = 5, // ... from the object put in the beam while the robot is stopped
    DIST_THROUGH = 6, // the beam passed through a clear wall and read something behind it
    DIST_NO_RETURN = 7, // 9999 (see DistNoReturn for why)
    DIST_TOO_CLOSE = 8, // closer than min_mm: reads below_min_value
    DIST_OFF = 9 // not fitted: PROS_ERR
};

// Why a reading is 9999
enum DistNoReturn {
    DIST_NR_NONE = 0,
    DIST_NR_OUT_OF_RANGE = 1, // farther than max_mm
    DIST_NR_CLEAR_DROPOUT = 2, // a clear wall farther than its dropout_in
    DIST_NR_TOO_SLANTED = 3, // more than dropout_incidence_deg off square
    DIST_NR_RANDOM = 4 // dropout_prob
};

// What a ray met
enum DistHitKind {
    DIST_HIT_NONE = 0,
    DIST_HIT_WALL = 1,
    DIST_HIT_ELEMENT = 2,
    DIST_HIT_OBJECT = 3,
    DIST_HIT_STOP_OBJECT = 4
};

// Bits of a "cone mask": what the rays inside a cone meet first
constexpr int DIST_CONE_WALL0 = 1; // wall 0 (+y); wall N is DIST_CONE_WALL0 << N
constexpr int DIST_CONE_WALLS = 15; // all four wall bits
constexpr int DIST_CONE_ELEMENT = 16; // any wall-mounted element
constexpr int DIST_CONE_OBJECT = 32; // any circle object
constexpr int DIST_CONE_STOP_OBJECT = 64; // the stop object

// Everything about one sensor's latest reading, for logs and tests. Ranges are inches along the ray, before
// noise, scale and offset
struct DistOutputInfo {
        std::uint32_t timeMs = 0; // when the reading was made (the time given to update())
        std::uint32_t count = 0; // readings so far, this one included
        DistState state = DIST_OFF;
        DistNoReturn noReturn = DIST_NR_NONE;
        double sensorX = 0, sensorY = 0; // where the sensor's face was, field inches
        double beamDeg = 0; // which way the centre ray pointed, degrees clockwise from +y, 0 to 360
        double coneHalfDeg = 0; // the half-angle used for this reading
        int centreKind = DIST_HIT_NONE, centreIndex = -1; // what the centre ray met first (index: wall, element...)
        int nearKind = DIST_HIT_NONE, nearIndex = -1; // what the nearest point in the cone belongs to
        double centreIn = 0; // range along the centre ray
        double nearestIn = 0; // range to the nearest point in the cone
        double wallIn = 0; // range along the centre ray to the walls and elements alone (no objects)
        double modelIn = 0; // what the cone model makes of it (plus what is behind a clear wall)
        double incidenceDeg = 0; // angle between the centre ray and the surface it meets, 0 = square
        int coneMask = 0; // DIST_CONE_* bits: what any ray in the cone met first
        double fill = 0; // fraction of the cone's evenly spread rays that meet the reported target
        bool stopObject = false; // the stop object is in place for this reading
        double stopObjectDistIn = 0; // its drawn distance (in front of the sensor or of the wall); -1 = no room
        bool through = false; // passed through a clear wall
};

class DistModel {
    public:
        DistModelParams p;

        DistModel() { reseed(1); }

        // A new seed: each sensor's generator is seeded from it and the sensor's number
        void reseed(std::uint64_t seed);
        // Forget every reading, the pose history and the stop objects (keeps the options and the generators)
        void reset();

        // Call with the time (ms) and the TRUE robot pose (inches, heading radians clockwise from +y), every ms
        // for exact timing. A sensor makes a new reading when one is due; between readings nothing changes
        void update(std::uint32_t nowMs, double xIn, double yIn, double headingRad);

        // What a sensor reads now (its latest reading). Before the first update(): 0, confidence 0, size -1
        DistReading read(int sensor) const;
        const DistOutputInfo& info(int sensor) const;
        bool nearlyStopped() const { return stopped; }

        // Geometry helpers that use the TRUE walls, elements and the objects there at nowMs (no noise, no state
        // change). Where sensor `sensor` would be at that pose, and what the rays in a cone of halfDeg meet first:
        // DIST_CONE_* bits. Two wall bits = the cone crosses a corner. For test W7's bookkeeping
        int coneMaskAt(std::uint32_t nowMs, double xIn, double yIn, double headingRad, int sensor,
                       double halfDeg) const;
        // The noise-free reading the cone model would give at that pose, inches (no update rate, no state change)
        double modelRangeAt(std::uint32_t nowMs, double xIn, double yIn, double headingRad, int sensor) const;

    private:
        struct Hit {
                double t = 0; // range along the ray, inches (INFINITY = nothing)
                int kind = DIST_HIT_NONE;
                int index = -1;
                double nx = 0, ny = 0; // the surface's normal where the ray meets it
        };
        struct Box {
                double x0, x1, y0, y1;
                int index;
        };
        struct Circle {
                double x, y, r;
                int kind; // DIST_HIT_OBJECT or DIST_HIT_STOP_OBJECT
                int index;
        };
        struct Scene {
                double wallPos[DIST_WALL_COUNT]; // each wall plane's distance from the centre (positive)
                Box box[DIST_MAX_ELEMENTS];
                int boxes = 0;
                Circle circle[DIST_MAX_OBJECTS + DIST_MAX_SENSORS];
                int circles = 0;
        };
        struct ConeResult {
                Hit centre, nearest; // what the centre ray meets, and the nearest point in the cone
                int mask = 0; // DIST_CONE_* bits
                int fillCentre = 0; // how many of the evenly spread rays meet the centre ray's target...
                int fillNearest = 0; // ...and the nearest point's target
        };
        struct PoseSample {
                std::uint32_t t;
                double x, y, h;
        };
        struct StopObject {
                bool active = false; // the robot is in a stop and the object has been decided
                bool placed = false; // ...and it is really there (it fits in front of the wall)
                double x = 0, y = 0, r = 0, distIn = 0;
        };
        struct SensorState {
                double nextDueMs = -1e300; // the first update() always makes a reading
                std::uint32_t outputs = 0;
                std::uint32_t lastOutputMs = 0;
                double slow = 0; // the slowly varying noise, in standard deviations
                DistReading reading;
                DistOutputInfo info;
                StopObject stop;
        };

        static constexpr int RING_SIZE = 512; // pose history: 512 updates (512 ms when called every ms)

        Rng rng[DIST_MAX_SENSORS];
        SensorState st[DIST_MAX_SENSORS];
        PoseSample ring[RING_SIZE];
        int ringNext = 0, ringCount = 0;
        bool haveLast = false;
        PoseSample last {0, 0, 0, 0};
        bool slowNow = false; // slower than the stop thresholds since the last update
        std::uint32_t slowSinceMs = 0;
        bool stopped = false;

        void pushPose(std::uint32_t nowMs, double x, double y, double h);
        PoseSample poseAt(double tMs) const;
        void makeReading(int i, std::uint32_t nowMs);
        void sensorPose(int i, double x, double y, double h, double& sx, double& sy, double& phi) const;
        Scene scene(std::uint32_t nowMs, int stopObjectsOf) const; // stopObjectsOf: a sensor number, or -1 for none
        static bool insideField(const Scene& s, double sx, double sy);
        static Hit castRay(const Scene& s, double sx, double sy, double heading, bool withObjects);
        static ConeResult cone(const Scene& s, double sx, double sy, double phi, double halfRad);
        double halfAngleFor(double centreIn) const;
        double blendRange(const ConeResult& c) const;
};

} // namespace sim
