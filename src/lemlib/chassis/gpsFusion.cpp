// GPS-aided odometry: the filter (GpsFusionCore, no PROS in it) and, at the bottom, the robot's side that reads
// the pros::Gps sensor. The specification is docs/GPS_FUSION_DESIGN.md: "step 1" to "step 6" and "check 1" to
// "check 7" below are its numbers, and "R1" to "R4" are the sections of its "Revision 2" (which wins where it
// disagrees with the text before it). Where the design does not say, the most cautious choice was made; those
// places say "(not in the design)".
//
// Wall fixes (distance sensors looking at the field walls) are a second kind of measurement in the same filter.
// Their specification is docs/WALL_FIX_DESIGN.md, whose "Revision A" wins over the text before it; "wall check 1"
// to "wall check 13" are its checks, and "Geometry" is its section of that name. Where it does not say, the
// cautious choice was made again, marked "(not in the wall design)".
//
// The host unit tests (tools/sim/tests) compile this file with GPSFUSION_CORE_ONLY defined, which leaves out
// the robot's side, so they test exactly the filter the robot runs.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include "lemlib/chassis/gpsFusion.hpp"

namespace lemlib::gpsfusion {

namespace {
constexpr float PI = 3.14159265358979f;

// (SPEED_WINDOW_MS, check 6's window, is in gpsFusion.hpp: MAX_LATENCY_MS is worked out from it)

// How unsure odometry is when fusion is switched on. Revision 2 started it at 0 (SIGMA_AT_SWITCH_ON_IN): nothing
// is corrected before the start check passes, and when it passes sigma is set to startSigmaIn (R3), so it only
// showed in the status until then. Revision A of the wall design starts both doubts at startSigmaIn instead,
// because a wall fix can come before any GPS start check (a robot with wall sensors and no GPS never has one).
// In a run with only the GPS nothing reads the doubt before its start check, so that changes only the status
// before then. See resetState()

// If the odometry task was held up, feed in (or, after a fault, feed back out) at most this much time's worth of
// correction in one tick, so a late tick never turns into a jump (not in the design)
constexpr float MAX_FEED_DT_S = 0.02f;

struct Vec {
        float x;
        float y;
};

float degToRadF(float deg) { return deg * PI / 180; }

float radToDegF(float rad) { return rad * 180 / PI; }

// an angle in degrees, wrapped into -180 .. 180
float wrapDeg(float deg) {
    float wrapped = std::fmod(deg + 180.0f, 360.0f);
    if (wrapped < 0) wrapped += 360.0f;
    return wrapped - 180.0f;
}

float length(float x, float y) { return std::sqrt(x * x + y * y); }

// Did a value read from the GPS change since the last read? A NaN never equals anything, not even itself, so
// without the second part a sensor stuck on NaN would look new on every tick
bool changed(double now, double before) { return now != before && !(std::isnan(now) && std::isnan(before)); }

// Where the sensor is relative to the robot's centre, in field coordinates, when the robot faces thetaRad.
// Heading is clockwise from +y, so the robot's forward is (sin, cos) and its right is (cos, -sin)
Vec mountOffset(const Config& config, float thetaRad) {
    const float s = std::sin(thetaRad);
    const float c = std::cos(thetaRad);
    return {config.mountRightIn * c + config.mountForwardIn * s, -config.mountRightIn * s + config.mountForwardIn * c};
}

// R3: of n readings judged, this many failing the heading check is a fault. The design says 8 of 10, and to keep
// it at 80 percent, rounded up, when n is changed: (4n + 4) / 5 is 80 percent of n rounded up, in whole numbers
int headingFailuresForFault(int n) { return (4 * n + 4) / 5; }

// ---- wall fixes (docs/WALL_FIX_DESIGN.md)

constexpr float MM_PER_INCH = 25.4f;
// Wall check 1: the V5 distance sensor is rated from 20 to 2000 mm
constexpr std::int32_t WALL_MIN_VALID_MM = 20;
constexpr std::int32_t WALL_MAX_VALID_MM = 2000;
// Wall check 2: the sensor only measures its confidence above 200 mm (below, PROS returns a fixed 10), and
// reports it from 0 to 63. A confidence outside 0 to 63 is a failed read, and fails wall check 1 (not in the
// wall design)
constexpr std::int32_t WALL_CONFIDENCE_ABOVE_MM = 200;
constexpr std::int32_t WALL_MAX_CONFIDENCE = 63;
// Wall check 4 compares readings "in a row". If the newest one kept is older than this when a new one comes,
// readings were missed (odometry held up, or the walls waiting after a GPS fault), so the row starts again (not
// in the wall design). Readings come every 40 ms (sampleMinPeriodMs 35 at 10 ms ticks)
constexpr float WALL_STEADY_MAX_GAP_MS = 100;

// The inches from a wall sensor's face to the point it looks from, in field coordinates: the mount (right,
// forward) turned by the robot's heading. "Geometry": S = (x + r cos t + f sin t, y - r sin t + f cos t)
Vec wallSensorOffset(const WallSensorMount& mount, float thetaRad) {
    const float s = std::sin(thetaRad);
    const float c = std::cos(thetaRad);
    return {mount.rightIn * c + mount.forwardIn * s, -mount.rightIn * s + mount.forwardIn * c};
}

// The coordinate a wall fixes: walls 1 and 3 (x = +-plane) fix x (0), walls 0 and 2 fix y (1)
int axisOfWall(int wall) { return (wall == WALL_PLUS_X || wall == WALL_MINUS_X) ? 0 : 1; }

// The farthest a wall is used (wall check 7), by its trust
float maxRangeOf(const WallConfig& walls, int wall) {
    return walls.trust[wall] == WallTrust::Solid ? walls.maxRangeSolidIn : walls.maxRangeClearIn;
}

// The largest single correction from a wall (wall check 12), by its trust
float capOf(const WallConfig& walls, int wall) {
    return walls.trust[wall] == WallTrust::Solid ? walls.capSolidIn : walls.capClearIn;
}

// The least confidence above 200 mm (wall check 2), by the trust of the wall the beam is predicted to hit. With no
// single wall, or an Ignore wall, the reading will be dropped by wall check 5 anyway; the stricter number is used
// so that it is not counted as confident (not in the wall design)
int minConfidenceFor(const WallConfig& walls, int wall) {
    if (wall != NO_WALL && walls.trust[wall] == WallTrust::Solid) return walls.minConfidence;
    if (wall != NO_WALL && walls.trust[wall] == WallTrust::Clear) return walls.minConfidenceClear;
    return std::max(walls.minConfidence, walls.minConfidenceClear);
}
} // namespace

const char* modeName(Mode mode) {
    switch (mode) {
        case Mode::Off: return "OFF";
        case Mode::Shadow: return "SHADOW";
        case Mode::On: return "ON";
    }
    return "?";
}

const char* faultText(Fault fault) {
    switch (fault) {
        case Fault::None: return "none";
        case Fault::StartDisagree: return "start pose and GPS disagree";
        case Fault::RelocTooFar: return "GPS and odometry too far apart to relocalise";
        case Fault::RelocTooMany: return "too many relocalisations: the GPS is probably wrong";
        case Fault::TotalTooLarge: return "total GPS correction too large: the GPS is probably wrong";
        case Fault::HeadingDisagree: return "GPS heading disagrees: field turned, frame mirrored or mount yaw wrong";
    }
    return "?";
}

const char* faultName(Fault fault) {
    switch (fault) {
        case Fault::None: return "none";
        case Fault::StartDisagree: return "StartDisagree";
        case Fault::RelocTooFar: return "RelocTooFar";
        case Fault::RelocTooMany: return "RelocTooMany";
        case Fault::TotalTooLarge: return "TotalTooLarge";
        case Fault::HeadingDisagree: return "HeadingDisagree";
    }
    return "?";
}

int formatStatusLine(char* out, unsigned size, std::uint32_t nowMs, float poseXIn, float poseYIn, float thetaRad,
                     const Status& s) {
    // [GPSFUSE] t=<ms> mode=<OFF|SHADOW|ON> pose=<x>,<y>,<heading deg> (what the motions read)
    //   est=<x>,<y> (best guess: pose + pending, + the shadow correction in Shadow) sigma=<in>
    //   pend=<length of pending, in> fed=<length of the total fed in (Shadow: would have), in; after a fault it
    //   counts down to 0> start=<readings>/<needed>, or ok once the start check passed acc=<accepted>
    //   drop=<check 1>/<2>/<3>/<4>/<5>/<5b>/<6>/<7>/<statistical gate> reloc=<n>
    //   nohist=<readings dropped because the history did not reach back to when they were taken: after the
    //   first few, a latencyMs too long for the history> reloc=<n>
    //   inn=<right>,<forward> (average innovation in the robot's frame, in: a steady value is a mount error)
    //   nis=<average innovation^2 / (sigma^2 + sigmaGps^2): about 1.4 for a healthy sensor in the simulator, about
    //   6 for one noisier than sigmaGpsIn; the real healthy value comes from the stage 1 logs> fault=<name>
    // About 215 characters at most in a 60 s run
    char start[24];
    if (s.startCheckPassed) std::snprintf(start, sizeof start, "ok");
    else std::snprintf(start, sizeof start, "%d/%d", s.startSamplesSeen, s.startSamplesNeeded);
    const float fedIn = length(s.appliedXIn + s.shadowXIn, s.appliedYIn + s.shadowYIn); // one of the two is zero
    return std::snprintf(out, size,
                         "[GPSFUSE] t=%lu mode=%s pose=%.1f,%.1f,%.1f est=%.1f,%.1f sigma=%.2f pend=%.1f fed=%.1f "
                         "start=%s acc=%lu drop=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu nohist=%lu reloc=%lu inn=%.1f,%.1f "
                         "nis=%.1f fault=%s",
                         (unsigned long)nowMs, modeName(s.mode), poseXIn, poseYIn, radToDegF(thetaRad), s.estimateXIn,
                         s.estimateYIn, s.sigmaIn, length(s.pendingXIn, s.pendingYIn), fedIn, start,
                         (unsigned long)s.accepted, (unsigned long)s.droppedByCheck[CHECK_FINITE],
                         (unsigned long)s.droppedByCheck[CHECK_IN_FIELD],
                         (unsigned long)s.droppedByCheck[CHECK_NOT_STALE],
                         (unsigned long)s.droppedByCheck[CHECK_DEAD_ZONE], (unsigned long)s.droppedByCheck[CHECK_HEADING],
                         (unsigned long)s.droppedBySensorError, (unsigned long)s.droppedByCheck[CHECK_SLOW],
                         (unsigned long)s.droppedByCheck[CHECK_RATE], (unsigned long)s.droppedByGate,
                         (unsigned long)s.droppedNoHistory, (unsigned long)s.relocalisations, s.innovationRightAvgIn,
                         s.innovationForwardAvgIn, s.nisAvg, s.faultName);
}

float distanceToWallIn(float xIn, float yIn, float headingRad) {
    // on or outside the wall (written so that a NaN also lands here)
    if (!(std::fabs(xIn) < FIELD_HALF_WIDTH_IN) || !(std::fabs(yIn) < FIELD_HALF_WIDTH_IN)) return 0;
    if (!std::isfinite(headingRad)) return 0;
    // heading is clockwise from +y: the ray goes (sin, cos)
    const float dirX = std::sin(headingRad);
    const float dirY = std::cos(headingRad);
    // how far along the ray it meets a wall at x = +-72, and a wall at y = +-72. It hits the nearer one first.
    // A ray parallel to a pair of walls never meets them
    float toXWall = INFINITY;
    float toYWall = INFINITY;
    if (dirX > 0) toXWall = (FIELD_HALF_WIDTH_IN - xIn) / dirX;
    if (dirX < 0) toXWall = (-FIELD_HALF_WIDTH_IN - xIn) / dirX;
    if (dirY > 0) toYWall = (FIELD_HALF_WIDTH_IN - yIn) / dirY;
    if (dirY < 0) toYWall = (-FIELD_HALF_WIDTH_IN - yIn) / dirY;
    return std::min(toXWall, toYWall);
}

WallHit castToWall(const float planeIn[NUM_WALLS], float fromXIn, float fromYIn, float dirX, float dirY) {
    WallHit hit;
    // a sensor on or outside a wall sees no wall we know of (written so that a NaN also lands here)
    if (!(fromYIn < planeIn[WALL_PLUS_Y]) || !(fromXIn < planeIn[WALL_PLUS_X]) ||
        !(fromYIn > -planeIn[WALL_MINUS_Y]) || !(fromXIn > -planeIn[WALL_MINUS_X]))
        return hit;
    // "Cast the ray from S along d to the four planes": how far along the ray it meets the x wall it is heading
    // for (x = +planeIn[1] or x = -planeIn[3]) and the y wall (y = +planeIn[0] or y = -planeIn[2]). A ray
    // parallel to a pair of walls never meets them
    float toXWall = INFINITY;
    int xWall = NO_WALL;
    if (dirX > 0) {
        toXWall = (planeIn[WALL_PLUS_X] - fromXIn) / dirX;
        xWall = WALL_PLUS_X;
    }
    if (dirX < 0) {
        toXWall = (-planeIn[WALL_MINUS_X] - fromXIn) / dirX;
        xWall = WALL_MINUS_X;
    }
    float toYWall = INFINITY;
    int yWall = NO_WALL;
    if (dirY > 0) {
        toYWall = (planeIn[WALL_PLUS_Y] - fromYIn) / dirY;
        yWall = WALL_PLUS_Y;
    }
    if (dirY < 0) {
        toYWall = (-planeIn[WALL_MINUS_Y] - fromYIn) / dirY;
        yWall = WALL_MINUS_Y;
    }
    // "the nearest hit is the wall". Exactly the same distance to both is a ray into a corner: it belongs to
    // neither wall, so it is no wall at all (not in the wall design: the cautious answer)
    if (toXWall < toYWall) {
        hit.wall = xWall;
        hit.rangeIn = toXWall;
        hit.alongIn = fromYIn + toXWall * dirY;
    } else if (toYWall < toXWall) {
        hit.wall = yWall;
        hit.rangeIn = toYWall;
        hit.alongIn = fromXIn + toYWall * dirX;
    }
    return hit;
}

const char* wallSensorStateName(WallSensorState state) {
    switch (state) {
        case WallSensorState::Unused: return "-";
        case WallSensorState::WaitingForStart: return "WAIT";
        case WallSensorState::Working: return "OK";
        case WallSensorState::StartFault: return "START";
        case WallSensorState::HealthOff: return "HEALTH";
    }
    return "?";
}

int formatWallStatusLine(char* out, unsigned size, std::uint32_t nowMs, const Status& s) {
    // [WALLFIX] t=<ms> walls=<run | wait: the GPS faulted and what it fed in is being fed back out | stop: the
    //   total limit stopped the walls too> sig=<doubt in x>,<doubt in y>
    //   then per sensor: | <number> <state: WAIT, OK, START (start fault), HEALTH (switched off)> fix=<fixes>
    //   drop=<wall check 1>/<2>/.../<13> inn=<last innovation, in> cor=<last correction, in>
    // About 190 characters with two busy sensors. Built piece by piece in a buffer here and then copied, so that a
    // short `out` is cut, not overrun, and the return value is the whole line's length, as snprintf's would be
    constexpr int LINE_SIZE = 640;
    char line[LINE_SIZE];
    int used = std::snprintf(line, LINE_SIZE, "[WALLFIX] t=%lu walls=%s sig=%.2f,%.2f", (unsigned long)nowMs,
                             s.wallsStopped ? "stop" : (s.wallsWaiting ? "wait" : "run"), s.sigmaXIn, s.sigmaYIn);
    for (int i = 0; i < s.wallSensorCount && i < MAX_WALL_SENSORS; i++) {
        const WallSensorStatus& w = s.walls[i];
        int at = std::clamp(used, 0, LINE_SIZE - 1); // each piece goes at the end of what fits so far
        used += std::snprintf(line + at, LINE_SIZE - at, " | %d %s fix=%lu drop=", i, wallSensorStateName(w.state),
                              (unsigned long)w.fixes);
        for (int c = 0; c < NUM_WALL_CHECKS; c++) {
            at = std::clamp(used, 0, LINE_SIZE - 1);
            used += std::snprintf(line + at, LINE_SIZE - at, c == 0 ? "%lu" : "/%lu",
                                  (unsigned long)w.droppedByCheck[c]);
        }
        at = std::clamp(used, 0, LINE_SIZE - 1);
        used += std::snprintf(line + at, LINE_SIZE - at, " inn=%+.2f cor=%+.2f", w.lastInnovationIn,
                              w.lastCorrectionIn);
    }
    std::snprintf(out, size, "%s", line);
    return used;
}

// ------------------------------------------------------------------------------------------------------------
// Checking the numbers (Revision 3)
// ------------------------------------------------------------------------------------------------------------
// Each rule is one number and the range it must be in. The ranges are wide on purpose: they catch a number in the
// wrong units, a typo or a value that switches a safety limit off, not a number that is merely untuned. Where a
// reviewer measured what a wrong value does, it is in the comment. A NaN is outside every range.

namespace {
struct FloatRule {
        float Config::*field;
        const char* name;
        float lo, hi;
};
struct IntRule {
        int Config::*field;
        const char* name;
        int lo, hi;
};

constexpr FloatRule CONFIG_FLOAT_RULES[] = {
    {&Config::mountRightIn, "mountRightIn", -18, 18}, // the sensor is on an 18 in robot
    {&Config::mountForwardIn, "mountForwardIn", -18, 18},
    {&Config::mountYawDeg, "mountYawDeg", -360, 360}, // degrees, not radians: any whole turn is accepted
    // above 4 the gate is about 12 in wide: outliers get in and nothing relocalises
    {&Config::sigmaGpsIn, "sigmaGpsIn", 0.3f, 4},
    {&Config::kDist, "kDist", 0, 0.1f}, // 10 percent of the distance driven is already a broken odometry
    {&Config::kTurn, "kTurn", 0, 2},
    {&Config::sigmaMaxIn, "sigmaMaxIn", 0.5f, 8}, // revision 1's 12 let the gate grow to 30 in
    {&Config::latencyMs, "latencyMs", 0, MAX_LATENCY_MS}, // refused, not only warned about: see latencyRefused()
    {&Config::maxSpeedInPerS, "maxSpeedInPerS", 0.5f, 24}, // 0: no reading is ever used
    {&Config::maxTurnRateDegPerS, "maxTurnRateDegPerS", 1, 90},
    {&Config::minPeriodMs, "minPeriodMs", 40, 1000}, // under 40 (the sensor's update period) check 7 does nothing
    {&Config::staleMs, "staleMs", 40, 1000}, // under 40 every reading looks stale: fusion never uses one
    {&Config::minViewIn, "minViewIn", 15, 60}, // under 15 readings from the camera's dead zone are used
    {&Config::headingGateDeg, "headingGateDeg", 1, 30}, // 180 would mean the heading fault can never fire
    {&Config::maxSensorErrorIn, "maxSensorErrorIn", 0.5f, 12}, // 0 (off) is fine too; 0.05 is metres, not inches
    {&Config::gate, "gate", 1.5f, 4},
    {&Config::slewInPerS, "slewInPerS", 0.5f, 4}, // faster makes the pose jump under the motions (A5)
    {&Config::maxStepIn, "maxStepIn", 0.1f, 2}, // 0: corrections only by relocalisation; large: no step limit
    {&Config::maxTotalCorrectionIn, "maxTotalCorrectionIn", 1, 12}, // larger takes away R1's limit
    {&Config::startTolIn, "startTolIn", 1, 12}, // larger: a wrong alliance or start tile passes the start check
    {&Config::startSigmaIn, "startSigmaIn", 0.1f, 5},
    {&Config::relocWindowMs, "relocWindowMs", 200, 3000}, // larger: readings far apart in time "agree"
    {&Config::relocSpreadIn, "relocSpreadIn", 0.5f, 6}, // larger: unrelated readings "agree"
    {&Config::relocMaxIn, "relocMaxIn", 1, 12},
};
constexpr IntRule CONFIG_INT_RULES[] = {
    {&Config::startSamples, "startSamples", 1, 20},
    {&Config::startHeadingSamples, "startHeadingSamples", 1, 64}, // 0: the heading fault is never judged
    {&Config::relocSamples, "relocSamples", 3, MAX_RELOC_SAMPLES}, // 1: one wrong reading moved the pose 10 in
    {&Config::relocMaxCount, "relocMaxCount", 0, 3}, // large: a GPS that keeps "shoving" the robot never faults
};
constexpr int NUM_CONFIG_FLOAT_RULES = int(sizeof CONFIG_FLOAT_RULES / sizeof CONFIG_FLOAT_RULES[0]);
constexpr int NUM_CONFIG_INT_RULES = int(sizeof CONFIG_INT_RULES / sizeof CONFIG_INT_RULES[0]);
static_assert(NUM_CONFIG_FLOAT_RULES + NUM_CONFIG_INT_RULES <= 32, "one bit per rule in a 32-bit mask");

struct WallFloatRule {
        float WallConfig::*field;
        const char* name;
        float lo, hi;
};
struct WallIntRule {
        int WallConfig::*field;
        const char* name;
        int lo, hi;
};
// The distance sensor reads 20 to 2000 mm: 0.8 to 78.7 in
constexpr float DISTANCE_SENSOR_MAX_IN = 78.7f;
constexpr WallFloatRule WALL_FLOAT_RULES[] = {
    {&WallConfig::wallNoiseIn, "wallNoiseIn", 0.05f, 2},
    {&WallConfig::wallNoiseFrac, "wallNoiseFrac", 0, 0.1f},
    {&WallConfig::sigmaHeadingDeg, "sigmaHeadingDeg", 0.1f, 5},
    {&WallConfig::wallMaxSpeedInPerS, "wallMaxSpeedInPerS", 0.1f, 4},
    {&WallConfig::wallMaxTurnRateDegPerS, "wallMaxTurnRateDegPerS", 0.5f, 15},
    // longer than the history: the robot never counts as still, so no wall fix ever happens
    {&WallConfig::stillWindowMs, "stillWindowMs", 20, (HISTORY_SIZE - 1) * ODOM_TICK_MS},
    {&WallConfig::steadySpreadIn, "steadySpreadIn", 0.05f, 2},
    {&WallConfig::steadySpreadFrac, "steadySpreadFrac", 0, 0.1f},
    {&WallConfig::sampleMinPeriodMs, "sampleMinPeriodMs", 10, 200},
    {&WallConfig::maxIncidenceDeg, "maxIncidenceDeg", 1, 20},
    {&WallConfig::minRangeIn, "minRangeIn", 0.8f, 12},
    {&WallConfig::maxRangeClearIn, "maxRangeClearIn", 1, DISTANCE_SENSOR_MAX_IN},
    {&WallConfig::maxRangeSolidIn, "maxRangeSolidIn", 1, DISTANCE_SENSOR_MAX_IN},
    {&WallConfig::coneHalfDeg, "coneHalfDeg", 5, 30},
    {&WallConfig::footprintMarginIn, "footprintMarginIn", 0, 12},
    {&WallConfig::cornerMarginIn, "cornerMarginIn", 0, 24},
    {&WallConfig::blockedMinIn, "blockedMinIn", 0.1f, 6},
    {&WallConfig::blockedMaxIn, "blockedMaxIn", 0.1f, 12},
    {&WallConfig::blockedSigmas, "blockedSigmas", 1, 5},
    {&WallConfig::throughMaxIn, "throughMaxIn", 0.5f, 12},
    {&WallConfig::throughSigmas, "throughSigmas", 1, 6},
    {&WallConfig::capClearIn, "capClearIn", 0.1f, 6},
    {&WallConfig::capSolidIn, "capSolidIn", 0.1f, 12},
    {&WallConfig::refixTravelIn, "refixTravelIn", 1, 48},
    {&WallConfig::refixTurnDeg, "refixTurnDeg", 1, 90},
    {&WallConfig::sigmaFloorIn, "sigmaFloorIn", 0.05f, 2}, // 0: repeated fixes make the filter overconfident
    {&WallConfig::wallStartTolIn, "wallStartTolIn", 0.5f, 12},
};
constexpr WallIntRule WALL_INT_RULES[] = {
    {&WallConfig::minConfidence, "minConfidence", 0, WALL_MAX_CONFIDENCE},
    {&WallConfig::minConfidenceClear, "minConfidenceClear", 0, WALL_MAX_CONFIDENCE},
    {&WallConfig::steadyCount, "steadyCount", 2, MAX_STEADY_COUNT},
    {&WallConfig::healthMinTries, "healthMinTries", 2, 100},
    {&WallConfig::keepOutCount, "keepOutCount", 0, MAX_KEEP_OUTS},
};
// The four wall planes are one rule (any of them outside): the field is 140.5 in inside, so about 70.25 from
// the centre; a plane in metres or in millimetres is caught
constexpr float WALL_PLANE_MIN_IN = 60;
constexpr float WALL_PLANE_MAX_IN = 72;
constexpr int NUM_WALL_FLOAT_RULES = int(sizeof WALL_FLOAT_RULES / sizeof WALL_FLOAT_RULES[0]);
constexpr int NUM_WALL_INT_RULES = int(sizeof WALL_INT_RULES / sizeof WALL_INT_RULES[0]);
static_assert(NUM_WALL_FLOAT_RULES + NUM_WALL_INT_RULES + 1 <= 64, "one bit per rule in a 64-bit mask");

// (written so that a NaN is outside)
bool outside(float value, float lo, float hi) { return !(value >= lo && value <= hi); }
} // namespace

const int NUM_CONFIG_PROBLEMS = NUM_CONFIG_FLOAT_RULES + NUM_CONFIG_INT_RULES;
const int NUM_WALL_CONFIG_PROBLEMS = NUM_WALL_FLOAT_RULES + NUM_WALL_INT_RULES + 1;

bool latencyRefused(const Config& config) { return outside(config.latencyMs, 0, MAX_LATENCY_MS); }

std::uint32_t checkConfig(const Config& config) {
    std::uint32_t problems = 0;
    for (int i = 0; i < NUM_CONFIG_FLOAT_RULES; i++) {
        const FloatRule& rule = CONFIG_FLOAT_RULES[i];
        const float value = config.*rule.field;
        // check 5b is off at 0, which is fine
        if (rule.field == &Config::maxSensorErrorIn && value == 0) continue;
        if (outside(value, rule.lo, rule.hi)) problems |= std::uint32_t(1) << i;
    }
    for (int i = 0; i < NUM_CONFIG_INT_RULES; i++) {
        const IntRule& rule = CONFIG_INT_RULES[i];
        const int value = config.*rule.field;
        if (value < rule.lo || value > rule.hi) problems |= std::uint32_t(1) << (NUM_CONFIG_FLOAT_RULES + i);
    }
    return problems;
}

int formatConfigProblem(char* out, unsigned size, int problem) {
    if (problem >= 0 && problem < NUM_CONFIG_FLOAT_RULES) {
        const FloatRule& rule = CONFIG_FLOAT_RULES[problem];
        if (rule.field == &Config::latencyMs)
            return std::snprintf(out, size, "latencyMs is outside 0 to %g: the history cannot reach that far back, "
                                 "so fusion stays OFF", double(rule.hi));
        if (rule.field == &Config::maxSensorErrorIn)
            return std::snprintf(out, size, "maxSensorErrorIn is outside %g to %g (or 0 = off): inches, not metres",
                                 double(rule.lo), double(rule.hi));
        return std::snprintf(out, size, "%s is outside %g to %g", rule.name, double(rule.lo), double(rule.hi));
    }
    if (problem >= NUM_CONFIG_FLOAT_RULES && problem < NUM_CONFIG_PROBLEMS) {
        const IntRule& rule = CONFIG_INT_RULES[problem - NUM_CONFIG_FLOAT_RULES];
        return std::snprintf(out, size, "%s is outside %d to %d", rule.name, rule.lo, rule.hi);
    }
    return std::snprintf(out, size, "unknown problem %d", problem);
}

std::uint64_t checkWallConfig(const WallConfig& config) {
    std::uint64_t problems = 0;
    for (int i = 0; i < NUM_WALL_FLOAT_RULES; i++) {
        const WallFloatRule& rule = WALL_FLOAT_RULES[i];
        if (outside(config.*rule.field, rule.lo, rule.hi)) problems |= std::uint64_t(1) << i;
    }
    for (int i = 0; i < NUM_WALL_INT_RULES; i++) {
        const WallIntRule& rule = WALL_INT_RULES[i];
        const int value = config.*rule.field;
        if (value < rule.lo || value > rule.hi) problems |= std::uint64_t(1) << (NUM_WALL_FLOAT_RULES + i);
    }
    for (int wall = 0; wall < NUM_WALLS; wall++)
        if (outside(config.planeIn[wall], WALL_PLANE_MIN_IN, WALL_PLANE_MAX_IN))
            problems |= std::uint64_t(1) << (NUM_WALL_CONFIG_PROBLEMS - 1);
    return problems;
}

int formatWallConfigProblem(char* out, unsigned size, int problem) {
    if (problem >= 0 && problem < NUM_WALL_FLOAT_RULES) {
        const WallFloatRule& rule = WALL_FLOAT_RULES[problem];
        return std::snprintf(out, size, "%s is outside %g to %g", rule.name, double(rule.lo), double(rule.hi));
    }
    if (problem >= NUM_WALL_FLOAT_RULES && problem < NUM_WALL_FLOAT_RULES + NUM_WALL_INT_RULES) {
        const WallIntRule& rule = WALL_INT_RULES[problem - NUM_WALL_FLOAT_RULES];
        return std::snprintf(out, size, "%s is outside %d to %d", rule.name, rule.lo, rule.hi);
    }
    if (problem == NUM_WALL_CONFIG_PROBLEMS - 1)
        return std::snprintf(out, size, "a planeIn is outside %g to %g", double(WALL_PLANE_MIN_IN),
                             double(WALL_PLANE_MAX_IN));
    return std::snprintf(out, size, "unknown problem %d", problem);
}

// ------------------------------------------------------------------------------------------------------------
// GpsFusionCore
// ------------------------------------------------------------------------------------------------------------

GpsFusionCore::GpsFusionCore(const Config& config)
    : config(config) {
    resetState();
}

void GpsFusionCore::setConfig(const Config& newConfig) { config = newConfig; }

void GpsFusionCore::setWallConfig(const WallConfig& newWallConfig, const WallSensorMount mounts[], int count) {
    wallConfig = newWallConfig;
    wallConfig.keepOutCount = std::clamp(newWallConfig.keepOutCount, 0, MAX_KEEP_OUTS);
    wallSensorCount = mounts == nullptr ? 0 : std::clamp(count, 0, MAX_WALL_SENSORS);
    for (int i = 0; i < wallSensorCount; i++) wallMounts[i] = mounts[i];
    resetWalls();
}

void GpsFusionCore::setMode(Mode newMode) {
    // switching on (Shadow or On) always starts from scratch, start check included. Off keeps the state for
    // the status; the pose keeps whatever was already fed in
    if (newMode != Mode::Off) {
        resetState();
        runMode = newMode;
    }
    mode = newMode;
}

Mode GpsFusionCore::getMode() const { return mode; }

void GpsFusionCore::resetState() {
    fault = Fault::None;
    unwinding = false;
    // Revision A of the wall design: both doubts start at startSigmaIn when fusion is switched on (kept within 0
    // and sigmaMaxIn, as R3's start check does (not in the design)). See the note at the top of this file
    sigmaXIn = sigmaYIn = floored(std::max(0.0f, std::min(config.startSigmaIn, config.sigmaMaxIn)));
    pendingXIn = pendingYIn = 0;
    fedInXIn = fedInYIn = 0;
    decidedXIn = decidedYIn = 0;
    historyCount = 0;
    historyNext = 0;
    haveLastRead = false;
    haveNewReading = false;
    haveUsedReading = false;
    startPassed = false;
    startCount = 0;
    startSumXIn = startSumYIn = 0;
    headingSeen = 0;
    headingFailed = 0;
    headingJudged = false;
    relocRun = 0;
    relocNext = 0;
    newReadings = 0;
    for (int i = 0; i < NUM_CHECKS; i++) droppedByCheck[i] = 0;
    droppedBySensorError = 0;
    droppedByGate = 0;
    droppedNoHistory = 0;
    accepted = 0;
    relocalisations = 0;
    maxReadingStepIn = 0;
    innovationCount = 0;
    innovationRightSumIn = innovationForwardSumIn = 0;
    nisCount = 0;
    nisSum = 0;
    lastInnovationXIn = lastInnovationYIn = 0;
    haveTick = false;
    estimateXIn = estimateYIn = 0;
    resetWalls();
}

void GpsFusionCore::resetWalls() {
    for (int i = 0; i < MAX_WALL_SENSORS; i++) {
        wallTrackers[i] = WallTracker();
        wallTrackers[i].status.state = i < wallSensorCount ? WallSensorState::WaitingForStart : WallSensorState::Unused;
    }
    wallsStopped = false;
}

float GpsFusionCore::floored(float sigmaIn) const {
    // Wall design: "A doubt never goes below sigmaFloorIn, so repeated fixes cannot make the filter
    // overconfident." Revision A: it belongs to the wall numbers and applies only while a wall sensor is
    // configured, so GPS-only results do not change
    return wallSensorCount > 0 ? std::max(wallConfig.sigmaFloorIn, sigmaIn) : sigmaIn;
}

bool GpsFusionCore::wallsCanCarryOn() const {
    if (wallSensorCount == 0 || wallsStopped) return false;
    for (int i = 0; i < wallSensorCount; i++) {
        const WallSensorState state = wallTrackers[i].status.state;
        if (state == WallSensorState::WaitingForStart || state == WallSensorState::Working) return true;
    }
    return false;
}

void GpsFusionCore::addToHistory(const HistoryEntry& entry) {
    history[historyNext] = entry;
    historyNext = (historyNext + 1) % HISTORY_SIZE;
    if (historyCount < HISTORY_SIZE) historyCount++;
}

const GpsFusionCore::HistoryEntry& GpsFusionCore::historyBack(int ticksBack) const {
    return history[(historyNext - 1 - ticksBack + 2 * HISTORY_SIZE) % HISTORY_SIZE];
}

const GpsFusionCore::HistoryEntry* GpsFusionCore::entryAtLeastThisOld(std::uint32_t nowMs, float ageMs) const {
    // newest first, so the first one old enough is the newest one old enough
    for (int back = 0; back < historyCount; back++) {
        const HistoryEntry& entry = historyBack(back);
        if (float(nowMs - entry.timeMs) >= ageMs) return &entry;
    }
    return nullptr;
}

const GpsFusionCore::HistoryEntry* GpsFusionCore::entryClosestToAge(std::uint32_t nowMs, float ageMs) const {
    if (historyCount == 0) return nullptr;
    // the history must reach back at least ageMs, or the closest entry could be far too new
    if (float(nowMs - historyBack(historyCount - 1).timeMs) < ageMs) return nullptr;
    const HistoryEntry* best = nullptr;
    float bestDifferenceMs = INFINITY;
    for (int back = 0; back < historyCount; back++) {
        const HistoryEntry& entry = historyBack(back);
        const float differenceMs = std::fabs(float(nowMs - entry.timeMs) - ageMs);
        if (differenceMs < bestDifferenceMs) {
            best = &entry;
            bestDifferenceMs = differenceMs;
        }
    }
    return best;
}

int GpsFusionCore::ticksBackAtLeast(std::uint32_t refMs, float ageMs, int fromBack) const {
    // newest first, and only entries no newer than fromBack, so refMs - timeMs never goes below zero
    for (int back = std::max(0, fromBack); back < historyCount; back++)
        if (float(refMs - historyBack(back).timeMs) >= ageMs) return back;
    return -1;
}

void GpsFusionCore::speedBetween(const HistoryEntry& older, const HistoryEntry& newer, float& inPerS,
                                 float& degPerS) const {
    const float spanS = float(newer.timeMs - older.timeMs) / 1000.0f;
    const float movedXIn = (newer.xIn - newer.appliedXIn) - (older.xIn - older.appliedXIn);
    const float movedYIn = (newer.yIn - newer.appliedYIn) - (older.yIn - older.appliedYIn);
    inPerS = length(movedXIn, movedYIn) / spanS;
    degPerS = radToDegF(std::fabs(newer.thetaRad - older.thetaRad)) / spanS;
}

void GpsFusionCore::setFault(Fault why) {
    // Revision A of the wall design keeps faults in two stages.
    // Stage 1, and revision 2 exactly when there is no wall sensor: while the GPS is in use, a fault takes it out
    // for the run. The total limit counts the walls' corrections too; a total-limit fault counts as the GPS's
    // only if the GPS could have added to that total, that is once its start check has passed. Otherwise the
    // walls alone went over it, and that is stage 2 (not in the wall design: the cautious reading of "the GPS
    // already out (or never configured)" for a GPS that never started working)
    const bool gpsInUse = fault == Fault::None && (wallSensorCount == 0 || why != Fault::TotalTooLarge || startPassed);
    if (gpsInUse) {
        fault = why;
        // R1: nothing still pending is ever used. tick() feeds what was already fed in back out, at slewInPerS.
        // With wall sensors that can carry on, they wait until that is done, then carry on alone (tick())
        pendingXIn = pendingYIn = 0;
        unwinding = true;
        return;
    }
    // Without wall sensors there is nothing more to stop: the first fault is the one that counts
    if (wallSensorCount == 0 || wallsStopped) return;
    // Stage 2: a total-limit fault with the GPS already out (or never working) stops the walls as well, and
    // feeds their corrections back out. In a run with no GPS fault before it, it is the run's fault too
    wallsStopped = true;
    if (fault == Fault::None) fault = why;
    pendingXIn = pendingYIn = 0;
    unwinding = true;
}

void GpsFusionCore::decide(float xIn, float yIn) {
    pendingXIn += xIn;
    pendingYIn += yIn;
    decidedXIn += xIn;
    decidedYIn += yIn;
    // R1, total limit: if the total correction decided since switching on is longer than maxTotalCorrectionIn,
    // fault. A calibrated robot does not drift that far in a run, so the GPS is the likelier liar. (Written so
    // that a NaN also faults.) Revision A: it counts every correction decided, from the GPS and the walls alike
    if (!(length(decidedXIn, decidedYIn) <= config.maxTotalCorrectionIn)) setFault(Fault::TotalTooLarge);
}

Correction GpsFusionCore::tick(std::uint32_t nowMs, float xIn, float yIn, float thetaRad, const GpsSample* sample) {
    return tick(nowMs, xIn, yIn, thetaRad, sample, nullptr, 0);
}

Correction GpsFusionCore::tick(std::uint32_t nowMs, float xIn, float yIn, float thetaRad, const GpsSample* sample,
                               const DistanceSample* wallSamples, int wallSampleCount) {
    const Correction nothing;

    // Off: do nothing at all and change nothing
    if (mode == Mode::Off) return nothing;
    // a broken pose (odometry lost a sensor): build nothing on it (not in the design)
    if (!std::isfinite(xIn) || !std::isfinite(yIn) || !std::isfinite(thetaRad)) return nothing;

    // The pose the filter believes in, the same in Shadow as in On. In On the pose odometry hands us already
    // contains everything fed in so far (odometry added it); in Shadow it never does, so add it here
    float estXIn = xIn;
    float estYIn = yIn;
    if (mode == Mode::Shadow) {
        estXIn += fedInXIn;
        estYIn += fedInYIn;
    }

    // seconds since the last tick, for the feed-in. None on the first tick
    const float dtS = haveTick ? float(nowMs - lastTickMs) / 1000.0f : 0;

    // A fault: the filter does nothing more for the rest of the run (no doubt, no history, no readings). Only
    // the feed-back-out below still runs. Revision A of the wall design: after a GPS fault, wall sensors that
    // can still fix keep the filter running (stage 1); the GPS is out for the run all the same
    if (fault == Fault::None || wallsCanCarryOn()) {
        // Odometry's doubt grows with how far odometry carried the robot this tick:
        // sigma += kDist * |distance| + kTurn * |angle|, capped at sigmaMax. The corrections are taken out of both
        // poses first, because being corrected is not driving. The wall design keeps one doubt per axis; "both
        // grow by the same rule as before"
        if (historyCount > 0) {
            const HistoryEntry& previous = historyBack(0);
            const float movedXIn = (estXIn - fedInXIn) - (previous.xIn - previous.appliedXIn);
            const float movedYIn = (estYIn - fedInYIn) - (previous.yIn - previous.appliedYIn);
            const float turnedRad = std::fabs(thetaRad - previous.thetaRad);
            const float movedIn = length(movedXIn, movedYIn);
            sigmaXIn = std::min(sigmaXIn + config.kDist * movedIn + config.kTurn * turnedRad, config.sigmaMaxIn);
            sigmaYIn = std::min(sigmaYIn + config.kDist * movedIn + config.kTurn * turnedRad, config.sigmaMaxIn);
            // wall check 13: how far each wall sensor's robot has travelled and turned since its last fix
            for (int i = 0; i < wallSensorCount; i++) {
                wallTrackers[i].travelSinceFixIn += movedIn;
                wallTrackers[i].turnSinceFixRad += turnedRad;
            }
        }

        // remember this tick: the pose before this tick's feed-in, and what had been fed in before it
        HistoryEntry thisTick;
        thisTick.timeMs = nowMs;
        thisTick.xIn = estXIn;
        thisTick.yIn = estYIn;
        thisTick.thetaRad = thetaRad;
        thisTick.appliedXIn = fedInXIn;
        thisTick.appliedYIn = fedInYIn;
        addToHistory(thisTick);

        // steps 1 to 5, and the start check and relocalisation (any of which may fault). Not once the GPS is out
        if (fault == Fault::None && sample != nullptr) handleReading(nowMs, *sample, estXIn, estYIn, thetaRad);

        // the wall sensors, after the GPS, so that their prediction includes what the GPS decided this tick.
        // Revision A: they pause while what was fed in is being fed back out after a fault
        if (!unwinding && wallSamples != nullptr) {
            const int sensors = std::min(wallSampleCount, wallSensorCount);
            for (int i = 0; i < sensors; i++) handleWallSample(i, nowMs, wallSamples[i], estXIn, estYIn, thetaRad);
        }
    }

    // Step 6, feed-in: move at most slewInPerS * dt of the pending correction into the pose. Never a jump.
    // R1: after a fault (perhaps raised just now) the same limit feeds everything fed in so far back out instead,
    // until the total fed in is exactly zero and the pose is plain odometry again. Pending is never used then
    const float feedLimitIn = std::max(0.0f, config.slewInPerS * std::min(dtS, MAX_FEED_DT_S));
    float toGoXIn = pendingXIn;
    float toGoYIn = pendingYIn;
    if (unwinding) {
        toGoXIn = -fedInXIn;
        toGoYIn = -fedInYIn;
    }
    Correction step;
    const float toGoIn = length(toGoXIn, toGoYIn);
    if (toGoIn <= feedLimitIn) {
        // the rest fits in this tick: all of it
        step.xIn = toGoXIn;
        step.yIn = toGoYIn;
    } else {
        // the same direction, shortened to the most allowed
        const float shorten = feedLimitIn / toGoIn;
        step.xIn = toGoXIn * shorten;
        step.yIn = toGoYIn * shorten;
    }
    if (!unwinding) {
        pendingXIn -= step.xIn;
        pendingYIn -= step.yIn;
    }
    fedInXIn += step.xIn;
    fedInYIn += step.yIn;

    // Revision A, fault stage 1: "When the pose is plain odometry again, wall sensors that are healthy carry on
    // alone: both doubts are set to sigmaMaxIn, the total starts again from zero, and each sensor may fix again
    // at once." Readings kept for wall check 4 from before the fault are forgotten (not in the wall design)
    if (unwinding && fault != Fault::None && fedInXIn == 0 && fedInYIn == 0 && wallsCanCarryOn()) {
        unwinding = false;
        sigmaXIn = sigmaYIn = floored(config.sigmaMaxIn);
        decidedXIn = decidedYIn = 0;
        pendingXIn = pendingYIn = 0;
        for (int i = 0; i < wallSensorCount; i++) {
            wallTrackers[i].fixedThisStop = false;
            wallTrackers[i].steadyHave = 0;
        }
    }

    // the best guess of where the robot is: the pose after this tick plus what is still to come
    estimateXIn = estXIn + step.xIn + pendingXIn;
    estimateYIn = estYIn + step.yIn + pendingYIn;
    haveTick = true;
    lastTickMs = nowMs;

    // Shadow: the step was kept apart (in fedIn) and the pose is left exactly as odometry made it
    if (mode == Mode::On) return step;
    return nothing;
}

void GpsFusionCore::handleReading(std::uint32_t nowMs, const GpsSample& sample, float estXIn, float estYIn,
                                  float thetaRad) {
    // Step 1: is it new? A reading is new when x, y or heading differ from the previous read. The sensor
    // updates about 25 times a second and odometry reads it 100 times a second, so most reads repeat the last
    // one; only new readings go any further
    const bool isNew = !haveLastRead || changed(sample.xMetres, lastReadX) || changed(sample.yMetres, lastReadY) ||
                       changed(sample.headingDeg, lastReadHeading);
    if (!isNew) return;
    const bool hadNewReadingBefore = haveNewReading;
    const std::uint32_t previousNewMs = lastNewMs;
    haveLastRead = true;
    lastReadX = sample.xMetres;
    lastReadY = sample.yMetres;
    lastReadHeading = sample.headingDeg;
    haveNewReading = true;
    lastNewMs = nowMs;
    newReadings++;

    // Check 1: finite numbers. PROS reports a failed read as infinity. The error value is checked too (not in
    // the design): a failed read fills every field with infinity
    if (!std::isfinite(sample.xMetres) || !std::isfinite(sample.yMetres) || !std::isfinite(sample.headingDeg) ||
        !std::isfinite(sample.errorMetres)) {
        droppedByCheck[CHECK_FINITE]++;
        return;
    }
    const float sensorXIn = float(sample.xMetres * INCHES_PER_METRE);
    const float sensorYIn = float(sample.yMetres * INCHES_PER_METRE);

    // Check 2: inside the field, |x| and |y| under 72 in (where the GPS says the sensor is)
    if (!(std::fabs(sensorXIn) < FIELD_HALF_WIDTH_IN) || !(std::fabs(sensorYIn) < FIELD_HALF_WIDTH_IN)) {
        droppedByCheck[CHECK_IN_FIELD]++;
        return;
    }

    // Check 3: not stale. The sensor must have been updating just before this reading: the previous new
    // reading was at most staleMs ago. So the first reading after switching on, and the first after a freeze,
    // are dropped: until then we have not seen the sensor alive (not in the design: it costs one reading per
    // switch-on). (While it is frozen nothing is new, so nothing is used.)
    if (!hadNewReadingBefore || float(nowMs - previousNewMs) > config.staleMs) {
        droppedByCheck[CHECK_NOT_STALE]++;
        return;
    }

    // Check 4: the camera's dead zone. The camera faces our heading (odometry's, not the GPS's) plus mountYaw
    // (R3: the mount yaw is used here as well as in check 5). Cast a ray from the sensor that way to the wall;
    // too close means it cannot read the strip properly.
    // Cast it from where the GPS says the sensor is AND from where odometry says it is (not in the design):
    // the reading could itself be the wrong one
    const float cameraHeadingRad = thetaRad + degToRadF(config.mountYawDeg);
    const Vec mountNow = mountOffset(config, thetaRad);
    const float wallFromGpsIn = distanceToWallIn(sensorXIn, sensorYIn, cameraHeadingRad);
    const float wallFromOdomIn = distanceToWallIn(estXIn + mountNow.x, estYIn + mountNow.y, cameraHeadingRad);
    if (wallFromGpsIn < config.minViewIn || wallFromOdomIn < config.minViewIn) {
        droppedByCheck[CHECK_DEAD_ZONE]++;
        return;
    }

    // Check 5: the heading agrees. A rear-facing sensor reports the robot's heading + 180, so take mountYaw off
    // before comparing with ours
    const float headingErrorDeg = wrapDeg(float(sample.headingDeg) - config.mountYawDeg - radToDegF(thetaRad));
    const bool headingAgrees = std::fabs(headingErrorDeg) <= config.headingGateDeg;

    // Check 6's verdict, worked out here because the heading fault below needs it. It only depends on the history,
    // so working it out before check 5 changes nothing else: a reading is still counted against the first check
    // it fails, and check 6's drop is counted further down, in its place
    const SlowVerdict slow = slowVerdict(nowMs);

    // R3, the heading fault: of the first startHeadingSamples readings since switching on that pass checks 1 to
    // 4, if 80 percent (8 of 10) fail the heading check, the field is turned round, the frame mirrored or the
    // mount yaw wrong: fault. Judged once. The fault is raised as soon as that many have failed, without waiting
    // for the rest: the verdict is the same, only sooner (not in the design). 0 or less: not judged (not in the
    // design).
    // Revision 3: only readings taken while the robot was slow by check 6's rule (both of its windows) are
    // judged. The GPS heading is latencyMs old and ours is from now, so in a fast turn they disagree by the turn
    // rate times the delay, and a healthy sensor on a routine that opens with a turn came within one reading of
    // a false fault. A reading check 6 cannot judge yet (no history) is not judged either
    if (!headingJudged && slow == SlowVerdict::Slow) {
        const int judged = config.startHeadingSamples;
        if (judged <= 0) {
            headingJudged = true;
        } else {
            headingSeen++;
            if (!headingAgrees) headingFailed++;
            if (headingFailed >= headingFailuresForFault(judged)) {
                headingJudged = true;
                droppedByCheck[CHECK_HEADING]++;
                setFault(Fault::HeadingDisagree);
                return;
            }
            if (headingSeen >= judged) headingJudged = true;
        }
    }
    if (!headingAgrees) {
        droppedByCheck[CHECK_HEADING]++;
        return;
    }

    // Check 5b (R4): drop the reading if the sensor's own error number (get_error(), metres, turned into inches)
    // is above maxSensorErrorIn. Not used while maxSensorErrorIn is 0 (the default): PROS does not say what the
    // number means. It has its own counter, not one of the seven (written so that a NaN is also dropped)
    if (config.maxSensorErrorIn > 0 &&
        !(float(sample.errorMetres * INCHES_PER_METRE) <= config.maxSensorErrorIn)) {
        droppedBySensorError++;
        return;
    }

    // Check 6: the robot is slow enough (worked out above, in slowVerdict(), which says how)
    if (slow == SlowVerdict::TooFast) {
        droppedByCheck[CHECK_SLOW]++;
        return;
    }
    if (slow == SlowVerdict::NoHistory) {
        droppedNoHistory++;
        return;
    }

    // Check 7: at most one reading every minPeriodMs is used. The sensor filters its output already, so
    // readings close together are not independent and would be trusted too much
    if (haveUsedReading && float(nowMs - lastUsedMs) < config.minPeriodMs) {
        droppedByCheck[CHECK_RATE]++;
        return;
    }
    haveUsedReading = true;
    lastUsedMs = nowMs;

    // Step 3, delay: the reading shows where the robot was latencyMs ago. Compare it with where we believed the
    // robot was then, moved by every correction decided since:
    //   past = history.pose + (appliedNow - history.applied) + pending
    // "Then" is the history entry closest to latencyMs old, a tie going to the newer one, while check 6's second
    // window ends at the newest entry at least latencyMs old (not in the design: with latencyMs a multiple of the
    // 10 ms tick, as 60 is, the two are the same entry)
    const HistoryEntry* past = entryClosestToAge(nowMs, config.latencyMs);
    if (past == nullptr) {
        // just switched on: the history does not go back latencyMs yet (not in the design). Check 6 already
        // needs more history than this, so it is only a safety net
        droppedNoHistory++;
        return;
    }
    const float pastXIn = past->xIn + (fedInXIn - past->appliedXIn) + pendingXIn;
    const float pastYIn = past->yIn + (fedInYIn - past->appliedYIn) + pendingYIn;
    // The robot's centre: the sensor's position minus the mount vector, turned by odometry's heading at the
    // time the reading was taken
    const Vec mountThen = mountOffset(config, past->thetaRad);
    const float gpsXIn = sensorXIn - mountThen.x;
    const float gpsYIn = sensorYIn - mountThen.y;
    const float innovationXIn = gpsXIn - pastXIn;
    const float innovationYIn = gpsYIn - pastYIn;
    lastInnovationXIn = innovationXIn;
    lastInnovationYIn = innovationYIn;
    // R4, for the [GPSFUSE] line: the innovation in the robot's own frame at the time the reading was taken
    // (right is (cos, -sin) and forward (sin, cos) in field coordinates). A mount offset error turns with the
    // robot in field coordinates but stays put in the robot's frame. Averaged over every reading that gets this
    // far, start check readings included, since switching on (not in the design: it does not say over what)
    {
        const float s = std::sin(past->thetaRad);
        const float c = std::cos(past->thetaRad);
        innovationRightSumIn += innovationXIn * c - innovationYIn * s;
        innovationForwardSumIn += innovationXIn * s + innovationYIn * c;
        innovationCount++;
    }

    // The start check: the first startSamples readings that pass checks 1 to 7 are averaged. If they disagree
    // with the pose by more than startTolIn, fault. They are only used for this check, not to correct anything
    // (not in the design). R3: when it passes, odometry's doubt is set to startSigmaIn (how well the team can
    // place the robot), so a start pose a few inches off is corrected at the first stop. It is kept within 0 and
    // sigmaMaxIn (not in the design). Revision A of the wall design: both doubts are set to it
    if (!startPassed) {
        startSumXIn += innovationXIn;
        startSumYIn += innovationYIn;
        startCount++;
        if (startCount >= std::max(1, config.startSamples)) {
            const float meanXIn = startSumXIn / float(startCount);
            const float meanYIn = startSumYIn / float(startCount);
            if (length(meanXIn, meanYIn) > config.startTolIn) {
                setFault(Fault::StartDisagree);
            } else {
                startPassed = true;
                sigmaXIn = sigmaYIn = floored(std::max(0.0f, std::min(config.startSigmaIn, config.sigmaMaxIn)));
            }
        }
        return;
    }

    // Step 4, statistical gate: drop the reading if |innovation|^2 > gate^2 * (sigma^2 + sigmaGps^2).
    // The wall design keeps one doubt per axis and gates per axis:
    //   ix^2 / (sigmaX^2 + sigmaGps^2) + iy^2 / (sigmaY^2 + sigmaGps^2) > gate^2
    // Both sides are multiplied here by the smaller of the two totals, which does not change the test:
    //   ix^2 * (smallerTotal / totalX) + iy^2 * (smallerTotal / totalY) > gate^2 * smallerTotal
    // Written this way, while the two doubts are equal (always, with no wall sensor) both fractions are exactly
    // 1, and every number is computed exactly as revision 2 computed it, so GPS-only results do not change
    const float odomVarianceX = sigmaXIn * sigmaXIn;
    const float odomVarianceY = sigmaYIn * sigmaYIn;
    const float gpsVariance = config.sigmaGpsIn * config.sigmaGpsIn;
    const float totalVarianceX = odomVarianceX + gpsVariance;
    const float totalVarianceY = odomVarianceY + gpsVariance;
    const float smallerTotal = std::min(totalVarianceX, totalVarianceY);
    // (a total of 0, no doubt and no GPS noise at all, would divide by zero: then only an exact match passes on
    // that axis, as in revision 2)
    const float fractionX = totalVarianceX > 0 ? smallerTotal / totalVarianceX : 1;
    const float fractionY = totalVarianceY > 0 ? smallerTotal / totalVarianceY : 1;
    const float weightedSquared =
        innovationXIn * innovationXIn * fractionX + innovationYIn * innovationYIn * fractionY;
    // R4, for the [GPSFUSE] line: the normalised innovation squared of every reading that reaches the gate,
    // averaged since switching on (not in the design: it does not say over what). Start check readings are left
    // out: sigma means nothing before the start check has passed. Per axis, it is weightedSquared / smallerTotal
    if (smallerTotal > 0) {
        nisSum += weightedSquared / smallerTotal;
        nisCount++;
    }
    if (weightedSquared > config.gate * config.gate * smallerTotal) {
        droppedByGate++;
        handleGateFailure(nowMs, innovationXIn, innovationYIn);
        return;
    }
    relocRun = 0; // a reading got through, so the run of gate failures is over

    // Step 5, update: K = sigma^2 / (sigma^2 + sigmaGps^2); pending += K * innovation; sigma = sqrt(1 - K) * sigma.
    // (Both sigmas zero would divide by zero: then the GPS is given no weight.) Per axis (wall design):
    // Kx = sigmaX^2 / (sigmaX^2 + sigmaGps^2), the same for y, and each doubt shrinks by its own gain
    const float gainX = totalVarianceX > 0 ? odomVarianceX / totalVarianceX : 0;
    const float gainY = totalVarianceY > 0 ? odomVarianceY / totalVarianceY : 0;
    // Each gain is written as the larger gain times a share: 1 for the axis with the larger gain, smaller /
    // larger for the other. The step limit below then scales the larger gain alone, which scales both gains by
    // the same factor. With equal doubts both shares are exactly 1 and this is revision 2's arithmetic exactly
    float gain = std::max(gainX, gainY);
    const float shareX = gain > 0 ? gainX / gain : 1;
    const float shareY = gain > 0 ? gainY / gain : 1;
    const float shapedXIn = shareX * innovationXIn; // the step is gain times (shapedX, shapedY)
    const float shapedYIn = shareY * innovationYIn;
    // R1, step limit: one reading may move the decided correction by at most maxStepIn. If K * innovation is
    // longer than that, K is scaled down so it is exactly that long, and the scaled K is the one used for sigma.
    // So one wrong reading can do at most maxStepIn. Revision A: the step is the vector (Kx * ix, Ky * iy); if
    // it is longer than maxStepIn both gains are scaled by the same factor so that it is exactly maxStepIn, and
    // the scaled gains are the ones used to shrink the doubts
    const float stepLimitIn = std::max(0.0f, config.maxStepIn);
    const float shapedIn = std::sqrt(shapedXIn * shapedXIn + shapedYIn * shapedYIn);
    if (gain * shapedIn > stepLimitIn) gain = stepLimitIn / shapedIn;
    const float stepXIn = gain * shapedXIn;
    const float stepYIn = gain * shapedYIn;
    maxReadingStepIn = std::max(maxReadingStepIn, length(stepXIn, stepYIn));
    sigmaXIn = floored(std::sqrt(1 - gain * shareX) * sigmaXIn);
    sigmaYIn = floored(std::sqrt(1 - gain * shareY) * sigmaYIn);
    accepted++;
    decide(stepXIn, stepYIn);
}

GpsFusionCore::SlowVerdict GpsFusionCore::slowVerdict(std::uint32_t nowMs) const {
    // Check 6: the robot is slow enough. Speed and turn rate from odometry over the last 50 ms of history,
    // with the corrections taken out (a correction being fed in is not the robot moving). Until the history
    // is 50 ms long we cannot tell how fast the robot is going, so the reading is dropped
    const HistoryEntry* then = entryAtLeastThisOld(nowMs, SPEED_WINDOW_MS);
    if (then == nullptr) return SlowVerdict::TooFast;
    float speedInPerS = 0;
    float turnRateDegPerS = 0;
    speedBetween(*then, historyBack(0), speedInPerS, turnRateDegPerS);
    if (speedInPerS > config.maxSpeedInPerS || turnRateDegPerS > config.maxTurnRateDegPerS)
        return SlowVerdict::TooFast;
    // R4: and over the 50 ms ending latencyMs ago, when the reading was taken: otherwise the first reading after
    // every stop was taken at full speed. The window is the newest history entry at least latencyMs old and the
    // newest one at least 50 ms older than that (not in the design: it says "the 50 ms ending latencyMs ago").
    // If the history does not reach back that far yet, the reading is counted as "no history" (not in the
    // design: the design's step 3 drops a reading for that reason too)
    const int takenBack = ticksBackAtLeast(nowMs, config.latencyMs, 0);
    const int beforeTakenBack =
        takenBack < 0 ? -1 : ticksBackAtLeast(historyBack(takenBack).timeMs, SPEED_WINDOW_MS, takenBack + 1);
    if (beforeTakenBack < 0) return SlowVerdict::NoHistory;
    speedBetween(historyBack(beforeTakenBack), historyBack(takenBack), speedInPerS, turnRateDegPerS);
    if (speedInPerS > config.maxSpeedInPerS || turnRateDegPerS > config.maxTurnRateDegPerS)
        return SlowVerdict::TooFast;
    return SlowVerdict::Slow;
}

void GpsFusionCore::handleGateFailure(std::uint32_t nowMs, float innovationXIn, float innovationYIn) {
    // R2, relocalise or give up. Keep the innovations of this run of gate failures, and when each came (a ring
    // of the latest ones). The run counts only readings that reach the gate: one that passes it ends the run,
    // and readings dropped by the checks neither add to it nor end it
    relocXIn[relocNext] = innovationXIn;
    relocYIn[relocNext] = innovationYIn;
    relocMs[relocNext] = nowMs;
    relocNext = (relocNext + 1) % MAX_RELOC_SAMPLES;
    if (relocRun < MAX_RELOC_SAMPLES) relocRun++;
    const int needed = std::clamp(config.relocSamples, 1, MAX_RELOC_SAMPLES);
    if (relocRun < needed) return;

    // The last `needed` gate failures in a row. They must all be within relocWindowMs of each other; if not,
    // keep watching (the next failure slides the window on)
    const int newest = (relocNext - 1 + MAX_RELOC_SAMPLES) % MAX_RELOC_SAMPLES;
    const int oldest = (relocNext - needed + MAX_RELOC_SAMPLES) % MAX_RELOC_SAMPLES;
    if (!(float(relocMs[newest] - relocMs[oldest]) <= config.relocWindowMs)) return;

    // Do they agree? Each must be within relocSpreadIn of their mean. Their innovations are compared, not their
    // positions, so the robot may have moved slowly in between
    float sumXIn = 0;
    float sumYIn = 0;
    for (int i = 0; i < needed; i++) {
        const int a = (relocNext - 1 - i + 2 * MAX_RELOC_SAMPLES) % MAX_RELOC_SAMPLES;
        sumXIn += relocXIn[a];
        sumYIn += relocYIn[a];
    }
    const float meanXIn = sumXIn / float(needed);
    const float meanYIn = sumYIn / float(needed);
    for (int i = 0; i < needed; i++) {
        const int a = (relocNext - 1 - i + 2 * MAX_RELOC_SAMPLES) % MAX_RELOC_SAMPLES;
        // they disagree: probably noise or a confused sensor. Keep watching
        if (!(length(relocXIn[a] - meanXIn, relocYIn[a] - meanYIn) <= config.relocSpreadIn)) return;
    }

    // They agree: odometry has probably slipped (a shove). If the GPS is close enough to believe, add the mean
    // innovation to pending in one go (it is an average, so the step limit does not apply), set sigma to
    // sigmaGpsIn and start a new run. Otherwise give up. When more than one limit is broken, the fault named is
    // the first of: too far, too many, total too large (not in the design)
    if (!(length(meanXIn, meanYIn) <= config.relocMaxIn)) {
        setFault(Fault::RelocTooFar);
        return;
    }
    // At most relocMaxCount relocalisations in a run: a robot that keeps getting "shoved" has a bad GPS
    if (int(relocalisations) >= config.relocMaxCount) {
        setFault(Fault::RelocTooMany);
        return;
    }
    relocalisations++;
    relocRun = 0;
    // sigma is kept within 0 and sigmaMaxIn (not in the design). Revision A of the wall design: both doubts
    sigmaXIn = sigmaYIn = floored(std::max(0.0f, std::min(config.sigmaGpsIn, config.sigmaMaxIn)));
    // the total limit applies to relocalisations too (decide checks it)
    decide(meanXIn, meanYIn);
}

// ------------------------------------------------------------------------------------------------------------
// Wall fixes (docs/WALL_FIX_DESIGN.md)
// ------------------------------------------------------------------------------------------------------------

bool GpsFusionCore::robotStill(std::uint32_t nowMs) const {
    // Wall check 3: "speed at most wallMaxSpeed and turn rate at most wallMaxTurnRate over the last 100 ms, from
    // the pose history", with the corrections fed in taken out (Revision A), as check 6 does for the GPS. Until
    // the history reaches back that far we cannot tell, so the robot does not count as still
    const HistoryEntry* then = entryAtLeastThisOld(nowMs, wallConfig.stillWindowMs);
    if (then == nullptr) return false;
    float speedInPerS = 0;
    float turnRateDegPerS = 0;
    speedBetween(*then, historyBack(0), speedInPerS, turnRateDegPerS);
    // (written so that a NaN is not still)
    return speedInPerS <= wallConfig.wallMaxSpeedInPerS && turnRateDegPerS <= wallConfig.wallMaxTurnRateDegPerS;
}

GpsFusionCore::WallView GpsFusionCore::viewWall(int sensor, float estXIn, float estYIn, float thetaRad) const {
    const WallSensorMount& mount = wallMounts[sensor];
    const float* plane = wallConfig.planeIn;
    WallView view;
    // "Geometry": sensor position S = (x + r cos t + f sin t, y - r sin t + f cos t);
    // beam heading phi = t + yaw, direction d = (sin phi, cos phi)
    const Vec offset = wallSensorOffset(mount, thetaRad);
    const float sensorXIn = estXIn + offset.x;
    const float sensorYIn = estYIn + offset.y;
    const float beamRad = thetaRad + degToRadF(mount.yawDeg);
    const float dirX = std::sin(beamRad);
    const float dirY = std::cos(beamRad);
    // "Cast the ray from S along d to the four planes; the nearest hit is the wall, at predicted range p"
    const WallHit hit = castToWall(plane, sensorXIn, sensorYIn, dirX, dirY);
    view.wall = hit.wall;
    view.predictedIn = hit.rangeIn;

    // Wall check 5, one wall: the ray hits a wall whose trust is not Ignore
    if (hit.wall == NO_WALL || wallConfig.trust[hit.wall] == WallTrust::Ignore) {
        view.failedCheck = WCHECK_ONE_WALL;
        return view;
    }
    // a wall x = Xw fixes x, and d.along is d.x; a wall y = Yw fixes y, and d.along is d.y. d.across is the other
    view.axis = axisOfWall(hit.wall);
    view.alongDir = view.axis == 0 ? dirX : dirY;
    view.acrossDir = view.axis == 0 ? dirY : dirX;

    // Wall check 6, square enough: the angle between the beam and the wall's normal is at most maxIncidenceDeg
    const float incidenceDeg = radToDegF(std::atan2(std::fabs(view.acrossDir), std::fabs(view.alongDir)));
    if (!(incidenceDeg <= wallConfig.maxIncidenceDeg)) {
        view.failedCheck = WCHECK_SQUARE;
        return view;
    }
    // Wall check 7, in range, for p (m is checked once there is a reading)
    if (!(view.predictedIn >= wallConfig.minRangeIn && view.predictedIn <= maxRangeOf(wallConfig, hit.wall))) {
        view.failedCheck = WCHECK_IN_RANGE;
        return view;
    }
    // Wall check 8, plain wall: "rays at phi plus and minus coneHalfDeg hit the same wall"...
    const float coneRad = degToRadF(wallConfig.coneHalfDeg);
    const WallHit edgeA =
        castToWall(plane, sensorXIn, sensorYIn, std::sin(beamRad - coneRad), std::cos(beamRad - coneRad));
    const WallHit edgeB =
        castToWall(plane, sensorXIn, sensorYIn, std::sin(beamRad + coneRad), std::cos(beamRad + coneRad));
    if (edgeA.wall != hit.wall || edgeB.wall != hit.wall) {
        view.failedCheck = WCHECK_PLAIN_WALL;
        return view;
    }
    // ...and "the cone's footprint on the wall, widened by footprintMarginIn": from one edge's hit to the other's
    const float lowIn = std::min(edgeA.alongIn, edgeB.alongIn) - wallConfig.footprintMarginIn;
    const float highIn = std::max(edgeA.alongIn, edgeB.alongIn) + wallConfig.footprintMarginIn;
    // "does not overlap any keep-out segment listed for that wall". Touching counts as overlapping (not in the
    // wall design: the cautious answer)
    for (int k = 0; k < wallConfig.keepOutCount; k++) {
        const KeepOut& keepOut = wallConfig.keepOuts[k];
        if (keepOut.wall != hit.wall) continue;
        const float keepLowIn = std::min(keepOut.fromIn, keepOut.toIn);
        const float keepHighIn = std::max(keepOut.fromIn, keepOut.toIn);
        if (lowIn <= keepHighIn && highIn >= keepLowIn) {
            view.failedCheck = WCHECK_PLAIN_WALL;
            return view;
        }
    }
    // "or come within cornerMarginIn of a corner". A wall x = +-X runs along y from -planeIn[2] to +planeIn[0];
    // a wall y = +-Y runs along x from -planeIn[3] to +planeIn[1]. Exactly cornerMarginIn away counts as within
    // (not in the wall design: the cautious answer)
    const float wallStartIn = view.axis == 0 ? -plane[WALL_MINUS_Y] : -plane[WALL_MINUS_X];
    const float wallEndIn = view.axis == 0 ? plane[WALL_PLUS_Y] : plane[WALL_PLUS_X];
    if (!(lowIn - wallStartIn > wallConfig.cornerMarginIn) || !(wallEndIn - highIn > wallConfig.cornerMarginIn)) {
        view.failedCheck = WCHECK_PLAIN_WALL;
        return view;
    }
    return view;
}

void GpsFusionCore::countHealthTry(WallTracker& tracker, bool bad) {
    // "If, over at least healthMinTries readings that passed the geometry checks (5 to 8), half or more were
    // invalid, low confidence or through, that sensor is switched off for the run. This is how it finds out the
    // walls are clear." Counted over the whole run since switching on (not in the wall design: it does not say
    // over what)
    WallSensorStatus& st = tracker.status;
    st.healthTries++;
    if (bad) st.healthBad++;
    if (int(st.healthTries) >= wallConfig.healthMinTries && 2 * st.healthBad >= st.healthTries)
        st.state = WallSensorState::HealthOff;
}

void GpsFusionCore::handleWallSample(int sensor, std::uint32_t nowMs, const DistanceSample& sample, float estXIn,
                                     float estYIn, float thetaRad) {
    WallTracker& tracker = wallTrackers[sensor];
    WallSensorStatus& st = tracker.status;
    // a sensor that failed its start check or its health rule stops for the run (Revision A). The GPS and the
    // other wall sensor carry on, and nothing is fed back out: each of its fixes was gated and capped
    if (st.state != WallSensorState::WaitingForStart && st.state != WallSensorState::Working) return;

    // Sampling (Revision A): "each sensor is sampled at most once every 35 ms". Odometry reads it every tick;
    // the readings in between are not looked at, so they are not counted anywhere
    if (tracker.haveSample && float(nowMs - tracker.lastSampleMs) < wallConfig.sampleMinPeriodMs) return;
    tracker.haveSample = true;
    tracker.lastSampleMs = nowMs;
    st.samples++;

    // Revision A: "The prediction uses the filter's estimate (pose plus what is decided but not yet fed in)".
    // What the beam meets is judged from the estimate alone, before the reading is looked at, because wall
    // check 2 needs the wall's trust and the health rule needs the geometry of every reading it judges
    const WallView view = viewWall(sensor, estXIn + pendingXIn, estYIn + pendingYIn, thetaRad);
    const bool still = robotStill(nowMs);
    // The health rule judges a reading only when the sensor plainly should see the wall: the robot is still
    // (wall check 3's test) and wall checks 5 to 8 pass from the estimate, wall check 7 on p alone (not in the
    // wall design: it says "readings that passed the geometry checks (5 to 8)", but a reading that fails wall
    // check 1 or 2 never reaches them in order, and judging moving readings would count clear-wall symptoms
    // only where they can be seen)
    const bool healthJudges = still && view.failedCheck == NUM_WALL_CHECKS;

    // Wall check 1, valid: not an error value, not 0, not 9999, between 20 and 2000 mm. A confidence that is not
    // 0 to 63 is an error value too (not in the wall design)
    const bool valid = sample.mm != DISTANCE_READ_FAILED && sample.mm != 0 && sample.mm != DISTANCE_NOTHING_SEEN_MM &&
                       sample.mm >= WALL_MIN_VALID_MM && sample.mm <= WALL_MAX_VALID_MM && sample.confidence >= 0 &&
                       sample.confidence <= WALL_MAX_CONFIDENCE;
    if (!valid) {
        st.droppedByCheck[WCHECK_VALID]++;
        tracker.steadyHave = 0; // the readings in a row for wall check 4 start again (not in the wall design)
        if (healthJudges) countHealthTry(tracker, true);
        return;
    }
    // Wall check 2, confident: above 200 mm, get_confidence() at least minConfidence (minConfidenceClear on a
    // wall marked Clear). A low-confidence reading the health rule judges is a bad try
    if (sample.mm > WALL_CONFIDENCE_ABOVE_MM && sample.confidence < minConfidenceFor(wallConfig, view.wall)) {
        st.droppedByCheck[WCHECK_CONFIDENT]++;
        tracker.steadyHave = 0;
        if (healthJudges) countHealthTry(tracker, true);
        return;
    }
    // Wall check 3, still. "Because the robot is still, no delay handling is needed"
    if (!still) {
        st.droppedByCheck[WCHECK_STILL]++;
        tracker.steadyHave = 0;
        return;
    }

    // Wall check 4, steady: "the last steadyCount valid readings, taken at least 35 ms apart, span at most the
    // larger of 0.5 in and 2 percent of their median. The median is the reading used." Readings in a row: any
    // reading that failed wall check 1, 2 or 3 above started the row again, and so does a gap (not in the wall
    // design). The sampling rule keeps them at least sampleMinPeriodMs apart. Each is calibrated first:
    // "Reading: m = scale * mm / 25.4 + offset inches"
    const float readingIn = wallMounts[sensor].scale * float(sample.mm) / MM_PER_INCH + wallMounts[sensor].offsetIn;
    if (tracker.steadyHave > 0) {
        const int newest = (tracker.steadyNext - 1 + MAX_STEADY_COUNT) % MAX_STEADY_COUNT;
        if (float(nowMs - tracker.steadyMs[newest]) > WALL_STEADY_MAX_GAP_MS) tracker.steadyHave = 0;
    }
    tracker.steadyIn[tracker.steadyNext] = readingIn;
    tracker.steadyMs[tracker.steadyNext] = nowMs;
    tracker.steadyNext = (tracker.steadyNext + 1) % MAX_STEADY_COUNT;
    if (tracker.steadyHave < MAX_STEADY_COUNT) tracker.steadyHave++;
    const int steadyCount = std::clamp(wallConfig.steadyCount, 1, MAX_STEADY_COUNT);
    if (tracker.steadyHave < steadyCount) {
        st.droppedByCheck[WCHECK_STEADY]++;
        return;
    }
    // the last steadyCount readings, sorted (a few numbers: a simple insertion sort)
    float sorted[MAX_STEADY_COUNT];
    for (int i = 0; i < steadyCount; i++) {
        const float value = tracker.steadyIn[(tracker.steadyNext - 1 - i + 2 * MAX_STEADY_COUNT) % MAX_STEADY_COUNT];
        int at = i;
        while (at > 0 && sorted[at - 1] > value) {
            sorted[at] = sorted[at - 1];
            at--;
        }
        sorted[at] = value;
    }
    const float medianIn = steadyCount % 2 == 1 ? sorted[steadyCount / 2]
                                                : (sorted[steadyCount / 2 - 1] + sorted[steadyCount / 2]) / 2;
    const float spanIn = sorted[steadyCount - 1] - sorted[0];
    if (!(spanIn <= std::max(wallConfig.steadySpreadIn, wallConfig.steadySpreadFrac * medianIn))) {
        st.droppedByCheck[WCHECK_STEADY]++;
        return;
    }
    const float measuredIn = medianIn; // "m"

    // Wall checks 5 to 8, as judged from the estimate above; wall check 7 also needs m
    if (view.failedCheck == WCHECK_ONE_WALL || view.failedCheck == WCHECK_SQUARE) {
        st.droppedByCheck[view.failedCheck]++;
        return;
    }
    if (view.failedCheck == WCHECK_IN_RANGE ||
        !(measuredIn >= wallConfig.minRangeIn && measuredIn <= maxRangeOf(wallConfig, view.wall))) {
        st.droppedByCheck[WCHECK_IN_RANGE]++;
        return;
    }
    if (view.failedCheck == WCHECK_PLAIN_WALL) {
        st.droppedByCheck[WCHECK_PLAIN_WALL]++;
        return;
    }

    // The measurement ("Geometry"). A wall x = Xw implies x = Xw - m * d.x - (S.x - x), and since p * d.x =
    // Xw - S.x, "the innovation on that axis is (p - m) * d.x" (or d.y for a wall y = Yw). The other axis is not
    // touched
    const float alongAbs = std::fabs(view.alongDir);
    const float acrossAbs = std::fabs(view.acrossDir);
    const float innovationIn = (view.predictedIn - measuredIn) * view.alongDir;
    // "Noise on that axis: R = (sigmaWall * |d.along|)^2 + (m * |d.across| * sigmaHeading)^2 where
    // sigmaWall = wallNoiseIn + wallNoiseFrac * m". The second term is what a heading error costs when the beam
    // is not square
    const float sigmaWallIn = wallConfig.wallNoiseIn + wallConfig.wallNoiseFrac * measuredIn;
    const float sigmaHeadingRad = degToRadF(wallConfig.sigmaHeadingDeg);
    const float noiseAlongIn = sigmaWallIn * alongAbs;
    const float noiseHeadingIn = measuredIn * acrossAbs * sigmaHeadingRad;
    const float noiseVariance = noiseAlongIn * noiseAlongIn + noiseHeadingIn * noiseHeadingIn; // R
    float& sigmaAxisIn = view.axis == 0 ? sigmaXIn : sigmaYIn;
    const float axisVariance = sigmaAxisIn * sigmaAxisIn;
    st.lastWall = view.wall;
    st.lastAxis = view.axis;
    st.lastPredictedIn = view.predictedIn;
    st.lastMeasuredIn = measuredIn;
    st.lastInnovationIn = innovationIn;
    st.lastNoiseIn = std::sqrt(noiseVariance);

    // Start check, per sensor: "The first reading that passes checks 1 to 8 is compared with the pose. If it
    // disagrees by more than wallStartTolIn, that sensor faults for the run." Compared on the wall's axis, as
    // the innovation. Like the GPS's start check readings, it is used only for this check, not to fix anything
    // (not in the wall design). (Written so that a NaN also faults)
    if (st.state == WallSensorState::WaitingForStart) {
        st.startInnovationIn = innovationIn;
        st.state = std::fabs(innovationIn) <= wallConfig.wallStartTolIn ? WallSensorState::Working
                                                                         : WallSensorState::StartFault;
        return;
    }

    // Revision A, checks 9 and 10: s = sqrt(sigmaAxis^2 + R), one standard deviation of the innovation
    const float innovationSdIn = std::sqrt(axisVariance + noiseVariance);
    // how much shorter than predicted the reading is, along the axis (negative: longer)
    const float shorterByIn = (view.predictedIn - measuredIn) * alongAbs;
    // Wall check 9, not blocked: "dropped if the reading is shorter than predicted by more than
    // shortGate = min(blockedMaxIn, max(blockedMinIn, 2*s)) along the axis"
    const float shortGateIn =
        std::min(wallConfig.blockedMaxIn, std::max(wallConfig.blockedMinIn, wallConfig.blockedSigmas * innovationSdIn));
    if (shorterByIn > shortGateIn) {
        st.droppedByCheck[WCHECK_NOT_BLOCKED]++;
        return;
    }
    // Wall check 10, not through: "dropped if longer than predicted by more than longGate = min(throughMaxIn,
    // 3*s)". For the health rule a through reading is a bad try, and one that gets past this check a good one
    const float longGateIn = std::min(wallConfig.throughMaxIn, wallConfig.throughSigmas * innovationSdIn);
    if (-shorterByIn > longGateIn) {
        st.droppedByCheck[WCHECK_NOT_THROUGH]++;
        countHealthTry(tracker, true);
        return;
    }
    countHealthTry(tracker, false);
    if (st.state != WallSensorState::Working) return; // the health rule has just switched it off

    // Wall check 11, statistical gate: innovation^2 <= gate^2 * (sigmaAxis^2 + R). The gate is the filter's
    // own (Config::gate), the design names no other (written so that a NaN is dropped)
    if (!(innovationIn * innovationIn <= config.gate * config.gate * (axisVariance + noiseVariance))) {
        st.droppedByCheck[WCHECK_GATE]++;
        return;
    }
    // Wall check 12, cap: "the correction K * innovation is at most the wall's cap. Larger ones are dropped, not
    // clipped". K = sigmaAxis^2 / (sigmaAxis^2 + R) (no doubt and no noise at all would divide by zero: then the
    // reading is given no weight)
    const float totalVariance = axisVariance + noiseVariance;
    const float gain = totalVariance > 0 ? axisVariance / totalVariance : 0;
    const float correctionIn = gain * innovationIn;
    if (!(std::fabs(correctionIn) <= capOf(wallConfig, view.wall))) {
        st.droppedByCheck[WCHECK_CAP]++;
        return;
    }
    // Wall check 13, once per stop: "after a sensor gives a fix, it gives no other until the robot has travelled
    // refixTravelIn or turned refixTurnDeg." Travelled and turned are added up tick by tick, by odometry with the
    // corrections taken out (tick())
    if (tracker.fixedThisStop && tracker.travelSinceFixIn < wallConfig.refixTravelIn &&
        radToDegF(tracker.turnSinceFixRad) < wallConfig.refixTurnDeg) {
        st.droppedByCheck[WCHECK_ONCE_PER_STOP]++;
        return;
    }

    // The fix: "pending += K * innovation on that axis; sigmaAxis = max(sigmaFloorIn, sqrt(1 - K) * sigmaAxis)".
    // It goes into the same pending as the GPS's corrections and is fed in at the same rate limit. Revision A:
    // it is not subject to maxStepIn (the cap above is its limit), but the total limit counts it
    st.fixes++;
    st.lastCorrectionIn = correctionIn;
    if (std::fabs(correctionIn) > std::fabs(st.largestCorrectionIn)) st.largestCorrectionIn = correctionIn;
    tracker.fixedThisStop = true;
    tracker.travelSinceFixIn = 0;
    tracker.turnSinceFixRad = 0;
    sigmaAxisIn = floored(std::sqrt(1 - gain) * sigmaAxisIn);
    decide(view.axis == 0 ? correctionIn : 0.0f, view.axis == 1 ? correctionIn : 0.0f);
}

Status GpsFusionCore::getStatus() const {
    Status s;
    s.mode = mode;
    s.fault = fault;
    s.faultText = faultText(fault);
    s.faultName = faultName(fault);
    s.unwinding = unwinding && mode != Mode::Off && (fedInXIn != 0 || fedInYIn != 0);
    s.sigmaIn = std::max(sigmaXIn, sigmaYIn);
    s.sigmaXIn = sigmaXIn;
    s.sigmaYIn = sigmaYIn;
    s.pendingXIn = pendingXIn;
    s.pendingYIn = pendingYIn;
    // what was fed in went into the pose in On, and was kept apart in Shadow
    if (runMode == Mode::Shadow) {
        s.shadowXIn = fedInXIn;
        s.shadowYIn = fedInYIn;
    } else {
        s.appliedXIn = fedInXIn;
        s.appliedYIn = fedInYIn;
    }
    s.decidedXIn = decidedXIn;
    s.decidedYIn = decidedYIn;
    s.estimateXIn = estimateXIn;
    s.estimateYIn = estimateYIn;
    s.startCheckPassed = startPassed;
    s.startSamplesSeen = startCount;
    s.startSamplesNeeded = std::max(1, config.startSamples);
    s.headingSamplesSeen = headingSeen;
    s.headingSamplesFailed = headingFailed;
    s.headingJudged = headingJudged;
    s.sensorFrozen = !haveNewReading || !haveTick || float(lastTickMs - lastNewMs) > config.staleMs;
    s.newReadings = newReadings;
    for (int i = 0; i < NUM_CHECKS; i++) s.droppedByCheck[i] = droppedByCheck[i];
    s.droppedBySensorError = droppedBySensorError;
    s.droppedByGate = droppedByGate;
    s.droppedNoHistory = droppedNoHistory;
    s.accepted = accepted;
    s.relocalisations = relocalisations;
    s.maxReadingStepIn = maxReadingStepIn;
    s.lastInnovationXIn = lastInnovationXIn;
    s.lastInnovationYIn = lastInnovationYIn;
    s.innovationCount = innovationCount;
    if (innovationCount > 0) {
        s.innovationRightAvgIn = innovationRightSumIn / float(innovationCount);
        s.innovationForwardAvgIn = innovationForwardSumIn / float(innovationCount);
    }
    s.nisCount = nisCount;
    if (nisCount > 0) s.nisAvg = nisSum / float(nisCount);
    s.wallSensorCount = wallSensorCount;
    s.wallsWaiting = unwinding && fault != Fault::None && wallsCanCarryOn();
    s.wallsStopped = wallsStopped;
    for (int i = 0; i < MAX_WALL_SENSORS; i++) s.walls[i] = wallTrackers[i].status;
    return s;
}

} // namespace lemlib::gpsfusion

// ------------------------------------------------------------------------------------------------------------
// The robot's side: one filter, fed by lemlib::update() and the pros::Gps sensor
// ------------------------------------------------------------------------------------------------------------
#ifndef GPSFUSION_CORE_ONLY

#include <atomic>
#include "pros/distance.hpp"
#include "pros/gps.hpp"
#include "pros/rtos.hpp"

namespace lemlib::gpsfusion {

// Revision 3: nothing in this file prints or formats text, except the logging task (printStatus()). The reason
// is PROS's serial output: printf takes the serial port's lock and then waits for room in its output buffer, both
// with no time limit. The odometry task must never wait like that (dead reckoning would stop while it waits), and
// a competition task that PROS deletes in the middle of a print (the end of autonomous, say) leaves that lock
// held for good, so the next printf anywhere waits for ever (reviewer finding, walls_result_2.json). So the
// odometry task and the competition tasks only leave flags here, and the logging task, at a lower priority,
// does every printf. If it ever hangs on a held serial lock, only the printing stops.
//
// Only the odometry task (in step()) may read the GPS and the distance sensors after initialize(). PROS takes a
// port's lock with no time limit for every read, so a routine that read the GPS itself and was deleted in the
// middle of a read would hang odometry on its next read.

namespace {
// the [GPSFUSE] line is printed at most this often: a printf every 10 ms floods the terminal
constexpr std::uint32_t PRINT_PERIOD_MS = 500;

// R4: the longest anyone waits for the lock, ms. The lock is only ever held for a few microseconds, so if it
// cannot be had in this time something is wrong (a competition task deleted while it held it, say), and
// nothing called from opcontrol(), disabled() or runAuton() may hang on it
constexpr std::uint32_t LOCK_WAIT_MS = 5;

// The logging task runs below the default priority, which odometry, opcontrol and the competition tasks use, so
// it only prints when none of them wants the processor
constexpr std::uint32_t LOG_TASK_PRIORITY = TASK_PRIORITY_DEFAULT - 1;
// Its stack, in 4-byte words (8 KB). printf with floats needs about 1 KB; the status and the line it prints are
// kept in static storage below, not on the stack
constexpr std::uint16_t LOG_TASK_STACK_WORDS = 0x800;
// Its name; the host testbench recognises it by this name and does not run it
constexpr const char* LOG_TASK_NAME = "gpsfuse log";

// Read by the odometry task outside the lock, so atomic (not in the design)
std::atomic<pros::Gps*> gpsSensor {nullptr};
GpsFusionCore robotFilter;
// The odometry task runs the filter while a routine's task switches it on and off. The lock makes sure one
// finishes before the other starts. The odometry task never waits for it: if it is taken, that tick skips fusion
pros::Mutex filterLock;
// Odometry reads the mode every tick without the lock, so it is atomic: never seen half-written. R4: setMode
// stores it before anything else, so Off takes effect even if the lock cannot be had
std::atomic<Mode> currentMode {Mode::Off};
// The wall sensors (configureWalls), read by the odometry task outside the lock like the GPS, so atomic. While
// wallSensorsUsed is 0 (the default) no distance sensor is read and the filter's wall code never runs
std::atomic<int> wallSensorsUsed {0};
std::atomic<pros::Distance*> wallSensors[MAX_WALL_SENSORS] = {};

// The On guards (configure, Revision 3): On is allowed only when both are true. False until configure() says
// otherwise, so a robot that never called configure() can never be On
std::atomic<bool> mountMeasured {false};
std::atomic<bool> odometryCalibrated {false};
// configure() refused the numbers (a latencyMs the history cannot serve): fusion stays Off
std::atomic<bool> configRefused {false};

// The pose and time of the last step(), for the [GPSFUSE] line. Written by step() and read by the logging task,
// both under filterLock. haveStepPose is cleared at every switch-on, so a line never shows the last run's pose
bool haveStepPose = false;
std::uint32_t stepMs = 0;
float stepXIn = 0, stepYIn = 0, stepThetaRad = 0;

// Messages for the logging task, one bit each, set with fetch_or by whoever has something to say (none of them
// prints) and taken all at once by the logging task
enum Message : std::uint32_t {
    MSG_CONFIGURE_LOCK_BUSY = 1u << 0, // configure() could not get the lock: the GPS is left out
    MSG_WALLS_LOCK_BUSY = 1u << 1, // configureWalls() could not get the lock: the wall sensors are left out
    MSG_SHADOW_LOCK_BUSY = 1u << 2, // setMode(Shadow) could not get the lock: fusion stays Off
    MSG_ON_LOCK_BUSY = 1u << 3, // setMode(On) could not get the lock: fusion stays Off
    MSG_SETPOSE_LOCK_BUSY = 1u << 4, // setPose could not restart the filter: fusion switched Off
    MSG_ON_NEEDS_MOUNT = 1u << 5, // On asked for, GPS_MOUNT_MEASURED false: Shadow instead
    MSG_ON_NEEDS_ODOMETRY = 1u << 6, // On asked for, ODOMETRY_CALIBRATED false: Shadow instead
    MSG_CONFIG_REFUSED = 1u << 7, // Shadow or On asked for, but configure() refused the numbers: stays Off
    MSG_CONFIG_PROBLEMS = 1u << 8, // configure() found numbers outside their range (configProblems says which)
    MSG_WALL_CONFIG_PROBLEMS = 1u << 9, // configureWalls() did (wallConfigProblems says which)
};
std::atomic<std::uint32_t> pendingMessages {0};
// Which numbers were outside their range. Written by configure() / configureWalls() before they set the message
// bit, read by the logging task after it took the bit (the atomic makes the write visible)
std::uint32_t configProblems = 0;
std::uint64_t wallConfigProblems = 0;

void leaveMessage(std::uint32_t message) { pendingMessages.fetch_or(message); }

// The logging task's own copies, so it formats and prints with the lock already given back. Only it uses them
Status logStatus;
std::uint32_t logMs = 0;
float logXIn = 0, logYIn = 0, logThetaRad = 0;
char logLine[320];

void logTaskLoop(void*) {
    while (true) {
        printStatus();
        pros::delay(PRINT_PERIOD_MS);
    }
}

// Start the logging task, once. From configure(), which initialize() calls: a task started from initialize() is
// not one of the competition tasks PROS deletes on a mode change. The C-function form of pros::Task needs no
// heap of ours (the kernel gives the task its stack, as it does for every task)
void startLogTask() {
    static bool started = false;
    if (started) return;
    started = true;
    static pros::Task logTask(logTaskLoop, nullptr, LOG_TASK_PRIORITY, LOG_TASK_STACK_WORDS, LOG_TASK_NAME);
    (void)logTask;
}

// What setMode and setPose share: switch to `wanted` (with the guards), starting the filter again from the pose
// odometry has now. fromSetPose only changes which message is left if the lock cannot be had
void switchMode(Mode wanted, bool fromSetPose) {
    // R4: Off first, straight into the atomic, without the lock. Odometry checks it every tick, so from here on
    // it stops calling the filter even if the lock below cannot be had. Shadow and On go through Off too, so
    // odometry never runs the old filter under the new mode while it is being reset
    currentMode = Mode::Off;
    Mode effective = wanted;
    if (wanted != Mode::Off && configRefused.load()) {
        // configure() refused the numbers: nothing may run on them
        leaveMessage(MSG_CONFIG_REFUSED);
        effective = Mode::Off;
    } else if (wanted == Mode::On && !(mountMeasured.load() && odometryCalibrated.load())) {
        // Revision 3, the On guards: Shadow logs everything and changes nothing, so it is the safe way to run
        // until both are true. Say which one is missing (both, if both are)
        if (!mountMeasured.load()) leaveMessage(MSG_ON_NEEDS_MOUNT);
        if (!odometryCalibrated.load()) leaveMessage(MSG_ON_NEEDS_ODOMETRY);
        effective = Mode::Shadow;
    }
    if (!filterLock.take(LOCK_WAIT_MS)) {
        // Off has already taken effect. Shadow or On cannot reset the filter safely without the lock, so fusion
        // stays Off (not in the design)
        if (fromSetPose) leaveMessage(MSG_SETPOSE_LOCK_BUSY);
        else if (effective == Mode::Shadow) leaveMessage(MSG_SHADOW_LOCK_BUSY);
        else if (effective == Mode::On) leaveMessage(MSG_ON_LOCK_BUSY);
        return;
    }
    robotFilter.setMode(effective);
    haveStepPose = false; // the next line waits for this run's first tick
    currentMode = effective; // inside the lock, so odometry's next tick finds the filter already reset
    filterLock.give();
}
} // namespace

void configure(pros::Gps* sensor, const Config& config, bool isMountMeasured, bool isOdometryCalibrated) {
    startLogTask();
    mountMeasured = isMountMeasured;
    odometryCalibrated = isOdometryCalibrated;
    // Revision 3: check the numbers, one warning per number outside its range. A latencyMs the history cannot
    // serve is refused: every reading would be dropped as "no history" without a word
    const std::uint32_t problems = checkConfig(config);
    configRefused = latencyRefused(config);
    if (configRefused.load()) currentMode = Mode::Off;
    if (problems != 0) {
        configProblems = problems;
        leaveMessage(MSG_CONFIG_PROBLEMS);
    }
    if (!filterLock.take(LOCK_WAIT_MS)) {
        // never expected at boot (nothing else uses the lock while fusion is Off). The cautious way out: no
        // sensor, so fusion can never change anything (not in the design)
        gpsSensor = nullptr;
        leaveMessage(MSG_CONFIGURE_LOCK_BUSY);
        return;
    }
    gpsSensor = sensor;
    robotFilter.setConfig(config);
    filterLock.give();
}

void configureWalls(pros::Distance* const sensors[], const WallSensorMount mounts[], int count,
                    const WallConfig& config) {
    const int used = (sensors == nullptr || mounts == nullptr) ? 0 : std::clamp(count, 0, MAX_WALL_SENSORS);
    // Revision 3: the wall numbers are checked too (warnings only)
    if (used > 0) {
        const std::uint64_t problems = checkWallConfig(config);
        if (problems != 0) {
            wallConfigProblems = problems;
            leaveMessage(MSG_WALL_CONFIG_PROBLEMS);
        }
    }
    // odometry stops reading wall sensors first, so it never reads a list that is being changed
    wallSensorsUsed = 0;
    if (!filterLock.take(LOCK_WAIT_MS)) {
        // never expected at boot, as for configure(). The cautious way out: no wall sensor is read, so no wall
        // fix can happen (not in the wall design)
        leaveMessage(MSG_WALLS_LOCK_BUSY);
        return;
    }
    for (int i = 0; i < used; i++) wallSensors[i] = sensors[i];
    robotFilter.setWallConfig(config, mounts, used);
    wallSensorsUsed = used;
    filterLock.give();
}

void setMode(Mode newMode) { switchMode(newMode, false); }

Mode mode() { return currentMode.load(); }

Mode pauseForSetPose() {
    // Off: one atomic read, nothing else (so with fusion Off a setPose is exactly what it was without fusion)
    const Mode before = currentMode.load();
    // Otherwise stop odometry using the filter before the pose is written: a tick between the write and the
    // restart would feed a correction into the new pose that the restarted filter knows nothing about.
    // (lemlib::update() checks the mode again before it adds a correction, so one already worked out is dropped)
    if (before != Mode::Off) currentMode = Mode::Off;
    return before;
}

void restartAfterSetPose(Mode modeBefore) {
    // start again from the new pose, exactly as setMode(the same mode) does. The mode was already allowed, so the
    // guards give the same mode again. If the lock cannot be had, fusion stays Off and the logging task says so
    if (modeBefore != Mode::Off) switchMode(modeBefore, true);
}

Status status() {
    Status s;
    if (filterLock.take(LOCK_WAIT_MS)) {
        s = robotFilter.getStatus();
        filterLock.give();
    } else {
        s.valid = false; // only the mode below is filled in (not in the design)
    }
    // the mode odometry goes by (the filter's own may be behind if a setMode could not get the lock)
    s.mode = currentMode.load();
    return s;
}

Correction step(float xIn, float yIn, float thetaRad) {
    // odometry only calls this when the mode is not Off, but a routine may have switched it Off since: then do
    // nothing at all (no sensor read)
    if (mode() == Mode::Off) return Correction();
    // read the sensor first, outside the lock: these are calls into PROS
    GpsSample sample;
    const GpsSample* reading = nullptr;
    pros::Gps* const sensor = gpsSensor.load();
    if (sensor != nullptr) {
        const pros::gps_position_s_t position = sensor->get_position();
        sample.xMetres = position.x;
        sample.yMetres = position.y;
        sample.headingDeg = sensor->get_heading();
        sample.errorMetres = sensor->get_error();
        reading = &sample;
    }
    // the wall sensors, if any (none by default: then nothing here runs), also outside the lock. A sensor left
    // as nullptr reads as unplugged (DistanceSample's defaults: what PROS returns for a failed read)
    DistanceSample wallSamples[MAX_WALL_SENSORS];
    const int wallsUsed = wallSensorsUsed.load();
    for (int i = 0; i < wallsUsed; i++) {
        pros::Distance* const wallSensor = wallSensors[i].load();
        if (wallSensor == nullptr) continue;
        wallSamples[i].mm = wallSensor->get();
        wallSamples[i].confidence = wallSensor->get_confidence();
    }
    const std::uint32_t nowMs = pros::millis();

    // a routine is switching the mode right now: skip fusion this tick rather than wait
    if (!filterLock.take(0)) return Correction();
    const Correction correction =
        robotFilter.tick(nowMs, xIn, yIn, thetaRad, reading, wallsUsed > 0 ? wallSamples : nullptr, wallsUsed);
    // for the logging task's line: the pose the motions will read after this tick (no printing here)
    haveStepPose = true;
    stepMs = nowMs;
    stepXIn = xIn + correction.xIn;
    stepYIn = yIn + correction.yIn;
    stepThetaRad = thetaRad;
    filterLock.give();
    return correction;
}

void printStatus() {
    // 1. the messages, whatever the mode (a "lock busy, fusion stays OFF" comes with the mode Off)
    const std::uint32_t messages = pendingMessages.exchange(0);
    if (messages & MSG_CONFIGURE_LOCK_BUSY)
        std::printf("[GPSFUSE] configure: lock busy, GPS left out: fusion will never use it\n");
    if (messages & MSG_WALLS_LOCK_BUSY)
        std::printf("[WALLFIX] configureWalls: lock busy, wall sensors left out: no wall fixes\n");
    if (messages & MSG_SHADOW_LOCK_BUSY)
        std::printf("[GPSFUSE] lock busy: could not switch to SHADOW, fusion stays OFF\n");
    if (messages & MSG_ON_LOCK_BUSY) std::printf("[GPSFUSE] lock busy: could not switch to ON, fusion stays OFF\n");
    if (messages & MSG_SETPOSE_LOCK_BUSY)
        std::printf("[GPSFUSE] setPose: lock busy, could not restart the filter from the new pose: "
                    "fusion switched OFF\n");
    if (messages & MSG_ON_NEEDS_MOUNT)
        std::printf("[GPSFUSE] On asked for, but GPS_MOUNT_MEASURED is false (robot-config.hpp): running SHADOW\n");
    if (messages & MSG_ON_NEEDS_ODOMETRY)
        std::printf("[GPSFUSE] On asked for, but ODOMETRY_CALIBRATED is false (robot-config.hpp): running SHADOW\n");
    if (messages & MSG_CONFIG_REFUSED)
        std::printf("[GPSFUSE] fusion asked for, but configure() refused gpsFusionConfig: fusion stays OFF\n");
    if (messages & MSG_CONFIG_PROBLEMS) {
        for (int i = 0; i < NUM_CONFIG_PROBLEMS; i++) {
            if (!(configProblems & (std::uint32_t(1) << i))) continue;
            formatConfigProblem(logLine, sizeof logLine, i);
            std::printf("[GPSFUSE] gpsFusionConfig (robot-config.cpp): %s\n", logLine);
        }
    }
    if (messages & MSG_WALL_CONFIG_PROBLEMS) {
        for (int i = 0; i < NUM_WALL_CONFIG_PROBLEMS; i++) {
            if (!(wallConfigProblems & (std::uint64_t(1) << i))) continue;
            formatWallConfigProblem(logLine, sizeof logLine, i);
            std::printf("[WALLFIX] wall config: %s\n", logLine);
        }
    }

    // 2. the status lines, only while fusion runs. Take the lock without waiting (if odometry has it, skip this
    // round), copy, give it back, and only then format and print
    if (mode() == Mode::Off) return;
    if (!filterLock.take(0)) return;
    const bool have = haveStepPose;
    if (have) {
        logStatus = robotFilter.getStatus();
        logMs = stepMs;
        logXIn = stepXIn;
        logYIn = stepYIn;
        logThetaRad = stepThetaRad;
    }
    filterLock.give();
    if (!have) return; // no tick of this run yet
    formatStatusLine(logLine, sizeof logLine, logMs, logXIn, logYIn, logThetaRad, logStatus);
    std::printf("%s\n", logLine);
    // the wall sensors' line, only when there are any
    if (logStatus.wallSensorCount > 0) {
        formatWallStatusLine(logLine, sizeof logLine, logMs, logStatus);
        std::printf("%s\n", logLine);
    }
}

} // namespace lemlib::gpsfusion

#endif // GPSFUSION_CORE_ONLY
