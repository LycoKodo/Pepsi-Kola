#pragma once

#include <cstdint>

// pros::Gps and pros::Distance are only named in this header, never used, so the header does not need the PROS
// headers: the host unit tests include it without them. gpsFusion.cpp includes pros/gps.hpp and
// pros/distance.hpp itself
namespace pros {
inline namespace v5 {
class Gps;
class Distance;
} // namespace v5
} // namespace pros

/**
 * GPS-aided odometry. The specification is docs/GPS_FUSION_DESIGN.md; "step 1" to "step 6" and "check 1" to
 * "check 7" in the comments are the numbers used there, and "R1" to "R4" are the sections of its "Revision 2",
 * which wins where it disagrees with the text before it.
 *
 * In short: odometry stays the estimate from moment to moment. Readings from the V5 GPS sensor that survive
 * a long list of checks slowly nudge x and y (never the heading) toward where the GPS says the robot is.
 * With the mode Off (the default) nothing here runs and odometry is exactly what it was before.
 *
 * Units: the robot's pose is in LemLib's units (inches; heading in radians, clockwise, 0 facing the 0 degree
 * wall). What the GPS reports is in PROS's units (metres from the field centre; heading in degrees).
 *
 * How a routine uses it: chassis.setPose(<measured start pose, field coordinates>), then startGpsFusion()
 * (include/autons.hpp, src/main.cpp). That is the only way a routine switches fusion on: it goes through the
 * guards configure() was given, so On on a robot that is not ready gives Shadow. Do not call setMode(On) from a
 * routine. runAuton() switches it Off again when the routine ends, before the driver drives.
 *
 * What a switch-on promises (Revision 3). R1's promise, "after a fault the pose is plain odometry again", holds
 * for one switch-on. A later setPose (lemlib::setPose tells the filter, which then starts again from the new
 * pose, exactly as setMode does) or a second setMode(Shadow or On) starts a new switch-on: the corrections fed in
 * before it stay in the pose, are never fed back out, and the total limit counts from zero again. Switching Off
 * while a fault is still feeding a correction back out leaves the rest of it in the pose.
 *
 * Wall fixes (docs/WALL_FIX_DESIGN.md; its "Revision A" wins where it disagrees with the text before it): V5
 * distance sensors looking at the field walls are a second kind of measurement in the same filter. A flat wall
 * pins down one coordinate, so a fix corrects x or y only, and only while the robot stands still, close and
 * square to one plain wall. "Wall check 1" to "wall check 13" in the comments are that design's checks. With no
 * wall sensor configured none of the wall code runs, and GPS-only results are what they were before it existed.
 */
namespace lemlib::gpsfusion {

enum class Mode {
    Off, // the default: odometry is exactly what it was before this file existed
    Shadow, // everything runs and is logged, but nothing is changed; the correction it WOULD make is kept apart
    On // corrections are fed into the pose the motions read
};

// Why fusion gave up. A fault lasts until fusion is switched on again (setMode Shadow or On). On any fault the
// correction already fed in is fed back out at slewInPerS, so a few seconds later the pose is plain odometry
// again (R1). The numbers stay as they are: new ones go at the end
enum class Fault {
    None,
    StartDisagree, // the start check: the GPS and the start pose disagree (wrong start pose, wrong alliance,
                   // sensor mounted the other way)
    RelocTooFar, // readings agreed with each other but were too far from odometry to trust a relocalisation
    RelocTooMany, // a third relocalisation in one run: a robot that keeps getting "shoved" has a bad GPS (R2)
    TotalTooLarge, // the total correction decided since switching on went over maxTotalCorrectionIn (R1)
    HeadingDisagree // most of the first readings failed the heading check: field turned round, frame mirrored
                    // or mount yaw wrong (R3)
};

// The seven checks of step 2, in the order they run. A reading that fails one is dropped and counted
// against the first check it failed. Check 5b (the sensor's own error number, R4) runs between check 5 and
// check 6 but is not in this list: it has its own counter (Status::droppedBySensorError), so the list and
// everything that reads it by number stay as they were
enum Check {
    CHECK_FINITE = 0, // 1. all numbers are numbers (PROS reports a failed read as infinity)
    CHECK_IN_FIELD, // 2. inside the field
    CHECK_NOT_STALE, // 3. the sensor was updating just before this reading (not frozen)
    CHECK_DEAD_ZONE, // 4. the camera is not too close to the wall it looks at
    CHECK_HEADING, // 5. the GPS heading agrees with ours
    CHECK_SLOW, // 6. the robot is moving and turning slowly enough
    CHECK_RATE, // 7. not too soon after the last reading that was used
    NUM_CHECKS
};

// metres (what the GPS reports) to inches (what odometry uses), as in the design
constexpr double INCHES_PER_METRE = 39.3701;
// the field is a square, plus or minus this many inches from the centre on both axes
constexpr float FIELD_HALF_WIDTH_IN = 72;
// how many odometry ticks of history are kept (64 ticks at 10 ms = 0.63 s). Check 6 looks at the 50 ms before
// the moment a reading was taken, so latencyMs + 50 must be shorter than that
constexpr int HISTORY_SIZE = 64;
// odometry runs once every this many ms (lemlib::init(): update(), then delay(10))
constexpr float ODOM_TICK_MS = 10;
// check 6 measures the robot's speed and turn rate over this much of the history
constexpr float SPEED_WINDOW_MS = 50;
// The longest latencyMs the history can serve: check 6's second window ends latencyMs ago and starts 50 ms before
// that, and the oldest entry is (HISTORY_SIZE - 1) ticks old: 630 - 50 = 580 ms. A longer latencyMs would drop
// every reading as "no history" without a word, so configure() refuses it and fusion stays Off (Revision 3)
constexpr float MAX_LATENCY_MS = (HISTORY_SIZE - 1) * ODOM_TICK_MS - SPEED_WINDOW_MS;
// the most readings the relocalisation can look back over (relocSamples is limited to this)
constexpr int MAX_RELOC_SAMPLES = 64;

/**
 * Every number the filter uses. The defaults are the design's values (revision 2 where it changed them): every
 * one of them is a first guess until it is measured on the robot (the design's tables say how). The team's own
 * copy, with a comment per number, is gpsFusionConfig in src/robot-config.cpp. It is written with designated
 * initialisers, so it must list the fields in the order they are declared here
 */
struct Config {
        float mountRightIn = 0; // where the sensor is from the robot's centre: inches to the right
        float mountForwardIn = 0; // and inches forward
        float mountYawDeg = 180; // which way the camera faces, degrees clockwise from the robot's front
        float sigmaGpsIn = 1.5; // GPS position noise, inches (1 standard deviation)
        float kDist = 0.02f; // odometry doubt gained per inch driven
        float kTurn = 0.3f; // inches of odometry doubt gained per radian turned
        float sigmaMaxIn = 5; // cap on odometry doubt, inches (R1: was 12)
        float latencyMs = 60; // how old a GPS reading is when we read it
        float maxSpeedInPerS = 6; // use the GPS only below this speed
        float maxTurnRateDegPerS = 30; // and below this turn rate
        float minPeriodMs = 100; // use at most one reading per this many ms
        float staleMs = 200; // no new reading for this long = the sensor is frozen
        float minViewIn = 20; // camera closer than this to the wall it faces = dead zone
        float headingGateDeg = 8; // the GPS heading must agree with ours this well
        float maxSensorErrorIn = 0; // check 5b: drop a reading whose get_error() is above this, inches. 0 = not used
        float gate = 2.5f; // readings more than this many standard deviations off are dropped (R1: was 3)
        float slewInPerS = 2; // fastest a correction is fed into the pose, and fed back out after a fault, in/s
        float maxStepIn = 1; // the most one reading may move the decided correction, inches (R1)
        // total correction decided since switching on above this = fault (R1: 12). Revision 3: 6, chosen on
        // 2026-11-22 from (12, 12, 2), (8, 8, 1) and (6, 6, 1) for (this, relocMaxIn, relocMaxCount) by the rule
        // fixed before the runs: the smallest whose healthy runs (calibrated robot both spins, perfect odometry,
        // 150 in all) fault at most once. All three faulted 0 times, so the smallest was taken
        float maxTotalCorrectionIn = 6;
        int startSamples = 5; // the start check averages this many readings
        float startTolIn = 8; // and faults if they disagree with the start pose by more than this
        float startSigmaIn = 1; // odometry's doubt once the start check passes: how well the start pose is known (R3)
        int startHeadingSamples = 10; // of the first this many readings that pass checks 1 to 4, 80 percent
                                      // (rounded up: 8 of 10) failing the heading check = fault (R3). 0 = not judged
        int relocSamples = 4; // relocalise after this many readings in a row fail the statistical gate (R2: was 20)
        float relocWindowMs = 2000; // all within this many ms of each other (R2)
        float relocSpreadIn = 4.5f; // each within this of their mean, inches (R2: was 2, and measured differently)
        // and their mean at most this far from odometry (further: fault) (R2: was 18, then 12). Revision 3: 6, by
        // the same rule and runs as maxTotalCorrectionIn
        float relocMaxIn = 6;
        // relocalisations allowed per run; one more = fault (R2: 2). Revision 3: 1, by the same rule and runs
        int relocMaxCount = 1;
};

// One reading from the GPS sensor, exactly as PROS returned it this tick
struct GpsSample {
        double xMetres = 0;
        double yMetres = 0;
        double headingDeg = 0;
        double errorMetres = 0; // get_error(). Its meaning is not documented, so it is only checked to be a number
};

// What to add to the pose this tick, inches
struct Correction {
        float xIn = 0;
        float yIn = 0;
};

// ------------------------------------------------------------------------------------------------------------
// Wall fixes (docs/WALL_FIX_DESIGN.md)
// ------------------------------------------------------------------------------------------------------------

// How far a wall's readings are trusted ("Safeguards: wall trust"). Every wall starts Clear until the team has
// proven on that kind of wall that the sensor reads it, and only then marks it Solid
enum class WallTrust {
    Ignore, // never used for a fix
    Clear, // clear plastic panels: the beam may pass through. Short range, strict confidence, small cap
    Solid // proven to read properly: longer range, normal confidence, larger cap
};

// The four walls, numbered as in Revision A: 0 = +y (the 0 degree wall), 1 = +x (90), 2 = -y (180), 3 = -x (270)
constexpr int NUM_WALLS = 4;
constexpr int WALL_PLUS_Y = 0;
constexpr int WALL_PLUS_X = 1;
constexpr int WALL_MINUS_Y = 2;
constexpr int WALL_MINUS_X = 3;
constexpr int NO_WALL = -1; // a ray that meets no single wall (it starts outside the field, or hits a corner)

constexpr int MAX_WALL_SENSORS = 2; // smart ports 6 and 21 are the only free ones (Revision A: fixed arrays)
constexpr int MAX_KEEP_OUTS = 8; // Revision A
constexpr int MAX_STEADY_COUNT = 16; // the most readings wall check 4 can compare (steadyCount is limited to it)

// What PROS returns from the distance sensor calls when a read fails (PROS_ERR, which is INT32_MAX), for an
// unplugged sensor among others. Written out here so the filter needs no PROS header
constexpr std::int32_t DISTANCE_READ_FAILED = 2147483647;
// What get() returns when the sensor sees nothing at all
constexpr std::int32_t DISTANCE_NOTHING_SEEN_MM = 9999;

// The thirteen wall checks, in the design's order ("Checks, in order"). A reading that fails one is dropped and
// counted against the first check it failed, per sensor
enum WallCheck {
    WCHECK_VALID = 0, // 1. not an error value, not 0, not 9999, between 20 and 2000 mm
    WCHECK_CONFIDENT, // 2. above 200 mm the sensor's confidence is high enough
    WCHECK_STILL, // 3. the robot is standing still
    WCHECK_STEADY, // 4. the last few readings agree with each other (their median is the reading used)
    WCHECK_ONE_WALL, // 5. the beam hits one wall, and that wall is not marked Ignore
    WCHECK_SQUARE, // 6. the beam is close enough to square to that wall
    WCHECK_IN_RANGE, // 7. the predicted and the measured range are both inside the wall's range
    WCHECK_PLAIN_WALL, // 8. the whole beam cone lands on that wall, clear of keep-outs and corners
    WCHECK_NOT_BLOCKED, // 9. the reading is not much shorter than predicted (something in the way)
    WCHECK_NOT_THROUGH, // 10. the reading is not much longer than predicted (through or past the wall)
    WCHECK_GATE, // 11. the statistical gate
    WCHECK_CAP, // 12. the correction is not larger than the wall's cap (larger ones are dropped, not clipped)
    WCHECK_ONCE_PER_STOP, // 13. the sensor has not already given a fix since the robot last moved enough
    NUM_WALL_CHECKS
};

// Where a wall sensor is on the robot and how its readings are corrected ("Geometry")
struct WallSensorMount {
        float rightIn = 0; // inches right of the robot's centre
        float forwardIn = 0; // inches forward of the robot's centre, to the sensor's face
        float yawDeg = 0; // which way it looks: 0 forward, 90 right, 180 back, 270 left
        float scale = 1; // bench calibration: reading in inches = scale * mm / 25.4 + offsetIn
        float offsetIn = 0;
};

// A stretch of wall with something mounted on it this season, where a beam must not land (wall check 8)
struct KeepOut {
        int wall = NO_WALL; // 0 to 3, as above
        float fromIn = 0; // where it starts and ends along the wall, in field inches: x for walls 0 and 2, y for
        float toIn = 0; // walls 1 and 3. Either order
};

/**
 * Every wall number. The defaults are the design's first values ("Numbers and their first values" and
 * Revision A); each one is a first guess until measured (the design's table says how). Field coordinates, inches
 */
struct WallConfig {
        // The inside face of each wall, inches from the field centre, by wall number: the walls are the planes
        // y = +planeIn[0], x = +planeIn[1], y = -planeIn[2], x = -planeIn[3]. An error here is a direct bias in
        // that coordinate, so measure the field (tape, three places per wall)
        float planeIn[NUM_WALLS] = {70.25f, 70.25f, 70.25f, 70.25f};
        WallTrust trust[NUM_WALLS] = {WallTrust::Clear, WallTrust::Clear, WallTrust::Clear, WallTrust::Clear};
        float wallNoiseIn = 0.3f; // reading noise, fixed part, inches (1 standard deviation)
        float wallNoiseFrac = 0.01f; // reading noise, per inch of range
        float sigmaHeadingDeg = 1; // how wrong the heading may be at a fix, degrees
        int minConfidence = 40; // wall check 2: least confidence (of 63) above 200 mm, on a Solid wall
        int minConfidenceClear = 50; // and on a Clear wall
        float wallMaxSpeedInPerS = 1; // wall check 3: "still" means moving no faster than this...
        float wallMaxTurnRateDegPerS = 3; // ...and turning no faster than this...
        float stillWindowMs = 100; // ...over this much of the history
        int steadyCount = 5; // wall check 4: this many readings in a row must agree (at most MAX_STEADY_COUNT)
        float steadySpreadIn = 0.5f; // they agree if they span at most the larger of this many inches...
        float steadySpreadFrac = 0.02f; // ...and this fraction of their median
        float sampleMinPeriodMs = 35; // Revision A: each sensor is sampled at most once every this many ms
        float maxIncidenceDeg = 10; // wall check 6: most the beam may be off square to the wall, degrees
        float minRangeIn = 1.5f; // wall check 7: shortest range used, inches
        float maxRangeClearIn = 24; // and longest, on a Clear wall
        float maxRangeSolidIn = 36; // and on a Solid wall
        float coneHalfDeg = 18; // wall check 8: the beam is a cone this many degrees either side of its centre
        float footprintMarginIn = 2; // its footprint on the wall is widened by this on each side...
        float cornerMarginIn = 6; // ...and must stay more than this from a corner
        float blockedMinIn = 1.0f; // wall check 9 (Revision A): shorter than predicted by more than
        float blockedMaxIn = 2.5f; // min(blockedMaxIn, max(blockedMinIn, blockedSigmas * s)) along the axis =
        float blockedSigmas = 2; // blocked. s = one standard deviation of the innovation
        float throughMaxIn = 4.0f; // wall check 10 (Revision A): longer than predicted by more than
        float throughSigmas = 3; // min(throughMaxIn, throughSigmas * s) along the axis = through the wall
        float capClearIn = 1.5f; // wall check 12: largest single correction on a Clear wall, inches
        float capSolidIn = 6; // and on a Solid wall
        float refixTravelIn = 6; // wall check 13: after a fix, the same sensor fixes again only after the robot
        float refixTurnDeg = 20; // has travelled this far or turned this much
        float sigmaFloorIn = 0.3f; // neither doubt ever goes below this (only while a wall sensor is configured)
        float wallStartTolIn = 3; // start check, per sensor: its first good reading this far off = fault
        int healthMinTries = 6; // health rule: needs at least this many tries before it can act
        KeepOut keepOuts[MAX_KEEP_OUTS] = {}; // this season's wall-mounted elements (wall check 8)
        int keepOutCount = 0; // how many of keepOuts are used
};

// One reading from a distance sensor, exactly as PROS returned it this tick
struct DistanceSample {
        std::int32_t mm = DISTANCE_READ_FAILED; // get(): mm, 9999 = nothing seen, PROS_ERR = the read failed
        std::int32_t confidence = DISTANCE_READ_FAILED; // get_confidence(): 0 to 63 (only meaningful above 200 mm)
};

// Where a wall sensor stands. The numbers stay as they are: new ones go at the end
enum class WallSensorState {
    Unused, // no sensor configured in this place
    WaitingForStart, // configured; its start check has not happened yet
    Working, // its start check passed: it can fix
    StartFault, // its first good reading disagreed with the pose by more than wallStartTolIn: off for the run
    HealthOff // the health rule switched it off for the run (the walls are probably clear)
};

// What one wall sensor has done since fusion was switched on, for logging and the unit tests
struct WallSensorStatus {
        WallSensorState state = WallSensorState::Unused;
        std::uint32_t samples = 0; // readings taken (at most one every sampleMinPeriodMs)
        std::uint32_t fixes = 0; // readings that corrected the pose
        std::uint32_t droppedByCheck[NUM_WALL_CHECKS] = {}; // indexed by WallCheck
        std::uint32_t healthTries = 0; // readings the health rule judged...
        std::uint32_t healthBad = 0; // ...and how many of them were invalid, low confidence or through
        float startInnovationIn = 0; // what its start check measured (the innovation of its first good reading)
        // the last reading that got as far as an innovation (passed wall checks 1 to 8)
        int lastWall = NO_WALL; // which wall it saw
        int lastAxis = -1; // which coordinate that wall fixes: 0 = x, 1 = y
        float lastPredictedIn = 0; // the range predicted from the filter's estimate ("p")
        float lastMeasuredIn = 0; // the range measured: the median of the steady readings, calibrated ("m")
        float lastInnovationIn = 0; // (p - m) times the beam's component toward the wall: + = toward +x or +y
        float lastNoiseIn = 0; // the square root of R, the reading's noise on that axis
        float lastCorrectionIn = 0; // the last fix: what it decided on its axis
        float largestCorrectionIn = 0; // the largest fix, by size
};

/**
 * Where a ray first meets a wall. `wall` is NO_WALL when it meets none: it starts on or outside a wall, it is a
 * zero direction, or it hits two walls at exactly the same distance (a corner: it belongs to neither)
 */
struct WallHit {
        int wall = NO_WALL;
        float rangeIn = 0; // how far along the ray, inches
        float alongIn = 0; // where on the wall: x for walls 0 and 2, y for walls 1 and 3, field inches
};

/**
 * Cast a ray from (fromXIn, fromYIn) along the direction (dirX, dirY) to the four wall planes of planeIn (as
 * WallConfig::planeIn), and return the nearest. The direction need not be of length 1 for the wall, but rangeIn
 * is in units of its length, so pass a unit vector to get inches
 */
WallHit castToWall(const float planeIn[NUM_WALLS], float fromXIn, float fromYIn, float dirX, float dirY);

// A snapshot of the filter, for logging and for the unit tests
struct Status {
        bool valid = true; // false: the PROS layer could not get its lock in time, and only `mode` is filled in
        Mode mode = Mode::Off;
        Fault fault = Fault::None;
        const char* faultText = "none"; // a sentence, for people
        const char* faultName = "none"; // one word (the Fault's name), for the [GPSFUSE] line
        bool unwinding = false; // after a fault: the correction fed in so far is being fed back out (R1)
        float sigmaIn = 0; // how unsure odometry is, inches (1 standard deviation): the larger of the two below
        float sigmaXIn = 0, sigmaYIn = 0; // the doubt in x and in y (the wall design keeps one per axis)
        float pendingXIn = 0, pendingYIn = 0; // correction decided but not fed in yet
        float shadowXIn = 0, shadowYIn = 0; // Shadow only: the total it would have fed in by now
        float appliedXIn = 0, appliedYIn = 0; // On only: the total fed into the pose by now
        float decidedXIn = 0, decidedYIn = 0; // every correction ever decided, added up (= fed in + pending until a
                                              // fault; after a fault it keeps the total that was decided)
        float estimateXIn = 0, estimateYIn = 0; // the filter's best guess of where the robot is (pose + pending)
        bool startCheckPassed = false;
        int startSamplesSeen = 0;
        int startSamplesNeeded = 0; // how many the start check averages (startSamples, at least 1)
        int headingSamplesSeen = 0; // readings judged for the heading fault so far (at most startHeadingSamples)
        int headingSamplesFailed = 0; // and how many of them failed the heading check
        bool headingJudged = false; // the heading fault has been decided, one way or the other
        bool sensorFrozen = true; // no new reading for staleMs (or none yet)
        std::uint32_t newReadings = 0; // readings that were new (step 1)
        std::uint32_t droppedByCheck[NUM_CHECKS] = {}; // indexed by Check
        std::uint32_t droppedBySensorError = 0; // check 5b
        std::uint32_t droppedByGate = 0; // step 4
        std::uint32_t droppedNoHistory = 0; // passed check 6's first window, but the history does not reach back to
                                            // the moment the reading was taken yet
        std::uint32_t accepted = 0; // readings that updated the filter (step 5)
        std::uint32_t relocalisations = 0;
        float maxReadingStepIn = 0; // the most any single reading moved the decided correction (step 5), inches
        float lastInnovationXIn = 0, lastInnovationYIn = 0; // GPS minus where we thought we were, last reading
        // Every reading that got an innovation (start check readings and readings that reached the gate): the
        // average innovation turned into the robot's own frame at the moment the reading was taken. A steady value
        // that is not zero is a mount offset error, and reads off directly
        std::uint32_t innovationCount = 0;
        float innovationRightAvgIn = 0, innovationForwardAvgIn = 0;
        // Every reading that reached the gate: the average of innovation^2 / (sigma^2 + sigmaGps^2). A healthy
        // sensor reads about 1.4 in the simulator, and one noisier than sigmaGpsIn says about 6 (design, "Revision 2
        // results"). Take the healthy value for this robot from the stage 1 logs, not from here
        std::uint32_t nisCount = 0;
        float nisAvg = 0;
        // Wall fixes (docs/WALL_FIX_DESIGN.md). Nothing here changes while no wall sensor is configured
        int wallSensorCount = 0; // how many are configured (0 to MAX_WALL_SENSORS)
        // Fault stage 1 (Revision A): the GPS is out and what was fed in is being fed back out; the wall sensors
        // wait, and carry on alone once the pose is plain odometry again
        bool wallsWaiting = false;
        // Fault stage 2 (Revision A): a total-limit fault with the GPS already out or absent stopped the walls too
        bool wallsStopped = false;
        WallSensorStatus walls[MAX_WALL_SENSORS];
};

const char* modeName(Mode mode);
const char* faultText(Fault fault);
const char* faultName(Fault fault);

/**
 * The [GPSFUSE] line (without a newline) for a status, written into `out`. Kept here, with no PROS in it, so the
 * unit tests can check it; the PROS layer prints it twice a second. Returns what snprintf returns
 */
int formatStatusLine(char* out, unsigned size, std::uint32_t nowMs, float poseXIn, float poseYIn, float thetaRad,
                     const Status& s);

// A wall sensor's state as one short word, for the [WALLFIX] line
const char* wallSensorStateName(WallSensorState state);

/**
 * The [WALLFIX] line (without a newline) for a status, written into `out`: per wall sensor its state, fixes,
 * drops per wall check, last innovation and last correction. Kept here, with no PROS in it, so the unit tests can
 * check it; the PROS layer prints it next to the [GPSFUSE] line when a wall sensor is configured. Returns what
 * snprintf returns
 */
int formatWallStatusLine(char* out, unsigned size, std::uint32_t nowMs, const Status& s);

/**
 * Checking the numbers (Revision 3). A wrong number in gpsFusionConfig can quietly switch off a safety limit or
 * the whole filter, so configure() checks each one against a sane range and warns once per number outside it.
 * checkConfig() returns one bit per number outside its range (bit i = problem i, 0 to NUM_CONFIG_PROBLEMS - 1);
 * formatConfigProblem() writes problem i as a sentence for people (it returns what snprintf returns). The ranges
 * are wide: they catch a value in the wrong units or a typo, not a value that is merely untuned
 */
std::uint32_t checkConfig(const Config& config);
int formatConfigProblem(char* out, unsigned size, int problem);
extern const int NUM_CONFIG_PROBLEMS;
// A latencyMs the history cannot serve (see MAX_LATENCY_MS, and NaN): configure() then leaves fusion Off
bool latencyRefused(const Config& config);

// The same for the wall numbers (configureWalls()). Up to 64 problems, so a 64-bit mask
std::uint64_t checkWallConfig(const WallConfig& config);
int formatWallConfigProblem(char* out, unsigned size, int problem);
extern const int NUM_WALL_CONFIG_PROBLEMS;

/**
 * Distance, inches, from the point (xIn, yIn) to the field wall along the direction headingRad (radians,
 * clockwise from +y). The field is the square of plus or minus FIELD_HALF_WIDTH_IN. Zero when the point is on
 * or outside the wall
 */
float distanceToWallIn(float xIn, float yIn, float headingRad);

/**
 * The filter itself, with no PROS in it, so it runs in the host simulator and the unit tests.
 * Call tick() once per odometry tick; it returns what to add to the pose this tick.
 */
class GpsFusionCore {
    public:
        explicit GpsFusionCore(const Config& config = Config());

        // Meant to be called once, before fusion is switched on
        void setConfig(const Config& config);

        /**
         * The wall sensors (docs/WALL_FIX_DESIGN.md): how many there are (0 to MAX_WALL_SENSORS, more are left
         * out), where each one is, and the wall numbers. Meant to be called once, before fusion is switched on; it
         * forgets everything the wall sensors have done. 0 sensors (the default) means none of the wall code runs
         */
        void setWallConfig(const WallConfig& wallConfig, const WallSensorMount mounts[], int count);

        // Shadow or On: forget everything and start again (the start check runs again). Off: stop; the state is
        // kept so the status can still be read, and the pose keeps the corrections already fed in (also when a
        // fault was still feeding them back out: Off means nothing runs at all)
        void setMode(Mode mode);
        Mode getMode() const;

        /**
         * One odometry tick.
         * @param timeMs the time now, ms
         * @param xIn, yIn, thetaRad the dead-reckoned pose this tick (in On, it already contains the corrections
         *        fed in on earlier ticks, because odometry adds them to its pose)
         * @param sample what the GPS read this tick, or nullptr when there is no sensor
         * @return what to add to x and y this tick. Always zero in Off and in Shadow. After a fault in On, it
         *         feeds back out what was fed in, at most slewInPerS * dt a tick, until the total fed in is zero
         */
        Correction tick(std::uint32_t timeMs, float xIn, float yIn, float thetaRad, const GpsSample* sample);

        /**
         * The same tick, with what the wall sensors read this tick as well.
         * @param wallSamples one reading per configured wall sensor, in the order of setWallConfig's mounts, or
         *        nullptr when none were read this tick
         * @param wallSampleCount how many readings wallSamples holds (more than are configured are ignored)
         */
        Correction tick(std::uint32_t timeMs, float xIn, float yIn, float thetaRad, const GpsSample* sample,
                        const DistanceSample* wallSamples, int wallSampleCount);

        Status getStatus() const;

    private:
        // One odometry tick of the past
        struct HistoryEntry {
                std::uint32_t timeMs = 0;
                float xIn = 0, yIn = 0, thetaRad = 0; // the pose as On shows it, before this tick's feed-in
                float appliedXIn = 0, appliedYIn = 0; // total correction fed in before this tick
        };

        void resetState();
        void addToHistory(const HistoryEntry& entry);
        const HistoryEntry& historyBack(int ticksBack) const; // 0 = this tick
        // the newest entry at least ageMs old, or nullptr if the history does not go back that far
        const HistoryEntry* entryAtLeastThisOld(std::uint32_t nowMs, float ageMs) const;
        // the entry closest to ageMs old, or nullptr if the history does not go back that far
        const HistoryEntry* entryClosestToAge(std::uint32_t nowMs, float ageMs) const;
        // how many ticks back the newest entry at least ageMs older than refMs is, looking no newer than fromBack
        // ticks back; -1 if the history does not go back that far
        int ticksBackAtLeast(std::uint32_t refMs, float ageMs, int fromBack) const;
        // the robot's speed (in/s) and turn rate (deg/s) by odometry between two history entries, with the
        // corrections fed in taken out (a correction being fed in is not the robot moving)
        void speedBetween(const HistoryEntry& older, const HistoryEntry& newer, float& inPerS, float& degPerS) const;
        // check 6: was the robot slow enough, over the last 50 ms and over the 50 ms before the reading was taken?
        enum class SlowVerdict { Slow, TooFast, NoHistory };
        SlowVerdict slowVerdict(std::uint32_t nowMs) const;
        void handleReading(std::uint32_t nowMs, const GpsSample& sample, float estXIn, float estYIn, float thetaRad);
        void handleGateFailure(std::uint32_t nowMs, float innovationXIn, float innovationYIn);
        // add a correction to pending and to the decided total; then the total limit (R1)
        void decide(float xIn, float yIn);
        void setFault(Fault why);
        // a doubt with the wall design's floor applied (only while a wall sensor is configured)
        float floored(float sigmaIn) const;

        // ---- wall fixes (docs/WALL_FIX_DESIGN.md)
        // What one wall sensor's beam meets, judged from the filter's estimate alone (no reading needed)
        struct WallView {
                int wall = NO_WALL;
                int axis = -1; // 0 = the wall fixes x, 1 = it fixes y
                float predictedIn = 0; // "p": the range from the sensor's face to the wall along the beam
                float alongDir = 0; // the beam's component toward the wall's axis ("d.along"; may be negative)
                float acrossDir = 0; // its component along the wall ("d.across")
                // the first of wall checks 5 to 8 that fails, judged from the estimate (check 7 on p only), or
                // NUM_WALL_CHECKS if none fails
                int failedCheck = NUM_WALL_CHECKS;
        };
        // Everything one wall sensor needs between readings
        struct WallTracker {
                WallSensorStatus status; // its counters and last values, as the status shows them
                bool haveSample = false; // the sampling rule: when it last took a reading
                std::uint32_t lastSampleMs = 0;
                // wall check 4: the latest readings in a row that passed checks 1 to 3 (inches) and their times
                float steadyIn[MAX_STEADY_COUNT] = {};
                std::uint32_t steadyMs[MAX_STEADY_COUNT] = {};
                int steadyHave = 0;
                int steadyNext = 0;
                // wall check 13: has it fixed since the robot last moved enough, and how far has it moved since
                bool fixedThisStop = false;
                float travelSinceFixIn = 0;
                float turnSinceFixRad = 0;
        };
        void resetWalls();
        // the wall sensors can still fix: at least one is configured and healthy, and stage 2 has not stopped them
        bool wallsCanCarryOn() const;
        // wall check 3's test: was the robot still over the last stillWindowMs (odometry, corrections taken out)?
        bool robotStill(std::uint32_t nowMs) const;
        WallView viewWall(int sensor, float estXIn, float estYIn, float thetaRad) const;
        void handleWallSample(int sensor, std::uint32_t nowMs, const DistanceSample& sample, float estXIn,
                              float estYIn, float thetaRad);
        // the health rule: count one try, and switch the sensor off if half or more of them were bad
        void countHealthTry(WallTracker& tracker, bool bad);

        Config config;
        Mode mode = Mode::Off;
        Mode runMode = Mode::Off; // the mode this run was switched on in (stays after switching Off)
        Fault fault = Fault::None;
        // After a fault: feeding what was fed in back out (R1). Without wall sensors this is the same as "fault
        // is not None"; with them, stage 1 ends it once the walls carry on alone (Revision A)
        bool unwinding = false;

        // the filter (design: "State kept by the filter"). The wall design keeps one doubt per axis; with no wall
        // sensor the two are always equal, and the filter is revision 2's exactly
        float sigmaXIn = 0, sigmaYIn = 0;
        float pendingXIn = 0, pendingYIn = 0;
        float fedInXIn = 0, fedInYIn = 0; // total moved out of pending (On: into the pose; Shadow: kept apart)
        float decidedXIn = 0, decidedYIn = 0;
        HistoryEntry history[HISTORY_SIZE];
        int historyCount = 0; // how many entries are filled
        int historyNext = 0; // where the next entry goes

        // step 1: telling new readings from repeats
        bool haveLastRead = false;
        double lastReadX = 0, lastReadY = 0, lastReadHeading = 0;
        bool haveNewReading = false;
        std::uint32_t lastNewMs = 0;
        // check 7
        bool haveUsedReading = false;
        std::uint32_t lastUsedMs = 0;

        // start check
        bool startPassed = false;
        int startCount = 0;
        float startSumXIn = 0, startSumYIn = 0;

        // the heading fault (R3)
        int headingSeen = 0;
        int headingFailed = 0;
        bool headingJudged = false;

        // relocalisation: the innovations of the readings in a row that failed the gate, and when they came (a ring)
        float relocXIn[MAX_RELOC_SAMPLES] = {};
        float relocYIn[MAX_RELOC_SAMPLES] = {};
        std::uint32_t relocMs[MAX_RELOC_SAMPLES] = {};
        int relocRun = 0; // how many in a row so far
        int relocNext = 0; // where the next one goes

        // counters and last values, for the status
        std::uint32_t newReadings = 0;
        std::uint32_t droppedByCheck[NUM_CHECKS] = {};
        std::uint32_t droppedBySensorError = 0;
        std::uint32_t droppedByGate = 0;
        std::uint32_t droppedNoHistory = 0;
        std::uint32_t accepted = 0;
        std::uint32_t relocalisations = 0;
        float maxReadingStepIn = 0;
        std::uint32_t innovationCount = 0;
        float innovationRightSumIn = 0, innovationForwardSumIn = 0;
        std::uint32_t nisCount = 0;
        float nisSum = 0;
        float lastInnovationXIn = 0, lastInnovationYIn = 0;
        bool haveTick = false;
        std::uint32_t lastTickMs = 0;
        float estimateXIn = 0, estimateYIn = 0;

        // wall fixes
        WallConfig wallConfig;
        WallSensorMount wallMounts[MAX_WALL_SENSORS];
        int wallSensorCount = 0;
        WallTracker wallTrackers[MAX_WALL_SENSORS];
        bool wallsStopped = false; // fault stage 2 (Revision A)
};

// ------------------------------------------------------------------------------------------------------------
// The robot's side (PROS). One filter for the robot, fed by lemlib::update()
// ------------------------------------------------------------------------------------------------------------

/**
 * Call once from initialize(), after chassis.calibrate(). sensor may be nullptr (no GPS: fusion then never
 * changes anything). Set the sensor's own PROS offset to zero first: this code removes the lever arm itself.
 *
 * The On guards (Revision 3) live here, so nothing can walk round them: mountMeasured and odometryCalibrated say
 * whether the GPS mount has been measured and odometry calibrated (src/main.cpp passes GPS_MOUNT_MEASURED and
 * ODOMETRY_CALIBRATED from robot-config.hpp). On is allowed only when both are true; otherwise setMode(On) gives
 * Shadow and the logging task says which is missing.
 *
 * It also checks every number in config (checkConfig) and has the logging task print one warning per number
 * outside its sane range; a latencyMs the history cannot serve is refused, and fusion then stays Off.
 *
 * The first call starts the logging task (below the default priority), which prints every line this code prints
 */
void configure(pros::Gps* sensor, const Config& config, bool mountMeasured, bool odometryCalibrated);

/**
 * Wall sensors (docs/WALL_FIX_DESIGN.md). Call once from initialize(), after configure(), only if wall-facing
 * distance sensors are fitted: sensors[i] is mounted as mounts[i]; count is 0 to MAX_WALL_SENSORS (more are left
 * out). Without this call (the default) no distance sensor is read, nothing new runs and nothing new is printed.
 * With it, every odometry tick reads each sensor and a [WALLFIX] line is printed next to the [GPSFUSE] line.
 * Its numbers are checked like configure()'s, with a warning per number outside its sane range
 */
void configureWalls(pros::Distance* const sensors[], const WallSensorMount mounts[], int count,
                    const WallConfig& config);

// Off is the boot default. Shadow or On: the filter starts again from the pose odometry has now (a new switch-on:
// see "What a switch-on promises" at the top). On gives Shadow unless configure() was told the robot is ready.
// Safe to call from any task, never waits more than a few ms and never prints: Off takes effect at once even if
// the lock cannot be had; Shadow or On that cannot get the lock leave fusion Off, and the logging task says why.
// Routines call startGpsFusion() (include/autons.hpp), not this
void setMode(Mode mode);
Mode mode();

// A snapshot of the filter. Safe to call from any task; waits a few ms at most (then Status::valid is false)
Status status();

/**
 * Called by lemlib::setPose (src/lemlib/chassis/odom.cpp), and by nothing else, around its write of the pose:
 * pauseForSetPose() before it, restartAfterSetPose() after it. With fusion Off, pauseForSetPose() reads one
 * atomic, changes nothing and returns Off, and restartAfterSetPose() is not called, so a setPose is exactly what
 * it was without fusion. Otherwise the filter starts again from the new pose exactly as setMode(the same mode)
 * does: nothing is fed back out (the old pose and its corrections are gone), the start checks run again and the
 * history is cleared. Waits at most a few ms; if the lock cannot be had, fusion is switched Off (the safe side)
 * and the logging task says so
 */
Mode pauseForSetPose();
void restartAfterSetPose(Mode modeBefore);

/**
 * Called by lemlib::update() at the end of every odometry tick, only when the mode is not Off. Reads the
 * sensors, runs the filter and returns what to add to the pose. Never prints, formats text or waits: the logging
 * task does the printing. Not for anyone else to call
 */
Correction step(float xIn, float yIn, float thetaRad);

/**
 * One round of the logging task: print the messages left by configure, setMode and setPose, then (only while the
 * mode is not Off) the [GPSFUSE] line and, with wall sensors, the [WALLFIX] line. The logging task calls it every
 * 500 ms; it is here so the host test can call it directly. Nothing else may call it: printf can wait on the
 * serial port for as long as it likes, and only the logging task is allowed to
 */
void printStatus();

} // namespace lemlib::gpsfusion
