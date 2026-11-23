// Route mode; see route.hpp for the route file and tools/sim/README.md for the outputs.
#include "route.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include "dist_model.hpp"
#include "gps_model.hpp"
#include "json.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/odom.hpp"
#include "pros/distance.hpp"
#include "pros/gps.hpp"
#include "pros/motors.h"
#include "world.hpp"

// The Makefile says whether this DUT has GPS fusion (chassis/gpsFusion.cpp). Older DUTs do not: route mode
// still runs on them, with fusion Off, which is how A1 compares them byte for byte
#ifndef DUT_HAS_GPSFUSION
#define DUT_HAS_GPSFUSION 0
#endif
#if DUT_HAS_GPSFUSION
#include "lemlib/chassis/gpsFusion.hpp"
#endif

namespace sim {

namespace {

constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr double DEG_TO_RAD = M_PI / 180.0;
// the PID tab's tolerances and window, used for each move's overshoot and settle time
constexpr double SETTLE_TOL_IN = 1.0;
constexpr double SETTLE_TOL_DEG = 1.0;
constexpr int SETTLE_WINDOW_AFTER_MS = 300;
// two runs' trajectories "still coincide" while the true robots are this close
constexpr double COINCIDE_IN = 0.1;
constexpr double COINCIDE_DEG = 0.1;
// a shove counts as recovered once the fused error stays within the tolerance for this long
constexpr int RECOVER_HOLD_MS = 1000;
// the model states of this many sampled readings are kept per wall sensor (the filter's MAX_STEADY_COUNT)
constexpr int MAX_STEADY_SAMPLES_KEPT = 16;
#if DUT_HAS_GPSFUSION
// the filter's wall check 8 cone (WallConfig::coneHalfDeg's first value): each fix is also judged against what a
// cone this wide really meets from the true pose (test W7)
constexpr double CHECK8_CONE_DEG = 18;
#endif

// ------------------------------------------------------------------------------------------------ the route
struct Step {
        enum Kind { POINT, TURN, POSE, WAIT } kind = WAIT;
        int line = 0;
        std::string text;
        double x = 0, y = 0, heading = 0;
        int timeoutMs = 0;
        bool forwards = true;
        int direction = 0; // turn: 0 the shorter way (LemLib's AUTO), 1 clockwise, -1 counter-clockwise
        double maxSpeed = 127, minSpeed = 0, earlyExit = 0, lead = 0.6, drift = 0;
        int waitMs = 0;
};

struct Route {
        bool haveStart = false;
        double x = 0, y = 0, heading = 0;
        std::vector<Step> steps;
};

Route loadRoute(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot read route file " + path);
    Route route;
    std::string line;
    int lineNo = 0;
    while (std::getline(in, line)) {
        lineNo++;
        const std::string original = line;
        const size_t hash = line.find('#');
        if (hash != std::string::npos) line.erase(hash);
        std::istringstream words(line);
        std::vector<std::string> w;
        for (std::string s; words >> s;) w.push_back(s);
        if (w.empty()) continue;
        auto fail = [&](const std::string& why) {
            throw std::runtime_error(path + ":" + std::to_string(lineNo) + ": " + why + " (" + original + ")");
        };
        // numbers first, then words: "back" or key=value
        std::vector<double> nums;
        std::map<std::string, double> opts;
        bool back = false;
        int direction = 0;
        for (size_t i = 1; i < w.size(); i++) {
            const size_t eq = w[i].find('=');
            char* end = nullptr;
            const double v = std::strtod(w[i].c_str(), &end);
            if (eq != std::string::npos) opts[w[i].substr(0, eq)] = std::atof(w[i].c_str() + eq + 1);
            else if (w[i] == "back") back = true;
            else if (w[i] == "cw") direction = 1;
            else if (w[i] == "ccw") direction = -1;
            else if (end && *end == '\0') nums.push_back(v);
            else fail("cannot read '" + w[i] + "'");
        }
        auto opt = [&](const char* key, double def) {
            auto it = opts.find(key);
            if (it == opts.end()) return def;
            const double v = it->second;
            opts.erase(it);
            return v;
        };
        auto need = [&](size_t n) {
            if (nums.size() != n) fail("needs " + std::to_string(n) + " numbers");
        };
        const std::string& cmd = w[0];
        if (cmd == "start") {
            need(3);
            if (route.haveStart) fail("a second start");
            route.haveStart = true;
            route.x = nums[0];
            route.y = nums[1];
            route.heading = nums[2];
            continue;
        }
        if (!route.haveStart) fail("the first command must be start");
        Step s;
        s.line = lineNo;
        s.text = original.substr(0, original.find('#'));
        while (!s.text.empty() && std::isspace(static_cast<unsigned char>(s.text.back()))) s.text.pop_back();
        s.forwards = !back;
        if (cmd == "point") {
            need(2);
            s.kind = Step::POINT;
            s.x = nums[0];
            s.y = nums[1];
            s.timeoutMs = int(opt("timeout", 4000));
            s.maxSpeed = opt("max", 127);
            s.minSpeed = opt("min", 0);
            s.earlyExit = opt("early", 0);
        } else if (cmd == "turn") {
            need(1);
            s.kind = Step::TURN;
            s.heading = nums[0];
            s.direction = direction;
            s.timeoutMs = int(opt("timeout", 2500));
            s.maxSpeed = opt("max", 127);
        } else if (cmd == "pose") {
            need(3);
            s.kind = Step::POSE;
            s.x = nums[0];
            s.y = nums[1];
            s.heading = nums[2];
            s.timeoutMs = int(opt("timeout", 4000));
            s.lead = opt("lead", 0.6);
            s.maxSpeed = opt("max", 127);
            s.minSpeed = opt("min", 0);
            s.drift = opt("drift", 0);
        } else if (cmd == "wait") {
            need(1);
            s.kind = Step::WAIT;
            s.waitMs = int(nums[0]);
        } else {
            fail("unknown command " + cmd);
        }
        if (!opts.empty()) fail("unknown option " + opts.begin()->first);
        if (direction != 0 && s.kind != Step::TURN) fail("cw and ccw are for turn only");
        route.steps.push_back(s);
    }
    if (!route.haveStart) throw std::runtime_error(path + ": no start line");
    return route;
}

// ------------------------------------------------------------------------------------------------ the log
// What the filter did with a new reading this tick (column `fate`). 1 to 7: dropped by that check
enum Fate {
    FATE_NONE = 0, // no new reading reached the filter this tick
    FATE_GATE = 8, // dropped by the statistical gate (step 4)
    FATE_NO_HISTORY = 9, // passed the checks, but the history does not reach back latencyMs yet
    FATE_START_CHECK = 10, // used by the start check (never corrects anything)
    FATE_ACCEPTED = 11, // updated the filter (step 5)
    FATE_SENSOR_ERROR = 12 // dropped by check 5b (the sensor's own error number; off by default)
};

struct Tick {
        int t = 0; // ms since the route started (setPose and fusion on)
        int step = 0; // route step running (0 = the start wait, 1 = the first command after start)
        int moving = 0; // a motion is running
        double trueX = 0, trueY = 0, trueDeg = 0; // the real robot
        double poseX = 0, poseY = 0, poseDeg = 0; // what the motions read (lemlib::getPose)
        double odomX = 0, odomY = 0; // odometry alone: the pose minus every correction fed into it
        double fusedX = 0, fusedY = 0; // On: the pose; Shadow: pose + what it would have fed in; Off: the pose
        double estX = NAN, estY = NAN; // the filter's best guess (pose + fed + pending)
        double sigma = NAN, pendX = NAN, pendY = NAN;
        double corrX = 0, corrY = 0; // correction fed in this tick (On: into the pose; Shadow: kept apart)
        double gpsX = NAN, gpsY = NAN, gpsDeg = NAN; // what the sensor reports now, inches / degrees
        int gpsNew = 0, gpsState = -1, gpsOutlier = 0;
        double gpsErrNow = NAN; // reported position minus where the camera is now, inches
        double gpsWall = NAN; // camera to the wall it faces now, inches
        int fate = FATE_NONE, reloc = 0, fault = 0;
        int cmdL = 0, cmdR = 0;
        double errOdom = 0, errFused = 0; // distance from the truth, inches
};

struct StepRecord {
        int n = 0;
        const Step* step = nullptr;
        int startMs = 0, endMs = 0;
        bool timedOut = false;
        lemlib::Pose poseAtStart {0, 0, 0}; // degrees, what the motion read
};

struct ShoveRecord {
        int startMs = 0, lengthMs = 0; // route time
        double rightIn = 0, forwardIn = 0;
};

// ------------------------------------------------------------------------------------------------ wall sensors
// The simulated distance sensors (bench/dist_model.*), one model for both, like gpsModel() for the GPS
DistModel& distModel() {
    static DistModel model;
    return model;
}

// The smart ports of the wall sensors: sensor 0 on port 6, sensor 1 on port 21, the two free ports the wall design
// names. include/ports.hpp has no wall sensor yet (none is fitted), so they are written here
constexpr int WALL_SENSOR_PORTS[DIST_MAX_SENSORS] = {6, 21};

// What the filter did with a wall sensor's reading this tick (CSV column wN_fate). 1 to 13: dropped by that wall
// check (docs/WALL_FIX_DESIGN.md, "Checks, in order")
enum WallFate {
    WFATE_NONE = 0, // not sampled this tick (between samples, or the sensor is stopped or waiting)
    WFATE_START_OK = 14, // its start check passed (that reading fixes nothing)
    WFATE_START_FAULT = 15, // its start check failed: the sensor is off for the run
    WFATE_FIX = 16, // the reading corrected the pose
    WFATE_HEALTH_OFF = 17, // it passed wall check 10 and the health rule switched the sensor off on it
    WFATE_OTHER = 18 // sampled, but none of the above moved (should never happen; counted so it would show)
};
#if DUT_HAS_GPSFUSION
// The fates that come with an innovation (the reading passed wall checks 1 to 8)
bool fateHasInnovation(int fate) { return fate >= 9 && fate <= WFATE_HEALTH_OFF; }
#endif

// One wall sensor at one odometry tick (CSV columns wN_*)
struct WallSensorTick {
        int mm = 0, conf = 0; // what the sensor reports now, PROS units
        int isNew = 0; // the model made a new reading since the last tick
        int modelState = -1; // DistState of the current reading
        double cleanIn = NAN; // the model's range along the centre ray to the walls and elements, from the truth
        double readIn = NAN; // the reading as the filter calibrates it (told scale and offset); nan if not valid
        int state = -1; // the filter's WallSensorState (-1 fusion Off)
        int fate = WFATE_NONE;
        int fixes = 0;
        int axis = -1; // with an innovation: which coordinate its wall fixes, 0 x, 1 y
        double correction = 0; // a fix this tick: what it decided on its axis, in
        double predictedIn = NAN, measuredIn = NAN, innovationIn = NAN; // p, m and the innovation (with one)
        double predictedNowIn = NAN; // the range predicted from the filter's estimate now (bench's own cast)
};

struct WallTick {
        double sigmaX = NAN, sigmaY = NAN;
        int waiting = 0, stopped = 0;
        WallSensorTick sensor[DIST_MAX_SENSORS];
};

// A fix, as the bench saw it
struct WallFixRecord {
        int t = 0, step = 0, wall = -1, axis = -1;
        double correction = 0, innovation = 0, predicted = 0, measured = 0;
        double errBefore = 0, errAfter = 0; // the filter's estimate minus the truth on the fix's axis
        double impliedErr = 0; // where the reading put the robot on that axis, minus the truth
        double cleanIn = NAN;
        int modelState = -1; // of the newest reading
        int statesMask = 0; // the model states of the readings the median came from, bit 1 << state
        int coneMaskModel = 0, coneMask18 = 0; // what the true cone meets (DIST_CONE_* bits), model's angle and 18
        double trueX = 0, trueY = 0, trueDeg = 0;
};

// Everything the bench keeps about one wall sensor over the run
struct WallBook {
        std::vector<WallFixRecord> fixes;
        int startMs = -1; // when its start check happened (-1 never)
        bool startFault = false;
        int healthOffMs = -1;
        // clean readings (the model read the wall itself) refused by wall check 9 or 10, the stops it happened at,
        // and the largest error of the estimate on that axis at such a moment
        int cleanBlocked = 0, cleanThrough = 0;
        std::vector<int> stopsBlocked, stopsThrough;
        double maxCleanRefusedErrIn = 0;
        int sampledByModelState[10] = {}; // readings the filter sampled, by the model's DistState
        int fixesNotClean = 0; // fixes whose readings were not all of the wall itself
        double maxErrorAddedIn = -INFINITY; // the largest |error after| - |error before| of any fix
        std::uint32_t lastCount = 0; // the model's reading count at the last tick
        int recentStates[MAX_STEADY_SAMPLES_KEPT] = {}; // model states of the latest sampled readings (a ring)
        int recentNext = 0, recentHave = 0;
};

struct Run {
        const RouteOptions* o = nullptr;
        lemlib::Chassis* chassis = nullptr;
        bool logging = false;
        std::uint32_t t0 = 0;
        int currentStep = 0;
        bool fusionOn = false; // Shadow or On
        bool fusionApplies = false; // On
        std::vector<Tick> ticks;
        FILE* poseTrace = nullptr;
        std::uint32_t lastGpsOutputs = 0;
#if DUT_HAS_GPSFUSION
        lemlib::gpsfusion::Status prev;
        lemlib::gpsfusion::Status last;
#endif
        double prevFedX = 0, prevFedY = 0;
        // bookkeeping on the readings the filter used
        int acceptedOutliers = 0, startCheckOutliers = 0, acceptedBlind = 0, acceptedCount = 0;
        double maxAcceptedOutlierIn = 0, maxAcceptedReadingErrIn = 0, sumSquaredAcceptedErr = 0;
        int faultMs = -1;
        double maxStepIn = 0;
        int maxStepMs = -1;
        // revision 2 bookkeeping: the decided total against its limit, and what was fed in around a fault
        double maxAcceptedBlindErrIn = 0;
        double maxDecidedIn = 0;
        int decidedOverLimitMs = -1; // first tick the decided total was past maxTotalCorrectionIn
        double maxFedIn = 0, maxFedBeforeFaultIn = 0, fedAtFaultIn = NAN;
        int lastFedOverHalfMs = -1; // after the fault: the last tick more than 0.5 in was still fed in
        int unwoundMs = -1; // after the fault: the first tick from which nothing at all is fed in
        double totalLimitIn = 0;
        // wall sensors (--wall-sensor); none by default, and then nothing below is used
        int wallCount = 0;
        std::vector<WallTick> wallTicks; // one per Tick
        WallBook wallBook[DIST_MAX_SENSORS];
#if DUT_HAS_GPSFUSION
        lemlib::gpsfusion::WallConfig wallConfig;
        lemlib::gpsfusion::WallSensorMount wallMounts[lemlib::gpsfusion::MAX_WALL_SENSORS];
#endif
};

Run R;

#if DUT_HAS_GPSFUSION
// The model states of a wall sensor's latest `count` sampled readings, as bits (1 << DistState)
int recentStatesMask(const WallBook& book, int count) {
    int mask = 0;
    for (int j = 0; j < std::min(count, book.recentHave); j++)
        mask |= 1 << book.recentStates[(book.recentNext - 1 - j + 2 * MAX_STEADY_SAMPLES_KEPT) % MAX_STEADY_SAMPLES_KEPT];
    return mask;
}

// The wall sensors' part of a logged tick: what the simulated sensors read now and, with fusion on (s and prev not
// null), what the filter did with it, told apart by which of its counters moved since the last tick
void logWalls(const Tick& k, const lemlib::Pose& pose, const lemlib::gpsfusion::Status* s,
              const lemlib::gpsfusion::Status* prev) {
    using namespace lemlib::gpsfusion;
    const World& w = world();
    const DistModel& model = distModel();
    WallTick wt;
    if (s != nullptr) {
        wt.sigmaX = s->sigmaXIn;
        wt.sigmaY = s->sigmaYIn;
        wt.waiting = s->wallsWaiting;
        wt.stopped = s->wallsStopped;
    }
    for (int i = 0; i < R.wallCount; i++) {
        WallSensorTick& ws = wt.sensor[i];
        WallBook& book = R.wallBook[i];
        const DistReading r = model.read(i);
        const DistOutputInfo& info = model.info(i);
        ws.mm = r.mm;
        ws.conf = r.confidence;
        ws.isNew = info.count != book.lastCount;
        book.lastCount = info.count;
        ws.modelState = info.state;
        ws.cleanIn = info.wallIn;
        // the reading as the filter calibrates it ("Reading: m = scale * mm / 25.4 + offset"), when wall check 1
        // would call it valid
        const WallSensorMount& m = R.wallMounts[i];
        if (r.mm != DISTANCE_READ_FAILED && r.mm != DISTANCE_NOTHING_SEEN_MM && r.mm >= 20 && r.mm <= 2000)
            ws.readIn = m.scale * double(r.mm) / 25.4 + m.offsetIn;
        if (s == nullptr) continue;

        // the range the filter would predict now from its estimate: the same cast as its own (castToWall), with
        // the mount and the wall planes it was told. (On a tick with a fix this is after the fix)
        const double sn = std::sin(pose.theta), cs = std::cos(pose.theta);
        const double sx = s->estimateXIn + m.rightIn * cs + m.forwardIn * sn;
        const double sy = s->estimateYIn - m.rightIn * sn + m.forwardIn * cs;
        const double beam = pose.theta + m.yawDeg * DEG_TO_RAD;
        const WallHit hit =
            castToWall(R.wallConfig.planeIn, float(sx), float(sy), float(std::sin(beam)), float(std::cos(beam)));
        if (hit.wall != NO_WALL) ws.predictedNowIn = hit.rangeIn;

        const WallSensorStatus& a = s->walls[i];
        const WallSensorStatus& b = prev->walls[i];
        ws.state = int(a.state);
        ws.fixes = int(a.fixes);
        if (a.samples > b.samples) {
            for (int c = 0; c < NUM_WALL_CHECKS && ws.fate == WFATE_NONE; c++)
                if (a.droppedByCheck[c] > b.droppedByCheck[c]) ws.fate = c + 1;
            if (ws.fate == WFATE_NONE) {
                if (a.fixes > b.fixes) ws.fate = WFATE_FIX;
                else if (b.state == WallSensorState::WaitingForStart && a.state == WallSensorState::Working)
                    ws.fate = WFATE_START_OK;
                else if (b.state == WallSensorState::WaitingForStart && a.state == WallSensorState::StartFault)
                    ws.fate = WFATE_START_FAULT;
                else if (a.state == WallSensorState::HealthOff && b.state != WallSensorState::HealthOff)
                    ws.fate = WFATE_HEALTH_OFF;
                else ws.fate = WFATE_OTHER;
            }
            book.recentStates[book.recentNext] = info.state;
            book.recentNext = (book.recentNext + 1) % MAX_STEADY_SAMPLES_KEPT;
            if (book.recentHave < MAX_STEADY_SAMPLES_KEPT) book.recentHave++;
            if (info.state >= 0 && info.state < 10) book.sampledByModelState[info.state]++;
        }
        if (fateHasInnovation(ws.fate)) {
            ws.axis = a.lastAxis;
            ws.predictedIn = a.lastPredictedIn;
            ws.measuredIn = a.lastMeasuredIn;
            ws.innovationIn = a.lastInnovationIn;
        }
        if (ws.fate == WFATE_FIX) ws.correction = a.lastCorrectionIn;
        if (b.state == WallSensorState::WaitingForStart && a.state != WallSensorState::WaitingForStart &&
            book.startMs < 0) {
            book.startMs = k.t;
            book.startFault = a.state == WallSensorState::StartFault;
        }
        if (a.state == WallSensorState::HealthOff && book.healthOffMs < 0) book.healthOffMs = k.t;
    }

    if (s != nullptr) {
        // The error of the filter's estimate on each axis after this tick. Taken back sensor by sensor, last first,
        // because the filter handles them in order: what a sensor's fix saw "before" is the estimate after the
        // sensors before it. (Within a tick the GPS comes before the walls, so its decision is in "before")
        double errAxis[2] = {s->estimateXIn - w.x, s->estimateYIn - w.y};
        const int steady = std::clamp(R.wallConfig.steadyCount, 1, MAX_STEADY_SAMPLES_KEPT);
        const int cleanBits = (1 << DIST_WALL) | (1 << DIST_CLEAR_WALL);
        for (int i = R.wallCount - 1; i >= 0; i--) {
            const WallSensorTick& ws = wt.sensor[i];
            WallBook& book = R.wallBook[i];
            const int mask = recentStatesMask(book, steady);
            if (ws.fate == WFATE_FIX && (ws.axis == 0 || ws.axis == 1)) {
                const WallSensorStatus& a = s->walls[i];
                WallFixRecord f;
                f.t = k.t;
                f.step = k.step;
                f.wall = a.lastWall;
                f.axis = ws.axis;
                f.correction = ws.correction;
                f.innovation = ws.innovationIn;
                f.predicted = ws.predictedIn;
                f.measured = ws.measuredIn;
                f.errAfter = errAxis[ws.axis];
                f.errBefore = errAxis[ws.axis] - ws.correction;
                errAxis[ws.axis] = f.errBefore;
                f.impliedErr = f.errBefore + ws.innovationIn;
                f.cleanIn = ws.cleanIn;
                f.modelState = ws.modelState;
                f.statesMask = mask;
                f.coneMaskModel = model.coneMaskAt(w.now, w.x, w.y, w.heading, i, model.info(i).coneHalfDeg);
                f.coneMask18 = model.coneMaskAt(w.now, w.x, w.y, w.heading, i, CHECK8_CONE_DEG);
                f.trueX = w.x;
                f.trueY = w.y;
                f.trueDeg = w.heading * RAD_TO_DEG;
                book.maxErrorAddedIn = std::max(book.maxErrorAddedIn, std::fabs(f.errAfter) - std::fabs(f.errBefore));
                if ((mask & ~cleanBits) != 0) book.fixesNotClean++;
                book.fixes.push_back(f);
            }
            // a clean reading (every reading its median came from was of the wall itself) refused as blocked or
            // through: Revision A asks how often, at which stops, and how far off the pose was then
            if ((ws.fate == WCHECK_NOT_BLOCKED + 1 || ws.fate == WCHECK_NOT_THROUGH + 1) && mask != 0 &&
                (mask & ~cleanBits) == 0 && (ws.axis == 0 || ws.axis == 1)) {
                const bool blocked = ws.fate == WCHECK_NOT_BLOCKED + 1;
                (blocked ? book.cleanBlocked : book.cleanThrough)++;
                std::vector<int>& stops = blocked ? book.stopsBlocked : book.stopsThrough;
                if (std::find(stops.begin(), stops.end(), k.step) == stops.end()) stops.push_back(k.step);
                book.maxCleanRefusedErrIn = std::max(book.maxCleanRefusedErrIn, std::fabs(errAxis[ws.axis]));
            }
        }
    }
    R.wallTicks.push_back(wt);
}
#endif

void logTick(std::uint32_t now) {
    World& w = world();
    if (R.poseTrace && w.odomStrobe) {
        const lemlib::Pose p = lemlib::getPose(true);
        std::fprintf(R.poseTrace, "%u %a %a %a\n", now, double(p.x), double(p.y), double(p.theta));
    }
    if (!R.logging || !w.odomStrobe) return;
    Tick k;
    k.t = int(now - R.t0);
    k.step = R.currentStep;
    k.moving = R.chassis->isInMotion();
    k.trueX = w.x;
    k.trueY = w.y;
    k.trueDeg = w.heading * RAD_TO_DEG;
    const lemlib::Pose pose = lemlib::getPose(true);
    k.poseX = pose.x;
    k.poseY = pose.y;
    k.poseDeg = pose.theta * RAD_TO_DEG;
    k.odomX = k.fusedX = pose.x;
    k.odomY = k.fusedY = pose.y;

    const GpsModel& gps = gpsModel();
    const bool haveGps = !R.o->noGps && gps.hasOutput();
    if (haveGps) {
        k.gpsX = gps.outXIn();
        k.gpsY = gps.outYIn();
        k.gpsDeg = gps.outHeadingDeg();
        k.gpsNew = gps.outputCount() != R.lastGpsOutputs;
        R.lastGpsOutputs = gps.outputCount();
        k.gpsState = gps.stateNow();
        k.gpsOutlier = gps.outlierNow();
        k.gpsErrNow = gps.errorVsNowIn(w);
        k.gpsWall = gps.wallDistanceNowIn(w);
    }

#if DUT_HAS_GPSFUSION
    if (R.fusionOn) {
        using namespace lemlib::gpsfusion;
        const Status s = status();
        k.sigma = s.sigmaIn;
        k.pendX = s.pendingXIn;
        k.pendY = s.pendingYIn;
        k.estX = s.estimateXIn;
        k.estY = s.estimateYIn;
        const double fedX = R.fusionApplies ? s.appliedXIn : s.shadowXIn;
        const double fedY = R.fusionApplies ? s.appliedYIn : s.shadowYIn;
        k.corrX = fedX - R.prevFedX;
        k.corrY = fedY - R.prevFedY;
        R.prevFedX = fedX;
        R.prevFedY = fedY;
        if (R.fusionApplies) {
            k.odomX = pose.x - s.appliedXIn;
            k.odomY = pose.y - s.appliedYIn;
        } else {
            k.fusedX = pose.x + s.shadowXIn;
            k.fusedY = pose.y + s.shadowYIn;
        }
        const double stepIn = std::hypot(k.corrX, k.corrY);
        if (stepIn > R.maxStepIn) {
            R.maxStepIn = stepIn;
            R.maxStepMs = k.t;
        }
        // which counter moved: that is what happened to this tick's new reading
        const Status& p = R.prev;
        for (int c = 0; c < NUM_CHECKS && k.fate == FATE_NONE; c++)
            if (s.droppedByCheck[c] > p.droppedByCheck[c]) k.fate = c + 1;
        if (k.fate == FATE_NONE && s.droppedBySensorError > p.droppedBySensorError) k.fate = FATE_SENSOR_ERROR;
        if (k.fate == FATE_NONE) {
            if (s.droppedByGate > p.droppedByGate) k.fate = FATE_GATE;
            else if (s.droppedNoHistory > p.droppedNoHistory) k.fate = FATE_NO_HISTORY;
            else if (s.startSamplesSeen > p.startSamplesSeen) k.fate = FATE_START_CHECK;
            else if (s.accepted > p.accepted) k.fate = FATE_ACCEPTED;
        }
        k.reloc = int(s.relocalisations);
        k.fault = int(s.fault);
        if (s.fault != Fault::None && R.faultMs < 0) R.faultMs = k.t;
        // the decided total against the total limit (R1), and the total fed in before and after a fault
        const double decided = std::hypot(double(s.decidedXIn), double(s.decidedYIn));
        R.maxDecidedIn = std::max(R.maxDecidedIn, decided);
        if (decided > R.totalLimitIn && R.decidedOverLimitMs < 0) R.decidedOverLimitMs = k.t;
        const double fedLen = std::hypot(fedX, fedY);
        R.maxFedIn = std::max(R.maxFedIn, fedLen);
        if (R.faultMs < 0 || k.t < R.faultMs) R.maxFedBeforeFaultIn = std::max(R.maxFedBeforeFaultIn, fedLen);
        if (R.faultMs >= 0) {
            if (std::isnan(R.fedAtFaultIn)) R.fedAtFaultIn = fedLen;
            if (fedLen > 0.5) R.lastFedOverHalfMs = k.t;
            if (fedLen != 0) R.unwoundMs = -1;
            else if (R.unwoundMs < 0) R.unwoundMs = k.t;
        }
        // the reading the filter just used is the sensor's current output
        if (haveGps && k.fate == FATE_ACCEPTED) {
            R.acceptedCount++;
            if (gps.outlierNow()) {
                R.acceptedOutliers++;
                R.maxAcceptedOutlierIn = std::max(R.maxAcceptedOutlierIn, gps.outlierSizeNow());
            }
            if (gps.stateNow() != GPS_SEES) {
                R.acceptedBlind++;
                R.maxAcceptedBlindErrIn = std::max(R.maxAcceptedBlindErrIn, gps.errorVsDelayedIn());
            }
            R.maxAcceptedReadingErrIn = std::max(R.maxAcceptedReadingErrIn, gps.errorVsDelayedIn());
            R.sumSquaredAcceptedErr += gps.errorVsDelayedIn() * gps.errorVsDelayedIn();
        }
        if (haveGps && k.fate == FATE_START_CHECK && gps.outlierNow()) R.startCheckOutliers++;
        if (R.wallCount > 0) logWalls(k, pose, &s, &R.prev);
        R.prev = s;
        R.last = s;
    } else if (R.wallCount > 0) {
        logWalls(k, pose, nullptr, nullptr);
    }
#endif
    k.cmdL = w.cmd[0];
    k.cmdR = w.cmd[1];
    k.errOdom = std::hypot(k.odomX - w.x, k.odomY - w.y);
    k.errFused = std::hypot(k.fusedX - w.x, k.fusedY - w.y);
    R.ticks.push_back(k);
}

// ------------------------------------------------------------------------------------------------ outputs
struct Column {
        const char* name;
        const char* help;
};
// the route CSV, one row per odometry tick (10 ms). README lists the same
const Column COLUMNS[] = {
    {"t_ms", "ms since the route started (setPose, fusion switched on)"},
    {"step", "route step running: 0 = --start-wait, 1 = first command after start, past the end = done"},
    {"moving", "a motion is running"},
    {"true_x", "the real robot centre, in"},
    {"true_y", ""},
    {"true_heading", "deg, clockwise"},
    {"pose_x", "the pose the motions read (lemlib::getPose), in"},
    {"pose_y", ""},
    {"pose_heading", "deg"},
    {"odom_x", "odometry alone: the pose minus every correction fed into it (= pose unless On)"},
    {"odom_y", ""},
    {"fused_x", "On: the pose; Shadow: pose + the correction it would have fed in; Off: the pose"},
    {"fused_y", ""},
    {"est_x", "the filter's estimate, pose + fed + pending (nan when fusion is Off)"},
    {"est_y", ""},
    {"sigma", "odometry doubt, in (nan when Off)"},
    {"pend_x", "correction decided but not fed in yet, in"},
    {"pend_y", ""},
    {"corr_x", "correction fed in this tick (On: into the pose; Shadow: kept apart), in"},
    {"corr_y", ""},
    {"gps_x", "what the GPS reports now: sensor position, in, field frame (nan with no sensor)"},
    {"gps_y", ""},
    {"gps_heading", "reported heading, deg (robot heading + mount yaw + noise)"},
    {"gps_new", "the sensor made a new output since the last tick"},
    {"gps_state", "0 sees the strips, 1 dead zone, 2 view lost, 3 no strips, 4 faces a wall with no strip (-1 no sensor)"},
    {"gps_outlier", "the current output is an injected outlier"},
    {"gps_err", "reported position minus where the camera is now, in"},
    {"gps_wall", "camera to the wall it faces now, in"},
    {"fate", "the new reading this tick: 0 none, 1-7 dropped by check N, 8 gate, 9 no history, 10 start check, "
             "11 accepted, 12 dropped by check 5b"},
    {"reloc", "relocalisations so far"},
    {"fault", "0 none, 1 StartDisagree, 2 RelocTooFar, 3 RelocTooMany, 4 TotalTooLarge, 5 HeadingDisagree"},
    {"cmdL", "left motor command, -127..127"},
    {"cmdR", "right motor command"},
    {"err_odom", "odometry alone minus the truth, in"},
    {"err_fused", "fused minus the truth, in"},
};

// With wall sensors (--wall-sensor), these follow the columns above; without, none of them is written
const Column WALL_COLUMNS[] = {
    {"sigma_x", "the filter's doubt in x, in (nan when Off)"},
    {"sigma_y", "and in y"},
    {"walls_waiting", "fault stage 1: the GPS is out and what was fed in is fed back out; the walls wait"},
    {"walls_stopped", "fault stage 2: a total-limit fault stopped the walls too"},
};
// ...then these for each wall sensor N (0, then 1), named wN_<name>
const Column WALL_SENSOR_COLUMNS[] = {
    {"mm", "what the sensor reports now, mm (9999 nothing seen, 2147483647 PROS_ERR = not fitted)"},
    {"conf", "its confidence, 0 to 63"},
    {"new", "the sensor made a new reading since the last tick"},
    {"mstate", "what the reading really is (the model's DistState): 0 wall, 1 clear wall, 2 corner, 3 element, "
               "4 object, 5 stop object, 6 through a clear wall, 7 no return, 8 too close, 9 not fitted"},
    {"clean", "what a clean reading would be: the centre ray from the true pose to the walls and elements, in"},
    {"read", "the reading as the filter calibrates it (told scale, offset), in; nan if wall check 1 refuses it"},
    {"pnow", "the range predicted from the filter's estimate now, in: the bench's own cast with the told mount and "
             "planes (nan when Off or no single wall)"},
    {"state", "the filter's state of the sensor: 0 unused, 1 waiting for its start check, 2 working, 3 start fault, "
              "4 switched off by the health rule (-1 when Off)"},
    {"fate", "this tick: 0 not sampled, 1-13 dropped by wall check N, 14 start check passed, 15 start check failed, "
             "16 fix, 17 switched off by the health rule after passing wall check 10"},
    {"fixes", "fixes so far"},
    {"axis", "with an innovation (fate 9 to 17): 0 its wall fixes x, 1 y; -1 otherwise"},
    {"pred", "with an innovation: p, the range the filter predicted, in"},
    {"meas", "with an innovation: m, the median of the steady readings, calibrated, in"},
    {"inn", "with an innovation: (p - m) times the beam's part toward the wall, in; + means toward +x or +y"},
    {"corr", "a fix this tick: the correction it decided on its axis, in; 0 otherwise"},
};

void put(std::string& s, double v) {
    char buf[40];
    if (std::isnan(v)) std::snprintf(buf, sizeof buf, ",nan");
    else std::snprintf(buf, sizeof buf, ",%.6g", v);
    s += buf;
}

void put(std::string& s, int v) { s += "," + std::to_string(v); }

bool writeCsv(const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return false;
    std::string line;
    for (const Column& c : COLUMNS) line += std::string(line.empty() ? "" : ",") + c.name;
    if (R.wallCount > 0) {
        for (const Column& c : WALL_COLUMNS) line += std::string(",") + c.name;
        for (int i = 0; i < R.wallCount; i++)
            for (const Column& c : WALL_SENSOR_COLUMNS) line += ",w" + std::to_string(i) + "_" + c.name;
    }
    std::fprintf(f, "%s\n", line.c_str());
    for (size_t n = 0; n < R.ticks.size(); n++) {
        const Tick& k = R.ticks[n];
        line = std::to_string(k.t);
        put(line, k.step);
        put(line, k.moving);
        put(line, k.trueX);
        put(line, k.trueY);
        put(line, k.trueDeg);
        put(line, k.poseX);
        put(line, k.poseY);
        put(line, k.poseDeg);
        put(line, k.odomX);
        put(line, k.odomY);
        put(line, k.fusedX);
        put(line, k.fusedY);
        put(line, k.estX);
        put(line, k.estY);
        put(line, k.sigma);
        put(line, k.pendX);
        put(line, k.pendY);
        put(line, k.corrX);
        put(line, k.corrY);
        put(line, k.gpsX);
        put(line, k.gpsY);
        put(line, k.gpsDeg);
        put(line, k.gpsNew);
        put(line, k.gpsState);
        put(line, k.gpsOutlier);
        put(line, k.gpsErrNow);
        put(line, k.gpsWall);
        put(line, k.fate);
        put(line, k.reloc);
        put(line, k.fault);
        put(line, k.cmdL);
        put(line, k.cmdR);
        put(line, k.errOdom);
        put(line, k.errFused);
        if (R.wallCount > 0) {
            const WallTick& wt = R.wallTicks[n];
            put(line, wt.sigmaX);
            put(line, wt.sigmaY);
            put(line, wt.waiting);
            put(line, wt.stopped);
            for (int i = 0; i < R.wallCount; i++) {
                const WallSensorTick& ws = wt.sensor[i];
                put(line, ws.mm);
                put(line, ws.conf);
                put(line, ws.isNew);
                put(line, ws.modelState);
                put(line, ws.cleanIn);
                put(line, ws.readIn);
                put(line, ws.predictedNowIn);
                put(line, ws.state);
                put(line, ws.fate);
                put(line, ws.fixes);
                put(line, ws.axis);
                put(line, ws.predictedIn);
                put(line, ws.measuredIn);
                put(line, ws.innovationIn);
                put(line, ws.correction);
            }
        }
        std::fprintf(f, "%s\n", line.c_str());
    }
    std::fclose(f);
    return true;
}

// A small JSON writer: keys in the order they are added
class Json {
    public:
        Json& num(const char* k, double v) {
            key(k);
            char buf[40];
            if (std::isfinite(v)) std::snprintf(buf, sizeof buf, "%.6g", v);
            else std::snprintf(buf, sizeof buf, "null");
            s += buf;
            return *this;
        }
        Json& integer(const char* k, long long v) {
            key(k);
            s += std::to_string(v);
            return *this;
        }
        Json& boolean(const char* k, bool v) {
            key(k);
            s += v ? "true" : "false";
            return *this;
        }
        Json& str(const char* k, const std::string& v) {
            key(k);
            s += '"';
            for (char c : v) {
                if (c == '"' || c == '\\') s += '\\';
                s += c;
            }
            s += '"';
            return *this;
        }
        Json& raw(const char* k, const std::string& json) {
            key(k);
            s += json;
            return *this;
        }
        std::string text() const { return "{" + s + "}"; }

    private:
        void key(const char* k) {
            if (!s.empty()) s += ',';
            s += '"';
            s += k;
            s += "\":";
        }
        std::string s;
};

struct ErrorStats {
        double end = NAN, max = 0, rms = 0;
};

ErrorStats errorStats(bool fused) {
    ErrorStats e;
    double sum = 0;
    for (const Tick& k : R.ticks) {
        const double v = fused ? k.errFused : k.errOdom;
        e.max = std::max(e.max, v);
        sum += v * v;
    }
    if (!R.ticks.empty()) {
        e.end = fused ? R.ticks.back().errFused : R.ticks.back().errOdom;
        e.rms = std::sqrt(sum / R.ticks.size());
    }
    return e;
}

std::string errorJson(const ErrorStats& e) { return Json().num("end", e.end).num("max", e.max).num("rms", e.rms).text(); }

double wrap180(double deg) { return std::remainder(deg, 360.0); }

// The PID tab's numbers for one move, from the pose the motions read: overshoot past the target, settle time
// (from the move's start until it stays within 1 in or 1 deg), and the error at the end of the window
std::string stepJson(const StepRecord& r, int windowEndMs) {
    const Step& s = *r.step;
    const char* kinds[] = {"point", "turn", "pose", "wait"};
    Json j;
    j.integer("n", r.n).str("kind", kinds[s.kind]).integer("line", s.line).str("text", s.text);
    j.integer("start_ms", r.startMs).integer("end_ms", r.endMs);
    if (s.kind == Step::WAIT) return j.text();
    j.boolean("timed_out", r.timedOut);
    const double p0x = r.poseAtStart.x, p0y = r.poseAtStart.y;
    const double d0 = std::hypot(s.x - p0x, s.y - p0y);
    const double ux = d0 > 1e-9 ? (s.x - p0x) / d0 : 0, uy = d0 > 1e-9 ? (s.y - p0y) / d0 : 0;
    const double e0 = wrap180(s.heading - r.poseAtStart.theta);
    const double turnDir = e0 < 0 ? -1 : 1;
    const double tol = s.kind == Step::TURN ? SETTLE_TOL_DEG : SETTLE_TOL_IN;
    double minErr = INFINITY, finalErr = NAN, trueFinal = NAN;
    int lastOutside = -1, lastT = -1;
    bool any = false;
    for (const Tick& k : R.ticks) {
        if (k.t < r.startMs || k.t > windowEndMs) continue;
        double err;
        if (s.kind == Step::POINT) err = d0 - ((k.poseX - p0x) * ux + (k.poseY - p0y) * uy);
        else if (s.kind == Step::TURN) err = wrap180(s.heading - k.poseDeg) * turnDir;
        else err = std::hypot(s.x - k.poseX, s.y - k.poseY);
        minErr = std::min(minErr, err);
        if (std::fabs(err) > tol) lastOutside = k.t;
        finalErr = err;
        lastT = k.t;
        trueFinal = s.kind == Step::TURN ? wrap180(s.heading - k.trueDeg) : std::hypot(s.x - k.trueX, s.y - k.trueY);
        any = true;
    }
    const double overshoot = (s.kind == Step::POSE || !any) ? 0 : std::max(0.0, -minErr);
    const int settle = !any ? -1 : (lastOutside == lastT ? -1 : (lastOutside < 0 ? 0 : lastOutside + 10 - r.startMs));
    j.num("overshoot", overshoot).integer("settle_ms", settle).num("final", finalErr).num("true_final", trueFinal);

    // The same move judged on the REAL robot, against where it finally comes to rest (the end of the window), so a
    // correction fed into the pose after the robot has stopped does not count: swing = how far it went past its
    // resting point (along the move's direction; turns: degrees past its resting heading), settle = until it stays
    // within 1 in (1 deg) of its resting point
    const Tick* rest = nullptr;
    for (const Tick& k : R.ticks)
        if (k.t >= r.startMs && k.t <= windowEndMs) rest = &k;
    double swing = 0;
    int lastAway = -1;
    if (rest != nullptr) {
        // pose moves end along their final heading; drives along the line from their start to the target
        const double ax = s.kind == Step::POSE ? std::sin(s.heading * DEG_TO_RAD) : ux;
        const double ay = s.kind == Step::POSE ? std::cos(s.heading * DEG_TO_RAD) : uy;
        for (const Tick& k : R.ticks) {
            if (k.t < r.startMs || k.t > windowEndMs) continue;
            double past, away;
            if (s.kind == Step::TURN) {
                past = wrap180(k.trueDeg - rest->trueDeg) * turnDir;
                away = std::fabs(wrap180(k.trueDeg - rest->trueDeg));
            } else {
                past = (k.trueX - rest->trueX) * ax + (k.trueY - rest->trueY) * ay;
                away = std::hypot(k.trueX - rest->trueX, k.trueY - rest->trueY);
            }
            swing = std::max(swing, past);
            if (away > tol) lastAway = k.t;
        }
    }
    j.num("swing_true", swing).integer("settle_true_ms", lastAway < 0 ? 0 : lastAway + 10 - r.startMs);
    return j.text();
}

#if DUT_HAS_GPSFUSION
using lemlib::gpsfusion::Config;
struct FloatKey {
        const char* name;
        float Config::*field;
};
struct IntKey {
        const char* name;
        int Config::*field;
};
const FloatKey FUSION_FLOAT_KEYS[] = {
    {"mountRightIn", &Config::mountRightIn},
    {"mountForwardIn", &Config::mountForwardIn},
    {"mountYawDeg", &Config::mountYawDeg},
    {"sigmaGpsIn", &Config::sigmaGpsIn},
    {"kDist", &Config::kDist},
    {"kTurn", &Config::kTurn},
    {"sigmaMaxIn", &Config::sigmaMaxIn},
    {"latencyMs", &Config::latencyMs},
    {"maxSpeedInPerS", &Config::maxSpeedInPerS},
    {"maxTurnRateDegPerS", &Config::maxTurnRateDegPerS},
    {"minPeriodMs", &Config::minPeriodMs},
    {"staleMs", &Config::staleMs},
    {"minViewIn", &Config::minViewIn},
    {"headingGateDeg", &Config::headingGateDeg},
    {"gate", &Config::gate},
    {"slewInPerS", &Config::slewInPerS},
    {"startTolIn", &Config::startTolIn},
    {"relocSpreadIn", &Config::relocSpreadIn},
    {"relocMaxIn", &Config::relocMaxIn},
    {"maxSensorErrorIn", &Config::maxSensorErrorIn},
    {"maxStepIn", &Config::maxStepIn},
    {"maxTotalCorrectionIn", &Config::maxTotalCorrectionIn},
    {"startSigmaIn", &Config::startSigmaIn},
    {"relocWindowMs", &Config::relocWindowMs},
};
const IntKey FUSION_INT_KEYS[] = {
    {"startSamples", &Config::startSamples},
    {"relocSamples", &Config::relocSamples},
    {"startHeadingSamples", &Config::startHeadingSamples},
    {"relocMaxCount", &Config::relocMaxCount},
};

bool setFusionKey(Config& c, const std::string& key, double value) {
    for (const FloatKey& k : FUSION_FLOAT_KEYS)
        if (key == k.name) {
            c.*k.field = float(value);
            return true;
        }
    for (const IntKey& k : FUSION_INT_KEYS)
        if (key == k.name) {
            c.*k.field = int(std::lround(value));
            return true;
        }
    return false;
}

std::string fusionConfigJson(const Config& c) {
    Json j;
    for (const FloatKey& k : FUSION_FLOAT_KEYS) j.num(k.name, c.*k.field);
    for (const IntKey& k : FUSION_INT_KEYS) j.integer(k.name, c.*k.field);
    return j.text();
}

// --wall KEY=VALUE: every lemlib::gpsfusion::WallConfig number by its C++ name; the arrays as planeIn0..planeIn3
// and trust0..trust3 (0 Ignore, 1 Clear, 2 Solid). Keep-outs come from --keep-out
using lemlib::gpsfusion::WallConfig;
struct WallFloatKey {
        const char* name;
        float WallConfig::*field;
};
struct WallIntKey {
        const char* name;
        int WallConfig::*field;
};
const WallFloatKey WALL_FLOAT_KEYS[] = {
    {"wallNoiseIn", &WallConfig::wallNoiseIn},
    {"wallNoiseFrac", &WallConfig::wallNoiseFrac},
    {"sigmaHeadingDeg", &WallConfig::sigmaHeadingDeg},
    {"wallMaxSpeedInPerS", &WallConfig::wallMaxSpeedInPerS},
    {"wallMaxTurnRateDegPerS", &WallConfig::wallMaxTurnRateDegPerS},
    {"stillWindowMs", &WallConfig::stillWindowMs},
    {"steadySpreadIn", &WallConfig::steadySpreadIn},
    {"steadySpreadFrac", &WallConfig::steadySpreadFrac},
    {"sampleMinPeriodMs", &WallConfig::sampleMinPeriodMs},
    {"maxIncidenceDeg", &WallConfig::maxIncidenceDeg},
    {"minRangeIn", &WallConfig::minRangeIn},
    {"maxRangeClearIn", &WallConfig::maxRangeClearIn},
    {"maxRangeSolidIn", &WallConfig::maxRangeSolidIn},
    {"coneHalfDeg", &WallConfig::coneHalfDeg},
    {"footprintMarginIn", &WallConfig::footprintMarginIn},
    {"cornerMarginIn", &WallConfig::cornerMarginIn},
    {"blockedMinIn", &WallConfig::blockedMinIn},
    {"blockedMaxIn", &WallConfig::blockedMaxIn},
    {"blockedSigmas", &WallConfig::blockedSigmas},
    {"throughMaxIn", &WallConfig::throughMaxIn},
    {"throughSigmas", &WallConfig::throughSigmas},
    {"capClearIn", &WallConfig::capClearIn},
    {"capSolidIn", &WallConfig::capSolidIn},
    {"refixTravelIn", &WallConfig::refixTravelIn},
    {"refixTurnDeg", &WallConfig::refixTurnDeg},
    {"sigmaFloorIn", &WallConfig::sigmaFloorIn},
    {"wallStartTolIn", &WallConfig::wallStartTolIn},
};
const WallIntKey WALL_INT_KEYS[] = {
    {"minConfidence", &WallConfig::minConfidence},
    {"minConfidenceClear", &WallConfig::minConfidenceClear},
    {"steadyCount", &WallConfig::steadyCount},
    {"healthMinTries", &WallConfig::healthMinTries},
};

bool setWallKey(WallConfig& c, const std::string& key, double value) {
    for (const WallFloatKey& k : WALL_FLOAT_KEYS)
        if (key == k.name) {
            c.*k.field = float(value);
            return true;
        }
    for (const WallIntKey& k : WALL_INT_KEYS)
        if (key == k.name) {
            c.*k.field = int(std::lround(value));
            return true;
        }
    for (int n = 0; n < lemlib::gpsfusion::NUM_WALLS; n++) {
        if (key == "planeIn" + std::to_string(n)) {
            c.planeIn[n] = float(value);
            return true;
        }
        if (key == "trust" + std::to_string(n)) {
            const long t = std::lround(value);
            if (t < 0 || t > 2) return false;
            c.trust[n] = lemlib::gpsfusion::WallTrust(t);
            return true;
        }
    }
    return false;
}

const char* trustName(lemlib::gpsfusion::WallTrust t) {
    switch (t) {
        case lemlib::gpsfusion::WallTrust::Ignore: return "ignore";
        case lemlib::gpsfusion::WallTrust::Clear: return "clear";
        default: return "solid";
    }
}

std::string wallConfigJson(const WallConfig& c) {
    Json j;
    std::string planes = "[", trust = "[", keepOuts = "[";
    for (int n = 0; n < lemlib::gpsfusion::NUM_WALLS; n++) {
        char buf[40];
        std::snprintf(buf, sizeof buf, "%s%.6g", n ? "," : "", double(c.planeIn[n]));
        planes += buf;
        trust += std::string(n ? "," : "") + "\"" + trustName(c.trust[n]) + "\"";
    }
    for (int n = 0; n < c.keepOutCount; n++) {
        char buf[80];
        std::snprintf(buf, sizeof buf, "%s[%d,%.6g,%.6g]", n ? "," : "", c.keepOuts[n].wall,
                      double(c.keepOuts[n].fromIn), double(c.keepOuts[n].toIn));
        keepOuts += buf;
    }
    j.raw("planeIn", planes + "]").raw("trust", trust + "]").raw("keep_outs", keepOuts + "]");
    for (const WallFloatKey& k : WALL_FLOAT_KEYS) j.num(k.name, c.*k.field);
    for (const WallIntKey& k : WALL_INT_KEYS) j.integer(k.name, c.*k.field);
    return j.text();
}
#endif

#if DUT_HAS_GPSFUSION
std::string intList(const std::vector<int>& v) {
    std::string out = "[";
    for (size_t i = 0; i < v.size(); i++) out += (i ? "," : "") + std::to_string(v[i]);
    return out + "]";
}

// The summary's "walls" object (only with wall sensors): what the filter was told, the simulated sensors' options,
// and per sensor its counters, how it was judged, and every fix with the truth next to it
std::string wallsJson() {
    using namespace lemlib::gpsfusion;
    Json j;
    j.integer("sensors", R.wallCount);
    // the distance model runs on world time (its objN_from_ms, stop_obj_from_ms...); the route's t_ms 0 is this
    j.integer("route_start_world_ms", R.t0);
    std::string mounts = "[";
    for (int i = 0; i < R.wallCount; i++) {
        const WallSensorMount& m = R.wallMounts[i];
        char buf[160];
        std::snprintf(buf, sizeof buf, "%s{\"port\":%d,\"right_in\":%.6g,\"forward_in\":%.6g,\"yaw_deg\":%.6g,"
                      "\"scale\":%.6g,\"offset_in\":%.6g}", i ? "," : "", WALL_SENSOR_PORTS[i], double(m.rightIn),
                      double(m.forwardIn), double(m.yawDeg), double(m.scale), double(m.offsetIn));
        mounts += buf;
    }
    j.raw("mounts_told", mounts + "]");
    j.raw("config", wallConfigJson(R.wallConfig));
    j.raw("model", distModel().p.toJson());
    if (R.fusionOn) {
        const Status& s = R.last;
        j.num("sigma_x_end", s.sigmaXIn).num("sigma_y_end", s.sigmaYIn);
        j.boolean("walls_waiting", s.wallsWaiting).boolean("walls_stopped", s.wallsStopped);
    }
    std::string list = "[";
    for (int i = 0; i < R.wallCount; i++) {
        const WallBook& book = R.wallBook[i];
        Json e;
        e.integer("port", WALL_SENSOR_PORTS[i]);
        if (R.fusionOn) {
            const WallSensorStatus& st = R.last.walls[i];
            std::string drops = "[";
            for (int c = 0; c < NUM_WALL_CHECKS; c++) drops += (c ? "," : "") + std::to_string(st.droppedByCheck[c]);
            e.str("state", wallSensorStateName(st.state))
                .integer("samples", st.samples)
                .integer("fixes", st.fixes)
                .raw("dropped_by_check", drops + "]")
                .integer("health_tries", st.healthTries)
                .integer("health_bad", st.healthBad)
                .num("start_innovation_in", st.startInnovationIn)
                .num("largest_correction_in", st.largestCorrectionIn);
        }
        e.integer("start_ms", book.startMs).boolean("start_fault", book.startFault);
        e.integer("health_off_ms", book.healthOffMs);
        e.integer("clean_refused_blocked", book.cleanBlocked).integer("clean_refused_through", book.cleanThrough);
        e.raw("stops_clean_blocked", intList(book.stopsBlocked)).raw("stops_clean_through", intList(book.stopsThrough));
        e.num("max_clean_refused_err_in", book.maxCleanRefusedErrIn);
        e.raw("sampled_by_model_state",
              intList(std::vector<int>(std::begin(book.sampledByModelState), std::end(book.sampledByModelState))));
        e.integer("fixes_not_clean", book.fixesNotClean);
        e.num("max_error_added_in", book.fixes.empty() ? NAN : book.maxErrorAddedIn);
        std::string fixes = "[";
        for (size_t n = 0; n < book.fixes.size(); n++) {
            const WallFixRecord& f = book.fixes[n];
            fixes += std::string(n ? "," : "") + Json()
                                                     .integer("t_ms", f.t)
                                                     .integer("step", f.step)
                                                     .integer("wall", f.wall)
                                                     .integer("axis", f.axis)
                                                     .num("corr", f.correction)
                                                     .num("inn", f.innovation)
                                                     .num("pred", f.predicted)
                                                     .num("meas", f.measured)
                                                     .num("err_before", f.errBefore)
                                                     .num("err_after", f.errAfter)
                                                     .num("implied_err", f.impliedErr)
                                                     .num("clean_in", f.cleanIn)
                                                     .integer("model_state", f.modelState)
                                                     .integer("states_mask", f.statesMask)
                                                     .integer("cone_mask_model", f.coneMaskModel)
                                                     .integer("cone_mask_18", f.coneMask18)
                                                     .num("true_x", f.trueX)
                                                     .num("true_y", f.trueY)
                                                     .num("true_deg", f.trueDeg)
                                                     .text();
        }
        e.raw("fix_list", fixes + "]");
        list += std::string(i ? "," : "") + e.text();
    }
    j.raw("per_sensor", list + "]");
    return j.text();
}
#endif

std::string readFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot read " + path);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// --compare-with: the motor commands of this run against a fusion-Off run with the same seed, tick by tick,
// while the two real robots are still in the same place (and on the same step). Up to that point any
// difference in command can only come from the corrections
std::string compareCommands(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot read " + path);
    std::string header;
    std::getline(in, header);
    std::vector<std::string> names;
    {
        std::stringstream hs(header);
        for (std::string n; std::getline(hs, n, ',');) names.push_back(n);
    }
    auto col = [&](const char* name) {
        for (size_t i = 0; i < names.size(); i++)
            if (names[i] == name) return int(i);
        throw std::runtime_error(path + ": no column " + name);
    };
    const int cT = col("t_ms"), cStep = col("step"), cX = col("true_x"), cY = col("true_y"),
              cH = col("true_heading"), cL = col("cmdL"), cR = col("cmdR");
    std::map<int, std::vector<double>> off;
    for (std::string line; std::getline(in, line);) {
        std::vector<double> v;
        std::stringstream ls(line);
        for (std::string f; std::getline(ls, f, ',');) v.push_back(std::strtod(f.c_str(), nullptr));
        if (int(v.size()) == int(names.size())) off[int(std::lround(v[cT]))] = v;
    }
    int compared = 0, comparedAfterCorrection = 0, splitMs = -1, maxMs = -1, firstCorrMs = -1;
    double maxDiff = 0, appliedSoFar = 0;
    for (const Tick& k : R.ticks) {
        appliedSoFar += std::hypot(k.corrX, k.corrY);
        if (appliedSoFar > 0 && firstCorrMs < 0 && R.fusionApplies) firstCorrMs = k.t;
        auto it = off.find(k.t);
        const bool together = it != off.end() && int(it->second[cStep]) == k.step &&
                              std::hypot(it->second[cX] - k.trueX, it->second[cY] - k.trueY) <= COINCIDE_IN &&
                              std::fabs(wrap180(it->second[cH] - k.trueDeg)) <= COINCIDE_DEG;
        if (!together) {
            splitMs = k.t;
            break;
        }
        compared++;
        if (firstCorrMs >= 0) comparedAfterCorrection++;
        const double d = std::max(std::fabs(it->second[cL] - k.cmdL), std::fabs(it->second[cR] - k.cmdR));
        if (d > maxDiff) {
            maxDiff = d;
            maxMs = k.t;
        }
    }
    return Json()
        .str("method", "same seed with fusion Off, motor commands compared tick by tick while both real robots are "
                       "within 0.1 in and 0.1 deg and on the same step")
        .str("off_csv", path)
        .integer("ticks_compared", compared)
        .integer("ticks_compared_after_first_correction", comparedAfterCorrection)
        .integer("first_correction_ms", firstCorrMs)
        .integer("split_ms", splitMs)
        .num("max_dcmd", maxDiff)
        .integer("max_dcmd_ms", maxMs)
        .text();
}

} // namespace

std::vector<double> jsonNumberArray(const std::string& text, const std::string& key) {
    std::vector<double> out;
    const std::string quoted = "\"" + key + "\"";
    const size_t at = text.find(quoted);
    if (at == std::string::npos) return out;
    size_t i = text.find('[', at + quoted.size());
    const size_t colon = text.find(':', at + quoted.size());
    if (i == std::string::npos || colon == std::string::npos || colon > i) return out;
    i++;
    while (i < text.size() && text[i] != ']') {
        if (std::isspace(static_cast<unsigned char>(text[i])) || text[i] == ',') {
            i++;
            continue;
        }
        char* end = nullptr;
        const double v = std::strtod(text.c_str() + i, &end);
        if (end == text.c_str() + i) throw std::runtime_error("bad number in \"" + key + "\"");
        out.push_back(v);
        i = size_t(end - text.c_str());
    }
    return out;
}

int runRoute(const RouteOptions& o, lemlib::Chassis& chassis, FILE* out) {
    World& w = world();
    R.o = &o;
    R.chassis = &chassis;
    Route route;
    GpsModel& gps = gpsModel();
    std::vector<LossSpec> losses = o.losses; // plus any "view_losses" in a --gps-json file
    try {
        route = loadRoute(o.routeFile);
        // the simulated GPS: JSON files first, then --gps, then the seed
        for (const std::string& file : o.gpsJsonFiles) {
            const std::string text = readFile(file);
            for (auto& [k, v] : JsonFlat::parse(text))
                if (!gps.p.set(k, v)) throw std::runtime_error(file + ": unknown GPS model key " + k);
            const std::vector<double> lossList = jsonNumberArray(text, "view_losses");
            for (size_t i = 0; i + 1 < lossList.size(); i += 2) {
                LossSpec l;
                l.routeMs = std::uint32_t(lossList[i]);
                l.lengthMs = std::uint32_t(lossList[i + 1]);
                losses.push_back(l);
            }
        }
        for (auto& [k, v] : o.gpsSets)
            if (!gps.p.set(k, v)) throw std::runtime_error("unknown GPS model key " + k + " (--gps)");
    } catch (const std::exception& e) {
        std::fprintf(stderr, "tb: %s\n", e.what());
        return 2;
    }
    gps.rng.reseed(o.seed * 2 + 2);

    const bool wantFusion = o.fusion != "off";
    if (o.fusion != "off" && o.fusion != "shadow" && o.fusion != "on") {
        std::fprintf(stderr, "tb: --fusion must be off, shadow or on\n");
        return 2;
    }
#if DUT_HAS_GPSFUSION
    Config config;
    try {
        for (const std::string& file : o.fusionJsonFiles)
            for (auto& [k, v] : JsonFlat::parse(readFile(file)))
                if (!setFusionKey(config, k, v)) throw std::runtime_error(file + ": unknown fusion key " + k);
        for (auto& [k, v] : o.fusionSets)
            if (!setFusionKey(config, k, v)) throw std::runtime_error("unknown fusion key " + k + " (--fz)");
    } catch (const std::exception& e) {
        std::fprintf(stderr, "tb: %s\n", e.what());
        return 2;
    }
#else
    if (wantFusion || !o.fusionSets.empty() || !o.fusionJsonFiles.empty()) {
        std::fprintf(stderr, "tb: this DUT has no GPS fusion (no chassis/gpsFusion.cpp): --fusion off only\n");
        return 2;
    }
#endif

    // Wall sensors (docs/WALL_FIX_DESIGN.md): what the filter is told (the WallConfig and each sensor's mount) and
    // the simulated sensors (the distance model: where the sensors truly are, the true walls, noise and so on).
    // Without --wall-sensor none of this happens and every output is what it was before
    const bool anyWallOption = !o.wallSensors.empty() || !o.wallSets.empty() || !o.wallJsonFiles.empty() ||
                               !o.wallTrusts.empty() || !o.keepOuts.empty() || !o.distSets.empty() ||
                               !o.distJsonFiles.empty();
#if DUT_HAS_GPSFUSION
    if (anyWallOption) {
        using namespace lemlib::gpsfusion;
        try {
            if (o.wallSensors.empty())
                throw std::runtime_error("--wall, --wall-json, --wall-trust, --keep-out, --dist and --dist-json need "
                                         "--wall-sensor");
            if (int(o.wallSensors.size()) > MAX_WALL_SENSORS)
                throw std::runtime_error("at most " + std::to_string(MAX_WALL_SENSORS) + " --wall-sensor");
            WallConfig& wc = R.wallConfig;
            for (const std::string& file : o.wallJsonFiles)
                for (auto& [k, v] : JsonFlat::parse(readFile(file)))
                    if (!setWallKey(wc, k, v)) throw std::runtime_error(file + ": unknown wall key " + k);
            for (auto& [k, v] : o.wallSets)
                if (!setWallKey(wc, k, v)) throw std::runtime_error("unknown wall key " + k + " (--wall)");
            for (auto& [wall, trust] : o.wallTrusts)
                for (int n = 0; n < NUM_WALLS; n++)
                    if (wall < 0 || wall == n) wc.trust[n] = WallTrust(trust);
            for (const KeepOutSpec& ko : o.keepOuts) {
                if (wc.keepOutCount >= MAX_KEEP_OUTS)
                    throw std::runtime_error("at most " + std::to_string(MAX_KEEP_OUTS) + " --keep-out");
                if (ko.wall < 0 || ko.wall >= NUM_WALLS) throw std::runtime_error("--keep-out: wall must be 0 to 3");
                wc.keepOuts[wc.keepOutCount++] = {ko.wall, float(ko.fromIn), float(ko.toIn)};
            }
            R.wallCount = int(o.wallSensors.size());
            for (int i = 0; i < R.wallCount; i++) {
                const WallSensorSpec& ws = o.wallSensors[size_t(i)];
                R.wallMounts[i] = {float(ws.rightIn), float(ws.forwardIn), float(ws.yawDeg), float(ws.scale),
                                   float(ws.offsetIn)};
            }
            // the simulated sensors: each wall sensor the filter has is fitted (sN_enabled 1), then JSON files,
            // then --dist (so --dist s1_enabled=0 can still unplug one), then its own seed
            DistModel& dm = distModel();
            for (int i = 0; i < R.wallCount; i++) dm.p.sensor[i].enabled = 1;
            for (const std::string& file : o.distJsonFiles)
                for (auto& [k, v] : JsonFlat::parse(readFile(file)))
                    if (!dm.p.set(k, v)) throw std::runtime_error(file + ": unknown distance model key " + k);
            for (auto& [k, v] : o.distSets)
                if (!dm.p.set(k, v))
                    throw std::runtime_error("unknown distance model key " + k + " (--dist; --dist help lists them)");
            dm.reseed(o.seed * 2 + 3);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "tb: %s\n", e.what());
            return 2;
        }
    }
#else
    if (anyWallOption) {
        std::fprintf(stderr, "tb: this DUT has no GPS fusion (no chassis/gpsFusion.cpp): no wall sensors\n");
        return 2;
    }
#endif

    // the robot starts where the route says; the GPS model watches it from boot
    w.placeAt(route.x, route.y, route.heading * DEG_TO_RAD);
    if (!o.noGps) {
        w.deviceRefresh = [](std::uint32_t) { gpsModel().refresh(world()); };
        pros::sim_detail::gpsReadHook = [](std::uint8_t, pros::sim_detail::GpsReading& r) {
            const GpsModel& m = gpsModel();
            if (!m.hasOutput()) return false;
            r = {m.outXMetres(), m.outYMetres(), m.outHeadingDeg(), m.outErrorMetres()};
            return true;
        };
    }
    if (R.wallCount > 0) {
        // the distance model watches the true robot from boot too, every ms, after the GPS model. The filter reads
        // it through the distance stub's hook: port 6 is sensor 0, port 21 sensor 1
        w.deviceRefresh = [](std::uint32_t now) {
            World& world_ = world();
            if (!R.o->noGps) gpsModel().refresh(world_);
            distModel().update(now, world_.x, world_.y, world_.heading);
        };
        pros::sim_detail::distanceReadHook = [](int port) -> pros::sim_detail::DistanceRead {
            for (int i = 0; i < DIST_MAX_SENSORS; i++)
                if (port == WALL_SENSOR_PORTS[i]) {
                    const DistReading r = distModel().read(i);
                    return {r.mm, r.confidence, r.objectSize};
                }
            errno = ENODEV; // not a wall sensor's port: unplugged
            return {PROS_ERR, PROS_ERR, PROS_ERR};
        };
    }
    if (!o.poseTrace.empty()) {
        R.poseTrace = std::fopen(o.poseTrace.c_str(), "w");
        if (!R.poseTrace) {
            std::fprintf(stderr, "tb: cannot write %s\n", o.poseTrace.c_str());
            return 2;
        }
    }
    w.preTick = nullptr;
    w.onDelay = nullptr;
    w.postEvents = [](std::uint32_t now) { logTick(now); };

    // boot as the other modes do: calibrate (starts odometry), then the routine's setPose and fusion on
    w.events(); // t = 0
    chassis.calibrate(true);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setPose(float(route.x + o.startErrXIn), float(route.y + o.startErrYIn),
                    float(route.heading + o.startErrDeg));
    R.t0 = w.now;
    gps.t0Ms = R.t0; // route time for the GPS model's bias_step_start_ms
    for (const LossSpec& l : losses) gps.losses.push_back({R.t0 + l.routeMs, l.lengthMs, l.drift, l.driftInS});
    for (const ShoveSpec& s : o.shoves) {
        if (s.atStep) continue;
        Shove sh;
        sh.startMs = R.t0 + s.routeMs;
        sh.lengthMs = s.lengthMs;
        sh.rightIn = s.rightIn;
        sh.forwardIn = s.forwardIn;
        w.shoves.push_back(sh);
    }
#if DUT_HAS_GPSFUSION
    static pros::Gps sensor(7); // the robot's GPS port (include/ports.hpp)
    if (wantFusion) {
        using namespace lemlib::gpsfusion;
        // the On guards (GPS_MOUNT_MEASURED, ODOMETRY_CALIBRATED) are facts about the real robot; the simulated
        // one is whatever the options say, so both are passed as true and --fusion decides On or Shadow
        configure(o.noGps ? nullptr : &sensor, config, true, true);
        if (R.wallCount > 0) {
            static pros::Distance wallSensor0(WALL_SENSOR_PORTS[0]), wallSensor1(WALL_SENSOR_PORTS[1]);
            pros::Distance* const wallSensors[MAX_WALL_SENSORS] = {&wallSensor0, &wallSensor1};
            configureWalls(wallSensors, R.wallMounts, R.wallCount, R.wallConfig);
        } else if (o.configureZeroWalls) {
            // wall test W1: configuring zero wall sensors must leave the GPS-only filter exactly as it was
            pros::Distance* const noSensors[MAX_WALL_SENSORS] = {nullptr, nullptr};
            configureWalls(noSensors, R.wallMounts, 0, R.wallConfig);
        }
        R.totalLimitIn = config.maxTotalCorrectionIn;
        setMode(o.fusion == "on" ? Mode::On : Mode::Shadow);
        R.fusionOn = true;
        R.fusionApplies = o.fusion == "on";
        R.prev = status();
        R.last = R.prev;
    }
#endif
    R.logging = true;
    if (o.startWaitMs > 0) w.advance(std::uint32_t(o.startWaitMs));

    // the route, one blocking move after another
    std::vector<StepRecord> records;
    for (size_t i = 0; i < route.steps.size(); i++) {
        const Step& s = route.steps[i];
        R.currentStep = int(i) + 1;
        for (const ShoveSpec& sh : o.shoves) {
            if (!sh.atStep || sh.step != R.currentStep) continue;
            Shove shove;
            shove.startMs = w.now;
            shove.lengthMs = sh.lengthMs;
            shove.rightIn = sh.rightIn;
            shove.forwardIn = sh.forwardIn;
            w.shoves.push_back(shove);
        }
        StepRecord rec;
        rec.n = R.currentStep;
        rec.step = &route.steps[i];
        rec.startMs = int(w.now - R.t0);
        rec.poseAtStart = chassis.getPose();
        switch (s.kind) {
            case Step::POINT: {
                lemlib::MoveToPointParams p;
                p.forwards = s.forwards;
                p.maxSpeed = float(s.maxSpeed);
                p.minSpeed = float(s.minSpeed);
                p.earlyExitRange = float(s.earlyExit);
                chassis.moveToPoint(float(s.x), float(s.y), s.timeoutMs, p, false);
                break;
            }
            case Step::TURN: {
                lemlib::TurnToHeadingParams p;
                p.maxSpeed = int(s.maxSpeed);
                if (s.direction > 0) p.direction = lemlib::AngularDirection::CW_CLOCKWISE;
                if (s.direction < 0) p.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE;
                chassis.turnToHeading(float(s.heading), s.timeoutMs, p, false);
                break;
            }
            case Step::POSE: {
                lemlib::MoveToPoseParams p;
                p.forwards = s.forwards;
                p.lead = float(s.lead);
                p.maxSpeed = float(s.maxSpeed);
                p.minSpeed = float(s.minSpeed);
                p.horizontalDrift = float(s.drift);
                chassis.moveToPose(float(s.x), float(s.y), float(s.heading), s.timeoutMs, p, false);
                break;
            }
            case Step::WAIT: w.advance(std::uint32_t(std::max(0, s.waitMs))); break;
        }
        rec.endMs = int(w.now - R.t0);
        rec.timedOut = s.kind != Step::WAIT && rec.endMs - rec.startMs >= s.timeoutMs;
        records.push_back(rec);
    }
    R.currentStep = int(route.steps.size()) + 1;
    R.logging = false;
    if (R.poseTrace) std::fclose(R.poseTrace);

    // ---- outputs
    if (!o.csv.empty() && !writeCsv(o.csv)) {
        std::fprintf(stderr, "tb: cannot write %s\n", o.csv.c_str());
        return 2;
    }
    // route statistics from the truth
    double travel = 0, turned = 0, maxAbsX = 0, maxAbsY = 0;
    for (size_t i = 0; i < R.ticks.size(); i++) {
        const Tick& k = R.ticks[i];
        maxAbsX = std::max(maxAbsX, std::fabs(k.trueX));
        maxAbsY = std::max(maxAbsY, std::fabs(k.trueY));
        if (i == 0) continue;
        travel += std::hypot(k.trueX - R.ticks[i - 1].trueX, k.trueY - R.ticks[i - 1].trueY);
        turned += std::fabs(k.trueDeg - R.ticks[i - 1].trueDeg);
    }
    int timeouts = 0, stopsInRange = 0;
    std::string stepsJson = "[";
    for (size_t i = 0; i < records.size(); i++) {
        const StepRecord& r = records[i];
        if (r.timedOut) timeouts++;
        if (r.step->kind == Step::WAIT && r.step->waitMs >= 300 && r.step->waitMs <= 500) stopsInRange++;
        // the move's window: until 300 ms after it ends, or the next move starts, whichever comes first
        int windowEnd = r.endMs + SETTLE_WINDOW_AFTER_MS;
        for (size_t j = i + 1; j < records.size(); j++)
            if (records[j].step->kind != Step::WAIT) {
                windowEnd = std::min(windowEnd, records[j].startMs);
                break;
            }
        stepsJson += (i ? "," : "") + stepJson(r, windowEnd);
    }
    stepsJson += "]";

    // shoves: how long until the fused pose is back within recoverTolIn of the truth (and stays there 1 s)
    std::string shovesJson = "[";
    for (size_t i = 0; i < w.shoves.size(); i++) {
        const Shove& sh = w.shoves[i];
        const int start = int(sh.startMs - R.t0);
        double before = NAN, after = NAN, at5s = NAN, odomAfter = NAN;
        int recover = -1;
        for (size_t a = 0; a < R.ticks.size(); a++) {
            const Tick& k = R.ticks[a];
            if (k.t < start) before = k.errFused;
            if (k.t >= start + sh.lengthMs && std::isnan(after)) {
                after = k.errFused;
                odomAfter = k.errOdom;
            }
            if (k.t >= start + 5000 && std::isnan(at5s)) at5s = k.errFused;
            if (k.t < start + sh.lengthMs || recover >= 0 || k.errFused > o.recoverTolIn) continue;
            bool holds = true;
            for (size_t b = a; b < R.ticks.size() && R.ticks[b].t < k.t + RECOVER_HOLD_MS; b++)
                if (R.ticks[b].errFused > o.recoverTolIn) holds = false;
            if (holds) recover = k.t - start;
        }
        shovesJson += std::string(i ? "," : "") + Json()
                                                       .integer("start_ms", start)
                                                       .integer("length_ms", sh.lengthMs)
                                                       .num("right_in", sh.rightIn)
                                                       .num("forward_in", sh.forwardIn)
                                                       .num("err_before", before)
                                                       .num("err_after", after)
                                                       .num("odom_err_after", odomAfter)
                                                       .integer("recover_ms", recover)
                                                       .num("err_at_5s", at5s)
                                                       .text();
    }
    shovesJson += "]";

    const ErrorStats odomErr = errorStats(false), fusedErr = errorStats(true);
    const Tick* last = R.ticks.empty() ? nullptr : &R.ticks.back();
    Json j;
    j.str("dut", DUT_NAME).str("route", o.routeFile).integer("seed", (long long)o.seed).str("fusion", o.fusion);
    j.str("sensor", o.noGps ? "none" : "model");
    j.integer("duration_ms", last ? last->t : 0).num("travel_in", travel).num("turn_deg", turned);
    j.num("max_abs_x", maxAbsX).num("max_abs_y", maxAbsY);
    j.integer("steps", int(records.size())).integer("timeouts", timeouts).integer("stops_300_500ms", stopsInRange);
    j.raw("odom", errorJson(odomErr));
    j.raw("fused", errorJson(fusedErr));
    if (last) {
        j.num("est_end_err", std::hypot(last->estX - last->trueX, last->estY - last->trueY));
        j.raw("true_end", "[" + std::to_string(last->trueX) + "," + std::to_string(last->trueY) + "," +
                              std::to_string(last->trueDeg) + "]");
        j.raw("pose_end", "[" + std::to_string(last->poseX) + "," + std::to_string(last->poseY) + "," +
                              std::to_string(last->poseDeg) + "]");
    }
    j.raw("world", Json()
                       .num("imu_scale_err_cw", w.p.IMU_SCALE_ERR_CW)
                       .num("imu_scale_err_ccw", w.p.IMU_SCALE_ERR_CCW)
                       .num("imu_drift_deg_s", w.p.IMU_DRIFT_DEG_S)
                       .num("vert_scale_err", w.p.VERT_SCALE_ERR)
                       .num("horiz_scale_err", w.p.HORIZ_SCALE_ERR)
                       .num("slip_per_in", w.p.SLIP_PER_IN)
                       .num("slip_max_in", w.p.SLIP_MAX_IN)
                       .integer("slips", w.slips)
                       .num("start_err_x", o.startErrXIn)
                       .num("start_err_y", o.startErrYIn)
                       .num("start_err_deg", o.startErrDeg)
                       .text());
    if (!o.noGps) {
        std::string lossJson = "[";
        for (size_t i = 0; i < gps.losses.size(); i++) {
            lossJson += std::string(i ? "," : "") + "[" + std::to_string(int(gps.losses[i].startMs - R.t0)) + "," +
                        std::to_string(gps.losses[i].lengthMs) + "," + std::to_string(gps.losses[i].drift);
            // a loss with its own drift rate says so as a fourth number (in/s)
            if (gps.losses[i].driftInS >= 0) {
                char rate[32];
                std::snprintf(rate, sizeof rate, ",%.6g", gps.losses[i].driftInS);
                lossJson += rate;
            }
            lossJson += "]";
        }
        lossJson += "]";
        j.raw("gps", Json()
                         .integer("outputs", gps.outputCount())
                         .integer("outliers", gps.outlierCount)
                         .integer("dead_zone_outputs", gps.deadZoneOutputs)
                         .integer("lost_outputs", gps.lostOutputs)
                         .integer("stripless_outputs", gps.striplessOutputs)
                         .num("rms_err_seeing",
                              gps.seeingOutputs ? std::sqrt(gps.sumSquaredErrorSeeing / gps.seeingOutputs) : NAN)
                         .raw("view_losses", lossJson)
                         .raw("model", gps.p.toJson())
                         .text());
    }
#if DUT_HAS_GPSFUSION
    if (R.fusionOn) {
        const lemlib::gpsfusion::Status& s = R.last;
        std::string drops = "[";
        for (int c = 0; c < lemlib::gpsfusion::NUM_CHECKS; c++)
            drops += (c ? "," : "") + std::to_string(s.droppedByCheck[c]);
        drops += "]";
        j.raw("filter",
              Json()
                  .integer("new_readings", s.newReadings)
                  .raw("dropped_by_check", drops)
                  .integer("dropped_by_gate", s.droppedByGate)
                  .integer("dropped_no_history", s.droppedNoHistory)
                  .integer("start_samples", s.startSamplesSeen)
                  .boolean("start_passed", s.startCheckPassed)
                  .integer("accepted", s.accepted)
                  .integer("relocalisations", s.relocalisations)
                  .integer("fault", int(s.fault))
                  .str("fault_text", s.faultText)
                  .integer("fault_ms", R.faultMs)
                  .integer("accepted_outliers", R.acceptedOutliers)
                  .num("max_accepted_outlier_in", R.maxAcceptedOutlierIn)
                  .integer("start_check_outliers", R.startCheckOutliers)
                  .integer("accepted_while_blind", R.acceptedBlind)
                  .num("max_accepted_reading_err_in", R.maxAcceptedReadingErrIn)
                  .num("rms_accepted_reading_err_in",
                       R.acceptedCount ? std::sqrt(R.sumSquaredAcceptedErr / R.acceptedCount) : NAN)
                  .num("sigma_end", s.sigmaIn)
                  .num("max_step_in", R.maxStepIn)
                  .integer("max_step_ms", R.maxStepMs)
                  .num("fed_x", R.fusionApplies ? s.appliedXIn : s.shadowXIn)
                  .num("fed_y", R.fusionApplies ? s.appliedYIn : s.shadowYIn)
                  // revision 2 (R1 to R4): what the Status now has, and what the bench tracks around the limits
                  .str("fault_name", s.faultName)
                  .boolean("unwinding", s.unwinding)
                  .integer("dropped_by_sensor_error", s.droppedBySensorError)
                  .num("max_reading_step_in", s.maxReadingStepIn)
                  .num("decided_x", s.decidedXIn)
                  .num("decided_y", s.decidedYIn)
                  .num("max_decided_in", R.maxDecidedIn)
                  .integer("decided_over_limit_ms", R.decidedOverLimitMs)
                  .num("max_fed_in", R.maxFedIn)
                  .num("max_fed_before_fault_in", R.maxFedBeforeFaultIn)
                  .num("fed_at_fault_in", R.fedAtFaultIn)
                  .num("fed_end_in", std::hypot(double(R.fusionApplies ? s.appliedXIn : s.shadowXIn),
                                                double(R.fusionApplies ? s.appliedYIn : s.shadowYIn)))
                  // -1: no fault, or still more than 0.5 in fed in when the route ended
                  .integer("fault_to_fed_under_half_ms",
                           R.faultMs < 0 || (last && R.lastFedOverHalfMs == last->t)
                               ? -1
                               : (R.lastFedOverHalfMs < 0 ? 0 : R.lastFedOverHalfMs + 10 - R.faultMs))
                  .integer("fault_to_unwound_ms", R.faultMs < 0 || R.unwoundMs < 0 ? -1 : R.unwoundMs - R.faultMs)
                  .num("max_accepted_blind_err_in", R.maxAcceptedBlindErrIn)
                  .boolean("start_heading_judged", s.headingJudged)
                  .integer("heading_samples_seen", s.headingSamplesSeen)
                  .integer("heading_samples_failed", s.headingSamplesFailed)
                  .integer("innovation_count", s.innovationCount)
                  .num("innovation_right_avg_in", s.innovationRightAvgIn)
                  .num("innovation_forward_avg_in", s.innovationForwardAvgIn)
                  .integer("nis_count", s.nisCount)
                  .num("nis_avg", s.nisAvg)
                  .raw("config", fusionConfigJson(config))
                  .text());
    }
    if (R.wallCount > 0) j.raw("walls", wallsJson());
#endif
    if (!o.compareWith.empty()) {
        try {
            j.raw("cmd_compare", compareCommands(o.compareWith));
        } catch (const std::exception& e) {
            std::fprintf(stderr, "tb: %s\n", e.what());
            return 2;
        }
    }
    j.raw("shoves", shovesJson);
    j.raw("step_list", stepsJson);

    if (o.summaryJson) {
        std::fprintf(out, "%s\n", j.text().c_str());
    } else {
        std::fprintf(out, "tb_%s  route %s  seed %llu  fusion %s  sensor %s\n", DUT_NAME, o.routeFile.c_str(),
                     (unsigned long long)o.seed, o.fusion.c_str(), o.noGps ? "none" : "model");
        std::fprintf(out, "  %d steps in %.1f s (%d timed out), %.0f in driven, %.0f deg turned, |x| <= %.1f, |y| <= %.1f\n",
                     int(records.size()), (last ? last->t : 0) / 1000.0, timeouts, travel, turned, maxAbsX, maxAbsY);
        std::fprintf(out, "  odometry alone: end %.2f in, max %.2f, rms %.2f\n", odomErr.end, odomErr.max, odomErr.rms);
        std::fprintf(out, "  fused:          end %.2f in, max %.2f, rms %.2f\n", fusedErr.end, fusedErr.max,
                     fusedErr.rms);
#if DUT_HAS_GPSFUSION
        if (R.fusionOn) {
            const lemlib::gpsfusion::Status& s = R.last;
            std::fprintf(out,
                         "  filter: %u new readings, dropped %u/%u/%u/%u/%u/%u/%u by checks 1-7, %u by the gate, %u "
                         "no history, %u accepted, %u relocalisations, fault: %s",
                         s.newReadings, s.droppedByCheck[0], s.droppedByCheck[1], s.droppedByCheck[2],
                         s.droppedByCheck[3], s.droppedByCheck[4], s.droppedByCheck[5], s.droppedByCheck[6],
                         s.droppedByGate, s.droppedNoHistory, s.accepted, s.relocalisations, s.faultText);
            if (R.faultMs >= 0) std::fprintf(out, " at %d ms", R.faultMs);
            std::fprintf(out, "\n  largest correction in one tick %.4f in; accepted outliers %d\n", R.maxStepIn,
                         R.acceptedOutliers);
            for (int i = 0; i < R.wallCount; i++) {
                const lemlib::gpsfusion::WallSensorStatus& st = s.walls[i];
                std::fprintf(out, "  wall sensor %d (port %d): %s, %u samples, %u fixes, dropped", i,
                             WALL_SENSOR_PORTS[i], lemlib::gpsfusion::wallSensorStateName(st.state), st.samples,
                             st.fixes);
                for (int c = 0; c < lemlib::gpsfusion::NUM_WALL_CHECKS; c++)
                    std::fprintf(out, "%s%u", c ? "/" : " ", st.droppedByCheck[c]);
                std::fprintf(out, " by wall checks 1-13, largest fix %.2f in\n", double(st.largestCorrectionIn));
            }
        }
#endif
    }
    std::fflush(out);
    return 0;
}

} // namespace sim
