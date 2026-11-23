// Unit tests for the wall fixes in the fusion filter (src/lemlib/chassis/gpsFusion.cpp, docs/WALL_FIX_DESIGN.md
// with its Revision A). Like gps_fusion_unit.cpp they test GpsFusionCore only, compiled with GPSFUSION_CORE_ONLY.
// "Wall check 1" to "wall check 13" are the design's checks; "stage 1" and "stage 2" are Revision A's two fault
// stages.
//
//   make -C tools/sim unit
//
// Prints one line per failed check and a summary; exits 1 if anything failed.
//
// Most tests use a pretend robot standing 20 in from the +x wall, facing it, with one distance sensor at its
// centre looking forward. The sensors here read 25 mm per inch (mount scale 25.4 / 25 = 1.016), so every reading
// in whole or 0.04 in steps is a whole number of mm and the hand-computed numbers come out exact.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string_view>
#include <vector>
#include "lemlib/chassis/gpsFusion.hpp"

using namespace lemlib::gpsfusion;

// ------------------------------------------------------------------------------------------------ checking
static int checksRun = 0;
static int checksFailed = 0;

static void check(bool ok, const char* what) {
    checksRun++;
    if (!ok) {
        checksFailed++;
        std::printf("  FAIL: %s\n", what);
    }
}

static void checkNear(double got, double want, double tolerance, const char* what) {
    checksRun++;
    if (!(std::fabs(got - want) <= tolerance)) {
        checksFailed++;
        std::printf("  FAIL: %s: got %.9g, want %.9g (+-%g)\n", what, got, want, tolerance);
    }
}

static void checkCount(std::uint32_t got, std::uint32_t want, const char* what) {
    checksRun++;
    if (got != want) {
        checksFailed++;
        std::printf("  FAIL: %s: got %u, want %u\n", what, unsigned(got), unsigned(want));
    }
}

static const double PI = 3.14159265358979323846;
static double rad(double deg) { return deg * PI / 180; }

// every whole inch a whole number of mm (see the top of the file)
static const float MM25_SCALE = 25.4f / 25.0f;

static const char* const CHECK_NAMES[NUM_WALL_CHECKS] = {"1 valid",    "2 confident", "3 still",       "4 steady",
                                                         "5 one wall", "6 square",    "7 in range",    "8 plain wall",
                                                         "9 blocked",  "10 through",  "11 gate",       "12 cap",
                                                         "13 once per stop"};

// ------------------------------------------------------------------------------------------------ the truth
// Where a beam really meets the walls, in double precision, written apart from the filter's castToWall so that
// the tests do not check the filter against itself. Returns INFINITY when it meets none
static double trueRange(const double plane[4], double x, double y, double headingDeg) {
    const double dx = std::sin(rad(headingDeg));
    const double dy = std::cos(rad(headingDeg));
    double best = INFINITY;
    if (dx > 1e-12) best = std::fmin(best, (plane[1] - x) / dx);
    if (dx < -1e-12) best = std::fmin(best, (-plane[3] - x) / dx);
    if (dy > 1e-12) best = std::fmin(best, (plane[0] - y) / dy);
    if (dy < -1e-12) best = std::fmin(best, (-plane[2] - y) / dy);
    return best;
}

// ------------------------------------------------------------------------------------------------ a pretend robot
struct TruePose {
        std::uint32_t t;
        double x, y, thetaDeg;
};

// One simulated distance sensor: it reads the true range from where it really is, plus what a test adds
struct SimSensor {
        bool fixed = false; // true: always reads exactly fixedMm (an error value, 9999, ...)
        int fixedMm = 0;
        int addMm = 0; // added to the true range: negative = something in front of the wall, positive = through it
        int confidence = 63; // what get_confidence() returns
        bool unplugged = false; // both calls return PROS_ERR
        std::vector<int> script; // if not empty: addMm for each reading the filter samples, in turn (the last repeats)
        std::vector<int> confidenceScript; // the same for the confidence (moves on with `script`'s count)
        size_t scriptAt = 0;
};

struct WallBot {
        GpsFusionCore core;
        Config config;
        WallConfig walls;
        WallSensorMount mounts[MAX_WALL_SENSORS];
        int sensorCount = 0;
        Mode mode = Mode::Off;
        std::uint32_t t = 1000;

        // where the robot really is, and where the walls really are
        double trueX = 0, trueY = 0, trueThetaDeg = 0;
        double truePlane[4] = {70.25, 70.25, 70.25, 70.25};
        std::vector<TruePose> truthLog;
        // what odometry says (moves exactly like the truth: an odometry error is a different start)
        float odomX = 0, odomY = 0, odomThetaRad = 0;

        SimSensor sim[MAX_WALL_SENSORS];
        bool feedWalls = true; // false: the tick passes no wall readings at all (nullptr)
        DistanceSample lastSamples[MAX_WALL_SENSORS];

        // a GPS (off by default: the filter gets nullptr). A perfect one, 60 ms late, a new output every 40 ms,
        // camera at the robot's centre facing backward
        bool gpsPlugged = false;
        bool gpsInfinity = false; // reads infinity: what PROS returns for an unplugged GPS
        double gpsDx = 0, gpsDy = 0; // added to what it reports
        double gpsHeadingOffDeg = 0; // added to its heading (20: every reading fails check 5)
        bool shotPending = false; // the next output alone: heading right and (shotDx, shotDy) added
        double shotDx = 0, shotDy = 0;
        int gpsUpdates = 0;
        GpsSample gps;

        double appliedX = 0, appliedY = 0; // the corrections the filter returned, added up

        WallBot(const Config& c, const WallConfig& w, std::vector<WallSensorMount> m, double x, double y,
                double thetaDeg)
            : core(c),
              config(c),
              walls(w),
              sensorCount(int(m.size())),
              trueX(x),
              trueY(y),
              trueThetaDeg(thetaDeg),
              odomX(float(x)),
              odomY(float(y)),
              odomThetaRad(float(rad(thetaDeg))) {
            for (int i = 0; i < sensorCount && i < MAX_WALL_SENSORS; i++) mounts[i] = m[i];
            core.setWallConfig(w, mounts, sensorCount);
            truthLog.push_back({t, trueX, trueY, trueThetaDeg});
            updateGps();
        }

        void start(Mode m) {
            mode = m;
            core.setMode(m);
        }

        TruePose truthAt(std::uint32_t when) const {
            TruePose found = truthLog.front();
            for (const TruePose& p : truthLog)
                if (p.t <= when) found = p;
            return found;
        }

        void updateGps() {
            const TruePose seen = truthAt(t >= 60 ? t - 60 : 0);
            const double wiggle = (gpsUpdates % 2 == 0) ? 0.01 : -0.01;
            double x = seen.x + gpsDx, y = seen.y + gpsDy, heading = seen.thetaDeg + 180 + wiggle + gpsHeadingOffDeg;
            if (shotPending) {
                x = seen.x + shotDx;
                y = seen.y + shotDy;
                heading = seen.thetaDeg + 180 + wiggle;
                shotPending = false;
            }
            heading = std::fmod(heading, 360.0);
            if (heading < 0) heading += 360;
            gps.xMetres = x / INCHES_PER_METRE;
            gps.yMetres = y / INCHES_PER_METRE;
            gps.headingDeg = heading;
            gps.errorMetres = 0.02;
            if (gpsInfinity) gps.xMetres = gps.yMetres = gps.headingDeg = gps.errorMetres = INFINITY;
            gpsUpdates++;
        }

        // what sensor i reads right now
        DistanceSample read(int i) const {
            DistanceSample out;
            const SimSensor& s = sim[i];
            if (s.unplugged) return out; // DistanceSample's defaults: PROS_ERR for both
            const WallSensorMount& m = mounts[i];
            const double th = rad(trueThetaDeg);
            const double sx = trueX + m.rightIn * std::cos(th) + m.forwardIn * std::sin(th);
            const double sy = trueY - m.rightIn * std::sin(th) + m.forwardIn * std::cos(th);
            const double range = trueRange(truePlane, sx, sy, trueThetaDeg + m.yawDeg);
            const int add = s.script.empty() ? s.addMm : s.script[std::min(s.scriptAt, s.script.size() - 1)];
            out.mm = std::isfinite(range) ? int(std::lround((range - m.offsetIn) / m.scale * 25.4)) + add : 9999;
            if (s.fixed) out.mm = s.fixedMm;
            out.confidence = s.confidenceScript.empty()
                                 ? s.confidence
                                 : s.confidenceScript[std::min(s.scriptAt, s.confidenceScript.size() - 1)];
            return out;
        }

        // one odometry tick: the robot drives at inPerS and turns at degPerS for dtMs
        Correction tick(int dtMs = 10, double inPerS = 0, double degPerS = 0) {
            t += dtMs;
            const double dtS = dtMs / 1000.0;
            const double mid = rad(trueThetaDeg + degPerS * dtS / 2);
            trueX += inPerS * dtS * std::sin(mid);
            trueY += inPerS * dtS * std::cos(mid);
            trueThetaDeg += degPerS * dtS;
            const float odomMid = float(odomThetaRad + rad(degPerS * dtS / 2));
            odomX += float(inPerS * dtS * std::sin(odomMid));
            odomY += float(inPerS * dtS * std::cos(odomMid));
            odomThetaRad += float(rad(degPerS * dtS));
            truthLog.push_back({t, trueX, trueY, trueThetaDeg});
            if (t % 40 == 0) updateGps();
            for (int i = 0; i < sensorCount; i++) lastSamples[i] = read(i);
            std::uint32_t samplesBefore[MAX_WALL_SENSORS] = {};
            const Status before = core.getStatus();
            for (int i = 0; i < sensorCount; i++) samplesBefore[i] = before.walls[i].samples;
            const Correction c = core.tick(t, odomX, odomY, odomThetaRad, gpsPlugged ? &gps : nullptr,
                                           feedWalls ? lastSamples : nullptr, sensorCount);
            odomX += c.xIn;
            odomY += c.yIn;
            appliedX += c.xIn;
            appliedY += c.yIn;
            // a script moves on each time the filter takes a reading
            const Status after = core.getStatus();
            for (int i = 0; i < sensorCount; i++)
                if (after.walls[i].samples != samplesBefore[i]) sim[i].scriptAt++;
            return c;
        }

        void run(int ms, double inPerS = 0, double degPerS = 0) {
            for (int done = 0; done < ms; done += 10) tick(10, inPerS, degPerS);
        }

        Status status() const { return core.getStatus(); }
        const WallSensorStatus& wall(int i, Status& keep) const {
            keep = status();
            return keep.walls[i];
        }
        std::uint32_t fixes(int i = 0) const { return status().walls[i].fixes; }
        WallSensorState state(int i = 0) const { return status().walls[i].state; }
        std::uint32_t dropped(int c, int i = 0) const { return status().walls[i].droppedByCheck[c]; }
};

// Tick until sensor i has given one more fix; false if not within maxMs. Stops on the tick of the fix, so the
// status then shows that reading's numbers
static bool runUntilFix(WallBot& b, int i = 0, int maxMs = 3000) {
    const std::uint32_t before = b.fixes(i);
    for (int waited = 0; waited < maxMs; waited += 10) {
        b.tick();
        if (b.fixes(i) != before) return true;
    }
    return false;
}

// Tick until sensor i's start check has happened (it is no longer waiting); false if not within maxMs
static bool runUntilStarted(WallBot& b, int i = 0, int maxMs = 3000) {
    for (int waited = 0; waited < maxMs; waited += 10) {
        if (b.state(i) != WallSensorState::WaitingForStart) return true;
        b.tick();
    }
    return b.state(i) != WallSensorState::WaitingForStart;
}

// The numbers the tests share: revision 2's filter numbers, but no doubt growth (so the doubts only change
// where a test says), and the GPS's heading fault not judged (so a test can feed it readings that fail check 5)
static Config quietConfig() {
    Config c;
    c.kDist = 0;
    c.kTurn = 0;
    c.startHeadingSamples = 0;
    return c;
}

// A sensor at the robot's centre looking straight ahead, 25 mm per inch
static WallSensorMount centreForward() {
    WallSensorMount m;
    m.scale = MM25_SCALE;
    return m;
}

// The usual scene: the robot faces the +x wall (heading 90) with its sensor 20 in from it, centred along the
// wall. dTrueX moves the real robot (odometry stays): + = really closer to the wall
static WallBot facingPlusX(const WallConfig& w = WallConfig(), const Config& c = quietConfig(), double dTrueX = 0,
                           double estX = 50.25, double estY = 0) {
    WallBot b(c, w, {centreForward()}, estX, estY, 90);
    b.trueX = estX + dTrueX;
    b.truthLog.clear();
    b.truthLog.push_back({b.t, b.trueX, b.trueY, b.trueThetaDeg});
    return b;
}

// Every wall check counter that is not zero, except the ones allowed, makes a failed check
static void checkOnlyDrops(const WallSensorStatus& w, std::initializer_list<int> allowed, const char* what) {
    for (int c = 0; c < NUM_WALL_CHECKS; c++) {
        bool ok = w.droppedByCheck[c] == 0;
        for (int a : allowed) ok = ok || a == c;
        if (!ok) {
            char text[200];
            std::snprintf(text, sizeof text, "%s: nothing dropped by wall check %s (got %u)", what, CHECK_NAMES[c],
                          unsigned(w.droppedByCheck[c]));
            check(false, text);
            return;
        }
    }
    check(true, what);
}

// ------------------------------------------------------------------------------------------------ geometry

static void testCastToWall() {
    std::printf("castToWall: the nearest of the four planes, corners belong to neither\n");
    const float planes[4] = {70.25f, 70.25f, 70.25f, 70.25f};
    struct Case {
            float x, y, dx, dy;
            int wall;
            float range, along;
            const char* what;
    };
    const Case cases[] = {
        {0, 0, 0, 1, WALL_PLUS_Y, 70.25f, 0, "from the centre along +y: wall 0 at 70.25"},
        {0, 0, 1, 0, WALL_PLUS_X, 70.25f, 0, "along +x: wall 1"},
        {0, 0, 0, -1, WALL_MINUS_Y, 70.25f, 0, "along -y: wall 2"},
        {0, 0, -1, 0, WALL_MINUS_X, 70.25f, 0, "along -x: wall 3"},
        {10, 20, 0.6f, 0.8f, WALL_PLUS_Y, 62.8125f, 47.6875f, "(10, 20) along (0.6, 0.8): wall 0 at 50.25/0.8"},
        {10, -20, 0.8f, -0.6f, WALL_PLUS_X, 75.3125f, -65.1875f, "(10, -20) along (0.8, -0.6): wall 1 at 60.25/0.8"},
        {-30, 5, -0.6f, -0.8f, WALL_MINUS_X, 67.0833333f, -48.6666667f, "(-30, 5) along (-0.6, -0.8): wall 3 at 40.25/0.6"},
        {5, -30, -0.6f, -0.8f, WALL_MINUS_Y, 50.3125f, -25.1875f, "(5, -30) along (-0.6, -0.8): wall 2 at 40.25/0.8"},
    };
    for (const Case& k : cases) {
        const WallHit h = castToWall(planes, k.x, k.y, k.dx, k.dy);
        check(h.wall == k.wall, k.what);
        checkNear(h.rangeIn, k.range, 1e-4, "  range");
        checkNear(h.alongIn, k.along, 1e-4, "  where along the wall");
    }
    // each plane is its own: walls at 70, 71, 72, 73
    const float uneven[4] = {70, 71, 72, 73};
    checkNear(castToWall(uneven, 1, 2, 0, 1).rangeIn, 68, 0, "wall 0 at y = +70: 68 from y = 2");
    checkNear(castToWall(uneven, 1, 2, 1, 0).rangeIn, 70, 0, "wall 1 at x = +71: 70 from x = 1");
    checkNear(castToWall(uneven, 1, 2, 0, -1).rangeIn, 74, 0, "wall 2 at y = -72: 74 from y = 2");
    checkNear(castToWall(uneven, 1, 2, -1, 0).rangeIn, 74, 0, "wall 3 at x = -73: 74 from x = 1");
    // a ray into a corner: both walls at exactly the same distance. It belongs to neither (the cautious answer)
    check(castToWall(planes, 0, 0, 0.5f, 0.5f).wall == NO_WALL, "exactly into the +x/+y corner: no wall");
    check(castToWall(planes, 0, 0, -0.5f, -0.5f).wall == NO_WALL, "exactly into the -x/-y corner: no wall");
    check(castToWall(planes, 0, 0.001f, 0.5f, 0.5f).wall == WALL_PLUS_Y, "a hair off the corner: the nearer wall");
    // outside or on a wall, a zero direction, a NaN
    check(castToWall(planes, 71, 0, -1, 0).wall == NO_WALL, "starting outside the field (+x): no wall");
    check(castToWall(planes, -71, 0, 1, 0).wall == NO_WALL, "starting outside the field (-x): no wall");
    check(castToWall(planes, 0, 71, 0, -1).wall == NO_WALL, "starting outside the field (+y): no wall");
    check(castToWall(planes, 0, -71, 0, 1).wall == NO_WALL, "starting outside the field (-y): no wall");
    check(castToWall(planes, 70.25f, 0, -1, 0).wall == NO_WALL, "starting on a wall: no wall");
    check(castToWall(planes, 0, 0, 0, 0).wall == NO_WALL, "a zero direction: no wall");
    check(castToWall(planes, NAN, 0, 1, 0).wall == NO_WALL, "a NaN position: no wall");
}

// The four walls with the sensor mounted 3 in right, 2 in forward, at each of the four yaws: the robot turns so
// the beam meets the wall square, and stands so the sensor is 20 in from it, centred along it. The real robot is
// 1 in closer to the wall (then 1 in farther) than the estimate. Hand-computed for every case: p = 20, m = 19
// (21), innovation +-1 toward the wall's side, R = (0.3 + 0.01 m)^2, K = 1 / (1 + R), correction K * innovation
// on that axis only, that axis's doubt sqrt(1 - K), the other doubt 1
static void testGeometryEveryWallAndYaw() {
    std::printf("geometry: every wall, every mount yaw, the innovation's sign both ways\n");
    const double beamHeading[4] = {0, 90, 180, 270}; // toward wall 0, 1, 2, 3
    const double sensorAt[4][2] = {{0, 50.25}, {50.25, 0}, {0, -50.25}, {-50.25, 0}};
    const int axisOf[4] = {1, 0, 1, 0};
    const double sideOf[4] = {+1, +1, -1, -1}; // +: the wall is on the + side of its axis
    int cases = 0, good = 0;
    for (int wall = 0; wall < 4; wall++) {
        for (double yaw : {0.0, 90.0, 180.0, 270.0}) {
            for (double closer : {+1.0, -1.0}) {
                WallSensorMount m;
                m.rightIn = 3;
                m.forwardIn = 2;
                m.yawDeg = float(yaw);
                m.scale = MM25_SCALE;
                const double t = std::fmod(beamHeading[wall] - yaw + 720, 360);
                // the robot's centre = the sensor's position minus the mount turned by the heading
                const double offX = 3 * std::cos(rad(t)) + 2 * std::sin(rad(t));
                const double offY = -3 * std::sin(rad(t)) + 2 * std::cos(rad(t));
                const double estX = sensorAt[wall][0] - offX;
                const double estY = sensorAt[wall][1] - offY;
                WallBot b(quietConfig(), WallConfig(), {m}, estX, estY, t);
                // the real robot is `closer` inches nearer the wall along its axis
                (axisOf[wall] == 0 ? b.trueX : b.trueY) += closer * sideOf[wall];
                b.truthLog.clear();
                b.truthLog.push_back({b.t, b.trueX, b.trueY, b.trueThetaDeg});
                b.start(Mode::On);
                const bool fixed = runUntilFix(b);
                const Status s = b.status();
                const WallSensorStatus& w = s.walls[0];
                const double mWant = 20 - closer;
                const double R = std::pow(0.3 + 0.01 * mWant, 2);
                const double K = 1 / (1 + R);
                const double innovationWant = closer * sideOf[wall];
                const double decidedAxis = axisOf[wall] == 0 ? s.decidedXIn : s.decidedYIn;
                const double decidedOther = axisOf[wall] == 0 ? s.decidedYIn : s.decidedXIn;
                const double sigmaAxis = axisOf[wall] == 0 ? s.sigmaXIn : s.sigmaYIn;
                const double sigmaOther = axisOf[wall] == 0 ? s.sigmaYIn : s.sigmaXIn;
                const bool ok = fixed && w.lastWall == wall && w.lastAxis == axisOf[wall] &&
                                std::fabs(w.lastPredictedIn - 20) < 2e-4 && std::fabs(w.lastMeasuredIn - mWant) < 2e-4 &&
                                std::fabs(w.lastInnovationIn - innovationWant) < 3e-4 &&
                                std::fabs(w.lastNoiseIn - std::sqrt(R)) < 1e-4 &&
                                std::fabs(decidedAxis - K * innovationWant) < 3e-4 && decidedOther == 0 &&
                                std::fabs(sigmaAxis - std::sqrt(1 - K)) < 1e-4 && sigmaOther == 1 &&
                                std::fabs(w.lastCorrectionIn - K * innovationWant) < 3e-4;
                cases++;
                good += ok;
                if (!ok) {
                    char what[300];
                    std::snprintf(what, sizeof what,
                                  "wall %d, yaw %.0f, heading %.0f, real robot %+.0f in toward the wall: fixed %d wall %d "
                                  "axis %d p %.4f m %.4f inn %.4f noise %.4f decided %.4f/%.4f sigma %.4f/%.4f",
                                  wall, yaw, t, closer, fixed, w.lastWall, w.lastAxis, w.lastPredictedIn,
                                  w.lastMeasuredIn, w.lastInnovationIn, w.lastNoiseIn, decidedAxis, decidedOther,
                                  sigmaAxis, sigmaOther);
                    check(false, what);
                }
            }
        }
    }
    char what[120];
    std::snprintf(what, sizeof what, "%d of %d cases: wall, axis, p, m, innovation, R, correction, doubts as hand-computed",
                  good, cases);
    check(good == cases && cases == 32, what);
}

static void testNoiseFormula() {
    std::printf("noise: R = (sigmaWall |d.along|)^2 + (m |d.across| sigmaHeading)^2\n");
    // the beam 5 degrees off square (heading 85), the sensor exactly 20 in from the wall along the beam:
    // sigmaWall = 0.3 + 0.01 * 20 = 0.5; R = (0.5 cos 5)^2 + (20 sin 5 * 1 deg in rad)^2 = 0.24902653
    {
        const double x = 70.25 - 20 * std::cos(rad(5));
        WallBot b(quietConfig(), WallConfig(), {centreForward()}, x, 0, 85);
        b.start(Mode::On);
        check(runUntilFix(b), "(an angled beam fixes)");
        Status keep;
        const WallSensorStatus& w = b.wall(0, keep);
        checkNear(w.lastMeasuredIn, 20, 1e-4, "  m = 20 (500 mm)");
        checkNear(w.lastPredictedIn, 20, 1e-3, "  p = 20");
        checkNear(w.lastNoiseIn, std::sqrt(0.24902653374216274), 1e-5, "  sqrt(R) = 0.4990256");
    }
    // square at 10 in: sigmaWall = 0.4, R = 0.16; the heading term is zero
    {
        WallBot b = facingPlusX(WallConfig(), quietConfig(), 0, 60.25);
        b.start(Mode::On);
        check(runUntilFix(b), "(a square beam at 10 in fixes)");
        Status keep;
        checkNear(b.wall(0, keep).lastNoiseIn, 0.4, 1e-5, "square at 10 in: sqrt(R) = 0.3 + 0.01 * 10 = 0.4");
    }
    // sigmaHeadingDeg 3 and wallNoiseFrac 0: R = (0.3 cos 5)^2 + (20 sin 5 * 3 deg)^2 = 0.0893151 + 0.0083627
    {
        WallConfig w;
        w.sigmaHeadingDeg = 3;
        w.wallNoiseFrac = 0;
        const double x = 70.25 - 20 * std::cos(rad(5));
        WallBot b(quietConfig(), w, {centreForward()}, x, 0, 85);
        b.start(Mode::On);
        runUntilFix(b);
        Status keep;
        const double want = std::pow(0.3 * std::cos(rad(5)), 2) + std::pow(20 * std::sin(rad(5)) * rad(3), 2);
        checkNear(b.wall(0, keep).lastNoiseIn, std::sqrt(want), 1e-5, "sigmaHeadingDeg 3, wallNoiseFrac 0: R as written");
    }
    // the reading's calibration: m = scale * mm / 25.4 + offset. Scale 1, offset 0.5: the sim reads 495.3 mm
    // -> 495, and m = 495 / 25.4 + 0.5
    {
        WallSensorMount m;
        m.scale = 1;
        m.offsetIn = 0.5f;
        WallBot b(quietConfig(), WallConfig(), {m}, 50.25, 0, 90);
        b.start(Mode::On);
        runUntilFix(b);
        Status keep;
        checkNear(b.wall(0, keep).lastMeasuredIn, 495 / 25.4 + 0.5, 1e-4, "m = scale * mm / 25.4 + offset (1, 0.5 in)");
    }
}

// ------------------------------------------------------------------------------------------------ the checks, one at a time

static void testCheck1Valid() {
    std::printf("wall check 1: valid readings only\n");
    // (the health rule would switch the sensor off after 6 of these at a clean view: tested on its own below)
    WallConfig noHealth;
    noHealth.healthMinTries = 1000000;
    const int bad[] = {DISTANCE_READ_FAILED, 0, 9999, 19, 2001, -5};
    for (int mm : bad) {
        WallBot b = facingPlusX(noHealth);
        b.sim[0].fixed = true;
        b.sim[0].fixedMm = mm;
        b.start(Mode::On);
        b.run(1000);
        Status keep;
        const WallSensorStatus& w = b.wall(0, keep);
        char what[120];
        std::snprintf(what, sizeof what, "%d mm: every reading dropped by wall check 1 (%u of %u)", mm,
                      unsigned(w.droppedByCheck[WCHECK_VALID]), unsigned(w.samples));
        check(w.samples > 20 && w.droppedByCheck[WCHECK_VALID] == w.samples && w.fixes == 0, what);
        checkOnlyDrops(w, {WCHECK_VALID}, "  and by nothing else");
    }
    for (int confidence : {-1, 64, DISTANCE_READ_FAILED}) {
        WallBot b = facingPlusX(noHealth);
        b.sim[0].confidence = confidence;
        b.start(Mode::On);
        b.run(1000);
        char what[120];
        std::snprintf(what, sizeof what, "confidence %d (not 0 to 63) is an error value: wall check 1", confidence);
        check(b.dropped(WCHECK_VALID) == b.status().walls[0].samples && b.fixes() == 0, what);
    }
    {
        WallBot b = facingPlusX(noHealth);
        b.sim[0].unplugged = true;
        b.start(Mode::On);
        b.run(1000);
        check(b.dropped(WCHECK_VALID) > 20 && b.fixes() == 0, "unplugged (PROS_ERR for both): wall check 1");
    }
    // 20 and 2000 mm are valid: 20 mm is 0.8 in, under minRangeIn; 2000 mm is 80 in, over the range: wall check 7
    for (int mm : {20, 2000}) {
        WallBot b = facingPlusX();
        b.sim[0].fixed = true;
        b.sim[0].fixedMm = mm;
        b.start(Mode::On);
        b.run(1000);
        char what[120];
        std::snprintf(what, sizeof what, "%d mm is valid: not wall check 1 (it fails wall check 7)", mm);
        check(b.dropped(WCHECK_VALID) == 0 && b.dropped(WCHECK_IN_RANGE) > 10, what);
    }
}

static void testCheck2Confident() {
    std::printf("wall check 2: confident above 200 mm (minConfidenceClear on Clear, minConfidence on Solid)\n");
    struct Case {
            WallTrust trust;
            int confidence;
            bool dropped;
            const char* what;
    };
    const Case cases[] = {{WallTrust::Clear, 49, true, "Clear wall, confidence 49: dropped"},
                          {WallTrust::Clear, 50, false, "Clear wall, confidence 50: used"},
                          {WallTrust::Solid, 39, true, "Solid wall, confidence 39: dropped"},
                          {WallTrust::Solid, 40, false, "Solid wall, confidence 40: used"}};
    for (const Case& k : cases) {
        WallConfig w;
        w.trust[WALL_PLUS_X] = k.trust;
        w.healthMinTries = 1000000; // (low confidence at a clean view is a bad try: tested on its own below)
        WallBot b = facingPlusX(w);
        b.sim[0].confidence = k.confidence;
        b.start(Mode::On);
        b.run(1500);
        Status keep;
        const WallSensorStatus& ws = b.wall(0, keep);
        if (k.dropped) {
            check(ws.droppedByCheck[WCHECK_CONFIDENT] > 20 && ws.fixes == 0, k.what);
            checkOnlyDrops(ws, {WCHECK_CONFIDENT}, "  and by nothing else");
        } else {
            check(ws.droppedByCheck[WCHECK_CONFIDENT] == 0 && ws.fixes == 1, k.what);
        }
    }
    // at or below 200 mm the confidence is not looked at: 8 in = 200 mm with confidence 0 is used; 201 mm is not
    {
        WallBot b = facingPlusX(WallConfig(), quietConfig(), 0, 62.25); // the sensor 8 in from the wall
        b.sim[0].confidence = 0;
        b.start(Mode::On);
        b.run(1500);
        check(b.dropped(WCHECK_CONFIDENT) == 0 && b.fixes() == 1, "200 mm, confidence 0: used (not measured there)");
        WallConfig w;
        w.healthMinTries = 1000000;
        WallBot c = facingPlusX(w, quietConfig(), 0, 62.25);
        c.sim[0].confidence = 0;
        c.sim[0].addMm = 1; // 201 mm
        c.start(Mode::On);
        c.run(1500);
        check(c.dropped(WCHECK_CONFIDENT) > 20 && c.fixes() == 0, "201 mm, confidence 0: dropped");
    }
    // the number comes from the trust of the wall the beam is predicted to meet. With no usable wall (one marked
    // Ignore) the stricter of the two is used, so confidence 45 there is counted under wall check 2 (50), not
    // passed on to wall check 5 (it would pass 40)
    {
        WallConfig w;
        w.trust[WALL_PLUS_X] = WallTrust::Ignore;
        WallBot b = facingPlusX(w);
        b.sim[0].confidence = 45;
        b.start(Mode::On);
        b.run(1500);
        check(b.dropped(WCHECK_CONFIDENT) > 25 && b.dropped(WCHECK_ONE_WALL) == 0,
              "an Ignore wall, confidence 45: wall check 2 (the stricter number, 50, is used)");
    }
}

static void testCheck3Still() {
    std::printf("wall check 3: still over the last 100 ms (corrections taken out)\n");
    // moving along the wall at 2 in/s (the range does not change, so wall check 4 would pass)
    {
        WallBot b = facingPlusX(WallConfig(), quietConfig(), 0, 50.25, -20);
        b.start(Mode::On);
        b.run(1500, 0, 0);
        const std::uint32_t before = b.dropped(WCHECK_STILL);
        // heading 90: driving "forward" goes toward the wall. Drive sideways instead: turn the truth and
        // odometry by moving them by hand along y
        for (int i = 0; i < 100; i++) {
            b.trueY += 0.02;
            b.odomY += 0.02f;
            b.tick();
        }
        Status keep;
        const WallSensorStatus& w = b.wall(0, keep);
        check(w.droppedByCheck[WCHECK_STILL] - before >= 20, "sliding along the wall at 2 in/s: dropped by wall check 3");
        check(w.fixes == 1, "  (only the fix from before it moved)");
    }
    // 1.2 in/s: dropped; 0.8 in/s: used
    for (double speed : {1.2, 0.8}) {
        WallBot b = facingPlusX(WallConfig(), quietConfig(), 0, 50.25, -20);
        b.start(Mode::On);
        for (int i = 0; i < 150; i++) {
            b.trueY += speed / 100;
            b.odomY += float(speed / 100);
            b.tick();
        }
        char what[100];
        std::snprintf(what, sizeof what, "sliding at %.1f in/s (wallMaxSpeed 1): %s", speed,
                      speed > 1 ? "never still" : "still");
        if (speed > 1) check(b.fixes() == 0 && b.dropped(WCHECK_STILL) > 30, what);
        else check(b.fixes() == 1, what);
    }
    // turning at 4 deg/s: dropped (3 deg/s is the limit). Square enough for a while either way
    {
        WallBot b = facingPlusX();
        b.trueThetaDeg = 88;
        b.odomThetaRad = float(rad(88));
        b.start(Mode::On);
        b.run(1000, 0, 4);
        check(b.fixes() == 0 && b.dropped(WCHECK_STILL) > 15, "turning at 4 deg/s: dropped by wall check 3");
        WallBot c = facingPlusX();
        c.trueThetaDeg = 88;
        c.odomThetaRad = float(rad(88));
        c.start(Mode::On);
        c.run(1000, 0, 2);
        check(c.fixes() == 1, "turning at 2 deg/s: still");
    }
    // the history must reach back 100 ms: the first readings after switching on are dropped by wall check 3
    {
        WallBot b = facingPlusX();
        b.start(Mode::On);
        b.run(100);
        Status keep;
        const WallSensorStatus& w = b.wall(0, keep);
        check(w.samples == 3 && w.droppedByCheck[WCHECK_STILL] == 3, "the first 100 ms: 3 readings, all wall check 3");
    }
    // a correction being fed in is not the robot moving: sensor 1 keeps working while sensor 0's fix (about
    // 1.2 in, fed in at 2 in/s for 0.6 s) moves the pose
    {
        WallConfig w;
        w.trust[WALL_PLUS_X] = WallTrust::Solid;
        WallSensorMount back = centreForward();
        back.yawDeg = 180; // looks at the -x wall, which never gets a fix here (too far)
        Config c = quietConfig();
        c.startSigmaIn = 2;
        WallBot b(c, w, {centreForward(), back}, 50.25, 0, 90);
        b.trueX = 51.75; // 1.5 in closer: a fix of about 1.4 in, fed in at 2 in/s
        b.truthLog.clear();
        b.truthLog.push_back({b.t, b.trueX, b.trueY, b.trueThetaDeg});
        b.start(Mode::On);
        check(runUntilFix(b, 0), "(sensor 0 fixed)");
        const std::uint32_t stillDropsBefore = b.dropped(WCHECK_STILL, 0);
        b.run(600);
        check(std::fabs(b.appliedX) > 1.0, "(the fix was being fed in over the next 0.6 s)");
        checkCount(b.dropped(WCHECK_STILL, 0) - stillDropsBefore, 0,
                   "while it is fed in at 2 in/s the robot is still still: no wall check 3 drops");
    }
}

static void testCheck4Steady() {
    std::printf("wall check 4: the last steadyCount readings agree; the median is the reading used\n");
    // alternating +-0.3 in (15 mm with 25 mm per inch... 7.5: use 8 mm = 0.32 in): a span of 0.64 in > 0.5
    {
        WallBot b = facingPlusX();
        b.sim[0].script = {8, -8, 8, -8, 8, -8, 8, -8, 8, -8, 8, -8, 8, -8, 8, -8, 8, -8, 8, -8, 8, -8, 8, -8, 8, -8, 8,
                           -8, 8, -8, 8, -8, 8, -8, 8, -8, 8, -8, 8, -8};
        b.start(Mode::On);
        b.run(1500);
        Status keep;
        const WallSensorStatus& w = b.wall(0, keep);
        check(w.fixes == 0 && w.droppedByCheck[WCHECK_STEADY] > 25, "span 0.64 in at 20 in (limit 0.5): dropped");
        checkOnlyDrops(w, {WCHECK_STILL, WCHECK_STEADY}, "  and by nothing after it");
    }
    // a span of 0.48 in (12 mm): used
    {
        WallBot b = facingPlusX();
        std::vector<int> s;
        for (int i = 0; i < 40; i++) s.push_back(i % 2 ? 6 : -6);
        b.sim[0].script = s;
        b.start(Mode::On);
        b.run(1500);
        check(b.fixes() == 1, "span 0.48 in at 20 in: used");
    }
    // 2 percent of the median when that is larger: at 30 in (a Solid wall) the limit is 0.6 in, so a span of 0.56 in
    // (14 mm) is used there, and the same span at 20 in is not
    {
        WallConfig w;
        w.trust[WALL_PLUS_X] = WallTrust::Solid;
        WallBot far = facingPlusX(w, quietConfig(), 0, 40.25);
        WallBot near = facingPlusX(w, quietConfig(), 0, 50.25);
        std::vector<int> s;
        for (int i = 0; i < 40; i++) s.push_back(i % 2 ? 7 : -7);
        far.sim[0].script = s;
        near.sim[0].script = s;
        far.start(Mode::On);
        near.start(Mode::On);
        far.run(1500);
        near.run(1500);
        check(far.fixes() == 1, "span 0.56 in at 30 in (limit 2 % = 0.6): used");
        check(near.fixes() == 0, "span 0.56 in at 20 in (limit 0.5): dropped");
    }
    // the median, not the mean: readings 1 in short, 1 in short, 1 in short, 0.6 in short, 0.6 in short (in the
    // order the filter samples them). Median: 1 in short, m = 19; the mean would be 19.16
    {
        WallBot b = facingPlusX();
        b.sim[0].script = {0, 0, 0, -25, -25, -15, -25, -15, -25, -15, -25, -15};
        b.start(Mode::On);
        runUntilStarted(b); // the start check uses the first 5 (all 0)
        check(runUntilFix(b), "(fixed)");
        Status keep;
        const WallSensorStatus& w = b.wall(0, keep);
        // the fix's readings: -25, -15, -25, -15, -25 in a row with the earlier 0s pushed out? The five newest at the
        // fix: 0 is pushed out once five new ones are in, so they are -25 -25 -15 -25 -15 (span 0.4 in)
        checkNear(w.lastMeasuredIn, 19, 1e-4, "m is the median of the five (19.0), not their mean (19.16 or so)");
    }
    // median, smallest and mean all different: the five newest at the fix are -20, -24, -20, -12, -20 mm in some
    // order (0.48 in span): median -20 mm (m 19.2), smallest -24 (19.04), mean -19.2 (19.232)
    {
        WallBot b = facingPlusX();
        b.sim[0].script = {0, 0, 0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 20; i++)
            for (int v : {-20, -24, -20, -12, -20}) b.sim[0].script.push_back(v);
        b.start(Mode::On);
        check(runUntilFix(b), "(fixed)");
        Status keep;
        checkNear(b.wall(0, keep).lastMeasuredIn, 19.2, 1e-4, "m is the median 19.2 (not the smallest 19.04 or the mean 19.23)");
    }
    // steadyCount 4 (even): the median is the mean of the middle two. Readings 4 to 7 are still and exact (the
    // start check comes on reading 7); the four newest at the fix are -24, -16, -20, -12 mm in some order (span
    // 0.48 in): 19.04, 19.36, 19.2, 19.52 in, so the median is (19.2 + 19.36) / 2 = 19.28
    {
        WallConfig w;
        w.steadyCount = 4;
        WallBot b = facingPlusX(w);
        b.sim[0].script = {0, 0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 20; i++)
            for (int v : {-24, -16, -20, -12}) b.sim[0].script.push_back(v);
        b.start(Mode::On);
        check(runUntilFix(b), "(fixed)");
        Status keep;
        checkNear(b.wall(0, keep).lastMeasuredIn, 19.28, 1e-4,
                  "steadyCount 4: m is the mean of the middle two (19.28, not 19.2 or 19.36)");
    }
    // a single reading with too little confidence (wall check 2) starts the row again too: the same, with the
    // 8th reading at confidence 30 (Clear wall: 50 needed)
    {
        WallBot b = facingPlusX();
        b.sim[0].confidenceScript = {63, 63, 63, 63, 63, 63, 63, 30, 63};
        b.start(Mode::On);
        runUntilStarted(b);
        Status keep;
        const WallSensorStatus& w = b.wall(0, keep);
        checkCount(w.samples, 13, "a low-confidence reading in the row: the start check on reading 13");
        checkCount(w.droppedByCheck[WCHECK_CONFIDENT], 1, "  (that one reading dropped by wall check 2)");
    }
    // a single invalid reading in the row starts it again, even with no gap: readings 4 to 7 of the run are
    // steady, the 8th is invalid, so the start check needs 5 more (it comes on reading 13, not 9)
    {
        WallBot b = facingPlusX();
        b.sim[0].script = {0, 0, 0, 0, 0, 0, 0, 100000, 0};
        b.start(Mode::On);
        runUntilStarted(b);
        Status keep;
        const WallSensorStatus& w = b.wall(0, keep);
        checkCount(w.samples, 13, "an invalid reading in the row: the start check on reading 13");
        checkCount(w.droppedByCheck[WCHECK_STEADY], 8, "  8 readings dropped by wall check 4 (4 before it, 4 after)");
    }
    // a reading that is not still starts the row again, even with no gap (stillWindowMs 30, a 0.2 in sideways bump
    // the range does not see, 20 ms before a reading)
    {
        WallConfig w;
        w.stillWindowMs = 30;
        WallBot b = facingPlusX(w);
        b.start(Mode::On);
        b.run(160); // readings at +10 (not still), 50, 90, 130 (3 in the row so far)
        check(b.dropped(WCHECK_STEADY) == 3 && b.dropped(WCHECK_STILL) == 1, "(3 steady readings in the row)");
        b.trueY += 0.2;
        b.odomY += 0.2f;
        b.run(10); // +170: a reading, 10 ms after the bump: not still
        check(b.dropped(WCHECK_STILL) == 2, "(the reading after the bump is not still)");
        runUntilStarted(b);
        checkCount(b.dropped(WCHECK_STEADY), 7, "a reading that was not still: the row starts again (4 more drops)");
    }
    // a gap in the readings (no readings for 150 ms) starts the row again: 5 more readings are needed
    {
        WallBot b = facingPlusX();
        b.start(Mode::On);
        b.run(250); // samples at +10, 50, 90 (not still), 130, 170, 210, 250: 4 steady readings so far
        const std::uint32_t steadyBefore = b.dropped(WCHECK_STEADY);
        b.feedWalls = false;
        b.run(150);
        b.feedWalls = true;
        runUntilStarted(b);
        checkCount(b.dropped(WCHECK_STEADY) - steadyBefore, 4,
                   "after a 150 ms gap the row starts again: the next 4 readings are dropped by wall check 4");
    }
    // steadyCount 1: the first still reading is enough
    {
        WallConfig w;
        w.steadyCount = 1;
        WallBot b = facingPlusX(w);
        b.start(Mode::On);
        b.run(170); // samples at +10, 50, 90 (not still), 130 (start check), 170 (fix)
        check(b.state() == WallSensorState::Working && b.fixes() == 1 && b.dropped(WCHECK_STEADY) == 0,
              "steadyCount 1: start check on the first still reading, a fix on the next");
    }
}

static void testCheck5To8() {
    std::printf("wall checks 5 to 8: one wall, square, in range, plain wall (keep-outs, corners)\n");
    // 5: the wall marked Ignore
    {
        WallConfig w;
        w.trust[WALL_PLUS_X] = WallTrust::Ignore;
        WallBot b = facingPlusX(w);
        b.start(Mode::On);
        b.run(1500);
        Status keep;
        const WallSensorStatus& ws = b.wall(0, keep);
        check(ws.fixes == 0 && ws.droppedByCheck[WCHECK_ONE_WALL] > 25, "wall marked Ignore: wall check 5");
        checkOnlyDrops(ws, {WCHECK_STILL, WCHECK_STEADY, WCHECK_ONE_WALL}, "  and nothing after it");
    }
    // 6: 11 degrees off square: dropped; 9 degrees: used
    for (double off : {11.0, -11.0, 9.0, -9.0}) {
        WallBot b = facingPlusX();
        b.trueThetaDeg = 90 + off;
        b.odomThetaRad = float(rad(90 + off));
        b.start(Mode::On);
        b.run(1500);
        char what[100];
        std::snprintf(what, sizeof what, "beam %+.0f deg off square (limit 10): %s", off,
                      std::fabs(off) > 10 ? "wall check 6" : "used");
        if (std::fabs(off) > 10) {
            Status keep;
            const WallSensorStatus& ws = b.wall(0, keep);
            check(ws.fixes == 0 && ws.droppedByCheck[WCHECK_SQUARE] > 25, what);
            checkOnlyDrops(ws, {WCHECK_STILL, WCHECK_STEADY, WCHECK_SQUARE}, "  and nothing after it");
        } else {
            check(b.fixes() == 1, what);
        }
    }
    // 7: 25 in from a Clear wall (24 at most): dropped; from a Solid wall (36): used; 1.4 in (1.5 at least): dropped
    {
        WallBot clear = facingPlusX(WallConfig(), quietConfig(), 0, 45.25);
        clear.start(Mode::On);
        clear.run(1500);
        Status keep;
        const WallSensorStatus& ws = clear.wall(0, keep);
        check(ws.fixes == 0 && ws.droppedByCheck[WCHECK_IN_RANGE] > 25, "25 in from a Clear wall: wall check 7");
        checkOnlyDrops(ws, {WCHECK_STILL, WCHECK_STEADY, WCHECK_IN_RANGE}, "  and nothing after it");
        WallConfig w;
        w.trust[WALL_PLUS_X] = WallTrust::Solid;
        WallBot solid = facingPlusX(w, quietConfig(), 0, 45.25);
        solid.start(Mode::On);
        solid.run(1500);
        check(solid.fixes() == 1, "25 in from a Solid wall: used");
        WallBot close = facingPlusX(WallConfig(), quietConfig(), 0, 68.85); // 1.4 in = 35 mm
        close.sim[0].confidence = 10;
        close.start(Mode::On);
        close.run(1500);
        check(close.fixes() == 0 && close.dropped(WCHECK_IN_RANGE) > 25, "1.4 in from the wall: wall check 7");
        WallBot justIn = facingPlusX(WallConfig(), quietConfig(), 0, 68.65); // 1.6 in = 40 mm
        justIn.sim[0].confidence = 10;
        justIn.start(Mode::On);
        justIn.run(1500);
        check(justIn.fixes() == 1, "1.6 in from the wall: used");
    }
    // 7 on p alone: p = 1.4 in (too close) while the reading says 1.6 in
    {
        WallBot b = facingPlusX(WallConfig(), quietConfig(), 0, 68.85);
        b.sim[0].confidence = 10;
        b.sim[0].addMm = 5; // +0.2 in
        b.start(Mode::On);
        b.run(1500);
        check(b.fixes() == 0 && b.dropped(WCHECK_IN_RANGE) > 25, "p 1.4 in (under 1.5), m 1.6 in: wall check 7");
    }
    // 7 on m alone: p = 23.8 in (in range) but the reading says 24.2 in (out of range): wall check 7, before 10
    {
        WallBot b = facingPlusX(WallConfig(), quietConfig(), 0, 46.45);
        b.sim[0].addMm = 10; // +0.4 in
        b.start(Mode::On);
        b.run(1500);
        check(b.fixes() == 0 && b.dropped(WCHECK_IN_RANGE) > 25 && b.dropped(WCHECK_NOT_THROUGH) == 0,
              "p 23.8 in range, m 24.2 out of range: wall check 7");
    }
    // 8, the cone across a corner: at y = 65 the +18 degree edge ray meets the +y wall first
    {
        WallBot b = facingPlusX(WallConfig(), quietConfig(), 0, 50.25, 65);
        b.start(Mode::On);
        b.run(1500);
        Status keep;
        const WallSensorStatus& ws = b.wall(0, keep);
        check(ws.fixes == 0 && ws.droppedByCheck[WCHECK_PLAIN_WALL] > 25, "cone across the corner: wall check 8");
        checkOnlyDrops(ws, {WCHECK_STILL, WCHECK_STEADY, WCHECK_PLAIN_WALL}, "  and nothing after it");
    }
    // 8, "rays at phi plus and minus coneHalfDeg hit the same wall", on its own (corner margin switched off):
    // at y = +65 the edge ray at heading 72 meets the +y wall first, at y = -65 the one at 108 meets the -y wall
    for (double y : {65.0, -65.0, 40.0}) {
        WallConfig w;
        w.cornerMarginIn = -1000;
        WallBot b = facingPlusX(w, quietConfig(), 0, 50.25, y);
        b.start(Mode::On);
        b.run(1500);
        char what[120];
        std::snprintf(what, sizeof what, "corner margin off, centre at y = %+.0f: %s", y,
                      std::fabs(y) > 60 ? "an edge ray meets another wall: wall check 8" : "used");
        check(std::fabs(y) > 60 ? (b.fixes() == 0 && b.dropped(WCHECK_PLAIN_WALL) > 25) : b.fixes() == 1, what);
    }
    // 8, the corners of the +x wall are where the y walls are: with the -y wall at 60.25 the footprint's low end
    // (y - 8.498) must stay above -60.25 + 6 = -54.25
    for (double y : {-46.5, -44.0}) {
        WallConfig w;
        w.planeIn[WALL_MINUS_Y] = 60.25f;
        WallBot b = facingPlusX(w, quietConfig(), 0, 50.25, y);
        b.truePlane[WALL_MINUS_Y] = 60.25;
        b.start(Mode::On);
        b.run(1500);
        char what[120];
        std::snprintf(what, sizeof what, "-y wall at 60.25, centre at y = %.1f (low end %.2f): %s", y, y - 8.498,
                      y < -45.75 ? "within 6 in of the corner: wall check 8" : "used");
        check(y < -45.75 ? (b.fixes() == 0 && b.dropped(WCHECK_PLAIN_WALL) > 25) : b.fixes() == 1, what);
    }
    // 8, corner margin: the footprint at 20 in reaches 20 tan 18 = 6.498 in either side, widened by 2 = 8.498, and
    // must stay more than 6 in from the corner at y = 70.25: the beam's centre below y = 55.752
    for (double y : {55.65, 55.85, -55.65, -55.85}) {
        WallBot b = facingPlusX(WallConfig(), quietConfig(), 0, 50.25, y);
        b.start(Mode::On);
        b.run(1500);
        const bool tooClose = std::fabs(y) > 55.752;
        char what[100];
        std::snprintf(what, sizeof what, "centre at y = %+.2f (corner margin edge at 55.75): %s", y,
                      tooClose ? "wall check 8" : "used");
        check(tooClose ? (b.fixes() == 0 && b.dropped(WCHECK_PLAIN_WALL) > 25) : b.fixes() == 1, what);
    }
    // 8, keep-outs. The widened footprint at y = 0 spans -8.498 .. 8.498
    struct KeepCase {
            int wall;
            float from, to;
            bool blocks;
            const char* what;
    };
    const KeepCase keeps[] = {
        {WALL_PLUS_X, -1, 1, true, "keep-out -1..1 on this wall: wall check 8"},
        {WALL_PLUS_X, 8.4f, 20, true, "keep-out 8.4..20 (overlaps the widened edge at 8.498): wall check 8"},
        {WALL_PLUS_X, 20, 8.4f, true, "the same keep-out written 20..8.4: wall check 8"},
        {WALL_PLUS_X, 8.6f, 20, false, "keep-out 8.6..20 (clear of 8.498): used"},
        {WALL_PLUS_X, -20, -8.6f, false, "keep-out -20..-8.6: used"},
        {WALL_PLUS_X, -20, -8.4f, true, "keep-out -20..-8.4: wall check 8"},
        {WALL_PLUS_Y, -1, 1, false, "keep-out on another wall: used"},
    };
    for (const KeepCase& k : keeps) {
        WallConfig w;
        w.keepOuts[0] = {k.wall, k.from, k.to};
        w.keepOutCount = 1;
        WallBot b = facingPlusX(w);
        b.start(Mode::On);
        b.run(1500);
        check(k.blocks ? (b.fixes() == 0 && b.dropped(WCHECK_PLAIN_WALL) > 25) : b.fixes() == 1, k.what);
    }
    // the eighth keep-out is used; a ninth (keepOutCount 9) is left out
    {
        WallConfig w;
        for (int i = 0; i < MAX_KEEP_OUTS; i++) w.keepOuts[i] = {WALL_MINUS_Y, float(10 * i), float(10 * i + 1)};
        w.keepOuts[MAX_KEEP_OUTS - 1] = {WALL_PLUS_X, -1, 1};
        w.keepOutCount = MAX_KEEP_OUTS;
        WallBot b = facingPlusX(w);
        b.start(Mode::On);
        b.run(1500);
        check(b.fixes() == 0 && b.dropped(WCHECK_PLAIN_WALL) > 25, "the 8th keep-out counts");
        w.keepOutCount = 7;
        WallBot c = facingPlusX(w);
        c.start(Mode::On);
        c.run(1500);
        check(c.fixes() == 1, "keepOutCount 7: the 8th is not used");
    }
}

// After a good start check, the sensor reads addMm off; returns the run's status after ms
static WallBot afterStart(const WallConfig& w, const Config& c, int addMm, int ms = 1500, double estX = 50.25) {
    WallBot b = facingPlusX(w, c, 0, estX);
    b.start(Mode::On);
    runUntilStarted(b);
    b.sim[0].addMm = addMm;
    b.run(ms);
    return b;
}

static void testChecks9To13() {
    std::printf("wall checks 9 to 13: blocked, through, gate, cap, once per stop\n");
    // with startSigmaIn 1 and m about 20: R about 0.25, s = sqrt(1 + R) = 1.118, 2s = 2.236, 3s = 3.354
    {
        WallBot b = afterStart(WallConfig(), quietConfig(), -75); // 3 in short: something 3 in in front of the wall
        Status keep;
        const WallSensorStatus& w = b.wall(0, keep);
        check(w.fixes == 0 && w.droppedByCheck[WCHECK_NOT_BLOCKED] > 25, "3 in shorter than predicted: wall check 9");
        checkOnlyDrops(w, {WCHECK_STILL, WCHECK_STEADY, WCHECK_NOT_BLOCKED}, "  and nothing else");
    }
    {
        WallBot b = afterStart(WallConfig(), quietConfig(), +100, 1500, 52.25); // 18 in, 4 in long: through
        Status keep;
        const WallSensorStatus& w = b.wall(0, keep);
        check(w.fixes == 0 && w.droppedByCheck[WCHECK_NOT_THROUGH] >= 6, "4 in longer than predicted: wall check 10");
        checkOnlyDrops(w, {WCHECK_STILL, WCHECK_STEADY, WCHECK_NOT_THROUGH}, "  and nothing else");
    }
    {
        // 3.0 in long: inside the through gate (3.354) but outside the statistical gate (2.5 s = 2.795)
        WallBot b = afterStart(WallConfig(), quietConfig(), +75);
        Status keep;
        const WallSensorStatus& w = b.wall(0, keep);
        check(w.fixes == 0 && w.droppedByCheck[WCHECK_GATE] > 25, "3.0 in long (2.5 s = 2.8, 3 s = 3.35): wall check 11");
        checkOnlyDrops(w, {WCHECK_STILL, WCHECK_STEADY, WCHECK_GATE}, "  and nothing else");
    }
    {
        // 2 in long: K = 1 / 1.2704 = 0.787, the correction 1.57 in > the Clear cap 1.5: dropped, not clipped
        WallBot b = afterStart(WallConfig(), quietConfig(), +50);
        Status keep;
        const WallSensorStatus& w = b.wall(0, keep);
        check(w.fixes == 0 && w.droppedByCheck[WCHECK_CAP] > 25, "a 1.57 in correction on a Clear wall (cap 1.5): wall check 12");
        checkOnlyDrops(w, {WCHECK_STILL, WCHECK_STEADY, WCHECK_CAP}, "  and nothing else");
        check(keep.decidedXIn == 0 && keep.pendingXIn == 0, "  dropped, not clipped: nothing decided");
        WallConfig solid;
        solid.trust[WALL_PLUS_X] = WallTrust::Solid;
        WallBot s = afterStart(solid, quietConfig(), +50);
        check(s.fixes() == 1, "the same on a Solid wall (cap 6): used");
        checkNear(s.status().decidedXIn, -2 / (1 + std::pow(0.3 + 0.01 * 22, 2)), 1e-4,
                  "  correction = -2 * K, K = 1 / (1 + (0.3 + 0.01 * 22)^2)");
    }
    {
        // 13: after a fix, every other reading at the same stop is dropped by wall check 13
        WallBot b = facingPlusX();
        b.start(Mode::On);
        check(runUntilFix(b), "(fixed)");
        const std::uint32_t before = b.dropped(WCHECK_ONCE_PER_STOP);
        b.run(2000);
        Status keep;
        const WallSensorStatus& w = b.wall(0, keep);
        check(w.fixes == 1 && w.droppedByCheck[WCHECK_ONCE_PER_STOP] - before >= 45,
              "2 s more at the same stop: no second fix, every reading wall check 13");
    }
}

// The gates of wall checks 9 and 10 are along the axis: the beam 9.5 degrees off square (heading 80.5), sigma 2
// (3s = 6.2, so the through gate is throughMaxIn = 4), the reading 4.04 in longer than predicted along the beam:
// along the axis that is 4.04 cos 9.5 = 3.985, inside the gate (along the beam it would be outside)
static void testGatesAlongAxis() {
    std::printf("wall checks 9 and 10 measure along the axis, not along the beam\n");
    WallConfig w;
    w.trust[WALL_PLUS_X] = WallTrust::Solid;
    Config c = quietConfig();
    c.startSigmaIn = 2;
    for (int extraMm : {101, 103}) { // 4.04 in (3.985 along the axis) and 4.12 in (4.063)
        WallBot b(c, w, {centreForward()}, 70.25 - 20 * std::cos(rad(9.5)), 0, 80.5);
        b.start(Mode::On);
        runUntilStarted(b);
        b.sim[0].addMm = extraMm;
        b.run(1500);
        char what[120];
        std::snprintf(what, sizeof what, "%.2f in long along a beam 9.5 deg off square (%.3f along the axis): %s",
                      extraMm / 25.0, extraMm / 25.0 * std::cos(rad(9.5)), extraMm == 101 ? "not through" : "through");
        check(extraMm == 101 ? (b.dropped(WCHECK_NOT_THROUGH) == 0 && b.fixes() == 1) : b.dropped(WCHECK_NOT_THROUGH) > 5,
              what);
    }
}

static void testGateEdges() {
    std::printf("Revision A gates at their edges: shortGate = min(2.5, max(1, 2s)), longGate = min(4, 3s)\n");
    // s = sqrt(sigma^2 + R). Each case: the doubt, how many mm long (+) or short (-) the reading is, and whether
    // it is dropped by the gate under test. The sensor stays at 20 in (Solid wall: cap 6, so the cap never acts).
    // Hand-computed: R = (0.3 + 0.01 m)^2 with m = 20 - mm/25
    struct Case {
            float startSigma;
            int mm;
            int gateCheck;
            bool dropped;
            const char* what;
    };
    const Case cases[] = {
        // sigma 0.3 (startSigmaIn 0.2, raised to the floor): s = sqrt(0.09 + 0.24) = 0.57.. -> 2s = 1.15 > 1:
        // use the floor case with a small R: see below. Here sigma 1: s = 1.1136 (m 19.1) ... 2s = 2.227
        {1, -54, WCHECK_NOT_BLOCKED, false, "sigma 1: 2.16 in short, under 2s = 2.23: not blocked"},
        {1, -58, WCHECK_NOT_BLOCKED, true, "sigma 1: 2.32 in short, over 2s = 2.23: blocked"},
        // sigma 2: s = 2.06, 2s = 4.1 -> capped at blockedMaxIn 2.5
        {2, -61, WCHECK_NOT_BLOCKED, false, "sigma 2: 2.44 in short, under blockedMaxIn 2.5: not blocked"},
        {2, -64, WCHECK_NOT_BLOCKED, true, "sigma 2: 2.56 in short, over blockedMaxIn 2.5: blocked"},
        // sigma 1: 3s = 3.35 (m about 23.3: R = 0.2841, s = 1.1333, 3s = 3.40)
        {1, +83, WCHECK_NOT_THROUGH, false, "sigma 1: 3.32 in long, under 3s = 3.40: not through"},
        {1, +87, WCHECK_NOT_THROUGH, true, "sigma 1: 3.48 in long, over 3s = 3.40: through"},
        // sigma 2: 3s = 6.2 -> capped at throughMaxIn 4 (m 23.9)
        {2, +98, WCHECK_NOT_THROUGH, false, "sigma 2: 3.92 in long, under throughMaxIn 4: not through"},
        {2, +102, WCHECK_NOT_THROUGH, true, "sigma 2: 4.08 in long, over throughMaxIn 4: through"},
    };
    for (const Case& k : cases) {
        WallConfig w;
        w.trust[WALL_PLUS_X] = WallTrust::Solid;
        Config c = quietConfig();
        c.startSigmaIn = k.startSigma;
        WallBot b = afterStart(w, c, k.mm);
        check(k.dropped ? b.dropped(k.gateCheck) > 5 : b.dropped(k.gateCheck) == 0, k.what);
    }
    // blockedMinIn: with the doubt at its floor 0.3 and the sensor 5 in from the wall, R = (0.35)^2 = 0.1225 and
    // s = sqrt(0.09 + 0.1225) = 0.461, 2s = 0.92 < 1: the short gate is blockedMinIn = 1.0
    {
        WallConfig w;
        w.trust[WALL_PLUS_X] = WallTrust::Solid;
        Config c = quietConfig();
        c.startSigmaIn = 0.1f; // raised to the floor, 0.3
        WallBot under = afterStart(w, c, -24, 1500, 65.25); // 0.96 in short (5 in: 125 mm, 101 mm)
        WallBot over = afterStart(w, c, -26, 1500, 65.25); // 1.04 in short
        Status keep = under.status();
        checkNear(keep.sigmaXIn, 0.3, 1e-6, "(the doubt starts at the floor 0.3)");
        check(under.dropped(WCHECK_NOT_BLOCKED) == 0, "doubt 0.3 at 5 in: 0.96 in short, under blockedMinIn 1.0: not blocked");
        check(over.dropped(WCHECK_NOT_BLOCKED) > 5, "  1.04 in short, over blockedMinIn 1.0: blocked");
        // and the statistical gate (2.5 s = 1.15) then lets 0.96 through
        check(under.fixes() == 1, "  (0.96 in short passes the statistical gate 2.5 s = 1.15 and fixes)");
    }
}

// ------------------------------------------------------------------------------------------------ safeguards

static void testWallTrust() {
    std::printf("wall trust: Ignore is never used; Clear and Solid differ in confidence, range and cap\n");
    // (the confidence, range and cap differences are each tested with their checks above; here the defaults)
    const WallConfig w;
    for (int wall = 0; wall < 4; wall++) check(w.trust[wall] == WallTrust::Clear, "every wall starts Clear");
    checkNear(w.planeIn[0] + w.planeIn[2], 140.5, 0, "inside width 140.5 in (y)");
    checkNear(w.planeIn[1] + w.planeIn[3], 140.5, 0, "inside width 140.5 in (x)");
    // a Clear wall: a reading 1.2 in long gives a 0.95 in fix (cap 1.5); the same on an Ignore wall gives nothing
    WallBot clear = afterStart(WallConfig(), quietConfig(), +30);
    check(clear.fixes() == 1, "Clear: a 1.2 in disagreement is fixed");
    WallConfig ignore;
    ignore.trust[WALL_PLUS_X] = WallTrust::Ignore;
    WallBot b = facingPlusX(ignore);
    b.start(Mode::On);
    b.run(3000);
    check(b.fixes() == 0 && b.state() == WallSensorState::WaitingForStart, "Ignore: no fix and no start check, ever");
}

static void testStartCheck() {
    std::printf("start check, per sensor: the first reading past wall checks 1 to 8 against the pose\n");
    // 2.88 in off (72 mm): passes; 3.12 in off (78 mm): faults (wallStartTolIn 3)
    for (int mm : {72, -72, 78, -78}) {
        WallBot b = facingPlusX();
        b.sim[0].addMm = mm;
        b.start(Mode::On);
        runUntilStarted(b);
        Status keep;
        const WallSensorStatus& w = b.wall(0, keep);
        const bool fault = std::abs(mm) > 75;
        char what[120];
        std::snprintf(what, sizeof what, "first reading %.2f in off (limit 3): %s", mm / 25.0,
                      fault ? "StartFault" : "Working");
        check(w.state == (fault ? WallSensorState::StartFault : WallSensorState::Working), what);
        checkNear(w.startInnovationIn, -mm / 25.0, 1e-4, "  the start check's innovation is kept");
        check(keep.decidedXIn == 0 && keep.decidedYIn == 0, "  the start check's reading fixes nothing");
    }
    // the start pose 5 in wrong (W8): the sensor faults and never fixes; the pose stays odometry
    {
        WallBot b = facingPlusX(WallConfig(), quietConfig(), 5);
        b.start(Mode::On);
        b.run(5000);
        check(b.state() == WallSensorState::StartFault && b.fixes() == 0, "start pose 5 in wrong: StartFault, no fix");
        check(b.appliedX == 0 && b.appliedY == 0, "  the pose is odometry alone");
        check(b.status().fault == Fault::None, "  it is the sensor's fault, not the filter's");
    }
    // one sensor faults, the other carries on: sensor 0 looks at the +x wall from a pose 5 in wrong in x, sensor 1
    // (looking left, at the +y wall) sees y right
    {
        WallSensorMount left = centreForward();
        left.yawDeg = 270; // heading 90, looking left: heading 0, the +y wall
        WallBot b(quietConfig(), WallConfig(), {centreForward(), left}, 50.25, 50.25, 90);
        b.trueX = 55.25;
        b.truthLog.clear();
        b.truthLog.push_back({b.t, b.trueX, b.trueY, b.trueThetaDeg});
        b.start(Mode::On);
        b.run(3000);
        check(b.state(0) == WallSensorState::StartFault, "sensor 0 (x 5 in wrong): StartFault");
        check(b.state(1) == WallSensorState::Working && b.fixes(1) == 1, "sensor 1 carries on and fixes y");
    }
}

static void testHealthRule() {
    std::printf("health: half or more of at least healthMinTries judged readings bad = switched off\n");
    // a Clear wall that the beam reads through (10 in long) at a stop, after a good stop gave G good tries.
    // Good tries at the first stop: the fix and every reading after it (wall check 13 drops them, but they passed
    // wall check 10). It switches off on the bad try that makes 2 * bad >= tries with tries >= 6
    for (int good : {0, 3, 4, 8}) {
        WallConfig w;
        w.wallStartTolIn = 20; // so the start check can happen on the through readings when good is 0
        WallBot b = facingPlusX(w, quietConfig(), 0, 55.25, -20); // 15 in from the wall
        b.start(Mode::On);
        if (good > 0) {
            runUntilStarted(b);
            for (int i = 0; i < 1000 && int(b.status().walls[0].healthTries) < good; i++) b.tick();
        }
        // slide 8 in along the wall at 4 in/s (not judged while moving), then stand
        for (int i = 0; i < 200; i++) {
            b.trueY += 0.04;
            b.odomY += 0.04f;
            b.tick();
        }
        Status keep = b.status();
        checkCount(keep.walls[0].healthTries, std::uint32_t(good), "(good tries before the bad stop; none while sliding)");
        b.sim[0].addMm = 125; // 5 in long from now on (m = 20: in range; over throughMaxIn 4)
        int badSeen = 0;
        int offAt = -1;
        for (int i = 0; i < 300 && offAt < 0; i++) {
            b.tick();
            keep = b.status();
            badSeen = int(keep.walls[0].healthBad);
            if (keep.walls[0].state == WallSensorState::HealthOff) offAt = badSeen;
        }
        // good 0: the start check takes the first through reading, then 6 bad; good 3: 3 bad (6 tries);
        // good 4: 4 bad (8 tries); good 8: 8 bad (16 tries)
        const int want = good == 0 ? 6 : good;
        char what[120];
        std::snprintf(what, sizeof what, "%d good tries first: switched off on the bad try number %d (got %d)", good,
                      want, offAt);
        check(offAt == want, what);
    }
    // invalid and low-confidence readings while still with a clean view are bad tries; while moving, or when the
    // view is not clean (the wall beyond its range), they are not judged
    {
        WallBot b = facingPlusX();
        b.sim[0].confidence = 10; // too low on a Clear wall (50)
        b.start(Mode::On);
        b.run(400);
        Status keep = b.status();
        check(keep.walls[0].state == WallSensorState::HealthOff, "low confidence while still, clean view: switched off");
        checkCount(keep.walls[0].healthBad, 6, "  after 6 bad tries (healthMinTries)");
        checkCount(keep.walls[0].droppedByCheck[WCHECK_CONFIDENT], 9, "  (3 of the 9 readings came before 100 ms of history)");
    }
    {
        WallBot b = facingPlusX();
        b.sim[0].fixed = true;
        b.sim[0].fixedMm = 9999; // sees nothing
        b.start(Mode::On);
        b.run(400);
        check(b.state() == WallSensorState::HealthOff, "nothing seen (9999) while still, clean view: switched off");
    }
    {
        WallBot b = facingPlusX(WallConfig(), quietConfig(), 0, 40.25); // 30 in from a Clear wall: out of range
        b.sim[0].fixed = true;
        b.sim[0].fixedMm = 9999;
        b.start(Mode::On);
        b.run(3000);
        Status keep = b.status();
        check(keep.walls[0].state == WallSensorState::WaitingForStart && keep.walls[0].healthTries == 0,
              "nothing seen 30 in from a Clear wall (beyond its 24 in): not judged");
    }
    {
        WallBot b = facingPlusX(WallConfig(), quietConfig(), 0, 50.25, -30);
        b.sim[0].fixed = true;
        b.sim[0].fixedMm = 9999;
        b.start(Mode::On);
        for (int i = 0; i < 300; i++) {
            b.trueY += 0.03;
            b.odomY += 0.03f;
            b.tick();
        }
        Status keep = b.status();
        check(keep.walls[0].state == WallSensorState::WaitingForStart && keep.walls[0].healthTries == 0,
              "nothing seen while sliding at 3 in/s: not judged");
    }
    // blocked readings are not judged (not a clear-wall symptom)
    {
        WallBot b = afterStart(WallConfig(), quietConfig(), -75, 3000);
        Status keep = b.status();
        check(keep.walls[0].state == WallSensorState::Working && keep.walls[0].healthTries == 0,
              "blocked readings: not judged, the sensor stays on");
    }
    // healthMinTries: 5 bad tries out of 5 do not switch it off when 6 are needed
    {
        WallBot b = facingPlusX();
        b.sim[0].confidence = 10;
        b.start(Mode::On);
        b.run(330); // samples at +10, 50, 90 (no history), then 130, 170, 210, 250, 290: 5 judged, 330: 6th
        Status keep = b.status();
        // at +330 the sixth has just been counted
        check(keep.walls[0].healthBad == 6 && keep.walls[0].state == WallSensorState::HealthOff, "6 of 6: off");
        WallBot c = facingPlusX();
        c.sim[0].confidence = 10;
        c.start(Mode::On);
        c.run(300);
        keep = c.status();
        check(keep.walls[0].healthBad == 5 && keep.walls[0].state == WallSensorState::WaitingForStart,
              "5 of 5: not yet (healthMinTries 6)");
    }
    // the health rule can switch the sensor off on a good try (healthMinTries 2: one invalid reading, then the
    // start check, then the first reading past wall check 10 makes 1 bad of 2): that reading does not fix
    {
        WallConfig w;
        w.healthMinTries = 2;
        WallBot b = facingPlusX(w, quietConfig(), 1);
        b.sim[0].script = {0, 0, 0, 100000, 0};
        b.start(Mode::On);
        b.run(1500);
        Status keep = b.status();
        check(keep.walls[0].state == WallSensorState::HealthOff && keep.walls[0].healthTries == 2,
              "switched off on a good try (1 bad of 2)");
        check(keep.walls[0].fixes == 0 && keep.decidedXIn == 0, "  and that reading fixes nothing");
    }
    // the other sensor and the GPS carry on, and nothing is fed back out
    {
        WallSensorMount left = centreForward();
        left.yawDeg = 270;
        WallBot b(quietConfig(), WallConfig(), {centreForward(), left}, 50.25, 50.25, 90);
        b.trueY = 51.25; // a 1 in y error for sensor 1 to fix
        b.truthLog.clear();
        b.truthLog.push_back({b.t, b.trueX, b.trueY, b.trueThetaDeg});
        b.start(Mode::On);
        runUntilFix(b, 1);
        const double decided = b.status().decidedYIn;
        b.sim[0].confidence = 10; // sensor 0 goes bad
        b.run(3000);
        check(b.state(0) == WallSensorState::HealthOff, "sensor 0 switched off");
        check(b.state(1) == WallSensorState::Working, "sensor 1 still working");
        checkNear(b.status().decidedYIn, decided, 0, "  its fix is kept: nothing fed back out");
        checkNear(b.appliedY, decided, 1e-5, "  and fed in");
    }
}

static void testOncePerStop() {
    std::printf("wall check 13: once per stop, until refixTravelIn travelled or refixTurnDeg turned\n");
    for (double travel : {5.8, 6.2}) {
        WallBot b = facingPlusX(WallConfig(), quietConfig(), 0, 50.25, -10);
        b.start(Mode::On);
        runUntilFix(b);
        // slide along the wall by `travel` at 2 in/s, then stand 1 s
        const int ticks = int(std::lround(travel / 0.02));
        for (int i = 0; i < ticks; i++) {
            b.trueY += 0.02;
            b.odomY += 0.02f;
            b.tick();
        }
        b.run(1000);
        char what[100];
        std::snprintf(what, sizeof what, "%.1f in travelled since the fix (6 needed): %s", travel,
                      travel > 6 ? "fixes again" : "no second fix");
        checkCount(b.fixes(), travel > 6 ? 2 : 1, what);
    }
    // turning: away by `turn` and back (the beam must be square again), so 2 * turn is turned in all
    for (double turn : {9.0, 10.5}) {
        WallBot b = facingPlusX();
        b.start(Mode::On);
        runUntilFix(b);
        const int ticks = int(std::lround(turn / 0.02)); // at 2 deg/s, under wallMaxTurnRate
        for (int i = 0; i < ticks; i++) b.tick(10, 0, 2);
        for (int i = 0; i < ticks; i++) b.tick(10, 0, -2);
        b.run(1000);
        char what[100];
        std::snprintf(what, sizeof what, "turned %.0f deg away and back (%.0f in all; 20 needed): %s", turn, 2 * turn,
                      2 * turn >= 20 ? "fixes again" : "no second fix");
        checkCount(b.fixes(), 2 * turn >= 20 ? 2 : 1, what);
    }
    // each sensor has its own: sensor 1's fix does not stop sensor 0
    {
        WallSensorMount left = centreForward();
        left.yawDeg = 270;
        WallBot b(quietConfig(), WallConfig(), {centreForward(), left}, 50.25, 50.25, 90);
        b.start(Mode::On);
        b.run(2000);
        check(b.fixes(0) == 1 && b.fixes(1) == 1, "two sensors at one stop: one fix each");
    }
}

static void testDoubtPerAxis() {
    std::printf("per-axis doubt: a fix shrinks its own axis only; never below sigmaFloorIn\n");
    // both start at startSigmaIn (Revision A), with a wall sensor configured or not
    {
        Config c = quietConfig();
        c.startSigmaIn = 1.7f;
        WallBot b = facingPlusX(WallConfig(), c);
        b.start(Mode::On);
        Status s = b.status();
        check(s.sigmaXIn == 1.7f && s.sigmaYIn == 1.7f && s.sigmaIn == 1.7f, "switched on: both doubts startSigmaIn");
        GpsFusionCore none(c);
        none.setMode(Mode::On);
        check(none.getStatus().sigmaXIn == 1.7f && none.getStatus().sigmaYIn == 1.7f,
              "  also with no wall sensor (the status only: nothing reads it before the GPS start check)");
        c.startSigmaIn = 0.1f;
        WallBot f = facingPlusX(WallConfig(), c);
        f.start(Mode::On);
        check(f.status().sigmaXIn == 0.3f, "  a startSigmaIn under the floor starts at the floor with walls");
        GpsFusionCore noFloor(c);
        noFloor.setMode(Mode::On);
        check(noFloor.getStatus().sigmaXIn == 0.1f, "  and at startSigmaIn without walls (no floor then)");
    }
    // an x fix: sigmaX = sqrt(1 - K), sigmaY stays; status sigmaIn is the larger
    {
        WallBot b = facingPlusX(WallConfig(), quietConfig(), 1);
        b.start(Mode::On);
        runUntilFix(b);
        const Status s = b.status();
        checkNear(s.sigmaXIn, 0.44001525, 1e-6, "an x fix (m 19): sigmaX = sqrt(1 - 1/1.2401) = 0.440015");
        checkNear(s.sigmaYIn, 1, 0, "  sigmaY untouched");
        checkNear(s.sigmaIn, 1, 0, "  status sigmaIn = the larger");
        check(s.decidedYIn == 0, "  y untouched");
    }
    // both grow by the same rule (kDist 0.02 per inch): 10 in of driving adds 0.2 to each
    {
        Config c = quietConfig();
        c.kDist = 0.02f;
        WallBot b = facingPlusX(WallConfig(), c, 1);
        b.start(Mode::On);
        runUntilFix(b);
        const Status s0 = b.status();
        b.run(2500, -4); // drive 10 in backward, away from the wall
        const Status s1 = b.status();
        checkNear(s1.sigmaXIn - s0.sigmaXIn, 0.2, 1e-4, "10 in driven: sigmaX + 0.2");
        checkNear(s1.sigmaYIn - s0.sigmaYIn, 0.2, 1e-4, "10 in driven: sigmaY + 0.2");
    }
    // and with turning (kTurn 0.3 per radian): 90 degrees in place adds 0.3 * pi/2 = 0.4712 to each
    {
        Config c = quietConfig();
        c.kTurn = 0.3f;
        WallBot b = facingPlusX(WallConfig(), c);
        b.start(Mode::On);
        b.tick(); // the first tick after switching on has no earlier pose to measure a turn from
        const Status s0 = b.status();
        b.run(3000, 0, 30);
        const Status s1 = b.status();
        checkNear(s1.sigmaXIn - s0.sigmaXIn, 0.3 * PI / 2, 1e-4, "90 deg turned: sigmaX + 0.4712");
        checkNear(s1.sigmaYIn - s0.sigmaYIn, 0.3 * PI / 2, 1e-4, "90 deg turned: sigmaY + 0.4712");
    }
    // the GPS start check sets both doubts to startSigmaIn (Revision A), also one that a wall fix shrank before
    // it: the robot faces the +y wall 1 in closer than odometry says, its sensor fixes y (sigmaY 0.44), and then
    // the GPS (perfect) passes its start check
    {
        WallBot b(quietConfig(), WallConfig(), {centreForward()}, 0, 50.25, 0);
        b.trueY = 51.25;
        b.truthLog.clear();
        b.truthLog.push_back({b.t, b.trueX, b.trueY, b.trueThetaDeg});
        b.gpsPlugged = true;
        b.start(Mode::On);
        check(runUntilFix(b), "(a y wall fix)");
        Status s = b.status();
        check(!s.startCheckPassed && s.sigmaYIn < 0.5f && s.sigmaXIn == 1.0f,
              "(it came before the GPS start check passed: sigmaY 0.44, sigmaX 1)");
        for (int i = 0; i < 300 && !b.status().startCheckPassed; i++) b.tick();
        s = b.status();
        check(s.startCheckPassed && s.sigmaXIn == 1.0f && s.sigmaYIn == 1.0f,
              "the GPS start check passes: both doubts set to startSigmaIn (1)");
    }
    // repeated fixes with travel in between: sigmaX reaches the floor 0.3 and stays there
    {
        WallConfig w;
        w.trust[WALL_PLUS_X] = WallTrust::Solid;
        WallBot b = facingPlusX(w, quietConfig(), 0, 50.25, -40);
        b.start(Mode::On);
        float lowest = 10;
        for (int stop = 0; stop < 8; stop++) {
            runUntilFix(b);
            lowest = std::fmin(lowest, b.status().sigmaXIn);
            for (int i = 0; i < 350; i++) {
                b.trueY += 0.02;
                b.odomY += 0.02f;
                b.tick();
            }
        }
        checkCount(b.fixes(), 8, "(8 fixes, one per stop)");
        checkNear(lowest, 0.3, 1e-6, "8 fixes: sigmaX stops at the floor 0.3");
        check(b.status().sigmaXIn == 0.3f, "  and is there now");
    }
}

// Feed the GPS until its start check has passed, then make every later reading fail check 5 (heading 20 deg
// off) so it keeps the sensor alive and changes nothing
static void gpsStartThenQuiet(WallBot& b) {
    b.gpsPlugged = true;
    b.feedWalls = false; // the start check's pass sets both doubts to startSigmaIn: no wall fix before it
    for (int i = 0; i < 300 && !b.status().startCheckPassed; i++) b.tick();
    b.gpsHeadingOffDeg = 20;
    b.feedWalls = true;
}

// One GPS reading (dx, dy) off the truth; returns the status before and after it was used
static void gpsShot(WallBot& b, double dx, double dy, Status& before, Status& after) {
    before = b.status();
    b.shotDx = dx;
    b.shotDy = dy;
    b.shotPending = true;
    const std::uint32_t accepted = before.accepted, gated = before.droppedByGate;
    for (int i = 0; i < 30; i++) {
        b.tick();
        const Status s = b.status();
        if (s.accepted != accepted || s.droppedByGate != gated) break;
    }
    after = b.status();
}

static void testGpsPerAxis() {
    std::printf("the GPS with unequal doubts: per-axis gain and gate, the vector step limit\n");
    // A perfect x fix first (m = 20, R = 0.25, K = 0.8): sigmaX = sqrt(0.2) = 0.4472136, sigmaY = 1.
    // sigmaGps = 1.5: totalX = 0.2 + 2.25 = 2.45, totalY = 1 + 2.25 = 3.25. Kx = 0.2 / 2.45 = 0.0816327,
    // Ky = 1 / 3.25 = 0.3076923
    {
        WallBot b = facingPlusX();
        b.start(Mode::On);
        gpsStartThenQuiet(b);
        check(b.status().startCheckPassed, "(the GPS start check passed)");
        check(runUntilFix(b), "(a perfect x wall fix)");
        checkNear(b.status().sigmaXIn, std::sqrt(0.2), 1e-6, "(sigmaX = sqrt(0.2))");
        b.run(500); // let it settle (a zero correction)
        Status before, after;
        // innovation (2, 2): gate 4/2.45 + 4/3.25 = 2.863 <= 6.25: used. Step (0.16327, 0.61538), 0.6367 < 1
        gpsShot(b, 2, 2, before, after);
        check(after.accepted == before.accepted + 1, "innovation (2, 2): accepted");
        checkNear(after.decidedXIn - before.decidedXIn, 2 * 0.2 / 2.45, 2e-4, "  x step = Kx * 2 = 0.163265");
        checkNear(after.decidedYIn - before.decidedYIn, 2 / 3.25, 2e-4, "  y step = Ky * 2 = 0.615385");
        checkNear(after.sigmaXIn, std::sqrt(1 - 0.2 / 2.45) * std::sqrt(0.2), 1e-5, "  sigmaX = sqrt(1 - Kx) sigmaX");
        checkNear(after.sigmaYIn, std::sqrt(1 - 1 / 3.25), 1e-5, "  sigmaY = sqrt(1 - Ky) sigmaY");
    }
    // the vector step limit: innovation (1, 4) with sigmaX 0.4472, sigmaY 1: gate 1/2.45 + 16/3.25 = 5.33: used.
    // Step (0.0816, 1.2308), length 1.2335 > 1: both gains times 1/1.2335 = 0.81070
    {
        WallBot b = facingPlusX();
        b.start(Mode::On);
        gpsStartThenQuiet(b);
        runUntilFix(b);
        b.run(500);
        Status before, after;
        gpsShot(b, 1, 4, before, after);
        const double kx = 0.2 / 2.45, ky = 1 / 3.25;
        const double len = std::hypot(kx * 1, ky * 4);
        const double f = 1 / len;
        check(after.accepted == before.accepted + 1, "innovation (1, 4): accepted");
        checkNear(std::hypot(after.decidedXIn - before.decidedXIn, after.decidedYIn - before.decidedYIn), 1, 1e-4,
                  "  the step is exactly maxStepIn = 1 long");
        checkNear(after.decidedXIn - before.decidedXIn, f * kx, 2e-4, "  x step = 0.81070 * Kx * 1 = 0.066180");
        checkNear(after.decidedYIn - before.decidedYIn, f * ky * 4, 2e-4, "  y step = 0.81070 * Ky * 4 = 0.997802");
        checkNear(after.sigmaXIn, std::sqrt(1 - f * kx) * std::sqrt(0.2), 1e-5,
                  "  sigmaX shrinks by the scaled gain: sqrt(1 - 0.81070 Kx) sigmaX");
        checkNear(after.sigmaYIn, std::sqrt(1 - f * ky), 1e-5, "  sigmaY shrinks by the scaled gain");
        checkNear(after.maxReadingStepIn, 1, 1e-4, "  maxReadingStepIn = 1");
    }
    // the per-axis gate: innovation (3.6, 0) with sigmaX 0.4472: 12.96 / 2.45 = 5.29 <= 6.25 used; (4, 0):
    // 16 / 2.45 = 6.53 > 6.25 dropped (the larger doubt, 1, would have let it through: 16 / 3.25 = 4.9)
    for (double dx : {3.6, 4.0}) {
        WallBot b = facingPlusX();
        b.start(Mode::On);
        gpsStartThenQuiet(b);
        runUntilFix(b);
        b.run(500);
        Status before, after;
        gpsShot(b, dx, 0, before, after);
        char what[120];
        std::snprintf(what, sizeof what, "innovation (%.1f, 0) against sigmaX 0.447: %s", dx,
                      dx > 3.8 ? "dropped by the gate (per axis)" : "accepted");
        if (dx > 3.8) check(after.droppedByGate == before.droppedByGate + 1 && after.accepted == before.accepted, what);
        else check(after.accepted == before.accepted + 1, what);
    }
    // the floor also holds for the GPS while a wall sensor is configured
    {
        WallConfig w;
        w.sigmaFloorIn = 0.9f;
        WallBot b = facingPlusX(w);
        b.start(Mode::On);
        gpsStartThenQuiet(b);
        Status before, after;
        b.run(500);
        gpsShot(b, 0.5, 0.5, before, after);
        check(after.accepted == before.accepted + 1, "(a GPS reading accepted)");
        check(after.sigmaXIn == 0.9f && after.sigmaYIn == 0.9f,
              "sigmaFloorIn 0.9: a GPS update does not take the doubts below it (sqrt(1 - 1/3.25) = 0.83)");
    }
}

// ------------------------------------------------------------------------------------------------ faults

static void testFaultStage1() {
    std::printf("Revision A stage 1: a GPS fault feeds everything back out, then the walls carry on alone\n");
    // odometry 1 in short of the +x wall; the GPS reports 10 in off, so its start check faults (StartDisagree)
    // after the wall has already fixed
    WallBot b = facingPlusX(WallConfig(), quietConfig(), 1);
    b.gpsPlugged = true;
    b.gpsDx = 10;
    b.start(Mode::On);
    check(runUntilFix(b), "the wall fixes before the GPS start check is over");
    const double firstFix = b.status().decidedXIn;
    checkNear(firstFix, 0.80638658, 1e-4, "  (0.806 in)");
    // run until the fault
    int ticks = 0;
    while (b.status().fault == Fault::None && ticks++ < 300) b.tick();
    Status s = b.status();
    check(s.fault == Fault::StartDisagree, "the GPS faults: StartDisagree");
    check(s.wallsWaiting && !s.wallsStopped, "  the walls wait (stage 1)");
    const double fedAtFault = b.appliedX;
    check(fedAtFault > 0.1, "  (part of the wall fix had been fed in)");
    // while it is fed back out, no wall fix
    const std::uint32_t fixesAtFault = s.walls[0].fixes;
    const std::uint32_t samplesAtFault = s.walls[0].samples;
    bool sawZero = false;
    bool lookedAt = false;
    for (int i = 0; i < 300 && !sawZero; i++) {
        b.tick();
        sawZero = b.status().appliedXIn == 0;
        if (!sawZero) lookedAt = lookedAt || b.status().walls[0].samples != samplesAtFault;
    }
    s = b.status();
    check(sawZero, "everything fed in is fed back out, to exactly 0");
    check(!lookedAt, "  no wall reading looked at meanwhile");
    checkCount(s.walls[0].fixes, fixesAtFault, "  no wall fix meanwhile");
    // now the walls carry on alone
    check(!s.wallsWaiting && !s.unwinding, "then the walls carry on (no longer waiting)");
    checkNear(s.sigmaXIn, 5, 0, "  both doubts sigmaMaxIn (5)");
    checkNear(s.sigmaYIn, 5, 0, "  ");
    checkNear(s.decidedXIn, 0, 0, "  the total starts again from 0");
    check(runUntilFix(b, 0, 1500), "and the same sensor fixes again at the same stop (no travel needed)");
    s = b.status();
    // with sigma 5: K = 25 / (25 + 0.2401) = 0.990487; innovation 1 (the real robot 1 in closer)
    checkNear(s.decidedXIn, 25 / 25.2401, 1e-4, "  a fix with doubt 5: K = 25 / 25.2401");
    b.run(2000);
    checkNear(b.appliedX, 25 / 25.2401, 1e-4, "  and it is fed in");
    check(b.status().fault == Fault::StartDisagree, "  the GPS stays out (the fault is still shown)");
    checkCount(b.status().newReadings, s.newReadings, "  and its readings are no longer looked at");

    // GPS-only, as revision 2: after the fault nothing runs again
    {
        WallBot g(quietConfig(), WallConfig(), {}, 0, 0, 0);
        g.gpsPlugged = true;
        g.gpsDx = 10;
        g.start(Mode::On);
        g.run(3000);
        const Status a = g.status();
        g.run(2000);
        const Status c = g.status();
        check(a.fault == Fault::StartDisagree && c.newReadings == a.newReadings && !c.wallsWaiting,
              "no wall sensor: after a GPS fault the filter stays frozen (revision 2)");
    }
    // a wall sensor that has already faulted cannot carry on: frozen as revision 2
    {
        WallBot f = facingPlusX(WallConfig(), quietConfig(), 5); // the wall start check will fault
        f.gpsPlugged = true;
        f.gpsDx = 10;
        f.start(Mode::On);
        f.run(3000);
        const Status a = f.status();
        check(a.fault == Fault::StartDisagree && a.walls[0].state == WallSensorState::StartFault && !a.wallsWaiting,
              "GPS fault with the only wall sensor already faulted: nothing carries on");
        check(a.sigmaXIn == 1 && a.sigmaYIn == 1, "  the filter stays frozen (the doubts are not set to sigmaMaxIn)");
    }
}

static void testFaultStage2() {
    std::printf("Revision A stage 2: the total limit with the GPS out or absent stops the walls too\n");
    // walls only, maxTotalCorrectionIn 2: odometry slips 1.2 in short before each of three stops (Solid wall)
    // (the doubt grows 0.2 in per inch driven, so it is large again at each stop and each fix takes most of the
    // slip: with no growth the fixes shrink, the pose drifts past the blocked gate, and the total is never reached)
    Config c = quietConfig();
    c.maxTotalCorrectionIn = 2;
    c.startSigmaIn = 3;
    c.kDist = 0.2f;
    WallConfig w;
    w.trust[WALL_PLUS_X] = WallTrust::Solid;
    WallBot b = facingPlusX(w, c, 0, 50.25, -30);
    b.start(Mode::On);
    runUntilStarted(b);
    int stops = 0;
    for (; stops < 4 && b.status().fault == Fault::None; stops++) {
        b.trueX += 1.2; // a slip the wheels did not see: really 1.2 in closer
        b.run(800);
        for (int i = 0; i < 350; i++) { // slide 7 in along the wall
            b.trueY += 0.02;
            b.odomY += 0.02f;
            b.tick();
        }
    }
    Status s = b.status();
    check(s.fault == Fault::TotalTooLarge && s.wallsStopped, "the walls' total went over 2 in: TotalTooLarge, walls stopped");
    check(s.walls[0].fixes == 2, "  on the second fix");
    b.run(4000);
    s = b.status();
    checkNear(s.appliedXIn, 0, 0, "  everything fed in is fed back out, to exactly 0");
    checkNear(b.appliedX, 0, 1e-5, "  the pose is odometry alone");
    const std::uint32_t samples = s.walls[0].samples;
    b.trueX += 1.2;
    b.run(2000);
    checkCount(b.status().walls[0].samples, samples, "  and no wall reading is looked at again");
    // switching on again starts everything again, the walls included: faults latch only until then
    b.start(Mode::On);
    s = b.status();
    check(s.fault == Fault::None && !s.wallsStopped && !s.unwinding && s.walls[0].state == WallSensorState::WaitingForStart,
          "switched on again: no fault, the walls not stopped, the sensor waiting for its start check");
    b.run(1000);
    s = b.status();
    // (its start check then faults it: odometry alone is now 3.6 in from the truth, over wallStartTolIn 3)
    check(s.walls[0].samples == 8 && s.walls[0].state == WallSensorState::StartFault,
          "  and its readings are looked at again: its start check runs again (8 readings)");

    // stage 1, then stage 2: the GPS faults, the walls carry on, then their own total goes over
    {
        Config c2 = quietConfig();
        c2.maxTotalCorrectionIn = 2;
        c2.kDist = 0.2f;
        WallBot g = facingPlusX(w, c2, 0, 50.25, -30);
        g.gpsPlugged = true;
        g.gpsDx = 10;
        g.start(Mode::On);
        for (int i = 0; i < 500 && g.status().fault == Fault::None; i++) g.tick();
        g.run(1000);
        Status a = g.status();
        check(a.fault == Fault::StartDisagree && !a.wallsWaiting && !a.wallsStopped, "(GPS out, walls carrying on)");
        for (int stop = 0; stop < 4 && !g.status().wallsStopped; stop++) {
            g.trueX += 1.2;
            g.run(800);
            for (int i = 0; i < 350; i++) {
                g.trueY += 0.02;
                g.odomY += 0.02f;
                g.tick();
            }
        }
        a = g.status();
        check(a.wallsStopped && a.fault == Fault::StartDisagree, "then the walls' total: walls stopped (fault shown: the first)");
        g.run(4000);
        checkNear(g.status().appliedXIn, 0, 0, "  their corrections fed back out");
    }
    // a GPS that is plugged in but never worked (reads infinity): the walls' total is stage 2 at once
    {
        WallBot u = facingPlusX(w, c, 0, 50.25, -30);
        u.gpsPlugged = true;
        u.gpsInfinity = true;
        u.start(Mode::On);
        runUntilStarted(u);
        for (int stop = 0; stop < 4 && u.status().fault == Fault::None; stop++) {
            u.trueX += 1.2;
            u.run(800);
            for (int i = 0; i < 350; i++) {
                u.trueY += 0.02;
                u.odomY += 0.02f;
                u.tick();
            }
        }
        const Status a = u.status();
        check(a.fault == Fault::TotalTooLarge && a.wallsStopped,
              "a GPS that never passed its start check: the walls' total stops the walls (no second budget)");
    }
}

// ------------------------------------------------------------------------------------------------ modes

static void testShadowAndWallOnly() {
    std::printf("Shadow and wall-only: no GPS at all, On and Shadow\n");
    // the same drifting run On and Shadow, walls only: Shadow returns zero every tick and its would-be total
    // equals On's applied total every tick
    WallConfig w;
    w.trust[WALL_PLUS_X] = WallTrust::Solid;
    Config c = quietConfig();
    c.kDist = 0.1f; // the doubt grows while it slides between stops
    WallBot on = facingPlusX(w, c, 0, 50.25, -30);
    WallBot sh = facingPlusX(w, c, 0, 50.25, -30);
    on.start(Mode::On);
    sh.start(Mode::Shadow);
    bool zero = true;
    double worstGap = 0;
    bool sameCounts = true;
    for (int stop = 0; stop < 5; stop++) {
        for (WallBot* r : {&on, &sh}) r->trueX += 0.8;
        for (int i = 0; i < 80; i++) {
            on.tick();
            const Correction c = sh.tick();
            zero = zero && c.xIn == 0.0f && c.yIn == 0.0f;
            const Status a = on.status(), s = sh.status();
            worstGap = std::fmax(worstGap, std::hypot(a.appliedXIn - s.shadowXIn, a.appliedYIn - s.shadowYIn));
            sameCounts = sameCounts && a.walls[0].fixes == s.walls[0].fixes && a.walls[0].samples == s.walls[0].samples;
        }
        for (int i = 0; i < 350; i++) {
            for (WallBot* r : {&on, &sh}) {
                r->trueY += 0.02;
                r->odomY += 0.02f;
            }
            on.tick();
            const Correction c = sh.tick();
            zero = zero && c.xIn == 0.0f && c.yIn == 0.0f;
            const Status a = on.status(), s = sh.status();
            worstGap = std::fmax(worstGap, std::hypot(a.appliedXIn - s.shadowXIn, a.appliedYIn - s.shadowYIn));
            sameCounts = sameCounts && a.walls[0].fixes == s.walls[0].fixes;
        }
    }
    check(on.fixes() >= 4, "(On fixed at every stop)");
    check(on.status().fault == Fault::None && !on.status().startCheckPassed, "wall-only: no fault, no GPS start check");
    check(zero && sh.appliedX == 0 && sh.appliedY == 0, "Shadow: zero on every tick, the pose is odometry alone");
    checkNear(worstGap, 0, 1e-4, "every tick: Shadow's would-have-applied = On's applied");
    check(sameCounts, "every tick: the same fixes and readings");
    checkNear(on.odomX - on.trueX, 0, 0.6, "On: the pose followed the real robot (within 0.6 in after 4 in of slips)");
    std::printf("    (slips 4.0 in; On ends %.2f in off in x, Shadow's odometry %.2f in off)\n", on.odomX - on.trueX,
                sh.odomX - sh.trueX);
}

static void testSamplingRule() {
    std::printf("sampling: each sensor at most once every 35 ms\n");
    WallBot b = facingPlusX();
    b.start(Mode::On);
    b.run(1000); // ticks at +10 .. +1000: samples at +10, 50, 90, ..., 970
    checkCount(b.status().walls[0].samples, 25, "1 s of 10 ms ticks: 25 readings (every 40 ms)");
    WallConfig w;
    w.sampleMinPeriodMs = 10;
    WallBot e = facingPlusX(w);
    e.start(Mode::On);
    e.run(1000);
    checkCount(e.status().walls[0].samples, 100, "sampleMinPeriodMs 10: every tick");
    WallBot g = facingPlusX();
    g.start(Mode::On);
    g.feedWalls = false;
    g.run(1000);
    checkCount(g.status().walls[0].samples, 0, "no readings passed in: none taken");
    // two sensors configured but a reading passed for one only: the second takes none (and nothing past the end
    // of what was passed is read)
    {
        const WallSensorMount pair[2] = {centreForward(), centreForward()};
        GpsFusionCore core(quietConfig());
        core.setWallConfig(WallConfig(), pair, 2);
        core.setMode(Mode::On);
        DistanceSample one[1];
        one[0].mm = 500;
        one[0].confidence = 63;
        for (int i = 1; i <= 100; i++) core.tick(1000 + 10 * i, 50.25f, 0, 1.5707963f, nullptr, one, 1);
        const Status s = core.getStatus();
        check(s.walls[0].samples == 25 && s.walls[1].samples == 0,
              "readings passed for 1 of 2 sensors: the first takes 25 in 1 s, the second none");
    }
}

static void testNoWallSensor() {
    std::printf("no wall sensor: wall readings are ignored, the filter is the GPS-only one\n");
    // two filters, one ticked with the 5-argument tick and one with readings it has no sensor for: identical
    GpsFusionCore a, b;
    WallBot r(Config(), WallConfig(), {}, 10, -20, 30);
    a.setMode(Mode::On);
    b.setMode(Mode::On);
    r.gpsPlugged = true;
    DistanceSample junk[2];
    junk[0].mm = 500;
    junk[0].confidence = 63;
    bool same = true;
    float x = 10, y = -20;
    for (int i = 0; i < 3000; i++) {
        r.t += 10;
        if (r.t % 40 == 0) r.updateGps();
        if (i == 1500) r.gpsDx = 4;
        x += (i / 300) % 2 ? 0.0f : 0.03f;
        const Correction ca = a.tick(r.t, x, y, 0.5f, &r.gps);
        const Correction cb = b.tick(r.t, x, y, 0.5f, &r.gps, junk, 2);
        same = same && ca.xIn == cb.xIn && ca.yIn == cb.yIn;
    }
    const Status sa = a.getStatus(), sb = b.getStatus();
    same = same && sa.sigmaIn == sb.sigmaIn && sa.accepted == sb.accepted && sa.decidedXIn == sb.decidedXIn &&
           sa.sigmaXIn == sa.sigmaYIn && sb.walls[0].samples == 0 && sb.wallSensorCount == 0;
    check(same, "readings for sensors that are not configured change nothing, and the two doubts stay equal");
    check(sa.accepted > 10, "(that run used the GPS)");
    // setWallConfig with more than MAX_WALL_SENSORS, or none
    GpsFusionCore c;
    WallSensorMount mounts[3];
    c.setWallConfig(WallConfig(), mounts, 3);
    check(c.getStatus().wallSensorCount == MAX_WALL_SENSORS, "3 sensors asked for: 2 configured");
    check(c.getStatus().walls[1].state == WallSensorState::WaitingForStart, "  waiting for their start checks");
    c.setWallConfig(WallConfig(), nullptr, 2);
    check(c.getStatus().wallSensorCount == 0 && c.getStatus().walls[0].state == WallSensorState::Unused,
          "no mounts: none configured");
}

static void testWallLine() {
    std::printf("the [WALLFIX] line\n");
    Status s;
    s.wallSensorCount = 2;
    s.sigmaXIn = 0.31f;
    s.sigmaYIn = 1.25f;
    s.walls[0].state = WallSensorState::Working;
    s.walls[0].fixes = 14;
    const std::uint32_t d0[NUM_WALL_CHECKS] = {3, 12, 1350, 160, 45, 230, 310, 95, 12, 4, 3, 1, 160};
    for (int c = 0; c < NUM_WALL_CHECKS; c++) s.walls[0].droppedByCheck[c] = d0[c];
    s.walls[0].lastInnovationIn = -0.42f;
    s.walls[0].lastCorrectionIn = -0.38f;
    s.walls[1].state = WallSensorState::HealthOff;
    const std::uint32_t d1[NUM_WALL_CHECKS] = {40, 25, 1201, 99, 0, 12, 340, 50, 0, 18, 0, 0, 0};
    for (int c = 0; c < NUM_WALL_CHECKS; c++) s.walls[1].droppedByCheck[c] = d1[c];
    s.walls[1].lastInnovationIn = 3.1f;
    char line[700];
    const int n = formatWallStatusLine(line, sizeof line, 61230, s);
    std::printf("    %s (%d characters)\n", line, n);
    check(n > 0 && n <= 200, "two busy sensors after 60 s: about 200 characters at most");
    const std::string_view text(line);
    check(text.find("[WALLFIX] t=61230 walls=run sig=0.31,1.25") == 0, "  starts with time, state and both doubts");
    check(text.find("| 0 OK fix=14 drop=3/12/1350/160/45/230/310/95/12/4/3/1/160 inn=-0.42 cor=-0.38") !=
              std::string_view::npos,
          "  sensor 0: state, fixes, the 13 drop counters in order, last innovation and correction");
    check(text.find("| 1 HEALTH fix=0 drop=40/25/1201/99/0/12/340/50/0/18/0/0/0 inn=+3.10 cor=+0.00") !=
              std::string_view::npos,
          "  sensor 1");
    s.wallsWaiting = true;
    formatWallStatusLine(line, sizeof line, 1, s);
    check(std::string_view(line).find("walls=wait") != std::string_view::npos, "  walls=wait in stage 1");
    s.wallsStopped = true;
    formatWallStatusLine(line, sizeof line, 1, s);
    check(std::string_view(line).find("walls=stop") != std::string_view::npos, "  walls=stop in stage 2");
    const int small = formatWallStatusLine(line, 40, 61230, s);
    check(small > 39 && std::string_view(line).size() == 39, "  a short buffer is cut, not overrun");
    check(std::strcmp(wallSensorStateName(WallSensorState::StartFault), "START") == 0 &&
              std::strcmp(wallSensorStateName(WallSensorState::WaitingForStart), "WAIT") == 0 &&
              std::strcmp(wallSensorStateName(WallSensorState::Unused), "-") == 0,
          "  state names");
}

int main() {
    testCastToWall();
    testGeometryEveryWallAndYaw();
    testNoiseFormula();
    testCheck1Valid();
    testCheck2Confident();
    testCheck3Still();
    testCheck4Steady();
    testCheck5To8();
    testChecks9To13();
    testGateEdges();
    testGatesAlongAxis();
    testWallTrust();
    testStartCheck();
    testHealthRule();
    testOncePerStop();
    testDoubtPerAxis();
    testGpsPerAxis();
    testFaultStage1();
    testFaultStage2();
    testShadowAndWallOnly();
    testSamplingRule();
    testNoWallSensor();
    testWallLine();
    std::printf("\n%d checks, %d failed: %s\n", checksRun, checksFailed, checksFailed == 0 ? "ALL PASSED" : "FAILED");
    return checksFailed == 0 ? 0 : 1;
}
