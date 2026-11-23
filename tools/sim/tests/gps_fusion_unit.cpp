// Unit tests for the GPS fusion filter (src/lemlib/chassis/gpsFusion.cpp, docs/GPS_FUSION_DESIGN.md).
// They test GpsFusionCore only: the file is compiled with GPSFUSION_CORE_ONLY, which leaves out its PROS half.
// "R1" to "R4" are the sections of the design's Revision 2. Where a revision 1 test needs revision 1's numbers
// (sigma starting at 0, gate 3, sigma capped at 12, no step limit) its Config says so, and the numbers it checks
// are the hand-computed ones from revision 1; the revision 2 rules have their own tests further down.
//
//   make -C tools/sim unit
//
// Prints one line per failed check and a summary; exits 1 if anything failed.

#include <algorithm>
#include <cmath>
#include <cstdio>
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

static double wrap360(double deg) {
    double w = std::fmod(deg, 360.0);
    if (w < 0) w += 360.0;
    return w;
}

// a GPS sample from where the SENSOR is (inches) and the heading it reports (degrees)
static GpsSample sampleAt(double sensorXIn, double sensorYIn, double headingDeg) {
    GpsSample s;
    s.xMetres = sensorXIn / INCHES_PER_METRE;
    s.yMetres = sensorYIn / INCHES_PER_METRE;
    s.headingDeg = wrap360(headingDeg);
    s.errorMetres = 0.02;
    return s;
}

// every field of two statuses is exactly the same
static bool sameStatus(const Status& a, const Status& b) {
    bool same = a.valid == b.valid && a.unwinding == b.unwinding && a.headingSamplesSeen == b.headingSamplesSeen &&
                a.headingSamplesFailed == b.headingSamplesFailed && a.headingJudged == b.headingJudged &&
                a.startSamplesNeeded == b.startSamplesNeeded && a.droppedBySensorError == b.droppedBySensorError &&
                a.maxReadingStepIn == b.maxReadingStepIn && a.innovationCount == b.innovationCount &&
                a.innovationRightAvgIn == b.innovationRightAvgIn &&
                a.innovationForwardAvgIn == b.innovationForwardAvgIn && a.nisCount == b.nisCount &&
                a.nisAvg == b.nisAvg && std::string_view(a.faultName) == b.faultName;
    same = same && a.mode == b.mode && a.fault == b.fault && a.sigmaIn == b.sigmaIn && a.pendingXIn == b.pendingXIn &&
                a.pendingYIn == b.pendingYIn && a.shadowXIn == b.shadowXIn && a.shadowYIn == b.shadowYIn &&
                a.appliedXIn == b.appliedXIn && a.appliedYIn == b.appliedYIn && a.decidedXIn == b.decidedXIn &&
                a.decidedYIn == b.decidedYIn && a.estimateXIn == b.estimateXIn && a.estimateYIn == b.estimateYIn &&
                a.startCheckPassed == b.startCheckPassed && a.startSamplesSeen == b.startSamplesSeen &&
                a.sensorFrozen == b.sensorFrozen && a.newReadings == b.newReadings &&
                a.droppedByGate == b.droppedByGate && a.droppedNoHistory == b.droppedNoHistory &&
                a.accepted == b.accepted && a.relocalisations == b.relocalisations &&
                a.lastInnovationXIn == b.lastInnovationXIn && a.lastInnovationYIn == b.lastInnovationYIn;
    for (int i = 0; i < NUM_CHECKS; i++) same = same && a.droppedByCheck[i] == b.droppedByCheck[i];
    return same;
}

static std::uint32_t totalDropped(const Status& s) {
    std::uint32_t n = s.droppedByGate + s.droppedNoHistory + s.droppedBySensorError;
    for (int i = 0; i < NUM_CHECKS; i++) n += s.droppedByCheck[i];
    return n;
}

// ------------------------------------------------------------------------------------------------ a pretend robot
// Drives the filter the way lemlib::update() does: odometry builds its pose up tick by tick, and in On the
// correction the filter returns is added to that pose. It also knows where the robot really is, and what the
// GPS sensor reports (updated every gpsPeriodMs, showing the truth gpsDelayMs late).
struct TruePose {
        std::uint32_t t;
        double x, y, thetaDeg;
};

struct Robot {
        GpsFusionCore core;
        Config config;
        Mode mode = Mode::Off;
        std::uint32_t t = 1000; // ms

        // where the robot really is
        double trueX = 0, trueY = 0, trueThetaDeg = 0;
        std::vector<TruePose> truthLog;

        // what odometry says (heading always right here; distances times odomDistanceScale)
        float odomX = 0, odomY = 0, odomThetaRad = 0;
        double odomDistanceScale = 1.0;

        // the GPS sensor
        bool gpsPlugged = true; // false: no sensor at all (the filter gets nullptr)
        bool gpsFrozen = false; // true: keeps reporting the same numbers
        int gpsPeriodMs = 40;
        int gpsDelayMs = 60;
        double gpsOffsetXIn = 0, gpsOffsetYIn = 0; // added to what it reports (a bias, or a shove the wheels missed)
        double gpsHeadingWiggleDeg = 0.01; // real readings always wiggle a little; this makes each update new
        // the GPS's field frame against ours: 0 the same, 1 turned 180 degrees, 2 mirrored left-right (x), 3
        // mirrored front-back (y)
        int gpsFrame = 0;
        int gpsUpdates = 0;
        GpsSample gps;

        // the corrections the filter returned
        double appliedXIn = 0, appliedYIn = 0;
        double largestStepIn = 0;
        Correction lastCorrection;

        Robot(const Config& c, double x, double y, double thetaDeg)
            : core(c),
              config(c),
              trueX(x),
              trueY(y),
              trueThetaDeg(thetaDeg),
              odomX(float(x)),
              odomY(float(y)),
              odomThetaRad(float(thetaDeg * PI / 180)) {
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
            const TruePose seen = truthAt(t >= std::uint32_t(gpsDelayMs) ? t - gpsDelayMs : 0);
            const double th = seen.thetaDeg * PI / 180;
            const double sensorX = seen.x + config.mountRightIn * std::cos(th) + config.mountForwardIn * std::sin(th);
            const double sensorY = seen.y - config.mountRightIn * std::sin(th) + config.mountForwardIn * std::cos(th);
            const double wiggle = (gpsUpdates % 2 == 0) ? gpsHeadingWiggleDeg : -gpsHeadingWiggleDeg;
            double x = sensorX + gpsOffsetXIn, y = sensorY + gpsOffsetYIn;
            double heading = seen.thetaDeg + config.mountYawDeg;
            if (gpsFrame == 1) x = -x, y = -y, heading += 180;
            if (gpsFrame == 2) x = -x, heading = -heading;
            if (gpsFrame == 3) y = -y, heading = 180 - heading;
            gps = sampleAt(x, y, heading + wiggle);
            gpsUpdates++;
        }

        // one odometry tick: the robot drives at inPerS and turns at degPerS for dtMs
        Correction tick(int dtMs, double inPerS = 0, double degPerS = 0) {
            t += dtMs;
            const double dtS = dtMs / 1000.0;
            const double midHeading = (trueThetaDeg + degPerS * dtS / 2) * PI / 180;
            trueX += inPerS * dtS * std::sin(midHeading);
            trueY += inPerS * dtS * std::cos(midHeading);
            trueThetaDeg += degPerS * dtS;
            const float odomMid = float(odomThetaRad + degPerS * dtS / 2 * PI / 180);
            odomX += float(inPerS * dtS * odomDistanceScale * std::sin(odomMid));
            odomY += float(inPerS * dtS * odomDistanceScale * std::cos(odomMid));
            odomThetaRad += float(degPerS * dtS * PI / 180);
            truthLog.push_back({t, trueX, trueY, trueThetaDeg});
            if (!gpsFrozen && t % gpsPeriodMs == 0) updateGps();

            const Correction c = core.tick(t, odomX, odomY, odomThetaRad, gpsPlugged ? &gps : nullptr);
            // like lemlib::update(): add the correction to the pose
            odomX += c.xIn;
            odomY += c.yIn;
            appliedXIn += c.xIn;
            appliedYIn += c.yIn;
            largestStepIn = std::fmax(largestStepIn, std::hypot(c.xIn, c.yIn));
            lastCorrection = c;
            return c;
        }

        void run(int ms, double inPerS = 0, double degPerS = 0) {
            for (int done = 0; done < ms; done += 10) tick(10, inPerS, degPerS);
        }

        Status status() const { return core.getStatus(); }

        // how far the pose the motions read is from the truth
        double poseErrorIn() const { return std::hypot(odomX - trueX, odomY - trueY); }
};

// Tick until the filter accepts one more reading; false if that has not happened within maxMs
static bool waitForAccepted(Robot& r, int maxMs = 5000) {
    const std::uint32_t before = r.status().accepted;
    for (int waited = 0; waited < maxMs; waited += 10) {
        r.tick(10);
        if (r.status().accepted != before) return true;
    }
    return false;
}

// A filter switched On with a stationary robot and a perfect GPS, run until the start check has passed
static Robot readyRobot(const Config& c, double x, double y, double thetaDeg) {
    Robot r(c, x, y, thetaDeg);
    r.start(Mode::On);
    r.run(1000);
    return r;
}

// Feeding readings by hand (the robot's GPS must be frozen first). Each one is a new reading, gapMs after the
// previous tick: the GPS says the robot's centre is offset (dx, dy) from where it really is, and its heading is
// headingErrorDeg off. Readings fed less than staleMs apart keep the sensor "alive" for check 3
static int handFed = 0;
static void feed(Robot& r, double dx, double dy, int gapMs = 120, double headingErrorDeg = 0, double errorIn = -1) {
    if (gapMs > 10) r.run(gapMs - 10);
    // never the +-0.01 the pretend sensor uses, and never the same twice in a row, so it is always new
    const double wiggle = 0.02 + 0.001 * (handFed++ % 2);
    r.gps = sampleAt(r.trueX + dx, r.trueY + dy, r.trueThetaDeg + r.config.mountYawDeg + headingErrorDeg + wiggle);
    if (errorIn >= 0) r.gps.errorMetres = errorIn / INCHES_PER_METRE;
    r.tick(10);
}

// Readings that check 5 drops (heading 20 deg off), one every 120 ms for about ms: the sensor stays alive, and
// nothing reaches the gate. Only for after the heading fault has been judged
static void filler(Robot& r, int ms) {
    for (int done = 0; done + 120 <= ms; done += 120) feed(r, 0, 0, 120, 20);
}

static double len(double x, double y) { return std::hypot(x, y); }

// The tests of the total limit and of relocalisation were written for revision 2's limits (total 12 in,
// relocalisation within 12 in, 2 per run) and check hand-worked numbers built on them. Revision 3 lowered the
// defaults (6, 6, 1), so those tests pin revision 2's values here instead of reading the defaults
static Config rev2Limits() {
    Config c;
    c.maxTotalCorrectionIn = 12;
    c.relocMaxIn = 12;
    c.relocMaxCount = 2;
    return c;
}

// ------------------------------------------------------------------------------------------------ tests

static void testOffChangesNothing() {
    std::printf("Off: zero correction, no state change that matters\n");
    // 1. never switched on: every tick returns exactly zero and the status stays that of a new filter
    {
        Robot r(Config(), 10, -20, 30);
        const Status fresh = GpsFusionCore().getStatus();
        bool allZero = true;
        for (int i = 0; i < 500; i++) {
            const Correction c = r.tick(10, 5, 10); // moving, GPS updating
            allZero = allZero && c.xIn == 0.0f && c.yIn == 0.0f;
        }
        r.gpsOffsetXIn = 30; // even a wildly wrong GPS
        r.run(1000);
        allZero = allZero && r.appliedXIn == 0 && r.appliedYIn == 0;
        check(allZero, "Off: every correction is exactly zero");
        check(sameStatus(r.status(), fresh), "Off: the status is exactly that of a filter never used");
    }
    // 2. switched Off after a run: ticks return zero and do not change the status
    {
        Robot r = readyRobot(Config(), 0, 0, 0);
        r.gpsOffsetXIn = 6;
        r.run(4000); // relocalises, feeds corrections in
        r.start(Mode::Off);
        const Status before = r.status();
        const double appliedBefore = r.appliedXIn;
        r.run(3000, 3, 5);
        check(r.appliedXIn == appliedBefore, "switched Off: no more corrections");
        check(sameStatus(r.status(), before), "switched Off: the status does not change");
    }
    // 3. what happened while Off does not leak into the next run: Off ticks, then On, behaves exactly like a new
    //    filter switched On
    {
        Robot a(Config(), 5, 5, 45);
        a.run(3000, 4, 10); // Off
        Robot b(Config(), 5, 5, 45);
        b.t = a.t;
        b.trueX = a.trueX, b.trueY = a.trueY, b.trueThetaDeg = a.trueThetaDeg;
        b.odomX = a.odomX, b.odomY = a.odomY, b.odomThetaRad = a.odomThetaRad;
        b.truthLog = a.truthLog;
        b.gps = a.gps;
        b.gpsUpdates = a.gpsUpdates;
        a.start(Mode::On);
        b.start(Mode::On);
        bool identical = true;
        for (int i = 0; i < 1500; i++) {
            const double v = (i / 300) % 2 ? 0 : 4;
            if (i == 700) a.gpsOffsetXIn = b.gpsOffsetXIn = 5;
            const Correction ca = a.tick(10, v, 0);
            const Correction cb = b.tick(10, v, 0);
            identical = identical && ca.xIn == cb.xIn && ca.yIn == cb.yIn;
        }
        check(identical, "after Off ticks, a run's corrections are bit-identical to a new filter's");
        check(sameStatus(a.status(), b.status()), "after Off ticks, the status is identical to a new filter's");
        check(a.status().accepted > 0 && a.appliedXIn != 0, "(that run did use the GPS)");
    }
}

static void testCheckFinite() {
    std::printf("check 1: finite numbers\n");
    Robot r = readyRobot(Config(), 0, 0, 0);
    r.gpsFrozen = true; // the tests below feed readings by hand
    const Status before = r.status();
    const GpsSample good = sampleAt(0, 0, 180);
    GpsSample bad[4] = {good, good, good, good};
    bad[0].xMetres = INFINITY;
    bad[1].yMetres = -INFINITY;
    bad[2].headingDeg = NAN;
    bad[3].errorMetres = INFINITY; // PROS fills every field with PROS_ERR_F on a failed read
    for (int i = 0; i < 4; i++) {
        r.run(120);
        r.gps = bad[i];
        r.tick(10);
    }
    const Status after = r.status();
    checkCount(after.droppedByCheck[CHECK_FINITE] - before.droppedByCheck[CHECK_FINITE], 4, "check 1 dropped 4");
    checkCount(totalDropped(after) - totalDropped(before), 4, "and nothing else was counted");
    checkCount(after.accepted, before.accepted, "nothing accepted");
    // a failed read that repeats (sensor unplugged: infinity every time) is one reading, not one per tick
    r.gps = sampleAt(0, 0, 180);
    r.gps.xMetres = r.gps.yMetres = r.gps.headingDeg = r.gps.errorMetres = INFINITY;
    r.run(500);
    checkCount(r.status().droppedByCheck[CHECK_FINITE], after.droppedByCheck[CHECK_FINITE] + 1,
               "a repeated infinity is counted once");
}

static void testCheckInField() {
    std::printf("check 2: inside the field\n");
    Robot r = readyRobot(Config(), 0, 0, 0);
    r.gpsFrozen = true;
    const Status before = r.status();
    const double outside[3][2] = {{72.8, 0}, {0, -72.05}, {80, 80}};
    for (const auto& p : outside) {
        r.run(120);
        r.gps = sampleAt(p[0], p[1], 180);
        r.tick(10);
    }
    // 71.9 in is inside: dropped later (by the gate, it is far from odometry), but not by check 2
    r.run(120);
    r.gps = sampleAt(71.9, 0, 180);
    r.tick(10);
    const Status after = r.status();
    checkCount(after.droppedByCheck[CHECK_IN_FIELD] - before.droppedByCheck[CHECK_IN_FIELD], 3, "check 2 dropped 3");
    checkCount(after.droppedByCheck[CHECK_FINITE], before.droppedByCheck[CHECK_FINITE], "check 1 counted none");
    checkCount(after.droppedByGate - before.droppedByGate, 1, "71.9 in got past check 2 (to the gate)");
}

static void testCheckStale() {
    std::printf("check 3: not stale\n");
    // the first reading after switching on is dropped: the sensor has not been seen updating yet
    {
        Robot r(Config(), 0, 0, 0);
        r.start(Mode::On);
        r.tick(10);
        checkCount(r.status().newReadings, 1, "first tick: one new reading");
        checkCount(r.status().droppedByCheck[CHECK_NOT_STALE], 1, "first reading after switching on: check 3");
    }
    // a frozen sensor: repeats are not new (never counted, never used); the first change after a freeze is dropped
    {
        Robot r = readyRobot(Config(), 0, 0, 0);
        r.gpsFrozen = true;
        const Status before = r.status();
        r.run(1000); // the same numbers for a second
        const Status frozen = r.status();
        checkCount(frozen.newReadings, before.newReadings, "frozen: no new readings");
        checkCount(totalDropped(frozen), totalDropped(before), "frozen: nothing counted");
        checkCount(frozen.accepted, before.accepted, "frozen: nothing used");
        check(frozen.sensorFrozen, "status says frozen");
        r.gpsFrozen = false;
        r.run(40); // the next update
        const Status after = r.status();
        checkCount(after.droppedByCheck[CHECK_NOT_STALE] - frozen.droppedByCheck[CHECK_NOT_STALE], 1,
                   "first reading after the freeze: check 3");
        check(!after.sensorFrozen, "status says no longer frozen");
        r.run(1000);
        check(r.status().accepted > after.accepted, "readings are used again after that");
    }
    // a gap of 150 ms (under staleMs) is not a freeze
    {
        Robot r = readyRobot(Config(), 0, 0, 0);
        r.gpsFrozen = true;
        r.run(150);
        const Status before = r.status();
        r.gpsFrozen = false;
        r.run(40);
        checkCount(r.status().droppedByCheck[CHECK_NOT_STALE], before.droppedByCheck[CHECK_NOT_STALE],
                   "150 ms gap: not stale");
    }
}

static void testCheckDeadZone() {
    std::printf("check 4: dead zone\n");
    // odometry says the camera (facing backward) is 17 in from the wall behind it: every reading that gets that
    // far is dropped, so the start check never completes
    {
        Robot r(Config(), 0, -55, 0);
        r.start(Mode::On);
        r.run(3000);
        const Status s = r.status();
        check(s.droppedByCheck[CHECK_DEAD_ZONE] > 10, "facing away from a wall 17 in behind: dropped by check 4");
        checkCount(s.startSamplesSeen, 0, "and the start check got no readings");
        checkCount(s.accepted, 0, "nothing accepted");
    }
    // the robot turned around: the camera now looks at the far wall, 127 in away: fine
    {
        Robot r(Config(), 0, -55, 180);
        r.start(Mode::On);
        r.run(3000);
        checkCount(r.status().droppedByCheck[CHECK_DEAD_ZONE], 0, "facing the wall, camera looking across the field: kept");
        check(r.status().startCheckPassed, "start check passed");
        // but a reading that puts the sensor 12 in from the wall the camera faces is dropped, even though odometry
        // would not have put it there
        r.gpsFrozen = true;
        r.run(120);
        const Status before = r.status();
        r.gps = sampleAt(0, 60, 0);
        r.tick(10);
        checkCount(r.status().droppedByCheck[CHECK_DEAD_ZONE] - before.droppedByCheck[CHECK_DEAD_ZONE], 1,
                   "GPS says 12 in from the wall it faces: check 4");
    }
    // odometry says the camera is 17 in from the wall behind; the GPS says 32 in. Dropped: the reading could be
    // the wrong one, so either view in the dead zone is enough
    {
        Robot r(Config(), 0, -55, 0);
        r.trueY = -40;
        r.truthLog.push_back({r.t, r.trueX, r.trueY, r.trueThetaDeg});
        r.updateGps();
        r.start(Mode::On);
        r.run(3000);
        check(r.status().droppedByCheck[CHECK_DEAD_ZONE] > 10, "dead zone from odometry's position only: check 4");
        checkCount(r.status().startSamplesSeen, 0, "  and nothing reaches the start check");
    }
    // a sensor mounted 10 in forward of the centre: what counts is where the camera is
    {
        Config c;
        c.mountForwardIn = 10;
        Robot r(c, 0, -45, 0); // camera faces backward from 10 in in front: 37 in to the wall. Fine
        r.start(Mode::On);
        r.run(2000);
        checkCount(r.status().droppedByCheck[CHECK_DEAD_ZONE], 0, "mount 10 in forward, 37 in to the wall: kept");
        Config c2;
        c2.mountForwardIn = -10;
        Robot r2(c2, 0, -45, 0); // camera 10 in behind the centre: 17 in to the wall. Dead zone
        r2.start(Mode::On);
        r2.run(2000);
        check(r2.status().droppedByCheck[CHECK_DEAD_ZONE] > 5, "mount 10 in back, 17 in to the wall: check 4");
    }
}

static void testCheckHeading() {
    std::printf("check 5: heading agrees\n");
    Robot r = readyRobot(Config(), 0, 0, 0);
    r.gpsFrozen = true;
    const Status before = r.status();
    r.run(120);
    r.gps = sampleAt(0, 0, 180 + 10); // 10 degrees off
    r.tick(10);
    r.run(120);
    r.gps = sampleAt(0, 0, 180 - 9);
    r.tick(10);
    const Status mid = r.status();
    checkCount(mid.droppedByCheck[CHECK_HEADING] - before.droppedByCheck[CHECK_HEADING], 2, "10 and 9 deg off: check 5");
    r.run(120);
    r.gps = sampleAt(0, 0, 180 + 7); // within 8
    r.tick(10);
    const Status after = r.status();
    checkCount(after.droppedByCheck[CHECK_HEADING], mid.droppedByCheck[CHECK_HEADING], "7 deg off: kept");
    checkCount(after.accepted - mid.accepted, 1, "7 deg off: accepted");
    // a GPS mounted facing forward, reporting the robot's own heading, fails every time with mountYaw 180
    {
        Robot f(Config(), 0, 0, 30);
        f.config.mountYawDeg = 0; // what the pretend sensor really does; the filter still thinks 180
        f.start(Mode::On);
        f.run(2000);
        check(f.status().droppedByCheck[CHECK_HEADING] > 5, "sensor mounted the other way: check 5 drops it");
        checkCount(f.status().startSamplesSeen, 0, "and nothing reaches the start check");
        check(f.status().fault == Fault::HeadingDisagree, "  and (R3) that is the heading fault, no longer silent");
    }
    // wrapping round 0/360: our heading 355, GPS 175 + 5 = 180 (10 off: dropped); GPS 179 (4 off: kept)
    {
        Robot w = readyRobot(Config(), 0, 0, 355);
        w.gpsFrozen = true;
        const Status b = w.status();
        w.run(120);
        w.gps = sampleAt(0, 0, 175 + 10);
        w.tick(10);
        w.run(120);
        w.gps = sampleAt(0, 0, 179);
        w.tick(10);
        const Status a = w.status();
        checkCount(a.droppedByCheck[CHECK_HEADING] - b.droppedByCheck[CHECK_HEADING], 1, "across 0/360: 10 off dropped");
        checkCount(a.accepted - b.accepted, 1, "across 0/360: 4 off accepted");
    }
}

static void testCheckSlow() {
    std::printf("check 6: slow enough\n");
    struct Case {
            double inPerS, degPerS;
            bool dropped;
            const char* what;
    } cases[] = {
        {10, 0, true, "10 in/s: check 6"},  {6.5, 0, true, "6.5 in/s: check 6"}, {4, 0, false, "4 in/s: kept"},
        {0, 40, true, "40 deg/s: check 6"}, {0, 20, false, "20 deg/s: kept"},
    };
    for (const Case& k : cases) {
        Robot r = readyRobot(Config(), 0, 0, 90); // facing +x; the camera looks at the far -x wall
        r.gpsFrozen = true;
        r.run(60);
        r.run(60, k.inPerS, k.degPerS); // moving for the last 60 ms
        const Status before = r.status();
        // a reading that agrees with odometry exactly (so check 5 and the gate are not the reason)
        const double headingNow = r.odomThetaRad * 180 / PI;
        r.gps = sampleAt(r.odomX, r.odomY, headingNow + 180);
        r.tick(10, k.inPerS, k.degPerS);
        const Status after = r.status();
        checkCount(after.droppedByCheck[CHECK_SLOW] - before.droppedByCheck[CHECK_SLOW], k.dropped ? 1 : 0, k.what);
        checkCount(totalDropped(after) - totalDropped(before) + after.accepted - before.accepted, 1,
                   "  (the reading was counted exactly once)");
    }
    // a correction being fed in (2 in/s) is not the robot moving: 5 in/s of driving plus 2 in/s of correction in
    // the same direction is still 5 in/s
    {
        Config c;
        c.kDist = 1;
        Robot r = readyRobot(c, -40, 0, 90);
        r.gpsPlugged = false;
        r.run(1000, 20, 0); // sigma up to 12
        r.run(300);
        r.gpsOffsetXIn = 5;
        r.gpsPlugged = true;
        check(waitForAccepted(r), "(a 5 in correction was decided)");
        const Status before = r.status();
        r.run(2000, 5, 0); // driving +x at 5 in/s while the correction feeds in +x
        const Status after = r.status();
        check(after.appliedXIn - before.appliedXIn > 3, "(it was feeding in while the robot drove)");
        checkCount(after.droppedByCheck[CHECK_SLOW], before.droppedByCheck[CHECK_SLOW],
                   "5 in/s plus a correction feeding in: not dropped by check 6");
    }
    // until the history is 50 ms long, the speed is unknown: dropped
    {
        Robot r(Config(), 0, 0, 0);
        r.start(Mode::On);
        r.run(40); // readings at +10 (first: stale) and +40 (history 30 ms)
        checkCount(r.status().droppedByCheck[CHECK_SLOW], 1, "history under 50 ms: check 6");
    }
}

static void testCheckRate() {
    std::printf("check 7: one reading per minPeriodMs\n");
    Robot r = readyRobot(Config(), 0, 0, 0);
    r.gpsFrozen = true;
    r.run(120);
    const Status s0 = r.status();
    r.gps = sampleAt(0, 0, 180.02);
    r.tick(10); // used
    r.run(30);
    r.gps = sampleAt(0, 0, 180.03); // 40 ms later
    r.tick(10);
    r.run(50);
    r.gps = sampleAt(0, 0, 180.04); // 100 ms after the used one
    r.tick(10);
    const Status s1 = r.status();
    checkCount(s1.droppedByCheck[CHECK_RATE] - s0.droppedByCheck[CHECK_RATE], 1, "40 ms after a used reading: check 7");
    checkCount(s1.accepted - s0.accepted, 2, "the first and the one 100 ms later: accepted");
    // with a sensor updating every 40 ms, readings are used every 120 ms: 2 of every 3 are dropped by check 7
    r.gpsFrozen = false;
    r.run(500);
    std::vector<std::uint32_t> usedAt;
    std::uint32_t rateDropsAtFirst = 0;
    for (int ticks = 0; usedAt.size() < 31 && ticks < 2000; ticks++) {
        const std::uint32_t before = r.status().accepted;
        r.tick(10);
        if (r.status().accepted != before) {
            usedAt.push_back(r.t);
            if (usedAt.size() == 1) rateDropsAtFirst = r.status().droppedByCheck[CHECK_RATE];
        }
    }
    bool every120 = true;
    for (size_t i = 1; i < usedAt.size(); i++) every120 = every120 && usedAt[i] - usedAt[i - 1] == 120;
    check(every120, "25 Hz sensor: one reading used every 120 ms, never closer than 100");
    checkCount(r.status().droppedByCheck[CHECK_RATE] - rateDropsAtFirst, 60,
               "and the 2 in between each pair dropped by check 7 (60 for 31 used)");
}

static void testCheckOrder() {
    std::printf("checks run in order: a reading is counted once, at the first check it fails\n");
    Robot r = readyRobot(Config(), 0, 0, 0);
    r.gpsFrozen = true;
    const Status before = r.status();
    r.run(120);
    GpsSample s = sampleAt(80, 0, 90); // outside the field, heading wrong, and...
    s.errorMetres = INFINITY; // ...not finite
    r.gps = s;
    r.tick(10);
    r.run(120);
    r.gps = sampleAt(0, -65, 90); // inside, but 7 in from the wall the camera faces, and the heading is wrong
    r.tick(10);
    const Status after = r.status();
    checkCount(after.droppedByCheck[CHECK_FINITE] - before.droppedByCheck[CHECK_FINITE], 1, "counted at check 1");
    checkCount(after.droppedByCheck[CHECK_IN_FIELD], before.droppedByCheck[CHECK_IN_FIELD], "not also at check 2");
    checkCount(after.droppedByCheck[CHECK_DEAD_ZONE] - before.droppedByCheck[CHECK_DEAD_ZONE], 1, "counted at check 4");
    checkCount(after.droppedByCheck[CHECK_HEADING], before.droppedByCheck[CHECK_HEADING], "not also at check 5");
}

static void testRayCast() {
    std::printf("dead-zone ray cast\n");
    const double tol = 1e-3;
    const float d = float(PI / 180);
    // from the centre, every wall is 72 in away
    checkNear(distanceToWallIn(0, 0, 0 * d), 72, tol, "centre, facing 0 (+y wall)");
    checkNear(distanceToWallIn(0, 0, 90 * d), 72, tol, "centre, facing 90 (+x wall)");
    checkNear(distanceToWallIn(0, 0, 180 * d), 72, tol, "centre, facing 180 (-y wall)");
    checkNear(distanceToWallIn(0, 0, 270 * d), 72, tol, "centre, facing 270 (-x wall)");
    checkNear(distanceToWallIn(0, 0, -90 * d), 72, tol, "centre, facing -90 (-x wall)");
    // near each wall, facing it
    checkNear(distanceToWallIn(0, 60, 0 * d), 12, tol, "12 in from the +y wall");
    checkNear(distanceToWallIn(60, 0, 90 * d), 12, tol, "12 in from the +x wall");
    checkNear(distanceToWallIn(0, -60, 180 * d), 12, tol, "12 in from the -y wall");
    checkNear(distanceToWallIn(-60, 0, 270 * d), 12, tol, "12 in from the -x wall");
    // near a wall but facing away: the far wall
    checkNear(distanceToWallIn(0, 60, 180 * d), 132, tol, "12 in from +y, facing -y");
    // off-centre and slanted: from (10, 20) at 30 deg, the +y wall is (72-20)/cos30 = 60.04 in, the +x wall
    // (72-10)/sin30 = 124 in: the +y wall first
    checkNear(distanceToWallIn(10, 20, 30 * d), 52 / std::cos(PI / 6), tol, "slanted, hits +y");
    checkNear(distanceToWallIn(10, 20, 80 * d), 62 / std::sin(80 * PI / 180), tol, "slanted, hits +x");
    // corners: straight at a corner, both walls at once
    checkNear(distanceToWallIn(60, 60, 45 * d), 12 * std::sqrt(2.0), tol, "at the +x+y corner");
    checkNear(distanceToWallIn(-60, 60, -45 * d), 12 * std::sqrt(2.0), tol, "at the -x+y corner");
    checkNear(distanceToWallIn(-60, -60, 225 * d), 12 * std::sqrt(2.0), tol, "at the -x-y corner");
    checkNear(distanceToWallIn(60, -60, 135 * d), 12 * std::sqrt(2.0), tol, "at the +x-y corner");
    checkNear(distanceToWallIn(0, 0, 45 * d), 72 * std::sqrt(2.0), tol, "centre to a corner");
    // in a corner, facing along a wall: the other wall 132 in away
    checkNear(distanceToWallIn(60, 60, 270 * d), 132, tol, "in a corner, facing along the +y wall");
    // just off the corner diagonal: the nearer wall wins
    checkNear(distanceToWallIn(60, 50, 45 * d), 12 * std::sqrt(2.0), tol, "beside the diagonal: +x wall first");
    // on or outside the wall
    checkNear(distanceToWallIn(72, 0, 270 * d), 0, 0, "on the wall");
    checkNear(distanceToWallIn(0, -80, 0), 0, 0, "outside the field");
    checkNear(distanceToWallIn(NAN, 0, 0), 0, 0, "NaN position");
}

static void testDelay() {
    std::printf("delay: constant speed, delayed perfect GPS -> zero innovation\n");
    Config c;
    c.mountRightIn = 2; // and a lever arm, so it is removed with the heading of the right moment
    c.mountForwardIn = -5;
    struct Case {
            int trueDelayMs;
            float configuredMs;
            double inPerS;
            double degPerS;
            double wantInnovation; // size, inches
            const char* what;
    } cases[] = {
        {60, 60, 4, 0, 0, "60 ms delay, 60 configured, 4 in/s"},
        {100, 100, 5, 0, 0, "100 ms delay, 100 configured, 5 in/s"},
        {60, 60, 0, 25, 0, "60 ms delay, turning at 25 deg/s (lever arm swings)"},
        {60, 0, 4, 0, 4 * 0.06, "60 ms delay, 0 configured: off by speed x delay"},
    };
    for (const Case& k : cases) {
        Config ck = c;
        ck.latencyMs = k.configuredMs;
        Robot r(ck, -30, 10, 90); // facing +x, camera looking at the -x wall
        r.gpsDelayMs = k.trueDelayMs;
        r.gpsHeadingWiggleDeg = 0; // the robot moves, so every update is new anyway
        r.start(Mode::On);
        double largest = 0;
        double first = -1;
        std::uint32_t seen = 0;
        for (int i = 0; i < 500; i++) {
            const Status before = r.status();
            r.tick(10, k.inPerS, k.degPerS);
            const Status after = r.status();
            const std::uint32_t reached = after.startSamplesSeen + after.accepted + after.droppedByGate;
            if (reached != std::uint32_t(before.startSamplesSeen) + before.accepted + before.droppedByGate) {
                const double size = std::hypot(after.lastInnovationXIn, after.lastInnovationYIn);
                largest = std::fmax(largest, size);
                if (first < 0) first = size;
                seen++;
            }
        }
        char what[160];
        std::snprintf(what, sizeof what, "%s (%u readings)", k.what, unsigned(seen));
        check(seen >= 30, what);
        if (k.wantInnovation == 0) {
            checkNear(largest, 0, 2e-3, what);
            checkNear(std::hypot(r.status().decidedXIn, r.status().decidedYIn), 0, 2e-3, "  and nothing decided");
        } else {
            // the first reading, before the filter has corrected anything (later ones shrink as it "corrects"
            // odometry toward the late GPS: what a wrong latencyMs does)
            checkNear(first, k.wantInnovation, 2e-3, what);
        }
    }
}

static void testGainArithmetic() {
    std::printf("gain and sigma arithmetic against hand-computed numbers\n");
    // Revision 1's numbers: sigma 0 when the start check passes (R3 made it startSigmaIn; tested separately).
    // Nothing below reaches the step limit (1 in) or the cap (5 in)
    Config c;
    c.startSigmaIn = 0;
    // stationary at the centre: the start check passes with a perfect GPS; nothing moves, so sigma stays 0
    Robot r = readyRobot(c, 0, 0, 0);
    checkNear(r.status().sigmaIn, 0, 0, "sigma starts at 0");
    check(r.status().startCheckPassed, "start check passed");
    // drive 50 in and turn 90 deg with the sensor unplugged: sigma = 0.02 * 50 + 0.3 * pi/2 = 1.4712389
    r.gpsPlugged = false;
    r.run(2000, 25, 0);
    checkNear(r.status().sigmaIn, 1.0, 1e-4, "after 50 in: sigma = 0.02 * 50 = 1.0");
    r.run(1000, 0, 90);
    checkNear(r.status().sigmaIn, 1.471238898, 1e-4, "after 90 deg more: sigma = 1.0 + 0.3 * pi/2");
    // plug it back in, reading 2 in to the right (+x) of the truth
    r.run(300);
    r.gpsOffsetXIn = 2;
    r.gpsPlugged = true;
    check(waitForAccepted(r), "the offset reading was accepted");
    // K = 1.4712389^2 / (1.4712389^2 + 1.5^2) = 0.490321; pending = 2 K = 0.980642; sigma = sqrt(1-K) 1.4712389 = 1.050344
    const Status s1 = r.status();
    checkNear(s1.lastInnovationXIn, 2, 1e-4, "innovation x = 2");
    checkNear(s1.lastInnovationYIn, 0, 1e-4, "innovation y = 0");
    checkNear(s1.decidedXIn, 0.9806421440, 1e-4, "decided = K * 2 = 0.980642");
    checkNear(s1.pendingXIn + s1.appliedXIn, 0.9806421440, 1e-4, "pending + applied = decided");
    checkNear(s1.sigmaIn, 1.0503439494, 1e-4, "sigma = sqrt(1 - K) * sigma = 1.050344");
    // the next reading: the GPS still says 2, we now believe 0.980642, so the innovation is 1.019358, whatever
    // part of it has been fed in meanwhile. K2 = 1.050344^2 / (1.050344^2 + 2.25) = 0.329004
    check(waitForAccepted(r), "the next reading was accepted");
    const Status s2 = r.status();
    checkNear(s2.lastInnovationXIn, 1.0193578560, 1e-4, "second innovation = 2 - 0.980642 = 1.019358");
    checkNear(s2.decidedXIn, 1.3160145991, 1e-4, "decided = 0.980642 + K2 * 1.019358 = 1.316015");
    checkNear(s2.sigmaIn, 0.8603825963, 1e-4, "sigma = sqrt(1 - K2) * 1.050344 = 0.860383");
    check(s2.appliedXIn > 0.1 && s2.pendingXIn > 0.1, "(part fed in, part still pending)");
}

static void testGate() {
    std::printf("statistical gate: |innovation|^2 > gate^2 * (sigma^2 + sigmaGps^2) is dropped\n");
    // Revision 1's numbers: gate 3 and sigma 0 after the start check (revision 2's gate is below)
    Config v1;
    v1.gate = 3;
    v1.startSigmaIn = 0;
    v1.relocSamples = MAX_RELOC_SAMPLES; // no relocalisation in the way
    // sigma 0 (nothing has moved), sigmaGps 1.5, gate 3: the limit is 3 * 1.5 = 4.5 in
    Robot r = readyRobot(v1, 0, 0, 0);
    r.gpsFrozen = true;
    const double offsets[] = {4.4, 4.6, -4.4, 3.0};
    const bool dropped[] = {false, true, false, false};
    for (int i = 0; i < 4; i++) {
        r.run(120);
        const Status before = r.status();
        r.gps = sampleAt(0, offsets[i], 180 + 0.001 * i);
        r.tick(10);
        const Status after = r.status();
        char what[80];
        std::snprintf(what, sizeof what, "%.1f in off with sigma 0: %s", offsets[i], dropped[i] ? "dropped" : "kept");
        checkCount(after.droppedByGate - before.droppedByGate, dropped[i] ? 1 : 0, what);
        checkCount(after.accepted - before.accepted, dropped[i] ? 0 : 1, what);
    }
    checkNear(r.status().decidedYIn, 0, 0, "kept with sigma 0: K = 0, nothing decided");
    // with sigma 2 the limit is 3 * sqrt(4 + 2.25) = 7.5 in
    Config c = v1;
    c.kDist = 0.1;
    Robot w = readyRobot(c, 0, 0, 0);
    w.gpsPlugged = false;
    w.run(1000, 20, 0); // 20 in: sigma 2
    w.run(300);
    checkNear(w.status().sigmaIn, 2, 1e-4, "sigma 2 after 20 in at kDist 0.1");
    w.gpsOffsetXIn = 7.6;
    w.gpsPlugged = true;
    const std::uint32_t acceptedBefore = w.status().accepted;
    w.run(400);
    check(w.status().droppedByGate >= 1, "7.6 in off with sigma 2: dropped by the gate");
    checkCount(w.status().accepted, acceptedBefore, "  and none accepted");
    w.gpsOffsetXIn = 7.4;
    check(waitForAccepted(w, 1000), "7.4 in off with sigma 2: kept");
    check(w.status().accepted == acceptedBefore + 1, "  (exactly one more accepted)");
    // R1, revision 2's gate of 2.5: right after the start check sigma is startSigmaIn = 1, so the limit is
    // 2.5 * sqrt(1 + 2.25) = 4.5069 in
    for (const double off : {4.45, 4.56}) {
        Robot g(Config(), 0, 0, 0);
        g.start(Mode::On);
        for (int i = 0; i < 300 && !g.status().startCheckPassed; i++) g.tick(10);
        checkNear(g.status().sigmaIn, 1, 0, "(start check passed: sigma = startSigmaIn = 1)");
        g.gpsFrozen = true;
        const Status before = g.status();
        feed(g, 0, off);
        const bool shouldDrop = off > 4.5069;
        char what[96];
        std::snprintf(what, sizeof what, "gate 2.5, sigma 1: %.2f in off is %s", off, shouldDrop ? "dropped" : "kept");
        checkCount(g.status().droppedByGate - before.droppedByGate, shouldDrop ? 1 : 0, what);
        checkCount(g.status().accepted - before.accepted, shouldDrop ? 0 : 1, what);
    }
}

static void testFeedIn() {
    std::printf("feed-in: at most slewInPerS * dt per tick; everything decided is fed in\n");
    // The feed-in did not change in revision 2. This keeps revision 1's numbers: sigma capped at 12 and no step
    // limit (R1's step limit and cap have their own tests)
    Config c;
    c.sigmaMaxIn = 12;
    c.maxStepIn = 100;
    c.kDist = 1; // a lot of doubt quickly: 20 in of driving caps sigma at 12
    Robot r = readyRobot(c, 0, 0, 0);
    r.gpsPlugged = false;
    r.run(1000, 20, 0);
    checkNear(r.status().sigmaIn, 12, 1e-5, "sigma capped at sigmaMax");
    r.run(300);
    r.gpsOffsetXIn = 3;
    r.gpsOffsetYIn = 4;
    r.gpsPlugged = true;
    check(waitForAccepted(r), "the offset reading was accepted");
    r.gpsPlugged = false; // one correction only, so the totals are easy to follow
    const Status s1 = r.status();
    // K = 144 / 146.25 = 0.984615: decided (2.953846, 3.938462), 4.923077 in long
    checkNear(s1.decidedXIn, 2.953846154, 1e-4, "decided x = 3 K");
    checkNear(s1.decidedYIn, 3.938461538, 1e-4, "decided y = 4 K");
    double appliedX = r.appliedXIn, appliedY = r.appliedYIn;
    bool withinLimit = true;
    bool totalsAddUp = true;
    double worst = 0;
    // 10 ms ticks, then uneven ones, then a stalled odometry task (500 ms late), then 10 ms again
    const int dts[] = {10, 10, 10, 7, 13, 10, 500, 10, 3, 10};
    for (int i = 0; i < 600; i++) {
        const int dt = dts[i % 10];
        const Correction step = r.tick(dt);
        const double limit = c.slewInPerS * std::fmin(dt, 20) / 1000.0; // a late tick feeds 20 ms' worth at most
        const double size = std::hypot(step.xIn, step.yIn);
        worst = std::fmax(worst, size / limit);
        if (size > limit * (1 + 1e-5)) withinLimit = false;
        appliedX += step.xIn;
        appliedY += step.yIn;
        const Status s = r.status();
        if (std::fabs(s.appliedXIn + s.pendingXIn - s.decidedXIn) > 1e-5 ||
            std::fabs(s.appliedYIn + s.pendingYIn - s.decidedYIn) > 1e-5)
            totalsAddUp = false;
    }
    check(withinLimit, "no tick moved more than slewInPerS * dt (a late tick: 20 ms' worth)");
    check(totalsAddUp, "every tick: applied + pending = decided");
    std::printf("    (largest step was %.3f of the limit)\n", worst);
    const Status s2 = r.status();
    checkNear(s2.pendingXIn, 0, 0, "pending x drained to exactly 0");
    checkNear(s2.pendingYIn, 0, 0, "pending y drained to exactly 0");
    checkNear(appliedX, s2.decidedXIn, 1e-5, "total applied x = total decided x");
    checkNear(appliedY, s2.decidedYIn, 1e-5, "total applied y = total decided y");
    checkNear(s2.appliedXIn, appliedX, 1e-5, "status applied x = what the ticks returned");
    // 4.923 in at 2 in/s, 10 ms ticks: 247 ticks. Never a jump
    Robot q = readyRobot(c, 0, 0, 0);
    q.gpsPlugged = false;
    q.run(1000, 20, 0);
    q.run(300);
    q.gpsOffsetXIn = 3;
    q.gpsOffsetYIn = 4;
    q.gpsPlugged = true;
    check(waitForAccepted(q), "the offset reading was accepted");
    q.gpsPlugged = false;
    int ticksToDrain = 0;
    while (q.status().pendingXIn != 0 || q.status().pendingYIn != 0) {
        q.tick(10);
        ticksToDrain++;
        if (ticksToDrain > 10000) break;
    }
    check(ticksToDrain >= 245 && ticksToDrain <= 248, "4.92 in at 2 in/s takes about 2.46 s of 10 ms ticks");
    checkNear(q.largestStepIn, 0.02, 1e-6, "largest step 0.02 in (2 in/s * 10 ms)");
}

static void testStartCheck() {
    std::printf("start check\n");
    // the start pose is 10 in off (the GPS says (6, 8) from where setPose put us): fault, quickly
    {
        Robot r(Config(), 0, 0, 0);
        r.gpsOffsetXIn = 6;
        r.gpsOffsetYIn = 8;
        r.start(Mode::On);
        const std::uint32_t t0 = r.t;
        std::uint32_t faultAt = 0;
        bool anyCorrection = false;
        for (int i = 0; i < 300; i++) {
            const Correction c = r.tick(10);
            anyCorrection = anyCorrection || c.xIn != 0 || c.yIn != 0;
            if (faultAt == 0 && r.status().fault != Fault::None) faultAt = r.t;
        }
        const Status s = r.status();
        check(s.fault == Fault::StartDisagree, "10 in off: fault StartDisagree");
        check(std::string_view(s.faultText) == "start pose and GPS disagree", "fault text");
        check(faultAt != 0 && faultAt - t0 <= 1000, "within 1 s of switching on");
        std::printf("    (faulted %u ms after switching on)\n", unsigned(faultAt - t0));
        check(!anyCorrection && r.appliedXIn == 0, "and the pose was never touched");
        checkCount(s.startSamplesSeen, 5, "after 5 readings");
    }
    // 2 in off: passes
    {
        Robot r(Config(), 20, -10, 270);
        r.gpsOffsetXIn = 2;
        r.start(Mode::On);
        Status atPass;
        for (int i = 0; i < 200 && !r.status().startCheckPassed; i++) r.tick(10);
        atPass = r.status();
        check(atPass.startCheckPassed, "2 in off: start check passed");
        checkCount(atPass.accepted + atPass.droppedByGate, 0, "the start check's readings went no further");
        r.run(2000);
        check(r.status().fault == Fault::None, "2 in off: no fault");
    }
    // a mirrored frame (the routine's start pose is the other alliance's): fault
    {
        Robot r(Config(), 30, -40, 0);
        r.gpsOffsetXIn = -60; // the GPS sees the robot at (-30, -40)
        r.start(Mode::On);
        r.run(2000);
        check(r.status().fault == Fault::StartDisagree, "mirrored start: fault");
    }
}

static void testRelocalise() {
    std::printf("relocalise or give up\n");
    // a 6 in sideways shove the wheels did not see
    {
        Robot r = readyRobot(Config(), 0, 0, 0);
        r.trueX += 6; // the robot really moved; odometry did not notice
        r.truthLog.push_back({r.t, r.trueX, r.trueY, r.trueThetaDeg});
        const std::uint32_t shoveAt = r.t;
        std::uint32_t backWithin2At = 0;
        std::uint32_t relocAt = 0;
        float sigmaAtReloc = 0;
        for (int i = 0; i < 1000; i++) {
            r.tick(10);
            const Status s = r.status();
            if (relocAt == 0 && s.relocalisations == 1) {
                relocAt = r.t;
                sigmaAtReloc = s.sigmaIn;
            }
            if (backWithin2At == 0 && r.poseErrorIn() < 2) backWithin2At = r.t;
        }
        const Status s = r.status();
        checkCount(s.relocalisations, 1, "6 in shove: relocalised once");
        check(s.fault == Fault::None, "no fault");
        // R2 (revision 1 set sigma to the distance, 6 in, after 20 failures)
        checkNear(sigmaAtReloc, 1.5, 0, "sigma set to sigmaGpsIn, 1.5 in");
        checkCount(s.droppedByGate, 4, "after 4 readings failed the gate");
        checkNear(r.poseErrorIn(), 0, 0.5, "10 s later the pose is back on the truth");
        check(backWithin2At != 0 && backWithin2At - shoveAt <= 5000, "back within 2 in within 5 s");
        std::printf("    (relocalised %u ms after the shove; pose within 2 in after %u ms)\n",
                    unsigned(relocAt - shoveAt), unsigned(backWithin2At - shoveAt));
    }
    // 3 gate failures, one accepted reading, then gate failures again: the run starts again from the accepted one
    // (R2: 4 in a row; revision 1 needed 20)
    {
        Robot r = readyRobot(Config(), 0, 0, 0);
        r.gpsFrozen = true;
        for (int i = 0; i < 3; i++) feed(r, 6, 0);
        feed(r, 0, 0); // agrees with odometry: accepted
        for (int i = 0; i < 3; i++) feed(r, 6, 0);
        checkCount(r.status().droppedByGate, 6, "6 gate failures");
        checkCount(r.status().relocalisations, 0, "but only 3 in a row: no relocalisation");
        feed(r, 6, 0);
        checkCount(r.status().relocalisations, 1, "the 4th in a row: relocalised");
    }
    // a 20 in jump that the readings agree on: too far to believe, fault
    {
        Robot r = readyRobot(Config(), 0, 0, 0);
        r.gpsOffsetXIn = 20;
        r.run(5000);
        check(r.status().fault == Fault::RelocTooFar, "20 in: fault RelocTooFar");
        checkCount(r.status().relocalisations, 0, "and no relocalisation");
        check(r.appliedXIn == 0, "and nothing fed in");
    }
    // readings that fail the gate but do not agree with each other: keep watching. (R2: each must be within
    // 4.5 in of their mean; 6 and 18 alternating are 6 in from it. Revision 1's 7.5 / 10.5 now agree)
    {
        Robot r = readyRobot(Config(), 0, 0, 0);
        r.gpsFrozen = true;
        for (int i = 0; i < 60; i++) feed(r, i % 2 ? 18 : 6, 0);
        const Status s = r.status();
        check(s.droppedByGate >= 50, "inconsistent readings: dropped by the gate");
        checkCount(s.relocalisations, 0, "inconsistent readings: no relocalisation");
        check(s.fault == Fault::None, "inconsistent readings: no fault");
    }
}

static void testFaultLatches() {
    std::printf("faults latch until switched on again\n");
    Robot r(Config(), 0, 0, 0);
    r.gpsOffsetXIn = 10;
    r.start(Mode::On);
    r.run(1500);
    check(r.status().fault == Fault::StartDisagree, "faulted");
    const Status atFault = r.status();
    r.gpsOffsetXIn = 0; // the GPS is now perfect
    r.run(5000, 3, 0);
    const Status later = r.status();
    check(later.fault == Fault::StartDisagree, "still faulted 5 s later with a perfect GPS");
    checkCount(later.newReadings, atFault.newReadings, "no more readings looked at");
    checkCount(later.accepted, atFault.accepted, "none accepted");
    check(later.sigmaIn == atFault.sigmaIn && later.pendingXIn == atFault.pendingXIn, "the filter is frozen");
    check(r.appliedXIn == 0 && r.appliedYIn == 0, "nothing fed in");
    // switching Off and on again clears it; the start check runs again
    r.start(Mode::Off);
    check(r.status().fault == Fault::StartDisagree, "Off keeps showing the fault");
    r.start(Mode::On);
    check(r.status().fault == Fault::None, "switched on again: fault cleared");
    r.run(1500);
    check(r.status().startCheckPassed && r.status().fault == Fault::None, "and the new start check passes");
    // a fault in the middle of a run leaves what was already fed in where it is
    {
        Config c;
        c.kDist = 1;
        Robot m = readyRobot(c, 0, 0, 0);
        m.run(500, 20, 0); // sigma up
        m.gpsOffsetXIn = 3; // odometry slipped: corrections start
        m.run(1500);
        const double fedBefore = m.appliedXIn;
        check(fedBefore > 1, "(corrections were being fed in)");
        m.gpsOffsetXIn = 30; // then the GPS goes badly wrong in a consistent way
        double fedAtFault = -1;
        for (int i = 0; i < 600 && fedAtFault < 0; i++) {
            m.tick(10);
            if (m.status().fault != Fault::None) fedAtFault = m.appliedXIn;
        }
        check(m.status().fault == Fault::RelocTooFar, "consistent 27 in disagreement: fault");
        check(fedAtFault >= fedBefore, "(what was fed in was still there when it faulted)");
        m.run(6000);
        // R1 (revision 1 left it in the pose): everything fed in is fed back out, and the pose is plain odometry
        checkNear(m.status().appliedXIn, 0, 0, "after the fault: what was fed in is fed back out, to exactly 0");
        checkNear(m.appliedXIn, 0, 1e-4, "  and the pose is back on plain odometry");
        checkNear(m.status().pendingXIn, 0, 0, "  and nothing pending is kept");
    }
}

static void testShadow() {
    std::printf("Shadow: never corrects, tracks what On would do\n");
    // a route with odometry 3 percent long, a shove halfway that the wheels do not see, turns and stops
    Robot on(Config(), -20, -20, 0);
    Robot shadow(Config(), -20, -20, 0);
    on.odomDistanceScale = shadow.odomDistanceScale = 1.03;
    on.start(Mode::On);
    shadow.start(Mode::Shadow);
    bool shadowAlwaysZero = true;
    double worstGap = 0, worstEstimateGap = 0;
    bool sameCounts = true;
    for (int i = 0; i < 6000; i++) {
        // drive 4 in/s for 3 s, stop 2 s, turn 90 at 20 deg/s, stop 1 s: repeat
        const int phase = (i % 1050);
        double v = 0, w = 0;
        if (phase < 300) v = 4;
        else if (phase >= 500 && phase < 950) w = 20;
        if (i == 3000) { // a 5 in shove sideways that the wheels do not see
            for (Robot* r : {&on, &shadow}) {
                r->trueX += 5;
                r->truthLog.push_back({r->t, r->trueX, r->trueY, r->trueThetaDeg});
            }
        }
        on.tick(10, v, w);
        const Correction c = shadow.tick(10, v, w);
        shadowAlwaysZero = shadowAlwaysZero && c.xIn == 0.0f && c.yIn == 0.0f;
        const Status a = on.status();
        const Status b = shadow.status();
        worstGap = std::fmax(worstGap, std::hypot(a.appliedXIn - b.shadowXIn, a.appliedYIn - b.shadowYIn));
        worstEstimateGap = std::fmax(worstEstimateGap, std::hypot(a.estimateXIn - b.estimateXIn, a.estimateYIn - b.estimateYIn));
        sameCounts = sameCounts && a.accepted == b.accepted && totalDropped(a) == totalDropped(b) &&
                     a.relocalisations == b.relocalisations;
    }
    const Status a = on.status();
    const Status b = shadow.status();
    check(shadowAlwaysZero, "Shadow returned exactly zero on every tick");
    check(shadow.appliedXIn == 0 && shadow.appliedYIn == 0, "Shadow's pose is plain odometry");
    check(b.appliedXIn == 0 && b.appliedYIn == 0, "Shadow reports nothing applied");
    check(a.accepted > 20 && std::hypot(a.appliedXIn, a.appliedYIn) > 2, "(On did correct a lot)");
    checkNear(worstGap, 0, 1e-3, "every tick: Shadow's would-have-applied = On's applied");
    checkNear(worstEstimateGap, 0, 1e-3, "every tick: same fused estimate");
    check(sameCounts, "every tick: same accepted, dropped and relocalisation counts");
    std::printf("    (On applied %.2f,%.2f in; Shadow would have applied %.2f,%.2f in; odometry alone ends %.2f in off, On %.2f in)\n",
                a.appliedXIn, a.appliedYIn, b.shadowXIn, b.shadowYIn, shadow.poseErrorIn(), on.poseErrorIn());
}

static void testNoSensor() {
    std::printf("no sensor, or a sensor that never updates: the pose is plain odometry\n");
    Robot none(Config(), 0, 0, 0);
    none.gpsPlugged = false;
    none.start(Mode::On);
    none.run(5000, 4, 10);
    check(none.appliedXIn == 0 && none.appliedYIn == 0, "no sensor (nullptr): nothing fed in");
    Robot frozen(Config(), 0, 0, 0);
    frozen.gpsFrozen = true;
    frozen.start(Mode::On);
    frozen.run(5000, 4, 10);
    check(frozen.appliedXIn == 0 && frozen.appliedYIn == 0, "frozen sensor: nothing fed in");
    checkCount(frozen.status().newReadings, 1, "frozen sensor: one new reading in 5 s");
    Robot unplugged(Config(), 0, 0, 0); // PROS's answer when the sensor is not there: infinity
    unplugged.gpsFrozen = true;
    unplugged.gps.xMetres = unplugged.gps.yMetres = unplugged.gps.headingDeg = unplugged.gps.errorMetres = INFINITY;
    unplugged.start(Mode::On);
    unplugged.run(5000, 4, 10);
    check(unplugged.appliedXIn == 0 && unplugged.appliedYIn == 0, "reads infinity: nothing fed in");
}

static void testNoHistory() {
    std::printf("delay longer than the history so far: dropped and counted\n");
    Config c;
    c.latencyMs = 200;
    Robot r(c, 0, 0, 0);
    r.start(Mode::On);
    r.run(150);
    check(r.status().droppedNoHistory >= 1, "history under latencyMs: counted as no history");
    checkCount(r.status().startSamplesSeen, 0, "and not used");
    r.run(1500);
    check(r.status().startCheckPassed, "later, with enough history, it works");
}

static void testBrokenPose() {
    std::printf("a NaN pose from odometry: nothing happens\n");
    GpsFusionCore core;
    core.setMode(Mode::On);
    const GpsSample s = sampleAt(0, 0, 180);
    const Correction c = core.tick(100, NAN, 0, 0, &s);
    check(c.xIn == 0 && c.yIn == 0, "zero correction");
    checkCount(core.getStatus().newReadings, 0, "the reading was not even looked at");
}


// ------------------------------------------------------------------------------------------------ revision 2

static void testStepLimit() {
    std::printf("R1 step limit: one reading moves the decided correction by at most maxStepIn\n");
    // kDist 1 so 10 in of driving caps sigma at 5. Then a GPS 4 in to the right of the truth, robot standing still.
    //   reading 1: innovation 4, K = 25 / 27.25 = 0.917431, K * 4 = 3.67 > 1: K = 1/4. decided 1,
    //              sigma = sqrt(3/4) * 5 = 4.330127
    //   reading 2: innovation 3, K = 18.75 / 21 = 0.892857, 2.68 > 1: K = 1/3. decided 2, sigma = 3.535534
    //   reading 3: innovation 2, K = 12.5 / 14.75 = 0.847458, 1.69 > 1: K = 1/2. decided 3, sigma = 2.5
    //   reading 4: innovation 1, K = 6.25 / 8.5 = 0.735294 (under the limit, kept). decided 3.735294,
    //              sigma = sqrt(1 - 0.735294) * 2.5 = 1.286239
    Config c;
    c.kDist = 1;
    Robot r = readyRobot(c, 0, -30, 0);
    r.gpsPlugged = false;
    r.run(1000, 10, 0);
    r.run(300);
    checkNear(r.status().sigmaIn, 5, 0, "(sigma at its cap of 5)");
    r.gpsOffsetXIn = 4;
    r.gpsPlugged = true;
    const double wantInnovation[4] = {4, 3, 2, 1};
    const double wantDecided[4] = {1, 2, 3, 3.735294118};
    const double wantSigma[4] = {4.330127019, 3.535533906, 2.5, 1.286239256};
    const double wantStep[4] = {1, 1, 1, 0.735294118};
    double decidedBefore = r.status().decidedXIn;
    for (int i = 0; i < 4; i++) {
        char what[96];
        std::snprintf(what, sizeof what, "reading %d accepted", i + 1);
        check(waitForAccepted(r), what);
        const Status s = r.status();
        std::snprintf(what, sizeof what, "reading %d: innovation %.0f", i + 1, wantInnovation[i]);
        checkNear(s.lastInnovationXIn, wantInnovation[i], 1e-4, what);
        std::snprintf(what, sizeof what, "reading %d: decided %.6f", i + 1, wantDecided[i]);
        checkNear(s.decidedXIn, wantDecided[i], 1e-4, what);
        std::snprintf(what, sizeof what, "reading %d: sigma with the scaled K %.6f", i + 1, wantSigma[i]);
        checkNear(s.sigmaIn, wantSigma[i], 1e-4, what);
        std::snprintf(what, sizeof what, "reading %d: moved the decided correction by %.6f", i + 1, wantStep[i]);
        checkNear(s.decidedXIn - decidedBefore, wantStep[i], 1e-4, what);
        decidedBefore = s.decidedXIn;
    }
    checkNear(r.status().maxReadingStepIn, 1, 1e-5, "status: largest single-reading step 1.0");
    // one wild reading 12 in off, accepted by a gate of 2.5 * sqrt(25 + 2.25) = 13.05 in: still only 1 in
    Robot w = readyRobot(c, 0, -30, 0);
    w.gpsPlugged = false;
    w.run(1000, 10, 0);
    w.run(300);
    w.gpsPlugged = true;
    w.gpsFrozen = true;
    feed(w, 0, 0, 10); // the first reading after the gap: stale (check 3)
    const Status before = w.status();
    feed(w, 12, 0, 40);
    const Status after = w.status();
    checkCount(after.accepted - before.accepted, 1, "a 12 in outlier with sigma 5 is accepted");
    checkNear(len(after.decidedXIn - before.decidedXIn, after.decidedYIn - before.decidedYIn), 1, 1e-5,
              "  but moves the decided correction by exactly 1 in");
    checkNear(after.sigmaIn, std::sqrt(1 - 1.0 / 12) * 5, 1e-4, "  sigma = sqrt(1 - 1/12) * 5 (the scaled K)");
}

// A run where the GPS slowly turns out wrong (12.5 in to the right) while the robot drives at 5 in/s with sigma
// at its cap: every reading passes the gate and moves the correction by up to 1 in, until the total goes over
// 12 in. Returns the robot; `onDecided` sees the status after every tick
template <class F> static Robot totalLimitRun(Mode mode, F onTick) {
    Config c = rev2Limits(); // the hand-worked numbers below are for a 12 in limit
    c.kDist = 1;
    Robot r(c, 0, -45, 0);
    r.start(mode);
    r.run(1000);
    r.gpsPlugged = false;
    r.run(1200, 5, 0); // 6 in: sigma up to its cap of 5
    r.gpsOffsetXIn = 12.5;
    r.gpsPlugged = true;
    for (int i = 0; i < 1000; i++) {
        r.tick(10, r.status().fault == Fault::None ? 5 : 0, 0);
        onTick(r);
    }
    return r;
}

static void testTotalLimit() {
    std::printf("R1 total limit: total correction decided over maxTotalCorrectionIn = fault TotalTooLarge\n");
    double decidedBeforeFault = 0, decidedAtFault = 0, largestStep = 0, lastDecided = 0;
    bool faultSeen = false, faultedEarly = false, pendingKept = false;
    Robot r = totalLimitRun(Mode::On, [&](Robot& k) {
        const Status s = k.status();
        const double decided = len(s.decidedXIn, s.decidedYIn);
        largestStep = std::fmax(largestStep, decided - lastDecided);
        if (!faultSeen && s.fault != Fault::None) {
            faultSeen = true;
            decidedAtFault = decided;
            decidedBeforeFault = lastDecided;
            faultedEarly = decided <= 12 - 1e-4;
        }
        if (s.fault != Fault::None && (s.pendingXIn != 0 || s.pendingYIn != 0)) pendingKept = true;
        lastDecided = decided;
    });
    const Status s = r.status();
    check(s.fault == Fault::TotalTooLarge, "a GPS 12.5 in off, every reading inside the gate: fault TotalTooLarge");
    check(std::string_view(s.faultName) == "TotalTooLarge", "fault name");
    // (12 steps of exactly 1 in land on 12.0 itself, which is not over the limit: hence the 1e-4)
    check(!faultedEarly && decidedBeforeFault <= 12 + 1e-4 && decidedAtFault > 12,
          "it faults on the reading that takes the total over 12 in, not before");
    std::printf("    (decided %.3f in before the last reading, %.3f with it; %u accepted)\n", decidedBeforeFault,
                decidedAtFault, unsigned(s.accepted));
    check(largestStep <= 1 + 1e-5, "  (no reading added more than 1 in on the way)");
    check(!pendingKept, "after the fault nothing is pending");
    checkCount(s.relocalisations, 0, "  (no relocalisation: every reading passed the gate)");
    // just under the limit: a GPS 11 in off is followed to 11 in, no fault
    Config c = rev2Limits();
    c.kDist = 1;
    Robot u(c, 0, -45, 0);
    u.start(Mode::On);
    u.run(1000);
    u.gpsPlugged = false;
    u.run(1200, 5, 0);
    u.gpsOffsetXIn = 11;
    u.gpsPlugged = true;
    u.run(4000, 5, 0);
    u.run(8000);
    check(u.status().fault == Fault::None, "a GPS 11 in off: no fault");
    checkNear(u.status().decidedXIn, 11, 0.05, "  and it is followed to 11 in");
    // a lower limit is a lower limit
    Config low = c;
    low.maxTotalCorrectionIn = 3;
    Robot l(low, 0, -45, 0);
    l.start(Mode::On);
    l.run(1000);
    l.gpsPlugged = false;
    l.run(1200, 5, 0);
    l.gpsOffsetXIn = 4;
    l.gpsPlugged = true;
    l.run(3000, 5, 0);
    check(l.status().fault == Fault::TotalTooLarge, "maxTotalCorrectionIn 3, GPS 4 in off: fault");
    check(len(l.status().decidedXIn, l.status().decidedYIn) <= 4, "  after going just over 3");
}

static void testUnwind() {
    std::printf("R1 a fault feeds everything fed in back out, at slewInPerS, to exactly zero (On and Shadow)\n");
    for (const Mode mode : {Mode::On, Mode::Shadow}) {
        const bool on = mode == Mode::On;
        double fedAtFault = -1;
        bool withinLimit = true, neverGrew = true, shadowZero = true, sawUnwinding = false;
        double prevFed = 0;
        int ticksAfterFault = 0;
        const int dts[] = {10, 10, 13, 7, 500, 10, 3, 10, 10, 10};
        Config c;
        c.kDist = 1;
        Robot r(c, 0, -45, 0);
        r.start(mode);
        r.run(1000);
        r.gpsPlugged = false;
        r.run(1200, 5, 0);
        r.gpsOffsetXIn = 12.5;
        r.gpsPlugged = true;
        for (int i = 0; i < 2000; i++) {
            const bool faulted = r.status().fault != Fault::None;
            const int dt = faulted ? dts[ticksAfterFault++ % 10] : 10;
            const Correction step = r.tick(dt, faulted ? 0 : 5, 0);
            const Status s = r.status();
            const double fed = on ? len(s.appliedXIn, s.appliedYIn) : len(s.shadowXIn, s.shadowYIn);
            if (!on) shadowZero = shadowZero && step.xIn == 0.0f && step.yIn == 0.0f;
            if (s.fault != Fault::None) {
                if (fedAtFault < 0) {
                    fedAtFault = prevFed;
                    prevFed = fed + 0; // the tick that faulted already unwinds
                }
                const double limit = c.slewInPerS * std::fmin(dt, 20) / 1000.0;
                const double change = std::fabs(fed - prevFed);
                if (change > limit * (1 + 1e-5) + 1e-7) withinLimit = false;
                if (fed > prevFed + 1e-7) neverGrew = false;
                if (s.unwinding) sawUnwinding = true;
            }
            prevFed = fed;
        }
        const Status s = r.status();
        const char* m = on ? "On" : "Shadow";
        char what[128];
        std::snprintf(what, sizeof what, "%s: faulted with %.2f in fed in", m, fedAtFault);
        check(s.fault == Fault::TotalTooLarge && fedAtFault > 1, what);
        std::snprintf(what, sizeof what, "%s: fed in total unwound to exactly 0", m);
        check(s.appliedXIn == 0 && s.appliedYIn == 0 && s.shadowXIn == 0 && s.shadowYIn == 0, what);
        std::snprintf(what, sizeof what, "%s: no unwinding tick moved more than slewInPerS * dt (late tick: 20 ms)", m);
        check(withinLimit, what);
        std::snprintf(what, sizeof what, "%s: the fed-in total only ever shrank after the fault", m);
        check(neverGrew, what);
        std::snprintf(what, sizeof what, "%s: status said unwinding meanwhile, and not once done", m);
        check(sawUnwinding && !s.unwinding, what);
        if (on) {
            checkNear(r.appliedXIn, 0, 1e-4, "On: the corrections the ticks returned add up to 0: plain odometry");
            checkNear(r.appliedYIn, 0, 1e-4, "On: (y too)");
        } else {
            check(shadowZero && r.appliedXIn == 0 && r.appliedYIn == 0, "Shadow: every tick returned exactly zero");
        }
    }
    // Shadow's would-be total unwinds exactly as On's real one does, tick by tick
    std::vector<double> onFed, shadowFed;
    totalLimitRun(Mode::On, [&](Robot& k) { onFed.push_back(k.status().appliedXIn); });
    totalLimitRun(Mode::Shadow, [&](Robot& k) { shadowFed.push_back(k.status().shadowXIn); });
    double worst = 0;
    for (size_t i = 0; i < onFed.size() && i < shadowFed.size(); i++)
        worst = std::fmax(worst, std::fabs(onFed[i] - shadowFed[i]));
    checkNear(worst, 0, 1e-3, "every tick: Shadow's would-be total = On's total, through the fault and unwinding");
    // switching Off in the middle of unwinding: Off means nothing runs, so what is left stays (design: Off leaves
    // the pose alone)
    {
        Config c;
        c.kDist = 1;
        Robot r(c, 0, -45, 0);
        r.start(Mode::On);
        r.run(1000);
        r.gpsPlugged = false;
        r.run(1200, 5, 0);
        r.gpsOffsetXIn = 12.5;
        r.gpsPlugged = true;
        for (int i = 0; i < 1000 && r.status().fault == Fault::None; i++) r.tick(10, 5, 0);
        r.run(200);
        r.start(Mode::Off);
        const double left = r.appliedXIn;
        r.run(3000);
        check(left > 0.1 && r.appliedXIn == left, "Off during unwinding: nothing more is fed in or out");
        check(!r.status().unwinding, "  and the status does not say unwinding");
    }
}

static void testRelocalisationR2() {
    std::printf("R2 relocalisation: 4 gate failures in a row, within 2 s, each within 4.5 in of their mean\n");
    {
        const Config d;
        check(d.maxTotalCorrectionIn == 6 && d.relocMaxIn == 6 && d.relocMaxCount == 1,
              "revision 3 defaults: total limit 6 in, relocalisation within 6 in, 1 per run");
    }
    // fires: 4 readings 6 in to the right, 120 ms apart
    {
        Robot r = readyRobot(rev2Limits(), 0, 0, 0);
        r.gpsFrozen = true;
        const Status s0 = r.status();
        for (int i = 0; i < 3; i++) feed(r, 6, 0);
        checkCount(r.status().relocalisations, 0, "3 gate failures: not yet");
        checkCount(r.status().droppedByGate - s0.droppedByGate, 3, "  (all 3 failed the gate)");
        feed(r, 6, 0);
        const Status s = r.status();
        checkCount(s.relocalisations, 1, "the 4th, 360 ms after the 1st: relocalised");
        checkNear(s.decidedXIn, 6, 1e-3, "  the mean innovation added in one go: decided = 6 in");
        checkNear(s.sigmaIn, 1.5, 0, "  sigma = sigmaGpsIn");
        checkNear(s.maxReadingStepIn, s0.maxReadingStepIn, 0, "  (not a single reading's step: no step limit)");
        check(s.pendingXIn > 5.9, "  and it is pending, to be fed in at slewInPerS");
        r.run(4000);
        checkNear(r.poseErrorIn(), 6, 1e-3, "  4 s later the pose has moved 6 in (to where the GPS says)");
    }
    // the same 4 readings spread over more than relocWindowMs: no relocalisation. Readings dropped by a check in
    // between neither add to the run nor end it; a 5th close behind makes the last 4 fit in 2 s
    {
        Robot r = readyRobot(rev2Limits(), 0, 0, 0);
        r.gpsFrozen = true;
        for (int i = 0; i < 4; i++) {
            if (i > 0) filler(r, 600);
            feed(r, 6, 0);
        }
        checkCount(r.status().relocalisations, 0, "4 failures over 2.16 s (window 2 s): no relocalisation");
        checkCount(r.status().droppedByGate, 4, "  (all 4 failed the gate)");
        feed(r, 6, 0);
        checkCount(r.status().relocalisations, 1, "a 5th 120 ms later: the last 4 span 1.56 s: relocalised");
    }
    // agreement is each within relocSpreadIn of the mean: 4.4 in from it agrees, 4.6 does not
    for (const double d : {4.4, 4.6}) {
        Robot r = readyRobot(rev2Limits(), 0, 0, 0);
        r.gpsFrozen = true;
        for (int i = 0; i < 4; i++) feed(r, 10 + (i % 2 ? d : -d), 0);
        char what[96];
        std::snprintf(what, sizeof what, "4 failures each %.1f in from their mean: %s", d,
                      d < 4.5 ? "agree, relocalised" : "disagree, not");
        checkCount(r.status().relocalisations, d < 4.5 ? 1 : 0, what);
        check(r.status().fault == Fault::None, "  no fault");
    }
    // scattered round the mean in 2 dimensions: (5,0) (9,0) (7,2) (7,-2) are each 2 in from (7,0)
    {
        Robot r = readyRobot(rev2Limits(), 0, 0, 0);
        r.gpsFrozen = true;
        const double pts[4][2] = {{5, 0}, {9, 0}, {7, 2}, {7, -2}};
        for (const auto& p : pts) feed(r, p[0], p[1]);
        checkCount(r.status().relocalisations, 1, "4 failures scattered 2 in round (7, 0): relocalised");
        checkNear(r.status().decidedXIn, 7, 1e-3, "  by their mean, 7 in");
        checkNear(r.status().decidedYIn, 0, 1e-3, "  (and 0 in y)");
    }
    // a relocalisation starts a new run: after one by (6, 0), readings at (6, 6) need 4 of their own. (Kept
    // from the old run, two of them would agree with the old ones round (3, 3) and relocalise at once)
    {
        Robot r = readyRobot(rev2Limits(), 0, 0, 0);
        r.gpsFrozen = true;
        for (int i = 0; i < 4; i++) feed(r, 6, 0);
        checkCount(r.status().relocalisations, 1, "relocalised by (6, 0)");
        for (int i = 0; i < 3; i++) feed(r, 6, 6);
        checkCount(r.status().relocalisations, 1, "then 3 failures at (6, 6): not yet (the run started again)");
        feed(r, 6, 6);
        checkCount(r.status().relocalisations, 2, "  the 4th: relocalised again");
        checkNear(r.status().decidedXIn, 6, 1e-3, "  decided (6, 6): x");
        checkNear(r.status().decidedYIn, 6, 1e-3, "  decided (6, 6): y");
    }
    // RelocTooFar: they agree but the mean is more than relocMaxIn (12) away. 11.5 is close enough
    {
        Robot r = readyRobot(rev2Limits(), 0, 0, 0);
        r.gpsFrozen = true;
        for (int i = 0; i < 4; i++) feed(r, 14, 0);
        check(r.status().fault == Fault::RelocTooFar, "4 agreeing failures 14 in off: fault RelocTooFar");
        checkCount(r.status().relocalisations, 0, "  no relocalisation");
        checkNear(r.status().pendingXIn, 0, 0, "  nothing pending");
        Robot k = readyRobot(rev2Limits(), 0, 0, 0);
        k.gpsFrozen = true;
        for (int i = 0; i < 4; i++) feed(k, 11.5, 0);
        check(k.status().fault == Fault::None && k.status().relocalisations == 1, "11.5 in off: relocalised, no fault");
    }
    // RelocTooMany: two relocalisations are allowed, a third faults. +6, back to 0, +6 again
    {
        Robot r = readyRobot(rev2Limits(), 0, 0, 0);
        r.gpsFrozen = true;
        for (int i = 0; i < 4; i++) feed(r, 6, 0);
        checkCount(r.status().relocalisations, 1, "first shove: relocalised");
        for (int i = 0; i < 4; i++) feed(r, 0, 0);
        checkCount(r.status().relocalisations, 2, "second (back again): relocalised");
        check(r.status().fault == Fault::None, "  no fault after two");
        checkNear(r.status().decidedXIn, 0, 1e-3, "  (decided back to about 0: under the total limit)");
        for (int i = 0; i < 4; i++) feed(r, 6, 0);
        check(r.status().fault == Fault::RelocTooMany, "third: fault RelocTooMany");
        checkCount(r.status().relocalisations, 2, "  still 2 relocalisations");
        Config one = rev2Limits();
        one.relocMaxCount = 1;
        Robot o = readyRobot(one, 0, 0, 0);
        o.gpsFrozen = true;
        for (int i = 0; i < 4; i++) feed(o, 6, 0);
        for (int i = 0; i < 4; i++) feed(o, 0, 0);
        check(o.status().fault == Fault::RelocTooMany, "relocMaxCount 1: the second faults");
    }
    // the total limit applies to relocalisations: +8 then +16 adds up to 16 in > 12
    {
        Robot r = readyRobot(rev2Limits(), 0, 0, 0);
        r.gpsFrozen = true;
        for (int i = 0; i < 4; i++) feed(r, 8, 0);
        checkCount(r.status().relocalisations, 1, "relocalised by 8 in");
        for (int i = 0; i < 4; i++) feed(r, 16, 0);
        check(r.status().fault == Fault::TotalTooLarge, "then by 8 more: total 16 in: fault TotalTooLarge");
        checkNear(r.status().pendingXIn, 0, 0, "  pending discarded");
        r.run(12000);
        checkNear(r.status().appliedXIn, 0, 0, "  and what was fed in is fed back out to exactly 0");
        checkNear(r.poseErrorIn(), 0, 1e-4, "  the pose is plain odometry again");
    }
}

static void testStartSigma() {
    std::printf("R3 startSigmaIn: sigma once the start check passes; a start pose 3 in off is corrected at the first stop\n");
    // the start pose is 3 in off: the robot is really at x = 3. When the start check passes sigma = 1. The next
    // reading: innovation 3, K = 1 / (1 + 2.25) = 0.307692, decided 0.923077, sigma = sqrt(1 - K) = 0.832050
    {
        Robot r(Config(), 0, 0, 0);
        r.trueX = 3;
        r.truthLog = {{r.t, r.trueX, r.trueY, r.trueThetaDeg}};
        r.updateGps();
        r.start(Mode::On);
        bool decidedEarly = false;
        for (int i = 0; i < 300 && !r.status().startCheckPassed; i++) {
            r.tick(10);
            if (!r.status().startCheckPassed && r.status().decidedXIn != 0) decidedEarly = true;
        }
        check(r.status().startCheckPassed, "start check passed (3 in is inside startTolIn)");
        check(!decidedEarly && r.status().decidedXIn == 0, "nothing decided before it passed");
        checkNear(r.status().sigmaIn, 1, 0, "sigma = startSigmaIn = 1 when it passes");
        check(waitForAccepted(r, 1000), "the next reading is accepted");
        checkNear(r.status().decidedXIn, 0.923076923, 1e-4, "  decided 3 / 3.25 = 0.923077");
        checkNear(r.status().sigmaIn, 0.832050294, 1e-4, "  sigma sqrt(1 - 1/3.25) = 0.832050");
        r.run(10000);
        checkNear(r.poseErrorIn(), 0, 0.5, "10 s standing still: the start pose is corrected");
    }
    // revision 1's behaviour, startSigmaIn 0: the same reading corrects nothing
    {
        Config c;
        c.startSigmaIn = 0;
        Robot r(c, 0, 0, 0);
        r.trueX = 3;
        r.truthLog = {{r.t, r.trueX, r.trueY, r.trueThetaDeg}};
        r.updateGps();
        r.start(Mode::On);
        r.run(3000);
        check(r.status().accepted > 5, "(startSigmaIn 0: readings accepted)");
        checkNear(r.status().decidedXIn, 0, 0, "startSigmaIn 0: nothing corrected standing still");
    }
    // other values, and one above the cap (kept within sigmaMaxIn: not in the design)
    for (const float want : {2.5f, 7.0f}) {
        Config c;
        c.startSigmaIn = want;
        Robot r(c, 0, 0, 0);
        r.start(Mode::On);
        for (int i = 0; i < 300 && !r.status().startCheckPassed; i++) r.tick(10);
        char what[80];
        std::snprintf(what, sizeof what, "startSigmaIn %.1f: sigma %.1f when the start check passes", want,
                      std::fmin(want, 5.0f));
        checkNear(r.status().sigmaIn, std::fmin(want, 5.0f), 0, what);
    }
}

// Revision 3: the heading fault only judges readings check 6 calls slow, and check 6 needs latencyMs + 50 ms of
// history (110 ms at the defaults) before it can call anything slow. The hand-fed tests below stand still this
// long after switching on before feeding (the GPS is frozen meanwhile, so check 3 still sees the sensor alive)
static const int HEADING_HISTORY_MS = 100;

static void testHeadingFault() {
    std::printf("R3 heading fault: 8 of the first 10 readings past checks 1-4 failing check 5 = HeadingDisagree\n");
    // the field turned 180 degrees (the GPS's frame against ours)
    {
        Robot r(Config(), 20, 30, 0);
        r.gpsFrame = 1;
        r.start(Mode::On);
        const std::uint32_t t0 = r.t;
        std::uint32_t faultAt = 0;
        for (int i = 0; i < 300; i++) {
            r.tick(10);
            if (!faultAt && r.status().fault != Fault::None) faultAt = r.t;
        }
        const Status s = r.status();
        check(s.fault == Fault::HeadingDisagree, "field turned 180: fault HeadingDisagree");
        check(faultAt && faultAt - t0 <= 1000, "  within 1 s");
        std::printf("    (faulted %u ms after switching on)\n", unsigned(faultAt - t0));
        checkCount(s.headingSamplesSeen, 8, "  after 8 readings");
        checkCount(s.headingSamplesFailed, 8, "  all 8 failed the heading check");
        check(s.headingJudged, "  judged");
        check(r.appliedXIn == 0 && r.appliedYIn == 0, "  the pose was never touched");
        checkCount(s.startSamplesSeen, 0, "  (the start check never got a reading)");
    }
    // the sensor really faces forward but mountYawDeg says backward, and the other way round
    for (const double trueYaw : {0.0, 180.0}) {
        Config c;
        c.mountYawDeg = float(180 - trueYaw); // what the filter is told: wrong by 180
        Robot r(c, 0, 0, 45);
        r.config.mountYawDeg = float(trueYaw); // what the pretend sensor really does
        r.start(Mode::On);
        r.run(2000);
        char what[96];
        std::snprintf(what, sizeof what, "camera faces %s, mount yaw says the other way: fault HeadingDisagree",
                      trueYaw == 0 ? "forward" : "backward");
        check(r.status().fault == Fault::HeadingDisagree, what);
    }
    // a good sensor: judged on 10, none failed, no fault; and the run goes on normally
    {
        Robot r(Config(), 0, 0, 30);
        r.start(Mode::On);
        r.run(3000);
        const Status s = r.status();
        check(s.fault == Fault::None, "good sensor: no fault");
        check(s.headingJudged && s.headingSamplesSeen == 10 && s.headingSamplesFailed == 0,
              "  judged on 10 readings, none failed");
        check(s.startCheckPassed && s.accepted > 5, "  and the run goes on");
    }
    // by hand: 7 of the first 10 failing is not a fault; 8 is. After that it is never judged again
    for (const int failing : {7, 8}) {
        Robot r(Config(), 0, 0, 0);
        r.gpsFrozen = true;
        r.start(Mode::On);
        r.tick(10); // the first reading after switching on: stale (check 3), not judged
        r.run(HEADING_HISTORY_MS); // Revision 3: only readings check 6 can call slow are judged
        for (int i = 0; i < 10; i++) feed(r, 0, 0, 40, i < failing ? 15 : 0);
        char what[80];
        std::snprintf(what, sizeof what, "%d of the first 10 fail the heading check: %s", failing,
                      failing >= 8 ? "fault" : "no fault");
        check((r.status().fault == Fault::HeadingDisagree) == (failing >= 8), what);
        if (failing == 7) {
            check(r.status().headingJudged, "  judged");
            for (int i = 0; i < 20; i++) feed(r, 0, 0, 40, 15);
            check(r.status().fault == Fault::None, "  20 failing readings after that: still no fault (judged once)");
            check(r.status().droppedByCheck[CHECK_HEADING] >= 27, "  (they were dropped by check 5)");
        }
    }
    // the fault comes as soon as 8 have failed (the verdict cannot change after that), and readings that pass
    // count: 1 good then 8 bad is 8 of 9, a fault at the 9th
    {
        Robot r(Config(), 0, 0, 0);
        r.gpsFrozen = true;
        r.start(Mode::On);
        r.tick(10); // stale
        r.run(HEADING_HISTORY_MS);
        for (int i = 0; i < 8; i++) feed(r, 0, 0, 40, 15);
        check(r.status().fault == Fault::HeadingDisagree, "8 failures in a row: fault at the 8th");
        checkCount(r.status().headingSamplesSeen, 8, "  after 8 readings");
        Robot g(Config(), 0, 0, 0);
        g.gpsFrozen = true;
        g.start(Mode::On);
        g.tick(10);
        g.run(HEADING_HISTORY_MS);
        feed(g, 0, 0, 40);
        for (int i = 0; i < 7; i++) feed(g, 0, 0, 40, 15);
        check(g.status().fault == Fault::None, "1 good then 7 bad: no fault yet");
        feed(g, 0, 0, 40, 15);
        check(g.status().fault == Fault::HeadingDisagree, "  the 8th bad (9th judged): fault");
        checkCount(g.status().headingSamplesSeen, 9, "  after 9 readings");
    }
    // readings dropped by checks 1 to 4 are not judged: 10 outside the field with the heading wrong
    {
        Robot r(Config(), 0, 0, 0);
        r.gpsFrozen = true;
        r.start(Mode::On);
        r.tick(10); // stale
        r.run(HEADING_HISTORY_MS);
        for (int i = 0; i < 10; i++) feed(r, 80, 0, 40, 15);
        check(r.status().fault == Fault::None, "10 readings outside the field with the heading wrong: no fault");
        checkCount(r.status().headingSamplesSeen, 0, "  none judged");
    }
    // startHeadingSamples 5: 80 percent rounded up is 4
    for (const int failing : {3, 4}) {
        Config c;
        c.startHeadingSamples = 5;
        Robot r(c, 0, 0, 0);
        r.gpsFrozen = true;
        r.start(Mode::On);
        r.tick(10); // stale
        r.run(HEADING_HISTORY_MS);
        for (int i = 0; i < 5; i++) feed(r, 0, 0, 40, i < failing ? 15 : 0);
        char what[80];
        std::snprintf(what, sizeof what, "startHeadingSamples 5, %d failing: %s", failing,
                      failing >= 4 ? "fault" : "no fault");
        check((r.status().fault == Fault::HeadingDisagree) == (failing >= 4), what);
    }
    // startHeadingSamples 3: ceil(2.4) = 3; and 0 = not judged
    {
        Config c;
        c.startHeadingSamples = 3;
        Robot r(c, 0, 0, 0);
        r.gpsFrozen = true;
        r.start(Mode::On);
        r.tick(10); // stale
        r.run(HEADING_HISTORY_MS);
        for (int i = 0; i < 3; i++) feed(r, 0, 0, 40, i < 2 ? 15 : 0);
        check(r.status().fault == Fault::None, "startHeadingSamples 3, 2 failing: no fault");
        Config z;
        z.startHeadingSamples = 0;
        Robot q(z, 0, 0, 0);
        q.gpsFrame = 1;
        q.start(Mode::On);
        q.run(3000);
        check(q.status().fault == Fault::None, "startHeadingSamples 0: not judged, no fault even turned 180");
    }
}

static void testCheckSlowSecondWindow() {
    std::printf("R4 check 6 also looks at the 50 ms ending latencyMs ago, when the reading was taken\n");
    for (const bool turning : {false, true}) {
        Robot r = readyRobot(Config(), 0, -30, 0);
        r.run(1000, turning ? 0 : 20, turning ? 60 : 0); // 20 in/s or 60 deg/s, then stop dead
        r.gpsFrozen = true; // the sensor updated at most 40 ms ago: still alive
        const Status before = r.status();
        // 60 ms after the stop: the last 50 ms are still, but the 50 ms ending 60 ms ago are not
        feed(r, 0, 0, 60);
        const Status mid = r.status();
        const char* what = turning ? "60 deg/s then stop: a reading 60 ms later is dropped by check 6"
                                   : "20 in/s then stop: a reading 60 ms later is dropped by check 6";
        checkCount(mid.droppedByCheck[CHECK_SLOW] - before.droppedByCheck[CHECK_SLOW], 1, what);
        // 120 ms after the stop both windows are still: used
        feed(r, 0, 0, 60);
        const Status after = r.status();
        checkCount(after.droppedByCheck[CHECK_SLOW], mid.droppedByCheck[CHECK_SLOW], "  120 ms after: not by check 6");
        checkCount(after.accepted - mid.accepted, 1, "  and accepted");
    }
    // the second window needs latencyMs + 50 ms of history: until then the reading is counted as no history
    {
        Robot r(Config(), 0, 0, 0);
        r.gpsFrozen = true;
        r.start(Mode::On);
        feed(r, 0, 0, 10); // t+10: stale
        feed(r, 0, 0, 50); // t+60: history 50 ms: window 1 fine, window 2 needs 110
        checkCount(r.status().droppedNoHistory, 1, "60 ms after switching on: no history for the second window");
        feed(r, 0, 0, 60); // t+120
        checkCount(r.status().droppedNoHistory, 1, "120 ms: enough history");
        checkCount(r.status().startSamplesSeen, 1, "  (the start check got it)");
    }
}

static void testCheck5b() {
    std::printf("R4 check 5b: the sensor's own error number, off while maxSensorErrorIn is 0\n");
    // off (the default): a reading with get_error() 1 m (39 in) is used
    {
        Robot r = readyRobot(Config(), 0, 0, 0);
        r.gpsFrozen = true;
        const Status before = r.status();
        feed(r, 0, 0, 120, 0, 39.37);
        checkCount(r.status().droppedBySensorError, 0, "off: 39 in error not dropped");
        checkCount(r.status().accepted - before.accepted, 1, "  accepted");
    }
    // on at 2 in: 2.5 in is dropped (its own counter, nothing else), 1.5 in is used
    {
        Config c;
        c.maxSensorErrorIn = 2;
        Robot r = readyRobot(c, 0, 0, 0);
        r.gpsFrozen = true;
        const Status before = r.status();
        feed(r, 0, 0, 120, 0, 2.5);
        const Status mid = r.status();
        checkCount(mid.droppedBySensorError - before.droppedBySensorError, 1, "maxSensorErrorIn 2: 2.5 in dropped");
        checkCount(totalDropped(mid) - totalDropped(before), 1, "  counted once, by check 5b only");
        checkCount(mid.accepted, before.accepted, "  not accepted");
        feed(r, 0, 0, 120, 0, 1.5);
        checkCount(r.status().accepted - mid.accepted, 1, "maxSensorErrorIn 2: 1.5 in accepted");
        // between check 5 and check 6: heading wrong as well -> check 5; moving fast as well -> check 5b
        const Status s0 = r.status();
        feed(r, 0, 0, 120, 15, 2.5);
        const Status s1 = r.status();
        checkCount(s1.droppedByCheck[CHECK_HEADING] - s0.droppedByCheck[CHECK_HEADING], 1,
                   "heading wrong and error too big: counted at check 5");
        checkCount(s1.droppedBySensorError, s0.droppedBySensorError, "  not at 5b");
        r.run(100, 20, 0);
        r.gps = sampleAt(r.trueX, r.trueY, r.trueThetaDeg + 180.003);
        r.gps.errorMetres = 2.5 / INCHES_PER_METRE;
        r.tick(10, 20, 0);
        const Status s2 = r.status();
        checkCount(s2.droppedBySensorError - s1.droppedBySensorError, 1, "error too big and moving fast: at 5b");
        checkCount(s2.droppedByCheck[CHECK_SLOW], s1.droppedByCheck[CHECK_SLOW], "  not at check 6");
    }
    // a sensor whose error is always too big: nothing reaches the start check, but the heading fault is still
    // judged (5b comes after check 5)
    {
        Config c;
        c.maxSensorErrorIn = 2;
        Robot r(c, 0, 0, 0);
        r.gps.errorMetres = 3 / INCHES_PER_METRE;
        r.gpsFrozen = true;
        r.start(Mode::On);
        for (int i = 0; i < 30; i++) feed(r, 0, 0, 40, 0, 3);
        check(r.status().headingJudged && r.status().headingSamplesFailed == 0, "error always 3 in: heading judged");
        checkCount(r.status().startSamplesSeen, 0, "  nothing reached the start check");
        check(r.status().droppedBySensorError >= 25, "  dropped by 5b");
    }
}

static void testDiagnostics() {
    std::printf("R4 diagnostics: innovation in the robot's frame, normalised innovation squared, the line\n");
    // A camera really 3 in to the right of and 5 in behind the centre, the mount left at 0. With no doubt at all
    // (sigma 0, kDist = kTurn = 0, startSigmaIn 0) nothing is ever corrected, so every innovation is the mount
    // error. In field coordinates it turns with the robot; in the robot's frame it is (3, -5) at every heading
    {
        Config c;
        c.kDist = 0;
        c.kTurn = 0;
        c.startSigmaIn = 0;
        c.gate = 100; // every reading reaches the update (with K = 0), so nothing relocalises
        Robot r(c, 0, 0, 0);
        r.config.mountRightIn = 3;
        r.config.mountForwardIn = -5;
        r.updateGps();
        r.start(Mode::Shadow);
        for (int k = 0; k < 4; k++) {
            r.run(1500); // readings at this heading
            r.run(3600, 0, 25); // turn 90 deg at 25 deg/s
        }
        const Status s = r.status();
        check(s.innovationCount > 20 && s.fault == Fault::None, "(readings at four headings, no fault)");
        checkNear(s.innovationRightAvgIn, 3, 0.02, "average innovation to the right: the 3 in mount error");
        checkNear(s.innovationForwardAvgIn, -5, 0.02, "average innovation forward: the -5 in mount error");
    }
    // NIS with sigma 0: a GPS 1.5 in off gives 1.5^2 / 1.5^2 = 1 every time; 3 in off gives 4
    for (const double off : {1.5, 3.0}) {
        Config c;
        c.kDist = 0;
        c.kTurn = 0;
        c.startSigmaIn = 0;
        Robot r(c, 0, 0, 0);
        r.gpsOffsetYIn = off;
        r.start(Mode::On);
        r.run(3000);
        char what[80];
        std::snprintf(what, sizeof what, "GPS %.1f in off, sigma 0: average NIS %.1f", off, off * off / 2.25);
        checkNear(r.status().nisAvg, off * off / 2.25, 1e-3, what);
        check(r.status().nisCount == r.status().accepted + r.status().droppedByGate, "  over every reading that reached the gate");
    }
    // NIS with sigma not 0: startSigmaIn 2 and nothing growing it. The first reading after the start check is 3 in
    // off: 9 / (4 + 2.25) = 1.44
    {
        Config c;
        c.kDist = 0;
        c.kTurn = 0;
        c.startSigmaIn = 2;
        c.gate = 100;
        Robot r(c, 0, 0, 0);
        r.start(Mode::On);
        for (int i = 0; i < 300 && !r.status().startCheckPassed; i++) r.tick(10);
        r.gpsFrozen = true;
        checkCount(r.status().nisCount, 0, "(no reading at the gate yet)");
        feed(r, 3, 0);
        checkCount(r.status().nisCount, 1, "one reading at the gate");
        checkNear(r.status().nisAvg, 9 / 6.25, 1e-4, "sigma 2, 3 in off: NIS 9 / (4 + 2.25) = 1.44");
    }
    // the [GPSFUSE] line: under about 200 characters even with big counters, start progress, 8 drop counters
    {
        Status s;
        s.mode = Mode::Shadow;
        s.estimateXIn = -48.1f;
        s.estimateYIn = -59.9f;
        s.sigmaIn = 1.23f;
        s.pendingXIn = 0.1f;
        s.shadowXIn = 10.2f;
        s.startCheckPassed = true;
        s.accepted = 123;
        const std::uint32_t drops[NUM_CHECKS] = {1, 0, 45, 210, 12, 301, 987};
        for (int i = 0; i < NUM_CHECKS; i++) s.droppedByCheck[i] = drops[i];
        s.droppedBySensorError = 7;
        s.droppedByGate = 23;
        s.droppedNoHistory = 249;
        s.relocalisations = 2;
        s.innovationRightAvgIn = -0.12f;
        s.innovationForwardAvgIn = -0.4f;
        s.nisAvg = 12.34f;
        s.fault = Fault::HeadingDisagree;
        s.faultName = faultName(s.fault);
        char line[400];
        const int n = formatStatusLine(line, sizeof line, 61230, -48.25f, -60.1f, -22.1657f, s);
        std::printf("    %s (%d characters)\n", line, n);
        check(n > 0 && n <= 216, "a busy 60 s line is about 215 characters at most (nohist= is back, Revision 3)");
        check(std::string_view(line).find("drop=1/0/45/210/12/7/301/987/23 ") != std::string_view::npos,
              "  drop= shows checks 1, 2, 3, 4, 5, 5b, 6, 7 and the gate");
        check(std::string_view(line).find(" nohist=249 ") != std::string_view::npos,
              "  nohist= shows the readings the history could not serve (Revision 3)");
        check(std::string_view(line).find("start=ok") != std::string_view::npos, "  start=ok once passed");
        check(std::string_view(line).find("inn=-0.1,-0.4 nis=12.3") != std::string_view::npos, "  inn= and nis=");
        check(std::string_view(line).find("fault=HeadingDisagree") != std::string_view::npos, "  the fault's name");
        check(std::string_view(line).find("fed=10.2") != std::string_view::npos, "  fed= (Shadow: would have)");
        s.startCheckPassed = false;
        s.startSamplesSeen = 3;
        s.startSamplesNeeded = 5;
        formatStatusLine(line, sizeof line, 61230, 0, 0, 0, s);
        check(std::string_view(line).find("start=3/5") != std::string_view::npos, "  start=3/5 before it passes");
        const int small = formatStatusLine(line, 40, 61230, 0, 0, 0, s);
        check(small > 39 && std::string_view(line).size() == 39, "  a short buffer is cut, not overrun");
    }
}

// ------------------------------------------------------------------------------------------------ Revision 3

// Ticks until the filter's fault is set (or maxMs), and returns how long that took (0: no fault)
static std::uint32_t msToFault(Robot& r, int maxMs, double inPerS = 0, double degPerS = 0) {
    const std::uint32_t t0 = r.t;
    for (int done = 0; done < maxMs; done += 10) {
        r.tick(10, inPerS, degPerS);
        if (r.status().fault != Fault::None) return r.t - t0;
    }
    return 0;
}

static void testHeadingFaultOnlyWhenSlow() {
    std::printf("Revision 3: the heading fault judges only readings taken while the robot is slow (check 6)\n");
    // A routine that opens with a 300 deg/s turn (180 deg in 0.6 s), a healthy sensor: no reading taken during
    // the turn is judged, so the fault does not come near (the reviewer saw 7 of 8 with the old rule)
    for (const int trueDelayMs : {60, 100}) {
        Robot r(Config(), 0, -24, 0);
        r.gpsDelayMs = trueDelayMs;
        r.start(Mode::On);
        int worstFailed = 0;
        for (int done = 0; done < 600; done += 10) {
            r.tick(10, 0, 300);
            worstFailed = std::max(worstFailed, r.status().headingSamplesFailed);
        }
        for (int done = 0; done < 3000; done += 10) {
            r.tick(10);
            worstFailed = std::max(worstFailed, r.status().headingSamplesFailed);
        }
        const Status s = r.status();
        char what[160];
        std::snprintf(what, sizeof what, "opening 300 deg/s turn, healthy sensor, true delay %d ms: no fault", trueDelayMs);
        check(s.fault == Fault::None, what);
        std::snprintf(what, sizeof what, "  at most 1 judged reading failed (worst %d; 8 = fault)", worstFailed);
        check(worstFailed <= 1, what);
        check(s.headingJudged && s.headingSamplesSeen == 10, "  judged on 10 readings after the turn");
        check(s.startCheckPassed && s.accepted > 5, "  and the run goes on");
    }
    // the same turn, but no reading judged until the robot stops: none judged during it
    {
        Robot r(Config(), 0, -24, 0);
        r.start(Mode::On);
        for (int done = 0; done < 600; done += 10) r.tick(10, 0, 300);
        checkCount(r.status().headingSamplesSeen, 0, "during the turn: no reading judged");
        check(r.status().droppedByCheck[CHECK_SLOW] + r.status().droppedByCheck[CHECK_HEADING] > 5,
              "  (they were dropped by check 5 or 6 instead)");
    }
    // readings check 6 cannot judge yet (the history is shorter than latencyMs + 50 ms) are not judged either
    {
        Robot r(Config(), 0, 0, 0);
        r.gpsFrozen = true;
        r.start(Mode::On);
        r.tick(10); // stale
        feed(r, 0, 0, 40, 15);
        feed(r, 0, 0, 40, 15);
        checkCount(r.status().headingSamplesSeen, 0, "two readings in the first 90 ms (too little history): not judged");
    }
    // the wrong set-ups still fault, standing still and after the same opening turn
    const char* names[] = {"", "field turned 180", "mirrored left-right", "mirrored front-back"};
    for (const int frame : {1, 2, 3}) {
        for (const bool turnFirst : {false, true}) {
            Robot r(Config(), 20, 30, 45);
            r.gpsFrame = frame;
            r.start(Mode::On);
            std::uint32_t at = 0;
            if (turnFirst) {
                for (int done = 0; done < 600 && !at; done += 10) {
                    r.tick(10, 0, 300);
                    if (r.status().fault != Fault::None) at = done + 10;
                }
            }
            const std::uint32_t still = at ? 0 : msToFault(r, 3000);
            const Status s = r.status();
            char what[160];
            std::snprintf(what, sizeof what, "%s%s: fault HeadingDisagree", names[frame],
                          turnFirst ? ", after an opening 300 deg/s turn" : ", standing still");
            check(s.fault == Fault::HeadingDisagree, what);
            check(r.appliedXIn == 0 && r.appliedYIn == 0, "  the pose was never touched");
            std::printf("    (%s: faulted %u ms after %s)\n", names[frame], unsigned(at ? at : still),
                        at ? "the turn began" : (turnFirst ? "the turn ended" : "switching on"));
            if (!turnFirst) {
                std::snprintf(what, sizeof what, "  within 1 s of switching on (%u ms)", unsigned(still));
                check(still > 0 && still <= 1000, what);
            }
        }
    }
}

static void testCheck7CountsGateFailures() {
    std::printf("check 7 restarts on every reading that passes it, gate failures included (reviewer test a)\n");
    Robot r = readyRobot(rev2Limits(), 0, 0, 0);
    r.run(2000);
    const Status s0 = r.status();
    r.gpsOffsetXIn = 8; // a shove the wheels missed: every reading from now fails the gate
    std::vector<std::uint32_t> failTimes;
    std::uint32_t relocAt = 0;
    std::uint32_t lastFails = s0.droppedByGate;
    for (int done = 0; done < 1500 && !relocAt; done += 10) {
        r.tick(10);
        const Status s = r.status();
        if (s.droppedByGate != lastFails) {
            failTimes.push_back(r.t);
            lastFails = s.droppedByGate;
        }
        if (s.relocalisations != s0.relocalisations) relocAt = r.t;
    }
    std::uint32_t smallestGap = 100000;
    for (size_t i = 1; i < failTimes.size(); i++) smallestGap = std::min(smallestGap, failTimes[i] - failTimes[i - 1]);
    char what[160];
    std::snprintf(what, sizeof what, "readings come every 40 ms, gate failures at most one per minPeriodMs (smallest gap %u ms)",
                  unsigned(smallestGap));
    check(failTimes.size() >= 2 && smallestGap >= 100, what);
    std::snprintf(what, sizeof what, "  a relocalisation needs 4 of them: at least 300 ms after the first (%u ms)",
                  unsigned(relocAt && !failTimes.empty() ? relocAt - failTimes.front() : 0));
    check(relocAt && !failTimes.empty() && relocAt - failTimes.front() >= 300, what);
}

static void testSideCameras() {
    std::printf("cameras facing right (yaw 90) and left (yaw 270): heading check and dead-zone wall (reviewer test b)\n");
    for (const float yaw : {90.0f, 270.0f}) {
        Config c;
        c.mountYawDeg = yaw;
        // heading: the sensor reports the robot's heading + yaw; a healthy one agrees, so no fault and readings used
        {
            Robot r(c, 0, 0, 30);
            r.start(Mode::On);
            r.run(3000);
            const Status s = r.status();
            char what[120];
            std::snprintf(what, sizeof what, "yaw %.0f, healthy sensor: no fault, readings accepted", double(yaw));
            check(s.fault == Fault::None && s.accepted > 5, what);
            checkCount(s.droppedByCheck[CHECK_HEADING], 0, "  none dropped by check 5");
        }
        // dead zone: 17 in from the wall the camera faces (x = +72 for yaw 90, x = -72 for 270), facing +y
        {
            const double x = yaw == 90.0f ? 55 : -55;
            Robot r(c, x, 0, 0);
            r.start(Mode::On);
            r.run(2000);
            const Status s = r.status();
            char what[120];
            std::snprintf(what, sizeof what, "yaw %.0f, 17 in from the wall it faces: every reading in the dead zone",
                          double(yaw));
            check(s.droppedByCheck[CHECK_DEAD_ZONE] > 20 && s.accepted == 0 && s.startSamplesSeen == 0, what);
        }
        // and 17 in from the wall behind it: not the dead zone
        {
            const double x = yaw == 90.0f ? -55 : 55;
            Robot r(c, x, 0, 0);
            r.start(Mode::On);
            r.run(2000);
            char what[120];
            std::snprintf(what, sizeof what, "yaw %.0f, 17 in from the wall behind it: readings used", double(yaw));
            check(r.status().droppedByCheck[CHECK_DEAD_ZONE] == 0 && r.status().accepted > 3, what);
        }
    }
}

static void testStep3EntryAtOddLatency() {
    std::printf("step 3 with latencyMs 25: the entry closest to 25 ms old, a tie going to the newer (reviewer test c)\n");
    // entries are 10 ms apart, so 20 and 30 ms are equally close: the 20 ms one is used. A GPS that really is
    // 20 ms late then gives zero innovation at 5.5 in/s; the 30 ms entry would give 5.5 * 0.01 = 0.055 in
    Config c;
    c.latencyMs = 25;
    Robot r(c, -30, 10, 90);
    r.gpsDelayMs = 20;
    r.gpsHeadingWiggleDeg = 0;
    r.start(Mode::On);
    double largest = 0;
    std::uint32_t seen = 0;
    for (int i = 0; i < 500; i++) {
        const Status before = r.status();
        r.tick(10, 5.5, 0);
        const Status after = r.status();
        if (after.startSamplesSeen + after.accepted + after.droppedByGate !=
            before.startSamplesSeen + before.accepted + before.droppedByGate) {
            largest = std::fmax(largest, std::hypot(after.lastInnovationXIn, after.lastInnovationYIn));
            seen++;
        }
    }
    char what[120];
    std::snprintf(what, sizeof what, "  %u readings, largest innovation %.4f in (the 30 ms entry: 0.055)", unsigned(seen),
                  largest);
    check(seen >= 20 && largest < 0.01, what);
}

static void testRelocalisationMeanOverLastSamples() {
    std::printf("relocalisation: the mean and the agreement are over the last relocSamples failures (reviewer test d)\n");
    Robot r = readyRobot(rev2Limits(), 0, 0, 0);
    r.gpsFrozen = true;
    feed(r, 0, -8); // the first failure disagrees with the rest
    for (int i = 0; i < 3; i++) feed(r, 8, 0);
    checkCount(r.status().relocalisations, 0, "4 failures, the first 11 in from the others: no relocalisation");
    feed(r, 8, 0);
    const Status s = r.status();
    checkCount(s.relocalisations, 1, "a 5th: the last 4 agree, relocalised");
    checkNear(s.decidedXIn, 8, 0.05, "  on their mean, x");
    checkNear(s.decidedYIn, 0, 0.05, "  and y (the first failure is left out)");
}

static void testDeadZoneNotJudged() {
    std::printf("dead-zone readings are not judged for the heading fault (reviewer test e)\n");
    // a rear camera 10 in from the wall behind it, the GPS heading 180 off: every reading is dropped by check 4
    Robot r(Config(), 0, -62, 0);
    r.gpsFrozen = true;
    r.start(Mode::On);
    r.tick(10);
    r.run(100);
    for (int i = 0; i < 12; i++) feed(r, 0, 0, 40, 180);
    const Status s = r.status();
    check(s.fault == Fault::None, "12 readings in the dead zone with the heading 180 off: no fault");
    checkCount(s.headingSamplesSeen, 0, "  none judged");
    check(s.droppedByCheck[CHECK_DEAD_ZONE] >= 12, "  all dropped by check 4");
}

static void testRestartKeepsEarlierCorrections() {
    std::printf("Revision 3: setPose (a restart, as setMode does) keeps earlier corrections; a later fault feeds out only its own\n");
    // the reviewer's case: odometry 3 in off, the GPS right; after 15 s about 3 in is fed in. The routine sets
    // the pose (here: to the truth) and the PROS layer restarts the filter (core.setMode(On)). The GPS then
    // lies by 30 in: RelocTooFar or a start-check fault. The unwinding must not take the old 3 in out of the new pose
    Robot r(Config(), 0, 0, 0);
    r.odomX = 3;
    r.start(Mode::On);
    r.run(15000);
    const double fedBefore = r.appliedXIn;
    check(fedBefore < -2.0, "  (about 3 in fed in before the setPose)");
    // setPose(truth) and the restart lemlib::setPose does
    r.odomX = float(r.trueX);
    r.odomY = float(r.trueY);
    r.core.setMode(Mode::On);
    const Status fresh = r.status();
    check(fresh.appliedXIn == 0 && fresh.decidedXIn == 0 && fresh.pendingXIn == 0 && !fresh.startCheckPassed,
          "after the restart: nothing fed in, decided or pending; the start check runs again");
    r.run(5000);
    r.gpsOffsetXIn = 30;
    r.run(8000);
    const Status s = r.status();
    check(s.fault != Fault::None, "the GPS then lies by 30 in: fault");
    char what[140];
    std::snprintf(what, sizeof what, "  8 s later the pose is the setPose one (off by %.3f in; the old rule left about 3)",
                  r.poseErrorIn());
    check(r.poseErrorIn() < 0.05, what);
}

static void testConfigChecks() {
    std::printf("Revision 3: checking the numbers\n");
    check(checkConfig(Config()) == 0, "the design's numbers: no problem");
    check(!latencyRefused(Config()), "  latencyMs 60 is not refused");
    char text[160];
    // one bit per number outside its range, and the sentence names it
    struct Bad {
            const char* name;
            void (*spoil)(Config&);
    } bads[] = {
        {"relocSamples", [](Config& c) { c.relocSamples = 1; }},
        {"maxSensorErrorIn", [](Config& c) { c.maxSensorErrorIn = 0.05f; }}, // metres by mistake
        {"sigmaMaxIn", [](Config& c) { c.sigmaMaxIn = 12; }},
        {"maxTotalCorrectionIn", [](Config& c) { c.maxTotalCorrectionIn = 30; }},
        {"minViewIn", [](Config& c) { c.minViewIn = 0; }},
        {"headingGateDeg", [](Config& c) { c.headingGateDeg = 180; }},
        {"startHeadingSamples", [](Config& c) { c.startHeadingSamples = 0; }},
        {"staleMs", [](Config& c) { c.staleMs = 20; }},
        {"maxSpeedInPerS", [](Config& c) { c.maxSpeedInPerS = 0; }},
        {"slewInPerS", [](Config& c) { c.slewInPerS = 50; }},
        {"maxStepIn", [](Config& c) { c.maxStepIn = 0; }},
        {"relocMaxCount", [](Config& c) { c.relocMaxCount = 99; }},
        {"sigmaGpsIn", [](Config& c) { c.sigmaGpsIn = NAN; }},
        {"latencyMs", [](Config& c) { c.latencyMs = 600; }},
    };
    for (const Bad& b : bads) {
        Config c;
        b.spoil(c);
        const std::uint32_t problems = checkConfig(c);
        int bits = 0, which = -1;
        for (int i = 0; i < 32; i++)
            if (problems & (std::uint32_t(1) << i)) bits++, which = i;
        formatConfigProblem(text, sizeof text, which);
        char what[220];
        std::snprintf(what, sizeof what, "%s wrong: exactly one problem, named (\"%s\")", b.name, text);
        check(bits == 1 && std::string_view(text).find(b.name) != std::string_view::npos, what);
    }
    Config ok;
    ok.maxSensorErrorIn = 2;
    check(checkConfig(ok) == 0, "maxSensorErrorIn 2 in: fine (0 = off is fine too)");
    Config lat;
    lat.latencyMs = MAX_LATENCY_MS;
    check(!latencyRefused(lat) && checkConfig(lat) == 0, "latencyMs 580 (the most the history serves): accepted");
    lat.latencyMs = MAX_LATENCY_MS + 1;
    check(latencyRefused(lat), "latencyMs 581: refused");
    lat.latencyMs = NAN;
    check(latencyRefused(lat), "latencyMs NaN: refused");
    // what refusing guards against: at 600 ms no reading is ever used, without a fault
    {
        Config c;
        c.latencyMs = 600;
        Robot r(c, 0, 0, 0);
        r.start(Mode::On);
        r.run(3000);
        check(r.status().accepted == 0 && r.status().startSamplesSeen == 0 && r.status().droppedNoHistory > 20 &&
                  r.status().fault == Fault::None,
              "  (latencyMs 600 in the filter itself: every reading 'no history', no fault)");
    }
    // the wall numbers
    check(checkWallConfig(WallConfig()) == 0, "the wall design's numbers: no problem");
    WallConfig w;
    w.planeIn[2] = 1784; // millimetres by mistake
    std::uint64_t wp = checkWallConfig(w);
    formatWallConfigProblem(text, sizeof text, NUM_WALL_CONFIG_PROBLEMS - 1);
    check(wp == (std::uint64_t(1) << (NUM_WALL_CONFIG_PROBLEMS - 1)) && std::string_view(text).find("planeIn") != std::string_view::npos,
          "a wall plane in mm: one problem, planeIn");
    WallConfig w2;
    w2.stillWindowMs = 1000;
    w2.steadyCount = 40;
    wp = checkWallConfig(w2);
    int bits = 0;
    for (int i = 0; i < 64; i++)
        if (wp & (std::uint64_t(1) << i)) bits++;
    check(bits == 2, "stillWindowMs longer than the history and steadyCount over 16: two problems");
}

int main() {
    testOffChangesNothing();
    testCheckFinite();
    testCheckInField();
    testCheckStale();
    testCheckDeadZone();
    testCheckHeading();
    testCheckSlow();
    testCheckRate();
    testCheckOrder();
    testRayCast();
    testDelay();
    testGainArithmetic();
    testGate();
    testFeedIn();
    testStartCheck();
    testRelocalise();
    testFaultLatches();
    testShadow();
    testNoSensor();
    testNoHistory();
    testBrokenPose();
    testStepLimit();
    testTotalLimit();
    testUnwind();
    testRelocalisationR2();
    testStartSigma();
    testHeadingFault();
    testCheckSlowSecondWindow();
    testCheck5b();
    testDiagnostics();
    testHeadingFaultOnlyWhenSlow();
    testCheck7CountsGateFailures();
    testSideCameras();
    testStep3EntryAtOddLatency();
    testRelocalisationMeanOverLastSamples();
    testDeadZoneNotJudged();
    testRestartKeepsEarlierCorrections();
    testConfigChecks();
    std::printf("\n%d checks, %d failed: %s\n", checksRun, checksFailed, checksFailed == 0 ? "ALL PASSED" : "FAILED");
    return checksFailed == 0 ? 0 : 1;
}
