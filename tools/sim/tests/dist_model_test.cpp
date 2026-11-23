// Checks of the simulated distance sensor (dist_model.hpp) against hand-computed numbers, and its repeatability.
//
// Build and run: `make -C tools/sim unit` builds it with bench/dist_model.cpp and runs it with the filter's tests.
// By hand, from tools/sim (on this laptop the default macOS SDK does not link, so pick one that does, as
// tools/sim/Makefile explains):
//
//   clang++ -std=c++20 -O2 -ffp-contract=off -Wall -Wextra \
//       -isysroot /Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk \
//       -Ibench bench/dist_model.cpp tests/dist_model_test.cpp -o dist_model_test && ./dist_model_test
//
// The expected numbers were worked out by hand (closed-form geometry, not this model's ray casting) and are
// written in the comments. The frame is the design's: inches from the field centre, +x toward the 90 degree wall,
// +y toward the 0 degree wall, heading clockwise from +y. Walls at 70.25 in unless a test says otherwise.
// Sensor 0 by default: facing back, right 0, forward -8.5. Sensor 1: facing right, right 8.5, forward 0.
//
// Prints one line per test and one per failed check; exits 1 if anything failed.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "dist_model.hpp"

using namespace sim;

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

static void checkInt(long got, long want, const char* what) {
    checksRun++;
    if (got != want) {
        checksFailed++;
        std::printf("  FAIL: %s: got %ld, want %ld\n", what, got, want);
    }
}

static const double DEG = M_PI / 180.0;
static const double MM = 25.4;

// A model that makes a new reading at every update (period 0), so a static test needs one update per reading
static DistModel everyUpdate() {
    DistModel m;
    m.p.period_ms = 0;
    return m;
}

// one update at time t (then t moves on 1 ms), and what the sensor reads
static DistReading readAt(DistModel& m, std::uint32_t& t, double x, double y, double headingDeg, int sensor = 0) {
    m.update(t, x, y, headingDeg * DEG);
    t++;
    return m.read(sensor);
}

static void show(const char* label, const DistReading& r, const DistOutputInfo& in) {
    std::printf("    %-34s mm=%-5d conf=%-3d size=%-4d state=%d centre=%.4f nearest=%.4f model=%.4f inc=%.2f\n", label,
                int(r.mm), int(r.confidence), int(r.objectSize), int(in.state), in.centreIn, in.nearestIn, in.modelIn,
                in.incidenceDeg);
}

// ------------------------------------------------------------------------------------------------ the tests

// T1. Square to a wall. Robot (0, -50) heading 0: the back sensor's face is at (0, -58.5), 11.75 in from the
// -y wall = 298.45 mm -> 298. Confidence 63 - 10 * 0.298 = 60.02 -> 60. A wall fills the cone: size 100.
static void testSquareWall() {
    std::printf("T1  square to a wall\n");
    DistModel m = everyUpdate();
    std::uint32_t t = 0;
    const DistReading r = readAt(m, t, 0, -50, 0);
    show("back sensor, 11.75 in", r, m.info(0));
    checkInt(r.mm, 298, "T1 mm");
    checkInt(r.confidence, 60, "T1 confidence");
    checkInt(r.objectSize, 100, "T1 object size");
    checkInt(m.info(0).state, DIST_WALL, "T1 state");
    checkNear(m.info(0).centreIn, 11.75, 1e-9, "T1 centre range");
    checkNear(m.info(0).nearestIn, 11.75, 1e-9, "T1 nearest range");
    checkInt(m.info(0).centreIndex, 2, "T1 wall met is wall 2 (-y)");
    checkNear(m.info(0).incidenceDeg, 0, 1e-6, "T1 incidence");
}

// T2. A wall at an angle, the three cone models. Robot (0, -50) heading 10: the sensor's face is at
// (-1.47601, -58.37087), D = 11.879134 in from the wall, 10 degrees off square (inside the 12 degree cone).
//   centre ray  D / cos 10            = 12.062389 in = 306.38 mm -> 306
//   nearest     D                     = 11.879134 in = 301.73 mm -> 302
//   blend 0.5   (centre + nearest)/2  = 11.970761 in = 304.06 mm -> 304
// Confidence 63 - 10 * 0.30x - 0.5 * 10 = 54.9x -> 55 in all three.
static void testAngledWall() {
    std::printf("T2  wall 10 degrees off square, three cone models\n");
    const double wantIn[3] = {12.062388890686337, 11.879134099396232, 11.970761495041284};
    const long wantMm[3] = {306, 302, 304};
    const char* names[3] = {"centre ray", "nearest point", "blend"};
    for (int model = 0; model < 3; model++) {
        DistModel m = everyUpdate();
        m.p.cone_model = model;
        std::uint32_t t = 0;
        const DistReading r = readAt(m, t, 0, -50, 10);
        show(names[model], r, m.info(0));
        checkNear(m.info(0).modelIn, wantIn[model], 1e-9, "T2 model range");
        checkInt(r.mm, wantMm[model], "T2 mm");
        checkInt(r.confidence, 55, "T2 confidence");
        checkNear(m.info(0).incidenceDeg, 10, 1e-9, "T2 incidence");
        checkInt(m.info(0).state, DIST_WALL, "T2 state (one wall: plain)");
    }
}

// T3. 20 degrees off square, outside the 12 degree cone: the nearest point is the cone's edge, 8 degrees off.
// D = 12.262613; centre D / cos 20 = 13.049600 -> 331 mm; nearest D / cos 8 = 12.383124 -> 315; blend 12.716362 -> 323.
static void testWideAngle() {
    std::printf("T3  wall 20 degrees off square (outside the cone)\n");
    const double wantIn[3] = {13.049599892637218, 12.38312443912593, 12.716362165881574};
    const long wantMm[3] = {331, 315, 323};
    for (int model = 0; model < 3; model++) {
        DistModel m = everyUpdate();
        m.p.cone_model = model;
        std::uint32_t t = 0;
        const DistReading r = readAt(m, t, 0, -50, 20);
        show(model == 0 ? "centre ray" : model == 1 ? "nearest point" : "blend", r, m.info(0));
        checkNear(m.info(0).modelIn, wantIn[model], 1e-9, "T3 model range");
        checkInt(r.mm, wantMm[model], "T3 mm");
    }
}

// T4. The cone is wider close up. Heading 17.5, nearest-point model.
//   D = 7 in: centre 7.3397 in = 186.4 mm, under 200 mm, so 18 degrees: 17.5 is inside, nearest = D = 7.
//   D = 9 in: centre 239.7 mm, so 12 degrees: nearest = 9 / cos 5.5 = 9.041626.
static void testCloseCone() {
    std::printf("T4  wider cone below 200 mm\n");
    DistModel m = everyUpdate();
    m.p.cone_model = 1;
    std::uint32_t t = 0;
    readAt(m, t, 0, -55.14340591864007, 17.5);
    std::printf("    D=7: half=%.1f nearest=%.6f   ", m.info(0).coneHalfDeg, m.info(0).nearestIn);
    checkNear(m.info(0).coneHalfDeg, 18, 0, "T4 half-angle close");
    checkNear(m.info(0).nearestIn, 7.0, 1e-9, "T4 nearest close");
    readAt(m, t, 0, -53.14340591864007, 17.5);
    std::printf("D=9: half=%.1f nearest=%.6f\n", m.info(0).coneHalfDeg, m.info(0).nearestIn);
    checkNear(m.info(0).coneHalfDeg, 12, 0, "T4 half-angle far");
    checkNear(m.info(0).nearestIn, 9.041625851860152, 1e-9, "T4 nearest far");
}

// T5. A corner in the cone. Robot (-68.25, -49.75) heading 0: the back sensor's face is 12 in from the -y wall and
// 2 in from the -x wall. The cone's edge ray (192 degrees) meets the -x wall at 2 / sin 12 = 9.619469 in.
//   centre 12 in -> 305 mm, state WALL; nearest 9.619469 -> 244, CORNER; blend 10.809734 = 274.57 -> 275, CORNER.
// The cone meets walls 2 and 3: mask 4 + 8 = 12.
static void testCorner() {
    std::printf("T5  corner in the cone\n");
    const double wantIn[3] = {12.0, 9.619468689488262, 10.80973434474413};
    const long wantMm[3] = {305, 244, 275};
    const int wantState[3] = {DIST_WALL, DIST_CORNER, DIST_CORNER};
    for (int model = 0; model < 3; model++) {
        DistModel m = everyUpdate();
        m.p.cone_model = model;
        std::uint32_t t = 0;
        const DistReading r = readAt(m, t, -68.25, -49.75, 0);
        show(model == 0 ? "centre ray" : model == 1 ? "nearest point" : "blend", r, m.info(0));
        checkNear(m.info(0).modelIn, wantIn[model], 1e-9, "T5 model range");
        checkInt(r.mm, wantMm[model], "T5 mm");
        checkInt(m.info(0).state, wantState[model], "T5 state");
        checkInt(m.info(0).coneMask, 12, "T5 cone mask");
        checkInt(m.info(0).nearIndex, 3, "T5 nearest point on wall 3");
    }
    DistModel m;
    checkInt(m.coneMaskAt(0, -68.25, -49.75, 0, 0, 12), 12, "T5 coneMaskAt");
    checkInt(m.coneMaskAt(0, -40, -49.75, 0, 0, 18), 4, "T5 coneMaskAt away from the corner");
    checkNear(m.modelRangeAt(0, -68.25, -49.75, 0, 0), 10.80973434474413, 1e-9, "T5 modelRangeAt");
}

// T6. Update rate. 30 a second with phase 0: new readings at 0, 34, 67, 100, 134, 167, 200 ms when update() is
// called every ms; in between a read repeats the last one. The robot drives along +y at 10 in/s from y = -50, so
// the range is (y - 8.5 + 70.25) in: 298.45, 307.09, 315.47, 323.85, 332.49, 340.87, 349.25 mm.
static void testUpdateRate() {
    std::printf("T6  update rate and hold\n");
    DistModel m; // the default period, 33.3 ms
    const std::uint32_t wantT[7] = {0, 34, 67, 100, 134, 167, 200};
    const long wantMm[7] = {298, 307, 315, 324, 332, 341, 349};
    int found = 0;
    bool heldOk = true;
    std::int32_t last = -1;
    std::printf("    new readings at:");
    for (std::uint32_t t = 0; t <= 200; t++) {
        m.update(t, 0, -50 + 10.0 * t / 1000.0, 0);
        const DistReading r = m.read(0);
        if (m.info(0).timeMs == t) {
            std::printf(" %u ms (%d)", unsigned(t), int(r.mm));
            if (found < 7) {
                checkInt(t, wantT[found], "T6 reading time");
                checkInt(r.mm, wantMm[found], "T6 reading");
            }
            found++;
        } else if (r.mm != last) {
            heldOk = false;
        }
        last = r.mm;
    }
    std::printf("\n");
    checkInt(found, 7, "T6 readings in 201 ms");
    check(heldOk, "T6 a read between readings repeats the last one");
    checkInt(m.info(0).count, 7, "T6 reading count");

    // period 0: a reading at every update
    DistModel e = everyUpdate();
    for (std::uint32_t t = 0; t <= 200; t++) e.update(t, 0, -50 + 10.0 * t / 1000.0, 0);
    checkInt(e.info(0).count, 201, "T6 period 0: a reading per update");

    // a phase: sensor 1 at phase 17 makes its second reading at 17, then at the first whole ms after
    // 17 + 33.33 = 50.33 (51) and 83.67 (84)
    DistModel ph;
    ph.p.sensor[1].enabled = 1;
    ph.p.sensor[1].phase_ms = 17;
    std::vector<std::uint32_t> times;
    for (std::uint32_t t = 0; t <= 90; t++) {
        ph.update(t, 0, 0, 0);
        if (ph.info(1).timeMs == t) times.push_back(t);
    }
    check(times.size() == 4 && times[0] == 0 && times[1] == 17 && times[2] == 51 && times[3] == 84,
          "T6 phase 17: readings at 0, 17, 51, 84");
}

// T7. Mounts. Sensor 1 facing right (right 8.5, forward 0, yaw 90):
//   robot (30, 10) heading 0: face at (38.5, 10), beam toward +x: 31.75 in = 806.45 mm -> 806
//   robot (30, 10) heading 90: right is -y, face at (30, 1.5), beam toward -y: 71.75 in = 1822.45 -> 1822, conf 45
// A front sensor (forward 8.5, yaw 0) at robot (0, 40) heading 0: 21.75 in -> 552; a left one (right -8.5, yaw 270)
// at robot (-40, 0) heading 0: 21.75 in -> 552.
static void testMounts() {
    std::printf("T7  mounts\n");
    DistModel m = everyUpdate();
    m.p.sensor[1].enabled = 1;
    std::uint32_t t = 0;
    DistReading r = readAt(m, t, 30, 10, 0, 1);
    show("right sensor, heading 0", r, m.info(1));
    checkInt(r.mm, 806, "T7 right sensor heading 0");
    checkInt(m.info(1).centreIndex, 1, "T7 meets wall 1 (+x)");
    r = readAt(m, t, 30, 10, 90, 1);
    show("right sensor, heading 90", r, m.info(1));
    checkInt(r.mm, 1822, "T7 right sensor heading 90");
    checkInt(r.confidence, 45, "T7 confidence at 1822 mm");
    checkNear(m.info(1).sensorX, 30, 1e-9, "T7 face x");
    checkNear(m.info(1).sensorY, 1.5, 1e-9, "T7 face y");

    DistModel f = everyUpdate();
    f.p.sensor[0].forward_in = 8.5;
    f.p.sensor[0].yaw_deg = 0;
    t = 0;
    checkInt(readAt(f, t, 0, 40, 0).mm, 552, "T7 front sensor");
    f.p.sensor[0].forward_in = 0;
    f.p.sensor[0].right_in = -8.5;
    f.p.sensor[0].yaw_deg = 270;
    checkInt(readAt(f, t, -40, 0, 0).mm, 552, "T7 left sensor");
    std::printf("    front sensor 552 mm, left sensor 552 mm\n");
}

// T8. True wall planes per wall: the -y wall at 70.75 instead of 70.25 reads 0.5 in farther: 12.25 in = 311.15 -> 311.
// Moving the +y wall changes nothing for the back sensor.
static void testWallPlanes() {
    std::printf("T8  true wall planes\n");
    DistModel m = everyUpdate();
    m.p.wall[2].in = 70.75;
    m.p.wall[0].in = 60;
    std::uint32_t t = 0;
    const DistReading r = readAt(m, t, 0, -50, 0);
    show("-y wall at 70.75", r, m.info(0));
    checkInt(r.mm, 311, "T8 mm");
    double v = 0;
    check(m.p.set("wall3_in", 69.5) && m.p.get("wall3_in", v) && v == 69.5, "T8 wall3_in by key");
}

// T9. Noise grows with range; scale and offset; the slowly varying part.
//   noise_mm 2, noise_frac 0.01: at 298.45 mm sd = 2 + 2.98 = 4.98 mm (4.99 with the rounding to whole mm);
//   at 1568.45 mm (robot (0, 0) heading 0: 61.75 in) sd = 17.68 mm. 20000 readings each: within 3 percent.
//   scale_err 0.02, offset 5 mm: 298.45 * 1.02 + 5 = 309.42 -> 309.
//   slow_mm 3, tau 2 s, a reading every 100 ms: sd 3, correlation between neighbours exp(-0.1/2) = 0.9512; the
//   rounding to whole mm adds a white variance of 1/12, so what is measured is sd sqrt(9 + 1/12) = 3.014 and
//   correlation 0.9512 * 9 / 9.0833 = 0.9425. From 20000 readings the correlation is good to about 0.002 (one
//   standard deviation), so the tolerance is 0.01. (A 400000-reading run gives 0.9422.)
static void testNoise() {
    std::printf("T9  noise, scale, offset\n");
    const double poses[2][2] = {{0, -50}, {0, 0}};
    const double meanMm[2] = {298.45, 1568.45};
    const double wantSd[2] = {4.992852249299326, 17.68685595529441};
    for (int k = 0; k < 2; k++) {
        DistModel m = everyUpdate();
        m.p.noise_mm = 2;
        m.p.noise_frac = 0.01;
        double sum = 0, sum2 = 0;
        const int n = 20000;
        for (int i = 0; i < n; i++) {
            m.update(std::uint32_t(i), poses[k][0], poses[k][1], 0);
            const double v = m.read(0).mm;
            sum += v;
            sum2 += v * v;
        }
        const double mean = sum / n, sd = std::sqrt(sum2 / n - mean * mean);
        std::printf("    range %.2f mm: mean %.3f sd %.3f (want %.3f)\n", meanMm[k], mean, sd, wantSd[k]);
        checkNear(mean, meanMm[k], 4 * wantSd[k] / std::sqrt(double(n)) + 0.3, "T9 mean");
        checkNear(sd, wantSd[k], 0.03 * wantSd[k], "T9 sd");
    }
    DistModel s = everyUpdate();
    s.p.sensor[0].scale_err = 0.02;
    s.p.sensor[0].offset_mm = 5;
    std::uint32_t t = 0;
    checkInt(readAt(s, t, 0, -50, 0).mm, 309, "T9 scale and offset");

    DistModel w; // slow noise only, read every 100 ms
    w.p.period_ms = 0;
    w.p.slow_mm = 3;
    w.p.slow_tau_s = 2;
    const int n = 20000;
    std::vector<double> e(n);
    for (int i = 0; i < n; i++) {
        w.update(std::uint32_t(i) * 100, 0, -50, 0);
        e[i] = w.read(0).mm - 298.45; // the reading's error
    }
    double s2 = 0, s11 = 0;
    for (int i = 0; i < n; i++) s2 += e[i] * e[i];
    for (int i = 1; i < n; i++) s11 += e[i] * e[i - 1];
    const double sd = std::sqrt(s2 / n), corr = s11 / s2;
    std::printf("    slow noise: sd %.3f (want 3.014), neighbour correlation %.4f (want 0.9425)\n", sd, corr);
    checkNear(sd, std::sqrt(9 + 1.0 / 12), 0.2, "T9 slow sd");
    checkNear(corr, 0.951229424500714 * 9 / (9 + 1.0 / 12), 0.01, "T9 slow correlation");
}

// T10. The 20 mm limit. A face 0.5 in (12.7 mm) from the wall reads 0 (below_min_value), confidence 10, size -1.
// 0.8 in = 20.32 mm -> 20 is a reading. A face past the wall also reads 0.
static void testTooClose() {
    std::printf("T10 closer than 20 mm\n");
    DistModel m = everyUpdate();
    std::uint32_t t = 0;
    DistReading r = readAt(m, t, 0, -61.25, 0);
    show("face 0.5 in from the wall", r, m.info(0));
    checkInt(r.mm, 0, "T10 reads 0");
    checkInt(r.confidence, 10, "T10 confidence");
    checkInt(r.objectSize, -1, "T10 size");
    checkInt(m.info(0).state, DIST_TOO_CLOSE, "T10 state");
    r = readAt(m, t, 0, -70.25 + 0.8 + 8.5, 0);
    checkInt(r.mm, 20, "T10 0.8 in reads 20");
    checkInt(m.info(0).state, DIST_WALL, "T10 0.8 in state");
    r = readAt(m, t, 0, -62.0, 0); // face at -70.5, past the wall
    checkInt(r.mm, 0, "T10 face past the wall");
    checkInt(m.info(0).state, DIST_TOO_CLOSE, "T10 past the wall state");
    m.p.below_min_value = 20;
    checkInt(readAt(m, t, 0, -61.25, 0).mm, 20, "T10 below_min_value 20");
}

// T11. Nothing in range: 9999. Robot (0, -50) heading 180: the back sensor looks at the +y wall from y = -41.5:
// 111.75 in = 2838 mm -> 9999, confidence 0, size -1. 78.75 in = 2000.25 mm -> 2000 is a reading; 78.8 in -> 9999.
static void testOutOfRange() {
    std::printf("T11 nothing in range\n");
    DistModel m = everyUpdate();
    std::uint32_t t = 0;
    DistReading r = readAt(m, t, 0, -50, 180);
    show("far wall, 2838 mm", r, m.info(0));
    checkInt(r.mm, 9999, "T11 9999");
    checkInt(r.confidence, 0, "T11 confidence");
    checkInt(r.objectSize, -1, "T11 size");
    checkInt(m.info(0).state, DIST_NO_RETURN, "T11 state");
    checkInt(m.info(0).noReturn, DIST_NR_OUT_OF_RANGE, "T11 reason");
    r = readAt(m, t, 0, -17.0, 180); // face at y = -8.5: 78.75 in
    checkInt(r.mm, 2000, "T11 2000 mm is a reading");
    r = readAt(m, t, 0, -17.05, 180); // 78.8 in = 2001.5 mm
    checkInt(r.mm, 9999, "T11 2002 mm is not");
}

// T12. Confidence: 48 in square = 1219 mm: 63 - 12.19 = 50.8 -> 51. 7 in (178 mm): 10. conf_model 0: 63.
static void testConfidence() {
    std::printf("T12 confidence\n");
    DistModel m = everyUpdate();
    std::uint32_t t = 0;
    DistReading r = readAt(m, t, 0, -70.25 + 48 + 8.5, 0);
    checkInt(r.mm, 1219, "T12 48 in");
    checkInt(r.confidence, 51, "T12 confidence at 48 in");
    r = readAt(m, t, 0, -70.25 + 7 + 8.5, 0);
    checkInt(r.confidence, 10, "T12 confidence below 200 mm");
    m.p.conf_model = 0;
    r = readAt(m, t, 0, -70.25 + 48 + 8.5, 0);
    checkInt(r.confidence, 63, "T12 conf_model 0");
    m.p.conf_model = 1;
    m.p.conf_sd = 4;
    int lo = 99, hi = -1;
    for (int i = 0; i < 2000; i++) {
        const int c = readAt(m, t, 0, -50, 0).confidence;
        lo = std::min(lo, c);
        hi = std::max(hi, c);
    }
    std::printf("    48 in: 51; 7 in: 10; conf_model 0: 63; with conf_sd 4 at 11.75 in: %d to %d (centre 60)\n", lo, hi);
    check(lo < 60 && hi == 63 && lo >= 40, "T12 conf_sd spreads it, clamped at 63");
}

// T13. Objects. Robot (0, -40) heading 0: face at (0, -48.5), the wall 21.75 in away (552 mm).
//   circle (0, -60) r 1: near side 10.5 in away = 266.7 -> 267, state OBJECT
//   a circle there only from 1000 to 2000 ms: 552 at 500 ms, 267 at 1500 ms, 552 at 2500 ms
//   circle (2.5, -60) r 1: its centre is 12.26 degrees off the beam; only the cone's edge (12 degrees) meets it, at
//   10.769957 in. centre ray 552 (WALL), nearest 273.56 -> 274 (OBJECT), blend 16.259978 = 413.0 -> 413 (OBJECT)
static void testObjects() {
    std::printf("T13 objects\n");
    {
        DistModel m = everyUpdate();
        m.p.set("obj0_x", 0);
        m.p.set("obj0_y", -60);
        m.p.set("obj0_r", 1);
        std::uint32_t t = 0;
        const DistReading r = readAt(m, t, 0, -40, 0);
        show("circle on the centre ray", r, m.info(0));
        checkInt(r.mm, 267, "T13 fixed circle");
        checkInt(m.info(0).state, DIST_OBJECT, "T13 state");
    }
    {
        DistModel m = everyUpdate();
        m.p.obj[3] = {0, -60, 1, 1000, 2000};
        std::uint32_t t = 500;
        const long a = readAt(m, t, 0, -40, 0).mm;
        t = 1500;
        const long b = readAt(m, t, 0, -40, 0).mm;
        t = 2500;
        const long c = readAt(m, t, 0, -40, 0).mm;
        std::printf("    timed circle: %ld at 500 ms, %ld at 1500 ms, %ld at 2500 ms\n", a, b, c);
        checkInt(a, 552, "T13 before it appears");
        checkInt(b, 267, "T13 while there");
        checkInt(c, 552, "T13 after it goes");
    }
    const long wantMm[3] = {552, 274, 413};
    const int wantState[3] = {DIST_WALL, DIST_OBJECT, DIST_OBJECT};
    const double wantIn[3] = {21.75, 10.769956570630617, 16.25997828531531};
    for (int model = 0; model < 3; model++) {
        DistModel m = everyUpdate();
        m.p.cone_model = model;
        m.p.obj[0] = {2.5, -60, 1, 0, -1};
        std::uint32_t t = 0;
        const DistReading r = readAt(m, t, 0, -40, 0);
        show(model == 0 ? "edge circle, centre ray" : model == 1 ? "edge circle, nearest" : "edge circle, blend", r,
             m.info(0));
        checkNear(m.info(0).modelIn, wantIn[model], 1e-9, "T13 edge circle range");
        checkInt(r.mm, wantMm[model], "T13 edge circle mm");
        checkInt(m.info(0).state, wantState[model], "T13 edge circle state");
        check((m.info(0).coneMask & DIST_CONE_OBJECT) != 0, "T13 edge circle in the cone mask");
    }
}

// T14. The object put in the beam while nearly stopped (W4).
//   Mode 1, distance 8 in: robot still at (0, -50). Readings at 0 and 34 ms show the wall (298); "nearly stopped"
//   starts after 50 ms, so the reading at 67 ms shows the object: 8 in = 203.2 -> 203, state STOP_OBJECT. Moving
//   at 10 in/s it is gone at the next reading; still again, a new one.
//   Mode 2, 2 in in front of the wall: 11.75 - 2 = 9.75 in = 247.65 -> 248.
//   Drawn between 6 and 20 in at each stop, held through the stop.
//   Turning in place at 20 deg/s is not "stopped".
static void testStopObject() {
    std::printf("T14 object in the beam while stopped\n");
    {
        DistModel m;
        m.p.stop_obj = 1;
        m.p.stop_obj_min_in = m.p.stop_obj_max_in = 8;
        long at66 = 0, at67 = 0;
        for (std::uint32_t t = 0; t <= 299; t++) {
            m.update(t, 0, -50, 0);
            if (t == 66) at66 = m.read(0).mm;
            if (t == 67) at67 = m.read(0).mm;
        }
        std::printf("    mode 1: %ld at 66 ms, %ld at 67 ms, state %d", at66, at67, int(m.info(0).state));
        checkInt(at66, 298, "T14 before the stop counts");
        checkInt(at67, 203, "T14 object at 8 in");
        checkInt(m.info(0).state, DIST_STOP_OBJECT, "T14 state");
        // moving at 10 in/s from 300 to 400 ms
        long moving = 0;
        bool sawObjectWhileMoving = false;
        for (std::uint32_t t = 300; t < 400; t++) {
            m.update(t, 0, -50 + 10.0 * (t - 299) / 1000.0, 0);
            if (t > 300 && m.info(0).state == DIST_STOP_OBJECT) sawObjectWhileMoving = true;
            moving = m.read(0).mm;
        }
        check(!sawObjectWhileMoving, "T14 gone while moving");
        std::printf(", moving %ld", moving);
        long again = 0;
        for (std::uint32_t t = 400; t < 600; t++) {
            m.update(t, 0, -49.0, 0);
            again = m.read(0).mm;
        }
        std::printf(", still again %ld\n", again);
        checkInt(again, 203, "T14 back at the next stop");
    }
    {
        DistModel m;
        m.p.stop_obj = 2;
        m.p.stop_obj_min_in = m.p.stop_obj_max_in = 2;
        for (std::uint32_t t = 0; t <= 200; t++) m.update(t, 0, -50, 0);
        std::printf("    mode 2: %d mm (want 248)\n", int(m.read(0).mm));
        checkInt(m.read(0).mm, 248, "T14 mode 2");
        checkNear(m.info(0).stopObjectDistIn, 2, 1e-12, "T14 mode 2 distance");
    }
    {
        DistModel m;
        m.reseed(7);
        m.p.stop_obj = 1; // 6 to 20 in, the defaults
        std::uint32_t t = 0;
        double lo = 99, hi = 0;
        bool heldOk = true, mmOk = true;
        // 30 in from the wall (robot y = -70.25 + 30 + 8.5), so every distance up to 20 in fits in front of it
        double x = -30;
        const double y = -31.75;
        for (int stop = 0; stop < 20; stop++) {
            for (int k = 0; k < 100; k++, t++) m.update(t, x += 0.03, y, 0); // 100 ms at 30 in/s along the wall
            double d = -1;
            for (int k = 0; k < 300; k++, t++) { // then 300 ms still
                m.update(t, x, y, 0);
                if (m.info(0).state != DIST_STOP_OBJECT) continue;
                if (d < 0) d = m.info(0).stopObjectDistIn;
                if (m.info(0).stopObjectDistIn != d) heldOk = false;
                if (m.read(0).mm != std::lround(d * MM)) mmOk = false;
            }
            lo = std::min(lo, d);
            hi = std::max(hi, d);
        }
        std::printf("    20 stops, distances drawn from %.2f to %.2f in\n", lo, hi);
        check(lo >= 6 && hi <= 20 && hi - lo > 5, "T14 drawn between 6 and 20 in");
        check(heldOk, "T14 held through each stop");
        check(mmOk, "T14 reading is the drawn distance");
    }
    {
        // the gap (stop_obj_gap_in 3): 11.75 in from the wall the draw's top is lowered to 11.75 - 3 = 8.75 in; at
        // 8 in from the wall even the bottom (6) does not fit under 8 - 3 = 5, so there is no object
        DistModel m;
        m.reseed(11);
        m.p.stop_obj = 1;
        std::uint32_t t = 0;
        double x = -30, lo = 99, hi = 0;
        for (int stop = 0; stop < 20; stop++) {
            for (int k = 0; k < 100; k++, t++) m.update(t, x += 0.03, -50, 0);
            for (int k = 0; k < 300; k++, t++) {
                m.update(t, x, -50, 0);
                if (m.info(0).state != DIST_STOP_OBJECT) continue;
                lo = std::min(lo, m.info(0).stopObjectDistIn);
                hi = std::max(hi, m.info(0).stopObjectDistIn);
            }
        }
        std::printf("    11.75 in from the wall, gap 3: distances drawn from %.2f to %.2f in (want within 6 to 8.75)\n", lo,
                    hi);
        check(lo >= 6 && hi <= 8.75 && hi - lo > 1.5, "T14 gap lowers the top of the draw");
        bool sawObject = false;
        for (int k = 0; k < 300; k++, t++) {
            m.update(t, x, -70.25 + 8 + 8.5, 0);
            if (m.info(0).state == DIST_STOP_OBJECT) sawObject = true;
        }
        check(!sawObject && m.info(0).stopObjectDistIn == -1 && m.read(0).mm == std::lround(8 * MM),
              "T14 no object when it cannot fit");
    }
    {
        DistModel m;
        m.p.stop_obj = 1;
        bool saw = false;
        for (std::uint32_t t = 0; t <= 1000; t++) {
            m.update(t, 0, -50, 20.0 * t / 1000.0 * DEG);
            if (m.info(0).state == DIST_STOP_OBJECT) saw = true;
        }
        check(!saw, "T14 turning in place is not stopped");
    }
}

// T15. Clear walls (W5). The -y wall clear, robot (0, -50):
//   always through, 12 in behind: 23.75 in = 603.25 -> 603, state THROUGH, confidence 63 - 6.03 - 10 = 46.97 -> 47
//   never through: 298, state CLEAR_WALL, confidence 63 - 2.98 - 10 = 50.02 -> 50
//   dropout beyond 28 in: 30 in away -> 9999; 27 in away -> a reading
//   through 30 percent of the time: about 0.3 of 5000 readings
//   behind 12 + 0 to 10 in: every reading between 603 and 857
static void testClearWall() {
    std::printf("T15 clear walls\n");
    DistModel m = everyUpdate();
    m.p.wall[2].clear = 1;
    m.p.wall[2].through_prob = 1;
    m.p.wall[2].behind_in = 12;
    std::uint32_t t = 0;
    DistReading r = readAt(m, t, 0, -50, 0);
    show("always through, 12 in behind", r, m.info(0));
    checkInt(r.mm, 603, "T15 through");
    checkInt(m.info(0).state, DIST_THROUGH, "T15 through state");
    checkInt(r.confidence, 47, "T15 through confidence");
    m.p.wall[2].through_prob = 0;
    r = readAt(m, t, 0, -50, 0);
    show("never through", r, m.info(0));
    checkInt(r.mm, 298, "T15 clear wall");
    checkInt(m.info(0).state, DIST_CLEAR_WALL, "T15 clear wall state");
    checkInt(r.confidence, 50, "T15 clear wall confidence");
    m.p.wall[2].dropout_in = 28;
    r = readAt(m, t, 0, -70.25 + 30 + 8.5, 0);
    checkInt(r.mm, 9999, "T15 dropout beyond 28 in");
    checkInt(m.info(0).noReturn, DIST_NR_CLEAR_DROPOUT, "T15 dropout reason");
    r = readAt(m, t, 0, -70.25 + 27 + 8.5, 0);
    checkInt(r.mm, std::lround(27 * MM), "T15 27 in still reads");
    m.p.wall[2].dropout_in = 0;
    m.p.wall[2].through_prob = 0.3;
    int through = 0;
    for (int i = 0; i < 5000; i++) {
        readAt(m, t, 0, -50, 0);
        if (m.info(0).through) through++;
    }
    std::printf("    through_prob 0.3: %d of 5000 through\n", through);
    checkNear(through / 5000.0, 0.3, 0.03, "T15 through fraction");
    m.p.wall[2].through_prob = 1;
    m.p.wall[2].behind_spread_in = 10;
    int lo = 99999, hi = 0;
    for (int i = 0; i < 2000; i++) {
        const int v = readAt(m, t, 0, -50, 0).mm;
        lo = std::min(lo, v);
        hi = std::max(hi, v);
    }
    std::printf("    behind 12 to 22 in: readings %d to %d mm (want within 603 to 857)\n", lo, hi);
    check(lo >= 603 && hi <= 857 && hi - lo > 200, "T15 behind spread");
    m.p.wall[2].clear = 0;
    checkInt(readAt(m, t, 0, -50, 0).mm, 298, "T15 clear 0: options ignored");
}

// T16. A wall-mounted element: wall 2 (-y), x from -5 to 5, 3 in deep.
//   robot (0, -50): its face is 8.75 in away = 222.25 -> 222, state ELEMENT
//   robot (20, -50): plain wall 298
//   robot (7, -50): the element's side (x = 5) is 2 in to the side; the cone's edge meets it at 2 / sin 12 = 9.619469.
//   centre 298 (WALL); nearest 244 (ELEMENT); blend 10.684734 = 271.39 -> 271 (ELEMENT)
static void testElement() {
    std::printf("T16 wall-mounted element\n");
    DistModel m = everyUpdate();
    m.p.set("elem0_wall", 2);
    m.p.set("elem0_from_in", -5);
    m.p.set("elem0_to_in", 5);
    m.p.set("elem0_depth_in", 3);
    std::uint32_t t = 0;
    DistReading r = readAt(m, t, 0, -50, 0);
    show("in front of it", r, m.info(0));
    checkInt(r.mm, 222, "T16 element face");
    checkInt(m.info(0).state, DIST_ELEMENT, "T16 state");
    r = readAt(m, t, 20, -50, 0);
    checkInt(r.mm, 298, "T16 away from it");
    checkInt(m.info(0).coneMask & DIST_CONE_ELEMENT, 0, "T16 not in the cone away from it");
    const long wantMm[3] = {298, 244, 271};
    const int wantState[3] = {DIST_WALL, DIST_ELEMENT, DIST_ELEMENT};
    for (int model = 0; model < 3; model++) {
        m.p.cone_model = model;
        r = readAt(m, t, 7, -50, 0);
        show(model == 0 ? "2 in beside it, centre ray" : model == 1 ? "2 in beside it, nearest" : "2 in beside it, blend",
             r, m.info(0));
        checkInt(r.mm, wantMm[model], "T16 beside it");
        checkInt(m.info(0).state, wantState[model], "T16 beside it state");
    }
    checkInt(m.coneMaskAt(0, 7, -50, 0, 0, 12) & DIST_CONE_ELEMENT, DIST_CONE_ELEMENT, "T16 coneMaskAt sees it");
}

// T17. Delay. Driving +y at 10 in/s, latency 50 ms: the reading at 200 ms shows y(150) = -48.5: 13.25 in = 336.55
// -> 337 (without delay 349).
static void testLatency() {
    std::printf("T17 latency\n");
    DistModel m = everyUpdate();
    m.p.latency_ms = 50;
    for (std::uint32_t t = 0; t <= 200; t++) m.update(t, 0, -50 + 10.0 * t / 1000.0, 0);
    std::printf("    at 200 ms with 50 ms delay: %d (want 337)\n", int(m.read(0).mm));
    checkInt(m.read(0).mm, 337, "T17 delayed");
    m.p.latency_ms = 0;
    m.update(201, 0, -48.0, 0);
    checkInt(m.read(0).mm, 349, "T17 no delay");
}

// T18. Dropouts. dropout_incidence_deg 15: 20 degrees off square -> 9999 (TOO_SLANTED), 10 degrees -> 304.
// dropout_prob 0.25: about a quarter of 4000 readings are 9999.
static void testDropouts() {
    std::printf("T18 dropouts\n");
    DistModel m = everyUpdate();
    m.p.dropout_incidence_deg = 15;
    std::uint32_t t = 0;
    checkInt(readAt(m, t, 0, -50, 20).mm, 9999, "T18 too slanted");
    checkInt(m.info(0).noReturn, DIST_NR_TOO_SLANTED, "T18 reason");
    checkInt(readAt(m, t, 0, -50, 10).mm, 304, "T18 10 degrees reads");
    m.p.dropout_incidence_deg = 0;
    m.p.dropout_prob = 0.25;
    int dropped = 0;
    for (int i = 0; i < 4000; i++)
        if (readAt(m, t, 0, -50, 0).mm == 9999) dropped++;
    std::printf("    dropout_prob 0.25: %d of 4000\n", dropped);
    checkNear(dropped / 4000.0, 0.25, 0.03, "T18 random dropout fraction");
}

// T19. Object size. A wall fills the cone: 100. A circle r 0.5 at 11.5 in on the centre ray spans +-2.49 degrees:
// 9 of the 49 even rays (every 0.5 degree) meet it: 100 * 9 / 49 = 18.4 -> 18. A circle r 3 whose near side is
// 3.5 in away spans +-27.5 degrees, more than the 18 degree close cone: 100.
static void testObjectSize() {
    std::printf("T19 object size\n");
    DistModel m = everyUpdate();
    m.p.obj[0] = {0, -60, 0.5, 0, -1};
    std::uint32_t t = 0;
    const DistReading small = readAt(m, t, 0, -40, 0);
    checkInt(small.objectSize, 18, "T19 small circle");
    m.p.obj[0] = {0, -55, 3, 0, -1};
    const DistReading big = readAt(m, t, 0, -40, 0);
    checkInt(big.objectSize, 100, "T19 big close circle");
    checkNear(m.info(0).coneHalfDeg, 18, 0, "T19 close cone");
    std::printf("    small circle %d, big close circle %d\n", int(small.objectSize), int(big.objectSize));
}

// T20. Not fitted, and before the first update
static void testDisabled() {
    std::printf("T20 sensor not fitted\n");
    DistModel m;
    check(m.read(0).mm == 0 && m.read(0).confidence == 0 && m.read(0).objectSize == -1, "T20 before any update");
    m.update(0, 0, -50, 0);
    const DistReading r = m.read(1);
    check(r.mm == DIST_PROS_ERR && r.confidence == DIST_PROS_ERR && r.objectSize == DIST_PROS_ERR, "T20 PROS_ERR");
    checkInt(m.info(1).state, DIST_OFF, "T20 state");
    check(m.read(5).mm == DIST_PROS_ERR, "T20 no such sensor");
}

// ------------------------------------------------------------------------------------------------ repeatability

// a 3 s drive along a curve near the -y wall, with stops, every ms: the readings of both sensors
struct Trace {
        std::vector<DistReading> r[2];
        std::vector<double> model[2];
};

static Trace runPath(DistModel& m) {
    Trace tr;
    for (std::uint32_t t = 0; t < 3000; t++) {
        const double s = t < 1000 ? t / 1000.0 : t < 1500 ? 1.0 : t < 2500 ? 1.0 + (t - 1500) / 1000.0 : 2.0;
        const double x = -30 + 20 * s, y = -50 + 2 * std::sin(s * 3), h = 4 * std::sin(s * 2) * DEG;
        m.update(t, x, y, h);
        for (int i = 0; i < 2; i++) {
            tr.r[i].push_back(m.read(i));
            tr.model[i].push_back(m.info(i).modelIn);
        }
    }
    return tr;
}

static bool sameMm(const Trace& a, const Trace& b, int sensor) {
    for (size_t k = 0; k < a.r[sensor].size(); k++)
        if (a.r[sensor][k].mm != b.r[sensor][k].mm) return false;
    return true;
}

static bool sameAll(const Trace& a, const Trace& b) {
    for (int i = 0; i < 2; i++)
        for (size_t k = 0; k < a.r[i].size(); k++) {
            const DistReading &x = a.r[i][k], &y = b.r[i][k];
            if (x.mm != y.mm || x.confidence != y.confidence || x.objectSize != y.objectSize) return false;
            if (std::memcmp(&a.model[i][k], &b.model[i][k], sizeof(double)) != 0) return false;
        }
    return true;
}

static DistModel noisy(std::uint64_t seed) {
    DistModel m;
    m.reseed(seed);
    m.p.noise_mm = 3;
    m.p.noise_frac = 0.01;
    return m;
}

// T21. The same seed and options give exactly the same readings; another seed does not; and switching on one
// effect leaves every other effect's numbers alone, because every reading draws the same count of random numbers
static void testRepeatability() {
    std::printf("T21 repeatability and independent effects\n");
    auto everything = [](std::uint64_t seed) {
        DistModel m = noisy(seed);
        m.p.sensor[1].enabled = 1;
        m.p.slow_mm = 2;
        m.p.slow_frac = 0.005;
        m.p.conf_sd = 3;
        m.p.size_sd = 5;
        m.p.dropout_prob = 0.05;
        m.p.wall[2].clear = 1;
        m.p.wall[2].through_prob = 0.3;
        m.p.wall[2].behind_spread_in = 8;
        m.p.stop_obj = 1;
        m.p.latency_ms = 20;
        return m;
    };
    DistModel a1 = everything(42), a2 = everything(42), b = everything(43);
    const Trace ta1 = runPath(a1), ta2 = runPath(a2), tb = runPath(b);
    check(sameAll(ta1, ta2), "T21 same seed, every option on: identical, bit for bit");
    check(!sameMm(ta1, tb, 0), "T21 another seed differs");
    // reset() and reseed() bring a model that has run back to the start exactly
    a2.reset();
    a2.reseed(42);
    check(sameAll(ta1, runPath(a2)), "T21 reset() + reseed() after a run: identical again");

    DistModel base = noisy(5);
    const Trace t0 = runPath(base);
    {
        DistModel m = noisy(5);
        m.p.conf_sd = 4;
        m.p.size_sd = 6;
        check(sameMm(t0, runPath(m), 0), "T21 confidence and size noise leave mm alone");
    }
    {
        DistModel m = noisy(5);
        m.p.dropout_prob = 0.2;
        const Trace t1 = runPath(m);
        bool ok = true;
        int dropped = 0, total = 0;
        for (size_t k = 0; k < t1.r[0].size(); k++) {
            if (t1.r[0][k].mm == 9999) {
                dropped++;
            } else if (t1.r[0][k].mm != t0.r[0][k].mm) {
                ok = false;
            }
            total++;
        }
        check(ok, "T21 random dropouts leave the other readings alone");
        std::printf("    dropouts: %d of %d held reads are 9999\n", dropped, total);
    }
    {
        DistModel m = noisy(5);
        m.p.sensor[1].enabled = 1;
        check(sameMm(t0, runPath(m), 0), "T21 fitting sensor 1 leaves sensor 0 alone");
    }
    {
        DistModel m = noisy(5);
        m.p.wall[0].clear = 1; // the back sensor never sees wall 0
        m.p.wall[0].through_prob = 0.5;
        check(sameMm(t0, runPath(m), 0), "T21 a clear wall it does not see changes nothing");
    }
    {
        DistModel m = noisy(5);
        m.p.wall[2].clear = 1; // the wall it sees, never through: only the confidence drops
        check(sameMm(t0, runPath(m), 0), "T21 a clear wall that never lets through leaves mm alone");
    }
    {
        DistModel m = noisy(5);
        m.p.slow_tau_s = 0.3; // slow noise still off (slow_mm = slow_frac = 0): its time constant changes nothing
        check(sameMm(t0, runPath(m), 0), "T21 the slow noise's time constant alone changes nothing");
    }
}

// T22. Every option has a help line and a key; unknown keys are refused
static void testKeys() {
    std::printf("T22 options and help\n");
    const std::string help = DistModelParams::helpText();
    const std::vector<std::string> keys = DistModelParams::allKeys();
    int missingHelp = 0, refused = 0;
    for (const std::string& k : keys) {
        // numbered families are listed once as wallN_..., elemN_..., objN_...; sensor keys by number
        std::string shown = k;
        for (const char* fam : {"wall", "elem", "obj"}) {
            const size_t n = std::strlen(fam);
            if (k.compare(0, n, fam) == 0 && k.size() > n && std::isdigit(static_cast<unsigned char>(k[n]))) {
                size_t i = n;
                while (std::isdigit(static_cast<unsigned char>(k[i]))) i++;
                shown = std::string(fam) + "N" + k.substr(i);
            }
        }
        if (help.find(" " + shown + "=") == std::string::npos && help.find("  " + shown + "=") == std::string::npos)
            missingHelp++;
        DistModelParams p;
        double v = 0;
        if (!p.set(k, 1.5) || !p.get(k, v) || v != 1.5) refused++;
    }
    std::printf("    %zu keys, %d without a help line, %d refused by set/get\n", keys.size(), missingHelp, refused);
    checkInt(missingHelp, 0, "T22 every key has a help line");
    checkInt(refused, 0, "T22 every key can be set");
    DistModelParams p;
    for (const char* bad : {"s2_enabled", "obj16_x", "wall4_in", "elem8_wall", "bogus", "s0_", "s_yaw_deg", "stop_obj_x",
                            "wall0_inn", "s0_yaw"})
        check(!p.set(bad, 1), (std::string("T22 refuses ") + bad).c_str());
    const std::string js = p.toJson();
    check(js.find("\"s0_yaw_deg\":180") != std::string::npos, "T22 json has s0_yaw_deg");
    check(js.find("obj0_x") == std::string::npos, "T22 json leaves out unused objects");
    p.set("obj2_r", 1);
    check(p.toJson().find("\"obj2_r\":1") != std::string::npos, "T22 json has a used object");
}

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--help") == 0) {
        std::fputs(DistModelParams::helpText().c_str(), stdout);
        return 0;
    }
    testSquareWall();
    testAngledWall();
    testWideAngle();
    testCloseCone();
    testCorner();
    testUpdateRate();
    testMounts();
    testWallPlanes();
    testNoise();
    testTooClose();
    testOutOfRange();
    testConfidence();
    testObjects();
    testStopObject();
    testClearWall();
    testElement();
    testLatency();
    testDropouts();
    testObjectSize();
    testDisabled();
    testRepeatability();
    testKeys();
    std::printf("\n%d checks, %d failed\n", checksRun, checksFailed);
    return checksFailed == 0 ? 0 : 1;
}
