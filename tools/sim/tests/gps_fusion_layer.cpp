// Host test of the robot's half of src/lemlib/chassis/gpsFusion.cpp: configure(), configureWalls(), setMode(),
// mode(), status(), step(), the setPose hooks and the logging task's round (printStatus()) with its [GPSFUSE] and
// [WALLFIX] lines and messages. The stub pros::Task never runs a task, so the test calls printStatus() itself every
// 500 ms of its clock, as the logging task would. Built against the testbench's stub PROS
// headers (tools/sim/stubs), whose GPS and distance sensors fail every read the way PROS reports an unplugged
// sensor (infinity; PROS_ERR) unless a test installs a reading hook, and whose mutex can be made busy (as if
// another task held it) to test that nothing waits on it for long.
// The filter itself is tested in gps_fusion_unit.cpp; this only checks the plumbing around it.
//
//   make -C tools/sim unit        (runs this after gps_fusion_unit)

#include <unistd.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include "lemlib/chassis/gpsFusion.hpp"
#include "pros/distance.hpp"
#include "pros/gps.hpp"
#include "pros/rtos.hpp"

using namespace lemlib::gpsfusion;

// The stub rtos.h declares these; the testbench defines them in its simulated world. This test keeps its own clock
static std::uint32_t nowMs = 5000;

namespace pros::c {
void delay(const uint32_t milliseconds) { nowMs += milliseconds; }

uint32_t millis(void) { return nowMs; }
} // namespace pros::c

// The stub distance.hpp declares the reading hook; whoever links it defines it once (the testbench does it in its
// own files). Null: every distance sensor unplugged
namespace pros::sim_detail {
DistanceRead (*distanceReadHook)(int port) = nullptr;
// the stub pros::Task calls this for every task created; the testbench defines it in its world, this test counts
// the logging tasks
static int logTasksStarted = 0;
static int otherTasksStarted = 0;
void task_created(const char* name) {
    if (name && std::strcmp(name, "gpsfuse log") == 0) logTasksStarted++;
    else otherTasksStarted++;
}
} // namespace pros::sim_detail

static int checksRun = 0;
static int checksFailed = 0;

static void check(bool ok, const char* what) {
    checksRun++;
    if (!ok) {
        checksFailed++;
        std::printf("  FAIL: %s\n", what);
    }
}

// Run `what` with stdout going to a file, and return everything it printed
static std::string capture(const std::function<void()>& what) {
    const char* path = SIM_DIR "/build/unit/layer_stdout.txt";
    std::fflush(stdout);
    const int terminal = dup(1);
    if (!std::freopen(path, "w", stdout)) return "";
    what();
    std::fflush(stdout);
    dup2(terminal, 1);
    close(terminal);
    std::string printed;
    if (FILE* f = std::fopen(path, "r")) {
        char line[512];
        while (std::fgets(line, sizeof line, f)) printed += line;
        std::fclose(f);
    }
    return printed;
}

// Call step() every 10 ms for `ms`, like odometry, and printStatus() every 500 ms, like the logging task (its
// first round right after the first step), and return everything printed. Also reports whether every correction
// was zero. With logging false, printStatus() is never called: then step() alone must print nothing
static std::string runSteps(int ms, bool& allZero, bool logging = true) {
    allZero = true;
    return capture([&] {
        for (int done = 0; done < ms; done += 10) {
            nowMs += 10;
            const Correction c = step(1.5f, -2.5f, 0.3f);
            allZero = allZero && c.xIn == 0.0f && c.yIn == 0.0f;
            if (logging && done % 500 == 0) printStatus();
        }
    });
}

// What one round of the logging task prints now
static std::string logRound() {
    return capture([] { printStatus(); });
}

static int countLines(const std::string& text, const char* start) {
    int n = 0;
    size_t at = 0;
    while ((at = text.find(start, at)) != std::string::npos) {
        n++;
        at++;
    }
    return n;
}

static size_t longestLine(const std::string& text) {
    size_t longest = 0, at = 0;
    while (at < text.size()) {
        size_t end = text.find('\n', at);
        if (end == std::string::npos) end = text.size();
        longest = std::max(longest, end - at);
        at = end + 1;
    }
    return longest;
}

// A GPS that sees the robot exactly where step() says it is (1.5, -2.5, heading 0.3 rad), camera facing backward,
// with a new reading every 40 ms of the test's clock (the heading wiggles by 0.01 deg so each one is new)
static bool workingGps(std::uint8_t, pros::sim_detail::GpsReading& out) {
    out.xMetres = 1.5 / INCHES_PER_METRE;
    out.yMetres = -2.5 / INCHES_PER_METRE;
    out.headingDeg = 0.3 * 180 / 3.14159265358979 + 180 + ((nowMs / 40) % 2 ? 0.01 : -0.01);
    out.errorMetres = 0.02;
    return true;
}

static void checkNear(double got, double want, double tolerance, const char* what) {
    checksRun++;
    if (!(std::fabs(got - want) <= tolerance)) {
        checksFailed++;
        std::printf("  FAIL: %s: got %.9g, want %.9g (+-%g)\n", what, got, want, tolerance);
    }
}

// Distance sensor hooks. countingDistance counts every call (to show when nothing is read); wallAt483 is a
// sensor that reads 483 mm at full confidence, and notes which ports it was asked for
static int distanceReads = 0;
static pros::sim_detail::DistanceRead countingDistance(int) {
    distanceReads++;
    return {PROS_ERR, PROS_ERR, PROS_ERR};
}
static int portsSeen = 0;
static pros::sim_detail::DistanceRead wallAt483(int port) {
    portsSeen |= 1 << port;
    return {483, 63, 100};
}

int main() {
    std::printf("the robot's side of GPS fusion (stub PROS, unplugged GPS)\n");
    pros::Gps gps(7);
    check(mode() == Mode::Off, "boot default is Off");
    check(status().mode == Mode::Off, "status says Off");

    configure(&gps, Config(), true, true);
    check(mode() == Mode::Off, "configure does not switch it on");

    bool allZero = false;
    // Off: step does nothing, prints nothing
    std::string printed = runSteps(1000, allZero);
    check(allZero, "Off: zero corrections");
    check(printed.empty(), "Off: nothing printed");
    check(status().newReadings == 0, "Off: the sensor was not read");

    // Shadow: runs, reads the (failing) sensor, prints at most twice a second, never corrects
    setMode(Mode::Shadow);
    check(mode() == Mode::Shadow, "setMode(Shadow)");
    printed = runSteps(3000, allZero);
    const int lines = countLines(printed, "[GPSFUSE]");
    char what[160];
    std::snprintf(what, sizeof what, "3 s of Shadow: 6 or 7 [GPSFUSE] lines, at most twice a second (got %d)", lines);
    check(lines >= 6 && lines <= 7, what);
    check(countLines(printed, "mode=SHADOW") == lines, "every line says mode=SHADOW");
    check(countLines(printed, "fault=none") == lines, "every line says fault=none");
    check(printed.find("pose=1.5,-2.5,17.2") != std::string::npos, "the line shows the pose (x, y, heading in deg)");
    check(allZero, "Shadow: zero corrections");
    Status s = status();
    check(s.mode == Mode::Shadow, "status says Shadow");
    check(s.newReadings == 1, "an unplugged sensor is one reading (it never changes)");
    check(s.droppedByCheck[CHECK_FINITE] == 1, "and it was dropped by check 1");
    check(s.sensorFrozen, "and the sensor shows as frozen");
    std::printf("    first line: %s", printed.substr(0, printed.find('\n') + 1).c_str());

    // On: starts again from scratch
    setMode(Mode::On);
    check(mode() == Mode::On, "setMode(On)");
    check(status().newReadings == 0, "switching on resets the filter");
    printed = runSteps(1000, allZero);
    check(allZero, "On with an unplugged sensor: zero corrections");
    check(countLines(printed, "mode=ON") >= 2, "lines say mode=ON");

    // no sensor at all
    configure(nullptr, Config(), true, true);
    setMode(Mode::On);
    printed = runSteps(1000, allZero);
    check(allZero, "no sensor: zero corrections");
    check(status().newReadings == 0, "no sensor: no readings");

    // Off again: quiet
    setMode(Mode::Off);
    check(mode() == Mode::Off, "setMode(Off)");
    printed = runSteps(1000, allZero);
    check(allZero && printed.empty(), "Off again: nothing returned, nothing printed");

    // ---------------------------------------------------------------- revision 2: the line (R4)
    std::printf("the [GPSFUSE] line with a working sensor (R4)\n");
    pros::sim_detail::gpsReadHook = workingGps;
    configure(&gps, Config(), true, true);
    setMode(Mode::Shadow);
    printed = runSteps(3000, allZero);
    s = status();
    check(allZero, "Shadow with a working sensor: zero corrections");
    check(s.startCheckPassed && s.accepted > 5, "the start check passed and readings were accepted");
    check(printed.find("start=0/5") != std::string::npos, "the first line shows start=0/5");
    check(printed.find("start=ok") != std::string::npos, "later lines show start=ok");
    check(countLines(printed, " inn=") == countLines(printed, "[GPSFUSE]"), "every line has inn= (robot frame)");
    check(countLines(printed, " nis=") == countLines(printed, "[GPSFUSE]"), "every line has nis=");
    check(printed.find("drop=0/0/1/0/0/0/") != std::string::npos, "drop= has checks 1-5, 5b, 6, 7 and the gate");
    check(countLines(printed, " nohist=") == countLines(printed, "[GPSFUSE]"), "every line has nohist= (Revision 3)");
    std::snprintf(what, sizeof what, "every line under about 215 characters (longest %zu)", longestLine(printed));
    check(longestLine(printed) <= 216, what);
    std::printf("    last line: %s", printed.substr(printed.rfind("[GPSFUSE]")).c_str());

    // ---------------------------------------------------------------- revision 2: bounded lock waits (R4)
    std::printf("the lock: nothing waits for it for more than a few ms; Off works without it (R4)\n");
    using pros::sim_detail::mutexBusy;
    using pros::sim_detail::mutexLongestWaitMs;
    mutexLongestWaitMs = 0;
    setMode(Mode::On);
    (void)status();
    setMode(Mode::Off);
    configure(&gps, Config(), true, true);
    std::snprintf(what, sizeof what, "setMode, status and configure wait at most 5 ms for the lock (asked %u)",
                  unsigned(mutexLongestWaitMs));
    check(mutexLongestWaitMs <= 5, what);
    mutexLongestWaitMs = 0;
    runSteps(200, allZero);
    check(mutexLongestWaitMs == 0, "step() (the odometry task) never waits for it");

    // a task holds the lock (or was deleted while holding it): Off still takes effect at once
    setMode(Mode::Shadow);
    check(mode() == Mode::Shadow, "(Shadow, lock free)");
    mutexBusy = true;
    printed = capture([] { setMode(Mode::Off); });
    check(mode() == Mode::Off, "lock busy: setMode(Off) still switches Off");
    check(printed.empty(), "  quietly");
    printed = runSteps(500, allZero);
    check(allZero && printed.empty(), "  and odometry's step() does nothing");
    s = status();
    check(!s.valid && s.mode == Mode::Off, "lock busy: status() comes back at once, marked not valid, mode Off");
    // Shadow or On cannot reset the filter without the lock: fusion stays Off and says why
    printed = capture([] { setMode(Mode::On); });
    check(mode() == Mode::Off, "lock busy: setMode(On) leaves fusion Off");
    check(printed.empty(), "  setMode itself prints nothing (Revision 3)");
    printed = logRound();
    check(printed.find("lock busy: could not switch to ON") != std::string::npos, "  the logging task says why");
    check(logRound().empty(), "  once");
    // a step while someone holds the lock is skipped, not waited for
    mutexBusy = false;
    setMode(Mode::Shadow);
    const std::uint32_t readingsBefore = status().newReadings;
    mutexBusy = true;
    nowMs += 10;
    const Correction skipped = step(1.5f, -2.5f, 0.3f);
    mutexBusy = false;
    check(skipped.xIn == 0 && skipped.yIn == 0 && status().newReadings == readingsBefore,
          "lock busy: odometry skips fusion for that tick");
    // configure while the lock is busy: the sensor is left out, so fusion can never use it
    mutexBusy = true;
    printed = capture([&] { configure(&gps, Config(), true, true); });
    mutexBusy = false;
    check(printed.empty(), "lock busy: configure prints nothing itself");
    check(logRound().find("configure: lock busy") != std::string::npos, "  the logging task says so");
    setMode(Mode::Shadow);
    runSteps(1000, allZero);
    check(status().newReadings == 0, "  and the GPS is never read");
    configure(&gps, Config(), true, true);
    setMode(Mode::Shadow);
    runSteps(1000, allZero);
    check(status().newReadings > 0, "configure again with the lock free: the GPS is read again");
    setMode(Mode::Off);
    pros::sim_detail::gpsReadHook = nullptr;

    // ---------------------------------------------------------------- wall sensors (docs/WALL_FIX_DESIGN.md)
    std::printf("wall sensors: configureWalls, step() reading them, the [WALLFIX] line\n");
    pros::sim_detail::gpsReadHook = nullptr;
    configure(nullptr, Config(), true, true); // walls only
    // no configureWalls yet: no distance sensor is read and no [WALLFIX] line is printed
    distanceReads = 0;
    pros::sim_detail::distanceReadHook = countingDistance;
    setMode(Mode::Shadow);
    printed = runSteps(1000, allZero);
    check(distanceReads == 0, "no wall sensor configured: no distance sensor is read");
    check(printed.find("[WALLFIX]") == std::string::npos, "  and no [WALLFIX] line");
    check(status().wallSensorCount == 0, "  and the status shows none");
    setMode(Mode::Off);

    // two sensors on ports 6 and 21, both unplugged (no hook): read every tick, every reading dropped by wall check 1
    pros::Distance back(6), side(21);
    pros::Distance* const pair[2] = {&back, &side};
    WallSensorMount mounts[2];
    mounts[1].yawDeg = 90;
    pros::sim_detail::distanceReadHook = nullptr;
    mutexLongestWaitMs = 0;
    configureWalls(pair, mounts, 2, WallConfig());
    check(mutexLongestWaitMs <= 5, "configureWalls waits at most 5 ms for the lock");
    check(mode() == Mode::Off, "configureWalls does not switch fusion on");
    setMode(Mode::Shadow);
    mutexLongestWaitMs = 0;
    printed = runSteps(3000, allZero);
    check(mutexLongestWaitMs == 0, "step() with wall sensors never waits for the lock");
    s = status();
    check(allZero, "Shadow, unplugged wall sensors: zero corrections");
    check(s.wallSensorCount == 2, "the status shows 2 wall sensors");
    check(s.walls[0].samples == 75 && s.walls[0].droppedByCheck[WCHECK_VALID] == 75,
          "unplugged (PROS_ERR): 75 readings in 3 s, all dropped by wall check 1");
    const int gpsLines = countLines(printed, "[GPSFUSE]");
    check(countLines(printed, "[WALLFIX]") == gpsLines && gpsLines >= 6, "a [WALLFIX] line after every [GPSFUSE] line");
    // (the last line comes up to 500 ms before the end, so its first counter is not the final 75)
    check(printed.find("/0/0/0/0/0/0/0/0/0/0/0/0 inn=+0.00 cor=+0.00 | 1 WAIT fix=0 drop=") != std::string::npos &&
              printed.find("| 0 WAIT fix=0 drop=") != std::string::npos,
          "  showing each sensor's state, fixes and drops");
    std::printf("    last line: %s", printed.substr(printed.rfind("[WALLFIX]")).c_str());
    setMode(Mode::Off);

    // a working sensor: the robot 20 in from the +x wall facing it (sensor 0 looks forward), the real robot 1 in
    // closer (483 mm, not 508). On: the fix is fed into the pose; the hook is asked for the right ports
    portsSeen = 0;
    pros::sim_detail::distanceReadHook = wallAt483;
    configureWalls(pair, mounts, 2, WallConfig());
    setMode(Mode::On);
    float x = 50.25f;
    printed = capture([&] {
        for (int i = 0; i < 300; i++) {
            nowMs += 10;
            const Correction c = step(x, 0, 1.5707963f);
            x += c.xIn;
            if (i % 50 == 0) printStatus();
        }
    });
    s = status();
    check(s.walls[0].fixes == 1, "a working sensor: one fix");
    checkNear(x - 50.25f, 0.794, 0.01, "  fed into the pose (about 0.79 in toward the wall)");
    check(portsSeen == (1 << 6 | 1 << 21), "  the hook was asked for ports 6 and 21");
    check(printed.find("| 0 OK fix=1 ") != std::string::npos, "  the line shows it");
    std::snprintf(what, sizeof what, "  every line under about 200 characters (longest %zu)", longestLine(printed));
    check(longestLine(printed) <= 202, what);
    setMode(Mode::Off);

    // a null sensor in the list reads as unplugged
    pros::Distance* const withNull[2] = {nullptr, &side};
    configureWalls(withNull, mounts, 2, WallConfig());
    setMode(Mode::Shadow);
    runSteps(1000, allZero);
    s = status();
    check(s.walls[0].droppedByCheck[WCHECK_VALID] == s.walls[0].samples && s.walls[0].samples > 20,
          "a null sensor pointer: read as unplugged");
    setMode(Mode::Off);

    // configureWalls while the lock is busy: the walls are left out and say so; nothing reads them
    mutexBusy = true;
    printed = capture([&] { configureWalls(pair, mounts, 2, WallConfig()); });
    mutexBusy = false;
    check(printed.empty(), "lock busy: configureWalls prints nothing itself");
    check(logRound().find("[WALLFIX] configureWalls: lock busy") != std::string::npos, "  the logging task says so");
    distanceReads = 0;
    pros::sim_detail::distanceReadHook = countingDistance;
    setMode(Mode::Shadow);
    runSteps(500, allZero);
    check(distanceReads == 0, "  and no distance sensor is read");
    // configureWalls(nothing): back to no wall sensors
    configureWalls(pair, mounts, 0, WallConfig());
    setMode(Mode::Shadow);
    printed = runSteps(1000, allZero);
    check(distanceReads == 0 && printed.find("[WALLFIX]") == std::string::npos && status().wallSensorCount == 0,
          "configureWalls with 0 sensors: none read, no line");
    setMode(Mode::Off);
    pros::sim_detail::distanceReadHook = nullptr;


    // ---------------------------------------------------------------- Revision 3
    std::printf("Revision 3: only the logging task prints; the On guards; setPose; the numbers checked\n");
    using pros::sim_detail::logTasksStarted;
    check(logTasksStarted == 1, "configure() started the logging task once, however often it was called");
    check(pros::sim_detail::otherTasksStarted == 0, "  and no other task");

    // step() and the mode switches never print, whatever happens (lines, walls, faults)
    pros::sim_detail::gpsReadHook = workingGps;
    configure(&gps, Config(), true, true);
    configureWalls(pair, mounts, 2, WallConfig());
    printed = capture([] { setMode(Mode::Shadow); });
    printed += runSteps(3000, allZero, false);
    printed += capture([] { setMode(Mode::On); setMode(Mode::Off); });
    check(printed.empty(), "3 s of step() with GPS and wall sensors, and setMode: nothing printed");
    configureWalls(pair, mounts, 0, WallConfig());
    // the logging task prints nothing while fusion is Off and nothing is pending
    check(logRound().empty(), "the logging task, fusion Off, nothing pending: prints nothing");
    // before this run's first tick it prints no [GPSFUSE] line (it would show the last run's pose)
    setMode(Mode::Shadow);
    check(logRound().empty(), "  switched on, no tick yet: no line");
    nowMs += 10;
    step(1.5f, -2.5f, 0.3f);
    check(countLines(logRound(), "[GPSFUSE]") == 1, "  after the first tick: one line");
    // Off, even when switched without the lock: no line (a line belongs to a run that is going on)
    mutexBusy = true;
    setMode(Mode::Off);
    mutexBusy = false;
    check(logRound().empty(), "  switched Off without the lock: no line");
    setMode(Mode::Shadow);
    nowMs += 10;
    step(1.5f, -2.5f, 0.3f);
    (void)pauseForSetPose();
    check(logRound().empty(), "  paused by a setPose: no line");
    restartAfterSetPose(Mode::Shadow);
    nowMs += 10;
    step(1.5f, -2.5f, 0.3f);
    // it skips a round when the lock is taken (it never waits)
    mutexLongestWaitMs = 0;
    mutexBusy = true;
    printed = logRound();
    mutexBusy = false;
    check(printed.empty() && mutexLongestWaitMs == 0, "  lock taken: that round is skipped, without waiting");
    setMode(Mode::Off);

    // the On guards: On gives Shadow unless both are true, and the logging task says which is missing
    struct Guard {
            bool mount, odometry;
            Mode want;
            const char* what;
    } guards[] = {
        {false, false, Mode::Shadow, "neither measured nor calibrated: Shadow, both named"},
        {true, false, Mode::Shadow, "mount measured, odometry not calibrated: Shadow, ODOMETRY_CALIBRATED named"},
        {false, true, Mode::Shadow, "odometry calibrated, mount not measured: Shadow, GPS_MOUNT_MEASURED named"},
        {true, true, Mode::On, "both: On, nothing said"},
    };
    for (const Guard& g : guards) {
        configure(&gps, Config(), g.mount, g.odometry);
        printed = capture([] { setMode(Mode::On); });
        printed += logRound();
        const bool namesMount = printed.find("GPS_MOUNT_MEASURED is false") != std::string::npos;
        const bool namesOdometry = printed.find("ODOMETRY_CALIBRATED is false") != std::string::npos;
        check(mode() == g.want && namesMount == !g.mount && namesOdometry == !g.odometry, g.what);
        setMode(Mode::Off);
    }
    configure(&gps, Config(), false, false);
    setMode(Mode::Shadow);
    check(mode() == Mode::Shadow && logRound().find("is false") == std::string::npos,
          "Shadow is always allowed, and nothing is said");
    setMode(Mode::Off);

    // the numbers: one warning per number outside its range; a latencyMs the history cannot serve is refused
    {
        Config c;
        c.relocSamples = 1;
        c.maxSensorErrorIn = 0.05f;
        printed = capture([&] { configure(&gps, c, true, true); });
        check(printed.empty(), "configure with two numbers out of range prints nothing itself");
        printed = logRound();
        check(countLines(printed, "[GPSFUSE] gpsFusionConfig (robot-config.cpp): ") == 2 &&
                  printed.find("relocSamples is outside 3 to 64") != std::string::npos &&
                  printed.find("maxSensorErrorIn is outside") != std::string::npos,
              "  the logging task prints one warning per number");
        std::printf("    %s", printed.c_str());
        setMode(Mode::Shadow);
        check(mode() == Mode::Shadow, "  warnings only: fusion still switches on");
        setMode(Mode::Off);
        configure(&gps, Config(), true, true);
        check(logRound().empty(), "the design's numbers: no warning");
        c = Config();
        c.latencyMs = 600;
        configure(&gps, c, true, true);
        printed = capture([] { setMode(Mode::Shadow); });
        printed += logRound();
        check(mode() == Mode::Off, "latencyMs 600: fusion stays Off");
        check(printed.find("latencyMs is outside 0 to 580") != std::string::npos &&
                  printed.find("refused gpsFusionConfig: fusion stays OFF") != std::string::npos,
              "  and the logging task says why");
        configure(&gps, Config(), true, true);
        setMode(Mode::Shadow);
        check(mode() == Mode::Shadow, "configure again with a good latencyMs: fusion switches on again");
        setMode(Mode::Off);
        WallConfig w;
        w.planeIn[1] = 1784;
        configureWalls(pair, mounts, 2, w);
        check(logRound().find("[WALLFIX] wall config: a planeIn is outside 60 to 72") != std::string::npos,
              "configureWalls: a wall plane in mm is warned about");
        configureWalls(pair, mounts, 0, WallConfig());
        check(logRound().empty(), "  (no wall sensor: nothing to check)");
    }

    // setPose: Off does nothing at all; otherwise the filter starts again from the new pose, as setMode does
    {
        configure(&gps, Config(), true, true);
        setMode(Mode::Off);
        const Status before = status();
        mutexLongestWaitMs = 0;
        const Mode paused = pauseForSetPose();
        check(paused == Mode::Off && mode() == Mode::Off && mutexLongestWaitMs == 0,
              "setPose with fusion Off: pauseForSetPose returns Off, touches no lock");
        const Status after = status();
        check(after.newReadings == before.newReadings && after.accepted == before.accepted,
              "  and the filter is not touched");
        setMode(Mode::On);
        runSteps(3000, allZero);
        check(status().startCheckPassed && status().accepted > 5, "On, 3 s: the start check passed, readings used");
        const Mode was = pauseForSetPose();
        check(was == Mode::On && mode() == Mode::Off, "setPose while On: odometry stops using the filter first");
        restartAfterSetPose(was);
        const Status fresh = status();
        check(mode() == Mode::On && fresh.newReadings == 0 && !fresh.startCheckPassed && fresh.accepted == 0 &&
                  fresh.appliedXIn == 0 && fresh.decidedXIn == 0,
              "  then starts again from the new pose (nothing fed in, the start check again)");
        check(mutexLongestWaitMs <= 5, "  waiting at most 5 ms for the lock");
        // the lock cannot be had: fusion goes Off and the logging task says so
        runSteps(1000, allZero);
        const Mode was2 = pauseForSetPose();
        mutexBusy = true;
        printed = capture([&] { restartAfterSetPose(was2); });
        mutexBusy = false;
        check(mode() == Mode::Off && printed.empty(), "setPose with the lock taken: fusion switched Off, quietly");
        check(logRound().find("setPose: lock busy") != std::string::npos, "  and the logging task says so");
        // Shadow is restarted as Shadow (and On given back only where it was allowed)
        setMode(Mode::Shadow);
        restartAfterSetPose(pauseForSetPose());
        check(mode() == Mode::Shadow, "setPose while Shadow: Shadow again");
        setMode(Mode::Off);
    }
    pros::sim_detail::gpsReadHook = nullptr;

    std::printf("\n%d checks, %d failed: %s\n", checksRun, checksFailed, checksFailed == 0 ? "ALL PASSED" : "FAILED");
    return checksFailed == 0 ? 0 : 1;
}
