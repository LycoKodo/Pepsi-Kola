// Drive testbench: the real LemLib motion code (the DUT) against the simulated robot in world.cpp.
// Mirrors the PID tab's test (src/pidtuner.cpp testTask): BRAKE mode, moveToPoint to a point
// `target` inches straight ahead (or turnToHeading), sampled every 10 ms until 300 ms after the end.
#include <unistd.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/util.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "json.hpp"
#include "world.hpp"

#ifndef SIM_DIR
#define SIM_DIR "."
#endif
#ifndef DUT_NAME
#define DUT_NAME "unknown"
#endif

namespace {

// ------------------------------------------------------------------------------------ options
struct Options {
        std::string move = "drive";
        double target = 24;
        double kP = NAN, kI = NAN, kD = NAN; // the active mode's controller
        int timeout = -1;
        std::string params = std::string(SIM_DIR) + "/robot.json";
        std::vector<std::pair<std::string, double>> sets;
        std::string csv, vcd, pidlog, dutStdout;
        bool summaryJson = false;
        bool selftest = false;
        uint32_t t0 = 1000;
};

[[noreturn]] void usage(const char* msg) {
    if (msg) std::fprintf(stderr, "tb: %s\n", msg);
    std::fprintf(stderr,
                 "usage: tb_" DUT_NAME " [--move drive|turn] [--target N] [--kP X --kI X --kD X]\n"
                 "        [--rot-ms N] [--rot-lock] [--rot-phase N] [--imu-ms N] [--imu-phase N] [--odom-phase N]\n"
                 "        [--latency MS] [--params robot.json] [--set KEY=VALUE ...] [--timeout MS]\n"
                 "        [--csv out.csv] [--vcd out.vcd] [--pidlog out.log|-] [--summary-json] [--dut-stdout FILE]\n"
                 "        [--selftest]   check the stubs (rotation data rate / --rot-lock) and exit\n");
    std::exit(2);
}

// ------------------------------------------------------------------------------------ columns
enum Col {
    T_MS, X, Y, HEADING, VL, VR, V, CMDL, CMDR, VCMDL, VCMDR, NETL, NETR, UL, UR, BRAKE, ROTV, ROTV_REFRESH, ROTH,
    IMU_DEG, ODOM_STROBE, ODOM_X, ODOM_Y, ODOM_THETA, ITER_STROBE, ITER_N, ITER_X, ITER_Y, ITER_THETA, ITER_DIST,
    ITER_ERR, ITER_FRESH, ITER_NEWSAMPLE, ITER_P, ITER_I, ITER_D, ITER_PID, CLOSE, IN_MOTION, FINISHED, TAB_STROBE,
    TAB_N, TAB_ERR, TAB_DEV, TARGET, NCOLS
};
struct ColInfo {
        const char* name;
        char kind; // 'r' real, 'i' integer, 'b' bit
        const char* help;
};
const ColInfo COLS[NCOLS] = {
    {"t_ms", 'i', "ms since the motion was called"},
    {"x", 'r', "true x, in (+ right of the start)"},
    {"y", 'r', "true y, in (+ ahead of the start)"},
    {"heading_deg", 'r', "true heading, deg, clockwise +"},
    {"vL", 'r', "left side speed, in/s"},
    {"vR", 'r', "right side speed, in/s"},
    {"v", 'r', "forward speed, in/s"},
    {"cmdL", 'i', "left command last set by the DUT, -127..127"},
    {"cmdR", 'i', "right command last set by the DUT"},
    {"vcmdL", 'r', "left command voltage after the latency, V"},
    {"vcmdR", 'r', "right command voltage after the latency, V"},
    {"netL", 'r', "left net voltage applied (command - back-EMF, limited), V"},
    {"netR", 'r', "right net voltage applied, V"},
    {"uL", 'r', "left terminal voltage (get_voltage), V"},
    {"uR", 'r', "right terminal voltage, V"},
    {"brake", 'i', "drive brake mode: 0 coast, 1 brake, 2 hold"},
    {"rotV_cdeg", 'i', "vertical rotation sensor reading, centidegrees"},
    {"rotV_refresh", 'b', "vertical sensor took a new sample this ms"},
    {"rotH_cdeg", 'i', "horizontal rotation sensor reading, centidegrees"},
    {"imu_deg", 'r', "IMU get_rotation(), deg"},
    {"odom_strobe", 'b', "lemlib::update() ran this ms"},
    {"odom_x", 'r', "LemLib odometry x, in"},
    {"odom_y", 'r', "LemLib odometry y, in"},
    {"odom_theta", 'r', "LemLib odometry heading, deg"},
    {"iter_strobe", 'b', "the DUT's motion loop called pros::delay this ms"},
    {"iter_n", 'i', "motion-loop iteration number (held)"},
    {"iter_x", 'r', "x the DUT saw on its last iteration (held)"},
    {"iter_y", 'r', "y the DUT saw (held)"},
    {"iter_theta", 'r', "heading the DUT saw, deg (held)"},
    {"iter_dist", 'r', "distance to the target the DUT saw, in (drive) (held)"},
    {"iter_err", 'r', "error the DUT's PID saw: lateral in / heading deg (held)"},
    {"iter_fresh", 'b', "drive: the position the DUT saw changed since its previous iteration; turn: the heading (held)"},
    {"iter_newsample", 'b', "a new vertical-sensor sample reached odometry since the previous iteration (held)"},
    {"iter_P", 'r', "bench-computed kP*error (held)"},
    {"iter_I", 'r', "bench-computed kI*integral, LemLib rules (held)"},
    {"iter_D", 'r', "bench-computed kD*(error - previous error) (held)"},
    {"iter_pid", 'r', "P+I+D before LemLib's clamp and slew (held)"},
    {"close", 'b', "drive: distance < 7.5 seen (latched); turn: |error| < 20 deg (no slew)"},
    {"in_motion", 'b', "chassis.isInMotion()"},
    {"finished", 'b', "the motion function has returned"},
    {"tab_strobe", 'b', "the PID tab sampled this ms"},
    {"tab_n", 'i', "PID tab sample index (held)"},
    {"tab_err", 'r', "PID tab error: in or deg, + = short of the target (held)"},
    {"tab_dev", 'r', "PID tab dev: drive heading change deg / turn wander in (held)"},
    {"target", 'r', "the commanded target: in (drive) or deg (turn), signed (constant)"},
};

using Row = std::array<double, NCOLS>;

// ------------------------------------------------------------------------------------ bench state
struct TabSample {
        int t;
        double err, dev, vL, vR, uL, uR;
};

struct Bench {
        Options o;
        bool drive = true;
        lemlib::Chassis* chassis = nullptr;
        pros::MotorGroup* left = nullptr;
        pros::MotorGroup* right = nullptr;
        double kP = 0, kI = 0, kD = 0, windup = 0;

        // motion
        lemlib::Pose start {0, 0, 0}; // degrees
        double tx = 0, ty = 0, targetHeading = 0;
        bool recording = false, finished = false;
        uint32_t tEnd = 0;

        // per-iteration (held)
        int iterN = 0;
        bool iterStrobeNow = false;
        uint32_t iterStrobeAt = UINT32_MAX;
        double ix = 0, iy = 0, ith = 0, idist = 0, ierr = 0, iP = 0, iI = 0, iD = 0, ipid = 0;
        bool ifresh = false, inew = false, closeLatched = false;
        double prevErr = 0, integral = 0, lastX = NAN, lastY = NAN, lastTh = NAN;
        uint32_t lastFreshCount = 0;
        struct Iter {
                int n;
                uint32_t t;
                double dist, err, P, D, pid, v;
                int cmdL, cmdR;
                bool fresh, newsample, close;
        };
        std::vector<Iter> iters;

        // PID tab sampler
        bool samplerActive = false, samplerDone = false;
        uint32_t nextSample = 0, tabStrobeAt = UINT32_MAX;
        int n = 0, endSample = -1;
        std::vector<TabSample> tab;

        std::vector<Row> rows;
};

Bench B;

double angleErrDeg(double target, double pos) { return lemlib::angleError(float(target), float(pos), false); }

// One motion-loop iteration: the DUT is about to call pros::delay. Odometry does not change inside an
// iteration, so the pose LemLib reports now is the pose the DUT read at the top of this iteration.
void onIteration(uint32_t now) {
    sim::World& w = sim::world();
    const lemlib::Pose p = lemlib::getPose(true); // heading convention, radians
    B.iterN++;
    B.iterStrobeAt = now;
    B.ix = p.x;
    B.iy = p.y;
    B.ith = p.theta * 180.0 / M_PI;
    double err;
    if (B.drive) {
        const double dx = B.tx - p.x, dy = B.ty - p.y;
        B.idist = std::hypot(dx, dy);
        if (B.idist < 7.5) B.closeLatched = true;
        const double stdTheta = M_PI_2 - p.theta;
        err = B.idist * std::cos(lemlib::angleError(float(stdTheta), float(std::atan2(dy, dx))));
    } else {
        err = angleErrDeg(B.targetHeading, B.ith);
        B.idist = 0;
        B.closeLatched = std::fabs(err) < 20;
    }
    // the bench's own PID terms, from the errors the DUT saw (LemLib's rules for I, one-sample D)
    B.integral += err;
    if (lemlib::sgn(err) != lemlib::sgn(B.prevErr)) B.integral = 0;
    if (std::fabs(err) > B.windup && B.windup != 0) B.integral = 0;
    B.ierr = err;
    B.iP = B.kP * err;
    B.iI = B.kI * B.integral;
    B.iD = B.kD * (err - B.prevErr);
    B.ipid = B.iP + B.iI + B.iD;
    B.prevErr = err;
    // fresh: drive = the position the DUT saw changed; turn = the heading changed
    B.ifresh = std::isnan(B.lastX) ||
               (B.drive ? std::hypot(p.x - B.lastX, p.y - B.lastY) > 1e-6 : std::fabs(p.theta - B.lastTh) > 1e-9);
    B.lastTh = p.theta;
    B.inew = B.iterN == 1 || w.odomFreshUpdates != B.lastFreshCount;
    B.lastX = p.x;
    B.lastY = p.y;
    B.lastFreshCount = w.odomFreshUpdates;
    B.iters.push_back({B.iterN, now - B.o.t0, B.idist, err, B.iP, B.iD, B.ipid, (std::fabs(w.v[0]) + std::fabs(w.v[1])) / 2, w.cmd[0],
                       w.cmd[1], B.ifresh, B.inew, B.closeLatched});
}

// The PID tab's sampler (testTask), run as a world event every 10 ms
void sampler(uint32_t now) {
    if (!B.samplerActive || B.samplerDone || now != B.nextSample) return;
    const lemlib::Pose pose = B.chassis->getPose(); // degrees
    const double th0 = B.start.theta * M_PI / 180.0;
    const double dir = B.o.target < 0 ? -1 : 1;
    double err, dev;
    if (B.drive) {
        const double progress = (pose.x - B.start.x) * std::sin(th0) + (pose.y - B.start.y) * std::cos(th0);
        err = std::fabs(B.o.target) - progress * dir;
        dev = pose.theta - B.start.theta;
    } else {
        err = (B.targetHeading - pose.theta) * dir;
        dev = std::hypot(pose.x - B.start.x, pose.y - B.start.y);
    }
    B.tab.push_back({B.n * 10, err, dev, B.left->get_actual_velocity(), B.right->get_actual_velocity(),
                     double(B.left->get_voltage()), double(B.right->get_voltage())});
    B.tabStrobeAt = now;
    B.n++;
    if (B.chassis->isInMotion()) B.endSample = -1;
    else if (B.endSample < 0) B.endSample = B.n;
    else if ((B.n - B.endSample) * 10 >= 300) B.samplerDone = true;
    if (B.n >= 512) B.samplerDone = true;
    B.nextSample += 10;
}

void record(uint32_t now) {
    if (!B.recording) return;
    sim::World& w = sim::world();
    Row r {};
    r[T_MS] = double(now) - double(B.o.t0);
    r[X] = w.x;
    r[Y] = w.y;
    r[HEADING] = w.heading * 180.0 / M_PI;
    r[VL] = w.v[0];
    r[VR] = w.v[1];
    r[V] = (w.v[0] + w.v[1]) / 2;
    r[CMDL] = w.cmd[0];
    r[CMDR] = w.cmd[1];
    r[VCMDL] = w.vcmd[0];
    r[VCMDR] = w.vcmd[1];
    r[NETL] = w.net[0];
    r[NETR] = w.net[1];
    r[UL] = w.applied[0];
    r[UR] = w.applied[1];
    r[BRAKE] = w.motors.count(1) ? w.motors[1].brake : 0;
    const int vp = w.vertPort();
    for (auto& [port, rd] : w.rot) {
        const double reading = (rd.reversed ? -1 : 1) * double(rd.sample - rd.zero);
        if (port == vp) {
            r[ROTV] = reading;
            r[ROTV_REFRESH] = rd.lastRefresh == now;
        } else {
            r[ROTH] = reading;
        }
    }
    r[IMU_DEG] = w.imu.empty() ? 0 : w.imu.begin()->second.sampleDeg - w.imu.begin()->second.zeroDeg;
    const lemlib::Pose op = lemlib::getPose(false);
    r[ODOM_STROBE] = w.odomStrobe;
    r[ODOM_X] = op.x;
    r[ODOM_Y] = op.y;
    r[ODOM_THETA] = op.theta;
    r[ITER_STROBE] = B.iterStrobeAt == now;
    r[ITER_N] = B.iterN;
    r[ITER_X] = B.ix;
    r[ITER_Y] = B.iy;
    r[ITER_THETA] = B.ith;
    r[ITER_DIST] = B.idist;
    r[ITER_ERR] = B.ierr;
    r[ITER_FRESH] = B.ifresh;
    r[ITER_NEWSAMPLE] = B.inew;
    r[ITER_P] = B.iP;
    r[ITER_I] = B.iI;
    r[ITER_D] = B.iD;
    r[ITER_PID] = B.ipid;
    r[CLOSE] = B.closeLatched;
    r[IN_MOTION] = B.chassis->isInMotion();
    r[FINISHED] = B.finished;
    r[TAB_STROBE] = B.tabStrobeAt == now;
    r[TAB_N] = B.n - 1;
    r[TAB_ERR] = B.tab.empty() ? NAN : B.tab.back().err;
    r[TAB_DEV] = B.tab.empty() ? NAN : B.tab.back().dev;
    r[TARGET] = B.o.target;
    B.rows.push_back(r);
}

// ------------------------------------------------------------------------------------ writers
void fmtVal(std::string& s, double v, char kind) {
    char buf[48];
    if (std::isnan(v)) std::snprintf(buf, sizeof buf, "nan");
    else if (kind == 'r') std::snprintf(buf, sizeof buf, "%.5g", v);
    else std::snprintf(buf, sizeof buf, "%lld", (long long)std::llround(v));
    s += buf;
}

void writeCsv(const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) usage(("cannot write " + path).c_str());
    std::string line;
    for (int c = 0; c < NCOLS; c++) line += std::string(c ? "," : "") + COLS[c].name;
    std::fprintf(f, "%s\n", line.c_str());
    for (const Row& r : B.rows) {
        line.clear();
        for (int c = 0; c < NCOLS; c++) {
            if (c) line += ',';
            fmtVal(line, r[c], COLS[c].kind);
        }
        std::fprintf(f, "%s\n", line.c_str());
    }
    std::fclose(f);
}

void writeVcd(const std::string& path, const std::string& about) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) usage(("cannot write " + path).c_str());
    std::fprintf(f, "$comment %s $end\n$timescale 1 ms $end\n$scope module tb $end\n", about.c_str());
    auto id = [](int c) { // letters and digits only, so no reader mistakes an id for '$end' or '#time'
        static const char* A = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
        std::string s;
        int k = c;
        do {
            s += A[k % 62];
            k /= 62;
        } while (k);
        return s;
    };
    for (int c = 1; c < NCOLS; c++) {
        const char* type = COLS[c].kind == 'r' ? "real 64" : (COLS[c].kind == 'b' ? "wire 1" : "integer 32");
        std::fprintf(f, "$var %s %s %s $end\n", type, id(c).c_str(), COLS[c].name);
    }
    std::fprintf(f, "$upscope $end\n$enddefinitions $end\n");
    Row prev;
    prev.fill(NAN);
    for (const Row& r : B.rows) {
        bool stamped = false;
        for (int c = 1; c < NCOLS; c++) {
            const double v = r[c];
            if (v == prev[c] || (std::isnan(v) && std::isnan(prev[c]))) continue;
            if (!stamped) {
                std::fprintf(f, "#%lld\n", (long long)std::llround(r[T_MS]));
                stamped = true;
            }
            if (COLS[c].kind == 'r') {
                std::fprintf(f, "r%.6g %s\n", std::isnan(v) ? 0.0 : v, id(c).c_str());
            } else if (COLS[c].kind == 'b') {
                std::fprintf(f, "%d%s\n", v != 0, id(c).c_str());
            } else {
                long long iv = std::llround(v);
                std::string bits;
                unsigned long long u = (unsigned long long)iv & 0xffffffffULL;
                for (int b = 31; b >= 0; b--) bits += ((u >> b) & 1) ? '1' : '0';
                const size_t nz = bits.find('1');
                std::fprintf(f, "b%s %s\n", nz == std::string::npos ? "0" : bits.c_str() + nz, id(c).c_str());
            }
            prev[c] = v;
        }
    }
    if (!B.rows.empty()) std::fprintf(f, "#%lld\n", (long long)std::llround(B.rows.back()[T_MS]) + 1);
    std::fclose(f);
}

struct Metrics {
        double over = 0, finalErr = NAN, settle = -1, endMs = -1;
};

Metrics tabMetrics(double tol) {
    Metrics m;
    if (B.tab.empty()) return m;
    double minErr = 0;
    int lastOutside = -1;
    for (size_t i = 0; i < B.tab.size(); i++) {
        minErr = std::min(minErr, B.tab[i].err);
        if (std::fabs(B.tab[i].err) > tol) lastOutside = int(i);
    }
    m.over = -minErr;
    m.finalErr = B.tab.back().err;
    m.settle = lastOutside == int(B.tab.size()) - 1 ? -1 : (lastOutside + 1) * 10;
    m.endMs = B.endSample < 0 ? -1 : (B.endSample - 1) * 10;
    return m;
}

void writePidlog(FILE* f, const Metrics& m) {
    const bool d = B.drive;
    std::fprintf(f,
                 "[PIDLOG] BEGIN v=1 run=1 mode=%s target=%.1f unit=%s kP=%.3f kI=%.4f kD=%.3f windup=%.2f slew=15.0 "
                 "small=%s large=%s timeout=%d batt_mV=0 batt_pct=0 dt_ms=10 ips_per_rpm=0.1080 track=11.61 "
                 "cols=t,err,dev,vL,vR,uL,uR src=sim dut=%s rot_ms=%d\n",
                 d ? "DRIVE" : "TURN", B.o.target, d ? "in" : "deg", B.kP, B.kI, B.kD, B.windup,
                 d ? "0.50/200" : "1.00/200", "3.00/500", B.o.timeout, DUT_NAME, sim::world().p.rotMs);
    uint32_t sum = 0;
    for (const TabSample& s : B.tab) {
        const int row[7] = {s.t,
                            int(std::lround(s.err * 100)),
                            int(std::lround(s.dev * 100)),
                            int(std::lround(s.vL)),
                            int(std::lround(s.vR)),
                            int(std::lround(s.uL)),
                            int(std::lround(s.uR))};
        for (int v : row) sum += uint32_t(v);
        std::fprintf(f, "~%d,%d,%d,%d,%d,%d,%d\n", row[0], row[1], row[2], row[3], row[4], row[5], row[6]);
    }
    std::fprintf(f, "[PIDLOG] END run=1 rows=%zu sum=%u end_ms=%d result=ok final=%.2f over=%.2f settle_ms=%d\n",
                 B.tab.size(), sum, int(m.endMs), m.finalErr, m.over, int(m.settle));
}

} // namespace

// ------------------------------------------------------------------------------------ main
int main(int argc, char** argv) {
    Options& o = B.o;
    sim::World& w = sim::world();
    std::vector<std::pair<std::string, std::string>> timing;
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) usage(("missing value for " + a).c_str());
            return argv[++i];
        };
        if (a == "--move") o.move = next();
        else if (a == "--target") o.target = std::atof(next().c_str());
        else if (a == "--kP") o.kP = std::atof(next().c_str());
        else if (a == "--kI") o.kI = std::atof(next().c_str());
        else if (a == "--kD") o.kD = std::atof(next().c_str());
        else if (a == "--timeout") o.timeout = std::atoi(next().c_str());
        else if (a == "--params") o.params = next();
        else if (a == "--csv") o.csv = next();
        else if (a == "--vcd") o.vcd = next();
        else if (a == "--pidlog") o.pidlog = next();
        else if (a == "--dut-stdout") o.dutStdout = next();
        else if (a == "--summary-json") o.summaryJson = true;
        else if (a == "--selftest") o.selftest = true;
        else if (a == "--rot-lock") w.p.rotLock = true;
        else if (a == "--rot-ms" || a == "--rot-phase" || a == "--imu-ms" || a == "--imu-phase" ||
                 a == "--odom-phase")
            timing.push_back({a, next()});
        else if (a == "--latency") o.sets.push_back({"LATENCY_MS", std::atof(next().c_str())});
        else if (a == "--set") {
            const std::string kv = next();
            const size_t eq = kv.find('=');
            if (eq == std::string::npos) usage("--set needs KEY=VALUE");
            o.sets.push_back({kv.substr(0, eq), std::atof(kv.c_str() + eq + 1)});
        } else if (a == "-h" || a == "--help") usage(nullptr);
        else usage(("unknown option " + a).c_str());
    }
    if (o.move != "drive" && o.move != "turn") usage("--move must be drive or turn");
    B.drive = o.move == "drive";
    if (o.timeout < 0) o.timeout = B.drive ? 4000 : 2500;

    // plant parameters: robot.json, then --set
    {
        std::ifstream in(o.params);
        if (!in) usage(("cannot read " + o.params).c_str());
        std::stringstream ss;
        ss << in.rdbuf();
        for (auto& [k, v] : sim::JsonFlat::parse(ss.str()))
            if (!w.p.set(k, v)) std::fprintf(stderr, "[tb] note: %s: ignoring unknown key %s\n", o.params.c_str(), k.c_str());
    }
    for (auto& [k, v] : o.sets)
        if (!w.p.set(k, v)) usage(("unknown parameter " + k).c_str());
    for (auto& [k, v] : timing) {
        const int iv = std::atoi(v.c_str());
        if (k == "--rot-ms") w.p.rotMs = std::max(5, iv / 5 * 5);
        else if (k == "--rot-phase") w.p.rotPhase = iv;
        else if (k == "--imu-ms") w.p.imuMs = std::max(1, iv);
        else if (k == "--imu-phase") w.p.imuPhase = iv;
        else if (k == "--odom-phase") w.p.odomPhase = iv;
    }

    if (o.selftest) {
        // a DUT call to set_data_rate(5) must take effect, unless --rot-lock keeps --rot-ms
        pros::Rotation probe(21);
        const int before = w.rot[21].periodMs;
        probe.set_data_rate(5);
        const int after = w.rot[21].periodMs;
        const int expect = w.p.rotLock ? w.p.rotMs : 5;
        std::printf("selftest: rotation period %d ms -> set_data_rate(5) -> %d ms (expected %d, rot_lock %d): %s\n",
                    before, after, expect, int(w.p.rotLock), after == expect ? "ok" : "FAIL");
        return after == expect ? 0 : 1;
    }

    // keep the DUT's printf traffic out of the bench's output
    std::fflush(stdout);
    const int benchFd = dup(1);
    if (!std::freopen(o.dutStdout.empty() ? "/dev/null" : o.dutStdout.c_str(), "w", stdout)) {
        std::perror("freopen");
        return 1;
    }
    FILE* out = fdopen(benchFd, "w");

    // ---- the robot, as in src/robot-config.cpp
    w.reset();
    pros::MotorGroup leftMotors({-1, -2}, pros::MotorGearset::blue);
    pros::MotorGroup rightMotors({11, 12}, pros::MotorGearset::blue);
    for (int p : {1, 2}) w.motors[p].side = sim::LEFT;
    for (int p : {11, 12}) w.motors[p].side = sim::RIGHT;
    lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 11.61, lemlib::Omniwheel::NEW_275, 450, 2);
    pros::Imu imu(13);
    pros::Rotation verticalEncoder(20);
    pros::Rotation horizontalEncoder(3);
    w.rot[20].bound = true;
    w.rot[20].isVertical = true;
    w.rot[3].bound = true;
    w.rot[3].isVertical = false;
    lemlib::TrackingWheel verticalWheel(&verticalEncoder, lemlib::Omniwheel::NEW_2, 0.9);
    lemlib::TrackingWheel horizontalWheel(&horizontalEncoder, lemlib::Omniwheel::NEW_2, 2.55);
    lemlib::OdomSensors sensors(&verticalWheel, nullptr, &horizontalWheel, nullptr, &imu);
    const bool d = B.drive;
    auto pick = [](double v, double def) { return std::isnan(v) ? def : v; };
    lemlib::ControllerSettings linear(d ? pick(o.kP, 12.5) : 12.5, d ? pick(o.kI, 0.1) : 0.1, d ? pick(o.kD, 100) : 100,
                                      3, 0.5, 200, 3, 500, 15);
    lemlib::ControllerSettings angular(!d ? pick(o.kP, 4.8) : 4.8, !d ? pick(o.kI, 0.25) : 0.25,
                                       !d ? pick(o.kD, 42) : 42, 5, 1, 200, 3, 500, 15);
    lemlib::ExpoDriveCurve throttleCurve(0, 10, 1);
    lemlib::ExpoDriveCurve steerCurve(0, 10, 1.010);
    lemlib::Chassis chassis(drivetrain, linear, angular, sensors, &throttleCurve, &steerCurve);
    const lemlib::ControllerSettings& active = d ? linear : angular;
    B.kP = active.kP;
    B.kI = active.kI;
    B.kD = active.kD;
    B.windup = active.windupRange;
    B.chassis = &chassis;
    B.left = &leftMotors;
    B.right = &rightMotors;

    w.preTick = record;
    w.postEvents = sampler;
    w.onDelay = [](uint32_t now) {
        if (B.recording && !B.finished && B.chassis->isInMotion()) onIteration(now);
    };

    // ---- boot: calibrate (starts odometry through lemlib::init), zero the pose, sit still
    w.events(); // t = 0
    chassis.calibrate(true);
    chassis.setPose(0, 0, 0);
    if (w.now < o.t0) w.advance(o.t0 - w.now);

    // ---- the PID tab test
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    B.start = chassis.getPose();
    const double th0 = B.start.theta * M_PI / 180.0;
    B.tx = B.start.x + o.target * std::sin(th0);
    B.ty = B.start.y + o.target * std::cos(th0);
    B.targetHeading = B.start.theta + o.target;
    B.recording = true;
    B.samplerActive = true;
    B.nextSample = w.now + 10; // the async call's delay(10) before testTask starts sampling
    if (d) {
        lemlib::MoveToPointParams mp;
        mp.forwards = o.target > 0;
        chassis.moveToPoint(float(B.tx), float(B.ty), o.timeout, mp, false);
    } else {
        chassis.turnToHeading(float(B.targetHeading), o.timeout, {}, false);
    }
    B.finished = true;
    B.tEnd = w.now;
    while (!B.samplerDone) w.advance(1);
    record(w.now);

    // ---- outputs
    const Metrics m = tabMetrics(d ? 1.0 : 1.0);
    if (!o.csv.empty()) writeCsv(o.csv);
    char about[256];
    std::snprintf(about, sizeof about, "tb_%s move=%s target=%g kP=%g kI=%g kD=%g rot_ms=%d rot_lock=%d", DUT_NAME,
                  o.move.c_str(), o.target, B.kP, B.kI, B.kD, w.p.rotMs, int(w.p.rotLock));
    if (!o.vcd.empty()) writeVcd(o.vcd, about);
    if (!o.pidlog.empty()) {
        FILE* f = o.pidlog == "-" ? out : std::fopen(o.pidlog.c_str(), "w");
        if (!f) usage(("cannot write " + o.pidlog).c_str());
        writePidlog(f, m);
        if (f != out) std::fclose(f);
    }

    // summary
    double peakV = 0, peakT = 0, peakDist = 0; // wheel speed (|vL|+|vR|)/2: forward speed on a drive
    for (const Row& r : B.rows)
        if ((std::fabs(r[VL]) + std::fabs(r[VR])) / 2 > peakV) {
            peakV = (std::fabs(r[VL]) + std::fabs(r[VR])) / 2;
            peakT = r[T_MS];
            peakDist = B.drive ? std::hypot(B.tx - r[X], B.ty - r[Y]) : 0;
        }
    int moving = 0, staleMoving = 0, stale = 0, closeIter = -1;
    for (size_t k = 0; k < B.iters.size(); k++) {
        const auto& it = B.iters[k];
        if (!it.fresh) stale++;
        if (std::fabs(it.v) > 2.0) {
            moving++;
            if (!it.fresh) staleMoving++;
        }
        if (closeIter < 0 && it.close && B.drive) closeIter = int(k);
    }
    std::string rotInfo;
    for (auto& [port, r] : w.rot) {
        std::string req;
        for (int q : r.requestedRates) req += (req.empty() ? "" : ",") + std::to_string(q);
        char buf[200];
        std::snprintf(buf, sizeof buf, "%s{\"port\":%d,\"wheel\":\"%s\",\"period_ms\":%d,\"dut_requests\":[%s]}",
                      rotInfo.empty() ? "" : ",", port, r.isVertical ? "vertical" : "horizontal", r.periodMs,
                      req.c_str());
        rotInfo += buf;
    }
    const double staleFrac = B.iters.empty() ? NAN : double(stale) / B.iters.size();
    const double staleMovFrac = moving ? double(staleMoving) / moving : NAN;
    const double closeT = closeIter >= 0 ? B.iters[closeIter].t : -1;
    const double closeDist = closeIter >= 0 ? B.iters[closeIter].dist : NAN;
    if (o.summaryJson) {
        auto j = [](double v) { // JSON has no NaN
            char buf[40];
            if (!std::isfinite(v)) return std::string("null");
            std::snprintf(buf, sizeof buf, "%.6g", v);
            return std::string(buf);
        };
        std::fprintf(out,
                     "{\"dut\":\"%s\",\"move\":\"%s\",\"target\":%s,\"kP\":%s,\"kI\":%s,\"kD\":%s,\"rot_ms_cli\":%d,"
                     "\"rot_lock\":%s,\"rotation\":[%s],\"imu_ms\":%d,\"latency_ms\":%s,\"motion_end_ms\":%d,"
                     "\"end_ms\":%s,\"iterations\":%zu,\"stale_frac\":%s,\"stale_frac_moving\":%s,"
                     "\"close_t\":%s,\"close_dist\":%s,\"overshoot\":%s,\"final\":%s,\"settle_ms\":%s,"
                     "\"peak_speed\":%s,\"peak_t\":%s,\"peak_togo\":%s,\"tasks\":%d}\n",
                     DUT_NAME, o.move.c_str(), j(o.target).c_str(), j(B.kP).c_str(), j(B.kI).c_str(), j(B.kD).c_str(),
                     sim::world().p.rotMs, w.p.rotLock ? "true" : "false", rotInfo.c_str(), w.p.imuMs,
                     j(w.p.LATENCY_MS).c_str(), int(B.tEnd - o.t0), j(m.endMs).c_str(), B.iters.size(),
                     j(staleFrac).c_str(), j(staleMovFrac).c_str(), j(closeT).c_str(), j(closeDist).c_str(),
                     j(m.over).c_str(), j(m.finalErr).c_str(), j(m.settle).c_str(), j(peakV).c_str(), j(peakT).c_str(),
                     j(peakDist).c_str(), w.tasksCreated);
    } else {
        std::fprintf(out, "tb_%s  %s target %g  kP %g kI %g kD %g  latency %g ms\n", DUT_NAME, o.move.c_str(),
                     o.target, B.kP, B.kI, B.kD, w.p.LATENCY_MS);
        for (auto& [port, r] : w.rot) {
            std::string req;
            for (int q : r.requestedRates) req += (req.empty() ? "" : ",") + std::to_string(q);
            std::fprintf(out, "  rotation port %d (%s wheel): refresh %d ms%s%s%s\n", port,
                         r.isVertical ? "vertical" : "horizontal", r.periodMs,
                         req.empty() ? "" : "; DUT asked set_data_rate(", req.c_str(),
                         req.empty() ? "" : (w.p.rotLock ? ") -> ignored (--rot-lock)" : ")"));
        }
        std::fprintf(out, "  motion ended at %d ms after %zu iterations (PID tab end_ms %g)\n", int(B.tEnd - o.t0),
                     B.iters.size(), m.endMs);
        if (B.drive)
            std::fprintf(out, "  close at iteration %d, t=%g ms, %.2f in to go\n", closeIter + 1, closeT, closeDist);
        std::fprintf(out, "  stale iterations (%s unchanged): %.0f%% of all, %.0f%% while moving (wheels > 2 in/s)\n",
                     B.drive ? "position" : "heading", 100 * staleFrac, 100 * staleMovFrac);
        if (B.drive) std::fprintf(out, "  peak speed %.1f in/s at t=%g ms, %.2f in to go\n", peakV, peakT, peakDist);
        else std::fprintf(out, "  peak wheel speed %.1f in/s at t=%g ms\n", peakV, peakT);
        std::fprintf(out, "  PID tab: overshoot %.2f %s, final %.2f, settle %g ms (1.0 tolerance)\n", m.over,
                     B.drive ? "in" : "deg", m.finalErr, m.settle);
    }
    std::fflush(out);
    return 0;
}
