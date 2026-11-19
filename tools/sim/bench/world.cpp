// The simulated robot and scheduler; see world.hpp for the tick order.
#include "world.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include "lemlib/chassis/odom.hpp"

namespace sim {

World& world() {
    static World w;
    return w;
}

bool Params::set(const std::string& k, double val) {
    struct Entry {
            const char* name;
            double Params::*field;
    };
    static const Entry table[] = {
        {"C", &Params::C},
        {"KV", &Params::KV},
        {"F", &Params::F},
        {"DV_ACCEL", &Params::DV_ACCEL},
        {"DV_BRAKE", &Params::DV_BRAKE},
        {"TRACK_EFF", &Params::TRACK_EFF},
        {"LATENCY_MS", &Params::LATENCY_MS},
        {"C_DRIVE_L", &Params::C_DRIVE_L},
        {"C_DRIVE_R", &Params::C_DRIVE_R},
        {"C_BRAKE_L", &Params::C_BRAKE_L},
        {"C_BRAKE_R", &Params::C_BRAKE_R},
        {"V_MAX", &Params::V_MAX},
        {"STICK_EPS", &Params::STICK_EPS},
        {"VERT_OFFSET", &Params::VERT_OFFSET},
        {"HORIZ_OFFSET", &Params::HORIZ_OFFSET},
        {"TRACK_WHEEL_D", &Params::TRACK_WHEEL_D},
        {"DRIVE_WHEEL_D", &Params::DRIVE_WHEEL_D},
        {"DRIVE_RPM", &Params::DRIVE_RPM},
    };
    for (const Entry& e : table) {
        if (k == e.name) {
            this->*e.field = val;
            return true;
        }
    }
    return false;
}

static int posMod(long a, long m) { return int(((a % m) + m) % m); }
static double sgn(double v) { return v < 0 ? -1.0 : (v > 0 ? 1.0 : 0.0); }

void World::reset() {
    now = 0;
    x = y = heading = 0;
    v[0] = v[1] = travel[0] = travel[1] = 0;
    vertTravel = horizTravelLeft = 0;
    cmd[0] = cmd[1] = 0;
    cmdHist[0].assign(1, 0);
    cmdHist[1].assign(1, 0);
}

void World::advance(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) tick();
}

void World::tick() {
    if (preTick) preTick(now);
    for (int s = 0; s < 2; s++) {
        if (cmdHist[s].size() < now + 1) cmdHist[s].resize(now + 1, cmd[s]);
        cmdHist[s][now] = cmd[s];
    }
    physics(0.001);
    now++;
    events();
}

void World::physics(double dt) {
    double vOld[2] = {v[0], v[1]};
    for (int s = 0; s < 2; s++) {
        // command latency, linear between whole milliseconds so the fit sees a smooth parameter
        const double L = std::max(0.0, p.LATENCY_MS);
        const long lo = long(std::floor(L));
        const double frac = L - lo;
        auto at = [&](long t) -> int {
            if (t < 0) return 0;
            if (size_t(t) >= cmdHist[s].size()) return cmd[s];
            return cmdHist[s][size_t(t)];
        };
        const int c0 = at(long(now) - lo), c1 = at(long(now) - lo - 1);
        const double c = (1 - frac) * c0 + frac * c1;
        vcmd[s] = c / 127.0 * p.V_MAX;

        // brake mode of the side (first port found on it)
        int brake = 0;
        for (auto& [port, m] : motors)
            if (m.side == s) {
                brake = m.brake;
                break;
            }
        const bool moving = std::fabs(v[s]) > p.STICK_EPS;
        coasting[s] = (c0 == 0 && c1 == 0 && brake == 0);
        double n = 0;
        bool opposing = false;
        if (!coasting[s]) {
            const double raw = vcmd[s] - p.KV * v[s];
            opposing = moving && raw * v[s] < 0;
            const double lim = opposing ? p.DV_BRAKE : p.DV_ACCEL;
            n = std::clamp(raw, -lim, lim);
        }
        net[s] = n;
        applied[s] = coasting[s] ? 0.0 : p.KV * v[s] + n;
        const double scale = s == LEFT ? (opposing ? p.C_BRAKE_L : p.C_DRIVE_L) : (opposing ? p.C_BRAKE_R : p.C_DRIVE_R);
        const double ad = p.C * scale * n;
        double a;
        if (!moving) {
            if (std::fabs(ad) <= p.F) {
                v[s] = 0;
                continue;
            }
            a = ad - p.F * sgn(ad);
        } else {
            a = ad - p.F * sgn(v[s]);
        }
        double vn = v[s] + a * dt;
        // friction brings a side to rest; it never pushes it backwards
        if (moving && vn * v[s] < 0 && std::fabs(ad) <= p.F) vn = 0;
        v[s] = vn;
    }
    const double va[2] = {(vOld[0] + v[0]) / 2, (vOld[1] + v[1]) / 2};
    const double vf = (va[LEFT] + va[RIGHT]) / 2;
    const double dth = (va[LEFT] - va[RIGHT]) / p.TRACK_EFF * dt; // clockwise positive
    const double hm = heading + dth / 2;
    x += vf * dt * std::sin(hm);
    y += vf * dt * std::cos(hm);
    heading += dth;
    travel[LEFT] += va[LEFT] * dt;
    travel[RIGHT] += va[RIGHT] * dt;
    // tracking wheels: the vertical one (right of centre) loses travel when turning clockwise; the
    // horizontal one (in front) is pushed to the right. LemLib counts horizontal travel + to the left
    vertTravel += vf * dt - p.VERT_OFFSET * dth;
    horizTravelLeft += -p.HORIZ_OFFSET * dth;
}

void World::refreshRotation(RotationDev& r) {
    const double travelIn = r.isVertical ? vertTravel : horizTravelLeft;
    r.sample = int32_t(std::lround(travelIn / (M_PI * p.TRACK_WHEEL_D) * 36000.0));
    r.seq++;
    r.lastRefresh = now;
}

void World::events() {
    odomStrobe = false;
    for (auto& [port, d] : imu)
        if (posMod(long(now) - p.imuPhase, p.imuMs) == 0) d.sampleDeg = heading * 180.0 / M_PI;
    for (auto& [port, r] : rot)
        if (posMod(long(now) - p.rotPhase, r.periodMs) == 0) refreshRotation(r);
    if (odomRunning && posMod(long(now) - p.odomPhase, 10) == 0) {
        lemlib::update();
        odomUpdates++;
        odomStrobe = true;
        const int vp = vertPort();
        if (vp >= 0 && rot[vp].seq != lastVertSeqSeen) {
            odomFreshUpdates++;
            lastVertSeqSeen = rot[vp].seq;
        }
    }
    if (postEvents) postEvents(now);
}

int World::vertPort() const {
    for (auto& [port, r] : rot)
        if (r.bound && r.isVertical) return port;
    return -1;
}

double World::motorPositionRaw(const MotorPort& m) const {
    if (m.side < 0) return 0;
    const double rev = (travel[m.side] - m.tare) / (M_PI * p.DRIVE_WHEEL_D) * (600.0 / p.DRIVE_RPM);
    switch (m.units) {
        case 1: return rev;
        case 2: return rev * 300.0; // blue cartridge: 300 counts per motor revolution
        default: return rev * 360.0;
    }
}

double World::motorRpm(int side) const {
    return v[side] / (M_PI * p.DRIVE_WHEEL_D) * 60.0 * (600.0 / p.DRIVE_RPM);
}

} // namespace sim
