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
        {"MU_LAT", &Params::MU_LAT},
        {"STICK_EPS", &Params::STICK_EPS},
        {"VERT_OFFSET", &Params::VERT_OFFSET},
        {"HORIZ_OFFSET", &Params::HORIZ_OFFSET},
        {"TRACK_WHEEL_D", &Params::TRACK_WHEEL_D},
        {"DRIVE_WHEEL_D", &Params::DRIVE_WHEEL_D},
        {"DRIVE_RPM", &Params::DRIVE_RPM},
        {"IMU_SCALE_ERR_CW", &Params::IMU_SCALE_ERR_CW},
        {"IMU_SCALE_ERR_CCW", &Params::IMU_SCALE_ERR_CCW},
        {"IMU_DRIFT_DEG_S", &Params::IMU_DRIFT_DEG_S},
        {"VERT_SCALE_ERR", &Params::VERT_SCALE_ERR},
        {"HORIZ_SCALE_ERR", &Params::HORIZ_SCALE_ERR},
        {"SLIP_PER_IN", &Params::SLIP_PER_IN},
        {"SLIP_MAX_IN", &Params::SLIP_MAX_IN},
        {"VERT_YAW_DEG", &Params::VERT_YAW_DEG},
        {"HORIZ_YAW_DEG", &Params::HORIZ_YAW_DEG},
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
    vSide = sideDist = peakSide = 0;
    vertTravel = horizTravelLeft = 0;
    cmd[0] = cmd[1] = 0;
    cmdHist[0].assign(1, 0);
    cmdHist[1].assign(1, 0);
    imuHeading = 0;
    vertSlip = horizSlip = 0;
    slips = 0;
}

void World::placeAt(double xIn, double yIn, double headingRad) {
    x = xIn;
    y = yIn;
    heading = headingRad;
    imuHeading = headingRad;
}

bool World::imuHasErrors() const {
    return p.IMU_SCALE_ERR_CW != 0 || p.IMU_SCALE_ERR_CCW != 0 || p.IMU_DRIFT_DEG_S != 0;
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
    const double omega = (va[LEFT] - va[RIGHT]) / p.TRACK_EFF; // rad/s, clockwise positive
    const double dth = omega * dt;

    // sideways slip. With no sideways force the robot keeps its world-frame velocity, so in the robot
    // frame the sideways speed changes at -omega * vf (it slides to the outside of a turn) and the
    // sideways speed turns into forward speed at +omega * vSide. Wheel scrub opposes sideways motion
    // with a Coulomb limit MU_LAT * g; at zero sideways speed it holds as long as the centripetal
    // acceleration needed, |omega * vf|, is within that limit (always, with the default MU_LAT)
    const double grip = p.MU_LAT * 386.09;
    const double vsOld = vSide;
    const double need = -omega * vf; // sideways acceleration if nothing held the robot
    if (vSide == 0 && std::fabs(need) <= grip) {
        // sticks: the wheels supply the centripetal force; vSide stays exactly 0
    } else {
        const double dir = vSide != 0 ? sgn(vSide) : sgn(need);
        double vn = vSide + (need - grip * dir) * dt;
        if (vSide != 0 && vn * vSide < 0 && std::fabs(need) <= grip) vn = 0; // scrub stops it, then it holds
        vSide = vn;
    }
    const double vsAvg = (vsOld + vSide) / 2;
    if (vsAvg != 0) {
        for (int s = 0; s < 2; s++) v[s] += omega * vsAvg * dt; // sideways momentum rotating into forward
    }
    sideDist += std::fabs(vsAvg) * dt;
    peakSide = std::max(peakSide, std::fabs(vSide));

    const double hm = heading + dth / 2;
    // world velocity = vf along the heading + vSide to its right; with vSide = 0 the extra terms are
    // exactly zero, so the result is bit-for-bit what it was before slip was modelled
    x += vf * dt * std::sin(hm) + vsAvg * dt * std::cos(hm);
    y += vf * dt * std::cos(hm) - vsAvg * dt * std::sin(hm);
    heading += dth;
    travel[LEFT] += va[LEFT] * dt;
    travel[RIGHT] += va[RIGHT] * dt;
    // tracking wheels: the vertical one (right of centre) loses travel when turning clockwise; the
    // horizontal one (in front) is pushed to the right by the turn and by sideways slip. LemLib counts
    // horizontal travel + to the left
    double vertStep = vf * dt - p.VERT_OFFSET * dth;
    double horizStep = -p.HORIZ_OFFSET * dth - vsAvg * dt;
    // A wheel that is not parallel to the robot (VERT_YAW_DEG / HORIZ_YAW_DEG; both 0 by default, and then this
    // is skipped and the steps above are exactly what they were). Each wheel stays where it is and rolls along a
    // line turned clockwise by its angle, so it reads the ground's motion under it along that line: forward
    // vf - omega * (inches right of centre), rightward vSide + omega * (inches in front of centre)
    if (p.VERT_YAW_DEG != 0 || p.HORIZ_YAW_DEG != 0) {
        const double a = p.VERT_YAW_DEG * M_PI / 180.0, b = p.HORIZ_YAW_DEG * M_PI / 180.0;
        vertStep = (vf * dt - p.VERT_OFFSET * dth) * std::cos(a) + vsAvg * dt * std::sin(a);
        horizStep = -(vsAvg * dt + p.HORIZ_OFFSET * dth) * std::cos(b) + vf * dt * std::sin(b);
    }
    vertTravel += vertStep;
    horizTravelLeft += horizStep;

    // ---- odometry error sources. All zero by default, and then nothing below changes what any sensor reads
    // The IMU: a scale error that can differ clockwise and counter-clockwise, and a drift. It is only read
    // (events()) when one of them is set; with a perfect IMU the sensor reads `heading` exactly as before
    const double imuScale = 1 + (dth > 0 ? p.IMU_SCALE_ERR_CW : p.IMU_SCALE_ERR_CCW);
    imuHeading += dth * imuScale + p.IMU_DRIFT_DEG_S * M_PI / 180.0 * dt;
    // Tracking-wheel slips: each wheel, each ms, has a chance SLIP_PER_IN * (inches it rolled) of slipping, and a
    // slip loses up to SLIP_MAX_IN of its travel (the wheel reads short, in the direction it was rolling)
    if (p.SLIP_PER_IN > 0) {
        const double steps[2] = {vertStep, horizStep};
        double* lost[2] = {&vertSlip, &horizSlip};
        for (int k = 0; k < 2; k++) {
            const double chance = rng.uniform(); // always two numbers per wheel: the stream stays in step
            const double size = rng.uniform() * p.SLIP_MAX_IN;
            if (chance < p.SLIP_PER_IN * std::fabs(steps[k])) {
                *lost[k] += size * sgn(steps[k]);
                slips++;
            }
        }
    }
    // Shoves: the robot is pushed sideways or back without its wheels or IMU noticing
    for (Shove& s : shoves) {
        if (now < s.startMs || now >= s.startMs + uint32_t(std::max(1, s.lengthMs))) continue;
        if (!s.started) {
            s.started = true;
            s.dxIn = s.rightIn * std::cos(heading) + s.forwardIn * std::sin(heading);
            s.dyIn = -s.rightIn * std::sin(heading) + s.forwardIn * std::cos(heading);
        }
        x += s.dxIn / std::max(1, s.lengthMs);
        y += s.dyIn / std::max(1, s.lengthMs);
    }
}

void World::refreshRotation(RotationDev& r) {
    double travelIn = r.isVertical ? vertTravel : horizTravelLeft;
    // odometry error sources: slips and a scale error (both zero by default, and then this is skipped)
    const double slip = r.isVertical ? vertSlip : horizSlip;
    const double scaleErr = r.isVertical ? p.VERT_SCALE_ERR : p.HORIZ_SCALE_ERR;
    if (slip != 0 || scaleErr != 0) travelIn = (travelIn - slip) * (1 + scaleErr);
    r.sample = int32_t(std::lround(travelIn / (M_PI * p.TRACK_WHEEL_D) * 36000.0));
    r.seq++;
    r.lastRefresh = now;
}

void World::events() {
    odomStrobe = false;
    const double imuReads = imuHasErrors() ? imuHeading : heading; // a perfect IMU reads the true heading
    for (auto& [port, d] : imu)
        if (posMod(long(now) - p.imuPhase, p.imuMs) == 0) d.sampleDeg = imuReads * 180.0 / M_PI;
    for (auto& [port, r] : rot)
        if (posMod(long(now) - p.rotPhase, r.periodMs) == 0) refreshRotation(r);
    if (deviceRefresh) deviceRefresh(now);
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
