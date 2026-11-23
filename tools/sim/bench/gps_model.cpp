// The simulated V5 GPS sensor; see gps_model.hpp for what it models and why.
#include "gps_model.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include "world.hpp"

namespace sim {

namespace {
constexpr double INCHES_PER_METRE = 39.3701; // as in the design and the filter
// drift_mode 2: below this true camera speed (in/s) the robot counts as standing still (the attacker's number)
constexpr double STILL_SPEED_IN_S = 0.5;

struct GpsKey {
        const char* name;
        double GpsModelParams::*field;
};
const GpsKey GPS_KEYS[] = {
    {"mount_right_in", &GpsModelParams::mount_right_in},
    {"mount_forward_in", &GpsModelParams::mount_forward_in},
    {"mount_yaw_deg", &GpsModelParams::mount_yaw_deg},
    {"period_ms", &GpsModelParams::period_ms},
    {"phase_ms", &GpsModelParams::phase_ms},
    {"latency_ms", &GpsModelParams::latency_ms},
    {"white_sd_in", &GpsModelParams::white_sd_in},
    {"slow_sd_in", &GpsModelParams::slow_sd_in},
    {"slow_tau_s", &GpsModelParams::slow_tau_s},
    {"wall_sd_per_100in", &GpsModelParams::wall_sd_per_100in},
    {"heading_sd_deg", &GpsModelParams::heading_sd_deg},
    {"dead_zone_in", &GpsModelParams::dead_zone_in},
    {"blind_drift_in_s", &GpsModelParams::blind_drift_in_s},
    {"recover_tau_s", &GpsModelParams::recover_tau_s},
    {"outlier_prob", &GpsModelParams::outlier_prob},
    {"outlier_min_in", &GpsModelParams::outlier_min_in},
    {"outlier_max_in", &GpsModelParams::outlier_max_in},
    {"loss_drift", &GpsModelParams::loss_drift},
    {"no_strips", &GpsModelParams::no_strips},
    {"no_strip_x_in", &GpsModelParams::no_strip_x_in},
    {"no_strip_y_in", &GpsModelParams::no_strip_y_in},
    {"no_strip_heading_deg", &GpsModelParams::no_strip_heading_deg},
    {"mirror", &GpsModelParams::mirror},
    {"error_floor_m", &GpsModelParams::error_floor_m},
    {"strip_walls", &GpsModelParams::strip_walls},
    {"bias_x_in", &GpsModelParams::bias_x_in},
    {"bias_y_in", &GpsModelParams::bias_y_in},
    {"no_strip_at_truth", &GpsModelParams::no_strip_at_truth},
    // the attack options (from here on) are written to the JSON only when not 0, so a summary without them is
    // byte-identical to the one from before they existed
    {"drift_mode", &GpsModelParams::drift_mode},
    {"bias_step_in", &GpsModelParams::bias_step_in},
    {"bias_step_ms", &GpsModelParams::bias_step_ms},
    {"bias_step_start_ms", &GpsModelParams::bias_step_start_ms},
    {"bias_step_count", &GpsModelParams::bias_step_count},
    {"bias_step_dir_deg", &GpsModelParams::bias_step_dir_deg},
};
// how many keys came before the attack options
constexpr int FIRST_ATTACK_KEY = 28;

double wrap360(double deg) {
    double w = std::fmod(deg, 360.0);
    if (w < 0) w += 360.0;
    return w;
}
} // namespace

bool GpsModelParams::set(const std::string& key, double value) {
    for (const GpsKey& k : GPS_KEYS) {
        if (key == k.name) {
            this->*k.field = value;
            return true;
        }
    }
    return false;
}

std::string GpsModelParams::toJson() const {
    std::string s = "{";
    int index = 0;
    for (const GpsKey& k : GPS_KEYS) {
        if (index++ >= FIRST_ATTACK_KEY && this->*k.field == 0) continue;
        char buf[96];
        std::snprintf(buf, sizeof buf, "%s\"%s\":%.6g", s.size() > 1 ? "," : "", k.name, this->*k.field);
        s += buf;
    }
    return s + "}";
}

GpsModel& gpsModel() {
    static GpsModel model;
    return model;
}

double wallDistanceIn(double xIn, double yIn, double headingRad) {
    const double half = 72.0; // the field is 144 in square, origin at its centre
    if (!(std::fabs(xIn) < half) || !(std::fabs(yIn) < half)) return 0;
    const double dx = std::sin(headingRad); // heading is clockwise from +y
    const double dy = std::cos(headingRad);
    double best = INFINITY;
    if (dx > 1e-12) best = std::min(best, (half - xIn) / dx);
    if (dx < -1e-12) best = std::min(best, (-half - xIn) / dx);
    if (dy > 1e-12) best = std::min(best, (half - yIn) / dy);
    if (dy < -1e-12) best = std::min(best, (-half - yIn) / dy);
    return best;
}

int wallFacedBit(double xIn, double yIn, double headingRad) {
    const double half = 72.0;
    if (!(std::fabs(xIn) < half) || !(std::fabs(yIn) < half)) return 0;
    const double dx = std::sin(headingRad);
    const double dy = std::cos(headingRad);
    double toX = INFINITY, toY = INFINITY;
    if (dx > 1e-12) toX = (half - xIn) / dx;
    if (dx < -1e-12) toX = (-half - xIn) / dx;
    if (dy > 1e-12) toY = (half - yIn) / dy;
    if (dy < -1e-12) toY = (-half - yIn) / dy;
    if (toY <= toX) return dy > 0 ? 1 : 4;
    return dx > 0 ? 2 : 8;
}

GpsModel::Truth GpsModel::truthAt(long ms) const {
    if (history.empty()) return {0, 0, 0};
    if (ms < 0) return history.front();
    if (size_t(ms) >= history.size()) return history.back();
    return history[size_t(ms)];
}

const GpsViewLoss* GpsModel::viewLostAt(std::uint32_t ms) const {
    for (const GpsViewLoss& loss : losses)
        if (ms >= loss.startMs && ms < loss.startMs + loss.lengthMs) return &loss;
    return nullptr;
}

double GpsModel::outXMetres() const { return outX / INCHES_PER_METRE; }

double GpsModel::outYMetres() const { return outY / INCHES_PER_METRE; }

double GpsModel::errorVsNowIn(const World& w) const {
    const double s = std::sin(w.heading), c = std::cos(w.heading);
    const double camX = w.x + p.mount_right_in * c + p.mount_forward_in * s;
    const double camY = w.y - p.mount_right_in * s + p.mount_forward_in * c;
    return std::hypot(outX - camX, outY - camY);
}

double GpsModel::wallDistanceNowIn(const World& w) const {
    const double s = std::sin(w.heading), c = std::cos(w.heading);
    const double camX = w.x + p.mount_right_in * c + p.mount_forward_in * s;
    const double camY = w.y - p.mount_right_in * s + p.mount_forward_in * c;
    return wallDistanceIn(camX, camY, w.heading + p.mount_yaw_deg * M_PI / 180.0);
}

void GpsModel::refresh(const World& w) {
    // the truth, one entry per ms (fill any gap with the current truth)
    const Truth now {w.x, w.y, w.heading};
    if (!started) {
        bootTruth = now;
        started = true;
    }
    while (history.size() <= w.now) history.push_back(now);
    history[w.now] = now;

    const long period = std::max(1L, std::lround(p.period_ms));
    const long phase = std::lround(p.phase_ms);
    if ((((long(w.now) - phase) % period) + period) % period == 0) makeOutput(w.now);
}

void GpsModel::makeOutput(std::uint32_t now) {
    const double dtS = outputs == 0 ? p.period_ms / 1000.0 : (now - lastOutputMs) / 1000.0;

    // Draw every random number this output might need, always the same count (see gps_model.hpp)
    const double nWhiteX = rng.normal(), nWhiteY = rng.normal();
    const double nSlowX = rng.normal(), nSlowY = rng.normal();
    const double nWallX = rng.normal(), nWallY = rng.normal();
    const double nHeading = rng.normal();
    const double uOutlier = rng.uniform(), uOutlierSize = rng.uniform(), uOutlierDirection = rng.uniform();
    const double uDriftDirection = rng.uniform();
    const double uErrorFactor = rng.uniform();

    // where the camera was when this reading was taken, and which way it faced
    const Truth then = truthAt(long(now) - std::lround(p.latency_ms));
    const double s = std::sin(then.heading), c = std::cos(then.heading);
    const double camX = then.x + p.mount_right_in * c + p.mount_forward_in * s;
    const double camY = then.y - p.mount_right_in * s + p.mount_forward_in * c;
    const double facingRad = then.heading + p.mount_yaw_deg * M_PI / 180.0;
    const double wallIn = wallDistanceIn(camX, camY, facingRad);

    // slowly varying noise: first order, settles to slow_sd_in (starts already settled)
    if (outputs == 0) {
        slowX = p.slow_sd_in * nSlowX;
        slowY = p.slow_sd_in * nSlowY;
    } else {
        const double a = p.slow_tau_s > 0 ? std::exp(-dtS / p.slow_tau_s) : 0.0;
        const double b = std::sqrt(std::max(0.0, 1 - a * a)) * p.slow_sd_in;
        slowX = a * slowX + b * nSlowX;
        slowY = a * slowY + b * nSlowY;
    }

    GpsState st = GPS_SEES;
    const GpsViewLoss* loss = viewLostAt(now);
    // strips on only some walls: can the camera see a strip on the wall it faces? (always, with all four)
    const int stripWalls = int(std::lround(p.strip_walls));
    const bool facesStrip = stripWalls == 15 || (stripWalls & wallFacedBit(camX, camY, facingRad)) != 0;
    if (p.no_strips != 0) st = GPS_NO_STRIPS;
    else if (loss != nullptr) st = GPS_VIEW_LOST;
    else if (wallIn < p.dead_zone_in) st = GPS_DEAD_ZONE;
    else if (!facesStrip) st = GPS_STRIPLESS_WALL;
    const bool lossDrifts = loss != nullptr && (loss->drift >= 0 ? loss->drift != 0 : p.loss_drift != 0);

    const bool frozen = (st == GPS_NO_STRIPS && p.no_strips == 1) || (st == GPS_VIEW_LOST && !lossDrifts);
    double x = camX, y = camY;
    double headingDeg = then.heading * 180.0 / M_PI + p.mount_yaw_deg + p.heading_sd_deg * nHeading;
    bool isOutlier = false;
    double size = 0;

    if (frozen && outputs > 0 && st == GPS_VIEW_LOST) {
        // a frozen output repeats the last one exactly, so a reader sees nothing new
        x = outX;
        y = outY;
        headingDeg = outHeading;
    } else if (frozen) {
        // never saw a strip: frozen where it thinks it is at boot
        x = p.no_strip_x_in;
        y = p.no_strip_y_in;
        headingDeg = p.no_strip_heading_deg;
        if (p.no_strip_at_truth != 0 && st == GPS_NO_STRIPS) {
            // ...which is where the camera really was at boot, facing the way it really faced
            const double bs = std::sin(bootTruth.heading), bc = std::cos(bootTruth.heading);
            x = bootTruth.x + p.mount_right_in * bc + p.mount_forward_in * bs;
            y = bootTruth.y - p.mount_right_in * bs + p.mount_forward_in * bc;
            headingDeg = bootTruth.heading * 180.0 / M_PI + p.mount_yaw_deg;
        }
    } else if (st == GPS_NO_STRIPS) {
        // never saw a strip, dead-reckoning: it follows the camera's movement since boot, starting from where it
        // thinks it was at boot, plus a drift
        if (outputs == 0) {
            driftX = p.blind_drift_in_s * std::sin(2 * M_PI * uDriftDirection);
            driftY = p.blind_drift_in_s * std::cos(2 * M_PI * uDriftDirection);
            const double bs = std::sin(bootTruth.heading), bc = std::cos(bootTruth.heading);
            noStripX = p.no_strip_x_in - (bootTruth.x + p.mount_right_in * bc + p.mount_forward_in * bs);
            noStripY = p.no_strip_y_in - (bootTruth.y - p.mount_right_in * bs + p.mount_forward_in * bc);
            // told where it really started: no offset at all, only the drift from here on
            if (p.no_strip_at_truth != 0) noStripX = noStripY = 0;
        }
        noStripX += driftX * dtS;
        noStripY += driftY * dtS;
        x = camX + noStripX;
        y = camY + noStripY;
        const double bootHeadingDeg =
            p.no_strip_at_truth != 0 ? bootTruth.heading * 180.0 / M_PI + p.mount_yaw_deg : p.no_strip_heading_deg;
        headingDeg = bootHeadingDeg + (then.heading - bootTruth.heading) * 180.0 / M_PI + p.heading_sd_deg * nHeading;
    } else if (st == GPS_DEAD_ZONE || st == GPS_VIEW_LOST || st == GPS_STRIPLESS_WALL) {
        // blind: dead-reckon. The output follows the camera's real movement (the sensor's own IMU) but drifts
        // away, smoothly (the noise is held), and nothing says it is blind.
        // A view loss with its own drift rate starts its own drift when it begins, even if the camera was
        // already blind (in the dead zone, say)
        const int thisLoss = st == GPS_VIEW_LOST ? int(loss - losses.data()) : -1;
        const bool ownRate = thisLoss >= 0 && loss->driftInS >= 0;
        if (!wasBlind || (ownRate && thisLoss != lossIndex)) {
            const double rate = ownRate ? loss->driftInS : p.blind_drift_in_s;
            driftX = rate * std::sin(2 * M_PI * uDriftDirection);
            driftY = rate * std::cos(2 * M_PI * uDriftDirection);
        }
        wasBlind = true;
        bool drifts = true;
        if (std::lround(p.drift_mode) == 2) {
            // drift only while standing still: the camera's true speed over the 50 ms before the reading
            const Truth before = truthAt(long(now) - std::lround(p.latency_ms) - 50);
            const double camSpeed = std::hypot(then.x - before.x, then.y - before.y) / 0.05;
            drifts = camSpeed < STILL_SPEED_IN_S;
        }
        if (drifts) {
            blindX += driftX * dtS;
            blindY += driftY * dtS;
        }
        x = camX + heldNoiseX + blindX;
        y = camY + heldNoiseY + blindY;
    } else {
        // sees the strips: the truth plus noise. Whatever built up while blind fades away
        wasBlind = false;
        const double fade = p.recover_tau_s > 0 ? std::exp(-dtS / p.recover_tau_s) : 0.0;
        blindX *= fade;
        blindY *= fade;
        const double wallSd = p.wall_sd_per_100in * wallIn / 100.0;
        heldNoiseX = p.white_sd_in * nWhiteX + slowX + wallSd * nWallX;
        heldNoiseY = p.white_sd_in * nWhiteY + slowY + wallSd * nWallY;
        x = camX + heldNoiseX + blindX;
        y = camY + heldNoiseY + blindY;
        if (uOutlier < p.outlier_prob) {
            isOutlier = true;
            size = p.outlier_min_in + (p.outlier_max_in - p.outlier_min_in) * uOutlierSize;
            x += size * std::sin(2 * M_PI * uOutlierDirection);
            y += size * std::cos(2 * M_PI * uOutlierDirection);
        }
    }

    // get_error: loosely related to how wrong the sensor really is (before any mirroring: in a mirrored frame
    // the sensor itself is not wrong, the frame is). A frozen sensor repeats its last value
    if (!frozen || outputs == 0) {
        const double ownErrorIn = std::hypot(x - camX, y - camY);
        outError = ownErrorIn * (0.5 + uErrorFactor) / INCHES_PER_METRE + p.error_floor_m;
    }

    // a constant bias, on every output that follows the camera (after get_error, which does not know about it)
    if ((p.bias_x_in != 0 || p.bias_y_in != 0) && !frozen && st != GPS_NO_STRIPS) {
        x += p.bias_x_in;
        y += p.bias_y_in;
    }

    // a bias that grows in steps (bias_step_*), on the same outputs as the constant bias
    const double routeMs = double(now) - double(t0Ms);
    if (p.bias_step_in != 0 && p.bias_step_ms > 0 && routeMs >= p.bias_step_start_ms && !frozen && st != GPS_NO_STRIPS) {
        const double steps =
            std::min(p.bias_step_count, std::floor((routeMs - p.bias_step_start_ms) / p.bias_step_ms) + 1);
        const double d = p.bias_step_dir_deg * M_PI / 180.0;
        x += steps * p.bias_step_in * std::sin(d);
        y += steps * p.bias_step_in * std::cos(d);
    }

    // a mirrored field frame (the routine thinks it is on the other side). Not applied again to a repeated
    // frozen output, which is already in the mirrored frame
    const int mirror = (frozen && outputs > 0 && st == GPS_VIEW_LOST) ? 0 : int(std::lround(p.mirror));
    if (mirror == 1) {
        x = -x;
        y = -y;
        headingDeg += 180;
    } else if (mirror == 2) {
        x = -x;
        headingDeg = -headingDeg;
    } else if (mirror == 3) {
        y = -y;
        headingDeg = 180 - headingDeg;
    }

    outX = x;
    outY = y;
    outHeading = wrap360(headingDeg);
    outlier = isOutlier;
    outlierSize = size;
    state = st;
    lossIndex = st == GPS_VIEW_LOST ? int(loss - losses.data()) : -1;
    errDelayed = std::hypot(outX - camX, outY - camY);
    lastOutputMs = now;
    outputs++;
    if (isOutlier) outlierCount++;
    if (st == GPS_DEAD_ZONE) deadZoneOutputs++;
    if (st == GPS_VIEW_LOST) lostOutputs++;
    if (st == GPS_STRIPLESS_WALL) striplessOutputs++;
    if (st == GPS_SEES && !isOutlier) {
        sumSquaredErrorSeeing += errDelayed * errDelayed;
        seeingOutputs++;
    }
}

} // namespace sim
