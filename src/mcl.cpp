#include "mcl.hpp"

#include <cmath>
#include <cstring>

#include "pros/rtos.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "mclfield.hpp"
#include "robot-config.hpp"

namespace mcl {

namespace {

// ---------------------------------------
// Tuning
// ---------------------------------------

constexpr float DEG_TO_RAD = 0.017453292519943295f;

/** @brief Filter period. The V5 distance sensor refreshes about every 30ms */
constexpr int TICK_MS = 25;

// Motion model. Noise added per tick is proportional to how far the robot
// actually moved, plus a floor so a stationary robot's cloud never collapses to
// a point and loses the ability to recover
constexpr float ALPHA_TRANS = 0.08f;  // inches of noise per inch driven
constexpr float ALPHA_TURN = 0.05f;   // inches of noise per degree turned
constexpr float ALPHA_ROT = 0.05f;    // degrees of noise per degree turned
constexpr float TRANS_FLOOR = 0.03f;  // inches
constexpr float ROT_FLOOR = 0.03f;    // degrees

// Sensor model. A long reading is less trustworthy than a short one, so sigma
// grows with the measurement instead of being flat -- which is what lets the
// usable range stay wide open in mclfield.cpp without long readings dragging
// the filter around. (Which readings count as usable at all lives there too.)
constexpr float SENSOR_SIGMA = 1.5f;          // inches at point blank
constexpr float SENSOR_SIGMA_PER_IN = 0.03f;  // extra inches per inch of range
constexpr float HIT_WEIGHT = 0.90f;      // share of the model that is the Gaussian
constexpr float RAND_WEIGHT = 0.10f;     // uniform floor: stops one bad sensor
                                         // reading from zeroing every particle
constexpr float OUT_OF_BOUNDS_WEIGHT = 0.01f; // particle drifted off the field

// With no usable reading the filter is running on dead reckoning, so its
// agreement figure is stale. Decay it rather than leaving it standing, or the
// estimate keeps claiming confidence it has not earned since the last fix
constexpr float AGREEMENT_DECAY = 0.97f;

// Resample once the effective sample size falls below this share of the set
constexpr float RESAMPLE_RATIO = 0.5f;

// Augmented MCL (Probabilistic Robotics 8.3.3, and the reason the published VEX
// filters recover from a bad starting pose instead of confidently driving into
// a wall). Two exponential averages of how well the sensors agree with the
// cloud: wFast reacts in a few ticks, wSlow over a few seconds. When the recent
// agreement drops below the long-run agreement the filter is doing worse than
// it usually does, which is what being wrong looks like from the inside -- so
// that share of the set is replaced with random poses to give a locked-on-the-
// wrong-spot cloud somewhere to escape to
constexpr float ALPHA_SLOW = 0.01f;
constexpr float ALPHA_FAST = 0.20f;
constexpr float MAX_INJECT_RATIO = 0.35f; // never throw away more than this at once

// Cloud spread at which confidence reads 0. Neff is no use as a confidence
// metric -- it springs back to PARTICLE_COUNT the instant the set is resampled,
// whether or not the filter actually knows where the robot is. How tightly the
// particles are bunched does not, so that is what confidence is built from
constexpr float SPREAD_LOST_IN = 12.0f;

// Correction write-back. The filter has to clear the confidence bar for a run
// of consecutive ticks before it is allowed to touch the chassis pose, and one
// bad tick puts it back to zero. A filter that has locked onto the wrong part
// of a symmetric field flickers in and out of high confidence rather than
// holding it, so the hold is what separates "actually localised" from "briefly
// looks localised" -- and being slow to correct costs far less than correcting
// to the wrong place
constexpr float CORRECTION_GAIN = 0.15f;      // fraction of the error per tick
constexpr float CORRECTION_MIN_CONF = 0.55f;  // below this the filter stays quiet
constexpr int CORRECTION_HOLD_TICKS = 40;     // ~1 second at TICK_MS

// An odometry delta larger than this in one tick is not driving, it is a
// chassis.setPose() somewhere. Re-seed rather than smearing the cloud
constexpr float TELEPORT_IN = 6.0f;
constexpr float TELEPORT_DEG = 25.0f;

// ================= State =================

pros::Mutex stateMutex;
pros::Task* filterTask = nullptr;

Particle particles[PARTICLE_COUNT];
Estimate estimate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};

// per-sensor geometric mean likelihood, 0..1 after rescaling off RAND_WEIGHT
float agreement = 0.0f;
float wSlow = 0.0f;
float wFast = 0.0f;
int confidentTicks = 0; // consecutive ticks over CORRECTION_MIN_CONF

float lastMeasured[SENSOR_COUNT] = {-1.0f, -1.0f, -1.0f, -1.0f};
float lastExpected[SENSOR_COUNT] = {-1.0f, -1.0f, -1.0f, -1.0f};

lemlib::Pose prevOdom(0, 0, 0);
std::uint32_t updateCount = 0;
std::uint32_t lastDurationUs = 0;

bool correctionEnabled = false;
bool headingCorrectionEnabled = false;

// ---------------------------------------
// Random numbers
// ---------------------------------------

// xorshift32 rather than <random>: a Mersenne twister costs more state and more
// time than a particle filter running at 40Hz can spare
std::uint32_t rngState = 0x2654Eu;

inline std::uint32_t xorshift() {
    rngState ^= rngState << 13;
    rngState ^= rngState >> 17;
    rngState ^= rngState << 5;
    return rngState;
}

/** @brief Uniform in [0, 1) */
inline float uniform() { return static_cast<float>(xorshift() >> 8) * (1.0f / 16777216.0f); }

/** @brief Uniform in [-a, a] */
inline float uniform(float a) { return (uniform() * 2.0f - 1.0f) * a; }

/** @brief Standard normal, Marsaglia polar method (no sin/cos) */
float gauss() {
    static bool haveSpare = false;
    static float spare = 0.0f;
    if (haveSpare) {
        haveSpare = false;
        return spare;
    }
    float u, v, s;
    do {
        u = uniform() * 2.0f - 1.0f;
        v = uniform() * 2.0f - 1.0f;
        s = u * u + v * v;
    } while (s >= 1.0f || s == 0.0f);
    s = std::sqrt(-2.0f * std::log(s) / s);
    spare = v * s;
    haveSpare = true;
    return u * s;
}

// ---------------------------------------
// Geometry
// ---------------------------------------

/** @brief Shortest signed difference between two compass headings, degrees */
float angleDelta(float to, float from) {
    float d = std::fmod(to - from + 540.0f, 360.0f);
    if (d < 0) d += 360.0f;
    return d - 180.0f;
}

// ---------------------------------------
// Filter steps (all called with stateMutex held)
// ---------------------------------------

void seed(float x, float y, float theta, float xySigma, float thetaSigma) {
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        particles[i].x = x + gauss() * xySigma;
        particles[i].y = y + gauss() * xySigma;
        particles[i].theta = theta + gauss() * thetaSigma;
        particles[i].weight = 1.0f / PARTICLE_COUNT;
    }
    estimate = {x, y, theta, xySigma, 0.0f, false};
    confidentTicks = 0;
    agreement = 0.0f;
    wSlow = 0.0f;
    wFast = 0.0f;
}

/** @brief A pose drawn uniformly from the field, keeping the IMU's heading */
Particle randomParticle() {
    return {uniform(FIELD_HALF - 6.0f), uniform(FIELD_HALF - 6.0f),
            estimate.theta + gauss() * 5.0f, 1.0f / PARTICLE_COUNT};
}

void motionUpdate(const lemlib::Pose& cur) {
    const float gdx = cur.x - prevOdom.x;
    const float gdy = cur.y - prevOdom.y;
    const float dtheta = angleDelta(cur.theta, prevOdom.theta);

    // project the global delta back into the robot frame it was driven in
    const float prevRad = prevOdom.theta * DEG_TO_RAD;
    const float psn = std::sin(prevRad);
    const float pcs = std::cos(prevRad);
    const float fwd = gdx * psn + gdy * pcs;
    const float strafe = gdx * pcs - gdy * psn;

    const float dist = std::sqrt(gdx * gdx + gdy * gdy);
    const float sigmaT = ALPHA_TRANS * dist + ALPHA_TURN * std::fabs(dtheta) + TRANS_FLOOR;
    const float sigmaR = ALPHA_ROT * std::fabs(dtheta) + ROT_FLOOR;

    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        Particle& p = particles[i];
        const float nf = fwd + gauss() * sigmaT;
        const float ns = strafe + gauss() * sigmaT;
        const float rad = p.theta * DEG_TO_RAD;
        const float sn = std::sin(rad);
        const float cs = std::cos(rad);
        p.x += nf * sn + ns * cs;
        p.y += nf * cs - ns * sn;
        p.theta += dtheta + gauss() * sigmaR;
    }
}

/** @return true if at least one sensor contributed */
bool sensorUpdate() {
    float meas[SENSOR_COUNT];
    int valid = 0;
    for (int s = 0; s < SENSOR_COUNT; ++s) {
        meas[s] = field::readSensor(s);
        lastMeasured[s] = meas[s];
        if (meas[s] > 0.0f) ++valid;
    }
    if (valid == 0) {
        // nothing to weigh against this tick. Leave the weights alone, but stop
        // the confidence figure from coasting on evidence that is now old
        agreement *= AGREEMENT_DECAY;
        return false;
    }

    // sigma depends only on the reading, so it is hoisted out of the particle
    // loop -- this is 300 iterations of the hottest loop in the program
    float invTwoSigmaSq[SENSOR_COUNT];
    for (int s = 0; s < SENSOR_COUNT; ++s) {
        if (meas[s] < 0.0f) continue;
        const float sigma = SENSOR_SIGMA + SENSOR_SIGMA_PER_IN * meas[s];
        invTwoSigmaSq[s] = 0.5f / (sigma * sigma);
    }

    float total = 0.0f;
    float likeSum = 0.0f;
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        Particle& p = particles[i];
        float like = 1.0f;

        if (std::fabs(p.x) >= FIELD_HALF || std::fabs(p.y) >= FIELD_HALF) {
            // a particle that has drifted through a wall cannot be the robot
            like = OUT_OF_BOUNDS_WEIGHT;
        } else {
            for (int s = 0; s < SENSOR_COUNT; ++s) {
                if (meas[s] < 0.0f) continue;
                const float err = meas[s] - field::expectedRange(p, field::mounts[s]);
                const float g = std::exp(-err * err * invTwoSigmaSq[s]);
                like *= HIT_WEIGHT * g + RAND_WEIGHT;
            }
        }
        // likeSum is the likelihood alone, with the carried-in weights left out,
        // so the agreement figure below means the same thing tick to tick
        likeSum += like;
        p.weight *= like;
        total += p.weight;
    }

    // geometric mean per sensor, so a tick where only two sensors saw a wall is
    // comparable with one where all four did. Runs RAND_WEIGHT..1
    const float perSensor = std::pow(likeSum / PARTICLE_COUNT, 1.0f / valid);
    agreement = (perSensor - RAND_WEIGHT) / (1.0f - RAND_WEIGHT);
    if (agreement < 0.0f) agreement = 0.0f;
    if (agreement > 1.0f) agreement = 1.0f;
    if (wSlow <= 0.0f) {
        wSlow = wFast = perSensor;
    } else {
        wSlow += ALPHA_SLOW * (perSensor - wSlow);
        wFast += ALPHA_FAST * (perSensor - wFast);
    }

    if (total <= 0.0f) {
        // every hypothesis was ruled out -- keep the cloud rather than dividing
        // by zero, and let the next tick's readings sort it out
        for (int i = 0; i < PARTICLE_COUNT; ++i) particles[i].weight = 1.0f / PARTICLE_COUNT;
        return false;
    }
    for (int i = 0; i < PARTICLE_COUNT; ++i) particles[i].weight /= total;
    return true;
}

/** @brief Low-variance (systematic) resampling */
void resample() {
    static Particle buf[PARTICLE_COUNT]; // static: 300 particles is too much stack
    const float step = 1.0f / PARTICLE_COUNT;

    float injectRatio = (wSlow > 0.0f) ? 1.0f - wFast / wSlow : 0.0f;
    if (injectRatio < 0.0f) injectRatio = 0.0f;
    if (injectRatio > MAX_INJECT_RATIO) injectRatio = MAX_INJECT_RATIO;

    float u = uniform() * step;
    float c = particles[0].weight;
    int i = 0;
    for (int m = 0; m < PARTICLE_COUNT; ++m) {
        const float target = u + m * step;
        while (target > c && i < PARTICLE_COUNT - 1) {
            ++i;
            c += particles[i].weight;
        }
        if (injectRatio > 0.0f && uniform() < injectRatio) {
            buf[m] = randomParticle();
        } else {
            buf[m] = particles[i];
            buf[m].weight = step;
        }
    }
    std::memcpy(particles, buf, sizeof(particles));

    // the injected particles came from nowhere the averages know about; let the
    // fast average re-converge rather than injecting again on their account
    if (injectRatio > 0.0f) wFast = wSlow;
}

/** @brief 1/sum(w^2): how many particles are effectively carrying the weight */
float effectiveSampleSize() {
    float sw2 = 0.0f;
    for (int i = 0; i < PARTICLE_COUNT; ++i) sw2 += particles[i].weight * particles[i].weight;
    return (sw2 > 0.0f) ? 1.0f / sw2 : 0.0f;
}

/** @brief Weighted mean of the cloud, with headings averaged as unit vectors */
void computeEstimate() {
    float sx = 0.0f, sy = 0.0f, ssin = 0.0f, scos = 0.0f;
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        const Particle& p = particles[i];
        sx += p.x * p.weight;
        sy += p.y * p.weight;
        const float rad = p.theta * DEG_TO_RAD;
        ssin += std::sin(rad) * p.weight;
        scos += std::cos(rad) * p.weight;
    }
    estimate.x = sx;
    estimate.y = sy;
    // a plain mean of 359 and 1 is 180; averaging the unit vectors gives 0
    estimate.theta = std::atan2(ssin, scos) / DEG_TO_RAD;

    float var = 0.0f;
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        const float ex = particles[i].x - sx;
        const float ey = particles[i].y - sy;
        var += (ex * ex + ey * ey) * particles[i].weight;
    }
    estimate.spread = std::sqrt(var);
    // A tight cloud on its own means nothing: a filter that has locked onto the
    // wrong corner of the field is tight and wrong, and that is exactly the
    // state in which letting it correct the chassis pose does damage. Confidence
    // is therefore the worse of "the cloud agrees with itself" and "the cloud
    // agrees with the distance sensors"
    float tight = 1.0f - estimate.spread / SPREAD_LOST_IN;
    if (tight < 0.0f) tight = 0.0f;
    if (tight > 1.0f) tight = 1.0f;
    estimate.confidence = (tight < agreement) ? tight : agreement;
    estimate.valid = true;

    const Particle at = {estimate.x, estimate.y, estimate.theta, 1.0f};
    for (int s = 0; s < SENSOR_COUNT; ++s) lastExpected[s] = field::expectedRange(at, field::mounts[s]);
}

void applyCorrection(const lemlib::Pose& cur) {
    // counted even when correction is switched off, so the visualiser's LOCK
    // readout means the same thing either way
    if (estimate.valid && estimate.confidence >= CORRECTION_MIN_CONF) ++confidentTicks;
    else confidentTicks = 0;

    if (!correctionEnabled) return;
    if (confidentTicks < CORRECTION_HOLD_TICKS) return;

    const float nx = cur.x + (estimate.x - cur.x) * CORRECTION_GAIN;
    const float ny = cur.y + (estimate.y - cur.y) * CORRECTION_GAIN;
    const float nt = headingCorrectionEnabled
                         ? cur.theta + angleDelta(estimate.theta, cur.theta) * CORRECTION_GAIN
                         : cur.theta;
    chassis.setPose(nx, ny, nt);
}

void filterLoop(void*) {
    std::uint32_t now = pros::millis();
    while (true) {
        const std::uint32_t t0 = static_cast<std::uint32_t>(pros::micros());
        const lemlib::Pose cur = chassis.getPose();

        stateMutex.take(TIMEOUT_MAX);

        const float jump = std::sqrt((cur.x - prevOdom.x) * (cur.x - prevOdom.x) +
                                     (cur.y - prevOdom.y) * (cur.y - prevOdom.y));
        if (jump > TELEPORT_IN || std::fabs(angleDelta(cur.theta, prevOdom.theta)) > TELEPORT_DEG) {
            // somebody called chassis.setPose(): the robot did not drive there
            seed(cur.x, cur.y, cur.theta, 2.0f, 3.0f);
        } else {
            motionUpdate(cur);
            if (sensorUpdate() && effectiveSampleSize() < PARTICLE_COUNT * RESAMPLE_RATIO) resample();
            computeEstimate();
            applyCorrection(cur);
        }

        // read the pose back rather than reusing cur: applyCorrection may have
        // moved it, and counting that move again next tick would double it
        prevOdom = chassis.getPose();
        ++updateCount;
        lastDurationUs = static_cast<std::uint32_t>(pros::micros()) - t0;

        stateMutex.give();
        pros::Task::delay_until(&now, TICK_MS);
    }
}

} // namespace

// ================= Public API =================

void init() {
    const lemlib::Pose cur = chassis.getPose();
    stateMutex.take(TIMEOUT_MAX);
    rngState = 0x2654Eu ^ pros::millis();
    if (rngState == 0) rngState = 1;
    prevOdom = cur;
    updateCount = 0;
    seed(cur.x, cur.y, cur.theta, 2.0f, 3.0f);
    stateMutex.give();
}

void start() {
    if (filterTask != nullptr) return;
    filterTask = new pros::Task(filterLoop, nullptr, "mcl");
}

void stop() {
    if (filterTask == nullptr) return;
    filterTask->remove();
    delete filterTask;
    filterTask = nullptr;
}

bool isRunning() { return filterTask != nullptr; }

void setPose(float x, float y, float theta, float xySigma, float thetaSigma) {
    stateMutex.take(TIMEOUT_MAX);
    seed(x, y, theta, xySigma, thetaSigma);
    prevOdom = chassis.getPose();
    stateMutex.give();
}

void scatter() {
    const lemlib::Pose cur = chassis.getPose();
    stateMutex.take(TIMEOUT_MAX);
    // randomParticle() keeps the estimate's heading, which is the IMU's -- the
    // one thing odometry does not lose, so there is nothing to gain by
    // scattering it too
    estimate.theta = cur.theta;
    for (int i = 0; i < PARTICLE_COUNT; ++i) particles[i] = randomParticle();
    estimate.valid = false;
    estimate.confidence = 0.0f;
    estimate.spread = FIELD_HALF;
    confidentTicks = 0;
    agreement = 0.0f;
    wSlow = 0.0f;
    wFast = 0.0f;
    prevOdom = cur;
    stateMutex.give();
}

Estimate getEstimate() {
    stateMutex.take(TIMEOUT_MAX);
    const Estimate out = estimate;
    stateMutex.give();
    return out;
}

void snapshot(Snapshot& out) {
    const lemlib::Pose cur = chassis.getPose();
    stateMutex.take(TIMEOUT_MAX);
    std::memcpy(out.particles, particles, sizeof(particles));
    out.estimate = estimate;
    std::memcpy(out.measured, lastMeasured, sizeof(lastMeasured));
    std::memcpy(out.expected, lastExpected, sizeof(lastExpected));
    std::memcpy(out.mounts, field::mounts, sizeof(field::mounts));
    out.locked = confidentTicks >= CORRECTION_HOLD_TICKS;
    out.updates = updateCount;
    out.usPerUpdate = lastDurationUs;
    stateMutex.give();
    out.odomX = cur.x;
    out.odomY = cur.y;
    out.odomTheta = cur.theta;
}

void setCorrectionEnabled(bool state) { correctionEnabled = state; }

bool isCorrectionEnabled() { return correctionEnabled; }

void setHeadingCorrectionEnabled(bool state) { headingCorrectionEnabled = state; }

bool isHeadingCorrectionEnabled() { return headingCorrectionEnabled; }

} // namespace mcl
