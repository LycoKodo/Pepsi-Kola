// The simulated V5 distance sensor; see dist_model.hpp for what it models and README.md for how to use it.
//
// Why the cone model defaults to a blend (cone_model 2, cone_blend 0.5). Nobody has published whether the V5
// sensor, looking at a flat wall at an angle, reports the range along the middle of its beam (the "centre ray",
// D / cos(angle) for a wall D inches away) or the nearest point its beam touches (D itself while the angle is
// inside the cone). A time-of-flight sensor times the light coming back from everything its cone lights up, so
// what it reports lies somewhere between the two, nearer the near edge, whose light comes back strongest. The
// filter predicts the centre ray. With the blend the sensor and the filter disagree a little at an angle (0.07 in
// at 24 in and 6 degrees off square), as they will on the robot; the two pure models are there to bracket the
// truth (centre ray: the filter's own assumption, no disagreement; nearest point: the worst case, where anything
// anywhere in the cone pulls the reading all the way). The same rule decides what a corner, an object or a
// wall-mounted element at the edge of the cone does: nothing with the centre ray, everything with the nearest
// point, half of it with the blend.
#include "dist_model.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace sim {

namespace {

constexpr double MM_PER_IN = 25.4;
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / M_PI;
// The cone is searched with this many rays spread evenly across it, both edges and the centre included. They
// give the object size (how much of the cone a target fills) and the cone mask. The nearest point does not depend
// on them: it is found exactly from the few directions where it can be (see cone())
constexpr int EVEN_RAYS = 49;
constexpr int MAX_RAYS = 128; // room for the even rays plus those directions
constexpr double CONF_MEANINGFUL_ABOVE_MM = 200; // PROS: confidence is only available above 200 mm
constexpr double CONF_TOP = 63; // highest confidence PROS reports
constexpr double SIZE_TOP = 400; // largest object size PROS reports
constexpr double MAX_LATENCY_MS = 500; // the pose history holds 512 updates
constexpr double TINY = 1e-12; // a direction component smaller than this counts as zero
constexpr double SAME_RANGE_IN = 1e-9; // a ray must be shorter than this to beat the centre ray as "nearest"

double wrapPi(double a) {
    a = std::fmod(a + M_PI, 2 * M_PI);
    if (a < 0) a += 2 * M_PI;
    return a - M_PI;
}

double wrap360(double deg) {
    double w = std::fmod(deg, 360.0);
    if (w < 0) w += 360.0;
    return w;
}

// mixes a seed into a well-spread 64-bit number, so that seeds 1, 2, 3 and sensors 0, 1 give unrelated streams
std::uint64_t splitmix64(std::uint64_t z) {
    z += 0x9E3779B97F4A7C15ULL;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

// ------------------------------------------------------------------------------------------------ the options
struct GlobalKey {
        const char* name;
        double DistModelParams::*field;
        const char* help;
};
const GlobalKey GLOBAL_KEYS[] = {
    {"period_ms", &DistModelParams::period_ms, "a new reading every this many ms (about 30 a second); 0 = a new one at every update()"},
    {"latency_ms", &DistModelParams::latency_ms, "a reading shows where the robot was this long ago (0 = none, at most 500)"},
    {"cone_model", &DistModelParams::cone_model, "range off a surface in the cone: 0 centre ray, 1 nearest point in the cone, 2 blend"},
    {"cone_blend", &DistModelParams::cone_blend, "blend: 0 = nearest point, 1 = centre ray, 0.5 = halfway"},
    {"cone_half_deg", &DistModelParams::cone_half_deg, "half-angle of the beam's cone, degrees, at long range"},
    {"cone_half_close_deg", &DistModelParams::cone_half_close_deg, "half-angle at short range (centre ray shorter than cone_close_mm)"},
    {"cone_close_mm", &DistModelParams::cone_close_mm, "below this range the short-range half-angle is used, mm"},
    {"min_mm", &DistModelParams::min_mm, "closer than this the sensor cannot measure, mm"},
    {"below_min_value", &DistModelParams::below_min_value, "what it reads when closer than min_mm (0 = VEXos's 'no valid reading')"},
    {"max_mm", &DistModelParams::max_mm, "farther than this it reads 9999, mm"},
    {"noise_mm", &DistModelParams::noise_mm, "noise new every reading, fixed part, mm (1 standard deviation)"},
    {"noise_frac", &DistModelParams::noise_frac, "... plus this fraction of the range (0.01 = 1 percent)"},
    {"slow_mm", &DistModelParams::slow_mm, "slowly varying noise, fixed part, mm (1 standard deviation once settled)"},
    {"slow_frac", &DistModelParams::slow_frac, "... plus this fraction of the range"},
    {"slow_tau_s", &DistModelParams::slow_tau_s, "... which changes over about this many seconds"},
    {"dropout_prob", &DistModelParams::dropout_prob, "chance, each reading, of a 9999 for no reason"},
    {"dropout_incidence_deg", &DistModelParams::dropout_incidence_deg, "centre ray more than this off square to the surface: 9999 (0 = never)"},
    {"conf_model", &DistModelParams::conf_model, "1 = confidence falls with range and angle; 0 = 63 (above 200 mm)"},
    {"conf_per_m", &DistModelParams::conf_per_m, "confidence lost per metre of range"},
    {"conf_per_deg", &DistModelParams::conf_per_deg, "confidence lost per degree the centre ray is off square"},
    {"conf_sd", &DistModelParams::conf_sd, "random part of the confidence, new every reading (1 standard deviation)"},
    {"conf_below_200", &DistModelParams::conf_below_200, "confidence at 200 mm or closer, where PROS says it means nothing"},
    {"size_full", &DistModelParams::size_full, "object size of a target that fills the whole cone (a wall)"},
    {"size_sd", &DistModelParams::size_sd, "random part of the object size, new every reading"},
    {"stop_obj", &DistModelParams::stop_obj, "object in the beam while nearly stopped: 0 off, 1 in front of the sensor, 2 in front of the wall"},
    {"stop_obj_min_in", &DistModelParams::stop_obj_min_in, "its distance, inches along the centre ray, drawn between min and max at each stop"},
    {"stop_obj_max_in", &DistModelParams::stop_obj_max_in, "... (min = max for a fixed distance)"},
    {"stop_obj_gap_in", &DistModelParams::stop_obj_gap_in, "mode 1: keep it at least this far in front of the wall (the draw's top is lowered to fit)"},
    {"stop_obj_r_in", &DistModelParams::stop_obj_r_in, "its radius, inches (its near side is on the centre ray)"},
    {"stop_obj_sensors", &DistModelParams::stop_obj_sensors, "which sensors get one, added up: 1 sensor 0, 2 sensor 1, 3 both"},
    {"stop_obj_from_ms", &DistModelParams::stop_obj_from_ms, "no stop object before this time (the time given to update(), ms); 0 = from the start"},
    {"stop_speed_in_s", &DistModelParams::stop_speed_in_s, "'nearly stopped': slower than this, in/s..."},
    {"stop_turn_deg_s", &DistModelParams::stop_turn_deg_s, "... turning slower than this, deg/s..."},
    {"stop_window_ms", &DistModelParams::stop_window_ms, "... for at least this long, ms"},
};

struct SensorKey {
        const char* suffix;
        double DistSensorParams::*field;
        const char* help;
};
const SensorKey SENSOR_KEYS[] = {
    {"enabled", &DistSensorParams::enabled, "1 = fitted; 0 = not fitted (reads PROS_ERR)"},
    {"right_in", &DistSensorParams::right_in, "the sensor's face, inches right of the tracking centre"},
    {"forward_in", &DistSensorParams::forward_in, "... inches in front of it"},
    {"yaw_deg", &DistSensorParams::yaw_deg, "which way it points, degrees clockwise from the robot's front (180 back, 90 right)"},
    {"scale_err", &DistSensorParams::scale_err, "reads (1 + this) times the true range"},
    {"offset_mm", &DistSensorParams::offset_mm, "... plus this many mm"},
    {"phase_ms", &DistSensorParams::phase_ms, "its readings come when (t - phase_ms) is a multiple of period_ms"},
};

struct WallKey {
        const char* suffix;
        double DistWallParams::*field;
        const char* help;
};
const WallKey WALL_KEYS[] = {
    {"in", &DistWallParams::in, "the TRUE inside face, inches from the field centre (N: 0 +y, 1 +x, 2 -y, 3 -x)"},
    {"clear", &DistWallParams::clear, "1 = clear plastic: the five options below apply"},
    {"through_prob", &DistWallParams::through_prob, "clear: chance, each reading, that the beam passes through (1 = always)"},
    {"behind_in", &DistWallParams::behind_in, "clear, through: reads something this many inches behind the wall..."},
    {"behind_spread_in", &DistWallParams::behind_spread_in, "... plus a random 0 to this many inches, new each reading"},
    {"conf_drop", &DistWallParams::conf_drop, "clear: confidence this much lower, through or not"},
    {"dropout_in", &DistWallParams::dropout_in, "clear: no return (9999) when farther than this, inches (0 = never)"},
};

struct ElemKey {
        const char* suffix;
        double DistElementParams::*field;
        const char* help;
};
const ElemKey ELEM_KEYS[] = {
    {"wall", &DistElementParams::wall, "wall-mounted element on wall 0 to 3; -1 = none"},
    {"from_in", &DistElementParams::from_in, "where it starts along the wall, field inches (x on walls 0 and 2, y on 1 and 3)"},
    {"to_in", &DistElementParams::to_in, "... and ends"},
    {"depth_in", &DistElementParams::depth_in, "how far it sticks out from the wall, inches (reads this much closer)"},
};

struct ObjKey {
        const char* suffix;
        double DistObjectParams::*field;
        const char* help;
};
const ObjKey OBJ_KEYS[] = {
    {"x", &DistObjectParams::x, "circle on the field: centre, field inches"},
    {"y", &DistObjectParams::y, "..."},
    {"r", &DistObjectParams::r, "... radius, inches (0 = none)"},
    {"from_ms", &DistObjectParams::from_ms, "there from this time (the time given to update())..."},
    {"to_ms", &DistObjectParams::to_ms, "... until this time (below 0 = forever)"},
};

// "s1_yaw_deg" with prefix "s": index 1, suffix "yaw_deg". false if the key is not of that shape or the index is
// too big
bool splitIndexed(const std::string& key, const char* prefix, int count, int& index, std::string& suffix) {
    const size_t n = std::strlen(prefix);
    if (key.size() <= n || key.compare(0, n, prefix) != 0) return false;
    size_t i = n;
    int value = 0, digits = 0;
    while (i < key.size() && std::isdigit(static_cast<unsigned char>(key[i]))) {
        value = value * 10 + (key[i] - '0');
        i++;
        if (++digits > 3) return false;
    }
    if (digits == 0 || i >= key.size() || key[i] != '_' || value >= count) return false;
    index = value;
    suffix = key.substr(i + 1);
    return true;
}

double* findField(DistModelParams& p, const std::string& key) {
    for (const GlobalKey& k : GLOBAL_KEYS)
        if (key == k.name) return &(p.*k.field);
    int i = 0;
    std::string suffix;
    if (splitIndexed(key, "s", DIST_MAX_SENSORS, i, suffix))
        for (const SensorKey& k : SENSOR_KEYS)
            if (suffix == k.suffix) return &(p.sensor[i].*k.field);
    if (splitIndexed(key, "wall", DIST_WALL_COUNT, i, suffix))
        for (const WallKey& k : WALL_KEYS)
            if (suffix == k.suffix) return &(p.wall[i].*k.field);
    if (splitIndexed(key, "elem", DIST_MAX_ELEMENTS, i, suffix))
        for (const ElemKey& k : ELEM_KEYS)
            if (suffix == k.suffix) return &(p.elem[i].*k.field);
    if (splitIndexed(key, "obj", DIST_MAX_OBJECTS, i, suffix))
        for (const ObjKey& k : OBJ_KEYS)
            if (suffix == k.suffix) return &(p.obj[i].*k.field);
    return nullptr;
}

std::string number(double v) {
    char buf[48];
    std::snprintf(buf, sizeof buf, "%.6g", v);
    return buf;
}

} // namespace

bool DistModelParams::set(const std::string& key, double value) {
    double* f = findField(*this, key);
    if (f == nullptr) return false;
    *f = value;
    return true;
}

bool DistModelParams::get(const std::string& key, double& value) const {
    const double* f = findField(const_cast<DistModelParams&>(*this), key);
    if (f == nullptr) return false;
    value = *f;
    return true;
}

std::vector<std::string> DistModelParams::allKeys() {
    std::vector<std::string> keys;
    for (const GlobalKey& k : GLOBAL_KEYS) keys.push_back(k.name);
    for (int i = 0; i < DIST_MAX_SENSORS; i++)
        for (const SensorKey& k : SENSOR_KEYS) keys.push_back("s" + std::to_string(i) + "_" + k.suffix);
    for (int i = 0; i < DIST_WALL_COUNT; i++)
        for (const WallKey& k : WALL_KEYS) keys.push_back("wall" + std::to_string(i) + "_" + k.suffix);
    for (int i = 0; i < DIST_MAX_ELEMENTS; i++)
        for (const ElemKey& k : ELEM_KEYS) keys.push_back("elem" + std::to_string(i) + "_" + k.suffix);
    for (int i = 0; i < DIST_MAX_OBJECTS; i++)
        for (const ObjKey& k : OBJ_KEYS) keys.push_back("obj" + std::to_string(i) + "_" + k.suffix);
    return keys;
}

std::string DistModelParams::toJson() const {
    std::string s = "{";
    for (const std::string& key : allKeys()) {
        int i = 0;
        std::string suffix;
        // elements and objects only when they are used, or the summary drowns in zeros
        if (splitIndexed(key, "elem", DIST_MAX_ELEMENTS, i, suffix) && elem[i].wall < 0) continue;
        if (splitIndexed(key, "obj", DIST_MAX_OBJECTS, i, suffix) && obj[i].r <= 0) continue;
        double v = 0;
        get(key, v);
        s += (s.size() > 1 ? ",\"" : "\"") + key + "\":" + number(v);
    }
    return s + "}";
}

std::string DistModelParams::helpText() {
    const DistModelParams d; // the defaults
    std::string s = "Distance sensor model options (KEY=VALUE; the default after '='):\n";
    auto line = [&](const std::string& left, const char* help) {
        std::string l = "  " + left;
        if (l.size() < 40) l.resize(40, ' ');
        s += l + " " + help + "\n";
    };
    for (const GlobalKey& k : GLOBAL_KEYS) line(std::string(k.name) + "=" + number(d.*k.field), k.help);
    for (const SensorKey& k : SENSOR_KEYS) {
        std::string left;
        for (int i = 0; i < DIST_MAX_SENSORS; i++)
            left += (i ? " " : "") + ("s" + std::to_string(i) + "_" + k.suffix + "=" + number(d.sensor[i].*k.field));
        line(left, k.help);
    }
    for (const WallKey& k : WALL_KEYS) line("wallN_" + std::string(k.suffix) + "=" + number(d.wall[0].*k.field), k.help);
    for (const ElemKey& k : ELEM_KEYS) line("elemN_" + std::string(k.suffix) + "=" + number(d.elem[0].*k.field), k.help);
    for (const ObjKey& k : OBJ_KEYS) line("objN_" + std::string(k.suffix) + "=" + number(d.obj[0].*k.field), k.help);
    s += "  (wallN: N = 0 to 3; elemN: N = 0 to 7; objN: N = 0 to 15)\n";
    return s;
}

// ------------------------------------------------------------------------------------------------ the model
void DistModel::reseed(std::uint64_t seed) {
    for (int i = 0; i < DIST_MAX_SENSORS; i++) rng[i].reseed(splitmix64(splitmix64(seed) + std::uint64_t(i)));
}

void DistModel::reset() {
    for (SensorState& s : st) s = SensorState {};
    ringNext = ringCount = 0;
    haveLast = false;
    slowNow = stopped = false;
    slowSinceMs = 0;
}

void DistModel::pushPose(std::uint32_t nowMs, double x, double y, double h) {
    if (ringCount > 0) {
        PoseSample& newest = ring[(ringNext - 1 + RING_SIZE) % RING_SIZE];
        if (newest.t == nowMs) { // called twice in one ms: keep the later pose
            newest = {nowMs, x, y, h};
            return;
        }
    }
    ring[ringNext] = {nowMs, x, y, h};
    ringNext = (ringNext + 1) % RING_SIZE;
    if (ringCount < RING_SIZE) ringCount++;
}

DistModel::PoseSample DistModel::poseAt(double tMs) const {
    PoseSample s = ring[(ringNext - 1 + RING_SIZE) % RING_SIZE];
    for (int k = 0; k < ringCount; k++) {
        s = ring[(ringNext - 1 - k + 2 * RING_SIZE) % RING_SIZE];
        if (double(s.t) <= tMs + 1e-9) return s; // the newest pose at or before tMs
    }
    return s; // tMs is older than the history: the oldest pose there is
}

void DistModel::update(std::uint32_t nowMs, double xIn, double yIn, double headingRad) {
    // Nearly stopped? From how far the robot went since the last update. Its own rule, looser than the filter's
    // "still", so that a stop object is always in place before the filter could take a fix
    if (haveLast && nowMs > last.t) {
        const double dtS = (nowMs - last.t) / 1000.0;
        const double speed = std::hypot(xIn - last.x, yIn - last.y) / dtS;
        const double turn = std::fabs(wrapPi(headingRad - last.h)) * RAD_TO_DEG / dtS;
        const bool slow = speed <= p.stop_speed_in_s && turn <= p.stop_turn_deg_s;
        if (slow && !slowNow) slowSinceMs = last.t;
        slowNow = slow;
    }
    stopped = slowNow && double(nowMs - slowSinceMs) >= p.stop_window_ms;
    pushPose(nowMs, xIn, yIn, headingRad);
    last = {nowMs, xIn, yIn, headingRad};
    haveLast = true;

    // each sensor makes a reading when one is due (the first update always makes one)
    for (int i = 0; i < DIST_MAX_SENSORS; i++) {
        const double period = p.period_ms;
        if (period > 0 && double(nowMs) + 1e-6 < st[i].nextDueMs) continue;
        makeReading(i, nowMs);
        if (period > 0) {
            const double phase = p.sensor[i].phase_ms;
            const double k = std::floor((double(nowMs) - phase) / period + 1e-9) + 1;
            st[i].nextDueMs = phase + k * period;
        }
    }
}

DistReading DistModel::read(int sensor) const {
    if (sensor < 0 || sensor >= DIST_MAX_SENSORS || p.sensor[sensor].enabled == 0)
        return {DIST_PROS_ERR, DIST_PROS_ERR, DIST_PROS_ERR}; // what an empty port returns
    return st[sensor].reading;
}

const DistOutputInfo& DistModel::info(int sensor) const {
    static const DistOutputInfo none;
    if (sensor < 0 || sensor >= DIST_MAX_SENSORS) return none;
    return st[sensor].info;
}

void DistModel::sensorPose(int i, double x, double y, double h, double& sx, double& sy, double& phi) const {
    const DistSensorParams& sp = p.sensor[i];
    const double s = std::sin(h), c = std::cos(h);
    // right is (cos h, -sin h) and forward is (sin h, cos h) in the field frame
    sx = x + sp.right_in * c + sp.forward_in * s;
    sy = y - sp.right_in * s + sp.forward_in * c;
    phi = h + sp.yaw_deg * DEG_TO_RAD;
}

DistModel::Scene DistModel::scene(std::uint32_t nowMs, int stopObjectsOf) const {
    Scene s;
    for (int w = 0; w < DIST_WALL_COUNT; w++) s.wallPos[w] = p.wall[w].in;
    for (int e = 0; e < DIST_MAX_ELEMENTS; e++) {
        const DistElementParams& el = p.elem[e];
        const long wall = std::lround(el.wall);
        if (wall < 0 || wall > 3 || !(el.depth_in > 0) || el.to_in == el.from_in) continue;
        const double lo = std::min(el.from_in, el.to_in), hi = std::max(el.from_in, el.to_in);
        const double W = s.wallPos[wall], d = el.depth_in;
        Box b {0, 0, 0, 0, e};
        if (wall == 0) b = {lo, hi, W - d, W, e}; // +y wall: along x, sticks out toward -y
        if (wall == 1) b = {W - d, W, lo, hi, e}; // +x wall: along y, sticks out toward -x
        if (wall == 2) b = {lo, hi, -W, -W + d, e}; // -y wall
        if (wall == 3) b = {-W, -W + d, lo, hi, e}; // -x wall
        s.box[s.boxes++] = b;
    }
    for (int o = 0; o < DIST_MAX_OBJECTS; o++) {
        const DistObjectParams& ob = p.obj[o];
        if (!(ob.r > 0) || double(nowMs) < ob.from_ms || (ob.to_ms >= 0 && double(nowMs) >= ob.to_ms)) continue;
        s.circle[s.circles++] = {ob.x, ob.y, ob.r, DIST_HIT_OBJECT, o};
    }
    if (stopObjectsOf >= 0 && stopObjectsOf < DIST_MAX_SENSORS) {
        const StopObject& so = st[stopObjectsOf].stop;
        if (so.active && so.placed) s.circle[s.circles++] = {so.x, so.y, so.r, DIST_HIT_STOP_OBJECT, stopObjectsOf};
    }
    return s;
}

bool DistModel::insideField(const Scene& s, double sx, double sy) {
    return sy < s.wallPos[0] && sx < s.wallPos[1] && sy > -s.wallPos[2] && sx > -s.wallPos[3];
}

// The first thing a ray from (sx, sy) along `heading` meets: a wall, an element, or (withObjects) a circle.
// From inside the field the four wall planes are enough: the nearest plane the ray crosses is the wall it meets.
// A ray that starts inside an element or a circle meets it at range 0
DistModel::Hit DistModel::castRay(const Scene& s, double sx, double sy, double heading, bool withObjects) {
    const double dx = std::sin(heading), dy = std::cos(heading);
    Hit best;
    best.t = INFINITY;
    auto consider = [&](double t, int kind, int index, double nx, double ny) {
        if (t >= 0 && t < best.t) best = {t, kind, index, nx, ny};
    };
    // walls; each normal points into the field
    if (dy > TINY) consider((s.wallPos[0] - sy) / dy, DIST_HIT_WALL, 0, 0, -1);
    if (dx > TINY) consider((s.wallPos[1] - sx) / dx, DIST_HIT_WALL, 1, -1, 0);
    if (dy < -TINY) consider((-s.wallPos[2] - sy) / dy, DIST_HIT_WALL, 2, 0, 1);
    if (dx < -TINY) consider((-s.wallPos[3] - sx) / dx, DIST_HIT_WALL, 3, 1, 0);
    // elements: boxes, by the slab method (the ray is inside the box between where it enters both slabs and
    // where it leaves either)
    for (int k = 0; k < s.boxes; k++) {
        const Box& b = s.box[k];
        double tEnter = -INFINITY, tExit = INFINITY, nx = 0, ny = 0;
        bool miss = false;
        if (std::fabs(dx) < TINY) {
            if (sx < b.x0 || sx > b.x1) miss = true;
        } else {
            const double t1 = (b.x0 - sx) / dx, t2 = (b.x1 - sx) / dx;
            if (std::min(t1, t2) > tEnter) {
                tEnter = std::min(t1, t2);
                nx = dx > 0 ? -1 : 1; // it enters through the face that looks back at it
                ny = 0;
            }
            tExit = std::min(tExit, std::max(t1, t2));
        }
        if (std::fabs(dy) < TINY) {
            if (sy < b.y0 || sy > b.y1) miss = true;
        } else {
            const double t1 = (b.y0 - sy) / dy, t2 = (b.y1 - sy) / dy;
            if (std::min(t1, t2) > tEnter) {
                tEnter = std::min(t1, t2);
                nx = 0;
                ny = dy > 0 ? -1 : 1;
            }
            tExit = std::min(tExit, std::max(t1, t2));
        }
        if (!miss && tEnter <= tExit && tExit >= 0) consider(std::max(0.0, tEnter), DIST_HIT_ELEMENT, b.index, nx, ny);
    }
    if (!withObjects) return best;
    // circles
    for (int k = 0; k < s.circles; k++) {
        const Circle& c = s.circle[k];
        const double ox = sx - c.x, oy = sy - c.y;
        const double b = dx * ox + dy * oy;
        const double cc = ox * ox + oy * oy - c.r * c.r;
        if (cc <= 0) {
            consider(0, c.kind, c.index, -dx, -dy); // the sensor is inside it
            continue;
        }
        const double disc = b * b - cc;
        if (disc < 0) continue;
        const double t = -b - std::sqrt(disc);
        if (t >= 0) consider(t, c.kind, c.index, (sx + t * dx - c.x) / c.r, (sy + t * dy - c.y) / c.r);
    }
    return best;
}

namespace {
int maskBit(int kind, int index) {
    if (kind == DIST_HIT_WALL) return DIST_CONE_WALL0 << index;
    if (kind == DIST_HIT_ELEMENT) return DIST_CONE_ELEMENT;
    if (kind == DIST_HIT_OBJECT) return DIST_CONE_OBJECT;
    if (kind == DIST_HIT_STOP_OBJECT) return DIST_CONE_STOP_OBJECT;
    return 0;
}
} // namespace

// The centre ray, the nearest point inside the cone (half-angle halfRad either side of phi), and what the rays in
// the cone meet. The nearest point is found exactly: within an angle range, the nearest point of a wall plane is
// square to it or at an edge of the range, that of a box is square to a face, at a corner or at an edge, and that
// of a circle is toward its centre or at an edge. So those directions, plus the evenly spread rays (which include
// both edges), are all that need casting. Because the nearest point over all targets is the nearest over each
// target's nearest point, taking the shortest first hit over these rays is exact
DistModel::ConeResult DistModel::cone(const Scene& s, double sx, double sy, double phi, double halfRad) {
    ConeResult r;
    double dirs[MAX_RAYS];
    int n = 0;
    for (int k = 0; k < EVEN_RAYS; k++) dirs[n++] = phi - halfRad + 2 * halfRad * k / (EVEN_RAYS - 1);
    auto add = [&](double a) {
        if (n < MAX_RAYS && std::fabs(wrapPi(a - phi)) <= halfRad + 1e-12) dirs[n++] = a;
    };
    for (int w = 0; w < DIST_WALL_COUNT; w++) add(w * M_PI / 2); // square to each wall and each box face
    for (int k = 0; k < s.boxes; k++) {
        const Box& b = s.box[k];
        add(std::atan2(b.x0 - sx, b.y0 - sy));
        add(std::atan2(b.x0 - sx, b.y1 - sy));
        add(std::atan2(b.x1 - sx, b.y0 - sy));
        add(std::atan2(b.x1 - sx, b.y1 - sy));
    }
    for (int k = 0; k < s.circles; k++) add(std::atan2(s.circle[k].x - sx, s.circle[k].y - sy));

    r.centre = castRay(s, sx, sy, phi, true);
    r.nearest = r.centre; // on a tie the centre ray wins, so a square wall is "the same target" exactly
    r.mask = maskBit(r.centre.kind, r.centre.index);
    int evenKind[EVEN_RAYS], evenIndex[EVEN_RAYS];
    for (int k = 0; k < n; k++) {
        const Hit h = castRay(s, sx, sy, dirs[k], true);
        r.mask |= maskBit(h.kind, h.index);
        if (h.t < r.nearest.t - SAME_RANGE_IN) r.nearest = h;
        if (k < EVEN_RAYS) {
            evenKind[k] = h.kind;
            evenIndex[k] = h.index;
        }
    }
    for (int k = 0; k < EVEN_RAYS; k++) {
        if (evenKind[k] == r.centre.kind && evenIndex[k] == r.centre.index) r.fillCentre++;
        if (evenKind[k] == r.nearest.kind && evenIndex[k] == r.nearest.index) r.fillNearest++;
    }
    return r;
}

double DistModel::halfAngleFor(double centreIn) const {
    return centreIn * MM_PER_IN < p.cone_close_mm ? p.cone_half_close_deg : p.cone_half_deg;
}

double DistModel::blendRange(const ConeResult& c) const {
    const int model = int(std::lround(p.cone_model));
    if (model == 0) return c.centre.t;
    if (model == 1) return c.nearest.t;
    const double w = std::clamp(p.cone_blend, 0.0, 1.0);
    return c.nearest.t + w * (c.centre.t - c.nearest.t);
}

void DistModel::makeReading(int i, std::uint32_t nowMs) {
    SensorState& s = st[i];
    const DistSensorParams& sp = p.sensor[i];
    Rng& g = rng[i];

    // Every random number this reading might need, always the same count, whatever the options (dist_model.hpp)
    const double nWhite = g.normal(); // noise new every reading
    const double nSlow = g.normal(); // the slowly varying noise's next step
    const double nConf = g.normal(); // confidence jitter
    const double nSize = g.normal(); // object size jitter
    const double uDropout = g.uniform(); // random 9999
    const double uThrough = g.uniform(); // clear wall: through or not
    const double uBehind = g.uniform(); // clear wall: how far behind
    const double uStopDist = g.uniform(); // the stop object's distance (used at the first reading of a stop)

    // Slowly varying noise, in standard deviations: first order, already settled at the first reading. It is
    // stepped whether or not it is switched on, so switching it on changes nothing else
    if (s.outputs == 0) {
        s.slow = nSlow;
    } else {
        const double dtS = (nowMs - s.lastOutputMs) / 1000.0;
        const double a = p.slow_tau_s > 0 ? std::exp(-dtS / p.slow_tau_s) : 0.0;
        s.slow = a * s.slow + std::sqrt(std::max(0.0, 1 - a * a)) * nSlow;
    }

    DistOutputInfo inf;
    inf.timeMs = nowMs;
    inf.count = ++s.outputs;
    s.lastOutputMs = nowMs;

    if (sp.enabled == 0) {
        s.reading = {DIST_PROS_ERR, DIST_PROS_ERR, DIST_PROS_ERR};
        inf.state = DIST_OFF;
        s.stop = StopObject {};
        s.info = inf;
        return;
    }

    // where the robot was when this reading was taken, and where the sensor and its beam were
    const PoseSample ps = poseAt(double(nowMs) - std::clamp(p.latency_ms, 0.0, MAX_LATENCY_MS));
    double sx = 0, sy = 0, phi = 0;
    sensorPose(i, ps.x, ps.y, ps.h, sx, sy, phi);
    inf.sensorX = sx;
    inf.sensorY = sy;
    inf.beamDeg = wrap360(phi * RAD_TO_DEG);

    // The stop object (test W4): decided at the first reading of each stop, fixed in the field, gone once the
    // robot moves again
    // (stop_obj_from_ms: added when the model went into the bench, so that a test can leave the first stop clean
    // and let the filter's start check pass before the objects come; it draws no random number)
    const bool stopWanted = p.stop_obj != 0 && ((std::lround(p.stop_obj_sensors) >> i) & 1) != 0 && stopped &&
                            double(nowMs) >= p.stop_obj_from_ms;
    if (!stopWanted) {
        s.stop = StopObject {};
    } else if (!s.stop.active) {
        s.stop.active = true;
        s.stop.placed = false;
        s.stop.distIn = p.stop_obj_min_in + (p.stop_obj_max_in - p.stop_obj_min_in) * uStopDist;
        s.stop.r = p.stop_obj_r_in;
        const Scene bare = scene(nowMs, -1);
        if (insideField(bare, sx, sy) && s.stop.r > 0) {
            const double wallT = castRay(bare, sx, sy, phi, false).t; // walls and elements only
            const bool frontOfWall = std::lround(p.stop_obj) == 2;
            if (!frontOfWall) {
                // in front of the sensor: lower the top of the draw so the object stays stop_obj_gap_in in front of
                // the wall (the same random number, so nothing else changes)
                const double hi = std::min(p.stop_obj_max_in, wallT - p.stop_obj_gap_in);
                if (hi >= p.stop_obj_min_in) s.stop.distIn = p.stop_obj_min_in + (hi - p.stop_obj_min_in) * uStopDist;
                else s.stop.distIn = -1; // it does not fit: no object at this stop
            }
            const double nearSide = frontOfWall ? wallT - s.stop.distIn : s.stop.distIn;
            if (s.stop.distIn >= 0 && nearSide > 0 && nearSide < wallT) { // between the sensor and the wall
                s.stop.x = sx + (nearSide + s.stop.r) * std::sin(phi);
                s.stop.y = sy + (nearSide + s.stop.r) * std::cos(phi);
                s.stop.placed = true;
            }
        }
    }
    inf.stopObject = s.stop.active && s.stop.placed;
    inf.stopObjectDistIn = s.stop.active ? s.stop.distIn : 0;

    // ---- geometry: what the cone sees
    const Scene sc = scene(nowMs, i);
    bool tooClose = !insideField(sc, sx, sy); // the sensor's face is on or past a wall
    ConeResult c;
    double range = 0;
    Hit source; // the target the reading comes from
    if (!tooClose) {
        const double half = halfAngleFor(castRay(sc, sx, sy, phi, true).t);
        c = cone(sc, sx, sy, phi, half * DEG_TO_RAD);
        inf.coneHalfDeg = half;
        range = blendRange(c);
        // with the centre ray only the centre counts; with the other two the nearest point pulls the reading
        source = std::lround(p.cone_model) == 0 ? c.centre : c.nearest;
        tooClose = !(c.centre.t > 0) || !(c.nearest.t > 0); // its face is inside an element or a circle
        inf.centreKind = c.centre.kind;
        inf.centreIndex = c.centre.index;
        inf.nearKind = c.nearest.kind;
        inf.nearIndex = c.nearest.index;
        inf.centreIn = c.centre.t;
        inf.nearestIn = c.nearest.t;
        inf.wallIn = castRay(sc, sx, sy, phi, false).t;
        inf.coneMask = c.mask;
        const double dx = std::sin(phi), dy = std::cos(phi);
        inf.incidenceDeg = std::acos(std::min(1.0, std::fabs(dx * c.centre.nx + dy * c.centre.ny))) * RAD_TO_DEG;
        const int fill = std::lround(p.cone_model) == 0 ? c.fillCentre : c.fillNearest;
        inf.fill = double(fill) / EVEN_RAYS;
    }

    // ---- clear walls (test W5): through, or the wall with lower confidence, or nothing beyond a range
    double confDrop = 0;
    bool clearDropout = false;
    const bool clearWall = !tooClose && source.kind == DIST_HIT_WALL && p.wall[source.index].clear != 0;
    if (clearWall) {
        const DistWallParams& w = p.wall[source.index];
        confDrop = w.conf_drop;
        if (w.dropout_in > 0 && range > w.dropout_in) {
            clearDropout = true;
        } else if (uThrough < w.through_prob) {
            inf.through = true;
            range += w.behind_in + w.behind_spread_in * uBehind;
        }
    }
    inf.modelIn = range;

    // ---- the sensor's own errors: scale, offset, noise that grows with range
    const double trueMm = range * MM_PER_IN;
    const double measured = trueMm * (1 + sp.scale_err) + sp.offset_mm + (p.noise_mm + p.noise_frac * trueMm) * nWhite +
                            (p.slow_mm + p.slow_frac * trueMm) * s.slow;
    const long mm = std::lround(measured);

    // ---- what it reports
    DistNoReturn nr = DIST_NR_NONE;
    if (uDropout < p.dropout_prob) nr = DIST_NR_RANDOM;
    else if (tooClose) nr = DIST_NR_NONE;
    else if (clearDropout) nr = DIST_NR_CLEAR_DROPOUT;
    else if (p.dropout_incidence_deg > 0 && inf.incidenceDeg > p.dropout_incidence_deg) nr = DIST_NR_TOO_SLANTED;
    else if (!std::isfinite(measured) || double(mm) > p.max_mm) nr = DIST_NR_OUT_OF_RANGE;
    if (nr == DIST_NR_NONE && !tooClose && double(mm) < p.min_mm) tooClose = true;

    if (nr != DIST_NR_NONE) {
        s.reading = {DIST_NOTHING_MM, 0, -1};
        inf.state = DIST_NO_RETURN;
        inf.noReturn = nr;
    } else if (tooClose) {
        s.reading = {std::int32_t(std::lround(p.below_min_value)), std::int32_t(std::lround(p.conf_below_200)), -1};
        inf.state = DIST_TOO_CLOSE;
    } else {
        double conf = 0;
        if (double(mm) <= CONF_MEANINGFUL_ABOVE_MM) conf = p.conf_below_200;
        else if (p.conf_model == 0) conf = CONF_TOP - confDrop;
        else
            conf = CONF_TOP - p.conf_per_m * (double(mm) / 1000.0) - p.conf_per_deg * inf.incidenceDeg - confDrop +
                   p.conf_sd * nConf;
        const double size = std::clamp(p.size_full * inf.fill + p.size_sd * nSize, 0.0, SIZE_TOP);
        s.reading = {std::int32_t(mm), std::int32_t(std::lround(std::clamp(conf, 0.0, CONF_TOP))),
                     std::int32_t(std::lround(size))};
        if (inf.through) inf.state = DIST_THROUGH;
        else if (source.kind == DIST_HIT_STOP_OBJECT) inf.state = DIST_STOP_OBJECT;
        else if (source.kind == DIST_HIT_OBJECT) inf.state = DIST_OBJECT;
        else if (source.kind == DIST_HIT_ELEMENT) inf.state = DIST_ELEMENT;
        else if (!(c.centre.kind == DIST_HIT_WALL && c.centre.index == source.index)) inf.state = DIST_CORNER;
        else if (clearWall) inf.state = DIST_CLEAR_WALL;
        else inf.state = DIST_WALL;
    }
    s.info = inf;
}

int DistModel::coneMaskAt(std::uint32_t nowMs, double xIn, double yIn, double headingRad, int sensor,
                          double halfDeg) const {
    if (sensor < 0 || sensor >= DIST_MAX_SENSORS) return 0;
    const Scene sc = scene(nowMs, sensor);
    double sx = 0, sy = 0, phi = 0;
    sensorPose(sensor, xIn, yIn, headingRad, sx, sy, phi);
    if (!insideField(sc, sx, sy)) return 0;
    return cone(sc, sx, sy, phi, halfDeg * DEG_TO_RAD).mask;
}

double DistModel::modelRangeAt(std::uint32_t nowMs, double xIn, double yIn, double headingRad, int sensor) const {
    if (sensor < 0 || sensor >= DIST_MAX_SENSORS) return 0;
    const Scene sc = scene(nowMs, sensor);
    double sx = 0, sy = 0, phi = 0;
    sensorPose(sensor, xIn, yIn, headingRad, sx, sy, phi);
    if (!insideField(sc, sx, sy)) return 0;
    const double half = halfAngleFor(castRay(sc, sx, sy, phi, true).t);
    return blendRange(cone(sc, sx, sy, phi, half * DEG_TO_RAD));
}

} // namespace sim
