#include "mclview.hpp"

#include <cmath>
#include <cstdio>

#include "liblvgl/widgets/lv_canvas.h"
#include "liblvgl/widgets/lv_label.h"
#include "pros/rtos.hpp"

#include "mcl.hpp"

namespace mclview {

namespace {

// The tab page is 480x190 once the tab bar is taken off the 480x240 screen.
// A square field view on the left leaves ~290px for the numbers
constexpr lv_coord_t CANVAS_W = 176;
constexpr lv_coord_t CANVAS_H = 176;

/** @brief Redraw period. The filter itself only ticks every 25ms */
constexpr std::uint32_t RENDER_MS = 50;

// LV_COLOR_DEPTH is 32 in this build, so the canvas buffer is one lv_color_t
// per pixel and can be written directly -- lv_canvas_set_px_color re-derives
// the format on every call, which adds up over ~300 particles a frame
lv_color_t canvasBuf[CANVAS_W * CANVAS_H];

lv_obj_t* canvas = nullptr;
lv_obj_t* readout = nullptr;
std::uint32_t lastRender = 0;

mcl::Snapshot snap;

const lv_color_t COL_BG = LV_COLOR_MAKE(0x10, 0x14, 0x1c);
const lv_color_t COL_WALL = LV_COLOR_MAKE(0x78, 0x82, 0x96);
const lv_color_t COL_GRID = LV_COLOR_MAKE(0x28, 0x2f, 0x3c);
const lv_color_t COL_PARTICLE = LV_COLOR_MAKE(0x3d, 0x8b, 0xff);
const lv_color_t COL_ODOM = LV_COLOR_MAKE(0xff, 0xa5, 0x2b);
const lv_color_t COL_EST = LV_COLOR_MAKE(0x3f, 0xd6, 0x7a);
const lv_color_t COL_RAY = LV_COLOR_MAKE(0x8a, 0x5c, 0xc8);

constexpr float DEG_TO_RAD = 0.017453292519943295f;

/** @brief Inches across the field per canvas pixel */
constexpr float SCALE = (CANVAS_W - 1) / (2.0f * mcl::FIELD_HALF);

inline void px(int x, int y, lv_color_t c) {
    if (x < 0 || y < 0 || x >= CANVAS_W || y >= CANVAS_H) return;
    canvasBuf[y * CANVAS_W + x] = c;
}

/** @brief Field inches to canvas pixels. +Y is up the screen, so y flips */
inline void toPx(float fx, float fy, int& outX, int& outY) {
    outX = static_cast<int>((fx + mcl::FIELD_HALF) * SCALE + 0.5f);
    outY = static_cast<int>((CANVAS_H - 1) - (fy + mcl::FIELD_HALF) * SCALE + 0.5f);
}

void line(int x0, int y0, int x1, int y1, lv_color_t c) {
    const int dx = std::abs(x1 - x0);
    const int dy = -std::abs(y1 - y0);
    const int sx = x0 < x1 ? 1 : -1;
    const int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        px(x0, y0, c);
        if (x0 == x1 && y0 == y1) break;
        const int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/** @brief Filled square centred on a point, side 2r+1 */
void blob(int cx, int cy, int r, lv_color_t c) {
    for (int y = cy - r; y <= cy + r; ++y)
        for (int x = cx - r; x <= cx + r; ++x) px(x, y, c);
}

/** @brief A pose marker: a blob with a stalk pointing where it faces */
void marker(float fx, float fy, float theta, lv_color_t c) {
    int cx, cy;
    toPx(fx, fy, cx, cy);
    const float rad = theta * DEG_TO_RAD;
    // compass heading: 0 is +Y (up the screen), increasing clockwise
    const int tx = cx + static_cast<int>(std::sin(rad) * 9.0f);
    const int ty = cy - static_cast<int>(std::cos(rad) * 9.0f);
    line(cx, cy, tx, ty, c);
    blob(cx, cy, 2, c);
}

void drawField() {
    for (int i = 0; i < CANVAS_W * CANVAS_H; ++i) canvasBuf[i] = COL_BG;

    // quarter lines, so a pose can be eyeballed against the field tiles
    for (int q = 1; q < 4; ++q) {
        const int p = (CANVAS_W - 1) * q / 4;
        line(p, 0, p, CANVAS_H - 1, COL_GRID);
        line(0, p, CANVAS_W - 1, p, COL_GRID);
    }

    line(0, 0, CANVAS_W - 1, 0, COL_WALL);
    line(0, CANVAS_H - 1, CANVAS_W - 1, CANVAS_H - 1, COL_WALL);
    line(0, 0, 0, CANVAS_H - 1, COL_WALL);
    line(CANVAS_W - 1, 0, CANVAS_W - 1, CANVAS_H - 1, COL_WALL);
}

/**
 * @brief Draw each sensor's actual reading as a spoke off the estimate
 *
 * The spoke is drawn in the estimate's frame using the mounting angles the
 * filter uses, so a spoke that stops short of a wall is the sensor and the
 * filter disagreeing -- which is exactly what the weighting step is reacting to
 */
void drawRays() {
    if (!snap.estimate.valid) return;
    const float poseRad = snap.estimate.theta * DEG_TO_RAD;
    const float sn = std::sin(poseRad);
    const float cs = std::cos(poseRad);

    for (int s = 0; s < mcl::SENSOR_COUNT; ++s) {
        if (snap.measured[s] < 0.0f) continue; // reading was rejected
        const mcl::SensorMount& m = snap.mounts[s];
        // same robot-to-field transform the filter uses: forward is (sin, cos),
        // right is (cos, -sin)
        int cx, cy;
        toPx(snap.estimate.x + m.y * sn + m.x * cs,
             snap.estimate.y + m.y * cs - m.x * sn, cx, cy);
        const float rad = (snap.estimate.theta + m.theta) * DEG_TO_RAD;
        const float len = snap.measured[s] * SCALE;
        line(cx, cy, cx + static_cast<int>(std::sin(rad) * len),
             cy - static_cast<int>(std::cos(rad) * len), COL_RAY);
    }
}

void drawText() {
    char buf[512];
    char dist[4][16];
    for (int s = 0; s < mcl::SENSOR_COUNT; ++s) {
        if (snap.measured[s] < 0.0f) snprintf(dist[s], sizeof(dist[s]), "  --");
        else snprintf(dist[s], sizeof(dist[s]), "%4.0f", snap.measured[s]);
    }

    snprintf(buf, sizeof(buf),
             "MCL  %s  %s  correct %s\n"
             "\n"
             "est   x %7.2f\n"
             "      y %7.2f\n"
             "      h %7.2f\n"
             "\n"
             "odom  x %7.2f\n"
             "      y %7.2f\n"
             "      h %7.2f\n"
             "\n"
             "spread %5.1f in   conf %3.0f%%\n"
             "\n"
             "dist  F%s R%s B%s L%s\n"
             "want  F%4.0f R%4.0f B%4.0f L%4.0f\n"
             "\n"
             "%d particles   %lu.%02lu ms/tick",
             mcl::isRunning() ? "RUN " : "STOP",
             snap.locked ? "LOCK" : "----",
             mcl::isCorrectionEnabled() ? "ON" : "off",
             snap.estimate.x, snap.estimate.y, snap.estimate.theta,
             snap.odomX, snap.odomY, snap.odomTheta,
             snap.estimate.spread, snap.estimate.confidence * 100.0f,
             dist[0], dist[1], dist[2], dist[3],
             snap.expected[0], snap.expected[1], snap.expected[2], snap.expected[3],
             mcl::PARTICLE_COUNT,
             static_cast<unsigned long>(snap.usPerUpdate / 1000),
             static_cast<unsigned long>((snap.usPerUpdate % 1000) / 10));

    lv_label_set_text(readout, buf);
}

} // namespace

void init(lv_obj_t* parent) {
    canvas = lv_canvas_create(parent);
    lv_canvas_set_buffer(canvas, canvasBuf, CANVAS_W, CANVAS_H, LV_IMG_CF_TRUE_COLOR);
    lv_obj_align(canvas, LV_ALIGN_TOP_LEFT, -10, -10);

    readout = lv_label_create(parent);
    lv_obj_set_style_text_font(readout, &lv_font_montserrat_10, 0);
    lv_obj_set_width(readout, 270);
    lv_obj_align(readout, LV_ALIGN_TOP_LEFT, 180, -10);
    lv_label_set_text(readout, "MCL starting...");

    drawField();
    lv_obj_invalidate(canvas);
}

void render() {
    if (canvas == nullptr) return;
    const std::uint32_t now = pros::millis();
    if (now - lastRender < RENDER_MS) return;
    lastRender = now;

    mcl::snapshot(snap);

    drawField();
    for (int i = 0; i < mcl::PARTICLE_COUNT; ++i) {
        int x, y;
        toPx(snap.particles[i].x, snap.particles[i].y, x, y);
        px(x, y, COL_PARTICLE);
    }
    drawRays();
    marker(snap.odomX, snap.odomY, snap.odomTheta, COL_ODOM);
    if (snap.estimate.valid) marker(snap.estimate.x, snap.estimate.y, snap.estimate.theta, COL_EST);

    lv_obj_invalidate(canvas);
    drawText();
}

} // namespace mclview
