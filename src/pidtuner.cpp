#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "main.h"
#include "robot-config.hpp"
#include "pidtuner.hpp"

namespace pidtuner {

constexpr int DRIVE_TIMEOUT_MS = 4000;
constexpr int TURN_TIMEOUT_MS = 2500;

constexpr int SAMPLE_MS = 10;
constexpr int COAST_MS = 300; // keep sampling after the motion ends to catch coasting
constexpr int MAX_SAMPLES = 512; // 5.1 s, longer than the longest timeout + coast
constexpr int STICK_LIMIT = 15;
constexpr float DRIVE_TOLERANCE = 1.0; // inches
constexpr float TURN_TOLERANCE = 1.5; // degrees
constexpr int CHART_POINTS = 100;
constexpr int CHART_SCALE = 10; // chart values are int16, so plot error * 10
// Driver control runs the drive in COAST, which would let a test move roll on past the target
// where an auton would not. Tests run in BRAKE (what the measured tuning routes used when these
// gains were tuned) and put the driver's mode back afterwards
constexpr pros::motor_brake_mode_e_t TEST_BRAKE_MODE = pros::E_MOTOR_BRAKE_BRAKE;

enum Mode { DRIVE = 0, TURN = 1 };

const float STEPS[] = {0.0001, 0.001, 0.01, 0.1, 1, 10}; // 0.0001 is there for kI
constexpr int STEP_COUNT = sizeof(STEPS) / sizeof(STEPS[0]);

// target settings per mode: default, step, min, max
const int TARGET_DEFAULT[] = {24, 90};
const int TARGET_STEP[] = {6, 15};
const int TARGET_MIN[] = {6, 15};
const int TARGET_MAX[] = {72, 165}; // not 180: LemLib can't tell which way to turn for exactly 180

// ---------------------------------------
// Shared between the test task and LVGL.
// The test task never touches LVGL; the refresh timer reads these.
// ---------------------------------------
struct TestParams {
    Mode mode;
    float target; // signed: inches for DRIVE, degrees for TURN
};

struct Result {
    Mode mode;
    float target;
    float finalError;
    float overshoot;
    int settleMs; // -1 = never settled
    const char* abortReason; // nullptr if the run finished normally
};

std::atomic<bool> running = false;
std::atomic<bool> stopRequested = false;
std::atomic<int> resultCount = 0; // bumped by the test task each time a result is ready
TestParams params;
Result result;
float samples[MAX_SAMPLES];
int sampleCount = 0;

// ---------------------------------------
// UI state (LVGL context only)
// ---------------------------------------
Mode mode = DRIVE;
int stepIndex = 3; // 0.1
int target[] = {TARGET_DEFAULT[DRIVE], TARGET_DEFAULT[TURN]};
bool reverseNext[] = {false, false};

// The whole left side is one button matrix: the brain only gives LVGL a 32 KB heap and the
// selector already uses most of it, so separate buttons + labels for every row don't fit.
enum Cell {
    CELL_DRIVE, CELL_TURN, CELL_STEP,
    CELL_KP, CELL_KP_MINUS, CELL_KP_VALUE, CELL_KP_PLUS,
    CELL_KI, CELL_KI_MINUS, CELL_KI_VALUE, CELL_KI_PLUS,
    CELL_KD, CELL_KD_MINUS, CELL_KD_VALUE, CELL_KD_PLUS,
    CELL_TARGET, CELL_TARGET_MINUS, CELL_TARGET_VALUE, CELL_TARGET_PLUS,
    CELL_COUNT
};
char stepText[24];
char valueText[4][12]; // kP, kI, kD, target
const char* cellMap[] = {
    "DRIVE", "TURN", stepText, "\n",
    "kP", LV_SYMBOL_MINUS, valueText[0], LV_SYMBOL_PLUS, "\n",
    "kI", LV_SYMBOL_MINUS, valueText[1], LV_SYMBOL_PLUS, "\n",
    "kD", LV_SYMBOL_MINUS, valueText[2], LV_SYMBOL_PLUS, "\n",
    "Target", LV_SYMBOL_MINUS, valueText[3], LV_SYMBOL_PLUS, ""
};

lv_obj_t* matrix;
lv_obj_t* runBtn;
lv_obj_t* runLabel;
lv_obj_t* runSubLabel;
lv_obj_t* statusLabel;
lv_obj_t* metricLabels[3];
lv_obj_t* chart;
lv_coord_t errPoints[CHART_POINTS];
lv_coord_t zeroPoints[CHART_POINTS]; // all 0: draws the zero line

lv_style_t styleRun, styleStop, styleFont12, styleFont14, styleFont24;

bool shownRunning = false;
int shownResults = 0;

bool isRunning() { return running; }

bool sticksMoved() {
    return std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) > STICK_LIMIT ||
           std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > STICK_LIMIT ||
           std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > STICK_LIMIT ||
           std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) > STICK_LIMIT;
}

lemlib::PID& pidFor(Mode m) { return m == DRIVE ? chassis.lateralPID : chassis.angularPID; }

lemlib::ControllerSettings& settingsFor(Mode m) { return m == DRIVE ? linearController : angularController; }

float signOf(float x) { return x < 0 ? -1 : 1; }

// ---------------------------------------
// Test run (own task, no LVGL calls in here)
// ---------------------------------------
void testTask() {
    const TestParams p = params;

    // LemLib has no brake mode getter: read it off the drive motors (setBrakeMode sets both sides)
    const pros::motor_brake_mode_e_t savedBrakeMode = leftMotors.get_brake_mode();
    chassis.setBrakeMode(TEST_BRAKE_MODE);

    const lemlib::Pose start = chassis.getPose();
    const float theta0 = start.theta * M_PI / 180;
    const float dir = signOf(p.target);
    const float dist = std::abs(p.target);
    const float targetHeading = start.theta + p.target;

    if (p.mode == DRIVE) {
        float tx = start.x + p.target * std::sin(theta0);
        float ty = start.y + p.target * std::cos(theta0);
        chassis.moveToPoint(tx, ty, DRIVE_TIMEOUT_MS, {.forwards = p.target > 0}, true);
    } else {
        // chassis is a TeamChassis, so this is its turnToHeading (with the team's small/big turn
        // rules), the same one the autons get
        chassis.turnToHeading(targetHeading, TURN_TIMEOUT_MS, {}, true);
    }

    // positive error = short of the target, negative = past it
    const char* abortReason = nullptr;
    int n = 0;
    int endSample = -1;
    std::uint32_t now = pros::millis();
    while (n < MAX_SAMPLES) {
        const lemlib::Pose pose = chassis.getPose();
        float err;
        if (p.mode == DRIVE) {
            float progress = (pose.x - start.x) * std::sin(theta0) + (pose.y - start.y) * std::cos(theta0);
            err = dist - progress * dir;
        } else {
            err = (targetHeading - pose.theta) * dir;
        }
        samples[n++] = err;

        if (abortReason == nullptr && (stopRequested || sticksMoved())) {
            abortReason = stopRequested ? "stopped" : "stick moved";
            chassis.cancelAllMotions();
        }

        if (chassis.isInMotion()) {
            endSample = -1;
        } else if (endSample < 0) {
            endSample = n;
        } else if ((n - endSample) * SAMPLE_MS >= COAST_MS) {
            break;
        }
        pros::Task::delay_until(&now, SAMPLE_MS);
    }
    if (chassis.isInMotion()) chassis.cancelAllMotions(); // ran out of buffer
    // every way out of the loop above (finished, stopped, stick moved, out of buffer) comes through
    // here, with the motion over: give the drive back in the mode it had
    chassis.setBrakeMode(savedBrakeMode);

    const float tolerance = p.mode == DRIVE ? DRIVE_TOLERANCE : TURN_TOLERANCE;
    float minErr = 0;
    int lastOutside = -1;
    for (int i = 0; i < n; i++) {
        minErr = std::min(minErr, samples[i]);
        if (std::abs(samples[i]) > tolerance) lastOutside = i;
    }

    result.mode = p.mode;
    result.target = p.target;
    result.finalError = samples[n - 1];
    result.overshoot = -minErr;
    result.settleMs = lastOutside == n - 1 ? -1 : (lastOutside + 1) * SAMPLE_MS;
    result.abortReason = abortReason;
    sampleCount = n;

    const lemlib::PID& pid = pidFor(p.mode);
    char settle[16];
    if (result.settleMs < 0) snprintf(settle, sizeof(settle), "none");
    else snprintf(settle, sizeof(settle), "%dms", result.settleMs);
    printf("[PID] %s kP=%.3f kI=%.4f kD=%.3f target=%+.0f%s final=%.2f overshoot=%.2f settle=%s%s%s\n",
           p.mode == DRIVE ? "DRIVE" : "TURN", pid.getKP(), pid.getKI(), pid.getKD(), p.target,
           p.mode == DRIVE ? "in" : "deg", result.finalError, result.overshoot, settle,
           abortReason ? " ABORTED: " : "", abortReason ? abortReason : "");

    resultCount++;
    stopRequested = false;
    running = false;
}

// ---------------------------------------
// LVGL side
// ---------------------------------------

// LVGL's own printf has floats disabled, so everything is formatted with snprintf
void formatCells() {
    const lemlib::PID& pid = pidFor(mode);
    snprintf(valueText[0], sizeof(valueText[0]), "%.3f", pid.getKP());
    snprintf(valueText[1], sizeof(valueText[1]), "%.4f", pid.getKI());
    snprintf(valueText[2], sizeof(valueText[2]), "%.3f", pid.getKD());
    snprintf(valueText[3], sizeof(valueText[3]), mode == DRIVE ? "%d in" : "%d deg", target[mode]);
    snprintf(stepText, sizeof(stepText), "step %g", (double)STEPS[stepIndex]);
}

void showCells() {
    formatCells();
    lv_btnmatrix_set_btn_ctrl(matrix, mode == DRIVE ? CELL_DRIVE : CELL_TURN, LV_BTNMATRIX_CTRL_CHECKED);
    lv_btnmatrix_clear_btn_ctrl(matrix, mode == DRIVE ? CELL_TURN : CELL_DRIVE, LV_BTNMATRIX_CTRL_CHECKED);
    lv_obj_invalidate(matrix);
}

void showRunButton() {
    if (running) {
        lv_obj_add_state(runBtn, LV_STATE_CHECKED);
        lv_label_set_text(runLabel, "STOP");
        lv_label_set_text(runSubLabel, "or touch a stick");
        return;
    }
    lv_obj_clear_state(runBtn, LV_STATE_CHECKED);
    lv_label_set_text(runLabel, "RUN");
    const char* way;
    if (mode == DRIVE) way = reverseNext[mode] ? "back" : "forward";
    else way = reverseNext[mode] ? "left" : "right";
    lv_label_set_text_fmt(runSubLabel, "%s %d %s", way, target[mode], mode == DRIVE ? "in" : "deg");
}

void setMetric(int i, const char* name, const char* value) {
    char buf[32];
    snprintf(buf, sizeof(buf), "#9e9e9e %s#\n%s", name, value);
    lv_label_set_text(metricLabels[i], buf);
}

void showResult() {
    const char* unit = result.mode == DRIVE ? "in" : "deg";
    if (result.abortReason) lv_label_set_text_fmt(statusLabel, "aborted (%s)", result.abortReason);
    else lv_label_set_text_fmt(statusLabel, "%s %+d %s done", result.mode == DRIVE ? "drive" : "turn",
                               (int)result.target, unit);

    char value[16];
    snprintf(value, sizeof(value), "%.2f", result.finalError);
    setMetric(0, "end err", value);
    snprintf(value, sizeof(value), "%.2f", result.overshoot);
    setMetric(1, "over", value);
    if (result.settleMs < 0) snprintf(value, sizeof(value), "none");
    else snprintf(value, sizeof(value), "%.2fs", result.settleMs / 1000.0f);
    setMetric(2, "settle", value);

    // plot error vs time, squeezed into CHART_POINTS points
    float lo = 0, hi = 0;
    for (int i = 0; i < sampleCount; i++) {
        lo = std::min(lo, samples[i]);
        hi = std::max(hi, samples[i]);
    }
    int margin = std::max(2, (int)((hi - lo) * CHART_SCALE / 10));
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, (int)std::floor(lo * CHART_SCALE) - margin,
                       (int)std::ceil(hi * CHART_SCALE) + margin);
    for (int i = 0; i < CHART_POINTS; i++) {
        int s = sampleCount > 1 ? i * (sampleCount - 1) / (CHART_POINTS - 1) : 0;
        errPoints[i] = sampleCount > 0 ? (lv_coord_t)std::lround(samples[s] * CHART_SCALE) : 0;
    }
    lv_chart_refresh(chart);
}

// runs in LVGL context every 50 ms; the only place test results reach the screen
void refresh(lv_timer_t*) {
    bool isRunningNow = running;
    if (isRunningNow != shownRunning) {
        shownRunning = isRunningNow;
        if (isRunningNow) lv_obj_add_state(matrix, LV_STATE_DISABLED);
        else lv_obj_clear_state(matrix, LV_STATE_DISABLED);
        showRunButton();
        if (isRunningNow) lv_label_set_text(statusLabel, "running...");
    }
    int results = resultCount;
    if (results != shownResults) {
        shownResults = results;
        showResult();
    }
}

void changeGain(int gain, float delta) {
    lemlib::PID& pid = pidFor(mode);
    float g[3] = {pid.getKP(), pid.getKI(), pid.getKD()};
    // round to 4 decimals so repeated float adds don't drift
    g[gain] = std::max(0.0f, std::round((g[gain] + delta) * 10000) / 10000);
    pid.setGains(g[0], g[1], g[2]);

    lemlib::ControllerSettings& settings = settingsFor(mode); // so the Developer tab shows live values
    settings.kP = g[0];
    settings.kI = g[1];
    settings.kD = g[2];
}

// fires on press and then about every 0.1 s while held (btnmatrix default), so holding +/- repeats
void onMatrix(lv_event_t*) {
    int cell = lv_btnmatrix_get_selected_btn(matrix);
    float step = STEPS[stepIndex];
    switch (cell) {
        case CELL_DRIVE: mode = DRIVE; break;
        case CELL_TURN: mode = TURN; break;
        case CELL_STEP: stepIndex = (stepIndex + 1) % STEP_COUNT; break;
        case CELL_KP_MINUS: changeGain(0, -step); break;
        case CELL_KP_PLUS: changeGain(0, step); break;
        case CELL_KI_MINUS: changeGain(1, -step); break;
        case CELL_KI_PLUS: changeGain(1, step); break;
        case CELL_KD_MINUS: changeGain(2, -step); break;
        case CELL_KD_PLUS: changeGain(2, step); break;
        case CELL_TARGET_MINUS:
        case CELL_TARGET_PLUS:
            target[mode] += cell == CELL_TARGET_PLUS ? TARGET_STEP[mode] : -TARGET_STEP[mode];
            target[mode] = std::clamp(target[mode], TARGET_MIN[mode], TARGET_MAX[mode]);
            break;
        default: return; // name/value cells
    }
    showCells();
    showRunButton();
}

bool isTextCell(int cell) {
    return cell == CELL_KP || cell == CELL_KI || cell == CELL_KD || cell == CELL_TARGET || cell == CELL_KP_VALUE ||
           cell == CELL_KI_VALUE || cell == CELL_KD_VALUE || cell == CELL_TARGET_VALUE;
}

// name/value cells are drawn as plain text instead of buttons
void onMatrixDraw(lv_event_t* e) {
    lv_obj_draw_part_dsc_t* dsc = lv_event_get_draw_part_dsc(e);
    if (dsc->class_p != &lv_btnmatrix_class || dsc->type != LV_BTNMATRIX_DRAW_PART_BTN) return;
    bool locked = lv_obj_has_state(matrix, LV_STATE_DISABLED); // grey everything out while a test runs
    if (locked) dsc->label_dsc->opa = LV_OPA_40;
    if (isTextCell(dsc->id)) {
        dsc->rect_dsc->bg_opa = LV_OPA_TRANSP;
        dsc->rect_dsc->border_opa = LV_OPA_TRANSP;
        dsc->rect_dsc->shadow_opa = LV_OPA_TRANSP;
        dsc->rect_dsc->outline_opa = LV_OPA_TRANSP;
        dsc->label_dsc->color = lv_color_white();
        if (dsc->id == CELL_TARGET) dsc->label_dsc->font = &lv_font_montserrat_14;
    } else if (dsc->id == CELL_STEP && !locked) {
        dsc->rect_dsc->bg_color = lv_palette_darken(LV_PALETTE_BLUE_GREY, 2);
    }
}

void onRun(lv_event_t*) {
    if (running) {
        stopRequested = true;
        lv_label_set_text(statusLabel, "stopping...");
        return;
    }
    const char* refuse = nullptr;
    if (pros::competition::is_disabled()) refuse = "robot is disabled";
    else if (pros::competition::is_autonomous()) refuse = "auton is running";
    else if (sticksMoved()) refuse = "let go of the sticks";
    if (refuse) {
        lv_label_set_text(statusLabel, refuse);
        return;
    }

    params.mode = mode;
    params.target = reverseNext[mode] ? -target[mode] : target[mode];
    reverseNext[mode] = !reverseNext[mode]; // shuttle back to the start next time
    stopRequested = false;
    running = true;
    pros::Task tuneTask(testTask, "pidtuner");
    refresh(nullptr);
}

lv_obj_t* makeLabel(lv_obj_t* parent, lv_style_t* font, const char* text) {
    lv_obj_t* label = lv_label_create(parent);
    lv_obj_add_style(label, font, 0);
    lv_label_set_text(label, text);
    return label;
}

void init(lv_obj_t* tabview) {
    lv_obj_t* tab = lv_tabview_add_tab(tabview, "PID");
    lv_obj_set_style_pad_all(tab, 4, 0);
    lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);
    // usable area is now 472 x 182

    // shared styles instead of per-widget ones, again to save LVGL heap
    lv_style_init(&styleFont12);
    lv_style_set_text_font(&styleFont12, &lv_font_montserrat_12);
    lv_style_init(&styleFont14);
    lv_style_set_text_font(&styleFont14, &lv_font_montserrat_14);
    lv_style_set_text_align(&styleFont14, LV_TEXT_ALIGN_CENTER);
    lv_style_init(&styleFont24);
    lv_style_set_text_font(&styleFont24, &lv_font_montserrat_24);
    lv_style_init(&styleRun);
    lv_style_set_bg_color(&styleRun, lv_palette_darken(LV_PALETTE_GREEN, 1));
    lv_style_set_pad_all(&styleRun, 0);
    lv_style_init(&styleStop);
    lv_style_set_bg_color(&styleStop, lv_palette_main(LV_PALETTE_RED));

    // left column: mode/step, kP, kI, kD, target
    matrix = lv_btnmatrix_create(tab);
    formatCells(); // an empty string ends a btnmatrix map, so fill the texts first
    lv_btnmatrix_set_map(matrix, cellMap);
    lv_obj_set_pos(matrix, 0, 0);
    lv_obj_set_size(matrix, 258, 182);
    lv_obj_set_style_pad_all(matrix, 0, 0);
    lv_obj_set_style_pad_gap(matrix, 4, 0);
    lv_obj_set_style_border_width(matrix, 0, 0);
    lv_obj_set_style_bg_opa(matrix, LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_font(matrix, &lv_font_montserrat_18, LV_PART_ITEMS);
    // relative widths: name 4, [-] 4, value 5, [+] 4
    for (int row = 0; row < 4; row++) {
        int first = CELL_KP + row * 4;
        lv_btnmatrix_set_btn_width(matrix, first, 4);
        lv_btnmatrix_set_btn_width(matrix, first + 1, 4);
        lv_btnmatrix_set_btn_width(matrix, first + 2, 5);
        lv_btnmatrix_set_btn_width(matrix, first + 3, 4);
    }
    lv_btnmatrix_set_btn_width(matrix, CELL_DRIVE, 4);
    lv_btnmatrix_set_btn_width(matrix, CELL_TURN, 4);
    lv_btnmatrix_set_btn_width(matrix, CELL_STEP, 7); // widest: has to fit "step 0.0001"
    // only +/- should repeat while held
    for (int cell = CELL_DRIVE; cell <= CELL_STEP; cell++)
        lv_btnmatrix_set_btn_ctrl(matrix, cell, LV_BTNMATRIX_CTRL_NO_REPEAT);
    for (int cell = 0; cell < CELL_COUNT; cell++)
        if (isTextCell(cell)) lv_btnmatrix_set_btn_ctrl(matrix, cell, LV_BTNMATRIX_CTRL_NO_REPEAT);
    lv_obj_add_event_cb(matrix, onMatrix, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_add_event_cb(matrix, onMatrixDraw, LV_EVENT_DRAW_PART_BEGIN, nullptr);

    // right column: RUN/STOP, status, three numbers, error plot
    const int rx = 266, rw = 472 - rx;
    runBtn = lv_btn_create(tab);
    lv_obj_set_pos(runBtn, rx, 0);
    lv_obj_set_size(runBtn, rw, 52);
    lv_obj_add_style(runBtn, &styleRun, 0);
    lv_obj_add_style(runBtn, &styleStop, LV_STATE_CHECKED);
    lv_obj_add_event_cb(runBtn, onRun, LV_EVENT_CLICKED, nullptr);
    runLabel = makeLabel(runBtn, &styleFont24, "RUN");
    lv_obj_align(runLabel, LV_ALIGN_TOP_MID, 0, 4);
    runSubLabel = makeLabel(runBtn, &styleFont12, "");
    lv_obj_align(runSubLabel, LV_ALIGN_BOTTOM_MID, 0, -3);

    statusLabel = makeLabel(tab, &styleFont14, "press RUN to test");
    lv_obj_set_pos(statusLabel, rx, 56);
    lv_obj_set_width(statusLabel, rw);

    for (int i = 0; i < 3; i++) {
        metricLabels[i] = makeLabel(tab, &styleFont14, "");
        lv_label_set_recolor(metricLabels[i], true);
        lv_obj_set_pos(metricLabels[i], rx + i * rw / 3, 74);
        lv_obj_set_width(metricLabels[i], rw / 3);
    }
    setMetric(0, "end err", "-");
    setMetric(1, "over", "-");
    setMetric(2, "settle", "-");

    chart = lv_chart_create(tab);
    lv_obj_set_pos(chart, rx, 112);
    lv_obj_set_size(chart, rw, 182 - 112);
    lv_obj_set_style_pad_all(chart, 4, 0);
    lv_obj_set_style_size(chart, 0, LV_PART_INDICATOR); // no dots on the points
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_div_line_count(chart, 0, 0);
    lv_chart_set_point_count(chart, CHART_POINTS);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -10, 10);
    for (int i = 0; i < CHART_POINTS; i++) errPoints[i] = LV_CHART_POINT_NONE;
    // static arrays so the points don't come out of the LVGL heap
    lv_chart_series_t* zero = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_GREY), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_ext_y_array(chart, zero, zeroPoints);
    lv_chart_series_t* err = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_YELLOW), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_ext_y_array(chart, err, errPoints);

    showCells();
    showRunButton();
    lv_timer_create(refresh, 50, nullptr);

    // the brain only gives LVGL 32 KB; if this gets near 100% the screen can hang
    lv_mem_monitor_t mem;
    lv_mem_monitor(&mem);
    printf("[PID] LVGL heap %d%% used, biggest free block %d bytes\n", (int)mem.used_pct, (int)mem.free_biggest_size);
}

} // namespace pidtuner
