#include "pidtuner.hpp"

#include <cstdio>

#include "liblvgl/lvgl.h"
#include "liblvgl/widgets/lv_textarea.h"
#include "liblvgl/widgets/lv_label.h"
#include "pros/misc.hpp"

#include "lemlib/pid.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "robot-config.hpp"
#include "autotune.hpp"

namespace pidtuner {

namespace {

lv_obj_t* pid_textarea = nullptr;
bool active = false;

/** @brief Fields the D-pad can land on, in cycle order */
enum Field {
    LAT_P = 0,
    LAT_I,
    LAT_D,
    ANG_P,
    ANG_I,
    ANG_D,
    STEP,
    AUTO,
    FIELD_COUNT
};

int selected = LAT_P;

/** @brief Increments LEFT/RIGHT apply, coarse to fine */
constexpr float STEPS[] = {0.001f, 0.01f, 0.1f, 1.0f, 5.0f};
constexpr int STEP_COUNT = sizeof(STEPS) / sizeof(STEPS[0]);

int stepIndex = 2; // 0.1

/**
 * @brief Format one gain into a fixed 9 character cell
 *
 * The width is constant whether or not the field is selected, so the columns
 * stay lined up as the marker moves. Montserrat's digits are tabular, so equal
 * character counts render at equal widths.
 */
void writeCell(char* dest, size_t size, float value, int field, int decimals) {
    const char open = (field == selected) ? '>' : ' ';
    const char close = (field == selected) ? '<' : ' ';
    snprintf(dest, size, "%c%7.*f%c", open, decimals, value, close);
}

/** @brief Gains cannot go negative, that only ever means a runaway */
float clampGain(float value) { return value < 0.0f ? 0.0f : value; }

/**
 * @brief Apply one step to the selected field
 *
 * @param dir +1 for RIGHT, -1 for LEFT
 */
void adjust(int dir) {
    if (selected == AUTO) {
        // RIGHT launches the relay auto-tuner. LEFT is deliberately inert so a
        // stray press cannot start the robot moving
        if (dir > 0) autotune::start();
        return;
    }
    if (selected == STEP) {
        stepIndex += dir;
        if (stepIndex < 0) stepIndex = 0;
        if (stepIndex >= STEP_COUNT) stepIndex = STEP_COUNT - 1;
        return;
    }

    const float delta = STEPS[stepIndex] * dir;
    const bool lateral = (selected <= LAT_D);

    // the live PID owned by the chassis, not the ControllerSettings struct that
    // was copied into it at construction. writing here takes effect immediately
    lemlib::PID& pid = lateral ? chassis.lateralPID : chassis.angularPID;

    float kP = pid.getKP();
    float kI = pid.getKI();
    float kD = pid.getKD();

    switch (selected) {
        case LAT_P:
        case ANG_P: kP = clampGain(kP + delta); break;
        case LAT_I:
        case ANG_I: kI = clampGain(kI + delta); break;
        case LAT_D:
        case ANG_D: kD = clampGain(kD + delta); break;
        default: return;
    }

    pid.setGains(kP, kI, kD);

    // mirror into the settings struct so anything else reading it reports what
    // the robot is actually running
    lemlib::ControllerSettings& settings = lateral ? linearController : angularController;
    settings.kP = kP;
    settings.kI = kI;
    settings.kD = kD;
}

} // namespace

void init(lv_obj_t* parent, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h) {
    pid_textarea = lv_textarea_create(parent);

    lv_obj_set_size(pid_textarea, w, h);
    lv_obj_align(pid_textarea, LV_ALIGN_TOP_LEFT, x, y);

    // font 12 and tight padding keep five lines inside the box. the Developer
    // tab only has ~110px left below the odom readout, so this must not grow
    lv_obj_set_style_text_font(pid_textarea, &lv_font_montserrat_12, 0);
    lv_obj_set_style_pad_all(pid_textarea, 2, 0);

    lv_textarea_set_cursor_click_pos(pid_textarea, false);
    lv_textarea_set_password_mode(pid_textarea, false);
    lv_obj_add_state(pid_textarea, LV_STATE_DISABLED);

    // hard overflow guards. by default the textarea's label wraps, so one gain
    // growing an extra digit would spill onto a sixth line and push the content
    // out of the box. clipping keeps the line count fixed at five no matter what
    // the numbers do, and killing scroll means nothing can slide out of view
    lv_label_set_long_mode(lv_textarea_get_label(pid_textarea), LV_LABEL_LONG_CLIP);
    lv_obj_clear_flag(pid_textarea, LV_OBJ_FLAG_SCROLLABLE);

    render();
}

void setActive(bool state) {
    if (active == state) return;
    active = state;
    // drop any half-held button so re-entering the tab does not jump a field
    if (active) {
        master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP);
        master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT);
        master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT);
    }
}

bool isActive() { return active; }

void handleInput() {
    if (!active) return;

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        selected = (selected + 1) % FIELD_COUNT;
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) { adjust(+1); }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) { adjust(-1); }
}

void render() {
    if (pid_textarea == nullptr) return;

    char latP[12], latI[12], latD[12];
    char angP[12], angI[12], angD[12];

    writeCell(latP, sizeof(latP), chassis.lateralPID.getKP(), LAT_P, 2);
    writeCell(latI, sizeof(latI), chassis.lateralPID.getKI(), LAT_I, 3);
    writeCell(latD, sizeof(latD), chassis.lateralPID.getKD(), LAT_D, 2);

    writeCell(angP, sizeof(angP), chassis.angularPID.getKP(), ANG_P, 2);
    writeCell(angI, sizeof(angI), chassis.angularPID.getKI(), ANG_I, 3);
    writeCell(angD, sizeof(angD), chassis.angularPID.getKD(), ANG_D, 2);

    const char stepOpen = (selected == STEP) ? '>' : ' ';
    const char stepClose = (selected == STEP) ? '<' : ' ';

    char buffer[256];
    snprintf(buffer, sizeof(buffer),
             "         P        I        D\n"
             "LAT%s%s%s\n"
             "ANG%s%s%s\n"
             "STEP%c%6.3f%c  %s\n"
             "%cAUTO-TUNE%c  UP=next  L/R=adj",
             latP, latI, latD,
             angP, angI, angD,
             stepOpen, STEPS[stepIndex], stepClose,
             active ? "EDIT" : "----",
             (selected == AUTO) ? '>' : ' ', (selected == AUTO) ? '<' : ' ');

    lv_textarea_set_text(pid_textarea, buffer);
}

} // namespace pidtuner
