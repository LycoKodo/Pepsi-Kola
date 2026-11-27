// Picking the routine on the controller (added 2026-11-24).
//
// The pick is the brain selector's own `auton` (lemlib::selector), so the brain and the controller always agree
// and either can change it. This file only steps through the selector's routine list and shows the pick.
//   Driver control (no field control needed): hold A, press LEFT / RIGHT. Hold A, press B runs it, as before.
//   Disabled by field control or a competition switch: LEFT / RIGHT alone (nothing else uses them then).
// The controller's screen is 3 lines of 18 characters and takes one line per 50 ms, so the lines are queued and
// pump() sends one at a time: the driver loop never waits for the screen.
//
//   AUTON 1/4    QUAL        which one of how many, and its tab
//   >james_67                its button's name
//   A+<> pick  A+B run       (disabled: "<> pick")

#include <cstdint>
#include <cstdio>

#include "main.h"
#include "lemlib/selector.hpp"
#include "autons.hpp"
#include "robot-config.hpp"

namespace ctrlselect {
namespace {

constexpr int COLS = 18;                 // characters on a controller line
constexpr std::uint32_t LINE_GAP_MS = 60; // the controller drops lines sent under 50 ms apart
constexpr int DISABLED_TICK_MS = 20;

char queued[3][COLS + 1];
bool waiting[3] = {false, false, false};
std::uint32_t lastSentAt = 0;
bool disabledHint = false; // the bottom line says "<> pick" instead of the driver-control hint

void queueLine(int row, const char* text) {
    std::snprintf(queued[row], sizeof(queued[row]), "%-*.*s", COLS, COLS, text);
    waiting[row] = true;
}

} // namespace

void show() {
    if (!pros::competition::is_disabled()) disabledHint = false;
    namespace sel = lemlib::selector;
    const int count = sel::routineCount();
    const int index = sel::routineIndexOf(sel::auton);
    char text[40];
    if (index < 0) {
        std::snprintf(text, sizeof(text), "AUTON 0/%d", count);
        queueLine(0, text);
        queueLine(1, ">NONE");
    } else {
        std::snprintf(text, sizeof(text), "AUTON %d/%d %6s", index + 1, count, sel::routineTabAt(index));
        queueLine(0, text);
        std::snprintf(text, sizeof(text), ">%s", sel::routineNameAt(index));
        queueLine(1, text);
    }
    queueLine(2, disabledHint ? "<> pick" : "A+<> pick  A+B run");
}

void pump() {
    if (pros::millis() - lastSentAt < LINE_GAP_MS) return;
    for (int row = 0; row < 3; row++) {
        if (!waiting[row]) continue;
        waiting[row] = false;
        master.set_text(static_cast<std::uint8_t>(row), 0, queued[row]);
        lastSentAt = pros::millis();
        return;
    }
}

void step(int direction) {
    namespace sel = lemlib::selector;
    // positions: -1 (none), 0 .. count-1, wrapping
    const int count = sel::routineCount();
    int index = sel::routineIndexOf(sel::auton) + (direction >= 0 ? 1 : -1);
    if (index >= count) index = -1;
    else if (index < -1) index = count - 1;
    sel::selectRoutine(index);
    std::printf("[select] controller picked routine %d (%s)\n", sel::auton, index < 0 ? "none" : sel::routineNameAt(index));
    master.rumble(".");
    show();
}

void showNow() {
    show();
    for (int row = 0; row < 3; row++) {
        pros::delay(LINE_GAP_MS);
        pump();
    }
}

void disabledLoop() {
    disabledHint = true;
    show();
    bool leftBefore = master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
    bool rightBefore = master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    while (true) {
        const bool left = master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
        const bool right = master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
        if (right && !rightBefore) step(1);
        else if (left && !leftBefore) step(-1);
        leftBefore = left;
        rightBefore = right;
        pump();
        pros::delay(DISABLED_TICK_MS);
    }
}

} // namespace ctrlselect
