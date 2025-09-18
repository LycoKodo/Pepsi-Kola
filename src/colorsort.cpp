#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <sys/wait.h>

#include "controls.hpp"
#include "robot-config.hpp"
#include "colorsort.hpp"
#include "lemlib/selector.hpp" // For auton selector

namespace macros {

ColorSort::ColorSort() {}

void ColorSort::initialise(char alliance) {
    // CPP DEFAULT PARAM SYNTAX GUIDDE: https://en.cppreference.com/w/cpp/language/default_arguments
    preload_optical.set_led_pwm(100);
    color_sort_optical.set_led_pwm(100);
    if (alliance != 'N') {
        this->alliance = alliance;
    } else {
        alliance = identify_alliance();
    }
}

// RED CLRSRT -> 10 
// RED PRELOAD -> 13

// BLUE CLRSRT -> 213
// BLUE PRELOAD -> 208
char ColorSort::identify_alliance() {
    // TODO - IMPLEMENT TUNING
    const float blue_lim_low = 185, blue_lim_high = 240;
    const float red_lim_low = 0, red_lim_high = 30;
    double hue = preload_optical.get_hue();

    if (red_lim_low < hue && hue < red_lim_high) {
        return 'R';
    } else if (blue_lim_low < hue && hue < blue_lim_high) {
        return 'B';
    }
    return 'N';
}

bool ColorSort::ringInspect() {
    // TODO - Tune these values
    const float blue_lim_low = 185, blue_lim_high = 240;
    const float red_lim_low = 0, red_lim_high = 30;
    double hue = color_sort_optical.get_hue();

    // TODO - figure out alliancing (color_sort_identifier, + monitoring on dev page)
    if (alliance == 'B') {
        return !(red_lim_low < hue && hue < red_lim_high);
    } else if (alliance == 'R') {
        return !(blue_lim_low < hue && hue < blue_lim_high);
    }
    return true; // Default: passes for any other color
}

void ColorSort::Intake_SortedMove(int voltage, float msDelay, int penaltyFactor, bool async) {
    // Get the start time
    if (async) {
        pros::Task task([&]() { Intake_SortedMove(voltage, msDelay, penaltyFactor, false); });
        pros::delay(10); // delay to give the task time to start
        return;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    auto endTime = startTime + std::chrono::milliseconds(static_cast<int>(msDelay));

    // While the elapsed time is less than msDelay
    while (std::chrono::high_resolution_clock::now() < endTime) {
        if (voltage > 0) {
            // Intaking
            bool passed = ringInspect();
            if (passed) {
                intake.move(voltage);
            } else {
                // Wrong color, redirect intake
                intake_redirect.set_value(1);
            }
        } else {
            // Outtaking, no color sorting
            intake.move(voltage);
        }
        pros::delay(10);
    }
    // Stop intake after delay
    intake.move(0);
}

void ColorSort::sorted_intake_loop() {
    while (true) {
        bool passed = ringInspect();
        if (master.get_digital(DIGITAL_R1)) {
            // Intaking
            if (passed) {
                intake_redirect.set_value(0);
                intake.move(127);
            } else {
                // Wrong color, redirect intake
                intake_redirect.set_value(1);
            }
        }
        else if (master.get_digital(DIGITAL_L1)) {
            // Outtaking, no color sorting
            intake_redirect.set_value(0);
            intake.move(-127);
        } else {
            intake.brake();
            intake_redirect.set_value(0);
        }
        pros::delay(10);
    }
}

}