#pragma once

#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "controls.hpp"

#include "lemlib/selector.hpp"
#include <chrono>

// Color sorting + intake control class
namespace macros{
    class ColorSort {
        public:
            // Constructor
            ColorSort();

            // Initialise with optional alliance ('R', 'B', or 'N' = auto-detect)
            void initialise(char alliance = 'N');

            // Run intake for a set time with color-sorting
            void Intake_SortedMove(int voltage, float msDelay, int penaltyFactor = 1, bool async = false);

            // Continuous loop that runs color-sorted intake control (blocking)
            void sorted_intake_loop();

            // Public alliance indicator
            char alliance;

            char get_alliance();

            // Ring inspection based on hue + alliance
            bool ringInspect();

        private:
            // Alliance detection via optical sensor
            char identify_alliance();
    };

}
