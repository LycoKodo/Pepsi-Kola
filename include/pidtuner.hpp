#ifndef PIDTUNER_HPP
#define PIDTUNER_HPP

#include "liblvgl/lvgl.h"

// On-brain PID tuning tab: edit LemLib's lateral/angular gains and run a test move
namespace pidtuner {

// adds the "PID" tab as the last tab of the given tabview
void init(lv_obj_t* tabview);

// true while a test motion is in progress (driver control should leave the drivetrain alone)
bool isRunning();

} // namespace pidtuner

#endif // PIDTUNER_HPP
