#ifndef SELECTOR_HPP
#define SELECTOR_HPP

#include <iostream>
#include <stdexcept>
#include "api.h"
#include "liblvgl/core/lv_event.h"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/lvgl.h"
#include "liblvgl/core/lv_disp.h"
#include "liblvgl/extra/widgets/tabview/lv_tabview.h"
#include "liblvgl/misc/lv_anim.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_types.h"
#include "liblvgl/widgets/lv_btnmatrix.h"

namespace lemlib::selector {

// Enumeration to define the autonomous state
enum AutonState {
    NONE = 0,
    QUAL = 1,
    AI = -1,
    SKILLS = 101
};

extern bool autonStarted;

// Global variables for selector logic
extern AutonState autonState;
extern int auton;
extern int autonCount;

extern uint16_t currentRedButton;
extern uint16_t currentBlueButton;

extern lv_timer_t* tabWatcher_timer; // runs tabWatcher on LVGL's task

constexpr int MAX_AUTONS = 100;
extern const char* redBtnmMap[MAX_AUTONS];
extern const char* blueBtnmMap[MAX_AUTONS];

extern lv_obj_t* tabview;
extern lv_obj_t* redBtnm;
extern lv_obj_t* blueBtnm;

// Function declarations
int textUpdate();
void log_error(const std::string& func_name, const std::string& msg);
uint16_t get_button_count(lv_obj_t* btnm);
void deselect_all_buttons(lv_obj_t* btnm);
void redBtnmAction(lv_event_t* e);
void blueBtnmAction(lv_event_t* e);
void tabWatcher(lv_timer_t* timer);
void init(int default_auton, const char** redAutons, const char** blueAutons, const char** skillsAutons,
            const char** redDesc, const char** blueDesc, const char** skillsDesc);
void destroy();

// The routines as one list (Qual, then Skills, then Tests; no "NONE" placeholders), for the controller's
// routine picker (src/ctrlselect.cpp)
int routineCount();
int routineNumberAt(int index);        // what `auton` becomes
const char* routineNameAt(int index);  // the button's text
const char* routineTabAt(int index);   // "QUAL", "SKILLS" or "TEST"
int routineIndexOf(int number);        // -1: not in the list (999, or a placeholder)
// Pick routine `index` (-1: none, auton 999). Safe from any task: the brain screen follows on LVGL's task
void selectRoutine(int index);

} // namespace lemlib::selector

#endif // SELECTOR_HPP