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
#include "liblvgl/misc/lv_timer.h"
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

extern uint16_t currentQualButton;
extern uint16_t currentAIButton;
extern uint16_t currentSKillsButton;

extern lv_timer_t* tabWatcher_timer;

constexpr int MAX_AUTONS = 100;
extern const char* qualBtnmMap[MAX_AUTONS];
extern const char* AIBtnmMap[MAX_AUTONS];
extern const char* skillsBtnmMap[MAX_AUTONS];

extern lv_obj_t* tabview;
extern lv_obj_t* qualBtnm;
extern lv_obj_t* AIBtnm;
extern lv_obj_t* skillsBtnm;

// Function declarations
int textUpdate();
void log_error(const std::string& func_name, const std::string& msg);
uint16_t get_button_count(lv_obj_t* btnm);
void deselect_all_buttons(lv_obj_t* btnm);
void qualBtnmAction(lv_event_t* e);
void AIBtnmAction(lv_event_t* e);
void skillsBtnmAction(lv_event_t* e);
void tabWatcher(lv_timer_t* timer);
void init(int default_auton, const char** qualAutons, const char** AIAutons, const char** skillsAutons,
            const char** qualDesc, const char** AIDesc, const char** skillsDesc);
void destroy();

} // namespace lemlib::selector

#endif // SELECTOR_HPP
