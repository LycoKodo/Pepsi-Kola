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
#include "liblvgl/widgets/lv_textarea.h"
#include "pros/rtos.hpp"

#include "robot-config.hpp"
#include "subsystems.hpp"
#include "lemlib/selector.hpp"

LV_IMG_DECLARE(dorito); // picture on the blank last tab (src/dorito.c); the TKSRC logo is `image`, in main.cpp

/**
 * @brief THIS IS A TODO LIST
 * [√] - TODO - Make different btnmtrx for different tabs, rn its just standard, and make it in main
 * [√] - TODO - LOW PRIORITY - (in main.cpp) put in all cases of auton (RED AND BLUE 1-6) for caseswitch
 * [√] - TODO - LOW PRIORITY - (in main.cpp) add a case for the "Skills" tab
 * [√] - TODO - Lock to dev tab once an auton is selected
 * [] - TODO - (colorsort): refractor a colorsort file with all colorsort routines in auton, driver and normal alg iykwis
 */

namespace lemlib::selector {

// TODO - Refactoring into OOP
    // 1. Use constructor to declare button number, tab number etc
    // 2. Allow methods to start/stop/modify selector
    // 3. (I can work on this if u wnat) add comment section below each button

AutonState autonState = NONE;
int auton = 999;
int AILength;
int qualLength;
int skillsLength;

bool autonStarted = false;

uint16_t currentQualButton = UINT16_MAX; // button IDs for buttons selected rn
uint16_t currentAIButton = UINT16_MAX;
uint16_t currentSKillsButton = UINT16_MAX;

void tabWatcher(lv_timer_t* timer);

lv_timer_t* tabWatcher_timer = nullptr;

const char* qualBtnmMap[MAX_AUTONS];
const char* AIBtnmMap[MAX_AUTONS];
const char* skillsBtnmMap[MAX_AUTONS];

const char* qualBtnmDesc[MAX_AUTONS];
const char* AIBtnmDesc[MAX_AUTONS];   
const char* skillsBtnmDesc[MAX_AUTONS];

int motorUpdate();
int odomUpdate();
int customUpdate();
int textUpdate();

lv_obj_t* tabview = lv_tabview_create(lv_scr_act(), LV_DIR_TOP, 50);
// Creating LVGL buttons & tab object

lv_obj_t* qualBtnm;
lv_obj_t* AIBtnm;
lv_obj_t* skillsBtnm;

lv_obj_t* devTab = lv_tabview_add_tab(tabview, "Developer");
lv_obj_t* qualTab = lv_tabview_add_tab(tabview, "Qual");
lv_obj_t* AITab = lv_tabview_add_tab(tabview, "AI");
lv_obj_t* skillsTab = lv_tabview_add_tab(tabview, "Skills");
lv_obj_t* bocchiTab = lv_tabview_add_tab(tabview, " ");

// Instead of lv_obj_t *motor_temps_textarea = lv_textarea_create(devTab);
lv_obj_t *motor_temps_textarea = lv_textarea_create(devTab);
lv_obj_t *odom_textarea = lv_textarea_create(devTab);
lv_obj_t *other_textarea = lv_textarea_create(devTab);

lv_obj_t *qual_textarea = lv_textarea_create(qualTab);
lv_obj_t *AI_textarea = lv_textarea_create(AITab);
lv_obj_t *skills_textarea = lv_textarea_create(skillsTab);


void log_error(const std::string& func_name, const std::string& msg) {
    std::cerr << "[ERROR] in function " << func_name << ": " << msg << std::endl;
}

uint16_t get_button_count(lv_obj_t* btnm) {
    const char** map = lv_btnmatrix_get_map(btnm);
    uint16_t count = 0;
    while (map[count] != NULL && strcmp(map[count], "") != 0) {
        count++;
    }
    return count;
}

void deselect_all_buttons(lv_obj_t* btnm) {
    uint16_t btn_count = get_button_count(btnm);
    for (uint16_t i = 0; i < btn_count; i++) {
        lv_btnmatrix_clear_btn_ctrl(btnm, i, LV_BTNMATRIX_CTRL_CHECKED);
    }
}

// Alliance button on the Qual tab: tap to swap red <-> blue. Picks which colours the colour toggle
// (port 17) spins on; the autons are the same for both. Runs on the display task, like every lv_ call
lv_obj_t* allianceBtn = nullptr;
lv_obj_t* allianceLabel = nullptr;

void showAlliance() {
    const bool blue = subsystems::isBlueAlliance();
    lv_obj_set_style_bg_color(allianceBtn, lv_palette_main(blue ? LV_PALETTE_BLUE : LV_PALETTE_RED), 0);
    lv_label_set_text(allianceLabel, blue ? "BLUE ALLIANCE" : "RED ALLIANCE");
}

void allianceBtnAction(lv_event_t*) {
    subsystems::setBlueAlliance(!subsystems::isBlueAlliance());
    showAlliance();
    std::printf("[selector] %s alliance\n", subsystems::isBlueAlliance() ? "blue" : "red");
}

void qualBtnmAction(lv_event_t* e) {
    try {
        lv_obj_t* btnm = lv_event_get_target(e); // know our button matrix
        uint16_t btn_id = lv_btnmatrix_get_selected_btn(btnm); // get button id
        const char* txt = lv_btnmatrix_get_btn_text(btnm, btn_id);

        if (txt == nullptr) throw std::runtime_error("No active button text found!");

        printf("Qual button: %s selected\n", txt);
        printf("Button ID: %i\n", btn_id);

        deselect_all_buttons(AIBtnm);
        deselect_all_buttons(skillsBtnm);

        currentQualButton = btn_id;
        currentAIButton = UINT16_MAX;
        currentSKillsButton = UINT16_MAX;

        for (int i = 0; i < qualLength; i++) {
            if (strcmp(txt, qualBtnmMap[i]) == 0) {
                // HOLY SHT THIS IS BAD BUT IT WORKS
                if (i > 3) auton = i;
                else auton = i + 1; // determining what program/routine is ran
                autonState = lemlib::selector::AutonState::QUAL;
                break;
            }
        }
    } catch (const std::exception& ex) {
        log_error("qualBtnmAction", ex.what());
    }
}

void AIBtnmAction(lv_event_t* e) {
    try {
        lv_obj_t* btnm = lv_event_get_target(e);
        uint16_t btn_id = lv_btnmatrix_get_selected_btn(btnm);
        const char* txt = lv_btnmatrix_get_btn_text(btnm, btn_id);

        if (txt == nullptr) throw std::runtime_error("No active button text found!");

        printf("AI button: %s selected\n", txt);
        printf("Button ID: %i\n", btn_id);

        deselect_all_buttons(qualBtnm);
        deselect_all_buttons(skillsBtnm);

        currentAIButton = btn_id;
        currentQualButton = UINT16_MAX;
        currentSKillsButton = UINT16_MAX;

        for (int i = 0; i < AILength; i++) {
            if (strcmp(txt, AIBtnmMap[i]) == 0) {
                if (i > 3) auton = -(i);
                else auton = -(i + 1);
                autonState = lemlib::selector::AutonState::AI;
                break;
            }
        }
    } catch (const std::exception& ex) {
        log_error("AIBtnmAction", ex.what());
    }
}

void skillsBtnmAction(lv_event_t* e) {
    try {
        lv_obj_t* btnm = lv_event_get_target(e);
        uint16_t btn_id = lv_btnmatrix_get_selected_btn(btnm);
        const char* txt = lv_btnmatrix_get_btn_text(btnm, btn_id);

        if (txt == nullptr) throw std::runtime_error("No active button text found!");

        printf("Skills button: %s selected\n", txt);
        printf("Button ID: %i\n", btn_id);

        deselect_all_buttons(qualBtnm);
        deselect_all_buttons(AIBtnm);

        currentAIButton = UINT16_MAX;
        currentQualButton = UINT16_MAX;
        currentSKillsButton = btn_id;
        
        for (int i = 0; i < skillsLength; i++) {
            if (strcmp(txt, skillsBtnmMap[i]) == 0) {
                if (i > 3) auton = (i + 100); 
                else auton = (i + 101);
                autonState = lemlib::selector::AutonState::SKILLS;
                break;
            }   
        }
    } catch (const std::exception& ex) {
        log_error("skillsBtnmAction", ex.what());
    }
}

// How often the screen is refreshed. Was a 10 ms loop in its own task
constexpr uint32_t TAB_WATCH_MS = 50;

// One refresh of the selector and Developer tab. Runs as an lv_timer, so on LVGL's own task.
// Until 2026-11-23 this was a separate task ("tabWatcher") looping every 10 ms and calling lv_* from outside
// LVGL's task, which PROS does not lock. That race crashed the program on the robot (DATA ABORT EXCEPTION,
// CURRENT TASK: tabWatcher, three times in one afternoon of UMBmark runs). The work per refresh is unchanged
void tabWatcher(lv_timer_t*) {
    try {
        int activeTab = lv_tabview_get_tab_act(tabview);
        if (!autonStarted) {
            // Handle actions for the active tab
            if (activeTab == 1 && currentQualButton < UINT16_MAX) {
                deselect_all_buttons(qualBtnm);
                lv_btnmatrix_set_btn_ctrl(qualBtnm, currentQualButton, LV_BTNMATRIX_CTRL_CHECKED);
            } else if (activeTab == 2 && currentAIButton < UINT16_MAX) {
                deselect_all_buttons(AIBtnm);
                lv_btnmatrix_set_btn_ctrl(AIBtnm, currentAIButton, LV_BTNMATRIX_CTRL_CHECKED);
            } else if (activeTab == 3 && currentSKillsButton < UINT16_MAX) {
                deselect_all_buttons(skillsBtnm);
                lv_btnmatrix_set_btn_ctrl(skillsBtnm, currentSKillsButton, LV_BTNMATRIX_CTRL_CHECKED);
            } else if (activeTab == 0) {
                motorUpdate();
                odomUpdate();
            }

            // "No auton picked" reminder: rumbled every 10 ms until a selector button was tapped.
            // Paused 2026-11-19 for PID tuning; set RUMBLE_UNTIL_PICKED to true to bring it back
            // (it would now rumble every TAB_WATCH_MS)
            constexpr bool RUMBLE_UNTIL_PICKED = false;
            if (RUMBLE_UNTIL_PICKED && auton == 999) {
                master.rumble(".");
            }
            textUpdate();
        } else {
            // Auton has started
            lv_tabview_set_act(tabview, 0, LV_ANIM_OFF);
            motorUpdate();
            odomUpdate();
        }
    } catch (const std::exception& ex) {
        log_error("tabWatcher", ex.what());
    }
}


void init(int default_auton, const char** qualAutons, const char** AIAutons, const char** skillsAutons,
            const char** qualDesc, const char** AIDesc, const char** skillsDesc) {    
    try {
        // For button names
        int i = 0;
        while (AIAutons[i] != nullptr && i < MAX_AUTONS) {
            AIBtnmMap[i] = AIAutons[i];
            i++;
        }
        AILength = i;
        i = 0;

        while (qualAutons[i] != nullptr && i < MAX_AUTONS) {
            qualBtnmMap[i] = qualAutons[i];
            i++;
        }
        qualLength = i;
        i = 0;

        while (skillsAutons[i] != nullptr && i < MAX_AUTONS) {
            skillsBtnmMap[i] = skillsAutons[i];
            i++;
        }
        skillsLength = i;

        // For button descriptions
        i = 0;
        while (qualDesc[i] != nullptr && i < MAX_AUTONS) {
            qualBtnmDesc[i] = qualDesc[i];
            i++;
        }
        i = 0;
        while (AIDesc[i] != nullptr && i < MAX_AUTONS) {
            AIBtnmDesc[i] = AIDesc[i];
            i++;
        }
        i = 0;
        while (skillsDesc[i] != nullptr && i < MAX_AUTONS) {
            skillsBtnmDesc[i] = skillsDesc[i];
            i++;
        }

        auton = default_auton;
        autonState = (auton > 0) ? lemlib::selector::AutonState::QUAL : ((auton < 0) ? lemlib::selector::AutonState::AI : NONE);

        // Declaring color theme
        lv_theme_t* th = lv_theme_default_init(
            NULL,
            lv_color_hsv_to_rgb(0, 100, 100),
            lv_color_hsv_to_rgb(0, 50, 100),
            true,
            NULL
        );
        lv_disp_set_theme(NULL, th);

        qualBtnm = lv_btnmatrix_create(qualTab);
        lv_btnmatrix_set_map(qualBtnm, qualBtnmMap);
        lv_obj_set_size(qualBtnm, 290,180);
        lv_obj_align(qualBtnm, LV_ALIGN_LEFT_MID, -15, 0);
        lv_btnmatrix_set_btn_ctrl_all(qualBtnm, LV_BTNMATRIX_CTRL_CHECKABLE);
        lv_obj_add_event_cb(qualBtnm, qualBtnmAction, LV_EVENT_VALUE_CHANGED, NULL);

        // the description, shortened to make room for the alliance button under it
        lv_obj_set_size(qual_textarea, 180, 128);
        lv_obj_align(qual_textarea, LV_ALIGN_LEFT_MID, 280, -26);

        allianceBtn = lv_btn_create(qualTab);
        lv_obj_set_size(allianceBtn, 180, 44);
        lv_obj_align(allianceBtn, LV_ALIGN_LEFT_MID, 280, 66);
        lv_obj_add_event_cb(allianceBtn, allianceBtnAction, LV_EVENT_CLICKED, NULL);
        allianceLabel = lv_label_create(allianceBtn);
        lv_obj_center(allianceLabel);
        showAlliance();

        lv_textarea_add_text(qual_textarea, "Greetings Adventure. Here lies the great QUAL alliance autonomous routines");
        lv_textarea_set_cursor_click_pos(qual_textarea, false);
        lv_textarea_set_password_mode(qual_textarea, false);
        lv_obj_add_state(qual_textarea, LV_STATE_DISABLED);

        AIBtnm = lv_btnmatrix_create(AITab);
        lv_btnmatrix_set_map(AIBtnm, AIBtnmMap);
        lv_obj_set_size(AIBtnm, 290, 180);
        lv_obj_align(AIBtnm, LV_ALIGN_LEFT_MID, -15, 0);
        lv_btnmatrix_set_btn_ctrl_all(AIBtnm, LV_BTNMATRIX_CTRL_CHECKABLE);
        lv_obj_add_event_cb(AIBtnm, AIBtnmAction, LV_EVENT_VALUE_CHANGED, NULL);

        lv_obj_set_size(AI_textarea, 180, 180);
        lv_obj_align(AI_textarea, LV_ALIGN_LEFT_MID, 280, 0);

        lv_textarea_add_text(AI_textarea, "Greetings Adventure. Here lies the great AI alliance autonomous routines");
        lv_textarea_set_cursor_click_pos(AI_textarea, false);
        lv_textarea_set_password_mode(AI_textarea, false);
        lv_obj_add_state(AI_textarea, LV_STATE_DISABLED);

        skillsBtnm = lv_btnmatrix_create(skillsTab);
        lv_btnmatrix_set_map(skillsBtnm, skillsBtnmMap);
        lv_obj_set_size(skillsBtnm, 290, 180);
        lv_obj_align(skillsBtnm, LV_ALIGN_LEFT_MID, -15, 0);
        lv_btnmatrix_set_btn_ctrl_all(skillsBtnm, LV_BTNMATRIX_CTRL_CHECKABLE);
        lv_obj_add_event_cb(skillsBtnm, skillsBtnmAction, LV_EVENT_VALUE_CHANGED, NULL);

        lv_obj_set_size(skills_textarea, 180, 180);
        lv_obj_align(skills_textarea, LV_ALIGN_LEFT_MID, 280, 0);
        
        lv_textarea_add_text(skills_textarea, "Greetings Adventure. Here lies the great SKILLS autonomous routines");
        lv_textarea_set_cursor_click_pos(skills_textarea, false);
        lv_textarea_set_password_mode(skills_textarea, false);
        lv_obj_add_state(skills_textarea, LV_STATE_DISABLED);

        // Define layout dimensions for L-shaped split screen
        const int width = 480;
        const int height = 272;

        const int left_width = width / 2; // Left side width for Motor Temps
        const int right_width = width - left_width;
        const int top_height = height / 2; // Top height for Odometry

        // Create "Motor Temps" text area
        lv_obj_set_size(motor_temps_textarea, left_width - 30, 177);
        lv_obj_align(motor_temps_textarea, LV_ALIGN_TOP_LEFT, -12, -12);
        lv_textarea_add_text(motor_temps_textarea, "Left 1: \n");
        lv_textarea_add_text(motor_temps_textarea, "Left 2: \n");
        lv_textarea_add_text(motor_temps_textarea, "Left 3: \n");
        lv_textarea_add_text(motor_temps_textarea, "Right 1: \n");
        lv_textarea_add_text(motor_temps_textarea, "Right 2: \n");
        lv_textarea_add_text(motor_temps_textarea, "Right 3: \n---------------------------- \n");
        lv_textarea_add_text(motor_temps_textarea, "Intake: \n");
        lv_textarea_add_text(motor_temps_textarea, "WallMech: ");

        lv_textarea_set_cursor_click_pos(motor_temps_textarea, false);
        lv_textarea_set_password_mode(motor_temps_textarea, false);
        lv_obj_add_state(motor_temps_textarea, LV_STATE_DISABLED);

        lv_obj_set_size(odom_textarea, left_width, 74);
        lv_obj_align(odom_textarea, LV_ALIGN_TOP_LEFT, 203, -12);
        lv_textarea_add_text(odom_textarea, "X: \n");
        lv_textarea_add_text(odom_textarea, "Y: \n");
        lv_textarea_add_text(odom_textarea, "Theta: ");

        lv_textarea_set_cursor_click_pos(odom_textarea, false);
        lv_textarea_set_password_mode(odom_textarea, false);
        lv_obj_add_state(odom_textarea, LV_STATE_DISABLED);

        lv_obj_set_size(other_textarea, left_width, 100);
        lv_obj_align(other_textarea, LV_ALIGN_TOP_LEFT, 203, 65);
        lv_textarea_add_text(other_textarea, "Alliance: \n");
        lv_textarea_add_text(other_textarea, "Lady Position: \n");
        lv_textarea_add_text(other_textarea, "Chassis Kp: \n");
        lv_textarea_add_text(other_textarea, "Chassis Ki: \n");
        lv_textarea_add_text(other_textarea, "Chassis Kd: ");

        lv_textarea_set_cursor_click_pos(other_textarea, false);
        lv_textarea_set_password_mode(other_textarea, false);
        lv_obj_add_state(other_textarea, LV_STATE_DISABLED);


        lv_obj_t *img = lv_img_create(bocchiTab);
        lv_img_set_src(img, &dorito); // Link to source image
        lv_obj_align(img, LV_ALIGN_CENTER, 0, 0); // Align image in the tab

        // Image scale: the zoom factor is in 256ths, so 200 = about 78%
        lv_img_set_zoom(img, 200);

        tabWatcher_timer = lv_timer_create(tabWatcher, TAB_WATCH_MS, nullptr);

        printf("Selector initialised!\n");

    } catch (const std::exception& ex) {
        log_error("init", ex.what());
    }
}

void destroy() {
    // nothing calls destroy() today. lv_timer_del is an lv_* call: only call this from LVGL's task
    if (tabWatcher_timer != nullptr) {
        lv_timer_del(tabWatcher_timer);
        tabWatcher_timer = nullptr;
    }
}

// Holy shit this is horrible but it fucking works so dont u dare touch it
// @LycoKodo *wags finger* nuh uh (i didnt touch it dw)
// TODO: make this less terrible            <-- this was an autocomplete suggested comment btw :skull:
// TODO: figure out how to highlight which motor to prioritise cooling

int motorUpdate() {
    int ret = 0;
    lv_textarea_set_text(motor_temps_textarea, ""); // Clear the text area

    std::vector<double> leftTemps = leftMotors.get_temperature_all();
    std::vector<double> rightTemps = rightMotors.get_temperature_all();

    char buffer[64]; // Buffer to hold formatted strings

    try {
        double current = 0; //leftTemps.at(0); // TODO - TEMPORARY
        if (current == PROS_ERR_F) {
            lv_textarea_add_text(motor_temps_textarea, "Left 1: ERROR (PROS)\n");
        } else {
            snprintf(buffer, sizeof(buffer), "Left 1: %d\n", static_cast<int>(current));
            lv_textarea_add_text(motor_temps_textarea, buffer);
        }
    } catch (std::out_of_range&) {
        lv_textarea_add_text(motor_temps_textarea, "Left 1: ERROR (OOR)\n");
        ret = 1;
    }

    try {
        double current = 0; //leftTemps.at(1); // TODO - TEMPORARY
        if (current == PROS_ERR_F) {
            lv_textarea_add_text(motor_temps_textarea, "Left 2: ERROR (PROS)\n");
        } else {
            snprintf(buffer, sizeof(buffer), "Left 2: %d\n", static_cast<int>(current));
            lv_textarea_add_text(motor_temps_textarea, buffer);
        }
    } catch (std::out_of_range&) {
        lv_textarea_add_text(motor_temps_textarea, "Left 2: ERROR (OOR)\n");
        ret = 1;
    }
    
    try {
        double current = 0; //leftTemps.at(2); // TODO - TEMPORARY
        if (current == PROS_ERR_F) {
            lv_textarea_add_text(motor_temps_textarea, "Left 3: ERROR (PROS)\n");
        } else {
            snprintf(buffer, sizeof(buffer), "Left 3: %d\n", static_cast<int>(current));
            lv_textarea_add_text(motor_temps_textarea, buffer);
        }
    } catch (std::out_of_range&) {
        lv_textarea_add_text(motor_temps_textarea, "Left 3: ERROR (OOR)\n");
        ret = 1;
    }

    try {
        double current = 0; // rightTemps.at(0); // TODO - TEMPORARY
        if (current == PROS_ERR_F) {
            lv_textarea_add_text(motor_temps_textarea, "Right 1: ERROR (PROS)\n");
        } else {
            snprintf(buffer, sizeof(buffer), "Right 1: %d\n", static_cast<int>(current));
            lv_textarea_add_text(motor_temps_textarea, buffer);
        }
    } catch (std::out_of_range&) {
        lv_textarea_add_text(motor_temps_textarea, "Right 1: ERROR (OOR)\n");
        ret = 1;
    }

    try {
        double current = 0; //rightTemps.at(1); // TODO - TEMPORARY
        if (current == PROS_ERR_F) {
            lv_textarea_add_text(motor_temps_textarea, "Right 2: ERROR (PROS)\n");
        } else {
            snprintf(buffer, sizeof(buffer), "Right 2: %d\n", static_cast<int>(current));
            lv_textarea_add_text(motor_temps_textarea, buffer);
        }
    } catch (std::out_of_range&) {
        lv_textarea_add_text(motor_temps_textarea, "Right 2: ERROR (OOR)\n");
        ret = 1;
    }

    try {
        double current = 0; //rightTemps.at(2); // TODO - TEMPORARY
        if (current == PROS_ERR_F) {
            lv_textarea_add_text(motor_temps_textarea, "Right 3: ERROR (PROS)\n---------------------------- \n");
        } else {
            snprintf(buffer, sizeof(buffer), "Right 3: %d\n---------------------------- \n", static_cast<int>(current));
            lv_textarea_add_text(motor_temps_textarea, buffer);
        }
    } catch (std::out_of_range&) {
        lv_textarea_add_text(motor_temps_textarea, "Right 3: ERROR (OOR)\n---------------------------- \n");
        ret = 1;
    }

    // @LycoKodo do we rlly need intake and wall mech? The hottest motors during the tournament are not going to be these two by a long shot
    // @LycoKodo in the worst case we remove lady cuz intake can sometimes get hot as well, and replace lady temp with hottest motor
    // @LycoKodo wdyt? 

    try {
        double current = 999;//intake.get_temperature();
        if (current == PROS_ERR_F) {
            lv_textarea_add_text(motor_temps_textarea, "Intake: ERROR (PROS)\n");
        } else {
            snprintf(buffer, sizeof(buffer), "Intake: %d\n", static_cast<int>(current));
            lv_textarea_add_text(motor_temps_textarea, buffer);
        }
    } catch (std::out_of_range&) {
        lv_textarea_add_text(motor_temps_textarea, "Intake: ERROR (OOR)\n");
        ret = 1;
    }

    // Note: IMO this is not rlly needed as in a tournament in a rush you will NOT prioritise cooling lady brown motor
    try {
        double current = 999; //lady.get_temperature();
        if (current == PROS_ERR_F) {
            lv_textarea_add_text(motor_temps_textarea, "WallMech: ERROR (PROS)");
        } else {
            snprintf(buffer, sizeof(buffer), "WallMech: %d", static_cast<int>(current));
            lv_textarea_add_text(motor_temps_textarea, buffer);
        }
    } catch (std::out_of_range&) {
        lv_textarea_add_text(motor_temps_textarea, "WallMech: ERROR (OOR)");
        ret = 1;
    }

    return ret;
}

int odomUpdate() {
    char buffer[64]; // Buffer to hold formatted strings

    float x = chassis.getPose().x;
    float y = chassis.getPose().y;
    float theta = chassis.getPose().theta;

    lv_textarea_set_text(odom_textarea, ""); // Clear

    snprintf(buffer, sizeof(buffer), "X: %f\n", x); //chassis.getPose().x
    lv_textarea_add_text(odom_textarea, buffer);

    snprintf(buffer, sizeof(buffer), "Y: %f\n", y); //chassis.getPose().y
    lv_textarea_add_text(odom_textarea, buffer);

    snprintf(buffer, sizeof(buffer), "Theta: %f", theta); //chassis.getPose().theta
    lv_textarea_add_text(odom_textarea, buffer); 

    return 0;
}

int customUpdate() {
    lv_textarea_set_text(other_textarea, ""); // Clear

    char buffer[64]; // Buffer to hold formatted strings

    // Logic to Update Alliance

    // char alliance = 'N';
    // if (auton >= 1 && auton <= 100){
    //     alliance = 'R';
    // } else if (auton >= -100 && auton <= -1) {
    //     alliance = 'B';
    // } else if (auton >= 101 && auton <= 201) {
    //     alliance = 'S'; // @note: alliance for skills is RED
    // }

    // char alliance = colorsort.alliance;

    char alliance = 'N'; // TODO - TEMPORARY

    snprintf(buffer, sizeof(buffer), "Alliance: %c\n", alliance);
    lv_textarea_add_text(other_textarea, buffer);

    snprintf(buffer, sizeof(buffer), "Lady Rotation: %.2f\n", 9.9);
    lv_textarea_add_text(other_textarea, buffer);

    snprintf(buffer, sizeof(buffer), "Drive P: %.2f I: %.2f D: %.2f\n", linearController.kP, linearController.kI, linearController.kD);
    lv_textarea_add_text(other_textarea, buffer);

    snprintf(buffer, sizeof(buffer), "Turn P: %.2f I: %.2f D: %.2f\n", angularController.kP, angularController.kI, angularController.kD);
    lv_textarea_add_text(other_textarea, buffer);

    return 0;
}

int textUpdate() {
    int activeTab = lv_tabview_get_tab_act(tabview);
    if (activeTab == 0) {
        motorUpdate();
        odomUpdate();
        customUpdate();
    } else if (activeTab == 1) {
        // Update text for Qual tab
        lv_textarea_set_text(qual_textarea, ""); // Clear the text area
        if (currentQualButton < qualLength) {
            lv_textarea_add_text(qual_textarea, qualBtnmDesc[currentQualButton]);
        }
    } else if (activeTab == 2) {
        // Update text for AI tab
        lv_textarea_set_text(AI_textarea, ""); // Clear the text area
        if (currentAIButton < AILength) {
            lv_textarea_add_text(AI_textarea, AIBtnmDesc[currentAIButton]);
        }
    } else if (activeTab == 3) {
        // Update text for Skills tab
        lv_textarea_set_text(skills_textarea, ""); // Clear the text area
        if (currentSKillsButton < skillsLength) {
            lv_textarea_add_text(skills_textarea, skillsBtnmDesc[currentSKillsButton]);
        }
    }
    return 0;
}




} // namespace lemlib::selector

//----------------------------------------------------------------//
//                                                                //
// Dear Programmer(s)/Manager(s),                                 //
//                                                                //
// When I wrote this code, only God and I knew how it worked.     //
// Now, only God knows it...                                      //
//                                                                //
// Therefore, if you are trying to optimize this routine          //
// and it fails (most surely), please increase this counter       //
// as a warning for the next person:                              //
//                                                                //
// total_hours_wasted_here = 5.1                                  //
//                                                                //
//----------------------------------------------------------------//

// From @LycoKodo: Nah I'd win

// SEE BELOW

/*
⠀⠀⠀⠀⠀⠀⠀⠀⢀⢔⠾⢋⠷⢃⠠⠒⠈⠀⢀⣀⢂⢠⣲⢦⡪⠝⠀⢠⠎⠀⠀⠀⠀⠀⣠⣴⢪⡃⠜⠋⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠄⡀⠖⠁⠀⢀⠀⠀⡀⠀⠀⠀⠀⢀⣤⠦⡻⠂⠀⠀⠀⢼⡆⠀⠀⠁⡔⡀⡸⠀⢠⠃⠀⢸⣐⣷⣏⠉⠁⠉⢻⡄⠀⠀⡱⠑⢆⣨⠟⠊⠉⠀⠀⠈⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⢀⢔⠕⢁⠔⠁⠐⣁⣤⠴⠚⠉⢀⣠⠖⡫⠃⠁⠀⠀⣰⠃⠀⠀⡠⠀⢠⠞⡵⠃⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⠀⢀⠔⠈⠀⠀⠀⠔⡰⢃⠔⠁⠀⠀⠀⠼⡫⣠⠎⠀⠀⢀⠤⠀⠀⠀⠀⠀⠀⠰⢁⠃⢀⠇⠀⠀⣼⠋⣟⡆⠇⢀⠀⠸⢎⢵⠀⠱⠱⡈⢧⠄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⣠⡡⠁⢠⣁⣀⡴⡚⠅⠐⠈⢀⡴⠋⠐⠁⢀⡠⡤⠄⣰⠃⠀⠀⣰⠁⠀⣠⠊⠀⠀⠀⠀⢀⣀⣠⣤⠤⠶⠚⠋⠉⢀⡠⠀⠀⠀⠀⢠⣮⠞⣐⠅⠀⠀⠀⠀⢀⡴⠋⠁⣀⣠⠔⠁⠀⠀⢠⠃⠀⠀⠀⢦⠂⢠⠊⠀⠀⠚⠙⢰⢸⣷⢰⠈⢆⠀⠙⣮⢣⠀⠐⡔⡒⣧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⢠⠟⣠⠖⠋⢁⠚⠃⣀⠔⠚⡷⠉⣉⡤⡲⢭⠞⢉⠃⣰⠏⠀⠀⡴⣉⡀⡚⠁⠑⠒⠀⡛⠛⢉⢁⠄⠀⣠⠗⠀⢀⡴⠋⠀⢀⠤⢠⢾⠋⢡⠞⠁⠀⡠⠒⠀⢀⣎⣠⢞⢵⠟⠁⠀⢀⠔⠠⠃⠀⡔⡐⠀⡇⠀⠁⠀⠀⠀⠀⡃⠈⠀⣿⠈⡀⡇⠣⡀⠈⢧⠡⡀⠈⢊⢜⣧⡀⠀⠀⠀⠀⠀⠀⠀⣀⡠⢤⣤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⢸⢶⠃⠀⢀⠃⢠⡞⠁⢀⡼⡷⢋⣥⣮⠴⠁⡠⣵⢻⡟⠀⢀⡼⢋⢊⠌⠀⡠⠊⢀⠊⢀⣀⣆⠃⠀⣰⠃⢀⡴⠋⠀⢀⠔⠕⡡⠞⣠⠝⠁⠀⣠⠊⢀⣤⠖⣡⠞⣕⡡⠁⠀⣠⡞⡡⣶⡵⠀⣸⢣⠇⢸⡟⠀⠀⠀⡀⠀⢠⠁⡆⠀⡷⠀⡇⣏⡄⢻⠄⠀⠱⡷⣄⠀⠡⡹⡇⠀⢀⡀⠄⠒⠈⠁⠀⠀⠀⠀⠉⠓⢤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠈⠙⠀⠀⡎⢠⠋⠀⣠⡮⠔⠈⣩⠞⠁⢀⢊⡾⢡⡄⠁⢠⡾⠡⠡⢂⠠⠊⢀⠔⣀⡴⢋⡏⠎⠀⣸⠃⣰⠟⠁⠀⡐⠁⡡⡊⠔⠈⡁⢐⣔⡟⢡⠞⡑⣡⠎⣡⠞⠝⠀⢀⣮⢟⠊⡸⠹⠁⢰⠃⣼⠀⣿⠂⢰⠀⢰⠃⠀⡌⢰⡗⢰⡧⠀⢫⡷⠇⠀⣎⢆⠀⡇⠏⢳⡤⠜⠓⠈⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠓⢄⡀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠑⣿⣖⡾⠋⠀⡠⠊⠁⢠⣖⣵⡭⡂⠁⠘⠄⢳⠁⡶⠓⣡⣰⣖⡥⠞⠁⣀⢼⢱⠀⣰⢃⡼⠃⠀⢠⡪⣪⠞⠋⡀⢔⣠⠦⠛⠉⣠⡳⢊⡴⢣⠞⢁⠊⠀⣠⡿⠛⢁⠎⢠⡳⢡⠏⢸⠟⢠⢟⠀⢸⠀⢸⠁⠀⢁⡿⠁⢠⠇⠀⠘⡇⠘⠀⡆⣾⠠⠓⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⠦⡀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⡀⠝⠛⠀⡠⢊⣠⠾⠗⡾⠁⢳⠀⣽⡄⠀⢘⠾⣊⠴⢋⡵⢫⣷⣃⢀⠔⠁⣿⠆⠀⠀⡞⢁⠀⠴⠛⠘⣀⣔⡬⢖⠋⠁⢀⠔⣶⡟⠡⢈⡕⠛⠠⠂⠀⣰⠋⠀⡰⠁⢀⣧⢡⡎⢀⠟⠀⣸⢹⠀⢸⠀⡸⠀⠀⣾⠁⠀⡌⠀⣀⡸⠙⣄⠶⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠦⡀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠉⠀⠀⣰⣪⠖⠋⠀⢀⠜⢀⢌⠜⡆⣿⠼⣺⢗⣟⣡⡎⢡⠃⣤⠹⠘⠢⡤⢄⣛⡐⠠⠼⠍⠐⠀⣀⡤⡞⠉⠁⣤⠋⢀⠔⠁⣼⠏⠐⢠⠎⠐⠰⠃⠀⡼⠁⠀⡜⠀⡰⣻⢃⡞⠀⣸⠃⢀⡟⢰⠀⢸⠀⣷⠀⣸⠁⠀⡘⠀⣼⡿⠁⢠⢏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢦⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⢎⣸⠃⠀⠀⡠⢃⣴⠟⣡⣴⣿⣷⠛⠛⡈⡇⢠⠗⢸⣾⠸⡇⠀⠀⠈⠑⡾⣫⢒⣴⣶⢟⠵⢡⠌⠀⠀⠔⠃⡠⠁⠀⣾⢋⠌⡰⠁⠀⠠⢁⡄⠐⠀⠀⢞⡒⡰⢠⡏⡾⢠⢧⡏⠀⢺⠯⢥⠀⢸⠀⡽⢠⠃⠀⡰⠇⣸⠗⠃⢠⢳⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠷⡄⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣨⣢⢔⣡⠖⢫⠽⠑⡟⠉⢸⢯⢏⠉⣵⡇⣸⠀⢘⢨⠃⡁⠓⠒⢢⢞⢜⣥⣫⣿⡧⠃⠀⡌⠀⠀⠈⠀⠊⠀⠀⣼⠃⠊⡐⠀⠀⢠⠣⡞⢠⡆⠀⡎⠀⠀⠁⣼⡝⢠⠟⡸⠀⠀⢸⢐⣸⠂⢸⠀⡇⢂⠄⡠⣧⣷⠏⠀⢠⡇⡈⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⣴⡆⠀⢰⡆⠀⢠⣾⣷⠀⢀⣾⠀⠀⣾⠆⠀⠀⠀⠀⠀⠀⠀⠀⠀⢹⡆
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠚⠿⠚⠋⠁⠌⠁⡠⠊⡐⣡⢟⡌⡈⢒⡏⢰⢸⢰⢸⢺⡄⠀⢀⡴⣷⢿⠏⢈⣿⠟⠷⢆⡤⢀⣀⠀⠀⠀⠀⠀⢀⣏⠌⡔⠀⠀⠀⣆⠾⢁⡞⡇⡜⠀⠀⠀⣀⣯⡴⣥⢷⠓⠒⠋⠉⠡⢸⠀⢸⠀⠇⠎⣠⠱⢸⠇⠀⢠⠃⢠⢁⡇⠀⠀⠀⠀⠀⠀⠀⠀⢰⣿⣷⢀⡿⢁⣴⠟⣹⣿⠀⣾⣷⣶⣾⡟⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⡇
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠔⠋⣀⣪⠞⡑⢁⢧⠙⠀⢇⢸⣿⢺⡞⡚⡯⢴⡙⠉⢀⡐⠂⡘⡞⠀⠅⠠⠉⠁⠚⠣⠝⣔⠶⣀⠀⠁⡰⠀⠀⠀⠀⡏⣄⡜⠀⡷⢓⣢⠿⠍⠛⠋⠠⡡⠂⠀⠀⠀⢀⠀⢀⡇⢨⠀⣶⡜⢸⠀⡟⠀⠀⢆⣠⠃⡈⡇⠀⠀⠀⠀⠀⠀⠀⢀⣿⠉⣿⣿⢃⣾⠿⠟⢻⣟⣸⡟⠁⢠⡿⠁⢀⣴⠆⠀⠀⠀⠀⠀⠀⠀⢠⡇
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⠴⠞⠧⠜⠋⢁⠂⠌⡰⢃⣾⣦⠀⢸⡀⣿⣄⢁⠇⠀⠀⢯⣝⣖⣿⣯⣿⣒⡭⠥⣐⡒⠤⢀⠀⠀⠈⠉⡛⢆⢠⠃⠀⠀⠀⠀⠑⠋⢀⣀⠻⠉⠁⠀⠀⢀⡠⠔⣒⣀⣭⣝⣛⣫⣿⣿⣧⠘⠀⣳⠀⣼⢠⠃⠀⠀⠸⣹⢀⠃⡇⠀⠀⠀⠀⠀⠀⠀⠘⠃⠀⠘⠃⠘⠃⠀⠀⠙⠋⠛⠀⠀⠙⠃⢠⠞⠁⠀⠀⠀⠀⠀⠀⠀⠀⢸⠃
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠁⠀⠀⠀⠀⢀⠂⣼⡞⠁⣾⡡⠘⡆⢸⡇⢸⡘⡈⢶⠀⠀⠀⢼⣣⠀⠀⢸⡄⠈⢽⡇⠛⠷⣦⣽⠀⠀⠀⠘⡟⡇⠀⠀⠀⠀⠀⠀⢠⣾⠆⠀⠀⠀⠀⢀⣯⡴⣾⠛⠩⠧⠄⣸⠃⠀⠀⣼⠆⢰⣇⡼⢰⣿⠄⠀⠀⠠⢿⠸⢰⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣀⣤⡀⣤⡄⢀⣠⣤⣤⣤⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣨⡴⠋⠀⢰⣿⠁⢀⣿⣸⠁⢶⠧⠀⠘⢇⠀⠀⠘⢞⡄⠀⠀⠙⠧⣀⡂⠤⠂⠈⢿⣇⠀⠀⠀⠼⢿⠀⠀⠀⠀⠀⠀⢠⠘⠆⠀⠀⠀⠀⣼⡏⠀⠈⠢⢄⡤⠴⠋⠀⠀⡰⠸⡄⢳⣷⢇⡂⣯⠀⠀⢠⠃⠎⢀⠟⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢻⡟⠉⣿⠋⠀⣿⠟⠉⠉⣽⡟⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠀⠀⠀⣿⡏⠀⣼⢣⣿⡄⢸⡇⠀⠀⠈⠂⠀⠀⠀⠜⢄⡀⠀⠀⠀⠀⠀⠀⢀⡟⠯⡶⡶⢀⡶⠋⠀⠀⠀⠀⠀⠀⠀⠁⠀⢀⣄⡀⢰⡟⠳⡀⠀⠀⠀⠀⠀⠀⢀⠔⠣⠀⡌⡟⡆⢸⠇⠈⠁⡤⢸⡀⢠⠎⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⡿⠁⠀⠀⠀⢰⡟⣀⣤⡾⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡟⢣⠜⠁⡾⠼⡆⠀⣷⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⠉⠁⠀⠚⠋⠀⠌⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠓⣞⣤⡀⠁⠉⠉⠉⠉⠀⠀⠀⠀⠀⠀⠀⣷⢸⡁⠈⠀⠀⢀⡇⠀⢡⢿⠁⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠠⣶⡾⠿⠟⠂⠀⠀⠿⠿⠛⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠄⠊⠁⠀⡸⡷⢣⠺⣆⢸⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡰⠇⠘⡄⠀⠀⠀⢸⠀⢠⢿⢹⠀⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡀⠀⠀⠀⠀⠀⠀⢀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⡵⠓⠁⠀⠈⠛⣷⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠐⠁⠀⠀⠁⠀⠀⢀⣇⣠⣟⡟⡜⡀⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⣾⡆⢠⣷⡆⠀⣼⠇⣰⡇⢰⣷⠀⢀⣾⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠁⠀⠀⠀⠀⠀⠀⢹⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⠋⣿⢰⠇⢸⣱⡀⢤⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⢠⡾⣿⣇⣼⠏⢠⡿⢀⣿⢿⣧⣼⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⠂
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⣧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⠇⠀⣿⣿⠀⠀⡏⠁⢸⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⡟⠁⢿⣿⠋⢀⣾⠁⣾⠃⠈⣿⡟⠀⣤⡦⠀⠀⠀⠀⠀⠀⠀⠀⢸⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢹⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢰⡟⠀⢰⣟⠇⠀⠈⠀⠀⡼⠗⠀⠀⠀⠀⠀⠀⠀⠈⠉⠀⠀⠈⠁⠀⠈⠃⠀⠉⠀⠀⠈⠁⠀⠉⠀⠀⠀⠀⠀⠀⠀⠀⢀⠞⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⣇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣿⠁⠀⡾⠊⠀⠀⠀⠀⠀⠹⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠎⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢻⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠓⢄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠋⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⣷⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠋⢦⡀⠀⢀⠄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣴⡟⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠂⢄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⠃⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢻⡽⢆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠁⠀⠈⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠞⣿⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠐⠢⢄⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣠⣴⡾⠁⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⡎⢳⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠞⠁⣼⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡀⠤⠀⠀⠀⠀⠀⠈⠉⠓⢾⠗⣀⠤⠤⠤⠄⠀⠀⠀⠀⠈⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣯⣳⠀⠹⣆⠀⠀⠀⠀⠀⠰⠦⠄⣀⣀⡀⠀⠀⠀⠀⠀⢀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⡀⠠⠴⠛⠁⠀⠀⠀⢀⡜⠁⠀⡰⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⢋⠖⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⢌⡆⠀⠈⢷⡀⠀⠀⠀⠀⠀⠀⠀⠀⠨⡙⢿⣿⣛⠛⠛⠛⠛⠛⠛⠻⣛⣩⢿⡋⠉⠀⠀⠀⠀⠀⠀⠀⢀⡴⠋⠀⢀⣴⠿⣿⠅⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⠗⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡏⠳⣵⠀⠀⠀⠹⣦⠀⠀⠀⠀⠀⠀⠀⠀⠀⠑⠦⣤⣤⣭⣭⣭⣭⣭⢩⡭⠖⠃⠀⠀⠀⠀⠀⠀⠀⢀⡴⠋⠀⠀⠀⣲⠯⢸⡿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠠⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡇⠀⠈⠀⠀⠀⠀⠈⢳⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡴⠋⠀⠀⠀⠀⢰⡿⢌⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡇⠀⠀⠀⠀⠀⠀⠀⠩⡿⣦⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠎⠀⠀⠀⠀⠀⠀⢐⡺⢁⣿⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣼⣧⠀⢈⠢⡀⠀⠀⠀⠀⢘⣝⢿⣆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣾⠏⠀⠀⠀⠀⠀⠀⠒⠁⠀⢸⢹⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣤⣶⣿⣿⣿⣿⠀⠀⠑⢌⢲⣀⠀⠀⠀⠳⣅⡫⠳⣤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣾⡽⠏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⣴⣾⣿⣿⣿⣿⣿⣿⣷⠀⠀⠀⢳⣍⠣⡀⠀⠀⠀⢻⡟⠆⠈⠹⠶⣤⣀⣀⣀⣀⡀⣀⢀⣀⣀⣠⣴⠿⣛⢝⡃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣶⣤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣴⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣆⠀⠀⠈⢯⡫⡳⡀⠀⠀⠀⠻⣯⡀⠀⠀⠀⠀⠈⠉⠉⠙⠙⠋⠛⠙⢫⣍⡳⢎⠋⠀⠀⠀⣠⡴⠖⠀⠀⠀⠀⠀⢀⣿⣿⣿⣿⣿⣷⣤⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⣴⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣄⠀⠈⡟⣬⡑⠄⠀⠀⠀⠱⡷⣂⠤⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣜⣪⡍⠀⠀⠀⠀⡰⠏⠁⣀⠀⠀⠀⠀⠀⣸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣦⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣤⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣶⣄⠸⡦⡋⡦⠀⠀⠀⠀⠈⢎⠙⠕⣓⠂⠤⢀⢠⠄⠀⠀⠜⠳⠋⠀⠀⠀⢀⡾⢋⡥⠴⣇⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣶⣤⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⢀⣀⣤⣴⣶⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣌⡢⡹⡄⠀⠀⠀⠀⠘⡖⠔⣒⡠⠄⡭⠃⠀⠀⠐⡺⠁⠀⠀⢀⡴⢿⡙⠃⣐⠒⠉⠁⢀⣠⣴⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣦⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⣀⣠⣴⣶⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣶⣧⣀⡀⠀⠀⠀⠻⠅⡒⠄⢹⡁⠀⠀⠀⢻⠀⠀⠀⢠⣾⣿⡋⢠⠑⢀⣃⣤⣶⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣦⣄⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
*/

/*
⠀⠀⠀⠀⠀⠀⢀⠞⡰⠁⠀⡼⠀⠀⠀⣰⠇⣸⡇⠀⠀⠀⠀⠀⠀⠀⢸⠁⠀⠀⠀⠀⡄⣂⠀⢞⡤⡄⠀⣧⠀⠠⡄⠀⠀⠘⡞⠿⠹⣞⢮⠻⡝⢦⠀⠀⠀⠀⠀⠀
⠀⢤⣀⣼⣁⣀⡞⡸⠁⠀⢰⠃⠀⠀⠀⠛⢀⠟⠁⠀⠀⢸⡇⠀⠀⠀⣸⠀⠀⠀⠀⢸⢸⠻⠀⠈⠃⡇⠀⢸⠀⠀⢧⠀⠀⠀⢹⡀⠀⠁⠀⢣⡹⡌⠳⡀⠀⠀⠀⠀
⠞⠉⠀⣐⠆⠈⠑⠣⡄⠀⡞⠀⠀⠀⠀⠀⣸⠀⠀⠀⠀⣸⠀⠀⠀⠀⡇⠀⠀⠀⠀⢸⠨⠀⠀⠀⠀⢳⠀⢸⡆⠀⢸⠀⠀⠀⠀⡇⠀⠀⠀⠀⢱⡹⡄⠙⣅⠀⠀⠀
⠀⠀⠈⠀⠀⠀⠈⠂⡇⢰⠃⠀⠀⠀⠀⠀⡏⠀⠀⠀⢠⣿⠀⠀⠀⢰⡇⠀⠀⠀⠀⢸⠀⠀⠀⠀⠀⢸⠀⢸⢧⠀⠘⡇⠀⠀⠀⢸⠀⠀⢦⠀⠀⢣⢱⠀⠙⡆⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⡧⡾⠀⠀⠀⠀⠀⠀⡇⠀⠰⢤⣸⡇⠀⠀⠀⡾⢳⠀⠀⠀⠀⠘⡆⠀⠀⠀⠀⢸⠀⢸⢸⠀⠀⣿⣠⠄⠀⢸⠀⠀⢸⠀⠀⡌⡎⣇⡁⠹⡄⠀
⠀⠀⠀⠀⠀⠀⠀⢠⠃⡇⠀⠀⠀⠀⠀⠀⡇⠀⠀⠀⡇⡏⠓⠲⢤⣇⣸⡀⠀⠀⠀⠀⡇⠀⠀⠀⠀⡼⣀⣾⣼⡖⠋⡏⡇⠀⠀⢸⠀⠀⠈⡇⠀⢹⣹⢸⢣⠀⢳⠀
⣻⠗⠲⢶⣒⡒⠲⡏⢹⠁⠀⠀⠀⠀⠀⠀⣇⠀⠀⢰⠃⢹⠀⠤⢼⠓⠛⣯⠍⠃⠀⠀⢻⡆⠀⠐⡾⡟⢻⠗⠚⡇⢠⠇⢹⠀⠀⢸⠀⠀⠀⣷⠀⢸⡏⡎⣟⠀⠈⠃
⠀⠠⠚⠁⠀⠈⡇⢱⣼⠀⠀⠀⠀⠀⠀⠀⣿⠀⠀⢸⣠⣼⣒⠾⢽⣒⡲⢼⡄⠀⠀⠀⠸⡇⠀⠀⣸⡵⣞⡭⠬⣯⣺⠤⠘⡄⠀⢸⠀⠀⢀⢹⠀⢸⢸⠇⡇⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠁⢸⡽⠀⠀⠀⠀⠀⠀⠀⡟⡆⣠⢿⣋⢭⣷⠖⢾⣦⣝⣷⠽⡄⠀⠀⠀⣇⠀⢀⣿⣿⣵⣶⡶⡷⣾⡙⢆⡇⠀⣿⡇⠀⢸⢸⠀⣸⠘⡄⣿⠀⠀⠀
⠀⠀⠀⠀⢀⣠⠆⢸⡇⠀⠀⠀⠀⠀⠀⠀⡇⣹⣣⢾⢱⣋⣾⣄⣸⠈⣧⠙⠂⠹⡄⠀⠀⢸⡄⠘⡿⢱⣣⣾⣿⣿⢹⡙⣎⣷⢰⡇⣇⠀⡼⡞⢀⡏⠀⣿⡇⠀⠀⠀
⠤⣀⣈⣀⡤⠴⠒⠁⡇⠀⠀⠀⠀⠀⠀⠀⡇⣇⣿⣸⣞⣩⣿⣷⣿⢭⣹⠀⠀⠀⠙⢦⠀⠀⢧⡞⠁⣾⣭⣿⣿⡯⣽⠇⢸⣾⡾⠀⢸⢠⠇⠁⡼⠀⠀⡟⡇⠀⠀⠀
⠀⠀⠀⢀⡇⠀⠀⠀⡇⠀⠀⠀⠀⠀⠀⢀⡇⠹⠆⠻⢹⡃⠙⠻⢟⠀⡿⠀⠀⠀⠀⢠⢗⣄⡈⣇⠀⢹⣀⠙⠯⡄⡿⠀⠘⢸⡇⠀⢸⡎⢀⣼⡇⠀⠀⠀⠈⠀⠀⠀
⠀⠀⠀⢸⠁⠀⠀⠀⡇⠀⠀⠀⠀⠀⠀⢸⠇⠀⠀⠀⠀⠙⠚⠀⠘⠚⠁⠀⠀⠀⢠⠿⠋⠀⠙⠺⠆⠀⠙⠒⠒⠊⠀⠀⠀⣼⠀⠀⢸⠀⠈⣿⡇⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠈⢺⠀⠀⠀⠀⡇⠀⠀⠀⠀⠀⠀⢸⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡇⣰⠆⠀⠀⠀⣿⡇⠀⠀⠀⠀⠀⠀⠀
⠀⠀⣰⢿⠀⠀⠀⠀⡇⠀⠀⠀⠀⠀⠀⢸⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⡰⠃⠀⠀⠀⠀⣿⡇⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠉⢸⠀⠀⠀⢠⡇⠀⠀⠀⠀⠀⠀⢸⣧⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⠁⠀⠀⠀⠀⠀⣿⡇⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⡼⠀⠀⠀⢸⡇⠀⠀⠀⠀⠀⠀⢸⡇⡷⣄⠀⠀⠀⠀⠀⠀⠀⠀⠠⠖⠦⠤⠤⠀⠤⠠⠀⠀⠀⠀⠀⠀⠀⢀⠜⢹⠀⠀⠀⠀⠀⠀⣿⡇⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⡇⠀⠀⠀⢸⢧⠀⢰⠀⠀⠀⠀⠀⣇⡇⢨⠗⢄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡤⠞⡇⠀⢸⠀⠀⠀⠀⠀⠀⣿⡇⠀⠀⠀⠀⠀⠀⠀
⠀⠀⡀⡇⠀⠀⠀⡏⢸⠀⢰⠀⠀⠀⠀⠀⣿⣁⡼⠤⠖⢻⠢⢄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡤⠚⡟⠀⠀⡇⠀⢸⠀⠀⠀⠀⠀⢀⢹⡇⠀⠀⠀⠀⠀⠀⠀
⠀⠀⢃⠇⠀⠀⢀⡇⢸⠀⠚⡆⠀⠀⠀⠀⣿⠹⡀⠀⠀⢸⠀⠀⠉⠓⠦⣄⡀⠀⠀⠀⠀⣀⡴⠚⠉⠉⣳⢧⡀⠀⡇⠀⢸⠀⠀⠀⠀⠀⢸⢸⡇⠀⠀⠀⠀⠀⠀⠀
⠀⠀⢼⠀⠀⠀⢸⠀⠈⡆⢸⣇⠀⠀⠀⠀⣷⠀⠈⠓⠢⠼⣄⣀⣀⠀⠀⠈⠙⠻⢶⣶⣿⢿⣡⠤⠖⠊⠁⢸⠃⠀⡇⠀⢸⠀⠀⠀⠀⡇⣸⢸⡇⠀⠀⠀⠀⠀⠀⠀
⠀⠀⣸⠀⠀⠀⡞⠀⠀⡇⠈⣿⡀⠀⠀⠀⣿⡆⠀⠀⠀⠀⠀⠀⠀⠉⠉⠉⠒⠢⢄⣀⡴⠋⠀⠀⠀⠀⠀⡾⠀⠀⢻⠀⢸⠀⠀⠀⢸⠁⡇⢸⡇⠀⠀⠀⠀⠀⠀⠀
⠀⠈⡇⠀⠀⢰⠃⣀⣀⣳⠀⣧⣇⠀⠀⠀⢹⢽⠄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢻⠀⠀⠀⠀⠀⠀⢰⣋⡉⠳⣼⣤⣼⠀⠀⠀⣼⢠⡇⢸⡇⠀⠀⠀⠀⠀⠀⠀
⠀⢠⠃⣠⡶⠟⠛⢉⣉⣹⡄⢻⣘⡆⠀⠀⢸⠀⠈⠲⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⠀⠀⠀⠀⠀⢀⡏⠁⠙⢦⣀⣀⣸⠀⠀⢠⡟⢸⠙⠻⣅⠀⠀⠀⠀⠀⠀⠀
⠀⣸⡾⠋⢀⠔⠋⠁⠀⠀⣇⢸⠀⠸⡄⠀⢸⡀⠀⠘⢮⡙⠢⢄⡀⠀⠀⠀⠀⠀⠀⢸⠀⠀⠀⠀⣠⢆⡄⠀⠀⠀⠀⠀⡇⠀⢀⡏⡇⡏⠑⢦⡈⢷⡀⠀⡀⠀⠀⠀
⢠⡿⠁⡴⠃⠀⠀⠀⠀⠀⢸⣼⠀⠀⠹⡄⠘⡇⠀⠀⠀⠙⠢⣄⠈⠙⠲⠤⠄⠀⠀⢩⠀⠀⣠⠞⡵⡻⠃⠀⠀⠀⠀⠀⡇⢠⠎⠀⣷⠁⠀⠀⠙⢦⠻⡀⠀⠀⠀⠀
⡿⠁⡼⠁⠀⠀⠀⡆⠀⠀⠀⠻⠀⠀⠀⠹⣄⡇⠀⠀⠀⠀⠀⠈⠓⢤⡀⠀⠀⠀⠀⠈⡇⠉⠀⢼⠜⠁⠀⠀⠀⠀⠀⢸⡵⠋⠀⠸⠏⠀⠀⠀⢠⡌⢣⢳⡇⠀⠀⠀
⠁⡼⠁⠀⠀⠀⠀⢳⠀⠀⠀⠀⠀⠀⠀⠀⠘⠃⠀⠀⠀⠀⠀⠀⠀⠀⠈⠳⢤⡀⠀⢸⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠀⠀⠀⠀⢰⠀⠀⠀⢸⠀⠀⢣⢧⠀⠀⠀
⠐⠃⠀⡀⠀⠀⠀⠘⡆⠀⠀⢀⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⡆⠀⠀⢸⠀⠀⠈⢿⡆⠀⠀
⠀⠀⠀⢻⠀⠀⠀⠀⢷⠀⠀⢸⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡇⠀⠀⢸⠀⠀⢀⠘⣿⡀⠀
⠀⠀⠀⠈⣇⠀⠀⠀⢸⡀⠀⢸⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡇⠀⠀⡏⠀⢀⡏⠀⢹⣇⠀
*/

/*

⠀⠀⠀⢎⣰⡡⠏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠠⡀⠄⠀⠀⡜⠡⡀⠤⢀⡐⣠⡤⠲⠤⠐⢶⣬⣳⡄⠀⠀⠀⠀⢱⣂⣒⡼⠃⠀⠀⠀⠸⢄⣣⣆⠟⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠠⠀⠀⠀⠀⠀⠀⠀⠄⠠⠀⠀⠀⠀
⠀⠀⠀⠈⠁⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡜⠠⡁⣐⠴⠊⠁⠁⠀⠀⠀⠀⠈⠀⠈⠙⣦⠀⠀⠀⠀⠉⠉⠀⠀⠀⠀⠀⠀⠈⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢰⣁⡖⠚⠁⠀⠀⠀⠀⢀⠀⣀⠠⠤⠤⠀⠒⠒⠒⠒⠒⠀⠠⠤⠤⢀⣀⠀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⡀⢄⢦⡀⠀⠀⠀⠀⣠⠖⠶⣄⠀⠀⠀⠀⠀⠄⠀⠠⡀⠀⠀⡾⠈⡠⠂⢰⡀⡄⠆⠃⢈⣁⠠⡤⠀⠴⠠⡀⠄⣀⠠⢀⠠⢐⠒⠶⠢⠠⢬⢁⡘⠰⠠⢄⡀⠀⠀⣠⠶⢲⣄⠀⠀⠀⠀⠀⡄⠂⠦⡀⠀⠀⠀⠀⢠⠔⠰⢄⠀⠀⠀⠀⠀
⠠⢉⢆⢢⡷⠀⠀⠀⠀⣇⢎⡱⣸⠆⠀⠀⠀⠈⠀⢀⠀⡱⠀⠀⠀⠀⡣⠖⠉⣀⠤⠒⠩⢁⠂⡐⠄⠡⢂⠑⡐⠂⢄⢂⠂⠒⣀⠊⠄⠡⠈⠄⠂⠌⡑⠢⢄⡈⠑⠄⣧⢊⡵⣸⠇⠀⠀⠀⠸⢀⣠⠠⠍⡀⠀⠀⠀⣇⠠⠀⣈⠇⠀⠀⠀⠀
⠀⠈⠀⠋⠀⠀⠀⠀⠀⠈⠃⠙⠉⠀⠀⠀⠀⠀⠙⠀⠋⠁⠀⢠⠔⢈⣠⠞⠉⠄⢂⢁⠒⡀⠒⠠⠌⡁⠆⠌⡄⠩⣀⢂⠩⠐⠄⢊⠌⠡⠑⡈⠌⡐⠠⢁⠂⡉⠓⢤⡈⠙⢖⠉⠀⠀⠀⡴⠘⠉⣀⢠⠄⡈⠱⡀⠀⠈⠃⠈⠉⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⠜⢁⡴⠃⠄⢂⠡⠈⡄⠢⠌⣰⠃⡅⠂⢌⠐⠡⣀⠃⢄⠂⢂⠉⡐⢂⠌⣀⠃⠰⡐⠠⣁⠂⠆⠡⣌⠠⢉⠳⢎⣁⠙⠔⠁⣠⠔⠩⠐⠂⠌⠱⡄⢩⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⢠⠀⠀⠀⠀⠀⣀⠴⠆⣄⠀⠀⢀⢠⠰⣒⡥⢁⡴⢫⡝⢨⠐⣦⢁⠃⣤⡁⢢⡇⠱⢀⠡⠂⢌⠐⠠⠌⠂⡌⠠⠘⠠⢈⠰⢀⡘⢰⠀⢳⠆⣈⠂⠡⠌⠇⢂⡾⢁⠈⣷⠒⢊⠡⢈⡐⠡⠉⡌⢄⠳⠈⡄⠀⢀⡤⢠⣄⠀⠀⠀⠀
⠀⠀⠀⢀⠀⡁⠀⠀⠀⠀⡇⢠⠤⢜⠂⠉⠀⠀⢸⠅⣠⠋⠄⠛⢠⠁⣼⠃⢄⢺⡰⣰⣟⠠⠑⡨⢀⣿⠌⡰⢁⠂⠡⢀⠡⢃⠡⢄⠂⠆⠠⢃⡇⢂⢻⡀⡘⠄⠚⡈⢸⡅⢂⢂⣸⠇⢂⠐⠤⢀⠃⠤⢘⠠⢸⠇⡇⠀⡇⠀⠄⢸⠄⠀⠀⠀
⠀⠀⠀⠀⠐⠁⠀⠀⠀⠀⠑⠶⠆⠋⠀⠀⠀⢠⠃⣜⠂⢌⠠⡁⠆⡐⡎⡐⠈⡄⢰⣃⣯⠆⡑⠠⣜⠂⣇⠰⢀⠡⢂⡁⠒⠰⠶⠶⢬⡴⣁⡢⠐⢂⠸⡅⡐⢈⠄⢡⠂⣹⠆⢲⣇⠨⠄⢊⠐⠂⠌⢠⠁⢂⠌⡀⡇⠀⠘⠣⠖⠋⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⠃⢬⠁⠌⢂⠰⢐⡐⣼⠁⢄⡣⢞⠋⢠⠆⡐⠨⢰⡏⠀⢹⡐⠄⢂⡐⢠⠉⡔⡀⠆⢸⣇⢈⠉⠓⠦⣆⢳⠈⡄⠈⠆⣸⠃⠌⠄⣸⡆⠌⡀⠎⠰⡈⠤⠈⢄⠂⠄⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠇⢠⡇⠌⢂⡿⠈⠤⣰⢹⡼⠋⣐⡎⠀⢠⣃⠄⢃⣶⠀⠀⠀⢳⡌⡐⠠⠙⣧⡐⠠⠃⡄⢿⡆⢉⡐⢂⡘⢹⡐⠄⡉⡐⠘⣧⣈⣔⡞⢀⠂⠔⣈⠡⡐⠌⡁⠂⢌⠠⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⢀⠄⠠⡀⠀⠀⠀⠀⠀⡤⢺⡄⡜⡇⢈⠂⣧⠘⡀⣏⢸⡀⠆⣼⠀⠀⠀⡿⠐⢠⡇⠀⠐⠀⠈⢳⡄⢃⠁⠿⢳⡁⠂⠔⠸⡛⣤⠐⡠⠐⡈⣇⠂⠔⡈⠡⢠⡿⣏⢀⠂⠌⡐⠄⠒⡠⠊⢄⠡⢂⠂⠄⠀⠀⠀⠀⠀⣠⠲⢠⡀
⠄⠀⠀⠀⠀⡁⠀⠀⢸⠀⠀⠀⠀⢸⠀⢸⠄⡇⡇⠌⡐⣿⡐⢠⡏⠀⣇⢰⡋⠀⣀⣄⣸⠇⣸⠁⠀⠌⠀⠁⠀⠙⢦⠁⡸⡇⠙⢦⡁⠆⢫⣈⣦⡁⠆⡁⡧⢈⡐⠠⠃⣌⠻⣇⡠⠌⡐⠠⢉⡐⠡⠘⣀⠂⠔⡈⢸⠀⠀⠀⠀⢸⣀⠃⠆⡴
⠀⠀⠀⠀⠀⠈⠐⠀⠀⠀⠀⠀⠀⠈⠑⠞⢇⡇⢳⠈⠄⣿⢧⠸⣇⠀⣹⣼⣷⠿⠛⢿⣿⣿⣸⡀⠄⠠⠈⠐⢀⠀⠈⠳⢤⣻⣴⡿⠟⢮⡙⡿⣿⣿⣦⠐⡇⠤⢀⠑⠂⣽⠀⠌⡙⡇⢠⠁⢂⠄⢃⠡⠄⡘⠠⠄⡁⠀⠀⠀⠀⠀⠁⠞⠊⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢧⠈⢧⡈⢿⣚⣷⡷⣴⣿⠟⠀⠀⠀⠀⣿⣿⣿⣷⠀⠠⠀⡁⠀⠂⠠⠀⢰⣿⣿⠁⠀⠀⠉⠺⣹⣿⣿⣷⣸⠐⡀⢊⡁⢾⠈⡐⢠⣿⠀⠌⠄⡊⠤⢁⠊⠄⡑⠠⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠓⣜⣷⢺⣷⣿⣿⣦⣀⣀⣠⣼⣿⣿⣿⣿⠀⠐⠀⠄⠁⠄⠁⢈⣿⣿⣿⣦⣀⣀⣠⣴⣿⣿⣿⣿⡏⠰⢀⠡⡀⢿⣰⠧⣼⢿⢈⡐⢈⠰⢀⠃⠌⡐⢈⠄⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⢠⠚⢉⠲⡀⠀⠀⠀⢀⡴⢋⡑⢦⠀⠀⠀⡇⢹⣟⣿⣿⣿⣿⡿⠛⢿⣿⣏⣹⣿⡟⠀⠈⠄⠈⡀⠂⢈⠀⢹⣿⣿⡏⠁⢙⣿⣿⢋⣿⣿⣿⡇⠱⡀⠁⠆⣿⢯⠿⣭⣿⡄⠐⣀⠊⢄⠊⠔⠠⢁⠂⡷⡄⠀⠀⠀⠀⡴⢩⢍⡢
⠀⠀⠀⠀⠀⢳⡈⠄⣢⠇⠀⠀⠀⠘⣆⠢⢘⡼⠀⠀⠀⠇⢠⠉⣷⠹⣿⣿⣷⣶⣿⣿⣿⠿⠋⠀⢀⠡⠀⢂⠀⠌⠀⡀⠀⠙⠻⢿⣿⣿⣿⣿⣿⠿⠿⠋⡧⠡⠐⡉⠄⣿⠾⠻⡶⢣⡇⠡⢀⠌⢂⠌⠂⡅⠊⠄⣷⠇⠀⠀⠀⠈⢇⡣⢎⡰
⠀⠀⠀⠀⠀⠀⠈⠉⠀⠀⠀⠀⠀⠀⠀⠉⠉⠀⠀⠀⠀⢸⠠⢈⡷⠀⠀⠉⠉⠉⠁⡀⠀⡀⠄⠐⠀⠠⠐⠀⡀⠂⠐⠀⡐⠀⡀⠀⠀⠀⠁⠈⢆⣀⡰⢸⠇⠡⢁⡘⢀⠯⠀⠀⠸⡄⣃⢁⠂⠌⢂⠌⠡⡀⠥⠈⡇⠀⠀⠀⠀⠀⠀⠉⠉⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⠠⢹⡇⠀⢀⠂⠁⠈⢀⠠⠐⠀⡀⠂⠈⠀⠀⠄⠀⡀⠁⢀⠠⠐⠀⠄⠁⡈⠠⠁⠀⢀⣀⢸⣀⠣⠀⡔⢈⡆⠐⠀⢘⠇⢽⠠⢈⠐⠡⡈⠅⡐⠠⠃⡏⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⢀⣀⡀⠀⠀⠀⠀⠀⠀⣀⣀⠀⠀⠀⠀⠀⠘⡄⣿⠀⠀⢀⠠⠈⠐⡀⠄⠐⡀⢀⢀⡶⣛⠹⠳⣄⣀⣐⡀⠀⠠⠁⡈⠄⠠⠐⠈⣰⠋⠈⡿⢀⠂⢡⠐⢨⡇⠀⠈⡸⢀⣨⡔⢀⠊⢡⠐⠌⠠⠑⡈⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⢰⠋⡄⢊⣇⠀⠀⠀⠀⡞⡡⢎⢳⡄⠀⠀⠀⢰⣇⠹⣄⡀⠀⡀⠀⠁⠀⠄⠂⠀⠄⢸⡧⢌⢃⠳⡘⢤⢃⡿⠀⠐⠠⠀⢀⠂⠠⠀⠱⣄⣠⡇⠢⠌⡀⠎⢸⣁⠤⠞⡝⡱⢸⠇⠂⠌⡀⠎⣀⠃⠌⡐⡇⠀⠀⠀⠀⡔⠁⠀⠀⠀
⠀⠀⠀⠀⠈⠶⣈⠶⠋⠀⠀⠀⠀⠳⢅⡮⠞⠁⠀⠀⠀⠘⠾⡀⠄⡙⠲⢤⣁⡀⠂⢀⠈⠀⢀⠻⣔⢊⡬⢡⡙⢤⡛⠁⠀⠐⠀⡈⠀⠀⢀⡤⠖⠊⢁⢃⢁⠂⠌⠄⡛⠀⠀⠀⠓⢵⡬⡟⢈⡐⠐⠌⢄⡈⠔⡀⡇⠀⠀⠀⠀⠐⠤⠈⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠐⣇⣿⢀⠡⢸⠇⠈⠑⠒⠰⠎⢠⣤⣈⠁⠈⠓⣚⠃⣀⢤⡈⠀⠄⣠⠶⠛⠉⠀⠀⠀⠸⢂⢂⣾⠋⢰⠃⠀⠀⠀⠀⠀⠀⢸⠄⡄⠃⠌⡄⡐⠄⢂⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⣬⢣⡐⠘⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡤⠜⠛⣍⢲⠏⠀⠀⡗⠂⠤⢄⠀⠀⠀⠟⠠⡜⡝⢠⠏⠀⠀⠀⠀⠀⠀⠀⠀⡇⠤⢁⠢⢐⠠⠘⡀⠆⠀⠀⠀⠀⠀⠀⠀⠀⠀
⡀⠀⠀⠀⠀⢠⢖⡒⢦⡀⠀⠀⠀⠀⡤⠂⠐⢄⠀⠀⠀⠀⢠⠖⢻⡤⡙⢤⣋⠀⠀⡤⠒⠒⢄⣀⣀⣴⠷⣉⡀⢹⡶⠋⢀⣤⠶⠋⠁⠀⠀⣧⠤⣸⣧⣊⢠⣣⠞⡢⡄⠀⠀⠀⠀⡠⠒⠒⢼⠐⡀⢂⢂⡁⠒⢠⢸⠀⠀⠀⠀⠀⡠⢀⠀⠄
⡐⠀⠀⠀⠀⢼⠢⡜⢢⡗⠀⠀⠀⠀⠇⠀⠀⣸⠀⠀⠀⠀⢸⡀⠀⢓⡅⠀⠙⠃⠀⣧⠜⢚⠏⣴⣰⢯⠞⣍⡃⣾⠀⢰⠋⠀⠀⢧⠀⠀⡼⠸⢀⢡⠀⣈⡟⠧⣜⣡⡟⠀⠀⠀⠀⢇⠀⠀⣈⡇⡐⠂⡄⠤⠑⢂⢸⠀⠀⠀⠀⠀⢇⠂⠌⠰
⠀⠀⠀⠀⠀⠀⠉⠚⠉⠀⠀⠀⠀⠀⠈⠁⠈⠀⠀⠀⠀⠀⠀⠉⠚⠉⠀⠀⠀⢀⠔⠁⢀⡞⣨⣧⣼⣷⣶⣶⣿⣧⣤⣷⣶⣶⣦⣤⣧⡞⠁⡇⠤⡟⠠⢸⠁⠀⠈⠣⡄⠀⠀⠀⠀⠈⠑⠊⠁⠸⡠⢁⡐⠄⣁⠢⢸⠀⠀⠀⠀⠀⠈⠁⠈⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⠋⠀⢀⣾⣝⡻⡽⣿⢿⣻⡿⣿⣻⣿⣿⡿⣿⠿⣿⠿⡿⢷⡇⢂⠗⡀⢺⠀⠀⠀⣸⠈⢆⠀⠀⠀⠀⠀⠀⠀⠀⢧⢂⠐⠤⢀⠂⠹⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣐⢣⠀⠀⢸⡷⣎⡷⣹⣿⣟⣯⣿⣿⣏⣿⣿⣽⢿⣟⡼⣫⣝⡟⠇⢨⡇⠐⡟⠀⠀⠀⣷⠀⠸⡄⠀⠀⠀⠀⠀⠀⠀⢸⡄⢊⠐⡠⢉⠐⡇⠀⠀⠀⠀⠀⠀⠀⠀
⡑⢢⢄⠀⠀⠀⠀⣰⢫⡙⡝⣆⠀⠀⠀⢀⠖⠁⠈⢣⡀⠀⠀⠀⣰⠊⢩⠃⣷⠀⠀⣼⠀⣎⣿⣿⢷⣻⣿⡿⣽⠇⣿⣿⣯⣿⣻⣾⢷⡾⢰⠈⢰⡇⠂⡇⠀⠀⠀⡟⠀⠀⢓⠀⠀⠀⠀⠴⠉⠈⢑⢿⡄⢂⡐⠄⠊⡇⠙⣦⠀⠀⠀⠀⠀⠄
⡑⠊⠜⠀⡤⡄⠀⠙⣆⣜⣰⠏⡀⣀⢠⣤⢃⡀⣀⡼⠁⢀⡀⣀⠐⢀⡏⢀⡗⣠⣄⡟⣰⣧⣸⢿⣿⣿⣿⣽⣿⠃⣿⣷⣿⣝⣻⣽⡟⠀⢸⠂⣹⠁⣸⠇⠀⠀⢠⠇⠀⠀⠘⡅⠀⠀⠀⠲⣀⣂⡼⠚⣷⡀⠆⢨⠁⢧⣠⠟⠀⠀⠀⠀⠀⠂
⠀⠀⠀⠀⡗⡇⣿⣏⣿⢹⣶⣿⡏⣿⢻⣿⣿⢽⣿⣿⢸⢹⡟⣿⣯⣟⡟⣿⣿⣿⣿⣿⣿⣿⢻⣿⣯⣿⣿⡟⣮⠁⣟⣿⡾⣿⣾⣿⠁⠀⠘⡠⢹⡀⢸⠄⠀⠀⣼⠂⠀⠀⠀⢱⠀⠀⠀⠀⠀⠀⠀⠀⠸⣽⡔⠂⠌⣸⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠛⠁⠉⠉⠉⠉⠀⠉⠉⠉⠉⠉⠉⠉⠉⠁⠈⠉⠉⠉⡿⠜⠀⠙⡛⡉⢩⠋⢉⢙⣿⣿⢿⡿⣿⣹⠏⠀⣿⣾⣟⣯⣷⣿⡄⠀⠀⡇⢂⠁⡟⠀⠀⣼⡇⠀⠀⠀⠀⠀⣇⠀⠀⠀⠀⠀⠀⠀⠀⢱⡻⣔⠂⢸⡀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⣠⠤⠤⣄⠀⠀⠀⠀⢀⡤⠴⢤⡀⠀⠀⠀⠀⣠⢴⠇⠀⠀⠘⣇⡇⢸⠐⠂⣼⣿⣯⣿⣻⣿⡼⡇⠀⢻⣷⡿⣽⣾⣯⣷⠀⠀⡗⡀⢂⡟⠀⡸⡷⠀⠀⠀⠀⠀⠀⠘⡤⠤⢤⡀⠀⠀⠀⠀⢀⣿⡹⣆⢈⡇⠀⠀⢀⡤⠤⢤⠀
⠀⠀⠀⠀⠀⠁⡀⠄⠸⠀⠀⠀⠀⠈⡀⠢⠄⡇⠀⠀⠀⠐⡇⠌⠳⣄⣀⠀⢩⢼⢸⠌⣱⣿⣟⣷⢿⣳⡿⣿⠀⠀⠘⠿⠛⠋⠉⠀⠀⠀⠀⢇⠐⢨⠇⣸⢹⠁⠀⠀⠀⠀⠀⠀⠀⢳⠌⡀⡇⠀⠀⠀⠀⡇⠘⣷⡹⣆⢣⠀⠀⢸⢠⠓⢢⠁

*/
