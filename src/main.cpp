#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"

#include "lemlib/selector.hpp" // For auton selector
#include "robot-config.hpp"
#include "controls.hpp"
#include "autons.hpp"
#include "macros.hpp"
#include "pidtuner.hpp"
#include "subsystems.hpp"

// THIS CODE IS POWERED BY A PROJECT MANAGER AND TWO DORITO FANS POWERED BY 
/*
         __                              ___   __        .ama     ,
      ,d888a                          ,d88888888888ba.  ,88"I)   d
     a88']8i                         a88".8"8)   `"8888:88  " _a8'
   .d8P' PP                        .d8P'.8  d)      "8:88:baad8P'
  ,d8P' ,ama,   .aa,  .ama.g ,mmm  d8P' 8  .8'        88):888P'
 ,d88' d8[ "8..a8"88 ,8I"88[ I88' d88   ]IaI"        d8[         
 a88' dP "bm8mP8'(8'.8I  8[      d88'    `"         .88          
,88I ]8'  .d'.8     88' ,8' I[  ,88P ,ama    ,ama,  d8[  .ama.g
[88' I8, .d' ]8,  ,88B ,d8 aI   (88',88"8)  d8[ "8. 88 ,8I"88[
]88  `888P'  `8888" "88P"8m"    I88 88[ 8[ dP "bm8m88[.8I  8[
]88,          _,,aaaaaa,_       I88 8"  8 ]P'  .d' 88 88' ,8' I[
`888a,.  ,aadd88888888888bma.   )88,  ,]I I8, .d' )88a8B ,d8 aI
  "888888PP"'        `8""""""8   "888PP'  `888P'  `88P"88P"8m"
*/
// Blessing of the water slug...
/*
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠠⠀⠤⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢊⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡠⠄⠒⠂⠀⠈⠀⠐⢒⡤⠄⠘⠂⠈⠉⠉⠠⠔⠋⠉⠀⠀⠀⠀⠀⠉⠐⢄⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠎⠀⣀⣄⡀⠀⠀⠀⠶⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡠⠒⠊⠣⡀⠀⠀⣣⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⠂⠹⣟⠀⡨⠱⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠑⢄⠀⠀⢨⡷⣤⡇⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢳⠊⠀⢰⣉⠆⠢⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⢔⣉⡆⠀⠀⠣⣠⡶⡑⡼⢢⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠄⠢⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠀⠀⠀⠀⠀⠀⠀⠀⠈⣷⡼⠁⠀⠡⡀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠠⠊⠀⢠⠃⠀⠀⠀⠀⠀⠀⡇⠀⠀⡄⠀⠀⣄⠀⡾⡀⠀⠀⠀⡇⠀⠀⠀⠀⠘⡆⠀⠀⠀⢷⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⢀⠊⠀⡔⢂⠇⠀⠀⢀⠀⠀⠀⢸⢸⠠⠞⠃⠀⠀⠛⡤⡇⡇⠀⠀⢠⡇⠀⠀⠀⠀⠀⢰⠀⠀⠀⠀⢣⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠈⠄⠀⠇⠸⠀⠀⠀⢸⡀⠀⠀⢺⠈⡄⠀⠀⠀⠀⠀⢠⠁⢰⠀⠀⢸⠇⠀⠀⠀⠀⠀⠈⡆⠀⠀⠀⠘⣆⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠈⠂⠰⡇⠀⠀⠀⡈⢹⠀⠀⣼⡀⢱⠀⠀⠀⠀⠀⣼⣀⢸⠀⠀⡆⢸⠀⠀⠀⠀⠀⠀⡇⠀⠀⠀⠀⢹⠢⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⢀⠤⣒⣷⠀⠀⠀⡇⠀⢱⣴⣿⠋⠀⢳⠀⠀⠀⢀⠋⣙⣿⣶⣼⡀⠸⠀⠀⡸⠀⠀⠀⡇⠀⠀⠀⠀⠀⡆⠡⠀⠀
⠀⠀⠀⠀⠀⠀⠀⣔⠔⠀⡊⢸⡀⡀⠀⣇⣴⢟⣿⣾⣿⣆⠀⢣⠀⠀⡎⣾⣟⣿⣼⣏⠙⢿⣦⡠⠃⠀⢀⣀⣔⡀⠀⠀⠀⠀⢱⠀⠡⠀
⠀⠀⠀⠀⠀⠀⢰⠂⠀⠀⢑⣸⠹⡧⡀⣿⠁⣸⠟⠛⠛⡻⠀⢠⡑⡜⠀⢟⠉⠉⠉⣻⡀⢺⠟⡇⢶⠭⠥⠤⣾⠀⠀⠀⠀⠀⠈⡄⠀⢡
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣸⠭⠭⣸⠚⠀⢊⣒⠒⠊⠁⠀⠚⠓⠛⠀⠀⠉⣭⢹⣁⠩⠔⠚⠓⠚⠛⠛⠣⢧⣇⠀⠀⠀⠀⠀⢱⠀⠀
⠀⠀⠀⠀⠀⢀⡠⠄⠂⠉⢁⠼⡒⠈⠑⠠⠤⣈⣉⣀⣀⣀⣀⣀⣀⣀⣀⣀⠄⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⣆⠀⠀⠀⠀⠈⠆⠀
⠀⠀⠀⡠⠂⠁⠀⠀⠀⠀⠀⠀⡇⠀⠀⠀⠀⡇⢠⠠⠄⠂⠤⠐⡘⢓⡞⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡀⠼⡆⠀⠀⠀⠀⠘⡀
⠀⡠⠚⠠⢄⡀⠀⠀⠀⠀⠀⠀⢸⠀⠀⠀⠀⡇⢂⢐⣈⣔⣆⠡⠐⡼⠐⠒⠂⠤⣀⠀⠀⢀⠠⠤⠀⠒⠈⣉⠀⠤⠐⢻⣀⠀⠀⠀⠀⠡
⢀⠯⡀⠀⠀⠈⠑⠢⡀⠀⠀⢀⠠⢇⢰⡀⠀⢗⡩⢣⠜⡠⢷⢈⡐⣇⣀⠀⠀⠀⠀⠉⠒⢄⡂⠀⠈⠉⠀⢀⡠⣄⡀⣸⠈⠢⣀⠀⠀⠘
⠘⠰⡄⣱⣢⣄⠀⠀⠈⣂⣔⠥⠒⠉⠺⡙⢠⢼⠰⡡⢊⠾⣉⢢⠄⠳⠠⡍⠁⠒⠠⠄⣀⠀⢀⠦⢤⣒⣊⠭⠝⠓⠉⠀⠀⠀⠈⠂⢤⠂
⠀⠀⠀⠁⠑⠐⠉⠂⠚⠉⠀⠀⠀⠀⠀⠀⢸⠌⡑⢆⡩⣼⠚⠤⣜⡠⠁⠜⡉⡖⠚⢒⠡⠉⠅⢂⠳⢀⣀⡲⠾⠂⠀⠀⠀⠀⠀⠀⠀⠁
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡆⢎⠱⡨⡐⣽⣉⣐⡆⠀⠉⠉⠒⢻⠨⠤⢂⣉⡐⠂⡧⠤⠤⠤⠔⠂⠀⠀⠀⠀⠀⠀⢠⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡏⡜⠰⣁⠖⡠⢯⡱⠀⠀⠀⠀⠀⢸⠠⠄⠁⠀⠀⠀⠈⢐⣒⣒⠲⠇⠀⢀⣀⠤⢶⢤⠜⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠑⠬⣑⠢⢌⡑⢢⡇⠀⠀⠀⠀⠀⠀⡇⠀⠀⠀⠀⠀⠀⠈⠒⠒⠒⠒⠉⠁⡇⢎⠼⠧⣀⣀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢐⠨⡑⠤⠓⣌⠸⢄⡇⠀⠀⠀⠀⠀⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡏⢆⠪⡑⠤⠀
 */
// ----------------------------------------------------
// IMPORTANT - Autonomous Selector Routine Declaration
// ----------------------------------------------------

LV_IMG_DECLARE(image); //TKSRC Logo
void initialize() {
    // ------------------------------------------
    // Displaying TKSRC Logo (During calibration)
    // ------------------------------------------
    lv_obj_t *img = lv_img_create(lv_scr_act()); // Add to screen
    lv_img_set_src(img, &image); // Link to source image
    lv_obj_align(img, LV_ALIGN_CENTER, 0, -20);

    // -----------------------
    // Initialise Chassis
    // -----------------------
    // @note - LLEMU CANNOT BE USED WITH LVGL | pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // -----------------------
    // Initialise Mechanisms
    // -----------------------
    // mechanisms are zeroed where they sit now: power on with the cascade down
    // and the roller stowed
    subsystems::init();
    subsystems::startColourToggle();
    macros::init();

    lv_obj_del(img); // stop displaying TKSRC Logo as Calibration has ended

    // -----------------------
    // LVGL INIT
    // -----------------------

    // Routine numbers: qual 1, 2, 3...  AI -1, -2...  skills 101, 102...
    // (see the button handlers in src/lemlib/selector.cpp for how a button maps to a number).
    // No routines exist for the new robot yet. Every tab needs at least one
    // button (LVGL crashes on an empty map), so each has a placeholder that
    // runAuton() treats as "nothing selected"
    static const char* qualBtnmMap[] = {"NONE", nullptr};
    static const char* qualBtnmDesc[] = {"No routines yet", nullptr};

    static const char* AIBtnmMap[] = {"NONE", nullptr};
    static const char* AIBtnmDesc[] = {"No routines yet", nullptr};

    static const char* skillsBtnmMap[] = {"NONE", nullptr};
    static const char* skillsBtnmDesc[] = {"No routines yet", nullptr};

    lemlib::selector::init(999, qualBtnmMap, AIBtnmMap, skillsBtnmMap, qualBtnmDesc, AIBtnmDesc, skillsBtnmDesc); // declaring default auton
    pidtuner::init(lemlib::selector::tabview); // PID tuning tab, added after the selector's tabs
}

// a macro started at the end of one period must not carry on into the next.
// Cancel it, give its task a moment to exit, then stop every mechanism so no
// motor is left chasing a move_absolute target
static void stopMechanisms() {
    macros::cancel();
    for (int t = 0; t < 100 && macros::isBusy(); t += 10) pros::delay(10);
    subsystems::stopAll();
}

void disabled() { stopMechanisms(); }

void competition_initialize() {}

void runAuton(int routine) {
    // one case per routine: 1..100 qual, -1..-100 AI, 101.. skills, 999 none.
    // No routines yet, so the placeholder buttons (1, -1, 101) land in default too
    switch (routine) {
        // nothing selected (999) or a placeholder: do nothing
        default: std::printf("[auton] no routine selected (%d)\n", routine); break;
    }
}

void autonomous() {
    stopMechanisms();
    lemlib::selector::autonStarted = true;
    runAuton(lemlib::selector::auton);
}

void opcontrol() {
    // the loop runs right here, in the opcontrol task, rather than in tasks it
    // spawns: PROS kills this task when the mode changes, but not its children,
    // so spawned loops would keep driving into the next auton/disabled period
    driverControl();
}
