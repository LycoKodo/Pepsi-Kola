#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <sys/wait.h>

#include "lemlib/selector.hpp" // For auton selector
#include "robot-config.hpp"
#include "controls.hpp"
#include "autons.hpp"
#include "mcl.hpp"

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

    // VEXcode when_started: capture mechanism home positions and brake modes
    mechanisms_init();

    // Seed the particle filter on the calibrated pose and let it run from here
    // on, through auton and driver control alike. It only observes until
    // mcl::setCorrectionEnabled(true) -- see include/mcl.hpp
    mcl::init();
    mcl::start();

    // -----------------------
    // Initialise Sensors
    // -----------------------
    lv_obj_del(img); // stop displaying TKSRC Logo as Calibration has ended

    // -----------------------
    // LVGL INIT
    // -----------------------

    // qual button 0 maps to auton 1 (selector.cpp: `auton = i > 3 ? i : i + 1`).
    // the AI and skills tabs have no routines yet, but an empty map would leave
    // LVGL's button matrix with zero buttons, which the selector has never run with
    static const char* qualBtnmMap[] = {"VEXCODE", nullptr};
    static const char* qualBtnmDesc[] = {"Ported VEXcode auton: 5 wall pushes, turn 90, score", nullptr};

    static const char* AIBtnmMap[] = {"NONE", nullptr};
    static const char* AIBtnmDesc[] = {"No AI routines yet", nullptr};

    static const char* skillsBtnmMap[] = {"NONE", nullptr};
    static const char* skillsBtnmDesc[] = {"No skills routines yet", nullptr};

    lemlib::selector::init(999, qualBtnmMap, AIBtnmMap, skillsBtnmMap, qualBtnmDesc, AIBtnmDesc, skillsBtnmDesc); // declaring default auton
}

void disabled() {} // @note -> Potentially for selecting clsrt (dont trust me bro fr)

void competition_initialize() {}

void autonomous() {
    lemlib::selector::autonStarted = true;

    // one case per routine: 1..100 qual, -1..-100 AI, 101.. skills
    switch (lemlib::selector::auton) {
        case 1: autonVexcode(); break;
        default: auton1ring(); break;
    }
}

void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.setPose(0,0,0);
    
    
    pros::Task driveTask(drivetrain_control);
    pros::Task teleopTask(teleop_loop);
}
