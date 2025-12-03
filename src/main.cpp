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
#include "colorsort.hpp"

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
    odomLift.set_value(1);
    pros::delay(1000);

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
    // Initialise Sensors
    // -----------------------
    lv_obj_del(img); // stop displaying TKSRC Logo as Calibration has ended

    // -----------------------
    // LVGL INIT
    // -----------------------

    static const char* qualBtnmMap[] = {
        "MAX-R", // auton 1
        "MAX-L", // auton 2
        "LONG-L", // auton 3
        "\n", 
        "[TODO] LONG-R", // auton 4
        "MID-L", // auton 5
        "[TODO] MID-R", // auton 6
        nullptr
    };
    static const char* qualBtnmDesc[] = {
        "9 Blocks in LG, Clears Loader", // auton 1
        "9 Blocks in LG, Clears Loader", // auton 2
        "[SAFE] 4 in LG, Clears Loader", // auton 3
        "[SAFE] 4 in LG, Clears Loader", // auton 4
        "[SAFE] 4 in Mid", // auton 5
        "[SAFE] 4 in Mid", // auton 6
        nullptr // keep it a null ptr
    };

    static const char* AIBtnmMap[] = {
        "COMB-L", // AI 1
        "[TODO] COMB-R", // AI 2
        "NULL", // AI 3
        "\n",
        "NULL", // AI 4
        "NULL", // AI 5
        "NULL", // AI 6
        nullptr
    };
    static const char* AIBtnmDesc[] = {
        "", // AI 1
        "", // AI 2
        "", // AI 3
        "", // AI 4
        "", // AI 5
        "", // AI 6
        nullptr
    };

    static const char* skillsBtnmMap[] = {
        "Spinnn", "H&P", "Skills 1", "\n",  
        "Skills 2", "Skills 3", "Skills 4", nullptr
    };
    static const char* skillsBtnmDesc[] = {
        "spin intake", "I hope this works", "Skills 1",  
        "Skills 2", "Skills 3", "Skills 4", nullptr
    };

    lemlib::selector::init(999, qualBtnmMap, AIBtnmMap, skillsBtnmMap, qualBtnmDesc, AIBtnmDesc, skillsBtnmDesc); // declaring default auton
}

void disabled() {} // @note -> Potentially for selecting clsrt (dont trust me bro fr)

void competition_initialize() {}

void autonomous() {  
    // @NOTE - UNCOMMENT THE FOLLOWING TO  ENABLE AUTON SELECTOR
    int auton = lemlib::selector::auton;

    lemlib::selector::autonStarted = true;
    
    if (auton >= 1 && auton <= 100) {
        switch (auton) {
            case 1: qual1(); break;
            case 2: qual2(); break;
            case 3: qual3(); break;
            case 4: qual4(); break;
            case 5: qual5(); break;
            case 6: qual6(); break;
            // add more red cases as needed up to 100
        }
    }
    else if (auton >= -100 && auton <= -1) {
        switch (auton) {
            case -1: AI1(); break;
            case -2: AI2(); break;
            case -3: AI3(); break;
            case -4: AI4(); break;
            case -5: AI5(); break;
            case -6: AI6(); break;
            // add more blue cases as needed up to -100
        }
    }
    else if (auton >= 101 && auton <= 201) {
        switch (auton) {
            case 101: skills1(); break;
            case 102: skills2(); break;
            case 103: skills3(); break;
            case 104: skills4(); break;
            case 105: skills5(); break;
            case 106: skills6(); break;
            // add more skills cases as needed up to 106
        }
    }
    else {
        // Default auton
        auton1ring();
    }
}

void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.setPose(0,0,0);
    odomLift.set_value(0);
    
    pros::Task driveTask(drivetrain_control);
    pros::Task teleopTask(teleop_loop);
}
