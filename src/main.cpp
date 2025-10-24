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
// AND
/*                                                                                                                                    
                                                                                                                                             
                                              -                                                                              
                                             --                                                                              
                                            --.-                                                                             
                                           -- .:-                                                                            
                                          -- ...:-                                                                           
                                          - .....--                                                                          
                                        -:...... --                                                                         
                                        -  .  ... --                                                                        
                                      -   ...  ..   -                                                                        
                                      -:   ....  ..  -                                                                       
                                     -- .. ....  . . .-                                                             
                                    :- . .  ... ......--                                                              
                                    -... . .... ... .. --                                                            
                                  --:..... ... .. ..... --                                                      
                                  -.....  .... ......... --                                                                 
                                 -- ..   ....... ....  . --                                                                  
                                -- ..   ....  ............:-                                                                 
                               .- .. ..  ....  .... .......--                                                                
                               -.... . ..... ....  . .......--                                                               
                              -..... ............... .... .. --                                                              
                             -:....... ..... ........... .... --                                                             
                            -- .... ....... . ................ -                                                             
                           --  ........ ................. ..... -                                                            
                          ----------------------------------------                                                                                                                                   
*/

// ----------------------------------------------------
// IMPORTANT - Autonomous Selector Routine Declaration
// ----------------------------------------------------

LV_IMG_DECLARE(image); //TKSRC Logo

// TODO - Temporary
// void autonwatcher() {
//     while (true) {
//         printf("Selected Auton: %i\n", lemlib::selector::auton);
//         pros::delay(10);
//     }
// }


void initialize() {
    // ------------------------------------------
    // Displaying TKSRC Logo (During calibration)
    // ------------------------------------------
    // lv_obj_t *img = lv_img_create(lv_scr_act()); // Add to screen
    // lv_img_set_src(img, &image); // Link to source image
    // lv_obj_align(img, LV_ALIGN_CENTER, 0, -20);

    // colorsort.initialise('N');

    // -----------------------
    // Initialise Chassis
    // -----------------------
    // @removed - LLEMU CANNOT BE USED WITH LVGL | pros::lcd::initialize(); // initialize brain screen

    chassis.calibrate(); // calibrate sensors

    // -----------------------
    // Initialise Sensors
    // -----------------------
    // colorSort.set_led_pwm(100);
    // lady_rotation.set_position(0);

    // colorsort.initialise(); // auto-detect alliance

    // lv_obj_del(img); // stop displaying TKSRC Logo as Calibration has ended

    // -----------------------
    // TODO - LVGL INIT
    // -----------------------


    static const char* qualBtnmMap[] = {
        "Left", "Right", "Q-Auton3", "\n",
        "Q-Auton4", "Q-Auton5", "Q-Auton6", nullptr
    };
    static const char* qualBtnmDesc[] = {
        "Standard 4 Block for LEFT, scoring MID", "Standard 4 Block for RIGHT, scoring LOW", "Q-Auton3",
        "Q-Auton4", "Q-Auton5", "Q-Auton6", nullptr
    };

    static const char* AIBtnmMap[] = {
        "AI-Auton1", "AI-Auton2", "AI-Auton3", "\n",
        "AI-Auton4", "AI-Auton5", "AI-Auton6", nullptr
    };
    static const char* AIBtnmDesc[] = {
        "AI-Auton1", "AI-Auton2", "AI-Auton3",
        "AI-Auton4", "AI-Auton5", "AI-Auton6", nullptr
    };

    static const char* skillsBtnmMap[] = {
        "Auton", "Preload", "Skills 1", "\n",  
        "Skills 2", "Skills 3", "Skills 4", nullptr
    };
    static const char* skillsBtnmDesc[] = {
        "Auton", "Preload", "Skills 1",  
        "Skills 2", "Skills 3", "Skills 4", nullptr
    };

    lemlib::selector::init(999, qualBtnmMap, AIBtnmMap, skillsBtnmMap, qualBtnmDesc, AIBtnmDesc, skillsBtnmDesc); // declaring default auton

    // TODO - temporary 
    // pros::Task watcher(autonwatcher);

}


void disabled() {} // @note -> Potentially for selecting clsrt (dont trust me bro fr)


void competition_initialize() {}


void autonomous() {  

    // @NOTE - UNCOMMENT THE FOLLOWING TO  ENABLE AUTON SELECTOR
    // int auton = lemlib::selector::auton;

    // lemlib::selector::autonStarted = true;
    
    // if (auton >= 1 && auton <= 100) {
    //     switch (auton) {
    //         case 1: qual1(); break;
    //         case 2: qual2(); break;
    //         case 3: qual3(); break;
    //         case 4: qual4(); break;
    //         case 5: qual5(); break;
    //         case 6: qual6(); break;
    //         // add more red cases as needed up to 100
    //     }
    // }
    // else if (auton >= -100 && auton <= -1) {
    //     switch (auton) {
    //         case -1: AI1(); break;
    //         case -2: AI2(); break;
    //         case -3: AI3(); break;
    //         case -4: AI4(); break;
    //         case -5: AI5(); break;
    //         case -6: AI6(); break;
    //         // add more blue cases as needed up to -100
    //     }
    // }
    // else if (auton >= 101 && auton <= 201) {
    //     switch (auton) {
    //         case 101: skills1(); break;
    //         case 102: skills2(); break;
    //         case 103: skills3(); break;
    //         case 104: skills4(); break;
    //         case 105: skills5(); break;
    //         case 106: skills6(); break;
    //         // add more skills cases as needed up to 106
    //     }
    // }
    // else {
    //     // Default auton
    //     auton1ring();
    // }
}



// TODO - Problem Lies in Controls
void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // DON'T CHANGE!: Multi-treading for robot controls (To prevent color sort interruption)

    
    pros::Task driveTask(drivetrain_control);
    pros::Task teleopTask(teleop_loop);
    // pros::Task intakeTask([] {
    //     colorsort.sorted_intake_loop();
    // }); // use a lambda
}
