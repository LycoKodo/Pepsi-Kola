#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/odom.hpp" // lemlib::setImuScale
#include "lemlib/chassis/gpsFusion.hpp"

#include "lemlib/selector.hpp" // For auton selector
#include "robot-config.hpp"
#include "ports.hpp"
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

static bool imuOkAtBoot = false; // set once in initialize(), right after chassis.calibrate()
bool imuCalibratedAtBoot() { return imuOkAtBoot; }

void initialize() {
    std::printf("[build] Yoake V0.1, small-upload test 1\n"); // shows which upload is running; bump the number to check a new one
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
    // the IMU scale is set before calibrate(), which starts the odometry task that reads it
    lemlib::setImuScale(IMU_SCALE_CLOCKWISE, IMU_SCALE_COUNTER_CLOCKWISE); // 1.0 and 1.0 until measured
    // The horizontal tracking sensor counts the wrong way and is reversed in the sensor itself. Its constructor
    // (robot-config.cpp, negative port) asks for that once, before initialize(), and nothing checks it took:
    // if the sensor was not ready then, X counts backwards for the whole run and every move steers the wrong
    // way. Ask again here, now that PROS is up, check it, and say so if it did not take (2026-11-24)
    horizontalEncoder.set_reversed(true);
    pros::delay(20);
    if (horizontalEncoder.get_reversed() != 1) {
        std::printf("[odom] horizontal tracking sensor did NOT take the reversed setting: X will count backwards. "
                    "Check its cable and port, then restart the program\n");
        master.set_text(1, 0, "ODOM X SENSOR BAD ");
        master.rumble("---");
    }
    chassis.calibrate(); // calibrate sensors
    // If the IMU did not calibrate (unplugged, wrong port, robot moved during calibration) LemLib carries
    // on without it and every heading is then a rough estimate from the wheels. Say so loudly. The odometry
    // test (src/umbmark.cpp) remembers it and refuses to run
    imuOkAtBoot = std::isfinite(imu.get_rotation());
    if (!imuOkAtBoot) {
        std::printf("[boot] IMU FAILED TO CALIBRATE: headings will be wrong. Check port %d, keep the robot still, restart\n",
                    ports::IMU);
        pros::delay(60); // the controller takes one message per 50 ms, and calibrate() just sent one
        master.set_text(0, 0, "IMU FAILED       ");
        pros::delay(60);
        master.rumble("---");
    } else {
        imuwatch::checkAtBoot(); // heading creeping while standing still: calibrate once more, or say so
    }
    imuwatch::start();

    // -----------------------
    // GPS-aided odometry (docs/GPS_FUSION_DESIGN.md)
    // -----------------------
    // Fusion stays Off (the GPS is not even read) until a routine calls startGpsFusion()
    // the filter removes the lever arm itself (gpsFusionConfig.mountRightIn/ForwardIn). If the sensor did not
    // take the zero offset (unplugged, another device on the port), say so once: the lever-arm maths assumes it
    if (gps.set_offset(0, 0) == PROS_ERR) {
        std::printf("[GPSFUSE] gps.set_offset(0, 0) failed on port %d (errno %d): check the GPS sensor\n", ports::GPS,
                    errno);
    }
    // the On guards go to the library, so nothing can walk round them (On needs both; robot-config.hpp). This also
    // starts the task that prints the [GPSFUSE] lines and checks every number in gpsFusionConfig
    lemlib::gpsfusion::configure(&gps, gpsFusionConfig, GPS_MOUNT_MEASURED, ODOMETRY_CALIBRATED);

    // -----------------------
    // Initialise Mechanisms
    // -----------------------
    // mechanisms are zeroed where they sit now: power on with the cascade down
    // and the roller stowed
    subsystems::init();
    // while a routine is picked and none has run yet, the colour toggle leaves toggleB alone (2026-11-20)
    subsystems::holdColourToggleUntilAuton([] { return routineExists(lemlib::selector::auton); });
    subsystems::startColourToggle();
    macros::init();

    lv_obj_del(img); // stop displaying TKSRC Logo as Calibration has ended

    // -----------------------
    // LVGL INIT
    // -----------------------

    // Routine numbers: qual 1, 2, 3...  tests (the old AI tab) -1, -2...  skills 101, 102...
    // (see the button handlers in src/lemlib/selector.cpp for how a button maps to a number).
    // Every tab needs at least one button (LVGL crashes on an empty map), so Qual and AI
    // have a "NONE" placeholder that runAuton() treats as "nothing selected"
    static const char* qualBtnmMap[] = {"NONE", "james_67", nullptr};
    static const char* qualBtnmDesc[] = {"No routine", "james_67: spins toggleB until it sees our alliance colour", nullptr};

    // The Tests tab (the old AI tab, numbers -1, -2...): test runs only, never in a match. Button names must
    // differ (the handler finds the button by its text).
    // UMB CW / UMB CCW, the odometry test (src/umbmark.cpp): the robot's centre drives a 48 in (two tile) square;
    // an 18 in robot's corners swing out about 13 in further when it turns, at every corner including the start.
    // So: about 61 in clear ahead of and beside the start (3 tiles past the robot's own tile leaves room), and
    // about 13 in behind and on the other side (1 tile).
    // IMU TEST, the inertial sensor test (src/imutest.cpp): nothing drives
    static const char* AIBtnmMap[] = {"UMB CW", "UMB CCW", "IMU TEST", nullptr};
    static const char* AIBtnmDesc[] = {
        "Odometry test, clockwise 2 tile square. Robot centred on a tile, lift down. Keep clear: 3 tiles "
        "ahead, 3 RIGHT, 1 behind, 1 left. Hold A, press B. A stick stops it",
        "Odometry test, counter-clockwise 2 tile square. Robot centred on a tile, lift down. Keep clear: "
        "3 tiles ahead, 3 LEFT, 1 behind, 1 right. Hold A, press B. A stick stops it",
        "Inertial sensor test, 2.5 min, nothing drives. Robot on the floor. Hold A, press B, then do what "
        "the controller says. A stick stops it",
        nullptr};

    // Every tab needs at least one button (LVGL crashes on an empty map): Skills has none yet
    static const char* skillsBtnmMap[] = {"NONE", nullptr};
    static const char* skillsBtnmDesc[] = {"No routines yet", nullptr};

    constexpr int DEFAULT_AUTON = 2; // james_67 (Qual tab, second button); 999 = none
    lemlib::selector::init(DEFAULT_AUTON, qualBtnmMap, AIBtnmMap, skillsBtnmMap, qualBtnmDesc, AIBtnmDesc, skillsBtnmDesc); // declaring default auton
    pidtuner::init(lemlib::selector::tabview); // PID tuning tab, added after the selector's tabs
}

// a macro started at the end of one period must not carry on into the next.
// Cancel it, give its task a moment to exit, then stop every mechanism so no
// motor is left chasing a move_absolute target
static void stopMechanisms() {
    macros::cancel();
    for (int t = 0; t < 100 && macros::isBusy(); t += 10) pros::delay(10);
    subsystems::stopAll();
    // an auton cut off inside spinToAllianceColour leaves the colour toggle paused: give toggleB back to it
    subsystems::pauseColourToggle(false);
}

void disabled() {
    stopMechanisms();
    lemlib::gpsfusion::setMode(lemlib::gpsfusion::Mode::Off);
    // Pick the routine on the controller while the robot is disabled (LEFT / RIGHT). Shelved: see
    // ctrlselect::ENABLED (autons.hpp). When on, this loop never returns: PROS deletes the disabled task when
    // the mode changes
    if (ctrlselect::ENABLED) ctrlselect::disabledLoop();
}

void competition_initialize() {}

void startGpsFusion() {
    // GPS_FUSION_MODE is Off until the team changes it in robot-config.hpp: then this does nothing at all. The
    // mount and odometry guards are in the library now (configure() in initialize() was told GPS_MOUNT_MEASURED
    // and ODOMETRY_CALIBRATED): asking for On before both are true gives Shadow, and the [GPSFUSE] logging task
    // says which one is missing. Nothing here prints: a routine's task can be deleted in the middle of a print
    lemlib::gpsfusion::setMode(GPS_FUSION_MODE);
    if (ctrlselect::ENABLED) ctrlselect::showNow(); // the controller shows which routine is picked
}

bool routineExists(int routine) {
    // keep in step with the cases in runAuton(). The test runs (-1, -2, -3) are that never touch
    // toggleB, so they do not hold the colour toggle
    return routine == 2;
}

void runAuton(int routine) {
    subsystems::colourToggleAutonStarted(); // the colour toggle may run normally from now on
    // one case per routine: 1..100 qual, -1..-100 tests, 101.. skills, 999 none.
    // The placeholder buttons ("NONE": 1, -1) land in default
    switch (routine) {
        case 2: james_67(); break;              // Qual tab, second button
        // the Tests tab (negative numbers): test runs, each refuses under field control
        case -1: umbmarkSquare(true); break;  // "UMB CW": odometry test, clockwise square
        case -2: umbmarkSquare(false); break; // "UMB CCW": the same, counter-clockwise
        case -3: // "IMU TEST": inertial sensor test, nothing drives
            imuwatch::pause(true);
            imuTest();
            imuwatch::pause(false);
            break;
        // nothing selected (999) or a placeholder: do nothing
        default: std::printf("[auton] no routine selected (%d)\n", routine); break;
    }
    // GPS fusion is never used while the driver drives, and a test run (hold A, press B) returns straight to
    // driver control, so switch it Off whenever a routine ends
    lemlib::gpsfusion::setMode(lemlib::gpsfusion::Mode::Off);
}

void autonomous() {
    stopMechanisms();
    lemlib::selector::autonStarted = true;
    runAuton(lemlib::selector::auton);
}

void opcontrol() {
    // GPS fusion is for routines only, never while the driver drives (docs/GPS_FUSION_DESIGN.md)
    lemlib::gpsfusion::setMode(lemlib::gpsfusion::Mode::Off);
    // the loop runs right here, in the opcontrol task, rather than in tasks it
    // spawns: PROS kills this task when the mode changes, but not its children,
    // so spawned loops would keep driving into the next auton/disabled period
    driverControl();
}
