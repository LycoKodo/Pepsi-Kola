#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/gpsFusion.hpp"
#include "pros/adi.hpp"
#include "pros/gps.hpp"

// Every device below is defined once in src/robot-config.cpp. Port numbers are
// in include/ports.hpp.

// ---------------------------------------
// Drivetrain & Chassis
// ---------------------------------------
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern lemlib::Drivetrain drivetrain;

// ---------------------------------------
// Subsystem Configuration
// ---------------------------------------
extern pros::Controller master;

extern pros::Motor toggleA; // port 10 (was intakeBottom), runs with intake()
extern pros::Motor toggleB; // port 17, spins on yellow or the other alliance's colour
extern pros::Motor rollerMain;
extern pros::Motor rollerTurn;
extern pros::MotorGroup cascade;

extern pros::adi::DigitalOut toggleLift;
extern pros::adi::DigitalOut intakeLift;
extern pros::adi::DigitalOut sideToggle;
extern pros::adi::DigitalOut pistonH;  // 3-wire H: the claw (single-acting: true = open), L1
extern pros::MotorGroup xMotors;       // ports 14 and 16: spin while X is held
extern pros::Distance distanceSensor; // hold-Y pickup trigger
extern pros::Optical opticalSensor;   // colour sensor for toggleB
extern pros::Distance clawSensor;     // the claw's distance sensor (L1)

// ---------------------------------------
// Odometry
// ---------------------------------------
extern pros::Imu imu;
extern pros::Rotation verticalEncoder;
extern pros::Rotation horizontalEncoder;
extern lemlib::TrackingWheel verticalWheel;
extern lemlib::TrackingWheel horizontalWheel;
extern lemlib::OdomSensors sensors;

// IMU heading scale (lemlib::setImuScale, applied in initialize()). 1.0 changes nothing. To measure: turn the
// robot ten full turns clockwise, divide 3600 by the degrees it reads; the same counter-clockwise
constexpr float IMU_SCALE_CLOCKWISE = 1.0f; // not measured yet
constexpr float IMU_SCALE_COUNTER_CLOCKWISE = 1.0f; // not measured yet

// ---------------------------------------
// GPS-aided odometry (docs/GPS_FUSION_DESIGN.md)
// ---------------------------------------
extern pros::Gps gps; // port 7

// What startGpsFusion() (src/main.cpp) switches fusion to when a routine starts. Off: the GPS is never read
// and odometry is exactly what it was without fusion. The design's stages on the robot: first Shadow (runs and
// prints [GPSFUSE] lines, changes nothing), then On for Skills routines once Shadow has proved it
constexpr lemlib::gpsfusion::Mode GPS_FUSION_MODE = lemlib::gpsfusion::Mode::Off;

// Mount guard (docs/GPS_FUSION_DESIGN.md, R4). Set to true only once gpsFusionConfig.mountRightIn and
// mountForwardIn have been measured on the robot (even if they really are 0). While it is false, asking for On
// gives Shadow instead (the library enforces it: configure() is told this in initialize()), and the [GPSFUSE]
// logging task says why: a camera 6 in off-centre with the mount left at 0 made fusion 4 in worse than odometry
// alone in the simulator
constexpr bool GPS_MOUNT_MEASURED = false;

// Odometry guard (docs/GPS_FUSION_DESIGN.md, "Revision 2 results"). Set to true only once odometry has been
// calibrated (stage 0): the IMU scale measured (IMU_SCALE_CLOCKWISE / IMU_SCALE_COUNTER_CLOCKWISE above) and the
// tracking wheel size checked with the tape test. While it is false, asking for On gives Shadow instead, like
// GPS_MOUNT_MEASURED: with the inertial sensor 1.2 percent wrong the filter fixes position but not the heading
// that keeps rebuilding the error, and in the simulator it missed its target in one spin direction; with a
// calibrated sensor that did not happen
constexpr bool ODOMETRY_CALIBRATED = false;

// Every number the filter uses (src/robot-config.cpp)
extern const lemlib::gpsfusion::Config gpsFusionConfig;

// ---------------------------------------
// PID Controller
// ---------------------------------------
extern lemlib::ControllerSettings linearController;
extern lemlib::ControllerSettings angularController;

// ---------------------------------------
// ExpoDrive Constraints
// ---------------------------------------
extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;
/**
 * LemLib's Chassis plus two team rules for turns:
 * - a turn under SMALL_TURN_DEG pushes at at least SMALL_TURN_MIN_SPEED and stops as soon as it
 *   reaches the target (LemLib's own minSpeed option), so small turns don't stop short
 * - a turn of BIG_TURN_DEG or more is capped at BIG_TURN_MAX_SPEED, so it doesn't build up the
 *   momentum that carries it past the target
 * A caller's own minSpeed / lower maxSpeed wins. Everything else is LemLib's, unchanged.
 * (Done here, not in src/lemlib: the stock LemLib in the cold image is what runs, edits there don't)
 */
class TeamChassis : public lemlib::Chassis {
    public:
        using lemlib::Chassis::Chassis;
        void turnToHeading(float theta, int timeout, lemlib::TurnToHeadingParams params = {}, bool async = true);
};

constexpr float SMALL_TURN_DEG = 45;        // turns smaller than this get the minimum speed
constexpr float SMALL_TURN_MIN_SPEED = 51; // out of 127: 40% (30% and 25% were tried; 50% overshot; 70% and about 94% too)
// turns this big or bigger get the speed cap. 100, not 90, so a 90 deg turn that starts slightly off
// (90.4) isn't capped
constexpr float BIG_TURN_DEG = 100;
constexpr float BIG_TURN_MAX_SPEED = 80; // out of 127: about 63% (90 still overshot a little)

extern TeamChassis chassis;

#endif // ROBOT_CONFIG_HPP
