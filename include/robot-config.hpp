#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"

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
