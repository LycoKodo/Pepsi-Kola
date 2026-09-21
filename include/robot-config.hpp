#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/device.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

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

extern pros::Motor intakeTop;
extern pros::Motor intakeBottom;
extern pros::MotorGroup cascade;
extern pros::Motor rollerTurn;
extern pros::Motor rollerMain;
extern pros::adi::DigitalOut toggleLift;
extern pros::adi::DigitalOut intakeLift;
extern pros::adi::DigitalOut sideToggle;

// ---------------------------------------
// Odometry
// ---------------------------------------
extern pros::Imu imu;
extern lemlib::OdomSensors sensors;

// MCL distance sensors, one per robot face. See src/mcl.cpp for the mounting
// offsets that pair with these
extern pros::Distance distFront;
extern pros::Distance distRight;
extern pros::Distance distBack;
extern pros::Distance distLeft;

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
extern lemlib::Chassis chassis;

#endif // ROBOT_CONFIG_HPP
