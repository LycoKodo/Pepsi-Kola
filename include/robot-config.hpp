#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

#include "colorsort.hpp"

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

<<<<<<< HEAD
extern pros::Motor top_intake;
extern pros::Motor front_intake;
extern pros::Motor middle_intake;
extern pros::Motor back_intake;
=======
extern pros::MotorGroup intake;
extern pros::Motor ramp;
extern pros::Motor top1;
extern pros::Motor top2;
>>>>>>> gavin-bot

extern pros::adi::DigitalOut tongue;
extern pros::adi::DigitalOut odomLift;
extern pros::adi::DigitalOut descore;
extern pros::adi::DigitalOut heaven;

// ---------------------------------------
// Sensors (Miscellaneous)
// ---------------------------------------
extern pros::Imu imu;
extern pros::Optical color_sort_optical;
<<<<<<< HEAD
extern pros::Optical preload_optical;
=======
>>>>>>> gavin-bot
extern macros::ColorSort colorsort;

// ---------------------------------------
// Macro
// ---------------------------------------


// ---------------------------------------
// Odometry
// ---------------------------------------
extern pros::Rotation horizontal_encoder;
extern pros::Rotation vertical_encoder;
extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::TrackingWheel vertical_tracking_wheel;
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
extern lemlib::Chassis chassis;

#endif // ROBOT_CONFIG_HPP
