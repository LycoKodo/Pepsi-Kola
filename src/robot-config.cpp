#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/smartMotor.hpp"
#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/device.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <sys/wait.h>

#include "colorsort.hpp"
// ---------------------------------------
// Drivetrain & Chassis
// ---------------------------------------


// CLAWBOT CONFIGS

//Gavin's Robot
pros::MotorGroup leftMotors({-11, -14, 15}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({1, 4, -3}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

//Quinn's Robot
// pros::MotorGroup leftMotors({-1, -3, 2}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
// pros::MotorGroup rightMotors({4, 6, -5}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// TODO - Change params for this
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.54, // 11.6 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 3"25' omnis
                              400, // drivetrain rpm is 200 (green direct)
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// ---------------------------------------
// Subsystem Configuration
// ---------------------------------------

pros::Controller master(pros::E_CONTROLLER_MASTER);

// pros::Motor top_intake(6, pros::MotorGearset::blue);
// pros::Motor front_intake(9, pros::MotorGearset::blue);
// pros::Motor middle_intake(4, pros::MotorGearset::blue);
// pros::Motor back_intake(2, pros::MotorGearset::blue);


//Gavin's Robot Config for Intake
pros::MotorGroup intake({17, 9, 19}, pros::MotorGearset::green);
pros::Motor ramp(9, pros::MotorGearset::blue);
pros::Motor top1(-17, pros::MotorGearset::green);
pros::Motor top2(19, pros::MotorGearset::green);  

//Quinn's Robot Config for Intake
// pros::MotorGroup intake({2, 6, 7, 18}, pros::MotorGearset::green);
// pros::Motor intake1(2, pros::MotorGearset::green);
// pros::Motor intake2(6, pros::MotorGearset::green);
// pros::Motor intake3(7, pros::MotorGearset::green);
// pros::Motor intake4(18, pros::MotorGearset::green);

pros::adi::DigitalOut heaven(1);
pros::adi::DigitalOut tongue(6);
pros::adi::DigitalOut descore(8);
pros::adi::DigitalOut odomLift(7);

// pros::adi::DigitalOut doinker (7);
// pros::adi::DigitalOut endgame (1);

// ---------------------------------------
// Sensors (Miscellaneous)
// ---------------------------------------

// pros::Optical color_sort_optical(15); // TODO - Change port

// macros::ColorSort colorsort = macros::ColorSort();

// ---------------------------------------
// Odometry
// ---------------------------------------

pros::Imu imu(6);

// TODO - check ports

pros::Rotation horizontal_encoder(-5); // Change to the "A" tagged encoder

pros::Rotation vertical_encoder(13);


lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, 3.348, -3.7);

lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, 2.025, +0.05);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_tracking_wheel, // &horizontal_tracking_wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu     // inertial sensor &imu
);

// ---------------------------------------
// PID Controller
// ---------------------------------------

lemlib::ControllerSettings linearController(  14.25, // proportional gain (kP) // k_cr = 48
                                              0, // integral gain (kI) 0.0035
                                              110, // derivative gain (kD) 13.5
                                              0, // anti windup
                                              1, // small error range, in inches 1
                                              2000, // small error range timeout, in milliseconds 200
                                              3, // large error range, in inches 3 
                                              5000, // large error range timeout, in milliseconds 500
                                              15 // maximum acceleration (slew) 75
);

lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI) .0029 was good
                                             10, // derivative gain (kD)
                                             0, // anti windup
                                             1, // small error range, in degrees 1
                                             200, // small error range timeout, in milliseconds (200)
                                             3, // large error range, in degrees 2
                                             500, // large error range timeout, in milliseconds (500)
                                             0 // maximum acceleration (slew)
); 

// ---------------------------------------
// ExpoDrive Constraints
// ---------------------------------------

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(0, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1 // expo curve gain 1.019 (LOG 0.9991)
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(0, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.010 // expo curve gain org 1.019
);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);