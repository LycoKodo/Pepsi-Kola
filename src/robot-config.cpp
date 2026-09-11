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

// ---------------------------------------
// Drivetrain & Chassis
// ---------------------------------------


// New season robot: 4-motor drive, 2 per side, 600rpm (blue) cartridges.
// Left side is reversed so that positive power drives forward on both sides.
// If one side drives backward, flip both its signs; if the two motors on a
// side fight each other (buzz without turning), flip just one of them
pros::MotorGroup leftMotors({-6, -7}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({8, 9}, pros::MotorGearset::blue);

lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.54, // TODO - measure on the new robot: centre of left wheels to centre of right wheels, in inches
                              lemlib::Omniwheel::NEW_275, // 2.75" omnis
                              450, // wheel rpm: 600rpm motors geared down to 450
                              2 // horizontal drift: 2 for all-omni, 8 if the drive has traction wheels
);

// ---------------------------------------
// Subsystem Configuration
// ---------------------------------------

pros::Controller master(pros::E_CONTROLLER_MASTER);

// ports, cartridges and directions from the VEXcode config (TestingProgram1)
pros::Motor intakeTop(16, pros::MotorGearset::green);
pros::Motor intakeBottom(15, pros::MotorGearset::green);
pros::MotorGroup cascade({10, -21}, pros::MotorGearset::green);
pros::Motor rollerTurn(5, pros::MotorGearset::green);
pros::Motor rollerMain(4, pros::MotorGearset::green);

// ---------------------------------------
// Odometry
// ---------------------------------------

pros::Imu imu(2);

// no tracking wheels yet. with both vertical slots empty LemLib tracks
// distance off the drive motor encoders, so pose drifts more than it would
// with dedicated wheels -- add them here when they are built
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2
                            &imu     // inertial sensor
);

// ---------------------------------------
// PID Controller
// ---------------------------------------

lemlib::ControllerSettings linearController(  53.93, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              225.83, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches 1
                                              2000, // small error range timeout, in milliseconds 200
                                              3, // large error range, in inches 3 
                                              5000, // large error range timeout, in milliseconds 500
                                              15 // maximum acceleration (slew) 75
);

lemlib::ControllerSettings angularController(2.37, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             14.53, // derivative gain (kD)
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