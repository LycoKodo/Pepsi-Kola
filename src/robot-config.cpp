#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/smartMotor.hpp"
#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/device.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
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
                              12.598, // 320mm track width from the VEXcode SmartDrive config
                              lemlib::Omniwheel::NEW_275, // 219.44mm wheel travel / pi = 2.75"
                              450, // wheel rpm. NOTE: the VEXcode config says externalGearRatio 1,
                                   // which would make this 600. Left at the team's 450 -- measure a
                                   // known distance and correct whichever is wrong
                              2 // horizontal drift: 2 for all-omni, 8 if the drive has traction wheels
);

// ---------------------------------------
// Subsystem Configuration
// ---------------------------------------

pros::Controller master(pros::E_CONTROLLER_MASTER);

// ports, cartridges and directions from the VEXcode config. RATIO_18_1 is the
// green cartridge; a leading '-' is VEXcode's `True` reversed flag
pros::Motor intakeTop(-15, pros::MotorGearset::green);
pros::Motor intakeBottom(1, pros::MotorGearset::green);
pros::MotorGroup cascade({10, -14}, pros::MotorGearset::green);
pros::Motor rollerTurn(-5, pros::MotorGearset::green);
pros::Motor rollerMain(4, pros::MotorGearset::green);

// three-wire solenoids. only intakeLift is bound to a button -- toggleLift and
// sideToggle are declared in the VEXcode config but never actuated by it
pros::adi::DigitalOut toggleLift('A', LOW);
pros::adi::DigitalOut intakeLift('B', LOW);
pros::adi::DigitalOut sideToggle('C', LOW);

// ---------------------------------------
// Odometry
// ---------------------------------------

pros::Imu imu(12);

pros::Rotation verticalEncoder(11);

lemlib::TrackingWheel vertical_tracking_wheel(&verticalEncoder, lemlib::Omniwheel::NEW_2, +1.00);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel
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