#include <cmath>
#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

#include "ports.hpp"
#include "robot-config.hpp"

// Every device is declared exactly once, here. Port numbers live in ports.hpp.

// ---------------------------------------
// Drivetrain & Chassis
// ---------------------------------------

// 4-motor drive, 2 per side, 600rpm (blue) cartridges. Left side is reversed so
// that positive power drives forward on both sides. If one side drives
// backward, flip both its signs in ports.hpp; if the two motors on a side fight
// each other (buzz without turning), flip just one of them
pros::MotorGroup leftMotors({ports::LEFT_A, ports::LEFT_B}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({ports::RIGHT_A, ports::RIGHT_B}, pros::MotorGearset::blue);

lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.61, // track width: 295mm, from the Blocks drivetrain config
                              lemlib::Omniwheel::NEW_275, // 2.75" omnis
                              450, // wheel rpm: 600rpm motors geared down to 450 (confirmed on the robot)
                              2 // horizontal drift: 2 for all-omni, 8 if the drive has traction wheels
);

// ---------------------------------------
// Subsystem Configuration
// ---------------------------------------

pros::Controller master(pros::E_CONTROLLER_MASTER);

// Encoder units are DEGREES, set explicitly (with no units given, PROS 4 keeps whatever
// mode the motor is already in, which was counts). Positions match the brain's Devices screen
pros::Motor toggleA(ports::TOGGLE_A, pros::MotorGearset::green, pros::MotorUnits::degrees); // was intakeBottom; spins with intake()
pros::Motor toggleB(ports::TOGGLE_B, pros::MotorGearset::green, pros::MotorUnits::degrees); // spins on yellow / other alliance's colour (startColourToggle)
pros::Motor rollerMain(ports::ROLLER_MAIN, pros::MotorGearset::green, pros::MotorUnits::degrees);
pros::Motor rollerTurn(ports::ROLLER_TURN, pros::MotorGearset::green, pros::MotorUnits::degrees);
pros::MotorGroup cascade({ports::CASCADE_A, ports::CASCADE_B}, pros::MotorGearset::green,
                         pros::MotorUnits::degrees);

pros::adi::DigitalOut toggleLift(ports::TOGGLE_LIFT);
pros::adi::DigitalOut intakeLift(ports::INTAKE_LIFT);
pros::adi::DigitalOut sideToggle(ports::SIDE_TOGGLE);
pros::adi::DigitalOut pistonH(ports::PISTON_H); // the claw, single-acting: true = open, false = closed
pros::MotorGroup xMotors({ports::X_MOTOR_A, ports::X_MOTOR_B}, pros::MotorGearset::green,
                         pros::MotorUnits::degrees); // ports 14 and 16, spin while X is held
pros::Distance distanceSensor(ports::DISTANCE);
pros::Optical opticalSensor(ports::OPTICAL);
pros::Distance clawSensor(ports::CLAW_DISTANCE); // L1: closes the claw when something is within 2 cm

// ---------------------------------------
// Odometry
// ---------------------------------------

pros::Imu imu(ports::IMU);

pros::Rotation verticalEncoder(ports::VERTICAL_ODOM);
pros::Rotation horizontalEncoder(ports::HORIZONTAL_ODOM);

// 2" omnis. Offset = distance from the tracking centre, in inches, signed the
// LemLib way: vertical wheel + right / - left, horizontal wheel + front / - back
lemlib::TrackingWheel verticalWheel(&verticalEncoder, lemlib::Omniwheel::NEW_2, 0.9);     // 0.9" right
lemlib::TrackingWheel horizontalWheel(&horizontalEncoder, lemlib::Omniwheel::NEW_2, 2.55); // 2.55" in front

lemlib::OdomSensors sensors(&verticalWheel, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2
                            nullptr, // horizontal tracking wheel: OFF, not plugged in on the new robot.
                                     // An empty port reads garbage, which would wreck odometry.
                                     // Plug it in, set its port in ports.hpp, then put &horizontalWheel back
                            nullptr, // horizontal tracking wheel 2
                            &imu     // inertial sensor
);

// ---------------------------------------
// PID Controller
// ---------------------------------------

// The gains below are what the brain's PID tab starts from on every boot (its edits are not saved).
// Tuned on the new robot with 24 in and 6-48 in out-and-back drives: avg err -0.32", max overshoot 0.7"
lemlib::ControllerSettings linearController(  12.5, // proportional gain (kP). Rocks twice at 20.5, so 60% of it
                                              0, // integral gain (kI)
                                              100, // derivative gain (kD) (was 170)
                                              // integral window, inches: the integral only builds while the
                                              // error is INSIDE this range (LemLib's own comment says the
                                              // opposite; pid.cpp is the truth). 0 = always builds, which
                                              // with kI > 0 winds up over the whole drive and overshoots
                                              3,
                                              0.5, // small error range, in inches (was 1: a drive that slid 0.75" past still counted as done)
                                              200, // small error range timeout, ms (was 2000: every drive ran to its timeout)
                                              3, // large error range, in inches
                                              500, // large error range timeout, ms (was 5000)
                                              15 // maximum acceleration (slew) 75
);

// Turning. The PID tab's TURN mode edits kP/kI/kD here. (A separate set for turns under 45 deg was tried and
// dropped; a second retune tried kP 8.4 / kD 78-95)
lemlib::ControllerSettings angularController(4.8, // proportional gain (kP). Wobbles at 7.5 (was 15.8)
                                             0.15, // integral gain (kI): pushes the last few degrees in
                                             36, // derivative gain (kD) (38 and 40 undershot a little; 35 was tried; 90 on the old robot)
                                             5, // integral window, degrees: kI only builds within 5 deg
                                             1, // small error range, in degrees 1
                                             200, // small error range timeout, in milliseconds (200)
                                             3, // large error range, in degrees 2
                                             500, // large error range timeout, in milliseconds (500)
                                             15 // maximum acceleration (slew): power may change by 15 per 10 ms
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

TeamChassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

void TeamChassis::turnToHeading(float theta, int timeout, lemlib::TurnToHeadingParams params, bool async) {
    // small turn, and the caller didn't set a minimum speed of its own: push hard all the way.
    // LemLib ends a turn with a minimum speed as soon as it crosses the target heading
    const float turnSize = std::fabs(lemlib::angleError(theta, getPose().theta, false));
    if (turnSize < SMALL_TURN_DEG && params.minSpeed == 0) params.minSpeed = SMALL_TURN_MIN_SPEED;
    // big turn: cap the top speed so it doesn't overshoot (a caller's lower maxSpeed stays)
    if (turnSize >= BIG_TURN_DEG && params.maxSpeed > BIG_TURN_MAX_SPEED) params.maxSpeed = BIG_TURN_MAX_SPEED;
    lemlib::Chassis::turnToHeading(theta, timeout, params, async);
}
