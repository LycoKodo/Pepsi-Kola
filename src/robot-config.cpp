#include <cmath>
#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/gps.hpp"
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
pros::Distance clawSensor(ports::CLAW_DISTANCE); // L1: closes the claw when something is within 10 cm (CLAW_CATCH_MM, controls.cpp)

// ---------------------------------------
// Odometry
// ---------------------------------------

pros::Imu imu(ports::IMU);

pros::Rotation verticalEncoder(ports::VERTICAL_ODOM);
// Reversed (negative port) on 2026-11-23: at the robot, pushing it right with Theta 0 made X go DOWN
// (-25.74 for one tile), twice, after a restart. The earlier "pushing right raises X" check in ports.hpp
// no longer holds. This likely also explains the positive horizontal [ODOMCAL] value noted below.
pros::Rotation horizontalEncoder(-ports::HORIZONTAL_ODOM);

// 2" omnis. Offset = distance from the tracking centre, in inches, signed the
// LemLib way: vertical wheel + right / - left, horizontal wheel + front / - back
// Both offsets were measured with a ruler by the team on 2026-11-20, from the centre of the robot.
// (Before that: 0.9" and 2.55", the old robot's values.)
// The PID tab's [ODOMCAL] lines measure something different: where each wheel sits relative to the point
// the robot actually spins about in a turn. On 2026-11-20 they read about -0.45" (vertical; +0.1 clockwise,
// -1.35 counter-clockwise) and +3.6" (horizontal). The spin point is not the robot's centre and it moves
// with the turn direction, so the ruler values are the ones to use.
// The team confirmed the sides, looking down on the robot: the vertical wheel is LEFT of the centre
// (negative offset) and the horizontal wheel is BEHIND it (negative offset).
// RESOLVED 2026-11-23: the horizontal sensor counted the wrong way (it is now reversed, see horizontalEncoder),
// which is why [ODOMCAL] read +3.6" and seemed to put the spin point 5.8" behind the centre. With the sensor
// reversed and 2.0" wheels, a hand half-turn about the centre read X 0.04, Y 0.19 at 180 deg (it read
// Y -11.94 before), so the ruler offsets below are right. [ODOMCAL] values recorded before this date have the
// horizontal sign flipped
// Effective tracking wheel diameter (double-stacked VEX 2" omnis). LemLib's NEW_2 assumes 2.125". On
// 2026-11-23, hand pushes along the tiles read 25.43 / 25.5 in per tile forward (50.93 / 51 for two tiles) and
// 25.74 sideways: about 6% long if a tile is 24", which is what a 2.000" wheel gives (2.125 / 2.0 = 1.0625).
// Then 2.0" was used until the tape push the same day: three 48" hand pushes along a tape measure (Y-button
// marks in calib.log, raw sensor degrees) gave 1.9781, 1.9830 and 1.9850" for the vertical wheel; mean 1.982.
// Two more forward pushes later agreed (1.9866, 1.9774). The horizontal wheel: sideways pushes of 24" (read
// as 24, from the odometry's 23.6 / 24.0) gave 1.9921, 1.9786 and 1.9890"; mean 1.987. A fourth (2.0100) was
// left out because that push started from the boot position, not from a zero at the tape mark.
constexpr float VERTICAL_WHEEL_DIAMETER = 1.982f;
constexpr float HORIZONTAL_WHEEL_DIAMETER = 1.987f;
// TRACKING CENTRE = THE SPIN POINT (2026-11-23). The ruler offsets (-0.6, -2.23) are from the robot's geometric
// centre, but the robot spins about a point 1.55" AHEAD of that centre, so every 90 deg turn swung the tracked
// position about 2" and the next drive started off its line (UMBmark squares ended 2-3" from the start mark
// with the odometry itself right to about 0.5"). Measured from [UMBLEG] lines, the pose before and after each of
// 8 turns: forward 1.56/1.43/1.70/1.59 (clockwise) and 1.59/1.49/1.52/1.52 (counter-clockwise); sideways -0.57
// (clockwise) and +0.21 (counter-clockwise), mean -0.18. The offsets below are the wheels' positions from that
// spin point, so a turn on the spot leaves the pose where it is. What is left is the sideways part that differs
// with the turn direction, about +-0.4". CONSEQUENCE: the pose is now the spin point, 1.55" ahead of the robot's
// centre and 0.18" left of it: start poses and targets in routines mean that point.
constexpr float SPIN_POINT_RIGHT_IN = -0.18f;   // spin point, right of the geometric centre
constexpr float SPIN_POINT_FORWARD_IN = 1.55f;  // spin point, ahead of the geometric centre
constexpr float VERTICAL_WHEEL_FROM_CENTRE_IN = -0.6f;    // ruler: 0.6" left of the geometric centre
constexpr float HORIZONTAL_WHEEL_FROM_CENTRE_IN = -2.23f; // ruler: 2.23" behind the geometric centre
lemlib::TrackingWheel verticalWheel(&verticalEncoder, VERTICAL_WHEEL_DIAMETER,
                                    VERTICAL_WHEEL_FROM_CENTRE_IN - SPIN_POINT_RIGHT_IN); // -0.42
lemlib::TrackingWheel horizontalWheel(&horizontalEncoder, HORIZONTAL_WHEEL_DIAMETER,
                                      HORIZONTAL_WHEEL_FROM_CENTRE_IN - SPIN_POINT_FORWARD_IN); // -3.78

lemlib::OdomSensors sensors(&verticalWheel, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2
                            &horizontalWheel, // horizontal tracking wheel, port 3 (was off: an empty port
                                              // reads garbage, which would wreck odometry)
                            nullptr, // horizontal tracking wheel 2
                            &imu     // inertial sensor
);

// ---------------------------------------
// GPS-aided odometry (docs/GPS_FUSION_DESIGN.md)
// ---------------------------------------

// Not read at all while GPS_FUSION_MODE (robot-config.hpp) is Off. Its own PROS offset is set to zero in
// initialize(): the filter removes the lever arm itself, using mountRightIn / mountForwardIn below
pros::Gps gps(ports::GPS);

// Every number below is the design's FIRST GUESS, still to be measured on the robot (the design's tables say
// how; "R1" to "R4" are its Revision 2 sections). Field coordinates: inches from the field centre, +x toward the
// 90 degree wall, +y toward the 0 degree wall. The order must match lemlib::gpsfusion::Config (gpsFusion.hpp)
const lemlib::gpsfusion::Config gpsFusionConfig {
    .mountRightIn = 0, // first guess, measure with a ruler: inches from the robot's centre to the sensor, + = right
    .mountForwardIn = 0, // first guess, measure with a ruler: same, + = forward. Then set GPS_MOUNT_MEASURED
    .mountYawDeg = 180, // first guess: the camera faces backward. Look at the robot
    .sigmaGpsIn = 1.5, // first guess: GPS noise (1 standard deviation). Log it standing still at several spots
    .kDist = 0.02f, // first guess: odometry doubt per inch driven. Measure the drift on a taped route
    .kTurn = 0.3f, // first guess: inches of doubt per radian turned. Same taped route
    .sigmaMaxIn = 5, // R1 (was 12): cap on the doubt, inches, so the gate never grows wider than relocMaxIn
    .latencyMs = 60, // first guess: how old a reading is. Compare GPS and odometry on a drive (under 580 ms)
    .maxSpeedInPerS = 6, // first guess: use the GPS only below this speed. Raise once the delay is measured
    .maxTurnRateDegPerS = 30, // first guess: and below this turn rate. Same
    .minPeriodMs = 100, // first guess: one reading used per 100 ms. Nothing to measure
    .staleMs = 200, // first guess: unchanged this long = frozen. Nothing to measure
    .minViewIn = 20, // first guess: camera closer than this to the wall it faces = dead zone. Log near a wall
    .headingGateDeg = 8, // first guess: GPS heading must agree with ours this well. Log it
    .maxSensorErrorIn = 0, // R4, check 5b, off: drop readings whose get_error() is above this (in). Stage 1: cover
                           // the camera and see whether get_error() rises before using it
    .gate = 2.5f, // R1 (was 3): drop readings more than 2.5 standard deviations off. Nothing to measure
    .slewInPerS = 2, // first guess: fastest a correction is fed in, and fed back out after a fault. Simulator
    .maxStepIn = 1, // R1: the most one reading can move the correction, inches. Nothing to measure
    // R1: total correction over this = fault, then back to plain odometry. Revision 3 (was 12): chosen on
    // 2026-11-22 from (12, 12, 2), (8, 8, 1), (6, 6, 1) for (this, relocMaxIn, relocMaxCount): the smallest whose
    // 150 healthy simulator runs fault at most once (all three: 0). Simulator
    .maxTotalCorrectionIn = 6,
    .startSamples = 5, // first guess: the start check averages 5 readings. Nothing to measure
    .startTolIn = 8, // first guess: and faults if they are more than 8 in from the start pose
    .startSigmaIn = 1, // R3: how well the team places the robot at the start, inches (1 standard deviation)
    .startHeadingSamples = 10, // R3: 8 of the first 10 readings failing the heading check = fault
    .relocSamples = 4, // R2 (was 20): shove recovery after 4 readings in a row fail the gate. Simulator
    .relocWindowMs = 2000, // R2: all 4 within 2 s of each other. Simulator
    .relocSpreadIn = 4.5f, // R2 (was 2): each within 4.5 in (3 x sigmaGpsIn) of their mean. Simulator
    // R2 (was 18, then 12): and their mean within 6 in of odometry (further: fault). Revision 3, same rule
    .relocMaxIn = 6,
    // R2 (was 2): at most 1 relocalisation per run; a second = fault. Revision 3, same rule
    .relocMaxCount = 1,
};

// Wall fixes from distance sensors (docs/WALL_FIX_DESIGN.md): no wall-facing distance sensor is fitted, so none is
// declared and the wall code never runs on the robot. A wall sensor would be declared here (its port in
// include/ports.hpp: 6 and 21 are free), with a lemlib::gpsfusion::WallSensorMount saying where it is and a
// lemlib::gpsfusion::WallConfig, and passed to lemlib::gpsfusion::configureWalls() in initialize() after
// configure(). The design's "Hardware and stages on the robot" lists the stages before trusting it.

// ---------------------------------------
// PID Controller
// ---------------------------------------

// The gains below are what the brain's PID tab starts from on every boot (its edits are not saved).
// Tuned on the new robot with 24 in and 6-48 in out-and-back drives: avg err -0.32", max overshoot 0.7"
lemlib::ControllerSettings linearController(  12.5, // proportional gain (kP). Rocks twice at 20.5, so 60% of it
                                              0.1, // integral gain (kI) (was 0; 0.1 found better on the PID tab, 2026-11-19)
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
                                             0.25, // integral gain (kI): pushes the last few degrees in (was 0.15: stopped 0.5-1.3 deg short)
                                             42, // derivative gain (kD) (was 36: overshot 1.8-5.2 deg at 90; 38 and 40 undershot a little; 35 was tried; 90 on the old robot)
                                             // 2026-11-19 PID tab, 90 deg: +90 ended 0.9 short / -90 0.4 short, no overshoot, settled 0.6-0.8 s
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
