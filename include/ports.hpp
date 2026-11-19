#ifndef PORTS_HPP
#define PORTS_HPP

#include <array>
#include <cstdint>

/**
 * @file ports.hpp
 * @brief The robot's wiring map. Every port number in the program comes from here.
 *
 * Source of truth: the team's VEXcode Blocks project ("Tank copy.v5blocks"),
 * except where marked. If you re-wire the robot, change it here and nowhere else.
 *
 * NEW ROBOT: drive left 1, 2 and right 11, 12, lift (cascade) on 8 and 9, intake
 * (toggle A) on 10, IMU on 13, toggle B on 17 (spins when the optical sensor on
 * 18 sees yellow or the other alliance's colour), horizontal odom on 3. Port 19
 * below is a PLACEHOLDER for the distance sensor until its real port on the new
 * robot is known. The rest are the old robot's.
 *
 *   SMART PORTS                                  3-WIRE (brain ADI)
 *   ----------------------------------------     ------------------------
 *    1  drive left A      motor, blue,  rev
 *    2  drive left B      motor, blue,  rev
 *    3  horizontal odom   rotation sensor  (was placeholder 6)
 *    4  rollerMain        motor, green           A  toggleLift   (unused)
 *    5  rollerTurn        motor, green, rev      B  intakeLift   (unused)
 *                                                C  sideToggle   (no button now)
 *                                                H  pistonH      claw: L1 (see controls.cpp)
 *    7  gps               GPS sensor       (not used in code yet; for the planned Kalman filter)
 *    8  cascade A        motor, green       (was 10)
 *    9  cascade B         motor, green, rev  (was 14)
 *   10  toggle A          motor, green, rev  (was intakeBottom, port 1)
 *   11  drive right A     motor, blue
 *   12  drive right B     motor, blue
 *   13  imu               inertial sensor
 *   14  x motor A         motor, green, rev  (X held: spins with 16)
 *   15  claw distance     distance sensor  (L1 closes the claw at 10 cm)
 *   16  x motor B         motor, green       (X held: spins with 14)
 *   17  toggle B          motor, green       (spins when 18 sees yellow/other alliance)
 *   18  optical           colour sensor    (drives toggle B)
 *   19  distance          distance sensor  (PLACEHOLDER; was 17. Hold-Y pickup)
 *   20  vertical odom     rotation sensor  (was 11)
 *
 * Port 15 was IntakeTop (green, reversed) in the Blocks project; on the new robot it is the
 * claw's sensor.
 *
 * A negative motor port means "reversed", which is the PROS convention.
 */
namespace ports {

// drivetrain
// new robot: 1, 2 drive the left side and 11, 12 the right (as wired, seen from behind the
// robot). 11, 12 forward / 1, 2 reversed drove straight right but steered backwards; all flipped,
// it steered right but drove backwards. Both mean the sides were swapped
constexpr std::int8_t LEFT_A = -1;
constexpr std::int8_t LEFT_B = -2;
constexpr std::int8_t RIGHT_A = 11;
constexpr std::int8_t RIGHT_B = 12;

// mechanisms
constexpr std::int8_t TOGGLE_A = -10; // was intakeBottom. Reversed vs the Blocks project
constexpr std::int8_t TOGGLE_B = 17; // spins when OPTICAL sees yellow or the other alliance's colour. Not reversed (was the wrong way reversed)
constexpr std::int8_t ROLLER_MAIN = 4;
constexpr std::int8_t ROLLER_TURN = -5;
constexpr std::int8_t CASCADE_A = 8;
constexpr std::int8_t CASCADE_B = -9;
constexpr std::int8_t X_MOTOR_A = -14; // X held: spins, with X_MOTOR_B. Reversed
constexpr std::int8_t X_MOTOR_B = 16;

// sensors
constexpr std::uint8_t VERTICAL_ODOM = 20; // new robot (was 11)
constexpr std::uint8_t IMU = 13;
constexpr std::uint8_t HORIZONTAL_ODOM = 3;  // new robot: rolls sideways, 2" omni (was placeholder 6, switched off). Checked: pushing right raises X
constexpr std::uint8_t DISTANCE = 19; // PLACEHOLDER (was 17, now toggle B)
constexpr std::uint8_t OPTICAL = 18;  // colour sensor for TOGGLE_B
constexpr std::uint8_t CLAW_DISTANCE = 15; // the claw's distance sensor
constexpr std::uint8_t GPS = 7; // GPS sensor, plugged in; not read by the code yet (for the planned Kalman filter)

// 3-wire ports on the brain
constexpr char TOGGLE_LIFT = 'A';
constexpr char INTAKE_LIFT = 'B';
constexpr char SIDE_TOGGLE = 'C';
constexpr char PISTON_H = 'H'; // the claw: L1 opens it, closes on CLAW_DISTANCE

// ---------------------------------------------------------------------------
// Compile-time wiring check. Two devices on one smart port is the bug that
// bit this codebase twice (IMU and cascade both on 14, a rotation sensor and
// a motor both on 15). Every smart device must be listed here; a duplicate
// fails the build instead of failing on the field.
// ---------------------------------------------------------------------------
constexpr int number(int port) { return port < 0 ? -port : port; }

constexpr std::array<int, 19> SMART = {
    number(LEFT_A), number(LEFT_B), number(RIGHT_A), number(RIGHT_B),
    number(TOGGLE_A), number(TOGGLE_B), number(ROLLER_MAIN), number(ROLLER_TURN),
    number(CASCADE_A), number(CASCADE_B), number(X_MOTOR_A), number(X_MOTOR_B),
    VERTICAL_ODOM, IMU, HORIZONTAL_ODOM, DISTANCE, OPTICAL, CLAW_DISTANCE, GPS,
};

constexpr bool allValidAndUnique() {
    for (std::size_t i = 0; i < SMART.size(); i++) {
        if (SMART[i] < 1 || SMART[i] > 21) return false;
        for (std::size_t j = i + 1; j < SMART.size(); j++) {
            if (SMART[i] == SMART[j]) return false;
        }
    }
    return true;
}

static_assert(allValidAndUnique(), "ports.hpp: a smart port is out of range (1-21) or used twice");

} // namespace ports

#endif // PORTS_HPP
