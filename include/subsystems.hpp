#ifndef SUBSYSTEMS_HPP
#define SUBSYSTEMS_HPP

#include <functional>
#include "pros/abstract_motor.hpp"

/**
 * @file subsystems.hpp
 * @brief Mechanism commands and tuning constants, shared by teleop, macros and autons.
 *
 * Positions are in motor DEGREES (the same numbers the brain's Devices screen shows),
 * measured from where each mechanism sat when
 * the program started (init() zeroes the encoders), exactly like the Blocks
 * project's CascadeSet / CascadeUpperLimit variables. So: power the robot on
 * with the cascade fully down and the roller stowed.
 */
namespace subsystems {

// --- constants, carried over from the Blocks project -----------------------
constexpr double CASCADE_STEP = 383.2;      // degrees per R2 preset (presets 1..3)
constexpr int CASCADE_PRESETS = 3;
// R2's cascade heights, presets 1..3. Neutral goal and alliance goal (what it boots with) both set by hand.
// Holding A 1 s swaps them
constexpr double CASCADE_NEUTRAL[CASCADE_PRESETS] = {720, 1020, 1310}; // +120 (was 600, 900, 1190)
// alliance goal: R2 presses 1-5 on the new robot (was 490, 800, 1080 for presses 1-3)
constexpr int CASCADE_ALLIANCE_COUNT = 5;
constexpr double CASCADE_ALLIANCE[CASCADE_ALLIANCE_COUNT] = {440, 640, 900, 1020, 1200}; // +120 (was 320, 520, 780, 900, 1080)
constexpr double CASCADE_L4 = 1390; // the old R2 top step; R2 doesn't go here any more
constexpr double CASCADE_MAX = 1390;        // highest R1 / the autons send the cascade (R2's L4; UP has no limit)
constexpr double CASCADE_MIN = 0;           // the bottom, for macros and autons (manual DOWN has no limit)
constexpr double CASCADE_LOAD = 90;         // loading height: B finds the bottom, then goes up to here (150 and 30 were tried)
constexpr double CASCADE_RIGHT = 240;       // the RaiseRight macro's cascade height (no button now; was D-pad RIGHT)
constexpr double CASCADE_UP_DEG = 100;      // above this the cascade counts as "up" for rollerStaysPut()
constexpr double ROLLER_STOWED = 0;
constexpr double ROLLER_DEPLOYED = 142;     // rollerTurn deploy unit; the autons use ROLLER_DEPLOYED * 2
constexpr double ROLLER_DEPLOYED_AUTON = 140.4;
constexpr double ROLLER_DEPLOYED_DRIVER = 340;  // rollerTurn when X / UP / R2 deploy it (was 350)
constexpr double ROLLER_A_POSITION = 260;       // rollerTurn for the auton pickups (was also A's position)
constexpr double ROLLER_A_DRIVER = ROLLER_A_POSITION + 25; // rollerTurn when A is tapped: 25 above the auton one
constexpr double ROLLER_HIGH = 500;             // rollerTurn's "high" position (Y is 50 past it)
constexpr double ROLLER_Y = ROLLER_HIGH + 50;   // rollerTurn on Y
constexpr double ROLLER_LEFT = 380;            // rollerTurn on D-pad LEFT
constexpr int Y_PICKUP_MM = 140;                // holding Y: something this close to the distance sensor runs PickUp
constexpr int LEFT_PICKUP_MM = 180;
// R1 (unloadMotion(), also for autons)
constexpr double R1_CASCADE_UP = 75;      // degrees up from where the cascade is (was 300 up after a 200 drop)
constexpr double R1_ROLLER_TO = ROLLER_Y; // Y's position, 550 deg (was 775)
constexpr int R1_OUTTAKE_FIRST_MS = 300;  // outtake alone this long first             // same for holding LEFT
// Most current rollerTurn may draw, mA: 80% of a green motor's default 2500. Less force
// means less chance of skipping a gear tooth (1500 was tried). If the roller can't lift or hold, raise it
constexpr int ROLLER_CURRENT_LIMIT_MA = 2000;
// rollerTurn speed, %, for driver control and the macros. Lower is gentler on the gears
// (65 was tried). The autons keep their own speeds
constexpr int ROLLER_SPEED_PCT = 100;
constexpr int CASCADE_DOWN_PCT = 100; // D-pad DOWN lowers the cascade at this % speed (was 80)
constexpr int ROLLER_SPIN_MS = 2000; // the deploy spin (X, UP) stops by itself after this
// R1 (Unload) sequence, also copied by autons: cascade down, outtake, then cascade up + roller out
constexpr double UNLOAD_CASCADE_DOWN = 200;  // degrees down from where the cascade is
constexpr int UNLOAD_OUTTAKE_FIRST_MS = 200; // outtake alone this long before lifting
constexpr double UNLOAD_CASCADE_UP = 400;    // degrees up from the bottom of the drop
constexpr double UNLOAD_ROLLER = 450;        // rollerTurn position while lifting

/** @brief The claw (pistonH), single-acting: air ON (true) holds it OPEN, air off lets the spring
 * close it (the other way round was tried and was backwards) */
constexpr bool CLAW_OPEN = true;
constexpr bool CLAW_CLOSED = false;

/** @brief Open (true) or close (false) the claw. Everything sets it through here, so clawOpen() is right */
void setClaw(bool open);
/** @brief True while the claw is open */
bool clawOpen();

/** @brief Percent of a green (200rpm) cartridge's top speed, as rpm */
constexpr int rpm(int percent) { return 200 * percent / 100; }

/** @brief Zero the mechanism encoders and set brake modes. Call once, in initialize() */
void init();

/** @brief toggleA (port 10) + rollerMain together, at a signed percent (+ in, - out) */
void intake(int percent);
void intakeStop();

/**
 * @brief Spin rollerMain at `percent` for at most `ms`, then it stops by itself. Doesn't wait.
 *
 * For the deploy spin, which otherwise runs until something else stops it, and overheats
 * the motor when there's nothing to take in. intake() and intakeStop() cancel the timer,
 * so a later intake or outtake is never cut short by it
 */
void rollerMainFor(int percent, int ms);

/**
 * @brief Should UP / R2 presses 1-3 leave rollerTurn where it is instead of sending it to
 * X's position? True when the cascade is already up (above CASCADE_UP_DEG) and the roller is
 * already out at X's position or higher (sent there, or within 25 deg of it)
 */
bool rollerStaysPut();
/** @brief Stop rollerMain if its rollerMainFor() time is up. The driver loop calls it every tick */
void rollerMainTimeout();
/** @brief intake(percent) for `ms` milliseconds, then stop: an L1/L2 tap, for autons. Blocking */
void intakeFor(int percent, int ms);

/** @brief Stop every mechanism motor (each in its own brake mode). Pistons are left as they are */
void stopAll();

/** @brief Flip the side piston (3-wire C) */
void toggleSide();

/**
 * @brief Spin toggleB (port 17) at full speed while the optical sensor (port 18) sees, within ~5 cm, yellow or
 * the other alliance's colour: yellow/blue on red alliance, yellow/red on blue alliance. Anything else
 * stops it dead. While D-pad LEFT or RIGHT (intake / outtake) is held it goes back to zero degrees
 * (the nearest full turn) and holds there, until 0.75 s after letting go. Each time toggleB starts
 * spinning, toggleA (port 10) goes back to zero degrees. Checks every 5 ms. Runs on its own task:
 * call once, in initialize()
 */
void startColourToggle();

/** @brief Which alliance the colour toggle plays for. Set from the selector's Qual tab; red to start */
void setBlueAlliance(bool blue);
bool isBlueAlliance();

/**
 * @brief Move a motor (or group) to an absolute position and wait until it arrives.
 *
 * The PROS equivalent of the Blocks "spin to position" block, with two safety
 * nets that block lacks: a timeout derived from the travel distance, so a jam
 * cannot hang the caller forever, and an optional abort check.
 *
 * @param motor    motor or motor group
 * @param target   absolute position, degrees
 * @param velocity rpm
 * @param abort    polled every 10ms; returning true stops waiting immediately
 * @return true if it arrived, false on timeout or abort
 */
bool spinTo(pros::AbstractMotor& motor, double target, int velocity = rpm(100),
            const std::function<bool()>& abort = nullptr);

/**
 * @brief Stow the roller and re-zero it against its stowed hard stop. Blocking, ~0.5-1 s.
 *
 * Moves near stowed at full speed, then pushes gently into the stop until it stops
 * moving, and zeroes the encoder there. So if the roller gear slipped, every stow puts
 * the encoder back in step with the real arm. Every driver stow (X, Y, A, B) goes through this.
 *
 * @param abort polled every 10ms; returning true stops (without re-zeroing)
 * @return true if it re-zeroed, false if aborted
 */
bool homeRoller(const std::function<bool()>& abort = nullptr);

/**
 * @brief The pickup (LEFT / Y's, after the sensor fires): intake on, cascade to the bottom (full
 * speed), THEN roller down to ROLLER_A_POSITION - 55; 0.05 s there, then starts the roller back to
 * X's position and returns without waiting for it. Leaves the intake running. Waits for the other
 * steps. @return false if aborted part way (intake stopped then)
 */
bool pickUpMotion(const std::function<bool()>& abort = nullptr);

/**
 * @brief Y's movement for autons, part 1 (doesn't wait): intake on, cascade down to the bottom,
 * roller up to ROLLER_Y. Call it before driving to the pin, then yFinish() at the pin
 */
void yStart();

/**
 * @brief Y's movement for autons, part 2 (doesn't wait for the roller to reach ROLLER_Y first):
 * pickUpMotion(), then the cascade up to `cascadeTo`. sensorWaitMs 0 = pick up straight away;
 * more = first wait (up to that long) for the distance sensor to see something within
 * Y_PICKUP_MM, picking up anyway if nothing is seen. With waitForLift false it returns without
 * waiting for the lift, so the auton can drive while it lifts; true waits until the cascade is
 * there. Leaves the intake running
 */
void yFinish(double cascadeTo, bool waitForLift = false, int sensorWaitMs = 0);

/**
 * @brief What R1 does (not its after-L4 version): outtake R1_OUTTAKE_FIRST_MS, then keep
 * outtaking while the cascade goes up R1_CASCADE_UP (no higher than CASCADE_MAX, never down)
 * and the roller to R1_ROLLER_TO, both at once. Stops the outtake when both arrive, or when
 * aborted. Waits. `cascadeUp` 0 = the cascade stays put (an auton scoring on a pin without lifting). `firstMs` 0 =
 * no outtake-alone wait: the roller starts straight away
 */
void unloadMotion(const std::function<bool()>& abort = nullptr, double cascadeUp = R1_CASCADE_UP,
                  int firstMs = R1_OUTTAKE_FIRST_MS);

} // namespace subsystems

#endif // SUBSYSTEMS_HPP
