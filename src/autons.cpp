#include <algorithm>
#include <cmath>
#include <functional>
#include "main.h"
#include "autons.hpp"
#include "macros.hpp"
#include "robot-config.hpp"
#include "subsystems.hpp"

// Autonomous routines. Each one is a plain blocking function; runAuton() (src/main.cpp) picks one by the
// selector's number. Set the alliance on the selector's Qual tab before the match: routines read it

// --- james_67 numbers -------------------------------------------------------------------------------
// Inches and degrees; the robot starts at 0, 0 facing 0 deg (= +y), clockwise positive
// STOP BUTTON FOR TEST RUNS (2026-11-27). While a routine is run as a test (hold A, press B or X; controls.cpp
// calls setTestRun), pressing Y stops it at once, wherever it is: the move in progress is cancelled, the drive
// holds, and the routine returns at its next check. Driver control then lets the drive coast, so the robot can be
// pushed by hand from there, and records the path. Never in a match: autonomous() does not set this.
// (Before this, three versions of a "teaching run" stopped at a place picked in the code, each time the wrong one,
// and a fourth paused after every movement.)
static bool testRun = false;     // this run was started as a test
static bool testStopped = false; // Y was pressed during it
static bool testYBefore = true;  // Y's last state: a Y still held from before must be let go first
void setTestRun(bool on) {
    testRun = on;
    if (on) {
        testStopped = false;
        testYBefore = true;
    }
}
bool testRunWasStopped() { return testStopped; }
// THE TEACHING RUN (hold A, press X): the same test run, but it also stops by itself after the turn on the spot
// numbered teachStopAfterTurn, counting the turns the routine makes after it has scored. In 67james those are:
//   1  after backing off the goal, to face the run-up point
//   2  at the run-up point, to face along the wall  (the robot's version of mark 3 of the hand recording: the
//      turn on the spot before the drive alongside)
//   3  after the drive in, the turn that gets the pin out ("the shove to the left")
// 2 was used to teach the drive in and that turn; 3, from the evening of 2026-11-27, to teach what comes after
// them ("the shove to the left was good, but the rest needs work"). After the stop the drive coasts (driver
// control does that) and the rest is shown by hand with taps of Y
// Both are on the controller now (controls.cpp): hold A + X stops after turn 2 ("the old A+X position"), hold
// A + Y after turn 3. 0 = an ordinary test run
static int teachStopAfterTurn = 0;
static bool teachRun = false;
static int teachTurns = 0;
void setTeachRun(int stopAfterTurn) {
    teachStopAfterTurn = stopAfterTurn;
    teachRun = stopAfterTurn > 0;
    teachTurns = 0;
}

namespace {
// true once Y has been pressed in a test run. Call it in every wait loop and after every step
bool stopAsked() {
    if (!testRun) return false;
    if (testStopped) return true;
    const bool y = master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    const bool pressed = y && !testYBefore;
    testYBefore = y;
    if (!pressed) return false;
    testStopped = true;
    chassis.cancelMotion(); // the move in progress ends; HOLD stops the drive
    leftMotors.brake();
    rightMotors.brake();
    xMotors.brake();
    const lemlib::Pose at = chassis.getPose();
    std::printf("[auton] STOPPED by Y at (%.2f, %.2f) heading %.1f. Push it by hand from here, a tap of Y at each key "
                "step\n", at.x, at.y, at.theta);
    master.rumble("..");
    return true;
}
// wait for the LemLib move that is running, or for Y
void waitMove() {
    while (chassis.isInMotion()) {
        if (stopAsked()) break;
        pros::delay(10);
    }
    chassis.waitUntilDone();
}
// at the start: toggleB turns this far, motor degrees
constexpr double START_TOGGLE_DEG = 360;
// 67james wiggles first: forward, back, forward again at WIGGLE_POWER, WIGGLE_MS each, the last forward
// WIGGLE_LAST_MS so it comes back in (each part first has to stop the last one). Straight motor power, not a
// LemLib drive: a 5 in moveToPoint only got about 1.2 in in 0.8 s and the drive back then counted as already there
// (2026-11-26). Driving each part by distance (5 in, back 3, forward to 5) was tried and taken out
constexpr int WIGGLE_POWER = 100; // of 127
constexpr int WIGGLE_MS = 200;
constexpr int WIGGLE_LAST_MS = 400; // was 200, then 300
// 67james after the wiggle: straight back this far, then to its first goal lifting to J67_GOAL1_LIFT_DEG on the
// way, and score there
constexpr float J67_BACK_IN = 10;
constexpr int J67_BACK_TIMEOUT_MS = 1000;
constexpr float J67_GOAL1_X = 14.5; // 2 in forward of 12.5 (facing 90: forward is +x)
constexpr float J67_GOAL1_Y = -14.2;
constexpr float J67_GOAL1_THETA = 90;
constexpr double J67_GOAL1_LIFT_DEG = 300;
constexpr int TO_J67_GOAL1_TIMEOUT_MS = 2000;
// after the score: straight back this far (the back-up after the wiggle stays J67_BACK_IN)
constexpr float J67_SCORE_BACK_IN = 9; // was 10 (J67_BACK_IN), 11, then 8
// then turn on the spot to this heading, and L1's grab at the first pin (closes once there)
constexpr float J67_TURN1_THETA = 45; // was 45, 40, 37.5, 35, 40, then 35
constexpr int J67_TURN1_TIMEOUT_MS = 1300; // was 800: a turn that settles takes 0.8 to 1.0 s
// The pose with the claw on the wall pin, read off a Y-button mark with the robot pushed there by hand from the
// start position (2026-11-27: x 13.508, y -0.131, theta 30.903). Was 17, 0, 13 (and 13.8 before: "the odometry
// only got to about 10.5 there")
// SUPERSEDED the same day by the marks below (the first mark put it at 13.51, -0.13, 30.9; the second recording,
// 1.7 in away, is the one in use: see the note at J67_PIN1_X)
// The wall pin's poses, from Y-button marks 4, 6 and 7 of the second hand-pushed recording (2026-11-27, robot
// pushed from the start position; mark numbers as in james67b). All three are in the same recording, so they are
// right relative to each other; against the start position they are only as good as where the robot stood when
// the program was started
constexpr float J67_SIDE_X = 9.25;   // mark 4: alongside, facing along the wall, about to turn in
constexpr float J67_SIDE_Y = -2.54;
constexpr float J67_SIDE_THETA = 86.9;
// TAUGHT BY HAND, second teaching run (2026-11-27): see james67b for the four marks these come from
// WHERE THE DRIVE IN STOPS. From the fifth teaching run (2026-11-27, 17:20, stopped after the turn along the wall,
// two marks): 1 = (11.76, -2.71) facing 90.3, the drive in; 2 = (13.50, 0.21) facing 29.8, after the turn. The
// drive in had been ending at about (10.9, -1.4): "too left", by 1.3 in, and 0.9 in short. Mark 2 is 3.4 in from
// mark 1 along a bearing of 30.8, within a degree of its own heading, so a turn on the spot at mark 1 followed by a
// straight push of 3.4 in lands on it: the drive in goes to mark 1 itself. Y is 0.15 in lower: the last drives in
// ended on average that much to the left of where they were sent.
// Before: (10.40, -1.92); (10.89, -1.82) ("0.5 inch further"); (10.96, -1.50) (fourth teaching run)
// (17:25) That missed the piece completely: "too far to the right, it doesn't scoop anything". A sixth teaching
// run gave the same two marks to within 0.15 in: 1 = (11.62, -2.69) facing 91.8, 2 = (13.47, 0.18) facing 28.4.
// The marks were right; turning ON THE SPOT at mark 1 was not. By hand the robot slides about 0.6 in to the LEFT
// of the straight line into mark 2 during the first 20 deg of the turn (91.8 -> 71), which is when the front-left
// edge sweeps the piece, and is back on the line by about 64 deg. So the turn is now made 0.6 in left of that line
// and 3.0 in back from mark 2, where the hand's turn was; the push then steers back onto the line. (The runs that
// were "too left" turned 1.4 to 1.6 in left of it; mark 1 itself is on it.) Y keeps the 0.15 in correction
constexpr float J67_IN_X = 11.46;
constexpr float J67_IN_Y = -2.28;
// where the push into the tunnel ends: mark 2 (sixth teaching run) and J67_TUNNEL_PAST_IN past it along the heading
constexpr float J67_MARK2_X = 13.47;
constexpr float J67_MARK2_Y = 0.18;
constexpr float J67_TUNNEL_PAST_IN = 0.4;
constexpr float J67_OUT_THETA = 29.8;  // mark 2's heading (fifth teaching run 29.8, sixth 28.4; was 38.1, then 33.5)
constexpr float J67_OUT_EXTRA_DEG = 15; // how much further left that turn swings before it comes back
constexpr float J67_PROOF_THETA = 14.8; // mark 4's heading: where the push ends
constexpr float J67_PUSH_IN = 4.3;     // the push from the turn to mark 4, through mark 3
constexpr float J67_HALF_TRACK_IN = 11.61f / 2; // half the drive's track width (robot-config.cpp)
constexpr float J67_KNOCK_THETA = 36.6; // mark 5's heading: the turn at mark 4 that knocks the other piece away
constexpr float J67_MOUTH_X = 11.31; // mark 6: at the mouth of the tunnel
constexpr float J67_MOUTH_Y = -0.90;
// WHERE THE PUSH INTO THE TUNNEL ENDS: mark 2 of the fourth teaching run, (12.13, 0.41) facing 38.1, plus
// J67_PUSH_EXTRA_IN further along that heading so that it cannot stop short (the wall stops it).
// Earlier values, each from a different recording: 13.46, 0.94, 29.3 (third teaching run); 11.80, -0.24, 31.3
// (hand-pushed from the start position); 13.51, -0.13, 30.9 (the first mark of all); 17, 0, 13; 13.8
// The push into the tunnel after the turn: straight ahead this far. Mark 2 is 2.1 in ahead of a turn made on the
// spot at J67_IN; the pose reads up to 0.75 in further back after that turn; and it must not stop short ("make
// sure we are not undershooting"), so 0.35 in more. The wall stops it if that is too far: it then gives up after
// J67_TUNNEL_PUSH_MAX_MS and the routine carries on
// the movement after the pin: where it ends (one Y mark, see james67b)
constexpr float J67_AFTER_X = 14.51;
constexpr float J67_AFTER_Y = 3.36;
constexpr float J67_AFTER_THETA = 9.3;
constexpr float J67_TUNNEL_PUSH_IN = 3.8; // mark 1 to mark 2 of the fifth teaching run is 3.4 in, plus 0.4 so it does not stop short (was 3.2)
constexpr int J67_TUNNEL_PUSH_MAX_MS = 1800;
constexpr float J67_PUSH_EXTRA_IN = 0.5;
constexpr float J67_PIN1_THETA = 38.1;
constexpr float J67_PIN1_X = 12.13f + J67_PUSH_EXTRA_IN * 0.6170f; // sin 38.1
constexpr float J67_PIN1_Y = 0.41f + J67_PUSH_EXTRA_IN * 0.7869f;  // cos 38.1
// the way from the goal to mark 4 (not shown by hand: a guess to be checked on the robot)
constexpr float J67_BACKOFF_X = 4.0;        // straight back off the goal to this x
constexpr float J67_SIDE_RUNUP_IN = 10;     // then to a point this far short of mark 4, on its line (was 6)
constexpr int J67_RUNUP_TIMEOUT_MS = 2000;
constexpr float J67_RUNUP_MAX_SPEED = 80;   // of 127
constexpr int J67_LINE_MAX_MS = 2500;       // each slow line drive gives up after this long
// the slow line drive's speed, motor rpm (120 rpm is about 13 in/s): top speed, the rpm it may have per inch still
// to go, its slowest, the time it takes to reach top speed, and how far short of the point it lets go
constexpr float J67_LINE_MAX_RPM = 60;     // was a flat 120 (J67_FORWARD_RPM)
constexpr float J67_LINE_RPM_PER_IN = 40;
constexpr float J67_LINE_MIN_RPM = 45;     // was 24, then 32: both stalled short of the end with the intake pushing back
constexpr float J67_LINE_RAMP_MS = 250;
constexpr float J67_LINE_STOP_EARLY_IN = 0.15;
constexpr int J67_STOP_MS = 150;            // standing after each slow line drive, before the next move
// the line-up point is this far back from the pin along the pin's heading. 16.4 puts it at (5.1, -14.2), on the
// goal's own line, so backing off the goal straight reaches it
constexpr float J67_LINEUP_IN = 16.4;
constexpr int J67_LINEUP_TIMEOUT_MS = 1500;
// on the slow drive to the pin: steer at the pin until it is this close, then hold J67_PIN1_THETA
constexpr float J67_PIN_AIM_UNTIL_IN = 5;
// and give up this far past the distance it started from the pin (the wall stops it before that)
constexpr float J67_PIN_PAST_IN = 1;
// between the turn and the pin drive: straight forward this far at J67_FORWARD_RPM on the motors' own speed control
// (move_velocity, percent of the blue motors' 600 rpm), both sides the same, until odometry says it got there.
// Tried before: a 5 in LemLib drive (about 1.2 in in 0.8 s); 50% straight power then a step down to a slow speed
// (twitched at the switch); a ramp up, easing from 50% down to 20% and holding the heading with the inertial
// sensor (too fast); straight power under about 28 of 127 stalled
constexpr float J67_FORWARD_IN = 16; // was 5, 10, 8, 12, 16, then 18
constexpr int J67_FORWARD_RPM = 600 * 20 / 100; // 20%, about 13 in/s
// heading hold on that drive: each degree off J67_TURN1_THETA moves the two sides this many rpm apart (each way),
// up to J67_FORWARD_HOLD_MAX_RPM. 5 deg off is 50 rpm on a 120 rpm drive
constexpr float J67_FORWARD_HOLD_RPM_PER_DEG = 5; // was 6 (sat 4.4 deg off after a turn that had not settled), then 10 (wiggled)
// the drive to the pin follows a line: degrees to turn toward the line per inch off it, and the most it may turn
constexpr float J67_PIN_LINE_DEG_PER_IN = 3; // was 6
constexpr float J67_PIN_LINE_MAX_DEG = 5;    // was 8
// its trace: one sample per J67_TRACE_MS, at most J67_TRACE_MAX of them
constexpr std::uint32_t J67_TRACE_MS = 40;
constexpr int J67_TRACE_MAX = 60;
constexpr float J67_FORWARD_HOLD_MAX_RPM = 40; // was 60
// after the drive (L1's grab running): grab, then turn to J67_TURN3_THETA
// the nudge after the drive, before the grab: only the right side drives forward (the left holds), at
// J67_NUDGE_POWER for J67_NUDGE_MS, which swings the robot to the left. (Stopping once the heading had dropped
// 5 deg was tried first: 40 of 127 turned it 1 deg in 0.5 s, 90 swung 5 deg in 0.22 s)
// the swing onto the pin after the forward drive: left side held, right side forward, to this heading. 45 (the
// forward drive's heading) minus about 10, the average of what the timed nudge did. TUNE THIS on the robot:
// lower = the right side goes further round
// 35 was the first guess. A photo of a miss (2026-11-27) shows the robot about 35 to 40 deg off square to the wall
// with its LEFT arm on the wall and the pin just inside that arm, so squaring up means turning to about 5 to 13
// (the pose drive that was tried earlier used 13)
constexpr float J67_SWING_THETA = 10;
constexpr int J67_SWING_TIMEOUT_MS = 1500; // about 35 deg with one side
constexpr float J67_SWING_MAX_SPEED = 64; // of 127, the nudge's old power
// (the timed nudge these replace; kept for the record)
constexpr int J67_NUDGE_POWER = 64; // of 127: 50%
constexpr int J67_NUDGE_MS = 1000;
constexpr float J67_TURN3_THETA = 40; // was 45
constexpr int J67_TURN3_TIMEOUT_MS = 1500; // was 800 for the turn on the spot
constexpr int J67_TURN3_POWER = 64; // of 127: 50%, the right side only (backward)
constexpr int J67_FORWARD_MAX_MS = 4500; // 9 in at 3.2 in/s is about 2.8 s; was 3500; // 2500 ran out at 28 power (17.4 of 18 in); // was 2000 before the longer slow part; 600 (for 5 in), 1000 (for 10), 1200 (for 12), 1500 (16 at 80)

// first goal: drive here, lifting on the way, and score
constexpr float GOAL_X = -13.27;
constexpr float GOAL_Y = 20.6;
constexpr float GOAL_THETA = 0;
constexpr int TO_GOAL_TIMEOUT_MS = 2000; // was 3000
// lift height on the way to the goal, motor degrees (tried: level 1, 440; half of level 1, 220)
constexpr double TO_GOAL_LIFT_DEG = 300;
constexpr int LIFT_DELAY_MS = 500; // the lift starts this long into the drive (was 100)
// (a min speed of 50 was tried and taken out: it also makes LemLib end the move still moving)
// (a horizontalDrift of 8 on this drive, then 6 on every pose drive, to go faster round curves: both
// tried and taken out)
// (a 5 in drive forward between the goal and scoring, 200 ms timeout, was tried and taken out)

// after each score: straight back this far, opposite to the way the robot faces
constexpr float BACK_IN = 10;
constexpr int BACK_TIMEOUT_MS = 1000;
// the drive back ends (HOLD stops it) as soon as the robot is this close to its point
// (chaining it into the pin drive, still moving, with a min speed of 50 was tried twice and taken out)
constexpr float BACK_EXIT_WITHIN_IN = 2;

// first pin: L1's grab (claw open + motors 14/16 in) while driving here, then close on it
// 1 in to the robot's right and 2 in back (facing -45 deg) of -36, 21 (tried: -36, 20; -34, 21; -36, 21;
// -34.59, 22.41; -35.29, 21.71)
constexpr float PIN_X = -33.88;
constexpr float PIN_Y = 20.30;
constexpr float PIN_THETA = -45;

// second goal: lift to level 1 alliance height while driving here, and score
constexpr float GOAL2_X = -7.3; // 2 in right (+x) of -9.3 (was -13.3, then 4 in forward to -9.3)
constexpr float GOAL2_Y = 22.5; // 1 in to the robot's right (facing 90 deg: left is +y) of 23.5 (tried 23.5, 21.5)
constexpr float GOAL2_THETA = 90;
constexpr int TO_GOAL2_TIMEOUT_MS = 2000; // was 3000
constexpr double GOAL2_LIFT_DEG = subsystems::CASCADE_ALLIANCE[0]; // first pin's score: level 1 alliance, 500 (400 tried; 440 was level 1)

// second pin: grab, the same way as the first
// 6 in back (facing 47.5 deg) of -9.71, 47.13 (tried: -10, -12, -14 at 45.9; -12.53, 47.25; -11.06, 48.6; -9.71, 47.13;
// -13.40, 43.75; -17.09, 40.37; -15.62, 41.72)
constexpr float PIN2_X = -14.15;
constexpr float PIN2_Y = 43.07;
constexpr float PIN2_THETA = 47.5;

// third goal: lift to level 2 alliance height while driving here, and score
constexpr float GOAL3_X = -10; // 5 in left of -15 (robot faces 180 here: its right is -x). Tried -10, -12, -15, -10, -15, -5.5, -0.5, -4.5, -7.5, -15
constexpr float GOAL3_Y = 26; // (facing 180: forward is -y). Tried 24, 22, 20, 28.9, 23.9, 20.9, 17.9
constexpr float GOAL3_THETA = 180;
constexpr int TO_GOAL3_TIMEOUT_MS = 2000; // was 3000
constexpr double GOAL3_LIFT_DEG = subsystems::CASCADE_ALLIANCE[1]; // second pin's score: level 2 alliance, 700 (640 was level 2)

// after the third goal: B's reset (lift to the bottom and re-zeroed, roller homed) while driving straight to the
// third pin. (Backing up to -7, 46 facing 180 first was tried and taken out.) Before that drive, straight back
// this far (+y: the robot faces 180 there)
constexpr float BACK3_IN = 25; // was 20; 30 tried and taken back
constexpr int BACK3_TIMEOUT_MS = 1800; // was 1500 (for 20 in), 2100 (for 30 in)
// B's reset starts once the robot has backed this far from the goal. B finds the bottom by driving the lift down
// until it stops; started at the goal, the lift is still sitting on it, so B "found the bottom" there, zeroed the
// lift at goal height and never went down (2026-11-20)
constexpr float RESET_AFTER_BACK_IN = 6;
// then turn on the spot to this heading before driving to the third pin
constexpr float PIN3_TURN_THETA = 90;
constexpr int PIN3_TURN_TIMEOUT_MS = 1000;

// third pin: grab, the same way as the others
constexpr float PIN3_X = 12;
constexpr float PIN3_Y = 55; // 2 in to the robot's right (facing 90 deg: right is -y) of 57
constexpr float PIN3_THETA = GOAL2_THETA; // the same heading as the goal scored after the first pin (90)
// the fourth pin (the second third-pin round): grabbed 2 in further forward (facing 90: forward is +x), same heading
constexpr float PIN4_X = PIN3_X + 2;
constexpr float PIN4_Y = PIN3_Y;
// and scored 1 in to the robot's left and 2 in forward of the third goal's point (facing 180: left is +x,
// forward is -y), same heading (was 2 in left)
constexpr float GOAL4_X = GOAL3_X + 1;
constexpr float GOAL4_Y = GOAL3_Y - 2;
// the fifth pin (L1's grab): 2 in to the robot's right of the fourth pin's point (facing 90: right
// is -y), same heading (2 in left was tried)
constexpr float PIN5_X = PIN4_X;
constexpr float PIN5_Y = PIN4_Y - 1; // 1 in to the robot's right of the fourth pin (2 in right tried, 2 in left)
// then straight back this far, and to the fifth goal lifting to GOAL5_LIFT_DEG on the way, and score there
constexpr float PIN5_BACK_IN = 10;
constexpr float GOAL5_X = -31; // 2 in forward of -29 (facing 270: forward is -x)
constexpr float GOAL5_Y = 42; // 3 in to the robot's left of 45 (facing 270: left is -y). Tried 45, 43
constexpr float GOAL5_THETA = 270;
constexpr double GOAL5_LIFT_DEG = 400; // was 300
constexpr int TO_GOAL5_TIMEOUT_MS = 2500; // about 43 in with a 180 deg turn
// after the fifth pin's score: back to the third-pin spot for a sixth pin. The robot faces 270 at the goal and the
// pin is behind it (+x), so: straight back PIN6_BACK_IN (B's reset once clear of the goal, as after the third goal),
// turn round on the spot to face 90, then the usual L1 grab (early catch) with its pose drive to the pin
constexpr float PIN6_X = PIN5_X; // the fifth pin's point, same heading (90)
constexpr float PIN6_Y = PIN5_Y;
constexpr float PIN6_BACK_IN = 10;
constexpr int PIN6_TURN_TIMEOUT_MS = 1000; // about 180 deg on the spot, to face 90
constexpr int TO_PIN6_TIMEOUT_MS = 3000; // about 40 in
// then straight back this far (-x: it faces 90 there), and back to the third goal's point (GOAL3_*, facing 180),
// lifting to level 3 alliance height (900 deg) on the way, and score there
constexpr float BACK4_IN = 11;
constexpr int BACK4_TIMEOUT_MS = 1000;
constexpr int RETURN_TO_GOAL3_TIMEOUT_MS = 3500; // about 30 in (was 2500: ran out, 2026-11-25)
// before that drive, turn on the spot to face the goal's point, so the drive is a straight line. One curved
// moveToPose from facing 90 to the goal facing 180 arced wide to the right (LemLib slows tight curves down) and ran
// out of time 18 and 24 in short (2026-11-25, pins 3 and 4)
constexpr int RETURN_TURN_TIMEOUT_MS = 800;
// then the whole third-pin cycle again (back up + B, turn, grab at the third pin, back up, to the third goal's
// point), this time lifting to level 4 alliance height (1020 deg). Driver control's anti-tip (controls.cpp) caps
// the drive at 80% with the lift at 1020, so this drive gets the same cap
constexpr float LEVEL4_MAX_SPEED = 127 * 0.8;
// the lift's speed (percent) every time it lifts up to a goal (was 100; 80 on pins 3 and 4 first, then all)
constexpr int LIFT_PERCENT = 80;

// after the fourth pin's score: the same back-up + B and turn to 90, then to the fifth pin's point (PIN5_*) with
// L1's grab (grabPin), motors 14/16 intaking through the PIN5_BACK_IN back-up after it. (L2's grab, claw closed
// and intaking all the way, was tried first and swapped for L1's)

// every drive to a pin
constexpr int TO_PIN_TIMEOUT_MS = 2000; // was 3000
// within this far of the pin point, finish the drive slower
constexpr float PIN_SLOW_WITHIN_IN = 3; // was 5 (pose drives end at 2 in, so about the last inch is slow)
// 50% of 127. NOTE: a moveToPose that starts inside 7.5 in of its target replaces maxSpeed with 60 (LemLib,
// moveToPose.cpp), so the robot actually gets 60 (47%) whatever this says
constexpr float PIN_SLOW_SPEED = 127 * 0.5;
constexpr int PIN_SLOW_TIMEOUT_MS = 1500;
// at the pin: wait up to this long for the claw sensor to see it (within CLAW_CATCH_MM), then close anyway
constexpr int PIN_CATCH_WAIT_MS = 200; // was 500
// pins 3, 4 and 5: from this long after the claw opens, close it as soon as the sensor sees
// something (within CLAW_CATCH_MM), even mid-drive, and end the drive there. The other pins wait for the point
constexpr std::uint32_t PIN3_CATCH_AFTER_MS = 250;
// pins 1 and 2: the same early catch (claw closes the moment the sensor sees the pin), but the drive carries on to
// the point
constexpr std::uint32_t PIN12_CATCH_AFTER_MS = 250;

// When a pose drive counts as arrived (waitPose). Same ranges as the drive's own exit settings in
// robot-config.cpp, but EITHER one ends the drive, as in moveToPoint: LemLib's moveToPose wants BOTH, so a robot
// stopped 0.6 in off (against a goal, say) sat there until the timeout
constexpr float ARRIVED_SMALL_IN = 0.5;
constexpr int ARRIVED_SMALL_MS = 200;
constexpr float ARRIVED_LARGE_IN = 3;
constexpr int ARRIVED_LARGE_MS = 500;
constexpr float ARRIVED_HEADING_DEG = 3; // and the heading within this of the target

// Every pose drive ends as soon as the robot is this close to its point (was 1, on the second pin only).
// Done in waitPose, not with LemLib's earlyExitRange: that only works together with a minSpeed, which made the
// robot miss the point. The heading may still be turning when it ends
constexpr float POSE_EXIT_WITHIN_IN = 2;

// Start R1's scoring motion (lift down until it lands, open the claw, outtake) once the robot is this close to a
// goal, while still driving, instead of after the drive. The lift takes ~0.3-0.4 s to come down from scoring
// height, so the robot is normally there before it lands. Lower it if the lift catches the edge of a goal; 0 = start
// it after the drive, as before
constexpr float SCORE_EARLY_IN = 4;

// --- steps used more than once ------------------------------------------------------------------------

// Wait for the pose drive to (x, y, theta) that is running now, and end it as soon as the robot has arrived
// (see ARRIVED_*; theta NAN = any heading, for a moveToPoint), or is within `exitWithin` of the point (0 = no early exit). `eachTick` runs every 10 ms
// meanwhile. Prints how long the drive took and why it ended
void waitPose(float x, float y, float theta, float exitWithin = POSE_EXIT_WITHIN_IN, const std::function<void()>& eachTick = nullptr) {
    const std::uint32_t start = pros::millis();
    const lemlib::Pose target(x, y);
    const char* why = "LemLib ended it (its own exit or the timeout)";
    int smallMs = 0, largeMs = 0;
    while (chassis.isInMotion()) {
        if (stopAsked()) break; // Y in a test run
        const lemlib::Pose pose = chassis.getPose();
        const float dist = pose.distance(target);
        const bool headingOk =
            std::isnan(theta) || std::fabs(std::remainder(pose.theta - theta, 360.0)) <= ARRIVED_HEADING_DEG;
        smallMs = dist <= ARRIVED_SMALL_IN && headingOk ? smallMs + 10 : 0;
        largeMs = dist <= ARRIVED_LARGE_IN && headingOk ? largeMs + 10 : 0;
        if (exitWithin > 0 && dist <= exitWithin) why = "within the early exit range";
        else if (smallMs >= ARRIVED_SMALL_MS) why = "arrived (small range)";
        else if (largeMs >= ARRIVED_LARGE_MS) why = "arrived (large range)";
        else {
            if (eachTick) eachTick();
            pros::delay(10);
            continue;
        }
        chassis.cancelMotion(); // HOLD stops it here
        break;
    }
    chassis.waitUntilDone();
    const lemlib::Pose end = chassis.getPose();
    std::printf("[auton] to (%.1f, %.1f, %.0f): %lu ms, %s; ended %.2f in off, heading %.1f deg off\n", x, y, theta,
                static_cast<unsigned long>(pros::millis() - start), why, end.distance(target),
                std::remainder(end.theta - theta, 360.0));
}

// R1's motion (lift down onto the goal, open the claw, outtake 0.3 s), the same code as the driver's R1. Waits
// only until the claw opens: the outtake carries on in the macro task while the robot drives away. (If the
// lift never lands, the macro ends without opening the claw, and this waits for that.)
// waitOuttake: wait for the 0.3 s outtake too, as before
void finishScore(bool waitOuttake = false) {
    while (macros::isBusy() && (waitOuttake || !subsystems::clawOpen()) && !stopAsked()) pros::delay(10);
}

// Start R1's motion in the macro task (finishScore() waits for it). Motors 14/16 stop here: the motion owns them
// from now on (it outtakes with them once the claw opens), so nothing else may drive them until it is over
void startScore() {
    xMotors.brake();
    macros::setPlaceDownPercent(100); // full speed down in the auton (driver control's R1 uses 80)
    macros::request(macros::Macro::PlaceRelease, true);
}

// Wait for the pose drive to the goal (x, y, theta) that is running now, starting the scoring motion once within
// SCORE_EARLY_IN of the point (or when the drive ends, if it never got that close). `eachTick` runs every 10 ms
// until scoring starts. finishScore() after it
void waitPoseThenScore(float x, float y, float theta, const std::function<void()>& eachTick = nullptr) {
    const lemlib::Pose goal(x, y);
    bool started = false;
    waitPose(x, y, theta, POSE_EXIT_WITHIN_IN, [&] {
        // the scoring motion opened the claw: the pin is on, end the drive. Against a goal the robot can stop short
        // of its point (3.8 in at the fifth goal) and used to push there until the timeout, a second each time
        // (2026-11-25)
        if (started && subsystems::clawOpen()) {
            chassis.cancelMotion();
            return;
        }
        if (started) return;
        if (chassis.getPose().distance(goal) <= SCORE_EARLY_IN) {
            startScore();
            started = true;
            std::printf("[auton] scoring started %.1f in from the goal\n", chassis.getPose().distance(goal));
            return;
        }
        if (eachTick) eachTick();
    });
    if (!started && !stopAsked()) startScore(); // not after a Y stop in a test run
}

// straight back `inches`, opposite to the way the robot faces now. Waits. `once` (if given) runs once, when the
// robot has backed `onceAfterIn` (or at the end, if the drive stopped short of that)
void driveBack(float inches, int timeoutMs, const std::function<void()>& once = nullptr, float onceAfterIn = 0) {
    const lemlib::Pose now = chassis.getPose();
    const float heading = lemlib::degToRad(now.theta);
    const lemlib::Pose target(now.x - inches * std::sin(heading), now.y - inches * std::cos(heading));
    chassis.moveToPoint(target.x, target.y, timeoutMs, {.forwards = false});
    bool done = once == nullptr;
    while (chassis.isInMotion() && chassis.getPose().distance(target) > BACK_EXIT_WITHIN_IN) {
        if (stopAsked()) break; // Y in a test run
        if (!done && chassis.getPose().distance(now) >= onceAfterIn) {
            once();
            done = true;
        }
        pros::delay(10);
    }
    if (!done && !stopAsked()) once();
    chassis.cancelMotion(); // (nothing to cancel if it already ended) HOLD stops it here
    chassis.waitUntilDone();
}

// L1's grab at a pin: claw open and motors 14/16 in while driving to (x, y, theta), full speed until within
// PIN_SLOW_WITHIN_IN, then slower; at the point close the claw as soon as the sensor sees the pin (or after
// PIN_CATCH_WAIT_MS anyway). Motors 14/16 are left intaking: L1 keeps them on CLAW_INTAKE_AFTER_MS more.
// exitWithin > 0: end the drive as soon as the robot is that close to the point (HOLD then stops it)
// @return millis() when the claw closed, for stopping them
// theta NAN: no final heading, the drives are moveToPoint instead of moveToPose
// catchAfterMs > 0: from that long after the claw opens, close as soon as the sensor sees something; with
// stopOnCatch, end the drive there too (pins 3-5), otherwise carry on to the point (pins 1 and 2)
std::uint32_t grabPin(float x, float y, float theta, float exitWithin = POSE_EXIT_WITHIN_IN,
                      std::uint32_t catchAfterMs = 0, bool stopOnCatch = true, int timeoutMs = TO_PIN_TIMEOUT_MS) {
    // finishScore() returns before its outtake is over, and the outtake ends by stopping motors 14/16: let it finish
    // first, or it would stop the intake started below (normally already over by the end of the drive back).
    // Other macros (B's reset) don't touch motors 14/16 and the claw, so the grab doesn't wait for them
    while (macros::isRunning(macros::Macro::PlaceRelease)) pros::delay(10);
    subsystems::setClaw(true);
    const std::uint32_t openedAt = pros::millis();
    xMotors.move_velocity(subsystems::rpm(100));
    bool caught = false;
    std::uint32_t closedAt = 0;
    auto sees = [] {
        const std::int32_t mm = clawSensor.get_distance();
        return mm > 0 && mm <= subsystems::CLAW_CATCH_MM;
    };
    // the early catch: closes the claw the moment it sees the pin (and ends the drive, with stopOnCatch)
    auto watch = [&] {
        if (catchAfterMs == 0 || caught || pros::millis() - openedAt < catchAfterMs || !sees()) return;
        caught = true;
        subsystems::setClaw(false);
        closedAt = pros::millis();
        if (stopOnCatch) chassis.cancelMotion();
    };
    auto stopped = [&] { return caught && stopOnCatch; };

    const bool anyHeading = std::isnan(theta);
    auto drive = [&](int timeoutMs, float maxSpeed) {
        if (anyHeading) chassis.moveToPoint(x, y, timeoutMs, {.forwards = true, .maxSpeed = maxSpeed});
        else chassis.moveToPose(x, y, theta, timeoutMs, {.forwards = true, .maxSpeed = maxSpeed});
    };
    drive(timeoutMs, 127);
    // LemLib can't change a move's speed part way, so the first move is cancelled and a slow one started
    const lemlib::Pose pin(x, y);
    while (chassis.isInMotion() && chassis.getPose().distance(pin) > PIN_SLOW_WITHIN_IN) {
        watch();
        pros::delay(10);
    }
    if (chassis.isInMotion() && !stopped()) {
        // a cancelled move sets the drive to 0 for ~10 ms before the next starts: coast through that
        // instead of HOLD braking hard at speed
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
        chassis.cancelMotion();
        chassis.waitUntilDone(); // the first move has fully ended, so the next isn't dropped
        drive(PIN_SLOW_TIMEOUT_MS, PIN_SLOW_SPEED);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    }
    if (!stopped()) waitPose(x, y, theta, exitWithin, watch);
    chassis.waitUntilDone();

    int waited = 0;
    for (; !caught && waited < PIN_CATCH_WAIT_MS; waited += 10) {
        if (sees() && pros::millis() - openedAt >= catchAfterMs) break;
        pros::delay(10);
    }
    if (!caught) {
        subsystems::setClaw(false);
        closedAt = pros::millis();
    }
    std::printf("[auton] pin (%.1f, %.1f): claw closed (%s, sensor %d mm, %.1f in from the point)\n", x, y,
                caught ? "caught on the way" : waited < PIN_CATCH_WAIT_MS ? "caught" : "timed out",
                static_cast<int>(clawSensor.get_distance()), chassis.getPose().distance(pin));
    return closedAt;
}

// after a grab: lift to `liftDeg` while driving to the goal (x, y, theta), scoring on the way in (finishScore()
// after it). Motors 14/16 keep
// intaking CLAW_INTAKE_AFTER_MS after the grab (`closedAt`, from grabPin), as L1 does, during the drive
void liftDrive(double liftDeg, float x, float y, float theta, int timeoutMs, std::uint32_t closedAt,
               float maxSpeed = 127, int liftPercent = LIFT_PERCENT) {
    // B's reset must be over before lifting, or the two fight over the lift (normally long over by now)
    while (macros::isRunning(macros::Macro::ResetAll)) pros::delay(10);
    cascade.move_absolute(liftDeg, subsystems::rpm(liftPercent));
    chassis.moveToPose(x, y, theta, timeoutMs, {.forwards = true, .maxSpeed = maxSpeed});
    waitPoseThenScore(x, y, theta, [closedAt] {
        if (pros::millis() - closedAt >= subsystems::CLAW_INTAKE_AFTER_MS) xMotors.brake();
    });
    // (if scoring started before the 2 s were up, startScore() stopped the intake: the outtake comes next anyway)
}

// From just after scoring at the third goal: back away (B's reset once clear of the goal), then turn to face the
// third pin. The scoring motion is completely over first (its outtake too): starting a macro cancels the one
// running. B carries on in the macro task during the next drive
void backUpAndTurn() {
    while (macros::isBusy()) pros::delay(10);
    driveBack(BACK3_IN, BACK3_TIMEOUT_MS, [] { macros::request(macros::Macro::ResetAll, true); }, RESET_AFTER_BACK_IN);
    chassis.turnToHeading(PIN3_TURN_THETA, PIN3_TURN_TIMEOUT_MS);
    chassis.waitUntilDone();
}

// From just after scoring at the third goal: back away (B's reset once clear of the goal), turn, grab the third
// pin, back away from it, then lift to `liftDeg` while driving back to the third goal's point, and score there.
// Waits until the claw opens on the goal
void thirdPinCycle(double liftDeg, float pinX, float pinY, float goalX, float goalY, float maxSpeed = 127) {
    // B's reset once clear of the goal (RESET_AFTER_BACK_IN). The scoring motion is completely over first (its
    // outtake too): starting a macro cancels the one running. B carries on in the macro task during the drive to
    // the pin
    backUpAndTurn();

    // the pin. The reset may still be running: it uses the lift and roller, the grab doesn't
    const std::uint32_t closedAt = grabPin(pinX, pinY, PIN3_THETA, POSE_EXIT_WITHIN_IN, PIN3_CATCH_AFTER_MS);
    // back away from the pin (motors 14/16 keep intaking their 2 s after the grab, as L1; liftDrive stops them)
    driveBack(BACK4_IN, BACK4_TIMEOUT_MS);

    // back to the third goal's point, lifting on the way, and score there (scoring starts SCORE_EARLY_IN out, as
    // at the other goals). B's reset is long over by now; wait for it if not, so it doesn't fight the lift
    while (macros::isRunning(macros::Macro::ResetAll)) pros::delay(10);
    chassis.turnToPoint(goalX, goalY, RETURN_TURN_TIMEOUT_MS);
    chassis.waitUntilDone();
    liftDrive(liftDeg, goalX, goalY, GOAL3_THETA, RETURN_TO_GOAL3_TIMEOUT_MS, closedAt, maxSpeed);
    finishScore();
}

// a check after a step: true = Y was pressed in a test run, end the routine here (the name is kept from when this
// paused after every movement)
bool teachPause(const char*) { return stopAsked(); }

// ---- slow, exact moves for the wall pin (67james) ------------------------------------------------------------------
// A turn on the spot with LemLib's own turn, not the team's wrapper: the wrapper gives a turn under SMALL_TURN_DEG a
// minimum speed, and LemLib then ends that turn the moment it crosses the target, still spinning. Without it the
// turn settles before it ends. Waits, and prints what it did
void settleTurn(const char* what, float heading, int timeoutMs) {
    const float before = chassis.getPose().theta;
    const std::uint32_t start = pros::millis();
    if (stopAsked()) return;
    chassis.lemlib::Chassis::turnToHeading(heading, timeoutMs);
    waitMove();
    std::printf("[auton] %s: turn to %.1f, heading %.1f -> %.1f after %u ms\n", what, heading, before,
                chassis.getPose().theta, static_cast<unsigned>(pros::millis() - start));
    if (teachRun && !testStopped && ++teachTurns == teachStopAfterTurn) {
        testStopped = true; // every check after this ends the routine
        const lemlib::Pose at = chassis.getPose();
        std::printf("[auton] TEACH: stopped after turn %d (%s) at (%.2f, %.2f) heading %.1f. Push it by hand from "
                    "here, a tap of Y at each key step\n", teachTurns, what, at.x, at.y, at.theta);
        master.rumble("..");
    }
}

// Drive slowly forward along the straight line that runs through (x, y) in the direction `heading`, and stop when
// the robot is level with (x, y) (or, with stopOnPin, as soon as the claw sensor sees a pin). Both sides run at
// J67_FORWARD_RPM on the motors' own speed control; the heading wanted is `heading`, turned toward the line by
// J67_PIN_LINE_DEG_PER_IN for every inch the robot is to one side of it. HOLD stops it. Prints where it ended and,
// after that, a trace of the drive (one [pin] line per J67_TRACE_MS)
void lineDrive(const char* what, float x, float y, float heading, int maxMs, bool stopOnPin) {
    const float rad = lemlib::degToRad(heading);
    const lemlib::Pose to(x, y);
    struct Sample { std::uint16_t ms; float ahead, side, heading; };
    static Sample trace[J67_TRACE_MAX];
    int traceCount = 0;
    std::uint32_t traceAt = 0;
    float worstOff = 0;
    const char* why = "time limit";
    const std::uint32_t start = pros::millis();
    while (pros::millis() - start < static_cast<std::uint32_t>(maxMs)) {
        if (stopAsked()) {
            why = "stopped by Y";
            break;
        }
        if (stopOnPin) {
            const std::int32_t mm = clawSensor.get_distance();
            if (mm > 0 && mm <= subsystems::CLAW_CATCH_MM) {
                why = "sensor saw the pin";
                break;
            }
        }
        const lemlib::Pose now = chassis.getPose();
        // inches still to go along the line, and inches to the RIGHT of it (negative: left)
        const float ahead = (to.x - now.x) * std::sin(rad) + (to.y - now.y) * std::cos(rad);
        const float side = (now.x - to.x) * std::cos(rad) - (now.y - to.y) * std::sin(rad);
        if (ahead <= J67_LINE_STOP_EARLY_IN) { // HOLD takes about this far to stop it from the slow speed
            why = "reached the point";
            break;
        }
        const float want = heading - std::clamp(side * J67_PIN_LINE_DEG_PER_IN, -J67_PIN_LINE_MAX_DEG,
                                                J67_PIN_LINE_MAX_DEG);
        // off = heading minus the heading wanted, so a robot turned left of it (negative) gets more left side
        const float off = lemlib::angleError(now.theta, want, false);
        if (std::fabs(off) > std::fabs(worstOff)) worstOff = off;
        if (traceCount < J67_TRACE_MAX && pros::millis() - traceAt >= J67_TRACE_MS) {
            traceAt = pros::millis();
            trace[traceCount++] = {static_cast<std::uint16_t>(traceAt - start), ahead, side, now.theta};
        }
        // Speed: ramp up over J67_LINE_RAMP_MS, and slow down with the distance left, never under J67_LINE_MIN_RPM.
        // At a flat 120 rpm (2026-11-27) the left side started late, which swung the heading 22 deg left in the
        // first 0.3 s, and the robot ran 0.8 to 1.9 in past every point before HOLD stopped it. That, with the
        // steering fighting it, was the S-curve that knocked all three pieces over
        const float ramp = std::min(1.0f, static_cast<float>(pros::millis() - start) / J67_LINE_RAMP_MS);
        const float rpm = std::max(J67_LINE_MIN_RPM, std::min(J67_LINE_MAX_RPM * ramp, ahead * J67_LINE_RPM_PER_IN));
        // steering is scaled with the speed, so it cannot spin the robot round while it is barely moving
        const float steer = std::clamp(off * J67_FORWARD_HOLD_RPM_PER_DEG, -J67_FORWARD_HOLD_MAX_RPM,
                                       J67_FORWARD_HOLD_MAX_RPM) * (rpm / J67_LINE_MAX_RPM);
        leftMotors.move_velocity(rpm - steer);
        rightMotors.move_velocity(rpm + steer);
        pros::delay(10);
    }
    leftMotors.brake(); // HOLD
    rightMotors.brake();
    pros::delay(J67_STOP_MS); // let it stop before the next move reads the pose
    const lemlib::Pose end = chassis.getPose();
    std::printf("[auton] %s to (%.2f, %.2f) along %.1f: %s after %u ms; ended (%.2f, %.2f) heading %.1f, %.2f in from "
                "the point, worst steering error %.1f deg, sensor %d mm\n",
                what, x, y, heading, why, static_cast<unsigned>(pros::millis() - start), end.x, end.y, end.theta,
                end.distance(to), worstOff, static_cast<int>(clawSensor.get_distance()));
    for (int k = 0; k < traceCount; k++)
        std::printf("[pin] %s t=%u ahead=%.2f side=%+.2f heading=%.1f\n", what, static_cast<unsigned>(trace[k].ms),
                    trace[k].ahead, trace[k].side, trace[k].heading);
}

// Drive slowly forward `inches`, easing the heading round from `fromDeg` at the start to `toDeg` at the end (a
// gentle arc). Speed as lineDrive: ramped up, slowed with the distance left. The two sides are driven apart by
// what the turn itself needs (from the arc and J67_HALF_TRACK_IN) plus a correction for the heading error. HOLD
// stops it. Prints where it ended, then a trace (one [pin] line per J67_TRACE_MS: ms, inches left, heading wanted,
// heading)
void easeDrive(const char* what, float inches, float fromDeg, float toDeg, int maxMs) {
    const lemlib::Pose from = chassis.getPose();
    const float radPerIn = lemlib::degToRad(toDeg - fromDeg) / inches;
    struct Sample { std::uint16_t ms; float left, want, heading; };
    static Sample trace[J67_TRACE_MAX];
    int traceCount = 0;
    std::uint32_t traceAt = 0;
    float worstOff = 0;
    const char* why = "time limit";
    const std::uint32_t start = pros::millis();
    while (pros::millis() - start < static_cast<std::uint32_t>(maxMs)) {
        if (stopAsked()) {
            why = "stopped by Y";
            break;
        }
        const lemlib::Pose now = chassis.getPose();
        // progress along the heading it started on, SIGNED: going backward counts as negative. (It was the plain
        // distance from the start, so when the robot was shoved 2.5 in BACKWARD at the start of a push, which the
        // traces of 2026-11-27 show every time the intake comes on against the pin, that read as 2.5 in of
        // progress.)
        const float gone = (now.x - from.x) * std::sin(lemlib::degToRad(fromDeg)) +
                           (now.y - from.y) * std::cos(lemlib::degToRad(fromDeg));
        const float left = inches - gone;
        if (left <= J67_LINE_STOP_EARLY_IN) {
            why = "went the distance";
            break;
        }
        const float want = fromDeg + (toDeg - fromDeg) * std::clamp(gone / inches, 0.0f, 1.0f);
        const float off = lemlib::angleError(now.theta, want, false); // heading minus the heading wanted
        if (std::fabs(off) > std::fabs(worstOff)) worstOff = off;
        if (traceCount < J67_TRACE_MAX && pros::millis() - traceAt >= J67_TRACE_MS) {
            traceAt = pros::millis();
            trace[traceCount++] = {static_cast<std::uint16_t>(traceAt - start), left, want, now.theta};
        }
        const float ramp = std::min(1.0f, static_cast<float>(pros::millis() - start) / J67_LINE_RAMP_MS);
        const float rpm = std::max(J67_LINE_MIN_RPM, std::min(J67_LINE_MAX_RPM * ramp, left * J67_LINE_RPM_PER_IN));
        // + steer turns left (the heading falls). The arc's own share: a heading falling radPerIn per inch needs
        // the sides this far apart at this speed
        const float arc = -radPerIn * J67_HALF_TRACK_IN * rpm;
        const float fix = std::clamp(off * J67_FORWARD_HOLD_RPM_PER_DEG, -J67_FORWARD_HOLD_MAX_RPM,
                                     J67_FORWARD_HOLD_MAX_RPM) * (rpm / J67_LINE_MAX_RPM);
        leftMotors.move_velocity(rpm - arc - fix);
        rightMotors.move_velocity(rpm + arc + fix);
        pros::delay(10);
    }
    leftMotors.brake(); // HOLD
    rightMotors.brake();
    pros::delay(J67_STOP_MS);
    const lemlib::Pose end = chassis.getPose();
    std::printf("[auton] %s, %.1f in easing %.1f -> %.1f: %s after %u ms; went %.2f in, ended (%.2f, %.2f) heading %.1f, "
                "worst heading error %.1f deg, sensor %d mm\n",
                what, inches, fromDeg, toDeg, why, static_cast<unsigned>(pros::millis() - start), end.distance(from),
                end.x, end.y, end.theta, worstOff, static_cast<int>(clawSensor.get_distance()));
    for (int n = 0; n < traceCount; n++)
        std::printf("[pin] %s t=%u left=%.2f want=%.1f heading=%.1f\n", what, static_cast<unsigned>(trace[n].ms),
                    trace[n].left, trace[n].want, trace[n].heading);
}
} // namespace

// Qual routine 2
void james_67() {
    chassis.setPose(0, 0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD); // driver control sets it back to COAST when it starts

    // toggleB one full turn. The colour toggle is paused meanwhile so it doesn't fight it. (Spinning until the
    // optical sensor saw our alliance colour, subsystems::spinToAllianceColour(), was tried and swapped for this;
    // turning it during the first drive instead of before was tried and taken out)
    subsystems::pauseColourToggle(true);
    subsystems::spinTo(toggleB, toggleB.get_position() + START_TOGGLE_DEG, subsystems::rpm(100));
    subsystems::pauseColourToggle(false);

    // first goal. moveToPose doesn't wait, so the robot is already moving during the lift delay
    chassis.moveToPose(GOAL_X, GOAL_Y, GOAL_THETA, TO_GOAL_TIMEOUT_MS, {.forwards = true});
    pros::delay(LIFT_DELAY_MS);
    cascade.move_absolute(TO_GOAL_LIFT_DEG, subsystems::rpm(LIFT_PERCENT));
    waitPoseThenScore(GOAL_X, GOAL_Y, GOAL_THETA);
    finishScore(true); // the first goal keeps the full 0.3 s outtake before driving away
    // B's reset on the way back, once clear of the goal (RESET_AFTER_BACK_IN), before the first pin's grab. It
    // carries on during the grab (it doesn't use the claw); the lift to the second goal waits for it (liftDrive)
    driveBack(BACK_IN, BACK_TIMEOUT_MS, [] { macros::request(macros::Macro::ResetAll, true); }, RESET_AFTER_BACK_IN);

    // first pin
    const std::uint32_t closedAt = grabPin(PIN_X, PIN_Y, PIN_THETA, POSE_EXIT_WITHIN_IN, PIN12_CATCH_AFTER_MS, false);

    // second goal: GOAL2_LIFT_DEG (500 deg; was level 1 alliance height, 440)
    liftDrive(GOAL2_LIFT_DEG, GOAL2_X, GOAL2_Y, GOAL2_THETA, TO_GOAL2_TIMEOUT_MS, closedAt);
    finishScore();
    driveBack(BACK_IN, BACK_TIMEOUT_MS);

    // second pin
    const std::uint32_t closed2At = grabPin(PIN2_X, PIN2_Y, PIN2_THETA, POSE_EXIT_WITHIN_IN, PIN12_CATCH_AFTER_MS, false);

    // turn on the spot to face the goal first, so the drive there is a straight line: the one curved drive from
    // facing 47.5 to facing 180 was too tight a turn and ended in the wrong place (2026-11-25)
    chassis.turnToPoint(GOAL3_X, GOAL3_Y, RETURN_TURN_TIMEOUT_MS);
    chassis.waitUntilDone();

    // third goal: GOAL3_LIFT_DEG (700 deg; was level 2 alliance height, 640)
    liftDrive(GOAL3_LIFT_DEG, GOAL3_X, GOAL3_Y, GOAL3_THETA, TO_GOAL3_TIMEOUT_MS, closed2At);
    finishScore();

    // third pin, scored at the third goal's point at level 3 alliance height (900 deg)
    thirdPinCycle(subsystems::CASCADE_ALLIANCE[2], PIN3_X, PIN3_Y, GOAL3_X, GOAL3_Y);

    // the same again, at level 4 alliance height (1020 deg)
    thirdPinCycle(subsystems::CASCADE_ALLIANCE[3], PIN4_X, PIN4_Y, GOAL4_X, GOAL4_Y, LEVEL4_MAX_SPEED);

    // back up + B and turn again, then the fifth pin with L1's grab
    backUpAndTurn();
    const std::uint32_t closed5At = grabPin(PIN5_X, PIN5_Y, PIN3_THETA, POSE_EXIT_WITHIN_IN, PIN3_CATCH_AFTER_MS);

    // back off still intaking, then to the fifth goal lifting to 300 on the way, and score there. The claw stays
    // closed from the grab until the scoring motion opens it
    driveBack(PIN5_BACK_IN, BACK_TIMEOUT_MS);
    xMotors.brake();
    // turn on the spot to face the goal first, so the drive there is a straight line (as for pins 3 and 4)
    chassis.turnToPoint(GOAL5_X, GOAL5_Y, RETURN_TURN_TIMEOUT_MS);
    chassis.waitUntilDone();
    liftDrive(GOAL5_LIFT_DEG, GOAL5_X, GOAL5_Y, GOAL5_THETA, TO_GOAL5_TIMEOUT_MS, closed5At);
    finishScore();

    // sixth pin, at the third-pin spot: back off the goal (B once clear), face the pin, grab it
    while (macros::isBusy()) pros::delay(10); // the score's outtake is over first: B would cancel it
    driveBack(PIN6_BACK_IN, BACK_TIMEOUT_MS, [] { macros::request(macros::Macro::ResetAll, true); },
              RESET_AFTER_BACK_IN);
    // turn round on the spot to face 90, then the grab's pose drive to the pin. (Tried first, all slower or worse:
    // a point 12 in short of the pin, in line with it, reached by turn + straight drive + turn to 90; by one pose
    // drive from facing 270, which drove into the wall turning round; by a turn + pose drive, 2026-11-25)
    chassis.turnToHeading(PIN3_THETA, PIN6_TURN_TIMEOUT_MS);
    chassis.waitUntilDone();
    const std::uint32_t closed6At = grabPin(PIN6_X, PIN6_Y, PIN3_THETA, POSE_EXIT_WITHIN_IN, PIN3_CATCH_AFTER_MS, true,
                                            TO_PIN6_TIMEOUT_MS);

    // and score it the way pin 5 was: back off still intaking, face the fifth goal, straight there lifting to the
    // first pin's score height (GOAL2_LIFT_DEG), score
    driveBack(PIN5_BACK_IN, BACK_TIMEOUT_MS);
    xMotors.brake();
    chassis.turnToPoint(GOAL5_X, GOAL5_Y, RETURN_TURN_TIMEOUT_MS);
    chassis.waitUntilDone();
    liftDrive(GOAL2_LIFT_DEG, GOAL5_X, GOAL5_Y, GOAL5_THETA, TO_GOAL5_TIMEOUT_MS, closed6At);
    finishScore();

    // rest of the routine goes here
}

// Qual routine 3, "67james" on the selector (the default)
void james67b() {
    // twice: the odometry task can write over a setPose that lands in the middle of its update (umbmark.cpp)
    chassis.setPose(0, 0, 0);
    pros::delay(20);
    chassis.setPose(0, 0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD); // driver control sets it back to COAST when it starts

    // the wiggle: forward, back, forward again (the last one longer). (toggleA's 720 deg spin after it was tried
    // and taken out)
    const int powers[] = {WIGGLE_POWER, -WIGGLE_POWER, WIGGLE_POWER};
    const int times[] = {WIGGLE_MS, WIGGLE_MS, WIGGLE_LAST_MS};
    for (int i = 0; i < 3; ++i) {
        if (stopAsked()) return;
        leftMotors.move(powers[i]);
        rightMotors.move(powers[i]);
        pros::delay(times[i]);
    }
    leftMotors.brake(); // HOLD
    rightMotors.brake();
    if (teachPause("the wiggle")) return;

    // back off, then to the first goal lifting to 300 on the way, and score there
    driveBack(J67_BACK_IN, J67_BACK_TIMEOUT_MS);
    if (teachPause("the first back-off")) return;
    liftDrive(J67_GOAL1_LIFT_DEG, J67_GOAL1_X, J67_GOAL1_Y, J67_GOAL1_THETA, TO_J67_GOAL1_TIMEOUT_MS, pros::millis());
    finishScore(true); // the full 0.3 s outtake before backing off, as james_67's first goal
    if (teachPause("the drive to the goal and the score")) return;
    // THE WALL PIN, the way it was shown by hand on 2026-11-27 (Y-button marks, robot pushed from the start
    // position). The pin sits in a narrow gap ("the tunnel") with another piece just left of it. The robot comes
    // in from the side, facing along the wall, and turns left into the gap: the turn swings the robot's front-left
    // edge across and knocks the other piece away, and leaves the robot pointing up the tunnel. The marks:
    //   4  (9.25, -2.54) facing 86.9   alongside, about to turn        = J67_SIDE_*
    //   5  (9.73, -0.94) facing 36.6   turned: the edge has swept the other piece away
    //   6  (11.31, -0.90) facing 31.7  facing the pin                  = J67_MOUTH_*
    //   7  (11.80, -0.24) facing 31.3  pushed in with the intake spinning: the pin intakes  = J67_PIN1_*
    // Between 5 and 6 the robot was slid 1.4 in sideways by hand, which it cannot drive. So from mark 4 it turns
    // to face mark 6, drives the 2.6 in to it, turns to the tunnel's heading and drives in: the same poses at 4, 6
    // and 7, with the sweep made by the two turns. Mark 5 is not used.
    // Getting to mark 4 from the goal was not shown (the start of the recording stood in for it), so this is a
    // guess: straight back off the goal to x = J67_BACKOFF_X, across to a point J67_SIDE_RUNUP_IN short of mark 4,
    // turn to face along the wall, and run up to mark 4 on a straight line.
    // Tried before, all missed: back 9 in, turn to 45, 16 in forward with no steering (veered 4 to 10 deg left,
    // once swung 72 deg round its right side on the wall), a timed right-side nudge, a swing to 35 / 10, and a
    // straight drive up the line through (13.51, -0.13) at 30.9 (reached the point but 12 deg left of the
    // heading, and took the piece on the left)
    {
        const lemlib::Pose at = chassis.getPose();
        const std::uint32_t start = pros::millis();
        chassis.moveToPoint(J67_BACKOFF_X, at.y, J67_LINEUP_TIMEOUT_MS, {.forwards = false});
        waitMove(); // LemLib's own exit (0.5 in), not the 2 in early exit driveBack uses
        if (stopAsked()) return;
        const lemlib::Pose end = chassis.getPose();
        std::printf("[auton] back off the goal to x %.1f: ended (%.2f, %.2f) heading %.1f after %u ms\n",
                    J67_BACKOFF_X, end.x, end.y, end.theta, static_cast<unsigned>(pros::millis() - start));
    }
    const float sideRad = lemlib::degToRad(J67_SIDE_THETA);
    const float runupX = J67_SIDE_X - J67_SIDE_RUNUP_IN * std::sin(sideRad);
    const float runupY = J67_SIDE_Y - J67_SIDE_RUNUP_IN * std::cos(sideRad);
    {
        const lemlib::Pose at = chassis.getPose();
        settleTurn("to the run-up", lemlib::radToDeg(std::atan2(runupX - at.x, runupY - at.y)), J67_TURN1_TIMEOUT_MS);
        if (stopAsked()) return;
        const std::uint32_t start = pros::millis();
        chassis.moveToPoint(runupX, runupY, J67_RUNUP_TIMEOUT_MS, {.forwards = true, .maxSpeed = J67_RUNUP_MAX_SPEED});
        waitMove();
        if (stopAsked()) return;
        const lemlib::Pose end = chassis.getPose();
        std::printf("[auton] run-up point (%.2f, %.2f): ended (%.2f, %.2f) heading %.1f, %.2f in off, after %u ms\n",
                    runupX, runupY, end.x, end.y, end.theta, end.distance(lemlib::Pose(runupX, runupY)),
                    static_cast<unsigned>(pros::millis() - start));
    }
    settleTurn("along the wall", J67_SIDE_THETA, J67_TURN1_TIMEOUT_MS);
    if (stopAsked()) return;
    // FROM HERE: TAUGHT BY HAND (2026-11-27, second teaching run). The routine was stopped right here, after the
    // turn above, at (-1.37, -4.42) facing 87.0, and the robot was pushed through the rest with a tap of Y at each
    // step:
    //   1  (11.23, -2.20) facing 89.3   drives in
    //   2  (10.91, 0.60) facing 33.5    turns, and gets the pin out
    //   3  (11.89, 1.30) facing 23.0    gets the pin (the intake is spinning before this)
    //   4  (12.15, 1.98) facing 14.8    "proofing": a slight turn more, to make sure of the pin
    // From 1 to 4 the hand moved the robot 4.3 in almost straight (toward 12 deg) while turning it left through 75
    // deg. A robot cannot slide sideways like that, so it does the same thing as: stop the drive in at a point
    // 0.8 in short of mark 1 (J67_IN_*), turn on the spot to mark 2's heading, then one slow push forward of
    // J67_PUSH_IN that eases the heading round from mark 2's to mark 4's. That ends on mark 4 exactly and passes
    // within about 0.3 in of mark 3. Mark 2's position is the one thing it does not reproduce: the robot turns
    // 2.2 in short of it and 1.8 in to its right, and makes that up during the push.
    // What this replaces (untaught guesses of 2026-11-27): a drive to an "alongside" point, a turn toward a
    // "mouth" point, a slow drive to it, a turn "up the tunnel" and a slow drive in, which S-curved and knocked
    // all three pieces over.
    {
        const std::uint32_t start = pros::millis();
        chassis.moveToPoint(J67_IN_X, J67_IN_Y, J67_RUNUP_TIMEOUT_MS, {.forwards = true, .maxSpeed = J67_RUNUP_MAX_SPEED});
        waitMove();
        if (stopAsked()) return;
        const lemlib::Pose end = chassis.getPose();
        std::printf("[auton] drives in (%.2f, %.2f): ended (%.2f, %.2f) heading %.1f, %.2f in off, after %u ms\n",
                    J67_IN_X, J67_IN_Y, end.x, end.y, end.theta, end.distance(lemlib::Pose(J67_IN_X, J67_IN_Y)),
                    static_cast<unsigned>(pros::millis() - start));
    }
    // The claw CLOSES here, after the drive in, and stays closed through the turn and into the tunnel (asked for
    // on 2026-11-27 after the first run of this section, which went in with it open, L1's way). The scoring
    // motion left it open; that motion must be over first, or the two would fight over the claw
    while (macros::isRunning(macros::Macro::PlaceRelease) && !stopAsked()) pros::delay(10);
    if (stopAsked()) return;
    subsystems::setClaw(false);
    // The turn that gets the pin out goes J67_OUT_EXTRA_DEG further left than mark 2's heading, then comes back to
    // it before the push ("turn more, like 15 degrees more, before turning back and going forward to get the
    // pin", 2026-11-27). In a teaching run the stop comes after the first of the two
    settleTurn("gets the pin out", J67_OUT_THETA - J67_OUT_EXTRA_DEG, J67_TURN1_TIMEOUT_MS);
    if (stopAsked()) return;
    settleTurn("back to the tunnel", J67_OUT_THETA, J67_TURN1_TIMEOUT_MS);
    if (stopAsked()) return;
    // the intake spins from before mark 3, as shown
    xMotors.move_velocity(subsystems::rpm(100));
    // (2026-11-27, later: the points are now the fourth teaching run's, see J67_IN_* and J67_PIN1_*; the moves
    // are the same.)
    // INTO THE TUNNEL, INTAKING: taught on the third teaching run (2026-11-27). The routine stopped right here,
    // after the turn, at (10.08, -2.22) facing 34.2, and the robot was pushed in to where the pin intakes: one
    // mark, (13.46, 0.94) facing 29.3 = J67_PIN1_*. The hand went 3.3 in straight ahead and then slid about 1.6 in
    // to the right ("it's not necessarily a straight line"), which the robot cannot do. So: the drive in now goes
    // 0.5 in further, as asked (J67_IN_*), which takes up part of that; from the turn the robot drives slowly
    // along the straight line from where it is to the mark, steering onto it; and it finishes with a small turn
    // to the mark's heading (the earlier recording had the same small left turn at the end: "proofing").
    // What this replaces: one 4.3 in push easing the heading from 33.5 to 14.8 (second teaching run), which ran
    // out of time 3.5 in in, 16 deg past its heading: jammed.
    // (2026-11-27, 17:15: the push is now a plain straight push, J67_TUNNEL_PUSH_IN forward holding the heading
    // the turn left, measured from where the robot stands, with no steering at a point. The version above steered
    // along a line to an absolute point: its line was 13 deg off the robot's heading, so it swung to 55 deg and
    // back, and it relied on the pose right after a turn, which reads up to 0.75 in off.)
    // the push follows the straight line into mark 2 at J67_OUT_THETA, steering gently back onto it (the turn
    // was made 0.6 in left of it, see J67_IN_X), and ends J67_TUNNEL_PAST_IN past mark 2
    lineDrive("into the tunnel", J67_MARK2_X + J67_TUNNEL_PAST_IN * std::sin(lemlib::degToRad(J67_OUT_THETA)),
              J67_MARK2_Y + J67_TUNNEL_PAST_IN * std::cos(lemlib::degToRad(J67_OUT_THETA)), J67_OUT_THETA,
              J67_TUNNEL_PUSH_MAX_MS, false);
    if (stopAsked()) return;

    // grab, then turn to J67_TURN3_THETA (40), still intaking, then stop the intake. (A right-side-only nudge before
    // the grab was tried and taken out.)
    // (Tried and taken out: the early catch during the drive, closing straight after the drive, turning to 35 after
    // the grab, L1's grab at the 0 / 25 deg turn, a timed right-side nudge and a right-side swing before the grab)
    // the grab: close, and intake from here
    subsystems::setClaw(false);
    xMotors.move_velocity(subsystems::rpm(100));
    const std::uint32_t closedAt = pros::millis();
    std::printf("[auton] L1 grab after the nudge: claw closed (sensor %d mm)\n",
                static_cast<int>(clawSensor.get_distance()));
    // back to J67_TURN3_THETA with the right side only (the left holds). The nudge left the robot turned left of
    // it (about 20 deg), so the right side drives BACKWARD, which swings the robot clockwise, until the heading
    // reaches the target or J67_TURN3_TIMEOUT_MS is up. (A turn on the spot with both sides was tried first)
    {
        const float startHeading = chassis.getPose().theta;
        const std::uint32_t start = pros::millis();
        leftMotors.brake(); // HOLD
        rightMotors.move(-J67_TURN3_POWER);
        while (pros::millis() - start < J67_TURN3_TIMEOUT_MS && !stopAsked() &&
               lemlib::angleError(J67_TURN3_THETA, chassis.getPose().theta, false) > 0)
            pros::delay(10);
        rightMotors.brake();
        std::printf("[auton] right side back to %.0f: heading %.1f -> %.1f after %u ms\n", J67_TURN3_THETA,
                    startHeading, chassis.getPose().theta, static_cast<unsigned>(pros::millis() - start));
    }
    if (stopAsked()) return;
    // AFTER THE PIN: one more movement, shown by hand after a full run (2026-11-27, one Y mark, the robot pushed
    // on from where the routine ended): (14.51, 3.36) facing 9.3 = J67_AFTER_*, "it turns and drives a little".
    // From where the routine ends (about (12.4, 0.7) facing 40) that mark is 3.4 in almost straight ahead, with
    // the robot turned 31 deg left once there. By hand the two happened together; the robot drives the 3.4 in
    // first, holding the heading that points at the mark, and then turns on the spot to 9.3. Turning first and
    // then driving 3.4 in would end 1.7 in to the left of the mark. The intake runs all the way through
    {
        const lemlib::Pose at = chassis.getPose();
        const lemlib::Pose after(J67_AFTER_X, J67_AFTER_Y);
        const float toAfter = lemlib::radToDeg(std::atan2(after.x - at.x, after.y - at.y));
        settleTurn("to the last point", toAfter, J67_TURN1_TIMEOUT_MS); // a few degrees at most
        if (stopAsked()) return;
        easeDrive("drives a little", chassis.getPose().distance(after), toAfter, toAfter, J67_TUNNEL_PUSH_MAX_MS);
        if (stopAsked()) return;
        settleTurn("turns", J67_AFTER_THETA, J67_TURN1_TIMEOUT_MS);
        if (stopAsked()) return;
    }
    if (pros::millis() - closedAt < subsystems::CLAW_INTAKE_AFTER_MS)
        pros::delay(subsystems::CLAW_INTAKE_AFTER_MS - (pros::millis() - closedAt));
    xMotors.brake();

    // rest of the routine goes here
}
