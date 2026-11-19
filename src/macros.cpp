#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>

#include "main.h"
#include "macros.hpp"
#include "robot-config.hpp"
#include "subsystems.hpp"

using namespace subsystems;

namespace macros {

namespace {

std::atomic<Macro> pending{Macro::None};
std::atomic<bool> running{false};
std::atomic<bool> cancelled{false};
std::atomic<Macro> current{Macro::None}; // the macro running now, for isRunning()

// Which R2 press is next, 1..4: presses 1-3 go to cascade presets 1-3 (the Blocks
// "Height"), press 4 takes the cascade up to CASCADE_L4.
// It stays on 4 after that (no wrap back to preset 1): B starts it over
std::atomic<int> height{1};
// Which heights R2's presets 1-3 use: true = CASCADE_NEUTRAL, false = CASCADE_ALLIANCE (boot)
std::atomic<bool> neutralHeights{false};
// Which LEFT press is next: 1 = cascade to level 1, 2 = level 2 (and every press after it).
// RIGHT starts it over at 1
constexpr int LEFT_LEVELS = 2;
std::atomic<int> leftLevel{1};
// true once R2's 4th press has run (until B): R1 then does unloadFromTop() instead
std::atomic<bool> r2AtFourth{false};
// What X's next press does: true = deploy to X's position, false = stow. Starts true
// because the robot boots with the roller stowed (false made the first X stow it again)
std::atomic<bool> rollerAutoDeploy{true};
// Y was pressed during B's reset: finish taking the cascade down, but send the roller to Y
// instead of stowing it (the driver loop does the rest of Y)
std::atomic<bool> resetToY{false};
// which alliance height Y's next pickup ends at: 0 = L1, 1 = L2, then back to 0 (B leaves it alone)
constexpr int Y_LEVELS = 2;
std::atomic<int> yLevel{0};

pros::Task* task = nullptr;

bool aborted() { return cancelled.load(); }

/** @brief Sleep, but give up early on cancel. @return false if cancelled */
bool wait(int ms) {
    for (int t = 0; t < ms; t += 10) {
        if (aborted()) return false;
        pros::delay(10);
    }
    return !aborted();
}

bool moveTo(pros::AbstractMotor& motor, double target, int velocity = rpm(100)) {
    return spinTo(motor, target, velocity, aborted);
}

// --- the macros. Each mirrors one Blocks "when button pressed" stack --------

// Score (no button: R1 is Unload now). On cancel, rollerTurn is stopped here because nothing else drives it;
// the cascade is left alone because the driver has just taken it over.
// rollerMain keeps spinning out afterwards, as in Blocks (L1/L2 stop it)
void score() {
    rollerMain.move_velocity(-rpm(100));
    if (!wait(1000)) return;
    cascade.move_velocity(rpm(100));
    if (!wait(100)) return;
    rollerTurn.move_velocity(rpm(ROLLER_SPEED_PCT));
    const bool ok = wait(200);
    rollerTurn.brake();
    if (!ok || !wait(800)) return;
    cascade.brake();
}

// R1 after R2's 4th press: roller to X's position, outtake for 0.4 s, then keep
// outtaking while the roller goes up to 600 deg. The outtake stops at the end
void unloadFromTop() {
    constexpr double ROLLER_TO = 600; // rollerTurn position, degrees
    constexpr int OUTTAKE_FIRST_MS = 400;

    if (!moveTo(rollerTurn, ROLLER_DEPLOYED_DRIVER, rpm(ROLLER_SPEED_PCT)) && aborted()) return;
    intake(-100);
    if (!wait(OUTTAKE_FIRST_MS)) return intakeStop();
    rollerAutoDeploy = true; // up high like Y's position: next X goes to X's position
    if (!moveTo(rollerTurn, ROLLER_TO, rpm(ROLLER_SPEED_PCT)) && aborted()) return intakeStop();
    intakeStop();
}

// R1. (After R2's 4th press it does unloadFromTop() instead.) No drop: outtake (like holding L2)
// for 0.3 s, then keep outtaking while the cascade goes up R1_CASCADE_UP from where it is AND the
// roller goes to Y's position (ROLLER_Y), both at once.
// The outtake stops when both have arrived, or straight away if the macro is cancelled
void unload() {
    if (r2AtFourth) return unloadFromTop();
    rollerAutoDeploy = false; // the roller is out after this: next X stows it
    unloadMotion(aborted);
}

// R2. Presses 1-3: cascade to presets 1-3 (alliance or neutral heights), roller to X's
// deployed position, intake on. Press 4 (and every press after it, until B): cascade up to
// CASCADE_L4, nothing else moves; R1 does unloadFromTop() from then on
void cycleHeight() {
    // the count goes up as soon as R2 is pressed, not when the cascade gets there, so
    // quick presses (each one cutting in on the last) step straight up to the level asked for
    // the alliance set has CASCADE_ALLIANCE_COUNT heights, the neutral one CASCADE_PRESETS. Presses
    // past the last height stay there (no CASCADE_L4 step any more)
    const int heights = neutralHeights ? CASCADE_PRESETS : CASCADE_ALLIANCE_COUNT;
    const int step = std::min(height.load(), heights); // swapping sets mid-way can't overrun
    height = std::min(step + 1, heights);

    // only the lift moves: no intake, no roller (R2 used to start the intake and send the roller out)
    moveTo(cascade, (neutralHeights ? CASCADE_NEUTRAL : CASCADE_ALLIANCE)[step - 1]);
}

// X
void toggleRoller() {
    if (!rollerAutoDeploy) {
        rollerMain.brake();
        if (!homeRoller(aborted)) return;
        rollerAutoDeploy = true;
    } else {
        // rollerMain in at 40% (roller only) the whole way out, and for 1 s after the
        // roller gets there. The timer stops it, so the macro doesn't wait out that second
        rollerMainFor(40, ROLLER_SPIN_MS);
        if (!moveTo(rollerTurn, ROLLER_DEPLOYED_DRIVER, rpm(ROLLER_SPEED_PCT)) && aborted()) return;
        rollerAutoDeploy = false;
        rollerMainFor(40, 1000);
    }
}

// Right
void home() {
    if (!homeRoller(aborted)) return;
    moveTo(cascade, CASCADE_MIN);
}

// B. Cascade all the way down first, and only then stow the roller. If Y is pressed while this
// runs (resetY()), the cascade still goes all the way down but the roller goes to Y, not stowed
// B: drive the lift down until it sits on its bottom stop, then call that 0 degrees. Found by
// stalling (under HOME_STILL_DEG in HOME_STALL_MS, after HOME_SETTLE_MS to get going), not by the
// position reading, so a gear that slipped can't leave it stopping too high or pushing on: every B
// puts 0 back at the real bottom, and R2's heights line up again. Then it goes up to CASCADE_LOAD,
// the loading height (it doesn't wait for that). @return false if cancelled
bool homeCascade() {
    constexpr int HOME_DOWN_PCT = 100; // was 70
    constexpr int HOME_SETTLE_MS = 200;
    constexpr double HOME_STILL_DEG = 2;
    constexpr int HOME_STALL_MS = 100;
    constexpr int HOME_MAX_MS = 4000;

    cascade.move_velocity(-rpm(HOME_DOWN_PCT));
    const double startPos = cascade.get_position();
    double windowPos = startPos;
    int windowMs = 0;
    for (int t = 0; t < HOME_MAX_MS; t += 10) {
        if (aborted()) {
            cascade.brake();
            return false;
        }
        pros::delay(10);
        const double pos = cascade.get_position();
        if (t < HOME_SETTLE_MS || std::fabs(pos - windowPos) >= HOME_STILL_DEG) {
            windowPos = pos; // still moving
            windowMs = 0;
        } else if ((windowMs += 10) >= HOME_STALL_MS) {
            cascade.tare_position_all(); // on the bottom stop: this is 0
            cascade.move_absolute(CASCADE_LOAD, rpm(100)); // then up a little, to the loading height
            std::printf("[home] lift on the bottom after %d ms (read %.0f there): zeroed, up to %.0f\n", t, pos, CASCADE_LOAD);
            return true;
        }
    }
    cascade.brake();
    std::printf("[home] lift never stopped going down in %d ms: NOT zeroed\n", HOME_MAX_MS);
    return true;
}

void resetAll() {
    height = 1;
    r2AtFourth = false;
    rollerAutoDeploy = true;
    resetToY = false;
    if (!homeCascade()) return; // to the real bottom and re-zeroed there. Returns if the driver grabbed the cascade
    if (!resetToY) {
        rollerMain.brake();
        homeRoller([] { return aborted() || resetToY.load(); }); // Y mid-stow stops the stow too
    }
    // Y's move again: a stow cut short by Y braked the roller on its way out
    if (resetToY) rollerTurn.move_absolute(ROLLER_Y, rpm(ROLLER_SPEED_PCT));
}

// LEFT. First press: cascade to level 1 (CASCADE_STEP, R2's first preset). Every press after
// that: level 2. RIGHT starts the count over. Like R2, the count goes up as soon as the press
// is taken, so a quick second press (which cuts in on the first) goes straight to level 2
void cascadeLevel() {
    const int level = leftLevel.load();
    leftLevel = std::min(level + 1, LEFT_LEVELS);
    moveTo(cascade, CASCADE_STEP * level);
}

// No button (was LEFT). Intake on; cascade to the bottom (0 deg) FIRST, and only once it's there the roller
// goes to 55 below the original A position (ROLLER_A_POSITION - 55 = 205); 0.3 s there, then
// the roller goes to X's position. The intake runs the whole time and keeps running after the
// macro ends, until R1 (or L1/L2) stops it
// @return false if cancelled part way (the intake is stopped then)
bool pickUp() {
    if (!pickUpMotion(aborted)) return false;
    rollerAutoDeploy = false; // at X's position: next X stows
    return true;              // intake left running on purpose
}

// Y's pickup: PickUp, then the cascade up to an alliance-goal scoring height. The first Y
// pickup goes to alliance L1, the next to L2, then back to L1, whatever else happens in between
void pickUpY() {
    const int level = yLevel.load();
    yLevel = (level + 1) % Y_LEVELS; // counts as soon as it starts, like R2
    if (!pickUp()) return;
    moveTo(cascade, CASCADE_ALLIANCE[level]); // intake still running
}

// No button (was RIGHT). Outtakes the way R1 does, without R1's drop: outtake for 0.2 s, then keep
// outtaking while the cascade goes to CASCADE_RIGHT AND the roller to X's position, both
// at once. The outtake stops when both have arrived, or straight away if cancelled
void raiseRight() {
    intake(-100);
    if (!wait(UNLOAD_OUTTAKE_FIRST_MS)) return intakeStop();

    rollerTurn.move_absolute(ROLLER_DEPLOYED_DRIVER, rpm(ROLLER_SPEED_PCT)); // starts; moves while the cascade does
    rollerAutoDeploy = false;                                   // at X's position: next X stows
    if (!moveTo(cascade, CASCADE_RIGHT) && aborted()) return intakeStop();
    if (!moveTo(rollerTurn, ROLLER_DEPLOYED_DRIVER, rpm(ROLLER_SPEED_PCT)) && aborted()) return intakeStop(); // instant if already there
    intakeStop();
}

// R1: lower the lift until it can't go further (on a pin/cup, or the bottom stop), then let go (open
// the claw). It's down when the lift stops moving although it is still driven down: under
// PLACE_STILL_DEG in PLACE_STALL_MS, after PLACE_SETTLE_MS to get going (by position: the velocity
// reading is too noisy). Only PLACE_MAX_MS running out stops it WITHOUT opening the claw. (It used
// to stop at CASCADE_MIN without opening, which kept the claw shut on low cups)
void placeRelease() {
    constexpr int PLACE_DOWN_PCT = 100; // was 50
    constexpr int PLACE_SETTLE_MS = 200;
    constexpr double PLACE_STILL_DEG = 2;
    constexpr int PLACE_STALL_MS = 40; // was 80, then 150
    constexpr int PLACE_MAX_MS = 3000;
    constexpr int PLACE_OUTTAKE_MS = 300; // after letting go, outtake this long

    cascade.move_velocity(-rpm(PLACE_DOWN_PCT));
    const double startPos = cascade.get_position();
    double windowPos = startPos; // where it was at the start of the current still window
    int windowMs = 0;
    for (int t = 0; t < PLACE_MAX_MS; t += 10) {
        if (aborted()) { // the driver grabbed the lift
            cascade.brake();
            return;
        }
        pros::delay(10);
        const double pos = cascade.get_position();
        if (t < PLACE_SETTLE_MS || std::fabs(pos - windowPos) >= PLACE_STILL_DEG) {
            windowPos = pos; // still moving: start the window again from here
            windowMs = 0;
        } else if ((windowMs += 10) >= PLACE_STALL_MS) {
            cascade.brake(); // landed: stop and hold here (no more going down)...
            setClaw(true);   // ...let go of the pin...
            std::printf("[place] landed at %.0f (from %.0f) after %d ms: claw opened\n", pos, startPos, t);
            // ...and outtake with the claw's motors (14 and 16) for PLACE_OUTTAKE_MS
            xMotors.move_velocity(-rpm(100));
            for (int out = 0; out < PLACE_OUTTAKE_MS && !aborted(); out += 10) pros::delay(10);
            xMotors.brake();
            return;
        }
    }
    cascade.brake();
    std::printf("[place] timed out without landing: claw kept closed\n");
}

// Y / A stow (queued from the driver loop, which can't wait for the homing push)
void stowRoller() {
    rollerMain.brake(); // stowing stops the roller, as X's stow does
    rollerAutoDeploy = true;
    homeRoller(aborted);
}

void run(Macro macro) {
    switch (macro) {
        case Macro::Score: score(); break;
        case Macro::CycleHeight: cycleHeight(); break;
        case Macro::ToggleRoller: toggleRoller(); break;
        case Macro::ResetAll: resetAll(); break;
        case Macro::Home: home(); break;
        case Macro::Unload: unload(); break;
        case Macro::StowRoller: stowRoller(); break;
        case Macro::PickUp: pickUp(); break;
        case Macro::PickUpY: pickUpY(); break;
        case Macro::RaiseRight: raiseRight(); break;
        case Macro::CascadeLevel: cascadeLevel(); break;
        case Macro::PlaceRelease: placeRelease(); break;
        case Macro::None: break;
    }
}

void loop(void*) {
    while (true) {
        if (pending.load() == Macro::None) {
            pros::delay(10);
            continue;
        }
        // busy before the hand-off, so isBusy() is never briefly false while
        // a macro is being picked up (a second press would queue behind it)
        running = true;
        const Macro macro = pending.exchange(Macro::None); // None if cancel() won the race
        cancelled = false;
        current = macro;
        run(macro);
        current = Macro::None;
        running = false;
    }
}

} // namespace

void init() {
    if (task == nullptr) task = new pros::Task(loop, nullptr, "macros");
}

bool request(Macro macro, bool preempt) {
    if (preempt) {
        cancelled = true;
    } else if (isBusy()) {
        return false;
    }
    pending = macro;
    return true;
}

void cancel() {
    pending = Macro::None;
    cancelled = true;
}

bool isBusy() { return running.load() || pending.load() != Macro::None; }

bool resetY() {
    if (!isRunning(Macro::ResetAll)) return false;
    resetToY = true;
    return true;
}

bool isRunning(Macro macro) { return current.load() == macro || pending.load() == macro; }

void resetHeight() { height = 1; }

bool toggleNeutralHeights() {
    const bool now = !neutralHeights.load(); // only the driver loop writes it
    neutralHeights = now;
    return now;
}

void resetLeftLevel() { leftLevel = 1; }

void setXDeploysNext(bool deploy) { rollerAutoDeploy = deploy; }

} // namespace macros
