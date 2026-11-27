#ifndef MACROS_HPP
#define MACROS_HPP

/**
 * @file macros.hpp
 * @brief Multi-step driver macros, ported from the Blocks project.
 *
 * Macros contain waits, and the driver loop must never wait, so they run on
 * one dedicated task. One macro runs at a time:
 *
 *   - request() while another macro is running is ignored (returns false),
 *     unless preempt is true, in which case the running one is cancelled first.
 *   - cancel() stops the running macro at its next wait. Teleop calls it when
 *     the driver takes the cascade manually, so a macro never fights the stick.
 *
 * | Button | Macro          | What it does                                         |
 * |--------|----------------|------------------------------------------------------|
 * | R1     | Unload         | outtake 0.3s, then outtake while cascade up 75 +     |
 * |        |                | roller to 550 (Y) together (no drop first). After    |
 * |        |                | R2's 4th press: roller to 340, outtake 0.4s, to 600  |
 * | (none) | Score          | roller out 1s, cascade up 0.9s, flick rollerTurn     |
 * |        |                | (was R1)                                             |
 * | R2     | CycleHeight    | presses 1-3: cascade presets 1-3 + roller deployed   |
 * |        |                | (roller holds if cascade up + roller already out).   |
 * |        |                | Alliance 490/800/1080 (boot) or neutral 600/900/1190|
 * |        |                | hold A 1s to swap. Press 4 on: cascade to 1390 only  |
 * | X      | ToggleRoller   | stow roller + arm auto-deploy  /  deploy roller,     |
 * |        |                | rollerMain 40% the whole way and 1s after            |
 * | B      | ResetAll       | presets back to 1, arm auto-deploy, everything home  |
 * |        |                | (Y during it: cascade still down, roller to Y)       |
 * | LEFT   | PickUp         | intaking: cascade to 0, THEN roller to 205; 0.3s     |
 * |        |                | there, then roller to X                               |
 * | RIGHT  | RaiseRight     | outtake 0.2s, then keep outtaking while cascade to   |
 * |        |                | 240 + roller to X                                    |
 * | (none) | Home           | roller stowed, cascade home (no button)              |
 */
namespace macros {

enum class Macro { None, Score, CycleHeight, ToggleRoller, ResetAll, Home, Unload, StowRoller, PickUp, RaiseRight, CascadeLevel, PickUpY, PlaceRelease };

/** @brief Start the macro task. Call once, in initialize() */
void init();

/** @brief Queue a macro. @return false if ignored because one is already running */
bool request(Macro macro, bool preempt = false);

/** @brief Stop the running macro, if any, at its next wait */
void cancel();

/** @brief True while a macro is queued or running */
bool isBusy();

/**
 * @brief Y pressed: if B's reset is running, let it finish taking the cascade down but send the
 * roller to Y instead of stowing it. @return false if no reset is running (do Y as usual)
 */
bool resetY();

/** @brief True while `macro` is queued or running */
bool isRunning(Macro macro);

/** @brief Next R2 preset goes back to 1 (the Blocks "Height = 1") */
void resetHeight();

/**
 * @brief How fast R1's PlaceRelease drives the lift down, percent, from its next run on. Driver control's R1 sets
 * 80, the autons 100 (src/controls.cpp, src/autons.cpp)
 */
void setPlaceDownPercent(int percent);
/** @brief Swap R2's heights between neutral and alliance goal (A held 1 s). @return true = neutral now */
bool toggleNeutralHeights();
/** @brief Next LEFT press goes back to cascade level 1 (RIGHT) */
void resetLeftLevel();
/**
 * @brief The driver moved the roller outside a macro: set what X's next press does.
 * @param deploy true: next X deploys (roller stowed, or at the A position).
 *               false: next X stows (roller fully out, e.g. after UP)
 */
void setXDeploysNext(bool deploy);

} // namespace macros

#endif // MACROS_HPP
