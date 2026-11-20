// A mirror of LemLib's Chassis::moveToPose arithmetic, run by the bench once per DUT iteration from
// the same pose the DUT saw. It is NOT read out of the DUT: it re-does the stock formulas of
// dut_fix1/chassis/motions/moveToPose.cpp (carrot, close, errors, the PIDs, the max-speed clamp, slew,
// the maxSlipSpeed cap, the overturn subtraction, direction, minSpeed, the left/right ratio) with the
// DUT's own lemlib::PID, lemlib::ExitCondition, lemlib::getCurvature, lemlib::angleError, lemlib::slew
// and lemlib::Pose, in the same float expressions. Every iteration the bench compares the mirror's
// left/right commands with what the DUT actually sent (`mirrorOk`): identical for stock moveToPose;
// once moveToPose is reworked they will differ, and the stage columns describe stock behaviour only.
#pragma once
#include <cstdint>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/exitcondition.hpp"
#include "lemlib/pid.hpp"
#include "lemlib/pose.hpp"

namespace sim {

// which stage last changed lateralOut on its way from the PID to the motors
enum LatLimit { LIM_NONE = 0, LIM_MAXSPEED, LIM_SLEW, LIM_CAP, LIM_OVERTURN, LIM_DIRECTION, LIM_MINSPEED };
const char* latLimitName(int code);

struct PoseMirror {
        PoseMirror(const lemlib::ControllerSettings& lat, const lemlib::ControllerSettings& ang, float x, float y,
                   float thetaDeg, lemlib::MoveToPoseParams params, float drivetrainDrift);

        // one iteration; pose is Chassis::getPose(true, true), exactly what the DUT read
        void step(const lemlib::Pose& pose);

        // fixed
        lemlib::Pose target {0, 0, 0}; // standard form, as the DUT builds it
        lemlib::MoveToPoseParams params;
        float slewSetting;

        // state carried between iterations, as in the DUT
        lemlib::PID latPID, angPID;
        lemlib::ExitCondition latLarge, latSmall, angLarge, angSmall;
        bool close = false, lateralSettled = false, prevSameSide = false;
        float prevLateralOut = 0, prevAngularOut = 0;

        // this iteration
        float distTarget = 0;
        lemlib::Pose carrot {0, 0, 0};
        bool sameSide = false, wouldBreakEarly = false;
        float angularErrorRad = 0, lateralError = 0;
        float latPidOut = 0, angPidOut = 0;
        float latAfterMax = 0, latAfterSlew = 0, latAfterCap = 0, latAfterOverturn = 0, latAfterDir = 0, latFinal = 0;
        float angFinal = 0;
        float radius = 0, cap = 0, overturn = 0, maxSpeedNow = 0;
        int limit = LIM_NONE;
        float leftPower = 0, rightPower = 0;
        int32_t cmdL = 0, cmdR = 0; // what move() receives after float -> int truncation and the +-127 clamp
        bool exitsSatisfied = false; // the DUT's loop condition would end the motion after this iteration
};

} // namespace sim
