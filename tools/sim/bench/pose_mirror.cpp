// Mirror of stock Chassis::moveToPose (see pose_mirror.hpp). The expressions below are copied from
// dut_fix1/chassis/motions/moveToPose.cpp with the same types and order, so the floats come out the
// same; only the stage-by-stage bookkeeping is added.
#include "pose_mirror.hpp"
#include <algorithm>
#include <cmath>
#include "lemlib/util.hpp"

namespace sim {

const char* latLimitName(int code) {
    static const char* names[] = {"none", "maxSpeed", "slew", "cap", "overturn", "direction", "minSpeed"};
    return code >= 0 && code <= LIM_MINSPEED ? names[code] : "?";
}

PoseMirror::PoseMirror(const lemlib::ControllerSettings& lat, const lemlib::ControllerSettings& ang, float x, float y,
                       float theta, lemlib::MoveToPoseParams p, float drivetrainDrift)
    : params(p),
      slewSetting(lat.slew),
      latPID(lat.kP, lat.kI, lat.kD, lat.windupRange, true),
      angPID(ang.kP, ang.kI, ang.kD, ang.windupRange, true),
      latLarge(lat.largeError, lat.largeErrorTimeout),
      latSmall(lat.smallError, lat.smallErrorTimeout),
      angLarge(ang.largeError, ang.largeErrorTimeout),
      angSmall(ang.smallError, ang.smallErrorTimeout) {
    using namespace lemlib;
    latPID.reset();
    latLarge.reset();
    latSmall.reset();
    angPID.reset();
    angLarge.reset();
    angSmall.reset();
    target = Pose(x, y, M_PI_2 - degToRad(theta));
    if (!params.forwards) target.theta = fmod(target.theta + M_PI, 2 * M_PI);
    if (params.horizontalDrift == 0) params.horizontalDrift = drivetrainDrift;
}

void PoseMirror::step(const lemlib::Pose& pose) {
    using namespace lemlib;
    distTarget = pose.distance(target);
    if (distTarget < 7.5 && close == false) {
        close = true;
        params.maxSpeed = fmax(fabs(prevLateralOut), 60);
    }
    if (latLarge.getExit() && latSmall.getExit()) lateralSettled = true;

    carrot = target - Pose(cos(target.theta), sin(target.theta)) * params.lead * distTarget;
    if (close) carrot = target;

    const bool robotSide =
        (pose.y - target.y) * -sin(target.theta) <= (pose.x - target.x) * cos(target.theta) + params.earlyExitRange;
    const bool carrotSide =
        (carrot.y - target.y) * -sin(target.theta) <= (carrot.x - target.x) * cos(target.theta) + params.earlyExitRange;
    sameSide = robotSide == carrotSide;
    wouldBreakEarly = !sameSide && prevSameSide && close && params.minSpeed != 0;
    prevSameSide = sameSide;

    const float adjustedRobotTheta = params.forwards ? pose.theta : pose.theta + M_PI;
    const float angularError =
        close ? angleError(adjustedRobotTheta, target.theta) : angleError(adjustedRobotTheta, pose.angle(carrot));
    float latErr = pose.distance(carrot);
    if (close) latErr *= cos(angleError(pose.theta, pose.angle(carrot)));
    else latErr *= sgn(cos(angleError(pose.theta, pose.angle(carrot))));
    angularErrorRad = angularError;
    lateralError = latErr;

    latSmall.update(latErr);
    latLarge.update(latErr);
    angSmall.update(radToDeg(angularError));
    angLarge.update(radToDeg(angularError));

    float lateralOut = latPID.update(latErr);
    float angularOut = angPID.update(radToDeg(angularError));
    latPidOut = lateralOut;
    angPidOut = angularOut;
    maxSpeedNow = params.maxSpeed;
    limit = LIM_NONE;
    auto mark = [&](float before, float after, int code) { // slew's prev + (t - prev) can round: ignore that
        if (std::fabs(after - before) > 1e-3f) limit = code;
    };

    angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);
    float before = lateralOut;
    lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);
    mark(before, lateralOut, LIM_MAXSPEED);
    latAfterMax = lateralOut;
    before = lateralOut;
    if (!close) lateralOut = slew(lateralOut, prevLateralOut, slewSetting);
    mark(before, lateralOut, LIM_SLEW);
    latAfterSlew = lateralOut;

    const float rad = 1 / fabs(getCurvature(pose, carrot));
    const float maxSlipSpeed(sqrt(params.horizontalDrift * rad * 9.8));
    before = lateralOut;
    lateralOut = std::clamp(lateralOut, -maxSlipSpeed, maxSlipSpeed);
    mark(before, lateralOut, LIM_CAP);
    latAfterCap = lateralOut;
    radius = rad;
    cap = maxSlipSpeed;

    const float ot = fabs(angularOut) + fabs(lateralOut) - params.maxSpeed;
    before = lateralOut;
    if (ot > 0) lateralOut -= lateralOut > 0 ? ot : -ot;
    mark(before, lateralOut, LIM_OVERTURN);
    latAfterOverturn = lateralOut;
    overturn = ot;

    before = lateralOut;
    if (params.forwards && !close) lateralOut = std::fmax(lateralOut, 0);
    else if (!params.forwards && !close) lateralOut = std::fmin(lateralOut, 0);
    mark(before, lateralOut, LIM_DIRECTION);
    latAfterDir = lateralOut;

    before = lateralOut;
    if (params.forwards && lateralOut < fabs(params.minSpeed) && lateralOut > 0) lateralOut = fabs(params.minSpeed);
    if (!params.forwards && -lateralOut < fabs(params.minSpeed) && lateralOut < 0) lateralOut = -fabs(params.minSpeed);
    mark(before, lateralOut, LIM_MINSPEED);
    latFinal = lateralOut;
    angFinal = angularOut;

    prevAngularOut = angularOut;
    prevLateralOut = lateralOut;

    float lp = lateralOut + angularOut;
    float rp = lateralOut - angularOut;
    const float ratio = std::max(std::fabs(lp), std::fabs(rp)) / params.maxSpeed;
    if (ratio > 1) {
        lp /= ratio;
        rp /= ratio;
    }
    leftPower = lp;
    rightPower = rp;
    cmdL = std::clamp<int32_t>(int32_t(lp), -127, 127);
    cmdR = std::clamp<int32_t>(int32_t(rp), -127, 127);
    exitsSatisfied = close && lateralSettled && (angLarge.getExit() || angSmall.getExit());
}

} // namespace sim
