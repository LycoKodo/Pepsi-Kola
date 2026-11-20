#include <cmath>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"
#include "pros/misc.hpp"

void lemlib::Chassis::moveToPoint(float x, float y, int timeout, MoveToPointParams params, bool async) {
    params.earlyExitRange = fabs(params.earlyExitRange);
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { moveToPoint(x, y, timeout, params, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    // reset PIDs and exit conditions
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.reset();

    // initialize vars used between iterations
    Pose lastPose = getPose();
    distTraveled = 0;
    Timer timer(timeout);
    bool close = false;
    float prevLateralOut = 0; // previous lateral power
    float prevAngularOut = 0; // previous angular power
    const int compState = pros::competition::get_status();
    std::optional<bool> prevSide = std::nullopt;
    // Heading to hold while settling (see the loop). Empty when the move starts inside the settle zone
    std::optional<float> settleHeading = std::nullopt;
    bool firstIteration = true;

    // calculate target pose in standard form
    Pose target(x, y);
    target.theta = lastPose.angle(target);

    // main loop
    while (!timer.isDone() && ((!lateralSmallExit.getExit() && !lateralLargeExit.getExit()) || !close) &&
           this->motionRunning) {
        // update position
        const Pose pose = getPose(true, true);

        // update distance traveled
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        // calculate distance to the target point
        const float distTarget = pose.distance(target);

        // check if the robot is close enough to the target to start settling
        if (distTarget < 7.5 && close == false) {
            close = true;
            params.maxSpeed = fmax(fabs(prevLateralOut), 60);
            // Remember the direction to the target at this moment. From here on the robot steers to
            // this fixed heading, which is the straight line through the target, instead of steering
            // at the target point (that angle swings wildly as the distance goes to zero, which is why
            // stock LemLib switches steering off here). Without it nothing corrects a yaw picked up
            // while braking: on the robot, 48 in drives ended 5 to 6 deg off heading (2026-11-20).
            const float entryHeading = params.forwards ? pose.theta : pose.theta + M_PI;
            if (firstIteration) {
                // the move starts inside the zone: hold the heading the robot has now. The error starts
                // at zero, so the robot never turns toward the target (as before); it only resists a yaw
                settleHeading = entryHeading;
            } else if (std::fabs(radToDeg(angleError(entryHeading, pose.angle(target)))) < 10) {
                // drove into the zone already pointing (within 10 deg) along the line to the target
                settleHeading = pose.angle(target);
            }
            // otherwise (a turn-and-drive move that reaches the zone still swinging round) there is no
            // sensible heading to lock in, so steering stays off while settling, as in stock LemLib
        }
        firstIteration = false;

        // motion chaining
        const bool side =
            (pose.y - target.y) * -sin(target.theta) <= (pose.x - target.x) * cos(target.theta) + params.earlyExitRange;
        if (prevSide == std::nullopt) prevSide = side;
        const bool sameSide = side == prevSide;
        // exit if close
        if (!sameSide && params.minSpeed != 0) break;
        prevSide = side;

        // calculate error
        const float adjustedRobotTheta = params.forwards ? pose.theta : pose.theta + M_PI;
        const float angularError = settleHeading ? angleError(adjustedRobotTheta, *settleHeading)
                                                 : angleError(adjustedRobotTheta, pose.angle(target));
        float lateralError = pose.distance(target) * cos(angleError(pose.theta, pose.angle(target)));

        // update exit conditions
        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);

        // get output from PIDs
        float lateralOut = lateralPID.update(lateralError);
        float angularOut = angularPID.update(radToDeg(angularError));
        if (close && !settleHeading) angularOut = 0; // settling with no heading to hold: no steering, as before

        // apply restrictions on angular speed
        angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);
        angularOut = slew(angularOut, prevAngularOut, angularSettings.slew);

        // apply restrictions on lateral speed
        lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);
        // constrain lateral output by max accel
        // but not for decelerating, since that would interfere with settling
        if (!close) lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);

        // prevent moving in the wrong direction
        if (params.forwards && !close) lateralOut = std::fmax(lateralOut, 0);
        else if (!params.forwards && !close) lateralOut = std::fmin(lateralOut, 0);

        // constrain lateral output by the minimum speed
        if (params.forwards && lateralOut < fabs(params.minSpeed) && lateralOut > 0) lateralOut = fabs(params.minSpeed);
        if (!params.forwards && -lateralOut < fabs(params.minSpeed) && lateralOut < 0)
            lateralOut = -fabs(params.minSpeed);

        // update previous output
        prevAngularOut = angularOut;
        prevLateralOut = lateralOut;

        infoSink()->debug("Angular Out: {}, Lateral Out: {}", angularOut, lateralOut);

        // ratio the speeds to respect the max speed
        float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;
        const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / params.maxSpeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

        // move the drivetrain
        drivetrain.leftMotors->move(leftPower);
        drivetrain.rightMotors->move(rightPower);

        // delay to save resources
        pros::delay(10);
    }

    // stop the drivetrain
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->endMotion();
}