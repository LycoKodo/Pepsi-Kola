#include <cstdint>
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/util.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"

lemlib::TrackingWheel::TrackingWheel(pros::adi::Encoder* encoder, float wheelDiameter, float distance,
                                     float gearRatio) {
    this->encoder = encoder;
    this->diameter = wheelDiameter;
    this->distance = distance;
    this->gearRatio = gearRatio;
}

lemlib::TrackingWheel::TrackingWheel(pros::Rotation* encoder, float wheelDiameter, float distance, float gearRatio) {
    this->rotation = encoder;
    this->diameter = wheelDiameter;
    this->distance = distance;
    this->gearRatio = gearRatio;
}

lemlib::TrackingWheel::TrackingWheel(pros::MotorGroup* motors, float wheelDiameter, float distance, float rpm) {
    this->motors = motors;
    this->motors->set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
    this->diameter = wheelDiameter;
    this->distance = distance;
    this->rpm = rpm;
}

void lemlib::TrackingWheel::reset() {
    if (this->encoder != nullptr) this->encoder->reset();
    if (this->rotation != nullptr) {
        this->rotation->reset_position();
        // refresh every 5 ms (the sensor's fastest) instead of its default 20 ms, so the 10 ms odometry
        // loop gets a new reading every time. Done here because reset() runs from Chassis::calibrate(),
        // after PROS has started; a constructor of a global object would be too early
        this->rotation->set_data_rate(5);
        this->lastRotationDistance = 0;
        this->rotationOffset = 0;
        this->rotationFault = false;
    }
    if (this->motors != nullptr) this->motors->tare_position_all();
}

float lemlib::TrackingWheel::getDistanceTraveled() {
    if (this->encoder != nullptr) {
        return (float(this->encoder->get_value()) * this->diameter * M_PI / 360) / this->gearRatio;
    } else if (this->rotation != nullptr) {
        // An unplugged or dropped-out rotation sensor reads PROS_ERR (2147483647 centidegrees), which
        // used to become a 398,000 in jump in the pose. Hold the last good distance instead, and when
        // the sensor comes back carry on from there: only the travel during the dropout is lost
        const std::int32_t raw = this->rotation->get_position();
        if (raw == INT32_MAX) {
            this->rotationFault = true;
            return this->lastRotationDistance;
        }
        const float distance = (float(raw) * this->diameter * M_PI / 36000) / this->gearRatio;
        if (this->rotationFault) {
            this->rotationFault = false;
            this->rotationOffset = this->lastRotationDistance - distance;
            this->rotation->set_data_rate(5); // a sensor that restarted is back on its slow default
        }
        this->lastRotationDistance = distance + this->rotationOffset;
        return this->lastRotationDistance;
    } else if (this->motors != nullptr) {
        // get distance traveled by each motor
        std::vector<pros::MotorGears> gearsets = this->motors->get_gearing_all();
        std::vector<double> positions = this->motors->get_position_all();
        std::vector<float> distances;
        for (int i = 0; i < this->motors->size(); i++) {
            float in;
            switch (gearsets[i]) {
                case pros::MotorGears::red: in = 100; break;
                case pros::MotorGears::green: in = 200; break;
                case pros::MotorGears::blue: in = 600; break;
                default: in = 200; break;
            }
            distances.push_back(positions[i] * (diameter * M_PI) * (rpm / in));
        }
        return lemlib::avg(distances);
    } else {
        return 0;
    }
}

float lemlib::TrackingWheel::getOffset() { return this->distance; }

int lemlib::TrackingWheel::getType() {
    if (this->motors != nullptr) return 1;
    return 0;
}
