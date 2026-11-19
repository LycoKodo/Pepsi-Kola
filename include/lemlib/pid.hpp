#pragma once

namespace lemlib {
class PID {
    public:
        /**
         * @brief Construct a new PID
         *
         * @param kP proportional gain
         * @param kI integral gain
         * @param kD derivative gain
         * @param windupRange integral anti windup range
         * @param signFlipReset whether to reset integral when sign of error flips
         *
         * @b Example
         * @code {.cpp}
         * // create a PID
         * PID pid(5, // kP
         *         0.01, // kI
         *         20, // kD
         *         5, // integral anti windup range
         *         false); // don't reset integral when sign of error flips
         * @endcode
         */
        PID(float kP, float kI, float kD, float windupRange = 0, bool signFlipReset = false);

        /**
         * @brief Update the PID
         *
         * @param error target minus position - AKA error
         * @return float output
         *
         * @b Example
         * @code {.cpp}
         * void opcontrol() {
         *     // create a PID
         *     PID pid(5, 0, 20);
         *     // give the pid a test input
         *     // the pid will then return an output
         *     float output = pid.update(10);
         * }
         * @endcode
         */
        float update(float error);

        /**
         * @brief reset integral, derivative, and prevTime
         *
         * @b Example
         * @code {.cpp}
         * void opcontrol() {
         *     // create a PID
         *     PID pid(5, 0, 20);
         *     // give the pid a test input
         *     // the pid will then return an output
         *     float output = pid.update(10);
         *     // reset the pid
         *     pid.reset();
         * }
         * @endcode
         */
        void reset();

        // inline on purpose: the prebuilt firmware/LemLib.a wins over out-of-line code in src/lemlib
        void setGains(float kP, float kI, float kD) {
            this->kP = kP;
            this->kI = kI;
            this->kD = kD;
        }

        float getKP() const { return kP; }

        float getKI() const { return kI; }

        float getKD() const { return kD; }
    protected:
        // gains (not const so they can be tuned at runtime; layout must match the prebuilt library)
        float kP;
        float kI;
        float kD;

        // optimizations
        const float windupRange;
        const bool signFlipReset;

        float integral = 0;
        float prevError = 0;
};
} // namespace lemlib