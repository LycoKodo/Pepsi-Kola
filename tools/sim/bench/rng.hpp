// A small random number generator for the simulator, so that a seed always gives exactly the same run.
// The standard library's distributions (std::normal_distribution and friends) are not used: each compiler's
// library may compute them differently. std::mt19937_64 itself is defined exactly by the C++ standard.
#pragma once
#include <cmath>
#include <cstdint>
#include <random>

namespace sim {

class Rng {
    public:
        explicit Rng(std::uint64_t seed = 1) { reseed(seed); }

        void reseed(std::uint64_t seed) { engine.seed(seed); }

        // a number from 0 up to (not including) 1
        double uniform() { return double(engine() >> 11) * (1.0 / 9007199254740992.0); }

        // a number from lo up to hi
        double uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }

        // a normally distributed number, mean 0, standard deviation 1 (Box-Muller). It always uses exactly two
        // uniform numbers, so the stream of numbers stays in step whatever the settings are
        double normal() {
            const double u1 = uniform();
            const double u2 = uniform();
            return std::sqrt(-2.0 * std::log(1.0 - u1)) * std::cos(2.0 * M_PI * u2);
        }

    private:
        std::mt19937_64 engine;
};

} // namespace sim
