#pragma once

#include <string_view>

namespace devcmd {

enum class Verb {
    Pose,
    Ports,
    Motors,
    Gains,
    Motion,
    Sel,
    Bat,
    Ver,
    Help,
    Set,
    Arm,
    Disarm,
    Drive,
    Turn,
    MoveTo,
    Run,
    Stop,
    Stream,
    Invalid
};

enum class Axis { Lateral, Angular };
enum class Gain { KP, KI, KD };
enum class Topic { Pose, Pid, Motors, Off };

struct Command {
    Verb verb = Verb::Invalid;
    Axis axis{};
    Gain gain{};
    Topic topic{};
    float args[3]{};
    int argc = 0;
    int period = 0;
    int duration = 0;
    char name[24]{};
    char error[64]{};
};

Command parse(std::string_view line);

} // namespace devcmd
