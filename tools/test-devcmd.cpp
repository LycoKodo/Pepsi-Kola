#include "devcmd.hpp"

#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>

namespace {

int failures = 0;

void check(bool cond, const char* msg) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", msg);
        ++failures;
    }
}

void expect_valid(std::string_view line, devcmd::Verb verb, const char* label) {
    devcmd::Command cmd = devcmd::parse(line);
    if (cmd.verb != verb) {
        std::fprintf(stderr, "FAIL: %s expected verb %d got %d err=%s\n", label,
                     static_cast<int>(verb), static_cast<int>(cmd.verb), cmd.error);
        ++failures;
    }
}

void expect_invalid_msg(std::string_view line, const char* want, const char* label) {
    devcmd::Command cmd = devcmd::parse(line);
    check(cmd.verb == devcmd::Verb::Invalid, label);
    check(std::strcmp(cmd.error, want) == 0, label);
}

void expect_set(std::string_view line, devcmd::Axis axis, devcmd::Gain gain, float value,
                const char* label) {
    devcmd::Command cmd = devcmd::parse(line);
    check(cmd.verb == devcmd::Verb::Set, label);
    check(cmd.axis == axis, label);
    check(cmd.gain == gain, label);
    check(cmd.args[0] == value, label);
    check(cmd.argc == 1, label);
}

void expect_drive(float inches, const char* label) {
    devcmd::Command cmd = devcmd::parse(std::string("drive ") + std::to_string(inches));
    check(cmd.verb == devcmd::Verb::Drive, label);
    check(cmd.args[0] == inches, label);
    check(cmd.argc == 1, label);
}

void expect_turn(float deg, const char* label) {
    devcmd::Command cmd = devcmd::parse(std::string("turn ") + std::to_string(deg));
    check(cmd.verb == devcmd::Verb::Turn, label);
    check(cmd.args[0] == deg, label);
    check(cmd.argc == 1, label);
}

void expect_moveto(float x, float y, float th, const char* label) {
    devcmd::Command cmd = devcmd::parse(
        std::string("moveto ") + std::to_string(x) + " " + std::to_string(y) + " " +
        std::to_string(th));
    check(cmd.verb == devcmd::Verb::MoveTo, label);
    check(cmd.args[0] == x && cmd.args[1] == y && cmd.args[2] == th, label);
    check(cmd.argc == 3, label);
}

void expect_run(const char* name, const char* label) {
    devcmd::Command cmd = devcmd::parse(std::string("run ") + name);
    check(cmd.verb == devcmd::Verb::Run, label);
    check(std::strcmp(cmd.name, name) == 0, label);
}

void expect_stream(devcmd::Topic topic, int period, int duration, const char* label) {
    devcmd::Command cmd = devcmd::parse(
        std::string("stream ") +
        (topic == devcmd::Topic::Pose   ? "pose"
         : topic == devcmd::Topic::Pid ? "pid"
                                       : "motors") +
        " " + std::to_string(period) + " " + std::to_string(duration));
    check(cmd.verb == devcmd::Verb::Stream, label);
    check(cmd.topic == topic, label);
    check(cmd.period == period, label);
    check(cmd.duration == duration, label);
}

void test_zero_arg_verbs() {
    expect_valid("pose", devcmd::Verb::Pose, "pose");
    expect_valid("ports", devcmd::Verb::Ports, "ports");
    expect_valid("motors", devcmd::Verb::Motors, "motors");
    expect_valid("gains", devcmd::Verb::Gains, "gains");
    expect_valid("motion", devcmd::Verb::Motion, "motion");
    expect_valid("sel", devcmd::Verb::Sel, "sel");
    expect_valid("bat", devcmd::Verb::Bat, "bat");
    expect_valid("ver", devcmd::Verb::Ver, "ver");
    expect_valid("help", devcmd::Verb::Help, "help");
    expect_valid("arm", devcmd::Verb::Arm, "arm");
    expect_valid("disarm", devcmd::Verb::Disarm, "disarm");
    expect_valid("stop", devcmd::Verb::Stop, "stop");
}

void test_set_combinations() {
    expect_set("set lat.kp 0", devcmd::Axis::Lateral, devcmd::Gain::KP, 0.f, "lat.kp");
    expect_set("set lat.ki 1.5", devcmd::Axis::Lateral, devcmd::Gain::KI, 1.5f, "lat.ki");
    expect_set("set lat.kd 1000", devcmd::Axis::Lateral, devcmd::Gain::KD, 1000.f, "lat.kd");
    expect_set("set ang.kp 14.25", devcmd::Axis::Angular, devcmd::Gain::KP, 14.25f, "ang.kp");
    expect_set("set ang.ki 0", devcmd::Axis::Angular, devcmd::Gain::KI, 0.f, "ang.ki");
    expect_set("set ang.kd 110", devcmd::Axis::Angular, devcmd::Gain::KD, 110.f, "ang.kd");
}

void test_motion_and_stream() {
    expect_drive(24.f, "drive");
    expect_drive(-72.f, "drive min");
    expect_drive(72.f, "drive max");
    expect_turn(90.f, "turn");
    expect_turn(-360.f, "turn min");
    expect_turn(360.f, "turn max");
    expect_moveto(1.f, -2.f, 180.f, "moveto");
    expect_moveto(-72.f, 72.f, -360.f, "moveto bounds");
    expect_run("qual3", "run");
    expect_run("skills1", "run skills");
    expect_stream(devcmd::Topic::Pose, 50, 8000, "stream pose");
    expect_stream(devcmd::Topic::Pid, 10, 1, "stream pid");
    expect_stream(devcmd::Topic::Motors, 1000, 60000, "stream motors");
    devcmd::Command off = devcmd::parse("stream off");
    check(off.verb == devcmd::Verb::Stream && off.topic == devcmd::Topic::Off, "stream off");
}

void test_malformed() {
    expect_invalid_msg("bogus", "unknown command", "unknown command");
    expect_invalid_msg("set foo.kp 1", "unknown axis", "unknown axis");
    expect_invalid_msg("set lat.xx 1", "unknown gain", "unknown gain");
    expect_invalid_msg("stream foo 50 100", "unknown topic", "unknown topic");
    expect_invalid_msg("", "empty line", "empty line");
    expect_invalid_msg("   ", "empty line", "whitespace only");
    expect_invalid_msg("pose extra", "too many arguments", "pose surplus");
    expect_invalid_msg("set lat.kp", "missing argument", "set missing value");
    expect_invalid_msg("set lat.kp 1 extra", "too many arguments", "set surplus");
    expect_invalid_msg("drive", "missing argument", "drive missing");
    expect_invalid_msg("drive x", "invalid number", "drive non-numeric");
    expect_invalid_msg("drive 1.2.3", "invalid number", "drive partial numeric");
    expect_invalid_msg("drive nan", "invalid number", "drive nan");
    expect_invalid_msg("drive inf", "invalid number", "drive inf");
    expect_invalid_msg("set lat.kp -1", "value out of range", "negative gain");
    expect_invalid_msg("set lat.kp 1000.1", "value out of range", "gain over 1000");
    expect_invalid_msg("turn", "missing argument", "turn missing");
    expect_invalid_msg("turn 361", "value out of range", "turn over");
    expect_invalid_msg("turn -361", "value out of range", "turn under");
    check(devcmd::parse("turn 360").verb == devcmd::Verb::Turn, "turn at max boundary");
    expect_invalid_msg("drive 73", "value out of range", "drive over");
    expect_invalid_msg("drive -73", "value out of range", "drive under");
    expect_invalid_msg("moveto 1 2", "missing argument", "moveto missing th");
    expect_invalid_msg("moveto 73 0 0", "value out of range", "moveto x over");
    expect_invalid_msg("moveto 0 -73 0", "value out of range", "moveto y under");
    expect_invalid_msg("moveto 0 0 361", "value out of range", "moveto th over");
    expect_invalid_msg("stream pose", "missing argument", "stream missing period");
    expect_invalid_msg("stream pose 50", "missing argument", "stream missing duration");
    expect_invalid_msg("stream pose 9 100", "period out of range", "stream period 9");
    expect_invalid_msg("stream pose 1001 100", "period out of range", "stream period 1001");
    expect_invalid_msg("stream pose 10 0", "duration out of range", "stream duration 0");
    expect_invalid_msg("stream pose 10 60001", "duration out of range", "stream duration 60001");
    expect_invalid_msg("run", "missing argument", "run missing");
    expect_invalid_msg("run ", "missing argument", "run empty name");
    expect_invalid_msg("run bad/name", "invalid name", "run bad char");
    expect_invalid_msg("run " + std::string(24, 'a'), "name too long", "run 24 chars");
    check(devcmd::parse("POSe").verb == devcmd::Verb::Pose, "mixed case verb pose");
    devcmd::Command mixed = devcmd::parse("DrIvE 6");
    check(mixed.verb == devcmd::Verb::Drive && mixed.args[0] == 6.f, "mixed case verb");
    check(devcmd::parse("drive  6").verb == devcmd::Verb::Drive, "repeated spaces");
    check(devcmd::parse("drive 6\r").verb == devcmd::Verb::Drive, "crlf strip");
    expect_invalid_msg(std::string(129, 'a'), "line too long", "129 bytes");
    check(devcmd::parse(std::string(128, 'a')).verb == devcmd::Verb::Invalid, "128 non-command");
    check(devcmd::parse("stream pose 10 1000").verb == devcmd::Verb::Stream, "stream bounds min");
    check(devcmd::parse("stream pose 1000 60000").verb == devcmd::Verb::Stream,
          "stream bounds max");
    check(devcmd::parse("turn 360").args[0] == 360.f, "turn at max");
    check(devcmd::parse("turn -360").args[0] == -360.f, "turn at min");
}

} // namespace

int main() {
    test_zero_arg_verbs();
    test_set_combinations();
    test_motion_and_stream();
    test_malformed();

    if (failures != 0) {
        std::fprintf(stderr, "%d test(s) failed\n", failures);
        return 1;
    }
    std::printf("all devcmd tests passed\n");
    return 0;
}
