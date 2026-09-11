#include "devcmd.hpp"

#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace devcmd {
namespace {

constexpr int kMaxLine = 128;

void set_error(Command& cmd, const char* msg) {
    cmd.verb = Verb::Invalid;
    std::strncpy(cmd.error, msg, sizeof(cmd.error) - 1);
    cmd.error[sizeof(cmd.error) - 1] = '\0';
}

bool is_space(char c) { return c == ' ' || c == '\t'; }

std::string_view strip_cr(std::string_view line) {
    if (!line.empty() && line.back() == '\r')
        return line.substr(0, line.size() - 1);
    return line;
}

bool all_whitespace(std::string_view s) {
    for (char c : s) {
        if (!is_space(c))
            return false;
    }
    return true;
}

char lower_char(char c) {
    if (c >= 'A' && c <= 'Z')
        return static_cast<char>(c - 'A' + 'a');
    return c;
}

bool iequals(std::string_view a, const char* b) {
    size_t i = 0;
    while (b[i] != '\0') {
        if (i >= a.size() || lower_char(a[i]) != b[i])
            return false;
        ++i;
    }
    return i == a.size();
}

bool next_token(std::string_view& rest, std::string_view& tok) {
    while (!rest.empty() && is_space(rest.front()))
        rest.remove_prefix(1);
    if (rest.empty()) {
        tok = {};
        return false;
    }
    size_t end = 0;
    while (end < rest.size() && !is_space(rest[end]))
        ++end;
    tok = rest.substr(0, end);
    rest.remove_prefix(end);
    return true;
}

bool only_whitespace_left(std::string_view rest) {
    for (char c : rest) {
        if (!is_space(c))
            return false;
    }
    return true;
}

bool require_no_more(std::string_view rest, Command& cmd) {
    if (!only_whitespace_left(rest)) {
        set_error(cmd, "too many arguments");
        return false;
    }
    return true;
}

bool parse_float(std::string_view tok, float& out) {
    char buf[64];
    if (tok.empty() || tok.size() >= sizeof(buf))
        return false;
    std::memcpy(buf, tok.data(), tok.size());
    buf[tok.size()] = '\0';
    char* end = nullptr;
    const double val = std::strtod(buf, &end);
    if (end == buf || *end != '\0')
        return false;
    if (!std::isfinite(val))
        return false;
    out = static_cast<float>(val);
    return true;
}

bool parse_int(std::string_view tok, int& out) {
    char buf[32];
    if (tok.empty() || tok.size() >= sizeof(buf))
        return false;
    std::memcpy(buf, tok.data(), tok.size());
    buf[tok.size()] = '\0';
    char* end = nullptr;
    errno = 0;
    const long val = std::strtol(buf, &end, 10);
    if (end == buf || *end != '\0')
        return false;
    if (errno == ERANGE)
        return false;
    if (val < INT_MIN || val > INT_MAX)
        return false;
    out = static_cast<int>(val);
    return true;
}

Verb parse_verb(std::string_view tok) {
    if (iequals(tok, "pose"))
        return Verb::Pose;
    if (iequals(tok, "ports"))
        return Verb::Ports;
    if (iequals(tok, "motors"))
        return Verb::Motors;
    if (iequals(tok, "gains"))
        return Verb::Gains;
    if (iequals(tok, "motion"))
        return Verb::Motion;
    if (iequals(tok, "sel"))
        return Verb::Sel;
    if (iequals(tok, "bat"))
        return Verb::Bat;
    if (iequals(tok, "ver"))
        return Verb::Ver;
    if (iequals(tok, "help"))
        return Verb::Help;
    if (iequals(tok, "set"))
        return Verb::Set;
    if (iequals(tok, "arm"))
        return Verb::Arm;
    if (iequals(tok, "disarm"))
        return Verb::Disarm;
    if (iequals(tok, "drive"))
        return Verb::Drive;
    if (iequals(tok, "turn"))
        return Verb::Turn;
    if (iequals(tok, "moveto"))
        return Verb::MoveTo;
    if (iequals(tok, "run"))
        return Verb::Run;
    if (iequals(tok, "stop"))
        return Verb::Stop;
    if (iequals(tok, "stream"))
        return Verb::Stream;
    return Verb::Invalid;
}

bool parse_set_target(std::string_view tok, Axis& axis, Gain& gain, Command& cmd) {
    const size_t dot = tok.find('.');
    if (dot == std::string_view::npos || dot == 0 || dot + 1 >= tok.size()) {
        set_error(cmd, "unknown axis");
        return false;
    }

    const std::string_view axis_part = tok.substr(0, dot);
    const std::string_view gain_part = tok.substr(dot + 1);

    if (axis_part == "lat")
        axis = Axis::Lateral;
    else if (axis_part == "ang")
        axis = Axis::Angular;
    else {
        set_error(cmd, "unknown axis");
        return false;
    }

    if (gain_part == "kp")
        gain = Gain::KP;
    else if (gain_part == "ki")
        gain = Gain::KI;
    else if (gain_part == "kd")
        gain = Gain::KD;
    else {
        set_error(cmd, "unknown gain");
        return false;
    }

    return true;
}

bool valid_run_char(char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
           c == '_' || c == '-';
}

bool copy_name(std::string_view tok, char* name, size_t cap, Command& cmd) {
    if (tok.empty()) {
        set_error(cmd, "missing argument");
        return false;
    }
    if (tok.size() >= cap) {
        set_error(cmd, "name too long");
        return false;
    }
    for (char c : tok) {
        if (!valid_run_char(c)) {
            set_error(cmd, "invalid name");
            return false;
        }
    }
    std::memcpy(name, tok.data(), tok.size());
    name[tok.size()] = '\0';
    return true;
}

bool parse_gain_value(std::string_view tok, float& out, Command& cmd) {
    if (!parse_float(tok, out)) {
        set_error(cmd, "invalid number");
        return false;
    }
    if (out < 0.f || out > 1000.f) {
        set_error(cmd, "value out of range");
        return false;
    }
    return true;
}

bool parse_motion_float(std::string_view tok, float lo, float hi, float& out, Command& cmd) {
    if (!parse_float(tok, out)) {
        set_error(cmd, "invalid number");
        return false;
    }
    if (out < lo || out > hi) {
        set_error(cmd, "value out of range");
        return false;
    }
    return true;
}

Topic parse_topic(std::string_view tok) {
    if (tok == "pose")
        return Topic::Pose;
    if (tok == "pid")
        return Topic::Pid;
    if (tok == "motors")
        return Topic::Motors;
    if (tok == "off")
        return Topic::Off;
    return static_cast<Topic>(-1);
}

bool parse_zero_arg(Verb verb, std::string_view rest, Command& cmd) {
    if (!require_no_more(rest, cmd))
        return false;
    cmd.verb = verb;
    return true;
}

} // namespace

Command parse(std::string_view line) {
    Command cmd;
    line = strip_cr(line);

    if (line.size() > kMaxLine) {
        set_error(cmd, "line too long");
        return cmd;
    }

    if (line.empty() || all_whitespace(line)) {
        set_error(cmd, "empty line");
        return cmd;
    }

    std::string_view rest = line;
    std::string_view verb_tok;
    if (!next_token(rest, verb_tok)) {
        set_error(cmd, "empty line");
        return cmd;
    }

    const Verb verb = parse_verb(verb_tok);
    if (verb == Verb::Invalid) {
        set_error(cmd, "unknown command");
        return cmd;
    }

    switch (verb) {
    case Verb::Pose:
    case Verb::Ports:
    case Verb::Motors:
    case Verb::Gains:
    case Verb::Motion:
    case Verb::Sel:
    case Verb::Bat:
    case Verb::Ver:
    case Verb::Help:
    case Verb::Arm:
    case Verb::Disarm:
    case Verb::Stop:
        if (parse_zero_arg(verb, rest, cmd))
            return cmd;
        return cmd;

    case Verb::Set: {
        std::string_view target_tok;
        if (!next_token(rest, target_tok)) {
            set_error(cmd, "missing argument");
            return cmd;
        }
        if (!parse_set_target(target_tok, cmd.axis, cmd.gain, cmd))
            return cmd;
        std::string_view value_tok;
        if (!next_token(rest, value_tok)) {
            set_error(cmd, "missing argument");
            return cmd;
        }
        if (!parse_gain_value(value_tok, cmd.args[0], cmd))
            return cmd;
        if (!require_no_more(rest, cmd))
            return cmd;
        cmd.verb = Verb::Set;
        cmd.argc = 1;
        return cmd;
    }

    case Verb::Drive: {
        std::string_view arg_tok;
        if (!next_token(rest, arg_tok)) {
            set_error(cmd, "missing argument");
            return cmd;
        }
        if (!parse_motion_float(arg_tok, -72.f, 72.f, cmd.args[0], cmd))
            return cmd;
        if (!require_no_more(rest, cmd))
            return cmd;
        cmd.verb = Verb::Drive;
        cmd.argc = 1;
        return cmd;
    }

    case Verb::Turn: {
        std::string_view arg_tok;
        if (!next_token(rest, arg_tok)) {
            set_error(cmd, "missing argument");
            return cmd;
        }
        if (!parse_motion_float(arg_tok, -360.f, 360.f, cmd.args[0], cmd))
            return cmd;
        if (!require_no_more(rest, cmd))
            return cmd;
        cmd.verb = Verb::Turn;
        cmd.argc = 1;
        return cmd;
    }

    case Verb::MoveTo: {
        for (int i = 0; i < 3; ++i) {
            std::string_view arg_tok;
            if (!next_token(rest, arg_tok)) {
                set_error(cmd, "missing argument");
                return cmd;
            }
            const float lo = (i < 2) ? -72.f : -360.f;
            const float hi = (i < 2) ? 72.f : 360.f;
            if (!parse_motion_float(arg_tok, lo, hi, cmd.args[i], cmd))
                return cmd;
        }
        if (!require_no_more(rest, cmd))
            return cmd;
        cmd.verb = Verb::MoveTo;
        cmd.argc = 3;
        return cmd;
    }

    case Verb::Run: {
        std::string_view name_tok;
        if (!next_token(rest, name_tok)) {
            set_error(cmd, "missing argument");
            return cmd;
        }
        if (!copy_name(name_tok, cmd.name, sizeof(cmd.name), cmd))
            return cmd;
        if (!require_no_more(rest, cmd))
            return cmd;
        cmd.verb = Verb::Run;
        return cmd;
    }

    case Verb::Stream: {
        std::string_view topic_tok;
        if (!next_token(rest, topic_tok)) {
            set_error(cmd, "missing argument");
            return cmd;
        }

        const Topic topic = parse_topic(topic_tok);
        if (topic == static_cast<Topic>(-1)) {
            set_error(cmd, "unknown topic");
            return cmd;
        }

        if (topic == Topic::Off) {
            if (!require_no_more(rest, cmd))
                return cmd;
            cmd.verb = Verb::Stream;
            cmd.topic = Topic::Off;
            return cmd;
        }

        std::string_view period_tok;
        if (!next_token(rest, period_tok)) {
            set_error(cmd, "missing argument");
            return cmd;
        }
        if (!parse_int(period_tok, cmd.period)) {
            set_error(cmd, "invalid number");
            return cmd;
        }
        if (cmd.period < 10 || cmd.period > 1000) {
            set_error(cmd, "period out of range");
            return cmd;
        }

        std::string_view duration_tok;
        if (!next_token(rest, duration_tok)) {
            set_error(cmd, "missing argument");
            return cmd;
        }
        if (!parse_int(duration_tok, cmd.duration)) {
            set_error(cmd, "invalid number");
            return cmd;
        }
        if (cmd.duration < 1 || cmd.duration > 60000) {
            set_error(cmd, "duration out of range");
            return cmd;
        }

        if (!require_no_more(rest, cmd))
            return cmd;
        cmd.verb = Verb::Stream;
        cmd.topic = topic;
        return cmd;
    }

    case Verb::Invalid:
        break;
    }

    set_error(cmd, "unknown command");
    return cmd;
}

} // namespace devcmd
