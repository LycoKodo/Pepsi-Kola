// Testbench stub of fmt/core.h, for lemlib/pose.cpp's format_as() only (the bench never calls it).
// Replaces each "{}" with the next argument and "{{"/"}}" with braces; no format specs.
#pragma once
#include <sstream>
#include <string>

namespace fmt {
namespace detail {
inline void format_rest(std::ostringstream& out, const char* f) {
    for (; *f; ++f) {
        if ((f[0] == '{' && f[1] == '{') || (f[0] == '}' && f[1] == '}')) out << *f++;
        else out << *f;
    }
}
template <typename A, typename... R> void format_rest(std::ostringstream& out, const char* f, const A& a, const R&... r) {
    for (; *f; ++f) {
        if ((f[0] == '{' && f[1] == '{') || (f[0] == '}' && f[1] == '}')) {
            out << *f++;
        } else if (f[0] == '{' && f[1] == '}') {
            out << a;
            format_rest(out, f + 2, r...);
            return;
        } else {
            out << *f;
        }
    }
}
} // namespace detail

template <typename... T> std::string format(const char* f, const T&... args) {
    std::ostringstream out;
    detail::format_rest(out, f, args...);
    return out.str();
}
} // namespace fmt
