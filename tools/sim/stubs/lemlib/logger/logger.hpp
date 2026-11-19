// Testbench stub of lemlib/logger/logger.hpp: every sink is a no-op, so fmt is not needed.
// It sits earlier on the include path than include/lemlib/logger/logger.hpp.
#pragma once

namespace lemlib {
class NullSink {
    public:
        template <typename... T> void debug(const char*, T&&...) {}
        template <typename... T> void info(const char*, T&&...) {}
        template <typename... T> void warn(const char*, T&&...) {}
        template <typename... T> void error(const char*, T&&...) {}
        template <typename... T> void fatal(const char*, T&&...) {}
};

inline NullSink* infoSink() {
    static NullSink sink;
    return &sink;
}

inline NullSink* telemetrySink() {
    static NullSink sink;
    return &sink;
}
} // namespace lemlib
