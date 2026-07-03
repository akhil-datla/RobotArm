// IClock.h — the timing seam.
//
// Pure-virtual interface (no Arduino). Returns "now" in microseconds. ArduinoClock
// returns micros(); FakeClock returns a value the test sets and advances by hand,
// so every PID/slew/profile behavior is deterministic in tests. See CLAUDE.md 5.1.
#pragma once

#include <cstdint>

namespace roboarm {

// A monotonic microsecond clock. Like Arduino's micros(), the value may wrap
// around at 2^32; callers use unsigned subtraction so a single wrap is harmless.
class IClock {
public:
    virtual ~IClock() = default;

    // Microseconds since some fixed start point.
    virtual uint32_t nowMicros() const = 0;
};

}  // namespace roboarm
