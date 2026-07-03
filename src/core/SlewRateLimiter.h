// SlewRateLimiter.h — cap how fast a value is allowed to change.
//
// THE IDEA: if you command a servo to jump from 20 deg to 120 deg, it slams
// there and the whole arm lurches. A slew-rate limiter feeds the target through
// gradually: each step it moves toward the target by at most (maxRate * dt), so
// the motion ramps smoothly. It is the simplest, safest kind of motion shaping.
//
// PURE CORE: no Arduino, no clock — you pass dt in seconds. See CLAUDE.md 3, 11.
#pragma once

namespace roboarm {

// Limits the rate of change of a signal to a maximum units-per-second.
class SlewRateLimiter {
public:
    // maxUnitsPerSec is the fastest the output may move (in the signal's units
    // per second, e.g. degrees/second for a joint).
    explicit SlewRateLimiter(float maxUnitsPerSec);

    // Advance one step toward `target`, moving at most maxRate*dt. Returns the
    // resulting (rate-limited) value.
    float calculate(float target, float dtSeconds);

    // Snap the internal value to `value` immediately (e.g. at startup so the
    // first move ramps from the real starting position).
    void reset(float value);

    // The current output value.
    float lastValue() const { return m_value; }

    // The configured maximum rate (units/second).
    float maxRate() const { return m_maxRate; }

private:
    float m_maxRate;
    float m_value;
};

}  // namespace roboarm
