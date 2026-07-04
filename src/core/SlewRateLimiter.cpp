#include "SlewRateLimiter.h"

#include "MathUtils.h"

namespace roboarm {

SlewRateLimiter::SlewRateLimiter() : m_maxRate(0.0f), m_value(0.0f) {}

SlewRateLimiter::SlewRateLimiter(float maxUnitsPerSec)
    : m_maxRate(maxUnitsPerSec), m_value(0.0f) {}

void SlewRateLimiter::reset(float value) { m_value = value; }

float SlewRateLimiter::calculate(float target, float dtSeconds) {
    if (dtSeconds <= 0.0f) {
        return m_value;  // no time has passed -> no movement
    }
    // Use the magnitude so a rate given with the wrong sign still ramps TOWARD
    // the target (a negative rate is meaningless) rather than driving away from it.
    const float maxStep = fabsf(m_maxRate * dtSeconds);
    // Move toward the target, but never by more than one step's worth. clamp on
    // the requested change caps both the rise and the fall (direction reversal
    // handled automatically).
    const float delta = clamp(target - m_value, -maxStep, maxStep);
    m_value += delta;
    return m_value;
}

}  // namespace roboarm
