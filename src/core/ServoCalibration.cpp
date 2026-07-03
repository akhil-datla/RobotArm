#include "ServoCalibration.h"

#include <cmath>

#include "MathUtils.h"

namespace roboarm {

ServoCalibration::ServoCalibration()
    : m_minUs(kRds3225MinPulseUs),
      m_maxUs(kRds3225MaxPulseUs),
      m_travelDeg(kServoTravelDeg),
      m_direction(1),
      m_offsetDeg(0.0f),
      m_minDeg(0.0f),
      m_maxDeg(kServoTravelDeg) {}

void ServoCalibration::setPulseRange(float minPulseUs, float maxPulseUs) {
    m_minUs = minPulseUs;
    m_maxUs = maxPulseUs;
}

void ServoCalibration::setTravel(float travelDeg) {
    if (travelDeg > 0.0f) {
        m_travelDeg = travelDeg;
    }
}

void ServoCalibration::setDirection(int direction) {
    m_direction = (direction < 0) ? -1 : 1;
}

void ServoCalibration::setOffset(float offsetDeg) { m_offsetDeg = offsetDeg; }

void ServoCalibration::setAngleLimits(float minDeg, float maxDeg) {
    m_minDeg = minDeg;
    m_maxDeg = maxDeg;
}

float ServoCalibration::clampAngle(float angleDeg) const {
    return clamp(angleDeg, m_minDeg, m_maxDeg);
}

float ServoCalibration::angleToPulseUs(float angleDeg) const {
    // 0. Final safety net: a NaN command (e.g. from a degenerate upstream calc)
    //    collapses to the mid-range angle so the servo is never sent garbage.
    if (angleDeg != angleDeg) {  // true only for NaN
        angleDeg = 0.5f * (m_minDeg + m_maxDeg);
    }
    // 1. Soft-limit the commanded angle so it can never leave the safe range.
    const float safeAngle = clampAngle(angleDeg);

    // 2. Apply the mounting offset.
    const float adjusted = safeAngle + m_offsetDeg;

    // 3. Linear map. frac is 0 at 0 deg and 1 at full travel. Direction -1 walks
    //    the pulse from the high end down instead of the low end up.
    const float frac = adjusted / m_travelDeg;
    const float span = m_maxUs - m_minUs;
    float pulse = (m_direction >= 0) ? (m_minUs + frac * span)
                                     : (m_maxUs - frac * span);

    // 4. Hard-clamp the pulse to the microsecond bounds (a final safety net).
    const float lo = fminf(m_minUs, m_maxUs);
    const float hi = fmaxf(m_minUs, m_maxUs);
    return clamp(pulse, lo, hi);
}

}  // namespace roboarm
