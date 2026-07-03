#include "Gripper.h"

#include "core/MathUtils.h"

namespace roboarm {

Gripper::Gripper()
    : Joint(), m_openDeg(kDefaultOpenDeg), m_closeDeg(kDefaultCloseDeg) {
    configure(kDefaultOpenDeg, kDefaultCloseDeg);
}

Gripper::Gripper(IServoOutput& output, IClock& clock, float openDeg,
                 float closeDeg)
    : Joint(output, clock), m_openDeg(openDeg), m_closeDeg(closeDeg) {
    configure(openDeg, closeDeg);
}

#ifdef ARDUINO
Gripper::Gripper(ServoDriver& board, uint8_t channel, float openDeg, float closeDeg)
    : Joint(board, channel), m_openDeg(openDeg), m_closeDeg(closeDeg) {
    configure(openDeg, closeDeg);
}
#endif

void Gripper::configure(float openDeg, float closeDeg) {
    m_openDeg = openDeg;
    m_closeDeg = closeDeg;
    // The soft limits must span both named positions so neither is clamped away.
    const float lo = (openDeg < closeDeg) ? openDeg : closeDeg;
    const float hi = (openDeg < closeDeg) ? closeDeg : openDeg;
    setLimits(lo, hi);
}

void Gripper::open() {
    setAngle(m_openDeg);
    update();
}

void Gripper::close() {
    setAngle(m_closeDeg);
    update();
}

void Gripper::set(float fraction) {
    const float t = clamp(fraction, 0.0f, 1.0f);
    setAngle(lerp(m_openDeg, m_closeDeg, t));
    update();
}

}  // namespace roboarm
