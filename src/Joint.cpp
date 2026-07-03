#include "Joint.h"

#include "MathUtils.h"

namespace roboarm {

Joint::Joint()
    : m_calib(),
      m_output(nullptr),
      m_clock(nullptr),
      m_slew(nullptr),
      m_pid(nullptr),
      m_targetDeg(kJointDefaultAngleDeg),
      m_currentDeg(kJointDefaultAngleDeg),
      m_feedbackDeg(0.0f),
      m_lastMicros(0),
      m_haveLast(false) {}

Joint::Joint(IServoOutput& output, IClock& clock) : Joint() {
    attach(output, clock);
}

#ifdef ARDUINO
Joint::Joint(ServoDriver& board, uint8_t channel) : Joint() {
    m_ownedOutput.attach(board, channel);
    attach(m_ownedOutput, sharedArduinoClock());
}
#endif

void Joint::attach(IServoOutput& output, IClock& clock) {
    m_output = &output;
    m_clock = &clock;
    m_haveLast = false;  // re-baseline dt on the next update
}

void Joint::setPulseRange(float minPulseUs, float maxPulseUs) {
    m_calib.setPulseRange(minPulseUs, maxPulseUs);
}

void Joint::setTravel(float travelDeg) { m_calib.setTravel(travelDeg); }

void Joint::setLimits(float minDeg, float maxDeg) {
    m_calib.setAngleLimits(minDeg, maxDeg);
    // Keep the target inside the (possibly narrower) limits.
    m_targetDeg = m_calib.clampAngle(m_targetDeg);
}

void Joint::setDirection(int direction) { m_calib.setDirection(direction); }

void Joint::setOffset(float offsetDeg) { m_calib.setOffset(offsetDeg); }

void Joint::setCalibration(const ServoCalibration& calibration) {
    m_calib = calibration;
    m_targetDeg = m_calib.clampAngle(m_targetDeg);
}

void Joint::setAngle(float angleDeg) {
    m_targetDeg = m_calib.clampAngle(angleDeg);
}

void Joint::useSmoothing(SlewRateLimiter& limiter) {
    m_slew = &limiter;
    m_pid = nullptr;  // smoothing and PID feedback are alternative modes
    m_slew->reset(m_currentDeg);
}

void Joint::usePID(PIDController& pid) {
    m_pid = &pid;
    m_slew = nullptr;
}

void Joint::update() {
    if (!m_output || !m_clock) {
        return;  // unattached: safe no-op
    }

    // Derive dt from the clock. Unsigned subtraction tolerates a single wrap of
    // micros() at 2^32. The first update after (re)attaching just baselines.
    const uint32_t now = m_clock->nowMicros();
    float dt = 0.0f;
    if (m_haveLast) {
        dt = static_cast<float>(now - m_lastMicros) * 1.0e-6f;
    }
    m_lastMicros = now;
    m_haveLast = true;

    if (m_pid) {
        // Closed-loop: the PID output is the commanded angle.
        const float cmd = m_pid->calculate(m_targetDeg, m_feedbackDeg, dt);
        m_currentDeg = m_calib.clampAngle(cmd);
    } else if (m_slew) {
        // Smoothed: ramp toward the target at the limiter's rate.
        m_currentDeg = m_calib.clampAngle(m_slew->calculate(m_targetDeg, dt));
    } else {
        // Direct: jump to the (already clamped) target.
        m_currentDeg = m_targetDeg;
    }

    m_output->writeMicroseconds(m_calib.angleToPulseUs(m_currentDeg));
}

}  // namespace roboarm
