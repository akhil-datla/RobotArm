#include "PIDController.h"

#include <cmath>

#include "MathUtils.h"

namespace roboarm {

namespace {
constexpr float kInf = std::numeric_limits<float>::infinity();
}

PIDController::PIDController(float kP, float kI, float kD)
    : m_kp(kP),
      m_ki(kI),
      m_kd(kD),
      m_integral(0.0f),
      m_lastError(0.0f),
      m_hasPrev(false),
      m_atSetpoint(false),
      m_tolerance(0.0f),
      m_outMin(-kInf),
      m_outMax(kInf),
      m_iMin(-kInf),
      m_iMax(kInf) {}

void PIDController::setOutputLimits(float minOutput, float maxOutput) {
    m_outMin = minOutput;
    m_outMax = maxOutput;
}

void PIDController::setIntegratorLimits(float minIntegral, float maxIntegral) {
    m_iMin = minIntegral;
    m_iMax = maxIntegral;
}

void PIDController::reset() {
    m_integral = 0.0f;
    m_lastError = 0.0f;
    m_hasPrev = false;
    m_atSetpoint = false;
}

float PIDController::calculate(float setpoint, float measurement, float dtSeconds) {
    const float err = setpoint - measurement;

    // Derivative on the error signal. Skipped on the first call after a reset
    // (no history yet) and whenever dt is non-positive (no valid rate).
    float derivative = 0.0f;
    if (m_hasPrev && dtSeconds > 0.0f) {
        derivative = (err - m_lastError) / dtSeconds;
    }

    // Provisional integral: accumulate only for a positive dt.
    float integral = m_integral;
    if (dtSeconds > 0.0f) {
        integral += err * dtSeconds;
    }
    // Anti-windup, part 1: the explicit integrator limits.
    integral = clamp(integral, m_iMin, m_iMax);
    // Anti-windup, part 2: bound the integrator so its OWN contribution
    // (kI * integral) can never exceed the output range. This caps windup, and —
    // unlike folding the P/D terms back into the integrator — it can never flip
    // the integrator to the wrong sign, so a saturating P or D term does not
    // produce a reverse-windup spike (a full-scale wrong-way output) at the next
    // low-error step.
    if (m_ki != 0.0f) {
        const float a = m_outMin / m_ki;
        const float b = m_outMax / m_ki;
        integral = clamp(integral, fminf(a, b), fmaxf(a, b));
    }

    // Blend the three terms, then clamp the output to its limits.
    float output = m_kp * err + m_ki * integral + m_kd * derivative;
    output = clamp(output, m_outMin, m_outMax);

    // Commit state for the next step.
    m_integral = integral;
    m_lastError = err;
    m_hasPrev = true;
    m_atSetpoint = fabsf(err) <= m_tolerance;

    return output;
}

}  // namespace roboarm
