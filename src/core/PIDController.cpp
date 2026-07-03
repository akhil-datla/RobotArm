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

    // Provisional integral: accumulate only for a positive dt, then bound it to
    // the configured integrator limits (primary anti-windup).
    float integral = m_integral;
    if (dtSeconds > 0.0f) {
        integral += err * dtSeconds;
    }
    integral = clamp(integral, m_iMin, m_iMax);

    // Blend the three terms.
    float output = m_kp * err + m_ki * integral + m_kd * derivative;

    // Output-limit anti-windup: if we saturate, clamp the output and, when there
    // is an active integrator, back-calculate it so it holds exactly the value
    // the clamped output can deliver — it cannot wind up further.
    const float clamped = clamp(output, m_outMin, m_outMax);
    if (clamped != output && m_ki != 0.0f) {
        integral = (clamped - m_kp * err - m_kd * derivative) / m_ki;
        integral = clamp(integral, m_iMin, m_iMax);
    }
    output = clamped;

    // Commit state for the next step.
    m_integral = integral;
    m_lastError = err;
    m_hasPrev = true;
    m_atSetpoint = fabsf(err) <= m_tolerance;

    return output;
}

}  // namespace roboarm
