// PIDController.h — a standalone, WPILib-style PID controller.
//
// THE IDEA: a PID controller nudges a system toward a target ("setpoint"). It
// looks at the error (how far off you are), and blends three responses:
//   P — push proportional to the error (the bigger the miss, the harder you push)
//   I — push based on the accumulated error over time (kills steady offset)
//   D — push against how fast the error is changing (damps overshoot)
// You call calculate(setpoint, measurement, dt) every loop and apply the result.
//
// PURE CORE: no Arduino, no clock. You pass dt in seconds yourself, which is what
// makes the behavior fully deterministic and host-testable. See CLAUDE.md 2, 11.
#pragma once

#include <limits>

namespace roboarm {

// A classic proportional-integral-derivative controller.
//
// Conventions (documented and tested):
//  - error = setpoint - measurement.
//  - The derivative is taken on the *error* signal, and is zero on the first
//    calculate() after construction/reset (no "derivative kick").
//  - Anti-windup: the integrator is bounded by setIntegratorLimits, and its own
//    contribution (kI * integral) is additionally bounded to the output range, so
//    it can never wind up — or reverse-wind — past what the output can deliver.
class PIDController {
public:
    // Build a controller from the three gains. Any may be zero.
    PIDController(float kP, float kI, float kD);

    // Run one control step. dt is the elapsed time in seconds since the last
    // call. Returns the output to apply (clamped to the output limits).
    float calculate(float setpoint, float measurement, float dtSeconds);

    // Clamp the returned output to [min, max] in both directions, with
    // anti-windup so a saturated output does not keep growing the integrator.
    void setOutputLimits(float minOutput, float maxOutput);

    // Bound the accumulated integral (in error-seconds) to [min, max]. This is
    // the primary guard against integral windup.
    void setIntegratorLimits(float minIntegral, float maxIntegral);

    // How close the measurement must be to the setpoint for atSetpoint() to be
    // true. In the same units as the error.
    void setTolerance(float positionTolerance) { m_tolerance = positionTolerance; }

    // True when the most recent step's |error| was within the tolerance.
    bool atSetpoint() const { return m_atSetpoint; }

    // Forget all history: zero the integral and the previous error.
    void reset();

    // Read-back accessors (handy in examples/tests).
    float kP() const { return m_kp; }
    float kI() const { return m_ki; }
    float kD() const { return m_kd; }
    float error() const { return m_lastError; }

private:
    float m_kp;
    float m_ki;
    float m_kd;

    float m_integral;    // accumulated error*dt (error-seconds)
    float m_lastError;   // previous error, for the derivative
    bool  m_hasPrev;     // false until the first post-reset calculate()
    float m_atSetpoint;  // cached result of the last tolerance check

    float m_tolerance;
    float m_outMin;
    float m_outMax;
    float m_iMin;
    float m_iMax;
};

}  // namespace roboarm
