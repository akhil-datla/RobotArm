// Joint.h — one servo joint: a channel + its calibration + soft limits + optional
// motion shaping. This is a public component the student uses directly.
//
// A Joint turns "go to this angle (degrees)" into a pulse for the servo, always
// clamped to the joint's safe range. Optionally it smooths the motion with a
// SlewRateLimiter, or closes a feedback loop with a PIDController (for a
// continuous-rotation servo with a position sensor). It talks to the hardware
// only through IServoOutput and reads time only through IClock, so it is fully
// host-testable with fakes. See CLAUDE.md sections 5.1, 8, 11.
#pragma once

#include <cstdint>

#include "IClock.h"
#include "IServoOutput.h"
#include "PIDController.h"
#include "ServoCalibration.h"
#include "SlewRateLimiter.h"

namespace roboarm {

// The angle a freshly-built joint assumes until told otherwise (servo center).
constexpr float kJointDefaultAngleDeg = 90.0f;

class Joint {
public:
    // Default: unattached. Used as array storage inside RobotArm; call attach()
    // (or use the injecting constructor) before update() does anything.
    Joint();

    // Inject the hardware seam and the clock (tests pass fakes; sketches pass the
    // real Pca9685Servo + ArduinoClock).
    Joint(IServoOutput& output, IClock& clock);

    // Bind (or re-bind) the output and clock.
    void attach(IServoOutput& output, IClock& clock);

    // --- calibration set-up (these configure the joint's ServoCalibration) ---
    void setPulseRange(float minPulseUs, float maxPulseUs);
    void setTravel(float travelDeg);
    void setLimits(float minDeg, float maxDeg);
    void setDirection(int direction);
    void setOffset(float offsetDeg);
    void setCalibration(const ServoCalibration& calibration);
    ServoCalibration& calibration() { return m_calib; }
    const ServoCalibration& calibration() const { return m_calib; }

    // --- commanding ---
    // Set the target angle (degrees). Immediately clamped to the soft limits so a
    // target can never ask the servo to go somewhere unsafe.
    void setAngle(float angleDeg);

    // Push the (possibly smoothed) target out to the servo. Derives dt from the
    // clock, so call it every loop().
    void update();

    // The angle currently commanded to the servo (after clamping/smoothing).
    float currentDeg() const { return m_currentDeg; }

    // The requested target angle (after clamping).
    float targetDeg() const { return m_targetDeg; }

    // The pulse width (us) that currentDeg maps to right now.
    float commandedPulseUs() const { return m_calib.angleToPulseUs(m_currentDeg); }

    // --- optional motion shaping ---
    // Smooth motion: ramp toward the target at the limiter's rate.
    void useSmoothing(SlewRateLimiter& limiter);

    // Closed-loop feedback (continuous-rotation servo with a position sensor):
    // update() drives the joint using the PID and the latest setFeedback() value.
    void usePID(PIDController& pid);

    // Provide the measured angle for the PID feedback path.
    void setFeedback(float measuredDeg) { m_feedbackDeg = measuredDeg; }

    // True once attached to an output and a clock.
    bool isAttached() const { return m_output != nullptr && m_clock != nullptr; }

private:
    ServoCalibration m_calib;
    IServoOutput* m_output;
    IClock* m_clock;
    SlewRateLimiter* m_slew;
    PIDController* m_pid;

    float m_targetDeg;
    float m_currentDeg;
    float m_feedbackDeg;

    uint32_t m_lastMicros;
    bool m_haveLast;
};

}  // namespace roboarm
