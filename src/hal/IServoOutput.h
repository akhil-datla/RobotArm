// IServoOutput.h — the hardware seam for one servo channel.
//
// This is a pure-virtual interface (no Arduino here). The angle->pulse
// calibration is done upstream by Joint/ServoCalibration (pure, host-tested), so
// this seam is just the raw hardware command: "put out this pulse width." That
// is what makes it trivial to swap the real PCA9685 for a FakeServoOutput in
// tests. See CLAUDE.md section 5.1.
#pragma once

namespace roboarm {

// One servo output. Implemented by Pca9685Servo (real) and FakeServoOutput (test).
class IServoOutput {
public:
    virtual ~IServoOutput() = default;

    // Command this output to a pulse width in microseconds (already calibrated
    // and clamped by the caller). The real driver turns this into a PWM signal;
    // the fake just records it.
    virtual void writeMicroseconds(float microseconds) = 0;
};

}  // namespace roboarm
