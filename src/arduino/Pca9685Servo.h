// Pca9685Servo.h — an IServoOutput backed by one PCA9685 channel.
//
// This is a thin adapter: it holds a ServoDriver + a channel index and forwards
// the (already calibrated) pulse width to that channel. The angle->pulse
// calibration lives upstream in Joint/ServoCalibration (pure, host-tested), which
// is why this wrapper carries no math — it is just the hardware seam.
//
// ARDUINO ONLY (guarded). See CLAUDE.md sections 5.1, 10 (repo layout).
#pragma once

#ifdef ARDUINO

#include <stdint.h>

#include "hal/IServoOutput.h"
#include "ServoDriver.h"

namespace roboarm {

class Pca9685Servo : public IServoOutput {
public:
    // Unbound; call attach() before use (used as array storage in RobotArm).
    Pca9685Servo();

    // Bind to a board channel (0-15).
    Pca9685Servo(ServoDriver& board, uint8_t channel);

    void attach(ServoDriver& board, uint8_t channel);

    // Forward the calibrated pulse to this channel.
    void writeMicroseconds(float microseconds) override;

    uint8_t channel() const { return m_channel; }

private:
    ServoDriver* m_board;
    uint8_t m_channel;
};

}  // namespace roboarm

#endif  // ARDUINO
