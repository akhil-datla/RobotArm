// Gripper.h — a Joint with two named positions: open and closed.
//
// THE IDEA: a gripper is just a servo you drive to two angles. This component
// makes that explicit — open(), close(), or set(0..1) to go partway — so the
// student sees a gripper is nothing more than a Joint with a friendly face.
// See CLAUDE.md sections 8, 11.
#pragma once

#include "Joint.h"

namespace roboarm {

// A Joint that knows an "open" angle and a "closed" angle.
class Gripper : public Joint {
public:
    // Default open/closed positions (degrees) for the unconfigured gripper.
    static constexpr float kDefaultOpenDeg = 30.0f;
    static constexpr float kDefaultCloseDeg = 120.0f;

    Gripper();

    // Inject the hardware seam + clock and set the open/closed angles.
    Gripper(IServoOutput& output, IClock& clock, float openDeg, float closeDeg);

#ifdef ARDUINO
    // Beginner convenience: bind to a PCA9685 channel (uses the shared clock).
    Gripper(ServoDriver& board, uint8_t channel, float openDeg, float closeDeg);
#endif

    // Set the two named positions. Also widens the soft limits to span them.
    void configure(float openDeg, float closeDeg);

    // Drive to the open position and command it immediately.
    void open();

    // Drive to the closed position and command it immediately.
    void close();

    // Set the opening fraction: 0 = fully open, 1 = fully closed. Clamped.
    void set(float fraction);

    float openDeg() const { return m_openDeg; }
    float closeDeg() const { return m_closeDeg; }

private:
    float m_openDeg;
    float m_closeDeg;
};

}  // namespace roboarm
