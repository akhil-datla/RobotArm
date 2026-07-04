// ArduinoClock.h — an IClock backed by Arduino's micros().
//
// ARDUINO ONLY (guarded). See CLAUDE.md sections 5.1, 10 (repo layout).
#pragma once

#ifdef ARDUINO

#include <Arduino.h>
#include <stdint.h>

#include "hal/IClock.h"

namespace roboarm {

class ArduinoClock : public IClock {
public:
    uint32_t nowMicros() const override { return micros(); }
};

// A single shared clock instance used by the convenience constructors
// (Joint(board, channel), the default RobotArm, etc.) so beginners never have to
// build a clock by hand.
ArduinoClock& sharedArduinoClock();

}  // namespace roboarm

#endif  // ARDUINO
