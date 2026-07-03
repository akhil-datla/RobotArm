// FakeClock — a test double for IClock. The test sets and advances "now" by hand,
// so PID / slew / profile timing is fully deterministic (no real micros()).
#pragma once

#include <cstdint>

#include "IClock.h"

namespace roboarm {

class FakeClock : public IClock {
public:
    uint32_t nowMicros() const override { return m_now; }

    // Set the absolute time.
    void setMicros(uint32_t micros) { m_now = micros; }

    // Move time forward.
    void advanceMicros(uint32_t micros) { m_now += micros; }

    // Convenience: advance by a number of seconds.
    void advanceSeconds(float seconds) {
        m_now += static_cast<uint32_t>(seconds * 1.0e6f);
    }

private:
    uint32_t m_now = 0;
};

}  // namespace roboarm
