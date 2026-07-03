// FakeServoOutput — a test double for IServoOutput. Records every commanded pulse
// so tests can assert exactly what a Joint/RobotArm sent to the hardware seam.
// Test-only: lives under test/ (invisible to Arduino) so std::vector is fine here.
#pragma once

#include <vector>

#include "IServoOutput.h"

namespace roboarm {

class FakeServoOutput : public IServoOutput {
public:
    void writeMicroseconds(float microseconds) override {
        m_last = microseconds;
        m_history.push_back(microseconds);
    }

    // The most recent pulse width commanded (microseconds).
    float lastMicroseconds() const { return m_last; }

    // How many commands have been issued.
    int writeCount() const { return static_cast<int>(m_history.size()); }

    // Full command history, oldest first.
    const std::vector<float>& history() const { return m_history; }

    void clear() {
        m_last = 0.0f;
        m_history.clear();
    }

private:
    float m_last = 0.0f;
    std::vector<float> m_history;
};

}  // namespace roboarm
