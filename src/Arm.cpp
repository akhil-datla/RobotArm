#include "Arm.h"

#include <cmath>

#include "core/ServoCalibration.h"  // for kServoTravelDeg default

#ifdef ARDUINO
#include "arduino/ArduinoClock.h"
#endif

namespace roboarm {

RobotArm::RobotArm(IClock& clock)
    : m_kin(kDefaultL1Mm, kDefaultL2Mm, kDefaultL3Mm),
      m_clock(&clock),
      m_jointCount(0),
      m_hasGripper(false),
      m_travelDeg(kServoTravelDeg),
      m_defaultApproachDeg(0.0f),
      m_maxSpeed(0.0f),
      m_lastSolution{true, false, 0.0f, 0.0f, 0.0f} {}

RobotArm::RobotArm()
    : m_kin(kDefaultL1Mm, kDefaultL2Mm, kDefaultL3Mm),
      m_clock(nullptr),
      m_jointCount(0),
      m_hasGripper(false),
      m_travelDeg(kServoTravelDeg),
      m_defaultApproachDeg(0.0f),
      m_maxSpeed(0.0f),
      m_lastSolution{true, false, 0.0f, 0.0f, 0.0f} {
#ifdef ARDUINO
    // On hardware the default arm times motion off the shared micros() clock.
    m_clock = &sharedArduinoClock();
#endif
}

void RobotArm::setClock(IClock& clock) { m_clock = &clock; }

// ---- configuration --------------------------------------------------------

void RobotArm::setServoTravel(float degrees) {
    m_travelDeg = degrees;
    for (int i = 0; i < m_jointCount; ++i) {
        m_joints[i].setTravel(degrees);
    }
    if (m_hasGripper) {
        m_gripper.setTravel(degrees);
    }
}

void RobotArm::setLinkLengths(float l1Mm, float l2Mm, float l3Mm) {
    m_kin.setLinkLengths(l1Mm, l2Mm, l3Mm);
}

void RobotArm::setDefaultApproachAngle(float approachDeg) {
    m_defaultApproachDeg = approachDeg;
}

void RobotArm::setMaxSpeed(float degPerSec) {
    m_maxSpeed = degPerSec;
    for (int i = 0; i < m_jointCount; ++i) {
        if (degPerSec > 0.0f) {
            m_slews[i].setMaxRate(degPerSec);
            m_joints[i].useSmoothing(m_slews[i]);
        } else {
            m_joints[i].useDirect();  // 0 or less -> smoothing off, instant moves
        }
    }
}

void RobotArm::begin() {
#ifdef ARDUINO
    // Bring up I2C and the PCA9685.
    m_board.begin();
#endif
    // Host build: nothing else to start.
}

#ifdef ARDUINO

void RobotArm::usePCA9685(uint8_t i2cAddress, float pwmFreqHz) {
    m_board.configure(i2cAddress, pwmFreqHz);
    m_clock = &sharedArduinoClock();
}

bool RobotArm::addShoulder(uint8_t channel) {
    m_outputs[kShoulderIndex].attach(m_board, channel);
    return attachJointAt(kShoulderIndex, m_outputs[kShoulderIndex]);
}

bool RobotArm::addElbow(uint8_t channel) {
    m_outputs[kElbowIndex].attach(m_board, channel);
    return attachJointAt(kElbowIndex, m_outputs[kElbowIndex]);
}

bool RobotArm::addWrist(uint8_t channel) {
    m_outputs[kWristIndex].attach(m_board, channel);
    return attachJointAt(kWristIndex, m_outputs[kWristIndex]);
}

bool RobotArm::addJoint(uint8_t channel) {
    if (m_jointCount >= kMaxJoints) {
        return false;
    }
    const int index = m_jointCount;
    m_outputs[index].attach(m_board, channel);
    return attachJointAt(index, m_outputs[index]);
}

bool RobotArm::addGripper(uint8_t channel, float openDeg, float closeDeg) {
    m_gripperOutput.attach(m_board, channel);
    return addGripper(m_gripperOutput, openDeg, closeDeg);
}

#endif  // ARDUINO

// ---- joint registration ---------------------------------------------------

bool RobotArm::attachJointAt(int index, IServoOutput& output) {
    if (m_clock == nullptr) {
        return false;  // no clock yet: cannot time motion
    }
    if (index < 0 || index >= kMaxJoints) {
        return false;
    }
    m_joints[index].attach(output, *m_clock);
    m_joints[index].setTravel(m_travelDeg);
    if (m_maxSpeed > 0.0f) {
        m_slews[index].setMaxRate(m_maxSpeed);
        m_joints[index].useSmoothing(m_slews[index]);
    }
    if (index + 1 > m_jointCount) {
        m_jointCount = index + 1;
    }
    return true;
}

bool RobotArm::addShoulder(IServoOutput& output) {
    return attachJointAt(kShoulderIndex, output);
}
bool RobotArm::addElbow(IServoOutput& output) {
    return attachJointAt(kElbowIndex, output);
}
bool RobotArm::addWrist(IServoOutput& output) {
    return attachJointAt(kWristIndex, output);
}

bool RobotArm::addJoint(IServoOutput& output) {
    // attachJointAt is the single validation point: it rejects (returns false,
    // leaving m_jointCount untouched) once the array is full, so this is a
    // graceful no-overflow rejection.
    return attachJointAt(m_jointCount, output);
}

bool RobotArm::addGripper(IServoOutput& output, float openDeg, float closeDeg) {
    if (m_clock == nullptr) {
        return false;
    }
    m_gripper.attach(output, *m_clock);
    m_gripper.setTravel(m_travelDeg);
    m_gripper.configure(openDeg, closeDeg);
    m_hasGripper = true;
    return true;
}

// ---- motion ---------------------------------------------------------------

bool RobotArm::commandJointWithinLimits(int index, float deg) {
    const bool within =
        std::fabs(m_joints[index].calibration().clampAngle(deg) - deg) < 1e-3f;
    m_joints[index].setAngle(deg);  // setAngle re-clamps, so the command is safe
    return within;
}

bool RobotArm::moveTo(float xMm, float yMm, float approachDeg) {
    const JointAngles a = m_kin.solve(xMm, yMm, approachDeg);
    m_lastSolution = a;
    // Command whichever of shoulder/elbow/wrist are present, and track whether any
    // joint's soft limit had to clamp the solution. setAngle clamps internally, so
    // even a clamped (unreachable) solution never produces an out-of-range command.
    bool withinLimits = true;
    if (m_jointCount > kShoulderIndex)
        withinLimits &= commandJointWithinLimits(kShoulderIndex, a.shoulder);
    if (m_jointCount > kElbowIndex)
        withinLimits &= commandJointWithinLimits(kElbowIndex, a.elbow);
    if (m_jointCount > kWristIndex)
        withinLimits &= commandJointWithinLimits(kWristIndex, a.wrist);
    update();
    // Truthful result: reached only if kinematically reachable AND no joint clamped.
    return a.reachable && withinLimits;
}

bool RobotArm::moveTo(float xMm, float yMm) {
    return moveTo(xMm, yMm, m_defaultApproachDeg);
}

void RobotArm::setJointAngles(float shoulderDeg, float elbowDeg, float wristDeg) {
    if (m_jointCount > kShoulderIndex) m_joints[kShoulderIndex].setAngle(shoulderDeg);
    if (m_jointCount > kElbowIndex) m_joints[kElbowIndex].setAngle(elbowDeg);
    if (m_jointCount > kWristIndex) m_joints[kWristIndex].setAngle(wristDeg);
    update();
}

void RobotArm::update() {
    for (int i = 0; i < m_jointCount; ++i) {
        m_joints[i].update();
    }
    if (m_hasGripper) {
        m_gripper.update();
    }
}

// ---- gripper --------------------------------------------------------------

void RobotArm::openGripper() {
    if (m_hasGripper) m_gripper.open();
}
void RobotArm::closeGripper() {
    if (m_hasGripper) m_gripper.close();
}
void RobotArm::setGripper(float fraction) {
    if (m_hasGripper) m_gripper.set(fraction);
}

// ---- queries --------------------------------------------------------------

bool RobotArm::reachable(float xMm, float yMm, float approachDeg) const {
    return m_kin.solve(xMm, yMm, approachDeg).reachable;
}

ToolPose RobotArm::currentPose() const {
    const float sh = (m_jointCount > kShoulderIndex)
                         ? m_joints[kShoulderIndex].currentDeg()
                         : 0.0f;
    const float el =
        (m_jointCount > kElbowIndex) ? m_joints[kElbowIndex].currentDeg() : 0.0f;
    const float wr =
        (m_jointCount > kWristIndex) ? m_joints[kWristIndex].currentDeg() : 0.0f;
    return m_kin.forward(sh, el, wr);
}

}  // namespace roboarm
