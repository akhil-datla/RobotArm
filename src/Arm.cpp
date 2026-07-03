#include "Arm.h"

#include "ServoCalibration.h"  // for kServoTravelDeg default

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
      m_lastSolution{true, false, 0.0f, 0.0f, 0.0f} {}

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
    if (degPerSec > 0.0f) {
        for (int i = 0; i < m_jointCount; ++i) {
            m_slews[i].setMaxRate(degPerSec);
            m_joints[i].useSmoothing(m_slews[i]);
        }
    }
}

void RobotArm::begin() {
    // Host build: nothing to start. The Arduino build overrides the hardware
    // start-up (Wire/driver) in the #ifdef ARDUINO layer.
}

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
    if (m_jointCount >= kMaxJoints) {
        return false;  // graceful rejection, no overflow
    }
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

bool RobotArm::moveTo(float xMm, float yMm, float approachDeg) {
    const JointAngles a = m_kin.solve(xMm, yMm, approachDeg);
    m_lastSolution = a;
    // Command whichever of shoulder/elbow/wrist are present. setAngle clamps to
    // each joint's soft limits, so even a clamped (unreachable) solution never
    // produces an out-of-range command.
    if (m_jointCount > kShoulderIndex) m_joints[kShoulderIndex].setAngle(a.shoulder);
    if (m_jointCount > kElbowIndex) m_joints[kElbowIndex].setAngle(a.elbow);
    if (m_jointCount > kWristIndex) m_joints[kWristIndex].setAngle(a.wrist);
    update();
    return a.reachable;
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
