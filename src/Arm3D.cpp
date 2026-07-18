#include "Arm3D.h"

#include <cmath>

#include "core/ServoCalibration.h"  // for kServoTravelDeg default

#ifdef ARDUINO
#include "arduino/ArduinoClock.h"
#endif

namespace roboarm {

RobotArm3D::RobotArm3D(IClock& clock)
    : m_kin(kDefaultL1Mm, kDefaultL2Mm, kDefaultL3Mm),
      m_arm(clock),
      m_clock(&clock),
      m_travelDeg(kServoTravelDeg),
      m_defaultApproachDeg(0.0f),
      m_maxSpeed(0.0f),
      m_hasBase(false),
      m_lastSolution{true, false, 0.0f, 0.0f, 0.0f, 0.0f} {}

RobotArm3D::RobotArm3D()
    : m_kin(kDefaultL1Mm, kDefaultL2Mm, kDefaultL3Mm),
      m_arm(),
      m_clock(nullptr),
      m_travelDeg(kServoTravelDeg),
      m_defaultApproachDeg(0.0f),
      m_maxSpeed(0.0f),
      m_hasBase(false),
      m_lastSolution{true, false, 0.0f, 0.0f, 0.0f, 0.0f} {
#ifdef ARDUINO
    // On hardware the default arm times motion off the shared micros() clock.
    m_clock = &sharedArduinoClock();
#endif
}

void RobotArm3D::setClock(IClock& clock) {
    m_clock = &clock;
    m_arm.setClock(clock);
}

// ---- configuration --------------------------------------------------------

void RobotArm3D::setServoTravel(float degrees) {
    m_travelDeg = degrees;
    m_arm.setServoTravel(degrees);
    if (m_hasBase) {
        m_base.setTravel(degrees);
        applyBaseConvention();  // re-center the base on the new travel
    }
}

void RobotArm3D::setLinkLengths(float l1Mm, float l2Mm, float l3Mm) {
    // Keep both kinematics objects in lock-step (the base adds yaw, not length).
    m_kin.setLinkLengths(l1Mm, l2Mm, l3Mm);
    m_arm.setLinkLengths(l1Mm, l2Mm, l3Mm);
}

void RobotArm3D::setDefaultApproachAngle(float approachDeg) {
    m_defaultApproachDeg = approachDeg;
    m_arm.setDefaultApproachAngle(approachDeg);
}

void RobotArm3D::setMaxSpeed(float degPerSec) {
    m_maxSpeed = degPerSec;
    m_arm.setMaxSpeed(degPerSec);
    if (m_hasBase) {
        if (degPerSec > 0.0f) {
            m_baseSlew.setMaxRate(degPerSec);
            m_base.useSmoothing(m_baseSlew);
        } else {
            m_base.useDirect();
        }
    }
}

void RobotArm3D::begin() { m_arm.begin(); }

// ---- base calibration convention ------------------------------------------

void RobotArm3D::applyBaseConvention() {
    // Center the base on "forward": azimuth 0 -> the servo's mechanical center
    // (offset = travel/2), and let it swing +/- travel/2 to either side. So a
    // 180-degree servo covers azimuth [-90, +90] with 0 pointing along +X.
    const float half = m_travelDeg * 0.5f;
    m_base.setOffset(half);
    m_base.setLimits(-half, half);
}

bool RobotArm3D::attachBase(IServoOutput& output) {
    if (m_clock == nullptr) {
        return false;  // no clock yet: cannot time motion
    }
    m_base.attach(output, *m_clock);
    m_base.setTravel(m_travelDeg);
    applyBaseConvention();
    m_base.setAngle(0.0f);  // rest facing forward (azimuth 0)
    if (m_maxSpeed > 0.0f) {
        m_baseSlew.setMaxRate(m_maxSpeed);
        m_base.useSmoothing(m_baseSlew);
    }
    m_hasBase = true;
    return true;
}

bool RobotArm3D::addBase(IServoOutput& output) { return attachBase(output); }

bool RobotArm3D::addShoulder(IServoOutput& output) {
    return m_arm.addShoulder(output);
}
bool RobotArm3D::addElbow(IServoOutput& output) { return m_arm.addElbow(output); }
bool RobotArm3D::addWrist(IServoOutput& output) { return m_arm.addWrist(output); }
bool RobotArm3D::addJoint(IServoOutput& output) { return m_arm.addJoint(output); }
bool RobotArm3D::addGripper(IServoOutput& output, float openDeg, float closeDeg) {
    return m_arm.addGripper(output, openDeg, closeDeg);
}

#ifdef ARDUINO

void RobotArm3D::usePCA9685(uint8_t i2cAddress, float pwmFreqHz) {
    m_arm.usePCA9685(i2cAddress, pwmFreqHz);
    m_clock = &sharedArduinoClock();
}

bool RobotArm3D::addBase(uint8_t channel) {
    m_baseOutput.attach(m_arm.board(), channel);
    return attachBase(m_baseOutput);
}

bool RobotArm3D::addShoulder(uint8_t channel) { return m_arm.addShoulder(channel); }
bool RobotArm3D::addElbow(uint8_t channel) { return m_arm.addElbow(channel); }
bool RobotArm3D::addWrist(uint8_t channel) { return m_arm.addWrist(channel); }
bool RobotArm3D::addJoint(uint8_t channel) { return m_arm.addJoint(channel); }
bool RobotArm3D::addGripper(uint8_t channel, float openDeg, float closeDeg) {
    return m_arm.addGripper(channel, openDeg, closeDeg);
}

#endif  // ARDUINO

// ---- motion ---------------------------------------------------------------

bool RobotArm3D::commandWithinLimits(Joint& joint, float deg) {
    const bool within =
        std::fabs(joint.calibration().clampAngle(deg) - deg) < 1e-3f;
    joint.setAngle(deg);  // setAngle re-clamps, so the command is always safe
    return within;
}

bool RobotArm3D::moveTo(float xMm, float yMm, float zMm, float approachDeg) {
    const JointAngles3D a = m_kin.solve(xMm, yMm, zMm, approachDeg);
    m_lastSolution = a;
    // Command the base (azimuth) + whichever planar joints are present, tracking
    // whether any soft limit had to clamp. setAngle clamps internally, so even a
    // clamped (unreachable) solution never produces an out-of-range command.
    bool withinLimits = true;
    if (m_hasBase) withinLimits &= commandWithinLimits(m_base, a.base);
    if (m_arm.jointCount() > kShoulderIndex)
        withinLimits &= commandWithinLimits(m_arm.joint(kShoulderIndex), a.shoulder);
    if (m_arm.jointCount() > kElbowIndex)
        withinLimits &= commandWithinLimits(m_arm.joint(kElbowIndex), a.elbow);
    if (m_arm.jointCount() > kWristIndex)
        withinLimits &= commandWithinLimits(m_arm.joint(kWristIndex), a.wrist);
    update();
    // Truthful result: reached only if kinematically reachable AND no joint clamped.
    return a.reachable && withinLimits;
}

bool RobotArm3D::moveTo(float xMm, float yMm, float zMm) {
    return moveTo(xMm, yMm, zMm, m_defaultApproachDeg);
}

void RobotArm3D::setJointAngles(float baseDeg, float shoulderDeg, float elbowDeg,
                                float wristDeg) {
    if (m_hasBase) m_base.setAngle(baseDeg);
    if (m_arm.jointCount() > kShoulderIndex)
        m_arm.joint(kShoulderIndex).setAngle(shoulderDeg);
    if (m_arm.jointCount() > kElbowIndex)
        m_arm.joint(kElbowIndex).setAngle(elbowDeg);
    if (m_arm.jointCount() > kWristIndex)
        m_arm.joint(kWristIndex).setAngle(wristDeg);
    update();
}

void RobotArm3D::update() {
    m_arm.update();
    if (m_hasBase) m_base.update();
}

// ---- gripper --------------------------------------------------------------

void RobotArm3D::openGripper() { m_arm.openGripper(); }
void RobotArm3D::closeGripper() { m_arm.closeGripper(); }
void RobotArm3D::setGripper(float fraction) { m_arm.setGripper(fraction); }

// ---- queries --------------------------------------------------------------

bool RobotArm3D::reachable(float xMm, float yMm, float zMm,
                           float approachDeg) const {
    return m_kin.solve(xMm, yMm, zMm, approachDeg).reachable;
}

ToolPose3D RobotArm3D::currentPose() const {
    const float base = m_hasBase ? m_base.currentDeg() : 0.0f;
    const float sh = (m_arm.jointCount() > kShoulderIndex)
                         ? m_arm.joint(kShoulderIndex).currentDeg()
                         : 0.0f;
    const float el = (m_arm.jointCount() > kElbowIndex)
                         ? m_arm.joint(kElbowIndex).currentDeg()
                         : 0.0f;
    const float wr = (m_arm.jointCount() > kWristIndex)
                         ? m_arm.joint(kWristIndex).currentDeg()
                         : 0.0f;
    return m_kin.forward(base, sh, el, wr);
}

}  // namespace roboarm
