#include "Kinematics.h"

#include <cmath>

#include "MathUtils.h"

namespace roboarm {

// ---- Forward kinematics ---------------------------------------------------

Arm2Points forward2Points(float l1, float l2, float theta1Rad, float theta2Rad) {
    Arm2Points p;
    p.elbow = Vec2(l1 * cosf(theta1Rad), l1 * sinf(theta1Rad));
    const float a12 = theta1Rad + theta2Rad;
    p.wrist = p.elbow + Vec2(l2 * cosf(a12), l2 * sinf(a12));
    return p;
}

Vec2 forward2(float l1, float l2, float theta1Rad, float theta2Rad) {
    return forward2Points(l1, l2, theta1Rad, theta2Rad).wrist;
}

ToolState forward3(float l1, float l2, float l3,
                   float theta1Rad, float theta2Rad, float theta3Rad) {
    ToolState s;
    s.elbow = Vec2(l1 * cosf(theta1Rad), l1 * sinf(theta1Rad));
    const float a12 = theta1Rad + theta2Rad;
    s.wrist = s.elbow + Vec2(l2 * cosf(a12), l2 * sinf(a12));
    const float a123 = a12 + theta3Rad;
    s.tip = s.wrist + Vec2(l3 * cosf(a123), l3 * sinf(a123));
    s.approachRad = a123;
    return s;
}

// ---- ArmKinematics --------------------------------------------------------

ArmKinematics::ArmKinematics(float l1Mm, float l2Mm, float l3Mm)
    : m_l1(l1Mm), m_l2(l2Mm), m_l3(l3Mm) {}

void ArmKinematics::setLinkLengths(float l1Mm, float l2Mm, float l3Mm) {
    m_l1 = l1Mm;
    m_l2 = l2Mm;
    m_l3 = l3Mm;
}

ToolPose ArmKinematics::forward(float shoulderDeg, float elbowDeg,
                                float wristDeg) const {
    const ToolState s = forward3(m_l1, m_l2, m_l3, degToRad(shoulderDeg),
                                 degToRad(elbowDeg), degToRad(wristDeg));
    ToolPose pose;
    pose.x = s.tip.x;
    pose.y = s.tip.y;
    pose.approachDeg = radToDeg(s.approachRad);
    return pose;
}

}  // namespace roboarm
