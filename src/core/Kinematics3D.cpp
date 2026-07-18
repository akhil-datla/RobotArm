#include "Kinematics3D.h"

#include <cmath>

#include "MathUtils.h"

namespace roboarm {

namespace {
// Below this horizontal distance the azimuth is undefined (the target sits on the
// vertical base axis), so we pick a deterministic base of 0.
constexpr float kAxisEpsMm = 1e-6f;
}  // namespace

SpatialPose spatialForward(float l1, float l2, float l3, float baseRad,
                           float theta1Rad, float theta2Rad, float theta3Rad) {
    // Solve the arm in its own vertical plane: planar x is the horizontal reach
    // r, planar y is the height.
    const ToolState planar = forward3(l1, l2, l3, theta1Rad, theta2Rad, theta3Rad);
    const float r = planar.tip.x;
    const float height = planar.tip.y;

    SpatialPose out;
    // Swing the radial reach into X/Z by the base yaw; height maps straight to Y.
    out.x = r * cosf(baseRad);
    out.z = r * sinf(baseRad);
    out.y = height;
    out.approachRad = planar.approachRad;
    return out;
}

SpatialIk spatialInverse(float l1, float l2, float l3, float x, float y, float z,
                         float phiRad, bool elbowUp) {
    // Aim: the base points at the target's compass direction.
    const float r = sqrtf(x * x + z * z);
    const float baseRad = (r > kAxisEpsMm) ? atan2f(z, x) : 0.0f;

    // Reach: the planar 3R arm solves for (horizontal distance, height).
    const Ik3Result planar = inverse3(l1, l2, l3, r, y, phiRad, elbowUp);

    SpatialIk out;
    out.reachable = planar.reachable;
    out.clamped = planar.clamped;
    out.baseRad = baseRad;
    out.theta1Rad = planar.theta1Rad;
    out.theta2Rad = planar.theta2Rad;
    out.theta3Rad = planar.theta3Rad;
    return out;
}

ArmKinematics3D::ArmKinematics3D(float l1Mm, float l2Mm, float l3Mm)
    : m_l1(l1Mm), m_l2(l2Mm), m_l3(l3Mm) {}

void ArmKinematics3D::setLinkLengths(float l1Mm, float l2Mm, float l3Mm) {
    m_l1 = l1Mm;
    m_l2 = l2Mm;
    m_l3 = l3Mm;
}

JointAngles3D ArmKinematics3D::solve(float xMm, float yMm, float zMm,
                                     float approachDeg, bool elbowUp) const {
    const SpatialIk r = spatialInverse(m_l1, m_l2, m_l3, xMm, yMm, zMm,
                                       degToRad(approachDeg), elbowUp);
    JointAngles3D a;
    a.reachable = r.reachable;
    a.clamped = r.clamped;
    a.base = radToDeg(r.baseRad);
    a.shoulder = radToDeg(r.theta1Rad);
    a.elbow = radToDeg(r.theta2Rad);
    a.wrist = radToDeg(r.theta3Rad);
    return a;
}

ToolPose3D ArmKinematics3D::forward(float baseDeg, float shoulderDeg,
                                    float elbowDeg, float wristDeg) const {
    const SpatialPose s =
        spatialForward(m_l1, m_l2, m_l3, degToRad(baseDeg), degToRad(shoulderDeg),
                       degToRad(elbowDeg), degToRad(wristDeg));
    ToolPose3D pose;
    pose.x = s.x;
    pose.y = s.y;
    pose.z = s.z;
    pose.approachDeg = radToDeg(s.approachRad);
    return pose;
}

}  // namespace roboarm
