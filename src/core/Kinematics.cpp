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

// ---- Inverse kinematics ---------------------------------------------------

namespace {
// A small tolerance (mm) so a target sitting exactly on the max-reach or inner
// boundary counts as reachable rather than tripping over float rounding.
constexpr float kReachEpsMm = 1e-3f;
}  // namespace

Ik2Result inverse2(float l1, float l2, float x, float y, bool elbowUp) {
    Ik2Result r;
    r.clamped = false;

    const float dMax = l1 + l2;
    const float dMin = fabsf(l1 - l2);
    float dist = sqrtf(x * x + y * y);

    // Reachability on the wrist point: the annulus [|L1-L2|, L1+L2].
    r.reachable = (dist <= dMax + kReachEpsMm) && (dist >= dMin - kReachEpsMm);

    // Effective target: if unreachable, project onto the nearest annulus edge
    // along the same ray so the arm points the right way and never NaNs. The
    // clamp uses the SAME epsilon band as the reachability test above, so
    // `clamped` is never set while `reachable` is still true (a point within the
    // tolerance of a boundary is treated as reachable and left alone).
    float ex = x;
    float ey = y;
    if (dist > dMax + kReachEpsMm) {
        r.clamped = true;
        if (dist > 0.0f) {
            const float s = dMax / dist;
            ex = x * s;
            ey = y * s;
        } else {
            ex = dMax;
            ey = 0.0f;
        }
        dist = dMax;
    } else if (dist < dMin - kReachEpsMm) {
        r.clamped = true;
        if (dist > 1e-6f) {
            const float s = dMin / dist;
            ex = x * s;
            ey = y * s;
        } else {
            // Target essentially at the origin: direction is undefined, so pick
            // the +X ray as a safe, deterministic default.
            ex = dMin;
            ey = 0.0f;
        }
        dist = dMin;
    }

    // Law of cosines for the elbow angle. Clamp the argument to [-1, 1] first —
    // this is the guard that keeps boundary inputs from returning NaN.
    float cos2 = (dist * dist - l1 * l1 - l2 * l2) / (2.0f * l1 * l2);
    cos2 = clamp(cos2, -1.0f, 1.0f);
    const float t2 = elbowUp ? -acosf(cos2) : acosf(cos2);

    // Shoulder angle: aim at the point, then rotate back by the elbow's offset.
    const float t1 = atan2f(ey, ex) - atan2f(l2 * sinf(t2), l1 + l2 * cosf(t2));

    r.theta1Rad = t1;
    r.theta2Rad = t2;
    return r;
}

Ik3Result inverse3(float l1, float l2, float l3, float x, float y, float phiRad,
                   bool elbowUp) {
    // Wrist decoupling: back off along the hand link to get the wrist point, then
    // solve the 2-link problem there. Reachability is inherited from that solve.
    const float xw = x - l3 * cosf(phiRad);
    const float yw = y - l3 * sinf(phiRad);
    const Ik2Result two = inverse2(l1, l2, xw, yw, elbowUp);

    Ik3Result r;
    r.reachable = two.reachable;
    r.clamped = two.clamped;
    r.theta1Rad = two.theta1Rad;
    r.theta2Rad = two.theta2Rad;
    // Keep the requested absolute approach angle: theta3 makes the angles sum to
    // phi. (When the wrist point was clamped, this holds phi relative to the
    // clamped shoulder/elbow.)
    r.theta3Rad = phiRad - (two.theta1Rad + two.theta2Rad);
    return r;
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

JointAngles ArmKinematics::solve(float xMm, float yMm, float approachDeg,
                                 bool elbowUp) const {
    const Ik3Result r =
        inverse3(m_l1, m_l2, m_l3, xMm, yMm, degToRad(approachDeg), elbowUp);
    JointAngles a;
    a.reachable = r.reachable;
    a.clamped = r.clamped;
    a.shoulder = radToDeg(r.theta1Rad);
    a.elbow = radToDeg(r.theta2Rad);
    a.wrist = radToDeg(r.theta3Rad);
    return a;
}

}  // namespace roboarm
