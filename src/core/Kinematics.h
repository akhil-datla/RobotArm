// Kinematics.h — the geometry of a planar 3R arm, made usable.
//
// THE IDEA (inverse kinematics): you know *where* you want the hand — a point
// (x, y) in millimeters and an approach angle. Inverse kinematics computes the
// joint angles that put it there, so you never have to do the trig by hand.
// Forward kinematics is the reverse (angles -> where the hand ends up), handy for
// checking your work.
//
// PURE CORE: no Arduino. All trig is in radians internally; the public
// ArmKinematics class converts degrees<->radians exactly once at the boundary.
//
// COORDINATE FRAME (see CLAUDE.md sections 5/10 and Appendix A):
//   +X is horizontal "forward", +Y is up. Origin at the shoulder pivot.
//   Links: L1 upper arm (shoulder->elbow), L2 forearm (elbow->wrist),
//          L3 hand (wrist->gripper tip).
//   theta1 (shoulder): measured from +X, positive lifting toward +Y.
//   theta2 (elbow):    joint angle, 0 = forearm straight in line with upper arm.
//   theta3 (wrist):    joint angle, 0 = hand straight in line with forearm.
//   phi (approach):    absolute hand direction from +X = theta1 + theta2 + theta3.
//
//   Math derivations for every formula below are in CLAUDE.md Appendix A.
#pragma once

#include "Vec2.h"

namespace roboarm {

// ---- Forward kinematics ---------------------------------------------------

// Elbow and wrist joint positions of a 2-link chain (radians in).
struct Arm2Points {
    Vec2 elbow;  // shoulder -> elbow tip
    Vec2 wrist;  // elbow -> wrist tip (the 2-link end point)
};

// 2-link forward kinematics: both joint positions. See Appendix A.
//   x = L1*cos(t1) + L2*cos(t1+t2);  y = L1*sin(t1) + L2*sin(t1+t2)
Arm2Points forward2Points(float l1, float l2, float theta1Rad, float theta2Rad);

// 2-link forward kinematics: just the wrist point (tip of the 2-link chain).
Vec2 forward2(float l1, float l2, float theta1Rad, float theta2Rad);

// Everything the 3-link forward pass produces (radians in).
struct ToolState {
    Vec2 elbow;         // shoulder -> elbow
    Vec2 wrist;         // elbow -> wrist (== the 2-link tip)
    Vec2 tip;           // wrist -> hand tip (the gripper reference point)
    float approachRad;  // phi = theta1 + theta2 + theta3
};

// 3-link forward kinematics: tip position + approach angle. See Appendix A.
ToolState forward3(float l1, float l2, float l3,
                   float theta1Rad, float theta2Rad, float theta3Rad);

// ---- Public, degrees-facing kinematics object -----------------------------

// The hand pose the student reads back from forward(): where the tip is and which
// way the gripper points, all in the public units (mm, degrees).
struct ToolPose {
    float x;            // mm
    float y;            // mm
    float approachDeg;  // absolute gripper angle from +X, degrees
};

// The geometry made usable: hold the three link lengths, convert degrees at the
// edge. solve() (inverse kinematics) is added in the IK phases.
class ArmKinematics {
public:
    // Build from the three link lengths in millimeters.
    ArmKinematics(float l1Mm, float l2Mm, float l3Mm);

    // Forward kinematics in degrees: given the three joint angles, where does the
    // hand end up and which way does it point? Use it to check your work.
    ToolPose forward(float shoulderDeg, float elbowDeg, float wristDeg) const;

    // Link-length access / update (mm).
    float l1() const { return m_l1; }
    float l2() const { return m_l2; }
    float l3() const { return m_l3; }
    void setLinkLengths(float l1Mm, float l2Mm, float l3Mm);

private:
    float m_l1;
    float m_l2;
    float m_l3;
};

}  // namespace roboarm
