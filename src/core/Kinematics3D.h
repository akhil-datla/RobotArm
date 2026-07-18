// Kinematics3D.h — a 4-DOF spatial arm: a rotating BASE joint on top of the
// planar 3R arm.
//
// THE IDEA: a base joint that yaws about the vertical axis doesn't change the arm
// itself — it just rotates the whole planar arm's *plane* to face a new
// direction. So reaching a 3D point splits cleanly into two easy steps you
// already understand:
//   1. aim: the base turns to the target's compass direction (azimuth);
//   2. reach: the planar 3R arm reaches the target's horizontal distance + height
//      in that (now-rotated) vertical plane — the exact same 2D math as before.
//
// FRAME (see README):  +X forward, +Y up, +Z sideways (to the arm's left).
//   base yaw is measured from +X toward +Z (0 = facing forward).
//   The horizontal distance from the vertical base axis is r = sqrt(x*x + z*z).
//   The plain 2D arm is exactly the z = 0 slice: solve(x, y, 0) with base = 0.
//
// PURE CORE: no Arduino. Radians internally; the ArmKinematics3D class converts
// degrees at the boundary. It reuses the host-tested planar forward3/inverse3.
#pragma once

#include "Kinematics.h"
#include "Vec2.h"

namespace roboarm {

// ---- free spatial functions (radians) -------------------------------------

// Result of the spatial forward pass: the tip in 3D plus the hand's approach
// (pitch) angle within the arm's vertical plane.
struct SpatialPose {
    float x;
    float y;
    float z;
    float approachRad;
};

// Spatial forward kinematics: base yaw + the planar 3R chain.
SpatialPose spatialForward(float l1, float l2, float l3, float baseRad,
                           float theta1Rad, float theta2Rad, float theta3Rad);

// Result of the spatial inverse solve (radians): the base yaw and the three
// planar joint angles, with the planar reachability/clamp flags.
struct SpatialIk {
    bool reachable;
    bool clamped;
    float baseRad;
    float theta1Rad;
    float theta2Rad;
    float theta3Rad;
};

// Spatial inverse kinematics: aim the base at the target's azimuth, then solve
// the planar 3R IK for (horizontal-distance, height). Reachability is the planar
// arm's (the base can always turn to any azimuth). Never returns NaN.
SpatialIk spatialInverse(float l1, float l2, float l3, float x, float y, float z,
                         float phiRad, bool elbowUp = false);

// ---- public, degrees-facing spatial kinematics ----------------------------

// What solve() hands back: reachability + the four joint angles in degrees.
struct JointAngles3D {
    bool reachable;
    bool clamped;
    float base;      // degrees, yaw about +Y (0 = facing +X)
    float shoulder;  // degrees
    float elbow;     // degrees
    float wrist;     // degrees
};

// The hand pose in 3D (mm) plus the approach/pitch angle (degrees).
struct ToolPose3D {
    float x;
    float y;
    float z;
    float approachDeg;
};

// The spatial geometry made usable. Holds the three link lengths (the base adds
// yaw, not length) and converts degrees at the edge. Built on the same planar
// kinematics you already use.
class ArmKinematics3D {
public:
    ArmKinematics3D(float l1Mm, float l2Mm, float l3Mm);

    // Inverse kinematics in degrees: what base/shoulder/elbow/wrist angles put
    // the hand at (x, y, z) mm with the gripper pitched at approachDeg? Returns
    // reachability; clamps (never NaN) when unreachable.
    JointAngles3D solve(float xMm, float yMm, float zMm, float approachDeg,
                        bool elbowUp = false) const;

    // Forward kinematics in degrees: where does the hand end up for these angles?
    ToolPose3D forward(float baseDeg, float shoulderDeg, float elbowDeg,
                       float wristDeg) const;

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
