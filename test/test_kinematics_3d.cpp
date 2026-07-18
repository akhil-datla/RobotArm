// 3D kinematics: a rotating base joint (yaw about the vertical Y axis) turns the
// planar 3R arm into a 4-DOF spatial arm. The base handles azimuth; the planar
// arm (shoulder/elbow/wrist) handles reach + height in its rotating vertical
// plane. Frame: +X forward, +Y up, +Z sideways; base yaw measured from +X toward
// +Z. The 2D case is exactly the z = 0 slice.
#include <cmath>
#include <initializer_list>
#include "doctest.h"
#include "TestHelpers.h"
#include "Kinematics.h"
#include "Kinematics3D.h"
#include "MathUtils.h"

using namespace roboarm;

TEST_CASE("3D forward kinematics: base yaw rotates the planar tip about +Y") {
    // Planar pose (30,60,-30) with L=100,100,60 puts the tip at radius 116.603,
    // height 201.962 (from the 2D vectors). A base yaw of 30 deg swings that
    // radius into X/Z.
    ArmKinematics3D kin(100.0f, 100.0f, 60.0f);
    ToolPose3D p = kin.forward(/*base=*/30.0f, /*sh=*/30.0f, /*el=*/60.0f, /*wr=*/-30.0f);
    CHECK(p.x == tst::approx(100.981));
    CHECK(p.y == tst::approx(201.962));
    CHECK(p.z == tst::approx(58.301));
    CHECK(p.approachDeg == tst::approxDeg(60.0));
}

TEST_CASE("3D FK with base=0 is exactly the 2D arm (z = 0 slice)") {
    ArmKinematics3D k3(120.0f, 80.0f, 50.0f);
    ArmKinematics k2(120.0f, 80.0f, 50.0f);
    ToolPose3D p3 = k3.forward(0.0f, 20.0f, 40.0f, 30.0f);
    ToolPose p2 = k2.forward(20.0f, 40.0f, 30.0f);
    CHECK(p3.x == tst::approx(p2.x));   // planar X (forward)
    CHECK(p3.y == tst::approx(p2.y));   // height
    CHECK(p3.z == tst::approx(0.0));    // no sideways component at base 0
    CHECK(p3.approachDeg == tst::approxDeg(p2.approachDeg));
}

TEST_CASE("3D FK with base=90 sends all reach sideways into +Z") {
    ArmKinematics3D kin(120.0f, 80.0f, 50.0f);
    ToolPose3D p = kin.forward(90.0f, 20.0f, 40.0f, 30.0f);
    // The planar tip radius is 152.763 (the 2D asym tip x); at base 90 -> +Z.
    CHECK(p.x == tst::approx(0.0));
    CHECK(p.z == tst::approx(152.763));
    CHECK(p.y == tst::approx(160.324));
}

TEST_CASE("3D inverse kinematics returns the base plus the three planar angles") {
    ArmKinematics3D kin(100.0f, 100.0f, 60.0f);
    JointAngles3D a = kin.solve(100.981f, 201.962f, 58.301f, 60.0f);
    CHECK(a.reachable);
    CHECK(a.base == tst::approxDeg(30.0));
    CHECK(a.shoulder == tst::approxDeg(30.0));
    CHECK(a.elbow == tst::approxDeg(60.0));
    CHECK(a.wrist == tst::approxDeg(-30.0));
}

TEST_CASE("3D IK base=0 matches the 2D solve for a z=0 target") {
    ArmKinematics3D k3(120.0f, 80.0f, 50.0f);
    ArmKinematics k2(120.0f, 80.0f, 50.0f);
    JointAngles3D a3 = k3.solve(152.763f, 160.324f, 0.0f, 90.0f);
    JointAngles a2 = k2.solve(152.763f, 160.324f, 90.0f);
    CHECK(a3.base == tst::approxDeg(0.0));
    CHECK(a3.shoulder == tst::approxDeg(a2.shoulder));
    CHECK(a3.elbow == tst::approxDeg(a2.elbow));
    CHECK(a3.wrist == tst::approxDeg(a2.wrist));
}

TEST_CASE("3D FK->IK->FK round-trips over the forward-reaching workspace") {
    // The base joint canonicalizes to "point at the target": the inverse always
    // aims the base along the target's azimuth, which makes the planar arm reach a
    // *non-negative* horizontal distance r = sqrt(x*x + z*z). A forward pose whose
    // planar tip.x is negative folds the arm *behind* its own base axis — a
    // non-canonical, redundant configuration the same 3D point is instead reached
    // by turning the base 180 deg and reaching forward. Those backward poses are
    // not part of the canonical workspace the IK targets (and at a fixed approach
    // angle the mirrored reach can leave the 2-link annulus), so the round-trip is
    // asserted over the forward-reaching subset: planar tip radius >= a small
    // margin. Every canonical pose must round-trip exactly.
    constexpr float kMinForwardReachMm = 1.0f;
    struct Links { float l1, l2, l3; };
    int checked = 0;
    for (Links L : {Links{100.0f, 100.0f, 60.0f}, Links{120.0f, 80.0f, 50.0f}}) {
        ArmKinematics3D kin(L.l1, L.l2, L.l3);
        for (float b = -150.0f; b <= 150.0f; b += 30.0f) {
            for (float t1 = -30.0f; t1 <= 80.0f; t1 += 20.0f) {
                for (float t2 = 20.0f; t2 <= 140.0f; t2 += 30.0f) {
                    for (float t3 = -50.0f; t3 <= 50.0f; t3 += 25.0f) {
                        // Canonical only: skip poses that reach behind the base axis.
                        const ToolState planar =
                            forward3(L.l1, L.l2, L.l3, degToRad(t1), degToRad(t2), degToRad(t3));
                        if (planar.tip.x < kMinForwardReachMm) continue;

                        ToolPose3D fk = kin.forward(b, t1, t2, t3);
                        JointAngles3D a = kin.solve(fk.x, fk.y, fk.z, fk.approachDeg);
                        CHECK(a.reachable);
                        ToolPose3D back = kin.forward(a.base, a.shoulder, a.elbow, a.wrist);
                        CHECK(back.x == tst::approx(fk.x));
                        CHECK(back.y == tst::approx(fk.y));
                        CHECK(back.z == tst::approx(fk.z));
                        ++checked;
                    }
                }
            }
        }
    }
    // Guard against the filter silently skipping everything.
    CHECK(checked > 300);
}

TEST_CASE("3D IK reachability follows the planar arm; unreachable is explicit") {
    ArmKinematics3D kin(100.0f, 100.0f, 60.0f);
    // A point whose horizontal distance exceeds the arm's max reach (260 mm),
    // regardless of azimuth.
    JointAngles3D far = kin.solve(300.0f, 0.0f, 200.0f, 0.0f);
    CHECK_FALSE(far.reachable);
    CHECK(std::isfinite(far.base));
    CHECK(std::isfinite(far.shoulder));
    CHECK(std::isfinite(far.elbow));
    CHECK(std::isfinite(far.wrist));
}

TEST_CASE("3D IK never leaks NaN, even for degenerate inputs") {
    for (float x = -400.0f; x <= 400.0f; x += 80.0f) {
        for (float y = -400.0f; y <= 400.0f; y += 80.0f) {
            for (float z = -400.0f; z <= 400.0f; z += 80.0f) {
                JointAngles3D a =
                    ArmKinematics3D(120.0f, 80.0f, 50.0f).solve(x, y, z, -30.0f);
                CHECK(std::isfinite(a.base));
                CHECK(std::isfinite(a.shoulder));
                CHECK(std::isfinite(a.elbow));
                CHECK(std::isfinite(a.wrist));
            }
        }
    }
    // Target on the base axis (x=z=0): azimuth is undefined -> deterministic 0.
    JointAngles3D onAxis = ArmKinematics3D(100.0f, 100.0f, 60.0f).solve(0.0f, 150.0f, 0.0f, 90.0f);
    CHECK(std::isfinite(onAxis.base));
    CHECK(onAxis.base == tst::approxDeg(0.0));
}
