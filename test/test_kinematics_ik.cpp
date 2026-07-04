// Phase 6 (2-link) and Phase 7 (3R) inverse-kinematics tests. Round-trip tests
// are the backbone: FK -> IK -> FK must return the same *position* (angles may
// differ between elbow-up/down solutions). Vectors from CLAUDE.md Appendix A.
#include <cmath>
#include <initializer_list>
#include "doctest.h"
#include "TestHelpers.h"
#include "Kinematics.h"
#include "MathUtils.h"

using namespace roboarm;

// ============================ 2-link IK (Phase 6) ==========================

TEST_CASE("2-link IK exact vectors (Appendix A)") {
    SUBCASE("symmetric (100,100) -> theta1=0, theta2=90") {
        Ik2Result r = inverse2(100.0f, 100.0f, 100.0f, 100.0f);
        CHECK(r.reachable);
        CHECK_FALSE(r.clamped);
        CHECK(radToDeg(r.theta1Rad) == tst::approxDeg(0.0));
        CHECK(radToDeg(r.theta2Rad) == tst::approxDeg(90.0));
    }
    SUBCASE("asymmetric (103.923,140) -> theta1=30, theta2=60") {
        Ik2Result r = inverse2(120.0f, 80.0f, 103.923f, 140.0f);
        CHECK(r.reachable);
        CHECK(radToDeg(r.theta1Rad) == tst::approxDeg(30.0));
        CHECK(radToDeg(r.theta2Rad) == tst::approxDeg(60.0));
    }
}

TEST_CASE("2-link IK round-trips through FK for many poses") {
    struct LenPair { float l1, l2; };
    for (LenPair L : {LenPair{100.0f, 100.0f}, LenPair{120.0f, 80.0f},
                      LenPair{80.0f, 120.0f}, LenPair{150.0f, 60.0f}}) {
        for (float t1d = -80.0f; t1d <= 120.0f; t1d += 10.0f) {
            for (float t2d = 10.0f; t2d <= 170.0f; t2d += 10.0f) {
                Vec2 target = forward2(L.l1, L.l2, degToRad(t1d), degToRad(t2d));
                // Both elbow configurations must land on the same target.
                for (bool up : {false, true}) {
                    Ik2Result r = inverse2(L.l1, L.l2, target.x, target.y, up);
                    CHECK(r.reachable);
                    Vec2 back = forward2(L.l1, L.l2, r.theta1Rad, r.theta2Rad);
                    CHECK(back.x == tst::approx(target.x));
                    CHECK(back.y == tst::approx(target.y));
                }
            }
        }
    }
}

TEST_CASE("2-link IK elbow-up and elbow-down are distinct branches") {
    Ik2Result down = inverse2(120.0f, 80.0f, 103.923f, 140.0f, /*elbowUp=*/false);
    Ik2Result up   = inverse2(120.0f, 80.0f, 103.923f, 140.0f, /*elbowUp=*/true);
    // Elbow-down uses +acos (theta2 > 0); elbow-up uses -acos (theta2 < 0).
    CHECK(down.theta2Rad > 0.0f);
    CHECK(up.theta2Rad < 0.0f);
    CHECK(radToDeg(down.theta2Rad) == tst::approxDeg(60.0));
    CHECK(radToDeg(up.theta2Rad) == tst::approxDeg(-60.0));
    // Both still reach the same point.
    Vec2 bd = forward2(120.0f, 80.0f, down.theta1Rad, down.theta2Rad);
    Vec2 bu = forward2(120.0f, 80.0f, up.theta1Rad, up.theta2Rad);
    CHECK(bd.x == tst::approx(bu.x));
    CHECK(bd.y == tst::approx(bu.y));
}

TEST_CASE("2-link IK: beyond max reach is unreachable and clamps onto the ray") {
    // L1=L2=100, target (250,0): D=250 > 200 -> unreachable, clamp to (200,0).
    Ik2Result r = inverse2(100.0f, 100.0f, 250.0f, 0.0f);
    CHECK_FALSE(r.reachable);
    CHECK(r.clamped);
    Vec2 back = forward2(100.0f, 100.0f, r.theta1Rad, r.theta2Rad);
    CHECK(back.x == tst::approx(200.0));
    CHECK(back.y == tst::approx(0.0));
    CHECK(radToDeg(r.theta2Rad) == tst::approxDeg(0.0));  // straight out
}

TEST_CASE("2-link IK: inside the dead zone is unreachable and clamps to inner radius") {
    // L1=120, L2=80, target (10,0): D=10 < |L1-L2|=40 -> dead zone.
    Ik2Result r = inverse2(120.0f, 80.0f, 10.0f, 0.0f);
    CHECK_FALSE(r.reachable);
    CHECK(r.clamped);
    Vec2 back = forward2(120.0f, 80.0f, r.theta1Rad, r.theta2Rad);
    // Nearest reachable point is on the same +X ray at radius |L1-L2| = 40.
    CHECK(back.length() == tst::approx(40.0));
    CHECK(std::fabs(radToDeg(r.theta2Rad)) == tst::approxDeg(180.0));  // folded
}

TEST_CASE("2-link IK boundaries: full stretch -> theta2=0, fully folded -> theta2=180") {
    // Exactly at max reach.
    Ik2Result stretched = inverse2(100.0f, 100.0f, 200.0f, 0.0f);
    CHECK(stretched.reachable);
    CHECK(radToDeg(stretched.theta2Rad) == tst::approxDeg(0.0));
    // Exactly at the inner radius (|L1-L2| = 40).
    Ik2Result folded = inverse2(120.0f, 80.0f, 40.0f, 0.0f);
    CHECK(folded.reachable);
    CHECK(std::fabs(radToDeg(folded.theta2Rad)) == tst::approxDeg(180.0));
}

TEST_CASE("2-link IK never returns NaN, even past the boundaries") {
    // A grid of targets including unreachable ones and the origin.
    for (float x = -300.0f; x <= 300.0f; x += 25.0f) {
        for (float y = -300.0f; y <= 300.0f; y += 25.0f) {
            Ik2Result r = inverse2(100.0f, 100.0f, x, y);
            CHECK(std::isfinite(r.theta1Rad));
            CHECK(std::isfinite(r.theta2Rad));
        }
    }
    // Target exactly at the origin (degenerate direction) is still finite.
    Ik2Result o = inverse2(120.0f, 80.0f, 0.0f, 0.0f);
    CHECK(std::isfinite(o.theta1Rad));
    CHECK(std::isfinite(o.theta2Rad));
}

TEST_CASE("2-link IK: OFF-AXIS clamping projects onto the correct ray") {
    // On-axis clamp tests can't tell x*s from x/s (y=0). Use off-axis targets so
    // the projected point must keep the target's direction, not just its radius.
    SUBCASE("off-axis over-reach") {
        Ik2Result r = inverse2(100.0f, 100.0f, 150.0f, 150.0f);  // D=212 > 200
        CHECK_FALSE(r.reachable);
        CHECK(r.clamped);
        Vec2 back = forward2(100.0f, 100.0f, r.theta1Rad, r.theta2Rad);
        CHECK(back.length() == tst::approx(200.0));            // on the outer circle
        CHECK(back.x == tst::approx(back.y));                  // still on the 45 deg ray
        CHECK(radToDeg(back.angleRad()) == tst::approxDeg(45.0));
    }
    SUBCASE("off-axis dead zone") {
        Ik2Result r = inverse2(120.0f, 80.0f, 20.0f, 20.0f);   // D=28.3 < 40
        CHECK_FALSE(r.reachable);
        CHECK(r.clamped);
        Vec2 back = forward2(120.0f, 80.0f, r.theta1Rad, r.theta2Rad);
        CHECK(back.length() == tst::approx(40.0));             // on the inner circle
        CHECK(radToDeg(back.angleRad()) == tst::approxDeg(45.0));
    }
}

TEST_CASE("2-link IK: origin dead-zone falls back to the +X ray exactly") {
    Ik2Result r = inverse2(120.0f, 80.0f, 0.0f, 0.0f);  // origin, direction undefined
    CHECK_FALSE(r.reachable);
    Vec2 back = forward2(120.0f, 80.0f, r.theta1Rad, r.theta2Rad);
    CHECK(back.length() == tst::approx(40.0));  // folded to the inner radius
    CHECK(back.y == tst::approx(0.0));          // along +X, not some other ray
    CHECK(back.x == tst::approx(40.0));
}

TEST_CASE("IK guards a zero (not just negative) link length against NaN") {
    // l1==0 was covered; l2==0 hits the same 2*l1*l2==0 divisor and must be guarded.
    for (float l2 : {0.0f, -30.0f}) {
        Ik2Result r = inverse2(100.0f, l2, 60.0f, 20.0f);
        CHECK_FALSE(r.reachable);
        CHECK(std::isfinite(r.theta1Rad));
        CHECK(std::isfinite(r.theta2Rad));
    }
    Ik3Result r3 = inverse3(100.0f, 0.0f, 50.0f, 100.0f, 30.0f, 0.0f);
    CHECK_FALSE(r3.reachable);
    CHECK(std::isfinite(r3.theta1Rad));
    CHECK(std::isfinite(r3.theta2Rad));
    CHECK(std::isfinite(r3.theta3Rad));
}

TEST_CASE("2-link IK reachability respects the epsilon band at the boundary") {
    // A target a hair OUTSIDE max reach (beyond the tolerance) is unreachable...
    Ik2Result out = inverse2(100.0f, 100.0f, 200.5f, 0.0f);  // 0.5 mm past 200
    CHECK_FALSE(out.reachable);
    // ...while one within the tolerance of the boundary still counts as reachable.
    Ik2Result on = inverse2(100.0f, 100.0f, 200.0005f, 0.0f);  // within 1e-3 mm
    CHECK(on.reachable);
}

TEST_CASE("2-link IK: reachable and clamped are never both true") {
    // Sweep finely across both reach boundaries (and the origin) and assert the
    // flag invariant holds — a reachable point is never also reported clamped.
    for (float d = 0.0f; d <= 260.0f; d += 0.25f) {
        Ik2Result r = inverse2(120.0f, 80.0f, d, 0.0f);  // annulus [40, 200]
        const bool both = r.reachable && r.clamped;
        CHECK_FALSE(both);
        if (r.reachable) CHECK_FALSE(r.clamped);
        else CHECK(r.clamped);  // unreachable is always reported as clamped
    }
}

TEST_CASE("degenerate link lengths never leak NaN out of kinematics") {
    // A zero (or negative) link length would divide by 2*l1*l2 == 0. §14 forbids
    // any NaN/Inf leaving kinematics.
    for (float l1 : {0.0f, -50.0f}) {
        Ik2Result r = inverse2(l1, 100.0f, 100.0f, 50.0f);
        CHECK_FALSE(r.reachable);
        CHECK(std::isfinite(r.theta1Rad));
        CHECK(std::isfinite(r.theta2Rad));
    }
    ArmKinematics kin(0.0f, 100.0f, 60.0f);
    JointAngles a = kin.solve(100.0f, 50.0f, 0.0f);
    CHECK_FALSE(a.reachable);
    CHECK(std::isfinite(a.shoulder));
    CHECK(std::isfinite(a.elbow));
    CHECK(std::isfinite(a.wrist));
}

// ============================ 3R IK (Phase 7) ==============================

TEST_CASE("3R IK exact vectors (Appendix A)") {
    SUBCASE("symmetric: tip (116.603, 201.962) phi=60 -> (30, 60, -30)") {
        Ik3Result r = inverse3(100.0f, 100.0f, 60.0f, 116.603f, 201.962f, degToRad(60.0f));
        CHECK(r.reachable);
        CHECK(radToDeg(r.theta1Rad) == tst::approxDeg(30.0));
        CHECK(radToDeg(r.theta2Rad) == tst::approxDeg(60.0));
        CHECK(radToDeg(r.theta3Rad) == tst::approxDeg(-30.0));
    }
    SUBCASE("asymmetric: tip (152.763, 160.324) phi=90 -> (20, 40, 30)") {
        Ik3Result r = inverse3(120.0f, 80.0f, 50.0f, 152.763f, 160.324f, degToRad(90.0f));
        CHECK(r.reachable);
        CHECK(radToDeg(r.theta1Rad) == tst::approxDeg(20.0));
        CHECK(radToDeg(r.theta2Rad) == tst::approxDeg(40.0));
        CHECK(radToDeg(r.theta3Rad) == tst::approxDeg(30.0));
    }
}

TEST_CASE("3R IK performs the wrist decoupling exactly as documented") {
    const float l1 = 100.0f, l2 = 100.0f, l3 = 60.0f;
    const float x = 116.603f, y = 201.962f, phi = degToRad(60.0f);
    // The documented recipe: subtract the hand link, 2-link IK, then theta3.
    const float xw = x - l3 * std::cos(phi);
    const float yw = y - l3 * std::sin(phi);
    Ik2Result two = inverse2(l1, l2, xw, yw);
    const float theta3 = phi - (two.theta1Rad + two.theta2Rad);

    Ik3Result r = inverse3(l1, l2, l3, x, y, phi);
    CHECK(r.theta1Rad == tst::approx(two.theta1Rad));
    CHECK(r.theta2Rad == tst::approx(two.theta2Rad));
    CHECK(r.theta3Rad == tst::approx(theta3));
}

TEST_CASE("3R IK round-trips through FK for many poses") {
    struct Links { float l1, l2, l3; };
    for (Links L : {Links{100.0f, 100.0f, 60.0f}, Links{120.0f, 80.0f, 50.0f},
                    Links{90.0f, 110.0f, 40.0f}}) {
        for (float t1d = -40.0f; t1d <= 90.0f; t1d += 15.0f) {
            for (float t2d = 20.0f; t2d <= 150.0f; t2d += 15.0f) {
                for (float t3d = -60.0f; t3d <= 60.0f; t3d += 30.0f) {
                    ToolState fk = forward3(L.l1, L.l2, L.l3,
                                            degToRad(t1d), degToRad(t2d), degToRad(t3d));
                    Ik3Result r = inverse3(L.l1, L.l2, L.l3, fk.tip.x, fk.tip.y, fk.approachRad);
                    CHECK(r.reachable);
                    ToolState back = forward3(L.l1, L.l2, L.l3,
                                              r.theta1Rad, r.theta2Rad, r.theta3Rad);
                    CHECK(back.tip.x == tst::approx(fk.tip.x));
                    CHECK(back.tip.y == tst::approx(fk.tip.y));
                    // Approach angle is preserved exactly.
                    CHECK(std::sin(back.approachRad) == tst::approx(std::sin(fk.approachRad)));
                    CHECK(std::cos(back.approachRad) == tst::approx(std::cos(fk.approachRad)));
                }
            }
        }
    }
}

TEST_CASE("3R IK: the approach angle changes reachability") {
    const float l1 = 100.0f, l2 = 100.0f, l3 = 60.0f;
    // Tip at (240, 0): with the hand pointing forward (phi=0) the wrist point is
    // (180,0), inside the 2-link reach; pointing back (phi=180) it is (300,0),
    // outside it.
    Ik3Result forward = inverse3(l1, l2, l3, 240.0f, 0.0f, degToRad(0.0f));
    CHECK(forward.reachable);
    Ik3Result backward = inverse3(l1, l2, l3, 240.0f, 0.0f, degToRad(180.0f));
    CHECK_FALSE(backward.reachable);
    CHECK(backward.clamped);
    // Even when unreachable, the angles are finite (no NaN leaks out).
    CHECK(std::isfinite(backward.theta1Rad));
    CHECK(std::isfinite(backward.theta2Rad));
    CHECK(std::isfinite(backward.theta3Rad));
}

TEST_CASE("3R IK elbow-up branch reaches the same tip via the other elbow config") {
    ToolState fk = forward3(100.0f, 100.0f, 60.0f,
                            degToRad(30.0f), degToRad(60.0f), degToRad(-30.0f));
    Ik3Result up = inverse3(100.0f, 100.0f, 60.0f, fk.tip.x, fk.tip.y,
                            fk.approachRad, /*elbowUp=*/true);
    CHECK(up.reachable);
    CHECK(up.theta2Rad < 0.0f);  // elbow-up uses the negative elbow angle
    ToolState back = forward3(100.0f, 100.0f, 60.0f, up.theta1Rad, up.theta2Rad,
                              up.theta3Rad);
    CHECK(back.tip.x == tst::approx(fk.tip.x));
    CHECK(back.tip.y == tst::approx(fk.tip.y));
    CHECK(std::cos(back.approachRad) == tst::approx(std::cos(fk.approachRad)));
    CHECK(std::sin(back.approachRad) == tst::approx(std::sin(fk.approachRad)));
}

TEST_CASE("ArmKinematics::solve exposes both elbow branches") {
    ArmKinematics kin(120.0f, 80.0f, 50.0f);
    JointAngles down = kin.solve(152.763f, 160.324f, 90.0f, /*elbowUp=*/false);
    JointAngles up = kin.solve(152.763f, 160.324f, 90.0f, /*elbowUp=*/true);
    CHECK(down.elbow > 0.0f);
    CHECK(up.elbow < 0.0f);
    // Both configurations still put the tip at the requested point.
    ToolPose pd = kin.forward(down.shoulder, down.elbow, down.wrist);
    ToolPose pu = kin.forward(up.shoulder, up.elbow, up.wrist);
    CHECK(pd.x == tst::approx(152.763));
    CHECK(pu.x == tst::approx(152.763));
    CHECK(pd.y == tst::approx(160.324));
    CHECK(pu.y == tst::approx(160.324));
}

TEST_CASE("example 03 wrist-hold sweep keeps every joint in its configured range") {
    // Mirrors 03_WristAngle: hold the wrist point fixed and sweep phi from level
    // to straight down. With the example's limits (shoulder/elbow [0,180], wrist
    // [-180,0]) the IK solution must never fall outside a joint's range — else the
    // servo would clamp and the sketch would silently hold the wrong pose.
    ArmKinematics kin(100.0f, 100.0f, 60.0f);
    const float xw = 120.0f, yw = 120.0f;  // the point 03_WristAngle holds fixed
    for (int phi = 0; phi >= -90; phi -= 5) {
        const float tipX = xw + 60.0f * std::cos(degToRad(static_cast<float>(phi)));
        const float tipY = yw + 60.0f * std::sin(degToRad(static_cast<float>(phi)));
        JointAngles a = kin.solve(tipX, tipY, static_cast<float>(phi));
        CHECK(a.reachable);
        CHECK(a.shoulder >= 0.0f);
        CHECK(a.shoulder <= 180.0f);
        CHECK(a.elbow >= 0.0f);
        CHECK(a.elbow <= 180.0f);
        CHECK(a.wrist >= -180.0f);
        CHECK(a.wrist <= 0.0f);
        // FK of the (in-range, unclamped) angles reproduces the tip and phi, which
        // means the held wrist point and the approach angle are exactly as claimed.
        ToolPose p = kin.forward(a.shoulder, a.elbow, a.wrist);
        CHECK(p.x == tst::approx(tipX));
        CHECK(p.y == tst::approx(tipY));
        CHECK(p.approachDeg == tst::approxDeg(static_cast<float>(phi)));
    }
}

// ======================= ArmKinematics::solve (Phase 7) ====================

TEST_CASE("ArmKinematics::solve is the degrees-facing IK") {
    SUBCASE("symmetric arm") {
        ArmKinematics kin(100.0f, 100.0f, 60.0f);
        JointAngles a = kin.solve(116.603f, 201.962f, 60.0f);
        CHECK(a.reachable);
        CHECK(a.shoulder == tst::approxDeg(30.0));
        CHECK(a.elbow == tst::approxDeg(60.0));
        CHECK(a.wrist == tst::approxDeg(-30.0));
    }
    SUBCASE("asymmetric arm") {
        ArmKinematics kin(120.0f, 80.0f, 50.0f);
        JointAngles a = kin.solve(152.763f, 160.324f, 90.0f);
        CHECK(a.reachable);
        CHECK(a.shoulder == tst::approxDeg(20.0));
        CHECK(a.elbow == tst::approxDeg(40.0));
        CHECK(a.wrist == tst::approxDeg(30.0));
    }
}

TEST_CASE("ArmKinematics::solve round-trips against forward") {
    ArmKinematics kin(120.0f, 80.0f, 50.0f);
    JointAngles a = kin.solve(152.763f, 160.324f, 90.0f);
    ToolPose pose = kin.forward(a.shoulder, a.elbow, a.wrist);
    CHECK(pose.x == tst::approx(152.763));
    CHECK(pose.y == tst::approx(160.324));
    CHECK(pose.approachDeg == tst::approxDeg(90.0));
}

TEST_CASE("ArmKinematics::solve flags an unreachable target and clamps") {
    ArmKinematics kin(100.0f, 100.0f, 60.0f);
    // Way past the 260 mm max reach.
    JointAngles a = kin.solve(400.0f, 0.0f, 0.0f);
    CHECK_FALSE(a.reachable);
    CHECK(a.clamped);
    CHECK(std::isfinite(a.shoulder));
    CHECK(std::isfinite(a.elbow));
    CHECK(std::isfinite(a.wrist));
}
