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
