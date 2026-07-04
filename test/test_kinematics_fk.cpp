// Phase 5 — forward kinematics: joint angles -> tip position. Vectors are the
// hand-verified ones from CLAUDE.md Appendix A (tolerance 1e-3 mm / 1e-2 deg).
#include "doctest.h"
#include "TestHelpers.h"
#include "Kinematics.h"
#include "MathUtils.h"

using namespace roboarm;

TEST_CASE("2-link FK, symmetric L1=L2=100") {
    // FK2(0, 90) = (100, 100)
    Vec2 p = forward2(100.0f, 100.0f, degToRad(0.0f), degToRad(90.0f));
    CHECK(p.x == tst::approx(100.0));
    CHECK(p.y == tst::approx(100.0));

    // FK2(0, 0) = (200, 0) — fully extended
    Vec2 q = forward2(100.0f, 100.0f, degToRad(0.0f), degToRad(0.0f));
    CHECK(q.x == tst::approx(200.0));
    CHECK(q.y == tst::approx(0.0));
}

TEST_CASE("2-link FK, asymmetric L1=120 L2=80") {
    // FK2(30, 60) = (103.923, 140.000)
    Vec2 p = forward2(120.0f, 80.0f, degToRad(30.0f), degToRad(60.0f));
    CHECK(p.x == tst::approx(103.923));
    CHECK(p.y == tst::approx(140.000));
}

TEST_CASE("2-link FK exposes the elbow joint position") {
    // With theta1 = 0, the elbow sits at (L1, 0).
    Arm2Points pts = forward2Points(120.0f, 80.0f, degToRad(30.0f), degToRad(60.0f));
    CHECK(pts.elbow.x == tst::approx(120.0f * std::cos(degToRad(30.0f))));
    CHECK(pts.elbow.y == tst::approx(120.0f * std::sin(degToRad(30.0f))));
    // The wrist point matches forward2().
    CHECK(pts.wrist.x == tst::approx(103.923));
    CHECK(pts.wrist.y == tst::approx(140.000));
}

TEST_CASE("3-link FK, symmetric L1=L2=100 L3=60") {
    // FK3(30, 60, -30) = tip (116.603, 201.962), phi = 60
    ToolState s = forward3(100.0f, 100.0f, 60.0f,
                           degToRad(30.0f), degToRad(60.0f), degToRad(-30.0f));
    CHECK(s.tip.x == tst::approx(116.603));
    CHECK(s.tip.y == tst::approx(201.962));
    CHECK(radToDeg(s.approachRad) == tst::approxDeg(60.0));
}

TEST_CASE("3-link FK, asymmetric L1=120 L2=80 L3=50") {
    // FK3(20, 40, 30) = tip (152.763, 160.324), phi = 90
    ToolState s = forward3(120.0f, 80.0f, 50.0f,
                           degToRad(20.0f), degToRad(40.0f), degToRad(30.0f));
    CHECK(s.tip.x == tst::approx(152.763));
    CHECK(s.tip.y == tst::approx(160.324));
    CHECK(radToDeg(s.approachRad) == tst::approxDeg(90.0));
}

TEST_CASE("3-link FK reports elbow and wrist positions consistently") {
    ToolState s = forward3(100.0f, 100.0f, 60.0f,
                           degToRad(30.0f), degToRad(60.0f), degToRad(-30.0f));
    // The wrist point of the 3-link chain equals the 2-link tip.
    Vec2 w = forward2(100.0f, 100.0f, degToRad(30.0f), degToRad(60.0f));
    CHECK(s.wrist.x == tst::approx(w.x));
    CHECK(s.wrist.y == tst::approx(w.y));
    // The hand link L3 connects wrist to tip and has length 60.
    CHECK((s.tip - s.wrist).length() == tst::approx(60.0));
    // The elbow lies at distance L1 from the origin.
    CHECK(s.elbow.length() == tst::approx(100.0));
}

TEST_CASE("ArmKinematics::forward is the degrees-facing FK") {
    ArmKinematics kin(100.0f, 100.0f, 60.0f);
    ToolPose pose = kin.forward(30.0f, 60.0f, -30.0f);
    CHECK(pose.x == tst::approx(116.603));
    CHECK(pose.y == tst::approx(201.962));
    CHECK(pose.approachDeg == tst::approxDeg(60.0));
}
