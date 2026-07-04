// Phase 1 — tests for the tiny pure math helpers everything else builds on.
#include <initializer_list>
#include "doctest.h"
#include "TestHelpers.h"
#include "MathUtils.h"
#include "Vec2.h"

using namespace roboarm;

TEST_CASE("degToRad is exact at key angles") {
    CHECK(degToRad(0.0f)   == tst::approx(0.0));
    CHECK(degToRad(90.0f)  == tst::approx(kPi / 2.0f));
    CHECK(degToRad(180.0f) == tst::approx(kPi));
    CHECK(degToRad(360.0f) == tst::approx(2.0f * kPi));
}

TEST_CASE("radToDeg is exact at key angles") {
    CHECK(radToDeg(0.0f)            == tst::approx(0.0));
    CHECK(radToDeg(kPi / 2.0f)      == tst::approx(90.0));
    CHECK(radToDeg(kPi)             == tst::approx(180.0));
    CHECK(radToDeg(2.0f * kPi)      == tst::approx(360.0));
}

TEST_CASE("deg<->rad round-trips") {
    for (float d : {0.0f, 12.5f, 30.0f, 90.0f, 175.0f, 359.0f, -47.0f}) {
        CHECK(radToDeg(degToRad(d)) == tst::approx(d));
    }
}

TEST_CASE("clamp at, inside, and outside bounds") {
    CHECK(clamp(5.0f, 0.0f, 10.0f) == tst::approx(5.0));   // inside
    CHECK(clamp(0.0f, 0.0f, 10.0f) == tst::approx(0.0));   // at low bound
    CHECK(clamp(10.0f, 0.0f, 10.0f) == tst::approx(10.0)); // at high bound
    CHECK(clamp(-3.0f, 0.0f, 10.0f) == tst::approx(0.0));  // below
    CHECK(clamp(42.0f, 0.0f, 10.0f) == tst::approx(10.0)); // above
    // Works for integer types too (used for pulse widths).
    CHECK(clamp(2500, 500, 2000) == 2000);
    CHECK(clamp(300, 500, 2000) == 500);
}

TEST_CASE("lerp endpoints and midpoint") {
    CHECK(lerp(10.0f, 20.0f, 0.0f) == tst::approx(10.0));
    CHECK(lerp(10.0f, 20.0f, 1.0f) == tst::approx(20.0));
    CHECK(lerp(10.0f, 20.0f, 0.5f) == tst::approx(15.0));
    CHECK(lerp(-100.0f, 100.0f, 0.25f) == tst::approx(-50.0));
}

TEST_CASE("wrapAngleDeg maps angles into a single turn") {
    CHECK(wrapAngleDeg(190.0f)  == tst::approxDeg(-170.0));
    CHECK(wrapAngleDeg(-190.0f) == tst::approxDeg(170.0));
    CHECK(wrapAngleDeg(360.0f)  == tst::approxDeg(0.0));
    CHECK(wrapAngleDeg(0.0f)    == tst::approxDeg(0.0));
    CHECK(wrapAngleDeg(45.0f)   == tst::approxDeg(45.0));
    CHECK(wrapAngleDeg(720.0f)  == tst::approxDeg(0.0));
    CHECK(wrapAngleDeg(-360.0f) == tst::approxDeg(0.0));
    CHECK(wrapAngleDeg(540.0f)  == tst::approxDeg(-180.0)); // 540 -> -180 (half turn)
    // Just below the +180 wrap boundary: must stay positive (pins the +180 offset).
    CHECK(wrapAngleDeg(179.5f)  == tst::approxDeg(179.5));
    CHECK(wrapAngleDeg(-179.5f) == tst::approxDeg(-179.5));
    CHECK(wrapAngleDeg(200.0f)  == tst::approxDeg(-160.0));
}

TEST_CASE("Vec2 length") {
    CHECK(Vec2(3.0f, 4.0f).length() == tst::approx(5.0));
    CHECK(Vec2(0.0f, 0.0f).length() == tst::approx(0.0));
    CHECK(Vec2(-6.0f, 8.0f).length() == tst::approx(10.0));
}

TEST_CASE("Vec2 add and subtract") {
    Vec2 a(1.0f, 2.0f), b(3.0f, -5.0f);
    Vec2 s = a + b;
    CHECK(s.x == tst::approx(4.0));
    CHECK(s.y == tst::approx(-3.0));
    Vec2 d = a - b;
    CHECK(d.x == tst::approx(-2.0));
    CHECK(d.y == tst::approx(7.0));
}

TEST_CASE("Vec2 direction via atan2") {
    CHECK(radToDeg(Vec2(1.0f, 0.0f).angleRad()) == tst::approxDeg(0.0));
    CHECK(radToDeg(Vec2(0.0f, 1.0f).angleRad()) == tst::approxDeg(90.0));
    CHECK(radToDeg(Vec2(-1.0f, 0.0f).angleRad()) == tst::approxDeg(180.0));
    CHECK(radToDeg(Vec2(0.0f, -1.0f).angleRad()) == tst::approxDeg(-90.0));
    CHECK(radToDeg(Vec2(1.0f, 1.0f).angleRad()) == tst::approxDeg(45.0));
}
