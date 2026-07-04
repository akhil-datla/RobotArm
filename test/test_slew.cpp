// Phase 3 — SlewRateLimiter tests. Caps how fast a signal can change so a servo
// ramps smoothly instead of snapping.
#include "doctest.h"
#include "TestHelpers.h"
#include "SlewRateLimiter.h"

using namespace roboarm;

TEST_CASE("output moves at most maxRate*dt per step") {
    SlewRateLimiter sr(10.0f);  // 10 units per second
    sr.reset(0.0f);
    CHECK(sr.calculate(100.0f, 0.1f) == tst::approx(1.0));  // max step = 10*0.1 = 1
    CHECK(sr.calculate(100.0f, 0.1f) == tst::approx(2.0));
    CHECK(sr.calculate(100.0f, 0.05f) == tst::approx(2.5)); // half the dt, half the step
}

TEST_CASE("does not overshoot a target within one step") {
    SlewRateLimiter sr(10.0f);
    sr.reset(0.0f);
    // Target 0.5 is inside the max step of 1.0, so it lands exactly on target.
    CHECK(sr.calculate(0.5f, 0.1f) == tst::approx(0.5));
    CHECK(sr.calculate(0.5f, 0.1f) == tst::approx(0.5));  // stays put
}

TEST_CASE("reaches the target eventually") {
    SlewRateLimiter sr(50.0f);
    sr.reset(0.0f);
    float out = 0.0f;
    for (int i = 0; i < 100; ++i) out = sr.calculate(10.0f, 0.1f);
    CHECK(out == tst::approx(10.0));
}

TEST_CASE("handles direction reversal") {
    SlewRateLimiter sr(10.0f);
    sr.reset(5.0f);
    CHECK(sr.calculate(0.0f, 0.1f) == tst::approx(4.0));  // ramp down by 1
    CHECK(sr.calculate(0.0f, 0.1f) == tst::approx(3.0));
    CHECK(sr.calculate(10.0f, 0.1f) == tst::approx(4.0)); // reverse: ramp up by 1
}

TEST_CASE("reset sets the current value") {
    SlewRateLimiter sr(10.0f);
    sr.reset(7.5f);
    CHECK(sr.lastValue() == tst::approx(7.5));
    // Asking for the same value keeps it there.
    CHECK(sr.calculate(7.5f, 0.1f) == tst::approx(7.5));
}

TEST_CASE("a wrongly-signed (negative) rate still ramps toward the target") {
    SlewRateLimiter sr(-10.0f);  // nonsensical negative rate
    sr.reset(0.0f);
    // Magnitude is used, so it moves TOWARD 100 by 1 per step, not away.
    CHECK(sr.calculate(100.0f, 0.1f) == tst::approx(1.0));
    CHECK(sr.calculate(100.0f, 0.1f) == tst::approx(2.0));
}

TEST_CASE("non-positive dt makes no move") {
    SlewRateLimiter sr(10.0f);
    sr.reset(2.0f);
    CHECK(sr.calculate(100.0f, 0.0f) == tst::approx(2.0));   // dt == 0
    CHECK(sr.calculate(100.0f, -0.5f) == tst::approx(2.0));  // dt < 0 also no move
}

TEST_CASE("a freshly constructed limiter starts at value 0") {
    SlewRateLimiter sr(10.0f);
    CHECK(sr.lastValue() == tst::approx(0.0));
    SlewRateLimiter def;  // default ctor
    CHECK(def.lastValue() == tst::approx(0.0));
    CHECK(def.maxRate() == tst::approx(0.0));
}
