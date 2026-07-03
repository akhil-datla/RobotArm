// Phase 2 — deterministic PID tests. No real clock: every call passes an explicit
// dt so behavior is reproducible (CLAUDE.md sections 2/13).
#include <cmath>
#include "doctest.h"
#include "TestHelpers.h"
#include "PIDController.h"

using namespace roboarm;

TEST_CASE("pure proportional") {
    PIDController pid(2.0f, 0.0f, 0.0f);
    // error = setpoint - measurement = 10; output = kP*error = 20.
    CHECK(pid.calculate(10.0f, 0.0f, 0.02f) == tst::approx(20.0));
}

TEST_CASE("zero output at setpoint") {
    PIDController pid(2.0f, 0.5f, 0.1f);
    // measurement already equals setpoint -> error 0 -> output 0.
    CHECK(pid.calculate(5.0f, 5.0f, 0.02f) == tst::approx(0.0));
}

TEST_CASE("integral accumulates kI*error*dt over N steps") {
    PIDController pid(0.0f, 1.0f, 0.0f);  // pure I, kI = 1
    // constant error 10, dt 1 -> integral term grows by 10 each step.
    CHECK(pid.calculate(10.0f, 0.0f, 1.0f) == tst::approx(10.0));
    CHECK(pid.calculate(10.0f, 0.0f, 1.0f) == tst::approx(20.0));
    CHECK(pid.calculate(10.0f, 0.0f, 1.0f) == tst::approx(30.0));
}

TEST_CASE("integrator clamp prevents windup") {
    PIDController pid(0.0f, 1.0f, 0.0f);
    pid.setIntegratorLimits(-30.0f, 30.0f);
    CHECK(pid.calculate(10.0f, 0.0f, 1.0f) == tst::approx(10.0));
    CHECK(pid.calculate(10.0f, 0.0f, 1.0f) == tst::approx(20.0));
    CHECK(pid.calculate(10.0f, 0.0f, 1.0f) == tst::approx(30.0));
    CHECK(pid.calculate(10.0f, 0.0f, 1.0f) == tst::approx(30.0));  // clamped, no windup
    CHECK(pid.calculate(10.0f, 0.0f, 1.0f) == tst::approx(30.0));
}

TEST_CASE("derivative responds to change and is zero when error constant") {
    PIDController pid(0.0f, 0.0f, 1.0f);  // pure D, kD = 1
    // First call after construction: no derivative kick.
    CHECK(pid.calculate(10.0f, 0.0f, 1.0f) == tst::approx(0.0));
    // Error still 10 -> unchanged -> derivative 0.
    CHECK(pid.calculate(10.0f, 0.0f, 1.0f) == tst::approx(0.0));
    // Error drops to 5 -> d(error)/dt = (5-10)/1 = -5.
    CHECK(pid.calculate(5.0f, 0.0f, 1.0f) == tst::approx(-5.0));
    // Error rises to 25 -> (25-5)/1 = 20.
    CHECK(pid.calculate(25.0f, 0.0f, 1.0f) == tst::approx(20.0));
}

TEST_CASE("setOutputLimits clamps output both directions") {
    PIDController pid(2.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-5.0f, 5.0f);
    CHECK(pid.calculate(10.0f, 0.0f, 0.02f) == tst::approx(5.0));    // would be 20 -> clamp high
    CHECK(pid.calculate(-10.0f, 0.0f, 0.02f) == tst::approx(-5.0));  // would be -20 -> clamp low
}

TEST_CASE("output-limit anti-windup stops the integrator saturating further") {
    PIDController pid(0.0f, 1.0f, 0.0f);
    pid.setOutputLimits(-25.0f, 25.0f);
    CHECK(pid.calculate(10.0f, 0.0f, 1.0f) == tst::approx(10.0));
    CHECK(pid.calculate(10.0f, 0.0f, 1.0f) == tst::approx(20.0));
    CHECK(pid.calculate(10.0f, 0.0f, 1.0f) == tst::approx(25.0));  // saturated
    CHECK(pid.calculate(10.0f, 0.0f, 1.0f) == tst::approx(25.0));  // still 25, not 40
    // Reverse the error: integrator must recover immediately (not be wound up).
    CHECK(pid.calculate(-10.0f, 0.0f, 1.0f) == tst::approx(15.0));
}

TEST_CASE("atSetpoint honors tolerance") {
    PIDController pid(1.0f, 0.0f, 0.0f);
    pid.setTolerance(0.5f);
    pid.calculate(10.0f, 10.4f, 0.02f);   // |error| = 0.4 < 0.5
    CHECK(pid.atSetpoint());
    pid.calculate(10.0f, 8.0f, 0.02f);    // |error| = 2.0 > 0.5
    CHECK_FALSE(pid.atSetpoint());
}

TEST_CASE("reset zeroes integral and previous-error state") {
    PIDController pid(0.0f, 1.0f, 1.0f);
    pid.calculate(10.0f, 0.0f, 1.0f);
    pid.calculate(10.0f, 0.0f, 1.0f);  // integral now 20
    pid.reset();
    // After reset, pure-I from scratch: first step integral = 10 again, and the
    // derivative term does not kick on the first post-reset call.
    CHECK(pid.calculate(10.0f, 0.0f, 1.0f) == tst::approx(10.0));
}

TEST_CASE("non-positive dt does not divide or integrate") {
    PIDController pid(2.0f, 1.0f, 1.0f);
    // dt = 0: only the proportional term contributes; no NaN/Inf.
    float out = pid.calculate(10.0f, 0.0f, 0.0f);
    CHECK(out == tst::approx(20.0));
    CHECK(std::isfinite(out));
}
