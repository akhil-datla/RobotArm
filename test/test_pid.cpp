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

TEST_CASE("derivative divides by dt (verified with dt != 1)") {
    PIDController pid(0.0f, 0.0f, 1.0f);  // pure D
    CHECK(pid.calculate(10.0f, 0.0f, 0.5f) == tst::approx(0.0));  // first call: no kick
    CHECK(pid.calculate(10.0f, 0.0f, 0.5f) == tst::approx(0.0));  // constant error
    // Error 10 -> 5 over dt=0.5: d(error)/dt = (5-10)/0.5 = -10 (NOT (5-10)*0.5).
    CHECK(pid.calculate(5.0f, 0.0f, 0.5f) == tst::approx(-10.0));
}

TEST_CASE("second call with dt=0 does not divide by zero") {
    PIDController pid(2.0f, 1.0f, 1.0f);
    pid.calculate(10.0f, 0.0f, 1.0f);              // establishes previous error
    const float out = pid.calculate(10.0f, 0.0f, 0.0f);  // dt=0 on a NON-first call
    CHECK(std::isfinite(out));                     // no (err-prev)/0 NaN
    CHECK(out == tst::approx(30.0));               // P(2*10) + I(held 10) + D(0)
}

TEST_CASE("integrator-contribution anti-windup uses outLimit/ki on BOTH bounds (ki != 1)") {
    // The integral's own contribution is bounded to the output range, i.e. the
    // integrator itself is bounded to [outMin/ki, outMax/ki] = [-5, +5] (NOT
    // [-20,+20] = outLimit*ki). Both saturation directions are exercised so a
    // mutation of EITHER bound (upper outMax/ki or lower outMin/ki) is caught.
    {  // saturate HIGH -> exercises the upper bound (outMax/ki = +5)
        PIDController pid(0.0f, 2.0f, 0.0f);
        pid.setOutputLimits(-10.0f, 10.0f);
        for (int i = 0; i < 4; ++i) pid.calculate(10.0f, 0.0f, 1.0f);
        // integral capped at +5, one reversed step -> 5-10=-5 -> output -10.
        CHECK(pid.calculate(-10.0f, 0.0f, 1.0f) == tst::approx(-10.0));
    }
    {  // saturate LOW -> exercises the lower bound (outMin/ki = -5)
        PIDController pid(0.0f, 2.0f, 0.0f);
        pid.setOutputLimits(-10.0f, 10.0f);
        for (int i = 0; i < 4; ++i) pid.calculate(-10.0f, 0.0f, 1.0f);
        // integral capped at -5, one reversed step -> -5+10=+5 -> output +10.
        CHECK(pid.calculate(10.0f, 0.0f, 1.0f) == tst::approx(10.0));
    }
}

TEST_CASE("atSetpoint default tolerance is zero and reset clears it") {
    PIDController pid(1.0f, 0.0f, 0.0f);
    CHECK_FALSE(pid.atSetpoint());              // before any calculate
    pid.calculate(10.0f, 9.5f, 0.02f);          // |error| = 0.5, default tolerance 0
    CHECK_FALSE(pid.atSetpoint());              // 0.5 is NOT within tolerance 0
    pid.setTolerance(1.0f);
    pid.calculate(10.0f, 9.5f, 0.02f);
    CHECK(pid.atSetpoint());                    // now within tolerance 1
    pid.reset();
    CHECK_FALSE(pid.atSetpoint());              // reset clears the flag
}

TEST_CASE("atSetpoint is inclusive of the tolerance boundary") {
    PIDController pid(1.0f, 0.0f, 0.0f);
    pid.setTolerance(0.5f);
    pid.calculate(10.0f, 9.5f, 0.02f);  // |error| == 0.5 exactly
    CHECK(pid.atSetpoint());            // <= boundary, so true
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

TEST_CASE("a saturating P term does not cause reverse windup") {
    // Regression: with kP large enough that P alone saturates the output, the
    // integrator must not be driven to the wrong sign. Previously this produced a
    // full-scale wrong-way output at the next zero-error step.
    PIDController pid(1.0f, 1.0f, 0.0f);
    pid.setOutputLimits(-10.0f, 10.0f);
    CHECK(pid.calculate(100.0f, 0.0f, 1.0f) == tst::approx(10.0));  // saturates high
    // Error now zero. The output must not slam to the negative limit.
    const float atZero = pid.calculate(0.0f, 0.0f, 1.0f);
    CHECK(atZero >= 0.0f);              // never reverse to -10
    CHECK(atZero <= 10.0f);
    // And it must not be stuck at the wrong-sign rail on subsequent steps.
    CHECK(pid.calculate(0.0f, 0.0f, 1.0f) >= 0.0f);
    // A genuine negative error still drives the output negative as expected.
    CHECK(pid.calculate(-100.0f, 0.0f, 1.0f) == tst::approx(-10.0));
}

TEST_CASE("integrator contribution is bounded by the output range") {
    // Pure I with a large accumulated error: the I-term cannot exceed the output
    // limit, so a later sign reversal recovers promptly (no deep windup).
    PIDController pid(0.0f, 1.0f, 0.0f);
    pid.setOutputLimits(-25.0f, 25.0f);
    for (int i = 0; i < 10; ++i) pid.calculate(10.0f, 0.0f, 1.0f);  // pin high
    CHECK(pid.calculate(10.0f, 0.0f, 1.0f) == tst::approx(25.0));
    // One step of reversed error moves the output down by exactly that step.
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
