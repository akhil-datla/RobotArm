// Phase 8 — Joint (servo channel + calibration + soft limits + motion shaping)
// and Gripper, all driven through fakes + FakeClock.
#include <cmath>
#include "doctest.h"
#include "TestHelpers.h"
#include "Joint.h"
#include "Gripper.h"
#include "PIDController.h"
#include "SlewRateLimiter.h"
#include "ServoCalibration.h"
#include "FakeServoOutput.h"
#include "FakeClock.h"

using namespace roboarm;

TEST_CASE("Joint clamps the target to its soft limits") {
    FakeServoOutput out;
    FakeClock clk;
    Joint j(out, clk);
    j.setPulseRange(500.0f, 2500.0f);
    j.setTravel(180.0f);
    j.setLimits(0.0f, 180.0f);

    j.setAngle(200.0f);  // beyond the high limit
    j.update();
    CHECK(j.currentDeg() == tst::approxDeg(180.0));
    CHECK(out.lastMicroseconds() == tst::approx(2500.0));  // 180 deg -> 2500 us

    j.setAngle(-40.0f);  // beyond the low limit
    j.update();
    CHECK(j.currentDeg() == tst::approxDeg(0.0));
    CHECK(out.lastMicroseconds() == tst::approx(500.0));
}

TEST_CASE("Joint without smoothing jumps straight to the target") {
    FakeServoOutput out;
    FakeClock clk;
    Joint j(out, clk);
    j.setLimits(0.0f, 180.0f);

    j.setAngle(45.0f);
    j.update();
    CHECK(j.currentDeg() == tst::approxDeg(45.0));
    CHECK(out.lastMicroseconds() == tst::approx(1000.0));  // 45 deg -> 1000 us
    CHECK(out.writeCount() == 1);
}

TEST_CASE("Joint currentDeg tracks the commanded value") {
    FakeServoOutput out;
    FakeClock clk;
    Joint j(out, clk);
    j.setLimits(0.0f, 180.0f);

    j.setAngle(30.0f);
    j.update();
    CHECK(j.currentDeg() == tst::approxDeg(30.0));
    j.setAngle(120.0f);
    j.update();
    CHECK(j.currentDeg() == tst::approxDeg(120.0));
}

TEST_CASE("Joint honors a custom calibration (direction/offset flow through)") {
    FakeServoOutput out;
    FakeClock clk;
    Joint j(out, clk);
    j.setLimits(0.0f, 180.0f);
    j.setDirection(-1);  // reversed mount
    j.setAngle(0.0f);
    j.update();
    CHECK(out.lastMicroseconds() == tst::approx(2500.0));  // reversed: 0 deg -> 2500 us
    j.setAngle(180.0f);
    j.update();
    CHECK(out.lastMicroseconds() == tst::approx(500.0));
}

TEST_CASE("Joint with a SlewRateLimiter ramps toward the target") {
    FakeServoOutput out;
    FakeClock clk;
    Joint j(out, clk);
    j.setLimits(0.0f, 180.0f);

    // Park at 0 first (no smoothing yet), then enable smoothing from there.
    j.setAngle(0.0f);
    j.update();
    CHECK(j.currentDeg() == tst::approxDeg(0.0));

    SlewRateLimiter slew(60.0f);  // 60 deg/sec
    j.useSmoothing(slew);
    j.setAngle(180.0f);

    clk.advanceSeconds(0.1f);
    j.update();
    CHECK(j.currentDeg() == tst::approxDeg(6.0));  // 60*0.1 = 6 deg step
    clk.advanceSeconds(0.1f);
    j.update();
    CHECK(j.currentDeg() == tst::approxDeg(12.0));
    clk.advanceSeconds(0.1f);
    j.update();
    CHECK(j.currentDeg() == tst::approxDeg(18.0));

    // Eventually it arrives at the target and the last pulse matches 180 deg.
    for (int i = 0; i < 200; ++i) {
        clk.advanceSeconds(0.1f);
        j.update();
    }
    CHECK(j.currentDeg() == tst::approxDeg(180.0));
    CHECK(out.lastMicroseconds() == tst::approx(2500.0));

    // The recorded pulse history is monotonically non-decreasing during the ramp.
    const auto& h = out.history();
    for (size_t i = 1; i < h.size(); ++i) {
        CHECK(h[i] >= h[i - 1] - 1e-3f);
    }
}

TEST_CASE("Joint with a PID + feedback drives the measured error toward zero") {
    FakeServoOutput out;
    FakeClock clk;
    Joint j(out, clk);
    j.setLimits(0.0f, 180.0f);

    PIDController pid(2.0f, 2.0f, 0.0f);
    pid.setOutputLimits(0.0f, 180.0f);
    j.usePID(pid);
    j.setAngle(90.0f);  // want the joint at 90 deg

    float plant = 0.0f;  // simulated servo position (the feedback source)
    for (int i = 0; i < 400; ++i) {
        j.setFeedback(plant);
        clk.advanceSeconds(0.02f);
        j.update();
        const float cmd = j.currentDeg();
        // First-order actuator: moves a fraction of the way to the command.
        plant += (cmd - plant) * 0.25f;
    }
    CHECK(plant == tst::approx(90.0, 0.5));  // converged to the setpoint
    CHECK(std::fabs(90.0f - plant) < 0.5f);
}

TEST_CASE("an unattached Joint is a safe no-op") {
    Joint j;  // no output, no clock
    j.setLimits(0.0f, 180.0f);
    j.setAngle(45.0f);
    j.update();  // must not crash / dereference null
    CHECK(j.targetDeg() == tst::approxDeg(45.0));
}

// ------------------------------- Gripper -----------------------------------

TEST_CASE("Gripper open/close command the configured endpoints") {
    FakeServoOutput out;
    FakeClock clk;
    Gripper g(out, clk, /*openDeg=*/30.0f, /*closeDeg=*/120.0f);

    g.open();
    CHECK(g.currentDeg() == tst::approxDeg(30.0));
    CHECK(out.lastMicroseconds() == tst::approx(30.0 / 180.0 * 2000.0 + 500.0));

    g.close();
    CHECK(g.currentDeg() == tst::approxDeg(120.0));
    CHECK(out.lastMicroseconds() == tst::approx(120.0 / 180.0 * 2000.0 + 500.0));
}

TEST_CASE("Gripper set(0..1) interpolates between open and closed") {
    FakeServoOutput out;
    FakeClock clk;
    Gripper g(out, clk, 30.0f, 120.0f);

    g.set(0.0f);
    CHECK(g.currentDeg() == tst::approxDeg(30.0));   // fully open
    g.set(1.0f);
    CHECK(g.currentDeg() == tst::approxDeg(120.0));  // fully closed
    g.set(0.5f);
    CHECK(g.currentDeg() == tst::approxDeg(75.0));   // halfway
    // Out-of-range fractions clamp.
    g.set(2.0f);
    CHECK(g.currentDeg() == tst::approxDeg(120.0));
    g.set(-1.0f);
    CHECK(g.currentDeg() == tst::approxDeg(30.0));
}

TEST_CASE("Gripper is a Joint (inherits limits and calibration)") {
    FakeServoOutput out;
    FakeClock clk;
    Gripper g(out, clk, 30.0f, 120.0f);
    // Commanding past the closed position is clamped by the soft limits.
    g.setAngle(200.0f);
    g.update();
    CHECK(g.currentDeg() <= 120.0f + 1e-3f);
}
