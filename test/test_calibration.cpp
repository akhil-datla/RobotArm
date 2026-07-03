// Phase 4 — ServoCalibration (angle<->pulse map + mounting + soft limits) and a
// seam test through FakeServoOutput.
#include <cmath>
#include "doctest.h"
#include "TestHelpers.h"
#include "ServoCalibration.h"
#include "FakeServoOutput.h"
#include "FakeClock.h"

using namespace roboarm;

TEST_CASE("default RDS3225 map: 0/90/180 deg -> 500/1500/2500 us") {
    ServoCalibration c;
    CHECK(c.angleToPulseUs(0.0f)   == tst::approx(500.0));
    CHECK(c.angleToPulseUs(90.0f)  == tst::approx(1500.0));
    CHECK(c.angleToPulseUs(180.0f) == tst::approx(2500.0));
}

TEST_CASE("defaults equal the named RDS3225 constants") {
    ServoCalibration c;
    CHECK(c.minPulseUs() == tst::approx(kRds3225MinPulseUs));
    CHECK(c.maxPulseUs() == tst::approx(kRds3225MaxPulseUs));
    CHECK(c.travelDeg()  == tst::approx(kServoTravelDeg));
    CHECK(c.angleToPulseUs(90.0f) == tst::approx(kRds3225NeutralUs));
    CHECK(kServoTravelDeg == tst::approx(180.0));
}

TEST_CASE("map is monotonic and linear") {
    ServoCalibration c;
    float prev = -1.0f;
    for (float a = 0.0f; a <= 180.0f; a += 5.0f) {
        float us = c.angleToPulseUs(a);
        CHECK(us > prev);  // strictly increasing
        prev = us;
    }
    // Linear: 45 deg is a quarter of the way -> 500 + 0.25*2000 = 1000.
    CHECK(c.angleToPulseUs(45.0f) == tst::approx(1000.0));
    // Halfway between two arbitrary angles maps halfway between their pulses.
    float mid = 0.5f * (c.angleToPulseUs(30.0f) + c.angleToPulseUs(120.0f));
    CHECK(c.angleToPulseUs(75.0f) == tst::approx(mid));
}

TEST_CASE("pulse clamps to the configured microsecond bounds") {
    ServoCalibration c;
    c.setAngleLimits(-90.0f, 270.0f);  // allow angles beyond the nominal travel
    // -90 would map below 500 us; 270 would map above 2500 us. Both clamp.
    CHECK(c.angleToPulseUs(-90.0f) == tst::approx(500.0));
    CHECK(c.angleToPulseUs(270.0f) == tst::approx(2500.0));
}

TEST_CASE("custom calibrated endpoints (setPulseRange)") {
    ServoCalibration c;
    c.setPulseRange(550.0f, 2450.0f);  // a real unit's true mechanical endpoints
    CHECK(c.angleToPulseUs(0.0f)   == tst::approx(550.0));
    CHECK(c.angleToPulseUs(180.0f) == tst::approx(2450.0));
    CHECK(c.angleToPulseUs(90.0f)  == tst::approx(1500.0));  // still centered
}

TEST_CASE("custom travel (e.g. a 270-degree unit)") {
    ServoCalibration c;
    c.setTravel(270.0f);
    c.setAngleLimits(0.0f, 270.0f);  // a 270-deg servo also gets a 270-deg soft range
    // Full 500..2500 us now spans 270 deg, so 135 deg is the midpoint.
    CHECK(c.angleToPulseUs(0.0f)   == tst::approx(500.0));
    CHECK(c.angleToPulseUs(135.0f) == tst::approx(1500.0));
    CHECK(c.angleToPulseUs(270.0f) == tst::approx(2500.0));
}

TEST_CASE("direction = -1 reverses the map") {
    ServoCalibration c;
    c.setDirection(-1);
    CHECK(c.angleToPulseUs(0.0f)   == tst::approx(2500.0));
    CHECK(c.angleToPulseUs(180.0f) == tst::approx(500.0));
    CHECK(c.angleToPulseUs(90.0f)  == tst::approx(1500.0));  // neutral unchanged
}

TEST_CASE("offsetDeg shifts the angle before mapping") {
    ServoCalibration c;
    c.setOffset(10.0f);
    // Commanding 80 behaves like commanding 90 without the offset.
    CHECK(c.angleToPulseUs(80.0f) == tst::approx(1500.0));
    CHECK(c.angleToPulseUs(0.0f)  == tst::approx(500.0 + (10.0 / 180.0) * 2000.0));
}

TEST_CASE("soft angle limits (minDeg/maxDeg) clamp the commanded angle") {
    ServoCalibration c;
    c.setAngleLimits(30.0f, 120.0f);
    CHECK(c.clampAngle(0.0f)   == tst::approx(30.0));   // below -> low limit
    CHECK(c.clampAngle(200.0f) == tst::approx(120.0));  // above -> high limit
    CHECK(c.clampAngle(75.0f)  == tst::approx(75.0));   // inside -> unchanged
    // And that clamp is reflected in the pulse.
    CHECK(c.angleToPulseUs(0.0f)   == tst::approx(c.angleToPulseUs(30.0f)));
    CHECK(c.angleToPulseUs(200.0f) == tst::approx(c.angleToPulseUs(120.0f)));
}

TEST_CASE("a NaN command collapses to a safe mid-range pulse (never garbage)") {
    ServoCalibration c;  // default limits [0, 180]
    const float pulse = c.angleToPulseUs(std::nan(""));
    CHECK(std::isfinite(pulse));
    CHECK(pulse == tst::approx(1500.0));  // mid of [0,180] -> 90 deg -> 1500 us
}

TEST_CASE("a calibration drives a fake servo output (the HAL seam)") {
    ServoCalibration c;
    FakeServoOutput fake;
    fake.writeMicroseconds(c.angleToPulseUs(90.0f));
    CHECK(fake.lastMicroseconds() == tst::approx(1500.0));
    CHECK(fake.writeCount() == 1);
    fake.writeMicroseconds(c.angleToPulseUs(0.0f));
    CHECK(fake.lastMicroseconds() == tst::approx(500.0));
    CHECK(fake.writeCount() == 2);
    CHECK(fake.history().front() == tst::approx(1500.0));
}

TEST_CASE("FakeClock advances deterministically") {
    FakeClock clk;
    CHECK(clk.nowMicros() == 0u);
    clk.setMicros(1000u);
    CHECK(clk.nowMicros() == 1000u);
    clk.advanceMicros(500u);
    CHECK(clk.nowMicros() == 1500u);
    clk.advanceSeconds(0.5f);
    CHECK(clk.nowMicros() == 501500u);
}
