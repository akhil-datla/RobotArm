// Phase 3 — TrapezoidProfile tests. A velocity/accel-limited approach to a
// setpoint: speed up, (maybe) cruise, slow down, arrive at rest.
#include <cmath>
#include <initializer_list>
#include "doctest.h"
#include "TestHelpers.h"
#include "TrapezoidProfile.h"

using namespace roboarm;

static TrapezoidProfile makeProfile(float vMax, float aMax) {
    return TrapezoidProfile(TrapezoidProfile::Constraints{vMax, aMax});
}

TEST_CASE("trapezoid respects max velocity and acceleration") {
    auto prof = makeProfile(/*vMax=*/20.0f, /*aMax=*/10.0f);
    TrapezoidProfile::State start{0.0f, 0.0f};
    TrapezoidProfile::State goal{100.0f, 0.0f};  // long move -> reaches cruise
    const float T = prof.totalTime(start, goal);
    float prevVel = 0.0f;
    const float dt = 0.01f;
    for (float t = 0.0f; t <= T; t += dt) {
        auto s = prof.calculate(t, start, goal);
        CHECK(std::fabs(s.velocity) <= 20.0f + 1e-3f);              // velocity cap
        const float accel = std::fabs(s.velocity - prevVel) / dt;
        CHECK(accel <= 10.0f + 1e-2f);                              // accel cap
        prevVel = s.velocity;
    }
}

TEST_CASE("triangular profile (short move) never reaches max velocity") {
    auto prof = makeProfile(/*vMax=*/100.0f, /*aMax=*/10.0f);
    TrapezoidProfile::State start{0.0f, 0.0f};
    TrapezoidProfile::State goal{5.0f, 0.0f};  // too short to reach vMax
    const float T = prof.totalTime(start, goal);
    float peak = 0.0f;
    for (float t = 0.0f; t <= T; t += 0.005f) {
        peak = std::max(peak, std::fabs(prof.calculate(t, start, goal).velocity));
    }
    // Peak velocity for a triangular move is sqrt(a*D) = sqrt(10*5) ~= 7.07.
    CHECK(peak == tst::approx(std::sqrt(10.0f * 5.0f), 0.1));
    CHECK(peak < 100.0f);
}

TEST_CASE("approach is monotonic and never overshoots") {
    auto prof = makeProfile(20.0f, 10.0f);
    TrapezoidProfile::State start{0.0f, 0.0f};
    TrapezoidProfile::State goal{100.0f, 0.0f};
    const float T = prof.totalTime(start, goal);
    float prevPos = 0.0f;
    for (float t = 0.0f; t <= T; t += 0.01f) {
        auto s = prof.calculate(t, start, goal);
        CHECK(s.position >= prevPos - 1e-4f);   // non-decreasing
        CHECK(s.position <= 100.0f + 1e-3f);    // never past the goal
        prevPos = s.position;
    }
}

TEST_CASE("arrives at the setpoint with ~zero velocity and holds") {
    auto prof = makeProfile(20.0f, 10.0f);
    TrapezoidProfile::State start{0.0f, 0.0f};
    TrapezoidProfile::State goal{100.0f, 0.0f};
    const float T = prof.totalTime(start, goal);
    auto atEnd = prof.calculate(T, start, goal);
    CHECK(atEnd.position == tst::approx(100.0));
    CHECK(atEnd.velocity == tst::approx(0.0, 1e-2));
    // Past the end, it holds the goal at rest.
    auto after = prof.calculate(T + 5.0f, start, goal);
    CHECK(after.position == tst::approx(100.0));
    CHECK(after.velocity == tst::approx(0.0, 1e-3));
    CHECK(prof.isFinished(T + 5.0f, start, goal));
    CHECK_FALSE(prof.isFinished(T * 0.5f, start, goal));
}

TEST_CASE("velocity profile is symmetric in and out") {
    auto prof = makeProfile(20.0f, 10.0f);
    TrapezoidProfile::State start{0.0f, 0.0f};
    TrapezoidProfile::State goal{100.0f, 0.0f};
    const float T = prof.totalTime(start, goal);
    // For a rest-to-rest move, accel and decel phases mirror each other:
    // v(t) == v(T - t).
    for (float t = 0.0f; t <= T * 0.5f; t += 0.02f) {
        auto a = prof.calculate(t, start, goal);
        auto b = prof.calculate(T - t, start, goal);
        CHECK(a.velocity == tst::approx(b.velocity, 1e-2));
    }
}

TEST_CASE("works in the negative direction") {
    auto prof = makeProfile(20.0f, 10.0f);
    TrapezoidProfile::State start{50.0f, 0.0f};
    TrapezoidProfile::State goal{-30.0f, 0.0f};  // move of -80
    const float T = prof.totalTime(start, goal);
    for (float t = 0.0f; t <= T; t += 0.01f) {
        auto s = prof.calculate(t, start, goal);
        CHECK(s.velocity <= 1e-3f);                 // never moves positive
        CHECK(s.velocity >= -20.0f - 1e-3f);        // within velocity cap
    }
    CHECK(prof.calculate(T, start, goal).position == tst::approx(-30.0));
}

TEST_CASE("trapezoid position matches the analytical profile at sampled times") {
    auto prof = makeProfile(20.0f, 10.0f);  // tAccel=2, cruise 2..5, total=7
    TrapezoidProfile::State start{0.0f, 0.0f}, goal{100.0f, 0.0f};
    // Accel phase: pos = 0.5*a*t^2, vel = a*t.
    CHECK(prof.calculate(1.0f, start, goal).position == tst::approx(5.0));
    CHECK(prof.calculate(1.0f, start, goal).velocity == tst::approx(10.0));
    // Cruise: pos = dAccel(20) + vPeak(20)*(t-2).
    CHECK(prof.calculate(3.0f, start, goal).position == tst::approx(40.0));
    CHECK(prof.calculate(3.0f, start, goal).velocity == tst::approx(20.0));
    // Decel: at t=6 (td=1), pos = 20+60+20-5 = 95, vel = 20-10 = 10.
    CHECK(prof.calculate(6.0f, start, goal).position == tst::approx(95.0));
    CHECK(prof.calculate(6.0f, start, goal).velocity == tst::approx(10.0));
    CHECK(prof.totalTime(start, goal) == tst::approx(7.0));
}

TEST_CASE("triangular-profile peak distance is at the halfway point") {
    auto prof = makeProfile(100.0f, 10.0f);  // short move -> triangle
    TrapezoidProfile::State start{0.0f, 0.0f}, goal{5.0f, 0.0f};
    const float T = prof.totalTime(start, goal);
    // Peak (turnaround) is at T/2 and exactly the halfway distance (2.5 mm).
    CHECK(prof.calculate(T * 0.5f, start, goal).position == tst::approx(2.5));
}

TEST_CASE("invalid constraints degrade to an instantaneous move (finite, no NaN)") {
    TrapezoidProfile::State start{10.0f, 0.0f}, goal{50.0f, 0.0f};
    for (auto prof : {makeProfile(0.0f, 10.0f), makeProfile(20.0f, 0.0f),
                      makeProfile(-5.0f, 10.0f)}) {
        CHECK(prof.totalTime(start, goal) == tst::approx(0.0));  // completes instantly
        CHECK(prof.calculate(0.0f, start, goal).position == tst::approx(10.0));  // at start
        auto s = prof.calculate(0.5f, start, goal);              // t>0 -> already done
        CHECK(std::isfinite(s.position));
        CHECK(s.position == tst::approx(50.0));  // at the goal, at rest
        CHECK(s.velocity == tst::approx(0.0));
    }
}

TEST_CASE("isFinished is true exactly at totalTime, false just before") {
    auto prof = makeProfile(20.0f, 10.0f);
    TrapezoidProfile::State start{0.0f, 0.0f}, goal{100.0f, 0.0f};
    const float T = prof.totalTime(start, goal);
    CHECK(prof.isFinished(T, start, goal));                 // >= boundary is inclusive
    CHECK_FALSE(prof.isFinished(T - 0.01f, start, goal));
}

TEST_CASE("zero-distance move is a no-op") {
    auto prof = makeProfile(20.0f, 10.0f);
    TrapezoidProfile::State start{10.0f, 0.0f};
    TrapezoidProfile::State goal{10.0f, 0.0f};
    CHECK(prof.totalTime(start, goal) == tst::approx(0.0));
    auto s = prof.calculate(0.0f, start, goal);
    CHECK(s.position == tst::approx(10.0));
    CHECK(s.velocity == tst::approx(0.0));
}
