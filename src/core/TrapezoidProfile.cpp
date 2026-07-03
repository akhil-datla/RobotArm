#include "TrapezoidProfile.h"

#include <cmath>

namespace roboarm {

namespace {

// The pre-computed shape of a rest-to-rest move over a non-negative distance D.
struct Shape {
    float dir;       // +1 or -1 (direction of travel), 0 for a zero move
    float distance;  // |goal - start|
    float vPeak;     // cruise speed actually reached (<= maxVelocity)
    float tAccel;    // time spent accelerating (== time decelerating)
    float tCruise;   // time spent at vPeak (0 for a triangular move)
    float dAccel;    // distance covered while accelerating
    float total;     // total move time
};

Shape shapeOf(const TrapezoidProfile::Constraints& c,
              const TrapezoidProfile::State& start,
              const TrapezoidProfile::State& goal) {
    Shape s{};
    const float delta = goal.position - start.position;
    s.distance = std::fabs(delta);
    s.dir = (delta > 0.0f) ? 1.0f : (delta < 0.0f ? -1.0f : 0.0f);

    // Degenerate / invalid constraints: treat as an instantaneous move.
    if (s.distance == 0.0f || c.maxVelocity <= 0.0f || c.maxAcceleration <= 0.0f) {
        s.vPeak = 0.0f;
        s.tAccel = 0.0f;
        s.tCruise = 0.0f;
        s.dAccel = 0.0f;
        s.total = 0.0f;
        return s;
    }

    const float a = c.maxAcceleration;
    const float v = c.maxVelocity;

    // Distance needed to accelerate from rest to the cruise speed (and the same
    // again to stop). If that fits within the move, it's a full trapezoid.
    const float dAccelFull = (v * v) / (2.0f * a);
    if (2.0f * dAccelFull <= s.distance) {
        s.vPeak = v;
        s.tAccel = v / a;
        s.dAccel = dAccelFull;
        const float dCruise = s.distance - 2.0f * dAccelFull;
        s.tCruise = dCruise / v;
    } else {
        // Too short to reach cruise speed: a triangular profile peaking at
        // sqrt(a*D), reached at exactly the halfway distance.
        s.vPeak = std::sqrt(a * s.distance);
        s.tAccel = s.vPeak / a;
        s.dAccel = s.distance * 0.5f;
        s.tCruise = 0.0f;
    }
    s.total = 2.0f * s.tAccel + s.tCruise;
    return s;
}

}  // namespace

TrapezoidProfile::TrapezoidProfile(const Constraints& constraints)
    : m_c(constraints) {}

float TrapezoidProfile::totalTime(const State& start, const State& goal) const {
    return shapeOf(m_c, start, goal).total;
}

bool TrapezoidProfile::isFinished(float t, const State& start, const State& goal) const {
    return t >= shapeOf(m_c, start, goal).total;
}

TrapezoidProfile::State TrapezoidProfile::calculate(float t, const State& start,
                                                    const State& goal) const {
    const Shape s = shapeOf(m_c, start, goal);

    // Magnitude of position (from start) and velocity along the travel direction.
    float pos;
    float vel;

    const float a = m_c.maxAcceleration;
    const float tCruiseEnd = s.tAccel + s.tCruise;

    if (t <= 0.0f || s.dir == 0.0f) {
        pos = 0.0f;
        vel = 0.0f;
    } else if (t < s.tAccel) {
        // Accelerating.
        vel = a * t;
        pos = 0.5f * a * t * t;
    } else if (t < tCruiseEnd) {
        // Cruising at the peak speed.
        vel = s.vPeak;
        pos = s.dAccel + s.vPeak * (t - s.tAccel);
    } else if (t < s.total) {
        // Decelerating to rest.
        const float td = t - tCruiseEnd;
        vel = s.vPeak - a * td;
        pos = s.dAccel + s.vPeak * s.tCruise + s.vPeak * td - 0.5f * a * td * td;
    } else {
        // Move complete: hold the goal at rest.
        pos = s.distance;
        vel = 0.0f;
    }

    State out;
    out.position = start.position + s.dir * pos;
    out.velocity = s.dir * vel;
    return out;
}

}  // namespace roboarm
