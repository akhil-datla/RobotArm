// TrapezoidProfile.h — a velocity/acceleration-limited move to a setpoint.
//
// THE IDEA: to move a joint from A to B nicely you don't just cap the speed, you
// also cap how fast the speed itself changes (acceleration). The result is the
// classic "trapezoid": ramp up to a cruise speed, hold it, then ramp down to
// arrive exactly on target at rest. For short moves there is no room to reach the
// cruise speed, so the profile is a triangle instead. This models rest-to-rest
// moves (start and end both stationary), which is what a joint going to a new
// angle actually does.
//
// PURE CORE: no Arduino, no clock — you query the state at a time you supply.
// See CLAUDE.md 3, 11 and Appendix intent.
#pragma once

namespace roboarm {

// A trapezoidal (or triangular) motion profile between two rest positions.
class TrapezoidProfile {
public:
    // Motion limits: the top cruise speed and the acceleration used to reach it.
    struct Constraints {
        float maxVelocity;      // units per second
        float maxAcceleration;  // units per second^2
    };

    // A point on the profile: where you are and how fast you're moving.
    struct State {
        float position;
        float velocity;
    };

    explicit TrapezoidProfile(const Constraints& constraints);

    // The state at time t seconds into the move from `start` to `goal` (both
    // treated as rest states). Before t=0 it returns the start; after the move
    // finishes it holds the goal at zero velocity.
    State calculate(float t, const State& start, const State& goal) const;

    // Total duration of the move (accel + optional cruise + decel), in seconds.
    float totalTime(const State& start, const State& goal) const;

    // True once t is at or beyond the end of the move.
    bool isFinished(float t, const State& start, const State& goal) const;

private:
    Constraints m_c;
};

}  // namespace roboarm
