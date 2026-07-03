// Vec2.h — a tiny 2D point/vector for the planar arm math.
//
// PURE CORE: no Arduino headers. The arm lives in one vertical X-Y plane, so a
// 2-component float vector is all the geometry needs. See CLAUDE.md section 10.
#pragma once

#include <cmath>

namespace roboarm {

// A point or vector in the arm's plane: x is horizontal ("forward"), y is up.
struct Vec2 {
    float x;
    float y;

    // Default is the origin (the shoulder pivot).
    constexpr Vec2() : x(0.0f), y(0.0f) {}
    constexpr Vec2(float xMm, float yMm) : x(xMm), y(yMm) {}

    // Vector addition / subtraction (component-wise).
    constexpr Vec2 operator+(const Vec2& o) const { return Vec2(x + o.x, y + o.y); }
    constexpr Vec2 operator-(const Vec2& o) const { return Vec2(x - o.x, y - o.y); }

    // Distance from the origin: sqrt(x*x + y*y). For the arm this is the
    // shoulder->point reach used by inverse kinematics.
    float length() const { return sqrtf(x * x + y * y); }

    // Direction from +X, in radians, via atan2 (handles all four quadrants).
    float angleRad() const { return atan2f(y, x); }
};

}  // namespace roboarm
