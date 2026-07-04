// MathUtils.h — tiny pure math helpers used across the whole library.
//
// PURE CORE: no Arduino headers, no dynamic memory, float-first for AVR. These
// are the deg/rad conversions, clamping, interpolation, and angle wrapping that
// every higher layer leans on. See CLAUDE.md sections 4.3 and 10.
#pragma once

#include <cmath>

namespace roboarm {

// Pi as a float. AVR has no FPU, so the library works in float throughout.
constexpr float kPi = 3.14159265358979323846f;

// Convert an angle in degrees to radians. The public API speaks degrees; the
// internal trig speaks radians. Convert exactly once at each boundary.
constexpr float degToRad(float deg) { return deg * (kPi / 180.0f); }

// Convert an angle in radians (internal) back to degrees (public).
constexpr float radToDeg(float rad) { return rad * (180.0f / kPi); }

// Clamp a value into [lo, hi]. Templated so it works for float angles/pulses and
// for integer pulse widths alike. This is a core safety primitive: it is what
// keeps a commanded value from ever leaving its allowed range.
template <typename T>
constexpr T clamp(T value, T lo, T hi) {
    return value < lo ? lo : (value > hi ? hi : value);
}

// Linear interpolation: t=0 gives a, t=1 gives b. The straight-line map behind
// the angle<->pulse calibration.
constexpr float lerp(float a, float b, float t) { return a + (b - a) * t; }

// Wrap an angle in degrees into a single turn, the half-open range [-180, 180).
// Branch-free and float-only so it is cheap on an Uno. Examples:
//   190 -> -170, -190 -> 170, 360 -> 0, 540 -> -180.
inline float wrapAngleDeg(float deg) {
    return deg - 360.0f * floorf((deg + 180.0f) / 360.0f);
}

}  // namespace roboarm
