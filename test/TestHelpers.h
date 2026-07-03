// Shared test tolerances and helpers (host-only).
#pragma once
#include "doctest.h"

namespace tst {
// A single family of tolerances per CLAUDE.md section 13: millimeters get 1e-3,
// degrees get 1e-2. Used everywhere via doctest::Approx so kinematics vectors are
// checked against Appendix A within the documented tolerance.
constexpr float kEps = 1e-3f;      // millimeters / general position tolerance
constexpr float kAngEps = 1e-2f;   // degrees

// doctest::Approx configured with our epsilon. doctest's Approx blends absolute
// and relative slack (tol = eps * (scale + max(|a|,|b|))), so near zero it is
// ~absolute (eps) and for larger magnitudes it stays comfortably tight for our
// millimeter/degree checks.
inline doctest::Approx approx(double value, double eps = kEps) {
    doctest::Approx a(value);
    a.epsilon(eps);
    return a;
}
inline doctest::Approx approxDeg(double value) { return approx(value, kAngEps); }
}  // namespace tst
