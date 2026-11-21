#ifndef EXVECTRCONTROL_EXECUTIVE_EXEC_MATH_HPP
#define EXVECTRCONTROL_EXECUTIVE_EXEC_MATH_HPP

/**
 * Small vector helpers of the executive core on plain float arrays (the core does not depend on ExVectrMath; see
 * docs/project/flight-executive.md, "Include boundary").
 */

#include <math.h>

namespace VCTR {
namespace Exec {

constexpr float kPi = 3.14159265358979f;
constexpr float kDegToRad = kPi / 180.0f;

/// Euclidean norm of a 3-vector.
inline float norm3(const float v[3]) { return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

/// out = a - b.
inline void sub3(const float a[3], const float b[3], float out[3]) {
  out[0] = a[0] - b[0];
  out[1] = a[1] - b[1];
  out[2] = a[2] - b[2];
}

/**
 * Unit vector of v, or zero if |v| < 0.0001. Same rule as ExVectrMath normalize() (zeroReturn), which the legacy
 * waypoint profile used; the go-to behaviour reproduces it bit for bit.
 */
inline void normalizeOrZero3(const float v[3], float out[3]) {
  const float m = norm3(v);
  if (double(m) < 0.0001) { // double literal as in ExVectrMath, for bit parity
    out[0] = out[1] = out[2] = 0.0f;
    return;
  }
  out[0] = v[0] / m;
  out[1] = v[1] / m;
  out[2] = v[2] / m;
}

inline bool allFinite(const float *v, int n) {
  for (int i = 0; i < n; i++)
    if (!isfinite(v[i]))
      return false;
  return true;
}

} // namespace Exec
} // namespace VCTR

#endif
