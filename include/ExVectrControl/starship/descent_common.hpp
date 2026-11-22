// Shared helpers of the descent (belly-down) guidance and flap law: angle helpers and the belly attitude.
//
// Pure, header-only, no scheduler, globals or heap. Plain float arrays in the ExVectr conventions (NWU world,
// attitude [w(3), q(4)] with q = [w,x,y,z] body -> world, SI units, angles in rad). Body frame: +Z nose, belly -X,
// flaps TL/BL on +Y, TR/BR on -Y. Design: ExVectrStarship docs/descent-control-design.md.
#ifndef EXVECTRCONTROL_STARSHIP_DESCENT_COMMON_HPP
#define EXVECTRCONTROL_STARSHIP_DESCENT_COMMON_HPP

#include <math.h>
#include <stdint.h>

#include <cmath>

namespace VCTR {
namespace CTRL {
namespace descent {

static constexpr float kPi = 3.14159265f;
static constexpr float kDegF = kPi / 180.0f;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float wrapPi(float a) {
  while (a > kPi)
    a -= 2 * kPi;
  while (a < -kPi)
    a += 2 * kPi;
  return a;
}
/// v_world = q * v_body * q^-1, q = [w,x,y,z].
inline void rotate(const float q[4], const float v[3], float out[3]) {
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  const float tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
  out[0] = v[0] + w * tx + (y * tz - z * ty);
  out[1] = v[1] + w * ty + (z * tx - x * tz);
  out[2] = v[2] + w * tz + (x * ty - y * tx);
}

/// Belly-down attitude quantities from [w, q]: nose azimuth (CCW from North), nose elevation (pitch, + nose up),
/// roll about the nose (+ = positive rotation about body Z), belly angle (body +X vs world up).
struct BellyAttitude {
  float heading, pitch, roll, belly;
  bool headingValid; ///< false when the nose is within 30 deg of vertical (azimuth meaningless)
};
inline BellyAttitude bellyAttitude(const float att[7]) {
  const float ex[3] = {1, 0, 0}, ey[3] = {0, 1, 0}, ez[3] = {0, 0, 1};
  float xw[3], yw[3], zw[3];
  rotate(att + 3, ex, xw);
  rotate(att + 3, ey, yw);
  rotate(att + 3, ez, zw);
  BellyAttitude b;
  const float horiz = sqrtf(zw[0] * zw[0] + zw[1] * zw[1]);
  b.heading = atan2f(zw[1], zw[0]);
  b.pitch = atan2f(zw[2], horiz);
  b.roll = -asinf(clampf(yw[2], -1, 1));
  b.belly = acosf(clampf(xw[2], -1, 1));
  b.headingValid = horiz > 0.5f;
  return b;
}

/// True when all n values are finite (no NaN, no infinity).
inline bool allFinite(const float *v, int n) {
  for (int i = 0; i < n; i++)
    if (!std::isfinite(v[i]))
      return false;
  return true;
}

} // namespace descent
} // namespace CTRL
} // namespace VCTR

#endif
