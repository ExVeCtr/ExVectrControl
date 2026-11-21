#include "ExVectrControl/executive/kick.hpp"

#include <math.h>

namespace VCTR {
namespace Exec {

KickEnd kickEndTrigger(int64_t elapsedNs, float remainingAngleRad, float pitchRateRadPs, const KickEndLimits &limits) {
  if (elapsedNs > limits.timeLimitNs)
    return KickEnd::Time;
  if (isfinite(remainingAngleRad) && remainingAngleRad < limits.endRemainingAngleRad)
    return KickEnd::Angle;
  if (isfinite(pitchRateRadPs) && fabsf(pitchRateRadPs) > limits.endPitchRateRadPs)
    return KickEnd::Rate;
  return KickEnd::None;
}

void kickRotate(const float q[4], const float v[3], float out[3]) {
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  // v' = v + 2 w (u x v) + 2 u x (u x v), u = (x, y, z)
  const float tx = 2.0f * (y * v[2] - z * v[1]);
  const float ty = 2.0f * (z * v[0] - x * v[2]);
  const float tz = 2.0f * (x * v[1] - y * v[0]);
  out[0] = v[0] + w * tx + (y * tz - z * ty);
  out[1] = v[1] + w * ty + (z * tx - x * tz);
  out[2] = v[2] + w * tz + (x * ty - y * tx);
}

void kickTargetAttitude(const float q0[4], float pitchRad, float out[4]) {
  static const float kBelly[3] = {-1.0f, 0.0f, 0.0f};
  float b[3];
  kickRotate(q0, kBelly, b);
  float psi = 0.0f;
  const float bh = sqrtf(b[0] * b[0] + b[1] * b[1]);
  if (isfinite(bh) && bh > 0.1f)
    psi = atan2f(-b[1], -b[0]); // Rz(psi) maps world -X onto the horizontal belly direction
  // out = Rz(psi) * Ry(-pitch)
  const float cz = cosf(0.5f * psi), sz = sinf(0.5f * psi);
  const float cy = cosf(-0.5f * pitchRad), sy = sinf(-0.5f * pitchRad);
  float w = cz * cy;
  float x = -sz * sy;
  float y = cz * sy;
  float z = sz * cy;
  if (w < 0) {
    w = -w;
    x = -x;
    y = -y;
    z = -z;
  }
  out[0] = w;
  out[1] = x;
  out[2] = y;
  out[3] = z;
}

float kickRemainingAngle(const float q[4], const float qTarget[4]) {
  static const float kX[3] = {1.0f, 0.0f, 0.0f};
  float a[3], t[3];
  kickRotate(q, kX, a);
  kickRotate(qTarget, kX, t);
  const float na = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  const float nt = sqrtf(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
  if (!isfinite(na) || !isfinite(nt) || na < 1e-6f || nt < 1e-6f)
    return NAN;
  float c = (a[0] * t[0] + a[1] * t[1] + a[2] * t[2]) / (na * nt);
  if (c > 1.0f)
    c = 1.0f;
  else if (c < -1.0f)
    c = -1.0f;
  return acosf(c);
}

} // namespace Exec
} // namespace VCTR
