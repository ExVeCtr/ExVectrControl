#include "ExVectrControl/executive/sanitize.hpp"

#include <math.h>

#include "ExVectrControl/executive/exec_math.hpp"

namespace VCTR {
namespace Exec {

bool sanitizeCommand(ControlCommand &c, const EstimateSnapshot &est, const float lastPos[3]) {
  bool bad = false;

  if (!allFinite(c.posSp, 6)) {
    const float *hold = allFinite(&est.pos[3], 3) ? &est.pos[3] : lastPos;
    for (int i = 0; i < 3; i++) {
      c.posSp[i] = 0.0f;
      c.posSp[3 + i] = hold[i];
    }
    bad = true;
  }

  const float qn = c.attSp[0] * c.attSp[0] + c.attSp[1] * c.attSp[1] + c.attSp[2] * c.attSp[2] + c.attSp[3] * c.attSp[3];
  if (!allFinite(c.attSp, 4) || !(qn > 1e-6f)) {
    c.attSp[0] = 1.0f;
    c.attSp[1] = c.attSp[2] = c.attSp[3] = 0.0f;
    if (c.tvc == TvcMode::Attitude) {
      c.tvc = TvcMode::Off;
      c.thrustN = 0.0f;
      c.attSpSource = AttSpSource::None;
    }
    bad = true;
  }

  if (!isfinite(c.thrustN)) {
    c.thrustN = 0.0f;
    bad = true;
  }
  if (!isfinite(c.thrustCapN) || c.thrustCapN <= 0.0f) {
    c.thrustCapN = 0.0f;
    bad = true;
  }
  if (!isfinite(c.flapAzimuthRad)) {
    c.flapAzimuthRad = 0.0f;
    bad = true;
  }
  if (!isfinite(c.flapPitchRad)) {
    c.flapPitchRad = 0.0f;
    bad = true;
  }

  c.sanitized = bad;
  return bad;
}

} // namespace Exec
} // namespace VCTR
