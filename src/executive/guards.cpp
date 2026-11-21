#include "ExVectrControl/executive/guards.hpp"

#include <math.h>

#include "ExVectrControl/executive/exec_math.hpp"
#include "ExVectrControl/executive/kick.hpp"

namespace VCTR {
namespace Exec {

namespace {

float distanceToTarget(const GuardContext &ctx) {
  float d[3];
  sub3(ctx.target, &ctx.est->pos[3], d);
  return norm3(d);
}

} // namespace

void guardInit(Guard, int64_t nowNs, GuardMem &mem) {
  mem.sinceNs = nowNs;
  mem.flag = false;
}

bool guardNeedsTarget(Guard g) { return g == Guard::InsideFor || g == Guard::Reached || g == Guard::NotReachedWithin; }

bool guardEval(Guard g, const GuardParams &p, const GuardContext &ctx, GuardMem &mem) {
  const EstimateSnapshot &est = *ctx.est;
  if (guardNeedsTarget(g) && !ctx.hasTarget)
    return false;
  switch (g) {
  case Guard::None:
    return false;

  case Guard::MissionTimeAbove:
    return ctx.missionTimeNs > p.timeNs;

  case Guard::InsideFor:
    if (!(distanceToTarget(ctx) < p.distanceM)) {
      mem.sinceNs = ctx.nowNs;
      return false;
    }
    return ctx.nowNs - mem.sinceNs > p.timeNs;

  case Guard::Reached:
    return distanceToTarget(ctx) < p.distanceM;

  case Guard::NotReachedWithin:
    if (distanceToTarget(ctx) < p.distanceM) {
      mem.flag = true;
      return false;
    }
    return !mem.flag && ctx.elapsedNs > p.timeNs;

  case Guard::SpeedBelow:
    return norm3(&est.pos[0]) < p.speedMps;

  case Guard::StopAltitude:
    return est.pos[5] < descentStopDistanceM(est.pos[2], p.thrustN, p.massKg) + p.altitudeM;

  case Guard::KickAngleBelow: {
    if (ctx.mem->active != Behaviour::KickAttitude)
      return false;
    const float remaining = kickRemainingAngle(&est.att[3], ctx.mem->kick.target);
    return isfinite(remaining) && remaining < p.angleRad;
  }

  case Guard::PitchRateAbove:
    return isfinite(est.att[1]) && fabsf(est.att[1]) > p.rateRadPs;

  case Guard::LandingDetected: {
    if (ctx.mem->active != Behaviour::Land)
      return false;
    if (est.pos[5] - ctx.mem->land.spZ < p.distanceM)
      mem.sinceNs = ctx.nowNs;
    const bool tracking = ctx.nowNs - mem.sinceNs > p.timeNs;
    const bool ground = est.groundFresh && est.groundDist < p.groundMaxM && est.groundDist > p.groundMinM &&
                        est.groundCov < p.groundCovMax;
    return tracking || ground;
  }

  case Guard::AltitudeAtLeast:
    return est.posValid && isfinite(est.pos[5]) && est.pos[5] >= p.altitudeM;
  }
  return false;
}

} // namespace Exec
} // namespace VCTR
