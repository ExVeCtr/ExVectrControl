#include "ExVectrControl/executive/validate_plan.hpp"

#include <math.h>

#include "ExVectrControl/executive/exec_math.hpp"

namespace VCTR {
namespace Exec {

namespace {

bool finite3(const float v[3]) { return allFinite(v, 3); }
bool inRange(float v, float lo, float hi) { return isfinite(v) && v >= lo && v <= hi; }
bool positive(float v) { return isfinite(v) && v > 0.0f; }
/// Thrust-to-weight above 1, so the stop-distance formula is finite and positive.
bool canStop(float thrustN, float massKg) { return positive(massKg) && positive(thrustN) && thrustN / massKg > 9.81f; }

bool behaviourParamsOk(const PhaseSpec &ph, const FlightPlan &plan, const PlanLimits &lim) {
  const BehaviourTables &t = plan.behaviours;
  const uint8_t i = ph.behaviourParamIdx;
  switch (ph.behaviour) {
  case Behaviour::Idle:
  case Behaviour::HoldMeasured:
    return true;
  case Behaviour::GoTo: {
    const GoToParams &p = t.goTo[i];
    return finite3(p.target) && finite3(p.entryPos) && inRange(p.speedMps, 0, lim.maxSpeedMps) &&
           inRange(p.startAccelMps2, 0, lim.maxAccelMps2) && inRange(p.stopAccelMps2, 0, lim.maxAccelMps2);
  }
  case Behaviour::Transit: {
    const TransitParams &p = t.transit[i];
    return finite3(p.home) && positive(p.maxSpeedMps) && p.maxSpeedMps <= lim.maxSpeedMps && positive(p.accelMps2) &&
           p.accelMps2 <= lim.maxAccelMps2;
  }
  case Behaviour::Land: {
    const LandParams &p = t.land[i];
    return finite3(p.home) && positive(p.descentRateMps) && p.descentRateMps <= lim.maxSpeedMps &&
           positive(p.holdAboveM);
  }
  case Behaviour::KickAttitude: {
    const KickParams &p = t.kick[i];
    return inRange(p.pitchRad, 0.0f, kPi) && p.pitchRad > 0.0f && positive(p.thrustN) && p.thrustN <= lim.maxThrustN &&
           p.thrustN <= ph.thrustCapN;
  }
  case Behaviour::DescentGuidance: {
    const DescentParams &p = t.descent[i];
    const float all[] = {p.stopMarginM,  p.highAltitudeM,  p.offsetDistM, p.pitchSlopeRadPerM, p.minPitchOffsetRad,
                         p.pitchBiasRad, p.lowExtraMarginM, p.lowPitchRad};
    return allFinite(all, 8) && canStop(p.thrustN, p.massKg) && p.minPitchOffsetRad <= 0.0f;
  }
  }
  return false;
}

bool guardParamsOk(Guard g, const GuardParams &p) {
  switch (g) {
  case Guard::None:
  case Guard::MissionTimeAbove:
    return true;
  case Guard::InsideFor:
    return positive(p.distanceM) && p.timeNs >= 0;
  case Guard::Reached:
    return positive(p.distanceM);
  case Guard::NotReachedWithin:
    return positive(p.distanceM) && p.timeNs > 0;
  case Guard::SpeedBelow:
    return positive(p.speedMps);
  case Guard::StopAltitude:
    return canStop(p.thrustN, p.massKg) && isfinite(p.altitudeM);
  case Guard::KickAngleBelow:
    return positive(p.angleRad) && p.angleRad <= kPi;
  case Guard::PitchRateAbove:
    return positive(p.rateRadPs);
  case Guard::LandingDetected:
    return positive(p.distanceM) && p.timeNs > 0 && isfinite(p.groundMinM) && isfinite(p.groundMaxM) &&
           p.groundMinM < p.groundMaxM && positive(p.groundCovMax);
  case Guard::AltitudeAtLeast:
    return isfinite(p.altitudeM);
  }
  return false;
}

/// Guard g may be used on a phase running behaviour b.
PlanError guardFitsBehaviour(Guard g, Behaviour b) {
  if (guardNeedsTarget(g) && b != Behaviour::GoTo && b != Behaviour::Transit && b != Behaviour::Land)
    return PlanError::GuardNeedsTarget;
  if (g == Guard::KickAngleBelow && b != Behaviour::KickAttitude)
    return PlanError::GuardNeedsBehaviour;
  if (g == Guard::LandingDetected && b != Behaviour::Land)
    return PlanError::GuardNeedsBehaviour;
  return PlanError::Ok;
}

bool requestsNothing(const PhaseSpec &ph) {
  return ph.tvc == TvcMode::Off && ph.flaps == FlapMode::Disabled && !ph.tvcActuators && !ph.motors &&
         !ph.estimatorZeroing && ph.entryActions == 0;
}

bool validTarget(const FlightPlan &plan, PhaseId id) { return findPhase(plan, id) >= 0; }

PlanCheck fail(PlanError e, int idx = -1) {
  PlanCheck c;
  c.error = e;
  c.phaseIndex = idx < 0 ? 0xFF : uint8_t(idx);
  return c;
}

/// Marks the phase with this id reached, and its entry-guard fallback.
void markReached(const FlightPlan &plan, PhaseId id, bool reached[kMaxPlanPhases]) {
  const int i = findPhase(plan, id);
  if (i < 0)
    return;
  reached[i] = true;
  const int f = findPhase(plan, plan.phases[i].entryGuard.fallback);
  if (plan.phases[i].entryGuard.guard != Guard::None && f >= 0)
    reached[f] = true;
}

PhaseId abortTarget(const FlightPlan &plan, AbortRule r) {
  switch (r) {
  case AbortRule::ResetAndStop:
    return plan.idle;
  case AbortRule::ToRth:
    return plan.rth;
  case AbortRule::ToNoPos:
    return plan.noPos;
  default:
    return kNoPhase;
  }
}

PlanCheck checkPhase(const FlightPlan &plan, const PlanLimits &lim, int idx) {
  const PhaseSpec &ph = plan.phases[idx];

  if (ph.exitOverflow)
    return fail(PlanError::TooManyExits, idx);
  if (ph.behaviourParamIdx >= behaviourParamCount(ph.behaviour, plan.behaviours))
    return fail(PlanError::BadParamIndex, idx);
  if (!behaviourParamsOk(ph, plan, lim))
    return fail(PlanError::BadParamValue, idx);
  if (!isfinite(ph.thrustCapN) || ph.thrustCapN <= 0.0f)
    return fail(PlanError::BadParamValue, idx);

  // Timeout.
  if (ph.timeoutNs == kNoTimeout) {
    if (ph.onTimeout != kNoPhase)
      return fail(PlanError::BadTimeout, idx);
  } else if (ph.timeoutNs <= 0 || !validTarget(plan, ph.onTimeout)) {
    return fail(PlanError::BadTimeout, idx);
  }

  // Exits.
  for (uint8_t e = 0; e < kMaxExits; e++) {
    const Exit &x = ph.exits[e];
    if (x.guard == Guard::None) {
      if (x.target != kNoPhase)
        return fail(PlanError::UnknownTarget, idx);
      continue;
    }
    if (!validTarget(plan, x.target))
      return fail(PlanError::UnknownTarget, idx);
    const PlanError fit = guardFitsBehaviour(x.guard, ph.behaviour);
    if (fit != PlanError::Ok)
      return fail(fit, idx);
    if (x.paramIdx >= plan.guardCount || plan.guards == nullptr)
      return fail(PlanError::BadParamIndex, idx);
    if (!guardParamsOk(x.guard, plan.guards[x.paramIdx]))
      return fail(PlanError::BadParamValue, idx);
  }

  // Entry guard: evaluated before the phase's behaviour exists, so it may not use the behaviour.
  if (ph.entryGuard.guard != Guard::None) {
    const int f = findPhase(plan, ph.entryGuard.fallback);
    if (f < 0 || f == idx || plan.phases[f].entryGuard.guard != Guard::None)
      return fail(PlanError::BadEntryGuard, idx);
    if (ph.entryGuard.paramIdx >= plan.guardCount || plan.guards == nullptr)
      return fail(PlanError::BadParamIndex, idx);
    if (!guardParamsOk(ph.entryGuard.guard, plan.guards[ph.entryGuard.paramIdx]))
      return fail(PlanError::BadParamValue, idx);
    if (guardFitsBehaviour(ph.entryGuard.guard, Behaviour::Idle) != PlanError::Ok ||
        ph.entryGuard.guard == Guard::InsideFor || ph.entryGuard.guard == Guard::NotReachedWithin)
      return fail(PlanError::BadEntryGuard, idx);
  }

  // Safety rules.
  if (ph.motors && !ph.tvcActuators)
    return fail(PlanError::MotorsWithoutTvcActuators, idx);
  if (ph.tvc == TvcMode::Attitude) {
    if (ph.timeoutNs == kNoTimeout || ph.timeoutNs > lim.maxAttitudePhaseNs)
      return fail(PlanError::AttitudeWithoutTimeout, idx);
    if (ph.thrustCapN > lim.maxThrustN)
      return fail(PlanError::AttitudeWithoutCap, idx);
  }
  if (!ph.onGround && ph.id != plan.failsafe &&
      (ph.abort == AbortRule::Ignore || ph.abortNoPos == AbortRule::Ignore))
    return fail(PlanError::AirborneAbortIgnored, idx);
  if ((ph.abort == AbortRule::ToNoPos || ph.abortNoPos == AbortRule::ToNoPos) && !validTarget(plan, plan.noPos))
    return fail(PlanError::MissingAbortTarget, idx);
  if (ph.id == plan.idle || ph.id == plan.complete || ph.id == plan.failsafe) {
    if (!requestsNothing(ph))
      return fail(PlanError::UnsafeTerminalPhase, idx);
  }
  if (ph.id == plan.idle || ph.id == plan.failsafe) {
    // Left only by go (idle) or a reset: no timeout, no exits.
    if (ph.timeoutNs != kNoTimeout || ph.exits[0].guard != Guard::None)
      return fail(PlanError::UnsafeTerminalPhase, idx);
  }
  return PlanCheck{};
}

} // namespace

PlanCheck validatePlan(const FlightPlan &plan, const PlanLimits &lim) {
  if (plan.phases == nullptr || plan.phaseCount == 0)
    return fail(PlanError::NoPhases);
  if (plan.phaseCount > kMaxPlanPhases)
    return fail(PlanError::TooManyPhases);

  for (int i = 0; i < plan.phaseCount; i++) {
    if (plan.phases[i].id == kNoPhase)
      return fail(PlanError::DuplicateId, i);
    for (int j = 0; j < i; j++)
      if (plan.phases[j].id == plan.phases[i].id)
        return fail(PlanError::DuplicateId, i);
  }
  const PhaseId special[] = {plan.idle, plan.first, plan.failsafe, plan.rth, plan.complete};
  for (PhaseId id : special)
    if (!validTarget(plan, id))
      return fail(PlanError::MissingSpecialPhase);
  if (plan.first == plan.idle)
    return fail(PlanError::FirstIsIdle);
  if (plan.noPos != kNoPhase && !validTarget(plan, plan.noPos))
    return fail(PlanError::UnknownTarget);

  for (int i = 0; i < plan.phaseCount; i++) {
    const PlanCheck c = checkPhase(plan, lim, i);
    if (!c.ok())
      return c;
  }

  // Reachability from idle: go, the global failsafe and reset, exits, timeouts, abort targets, entry fallbacks.
  bool reached[kMaxPlanPhases] = {};
  markReached(plan, plan.idle, reached);
  markReached(plan, plan.first, reached);
  markReached(plan, plan.failsafe, reached);
  for (bool grew = true; grew;) {
    grew = false;
    for (int i = 0; i < plan.phaseCount; i++) {
      if (!reached[i])
        continue;
      const PhaseSpec &ph = plan.phases[i];
      PhaseId next[kMaxExits + 3];
      int n = 0;
      for (uint8_t e = 0; e < kMaxExits; e++)
        next[n++] = ph.exits[e].target;
      next[n++] = ph.onTimeout;
      next[n++] = abortTarget(plan, ph.abort);
      next[n++] = abortTarget(plan, ph.abortNoPos);
      for (int k = 0; k < n; k++) {
        const int j = findPhase(plan, next[k]);
        if (j < 0 || reached[j])
          continue;
        markReached(plan, next[k], reached);
        grew = true;
      }
    }
  }
  for (int i = 0; i < plan.phaseCount; i++)
    if (!reached[i])
      return fail(PlanError::Unreachable, i);

  return PlanCheck{};
}

const char *toString(PlanError e) {
  switch (e) {
  case PlanError::Ok:
    return "ok";
  case PlanError::NoPhases:
    return "no phases";
  case PlanError::TooManyPhases:
    return "too many phases";
  case PlanError::DuplicateId:
    return "duplicate or invalid phase id";
  case PlanError::MissingSpecialPhase:
    return "idle/first/failsafe/rth/complete phase missing";
  case PlanError::FirstIsIdle:
    return "first phase is the idle phase";
  case PlanError::UnknownTarget:
    return "unknown target phase";
  case PlanError::TooManyExits:
    return "too many exits";
  case PlanError::BadParamIndex:
    return "parameter index out of range";
  case PlanError::BadParamValue:
    return "parameter value not finite or out of limits";
  case PlanError::BadTimeout:
    return "bad timeout";
  case PlanError::AttitudeWithoutTimeout:
    return "attitude-mode phase without a finite timeout within the limit";
  case PlanError::AttitudeWithoutCap:
    return "attitude-mode phase without a thrust cap within the limit";
  case PlanError::MotorsWithoutTvcActuators:
    return "motors requested without TVC actuators";
  case PlanError::AirborneAbortIgnored:
    return "airborne phase ignores an abort";
  case PlanError::UnsafeTerminalPhase:
    return "idle/complete/failsafe phase requests something or can leave by itself";
  case PlanError::GuardNeedsTarget:
    return "guard needs a behaviour with a target";
  case PlanError::GuardNeedsBehaviour:
    return "guard needs a specific behaviour";
  case PlanError::BadEntryGuard:
    return "bad entry guard";
  case PlanError::MissingAbortTarget:
    return "ToNoPos used without a noPos phase";
  case PlanError::Unreachable:
    return "phase unreachable";
  }
  return "?";
}

} // namespace Exec
} // namespace VCTR
