#ifndef EXVECTRCONTROL_EXECUTIVE_VALIDATE_PLAN_HPP
#define EXVECTRCONTROL_EXECUTIVE_VALIDATE_PLAN_HPP

/**
 * Plan validator. Every plan must pass it before a FlightExecutive runs it (the vehicle's constexpr plans in a host
 * test; a plan uploaded later on the vehicle at load time). It checks structure (ids, targets, indices,
 * reachability) and safety rules; the vehicle supplies its physical limits.
 */

#include <stdint.h>

#include "ExVectrControl/executive/plan.hpp"

namespace VCTR {
namespace Exec {

/// Vehicle limits the plan's numbers must respect.
struct PlanLimits {
  float maxThrustN = 0;          ///< attitude-mode thrust requests and caps must be in (0, this]
  int64_t maxAttitudePhaseNs = 0; ///< TvcMode::Attitude phases need a finite timeout <= this
  float maxSpeedMps = 0;         ///< go-to and transit speeds
  float maxAccelMps2 = 0;        ///< go-to and transit accelerations
};

enum class PlanError : uint8_t {
  Ok = 0,
  NoPhases,               ///< null table or count 0
  TooManyPhases,          ///< more than kMaxPlanPhases
  DuplicateId,            ///< two rows with the same id, or a row with id kNoPhase
  MissingSpecialPhase,    ///< idle, first, failsafe, rth or complete is not in the table
  FirstIsIdle,            ///< go would not leave the idle phase
  UnknownTarget,          ///< exit, timeout, entry fallback or noPos target not in the table
  TooManyExits,           ///< withExit() overflowed
  BadParamIndex,          ///< behaviour or guard parameter index out of range
  BadParamValue,          ///< a non-finite or out-of-limit number
  BadTimeout,             ///< timeout <= 0, or finite without a target, or a target without a finite timeout
  AttitudeWithoutTimeout, ///< TvcMode::Attitude phase without a finite timeout <= maxAttitudePhaseNs
  AttitudeWithoutCap,     ///< TvcMode::Attitude phase without a thrust cap in (0, maxThrustN]
  MotorsWithoutTvcActuators,
  AirborneAbortIgnored,   ///< a phase that is not on the ground (and not the failsafe phase) ignores an abort
  UnsafeTerminalPhase,    ///< idle, complete or failsafe phase requests a controller, actuator or motor
  GuardNeedsTarget,       ///< a target-based guard on a phase whose behaviour has no target
  GuardNeedsBehaviour,    ///< KickAngleBelow without KickAttitude, LandingDetected without Land
  BadEntryGuard,          ///< entry guard without a valid fallback, or the fallback has an entry guard itself
  MissingAbortTarget,     ///< ToNoPos used but plan.noPos is kNoPhase
  Unreachable,            ///< a phase cannot be reached from idle
};

constexpr uint8_t kMaxPlanPhases = 32;

struct PlanCheck {
  PlanError error = PlanError::Ok;
  uint8_t phaseIndex = 0xFF; ///< row of the first error (0xFF: plan level)
  bool ok() const { return error == PlanError::Ok; }
};

PlanCheck validatePlan(const FlightPlan &plan, const PlanLimits &limits);

/// Short text for logs and test messages.
const char *toString(PlanError e);

} // namespace Exec
} // namespace VCTR

#endif
