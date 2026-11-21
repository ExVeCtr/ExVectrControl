#ifndef EXVECTRCONTROL_EXECUTIVE_PLAN_HPP
#define EXVECTRCONTROL_EXECUTIVE_PLAN_HPP

/**
 * Flight plan: a table of phases (PhaseSpec rows) plus typed parameter tables. A plan is data only (enums, numbers
 * and indices, no function pointers), normally a constexpr table in flash. It must pass validatePlan() before a
 * FlightExecutive runs it.
 *
 * A phase row says which controllers run (tvc, flaps), which actuators are requested, which behaviour makes the
 * setpoints, how the phase reacts to an abort, its timeout, an optional entry condition, and up to kMaxExits
 * guarded exits in priority order.
 *
 * Rows are written with the constexpr builder below; every field not set keeps its safe default (everything off,
 * no timeout, abort ignored - the validator refuses an airborne phase that ignores an abort):
 *
 *   constexpr PhaseSpec kHover = PhaseSpec::make(1, "Hover")
 *       .withTvc(TvcMode::Position).withActuators(true, true).withFlaps(FlapMode::Retracted)
 *       .withBehaviour(Behaviour::GoTo, 0).withExit(Guard::InsideFor, 0, 2);
 */

#include <stddef.h>
#include <stdint.h>

#include "ExVectrControl/executive/behaviours.hpp"
#include "ExVectrControl/executive/exec_types.hpp"
#include "ExVectrControl/executive/guards.hpp"

namespace VCTR {
namespace Exec {

/// What a phase does with an abort level (operator abort, magnetometer timeout) or a no-position abort.
enum class AbortRule : uint8_t {
  Ignore = 0,   ///< nothing to abort (idle, failsafe)
  ResetAndStop, ///< on the ground: go to the plan's idle phase (full mission reset)
  ToRth,        ///< in flight: go to the plan's rth phase
  Stay,         ///< already recovering: keep the phase (its own exits continue)
  ToNoPos,      ///< in flight without position: go to the plan's noPos phase (T9b-7)
};

/// How the behaviour memory is set at phase entry.
enum class BehaviourEntry : uint8_t {
  Init = 0, ///< initialise from the estimate and the parameters
  Continue, ///< keep the memory if the previous phase ran the same behaviour with the same parameters, else Init
};

/// Entry action bits (PhaseSpec::entryActions).
constexpr uint8_t kEntryTvcActuatorTest = 1u << 0; ///< request one TVC actuator test (actuatorTestSeq + 1)

constexpr uint8_t kMaxExits = 4;
constexpr int64_t kNoTimeout = INT64_MAX;

struct Exit {
  Guard guard = Guard::None;
  uint8_t paramIdx = 0; ///< index into FlightPlan::guards
  PhaseId target = kNoPhase;
};

/// Condition for entering a phase. If it is false when the phase is targeted, the fallback is entered instead.
struct EntryGuard {
  Guard guard = Guard::None; ///< None = no condition
  uint8_t paramIdx = 0;
  PhaseId fallback = kNoPhase;
};

struct PhaseSpec {
  PhaseId id = kNoPhase;
  const char *name = "";
  TvcMode tvc = TvcMode::Off;
  FlapMode flaps = FlapMode::Disabled; ///< flap actuators are requested whenever flaps != Disabled
  bool tvcActuators = false;
  bool motors = false;                 ///< requires tvcActuators (validatePlan)
  float thrustCapN = kNoThrustCapN;
  Behaviour behaviour = Behaviour::Idle;
  uint8_t behaviourParamIdx = 0;
  BehaviourEntry behaviourEntry = BehaviourEntry::Init;
  uint8_t ctrlGroup = 0; ///< TVC controllers are reset when the group changes while they stay on
  BoundsProfile bounds = BoundsProfile::None;
  bool onGround = false;
  bool estimatorZeroing = false;
  uint8_t entryActions = 0; ///< kEntry* bits
  int64_t timeoutNs = kNoTimeout;
  PhaseId onTimeout = kNoPhase;
  AbortRule abort = AbortRule::Ignore;
  AbortRule abortNoPos = AbortRule::Ignore;
  EntryGuard entryGuard;
  Exit exits[kMaxExits];
  bool exitOverflow = false; ///< set by withExit() when more than kMaxExits were added (validatePlan refuses it)

  // ---- constexpr builder (each call returns a modified copy) ----

  static constexpr PhaseSpec make(PhaseId phaseId, const char *phaseName) {
    PhaseSpec s{};
    s.id = phaseId;
    s.name = phaseName;
    return s;
  }
  constexpr PhaseSpec withTvc(TvcMode mode, float capN = kNoThrustCapN) const {
    PhaseSpec s = *this;
    s.tvc = mode;
    s.thrustCapN = capN;
    return s;
  }
  constexpr PhaseSpec withActuators(bool tvcActuatorsOn, bool motorsOn) const {
    PhaseSpec s = *this;
    s.tvcActuators = tvcActuatorsOn;
    s.motors = motorsOn;
    return s;
  }
  constexpr PhaseSpec withFlaps(FlapMode mode) const {
    PhaseSpec s = *this;
    s.flaps = mode;
    return s;
  }
  constexpr PhaseSpec withBehaviour(Behaviour b, uint8_t paramIdx = 0,
                                    BehaviourEntry entry = BehaviourEntry::Init) const {
    PhaseSpec s = *this;
    s.behaviour = b;
    s.behaviourParamIdx = paramIdx;
    s.behaviourEntry = entry;
    return s;
  }
  constexpr PhaseSpec withCtrlGroup(uint8_t group) const {
    PhaseSpec s = *this;
    s.ctrlGroup = group;
    return s;
  }
  constexpr PhaseSpec withBounds(BoundsProfile b) const {
    PhaseSpec s = *this;
    s.bounds = b;
    return s;
  }
  constexpr PhaseSpec withOnGround(bool ground) const {
    PhaseSpec s = *this;
    s.onGround = ground;
    return s;
  }
  constexpr PhaseSpec withEstimatorZeroing(bool zeroing) const {
    PhaseSpec s = *this;
    s.estimatorZeroing = zeroing;
    return s;
  }
  constexpr PhaseSpec withEntryActions(uint8_t actions) const {
    PhaseSpec s = *this;
    s.entryActions = actions;
    return s;
  }
  constexpr PhaseSpec withTimeout(int64_t ns, PhaseId target) const {
    PhaseSpec s = *this;
    s.timeoutNs = ns;
    s.onTimeout = target;
    return s;
  }
  constexpr PhaseSpec withAbort(AbortRule rule, AbortRule noPosRule) const {
    PhaseSpec s = *this;
    s.abort = rule;
    s.abortNoPos = noPosRule;
    return s;
  }
  constexpr PhaseSpec withEntryGuard(Guard g, uint8_t paramIdx, PhaseId fallback) const {
    PhaseSpec s = *this;
    s.entryGuard.guard = g;
    s.entryGuard.paramIdx = paramIdx;
    s.entryGuard.fallback = fallback;
    return s;
  }
  /// Appends an exit (priority = order of the calls).
  constexpr PhaseSpec withExit(Guard g, uint8_t paramIdx, PhaseId target) const {
    PhaseSpec s = *this;
    for (uint8_t i = 0; i < kMaxExits; i++) {
      if (s.exits[i].guard == Guard::None) {
        s.exits[i].guard = g;
        s.exits[i].paramIdx = paramIdx;
        s.exits[i].target = target;
        return s;
      }
    }
    s.exitOverflow = true;
    return s;
  }
};

struct FlightPlan {
  const PhaseSpec *phases = nullptr;
  uint8_t phaseCount = 0;
  PhaseId idle = kNoPhase;     ///< boot phase, target of reset and ResetAndStop; go is accepted only here
  PhaseId first = kNoPhase;    ///< entered on go
  PhaseId failsafe = kNoPhase; ///< entered while the supervisor's failsafe latch is set
  PhaseId rth = kNoPhase;      ///< AbortRule::ToRth target
  PhaseId noPos = kNoPhase;    ///< AbortRule::ToNoPos target (may be kNoPhase if no phase uses ToNoPos)
  PhaseId complete = kNoPhase; ///< mission complete (landed): ExecStatus::missionComplete on entry
  BehaviourTables behaviours;
  const GuardParams *guards = nullptr;
  uint8_t guardCount = 0;
};

/// Index of the phase with this id in the plan, or -1.
int findPhase(const FlightPlan &plan, PhaseId id);

} // namespace Exec
} // namespace VCTR

#endif
