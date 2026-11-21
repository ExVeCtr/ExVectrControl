#ifndef EXVECTRCONTROL_EXECUTIVE_FLIGHT_EXECUTIVE_HPP
#define EXVECTRCONTROL_EXECUTIVE_FLIGHT_EXECUTIVE_HPP

/**
 * Flight executive engine. Each step takes ExecInputs and returns one ControlCommand.
 *
 * Step semantics (deterministic, docs/project/flight-executive.md):
 * 1. Global transitions, first match:
 *    a. sup.failsafe          -> plan.failsafe (nothing else is evaluated while the latch is set);
 *    b. cmd.resetEdge         -> plan.idle (full mission reset);
 *    c. sup.abortNoPos        -> the phase's abortNoPos rule;
 *    d. cmd.abortRequest or sup.magTimeout -> the phase's abort rule;
 *    e. cmd.goEdge in idle    -> plan.first (refused outside idle, and while an abort level of c/d is active).
 *    An abort rule Ignore or Stay causes no transition; the phase's own timeout and exits then still run.
 * 2. Phase timeout (time in phase >= timeoutNs) -> onTimeout.
 * 3. Exits in table order; the first true guard wins.
 * At most one transition per step. A transition to a phase whose entry guard is false enters its fallback instead
 * (still one transition). On entry: phaseSeq + 1, guard memories and (unless BehaviourEntry::Continue applies) the
 * behaviour are initialised from this step's estimate, entry actions run. Entering the idle phase clears all mission
 * state. Then the (new) phase's behaviour steps and the command is assembled from the phase row and sanitised, so
 * the new phase's command is produced in the same step.
 *
 * TVC reset rule: resetTvc is true for one step when the TVC mode becomes non-Off after Off, or stays non-Off and
 * the phase's ctrlGroup differs from the previous step's.
 *
 * The object is copyable (tests clone it to branch scenarios). It holds a pointer to the plan, which must outlive
 * it and must have passed validatePlan().
 */

#include <stdint.h>

#include "ExVectrControl/executive/behaviours.hpp"
#include "ExVectrControl/executive/exec_types.hpp"
#include "ExVectrControl/executive/guards.hpp"
#include "ExVectrControl/executive/plan.hpp"

namespace VCTR {
namespace Exec {

class FlightExecutive {
public:
  explicit FlightExecutive(const FlightPlan &plan);

  /// Exactly one call per executive tick.
  const ControlCommand &step(const ExecInputs &in);

  /// Boot state (as after construction, except that phaseSeq is kept). The reset edge does the same in a step.
  void reset();

  PhaseId phase() const { return plan_->phases[phaseIdx_].id; }
  const PhaseSpec &phaseSpec() const { return plan_->phases[phaseIdx_]; }
  /// Committed to a flight: the phase is not idle, complete or failsafe.
  bool committed() const;
  const ControlCommand &command() const { return cmd_; }
  const ExecStatus &status() const { return status_; }
  /// Time of the current phase's entry (for logs and tests).
  int64_t phaseEntryNs() const { return phaseEntryNs_; }
  const BehaviourMem &behaviourMemory() const { return behMem_; }

private:
  struct Transition {
    PhaseId target = kNoPhase;
    ExitReason reason = ExitReason::None;
  };

  Transition globalTransition(const ExecInputs &in, bool &stopHere);
  Transition abortTransition(AbortRule rule, ExitReason reason);
  Transition phaseTransition(const ExecInputs &in);
  void enter(PhaseId target, ExitReason reason, const ExecInputs &in);
  void produceCommand(const ExecInputs &in, float dtS);
  int64_t missionTime(int64_t nowNs) const;
  GuardContext guardContext(const ExecInputs &in) const;
  void clearMissionState();

  const FlightPlan *plan_;
  int phaseIdx_ = 0;
  uint32_t phaseSeq_ = 0;
  int64_t phaseEntryNs_ = 0;
  bool clockRunning_ = false;
  int64_t goNs_ = 0;
  int64_t missionStartNs_ = 0; ///< mission time at go
  uint16_t actuatorTestSeq_ = 0;
  BehaviourMem behMem_;
  GuardMem guardMem_[kMaxExits];
  TvcMode prevTvc_ = TvcMode::Off;
  uint8_t prevCtrlGroup_ = 0;
  float lastPos_[3] = {0, 0, 0}; ///< last finite setpoint position (sanitiser fallback)
  ControlCommand cmd_;
  ExecStatus status_;
};

} // namespace Exec
} // namespace VCTR

#endif
