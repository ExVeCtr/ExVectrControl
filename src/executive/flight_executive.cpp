#include "ExVectrControl/executive/flight_executive.hpp"

#include "ExVectrControl/executive/sanitize.hpp"

namespace VCTR {
namespace Exec {

FlightExecutive::FlightExecutive(const FlightPlan &plan) : plan_(&plan) { reset(); }

void FlightExecutive::reset() {
  const int idle = findPhase(*plan_, plan_->idle);
  phaseIdx_ = idle >= 0 ? idle : 0;
  phaseEntryNs_ = 0;
  clearMissionState();
  prevTvc_ = TvcMode::Off;
  prevCtrlGroup_ = 0;
  cmd_ = ControlCommand{};
  cmd_.phase = phase();
  cmd_.phaseSeq = phaseSeq_;
  cmd_.actuatorTestSeq = actuatorTestSeq_;
  status_ = ExecStatus{};
}

void FlightExecutive::clearMissionState() {
  clockRunning_ = false;
  goNs_ = 0;
  missionStartNs_ = 0;
  behMem_ = BehaviourMem{};
  for (GuardMem &g : guardMem_)
    g = GuardMem{};
  for (float &p : lastPos_)
    p = 0.0f;
}

bool FlightExecutive::committed() const {
  const PhaseId id = phase();
  return id != plan_->idle && id != plan_->complete && id != plan_->failsafe;
}

int64_t FlightExecutive::missionTime(int64_t nowNs) const {
  return clockRunning_ ? missionStartNs_ + (nowNs - goNs_) : 0;
}

const ControlCommand &FlightExecutive::step(const ExecInputs &in) {
  status_ = ExecStatus{};
  const float dtS = in.dtNs > 0 ? float(in.dtNs) / float(kNsPerS) : 0.0f;

  bool stopHere = false;
  Transition t = globalTransition(in, stopHere);
  if (t.target == kNoPhase && !stopHere)
    t = phaseTransition(in);
  if (t.target != kNoPhase)
    enter(t.target, t.reason, in);

  produceCommand(in, dtS);
  return cmd_;
}

FlightExecutive::Transition FlightExecutive::abortTransition(AbortRule rule, ExitReason reason) {
  Transition t;
  switch (rule) {
  case AbortRule::Ignore:
    status_.abort = ExecStatus::AbortOutcome::Ignored;
    break;
  case AbortRule::Stay:
    status_.abort = ExecStatus::AbortOutcome::Stayed;
    break;
  case AbortRule::ResetAndStop:
    status_.abort = ExecStatus::AbortOutcome::ResetAndStop;
    t.target = plan_->idle;
    break;
  case AbortRule::ToRth:
    status_.abort = ExecStatus::AbortOutcome::ToRth;
    t.target = plan_->rth;
    break;
  case AbortRule::ToNoPos:
    status_.abort = ExecStatus::AbortOutcome::ToNoPos;
    t.target = plan_->noPos;
    break;
  }
  if (t.target == phase())
    t.target = kNoPhase; // already there: nothing to do
  t.reason = reason;
  return t;
}

FlightExecutive::Transition FlightExecutive::globalTransition(const ExecInputs &in, bool &stopHere) {
  const PhaseSpec &ph = phaseSpec();
  Transition t;
  if (in.sup.failsafe) {
    stopHere = true; // nothing else while the latch is set (not even a reset: the supervisor clears it first)
    if (phase() != plan_->failsafe) {
      t.target = plan_->failsafe;
      t.reason = ExitReason::Failsafe;
    }
    return t;
  }
  if (in.cmd.resetEdge) {
    t.target = plan_->idle; // also from idle: re-entry clears the mission state
    t.reason = ExitReason::Reset;
    return t;
  }
  bool abortActive = false;
  if (in.sup.abortNoPos) {
    abortActive = true;
    t = abortTransition(ph.abortNoPos, ExitReason::AbortNoPos);
  } else if (in.cmd.abortRequest || in.sup.magTimeout) {
    abortActive = true;
    t = abortTransition(ph.abort, ExitReason::Abort);
  }
  if (in.cmd.goEdge) {
    if (phase() != plan_->idle) {
      status_.go = ExecStatus::GoResult::RefusedNotIdle;
    } else if (abortActive) {
      status_.go = ExecStatus::GoResult::RefusedBlocked;
    } else {
      status_.go = ExecStatus::GoResult::Accepted;
      t.target = plan_->first;
      t.reason = ExitReason::Go;
    }
  }
  return t;
}

GuardContext FlightExecutive::guardContext(const ExecInputs &in) const {
  const PhaseSpec &ph = phaseSpec();
  GuardContext ctx;
  ctx.est = &in.est;
  ctx.mem = &behMem_;
  ctx.nowNs = in.nowNs;
  ctx.elapsedNs = in.nowNs - phaseEntryNs_;
  ctx.missionTimeNs = missionTime(in.nowNs);
  ctx.hasTarget = behaviourTarget(ph.behaviour, ph.behaviourParamIdx, plan_->behaviours, ctx.target);
  return ctx;
}

FlightExecutive::Transition FlightExecutive::phaseTransition(const ExecInputs &in) {
  const PhaseSpec &ph = phaseSpec();
  Transition t;
  if (ph.timeoutNs != kNoTimeout && in.nowNs - phaseEntryNs_ >= ph.timeoutNs) {
    t.target = ph.onTimeout;
    t.reason = ExitReason::Timeout;
    return t;
  }
  const GuardContext ctx = guardContext(in);
  for (uint8_t i = 0; i < kMaxExits; i++) {
    const Exit &x = ph.exits[i];
    if (x.guard == Guard::None)
      continue;
    if (guardEval(x.guard, plan_->guards[x.paramIdx], ctx, guardMem_[i])) {
      t.target = x.target;
      t.reason = ExitReason(uint8_t(ExitReason::Exit0) + i);
      return t;
    }
  }
  return t;
}

void FlightExecutive::enter(PhaseId target, ExitReason reason, const ExecInputs &in) {
  int idx = findPhase(*plan_, target);
  if (idx < 0) // cannot happen with a validated plan; fail safe
    idx = findPhase(*plan_, plan_->failsafe);
  if (idx < 0)
    return;

  // Entry guard: refused entry goes to the fallback (validatePlan: the fallback has no entry guard).
  const EntryGuard &eg = plan_->phases[idx].entryGuard;
  if (eg.guard != Guard::None) {
    GuardContext ctx;
    ctx.est = &in.est;
    ctx.mem = &behMem_;
    ctx.nowNs = in.nowNs;
    ctx.missionTimeNs = missionTime(in.nowNs);
    GuardMem scratch;
    guardInit(eg.guard, in.nowNs, scratch);
    if (!guardEval(eg.guard, plan_->guards[eg.paramIdx], ctx, scratch)) {
      const int f = findPhase(*plan_, eg.fallback);
      if (f >= 0) {
        idx = f;
        status_.entryRedirected = true;
      }
    }
  }

  const PhaseSpec &next = plan_->phases[idx];
  if (next.id == plan_->idle)
    clearMissionState();
  if (reason == ExitReason::Go) {
    clockRunning_ = true;
    goNs_ = in.nowNs;
    missionStartNs_ = int64_t(in.cmd.legacyStartS) * kNsPerS;
  }

  const bool keepBehaviour = next.behaviourEntry == BehaviourEntry::Continue && behMem_.active == next.behaviour &&
                             behMem_.paramIdx == next.behaviourParamIdx;
  phaseIdx_ = idx;
  phaseSeq_++;
  phaseEntryNs_ = in.nowNs;
  for (uint8_t i = 0; i < kMaxExits; i++)
    guardInit(next.exits[i].guard, in.nowNs, guardMem_[i]);
  if (!keepBehaviour)
    behaviourInit(next.behaviour, next.behaviourParamIdx, plan_->behaviours, in.est, behMem_);
  if (next.entryActions & kEntryTvcActuatorTest)
    actuatorTestSeq_++;
  if (next.id == plan_->complete)
    status_.missionComplete = true;
  status_.transition = reason;
}

void FlightExecutive::produceCommand(const ExecInputs &in, float dtS) {
  const PhaseSpec &ph = phaseSpec();
  BehaviourOut out;
  behaviourStep(plan_->behaviours, in.est, dtS, behMem_, out);

  ControlCommand c;
  c.phase = ph.id;
  c.phaseSeq = phaseSeq_;
  c.missionTimeNs = missionTime(in.nowNs);
  c.tvc = ph.tvc;
  for (int i = 0; i < 6; i++)
    c.posSp[i] = out.posSp[i];
  for (int i = 0; i < 4; i++)
    c.attSp[i] = out.attSp[i];
  c.attSpSource = ph.tvc == TvcMode::Attitude ? AttSpSource::Command
                  : ph.tvc == TvcMode::Position ? AttSpSource::Mapper
                                                : AttSpSource::None;
  c.thrustN = ph.tvc == TvcMode::Attitude ? out.thrustN : 0.0f;
  c.thrustCapN = ph.thrustCapN;
  c.flaps = ph.flaps;
  c.flapAzimuthRad = out.flapAzimuthRad;
  c.flapPitchRad = out.flapPitchRad;
  c.tvcActuators = ph.tvcActuators;
  c.flapActuators = ph.flaps != FlapMode::Disabled;
  c.motors = ph.motors;
  c.estimatorZeroing = ph.estimatorZeroing;
  c.actuatorTestSeq = actuatorTestSeq_;
  c.bounds = ph.bounds;
  c.onGround = ph.onGround;

  sanitizeCommand(c, in.est, lastPos_);
  for (int i = 0; i < 3; i++)
    lastPos_[i] = c.posSp[3 + i];

  c.resetTvc = c.tvc != TvcMode::Off && (prevTvc_ == TvcMode::Off || prevCtrlGroup_ != ph.ctrlGroup);
  prevTvc_ = c.tvc;
  prevCtrlGroup_ = ph.ctrlGroup;
  cmd_ = c;
}

} // namespace Exec
} // namespace VCTR
