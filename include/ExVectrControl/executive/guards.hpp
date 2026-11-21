#ifndef EXVECTRCONTROL_EXECUTIVE_GUARDS_HPP
#define EXVECTRCONTROL_EXECUTIVE_GUARDS_HPP

/**
 * Guards: the conditions of phase exits (and of phase entry, see PhaseSpec::entryGuard). A guard is enum-selected
 * with an index into the plan's GuardParams table. Guards that need time history (InsideFor, NotReachedWithin,
 * LandingDetected) keep it in a small GuardMem owned by the exit slot; the engine initialises it at phase entry.
 *
 * Guards see the state at the start of the step: the new estimate and the behaviour memory of the previous step
 * (the behaviour steps after the transitions, so a new phase's first command is computed in the same step).
 *
 * Adding a guard: an enum value, the fields it reads in GuardParams (documented below), its case in guards.cpp,
 * its checks in validate_plan.cpp, and a unit test.
 */

#include <stdint.h>

#include "ExVectrControl/executive/behaviours.hpp"
#include "ExVectrControl/executive/exec_types.hpp"

namespace VCTR {
namespace Exec {

enum class Guard : uint8_t {
  None = 0,         ///< never true (unused exit slot)
  MissionTimeAbove, ///< mission time > timeNs (strict, legacy countdown edges)
  InsideFor,        ///< |target - P| < distanceM continuously for > timeNs (timer restarts outside; legacy loiter)
  Reached,          ///< |target - P| < distanceM
  NotReachedWithin, ///< outside distanceM now, never inside since entry, and phase time > timeNs (legacy WP limit)
  SpeedBelow,       ///< |V| < speedMps
  StopAltitude,     ///< Pz < descentStopDistanceM(Vz, thrustN, massKg) + altitudeM (legacy freefall end)
  KickAngleBelow,   ///< kickRemainingAngle(q, kick target) < angleRad (finite only); needs KickAttitude
  PitchRateAbove,   ///< |body pitch rate wy| > rateRadPs (finite only)
  LandingDetected,  ///< landing: tracking error or ground window (below); needs Land
  AltitudeAtLeast,  ///< position valid and Pz >= altitudeM
};

/**
 * Guard parameters. One record type for all guards; each guard reads only the fields listed in its comment above
 * (and LandingDetected below); the others stay 0. All values SI.
 *
 * LandingDetected = (Pz - Land Z setpoint >= distanceM continuously for > timeNs, timer restarts below)
 *                   OR (ground estimate fresh and groundMinM < groundDist < groundMaxM and groundCov < groundCovMax).
 */
struct GuardParams {
  float distanceM = 0;
  float speedMps = 0;
  float angleRad = 0;
  float rateRadPs = 0;
  float altitudeM = 0;
  int64_t timeNs = 0;
  float massKg = 0;
  float thrustN = 0;
  float groundMinM = 0;
  float groundMaxM = 0;
  float groundCovMax = 0;

  // ---- constexpr factories, one per guard, setting exactly the fields that guard reads ----

  static constexpr GuardParams missionTimeAbove(int64_t ns) {
    GuardParams p{};
    p.timeNs = ns;
    return p;
  }
  /// InsideFor, Reached (timeNs unused) and NotReachedWithin.
  static constexpr GuardParams distance(float m, int64_t ns = 0) {
    GuardParams p{};
    p.distanceM = m;
    p.timeNs = ns;
    return p;
  }
  static constexpr GuardParams speedBelow(float mps) {
    GuardParams p{};
    p.speedMps = mps;
    return p;
  }
  static constexpr GuardParams stopAltitude(float massKg_, float thrustN_, float marginM) {
    GuardParams p{};
    p.massKg = massKg_;
    p.thrustN = thrustN_;
    p.altitudeM = marginM;
    return p;
  }
  static constexpr GuardParams angleBelow(float rad) {
    GuardParams p{};
    p.angleRad = rad;
    return p;
  }
  static constexpr GuardParams rateAbove(float radPs) {
    GuardParams p{};
    p.rateRadPs = radPs;
    return p;
  }
  static constexpr GuardParams landing(float trackingM, int64_t trackingNs, float groundMin, float groundMax,
                                       float groundCovMax_) {
    GuardParams p{};
    p.distanceM = trackingM;
    p.timeNs = trackingNs;
    p.groundMinM = groundMin;
    p.groundMaxM = groundMax;
    p.groundCovMax = groundCovMax_;
    return p;
  }
  static constexpr GuardParams altitudeAtLeast(float m) {
    GuardParams p{};
    p.altitudeM = m;
    return p;
  }
};

/// Time history of one guard instance.
struct GuardMem {
  int64_t sinceNs = 0; ///< InsideFor: last time outside; LandingDetected: last time below the tracking threshold
  bool flag = false;   ///< NotReachedWithin: was inside since entry
};

/// What a guard sees in one step.
struct GuardContext {
  const EstimateSnapshot *est = nullptr;
  const BehaviourMem *mem = nullptr;
  int64_t nowNs = 0;
  int64_t elapsedNs = 0;     ///< time since phase entry
  int64_t missionTimeNs = 0;
  bool hasTarget = false;    ///< the phase's behaviour has a target point (behaviourTarget)
  float target[3] = {0, 0, 0};
};

/// Initialises the guard memory at phase entry.
void guardInit(Guard g, int64_t nowNs, GuardMem &mem);

/// Evaluates guard g and updates its memory.
bool guardEval(Guard g, const GuardParams &p, const GuardContext &ctx, GuardMem &mem);

/// True if guard g measures against the behaviour's target point.
bool guardNeedsTarget(Guard g);

} // namespace Exec
} // namespace VCTR

#endif
