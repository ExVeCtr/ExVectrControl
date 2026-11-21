#ifndef EXVECTRCONTROL_EXECUTIVE_BEHAVIOURS_HPP
#define EXVECTRCONTROL_EXECUTIVE_BEHAVIOURS_HPP

/**
 * Setpoint behaviours: deterministic step functions over an explicit memory POD. A behaviour is initialised at
 * phase entry from the estimate and its parameters and stepped once per executive step with the estimate and dt.
 * It has no other inputs and no hidden state. The plan selects a behaviour by enum and a parameter index into the
 * typed table of that behaviour (no function pointers, so a plan can be serialised and validated).
 *
 * Adding a behaviour: a header behaviour_<name>.hpp (params, memory, init, step), a source file, an enum value
 * here, a table in BehaviourTables, the dispatch cases in behaviours.cpp, its parameter checks in
 * validate_plan.cpp, and a unit test (docs/project/flight-executive.md).
 */

#include <stdint.h>

#include "ExVectrControl/executive/behaviour_descent.hpp"
#include "ExVectrControl/executive/behaviour_goto.hpp"
#include "ExVectrControl/executive/behaviour_kick.hpp"
#include "ExVectrControl/executive/behaviour_land.hpp"
#include "ExVectrControl/executive/behaviour_out.hpp"
#include "ExVectrControl/executive/behaviour_transit.hpp"
#include "ExVectrControl/executive/exec_types.hpp"

namespace VCTR {
namespace Exec {

enum class Behaviour : uint8_t {
  Idle = 0,        ///< zero setpoint, no parameters
  HoldMeasured,    ///< posSp = [0,0,0, measured P] every step (velocity damper), no parameters
  GoTo,            ///< GoToParams
  Transit,         ///< TransitParams
  Land,            ///< LandParams
  KickAttitude,    ///< KickParams
  DescentGuidance, ///< DescentParams
};

/// The plan's typed parameter tables (pointer + count each; unused tables may be null with count 0).
struct BehaviourTables {
  const GoToParams *goTo = nullptr;
  uint8_t goToCount = 0;
  const TransitParams *transit = nullptr;
  uint8_t transitCount = 0;
  const LandParams *land = nullptr;
  uint8_t landCount = 0;
  const KickParams *kick = nullptr;
  uint8_t kickCount = 0;
  const DescentParams *descent = nullptr;
  uint8_t descentCount = 0;
};

/// Memory of the active behaviour (one sub-struct per behaviour; only the active one is meaningful).
struct BehaviourMem {
  Behaviour active = Behaviour::Idle;
  uint8_t paramIdx = 0;
  GoToMem goTo;
  TransitMem transit;
  LandMem land;
  KickMem kick;
  DescentMem descent;
};

/// Number of parameter records the plan has for behaviour b (Idle and HoldMeasured: 1, they take none).
uint8_t behaviourParamCount(Behaviour b, const BehaviourTables &t);

/// Initialises mem for behaviour b with parameter record idx. idx must be valid (validatePlan).
void behaviourInit(Behaviour b, uint8_t idx, const BehaviourTables &t, const EstimateSnapshot &est,
                   BehaviourMem &mem);

/// One step of the active behaviour (mem.active). out starts from its defaults.
void behaviourStep(const BehaviourTables &t, const EstimateSnapshot &est, float dtS, BehaviourMem &mem,
                   BehaviourOut &out);

/// The point the behaviour b flies to (GoTo target, Transit and Land home). False if it has none.
bool behaviourTarget(Behaviour b, uint8_t idx, const BehaviourTables &t, float out[3]);

} // namespace Exec
} // namespace VCTR

#endif
