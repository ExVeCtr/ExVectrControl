#ifndef EXVECTRCONTROL_EXECUTIVE_BEHAVIOUR_LAND_HPP
#define EXVECTRCONTROL_EXECUTIVE_BEHAVIOUR_LAND_HPP

/**
 * Land: XY setpoint at home, Z setpoint starts at the measured altitude and ramps down at descentRate while the
 * vehicle is less than holdAboveM above it (legacy RTH Descent, Starship src/mission/mission_rth.hpp). The touchdown
 * decision is the LandingDetected guard.
 */

#include "ExVectrControl/executive/behaviour_out.hpp"
#include "ExVectrControl/executive/exec_types.hpp"

namespace VCTR {
namespace Exec {

struct LandParams {
  float home[3] = {0, 0, 0}; ///< m, world (only X and Y are used)
  float descentRateMps = 0;  ///< Z setpoint ramp, > 0
  float holdAboveM = 0;      ///< the ramp pauses while the vehicle is this far or more above the Z setpoint
};

struct LandMem {
  float spZ = 0; ///< Z setpoint
};

void landInit(const LandParams &p, const EstimateSnapshot &est, LandMem &m);
void landStep(const LandParams &p, const EstimateSnapshot &est, float dtS, LandMem &m, BehaviourOut &out);

} // namespace Exec
} // namespace VCTR

#endif
