#ifndef EXVECTRCONTROL_EXECUTIVE_BEHAVIOUR_TRANSIT_HPP
#define EXVECTRCONTROL_EXECUTIVE_BEHAVIOUR_TRANSIT_HPP

/**
 * Transit: straight 3-D line from the measured position at entry to a home point. Reproduces the legacy RTH
 * Translation state (Starship src/mission/mission_rth.hpp) including its known issues (design X1): no climb before
 * the transit, braking cap sqrt(d * accel) applied after the position step, and the unit vector computed without a
 * zero guard (non-finite if the setpoint reaches home exactly; the engine's output sanitiser catches it).
 */

#include "ExVectrControl/executive/behaviour_out.hpp"
#include "ExVectrControl/executive/exec_types.hpp"

namespace VCTR {
namespace Exec {

struct TransitParams {
  float home[3] = {0, 0, 0}; ///< m, world
  float maxSpeedMps = 0;
  float accelMps2 = 0; ///< ramp-up rate and braking cap
};

struct TransitMem {
  float spPos[3] = {0, 0, 0};
  float speedMps = 0;
};

void transitInit(const TransitParams &p, const EstimateSnapshot &est, TransitMem &m);
void transitStep(const TransitParams &p, const EstimateSnapshot &est, float dtS, TransitMem &m, BehaviourOut &out);

} // namespace Exec
} // namespace VCTR

#endif
