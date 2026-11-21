#ifndef EXVECTRCONTROL_EXECUTIVE_BEHAVIOUR_KICK_HPP
#define EXVECTRCONTROL_EXECUTIVE_BEHAVIOUR_KICK_HPP

/**
 * Kick attitude: the attitude target is computed once at entry from the measured attitude (kickTargetAttitude, a
 * pure pitch about body +Y keeping the heading) and held; the thrust request is constant. The end of the kick is
 * decided by the phase (timeout and the KickAngleBelow / PitchRateAbove guards).
 */

#include "ExVectrControl/executive/behaviour_out.hpp"
#include "ExVectrControl/executive/exec_types.hpp"

namespace VCTR {
namespace Exec {

struct KickParams {
  float pitchRad = 0; ///< pitch-over angle about body +Y from the attitude at entry
  float thrustN = 0;  ///< thrust request during the kick
};

struct KickMem {
  float target[4] = {1, 0, 0, 0}; ///< attitude target, body -> world
  float holdPos[3] = {0, 0, 0};   ///< measured position at entry (posSp for telemetry only)
};

void kickInit(const KickParams &p, const EstimateSnapshot &est, KickMem &m);
void kickStep(const KickParams &p, const KickMem &m, BehaviourOut &out);

} // namespace Exec
} // namespace VCTR

#endif
