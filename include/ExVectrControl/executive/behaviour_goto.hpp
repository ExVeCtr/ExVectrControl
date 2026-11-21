#ifndef EXVECTRCONTROL_EXECUTIVE_BEHAVIOUR_GOTO_HPP
#define EXVECTRCONTROL_EXECUTIVE_BEHAVIOUR_GOTO_HPP

/**
 * Go-to profile: moves a position setpoint towards one target along a speed profile. Reproduces the legacy
 * waypoint profile (Starship src/mission/mission_waypoint.hpp, missionHover, steps 4 and 5) bit for bit, known
 * issues included (design X1, fixes in E7):
 * - the braking cap sqrt(d * stopAccel), d measured from the MEASURED position, is a deceleration of stopAccel/2;
 * - the setpoint is integrated (memory), so this is feedback on the estimate, not a function of time.
 * Reaching, loitering and time limits of the old waypoint list are phase exits (guards), not part of this behaviour.
 */

#include "ExVectrControl/executive/behaviour_out.hpp"
#include "ExVectrControl/executive/exec_types.hpp"

namespace VCTR {
namespace Exec {

/// Where the setpoint starts when the behaviour is (re)initialised.
enum class GoToEntry : uint8_t {
  FromMeasured = 0, ///< at the measured position
  JumpTo,           ///< at entryPos (the old "snap to the previous waypoint" / instant-jump waypoint)
};

struct GoToParams {
  float target[3] = {0, 0, 0}; ///< m, world
  float speedMps = 0;          ///< cruise speed; < 0.0001 = instant jump to the target at entry
  float startAccelMps2 = 0;    ///< speed ramp-up; < 0.001 = jump to cruise speed
  float stopAccelMps2 = 0;     ///< braking cap sqrt(d * stopAccel); <= 0.001 = no braking cap
  GoToEntry entry = GoToEntry::FromMeasured;
  float entryPos[3] = {0, 0, 0}; ///< m, used with GoToEntry::JumpTo
};

struct GoToMem {
  float spPos[3] = {0, 0, 0}; ///< moving setpoint position
  float speedMps = 0;         ///< current setpoint speed
};

void goToInit(const GoToParams &p, const EstimateSnapshot &est, GoToMem &m);

/// One profile step of dtS seconds. Writes posSp.
void goToStep(const GoToParams &p, const EstimateSnapshot &est, float dtS, GoToMem &m, BehaviourOut &out);

} // namespace Exec
} // namespace VCTR

#endif
