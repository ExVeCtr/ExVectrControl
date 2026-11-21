#ifndef EXVECTRCONTROL_EXECUTIVE_BEHAVIOUR_DESCENT_HPP
#define EXVECTRCONTROL_EXECUTIVE_BEHAVIOUR_DESCENT_HPP

/**
 * Descent flap guidance: azimuth and pitch for the flap attitude controller during the unpowered belly-down fall.
 * Reproduces the legacy FlapModeObserverTask Stabilize branch (Starship src/main.cpp) with its numbers as
 * parameters:
 * - above highAltitudeM: azimuth = -atan2(Py, -Px) (back towards the origin),
 *   pitch = clamp(-(dXY - offsetDistM) * slope, minOffset, 0) + bias;
 * - below stop altitude + lowExtraMarginM: pitch = lowPitch, azimuth kept;
 * - otherwise: pitch 0, azimuth kept.
 * The stop altitude is descentStopDistanceM() + stopMarginM (legacy freefall formula, sign-blind, known issue).
 */

#include "ExVectrControl/executive/behaviour_out.hpp"
#include "ExVectrControl/executive/exec_types.hpp"

namespace VCTR {
namespace Exec {

/**
 * Distance needed to stop a vertical speed vz with the thrust T of a vehicle of mass m:
 * 0.5 vz^2 / (T/m - 9.81). Legacy freefall formula: it ignores the sign of vz and the flip-up time, and is not
 * finite or negative if T/m <= 9.81 (the plan validator refuses such parameters).
 */
float descentStopDistanceM(float vz, float thrustN, float massKg);

struct DescentParams {
  float massKg = 0;
  float thrustN = 0;
  float stopMarginM = 0;        ///< stop altitude = stop distance + this
  float highAltitudeM = 0;      ///< homing guidance above this altitude
  float offsetDistM = 0;        ///< horizontal distance without pitch offset
  float pitchSlopeRadPerM = 0;  ///< pitch offset per metre beyond offsetDistM
  float minPitchOffsetRad = 0;  ///< lower clamp of the offset (<= 0)
  float pitchBiasRad = 0;       ///< added to the offset
  float lowExtraMarginM = 0;    ///< low-altitude pitch below stop altitude + this
  float lowPitchRad = 0;
};

struct DescentMem {
  float azimuthRad = 0; ///< kept between steps (legacy: kept since boot, starting at 0)
};

void descentInit(const DescentParams &p, const EstimateSnapshot &est, DescentMem &m);
void descentStep(const DescentParams &p, const EstimateSnapshot &est, DescentMem &m, BehaviourOut &out);

} // namespace Exec
} // namespace VCTR

#endif
