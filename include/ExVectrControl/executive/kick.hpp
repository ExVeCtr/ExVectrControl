#ifndef EXVECTRCONTROL_EXECUTIVE_KICK_HPP
#define EXVECTRCONTROL_EXECUTIVE_KICK_HPP

/**
 * Powered pitch-over ("kick") geometry and end trigger. Moved unchanged from the Starship flight software
 * (src/safety/bellyflop_kick.*, 2026-11-21); the vehicle constants stay with the vehicle (Starship:
 * src/executive/starship_constants.hpp), the old header re-exports these functions for the legacy mission code.
 *
 * Quaternions are float[4] = [w, x, y, z], rotating body -> world (NWU). The kick pitches about body +Y so that
 * body +X points up and the "belly" (body -X) down.
 */

#include <stdint.h>

namespace VCTR {
namespace Exec {

enum class KickEnd : uint8_t { None = 0, Angle, Rate, Time };

/// Limits of kickEndTrigger().
struct KickEndLimits {
  int64_t timeLimitNs;        ///< hard limit: elapsed > this ends the kick
  float endRemainingAngleRad; ///< normal end: remaining angle below this
  float endPitchRateRadPs;    ///< spin guard: |body pitch rate| above this
};

/**
 * End trigger of the kick. Time first (it always ends the kick), then angle, then rate. Non-finite angle or rate
 * never trigger (the time limit still does).
 */
KickEnd kickEndTrigger(int64_t elapsedNs, float remainingAngleRad, float pitchRateRadPs, const KickEndLimits &limits);

/// Rotates v (body) into the world frame with q (body -> world).
void kickRotate(const float q[4], const float v[3], float out[3]);

/**
 * Pitch-over target from the attitude at kick start q0: q0's heading (world-Z rotation that keeps the belly
 * direction) followed by pitchRad about body +Y towards belly down. For q0 = identity and 90 deg this is exactly
 * Quat(y, -90 deg). If the belly direction is nearly vertical (already lying flat) the heading 0 is used.
 * Output unit, w >= 0. A non-finite q0 gives heading 0 (finite output).
 */
void kickTargetAttitude(const float q0[4], float pitchRad, float out[4]);

/// Remaining angle of the pitch-over: angle between body +X of q and body +X of qTarget (rad, [0, pi]).
/// NaN if either quaternion is not finite.
float kickRemainingAngle(const float q[4], const float qTarget[4]);

} // namespace Exec
} // namespace VCTR

#endif
