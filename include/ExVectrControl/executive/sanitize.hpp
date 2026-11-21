#ifndef EXVECTRCONTROL_EXECUTIVE_SANITIZE_HPP
#define EXVECTRCONTROL_EXECUTIVE_SANITIZE_HPP

/**
 * Output sanitiser: the last stage of every executive step. Every float of the command must be finite.
 * - A non-finite position setpoint becomes "hold": velocity 0, position = the measured position if finite, else
 *   lastPos (the last finite setpoint position).
 * - A non-finite or zero attitude target in TvcMode::Attitude switches the TVC controllers off for that step
 *   (thrust 0, identity target, no attitude-setpoint source); in other modes the target becomes the identity.
 * - A non-finite thrust request becomes 0, a non-finite or non-positive thrust cap becomes 0 (no thrust), and
 *   non-finite flap guidance becomes 0.
 * The vehicle keeps flying; ControlCommand::sanitized shows the event in telemetry. The host scenarios assert that
 * it never fires in a normal flight.
 */

#include "ExVectrControl/executive/exec_types.hpp"

namespace VCTR {
namespace Exec {

/// Sanitises c in place and sets c.sanitized. Returns c.sanitized.
bool sanitizeCommand(ControlCommand &c, const EstimateSnapshot &est, const float lastPos[3]);

} // namespace Exec
} // namespace VCTR

#endif
