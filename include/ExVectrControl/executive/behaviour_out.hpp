#ifndef EXVECTRCONTROL_EXECUTIVE_BEHAVIOUR_OUT_HPP
#define EXVECTRCONTROL_EXECUTIVE_BEHAVIOUR_OUT_HPP

/// Output of one behaviour step (the setpoint part of the command; the phase row supplies the rest).

namespace VCTR {
namespace Exec {

struct BehaviourOut {
  float posSp[6] = {0, 0, 0, 0, 0, 0}; ///< [V,P] setpoint
  float attSp[4] = {1, 0, 0, 0};       ///< attitude target, body -> world
  float thrustN = 0;                   ///< thrust request (Attitude mode)
  float flapAzimuthRad = 0;            ///< flap guidance (Descent flap mode)
  float flapPitchRad = 0;
};

/// posSp = [0,0,0, p]: hold a point with zero velocity.
inline void setHoldPoint(const float p[3], BehaviourOut &out) {
  out.posSp[0] = out.posSp[1] = out.posSp[2] = 0.0f;
  out.posSp[3] = p[0];
  out.posSp[4] = p[1];
  out.posSp[5] = p[2];
}

} // namespace Exec
} // namespace VCTR

#endif
