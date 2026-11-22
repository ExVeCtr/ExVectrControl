// Descent wiring of ControlAttitudeFlaps: runs the DescentGuidance every flap-task tick and turns the
// Descent mode setting (no heading, no split) into the setting FlapModeMapper expects (heading setpoint and split).
//
// Pure, deterministic, header-only, no scheduler, topics or clock (host-tested, test/test_descent_wiring in the flight
// software). The caller passes the position/attitude estimate, whether the position estimate is fresh, the target and
// the time step. Behaviour:
//   * Entry into Descent (first tick in that mode): DescentGuidance::init (integrator zero, heading setpoint = the
//     current nose heading, entry hold). Leaving Descent (any other mode, including Stabilize, Upright, Acsent) resets
//     the guidance memory at once, so the next entry starts fresh.
//   * Invalid inputs (target.valid false, stale or non-finite position, non-finite attitude or dt): the guidance holds
//     the heading, the split slews to 0 (neutral 30/78, a vertical fall) and the velocity integral is frozen, per the
//     failure table of docs/descent-control-design.md (position invalid: heading held, glide 0, vertical fall).
// Other modes pass through unchanged.
#ifndef EXVECTRCONTROL_STARSHIP_DESCENT_FLAP_CONTROLLER_HPP
#define EXVECTRCONTROL_STARSHIP_DESCENT_FLAP_CONTROLLER_HPP

#include "ExVectrControl/starship/descent_guidance.hpp"
#include "ExVectrControl/starship/flap_types.hpp"

namespace VCTR {
namespace CTRL {
namespace descent {

/// What the telemetry shows of the descent guidance (read by the telemetry sampler, same scheduler thread).
struct DescentStatus {
  bool active = false;       ///< the flap setting mode is Descent and the guidance ran in the last tick
  bool inputsValid = false;  ///< the last step had a valid target and a fresh, finite position
  float headingSp_rad = 0;   ///< nose azimuth setpoint (CCW from North)
  float pitchSplit_rad = 0;  ///< pitch split sent to the flap law
  float glideCmd_mps = 0;    ///< air speed command along the nose
};

class DescentFlapController {
 public:
  DescentGuidanceParams params;

  /// @param s       the setting published by FlapModeObserverTask
  /// @param target  the DescentTarget from the setting topic
  /// @param pos     [V(3), P(3)] NWU
  /// @param posFresh false when the position estimate is stale (checked by the caller against the clock)
  /// @param att     [w(3), q(4)] body -> world
  /// @param dt      time step of this tick, s
  /// @return the setting for FlapModeMapper::map (Descent: heading and split filled in)
  ControlAttitudeBellyFlopSetting apply(const ControlAttitudeBellyFlopSetting &s, const DescentTarget &target,
                                        const float pos[6], bool posFresh, const float att[7], float dt) {
    using Mode = ControlAttitudeBellyFlopSetting::BellyFlopMode;
    if (s.bellyFlopMode != Mode::BellyFlopMode_Descent) {
      reset();
      return s;
    }
    if (!active_) {
      DescentGuidance::init(params, att, mem_);
      active_ = true;
    }
    DescentTarget t = target;
    if (!posFresh)
      t.valid = false;
    const bool ok = t.valid && allFinite(pos, 6) && allFinite(att, 7);
    const DescentGuidanceOut o = DescentGuidance::step(params, t, pos, att, dt, mem_);
    status_.active = true;
    status_.inputsValid = ok;
    status_.headingSp_rad = o.headingSp_rad;
    status_.pitchSplit_rad = o.pitchSplit_rad;
    status_.glideCmd_mps = o.glideCmd_mps;
    ControlAttitudeBellyFlopSetting r = s;
    r.azimuthAngle_Rad = o.headingSp_rad;
    r.pitchSplit_Rad = o.pitchSplit_rad;
    return r;
  }

  /// Leave Descent: guidance memory and status back to the initial state.
  void reset() {
    active_ = false;
    mem_ = DescentGuidanceMem();
    status_ = DescentStatus();
  }

  const DescentStatus &status() const { return status_; }
  const DescentGuidanceMem &memory() const { return mem_; }

 private:
  DescentGuidanceMem mem_;
  DescentStatus status_;
  bool active_ = false;
};

}  // namespace descent
}  // namespace CTRL
}  // namespace VCTR

#endif
