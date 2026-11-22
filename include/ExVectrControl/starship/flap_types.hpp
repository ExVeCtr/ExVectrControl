#ifndef EXVECTRCONTROL_STARSHIP_FLAP_TYPES_HPP
#define EXVECTRCONTROL_STARSHIP_FLAP_TYPES_HPP

#include <stdint.h>

namespace VCTR {
namespace CTRL {

struct ControlAttitudeFlapSetting {
  float flapTLAngle_Rad = 0;  // Flap angle in radians.
  float flapTRAngle_Rad = 0;  // Flap angle in radians.
  float flapBLAngle_Rad = 0;  // Flap angle in radians.
  float flapBRAngle_Rad = 0;  // Flap angle in radians.

  bool enableFlaps = false;  // Enable or disable the flap actuators
};

struct ControlAttitudeBellyFlopSetting {
  float azimuthAngle_Rad =
      0;  // In what respect to north the vehicle should point to.
  float pitchAngle_Rad =
      0;  // What angle to pitch the vehicle to in respect to the Body Y Axis.
  float rollAngle_Rad =
      0;  // What angle to roll the vehicle to in respect to the Body X Axis.

  bool extentFlapsOnAscent =
      false;  // If true, the flaps are fully extended in ascent mode. If false,
              // the flaps are fully retracted.

  float pitchSplit_Rad =
      0;  // Descent mode only: top/bottom pitch split (+ = nose-up / slower
          // glide, - = forward glide). azimuthAngle_Rad is the heading
          // setpoint in that mode. Not used by the other modes.

  enum class BellyFlopMode : uint8_t {
    BellyFlopMode_Acsent,     // Flaps are disabled and depending on the bool,
                              // either fully restracted or fully extended.
    BellyFlopMode_Stabilize,  // Stabilize the vehicle in belly flop mode.
                              // (Attempts to reach the given angles)
    BellyFlopMode_Upright,    // Help to upright the vehicle. (Top flaps are
                              // extended, bottom flaps are retracted)
    BellyFlopMode_Descent,    // Descent law (FlapDescentLaw): neutral 30/78 deg,
                              // heading setpoint azimuthAngle_Rad, glide set by
                              // pitchSplit_Rad. Appended: earlier values are
                              // unchanged.
  };

  BellyFlopMode bellyFlopMode =
      BellyFlopMode::BellyFlopMode_Acsent;  // The current belly flop mode.
};

}  // namespace CTRL
}  // namespace VCTR

#endif
