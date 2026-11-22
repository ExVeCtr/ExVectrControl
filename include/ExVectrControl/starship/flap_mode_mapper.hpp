// Pure mode mapping of ControlAttitudeFlaps: setting + attitude estimate -> four flap angles.
//
// Header-only, no scheduler, topics or clock (host-tested). The Stabilize, Upright and Acsent branches are the
// original ControlAttitudeFlaps code moved here unchanged (same expressions, same operation order), so their outputs
// are bit-identical (test/test_descent_unit in the flight software). The Descent branch runs FlapDescentLaw.
#ifndef EXVECTRCONTROL_STARSHIP_FLAP_MODE_MAPPER_HPP
#define EXVECTRCONTROL_STARSHIP_FLAP_MODE_MAPPER_HPP

#include "ExVectrControl/starship/flap_descent_law.hpp"
#include "ExVectrControl/starship/flap_types.hpp"
#include "ExVectrMath.hpp"

namespace VCTR {
namespace CTRL {

/// Parameters of the legacy Stabilize law (defaults = the previous class members).
struct FlapStabilizeParams {
  float flapTopNeutralAngle_Rad = 50 * DEGREES;     // Neutral angle for the top flaps in radians.
  float flapBottomNeutralAngle_Rad = 40 * DEGREES;  // Neutral angle for the bottom flaps in radians.
  float enableThresAngle_Rad = 35 * DEGREES;        // Belly-down angle where the flap factor starts to fade.
  float attitudeAzimuGain = 0.3;                    // Attitude in azimuth control gain.
  float attitudePitchGain = 1.2;                    // Attitude in pitch control gain.
  float attitudeRateXGain = 0.2;                    // Attitude rate control gain.
  float attitudeRateYGain = 0.4;                    // Attitude rate control gain.
  float attitudeRateZGain = 0.1;                    // Attitude rate control gain.
  float attitudeAzimuLimit_Perc = 0.5;              // Limit for the azimuth/yaw output.
};

class FlapModeMapper {
 public:
  /**
   * @param attitudeEstimation [w(3), q(4)], q = [w,x,y,z] body -> world.
   * @param descentMem state of the descent law (reset here when the mode is not Descent).
   */
  static ControlAttitudeFlapSetting map(const ControlAttitudeBellyFlopSetting &controlSettings,
                                        const Math::Vector<float, 7> &attitudeEstimation,
                                        const FlapStabilizeParams &stab,
                                        const descent::FlapDescentLawParams &descentParams,
                                        descent::FlapDescentLawMem &descentMem) {
    Math::Quat_F attitude = attitudeEstimation.block<4, 1>(3, 0);
    auto angularVelocity = attitudeEstimation.block<3, 1>(0, 0);

    auto bodyXAxis = attitude.rotate(Math::Vector_F({1, 0, 0}));  // Body X-axis in reference frame
    auto bodyZAxis = attitude.rotate(Math::Vector_F({0, 0, 1}));  // Body Z-axis in reference frame
    auto bellyDownAngle = bodyXAxis.getAngleTo(Math::Vector_F({0, 0, 1}));
    // Scale the flap factor so if over threshold, then begin reducing factor until 20 deg over the threshold, then
    // disable flaps by setting factor 0
    float flapFactor = (bellyDownAngle - stab.enableThresAngle_Rad) / (20 * DEGREES);
    if (flapFactor < 0)
      flapFactor = 0;
    if (flapFactor > 1)
      flapFactor = 1;
    flapFactor = 1 - flapFactor;

    ControlAttitudeFlapSetting flapSetting;
    using Mode = ControlAttitudeBellyFlopSetting::BellyFlopMode;
    if (controlSettings.bellyFlopMode != Mode::BellyFlopMode_Descent)
      descentMem = descent::FlapDescentLawMem();  // the turn latch starts fresh at every Descent entry

    if (controlSettings.bellyFlopMode == Mode::BellyFlopMode_Stabilize) {
      // Calculate the azimuth (with atan) and pitch when the vehicle is pitched down.
      auto azimuthIs = atan2f(bodyZAxis(1), bodyZAxis(0));
      auto pitchIs = atan2f(bodyZAxis(2), sqrtf(bodyZAxis(0) * bodyZAxis(0) + bodyZAxis(1) * bodyZAxis(1)));

      Math::Vector<float, 3> attRateCtrlOutput = 0;

      float azimuthOutput = controlSettings.azimuthAngle_Rad - azimuthIs;
      float pitchOutput = controlSettings.pitchAngle_Rad - pitchIs;

      if (azimuthOutput > M_PI) {
        azimuthOutput -= 2 * M_PI;
      } else if (azimuthOutput < -M_PI) {
        azimuthOutput += 2 * M_PI;
      }

      azimuthOutput = azimuthOutput * stab.attitudeAzimuGain;
      pitchOutput = pitchOutput * stab.attitudePitchGain;

      if (azimuthOutput > stab.attitudeAzimuLimit_Perc) {
        azimuthOutput = stab.attitudeAzimuLimit_Perc;
      } else if (azimuthOutput < -stab.attitudeAzimuLimit_Perc) {
        azimuthOutput = -stab.attitudeAzimuLimit_Perc;
      }

      // Azimuth and pitch control
      attRateCtrlOutput = attRateCtrlOutput + Math::Vector<float, 3>({azimuthOutput, pitchOutput, 0});

      // Ang Vel influence.
      attRateCtrlOutput = attRateCtrlOutput + Math::Vector<float, 3>{
                                                  -(angularVelocity(0)) * stab.attitudeRateXGain,
                                                  -(angularVelocity(1)) * stab.attitudeRateYGain,
                                                  -(angularVelocity(2)) * stab.attitudeRateZGain,
                                              };

      // Output to flap mapping
      flapSetting.flapTLAngle_Rad =
          stab.flapTopNeutralAngle_Rad +
          (-attRateCtrlOutput(0) - attRateCtrlOutput(1) + attRateCtrlOutput(2)) * flapFactor;
      flapSetting.flapTRAngle_Rad =
          stab.flapTopNeutralAngle_Rad +
          (+attRateCtrlOutput(0) - attRateCtrlOutput(1) - attRateCtrlOutput(2)) * flapFactor;
      flapSetting.flapBLAngle_Rad =
          stab.flapBottomNeutralAngle_Rad +
          (+attRateCtrlOutput(0) + attRateCtrlOutput(1) + attRateCtrlOutput(2)) * flapFactor;
      flapSetting.flapBRAngle_Rad =
          stab.flapBottomNeutralAngle_Rad +
          (-attRateCtrlOutput(0) + attRateCtrlOutput(1) - attRateCtrlOutput(2)) * flapFactor;
    } else if (controlSettings.bellyFlopMode == Mode::BellyFlopMode_Descent) {
      float att[7];
      for (int i = 0; i < 7; i++)
        att[i] = attitudeEstimation(i);
      descent::FlapDescentCmd cmd;
      cmd.headingSp_rad = controlSettings.azimuthAngle_Rad;
      cmd.pitchSplit_rad = controlSettings.pitchSplit_Rad;
      const descent::FlapDescentLawOut o = descent::FlapDescentLaw::step(descentParams, att, cmd, descentMem);
      flapSetting.flapTLAngle_Rad = o.flap[0];
      flapSetting.flapTRAngle_Rad = o.flap[1];
      flapSetting.flapBLAngle_Rad = o.flap[2];
      flapSetting.flapBRAngle_Rad = o.flap[3];
    } else if (controlSettings.bellyFlopMode == Mode::BellyFlopMode_Upright) {
      flapSetting.flapTLAngle_Rad = 0;
      flapSetting.flapTRAngle_Rad = 0;
      flapSetting.flapBLAngle_Rad = 90 * DEGREES;
      flapSetting.flapBRAngle_Rad = 90 * DEGREES;
    } else {
      flapSetting.flapTLAngle_Rad = 90 * DEGREES * !controlSettings.extentFlapsOnAscent;
      flapSetting.flapTRAngle_Rad = 90 * DEGREES * !controlSettings.extentFlapsOnAscent;
      flapSetting.flapBLAngle_Rad = 90 * DEGREES * !controlSettings.extentFlapsOnAscent;
      flapSetting.flapBRAngle_Rad = 90 * DEGREES * !controlSettings.extentFlapsOnAscent;
    }

    flapSetting.enableFlaps = true;
    return flapSetting;
  }
};

}  // namespace CTRL
}  // namespace VCTR

#endif
