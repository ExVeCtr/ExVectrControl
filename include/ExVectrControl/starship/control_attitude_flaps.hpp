#ifndef EXVECTRCONTROL_CONTROLATTITUDEFLAPS_HPP
#define EXVECTRCONTROL_CONTROLATTITUDEFLAPS_HPP

#include "ExVectrControl/starship/descent_flap_controller.hpp"
#include "ExVectrControl/starship/flap_mode_mapper.hpp"
#include "ExVectrControl/starship/flap_types.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrDSP/value_covariance.hpp"
#include "ExVectrMath.hpp"

namespace VCTR {
namespace CTRL {

/**
 * @brief Simple one dimensional PID controller.
 */
class ControlAttitudeFlaps : public Core::Task_Periodic {
 private:
  // Setpoint for the position. In form: [V, P], where V is the linear velocity
  // vector and P is the position vector.
  // Core::Simple_Subscriber<Math::Vector<float, 7>> stateSetpointSubr_;
  // Math::Vector<float, 7> stateSetpoint_; // Setpoint for the position. In
  // form: [V, P], where V is the linear velocity vector and P is the position
  // vector.
  // Setpoint for the acceleration in reference frame.

  // Current attitude estimation. In form: [W, Q], where W is the angular
  // velocity vector and Q is a unit quaternion rotation from the reference
  // frame to body frame.
  Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 7>>>
      attMeasSubr_;
  Core::Timestamped<Math::Vector<float, 7>> attitudeEstimation_;

  // Current position estimation. In form: [V, P] in reference frame, where V is
  // the linear velocity vector and P is the position vector.
  Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 6>>>
      posMeasSubr_;
  Core::Timestamped<Math::Vector<float, 6>> positionEstimation_;

  Core::Simple_Subscriber<ControlAttitudeBellyFlopSetting> controlSetting_;

  // Output topic for the flap settings.
  Core::Topic_Publisher<ControlAttitudeFlapSetting> flapOutputTopic_;

  // Control parameters: legacy Stabilize law (neutral 50/40 deg) and the Descent law (neutral 30/78 deg).
  FlapStabilizeParams stabilizeParams_;
  descent::FlapDescentLawParams descentParams_;
  descent::FlapDescentLawMem descentMem_;

  // Descent guidance (runs in this 50 Hz task): target from a small setting topic, state in descentCtrl_.
  Core::Simple_Subscriber<descent::DescentTarget> descentTargetSubr_;
  descent::DescentTarget descentTarget_;
  descent::DescentFlapController descentCtrl_;
  int64_t lastRunNs_ = 0;

 public:
  /**
   * @brief Constructor for the ControlRocket class.
   */
  ControlAttitudeFlaps();

  /// @brief The P-Term gain for the attitude rate controller.
  void setAttitudeRateXGain(float gain) { stabilizeParams_.attitudeRateXGain = gain; }
  /// @brief The P-Term gain for the attitude rate controller.
  void setAttitudeRateYGain(float gain) { stabilizeParams_.attitudeRateYGain = gain; }
  /// @brief The P-Term gain for the attitude rate controller.
  void setAttitudeRateZGain(float gain) { stabilizeParams_.attitudeRateZGain = gain; }

  /**
   * * @brief Subsribes to a topic to which the attitude estimation is published
   * in form: [W, Q], where W is the angular velocity vector and Q is a unit
   * quaternion rotation from the reference frame to body frame.
   */
  void subscribeAttitudeMeasurement(
      Core::Topic<Core::Timestamped<Math::Vector<float, 7>>>& attTopic);

  /**
   * @brief Subsribes to a topic to which the position estimation is published
   * in form: [V, P], where V is the linear velocity vector and P is the
   * position vector.
   */
  void subscribePositionMeasurement(
      Core::Topic<Core::Timestamped<Math::Vector<float, 6>>>& posTopic);

  /**
   * @brief Subsribes to a topic to which the setpoint is published in form: [V,
   * P], where V is the linear velocity vector and P is the position vector.
   */
  // void subscribeAttitudeSetpoint(Core::Topic<Math::Vector<float, 7>>
  // &setpointTopic);

  void subscribeControlSetting(
      Core::Topic<ControlAttitudeBellyFlopSetting>& controlSettingTopic) {
    controlSetting_.subscribe(controlSettingTopic);
  }

  /// Topic of the DescentTarget (published by the mission side at 10 Hz). Without it the target stays invalid (zero
  /// glide, straight fall).
  void subscribeDescentTarget(Core::Topic<descent::DescentTarget>& targetTopic) {
    descentTargetSubr_.subscribe(targetTopic);
  }

  /// Guidance output for telemetry (heading setpoint, split); inactive outside the Descent mode.
  const descent::DescentStatus& getDescentStatus() const { return descentCtrl_.status(); }

  /**
   * @brief Uses the given topic to publish the flap settings for this
   * controllers output.
   *
   */
  void subscribeFlapSettingOutputTopic(
      Core::Topic<ControlAttitudeFlapSetting>& flapOutputTopic);

 private:
  void taskCheck() override;

  void taskInit() override;

  void taskThread() override;
};

}  // namespace CTRL
}  // namespace VCTR

#endif
