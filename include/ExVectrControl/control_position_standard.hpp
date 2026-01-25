#ifndef EXVECTRCONTROL_CONTROLPOSITIONSTANDARD_HPP
#define EXVECTRCONTROL_CONTROLPOSITIONSTANDARD_HPP

#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrMath.hpp"

#include "ExVectrDSP/value_covariance.hpp"

namespace VCTR {
namespace CTRL {

/**
 * @brief   A generalized position control system for a 3D vehicle. This take
 * position and velocity setpoints and outputs an acceleration vector in the
 * reference frame. This output usually is mapped/transformed to get attitude
 * and thrust controls. E,g ControlMappingPosToDrone that is used for drones,
 * tail sitter and other similar underactuated vehicles.
 */
class ControlPositionStandard : public Core::Task_Periodic {
private:
  Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 6>>> posSubr_;

  // Setpoint for the position. In form: [V, P], where V is the linear velocity
  // vector and P is the position vector.
  Core::Simple_Subscriber<Math::Vector<float, 6>> stateSetpointSubr_;

  // This is where the accel control output is published. In form: [X, Y, Z],
  // where X, Y, Z show the accel vector in reference frame
  Core::Topic<Math::Vector<float, 3>> accelTopic_;

  // Current vehicle state estimation. In form: [V, P], where V is the linear
  // velocity vector and P is the position vector.
  Core::Timestamped<Math::Vector<float, 6>> stateEstimation_;

  Math::Vector<float, 3> velCtrlIntegral_;

  // Control parameters
  float positionHZTGain_ = 0.5;
  float positionVRTGain_ = 2;
  float positionHZTLimit_ms_ = 3;
  float positionVRTLimit_ms_ = 15;

  float velocityHZTGain_ = 2.2;
  float velocityVRTGain_ = 2.5;
  float velocityHZTIntegral_ = 0.2;
  float velocityVRTIntegral_ = 0.8;
  float velocityIntegralLimit_ms_ = 3;
  float velocityHZTLimit_mss_ = 2;
  float velocityVRTLimit_mss_ = 8;

  // Runtime data
  int64_t lastRunTimestamp_ = 0; // Timestamp of the last run in microseconds.
  Math::Vector<float, 6> stateSetpoint_;

  bool enableControl_ = false; // Enable or disable the control system.

public:
  /**
   * @brief Constructor for the ControlRocket class.
   */
  ControlPositionStandard();

  /// @brief The P-Term gain for the position controller in the horizontal axis.
  void setPositionHZTGain(float gain) { positionHZTGain_ = gain; }
  /// @brief The P-Term gain for the position controller in the vertical axis.
  void setPositionVRTGain(float gain) { positionVRTGain_ = gain; }
  /// @brief The limit of the position controller output in the horizontal axis.
  /// (Maximum velocity in m/s)
  void setPositionHZTLimit(float limit) { positionHZTLimit_ms_ = limit; }
  /// @brief The limit of the position controller output in the vertical axis.
  /// (Maximum velocity in m/s)
  void setPositionVRTLimit(float limit) { positionVRTLimit_ms_ = limit; }

  /// @brief The P-Term gain for the velocity controller in the horizontal axis.
  void setVelocityHZTGain(float gain) { velocityHZTGain_ = gain; }
  /// @brief The P-Term gain for the velocity controller in the vertical axis.
  void setVelocityVRTGain(float gain) { velocityVRTGain_ = gain; }

  /**
   * @brief Subsribes to a topic to which the position estimation is published
   * in form: [V, P], where V is the linear velocity vector and P is the
   * position vector.
   */
  void subscribePositionMeasurement(
      Core::Topic<Core::Timestamped<Math::Vector<float, 6>>> &posTopic);

  /**
   * @brief Subsribes to a topic to which the setpoint is published in form: [V,
   * P], where V is the linear velocity vector and P is the position vector.
   */
  void subscribeSetpoint(Core::Topic<Math::Vector<float, 6>> &setpointTopic);

  /**
   * @brief Returns the topic to which the wanted acceleration in world frame is
   * published. In form: [X, Y, Z], where X North, Y West, Z Up.
   */
  Core::Topic<Math::Vector<float, 3>> &getAccelTopic();

  void enableControl(bool enable) { enableControl_ = enable; }

  void taskCheck() override;

  void taskInit() override;

  void taskThread() override;
};

} // namespace CTRL
} // namespace VCTR

#endif // EXVECTRCONTROL_SIMPLE_PID_HPP_