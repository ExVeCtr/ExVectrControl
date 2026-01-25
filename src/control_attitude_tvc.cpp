#include "ExVectrControl/control_attitude_tvc.hpp"

#include "ExVectrCore/print.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrDSP/value_covariance.hpp"
#include "ExVectrMath/constants.hpp"
#include "ExVectrMath/matrix_quaternion.hpp"
#include "ExVectrMath/matrix_vector.hpp"

namespace VCTR {
namespace CTRL {

ControlAttitudeTvc::ControlAttitudeTvc(float vehicleMass_kg,
                                       float tvcThrustLimit_N,
                                       float tvcAngleLimit_Rad)
    : Core::Task_Periodic("Control Rocket", 20 * Core::MILLISECONDS) {
  // Initialize the control parameters
  vehicleMass_kg_ = vehicleMass_kg;
  tvcThrustLimit_N_ = tvcThrustLimit_N;
  tvcAngleLimit_Rad_ = tvcAngleLimit_Rad;

  // attach to scheduler
  Core::getSystemScheduler().addTask(*this);
}

/**
 * * @brief Subsribes to a topic to which the attitude estimation is published
 * in form: [W, Q], where W is the angular velocity vector and Q is a unit
 * quaternion rotation from the reference frame to body frame.
 */
void ControlAttitudeTvc::subscribeAttitudeMeasurement(
    Core::Topic<Core::Timestamped<Math::Vector<float, 7>>> &attTopic) {
  attSubr_.subscribe(attTopic);
}

void ControlAttitudeTvc::subscribeAccelerationSetpoint(
    Core::Topic<Math::Vector<float, 3>> &setpointTopic) {
  accelSetpointSubr_.subscribe(setpointTopic);
}

/**
 * @brief Subsribes to a topic to which the setpoint is published in form: [V,
 * P], where V is the linear velocity vector and P is the position vector.
 */
void ControlAttitudeTvc::subscribeAttitudeSetpoint(
    Core::Topic<Math::Vector<float, 7>> &setpointTopic) {
  stateSetpointSubr_.subscribe(setpointTopic);
}

/**
 * @brief Returns the topic to which the thrust vector control output is
 * published. In form: [X, Y, Z, T], where X, Y, Z show the thrust vector in
 * body frame (magnitude of vector is thrust magnitude) and T is the roll torque
 * (Z-Axis) angle in radians.
 */
Core::Topic<Math::Vector<float, 4>> &ControlAttitudeTvc::getTvcTopic() {
  return tvcTopic_;
}

void ControlAttitudeTvc::taskCheck() {}

void ControlAttitudeTvc::taskInit() {
  // Only run when all subscribers have gotten data at least once
  if (!attSubr_.isDataNew() || !stateSetpointSubr_.isDataNew()) {
    setInitialised(false);
  }

  lastRunTimestamp_ = Core::NOW();
  accelSetpoint_ = 0;
  integral_ = 0; // Reset the integral term for the attitude control
}

void ControlAttitudeTvc::taskThread() {
  using namespace VCTR::Math;
  /**
   * To understand this controller, we will go through a chain of controller
   * from the bottom (TVC output) to the top (position setpoint). The chain is
   * as follows: Setpoint -> Position -> Velocity -> Attitude -> TVC output. TVC
   * Output:
   * - The TVC output is a vector in body frame (X, Y, Z) and a roll torque (T).
   * - The TVC Input is a wanted torque from attitude control and thrust
   * magnitude from velocity control. Both in body frame.
   * - The input to the calculation is a torque to achieve the wanted attitude
   * and a thrust magnitude.
   * - The TVC angle and magnitude are optimized to achieve the wanted torque as
   * close as possible and thrust magnitude comes second.
   *
   * Attitude:
   * - The attitude output is a torque vector in body frame.
   * - The attitude input is a wanted attitude in quaternion from reference from
   * to body frame.
   * - The attitude output uses a simple quaterion based controller to calculate
   * the torque vector.
   *
   * Velocity:
   * - The velocity output is a wanted attitude and thrust magnitude in body
   * frame.
   * - The velocity input is a wanted velocity in reference frame
   * - The attitude output can be limited to a certain angle off the vertical
   * axis (Z-axis). (This is for safety during hovering).
   * - The velocity control points the rocket in the wanted force direction and
   * applys a force to achieve the wanted velocity.
   * - The velocity control algorithm takes the gravity force into account.
   *
   * Position:
   * - The position output is a wanted velocity in reference frame.
   * - The position input is a wanted position in reference frame.
   * - The position control simply calculated the wanted velocity via a simple
   * proportional controller.
   * - The position control limits the wanted velocity to a certain value (this
   * is for safety during hovering).
   */

  // Calculate the the delta time difference between control runs
  float dTime = double(Core::NOW() - lastRunTimestamp_) / Core::SECONDS;
  lastRunTimestamp_ = Core::NOW();

  // Retrieve the latest data from the subscribers
  Quat_F attitude = attSubr_.getItem().data.block<4, 1>(3, 0);
  auto angularVelocity = attSubr_.getItem().data.block<3, 1>(0, 0);

  if (stateSetpointSubr_.isDataNew()) {
    stateSetpoint_ =
        stateSetpointSubr_.getItem(); // Get the setpoint from the subscriber
  } else {
    // Propagate the setpoint if no new data is available
    Quat<float> att = stateSetpoint_.block<4, 1>(3, 0);
    att = att * Quat<float>(angularVelocity.normalize(),
                            angularVelocity.magnitude() * dTime);
    stateSetpoint_(3) = att(0);
    stateSetpoint_(4) = att(1);
    stateSetpoint_(5) = att(2);
    stateSetpoint_(6) = att(3);
  }

  if (accelSetpointSubr_.isDataNew()) {
    accelSetpoint_ = accelSetpointSubr_.getItem();
  }

  Quat<float> wantedAttitude = stateSetpoint_.block<4, 1>(3, 0);
  Vector_F wantedAngularVelocity = stateSetpoint_.block<3, 1>(0, 0);

  // wantedAttitude = Quat_F(Vector_F{0, 1, 0}, 90 * DEGREES);

  // #################### Calculate the attitude error ################
  auto quatOut = attitude.conjugate() * wantedAttitude;
  if (quatOut(0) < 0)
    quatOut = -quatOut; // Ensure a positive rotation

  auto axisZ = Vector<float, 3>{0, 0, 1};
  auto wantedZInBody = quatOut.rotate(axisZ);
  auto bodyZRotAng = axisZ.getAngleTo(wantedZInBody);
  auto bodyZRotVec = axisZ.cross(wantedZInBody).normalize() * bodyZRotAng;

  if (bodyZRotAng < 10 * DEGREES) {
    auto buf = quatOut;
    auto yawError = atan2(buf(3), buf(0));
    bodyZRotVec(2) = yawError;
  }
  // The rotation error in body frame to rotate the current attitude to the
  // wanted attitude in Rad.
  Vector<float, 3> bodyRotationAngleError = bodyZRotVec;

  // LOG_MSG("Attitude error: %.2f %.2f %.2f\n", bodyRotationAngleError(0),
  // bodyRotationAngleError(1), bodyRotationAngleError(2));

  // ##### Calculate the attitude controller output #####
  // We use a quaternion based algorithm to calculate the rotation error between
  // the wanted attitude and the current attitude. The result is in body frame.
  Vector<float, 3> attCtrlOutput({bodyRotationAngleError(0) * attitudeGain_,
                                  bodyRotationAngleError(1) * attitudeGain_,
                                  bodyRotationAngleError(2) * attitudeZGain_});

  // ################### Calculate the integral term ################
  // We use a simple integral term to reduce the steady state error. The
  // integral term is calculated by summing up the attitude error multiplied by
  // the delta time and the integral gain. We must calculate the XY seperate
  // from the Z-axis, because the Z-axis dynamics are different.
  if (enableControl_) {
    integral_(0) += bodyRotationAngleError(0) * dTime * attitudeIntegralGain_;
    integral_(1) += bodyRotationAngleError(1) * dTime * attitudeIntegralGain_;
    integral_(2) += bodyRotationAngleError(2) * dTime * attitudeIntegralZGain_;
  } else {
    integral_ = 0; // Reset the integral term if control is disabled
  }
  // We must limit the integral term to prevent windup. We do this by clamping
  // the integral term to a maximum value.
  if (integral_(0) > integralLimit_)
    integral_(0) = integralLimit_;
  else if (integral_(0) < -integralLimit_)
    integral_(0) = -integralLimit_;
  if (integral_(1) > integralLimit_)
    integral_(1) = integralLimit_;
  else if (integral_(1) < -integralLimit_)
    integral_(1) = -integralLimit_;
  if (integral_(2) > integralZLimit_)
    integral_(2) = integralZLimit_;
  else if (integral_(2) < -integralZLimit_)
    integral_(2) = -integralZLimit_;

  // ####### Calculate the attitude rate controller output #######
  Vector<float, 3> attRateCtrlOutput({
      (wantedAngularVelocity(0) - angularVelocity(0)) * attitudeRateGain_,
      (wantedAngularVelocity(1) - angularVelocity(1)) * attitudeRateGain_,
      (wantedAngularVelocity(2) - angularVelocity(2)) * attitudeRateZGain_,
  });

  attCtrlOutput = attCtrlOutput + attRateCtrlOutput + integral_;

  // ################### Calculate the TVC output ################
  Vector_F wantedBodyForce =
      attitude.conjugate().rotate(accelSetpoint_ * vehicleMass_kg_);
  Vector_F torqueVec =
      attCtrlOutput.cross(Vector<float, 3>({0, 0, 1 / tvcCGOffset_m_}));
  float bodyForceMagnitude = wantedBodyForce.magnitude();
  float cosLosses =
      cos(wantedBodyForce.getAngleTo(Vector<float, 3>({0, 0, 1})));
  if (cosLosses < 0) {
    cosLosses = 0;
  }
  Vector<float, 3> forceVector = torqueVec;
  forceVector(2) += bodyForceMagnitude * cosLosses;

  // LOG_MSG("Wanted body force: %.2f %.2f %.2f |%.2f|\n", wantedBodyForce(0),
  // wantedBodyForce(1), wantedBodyForce(2), wantedBodyForce.magnitude());

  // We now must take the TVC angle limit into account.
  // If the requested vector angle is over the limit, we use the closest
  // possible and then we scale the output until the requested torque is
  // reached.
  auto tvcAngle = forceVector.getAngleTo(Vector<float, 3>{0, 0, 1});
  auto tvcRotationAxis =
      (Vector<float, 3>{0, 0, 1}.cross(forceVector)).normalize();
  if (tvcAngle > tvcAngleLimit_Rad_) {
    auto tvcRotation = Quat_F(tvcRotationAxis, tvcAngleLimit_Rad_);
    auto forceVectorNew = tvcRotation.rotate(Vector<float, 3>{0, 0, 1});
    // Now we must scale it so the resulting torque is the same as the requested
    // torque. The torque is the cross product of the force vector and the
    // distance to the CG.
    auto xyMag =
        sqrt(forceVector(0) * forceVector(0) + forceVector(1) * forceVector(1));
    auto xyMagNew = sqrt(forceVectorNew(0) * forceVectorNew(0) +
                         forceVectorNew(1) * forceVectorNew(1));
    auto correctionFactor = xyMag / xyMagNew;
    if (compensateTVCAngle_)
      forceVectorNew = forceVectorNew * correctionFactor;
    forceVector = forceVectorNew;
  }

  // LOG_MSG("Force vector: %.2f %.2f %.2f |%.2f|\n", forceVector(0),
  // forceVector(1), forceVector(2), forceVector.magnitude()); // Print the
  // force vector to the console

  // ################### publish the TVC output ################
  Vector<float, 4> tvcOutput = Vector<float, 4>(
      {forceVector(0), forceVector(1), forceVector(2), attCtrlOutput(2)});

  // enableControl_ = true; //Enable the control fro debuggin

  if (!enableControl_) {
    tvcOutput = Vector<float, 4>({0, 0, 0.1, 0});
  }

  tvcTopic_.publish(tvcOutput); // Publish the TVC output
}

} // namespace CTRL
} // namespace VCTR
