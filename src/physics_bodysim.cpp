#include "ExVectrControl/physics_bodysim.hpp"

#include "ExVectrCore/print.hpp"
#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrMath.hpp"

namespace VCTR {

namespace CTRL {

PhysicsBodySim::PhysicsBodySim(float mass,
                               const Math::Matrix<float, 3, 3> &inertiaTensor) {
  mass_ = mass;
  inertiaTensor_ = inertiaTensor;

  positionState_ = {0, 0, 0, 0, 0, 0};
  attitudeState_ = {0, 0, 0, 1, 0, 0, 0};

  forceSumBody_ = 0;
  torqueSumBody_ = 0;
  forceSumNav_ = 0;
  torqueSumNav_ = 0;
}

void PhysicsBodySim::clearForces(bool force, bool torque) {
  if (force) {
    forceSumBody_ = 0;
    forceSumNav_ = 0;
  }
  if (torque) {
    torqueSumBody_ = 0;
    torqueSumNav_ = 0;
  }
}

void PhysicsBodySim::addForce(const Math::Vector<float, 3> &force,
                              const Math::Vector<float, 3> &position,
                              bool bodyFrame, bool isAccel) {
  auto forceTrue = force; // The force vector in body frame

  // Lets start by converting to a force if its an acceleration
  if (isAccel) {
    forceTrue = force * mass_; // Convert to force by multiplying with mass
  }

  if (bodyFrame) {
    forceSumBody_ = forceSumBody_ + forceTrue;
  } else {
    forceSumNav_ = forceSumNav_ + forceTrue;
  }

  // Now we need to add the force to the torque vector as well. We do this by
  // calculating the torque vector from the force vector and the position
  // vector.

  auto torque = position.cross(forceTrue);
  if (bodyFrame) {
    torqueSumBody_ = torqueSumBody_ + torque;
  } else {
    torqueSumNav_ = torqueSumNav_ + torque;
  }

  // LOG_MSG("Force body sum: %.2f %.2f %.2f\n", forceSumBody_(0),
  // forceSumBody_(1), forceSumBody_(2)); // Print the force vector to the
  // console LOG_MSG("Force nav sum: %.2f %.2f %.2f\n", forceSumNav_(0),
  // forceSumNav_(1), forceSumNav_(2)); // Print the force vector to the console
  // LOG_MSG("Torque body sum: %.2f %.2f %.2f\n", torqueSumBody_(0),
  // torqueSumBody_(1), torqueSumBody_(2)); // Print the torque vector to the
  // console LOG_MSG("Torque nav sum: %.2f %.2f %.2f\n", torqueSumNav_(0),
  // torqueSumNav_(1), torqueSumNav_(2)); // Print the torque vector to the
  // console
}

void PhysicsBodySim::addTorque(const Math::Vector<float, 3> &torque,
                               const Math::Vector<float, 3> &position,
                               bool bodyFrame, bool isAccel) {
  auto torqueTrue = torque; // The torque vector in body frame

  // Lets start by converting to a force if its an acceleration
  if (isAccel) {
    torqueTrue = torque * mass_; // Convert to force by multiplying with mass
  }

  if (bodyFrame) {
    torqueSumBody_ = torqueSumBody_ + torqueTrue;
  } else {
    torqueSumNav_ = torqueSumNav_ + torqueTrue;
  }

  // LOG_MSG("Torque body sum: %.2f %.2f %.2f\n", torqueSumBody_(0),
  // torqueSumBody_(1), torqueSumBody_(2)); // Print the torque vector to the
  // console LOG_MSG("Torque nav sum: %.2f %.2f %.2f\n", torqueSumNav_(0),
  // torqueSumNav_(1), torqueSumNav_(2)); // Print the torque vector to the
  // console
}

void PhysicsBodySim::simulateForTime(int64_t time) {
  float dTime = double(time) / double(Core::SECONDS);
  float hdtsq = 0;

  // Calculate the position state transition and input models
  Math::Matrix<float, 6, 6> F = {
      1,     0, 0, 0, 0, 0, 0, 1,     0, 0, 0, 0, 0, 0, 1,     0, 0, 0,
      dTime, 0, 0, 1, 0, 0, 0, dTime, 0, 0, 1, 0, 0, 0, dTime, 0, 0, 1};

  Math::Matrix<float, 6, 3> B = {
      dTime / mass_, 0, 0, 0, dTime / mass_, 0, 0, 0, dTime / mass_,
      hdtsq / mass_, 0, 0, 0, hdtsq / mass_, 0, 0, 0, hdtsq / mass_};

  Math::Quat_F attQuat = attitudeState_.block<4, 1>(3, 0);
  Math::Vector<float, 3> totalworldForceRef =
      attQuat.rotate(forceSumBody_) + forceSumNav_;
  positionState_ = F * positionState_ + B * totalworldForceRef;

  Math::Vector<float, 3> angVel = attitudeState_.block<3, 1>(0, 0);

  // Update the attitude and angular velocity
  auto totalTorque = torqueSumBody_;
  attQuat =
      attQuat * Math::Quat_F(angVel.normalize(), angVel.magnitude() * dTime);
  angVel(0) = angVel(0) + totalTorque(0) / inertiaTensor_(0, 0) * dTime;
  angVel(1) = angVel(1) + totalTorque(1) / inertiaTensor_(1, 1) * dTime;
  angVel(2) = angVel(2) + totalTorque(2) / inertiaTensor_(2, 2) * dTime;
  attQuat = attQuat.normalize();
  if (attQuat(0) < 0) { // If the scalar part is negative, invert the rotation
    attQuat = -attQuat;
  }

  // Update the attitude state
  attitudeState_.block(angVel, 0, 0);
  attitudeState_.block(attQuat, 3, 0);
}

} // namespace CTRL

} // namespace VCTR
