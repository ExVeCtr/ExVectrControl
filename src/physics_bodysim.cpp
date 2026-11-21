#include "ExVectrControl/physics_bodysim.hpp"

#include <math.h>

#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrMath.hpp"

namespace VCTR {

namespace CTRL {

PhysicsBodySim::PhysicsBodySim(float mass,
                               const Math::Matrix<float, 3, 3> &inertiaTensor) {
  mass_ = mass;
  setInertiaTensor(inertiaTensor);

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
  (void)position; // a pure torque is independent of where it acts
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

void PhysicsBodySim::setInertiaTensor(
    const Math::Matrix<float, 3, 3> &inertiaTensor) {
  inertiaTensor_ = inertiaTensor;
  inertiaInverse_ = inertiaTensor.inverse();
}

void PhysicsBodySim::simulateForTime(int64_t time) {
  step(float(double(time) / double(Core::SECONDS)));
}

void PhysicsBodySim::step(float dt) {
  if (!(dt > 0.0f) || !isfinite(dt))
    return;

  // Translation: world-frame force = R(q) * body force + world force.
  Math::Quat_F attQuat = attitudeState_.block<4, 1>(3, 0);
  const Math::Vector<float, 3> forceWorld =
      attQuat.rotate(forceSumBody_) + forceSumNav_;
  for (size_t i = 0; i < 3; i++) {
    positionState_(i) += forceWorld(i) / mass_ * dt; // v first
    positionState_(i + 3) += positionState_(i) * dt; // then p with the new v
  }

  // Rotation: I domega/dt = tau - omega x (I omega), body frame. A world-frame torque sum (unused by the
  // simulator) is rotated into the body frame.
  Math::Vector<float, 3> angVel = attitudeState_.block<3, 1>(0, 0);
  const Math::Vector<float, 3> torqueBody =
      torqueSumBody_ + attQuat.conjugate().rotate(torqueSumNav_);
  const Math::Vector<float, 3> angMom = inertiaTensor_ * angVel;
  const Math::Vector<float, 3> angAcc =
      inertiaInverse_ * (torqueBody - angVel.cross(angMom));
  angVel = angVel + angAcc * dt; // omega first

  // q <- q * dq(omega dt) with the new omega (body-frame rate, Hamilton).
  const float rate = angVel.magnitude();
  if (rate > 1e-9f) {
    attQuat = attQuat * Math::Quat_F(angVel / rate, rate * dt);
  }
  attQuat = attQuat.normalize();
  if (attQuat(0) < 0) { // keep the scalar part non-negative (same rotation)
    attQuat = -attQuat;
  }

  attitudeState_.block(angVel, 0, 0);
  attitudeState_.block(attQuat, 3, 0);
}

} // namespace CTRL

} // namespace VCTR
