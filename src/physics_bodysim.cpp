#include "ExVectrCore/print.hpp"

#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/time_definitions.hpp"

#include "ExVectrMath.hpp"

#include "ExVectrControl/physics_bodysim.hpp"


namespace VCTR
{

    namespace CTRL
    {

        PhysicsBodySim::PhysicsBodySim(float mass, const Math::Matrix<float, 3, 3>& inertiaTensor) {
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

        void PhysicsBodySim::addForce(const Math::Vector<float, 3>& force, const Math::Vector<float, 3>& position, bool bodyFrame, bool isAccel) {

            auto forceTrue = force; // The force vector in body frame

            //Lets start by converting to a force if its an acceleration
            if (isAccel) {
                forceTrue = force * mass_; // Convert to force by multiplying with mass
            }

            if (bodyFrame) {
                forceSumBody_ = forceSumBody_ + forceTrue; // Add the force vector to the total force vector in body frame
            } else {
                forceSumNav_ = forceSumNav_ + forceTrue; // Add the force vector to the total force vector in reference frame
            }

            //Now we need to add the force to the torque vector as well. We do this by calculating the torque vector from the force vector and the position vector.
            
            auto torque = position.cross(forceTrue); // Calculate the torque vector from the force vector and the position vector
            if (bodyFrame) {
                torqueSumBody_ = torqueSumBody_ + torque; // Add the torque vector to the total torque vector in body frame
            } else {
                torqueSumNav_ = torqueSumNav_ + torque; // Add the torque vector to the total torque vector in reference frame
            }

            //LOG_MSG("Force body sum: %.2f %.2f %.2f\n", forceSumBody_(0), forceSumBody_(1), forceSumBody_(2)); // Print the force vector to the console
            //LOG_MSG("Force nav sum: %.2f %.2f %.2f\n", forceSumNav_(0), forceSumNav_(1), forceSumNav_(2)); // Print the force vector to the console
            //LOG_MSG("Torque body sum: %.2f %.2f %.2f\n", torqueSumBody_(0), torqueSumBody_(1), torqueSumBody_(2)); // Print the torque vector to the console
            //LOG_MSG("Torque nav sum: %.2f %.2f %.2f\n", torqueSumNav_(0), torqueSumNav_(1), torqueSumNav_(2)); // Print the torque vector to the console
            

        }

        void PhysicsBodySim::addTorque(const Math::Vector<float, 3>& torque, const Math::Vector<float, 3>& position, bool bodyFrame, bool isAccel) {

            auto torqueTrue = torque; // The torque vector in body frame

            //Lets start by converting to a force if its an acceleration
            if (isAccel) {
                torqueTrue = torque * mass_; // Convert to force by multiplying with mass
            }

            if (bodyFrame) {
                torqueSumBody_ = torqueSumBody_ + torqueTrue; // Add the torque vector to the total torque vector in body frame
            } else {
                torqueSumNav_ = torqueSumNav_ + torqueTrue; // Add the torque vector to the total torque vector in reference frame
            }

            //LOG_MSG("Torque body sum: %.2f %.2f %.2f\n", torqueSumBody_(0), torqueSumBody_(1), torqueSumBody_(2)); // Print the torque vector to the console
            //LOG_MSG("Torque nav sum: %.2f %.2f %.2f\n", torqueSumNav_(0), torqueSumNav_(1), torqueSumNav_(2)); // Print the torque vector to the console

        }

        void PhysicsBodySim::simulateForTime(int64_t time) {

            float dTime = double(time) / double(Core::SECONDS); // Convert time to seconds
            float hdtsq = 0 ;// 0.5 * dTime * dTime;

            // Calculate the position state transition and input models
            Math::Matrix<float, 6, 6> F = {
                1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0,
                dTime, 0, 0, 1, 0, 0,
                0, dTime, 0, 0, 1, 0,
                0, 0, dTime, 0, 0, 1
            };

            Math::Matrix<float, 6, 3> B = {
                dTime / mass_, 0, 0,
                0, dTime / mass_, 0,
                0, 0, dTime / mass_,
                hdtsq, 0, 0,
                0, hdtsq, 0,
                0, 0, hdtsq
            };

            // Calculate total force in reference frame from body frame
            Math::Quat_F attQuat = attitudeState_.block<4, 1>(3, 0); // Quaternion in body frame
            //Math::Vector<float, 3> additionalForce = {0, 1, 0}; // Additional force in body frame
            Math::Vector<float, 3> totalBodyForceRef = attQuat.conjugate().rotate(forceSumBody_) + forceSumNav_;// + additionalForce; // Rotate the force vector to reference frame
            //LOG_MSG("Total body force: %.2f %.2f %.2f\n", totalBodyForceRef(0), totalBodyForceRef(1), totalBodyForceRef(2)); // Print the force vector to the console
            // Calculate the new position state
            positionState_ = F * positionState_ + B * totalBodyForceRef;

            Math::Vector<float, 3> angVel = attitudeState_.block<3, 1>(0, 0); // Angular velocity in body frame

            // Update the attitude and angular velocity
            attQuat = attQuat * Math::Quat_F(angVel.normalize(), angVel.magnitude() * dTime); // Update quaternion using angular velocity
            angVel(0) += torqueSumBody_(0) / inertiaTensor_(0, 0) * dTime; // Update angular velocity using torque and inertia tensor
            angVel(1) += torqueSumBody_(1) / inertiaTensor_(1, 1) * dTime; // Update angular velocity using torque and inertia tensor
            angVel(2) += torqueSumBody_(2) / inertiaTensor_(2, 2) * dTime; // Update angular velocity using torque and inertia tensor
            attQuat.normalize(); // Normalize the quaternion

            //Update the attitude state
            attitudeState_.block(angVel, 0, 0, 0, 0);
            attitudeState_.block(attQuat, 3, 0, 0, 0); // Update the quaternion in the state vector

            //LOG_MSG("Interia tensor: %.2f %.2f %.2f\n", inertiaTensor_(0, 0), inertiaTensor_(1, 1), inertiaTensor_(2, 2)); // Print the inertia tensor to the console
            //LOG_MSG("Attitude: %.2f %.2f %.2f %.2f\n", attitudeState_(0), attitudeState_(1), attitudeState_(2), attitudeState_(3)); // Print the attitude to the console
            //LOG_MSG("Angular velocity: %.2f %.2f %.2f\n", attitudeState_(4), attitudeState_(5), attitudeState_(6)); // Print the angular velocity to the console
            //LOG_MSG("Position: %.2f %.2f %.2f\n", positionState_(3), positionState_(4), positionState_(5)); // Print the position to the console
            //LOG_MSG("Velocity: %.2f %.2f %.2f\n", positionState_(0), positionState_(1), positionState_(2)); // Print the velocity to the console


        }


    }
    
}
