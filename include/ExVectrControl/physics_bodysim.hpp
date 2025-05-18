#ifndef EXVECTRCONTROL_BODYSIM_HPP
#define EXVECTRCONTROL_BODYSIM_HPP

#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/time_definitions.hpp"

#include "ExVectrMath.hpp"


namespace VCTR
{

    namespace CTRL
    {

        /**
         * @brief A class implementing a simple simulation for forces/torques on a fixed body.
         */
        class PhysicsBodySim 
        {
        protected:

            Math::Vector<float, 3> forceSumBody_; // Sum of forces in body frame
            Math::Vector<float, 3> torqueSumBody_; // Sum of torques in body frame

            Math::Vector<float, 3> forceSumNav_; // Sum of forces in Nav frame
            Math::Vector<float, 3> torqueSumNav_; // Sum of torques in Nav frame

            Math::Vector<float, 6> positionState_; // Position of the body in reference frame
            Math::Vector<float, 7> attitudeState_; // Attitude of the body in reference frame

            float mass_; // Mass of the body in kg
            Math::Matrix<float, 3, 3> inertiaTensor_; // Inertia tensor of the body in kg*m^2


        public:
            
            /**
             * @brief Standard constructor. Sets state to 0 and covariance to 1000 as starting values.
             */
            PhysicsBodySim(float mass, const Math::Matrix<float, 3, 3>& inertiaTensor);

            /**
             * @brief adds a force to a body at a given position.
             * @param force The force to add in body frame.
             * @param position The position to add the force at in body frame.
             * @param bodyFrame If true, then the force is added in body frame. If false, then the force is added in reference frame (World frame).
             * @param isAccel If true, then the force is added as an acceleration (Ignoring mass).
             */
            void addForce(const Math::Vector<float, 3>& force, const Math::Vector<float, 3>& position = 0, bool bodyFrame = true, bool isAccel = false);

            /**
             * @brief adds a torque to a body at a given position.
             * @param torque The torque to add in body frame.
             * @param position The position to add the torque at in body frame.
             * @param bodyFrame If true, then the torque is added in body frame. If false, then the torque is added in reference frame (World frame).
             */
            void addTorque(const Math::Vector<float, 3>& torque, const Math::Vector<float, 3>& position = 0, bool bodyFrame = true, bool isAccel = false);

            /**
             * @brief Clears the forces and torques applied to the body.
             * @param force If true, then the forces are cleared.
             * @param torque If true, then the torques are cleared.
             */
            void clearForces(bool force = true, bool torque = true);

            void setMass(float mass) { mass_ = mass; } // Set the mass of the body in kg
            void setInertiaTensor(const Math::Matrix<float, 3, 3>& inertiaTensor) { inertiaTensor_ = inertiaTensor; } // Set the inertia tensor of the body in kg*m^2

            /**
             * @brief Sets the position state [V, P] of the body in reference frame.
             * @param positionState The position state to set in reference frame.
             */
            void setPositionState(const Math::Vector<float, 6>& positionState) { positionState_ = positionState; }
            
            void setPosition(const Math::Vector<float, 3>& position) { positionState_.block(position, 3, 0); } // Set the position of the body in reference frame
            void setVelocity(const Math::Vector<float, 3>& velocity) { positionState_.block(velocity); } // Set the velocity of the body in reference frame

            /**
             * @brief Sets the attitude state [W, Q] of the body in reference frame.
             * @param attitudeState The attitude state to set in reference frame.
             */
            void setAttitudeState(const Math::Vector<float, 7>& attitudeState) { attitudeState_ = attitudeState; }

            void setAttitude(const Math::Quat<float>& attitude) { attitudeState_.block(attitude, 3, 0); } // Set the attitude of the body in reference frame
            void setAngularVelocity(const Math::Vector<float, 3>& angularVelocity) { attitudeState_.block(angularVelocity); } // Set the angular velocity of the body in reference frame

            /**
             * @returns the current position state of the body in reference frame as [V, P].
             */
            Math::Vector<float, 6>& getPositionState() { return positionState_; } // Get the position state in reference frame

            Math::Vector<float, 3> getPosition() { return positionState_.block<3, 1>(3); } // Get the position of the body in reference frame
            Math::Vector<float, 3> getVelocity() { return positionState_.block<3, 1>(0); } // Get the velocity of the body in reference frame

            /**
             * @returns the current attitude state of the body in reference frame as [W, Q].
             */
            Math::Vector<float, 7>& getAttitudeState() { return attitudeState_; } // Get the attitude state in reference frame

            Math::Quat<float> getAttitude() { return attitudeState_.block<4, 1>(3); } // Get the attitude of the body in reference frame
            Math::Vector<float, 3> getAngularVelocity() { return attitudeState_.block<3, 1>(0); } // Get the angular velocity of the body in reference frame

            /**
             * @returns the current force sum of the body in body frame.
             */
            Math::Vector<float, 3>& getBodyForceSum() { return forceSumBody_; }
            Math::Vector<float, 3>& getNavForceSum() { return forceSumNav_; } // Get the force sum in reference frame

            /**
             * @returns the current torque sum of the body in body frame.
             */
            Math::Vector<float, 3>& getBodyTorqueSum() { return torqueSumBody_; }
            Math::Vector<float, 3>& getNavTorqueSum() { return torqueSumNav_; } // Get the torque sum in reference frame


            Math::Matrix<float, 3, 3> getInertiaTensor() { return inertiaTensor_; } // Get the inertia tensor of the body
            float getMass() { return mass_; } // Get the mass of the body


            void simulateForTime(int64_t time); // Simulate the body for a given time.


        private:

            

        };

    }
    
}

#endif