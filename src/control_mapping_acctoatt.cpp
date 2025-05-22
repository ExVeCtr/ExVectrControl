#include "ExVectrCore/print.hpp"

#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrMath/matrix_vector.hpp"
#include "ExVectrMath/matrix_quaternion.hpp"
#include "ExVectrMath/constants.hpp"

#include "ExVectrDSP/value_covariance.hpp"

#include "ExVectrControl/control_mapping_acctoatt.hpp"


namespace VCTR
{
    namespace CTRL
    {

        
        ControlMappingAccToAtt::ControlMappingAccToAtt() 
        {

            accelSubr_.setCallback(this, &ControlMappingAccToAtt::callbackMapper);

        }


        /**
         * @brief Subsribes to a topic to which the position estimation is published in form: [V, P], where V is the linear velocity vector and P is the position vector.
         */
        void ControlMappingAccToAtt::subscribeAccelSetpoint(Core::Topic<Math::Vector<float, 3>> &posTopic)
        {
            accelSubr_.unsubscribe();
            accelSubr_.subscribe(posTopic);
        }

        /**
         * @brief Returns the topic to which the thrust vector control output is published. In form: [X, Y, Z, T], where X, Y, Z show the thrust vector in body frame (magnitude of vector is thrust magnitude) and T is the roll torque (Z-Axis) angle in radians.
         */
        Core::Topic<Math::Vector<float, 7>> &ControlMappingAccToAtt::getAttitudeTopic()
        {
            return attitudeTopic_;
        }

        void ControlMappingAccToAtt::callbackMapper(const Math::Vector<float, 3> &accelSetpoint)
        {

            //Now we want to calculate the amount we must tilt the rocket and in what yaw (azimuth) direction to achieve the wanted force.
            auto tilt = accelSetpoint.getAngleTo(Math::Vector<float, 3>({0, 0, 1}));
            //Now we limit the tilt angle to the maximum tilt angle
            auto tiltOffAngle = 0;
            if (tilt > velocityLimit_Rad_) {
                tiltOffAngle = tilt - velocityLimit_Rad_;
                tilt = velocityLimit_Rad_;
            }
            //auto wantedForceMagnitude = accelSetpoint.magnitude();

            //auto wantedForce = velCtrlOutput.magnitude() * vehicleMass_kg_;
            //auto cosLosses = cos(tiltOffAngle); //Cosine losses due to the wanted tilt angle possibly not being reached.
            //if (cosLosses < 0) cosLosses = 0; //If the tilt angle is over 90 degrees, we don't want to apply any force, othewise we technically would need a negative force.
            //wantedForceMagnitude = wantedForceMagnitude / cosLosses; //Apply the cosine losses to the wanted force.

            //Now we calculate the wanted attitude to achieve the wanted acceleration vector direction.
            Math::Quat_F wantedAttitude;
            auto rotAxis = Math::Vector<float, 3>({0, 0, 1}).cross(accelSetpoint).normalize(); //Calculate the rotation axis from the wanted force vector to the Z-axis
            auto rotAngle = Math::Vector<float, 3>({0, 0, 1}).getAngleTo(accelSetpoint); //Calculate the rotation angle from the wanted force vector to the Z-axis

            //LOG_MSG("Rot ang: %.2f, axis: %.2f %.2f %.2f\n", rotAngle * 180/3.14, rotAxis(0), rotAxis(1), rotAxis(2)); // Print the rotation axis to the console

            if (rotAngle > velocityLimit_Rad_)
                rotAngle = velocityLimit_Rad_;
            else if (rotAngle < -velocityLimit_Rad_)
                rotAngle = -velocityLimit_Rad_;

            wantedAttitude = Math::Quat_F(rotAxis, rotAngle);
            //LOG_MSG("Wanted attitude: %.2f %.2f %.2f %.2f\n", wantedAttitude(0), wantedAttitude(1), wantedAttitude(2), wantedAttitude(3)); // Print the wanted attitude to the console
            attitudeTopic_.publish({0, 0, 0, wantedAttitude(0), wantedAttitude(1), wantedAttitude(2), wantedAttitude(3)}); //Publish the wanted attitude to the topic

        }

    }
}

