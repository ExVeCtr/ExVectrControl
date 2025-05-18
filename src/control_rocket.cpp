#include "ExVectrCore/print.hpp"

#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrMath/matrix_vector.hpp"
#include "ExVectrMath/matrix_quaternion.hpp"
#include "ExVectrMath/constants.hpp"

#include "ExVectrDSP/value_covariance.hpp"

#include "ExVectrControl/control_rocket.hpp"


namespace VCTR
{
    namespace CTRL
    {

        
        ControlRocket::ControlRocket(float vehicleMass_kg, float tvcThrustLimit_N, float tvcAngleLimit_Rad) :
            Core::Task_Periodic("Control Rocket", 50*Core::MILLISECONDS)
        {
            // Initialize the control parameters
            vehicleMass_kg_ = vehicleMass_kg;
            tvcThrustLimit_N_ = tvcThrustLimit_N;
            tvcAngleLimit_Rad_ = tvcAngleLimit_Rad;

            // attach to scheduler
            Core::getSystemScheduler().addTask(*this);

        }

        /**
         * * @brief Subsribes to a topic to which the attitude estimation is published in form: [W, Q], where W is the angular velocity vector and Q is a unit quaternion rotation from the reference frame to body frame.
         */
        void ControlRocket::subscribeAttitudeMeasurement(Core::Topic<Core::Timestamped<Math::Vector<float, 7>>> &attTopic)
        {
            attSubr_.subscribe(attTopic);
        }

        /**
         * @brief Subsribes to a topic to which the position estimation is published in form: [V, P], where V is the linear velocity vector and P is the position vector.
         */
        void ControlRocket::subscribePositionMeasurement(Core::Topic<Core::Timestamped<Math::Vector<float, 6>>> &posTopic) 
        {
            posSubr_.subscribe(posTopic);
        }

        /**
         * @brief Subsribes to a topic to which the setpoint is published in form: [V, P], where V is the linear velocity vector and P is the position vector.
         */
        void ControlRocket::subscribeSetpoint(Core::Topic<Math::Vector<float, 6>> &setpointTopic)
        {
            stateSetpointSubr_.subscribe(setpointTopic);
        }

        /**
         * @brief Returns the topic to which the thrust vector control output is published. In form: [X, Y, Z, T], where X, Y, Z show the thrust vector in body frame (magnitude of vector is thrust magnitude) and T is the roll torque (Z-Axis) angle in radians.
         */
        Core::Topic<Math::Vector<float, 4>> &ControlRocket::getTvcTopic()
        {
            return tvcTopic_;
        }

        void ControlRocket::taskCheck() {

        }

        void ControlRocket::taskInit() {

            //Only run when all subscribers have gotten data at least once
            if (!attSubr_.isDataNew() || !posSubr_.isDataNew() || !stateSetpointSubr_.isDataNew()) {
                setInitialised(false);
            }

            lastRunTimestamp_ = Core::NOW();

        }

        void ControlRocket::taskThread() {

            /**
             * To understand this controller, we will go through a chain of controller from the bottom (TVC output) to the top (position setpoint).
             * The chain is as follows: Setpoint -> Position -> Velocity -> Attitude -> TVC output.
             * TVC Output: 
             * - The TVC output is a vector in body frame (X, Y, Z) and a roll torque (T).
             * - The TVC Input is a wanted torque from attitude control and thrust magnitude from velocity control. Both in body frame.
             * - The input to the calculation is a torque to achieve the wanted attitude and a thrust magnitude.
             * - The TVC angle and magnitude are optimized to achieve the wanted torque as close as possible and thrust magnitude comes second.
             * 
             * Attitude:
             * - The attitude output is a torque vector in body frame.
             * - The attitude input is a wanted attitude in quaternion from reference from to body frame.
             * - The attitude output uses a simple quaterion based controller to calculate the torque vector.
             * 
             * Velocity:
             * - The velocity output is a wanted attitude and thrust magnitude in body frame.
             * - The velocity input is a wanted velocity in reference frame
             * - The attitude output can be limited to a certain angle off the vertical axis (Z-axis). (This is for safety during hovering).
             * - The velocity control points the rocket in the wanted force direction and applys a force to achieve the wanted velocity.
             * - The velocity control algorithm takes the gravity force into account.
             * 
             * Position:
             * - The position output is a wanted velocity in reference frame.
             * - The position input is a wanted position in reference frame.
             * - The position control simply calculated the wanted velocity via a simple proportional controller.
             * - The position control limits the wanted velocity to a certain value (this is for safety during hovering).
             */


            //Calculate the the delta time difference between control runs
            float dTime = double(Core::NOW() - lastRunTimestamp_) / Core::SECONDS;
            lastRunTimestamp_ = Core::NOW();

            //Retrieve the latest data from the subscribers
            auto position = posSubr_.getItem().data.block<3, 1>(3, 0);
            auto velocity = posSubr_.getItem().data.block<3, 1>(0, 0);
            
            Math::Quat_F attitude = attSubr_.getItem().data.block<4, 1>(3, 0);
            auto angularVelocity = attSubr_.getItem().data.block<3, 1>(0, 0);

            if (stateSetpointSubr_.isDataNew()) {
                stateSetpoint_ = stateSetpointSubr_.getItem(); // Get the setpoint from the subscriber
            } else {
                //Propagate the setpoint if no new data is available
                stateSetpoint_(3) += stateSetpoint_(0) * dTime;
                stateSetpoint_(4) += stateSetpoint_(1) * dTime;
                stateSetpoint_(5) += stateSetpoint_(2) * dTime;
            }
            auto& setpointPosition = stateSetpoint_;

            //setpointPosition = {0, 0, 1}; // Setpoint position in reference frame (X, Y, Z)
            //position = {1, 0, 1};
            //velocity = {0, 0, 0};

            //LOG_MSG("Position: %.2f %.2f %.2f\n", setpointPosition(0), setpointPosition(1), setpointPosition(2));

            //################ Calculate the position controller output ################
            Math::Vector<float, 3> posCtrlOutput({
                positionHZTGain_ * (setpointPosition(3) - position(0)),
                positionHZTGain_ * (setpointPosition(4) - position(1)),
                positionVRTGain_ * (setpointPosition(5) - position(2))
            });

            { //Lets keep these vars local to this scope
                auto length = sqrt(posCtrlOutput(0)*posCtrlOutput(0) + posCtrlOutput(1)*posCtrlOutput(1)); //Calculate the length of the horizontal vector
                if (length > positionHZTLimit_ms_) { // If over the limit, scale down the output
                    posCtrlOutput(0) *= positionHZTLimit_ms_ / length;
                    posCtrlOutput(1) *= positionHZTLimit_ms_ / length;
                }
            }

            if (posCtrlOutput(2) > positionVRTLimit_ms_) {
                posCtrlOutput(2) = positionVRTLimit_ms_;
            }

            //LOG_MSG("Position controller output: %.2f %.2f %.2f\n", posCtrlOutput(0), posCtrlOutput(1), posCtrlOutput(2)); // Print the position controller output to the console

            //Add the setpoint velocity control to the position controller output
            posCtrlOutput = posCtrlOutput + setpointPosition.block<3, 1>(0, 0); //Add the setpoint velocity to the position controller output

            //################# Calculate the velocity controller output ################
            Math::Vector<float, 3> velCtrlError({
                (posCtrlOutput(0) - velocity(0)),
                (posCtrlOutput(1) - velocity(1)),
                (posCtrlOutput(2) - velocity(2))
            });
            Math::Vector<float, 3> velCtrlOutput({
                velocityHZTGain_ * velCtrlError(0),
                velocityHZTGain_ * velCtrlError(1),
                velocityVRTGain_ * velCtrlError(2)
            });

            velCtrlIntegral_(0) = velCtrlIntegral_(0) + velCtrlError(0) * velocityHZTIntegral_ * dTime;
            velCtrlIntegral_(1) = velCtrlIntegral_(1) + velCtrlError(1) * velocityHZTIntegral_ * dTime;
            velCtrlIntegral_(2) = velCtrlIntegral_(2) + velCtrlError(2) * velocityVRTIntegral_ * dTime;

            if (!enableControl_) {
                velCtrlIntegral_(0) = 0;
                velCtrlIntegral_(1) = 0;
                velCtrlIntegral_(2) = 0;
            }

            //limit the output of the velocity controller including the integral. If we are over the limit, with the P and I term, then we subtract from the integral term so we reach the limit. But do not go over zero for the integral term.
            if (velCtrlOutput(0) + velCtrlIntegral_(0) > velocityHZTLimit_mss_) {
                velCtrlIntegral_(0) = velocityHZTLimit_mss_ - velCtrlOutput(0);
                if (velCtrlIntegral_(0) < 0) velCtrlIntegral_(0) = 0; //Don't let the integral term go negative
            } else if (velCtrlOutput(0) + velCtrlIntegral_(0) < -velocityHZTLimit_mss_) {
                velCtrlIntegral_(0) = -velocityHZTLimit_mss_ - velCtrlOutput(0);
                if (velCtrlIntegral_(0) > 0) velCtrlIntegral_(0) = 0; 
            }
            if (velCtrlOutput(1) + velCtrlIntegral_(1) > velocityHZTLimit_mss_) {
                velCtrlIntegral_(1) = velocityHZTLimit_mss_ - velCtrlOutput(1);
                if (velCtrlIntegral_(1) < 0) velCtrlIntegral_(1) = 0; //Don't let the integral term go negative
            } else if (velCtrlOutput(1) + velCtrlIntegral_(1) < -velocityHZTLimit_mss_) {
                velCtrlIntegral_(1) = -velocityHZTLimit_mss_ - velCtrlOutput(1);
                if (velCtrlIntegral_(1) > 0) velCtrlIntegral_(1) = 0; 
            }
            if (velCtrlOutput(2) + velCtrlIntegral_(2) > velocityVRTLimit_mss_) {
                velCtrlIntegral_(2) = velocityVRTLimit_mss_ - velCtrlOutput(2);
                if (velCtrlIntegral_(2) < 0) velCtrlIntegral_(2) = 0; //Don't let the integral term go negative
            } else if (velCtrlOutput(2) + velCtrlIntegral_(2) < -velocityVRTLimit_mss_) {
                velCtrlIntegral_(2) = -velocityVRTLimit_mss_ - velCtrlOutput(2);
                if (velCtrlIntegral_(2) > 0) velCtrlIntegral_(2) = 0; 
            }

            if (velCtrlIntegral_(0) > velocityIntegralLimit_ms_) {
                velCtrlIntegral_(0) = velocityIntegralLimit_ms_;
            } else if (velCtrlIntegral_(0) < -velocityIntegralLimit_ms_) {
                velCtrlIntegral_(0) = -velocityIntegralLimit_ms_;
            }
            if (velCtrlIntegral_(1) > velocityIntegralLimit_ms_) {
                velCtrlIntegral_(1) = velocityIntegralLimit_ms_;
            } else if (velCtrlIntegral_(1) < -velocityIntegralLimit_ms_) {
                velCtrlIntegral_(1) = -velocityIntegralLimit_ms_;
            }
            if (velCtrlIntegral_(2) > velocityIntegralLimit_ms_) {
                velCtrlIntegral_(2) = velocityIntegralLimit_ms_;
            } else if (velCtrlIntegral_(2) < -velocityIntegralLimit_ms_) {
                velCtrlIntegral_(2) = -velocityIntegralLimit_ms_;
            }

            //LOG_MSG("Integral: %.2f %.2f %.2f, dtime: %.5f\n", velCtrlIntegral_(0), velCtrlIntegral_(1), velCtrlIntegral_(2), dTime); // Print the velocity controller output to the console

            //Add the gravity acceleration to be compensated to the velocity controller output
            velCtrlOutput(2) += Math::GRAVITY;
            velCtrlOutput = velCtrlOutput + velCtrlIntegral_; //Add the integral to the velocity controller output
            velCtrlOutput = velCtrlOutput * vehicleMass_kg_;

            //LOG_MSG("Velocity controller output: %.2f %.2f %.2f\n", velCtrlOutput(0), velCtrlOutput(1), velCtrlOutput(2)); // Print the velocity controller output to the console

            //Now we want to calculate the amount we must tilt the rocket and in what yaw (azimuth) direction to achieve the wanted force.
            auto tilt = velCtrlOutput.getAngleTo(Math::Vector<float, 3>({0, 0, 1}));
            //Now we limit the tilt angle to the maximum tilt angle
            auto tiltOffAngle = 0;
            if (tilt > velocityLimit_Rad_) {
                tiltOffAngle = tilt - velocityLimit_Rad_;
                tilt = velocityLimit_Rad_;
            }
            auto wantedForceMagnitude = velCtrlOutput.magnitude();

            //The force needed to be applied to the rocket in body frame.
            auto wantedBodyForce = attitude.conjugate().rotate(velCtrlOutput);
            //LOG_MSG("Wanted force magnitude: %.2f\n", wantedForceMagnitude);

            //auto wantedForce = velCtrlOutput.magnitude() * vehicleMass_kg_;
            auto cosLosses = cos(tiltOffAngle); //Cosine losses due to the wanted tilt angle possibly not being reached.
            if (cosLosses < 0) cosLosses = 0; //If the tilt angle is over 90 degrees, we don't want to apply any force, othewise we technically would need a negative force.
            wantedForceMagnitude = wantedForceMagnitude / cosLosses; //Apply the cosine losses to the wanted force.

            //Now we calculate the wanted attitude to achieve the wanted acceleration vector direction.
            Math::Quat_F wantedAttitude;
            auto rotAxis = Math::Vector<float, 3>({0, 0, 1}).cross(velCtrlOutput).normalize(); //Calculate the rotation axis from the wanted force vector to the Z-axis
            auto rotAngle = Math::Vector<float, 3>({0, 0, 1}).getAngleTo(velCtrlOutput); //Calculate the rotation angle from the wanted force vector to the Z-axis

            //LOG_MSG("Rot ang: %.2f, axis: %.2f %.2f %.2f\n", rotAngle * 180/3.14, rotAxis(0), rotAxis(1), rotAxis(2)); // Print the rotation axis to the console

            if (rotAngle > velocityLimit_Rad_)
                rotAngle = velocityLimit_Rad_;
            else if (rotAngle < -velocityLimit_Rad_)
                rotAngle = -velocityLimit_Rad_;

            wantedAttitude = Math::Quat_F(rotAxis, rotAngle);

            //LOG_MSG("Attitude: %.2f %.2f %.2f %.2f\n", wantedAttitude(0), wantedAttitude(1), wantedAttitude(2), wantedAttitude(3));
            //FOR TESTING!!!! FORCES UPRIGHT POSITION
            //wantedAttitude = Math::Quat_F(1, 0, 0, 0);
            

            //################### Calculate the attitude controller output ################
            //We use a quaternion based algorithm to calculate the rotation error between the wanted attitude and the current attitude. The result is in body frame.
            Math::Vector<float, 3> attCtrlOutput;
            {

                /*auto quatOut = wantedAttitude.normalize() * attitude.conjugate();
                attCtrlOutput = {quatOut(1), quatOut(2), quatOut(3)}; // Get the quaternion output
                attCtrlOutput = attitude.rotate(attCtrlOutput); // Rotate the quaternion output to body frame
                
                attCtrlOutput(0) = -asin(quatOut(2)) * attitudeGain_; // Map to vector and linearize output using asin
                attCtrlOutput(1) = asin(quatOut(1)) * attitudeGain_;
                attCtrlOutput(2) = asin(quatOut(3)) * attitudeZGain_;*/

                auto quatOut = wantedAttitude * attitude.conjugate(); //Calculate the quaternion rotation error
                
                attCtrlOutput(0) = asin(quatOut(1)) * attitudeGain_; // Map to vector and linearize output using asin
                attCtrlOutput(1) = asin(quatOut(2)) * attitudeGain_;
                attCtrlOutput(2) = asin(quatOut(3)) * attitudeZGain_;

                if (quatOut(0) < 0) attCtrlOutput = -attCtrlOutput; //Make sure the quaternion is in the right direction

                //attCtrlOutput = attCtrlOutput * attitudeGain_;

            }

            //attCtrlOutput = Math::Vector<float, 3>();

            
            //################### Calculate the attitude rate controller output ################
            Math::Vector<float, 3> attRateCtrlOutput({
                -(angularVelocity(0)) * attitudeRateGain_,
                -(angularVelocity(1)) * attitudeRateGain_,
                -(angularVelocity(2)) * attitudeRateZGain_,
            });

            attCtrlOutput = attCtrlOutput + attRateCtrlOutput; //Add the attitude rate controller output to the attitude controller output.
 

            //################### Calculate the TVC output ################
            //Current implementation does not take care of limits
            auto forceVector = attCtrlOutput.cross(Math::Vector<float, 3>({0, 0, 1/tvcCGOffset_m_}));
            //auto bodyZAxis = attitude.rotate(Math::Vector<float, 3>({0, 0, 1})); //Get the Z-axis in body frame
            forceVector = forceVector + wantedBodyForce.getProjectionOn(Math::Vector<float, 3>({0, 0, 1}));

            //LOG_MSG("Force vector: %.2f %.2f %.2f |%.2f|\n", forceVector(0), forceVector(1), forceVector(2), forceVector.magnitude()); // Print the force vector to the console

            //################### publish the TVC output ################
            Math::Vector<float, 4> tvcOutput = Math::Vector<float, 4>({
                forceVector(0), forceVector(1), forceVector(2), attCtrlOutput(2)
            });

            if (!enableControl_) {
                tvcOutput = Math::Vector<float, 4>({0, 0, 0.1, 0}); //If the control is disabled, we send a tiny force in z axis to keep actuators alive, but pointing straight up.
            }

            tvcTopic_.publish(tvcOutput); //Publish the TVC output


        }

    }
}

