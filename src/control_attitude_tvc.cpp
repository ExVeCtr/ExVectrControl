#include "ExVectrCore/print.hpp"

#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrMath/matrix_vector.hpp"
#include "ExVectrMath/matrix_quaternion.hpp"
#include "ExVectrMath/constants.hpp"

#include "ExVectrDSP/value_covariance.hpp"

#include "ExVectrControl/control_attitude_tvc.hpp"


namespace VCTR
{
    namespace CTRL
    {

        
        ControlAttitudeTvc::ControlAttitudeTvc(float vehicleMass_kg, float tvcThrustLimit_N, float tvcAngleLimit_Rad) :
            Core::Task_Periodic("Control Rocket", 20*Core::MILLISECONDS)
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
        void ControlAttitudeTvc::subscribeAttitudeMeasurement(Core::Topic<Core::Timestamped<Math::Vector<float, 7>>> &attTopic)
        {
            attSubr_.subscribe(attTopic);
        }

        void ControlAttitudeTvc::subscribeAccelerationSetpoint(Core::Topic<Math::Vector<float, 3>> &setpointTopic)
        {
            accelSetpointSubr_.subscribe(setpointTopic);
        }

        /**
         * @brief Subsribes to a topic to which the setpoint is published in form: [V, P], where V is the linear velocity vector and P is the position vector.
         */
        void ControlAttitudeTvc::subscribeAttitudeSetpoint(Core::Topic<Math::Vector<float, 7>> &setpointTopic)
        {
            stateSetpointSubr_.subscribe(setpointTopic);
        }

        /**
         * @brief Returns the topic to which the thrust vector control output is published. In form: [X, Y, Z, T], where X, Y, Z show the thrust vector in body frame (magnitude of vector is thrust magnitude) and T is the roll torque (Z-Axis) angle in radians.
         */
        Core::Topic<Math::Vector<float, 4>> &ControlAttitudeTvc::getTvcTopic()
        {
            return tvcTopic_;
        }

        void ControlAttitudeTvc::taskCheck() {

        }

        void ControlAttitudeTvc::taskInit() {

            //Only run when all subscribers have gotten data at least once
            if (!attSubr_.isDataNew() || !stateSetpointSubr_.isDataNew()) {
                setInitialised(false);
            }

            lastRunTimestamp_ = Core::NOW();
            accelSetpoint_ = 0;
            integral_ = 0; // Reset the integral term for the attitude control

        }

        void ControlAttitudeTvc::taskThread() {

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
            Math::Quat_F attitude = attSubr_.getItem().data.block<4, 1>(3, 0);
            auto angularVelocity = attSubr_.getItem().data.block<3, 1>(0, 0);

            if (stateSetpointSubr_.isDataNew()) {
                stateSetpoint_ = stateSetpointSubr_.getItem(); // Get the setpoint from the subscriber
            } else {
                //Propagate the setpoint if no new data is available
                Math::Quat<float> att = stateSetpoint_.block<4, 1>(3, 0); //Get the wanted attitude from the setpoint
                //Math::Vector<float, 3> vel = stateSetpoint_.block<3, 1>(0, 0); //Get the wanted velocity from the setpoint
                att = att * Math::Quat<float>(angularVelocity.normalize(), angularVelocity.magnitude() * dTime); //Propagate the attitude quaternion
                stateSetpoint_(3) = att(0);
                stateSetpoint_(4) = att(1);
                stateSetpoint_(5) = att(2);
                stateSetpoint_(6) = att(3);
            }

            if (accelSetpointSubr_.isDataNew()) {
                accelSetpoint_ = accelSetpointSubr_.getItem(); // Get the setpoint from the subscriber
            } 


            Math::Quat<float> wantedAttitude = stateSetpoint_.block<4, 1>(3, 0); //Get the wanted attitude from the setpoint
            Math::Vector_F wantedAngularVelocity = stateSetpoint_.block<3, 1>(0, 0); //Get the wanted angular velocity from the setpoint

            //swantedAttitude = Math::Quat_F(Math::Vector<float, 3>({0, 1, 0}), -0*DEGREES  );
            //accelSetpoint_ = Math::GRAVITY_3F;
            //wantedAngularVelocity = {0, 0, 0}; //For testing purposes, we set the wanted attitude to upright and the wanted angular velocity to zero.

            //LOG_MSG("Attitude: %.2f %.2f %.2f %.2f\n", wantedAttitude(0), wantedAttitude(1), wantedAttitude(2), wantedAttitude(3));


            //#################### Calculate the attitude error ################
            auto quatOut = wantedAttitude * attitude.conjugate() ; //Calculate the quaternion rotation error
            //auto rotVec = quatOut.toRotVec();
            //auto attitudeError = Math::Vector<float, 3>({asin(quatOut(1)), asin(quatOut(2)), asin(quatOut(3))});
            if (quatOut(0) < 0) quatOut = -quatOut; //Make sure the quaternion is in the right direction

            //Created a new attitude from the wanted attitude but without yaw roation in world Z axis.
            //For this we first calculate the wanted attitude and current yaw rotation in world Z axis by projecting the quats to the wz axis and then norming
            //auto wantedYaw = atan2(wantedAttitude(3), wantedAttitude(0)); //Get the yaw angle from the quaternion
            //auto currentYaw = atan2(attitude(3), attitude(0)); //Get the current yaw angle from the quaternion
            //auto wantedAttitudeWithCurrentYaw = Math::Quat_F({0, 0, 1}, currentYaw - wantedYaw) * wantedAttitude; //Set the Z-axis to 0
            auto bodyZ = Math::Vector<float, 3>({0, 0, 1}); //Get the Z-axis in body frame
            auto wantedZInBody = (quatOut).conjugate().rotate(Math::Vector<float, 3>({0, 0, 1})); //Get the Z-axis in body frame for the wanted attitude
            auto bodyZRotAng = bodyZ.getAngleTo(wantedZInBody);
            auto bodyZRotVec = bodyZ.cross(wantedZInBody).normalize() * bodyZRotAng; //Calculate the cross product of the two Z-axes to get the rotation axis
            //auto bodyZAngle = bodyZInWorld.getAngleTo(bodyZInWorldWanted); //Calculate the angle between the two Z-axes
            //bodyZRotVec = attitude.rotate(bodyZRotVec); //Rotate the rotation vector to body frame

            //LOG_MSG("Wanted Z in body: %.2f %.2f %.2f\n", wantedZInBody(0), wantedZInBody(1), wantedZInBody(2));

            if (bodyZRotAng < 20 * DEGREES) {
                auto buf = wantedAttitude * attitude.conjugate();
                auto yawError = atan2(buf(3), buf(0)); // Calculate the yaw error from the quaternion
                bodyZRotVec(2) = yawError; 
            }
            // The rotation error in body frame to rotate the current attitude to the wanted attitude in Rad.
            Math::Vector<float, 3> bodyRotationAngleError = bodyZRotVec;

            //LOG_MSG("Attitude error: %.2f %.2f %.2f\n", bodyRotationAngleError(0), bodyRotationAngleError(1), bodyRotationAngleError(2));
            
            //################### Calculate the attitude controller output ################
            //We use a quaternion based algorithm to calculate the rotation error between the wanted attitude and the current attitude. The result is in body frame.
            Math::Vector<float, 3> attCtrlOutput({
                bodyRotationAngleError(0) * attitudeGain_,
                bodyRotationAngleError(1) * attitudeGain_,
                bodyRotationAngleError(2) * attitudeZGain_
            });

            
            //################### Calculate the integral term ################
            //We use a simple integral term to reduce the steady state error. The integral term is calculated by summing up the attitude error multiplied by the delta time and the integral gain.
            //We must calculate the XY seperate from the Z-axis, because the Z-axis dynamics are different.
            if (enableControl_) {
                integral_(0) += bodyRotationAngleError(0) * dTime * attitudeIntegralGain_;
                integral_(1) += bodyRotationAngleError(1) * dTime * attitudeIntegralGain_;
                integral_(2) += bodyRotationAngleError(2) * dTime * attitudeIntegralZGain_;
            } else {
                integral_ = 0; // Reset the integral term if control is disabled
            }
            //We must limit the integral term to prevent windup. We do this by clamping the integral term to a maximum value.
            if (integral_(0) > integralLimit_) integral_(0) = integralLimit_;
            else if (integral_(0) < -integralLimit_) integral_(0) = -integralLimit_;
            if (integral_(1) > integralLimit_) integral_(1) = integralLimit_;
            else if (integral_(1) < -integralLimit_) integral_(1) = -integralLimit_;
            if (integral_(2) > integralZLimit_) integral_(2) = integralZLimit_;
            else if (integral_(2) < -integralZLimit_) integral_(2) = -integralZLimit_;


            //################### Calculate the attitude rate controller output ################
            Math::Vector<float, 3> attRateCtrlOutput({
                (wantedAngularVelocity(0) - angularVelocity(0)) * attitudeRateGain_,
                (wantedAngularVelocity(1) - angularVelocity(1)) * attitudeRateGain_,
                (wantedAngularVelocity(2) - angularVelocity(2)) * attitudeRateZGain_,
            });

            attCtrlOutput = attCtrlOutput + attRateCtrlOutput + integral_; //Add the attitude rate controller output to the attitude controller output.


            //################### Calculate the TVC output ################
            Math::Vector_F wantedBodyForce = attitude.rotate(accelSetpoint_ * vehicleMass_kg_); //Calculate the wanted body force in body frame. The force is in the direction of the acceleration vector.
            Math::Vector_F torqueVec = attCtrlOutput.cross(Math::Vector<float, 3>({0, 0, 1/tvcCGOffset_m_}));
            //auto bodyZAxis = attitude.rotate(Math::Vector<float, 3>({0, 0, 1})); //Get the Z-axis in body frame
            float bodyForceMagnitude = wantedBodyForce.magnitude();
            float cosLosses = cos(wantedBodyForce.getAngleTo(Math::Vector<float, 3>({0, 0, 1}))); //Cosine losses due to the wanted tilt angle possibly not being reached.
            if (cosLosses < 0) cosLosses = 0; //If the tilt angle is over 90 degrees, we don't want to apply any force, othewise we technically would need a negative force.
            Math::Vector<float, 3> forceVector = torqueVec;
            forceVector(2) += bodyForceMagnitude * cosLosses; //Add the force vector to the Z-axis. The Z-axis is the thrust vector in body frame.
            //forceVector(2) = 15;

            //LOG_MSG("Wanted body force: %.2f %.2f %.2f |%.2f|\n", wantedBodyForce(0), wantedBodyForce(1), wantedBodyForce(2), wantedBodyForce.magnitude());

            // We now must take the TVC angle limit into account. 
            // If the requested vector angle is over the limit, we use the closest possible and then we scale the output until the requested torque is reached.
            auto tvcAngle = forceVector.getAngleTo(Math::Vector<float, 3>({0, 0, 1})); //Get the angle of the force vector to the Z-axis
            auto tvcRotationAxis = (forceVector.cross(Math::Vector<float, 3>({0, 0, 1}))).normalize(); //Rotation axis is the cross product of the vector and the Z-Axis.
            if (tvcAngle > tvcAngleLimit_Rad_) {
                auto tvcRotation = Math::Quat_F(-tvcRotationAxis, tvcAngle - tvcAngleLimit_Rad_);//.conjugate();
                auto forceVectorNew = tvcRotation.rotate(forceVector); //Rotate the vector back to the limit.
                //Now we must scale it so the resulting torque is the same as the requested torque. The torque is the cross product of the force vector and the distance to the CG.
                //auto forceMagnitude = forceVector.magnitude();
                auto xyMag = sqrt(forceVector(0)*forceVector(0) + forceVector(1)*forceVector(1)); //Calculate the XY magnitude of the vector
                auto xyMagNew = sqrt(forceVectorNew(0)*forceVectorNew(0) + forceVectorNew(1)*forceVectorNew(1)); //Calculate the XY magnitude of the new vector
                auto correctionFactor = xyMag / xyMagNew; //Calculate the correction factor to scale the vector back to the limit.
                if (compensateTVCAngle_)
                    forceVectorNew = forceVectorNew * correctionFactor; //Scale the vector back to the limit.
                forceVector = forceVectorNew;
            } 

            //LOG_MSG("Force vector: %.2f %.2f %.2f |%.2f|\n", forceVector(0), forceVector(1), forceVector(2), forceVector.magnitude()); // Print the force vector to the console
            
            //################### publish the TVC output ################
            Math::Vector<float, 4> tvcOutput = Math::Vector<float, 4>({
                forceVector(0), forceVector(1), forceVector(2), attCtrlOutput(2)
            });

            //enableControl_ = true; //Enable the control fro debuggin

            if (!enableControl_) {
                tvcOutput = Math::Vector<float, 4>({0, 0, 0.1, 0});
            }

            tvcTopic_.publish(tvcOutput); //Publish the TVC output

        }

    }
}

