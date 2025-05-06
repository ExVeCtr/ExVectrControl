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
                Math::Vector<float, 3> vel = stateSetpoint_.block<3, 1>(0, 0); //Get the wanted velocity from the setpoint
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

            //LOG_MSG("Attitude: %.2f %.2f %.2f %.2f\n", wantedAttitude(0), wantedAttitude(1), wantedAttitude(2), wantedAttitude(3));
            //FOR TESTING!!!! FORCES UPRIGHT POSITION
            //wantedAttitude = Math::Quat_F(1, 0, 0, 0);
            

            //################### Calculate the attitude controller output ################
            //We use a quaternion based algorithm to calculate the rotation error between the wanted attitude and the current attitude. The result is in body frame.
            Math::Vector<float, 3> attCtrlOutput;
            {

                auto quatOut = wantedAttitude * attitude.conjugate(); //Calculate the quaternion rotation error
                
                attCtrlOutput(0) = asin(quatOut(1)) * attitudeGain_;
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
            auto wantedBodyForce = attitude.conjugate().rotate(accelSetpoint_ * vehicleMass_kg_);
            //Current implementation does not take care of limits
            Math::Vector_F forceVector = attCtrlOutput.cross(Math::Vector<float, 3>({0, 0, 1/tvcCGOffset_m_}));
            forceVector = forceVector + wantedBodyForce.getProjectionOn(Math::Vector<float, 3>({0, 0, 1}));

            //LOG_MSG("Force vector: %.2f %.2f %.2f |%.2f|\n", forceVector(0), forceVector(1), forceVector(2), forceVector.magnitude()); // Print the force vector to the console

            //################### publish the TVC output ################
            Math::Vector<float, 4> tvcOutput = Math::Vector<float, 4>({
                forceVector(0), forceVector(1), forceVector(2), attCtrlOutput(2)
            });

            if (!enableControl_) {
                tvcOutput = Math::Vector<float, 4>({0, 0, 0.1, 0}); //FOR TESTING ONLY
            }

            tvcTopic_.publish(tvcOutput); //Publish the TVC output


        }

    }
}

