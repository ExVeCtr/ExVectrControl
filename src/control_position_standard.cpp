#include "ExVectrCore/print.hpp"

#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrMath/matrix_vector.hpp"
#include "ExVectrMath/matrix_quaternion.hpp"
#include "ExVectrMath/constants.hpp"

#include "ExVectrDSP/value_covariance.hpp"

#include "ExVectrControl/control_position_standard.hpp"


namespace VCTR
{
    namespace CTRL
    {

        
        ControlPositionStandard::ControlPositionStandard() :
            Core::Task_Periodic("Control Rocket", 50*Core::MILLISECONDS)
        {

            // attach to scheduler
            Core::getSystemScheduler().addTask(*this);

        }


        /**
         * @brief Subsribes to a topic to which the position estimation is published in form: [V, P], where V is the linear velocity vector and P is the position vector.
         */
        void ControlPositionStandard::subscribePositionMeasurement(Core::Topic<Core::Timestamped<Math::Vector<float, 6>>> &posTopic) 
        {
            posSubr_.subscribe(posTopic);
        }

        /**
         * @brief Subsribes to a topic to which the setpoint is published in form: [V, P], where V is the linear velocity vector and P is the position vector.
         */
        void ControlPositionStandard::subscribeSetpoint(Core::Topic<Math::Vector<float, 6>> &setpointTopic)
        {
            stateSetpointSubr_.subscribe(setpointTopic);
        }

        /**
         * @brief Returns the topic to which the thrust vector control output is published. In form: [X, Y, Z, T], where X, Y, Z show the thrust vector in body frame (magnitude of vector is thrust magnitude) and T is the roll torque (Z-Axis) angle in radians.
         */
        Core::Topic<Math::Vector<float, 3>> &ControlPositionStandard::getAccelTopic()
        {
            return accelTopic_;
        }

        void ControlPositionStandard::taskCheck() {

        }

        void ControlPositionStandard::taskInit() {

            //Only run when all subscribers have gotten data at least once
            if (!posSubr_.isDataNew() || !stateSetpointSubr_.isDataNew()) {
                setInitialised(false);
            }

            lastRunTimestamp_ = Core::NOW();

        }

        void ControlPositionStandard::taskThread() {

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

            //

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
            
            accelTopic_.publish(velCtrlOutput); //Publish the velocity controller output to the topic


        }

    }
}

