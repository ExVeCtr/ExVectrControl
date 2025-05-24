#include "ExVectrCore/print.hpp"

#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrMath/matrix_vector.hpp"
#include "ExVectrMath/matrix_quaternion.hpp"
#include "ExVectrMath/constants.hpp"

#include "ExVectrDSP/value_covariance.hpp"

#include "ExVectrControl/starship/control_attitude_flaps.hpp"


namespace VCTR
{
    namespace CTRL
    {

        
        ControlAttitudeFlaps::ControlAttitudeFlaps() :
            Core::Task_Periodic("Control Rocket", 20*Core::MILLISECONDS)
        {

            // attach to scheduler
            Core::getSystemScheduler().addTask(*this);

        }

        /**
         * * @brief Subsribes to a topic to which the attitude estimation is published in form: [W, Q], where W is the angular velocity vector and Q is a unit quaternion rotation from the reference frame to body frame.
         */
        void ControlAttitudeFlaps::subscribeAttitudeMeasurement(Core::Topic<Core::Timestamped<Math::Vector<float, 7>>> &attTopic)
        {
            attMeasSubr_.subscribe(attTopic);
        }

        void ControlAttitudeFlaps::subscribePositionMeasurement(Core::Topic<Core::Timestamped<Math::Vector<float, 6>>> &posTopic) {
            posMeasSubr_.subscribe(posTopic);
        }

        void ControlAttitudeFlaps::subscribeFlapSettingOutputTopic(Core::Topic<ControlAttitudeFlapSetting> &flapOutputTopic){
            flapOutputTopic_.subscribe(flapOutputTopic);
        }

        void ControlAttitudeFlaps::taskCheck() {

        }

        void ControlAttitudeFlaps::taskInit() {

            //Only run when all subscribers have gotten data at least once
            if (!attMeasSubr_.isDataNew()/* || !stateSetpointSubr_.isDataNew()*/) {
                setInitialised(false);
            }

            attitudeEstimation_.data = {0, 0, 0, 1, 0, 0, 0};
            
        }

        void ControlAttitudeFlaps::taskThread() {

            //if (stateSetpointSubr_.isDataNew()) {
            //    stateSetpoint_ = stateSetpointSubr_.getItem();
            //}
            if (posMeasSubr_.isDataNew()) {
                positionEstimation_ = posMeasSubr_.getItem();
            }
            if (attMeasSubr_.isDataNew()) {
                attitudeEstimation_ = attMeasSubr_.getItem();
            }

            //Retrieve the latest data from the state vectors
            //Math::Vector_F position = positionEstimation_.data.block<3, 1>(3, 0);
            Math::Vector_F velocity = positionEstimation_.data.block<3, 1>(0, 0);
            Math::Quat_F attitude = attitudeEstimation_.data.block<4, 1>(3, 0);
            auto angularVelocity = attitudeEstimation_.data.block<3, 1>(0, 0);

            // Determin of the control should be enabled or not
            //We want to only enalbe the flaps if the velocity in the x direction is above a certain threshold and ZY is below a certain threshold
            auto bodyXAxis = attitude.conjugate().rotate(Math::Vector_F({1, 0, 0})); // Get the body X-axis in reference frame
            auto velXAxis = velocity.getProjectionOn(bodyXAxis); // Project the velocity onto the body X-axis plane
            auto bellyDownAngle = bodyXAxis.getAngleTo(Math::Vector_F({0, 0, 1})); // Calculate the angle between the body X-axis and the Z-axis
            // Scale the flap factor so if over threshold, then begin reducing factor until 10 deg over the threshold, then disable flaps be setting factor 0
            float flapFactor = (bellyDownAngle - enableThresAngle_Rad_)/(10*DEGREES); // Calculate the flap factor based on the belly down angle
            if (flapFactor < 0) flapFactor = 0; // If the belly down angle is below the threshold, set the flap factor to 0
            if (flapFactor > 1) flapFactor = 1; // If the belly down angle is above the threshold, set the flap factor to 1
            flapFactor = 1 - flapFactor;
            //LOG_MSG("Belly down angle: %.2f, thres: %.2f, flap factor: %.2f\n", bellyDownAngle / DEGREES, enableThresAngle_Rad_ / DEGREES, flapFactor); // Print the belly down angle and flap factor to the console

            // Calculate the controller output
            Math::Vector<float, 3> attRateCtrlOutput({
                -(angularVelocity(0)) * attitudeRateXGain_,
                -(angularVelocity(1)) * attitudeRateYGain_,
                -(angularVelocity(2)) * attitudeRateZGain_,
            });


            //Output to flap mapping
            ControlAttitudeFlapSetting flapSetting;
            flapSetting.flapTLAngle_Rad = flapTopNeutralAngle_Rad_ - attRateCtrlOutput(0) - attRateCtrlOutput(1) + attRateCtrlOutput(2);
            flapSetting.flapTRAngle_Rad = flapTopNeutralAngle_Rad_ + attRateCtrlOutput(0) - attRateCtrlOutput(1) - attRateCtrlOutput(2);
            flapSetting.flapBLAngle_Rad = flapBottomNeutralAngle_Rad_ + attRateCtrlOutput(0) + attRateCtrlOutput(1) + attRateCtrlOutput(2);
            flapSetting.flapBRAngle_Rad = flapBottomNeutralAngle_Rad_ - attRateCtrlOutput(0) + attRateCtrlOutput(1) - attRateCtrlOutput(2);
            flapSetting.enableFlaps = true;//enableControl_; // Enable or disable the flap actuators

            // LERP the flap angles based on the flap factor
            flapSetting.flapTLAngle_Rad = flapSetting.flapTLAngle_Rad * flapFactor + 90*DEGREES * (1 - flapFactor); // Scale the flap angle based on the flap factor
            flapSetting.flapTRAngle_Rad = flapSetting.flapTRAngle_Rad * flapFactor + 90*DEGREES * (1 - flapFactor); // Scale the flap angle based on the flap factor
            flapSetting.flapBLAngle_Rad = flapSetting.flapBLAngle_Rad * flapFactor + 90*DEGREES * (1 - flapFactor); // Scale the flap angle based on the flap factor
            flapSetting.flapBRAngle_Rad = flapSetting.flapBRAngle_Rad * flapFactor + 90*DEGREES * (1 - flapFactor); // Scale the flap angle based on the flap factor

            //Publish the flap setting to the topic
            flapOutputTopic_.publish(flapSetting);

        }

    }
}

