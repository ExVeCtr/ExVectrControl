#ifndef EXVECTRCONTROL_CONTROLATTITUDEFLAPS_HPP
#define EXVECTRCONTROL_CONTROLATTITUDEFLAPS_HPP

#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrMath.hpp"

#include "ExVectrDSP/value_covariance.hpp"


namespace VCTR
{
    namespace CTRL
    {   

        struct ControlAttitudeFlapSetting
        {
            float flapTLAngle_Rad = 0; // Flap angle in radians.
            float flapTRAngle_Rad = 0; // Flap angle in radians.
            float flapBLAngle_Rad = 0; // Flap angle in radians.
            float flapBRAngle_Rad = 0; // Flap angle in radians.

            bool enableFlaps = false; // Enable or disable the flap actuators
        };

        /**
         * @brief Simple one dimensional PID controller.
         */
        class ControlAttitudeFlaps : public Core::Task_Periodic
        {
        private:

            Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 7>>> attSubr_;

            // Setpoint for the position. In form: [V, P], where V is the linear velocity vector and P is the position vector.
            Core::Simple_Subscriber<Math::Vector<float, 7>> stateSetpointSubr_;
            Math::Vector<float, 7> stateSetpoint_; // Setpoint for the position. In form: [V, P], where V is the linear velocity vector and P is the position vector.
            // Setpoint for the acceleration in reference frame.

            // Current attitude estimation. In form: [W, Q], where W is the angular velocity vector and Q is a unit quaternion rotation from the reference frame to body frame.
            Core::Timestamped<Math::Vector<float, 7>> attitudeEstimation_;

            

            // Control parameters
            //float attitudeXGain_ = 1; // Attitude control gain.
            //float attitudeYGain_ = 1; // Attitude control gain.
            //float attitudeZGain_ = 1; // Attitude control gain.
            float attitudeRateXGain_ = 1; // Attitude rate control gain.
            float attitudeRateYGain_ = 1; // Attitude rate control gain.
            float attitudeRateZGain_ = 1; // Attitude rate control gain.

            bool enableControl_ = false; // Enable or disable the control system.


        public:
            
            /**
             * @brief Constructor for the ControlRocket class.
             */
            ControlAttitudeFlaps();

            /// @brief The P-Term gain for the attitude rate controller.
            void setAttitudeRateXGain(float gain) { attitudeRateXGain_ = gain; }
            /// @brief The P-Term gain for the attitude rate controller.
            void setAttitudeRateYGain(float gain) { attitudeRateYGain_ = gain; }
            /// @brief The P-Term gain for the attitude rate controller.
            void setAttitudeRateZGain(float gain) { attitudeRateZGain_ = gain; }

            /**
             * * @brief Subsribes to a topic to which the attitude estimation is published in form: [W, Q], where W is the angular velocity vector and Q is a unit quaternion rotation from the reference frame to body frame.
             */
            void subscribeAttitudeMeasurement(Core::Topic<Core::Timestamped<Math::Vector<float, 7>>> &attTopic);

            /**
             * @brief Subsribes to a topic to which the setpoint is published in form: [V, P], where V is the linear velocity vector and P is the position vector.
             */
            //void subscribeAttitudeSetpoint(Core::Topic<Math::Vector<float, 7>> &setpointTopic);

            /**
             * @brief Uses the given topic to publish the flap settings for this controllers output.
             * 
             */
            void subscribeFlapSettingOutputTopic(Core::Topic<ControlAttitudeFlapSetting> &flapOutputTopic);


            void enableControl(bool enable) { enableControl_ = enable; }

        private:

            void taskCheck() override;

            void taskInit() override;

            void taskThread() override;

            
        };

    }
}

#endif