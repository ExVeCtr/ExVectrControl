#ifndef EXVECTRCONTROL_CONTROLPOSITIONSTANDARD_HPP
#define EXVECTRCONTROL_CONTROLPOSITIONSTANDARD_HPP

#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrMath/matrix_vector.hpp"

#include "ExVectrDSP/value_covariance.hpp"


namespace VCTR
{
    namespace CTRL
    {

        /**
         * @brief   A generalized position control system for a 3D vehicle. This take position and velocity setpoints and outputs an acceleration vector in the reference frame.
         *          This output usually is mapped/transformed to get attitude and thrust controls. E,g ControlMappingPosToDrone that is used for drones, tail sitter and other similar underactuated vehicles.
         */
        class ControlPositionStandard : public Core::Task_Periodic
        {
        private:

            Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 6>>> posSubr_;

            // Setpoint for the position. In form: [V, P], where V is the linear velocity vector and P is the position vector.
            Core::Simple_Subscriber<Math::Vector<float, 6>> stateSetpointSubr_;

            //This is where the accel control output is published. In form: [X, Y, Z], where X, Y, Z show the accel vector in reference frame
            Core::Topic<Math::Vector<float, 3>> accelTopic_;


            // Current vehicle state estimation. In form: [V, P], where V is the linear velocity vector and P is the position vector.
            Core::Timestamped<Math::Vector<float, 6>> stateEstimation_;

            Math::Vector<float, 3> velCtrlIntegral_; // Last position estimation in reference frame (X-North, Y-West, Z-Up).


            // Control parameters
            float positionHZTGain_ = 1; // Position control gain for the horizontal axis. Maps position space to velocity space.
            float positionVRTGain_ = 2; // Position control gain for the vertical axis. Maps position space to velocity space.
            float positionHZTLimit_ms_ = 2; // Position limit in m/s for the horizontal axis. Limits the maximum velocity of the vehicle for correcting position.
            float positionVRTLimit_ms_ = 5; // Position limit in m/s for the vertical axis. Limits the maximum velocity of the vehicle for correcting position.

            float velocityHZTGain_ = 2.5; // Velocity control gain for the horizontal axis. Maps velocity space to acceleration space.
            float velocityVRTGain_ = 3; // Velocity control gain for the vertical axis. Maps velocity space to acceleration space.
            float velocityHZTIntegral_ = 0.4;
            float velocityVRTIntegral_ = 0.8; // Velocity integral gain for the horizontal and vertical axis. Maps velocity space to acceleration space.
            float velocityIntegralLimit_ms_ = 5; // Velocity integral limit in m/s/s. Limits the maximum velocity of the vehicle for correcting position.


            // Runtime data
            int64_t lastRunTimestamp_ = 0; // Timestamp of the last run in microseconds.
            Math::Vector<float, 6> stateSetpoint_; // Setpoint for the position. In form: [V, P], where V is the linear velocity vector and P is the position vector.

            bool enableControl_ = false; // Enable or disable the control system.


        public:
            
            /**
             * @brief Constructor for the ControlRocket class.
             */
            ControlPositionStandard();

            /// @brief The P-Term gain for the position controller in the horizontal axis.
            void setPositionHZTGain(float gain) { positionHZTGain_ = gain; }
            /// @brief The P-Term gain for the position controller in the vertical axis.
            void setPositionVRTGain(float gain) { positionVRTGain_ = gain; }
            /// @brief The limit of the position controller output in the horizontal axis. (Maximum velocity in m/s)
            void setPositionHZTLimit(float limit) { positionHZTLimit_ms_ = limit; }
            /// @brief The limit of the position controller output in the vertical axis. (Maximum velocity in m/s)
            void setPositionVRTLimit(float limit) { positionVRTLimit_ms_ = limit; }

            /// @brief The P-Term gain for the velocity controller in the horizontal axis.
            void setVelocityHZTGain(float gain) { velocityHZTGain_ = gain; }
            /// @brief The P-Term gain for the velocity controller in the vertical axis.
            void setVelocityVRTGain(float gain) { velocityVRTGain_ = gain; }

            /**
             * @brief Subsribes to a topic to which the position estimation is published in form: [V, P], where V is the linear velocity vector and P is the position vector.
             */
            void subscribePositionMeasurement(Core::Topic<Core::Timestamped<Math::Vector<float, 6>>> &posTopic);

            /**
             * @brief Subsribes to a topic to which the setpoint is published in form: [V, P], where V is the linear velocity vector and P is the position vector.
             */
            void subscribeSetpoint(Core::Topic<Math::Vector<float, 6>> &setpointTopic);

            /**
             * @brief Returns the topic to which the thrust vector control output is published. In form: [X, Y, Z, T], where X, Y, Z show the thrust vector in body frame (magnitude of vector is thrust magnitude) and T is the roll torque (Z-Axis) angle in radians.
             */
            Core::Topic<Math::Vector<float, 3>> &getAccelTopic();


            void enableControl(bool enable) { enableControl_ = enable; }
            
            
            void taskCheck() override;

            void taskInit() override;

            void taskThread() override;

            
        };

    }
}

#endif // EXVECTRCONTROL_SIMPLE_PID_HPP_