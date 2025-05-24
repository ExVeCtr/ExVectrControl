#ifndef EXVECTRCONTROL_CONTROLROCKET_HPP
#define EXVECTRCONTROL_CONTROLROCKET_HPP

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
         * @brief Simple one dimensional PID controller.
         */
        class ControlRocket : public Core::Task_Periodic
        {
        private:

            Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 7>>> attSubr_;
            Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 6>>> posSubr_;

            // Setpoint for the position. In form: [V, P], where V is the linear velocity vector and P is the position vector.
            Core::Simple_Subscriber<Math::Vector<float, 6>> stateSetpointSubr_;

            //This is where the tvc control output is published. In form: [X, Y, Z, T], where X, Y, Z show the thrust vector in body frame (magnitude of vector is thrust magnitude) and T is the roll torque (Z-Axis) angle in radians.
            Core::Topic<Math::Vector<float, 4>> tvcTopic_;


            // Current vehicle state estimation. In form: [V, P], where V is the linear velocity vector and P is the position vector.
            Core::Timestamped<Math::Vector<float, 6>> stateEstimation_;
            // Current attitude estimation. In form: [W, Q], where W is the angular velocity vector and Q is a unit quaternion rotation from the reference frame to body frame.
            Core::Timestamped<Math::Vector<float, 7>> attitudeEstimation_;

            Math::Vector<float, 3> velCtrlIntegral_; // Last position estimation in reference frame (X-North, Y-West, Z-Up).

            // last control output torque
            Math::Vector<float, 3> lastControlOutputTorque_;
            // last control output force
            float lastControlOutputForce_;


            // Control parameters
            float vehicleMass_kg_ = 1; // Mass of the vehicle in kg.
            float tvcThrustLimit_N_ = 20; // Maximum thrust in Newtons.
            float tvcAngleLimit_Rad_ = 1; // Maximum angle in radians.
            float tvcCGOffset_m_ = -0.35; // Center of gravity offset in meters. This is the distance from the CG of the vehicle to the center of the thrust vector control system.

            float positionHZTGain_ = 1; // Position control gain for the horizontal axis. Maps position space to velocity space.
            float positionVRTGain_ = 2; // Position control gain for the vertical axis. Maps position space to velocity space.
            float positionHZTLimit_ms_ = 2; // Position limit in m/s for the horizontal axis. Limits the maximum velocity of the vehicle for correcting position.
            float positionVRTLimit_ms_ = 10; // Position limit in m/s for the vertical axis. Limits the maximum velocity of the vehicle for correcting position.

            float velocityHZTGain_ = 2; // Velocity control gain for the horizontal axis. Maps velocity space to acceleration space.
            float velocityVRTGain_ = 3; // Velocity control gain for the vertical axis. Maps velocity space to acceleration space.
            float velocityHZTIntegral_ = 0.4;
            float velocityVRTIntegral_ = 0.8; // Velocity integral gain for the horizontal and vertical axis. Maps velocity space to acceleration space.
            float velocityIntegralLimit_ms_ = 3; // Velocity integral limit in m/s/s. Limits the maximum velocity of the vehicle for correcting position.
            float velocityHZTLimit_mss_ = 2; // Velocity controller output limit in m/s/s for the horizontal axis. Limits the maximum acceleration of the vehicle for correcting velocity.
            float velocityVRTLimit_mss_ = 8; // Velocity controller output limit in m/s/s for the vertical axis. Limits the maximum acceleration of the vehicle for correcting velocity.
            float velocityLimit_Rad_ = 35 * 3.14/180; // Velocity tilt limit in radians. Limits the maximum tilt angle from the Z-Axis of the vehicle for correcting velocity.

            float attitudeGain_ = 1.6; // Attitude control gain.
            float attitudeZGain_ = 0.6; // Attitude control gain.
            float attitudeRateGain_ = 0.18; // Attitude rate control gain.
            float attitudeRateZGain_ = 0.08; // Attitude rate control gain.


            // Runtime data
            int64_t lastRunTimestamp_ = 0; // Timestamp of the last run in microseconds.
            Math::Vector<float, 6> stateSetpoint_; // Setpoint for the position. In form: [V, P], where V is the linear velocity vector and P is the position vector.

            bool enableControl_ = false; // Enable or disable the control system.

            float tvcThrustMaxEstimated_ = 0; // Maximum thrust estimated by the control system. This is used to limit the maximum thrust of the vehicle.


        public:
            
            /**
             * @brief Constructor for the ControlRocket class.
             * @par_kg Mass of the vehicle in kg.
             * @param tvcThrustLimit_N Maximum thrust in Newtons.
             * @param tvcAngleLimit_Rad Maximum angle in radians.
             */
            ControlRocket(float vehicleMass_kg = 1.0f, float tvcThrustLimit_N = 20.0f, float tvcAngleLimit_Rad = 15*3.14/180);

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
            /// @brief The limit of the velocity controller output in the horizontal axis. (Maximum tilt angle in radians)
            void setVelocityLimit(float limit) { velocityLimit_Rad_ = limit; }

            /// @brief The P-Term gain for the attitude controller.
            void setAttitudeGain(float gain) { attitudeGain_ = gain; }
            /// @brief The P-Term gain for the attitude controller in the Z-Axis.
            void setAttitudeZGain(float gain) { attitudeZGain_ = gain; }
            /// @brief The P-Term gain for the attitude rate controller.
            void setAttitudeRateGain(float gain) { attitudeRateGain_ = gain; }
            /// @brief The P-Term gain for the attitude rate controller in the Z-Axis.
            void setAttitudeRateZGain(float gain) { attitudeRateZGain_ = gain; }

            /**
             * * @brief Subsribes to a topic to which the attitude estimation is published in form: [W, Q], where W is the angular velocity vector and Q is a unit quaternion rotation from the reference frame to body frame.
             */
            void subscribeAttitudeMeasurement(Core::Topic<Core::Timestamped<Math::Vector<float, 7>>> &attTopic);

            /**am vehicleMass
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
            Core::Topic<Math::Vector<float, 4>> &getTvcTopic();

            float getTvcThrustMaxEstimated() { return tvcThrustMaxEstimated_; }


            void enableControl(bool enable) { enableControl_ = enable; }
            
            
            void taskCheck() override;

            void taskInit() override;

            void taskThread() override;

            
        };

    }
}

#endif // EXVECTRCONTROL_SIMPLE_PID_HPP_