#ifndef EXVECTRCONTROL_CONTROLATTITUDETVC_HPP
#define EXVECTRCONTROL_CONTROLATTITUDETVC_HPP

#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrMath.hpp"

#include "ExVectrDSP/value_covariance.hpp"


namespace VCTR
{
    namespace CTRL
    {

        /**
         * @brief Simple one dimensional PID controller.
         */
        class ControlAttitudeTvc : public Core::Task_Periodic
        {
        private:

            Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 7>>> attSubr_;

            // Setpoint for the position. In form: [V, P], where V is the linear velocity vector and P is the position vector.
            Core::Simple_Subscriber<Math::Vector<float, 7>> stateSetpointSubr_;
            Math::Vector<float, 7> stateSetpoint_; // Setpoint for the position. In form: [V, P], where V is the linear velocity vector and P is the position vector.
            // Setpoint for the acceleration in reference frame.
            Core::Simple_Subscriber<Math::Vector<float, 3>> accelSetpointSubr_;
            Math::Vector<float, 3> accelSetpoint_; // Setpoint for the acceleration in reference frame.

            //This is where the tvc control output is published. In form: [X, Y, Z, T], where X, Y, Z show the thrust vector in body frame (magnitude of vector is thrust magnitude) and T is the roll torque (Z-Axis) angle in radians.
            Core::Topic<Math::Vector<float, 4>> tvcTopic_;

            // Current attitude estimation. In form: [W, Q], where W is the angular velocity vector and Q is a unit quaternion rotation from the reference frame to body frame.
            Core::Timestamped<Math::Vector<float, 7>> attitudeEstimation_;

            // last control output torque
            Math::Vector<float, 3> lastControlOutputTorque_;
            // last control output force
            float lastControlOutputForce_;


            // Control parameters   
            float vehicleMass_kg_ = 1; // Mass of the vehicle in kg.
            float tvcThrustLimit_N_ = 20; // Maximum thrust in Newtons.
            float tvcAngleLimit_Rad_ = 1; // Maximum angle in radians.
            float tvcCGOffset_m_ = -0.35; // Center of gravity offset in meters. This is the distance from the CG of the vehicle to the center of the thrust vector control system.

            float tiltLimit_Rad_ = 35 * DEGREES; // Tilt limit in radians. Limits the maximum tilt angle from the Z-Axis of the vehicle for correcting velocity.

            float attitudeGain_ = 2; // Attitude control gain.
            float attitudeZGain_ = 0.8; // Attitude control gain.
            float attitudeRateGain_ = 0.2; // Attitude rate control gain.
            float attitudeRateZGain_ = 0.1; // Attitude rate control gain.

            bool compensateTVCAngle_ = false; //If true, then if a TVC angle greater than the limit is needed, then the TVC thrust is increased to achieve the desired torque.


            // Runtime data
            int64_t lastRunTimestamp_ = 0; // Timestamp of the last run in microseconds.

            bool enableControl_ = false; // Enable or disable the control system.


        public:
            
            /**
             * @brief Constructor for the ControlRocket class.
             * @param vehicleMass_kg Mass of the vehicle in kg.
             * @param tvcThrustLimit_N Maximum thrust in Newtons.
             * @param tvcAngleLimit_Rad Maximum angle in radians.
             */
            ControlAttitudeTvc(float vehicleMass_kg = 1.0f, float tvcThrustLimit_N = 20.0f, float tvcAngleLimit_Rad = 15*3.14/180);

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
            
            /**
             * @brief Subsribes to a topic to which the acceleration setpoint is published in reference frame.
             */
            void subscribeAccelerationSetpoint(Core::Topic<Math::Vector<float, 3>> &setpointTopic);

            /**
             * @brief Subsribes to a topic to which the setpoint is published in form: [V, P], where V is the linear velocity vector and P is the position vector.
             */
            void subscribeAttitudeSetpoint(Core::Topic<Math::Vector<float, 7>> &setpointTopic);

            /**
             * @brief Returns the topic to which the thrust vector control output is published. In form: [X, Y, Z, T], where X, Y, Z show the thrust vector in body frame (magnitude of vector is thrust magnitude) and T is the roll torque (Z-Axis) angle in radians.
             */
            Core::Topic<Math::Vector<float, 4>> &getTvcTopic();


            void enableControl(bool enable) { enableControl_ = enable; }

            /**
             * @brief if a TVC angle greater than the limit is needed, then the TVC thrust is increased to achieve the desired torque.
             * @note this can cause runwaway if the required TVC angle is constantly greater than the limit.
             */
            void setTVCLimitCompensation(bool enable) { compensateTVCAngle_ = enable; }
            
            void taskCheck() override;

            void taskInit() override;

            void taskThread() override;

            
        };

    }
}

#endif // EXVECTRCONTROL_SIMPLE_PID_HPP_