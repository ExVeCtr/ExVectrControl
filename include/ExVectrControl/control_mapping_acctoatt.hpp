#ifndef EXVECTRCONTROL_CONTROLMAPPING_POSTODRONE_HPP
#define EXVECTRCONTROL_CONTROLMAPPING_POSTODRONE_HPP

#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/timestamped.hpp"

#include "ExVectrMath.hpp"

#include "ExVectrDSP/value_covariance.hpp"


namespace VCTR
{
    namespace CTRL
    {

        /**
         * @brief This takes an acceleration setpoint and produces a wanted attitude. Used by underactuated vehicles like drones and tail sitters.
         * @note The simply produces the wanted atittude, the accel vector must still be given to the following control to get the total force vector.
         */
        class ControlMappingAccToAtt
        {
        private:

            // Setpoint output for the attitude. In form: [V, P], where V is the linear velocity vector and P is the position vector.
            Core::Topic<Math::Vector<float, 7>> attitudeTopic_;

            // Callback subr that takes the accel setpoint and called the callback function to produce the wanted attitude and thrust vector.
            Core::Callback_Subscriber<Math::Vector<float, 3>, ControlMappingAccToAtt> accelSubr_;


            // Control parameters
            float velocityLimit_Rad_ = 35 * DEGREES; // Velocity tilt limit in radians. Limits the maximum tilt angle from the Z-Axis of the vehicle for correcting velocity.


            // Runtime data
            int64_t lastRunTimestamp_ = 0; // Timestamp of the last run in microseconds.


        public:
            
            /**
             * @brief Constructor for the ControlRocket class.
             */
            ControlMappingAccToAtt();

            /**
             * @brief Subsribes to a topic to which the position estimation is published in form: [V, P], where V is the linear velocity vector and P is the position vector.
             */
            void subscribeAccelSetpoint(Core::Topic<Math::Vector<float, 3>> &posTopic);

            /**
             * @brief Returns the topic to which the the wanted attitude is published.
             */
            Core::Topic<Math::Vector<float, 7>> &getAttitudeTopic();
            
        
        private:
            
            void callbackMapper(const Math::Vector<float, 3> &accelSetpoint);

            
        };

    }
}

#endif // EXVECTRCONTROL_SIMPLE_PID_HPP_