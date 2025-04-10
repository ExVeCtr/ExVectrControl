#ifndef EXVECTRCONTROL_CONTROLROCKET_HPP
#define EXVECTRCONTROL_CONTROLROCKET_HPP


#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/timestamped.hpp"

#include "ExVectrMath.hpp"

#include "ExVectrDSP/value_covariance.hpp"


namespace VCTR
{
    namespace CTRL
    {

        /**
         * @brief Simple one dimensional PID controller.
         */
        class ControlRocket
        {
        private:

            Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 7>>> attSubr_;
            Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 6>>> posSubr_;

            Core::Simple_Subscriber<Math::Vector<float, 6>> stateSetpoint_;


        

        public:
            
            //SimplePID();

            /**
             * * @brief Subsribes to a topic to which the attitude estimation is published in form: [W, Q], where W is the angular velocity vector and Q is a unit quaternion rotation from the reference frame to body frame.
             */
            void subscribeAttitudeMeasurement(Core::Topic<Core::Timestamped<DSP::ValueCov<float, 7>>> &attTopic)
            {
                attSubr_.subscribe(attTopic);
            }

            /**
             * @brief Subsribes to a topic to which the position estimation is published in form: [V, P], where V is the linear velocity vector and P is the position vector.
             */
            void subscribePositionMeasurement(Core::Topic<Core::Timestamped<DSP::ValueCov<float, 6>>> &posTopic) 
            {
                posSubr_.subscribe(posTopic);
            }



            
        };

    }
}

#endif // EXVECTRCONTROL_SIMPLE_PID_HPP_