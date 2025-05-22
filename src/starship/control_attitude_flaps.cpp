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
            attSubr_.subscribe(attTopic);
        }

        void ControlAttitudeFlaps::taskCheck() {

        }

        void ControlAttitudeFlaps::taskInit() {

            //Only run when all subscribers have gotten data at least once
            if (!attSubr_.isDataNew() || !stateSetpointSubr_.isDataNew()) {
                setInitialised(false);
            }

            

        }

        void ControlAttitudeFlaps::taskThread() {

            

        }

    }
}

