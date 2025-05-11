#include "ExVectrCore/time_definitions.hpp"

#include "ExVectrMath.hpp"

#include "ExVectrControl/simple_pid.hpp"


using namespace VCTR::CTRL;

// Implementation of the SimplePID class.

SimplePID::SimplePID(float kp, float ki, float kd, float maxOutput) 
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;

    maxOutput_ = maxOutput;

    integral_ = 0;
    lastError_ = 0;

    setPoint_ = 0;

    lastTime_ = Core::NOW();   
}

void SimplePID::setSetPoint(float setPoint) {

    setPoint_ = setPoint;

}

float SimplePID::getSetPoint() const {

    return setPoint_;

}

float SimplePID::update(float state, int64_t stateTime) {

    float error = setPoint_ - state;
    float errorDerivative = (error - lastError_) * Core::SECONDS/(stateTime - lastTime_);

    integral_ += error * (stateTime - lastTime_) / Core::SECONDS;

    float output = kp_*error + ki_*integral_ + kd_*errorDerivative;

    lastError_ = error;
    lastTime_ = stateTime;

    //Limit the output and do integral anti windup using back calculation
    if (output > maxOutput_) {
        output = maxOutput_;
        integral_ = 0;
    } else if (output < -maxOutput_) {
        output = -maxOutput_;
        integral_ = 0;
    }

    return output;

}

void SimplePID::reset() {
    integral_ = 0;
}