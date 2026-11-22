#include "ExVectrControl/starship/control_attitude_flaps.hpp"

#include "ExVectrCore/print.hpp"
#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrDSP/value_covariance.hpp"
#include "ExVectrMath/constants.hpp"
#include "ExVectrMath/matrix_quaternion.hpp"
#include "ExVectrMath/matrix_vector.hpp"

namespace VCTR {
namespace CTRL {

ControlAttitudeFlaps::ControlAttitudeFlaps()
    : Core::Task_Periodic("Control Rocket", 20 * Core::MILLISECONDS) {
  // attach to scheduler
  Core::getSystemScheduler().addTask(*this);
}

/**
 * * @brief Subsribes to a topic to which the attitude estimation is published
 * in form: [W, Q], where W is the angular velocity vector and Q is a unit
 * quaternion rotation from the reference frame to body frame.
 */
void ControlAttitudeFlaps::subscribeAttitudeMeasurement(
    Core::Topic<Core::Timestamped<Math::Vector<float, 7>>> &attTopic) {
  attMeasSubr_.subscribe(attTopic);
}

void ControlAttitudeFlaps::subscribePositionMeasurement(
    Core::Topic<Core::Timestamped<Math::Vector<float, 6>>> &posTopic) {
  posMeasSubr_.subscribe(posTopic);
}

void ControlAttitudeFlaps::subscribeFlapSettingOutputTopic(
    Core::Topic<ControlAttitudeFlapSetting> &flapOutputTopic) {
  flapOutputTopic_.subscribe(flapOutputTopic);
}

void ControlAttitudeFlaps::taskCheck() {}

void ControlAttitudeFlaps::taskInit() {
  // Only run when all subscribers have gotten data at least once
  if (!attMeasSubr_.isDataNew() /* || !stateSetpointSubr_.isDataNew()*/) {
    setInitialised(false);
  }

  attitudeEstimation_.data = {0, 0, 0, 1, 0, 0, 0};
}

void ControlAttitudeFlaps::taskThread() {
  // if (stateSetpointSubr_.isDataNew()) {
  //     stateSetpoint_ = stateSetpointSubr_.getItem();
  // }
  if (posMeasSubr_.isDataNew()) {
    positionEstimation_ = posMeasSubr_.getItem();
  }
  if (attMeasSubr_.isDataNew()) {
    attitudeEstimation_ = attMeasSubr_.getItem();
  }

  if (descentTargetSubr_.isDataNew()) {
    descentTarget_ = descentTargetSubr_.getItem();
  }

  // Time step of this tick (clamped: a paused scheduler must not give a huge step).
  const int64_t now = Core::NowNs();
  float dt = 0.02f;
  if (lastRunNs_ != 0) {
    dt = static_cast<float>(now - lastRunNs_) * 1e-9f;
    if (!(dt > 0.001f)) dt = 0.001f;
    if (dt > 0.1f) dt = 0.1f;
  }
  lastRunNs_ = now;

  // Position estimate fresh? (the estimators stamp with NowNs; a stopped estimator makes the descent target invalid)
  const int64_t posAge = now - positionEstimation_.timestamp;
  const bool posFresh = positionEstimation_.timestamp != 0 && posAge >= 0 && posAge <= 500 * Core::MILLISECONDS;
  float pos[6], att[7];
  for (int i = 0; i < 6; i++) pos[i] = positionEstimation_.data(i);
  for (int i = 0; i < 7; i++) att[i] = attitudeEstimation_.data(i);

  // Descent: guidance step (init on entry, reset on leaving); other modes pass through.
  const ControlAttitudeBellyFlopSetting controlSettings =
      descentCtrl_.apply(controlSetting_.getItem(), descentTarget_, pos, posFresh, att, dt);

  // All mode mapping is in FlapModeMapper (pure, host-tested).
  ControlAttitudeFlapSetting flapSetting = FlapModeMapper::map(
      controlSettings, attitudeEstimation_.data, stabilizeParams_, descentParams_, descentMem_);

  // Publish the flap setting to the topic
  flapOutputTopic_.publish(flapSetting);
}

} // namespace CTRL
} // namespace VCTR
