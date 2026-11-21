#include "ExVectrControl/executive/behaviours.hpp"

#include <math.h>

#include "ExVectrControl/executive/exec_math.hpp"
#include "ExVectrControl/executive/kick.hpp"

namespace VCTR {
namespace Exec {

// ---------------------------------------------------------------------------------------------------------------
// GoTo (legacy MissionWaypoint::missionHover profile, steps 4 and 5)

void goToInit(const GoToParams &p, const EstimateSnapshot &est, GoToMem &m) {
  const float *start = p.entry == GoToEntry::JumpTo ? p.entryPos : &est.pos[3];
  if (double(p.speedMps) < 0.0001) // instant-jump waypoint (legacy: velocity < 0.0001)
    start = p.target;
  for (int i = 0; i < 3; i++)
    m.spPos[i] = start[i];
  m.speedMps = 0.0f;
}

void goToStep(const GoToParams &p, const EstimateSnapshot &est, float dtS, GoToMem &m, BehaviourOut &out) {
  float d[3];
  sub3(p.target, &est.pos[3], d);
  const float dMag = norm3(d);

  // Speed profile: brake to sqrt(|d| * stopAccel), otherwise ramp up by startAccel * dt up to the cruise speed.
  const float brakeLimit = sqrtf(dMag * p.stopAccelMps2);
  if (double(p.stopAccelMps2) > 0.001 && m.speedMps > brakeLimit) {
    m.speedMps = brakeLimit;
  } else if (p.speedMps > m.speedMps) {
    m.speedMps += p.startAccelMps2 * dtS;
    if (m.speedMps > p.speedMps || double(p.startAccelMps2) < 0.001)
      m.speedMps = p.speedMps;
  }

  // Move the setpoint towards the target by min(remaining, v * dt); velocity setpoint = unit * v.
  float travel[3], dir[3];
  sub3(p.target, m.spPos, travel);
  normalizeOrZero3(travel, dir);
  if (norm3(travel) / dtS > m.speedMps) {
    for (int i = 0; i < 3; i++)
      travel[i] = dir[i] * m.speedMps * dtS;
  }
  for (int i = 0; i < 3; i++) {
    out.posSp[i] = dir[i] * m.speedMps;
    m.spPos[i] += travel[i];
    out.posSp[3 + i] = m.spPos[i];
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Transit (legacy MissionRTH Translation)

void transitInit(const TransitParams &, const EstimateSnapshot &est, TransitMem &m) {
  for (int i = 0; i < 3; i++)
    m.spPos[i] = est.pos[3 + i];
  m.speedMps = 0.0f;
}

void transitStep(const TransitParams &p, const EstimateSnapshot &, float dtS, TransitMem &m, BehaviourOut &out) {
  float travel[3], dir[3];
  sub3(p.home, m.spPos, travel);
  const float mag = norm3(travel);
  for (int i = 0; i < 3; i++)
    dir[i] = travel[i] / mag; // no zero guard (legacy); the sanitiser catches a non-finite result

  if (m.speedMps < p.maxSpeedMps)
    m.speedMps += dtS * p.accelMps2;
  if (m.speedMps > p.maxSpeedMps)
    m.speedMps = p.maxSpeedMps;

  if (mag / dtS > m.speedMps) {
    for (int i = 0; i < 3; i++)
      travel[i] = dir[i] * m.speedMps * dtS;
  }

  const float brakeLimit = sqrtf(mag * p.accelMps2);
  if (m.speedMps > brakeLimit)
    m.speedMps = brakeLimit;

  for (int i = 0; i < 3; i++) {
    out.posSp[i] = dir[i] * m.speedMps;
    m.spPos[i] += travel[i];
    out.posSp[3 + i] = m.spPos[i];
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Land (legacy MissionRTH Descent)

void landInit(const LandParams &, const EstimateSnapshot &est, LandMem &m) { m.spZ = est.pos[5]; }

void landStep(const LandParams &p, const EstimateSnapshot &est, float dtS, LandMem &m, BehaviourOut &out) {
  if (est.pos[5] - m.spZ < p.holdAboveM)
    m.spZ -= dtS * p.descentRateMps;
  out.posSp[0] = out.posSp[1] = out.posSp[2] = 0.0f;
  out.posSp[3] = p.home[0];
  out.posSp[4] = p.home[1];
  out.posSp[5] = m.spZ;
}

// ---------------------------------------------------------------------------------------------------------------
// Kick attitude

void kickInit(const KickParams &p, const EstimateSnapshot &est, KickMem &m) {
  kickTargetAttitude(&est.att[3], p.pitchRad, m.target);
  for (int i = 0; i < 3; i++)
    m.holdPos[i] = est.pos[3 + i];
}

void kickStep(const KickParams &p, const KickMem &m, BehaviourOut &out) {
  for (int i = 0; i < 4; i++)
    out.attSp[i] = m.target[i];
  out.thrustN = p.thrustN;
  setHoldPoint(m.holdPos, out);
}

// ---------------------------------------------------------------------------------------------------------------
// Descent flap guidance (legacy FlapModeObserverTask, Stabilize branch)

float descentStopDistanceM(float vz, float thrustN, float massKg) {
  const float aSum = thrustN / massKg - 9.81f;
  return 0.5f * vz * vz / aSum;
}

void descentInit(const DescentParams &, const EstimateSnapshot &, DescentMem &m) { m.azimuthRad = 0.0f; }

void descentStep(const DescentParams &p, const EstimateSnapshot &est, DescentMem &m, BehaviourOut &out) {
  const float px = est.pos[3], py = est.pos[4], pz = est.pos[5];
  float pitch = 0.0f;
  if (pz > p.highAltitudeM) {
    const float dXY = sqrtf(px * px + py * py);
    float offset = -(dXY - p.offsetDistM) * p.pitchSlopeRadPerM;
    if (offset > 0.0f)
      offset = 0.0f;
    else if (offset < p.minPitchOffsetRad)
      offset = p.minPitchOffsetRad;
    m.azimuthRad = -atan2f(py, -px); // back towards the origin
    pitch = offset + p.pitchBiasRad;
  } else if (pz < descentStopDistanceM(est.pos[2], p.thrustN, p.massKg) + p.stopMarginM + p.lowExtraMarginM) {
    pitch = p.lowPitchRad;
  }
  out.flapAzimuthRad = m.azimuthRad;
  out.flapPitchRad = pitch;
  for (int i = 0; i < 6; i++)
    out.posSp[i] = 0.0f; // legacy freefall published a zero setpoint
}

// ---------------------------------------------------------------------------------------------------------------
// Dispatch

uint8_t behaviourParamCount(Behaviour b, const BehaviourTables &t) {
  switch (b) {
  case Behaviour::Idle:
  case Behaviour::HoldMeasured:
    return 1;
  case Behaviour::GoTo:
    return t.goTo ? t.goToCount : 0;
  case Behaviour::Transit:
    return t.transit ? t.transitCount : 0;
  case Behaviour::Land:
    return t.land ? t.landCount : 0;
  case Behaviour::KickAttitude:
    return t.kick ? t.kickCount : 0;
  case Behaviour::DescentGuidance:
    return t.descent ? t.descentCount : 0;
  }
  return 0;
}

void behaviourInit(Behaviour b, uint8_t idx, const BehaviourTables &t, const EstimateSnapshot &est,
                   BehaviourMem &mem) {
  mem = BehaviourMem{};
  mem.active = b;
  mem.paramIdx = idx;
  switch (b) {
  case Behaviour::Idle:
  case Behaviour::HoldMeasured:
    break;
  case Behaviour::GoTo:
    goToInit(t.goTo[idx], est, mem.goTo);
    break;
  case Behaviour::Transit:
    transitInit(t.transit[idx], est, mem.transit);
    break;
  case Behaviour::Land:
    landInit(t.land[idx], est, mem.land);
    break;
  case Behaviour::KickAttitude:
    kickInit(t.kick[idx], est, mem.kick);
    break;
  case Behaviour::DescentGuidance:
    descentInit(t.descent[idx], est, mem.descent);
    break;
  }
}

void behaviourStep(const BehaviourTables &t, const EstimateSnapshot &est, float dtS, BehaviourMem &mem,
                   BehaviourOut &out) {
  const uint8_t idx = mem.paramIdx;
  switch (mem.active) {
  case Behaviour::Idle:
    break;
  case Behaviour::HoldMeasured:
    setHoldPoint(&est.pos[3], out);
    break;
  case Behaviour::GoTo:
    goToStep(t.goTo[idx], est, dtS, mem.goTo, out);
    break;
  case Behaviour::Transit:
    transitStep(t.transit[idx], est, dtS, mem.transit, out);
    break;
  case Behaviour::Land:
    landStep(t.land[idx], est, dtS, mem.land, out);
    break;
  case Behaviour::KickAttitude:
    kickStep(t.kick[idx], mem.kick, out);
    break;
  case Behaviour::DescentGuidance:
    descentStep(t.descent[idx], est, mem.descent, out);
    break;
  }
}

bool behaviourTarget(Behaviour b, uint8_t idx, const BehaviourTables &t, float out[3]) {
  if (idx >= behaviourParamCount(b, t))
    return false;
  const float *p = nullptr;
  switch (b) {
  case Behaviour::GoTo:
    p = t.goTo[idx].target;
    break;
  case Behaviour::Transit:
    p = t.transit[idx].home;
    break;
  case Behaviour::Land:
    p = t.land[idx].home;
    break;
  default:
    return false;
  }
  for (int i = 0; i < 3; i++)
    out[i] = p[i];
  return true;
}

} // namespace Exec
} // namespace VCTR
