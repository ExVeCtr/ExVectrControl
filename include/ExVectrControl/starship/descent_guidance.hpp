// Descent (belly-down) guidance: position and velocity estimate -> heading setpoint, glide command and pitch split.
//
// Pure, deterministic, header-only, no scheduler, topics, clock, globals or heap. Plain float arrays: position
// [V(3), P(3)] (NWU, SI), attitude [w(3), q(4)] (q = [w,x,y,z] body -> world). Cascade (strategy B): target above
// the landing point -> funnel -> desired ground velocity -> velocity PI -> air-velocity command; the nose turns toward
// that command and the glide speed along the nose is set by a top/bottom pitch split (FlapDescentLaw).
// Invalid input (target invalid, NaN or infinite position, attitude, target or dt): the heading setpoint is held,
// the split slews to 0 (neutral 30/78, a vertical fall) and the velocity integral is frozen. Design and numbers:
// ExVectrStarship docs/descent-control-design.md.
#ifndef EXVECTRCONTROL_STARSHIP_DESCENT_GUIDANCE_HPP
#define EXVECTRCONTROL_STARSHIP_DESCENT_GUIDANCE_HPP

#include "ExVectrControl/starship/descent_common.hpp"

namespace VCTR {
namespace CTRL {
namespace descent {

// =====================================================================================================================
// Target interface
// =====================================================================================================================
/// "TargetAbove": the point the descent flies down above (set by the mission/executive above the landing point).
struct DescentTarget {
  float x_m = 0, y_m = 0;     ///< world NWU horizontal target, m
  float endAltitude_m = 27;   ///< altitude at which the descent ends (flip start estimate, world z), m, for the funnel
  bool valid = false;         ///< false: no target (hold heading, zero glide = fall straight down)
};

// =====================================================================================================================
// Guidance: position/velocity -> heading setpoint + pitch split
// =====================================================================================================================
struct DescentGuidanceParams {
  float posTimeConst_s = 3.0f;   ///< v_des = e / T (horizontal position error -> desired ground velocity)
  float vMax_mps = 4.5f;         ///< desired ground speed limit, far above the end altitude
  float vMaxEnd_mps = 1.5f;      ///< limit at the end altitude (linear between, over funnelHeight_m)
  float funnelHeight_m = 20.0f;
  float velKp = 0.8f;            ///< air-velocity command per ground-velocity error, (m/s)/(m/s)
  float velKi = 0.25f;           ///< integral (wind, trim and model error), 1/s
  float windIntLimit_mps = 7.0f; ///< |integral| limit, m/s (wind authority)
  float turnOnSpeed_mps = 1.2f;  ///< |air-velocity command| above which the heading follows it
  float turnOffSpeed_mps = 0.6f; ///< below which the heading setpoint is held (hysteresis)
  float glideMin_mps = -1.1f;    ///< backward glide limit along the nose (model: -1.15 m/s at split +12 deg)
  float glideMax_mps = 4.9f;     ///< forward glide limit (model: 4.95 m/s at split -80, top 90 / bottom 58 deg)
  float glideYawSafe_mps = 3.3f; ///< glide limit while |heading error| > fastGlideGate (bottom pair >= 70 deg: full
                                 ///< yaw authority in both aero models; below 65 deg it is model-dependent)
  float fastGlideGate_rad = 20 * kDegF;
  /// Glide map (model, shield 0.7, neutral 30/78 deg, the law's split schedule, free body; test_glide_map): air
  /// speed along the nose -> pitch split, rad. Feed-forward only; the velocity integral removes its error.
  static constexpr int kMapMax = 16;
  int mapN = 15; ///< points used (12 = yaw-safe envelope only: bottom pair >= 70 deg)
  float mapGlide_mps[kMapMax] = {-1.15f, -0.99f, -0.68f, -0.24f, 0.13f, 0.87f, 1.42f, 1.80f,
                                 2.19f,  2.59f,  2.78f,  3.37f,  3.79f, 4.29f, 4.95f};
  float mapSplit_rad[kMapMax] = {12 * kDegF,  8 * kDegF,   4 * kDegF,   0,           -4 * kDegF,
                                 -12 * kDegF, -20 * kDegF, -30 * kDegF, -40 * kDegF, -50 * kDegF,
                                 -60 * kDegF, -68 * kDegF, -72 * kDegF, -76 * kDegF, -80 * kDegF};
  float glideHeadingGate_rad = 60 * kDegF; ///< |heading error| above which the glide command is 0 (turn first)
  float splitRate_radps = 40 * kDegF;      ///< slew limit of the pitch split (smooth glide changes)
  float entryHold_s = 1.5f;      ///< after phase entry: heading held, entry split (flaps deploy from the kick)
  float entrySplit_rad = -20 * kDegF; ///< split during the entry hold: top pair 50 deg (damps the deploy pitch-up)
  bool forwardOnlyHeading = true; ///< true: nose toward the command (B). false: nose axis mod 180 deg (study)
};

/// Explicit memory (the flight executive's behaviour memory POD).
struct DescentGuidanceMem {
  float windInt[2] = {0, 0}; ///< velocity integral, world N/W, m/s
  float headingSp = 0;       ///< held heading setpoint, rad
  bool following = false;    ///< heading follows the command (hysteresis state)
  float elapsed_s = 0;       ///< time since phase entry
  float split = 0;           ///< slewed pitch split, rad
};

struct DescentGuidanceOut {
  float headingSp_rad = 0;  ///< nose azimuth setpoint (CCW from North)
  float glideCmd_mps = 0;   ///< air speed command along the measured nose
  float pitchSplit_rad = 0; ///< pitch split for FlapDescentLaw (+ = nose-up / slower)
  float vDes[2] = {0, 0};   ///< desired ground velocity (telemetry)
  float vAirCmd[2] = {0, 0};///< air-velocity command (telemetry)
};

class DescentGuidance {
 public:
  /// Phase entry: integrator zero, heading setpoint = current nose heading (0 if the attitude is invalid).
  static void init(const float att[7], DescentGuidanceMem &m) {
    m = DescentGuidanceMem();
    m.headingSp = allFinite(att, 7) ? bellyAttitude(att).heading : 0.0f;
    if (!std::isfinite(m.headingSp))
      m.headingSp = 0;
    m.split = 0;
  }
  /// Phase entry with an initial split (the entry hold value applies at once, not slewed).
  static void init(const DescentGuidanceParams &p, const float att[7], DescentGuidanceMem &m) {
    init(att, m);
    m.split = p.entrySplit_rad;
  }

  static float slew(float x, float target, float maxStep) { return x + clampf(target - x, -maxStep, maxStep); }

  static float glideToSplit(const DescentGuidanceParams &p, float g) {
    if (g <= p.mapGlide_mps[0])
      return p.mapSplit_rad[0];
    for (int i = 1; i < p.mapN; i++)
      if (g <= p.mapGlide_mps[i]) {
        const float f = (g - p.mapGlide_mps[i - 1]) / (p.mapGlide_mps[i] - p.mapGlide_mps[i - 1]);
        return p.mapSplit_rad[i - 1] + f * (p.mapSplit_rad[i] - p.mapSplit_rad[i - 1]);
      }
    return p.mapSplit_rad[p.mapN - 1];
  }

  static DescentGuidanceOut step(const DescentGuidanceParams &p, const DescentTarget &t, const float pos[6],
                                 const float att[7], float dt, DescentGuidanceMem &m) {
    DescentGuidanceOut o;
    // Sanitise the memory (a corrupted state must not propagate).
    if (!allFinite(m.windInt, 2)) // before any use
      m.windInt[0] = m.windInt[1] = 0;
    if (!std::isfinite(m.headingSp))
      m.headingSp = 0;
    if (!std::isfinite(m.split))
      m.split = 0;
    if (!std::isfinite(m.elapsed_s))
      m.elapsed_s = 0;
    const bool dtOk = std::isfinite(dt) && dt > 0;
    const bool inputsOk = allFinite(pos, 6) && allFinite(att, 7) && std::isfinite(t.x_m) && std::isfinite(t.y_m) &&
                          std::isfinite(t.endAltitude_m) && dtOk;
    if (dtOk)
      m.elapsed_s += dt;
    if (!t.valid || !inputsOk) { // no target or invalid input: hold heading, zero glide
      o.headingSp_rad = m.headingSp;
      const float hold = 0.0f; // 0 = neutral 30/78, vertical fall
      m.split = slew(m.split, hold, p.splitRate_radps * (dtOk ? dt : 0));
      o.pitchSplit_rad = m.split;
      return o;
    }
    const BellyAttitude b = bellyAttitude(att);
    // 1. Funnel: desired ground velocity toward the target, limit shrinking toward the end altitude.
    const float ex = t.x_m - pos[3], ey = t.y_m - pos[4];
    const float hGo = pos[5] - t.endAltitude_m;
    const float vLim = p.vMaxEnd_mps + (p.vMax_mps - p.vMaxEnd_mps) * clampf(hGo / p.funnelHeight_m, 0, 1);
    float vdx = ex / p.posTimeConst_s, vdy = ey / p.posTimeConst_s;
    const float vd = sqrtf(vdx * vdx + vdy * vdy);
    if (vd > vLim)
      vdx *= vLim / vd, vdy *= vLim / vd;
    // 2. Velocity PI -> air-velocity command (the integral learns the wind).
    const float evx = vdx - pos[0], evy = vdy - pos[1];
    float cx = vdx + p.velKp * evx + m.windInt[0], cy = vdy + p.velKp * evy + m.windInt[1];
    // 3. Heading: follow the command direction above turnOn, hold below turnOff.
    const float c = sqrtf(cx * cx + cy * cy);
    if (c > p.turnOnSpeed_mps)
      m.following = true;
    else if (c < p.turnOffSpeed_mps)
      m.following = false;
    if (m.following) {
      float h = atan2f(cy, cx);
      if (!p.forwardOnlyHeading && fabsf(wrapPi(h - b.heading)) > 0.5f * kPi)
        h = wrapPi(h + kPi);
      m.headingSp = h;
    }
    // 3b. Entry hold (flaps deploy from the kick): the heading setpoint already steers towards the target (the
    //     guidance is active from the first tick, at any altitude), but the pitch split stays at the entry split, no
    //     glide command and no wind integration yet.
    if (m.elapsed_s < p.entryHold_s) {
      o.headingSp_rad = m.headingSp;
      m.split = slew(m.split, p.entrySplit_rad, p.splitRate_radps * dt);
      o.pitchSplit_rad = m.split;
      return o;
    }
    // 4. Glide command = projection of the air-velocity command on the measured nose direction.
    const float nx = cosf(b.heading), ny = sinf(b.heading);
    float g = cx * nx + cy * ny;
    const float ehAbs = fabsf(wrapPi(m.headingSp - b.heading));
    if (ehAbs > p.glideHeadingGate_rad)
      g = 0; // turn first: no glide (and no brake) while the nose is far from the setpoint
    else if (ehAbs > p.fastGlideGate_rad && g > p.glideYawSafe_mps)
      g = p.glideYawSafe_mps; // fast glide (bottom pair < 70 deg) only when aligned
    const float gSat = clampf(g, p.glideMin_mps, p.glideMax_mps);
    // 5. Integrate, with anti-windup: no integration of the along-nose part while the glide is saturated in that
    //    direction, and a hard magnitude limit.
    float ix = p.velKi * evx * dt, iy = p.velKi * evy * dt;
    if (g != gSat) {
      const float along = ix * nx + iy * ny;
      if ((g > gSat && along > 0) || (g < gSat && along < 0))
        ix -= along * nx, iy -= along * ny;
    }
    m.windInt[0] += ix, m.windInt[1] += iy;
    const float wi = sqrtf(m.windInt[0] * m.windInt[0] + m.windInt[1] * m.windInt[1]);
    if (wi > p.windIntLimit_mps)
      m.windInt[0] *= p.windIntLimit_mps / wi, m.windInt[1] *= p.windIntLimit_mps / wi;
    o.headingSp_rad = m.headingSp;
    o.glideCmd_mps = gSat;
    m.split = slew(m.split, glideToSplit(p, gSat), p.splitRate_radps * dt);
    o.pitchSplit_rad = m.split;
    o.vDes[0] = vdx, o.vDes[1] = vdy;
    o.vAirCmd[0] = cx, o.vAirCmd[1] = cy;
    return o;
  }
};

} // namespace descent
} // namespace CTRL
} // namespace VCTR

#endif
