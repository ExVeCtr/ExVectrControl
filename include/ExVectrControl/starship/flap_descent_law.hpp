// Flap attitude law for the descent (belly-down) mode: heading setpoint + pitch split + roll hold -> four flap angles.
//
// Pure, deterministic, header-only, no scheduler, topics, clock, globals or heap. Attitude [w(3), q(4)] (q = [w,x,y,z]
// body -> world), angles in rad. Neutral 30/78 deg (top/bottom): zero glide, zero pitch moment, sign-robust yaw.
// Yaw = bottom-pair differential, roll hold = top-pair differential, glide = top/bottom pitch split.
// Invalid attitude (NaN or infinite): neutral flaps 30/78 (a vertical fall). Invalid heading setpoint: no heading
// loop (yaw-rate damping only). Invalid split: 0. Design and numbers: ExVectrStarship docs/descent-control-design.md.
#ifndef EXVECTRCONTROL_STARSHIP_FLAP_DESCENT_LAW_HPP
#define EXVECTRCONTROL_STARSHIP_FLAP_DESCENT_LAW_HPP

#include "ExVectrControl/starship/descent_common.hpp"

namespace VCTR {
namespace CTRL {
namespace descent {

struct FlapDescentLawParams {
  float neutralTop_rad = 30 * kDegF;    ///< zero-glide trim (model): pitch moment ~0, glide about -0.2 m/s
  float neutralBottom_rad = 78 * kDegF; ///< bottom >= 70 deg: yaw sign robust with and without the fold shield
  float kHeading = 1.5f;                ///< heading error -> yaw-rate command, 1/s
  float yawRateMax = 75 * kDegF;        ///< yaw-rate command limit, rad/s
  float kYawRate = 0.40f;               ///< virtual yaw command Y per yaw-rate error, rad/(rad/s)
  float yawMax_rad = 12 * kDegF;        ///< bottom differential limit (78 + 12 = 90)
  float kPitch = 0.60f;                 ///< pitch command per pitch error (setpoint 0), rad/rad
  float kPitchRate = 0.40f;             ///< pitch command per pitch rate, rad/(rad/s)
  float kRoll = 2.0f;                   ///< virtual roll command R per roll error, rad/rad
  float kRollRate = 0.25f;              ///< R per roll rate, rad/(rad/s)
  float rollMax_rad = 15 * kDegF;       ///< top differential limit (30 - 15 = 15 deg clamp)
  /// Yaw/roll decoupler: bottom diff = dYY*Y + dYR*R, top diff = dRY*Y + dRR*R, from the inverse of the model's
  /// 2x2 moment matrix at 30/78 deg (shield 0.7): [tau_x; tau_z] = [[0.0154 0.0010] [0.0028 0.0036]] N m/deg
  /// times [bottom diff; top diff], normalised so Y gives 0.0154 N m/deg of yaw and R 0.0036 N m/deg of roll.
  float dYY = 1.05f, dYR = -0.07f, dRY = -0.82f, dRR = 1.05f;
  float splitMin_rad = -80 * kDegF; ///< forward: top pair 30 -> 90 deg, then bottom 78 -> 58 deg
  float splitMax_rad = 12 * kDegF;  ///< brake: top 30 -> 18, bottom 78 -> 90 deg
  float flapMin_rad = 15 * kDegF, flapMax_rad = 90 * kDegF; ///< StarshipFlaps clamp
  float fadeStart_rad = 35 * kDegF, fadeWidth_rad = 20 * kDegF;
  float turnLatch_rad = 150 * kDegF;    ///< beyond this heading error the turn direction is latched
};

struct FlapDescentCmd {
  float headingSp_rad = 0;
  float pitchSplit_rad = 0;
  float rollSp_rad = 0;      ///< roll setpoint (strategy C only; 0 in the recommended law)
  bool headingFree = false;  ///< true: no heading loop (yaw-rate damping only), strategy C
};

struct FlapDescentLawMem {
  float lastHeadingErr = 0;
};

struct FlapDescentLawOut {
  float flap[4];             ///< TL, TR, BL, BR, rad (clamped)
  float u[3];                ///< yaw diff (bottom), pitch common, roll diff (top), rad; telemetry
  float headingErr;
};

/**
 * Allocation (from the model's per-flap moment Jacobian at 25/72 deg, 6.4 m/s, N m/deg; shield 0.7 / off):
 *   bottom flap: tau_x (yaw) +-0.0070 / +-0.0105, tau_y +0.0048 / +0.0094, tau_z +-0.0019 / +-0.0035
 *   top flap:    tau_x +-0.0010 / +-0.0018,         tau_y -0.0041 / -0.0025, tau_z +-0.0016 / +-0.0010
 * so yaw = bottom differential (BL +, BR -), roll = top differential (TL +, TR -), pitch = top -, bottom +.
 * The legacy mixer drove yaw on all four flaps, and its roll command made 1.35x more yaw moment than its yaw
 * command (u2 -> tau_x +0.016 vs u0 -> tau_x +0.012 N m/deg).
 */
class FlapDescentLaw {
 public:
  /// Bottom-pair part of the split schedule: brake retracts the bottom pair; beyond the top travel (split below
  /// -60 deg) the bottom pair extends down to 70 deg for the last bit of forward glide.
  static float bottomSplit(const FlapDescentLawParams &p, float split) {
    if (split > 0)
      return split;
    const float topTravel = p.flapMax_rad - p.neutralTop_rad;
    return split < -topTravel ? split + topTravel : 0;
  }

  /// Neutral flaps (30/78 by default), used when the attitude is invalid.
  static FlapDescentLawOut neutral(const FlapDescentLawParams &p, FlapDescentLawMem &m) {
    FlapDescentLawOut o;
    o.flap[0] = o.flap[1] = clampf(p.neutralTop_rad, p.flapMin_rad, p.flapMax_rad);
    o.flap[2] = o.flap[3] = clampf(p.neutralBottom_rad, p.flapMin_rad, p.flapMax_rad);
    o.u[0] = o.u[1] = o.u[2] = 0;
    o.headingErr = 0;
    m.lastHeadingErr = 0;
    return o;
  }

  static FlapDescentLawOut step(const FlapDescentLawParams &p, const float att[7], const FlapDescentCmd &cIn,
                                FlapDescentLawMem &m) {
    if (!allFinite(att, 7))
      return neutral(p, m);
    if (!std::isfinite(m.lastHeadingErr))
      m.lastHeadingErr = 0;
    FlapDescentCmd c = cIn; // sanitised copy
    const bool headingOk = std::isfinite(c.headingSp_rad);
    if (!std::isfinite(c.pitchSplit_rad))
      c.pitchSplit_rad = 0;
    if (!std::isfinite(c.rollSp_rad))
      c.rollSp_rad = 0;
    const BellyAttitude b = bellyAttitude(att);
    FlapDescentLawOut o;
    float eh = 0;
    if (!c.headingFree && headingOk && b.headingValid) {
      eh = wrapPi(c.headingSp_rad - b.heading);
      // Turn-direction latch: near 180 deg keep turning the way we started (no snapping between +-180).
      if (fabsf(eh) > p.turnLatch_rad && eh * m.lastHeadingErr < 0)
        eh += eh < 0 ? 2 * kPi : -2 * kPi;
      m.lastHeadingErr = eh;
    }
    const float rCmd = c.headingFree ? 0 : clampf(p.kHeading * eh, -p.yawRateMax, p.yawRateMax);
    // Virtual commands: Y (yaw moment), R (roll moment), decoupled into the bottom (yaw) and top (roll) pairs.
    // Roll has priority: if the top differential would saturate, Y is reduced first (without this the yaw command's
    // roll moment wins over the weak top-pair roll authority and the vehicle spins, no-shield model).
    float Y = p.kYawRate * (rCmd - att[0]);
    const float R = p.kRoll * (c.rollSp_rad - b.roll) - p.kRollRate * att[2];
    float topD = p.dRY * Y + p.dRR * R;
    if (fabsf(topD) > p.rollMax_rad && fabsf(p.dRY) > 1e-6f) {
      const float lim = topD > 0 ? p.rollMax_rad : -p.rollMax_rad;
      const float yNew = (lim - p.dRR * R) / p.dRY;
      Y = (yNew * Y > 0) ? (fabsf(yNew) < fabsf(Y) ? yNew : Y) : 0;
      topD = p.dRY * Y + p.dRR * R;
    }
    const float roll = clampf(topD, -p.rollMax_rad, p.rollMax_rad);
    const float yaw = clampf(p.dYY * Y + p.dYR * R, -p.yawMax_rad, p.yawMax_rad);
    const float split = clampf(c.pitchSplit_rad, p.splitMin_rad, p.splitMax_rad);
    // Pitch: the split is scheduled so that the bottom flaps stay >= 70 deg (yaw authority, see the class comment):
    // forward glide (split < 0) retracts only the top pair, braking (split > 0) extends the top pair and retracts
    // the bottom pair. The pitch attitude/rate damping u1 acts on both pairs (small).
    const float u1 = -p.kPitch * b.pitch - p.kPitchRate * att[1];
    const float ff = 1 - clampf((b.belly - p.fadeStart_rad) / p.fadeWidth_rad, 0, 1);
    float topBase = p.neutralTop_rad - split - u1;
    float botBase = p.neutralBottom_rad + bottomSplit(p, split) + u1;
    // Priority: the common parts are clamped so that every flap keeps its differential inside the travel (a pinned
    // flap turns the split into a yaw/roll moment).
    const float dT = fabsf(roll), dB = fabsf(yaw);
    topBase = clampf(topBase, p.flapMin_rad + dT, fmaxf(p.flapMin_rad + dT, p.flapMax_rad - dT));
    botBase = clampf(botBase, p.flapMin_rad + dB, fmaxf(p.flapMin_rad + dB, p.flapMax_rad - dB));
    // Fade (belly angle beyond 35..55 deg): blend toward the neutral, as the legacy law does.
    const float tb = p.neutralTop_rad + (topBase - p.neutralTop_rad) * ff;
    const float bb = p.neutralBottom_rad + (botBase - p.neutralBottom_rad) * ff;
    o.flap[0] = tb + roll * ff;
    o.flap[1] = tb - roll * ff;
    o.flap[2] = bb + yaw * ff;
    o.flap[3] = bb - yaw * ff;
    for (int i = 0; i < 4; i++)
      o.flap[i] = clampf(o.flap[i], p.flapMin_rad, p.flapMax_rad);
    o.u[0] = yaw, o.u[1] = u1, o.u[2] = roll;
    o.headingErr = eh;
    return o;
  }
};

} // namespace descent
} // namespace CTRL
} // namespace VCTR

#endif
