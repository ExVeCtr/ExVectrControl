#ifndef EXVECTRCONTROL_EXECUTIVE_EXEC_TYPES_HPP
#define EXVECTRCONTROL_EXECUTIVE_EXEC_TYPES_HPP

/**
 * Flight executive: the inputs of one step and the one command it produces.
 *
 * The executive core (everything under ExVectrControl/executive/) is pure: it includes only <stdint.h>, <stddef.h>,
 * <math.h> and its own headers (checked by the host test test_executive_includes). No hardware, no ExVectrCore, no
 * scheduler, no globals, no dynamic allocation. The vehicle adapters fill ExecInputs and apply ControlCommand.
 *
 * Conventions: NWU world frame, attitude [wx,wy,wz, qw,qx,qy,qz] with q rotating body -> world, position
 * [Vx,Vy,Vz, Px,Py,Pz], SI units, time int64_t nanoseconds.
 */

#include <math.h>
#include <stdint.h>

namespace VCTR {
namespace Exec {

/// Phase identifier. In a vehicle plan it equals the wire value of the telemetry phase.
using PhaseId = uint8_t;

/// "No phase" (unused exit, no timeout target).
constexpr PhaseId kNoPhase = 0xFF;

/// thrustCapN when the phase applies no cap of its own (the actuator limit then limits alone). Finite on purpose.
constexpr float kNoThrustCapN = 1.0e9f;

constexpr int64_t kNsPerS = 1000000000LL;

/// Snapshot of the state estimate used by one step. The input gatherer decides validity and freshness.
struct EstimateSnapshot {
  int64_t tNs = 0;                        ///< timestamp of the newest estimate used
  float att[7] = {0, 0, 0, 1, 0, 0, 0};   ///< [wx,wy,wz, qw,qx,qy,qz], body -> world, NWU
  float pos[6] = {0, 0, 0, 0, 0, 0};      ///< [Vx,Vy,Vz, Px,Py,Pz], m/s and m
  bool attValid = false;                  ///< attitude fresh and finite
  bool posValid = false;                  ///< position fresh and finite
  float groundDist = NAN;                 ///< ground-distance estimate (rangefinder channel), m
  float groundCov = NAN;                  ///< its variance
  bool groundFresh = false;               ///< ground estimate newer than the freshness limit of the gatherer
};

/// Operator commands, already gated by the caller (legacy telecommand handler, later the T9b mapper).
struct OperatorCommands {
  bool goEdge = false;       ///< accepted go (preconditions checked by the caller)
  int16_t legacyStartS = 0;  ///< mission clock at go, s (legacy MissionBegin paramInt; parity until T9b-4)
  bool abortRequest = false; ///< level: abort wanted (re-evaluated every step, so repeating it is idempotent)
  bool resetEdge = false;    ///< system reset (gated by the caller)
};

/// Latched state from the supervisor (failure monitor).
struct SupervisorLatches {
  bool failsafe = false;        ///< latched failure: everything off. The caller clears it before a reset edge.
  bool abortNoPos = false;      ///< T9b-7: magnetometer/GNSS lost, latched (inactive until T9b-7)
  bool magTimeout = false;      ///< magnetometer timeout, routed through the phase's abort rule (X11)
  bool supervisorReady = false; ///< initialised, no zeroing window. Not read by the core (used by the applier).
};

/// Everything one step sees.
struct ExecInputs {
  int64_t nowNs = 0; ///< monotonic time of this step
  int64_t dtNs = 0;  ///< time since the previous step (the step rate is the caller's choice); <= 0 counts as 0
  EstimateSnapshot est;
  OperatorCommands cmd;
  SupervisorLatches sup;
};

enum class TvcMode : uint8_t {
  Off = 0,  ///< TVC controllers disabled (neutral output)
  Position, ///< position loop -> acc->att mapper -> TVC attitude controller; posSp is the input
  Attitude, ///< TVC attitude controller takes attSp directly, thrust request thrustN (capped by thrustCapN)
};

enum class FlapMode : uint8_t {
  Disabled = 0, ///< flap actuators off
  Retracted,    ///< flaps retracted (old BellyFlopMode_Acsent, extend false)
  Extended,     ///< flaps extended on ascent (old BellyFlopMode_Acsent, extend true)
  Descent,      ///< flap attitude control, belly down (old BellyFlopMode_Stabilize), guided by flapAzimuth/Pitch
  Upright,      ///< flaps help the vehicle upright (old BellyFlopMode_Upright)
};

/// Out-of-bounds profile read by the failure monitor.
enum class BoundsProfile : uint8_t { None = 0, Stationary, Flight };

/// Where the telemetry attitude setpoint comes from (design X7).
enum class AttSpSource : uint8_t {
  None = 0, ///< no attitude setpoint
  Command,  ///< ControlCommand::attSp
  Mapper,   ///< the acc->att mapper downstream of the position loop (TvcMode::Position)
};

/// The one command of a step. Everything that switches a controller or moves an actuator reads only this.
struct ControlCommand {
  PhaseId phase = 0;
  uint32_t phaseSeq = 0;       ///< increments on every phase entry (monotonic, survives a reset)
  int64_t missionTimeNs = 0;   ///< mission clock: 0 in the idle phase, negative in a countdown
  TvcMode tvc = TvcMode::Off;
  bool resetTvc = false;       ///< one step: reset the TVC controllers' state before using them
  float posSp[6] = {0, 0, 0, 0, 0, 0};  ///< [V,P] setpoint (Position mode; also telemetry)
  float attSp[4] = {1, 0, 0, 0};        ///< attitude target, body -> world (Attitude mode)
  AttSpSource attSpSource = AttSpSource::None;
  float thrustN = 0;                    ///< thrust request in Attitude mode, N
  float thrustCapN = kNoThrustCapN;     ///< thrust cap for the TVC actuator layer, N
  FlapMode flaps = FlapMode::Disabled;
  float flapAzimuthRad = 0;             ///< Descent mode guidance: azimuth
  float flapPitchRad = 0;               ///< Descent mode guidance: pitch
  bool tvcActuators = false;            ///< request; ANDed with the supervisor downstream
  bool flapActuators = false;           ///< request (= flaps != Disabled)
  bool motors = false;                  ///< request; ANDed with the supervisor and the motor interlock downstream
  bool estimatorZeroing = false;        ///< level, ORed with the zeroing window downstream
  uint16_t actuatorTestSeq = 0;         ///< increments once per requested TVC actuator test
  BoundsProfile bounds = BoundsProfile::None;
  bool onGround = false;                ///< phase attribute: before liftoff or landed
  bool sanitized = false;               ///< a non-finite value was replaced this step
};

/// Why the last phase change happened.
enum class ExitReason : uint8_t {
  None = 0,
  Failsafe,
  Reset,
  AbortNoPos,
  Abort,
  Go,
  Timeout,
  Exit0, ///< Exit0 + i = exit i of the phase
  Exit1,
  Exit2,
  Exit3,
};

/// Per-step status, for telemetry and logs only.
struct ExecStatus {
  enum class GoResult : uint8_t { None = 0, Accepted, RefusedNotIdle, RefusedBlocked };
  enum class AbortOutcome : uint8_t { None = 0, Ignored, ResetAndStop, ToRth, Stayed, ToNoPos };
  GoResult go = GoResult::None;              ///< this step's go edge, if any
  AbortOutcome abort = AbortOutcome::None;   ///< this step's abort level outcome, if any
  bool missionComplete = false;              ///< one step on entering the plan's complete phase
  bool entryRedirected = false;              ///< this step's target phase refused entry, its fallback was entered
  ExitReason transition = ExitReason::None;  ///< this step's phase change, if any
};

} // namespace Exec
} // namespace VCTR

#endif
