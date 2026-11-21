#ifndef EXVECTRCONTROL_EXECUTIVE_HPP
#define EXVECTRCONTROL_EXECUTIVE_HPP

/**
 * Flight executive core: one pure engine that turns per-step inputs into one control command, driven by a
 * declarative flight plan (phases as data). Hardware-free; the vehicle supplies the plan and the adapters.
 * Includes only <stdint.h>, <stddef.h>, <math.h> and its own headers.
 */

#include "ExVectrControl/executive/behaviours.hpp"
#include "ExVectrControl/executive/exec_types.hpp"
#include "ExVectrControl/executive/flight_executive.hpp"
#include "ExVectrControl/executive/guards.hpp"
#include "ExVectrControl/executive/kick.hpp"
#include "ExVectrControl/executive/plan.hpp"
#include "ExVectrControl/executive/sanitize.hpp"
#include "ExVectrControl/executive/validate_plan.hpp"

#endif
