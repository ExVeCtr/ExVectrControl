#include "ExVectrControl/executive/plan.hpp"

namespace VCTR {
namespace Exec {

int findPhase(const FlightPlan &plan, PhaseId id) {
  if (plan.phases == nullptr || id == kNoPhase)
    return -1;
  for (int i = 0; i < plan.phaseCount; i++)
    if (plan.phases[i].id == id)
      return i;
  return -1;
}

} // namespace Exec
} // namespace VCTR
