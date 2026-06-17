#pragma once

#include "sim/Simulation.h"

namespace sim {

// Nearest idle vehicle to the job origin (straight-line distance), lowest id breaks ties.
// Returns -1 when no vehicle is available. This is the default STEP 3 dispatching policy.
VehicleId nearestIdleAssignment(const std::vector<Vehicle>& vehicles, const Job& job,
                                const rail::RailNetwork& net);

}  // namespace sim
