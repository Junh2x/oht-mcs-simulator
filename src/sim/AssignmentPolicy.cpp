#include "sim/AssignmentPolicy.h"

#include <limits>

namespace sim {

VehicleId nearestIdleAssignment(const std::vector<Vehicle>& vehicles, const Job& job,
                                const rail::RailNetwork& net) {
    VehicleId best = -1;
    float best_dist = std::numeric_limits<float>::infinity();
    for (const Vehicle& v : vehicles) {
        if (!v.alive || v.retiring || v.state != VehState::Idle) continue;
        float d = net.distanceBetween(v.at, job.origin);
        if (d < best_dist) {
            best_dist = d;
            best = v.id;
        }
    }
    return best;
}

}  // namespace sim
