#pragma once

#include "sim/Simulation.h"
#include "rail/RailNetwork.h"

#include <deque>
#include <vector>

namespace sim {

// One dispatch decision: hand job to vehicle. Produced by a policy, applied by the Simulation.
struct DispatchAssignment {
    JobId job = -1;
    VehicleId vehicle = -1;
};

// Read-only view a policy may inspect when choosing assignments for one step.
struct DispatchContext {
    const std::vector<Vehicle>& vehicles;
    const std::vector<Job>& jobs;
    const std::deque<JobId>& pending;  // FIFO order, front is the oldest waiting job
    const rail::RailNetwork& net;
    int window;                        // consider at most this many jobs from the front
};

// Strategy interface. Policies are stateless and deterministic so a batch run reproduces the live run.
// Each call returns pairings drawn from the front of the queue; every idle vehicle and job appears once.
class DispatchPolicy {
public:
    virtual ~DispatchPolicy() = default;
    virtual const char* name() const = 0;
    virtual std::vector<DispatchAssignment> assign(const DispatchContext& ctx) const = 0;
};

// Baseline: ignore distance, give each job (FIFO) the lowest-id idle vehicle.
class FirstIdlePolicy : public DispatchPolicy {
public:
    const char* name() const override { return "first-idle (baseline)"; }
    std::vector<DispatchAssignment> assign(const DispatchContext& ctx) const override;
};

// Job-initiated greedy: each job (FIFO) takes the idle vehicle with the shortest rail path to pickup.
class NearestVehiclePolicy : public DispatchPolicy {
public:
    const char* name() const override { return "nearest vehicle"; }
    std::vector<DispatchAssignment> assign(const DispatchContext& ctx) const override;
};

// Global greedy: commit the cheapest (vehicle, job) pair first, minimizing total empty travel this step.
class GreedyMatchPolicy : public DispatchPolicy {
public:
    const char* name() const override { return "greedy min empty-travel"; }
    std::vector<DispatchAssignment> assign(const DispatchContext& ctx) const override;
};

// Used when the app sets no policy, so the core runs standalone (tests, batch).
const DispatchPolicy& defaultDispatchPolicy();

}  // namespace sim
