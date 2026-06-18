#include "sim/Dispatch.h"

#include <algorithm>
#include <limits>

namespace sim {

namespace {

constexpr float kUnreachable = std::numeric_limits<float>::infinity();

// Idle vehicles eligible for work, in ascending id order (deterministic tie-break).
std::vector<VehicleId> idleVehicles(const std::vector<Vehicle>& vs) {
    std::vector<VehicleId> idle;
    for (const Vehicle& v : vs) {
        if (v.alive && !v.retiring && v.state == VehState::Idle) idle.push_back(v.id);
    }
    return idle;
}

// Front slice of the pending queue, bounded by the look-ahead window.
std::vector<JobId> frontJobs(const std::deque<JobId>& pending, int window) {
    int n = static_cast<int>(pending.size());
    int take = (window > 0 && window < n) ? window : n;
    std::vector<JobId> jobs;
    jobs.reserve(take);
    for (int i = 0; i < take; ++i) jobs.push_back(pending[i]);
    return jobs;
}

// Empty-travel cost: rail path distance the vehicle drives to reach the pickup.
float pickupCost(const rail::RailNetwork& net, rail::NodeId from, rail::NodeId origin) {
    rail::PathResult p = rail::dijkstra(net, from, origin);
    return p.found ? p.distance : kUnreachable;
}

}  // namespace

std::vector<DispatchAssignment> FirstIdlePolicy::assign(const DispatchContext& ctx) const {
    std::vector<DispatchAssignment> out;
    std::vector<VehicleId> idle = idleVehicles(ctx.vehicles);
    std::size_t next = 0;
    for (JobId jid : frontJobs(ctx.pending, ctx.window)) {
        if (next >= idle.size()) break;
        out.push_back({jid, idle[next++]});
    }
    return out;
}

std::vector<DispatchAssignment> NearestVehiclePolicy::assign(const DispatchContext& ctx) const {
    std::vector<DispatchAssignment> out;
    std::vector<VehicleId> idle = idleVehicles(ctx.vehicles);
    std::vector<char> used(idle.size(), 0);
    for (JobId jid : frontJobs(ctx.pending, ctx.window)) {
        const Job& job = ctx.jobs[jid];
        int best = -1;
        float best_cost = kUnreachable;
        for (std::size_t i = 0; i < idle.size(); ++i) {
            if (used[i]) continue;
            float c = pickupCost(ctx.net, ctx.vehicles[idle[i]].at, job.origin);
            if (c < best_cost) { best_cost = c; best = static_cast<int>(i); }
        }
        if (best < 0) break;  // every idle vehicle already taken this step
        used[best] = 1;
        out.push_back({jid, idle[best]});
    }
    return out;
}

std::vector<DispatchAssignment> GreedyMatchPolicy::assign(const DispatchContext& ctx) const {
    std::vector<DispatchAssignment> out;
    std::vector<VehicleId> idle = idleVehicles(ctx.vehicles);
    std::vector<JobId> jobs = frontJobs(ctx.pending, ctx.window);
    if (idle.empty() || jobs.empty()) return out;

    struct Pair { float cost; int ji; int vi; };  // ji, vi index into jobs / idle
    std::vector<Pair> pairs;
    pairs.reserve(idle.size() * jobs.size());
    for (int ji = 0; ji < static_cast<int>(jobs.size()); ++ji) {
        const Job& job = ctx.jobs[jobs[ji]];
        for (int vi = 0; vi < static_cast<int>(idle.size()); ++vi) {
            pairs.push_back({pickupCost(ctx.net, ctx.vehicles[idle[vi]].at, job.origin), ji, vi});
        }
    }
    // Cheapest pair first; ties by job order then vehicle id keep the result deterministic.
    std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) {
        if (a.cost != b.cost) return a.cost < b.cost;
        if (a.ji != b.ji) return a.ji < b.ji;
        return a.vi < b.vi;
    });

    std::vector<char> used_j(jobs.size(), 0), used_v(idle.size(), 0);
    int remaining = static_cast<int>(std::min(jobs.size(), idle.size()));
    for (const Pair& p : pairs) {
        if (remaining == 0) break;
        if (used_j[p.ji] || used_v[p.vi] || p.cost == kUnreachable) continue;
        used_j[p.ji] = 1;
        used_v[p.vi] = 1;
        out.push_back({jobs[p.ji], idle[p.vi]});
        --remaining;
    }
    return out;
}

const DispatchPolicy& defaultDispatchPolicy() {
    static const NearestVehiclePolicy kDefault;
    return kDefault;
}

}  // namespace sim
