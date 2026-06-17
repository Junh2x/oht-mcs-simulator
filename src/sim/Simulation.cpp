#include "sim/Simulation.h"
#include "sim/AssignmentPolicy.h"

#include <algorithm>
#include <cassert>
#include <utility>

namespace sim {

namespace {
constexpr float kThroughputWindowSec = 60.0f;
constexpr std::size_t kDeliverySamples = 256;
}  // namespace

Simulation::Simulation(const rail::RailNetwork& net, const SimConfig& cfg)
    : net_(net), cfg_(cfg), assign_(&nearestIdleAssignment), rng_(cfg.seed) {
    target_count_ = cfg.oht_count;
    seg_occupancy_.assign(net_.segments().size(), 0);

    // Depots are the ring junctions (their names start with 'R'); ports feed job origins and destinations.
    for (const rail::Node& nd : net_.nodes()) {
        if (nd.is_port) ports_.push_back(nd.id);
        else if (!nd.name.empty() && nd.name.front() == 'R') depots_.push_back(nd.id);
    }
    assert(!depots_.empty() && "Simulation needs at least one ring junction to spawn vehicles");

    for (int i = 0; i < cfg.oht_count; ++i) spawnVehicle();
}

void Simulation::spawnVehicle() {
    if (depots_.empty()) return;  // no valid spawn node (the constructor assert catches this in debug)
    Vehicle v;
    v.id = next_vehicle_id_++;
    v.state = VehState::Idle;
    v.speed = cfg_.speed;
    v.at = depots_[spawn_rr_++ % depots_.size()];
    vehicles_.push_back(std::move(v));
}

void Simulation::setTargetOhtCount(int n) { target_count_ = n < 0 ? 0 : n; }

void Simulation::setArrivalRate(float per_sec) { cfg_.arrival_per_sec = per_sec < 0.0f ? 0.0f : per_sec; }

void Simulation::step(float dt) {
    generateJobs(dt);
    assignJobs();
    advanceVehicles(dt);
    reconcileFleet();
    recomputeOccupancy();
    clock_ += dt;
    while (!recent_complete_.empty() && recent_complete_.front() < clock_ - kThroughputWindowSec) {
        recent_complete_.pop_front();
    }
}

void Simulation::generateJobs(float dt) {
    if (ports_.size() < 2) return;
    // Deterministic fractional accumulator. A true exponential inter-arrival is deferred because
    // exponential/poisson draw a variable, implementation-defined number of mt19937 values.
    arrival_accum_ += cfg_.arrival_per_sec * dt;
    std::uniform_int_distribution<std::size_t> pick(0, ports_.size() - 1);
    while (arrival_accum_ >= 1.0f) {
        arrival_accum_ -= 1.0f;
        std::size_t oi = pick(rng_);
        std::size_t di = pick(rng_);
        if (di == oi) di = (oi + 1) % ports_.size();
        Job job;
        job.id = static_cast<JobId>(jobs_.size());
        job.origin = ports_[oi];
        job.dest = ports_[di];
        job.created = clock_;
        jobs_.push_back(job);
        pending_.push_back(job.id);
        ++n_pending_;
    }
}

void Simulation::assignJobs() {
    while (!pending_.empty()) {
        JobId jid = pending_.front();
        VehicleId vid = assign_(vehicles_, jobs_[jid], net_);
        if (vid < 0) break;  // no idle vehicle available this step
        pending_.pop_front();

        Job& job = jobs_[jid];
        Vehicle& v = vehicles_[vid];
        rail::PathResult to_pickup = rail::dijkstra(net_, v.at, job.origin);
        pending_.pop_front();
        --n_pending_;
        if (!to_pickup.found) continue;  // defensive: skip an unreachable origin rather than stall a vehicle

        job.state = JobState::ToPickup;
        job.vehicle = vid;
        job.assigned = clock_;
        v.job = jid;
        v.state = VehState::ToPickup;
        setRoute(v, to_pickup);
        ++n_active_;
    }
}

void Simulation::setRoute(Vehicle& v, const rail::PathResult& route) {
    v.route_nodes = route.nodes;
    v.route_segs = route.segments;
    v.leg = 0;
    v.progress = 0.0f;
}

void Simulation::advanceVehicles(float dt) {
    for (Vehicle& v : vehicles_) {
        if (!v.alive || v.state == VehState::Idle) continue;
        float remaining = v.speed * dt;
        int transitions = 0;
        // Keep spending this step's distance budget across nodes and across a pickup or dropoff, so a
        // fast OHT never loses a substep at a boundary. Pickup to carry to done is at most two arrivals.
        while (remaining > 0.0f && v.alive && v.state != VehState::Idle) {
            while (remaining > 0.0f && v.leg < v.route_segs.size()) {
                float seg_len = std::max(net_.segments()[v.route_segs[v.leg]].length, 1e-4f);
                float dist_left = (1.0f - v.progress) * seg_len;
                if (remaining < dist_left) {
                    v.progress += remaining / seg_len;
                    remaining = 0.0f;
                } else {
                    remaining -= dist_left;
                    v.progress = 0.0f;
                    ++v.leg;
                }
            }
            if (v.leg < v.route_segs.size()) break;  // budget spent mid-segment
            onArrival(v);                            // installs the next route, goes idle, or retires
            if (++transitions > 2) break;            // guard against a degenerate zero-length route loop
        }
    }
}

void Simulation::onArrival(Vehicle& v) {
    v.at = v.route_nodes.empty() ? v.at : v.route_nodes.back();
    v.progress = 0.0f;
    v.leg = 0;

    if (v.state == VehState::ToPickup) {
        Job& job = jobs_[v.job];
        rail::PathResult to_dest = rail::dijkstra(net_, v.at, job.dest);
        if (!to_dest.found) {  // defensive: unreachable destination, release without a phantom completion
            job.state = JobState::Done;
            v.job = -1;
            v.route_nodes.clear();
            v.route_segs.clear();
            --n_active_;
            if (v.retiring) v.alive = false;
            else v.state = VehState::Idle;
            return;
        }
        job.state = JobState::Carrying;
        job.picked = clock_;
        v.state = VehState::Carrying;
        setRoute(v, to_dest);
    } else if (v.state == VehState::Carrying) {
        Job& job = jobs_[v.job];
        job.state = JobState::Done;
        job.completed = clock_;
        recordCompletion(job.completed - job.created);
        v.job = -1;
        v.route_nodes.clear();
        v.route_segs.clear();
        --n_active_;
        ++n_done_;
        if (v.retiring) v.alive = false;
        else v.state = VehState::Idle;
    }
}

void Simulation::recordCompletion(float delivery) {
    recent_delivery_.push_back(delivery);
    if (recent_delivery_.size() > kDeliverySamples) recent_delivery_.erase(recent_delivery_.begin());
    recent_complete_.push_back(clock_);
}

void Simulation::reconcileFleet() {
    int effective = 0;  // vehicles that will stay (alive and not retiring)
    for (const Vehicle& v : vehicles_) {
        if (v.alive && !v.retiring) ++effective;
    }

    if (effective < target_count_) {
        int need = target_count_ - effective;
        for (Vehicle& v : vehicles_) {  // un-retire before spawning new vehicles
            if (need == 0) break;
            if (v.alive && v.retiring) { v.retiring = false; --need; }
        }
        while (need-- > 0) spawnVehicle();
    } else if (effective > target_count_) {
        int surplus = effective - target_count_;
        for (Vehicle& v : vehicles_) {  // retire idle vehicles right away
            if (surplus == 0) break;
            if (v.alive && !v.retiring && v.state == VehState::Idle) { v.alive = false; --surplus; }
        }
        for (Vehicle& v : vehicles_) {  // never delete a busy vehicle: let it finish, then leave
            if (surplus == 0) break;
            if (v.alive && !v.retiring && v.state != VehState::Idle) { v.retiring = true; --surplus; }
        }
    }

    for (Vehicle& v : vehicles_) {  // a retiring vehicle that is now idle leaves the fleet
        if (v.alive && v.retiring && v.state == VehState::Idle) v.alive = false;
    }
}

void Simulation::recomputeOccupancy() {
    std::fill(seg_occupancy_.begin(), seg_occupancy_.end(), 0);
    for (const Vehicle& v : vehicles_) {
        if (v.alive && v.state != VehState::Idle && v.leg < v.route_segs.size()) {
            ++seg_occupancy_[v.route_segs[v.leg]];
        }
    }
}

rail::Vec2 Simulation::vehicleWorldPos(const Vehicle& v) const {
    if (v.state == VehState::Idle || v.leg >= v.route_segs.size() || v.route_nodes.size() < 2) {
        return net_.nodes()[v.at].pos;
    }
    const rail::Vec2& a = net_.nodes()[v.route_nodes[v.leg]].pos;
    const rail::Vec2& b = net_.nodes()[v.route_nodes[v.leg + 1]].pos;
    float t = v.progress;
    return rail::Vec2{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
}

SimStats Simulation::stats() const {
    SimStats s;
    s.sim_time = clock_;
    s.vehicles_target = target_count_;
    for (const Vehicle& v : vehicles_) {
        if (!v.alive) continue;
        ++s.vehicles_live;
        if (v.state != VehState::Idle) ++s.vehicles_busy;
        if (v.retiring) ++s.vehicles_retiring;
    }
    s.jobs_pending = n_pending_;
    s.jobs_active = n_active_;
    s.jobs_done = n_done_;
    s.throughput_per_min = static_cast<float>(recent_complete_.size());  // window is 60 s, so count is per minute
    s.utilization = s.vehicles_live > 0 ? static_cast<float>(s.vehicles_busy) / s.vehicles_live : 0.0f;

    if (!recent_delivery_.empty()) {
        double sum = 0.0;
        for (float d : recent_delivery_) sum += d;
        s.avg_delivery = static_cast<float>(sum / recent_delivery_.size());
        std::vector<float> tmp = recent_delivery_;
        std::size_t k = static_cast<std::size_t>(0.95 * (tmp.size() - 1));
        std::nth_element(tmp.begin(), tmp.begin() + k, tmp.end());
        s.p95_delivery = tmp[k];
    }
    return s;
}

}  // namespace sim
