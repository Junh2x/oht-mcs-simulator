#pragma once

#include "rail/RailNetwork.h"

#include <cstddef>
#include <deque>
#include <random>
#include <vector>

namespace sim {

using JobId = int;
using VehicleId = int;

class DispatchPolicy;  // STEP 3 dispatching strategy, defined in Dispatch.h

enum class JobState { Pending, ToPickup, Carrying, Done };
enum class VehState { Idle, ToPickup, Carrying };

struct Job {
    JobId id = -1;
    rail::NodeId origin = -1;   // pickup port
    rail::NodeId dest = -1;     // dropoff port
    JobState state = JobState::Pending;
    VehicleId vehicle = -1;
    float created = 0.0f;
    float assigned = 0.0f;
    float picked = 0.0f;
    float completed = 0.0f;
};

struct Vehicle {
    VehicleId id = -1;
    bool alive = true;
    bool retiring = false;          // marked to leave the fleet once it next goes idle
    VehState state = VehState::Idle;
    rail::NodeId at = -1;           // node the vehicle sits at, or the one it last departed
    JobId job = -1;
    std::vector<rail::NodeId> route_nodes;   // size is route_segs.size() + 1
    std::vector<rail::SegmentId> route_segs;
    std::size_t leg = 0;            // index of the current segment within route_segs
    float progress = 0.0f;          // [0,1] along the current segment
    float speed = 0.0f;
    rail::SegmentId held_seg = -1;  // segment currently occupied (reserved), -1 if sitting at a node
    bool blocked = false;           // could not enter its next segment this step (waiting at a node)
    rail::SegmentId want_seg = -1;  // segment it is waiting to enter while blocked
};

struct SimConfig {
    unsigned seed = 1;
    int oht_count = 10;
    float speed = 80.0f;            // world units per sim second
    float arrival_per_sec = 0.6f;   // job creation rate
};

struct SimStats {
    float sim_time = 0.0f;
    int vehicles_live = 0;
    int vehicles_target = 0;
    int vehicles_busy = 0;
    int vehicles_retiring = 0;
    int jobs_pending = 0;
    int jobs_active = 0;
    int jobs_done = 0;
    float throughput_per_min = 0.0f;
    float utilization = 0.0f;
    float avg_delivery = 0.0f;
    float p95_delivery = 0.0f;
    float empty_travel = 0.0f;       // cumulative empty (to-pickup) distance driven
    float loaded_travel = 0.0f;      // cumulative loaded (to-dest) distance driven
    float empty_ratio = 0.0f;        // empty / (empty + loaded): the deadhead share dispatching controls
    int vehicles_blocked = 0;        // waiting at a node for a segment this step
    int vehicles_deadlocked = 0;     // in a wait-for cycle (stays zero under avoidance)
};

class Simulation {
public:
    Simulation(const rail::RailNetwork& net, const SimConfig& cfg);

    void step(float dt);                 // advance by dt sim seconds; reads no wall clock
    void setTargetOhtCount(int n);
    void setArrivalRate(float per_sec);
    void setPolicy(const DispatchPolicy* p) { policy_ = p; }  // swap the dispatching strategy live
    void setAvoidance(bool on) { avoidance_ = on; }           // ON = deadlock-avoidance gate, OFF = greedy
    bool avoidance() const { return avoidance_; }

    const std::vector<Vehicle>& vehicles() const { return vehicles_; }
    rail::Vec2 vehicleWorldPos(const Vehicle& v) const;
    float segmentLoad(rail::SegmentId s) const { return static_cast<float>(seg_occupancy_[s]); }  // STEP 7 hook
    const std::vector<float>& segmentCongestion() const { return seg_congestion_; }  // occupancy EMA, for the heatmap
    int pendingCount() const { return static_cast<int>(pending_.size()); }  // queue size, for accounting checks
    SimStats stats() const;

private:
    void generateJobs(float dt);
    void assignJobs();
    void advanceVehicles(float dt);
    bool tryEnter(Vehicle& v, rail::SegmentId seg);                    // capacity-1 claim, plus safe gate when ON
    bool safeToEnter(const Vehicle& mover, rail::SegmentId next) const;  // banker-style safe-state check
    void releaseHeld(Vehicle& v);
    int countDeadlocked() const;                                       // vehicles trapped in a wait-for cycle
    void onArrival(Vehicle& v);
    void reconcileFleet();
    void recomputeOccupancy();
    void spawnVehicle();
    void setRoute(Vehicle& v, const rail::PathResult& route);
    void recordCompletion(float delivery);

    const rail::RailNetwork& net_;
    SimConfig cfg_;
    const DispatchPolicy* policy_ = nullptr;
    std::mt19937 rng_;

    float clock_ = 0.0f;
    int target_count_ = 0;
    VehicleId next_vehicle_id_ = 0;
    int spawn_rr_ = 0;
    float arrival_accum_ = 0.0f;

    std::vector<Vehicle> vehicles_;       // never erased; dead vehicles keep alive=false so id stays the index
    std::vector<Job> jobs_;               // id is the index; grows over the run (bounded batch runs are fine)
    std::deque<JobId> pending_;           // FIFO queue of unassigned jobs
    std::vector<int> seg_occupancy_;      // indexed by SegmentId, recomputed each step (render/cost)
    std::vector<VehicleId> seg_owner_;    // segment -> owning vehicle id, -1 if free (capacity 1)
    std::vector<float> seg_congestion_;   // per-segment occupancy EMA in [0,1] for the live heatmap
    bool avoidance_ = true;               // ON = safe-state admission gate; OFF = greedy (can deadlock)

    std::vector<rail::NodeId> depots_;    // ring junctions where vehicles spawn
    std::vector<rail::NodeId> ports_;     // candidate job origins and destinations

    int n_pending_ = 0;
    int n_active_ = 0;
    int n_done_ = 0;

    float empty_dist_ = 0.0f;   // accumulated to-pickup distance (deadhead)
    float loaded_dist_ = 0.0f;  // accumulated to-dest distance

    std::vector<float> recent_delivery_;  // bounded trailing window for avg and p95
    std::deque<float> recent_complete_;   // completion times inside the throughput window
};

}  // namespace sim
