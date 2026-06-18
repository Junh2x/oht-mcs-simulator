#include "sim/Simulation.h"
#include "sim/Dispatch.h"

#include <algorithm>
#include <cassert>
#include <unordered_set>
#include <utility>

namespace sim {

namespace {
constexpr float kThroughputWindowSec = 60.0f;
constexpr std::size_t kDeliverySamples = 256;
constexpr int kDispatchWindow = 24;  // bounded look-ahead: jobs near the front a policy may reorder
}  // namespace

Simulation::Simulation(const rail::RailNetwork& net, const SimConfig& cfg)
    : net_(net), cfg_(cfg), policy_(&defaultDispatchPolicy()), rng_(cfg.seed) {
    target_count_ = cfg.oht_count;
    seg_occupancy_.assign(net_.segments().size(), 0);
    seg_owner_.assign(net_.segments().size(), -1);
    seg_congestion_.assign(net_.segments().size(), 0.0f);

    // Ports feed job origins and destinations; every other (junction) node can host a spawned vehicle.
    for (const rail::Node& nd : net_.nodes()) {
        if (nd.is_port) ports_.push_back(nd.id);
        else depots_.push_back(nd.id);
    }
    assert(!depots_.empty() && "Simulation needs at least one junction to spawn vehicles");

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

void Simulation::setTargetOhtCount(int n) {
    target_count_ = n < 0 ? 0 : n;
    reconcileFleet();  // apply right away so the slider responds even while paused or at zero speed
}

void Simulation::setArrivalRate(float per_sec) { cfg_.arrival_per_sec = per_sec < 0.0f ? 0.0f : per_sec; }

void Simulation::step(float dt) {
    generateJobs(dt);
    assignJobs();
    advanceVehicles(dt);
    reconcileFleet();
    recomputeOccupancy();

    // Smooth each segment's occupancy into a [0,1] congestion estimate for the heatmap (and STEP 7).
    const float kCongAlpha = 0.02f;
    for (std::size_t s = 0; s < seg_congestion_.size(); ++s) {
        float occ = seg_occupancy_[s] > 0 ? 1.0f : 0.0f;
        seg_congestion_[s] += kCongAlpha * (occ - seg_congestion_[s]);
    }

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
    std::uniform_real_distribution<float> coin(0.0f, 1.0f);
    bool hot_lane = cfg_.hot_fraction > 0.0f && cfg_.hot_origin >= 0 && cfg_.hot_dest >= 0;
    while (arrival_accum_ >= 1.0f) {
        arrival_accum_ -= 1.0f;
        Job job;
        job.id = static_cast<JobId>(jobs_.size());
        // A share of jobs follows the hot lane (concentrated demand); the rest are random. The coin is
        // only drawn when a hot lane is set, so random-only runs keep their exact RNG sequence.
        if (hot_lane && coin(rng_) < cfg_.hot_fraction) {
            job.origin = cfg_.hot_origin;
            job.dest = cfg_.hot_dest;
        } else {
            std::size_t oi = pick(rng_);
            std::size_t di = pick(rng_);
            if (di == oi) di = (oi + 1) % ports_.size();
            job.origin = ports_[oi];
            job.dest = ports_[di];
        }
        job.created = clock_;
        jobs_.push_back(job);
        pending_.push_back(job.id);
        ++n_pending_;
    }
}

rail::PathResult Simulation::route(rail::NodeId from, rail::NodeId to) const {
    if (lambda_ <= 0.0f) return rail::dijkstra(net_, from, to);  // static shortest path
    // Congestion-aware cost: detour around busy segments. Weight stays positive, so Dijkstra is valid.
    rail::EdgeCostFn cost = [this](const rail::RailSegment& s) {
        return s.length * (1.0f + lambda_ * seg_congestion_[s.id]);
    };
    return rail::dijkstra(net_, from, to, cost);
}

float Simulation::pathLength(const rail::PathResult& p) const {
    float len = 0.0f;
    for (rail::SegmentId s : p.segments) len += net_.segments()[s].length;
    return len;  // physical distance, independent of the routing cost weights
}

void Simulation::assignJobs() {
    if (policy_ == nullptr || pending_.empty()) return;

    // The policy chooses job->vehicle pairs; the Simulation owns the side effects (routes, queue, stats).
    DispatchContext ctx{vehicles_, jobs_, pending_, net_, kDispatchWindow};
    std::vector<DispatchAssignment> picks = policy_->assign(ctx);
    if (picks.empty()) return;

    std::unordered_set<JobId> removed;
    for (const DispatchAssignment& a : picks) {
        Job& job = jobs_[a.job];
        Vehicle& v = vehicles_[a.vehicle];
        rail::PathResult to_pickup = route(v.at, job.origin);
        removed.insert(a.job);
        if (!to_pickup.found) continue;  // synthetic fab is connected; defensive only

        job.state = JobState::ToPickup;
        job.vehicle = a.vehicle;
        job.assigned = clock_;
        v.job = a.job;
        v.state = VehState::ToPickup;
        setRoute(v, to_pickup);
        empty_dist_ += pathLength(to_pickup);
        ++n_active_;
    }

    // A policy may skip jobs (bounded look-ahead), so drop assigned ids while keeping queue order.
    std::deque<JobId> rest;
    for (JobId jid : pending_) {
        if (removed.find(jid) == removed.end()) rest.push_back(jid);
    }
    pending_.swap(rest);
    n_pending_ = static_cast<int>(pending_.size());
}

void Simulation::setRoute(Vehicle& v, const rail::PathResult& route) {
    v.route_nodes = route.nodes;
    v.route_segs = route.segments;
    v.leg = 0;
    v.progress = 0.0f;
}

void Simulation::advanceVehicles(float dt) {
    for (Vehicle& v : vehicles_) {
        v.blocked = false;
        v.want_seg = -1;
        if (!v.alive || v.state == VehState::Idle) continue;
        if (v.route_segs.empty()) { onArrival(v); continue; }  // already at the target node: do the handoff now

        float remaining = v.speed * dt;
        int transitions = 0;
        // Spend this step's distance budget, claiming each segment before entering it. A claim is
        // refused when the segment is taken (capacity 1) or, under avoidance, when entering it would
        // strand someone (unsafe state); the vehicle then waits at its node holding what it has.
        while (remaining > 0.0f && v.alive && v.state != VehState::Idle && transitions < 16) {
            if (v.held_seg < 0) {  // waiting at a node to enter the current leg's segment
                rail::SegmentId want = v.route_segs[v.leg];
                if (!tryEnter(v, want)) { v.blocked = true; v.want_seg = want; break; }
                v.held_seg = want;
                v.progress = 0.0f;
            }

            float seg_len = std::max(net_.segments()[v.held_seg].length, 1e-4f);
            float dist_left = (1.0f - v.progress) * seg_len;
            if (remaining < dist_left) {
                v.progress += remaining / seg_len;
                remaining = 0.0f;
                break;
            }
            remaining -= dist_left;
            v.progress = 1.0f;  // reached the far node of the held segment

            if (v.leg + 1 >= v.route_segs.size()) {  // arrived at the route's end (pickup or dropoff)
                releaseHeld(v);
                onArrival(v);   // installs the carry route (held stays -1), or goes idle/retires
                ++transitions;
                continue;
            }
            rail::SegmentId next = v.route_segs[v.leg + 1];
            if (!tryEnter(v, next)) { v.blocked = true; v.want_seg = next; break; }  // wait, still holding current
            releaseHeld(v);
            v.leg += 1;
            v.held_seg = next;
            v.progress = 0.0f;
            ++transitions;
        }
    }
}

bool Simulation::tryEnter(Vehicle& v, rail::SegmentId seg) {
    if (seg_owner_[seg] != -1 && seg_owner_[seg] != v.id) return false;  // capacity 1
    if (avoidance_ && !safeToEnter(v, seg)) return false;               // refuse moves into unsafe states
    seg_owner_[seg] = v.id;
    return true;
}

void Simulation::releaseHeld(Vehicle& v) {
    if (v.held_seg >= 0) {
        seg_owner_[v.held_seg] = -1;
        v.held_seg = -1;
    }
}

// Banker-style safe-state test. Tentatively move `mover` onto `next`, then ask: is there an order in
// which every in-flight vehicle can drive its whole remaining route to the end and vacate? A vehicle
// can finish when none of its remaining segments is occupied by another unfinished vehicle; let it
// finish, free its segment, and repeat. Idle vehicles hold nothing and are ignored. Admitting only
// moves that keep a completion order available means the fleet never enters a deadlock.
bool Simulation::safeToEnter(const Vehicle& mover, rail::SegmentId next) const {
    struct M { const Vehicle* v; int remain_start; rail::SegmentId own; bool done; };
    std::vector<M> ms;
    ms.reserve(vehicles_.size());
    for (const Vehicle& v : vehicles_) {
        if (!v.alive || v.state == VehState::Idle || v.route_segs.empty()) continue;
        int remain_start;
        rail::SegmentId own;
        if (&v == &mover) {  // tentative: mover releases its current segment and occupies `next`
            remain_start = (v.held_seg >= 0) ? static_cast<int>(v.leg) + 1 : static_cast<int>(v.leg);
            own = next;
        } else {
            remain_start = static_cast<int>(v.leg);
            own = v.held_seg;  // -1 when it is waiting at a node holding nothing
        }
        ms.push_back({&v, remain_start, own, false});
    }

    std::vector<int> occ(seg_owner_.size(), 0);  // segments held by still-unfinished vehicles
    for (const M& m : ms) if (m.own >= 0) ++occ[m.own];

    int remaining = static_cast<int>(ms.size());
    bool found = true;
    int guard = 0;
    while (found && remaining > 0 && guard++ < 100000) {
        found = false;
        for (M& m : ms) {
            if (m.done) continue;
            const std::vector<rail::SegmentId>& rs = m.v->route_segs;
            bool clear = true;  // whole remaining route free of other vehicles' held segments
            for (int i = m.remain_start; i < static_cast<int>(rs.size()); ++i) {
                rail::SegmentId s = rs[i];
                if (occ[s] > 0 && s != m.own) { clear = false; break; }
            }
            if (clear) {
                if (m.own >= 0) --occ[m.own];
                m.done = true;
                --remaining;
                found = true;
            }
        }
    }
    return remaining == 0;
}

int Simulation::countDeadlocked() const {
    int n = static_cast<int>(vehicles_.size());
    std::vector<int> nxt(n, -1);  // blocked vehicle -> owner of the segment it waits on
    for (const Vehicle& v : vehicles_) {
        if (v.alive && v.blocked && v.want_seg >= 0) {
            VehicleId o = seg_owner_[v.want_seg];
            if (o >= 0 && o != v.id) nxt[v.id] = o;
        }
    }
    std::vector<int> color(n, 0);   // 0 unvisited, 1 on current walk, 2 settled
    std::vector<char> oncycle(n, 0);
    for (int s = 0; s < n; ++s) {
        if (color[s] != 0 || nxt[s] < 0) continue;
        std::vector<int> stack;
        int u = s;
        while (u >= 0 && color[u] == 0) {
            color[u] = 1;
            stack.push_back(u);
            u = nxt[u];
        }
        if (u >= 0 && color[u] == 1) {  // closed a cycle at u; mark from the top of the walk back to u
            for (int i = static_cast<int>(stack.size()) - 1; i >= 0; --i) {
                oncycle[stack[i]] = 1;
                if (stack[i] == u) break;
            }
        }
        for (int x : stack) color[x] = 2;
    }
    int cnt = 0;
    for (int i = 0; i < n; ++i) if (oncycle[i]) ++cnt;
    return cnt;
}

void Simulation::onArrival(Vehicle& v) {
    v.at = v.route_nodes.empty() ? v.at : v.route_nodes.back();
    v.progress = 0.0f;
    v.leg = 0;

    if (v.state == VehState::ToPickup) {
        Job& job = jobs_[v.job];
        rail::PathResult to_dest = route(v.at, job.dest);
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
        loaded_dist_ += pathLength(to_dest);
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
    total_cycle_ += delivery;
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
        if (v.alive && v.held_seg >= 0) ++seg_occupancy_[v.held_seg];
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
    for (const Vehicle& v : vehicles_) {
        if (v.alive && v.blocked) ++s.vehicles_blocked;
    }
    s.vehicles_deadlocked = countDeadlocked();
    s.throughput_per_min = static_cast<float>(recent_complete_.size());  // window is 60 s, so count is per minute
    s.utilization = s.vehicles_live > 0 ? static_cast<float>(s.vehicles_busy) / s.vehicles_live : 0.0f;
    s.empty_travel = empty_dist_;
    s.loaded_travel = loaded_dist_;
    float travel = empty_dist_ + loaded_dist_;
    s.empty_ratio = travel > 0.0f ? empty_dist_ / travel : 0.0f;

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
