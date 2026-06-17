#pragma once

#include <functional>
#include <string>
#include <vector>

namespace rail {

using NodeId = int;
using SegmentId = int;

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;
};

struct Node {
    NodeId id = -1;
    Vec2 pos;
    bool is_port = false;
    std::string name;
};

struct RailSegment {
    SegmentId id = -1;          // equals the index in segments(); ascending id is the STEP 4 resource order
    NodeId a = -1;
    NodeId b = -1;
    float length = 0.0f;        // base cost, derived from the two node coordinates
    bool bidirectional = true;  // one physical track, two directions (lets STEP 4 face head-on contention)
    bool is_bottleneck = false; // render highlight only
    float congestion = 0.0f;    // STEP 7 runtime load, owned here so one write reaches every router
};

// Adjacency entry. It references the shared RailSegment by id instead of copying its weight,
// so congestion and any future reservation live on a single object.
struct HalfEdge {
    SegmentId segment = -1;
    NodeId to = -1;
    int dir = 0;                // +1 travels a->b, -1 travels b->a (vehicle orientation for STEP 2)
};

struct PathResult {
    bool found = false;
    float distance = 0.0f;
    std::vector<NodeId> nodes;
    std::vector<SegmentId> segments;  // exact segments traversed; parallel rails make node pairs ambiguous
};

class RailNetwork {
public:
    NodeId addNode(Vec2 pos, bool is_port, std::string name);
    // Length is computed from the endpoint coordinates. Emits two half-edges if bidirectional, else one.
    SegmentId addSegment(NodeId a, NodeId b, bool bidirectional = true, bool is_bottleneck = false);

    const std::vector<Node>& nodes() const { return nodes_; }
    const std::vector<RailSegment>& segments() const { return segments_; }
    RailSegment& segment(SegmentId id) { return segments_[id]; }            // STEP 7 mutation point
    const std::vector<HalfEdge>& neighbors(NodeId n) const { return adjacency_[n]; }

    float distanceBetween(NodeId a, NodeId b) const;

private:
    std::vector<Node> nodes_;
    std::vector<RailSegment> segments_;             // single source of truth
    std::vector<std::vector<HalfEdge>> adjacency_;  // adjacency_[node] holds its outgoing half-edges
};

using EdgeCostFn = std::function<float(const RailSegment&)>;

inline float defaultEdgeCost(const RailSegment& s) { return s.length; }

// All edge costs must be non-negative (length >= 0, congestion >= 0) for Dijkstra to be valid.
PathResult dijkstra(const RailNetwork& net, NodeId start, NodeId goal,
                    const EdgeCostFn& cost = defaultEdgeCost);

}  // namespace rail
