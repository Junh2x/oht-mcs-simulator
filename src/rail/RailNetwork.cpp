#include "rail/RailNetwork.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace rail {

NodeId RailNetwork::addNode(Vec2 pos, bool is_port, std::string name) {
    NodeId id = static_cast<NodeId>(nodes_.size());
    nodes_.push_back(Node{id, pos, is_port, std::move(name)});
    adjacency_.emplace_back();
    return id;
}

SegmentId RailNetwork::addSegment(NodeId a, NodeId b, bool bidirectional, bool is_bottleneck) {
    SegmentId id = static_cast<SegmentId>(segments_.size());
    RailSegment s;  // named fields, so reordering or adding members later cannot silently misassign
    s.id = id;
    s.a = a;
    s.b = b;
    s.length = distanceBetween(a, b);
    s.bidirectional = bidirectional;
    s.is_bottleneck = is_bottleneck;  // congestion keeps its in-class default of 0
    segments_.push_back(s);

    adjacency_[a].push_back(HalfEdge{id, b, +1});
    if (bidirectional) {
        adjacency_[b].push_back(HalfEdge{id, a, -1});
    }
    return id;
}

float RailNetwork::distanceBetween(NodeId a, NodeId b) const {
    const Vec2& pa = nodes_[a].pos;
    const Vec2& pb = nodes_[b].pos;
    return std::hypot(pb.x - pa.x, pb.y - pa.y);
}

PathResult dijkstra(const RailNetwork& net, NodeId start, NodeId goal, const EdgeCostFn& cost) {
    const std::size_t n = net.nodes().size();
    if (start < 0 || goal < 0 ||
        static_cast<std::size_t>(start) >= n || static_cast<std::size_t>(goal) >= n) {
        return PathResult{};  // invalid query returns found=false instead of indexing out of bounds
    }
    const float kInf = std::numeric_limits<float>::infinity();

    std::vector<float> dist(n, kInf);
    std::vector<NodeId> prev_node(n, -1);
    std::vector<SegmentId> prev_seg(n, -1);

    using PQItem = std::pair<float, NodeId>;  // (distance, node)
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<>> pq;  // min-heap, not the default max-heap

    dist[start] = 0.0f;
    pq.push({0.0f, start});

    while (!pq.empty()) {
        float d = pq.top().first;
        NodeId u = pq.top().second;
        pq.pop();
        if (d > dist[u]) continue;  // stale entry left from an earlier, longer relaxation
        if (u == goal) break;

        for (const HalfEdge& h : net.neighbors(u)) {
            float w = cost(net.segments()[h.segment]);
            float nd = dist[u] + w;
            if (nd < dist[h.to]) {
                dist[h.to] = nd;
                prev_node[h.to] = u;
                prev_seg[h.to] = h.segment;
                pq.push({nd, h.to});
            }
        }
    }

    PathResult result;
    if (dist[goal] == kInf) return result;  // unreachable

    result.found = true;
    result.distance = dist[goal];
    for (NodeId at = goal; at != -1; at = prev_node[at]) {
        result.nodes.push_back(at);
        if (prev_seg[at] != -1) result.segments.push_back(prev_seg[at]);
    }
    std::reverse(result.nodes.begin(), result.nodes.end());
    std::reverse(result.segments.begin(), result.segments.end());
    return result;
}

}  // namespace rail
