#include "rail/FabTopology.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace rail {

// Deterministic synthetic fab: a 12-bay interbay ring (the long way around), each bay an intrabay
// port loop (no dead-end stubs, so OHTs circulate), and three cross-ring chords (shortcuts and the
// contention points). The bay-0 to bay-6 shortest path crosses a chord, the verified bottleneck.
RailNetwork buildSyntheticFab(FabDemo& demo) {
    RailNetwork net;

    const Vec2 center{600.0f, 450.0f};
    const int kBays = 12;             // interbay ring junctions, one bay each
    const float ring_r = 360.0f;      // interbay loop radius
    const float bay_offset = 80.0f;   // entry node sits this far outside the ring
    const float port_near = 45.0f;    // intrabay port-loop near/far radial offsets beyond the entry
    const float port_far = 130.0f;
    const float port_half = 55.0f;    // tangential half-width of the port loop
    const float kPi = 3.14159265358979323846f;

    // Interbay ring junctions (also the depots / spawn points; their names start with 'R').
    std::vector<NodeId> ring(kBays);
    for (int k = 0; k < kBays; ++k) {
        float ang = 2.0f * kPi * static_cast<float>(k) / static_cast<float>(kBays);
        Vec2 p{center.x + ring_r * std::cos(ang), center.y + ring_r * std::sin(ang)};
        ring[k] = net.addNode(p, false, "R" + std::to_string(k));
    }
    for (int k = 0; k < kBays; ++k) net.addSegment(ring[k], ring[(k + 1) % kBays]);

    // Each bay: an entry node plus a small intrabay loop of four ports E -> P0 -> P1 -> P2 -> P3 -> E.
    std::vector<NodeId> first_port(kBays);
    for (int k = 0; k < kBays; ++k) {
        float ang = 2.0f * kPi * static_cast<float>(k) / static_cast<float>(kBays);
        Vec2 outward{std::cos(ang), std::sin(ang)};
        Vec2 tangent{-std::sin(ang), std::cos(ang)};
        Vec2 epos{center.x + (ring_r + bay_offset) * outward.x,
                  center.y + (ring_r + bay_offset) * outward.y};
        NodeId entry = net.addNode(epos, false, "E" + std::to_string(k));
        net.addSegment(ring[k], entry);

        auto portPos = [&](float radial, float tang) {
            return Vec2{epos.x + radial * outward.x + tang * tangent.x,
                        epos.y + radial * outward.y + tang * tangent.y};
        };
        NodeId p0 = net.addNode(portPos(port_near, -port_half), true, "P" + std::to_string(k) + "_0");
        NodeId p1 = net.addNode(portPos(port_far, -port_half), true, "P" + std::to_string(k) + "_1");
        NodeId p2 = net.addNode(portPos(port_far, port_half), true, "P" + std::to_string(k) + "_2");
        NodeId p3 = net.addNode(portPos(port_near, port_half), true, "P" + std::to_string(k) + "_3");
        net.addSegment(entry, p0);
        net.addSegment(p0, p1);
        net.addSegment(p1, p2);
        net.addSegment(p2, p3);
        net.addSegment(p3, entry);
        first_port[k] = p0;
    }

    // Cross-ring chords: shortcuts between opposite junctions, the bottleneck contention points.
    const int opp = kBays / 2;
    SegmentId demo_chord = net.addSegment(ring[0], ring[opp], true, true);
    net.addSegment(ring[2], ring[(2 + opp) % kBays], true, true);
    net.addSegment(ring[4], ring[(4 + opp) % kBays], true, true);

    // Verify the bottleneck-prefers-chord invariant rather than assuming it: the chord must beat the
    // shorter ring detour between the same two junctions (both half-rings are equal for an even ring).
    float detour = 0.0f;
    for (int k = 0; k < opp; ++k) detour += net.distanceBetween(ring[k], ring[k + 1]);
    if (!(net.segments()[demo_chord].length < detour)) {
        std::fprintf(stderr, "warning: demo chord (%.1f) is not shorter than the ring detour (%.1f)\n",
                     net.segments()[demo_chord].length, detour);
    }

    demo.start = first_port[0];
    demo.goal = first_port[opp];
    return net;
}

}  // namespace rail
