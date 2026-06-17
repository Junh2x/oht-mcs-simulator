#include "rail/FabTopology.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace rail {

RailNetwork buildSyntheticFab(FabDemo& demo) {
    RailNetwork net;

    const Vec2 center{400.0f, 300.0f};
    const float ring_r = 300.0f;    // hexagon inscribed in this radius: adjacent junctions are ring_r apart
    const float bay_offset = 90.0f; // entry node sits this far outside the ring
    const float port_stub = 35.0f;  // entry to port radial distance
    const float port_spread = 55.0f;
    const float kPi = 3.14159265358979323846f;

    // 6 interbay ring junctions (not ports).
    NodeId ring[6];
    for (int k = 0; k < 6; ++k) {
        float ang = kPi / 3.0f * static_cast<float>(k);  // 60 degrees per step
        Vec2 p{center.x + ring_r * std::cos(ang), center.y + ring_r * std::sin(ang)};
        ring[k] = net.addNode(p, false, "R" + std::to_string(k));
    }

    // Ring segments R0-R1 ... R5-R0: the longer alternate route around the chord.
    for (int k = 0; k < 6; ++k) {
        net.addSegment(ring[k], ring[(k + 1) % 6]);
    }

    // One bay per junction: an entry node radially outward plus 2 or 3 port stubs.
    NodeId first_port_of[6];
    for (int k = 0; k < 6; ++k) {
        float ang = kPi / 3.0f * static_cast<float>(k);
        Vec2 outward{std::cos(ang), std::sin(ang)};
        Vec2 tangent{-std::sin(ang), std::cos(ang)};
        Vec2 epos{center.x + (ring_r + bay_offset) * outward.x,
                  center.y + (ring_r + bay_offset) * outward.y};
        NodeId entry = net.addNode(epos, false, "E" + std::to_string(k));
        net.addSegment(ring[k], entry);

        int port_count = (k % 2 == 0) ? 3 : 2;  // mix 2 and 3 ports per bay for load/unload variety
        for (int j = 0; j < port_count; ++j) {
            float t = static_cast<float>(j) - static_cast<float>(port_count - 1) * 0.5f;
            Vec2 ppos{epos.x + outward.x * port_stub + tangent.x * t * port_spread,
                      epos.y + outward.y * port_stub + tangent.y * t * port_spread};
            NodeId port = net.addNode(ppos, true, "P" + std::to_string(k) + "_" + std::to_string(j));
            net.addSegment(entry, port);
            if (j == 0) first_port_of[k] = port;
        }
    }

    // Bottleneck: one short chord between opposite junctions R0 and R3.
    SegmentId chord = net.addSegment(ring[0], ring[3], true, true);

    // Verify the bottleneck-prefers-chord invariant rather than assuming it: the chord must beat
    // the shorter of the two half-ring detours the router could take instead (both ways around).
    float detour_cw = net.distanceBetween(ring[0], ring[1]) +
                      net.distanceBetween(ring[1], ring[2]) +
                      net.distanceBetween(ring[2], ring[3]);
    float detour_ccw = net.distanceBetween(ring[0], ring[5]) +
                       net.distanceBetween(ring[5], ring[4]) +
                       net.distanceBetween(ring[4], ring[3]);
    float detour = std::min(detour_cw, detour_ccw);
    if (!(net.segments()[chord].length < detour)) {
        std::fprintf(stderr,
                     "warning: bottleneck chord (%.1f) is not shorter than the shortest ring detour (%.1f)\n",
                     net.segments()[chord].length, detour);
    }

    demo.start = first_port_of[0];
    demo.goal = first_port_of[3];
    return net;
}

}  // namespace rail
