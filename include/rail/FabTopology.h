#pragma once

#include "rail/RailNetwork.h"

namespace rail {

// Demo ports for the console shortest-path query, plus a hot-lane pair chosen so its shortest path
// loads one interbay rail (leaving the parallel rail as the congestion-aware detour).
struct FabDemo {
    NodeId start = -1;
    NodeId goal = -1;
    NodeId hot_a = -1;
    NodeId hot_b = -1;
};

// Deterministic synthetic fab: an interbay hexagon ring (the long alternate route),
// intrabay bays with ports, and one short cross-ring chord (the bottleneck).
RailNetwork buildSyntheticFab(FabDemo& demo);

}  // namespace rail
