#pragma once

#include "rail/RailNetwork.h"

namespace rail {

// The two demo ports whose shortest path must cross the bottleneck chord (STEP 1 console query).
struct FabDemo {
    NodeId start = -1;
    NodeId goal = -1;
};

// Deterministic synthetic fab: an interbay hexagon ring (the long alternate route),
// intrabay bays with ports, and one short cross-ring chord (the bottleneck).
RailNetwork buildSyntheticFab(FabDemo& demo);

}  // namespace rail
