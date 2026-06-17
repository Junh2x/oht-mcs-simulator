#pragma once

#include "imgui.h"
#include "rail/RailNetwork.h"

namespace render {

struct RailViewStyle {
    float pad = 20.0f;
    float node_r = 5.0f;
    float port_half = 6.0f;        // ports are drawn as squares of this half-size
    float seg_thick = 2.0f;
    float bottleneck_thick = 8.0f;  // wider than the path so a bottleneck on the path shows as an amber border
    float path_thick = 4.0f;
};

// Fits the whole network into the given screen rectangle and draws it with the window draw list.
// This is the only place that knows about ImGui types; the rail core stays renderer-free.
void drawRailNetwork(ImDrawList* dl, ImVec2 origin, ImVec2 size,
                     const rail::RailNetwork& net, const rail::PathResult& path,
                     const RailViewStyle& style = {});

}  // namespace render
