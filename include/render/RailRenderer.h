#pragma once

#include "imgui.h"
#include "rail/RailNetwork.h"
#include "render/RailView.h"

namespace render {

struct RailViewStyle {
    float pad = 20.0f;
    float node_r = 5.0f;
    float port_half = 6.0f;        // ports are drawn as squares of this half-size
    float seg_thick = 2.0f;
    float bottleneck_thick = 8.0f;  // wider than the path so a bottleneck on the path shows an amber border
    float path_thick = 4.0f;
};

// Draws the rail network with the given shared view. The rail core stays renderer-free; only this
// render layer knows about ImGui.
void drawRailNetwork(ImDrawList* dl, const RailView& view,
                     const rail::RailNetwork& net, const rail::PathResult& path,
                     const RailViewStyle& style = {});

}  // namespace render
