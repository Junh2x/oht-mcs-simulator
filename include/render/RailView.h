#pragma once

#include "imgui.h"
#include "rail/RailNetwork.h"

#include <algorithm>

namespace render {

// Maps fab world coordinates into a screen rectangle. Shared by the rail and vehicle renderers
// so they never drift apart. World y grows upward like a fab map, so it is flipped to screen space.
struct RailView {
    ImVec2 origin{0.0f, 0.0f};
    float pad = 0.0f;
    float scale = 1.0f;
    float min_x = 0.0f;
    float max_y = 0.0f;

    ImVec2 toScreen(rail::Vec2 w) const {
        return ImVec2(origin.x + pad + (w.x - min_x) * scale,
                      origin.y + pad + (max_y - w.y) * scale);
    }
};

inline RailView makeRailView(ImVec2 origin, ImVec2 size, const rail::RailNetwork& net, float pad) {
    RailView v;
    v.origin = origin;
    v.pad = pad;
    const auto& nodes = net.nodes();
    if (nodes.empty()) return v;

    float min_x = nodes[0].pos.x, max_x = nodes[0].pos.x;
    float min_y = nodes[0].pos.y, max_y = nodes[0].pos.y;
    for (const rail::Node& nd : nodes) {
        min_x = std::min(min_x, nd.pos.x);
        max_x = std::max(max_x, nd.pos.x);
        min_y = std::min(min_y, nd.pos.y);
        max_y = std::max(max_y, nd.pos.y);
    }
    float ext_x = std::max(max_x - min_x, 1.0f);
    float ext_y = std::max(max_y - min_y, 1.0f);
    float scale = std::min((size.x - 2.0f * pad) / ext_x, (size.y - 2.0f * pad) / ext_y);
    if (scale <= 0.0f) scale = 1.0f;

    v.scale = scale;
    v.min_x = min_x;
    v.max_y = max_y;
    return v;
}

}  // namespace render
