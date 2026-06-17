#include "render/RailRenderer.h"

#include <algorithm>
#include <vector>

namespace render {

void drawRailNetwork(ImDrawList* dl, ImVec2 origin, ImVec2 size,
                     const rail::RailNetwork& net, const rail::PathResult& path,
                     const RailViewStyle& style) {
    const auto& nodes = net.nodes();
    const auto& segments = net.segments();
    if (nodes.empty()) return;

    // World bounding box, recomputed every frame so a window resize keeps the fit correct.
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
    float scale = std::min((size.x - 2.0f * style.pad) / ext_x,
                           (size.y - 2.0f * style.pad) / ext_y);
    if (scale <= 0.0f) scale = 1.0f;

    // World y grows upward like a fab map, so flip it when mapping to screen space.
    auto toScreen = [&](rail::Vec2 w) {
        return ImVec2(origin.x + style.pad + (w.x - min_x) * scale,
                      origin.y + style.pad + (max_y - w.y) * scale);
    };

    const ImU32 col_seg = IM_COL32(120, 130, 140, 255);
    const ImU32 col_bottleneck = IM_COL32(240, 170, 60, 255);
    const ImU32 col_path = IM_COL32(70, 220, 160, 255);
    const ImU32 col_junction = IM_COL32(150, 160, 170, 255);
    const ImU32 col_port = IM_COL32(90, 170, 240, 255);
    const ImU32 col_endpoint = IM_COL32(250, 240, 120, 255);

    // Mark path segments for an O(path) membership test that works even with parallel rails.
    std::vector<unsigned char> on_path(segments.size(), 0);
    for (rail::SegmentId sid : path.segments) on_path[sid] = 1;

    // Painter's order, back to front: base segments, bottleneck overlay, path overlay, then nodes.
    for (const rail::RailSegment& s : segments) {
        dl->AddLine(toScreen(nodes[s.a].pos), toScreen(nodes[s.b].pos), col_seg, style.seg_thick);
    }
    for (const rail::RailSegment& s : segments) {
        if (s.is_bottleneck) {
            dl->AddLine(toScreen(nodes[s.a].pos), toScreen(nodes[s.b].pos),
                        col_bottleneck, style.bottleneck_thick);
        }
    }
    for (const rail::RailSegment& s : segments) {
        if (on_path[s.id]) {
            dl->AddLine(toScreen(nodes[s.a].pos), toScreen(nodes[s.b].pos), col_path, style.path_thick);
        }
    }

    for (const rail::Node& nd : nodes) {
        ImVec2 p = toScreen(nd.pos);
        if (nd.is_port) {
            dl->AddRectFilled(ImVec2(p.x - style.port_half, p.y - style.port_half),
                              ImVec2(p.x + style.port_half, p.y + style.port_half), col_port);
        } else {
            dl->AddCircleFilled(p, style.node_r, col_junction);
        }
    }

    // Mark the queried path endpoints with an extra ring so the demo reads at a glance.
    if (path.found && !path.nodes.empty()) {
        float ring_r = style.port_half + style.node_r;
        dl->AddCircle(toScreen(nodes[path.nodes.front()].pos), ring_r, col_endpoint, 0, style.seg_thick);
        dl->AddCircle(toScreen(nodes[path.nodes.back()].pos), ring_r, col_endpoint, 0, style.seg_thick);
    }
}

}  // namespace render
