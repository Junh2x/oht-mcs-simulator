#include "render/RailRenderer.h"

#include <vector>

namespace render {

void drawRailNetwork(ImDrawList* dl, const RailView& view,
                     const rail::RailNetwork& net, const rail::PathResult& path,
                     const RailViewStyle& style) {
    const auto& nodes = net.nodes();
    const auto& segments = net.segments();
    if (nodes.empty()) return;

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
        dl->AddLine(view.toScreen(nodes[s.a].pos), view.toScreen(nodes[s.b].pos), col_seg, style.seg_thick);
    }
    for (const rail::RailSegment& s : segments) {
        if (s.is_bottleneck) {
            dl->AddLine(view.toScreen(nodes[s.a].pos), view.toScreen(nodes[s.b].pos),
                        col_bottleneck, style.bottleneck_thick);
        }
    }
    for (const rail::RailSegment& s : segments) {
        if (on_path[s.id]) {
            dl->AddLine(view.toScreen(nodes[s.a].pos), view.toScreen(nodes[s.b].pos), col_path, style.path_thick);
        }
    }

    for (const rail::Node& nd : nodes) {
        ImVec2 p = view.toScreen(nd.pos);
        if (nd.is_port) {
            dl->AddRectFilled(ImVec2(p.x - style.port_half, p.y - style.port_half),
                              ImVec2(p.x + style.port_half, p.y + style.port_half), col_port);
        } else {
            dl->AddCircleFilled(p, style.node_r, col_junction);
        }
    }

    if (path.found && !path.nodes.empty()) {
        float ring_r = style.port_half + style.node_r;
        dl->AddCircle(view.toScreen(nodes[path.nodes.front()].pos), ring_r, col_endpoint, 0, style.seg_thick);
        dl->AddCircle(view.toScreen(nodes[path.nodes.back()].pos), ring_r, col_endpoint, 0, style.seg_thick);
    }
}

}  // namespace render
