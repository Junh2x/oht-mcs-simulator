#include "rail/FabTopology.h"

#include <string>

namespace rail {

// Deterministic synthetic fab in the rectilinear style of a real 300mm wafer fab: a central interbay
// highway (a racetrack of two parallel rails joined at the ends, with a couple of cross-links) and
// intrabay bays hanging off it as ribs, above and below. Each bay is a small rectangular loop with
// four ports. OHTs cross the fab along the highway and dip into a bay only to serve its ports, which
// matches how interbay and intrabay transport are physically separated.
RailNetwork buildSyntheticFab(FabDemo& demo) {
    RailNetwork net;

    constexpr int COLS = 6;
    const float ox = 110.0f, col_w = 200.0f;
    const float top_y = 360.0f, bot_y = 560.0f;   // the two interbay rails
    const float spur = 55.0f, bay_half = 70.0f, bay_h = 80.0f, bay_gap = 22.0f;

    // Interbay highway: two horizontal rails of COLS+1 junctions, joined into a racetrack at both ends.
    NodeId HT[COLS + 1], HB[COLS + 1];
    for (int i = 0; i <= COLS; ++i) {
        HT[i] = net.addNode({ox + i * col_w, top_y}, false, "HT" + std::to_string(i));
        HB[i] = net.addNode({ox + i * col_w, bot_y}, false, "HB" + std::to_string(i));
    }
    for (int i = 0; i < COLS; ++i) {
        net.addSegment(HT[i], HT[i + 1]);
        net.addSegment(HB[i], HB[i + 1]);
    }
    net.addSegment(HT[0], HB[0]);          // left and right end connectors close the racetrack
    net.addSegment(HT[COLS], HB[COLS]);
    net.addSegment(HT[2], HB[2]);          // internal cross-links: grid connectivity and contention
    net.addSegment(HT[4], HB[4]);

    // A bay is a rectangular loop on a spur off a highway junction. out_dir = -1 hangs it above the
    // top rail, +1 below the bottom rail. Returns the bay's first port.
    auto build_bay = [&](NodeId hub, float cx, float hub_y, float out_dir, const std::string& tag) {
        float ey = hub_y + out_dir * spur;
        NodeId e = net.addNode({cx, ey}, false, "E" + tag);
        net.addSegment(hub, e);
        float y0 = ey + out_dir * bay_gap;
        float y1 = ey + out_dir * (bay_gap + bay_h);
        NodeId p0 = net.addNode({cx - bay_half, y0}, true, "P" + tag + "_0");
        NodeId p1 = net.addNode({cx - bay_half, y1}, true, "P" + tag + "_1");
        NodeId p2 = net.addNode({cx + bay_half, y1}, true, "P" + tag + "_2");
        NodeId p3 = net.addNode({cx + bay_half, y0}, true, "P" + tag + "_3");
        net.addSegment(e, p0);
        net.addSegment(p0, p1);
        net.addSegment(p1, p2);
        net.addSegment(p2, p3);
        net.addSegment(p3, e);
        return p0;
    };

    NodeId top_first[COLS], bot_first[COLS];
    for (int c = 0; c < COLS; ++c) {
        float cx = ox + c * col_w;
        top_first[c] = build_bay(HT[c], cx, top_y, -1.0f, "T" + std::to_string(c));
        bot_first[c] = build_bay(HB[c], cx, bot_y, +1.0f, "B" + std::to_string(c));
    }

    // Demo query: a port in the top-left bay to a port in the bottom-right bay, crossing the highway.
    demo.start = top_first[0];
    demo.goal = bot_first[COLS - 1];
    // Hot lane crosses the highway through the single HT2-HB2 cross-link, so detouring via the ends
    // or the other cross-link is the congestion-aware alternative.
    demo.hot_a = top_first[2];
    demo.hot_b = bot_first[2];
    return net;
}

}  // namespace rail
