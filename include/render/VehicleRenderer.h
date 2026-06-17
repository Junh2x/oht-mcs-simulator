#pragma once

#include "imgui.h"
#include "rail/RailNetwork.h"
#include "render/RailView.h"

#include <vector>

namespace render {

// A renderer-side snapshot of one OHT. main.cpp fills these from the sim, so render never includes sim.
struct VehicleMarker {
    rail::Vec2 pos;
    int state = 0;  // 0 idle, 1 moving to pickup, 2 carrying
};

struct VehicleStyle {
    float radius = 6.0f;
};

void drawVehicles(ImDrawList* dl, const RailView& view,
                  const std::vector<VehicleMarker>& markers, const VehicleStyle& style = {});

}  // namespace render
