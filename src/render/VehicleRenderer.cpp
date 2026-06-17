#include "render/VehicleRenderer.h"

namespace render {

void drawVehicles(ImDrawList* dl, const RailView& view,
                  const std::vector<VehicleMarker>& markers, const VehicleStyle& style) {
    const ImU32 col_idle = IM_COL32(120, 130, 140, 255);
    const ImU32 col_empty = IM_COL32(90, 200, 250, 255);
    const ImU32 col_carry = IM_COL32(250, 225, 80, 255);
    const ImU32 col_outline = IM_COL32(20, 20, 20, 200);

    for (const VehicleMarker& m : markers) {
        ImU32 fill = col_idle;
        if (m.state == 1) fill = col_empty;
        else if (m.state == 2) fill = col_carry;
        ImVec2 p = view.toScreen(m.pos);
        dl->AddCircleFilled(p, style.radius, fill);
        dl->AddCircle(p, style.radius, col_outline, 0, 1.5f);
    }
}

}  // namespace render
