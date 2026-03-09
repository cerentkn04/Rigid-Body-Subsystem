#pragma once
#include <vector>
#include <unordered_map>
#include <cstring>
#include <cmath>
#include "RegionType.hpp"
#include <stdio.h>
namespace rigid {

template<typename CellType>
struct MotionSystemState {
    std::vector<CellType> back_buffer;
};

namespace MotionSystem {

    template<typename CellType>
    void Apply(
        MotionSystemState<CellType>& state,
        std::vector<CellType>& pixel_grid, 
        const std::vector<uint32_t>& label_grid,
        std::unordered_map<RegionID, RegionRecord>& active_regions,
        int width, 
        int height,
        CellType empty_cell_template) 
    {
        if (state.back_buffer.size() != pixel_grid.size()) {
            state.back_buffer.resize(pixel_grid.size());
        }
// 1. FRESH COPY
std::memcpy(state.back_buffer.data(), pixel_grid.data(), pixel_grid.size() * sizeof(CellType));

for (auto& [id, region] : active_regions) {
    if (!region.is_dynamic) {
    region.prev_center_f.x = region.center_f.x;
    region.prev_center_f.y = region.center_f.y;
        continue;
    }

    int dx = (int)std::round(region.center_f.x - region.prev_center_f.x);
    int dy = (int)std::round(region.center_f.y - region.prev_center_f.y);

    if (dx == 0 && dy == 0) continue;

    // STEP A: CLEAR OLD POSITION IN BACK BUFFER
    for (int y = region.bounds.min_y; y <= region.bounds.max_y; ++y) {
        int row = y * width;
        for (int x = region.bounds.min_x; x <= region.bounds.max_x; ++x) {
            if (label_grid[row + x] == id) {
                state.back_buffer[row + x] = empty_cell_template;
            }
        }
    }

    // STEP B: WRITE TO NEW POSITION IN BACK BUFFER
    // We read from 'pixel_grid' (old truth) and write to 'back_buffer' (new truth)
    for (int y = region.bounds.min_y; y <= region.bounds.max_y; ++y) {
        int old_row = y * width;
        int ny = y + dy;
        if (ny < 0 || ny >= height) continue;
        int new_row = ny * width;

        for (int x = region.bounds.min_x; x <= region.bounds.max_x; ++x) {
            if (label_grid[old_row + x] == id) {
                int nx = x + dx;
                if (nx >= 0 && nx < width) {
                    state.back_buffer[new_row + nx] = pixel_grid[old_row + x];
                }
            }
        }
    }

    // UPDATE METADATA
    region.prev_center_f.x = region.center_f.x;
    region.bounds.min_x += dx; region.bounds.max_x += dx;
    region.prev_center_f.y = region.center_f.y;
    region.bounds.min_y += dy; region.bounds.max_y += dy;
}
// SWAP
pixel_grid.swap(state.back_buffer);
         }

} // namespace MotionSystem
} // namespace rigid
