/*#include <RegionMotion.hpp>
#include <cstring>
#include <cmath>

namespace rigid {
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
    // 1. Ensure buffer size
    if (state.back_buffer.size() != pixel_grid.size()) {
        state.back_buffer.resize(pixel_grid.size());
    }

    // 2. Step A: Initial Copy (authoritative world -> scratchpad)
    // DOD rule: Use fast memory operations for bulk data
    std::memcpy(state.back_buffer.data(), pixel_grid.data(), pixel_grid.size() * sizeof(CellType));

    // 3. Step B: Process Regions (Data-driven loop)
    for (auto& [id, region] : active_regions) {
        if (!region.is_dynamic) {
        region.prev_center_f.x = region.center_f.x;
        region.prev_center_f.y = region.center_f.y;
            continue;
        }

        // Calculate discrete integer delta from physics float positions
        int dx = static_cast<int>(std::round(region.center_f.x - region.prev_center_f.x));
        int dy = static_cast<int>(std::round(region.center_f.y - region.prev_center_f.y));

        // Skip regions that haven't moved a full pixel (Jitter filter)
        if (dx == 0 && dy == 0) continue;

        // I. Clear the OLD pixels in the back_buffer first
        // This prevents the region from leaving a trail behind
        for (int y = region.bounds.min_y; y <= region.bounds.max_y; ++y) {
            int row_start = y * width;
            for (int x = region.bounds.min_x; x <= region.bounds.max_x; ++x) {
                int idx = row_start + x;
                if (label_grid[idx] == id) {
                    state.back_buffer[idx] = empty_cell_template;
                }
            }
        }

        // II. Write the pixels to the NEW location in the back_buffer
        for (int y = region.bounds.min_y; y <= region.bounds.max_y; ++y) {
            int old_row_start = y * width;
            int new_y = y + dy;
            
            if (new_y < 0 || new_y >= height) continue;
            int new_row_start = new_y * width;

            for (int x = region.bounds.min_x; x <= region.bounds.max_x; ++x) {
                int old_idx = old_row_start + x;

                if (label_grid[old_idx] == id) {
                    int new_x = x + dx;
                    if (new_x >= 0 && new_x < width) {
                        state.back_buffer[new_row_start + new_x] = pixel_grid[old_idx];
                    }
                }
            }
        }

        // 4. Update Region Metadata (Feedback for next frame)
        region.prev_center_f.x = region.center_f.x;
        region.prev_center_f.y = region.center_f.y;
        region.bounds.min_x += dx; region.bounds.max_x += dx;
        region.bounds.min_y += dy; region.bounds.max_y += dy;
    }

    // 5. Final Step: Swap the buffers back to the authoritative grid
    pixel_grid.swap(state.back_buffer);
}

} // namespace MotionSystem
} */// namespace rigid
