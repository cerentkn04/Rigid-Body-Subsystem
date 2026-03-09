/*#include "RegionMotionApplier.hpp"
#include <cmath>
#include <cstring>

namespace rigid {
namespace RegionMotionSystem {

template<typename PixelType>
void apply(
    std::vector<RegionRecord>& active_regions,
    const RegionID* label_grid,
    const PixelType* old_grid,
    PixelType* new_grid,
    int32_t width,
    int32_t height,
    PixelType empty_pixel_value) 
{
    // 1. Full Grid Copy (Sequential Memory Access)
    // This establishes the "base" world (static geometry/background)
    std::memcpy(new_grid, old_grid, (size_t)width * height * sizeof(PixelType));

    for (auto& region : active_regions) {
        if (!region.is_dynamic) continue;

        // 2. Compute Integer Delta
        // Using round() ensures we only move when the physics center crosses a 0.5 threshold
        int32_t dx = (int32_t)std::round(region.center_f.x - region.prev_center_f.x);
        int32_t dy = (int32_t)std::round(region.center_f.y - region.prev_center_f.y);

        // Optimization: No pixel-level change detected
        if (dx == 0 && dy == 0) continue;

        // 3. Iterate Region Bounds
        // Rule: [min, max) - Inclusive min, Exclusive max
        for (CellCoord y = region.bounds.min_y; y < region.bounds.max_y; ++y) {
            if (y < 0 || y >= height) continue;

            for (CellCoord x = region.bounds.min_x; x < region.bounds.max_x; ++x) {
                if (x < 0 || x >= width) continue;

                int32_t src_idx = y * width + x;

                // 4. Membership Check
                // Only move pixels that the label_grid says belong to this region ID
                if (label_grid[src_idx] == region.id) {
                    int32_t nx = x + dx;
                    int32_t ny = y + dy;

                    // 5. Bounds Check for Destination
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        int32_t dst_idx = ny * width + nx;
                        
                        // Write pixel to new location
                        new_grid[dst_idx] = old_grid[src_idx];
                        
                        // 6. Clear Old Position
                        // We must clear the source in the NEW grid, otherwise 
                        // the pixel will remain at both (x,y) and (nx,ny).
                        new_grid[src_idx] = empty_pixel_value;
                    }
                }
            }
        }


region.prev_center_f.x = region.center_f.x;
region.prev_center_f.y = region.center_f.y;
        // Update bounds to match the new pixel positions
        region.bounds.min_x += dx;
        region.bounds.max_x += dx;
        region.bounds.min_y += dy;
        region.bounds.max_y += dy;
    }
}

} // namespace RegionMotionSystem
} */// namespace rigid
