#pragma once
#include <vector>
#include <cmath>
#include <cstring>
#include <algorithm>
#include "RegionType.hpp"

namespace rigid {
namespace RegionMotionSystem {

template<typename PixelType>
void apply(
    std::vector<RegionRecord>& active_regions,
    const RegionID* old_labels,  // Source Labels
    RegionID* new_labels,        // Destination Labels
    const PixelType* old_pixels, // Source Pixels
    PixelType* new_pixels,       // Destination Pixels
    int32_t width, int32_t height,
    PixelType empty_pixel_value)
   
{
   
// Step 1: Sequential Copy (DOD Principle)
    std::memcpy(new_pixels, old_pixels, (size_t)width * height * sizeof(PixelType));
    std::memcpy(new_labels, old_labels, (size_t)width * height * sizeof(RegionID));

    for (auto& region : active_regions) {
        if (!region.is_dynamic) continue;

        // Step 2: Displacement math
        int32_t dx = (int32_t)std::round(region.center_f.x - region.prev_center_f.x);
        int32_t dy = (int32_t)std::round(region.center_f.y - region.prev_center_f.y);

        if (dx == 0 && dy == 0) continue;

        // Step 3: Clipped bounds to avoid inner-loop branches
        int32_t y0 = std::max(0, region.bounds.min_y);
        int32_t y1 = std::min(height, region.bounds.max_y);
        int32_t x0 = std::max(0, region.bounds.min_x);
        int32_t x1 = std::min(width, region.bounds.max_x);

        for (int32_t y = y0; y < y1; ++y) {
            for (int32_t x = x0; x < x1; ++x) {
                int32_t src = y * width + x;

                // Step 4: Membership Check (Read only from OLD)
                if (old_labels[src] == region.id) {
                    int32_t nx = x + dx;
                    int32_t ny = y + dy;

                    // Clear the "Old" spot in the NEW grid first
                    new_pixels[src] = empty_pixel_value;
                    new_labels[src] = 0;

                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        int32_t dst = ny * width + nx;
                        // Move data to NEW spot
                        new_pixels[dst] = old_pixels[src];
                        new_labels[dst] = region.id;
                    }
                }
            }
        }


  
        // Step 7: Update state
        region.prev_center_f.x = region.center_f.x;
        region.prev_center_f.y = region.center_f.y;
        region.bounds.min_x += dx;
        region.bounds.max_x += dx;
        region.bounds.min_y += dy;
        region.bounds.max_y += dy;
    }
}

} // namespace RegionMotionSystem
} // namespace rigid
