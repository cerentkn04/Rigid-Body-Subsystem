#pragma once
#include <RigidPixelTypes.hpp>
#include <unordered_map>
#include <RegionType.hpp>
#include <vector>
#include <cmath>
#include <algorithm> // Required for std::copy
namespace rigid {


template<typename PixelType>
void ApplyRegionMotion(
    PixelType* pixel_grid, 
    const uint32_t* label_grid, 
    int width, int height,
    uint64_t& world_revision,
    std::unordered_map<uint32_t, RegionRecord>& active_regions,
    PixelType empty_value) 
{
    // 1. Create a temporary buffer for the write-pass
    // static to reuse memory across frames
    static std::vector<PixelType> write_buffer;
    size_t grid_size = static_cast<size_t>(width) * height;
    
    if (write_buffer.size() != grid_size) {
        write_buffer.resize(grid_size);
    }

    // Initial copy: Background stays put
    std::copy(pixel_grid, pixel_grid + grid_size, write_buffer.begin());

    bool moved_any = false;

    for (auto& [id, region] : active_regions) {
        if (!region.is_dynamic) {
            region.prev_center_f.x = region.center_f.x;
            region.prev_center_f.y = region.center_f.y;
            continue;
        }

        // Delta from where pixels currently ARE (prev_center_f) to where Box2D says they should be.
        // prev_center_f tracks the pixel position in float space — it is updated by the integer
        // delta actually applied, NOT set to center_f directly.  This prevents sub-pixel
        // remainders from being discarded each frame, which would cause pixels to drift behind
        // the physics body over time and appear to float above objects they've landed on.
        float fdx = region.center_f.x - region.prev_center_f.x;
        float fdy = region.center_f.y - region.prev_center_f.y;
        int dx = static_cast<int>(std::round(fdx));
        int dy = static_cast<int>(std::round(fdy));

        // Sanity-clamp: skip insane deltas that would trash the grid
        if (std::abs(dx) > width || std::abs(dy) > height) {
            region.prev_center_f = region.center_f;
            continue;
        }




        if (dx == 0 && dy == 0) continue;
        moved_any = true;

        // Pass 1: clear all source pixels from write_buffer
        for (int y = region.bounds.min_y; y <= region.bounds.max_y; ++y) {
            if (y < 0 || y >= height) continue;
            for (int x = region.bounds.min_x; x <= region.bounds.max_x; ++x) {
                if (x < 0 || x >= width) continue;
                size_t src_idx = static_cast<size_t>(y) * width + x;
                if (label_grid[src_idx] != region.current_index) continue;
                write_buffer[src_idx] = empty_value;
            }
        }

        // Pass 2: write all pixels to their displaced destinations
        for (int y = region.bounds.min_y; y <= region.bounds.max_y; ++y) {
            if (y < 0 || y >= height) continue;
            for (int x = region.bounds.min_x; x <= region.bounds.max_x; ++x) {
                if (x < 0 || x >= width) continue;
                size_t src_idx = static_cast<size_t>(y) * width + x;
                if (label_grid[src_idx] != region.current_index) continue;
                int tx = x + dx;
                int ty = y + dy;
                if (tx >= 0 && tx < width && ty >= 0 && ty < height) {
                    write_buffer[static_cast<size_t>(ty) * width + tx] = pixel_grid[src_idx];
                }
            }
        }

        // Advance prev_center_f by exactly the integer pixels we moved —
        // the fractional remainder is preserved for the next frame.
        region.prev_center_f.x += dx;
        region.prev_center_f.y += dy;
        region.bounds.min_x += dx;
        region.bounds.max_x += dx;
        region.bounds.min_y += dy;
        region.bounds.max_y += dy;
    }

    if (moved_any) {
        std::copy(write_buffer.begin(), write_buffer.end(), pixel_grid);
        world_revision++;
    }

} 
}// namespace rigid
