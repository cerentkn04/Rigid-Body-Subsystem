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

        // Calculate Pixel Delta
        int dx = static_cast<int>(std::round(region.center_f.x - region.prev_center_f.x));
        int dy = static_cast<int>(std::round(region.center_f.y - region.prev_center_f.y));

// DEBUG PRINT: Check for insane deltas
if (std::abs(dx) > width || std::abs(dy) > height) {
    fprintf(stderr, "[CRITICAL] Insane Delta Detected for Region %u: dx=%d, dy=%d\n", id, dx, dy);
    fprintf(stderr, "Center: %f, %f | Prev: %f, %f\n", 
            region.center_f.x, region.center_f.y, 
            region.prev_center_f.x, region.prev_center_f.y);
    
    // Recovery: prevent the loop from running and crashing
    region.prev_center_f.x = region.center_f.x;
    region.prev_center_f.y = region.center_f.y;
    continue;
}




        if (dx == 0 && dy == 0) continue;
        moved_any = true;

        // Bounded iteration
        for (int y = region.bounds.min_y; y <= region.bounds.max_y; ++y) {
            if (y < 0 || y >= height) continue;
            
            for (int x = region.bounds.min_x; x <= region.bounds.max_x; ++x) {
                if (x < 0 || x >= width) continue;

                size_t src_idx = static_cast<size_t>(y) * width + x;
                
                // Masking via label grid
                if (label_grid[src_idx] != region.id) continue;

                int tx = x + dx;
                int ty = y + dy;

                // Move pixel if target is in bounds
                if (tx >= 0 && tx < width && ty >= 0 && ty < height) {
                    size_t dst_idx = static_cast<size_t>(ty) * width + tx;
                    
                    // Clear old, write new in the write_buffer
                    write_buffer[src_idx] = empty_value;
                    write_buffer[dst_idx] = pixel_grid[src_idx];
                }
            }
        }

        // Update tracking state
        region.prev_center_f.x = region.center_f.x;
        region.prev_center_f.y = region.center_f.y;
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
