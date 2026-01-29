#pragma once
#include <cstdint>

namespace rigid {

// Temporary index into region array (frame-local)
using RegionIndex = uint32_t;
static constexpr RegionIndex InvalidRegionIndex = UINT32_MAX;

// Label buffer: one entry per cell
struct RegionLabelBuffer {
    RegionIndex* labels;   // size = width * height
    int width;
    int height;
};

// Temporary region build record
struct RegionBuildRecord {
    // Bounding box (cell-space)
    int32_t min_x;
    int32_t min_y;
    int32_t max_x;
    int32_t max_y;

    // Pixel count
    uint32_t pixel_count;
};

} // namespace rigid

