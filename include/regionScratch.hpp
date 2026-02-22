#pragma once
#include <cstdint>
#include <RigidPixelTypes.hpp>
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
    CellAABB bounds;
    uint32_t pixel_count;
};

} // namespace rigid

