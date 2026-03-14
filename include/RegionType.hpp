#pragma once
#include <cstdint>
#include "RigidPixelTypes.hpp" // Use the frozen definitions

namespace rigid {

// Use the frozen IDs
using RegionID = uint32_t;
static constexpr RegionID InvalidRegionID = 0;
using RegionGeneration = uint32_t;

// We need a struct for the flood-fill stack that isn't named CellCoord
// because the frozen CellCoord is an int32_t.
struct RegionPixel {
    int32_t x;
    int32_t y;
};

// RegionRecord uses the frozen CellAABB
struct RegionRecord {
    RegionID id;
    RegionGeneration generation;
    CellAABB bounds; 
    uint32_t pixel_count;
     BodyVersion version;
     bool is_dynamic;
     uint32_t group_id;
     struct { float x, y; } center_f;
     struct { float x, y; } prev_center_f;
};

} // namespace rigid


