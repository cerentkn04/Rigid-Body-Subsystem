#pragma once
#include <cstdint>
#include <RigidPixelTypes.hpp>
namespace rigid {

using RegionIndex = uint32_t;
static constexpr RegionIndex InvalidRegionIndex = UINT32_MAX;

struct RegionLabelBuffer {
    RegionIndex* labels;
    int width;
    int height;
};

struct RegionBuildRecord {
    CellAABB bounds;
    uint32_t pixel_count;
    uint32_t group_id;   // populated by extractor from world.group_id_at
};

} // namespace rigid
