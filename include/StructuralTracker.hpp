#pragma once
#include <vector>
#include <cstdint>
#include <RigidPixelTypes.hpp>
#include <RegionType.hpp>

struct Bin {
    std::vector<uint32_t> region_indices;
};

struct StructuralTracker {
    int bins_x = 0;
    int bins_y = 0;
    static constexpr int BIN_SIZE = 32;

    // Parallel arrays
    std::vector<rigid::RegionID>  ids;
    std::vector<rigid::CellAABB>  influence_bounds;
    std::vector<uint64_t>         revisions;
    std::vector<uint8_t>          dirty_flags;
    std::vector<bool>             is_stable;

    std::vector<Bin> bins;
};

void tracker_init_bins  (StructuralTracker& st, int world_width, int world_height);
void tracker_propagate_dirt(StructuralTracker& st);
