#pragma once
#include <vector>
#include <cstdint>
#include <RigidPixelTypes.hpp>
#include <RegionType.hpp>
#include <RigidPixelWorldView.hpp>
struct Bin {
    std::vector<uint32_t> region_indices;
};

struct StructuralTracker {
    // Spatial configuration (needed for bin math)
    int bins_x = 0;
    int bins_y = 0;
    static constexpr int BIN_SIZE = 32;

    // Parallel Arrays
    std::vector<rigid::RegionID>  ids;
    std::vector<rigid::CellAABB>  influence_bounds;
    std::vector<uint64_t>         revisions;
    std::vector<uint8_t>          dirty_flags;
    std::vector<bool>             is_stable; // Moved from snapshot to array

    std::vector<Bin> bins;
    void init_bins(int world_width, int world_height);
    void rebuild_bins();

    void mark_mutated_regions(const world::WorldView& world);
    void propagate_dirt(); 
};
