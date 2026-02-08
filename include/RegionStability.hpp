#pragma once
#include <vector>
#include <unordered_map>
#include <cstdint>
#include "RigidPixelTypes.hpp"
#include "RegionType.hpp"

namespace world { struct WorldView; }

struct RegionSnapshot {
    rigid::RegionID id;
    rigid::CellAABB influence_bounds;
    uint64_t revision_at_last_build = 0;

    // Cached bin range
    int bx0, by0, bx1, by1;
};

struct Bin {
    std::vector<uint32_t> region_indices;
};

struct StabilitySystem {
    static constexpr int BIN_SIZE = 32;

    // Dense storage
    std::vector<RegionSnapshot> active_snapshots;
    std::vector<uint8_t> dirty_flags;

    // Spatial bins
    int bins_x = 0;
    int bins_y = 0;
    std::vector<Bin> bins;
    bool bins_dirty = false;

    // RegionID → dense index
    std::unordered_map<rigid::RegionID, uint32_t> id_to_index;

    // Per-call visited stamping
    std::vector<uint32_t> visited_stamp;
    uint32_t current_stamp = 1;

    // Lifecycle
    void init_bins(int world_width, int world_height);
    void rebuild_bins();

    void sync_with_tracker(
        const std::unordered_map<rigid::RegionID, rigid::RegionRecord>& active_regions);

    void set_capacity(size_t count);
    void reset_dirty_flags(uint8_t default_state = 0);

    // Core logic
    bool validate_snapshot(uint32_t index, const world::WorldView& world) const;
    void update_snapshot(rigid::RegionID id,
                         const rigid::CellAABB& bounds,
                         uint64_t current_revision);

    void propagate_dirty_bounds();
};

