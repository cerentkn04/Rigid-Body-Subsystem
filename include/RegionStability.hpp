
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
    uint64_t revision_at_last_build;
};

struct StabilitySystem {
    // Dense storage: indices 0 to N-1
    std::vector<RegionSnapshot> active_snapshots;
    std::vector<uint8_t> dirty_flags;
    
    // Indirection: RegionID -> Index in the vectors above
    std::unordered_map<rigid::RegionID, uint32_t> id_to_index;

    // STEP 5/6: Syncs snapshots with the tracker's active set to prevent memory leaks
    void sync_with_tracker(const std::unordered_map<rigid::RegionID, rigid::RegionRecord>& active_regions);

    // Checks a specific dense index for CA changes
    bool validate_snapshot(uint32_t index, const world::WorldView& world) const;
    
    // O(N*K) propagation using a work queue
    void propagate_dirty_bounds();
    
    // Commits new data to a specific ID
    void update_snapshot(rigid::RegionID id, const rigid::CellAABB& bounds, uint64_t revision);

    // Helpers for lifecycle management
    void set_capacity(size_t count);
    void reset_dirty_flags(uint8_t default_state = 0);
};
