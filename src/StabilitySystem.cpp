#include "RegionStability.hpp"
#include "RigidPixelWorldView.hpp"
#include <algorithm>
#include <deque>

// Internal helper for AABB intersection logic
static bool intersects(const rigid::CellAABB& a, const rigid::CellAABB& b) {
    return (a.min_x <= b.max_x && a.max_x >= b.min_x) &&
           (a.min_y <= b.max_y && a.max_y >= b.min_y);
}
void StabilitySystem::sync_with_tracker(const std::unordered_map<rigid::RegionID, rigid::RegionRecord>& active_regions) {
    std::vector<RegionSnapshot> next_snapshots;
    next_snapshots.reserve(active_regions.size());

    std::unordered_map<rigid::RegionID, uint32_t> next_id_to_index;
    next_id_to_index.reserve(active_regions.size());

    for (const auto& [id, record] : active_regions) {
        uint32_t new_idx = static_cast<uint32_t>(next_snapshots.size());
        next_id_to_index[id] = new_idx;

        auto it = id_to_index.find(id);
        if (it != id_to_index.end()) {
            // Carry over existing snapshot
            next_snapshots.push_back(active_snapshots[it->second]);
        } else {
            // Initialize new snapshot for new ID
            RegionSnapshot blank;
            blank.id = id;
            blank.revision_at_last_build = 0; 
            blank.influence_bounds = {0, 0, 0, 0};
            next_snapshots.push_back(blank);
        }
    }

    active_snapshots = std::move(next_snapshots);
    id_to_index = std::move(next_id_to_index);
    dirty_flags.assign(active_snapshots.size(), 0);
}
void StabilitySystem::set_capacity(size_t count) {
    // Note: We no longer index directly by RegionID to avoid O(Infinity) memory growth.
    // Capacity is now managed dynamically based on the active set.
    active_snapshots.reserve(count);
    dirty_flags.reserve(count);
}

bool StabilitySystem::validate_snapshot(uint32_t index, const world::WorldView& world) const {
    if (index >= active_snapshots.size()) return false;
    
    const auto& snap = active_snapshots[index];
    
    // STEP 4: Explicitly clamp the search area to world bounds to prevent out-of-bounds reads
    int32_t x_start = std::max(0, snap.influence_bounds.min_x);
    int32_t y_start = std::max(0, snap.influence_bounds.min_y);
    int32_t x_end   = std::min(world.width - 1, snap.influence_bounds.max_x);
    int32_t y_end   = std::min(world.height - 1, snap.influence_bounds.max_y);

    for (int32_t y = y_start; y <= y_end; ++y) {
        for (int32_t x = x_start; x <= x_end; ++x) {
            // STEP 6.3: Cheap integer comparison using revision function pointer
            if (world.region_revision(x, y) != snap.revision_at_last_build) {
                return false; // Found local mutation
            }
        }
    }
    
    return true; // Topology is stable
}

void StabilitySystem::update_snapshot(rigid::RegionID id, const rigid::CellAABB& bounds, uint64_t current_revision) {
    // If we already have this ID in our dense list, update it.
    // Otherwise, this should be handled by your "Sync" step at frame start.
    if (id_to_index.find(id) != id_to_index.end()) {
        uint32_t idx = id_to_index[id];
        auto& snap = active_snapshots[idx];

        // STEP 6.1: Define influence bounds (Expand by 1)
        snap.influence_bounds.min_x = bounds.min_x - 1;
        snap.influence_bounds.min_y = bounds.min_y - 1;
        snap.influence_bounds.max_x = bounds.max_x + 1;
        snap.influence_bounds.max_y = bounds.max_y + 1;

        snap.revision_at_last_build = current_revision;
        dirty_flags[idx] = 0; // Commit as clean
    }
}

void StabilitySystem::reset_dirty_flags(uint8_t default_state) {
    std::fill(dirty_flags.begin(), dirty_flags.end(), default_state);
}

void StabilitySystem::propagate_dirty_bounds() {
    // STEP 3: Replace O(N^2) global scanning with a Work Queue (Propagating Transitive Dirt)
    std::deque<uint32_t> work_queue;

    // Seed the queue with regions already marked dirty by the revision check
    for (uint32_t i = 0; i < dirty_flags.size(); ++i) {
        if (dirty_flags[i] == 1) {
            work_queue.push_back(i);
        }
    }

    // Process the chain reaction
    while (!work_queue.empty()) {
        uint32_t dirty_idx = work_queue.front();
        work_queue.pop_front();

        const auto& dirty_bounds = active_snapshots[dirty_idx].influence_bounds;

        // Check against all other regions in the active set
        for (uint32_t i = 0; i < active_snapshots.size(); ++i) {
            // If already dirty, no need to process it again
            if (dirty_flags[i] == 1) continue;

            if (intersects(dirty_bounds, active_snapshots[i].influence_bounds)) {
                dirty_flags[i] = 1;
                work_queue.push_back(i); // This neighbor is now dirty; push to check ITS neighbors
            }
        }
    }
}

