/*#include "RegionStability.hpp"
#include "RigidPixelWorldView.hpp"
#include <algorithm>
#include <deque>
#include <cstdio>
// Internal helper for AABB intersection logic
static bool intersects(const rigid::CellAABB& a, const rigid::CellAABB& b) {
   
    return (a.min_x <= b.max_x && a.max_x >= b.min_x) &&
           (a.min_y <= b.max_y && a.max_y >= b.min_y);
}
void StabilitySystem::init_bins(int world_width, int world_height) {
    bins_x = (world_width  + BIN_SIZE - 1) / BIN_SIZE;
    bins_y = (world_height + BIN_SIZE - 1) / BIN_SIZE;

    bins.clear();
    bins.resize(bins_x * bins_y);
}
void StabilitySystem::rebuild_bins() {
    for (auto& bin : bins)
        bin.region_indices.clear();

    for (uint32_t i = 0; i < active_snapshots.size(); ++i) {
        const auto& b = active_snapshots[i].influence_bounds;

        int bx0 = std::max(0, b.min_x / BIN_SIZE);
        int by0 = std::max(0, b.min_y / BIN_SIZE);
        int bx1 = std::min(bins_x - 1, b.max_x / BIN_SIZE);
        int by1 = std::min(bins_y - 1, b.max_y / BIN_SIZE);

        for (int by = by0; by <= by1; ++by) {
            for (int bx = bx0; bx <= bx1; ++bx) {
                bins[by * bins_x + bx].region_indices.push_back(i);
            }
        }
    }
}
// Inside StabilitySystem logic
void StabilitySystem::check_stability(uint32_t index, const world::WorldView& world) {
    auto& snapshot = active_snapshots[index];
    bool touches_floor = (snapshot.influence_bounds.max_y >= world.height - 1);
    snapshot.is_stable = touches_floor;
   
    if (!snapshot.is_stable) {
        // This should trigger the "Region X changed state to DYNAMIC" log in your Manager
        printf("Stability: Region %u is FLOATING (Bounds MaxY: %d, WorldH: %d)\n", 
                snapshot.id, snapshot.influence_bounds.max_y, world.height);
    }
    // Example Logic: If the region touches the bottom of the world, it is stable.
    if (snapshot.influence_bounds.max_y >= world.height - 1) {
        snapshot.is_stable = true;
    } else {
        // Otherwise, it might be floating (dynamic physics)
        snapshot.is_stable = false;
    }
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
        int bx0 = std::max(0, dirty_bounds.min_x / BIN_SIZE);
int by0 = std::max(0, dirty_bounds.min_y / BIN_SIZE);
int bx1 = std::min(bins_x - 1, dirty_bounds.max_x / BIN_SIZE);
int by1 = std::min(bins_y - 1, dirty_bounds.max_y / BIN_SIZE);

for (int by = by0; by <= by1; ++by) {
    for (int bx = bx0; bx <= bx1; ++bx) {
        const auto& bin = bins[by * bins_x + bx];

        for (uint32_t region_idx : bin.region_indices) {
            if (dirty_flags[region_idx]) continue;

            // Optional narrow-phase check (usually cheap)
            if (intersects(dirty_bounds,
                           active_snapshots[region_idx].influence_bounds)) {
                dirty_flags[region_idx] = 1;
                work_queue.push_back(region_idx);
            }
        }
    }
}

    }
}
*/
