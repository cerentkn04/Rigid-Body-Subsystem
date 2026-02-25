#pragma once
#include <vector>
#include <unordered_map>
#include <RigidPixelTypes.hpp>
#include <RegionExtractor.hpp>
#include <RegionTracker.hpp>
#include <RegionStability.hpp>
#include <RigidPixelWorldView.hpp>
#include "RegionMesher.hpp" 
#include "stdio.h"
namespace rigid {

class RigidPixelSystem {
public:
    RegionExtractor extractor;
    RegionTracker tracker;
    StabilitySystem stability;
    
    std::unordered_map<uint32_t, RegionGeometry> geometry_cache;
    std::vector<RegionBuildRecord> build_records;
    uint64_t last_processed_rev = 0;

// Example: If you have a group_grid array
    void update(const world::WorldView& view) {
        uint64_t current_world_rev = view.world_revision();

        // 1. Always keep stability map synced with tracker
        stability.sync_with_tracker(tracker.get_active_regions());
        
        // 2. Dirtiness Logic
        if (current_world_rev != last_processed_rev) {
            for (uint32_t i = 0; i < stability.active_snapshots.size(); ++i) {
                if (!stability.validate_snapshot(i, view)) {
                    stability.dirty_flags[i] = 1;
                }
            }
        }
        stability.propagate_dirty_bounds();

        // 3. Check if we need to run the heavy lifting
        bool needs_extract = false;
        for (uint8_t flag : stability.dirty_flags) {
            if (flag == 1) { needs_extract = true; break; }
        }

        if (current_world_rev > last_processed_rev || tracker.get_active_regions().empty()) {
            if (current_world_rev > 0) needs_extract = true;
        }

        // 4. Core Execution
        if (needs_extract) {
            extractor.extract(view, build_records);
            tracker.process_frame(extractor.label_grid(), build_records, view.width, view.height);
            stability.sync_with_tracker(tracker.get_active_regions());
            last_processed_rev = current_world_rev;

            // REFRESH GEOMETRY HERE
           cleanup_dead_geometry(); 
            refresh_geometry_cache(view);

            // Update stability snapshots for the next frame
            const auto& finalized_regions = tracker.get_active_regions();
            for (const auto& [id, record] : finalized_regions) {
                stability.update_snapshot(id, record.bounds, current_world_rev);
            }
            
            // 5. Reset flags ONLY after geometry and snapshots are handled
            stability.reset_dirty_flags(0);
        }
    }

private:
    void refresh_geometry_cache(const world::WorldView& view) {
    const auto& active_regions = tracker.get_active_regions();
    const auto& label_grid = extractor.label_grid();
    const auto& index_to_id = tracker.get_index_mapping();

    //printf("Min region index: %u\n", index_to_id[0]);
for (RegionIndex idx = 0; idx < index_to_id.size(); ++idx) {
    RegionID id = index_to_id[idx];

    const RegionBuildRecord& record = build_records[idx];
    const RegionRecord& region = active_regions.at(id);

    bool is_missing = geometry_cache.find(id) == geometry_cache.end();
    bool version_mismatch = false;

    if (!is_missing) {
        version_mismatch = (geometry_cache[id].version != region.version);
    }

    if (is_missing || version_mismatch) {
        RegionGeometry geo =
            GeometryExtractor::Build(idx, record.bounds, label_grid, view.width,view.height);

        geo.version = region.version;
        geometry_cache[id] = std::move(geo);
    }
}


}
 void cleanup_dead_geometry() {
    const auto& active = tracker.get_active_regions();

    for (auto it = geometry_cache.begin(); it != geometry_cache.end();) {
        if (active.find(it->first) == active.end())
            it = geometry_cache.erase(it);
        else
            ++it;
    }
}          // 2. Process active regions
};

} // namespace rigid
