

#pragma once
#include <vector>
#include <unordered_map>
#include <RigidPixelTypes.hpp>
#include <RegionExtractor.hpp>
#include <RegionTracker.hpp>
#include <RegionStability.hpp>
#include <RigidPixelWorldView.hpp>
#include "RegionMesher.hpp" // The Step 7 Header we just fixed

namespace rigid {

class RigidPixelSystem {
public:
    RegionExtractor extractor;
    RegionTracker tracker;
    StabilitySystem stability;
    
    // The Geometry Cache (Step 7)
    // Map: RegionID -> The geometric contours
    std::unordered_map<uint32_t, RegionGeometry> geometry_cache;

    std::vector<RegionBuildRecord> build_records;
    uint64_t last_processed_rev = 0;

    void update(const world::WorldView& view) {
        uint64_t current_world_rev = view.world_revision();

        stability.sync_with_tracker(tracker.get_active_regions());
        
        // --- Step 6: Dirtiness Logic ---
        if (current_world_rev != last_processed_rev) {
            for (uint32_t i = 0; i < stability.active_snapshots.size(); ++i) {
                if (!stability.validate_snapshot(i, view)) {
                    stability.dirty_flags[i] = 1;
                }
            }
        }
        stability.propagate_dirty_bounds();

        // Check if extraction is needed
        bool needs_extract = false;
        for (uint8_t flag : stability.dirty_flags) {
            if (flag == 1) { needs_extract = true; break; }
        }

        const auto& index_mapping = tracker.get_index_mapping();
        if (current_world_rev > last_processed_rev || index_mapping.empty()) {
            if (current_world_rev > 0) needs_extract = true;
        }

        // --- Core Execution ---
        if (needs_extract) {
            extractor.extract(view, build_records);
            tracker.process_frame(extractor.label_grid(), build_records, view.width, view.height);
            last_processed_rev = current_world_rev;

            const auto& finalized_regions = tracker.get_active_regions();
            
            // --- Step 7: Geometry Extraction ---
            // Only rebuild what is actually dirty or brand new
            refresh_geometry_cache(view);

            for (const auto& [id, record] : finalized_regions) {
                stability.update_snapshot(id, record.bounds, current_world_rev);
            }
        }
        
        // Reset flags AFTER geometry is harvested
        stability.reset_dirty_flags(0);
    }

private:
    void refresh_geometry_cache(const world::WorldView& view) {
    const auto& active_regions = tracker.get_active_regions();
    const auto& label_grid = extractor.label_grid();

    // 1. Cleanup dead regions
    for (auto it = geometry_cache.begin(); it != geometry_cache.end(); ) {
        if (active_regions.find(it->first) == active_regions.end()) {
            it = geometry_cache.erase(it);
        } else {
            ++it;
        }
    }

    // 2. Process active regions
    for (const auto& [id, record] : active_regions) {
        // Correct lookup using the unordered_map in StabilitySystem
        auto it = stability.id_to_index.find(id);
        if (it == stability.id_to_index.end()) continue;

        uint32_t idx = it->second;
        bool is_dirty = (stability.dirty_flags[idx] == 1);
        bool is_missing = (geometry_cache.find(id) == geometry_cache.end());

        if (is_dirty || is_missing) {
            // Pass view.width for flat grid indexing
            RegionGeometry new_geo = GeometryExtractor::Build(id, record.bounds, label_grid, view.width);
            
            if (!is_missing) {
                new_geo.version = geometry_cache[id].version + 1;
            } else {
                new_geo.version = 1;
            }
            geometry_cache[id] = std::move(new_geo);
        }
    }
}
};

} // namespace rigid

