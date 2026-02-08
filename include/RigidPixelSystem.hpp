#pragma once
#include <vector>
#include <RigidPixelTypes.hpp>
#include <RegionExtractor.hpp>
#include <RegionTracker.hpp>
#include <RegionStability.hpp>
#include <RigidPixelWorldView.hpp>
namespace rigid {

class RigidPixelSystem {
public:
    RegionExtractor extractor;
    RegionTracker tracker;
    StabilitySystem stability;
    std::vector<RegionBuildRecord> build_records;
    uint64_t last_processed_rev = 0;

    void update(const world::WorldView& view) {
        uint64_t current_world_rev = view.world_revision();

        
        stability.sync_with_tracker(tracker.get_active_regions());
        stability.reset_dirty_flags(0);

        // 4.1 Check if existing rocks moved or changed
        if (current_world_rev != last_processed_rev) {
            for (uint32_t i = 0; i < stability.active_snapshots.size(); ++i) {
                if (!stability.validate_snapshot(i, view)) {
                    stability.dirty_flags[i] = 1;
                }
            }
        }

        stability.propagate_dirty_bounds();

        // 4.2 Extraction Logic
        bool needs_extract = false;
        for (uint8_t flag : stability.dirty_flags) {
            if (flag == 1) { needs_extract = true; break; }
        }

        const auto& index_mapping = tracker.get_index_mapping();
        if (current_world_rev > last_processed_rev || index_mapping.empty()) {
            if (current_world_rev > 0) needs_extract = true;
        }

        if (needs_extract) {
            extractor.extract(view, build_records);
            tracker.process_frame(extractor.label_grid(), build_records, view.width, view.height);
            last_processed_rev = current_world_rev;

            const auto& finalized_regions = tracker.get_active_regions();
            for (const auto& [id, record] : finalized_regions) {
                stability.update_snapshot(id, record.bounds, current_world_rev);
            }
        }
    }
};

} // namespace rigid
