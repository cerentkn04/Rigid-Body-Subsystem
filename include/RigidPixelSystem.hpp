#pragma once
#include <vector>
#include <RigidPixelTypes.hpp>
#include <RegionExtractor.hpp>
#include <RegionTracker.hpp>
#include <RigidPixelWorldView.hpp>
#include <StructuralTracker.hpp>
#include <StructuralGraph.hpp>
#include <RigidBodyManager.hpp>
#include <StabilityResolver.hpp>
#include "RegionMesher.hpp"
#include <RegionMotion.hpp>
namespace rigid {

class RigidPixelSystem {
public:
  RegionExtractor extractor;
  TrackerState tracker;
  StructuralTracker structural_engine; 
  StructuralGraph structural_graph;      
  StabilityPolicyFunc stability_policy = nullptr;
    // --- Data ---
  std::unordered_map<uint32_t, RegionGeometry> geometry_cache;
  std::vector<RegionBuildRecord> build_records;
  BodyStore body_store;
  uint64_t last_processed_rev = 0;

    // --- Lifecycle ---
  void init_physics(b2WorldId world_id, int width, int height) {
     body_store_init(body_store, world_id);
     structural_engine.init_bins(width, height);
     stability_policy = &apply_basic_connectivity_policy;
  }
    // Replace set_policy with a simple pointer assignment
    void set_policy(StabilityPolicyFunc new_policy) {
        stability_policy = new_policy;
    }

template<typename CellType>
    void apply_region_motion(
        std::vector<CellType>& grid, 
        int width, 
        int height, 
        CellType empty_cell,
        MotionSystemState<CellType>& state) // Pass state in here
    {
        if (!b2World_IsValid(body_store.world_id)) return;

        MotionSystem::Apply<CellType>(
            state,
            grid,
            extractor.label_grid(),
            tracker.active_regions,
            width,
            height,
            empty_cell
        );
    }

        // --- The Pipeline ---
    void update(const world::WorldView& view) {
        uint64_t current_world_rev = view.world_revision();

        // 1. ENGINE: Sync Tracker (ID persistence)
        sync_engine_with_tracker();

        // 2. ENGINE: Dirty Detection (Check for pixel mutations)
        if (current_world_rev != last_processed_rev) {
            for (uint32_t i = 0; i < structural_engine.ids.size(); ++i) {
                if (!validate_region_topology(i, view)) {
                    structural_engine.dirty_flags[i] = 1;
                }
            }
        }

        // 3. PHYSICS & EXTRACTION: Heavy Lifting
        bool needs_physics_sync = (current_world_rev > last_processed_rev);
        for (uint8_t flag : structural_engine.dirty_flags) {
            if (flag == 1) { needs_physics_sync = true; break; }
        }

        if (needs_physics_sync) {
            extractor.extract(view, build_records);
            tracker_process_frame(tracker, extractor.label_grid(), build_records, view.width, view.height);
            cleanup_dead_geometry();
            refresh_geometry_cache(view);

            // Graph and stability only needed when topology changes
            structural_graph.build(tracker.active_regions, extractor.label_grid(), tracker.index_to_id, view.width, view.height);
            structural_engine.is_stable.assign(structural_graph.nodes.size(), true);
            if (stability_policy)
                stability_policy(structural_graph, structural_engine.is_stable);

            if (b2World_IsValid(body_store.world_id)) {
                physics_sync(
                    body_store,
                    structural_graph,
                    structural_engine.is_stable,
                    geometry_cache,
                    tracker.active_regions
                );
            }
            for (uint32_t i = 0; i < structural_engine.revisions.size(); ++i) {
        structural_engine.revisions[i] = current_world_rev;
    }
            
    last_processed_rev = current_world_rev;

            std::fill(structural_engine.dirty_flags.begin(), structural_engine.dirty_flags.end(), 0);
        }
    }

private:

void sync_engine_with_tracker() {
    const auto& active = tracker.active_regions;
    const size_t count = active.size();

    // Resize but keep existing data for the IDs we already know
    structural_engine.ids.assign(count, 0);
    structural_engine.influence_bounds.assign(count, {0,0,0,0});
    structural_engine.dirty_flags.assign(count, 0);
   // structural_engine.is_stable.assign(count, true);

    // Use a temporary map or persistent storage for revisions 
    // to prevent "Time Leaking" between different objects.
    static std::unordered_map<RegionID, uint64_t> persistent_revisions;

    size_t i = 0;
    for (const auto& [id, record] : active) {
        structural_engine.ids[i] = id;
        rigid::CellAABB b = record.bounds;
        structural_engine.influence_bounds[i] = {b.min_x-1, b.min_y-1, b.max_x+1, b.max_y+1};
        
        // Ensure the engine's array matches our persistent memory
        structural_engine.revisions.resize(count);
        structural_engine.revisions[i] = persistent_revisions[id]; 
        i++;
    }
}



    bool validate_region_topology(uint32_t index, const world::WorldView& world) {
        const auto& b = structural_engine.influence_bounds[index];
        uint64_t last_rev = structural_engine.revisions[index];

        for (int y = std::max(0, b.min_y); y <= std::min(world.height-1, b.max_y); ++y) {
            for (int x = std::max(0, b.min_x); x <= std::min(world.width-1, b.max_x); ++x) {
                if (world.region_revision(x, y) != last_rev) return false;
            }
        }
        return true;
    }

    void refresh_geometry_cache(const world::WorldView& view) {
        const auto& active_regions = tracker.active_regions;
        const auto& label_grid     = extractor.label_grid();
        const auto& index_to_id    = tracker.index_to_id;

        for (RegionIndex idx = 0; idx < (RegionIndex)index_to_id.size(); ++idx) {
            RegionID id = index_to_id[idx];
            if (active_regions.find(id) == active_regions.end()) continue;

            const RegionRecord& region = active_regions.at(id);
            auto cache_it = geometry_cache.find(id);
            if (cache_it == geometry_cache.end() || cache_it->second.version != region.version) {
                RegionGeometry geo = GeometryExtractor::Build(idx, build_records[idx].bounds, label_grid, view.width, view.height);
                geo.version = region.version;
                geometry_cache[id] = std::move(geo);
            }
        }

            }

    void cleanup_dead_geometry() {
        const auto& active = tracker.active_regions;
        for (auto it = geometry_cache.begin(); it != geometry_cache.end();) {
            if (active.find(it->first) == active.end()) it = geometry_cache.erase(it);
            else ++it;
        }
    }
};

} // namespace rigid
