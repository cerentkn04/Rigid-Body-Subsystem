#pragma once
#include <vector>
#include <memory>
#include <RigidPixelTypes.hpp>
#include <RegionExtractor.hpp>
#include <RegionTracker.hpp>
#include <RigidPixelWorldView.hpp>
#include <StructuralTracker.hpp>
#include <StructuralGraph.hpp>
#include <RigidBodyManager.hpp>
#include "RegionMesher.hpp" 
#include <RegionMotion.hpp>
namespace rigid {

class RigidPixelSystem {
public:
    // --- Systems ---
    RegionExtractor extractor;
    RegionTracker tracker;
    StructuralTracker structural_engine; 
    StructuralGraph structural_graph;      
    StabilityPolicyFunc stability_policy = nullptr;
    // --- Data ---
    std::unordered_map<uint32_t, RegionGeometry> geometry_cache;
    std::vector<RegionBuildRecord> build_records;
    std::unique_ptr<RigidBodyManager> body_manager;
    uint64_t last_processed_rev = 0;

    // --- Lifecycle ---
    void init_physics(b2WorldId world_id, int width, int height) {
        body_manager = std::make_unique<RigidBodyManager>(world_id);
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
        if (!body_manager) return;

        MotionSystem::Apply<CellType>(
            state,               
            grid, 
            extractor.label_grid(), 
            tracker.get_active_regions(), 
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

        // 3. ENGINE: Build Structural Graph (Find who touches whom)
        structural_graph.build(tracker.get_active_regions(), extractor.label_grid(), view.width, view.height);
        
        size_t node_count = structural_graph.nodes.size();
        structural_engine.is_stable.assign(node_count, true);
        // 4. POLICY: Evaluate Stability (The Decision Layer)

         if (stability_policy) {
            stability_policy(structural_graph, structural_engine.is_stable);
        } 
        // 5. PHYSICS & EXTRACTION: Heavy Lifting
        bool needs_physics_sync = (current_world_rev > last_processed_rev);
        for (uint8_t flag : structural_engine.dirty_flags) {
            if (flag == 1) { needs_physics_sync = true; break; }
        }

    

        if (needs_physics_sync) {
            extractor.extract(view, build_records);
            tracker.process_frame(extractor.label_grid(), build_records, view.width, view.height);
            
            cleanup_dead_geometry(); 
            refresh_geometry_cache(view);
            if (body_manager) {
              // We now have a vector of bools 'is_stable' from the Policy
    body_manager->synchronize(
        structural_graph, 
        structural_engine.is_stable, // This is your vector<bool>
        geometry_cache, 
        tracker.get_active_regions()
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
    const auto& active = tracker.get_active_regions();
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
        const auto& active_regions = tracker.get_active_regions();
        const auto& label_grid = extractor.label_grid();
        const auto& index_to_id = tracker.get_index_mapping();

        for (RegionIndex idx = 0; idx < index_to_id.size(); ++idx) {
            RegionID id = index_to_id[idx];
            const RegionBuildRecord& record = build_records[idx];
            const RegionRecord& region = active_regions.at(id);

            bool is_missing = geometry_cache.find(id) == geometry_cache.end();
            if (is_missing || geometry_cache[id].version != region.version) {
                RegionGeometry geo = GeometryExtractor::Build(idx, record.bounds, label_grid, view.width, view.height);
                geo.version = region.version;
                geometry_cache[id] = std::move(geo);
            }
        }
    }

    void cleanup_dead_geometry() {
        const auto& active = tracker.get_active_regions();
        for (auto it = geometry_cache.begin(); it != geometry_cache.end();) {
            if (active.find(it->first) == active.end()) it = geometry_cache.erase(it);
            else ++it;
        }
    }
};

} // namespace rigid
