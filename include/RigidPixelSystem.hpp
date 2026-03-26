#pragma once
#include <vector>
#include <unordered_map>
#include <RigidPixelTypes.hpp>
#include <RegionExtractor.hpp>
#include <RegionTracker.hpp>
#include <RigidPixelWorldView.hpp>
#include <StructuralTracker.hpp>
#include <StructuralGraph.hpp>
#include <RigidBodyManager.hpp>
#include <StabilityResolver.hpp>
#include "RegionMesher.hpp"

namespace rigid {

class RigidPixelSystem {
public:
    // ── Sub-system state (all plain structs) ─────────────────────────────
    ExtractorState    extractor;
    TrackerState      tracker;
    StructuralTracker structural_engine;
    StructuralGraph   structural_graph;
    BodyStore         body_store;
    StabilityPolicyFunc stability_policy = nullptr;

    // ── Frame data ────────────────────────────────────────────────────────
    std::unordered_map<RegionID, RegionGeometry> geometry_cache;
    std::vector<RegionBuildRecord> build_records;
    uint64_t last_processed_rev = 0;

    // ── Lifecycle ─────────────────────────────────────────────────────────
    void init_physics(b2WorldId world_id, int width, int height) {
        body_store_init(body_store, world_id);
        tracker_init_bins(structural_engine, width, height);
        stability_policy = &apply_basic_connectivity_policy;
    }

    void set_policy(StabilityPolicyFunc p) { stability_policy = p; }

    // ── Per-frame pipeline ────────────────────────────────────────────────
    void update(const world::WorldView& view) {
        const uint64_t current_rev = view.world_revision();

        // 1. Sync structural engine arrays with current tracker state
        sync_engine_with_tracker();

        // 2. Dirty detection — mark regions whose pixels changed
        if (current_rev != last_processed_rev) {
            for (uint32_t i = 0; i < (uint32_t)structural_engine.ids.size(); ++i) {
                if (!validate_region_topology(i, view))
                    structural_engine.dirty_flags[i] = 1;
            }
        }

        // 3. Heavy lifting — only when topology changed
        bool needs_sync = (current_rev > last_processed_rev);
        for (uint8_t f : structural_engine.dirty_flags)
            if (f) { needs_sync = true; break; }

        if (needs_sync) {
            extractor_extract(extractor, view, build_records);
            tracker_process_frame(tracker, extractor.label_grid, build_records, view.width, view.height);
            cleanup_dead_geometry();
            refresh_geometry_cache(view);

            graph_build(structural_graph, tracker.active_regions, extractor.label_grid, tracker.index_to_id, view.width, view.height);
            structural_engine.is_stable.assign(structural_graph.nodes.size(), true);
            if (stability_policy)
                stability_policy(structural_graph, structural_engine.is_stable);

            if (b2World_IsValid(body_store.world_id))
                physics_sync(body_store, structural_graph, structural_engine.is_stable, geometry_cache, tracker.active_regions);

            for (uint32_t i = 0; i < (uint32_t)structural_engine.revisions.size(); ++i)
                structural_engine.revisions[i] = current_rev;
            last_processed_rev = current_rev;
            std::fill(structural_engine.dirty_flags.begin(), structural_engine.dirty_flags.end(), 0);
        }
    }

private:
    void sync_engine_with_tracker() {
        const auto& active = tracker.active_regions;
        const size_t count = active.size();

        structural_engine.ids.assign(count, 0);
        structural_engine.influence_bounds.assign(count, {0,0,0,0});
        structural_engine.dirty_flags.assign(count, 0);
        structural_engine.revisions.resize(count);

        static std::unordered_map<RegionID, uint64_t> persistent_revisions;
        size_t i = 0;
        for (const auto& [id, record] : active) {
            structural_engine.ids[i] = id;
            const CellAABB& b = record.bounds;
            structural_engine.influence_bounds[i] = { b.min_x-1, b.min_y-1, b.max_x+1, b.max_y+1 };
            structural_engine.revisions[i] = persistent_revisions[id];
            ++i;
        }
    }

    bool validate_region_topology(uint32_t index, const world::WorldView& world) {
        const auto& b = structural_engine.influence_bounds[index];
        uint64_t last_rev = structural_engine.revisions[index];
        for (int y = std::max(0, b.min_y); y <= std::min(world.height-1, b.max_y); ++y)
            for (int x = std::max(0, b.min_x); x <= std::min(world.width-1, b.max_x); ++x)
                if (world.region_revision(x, y) != last_rev) return false;
        return true;
    }

    void refresh_geometry_cache(const world::WorldView& view) {
        const auto& active    = tracker.active_regions;
        const auto& id_map    = tracker.index_to_id;

        for (RegionIndex idx = 0; idx < (RegionIndex)id_map.size(); ++idx) {
            RegionID id = id_map[idx];
            auto rec_it = active.find(id);
            if (rec_it == active.end()) continue;

            const RegionRecord& region = rec_it->second;
            auto cache_it = geometry_cache.find(id);
            if (cache_it == geometry_cache.end() || cache_it->second.version != region.version) {
                RegionGeometry geo = build_geometry(idx, build_records[idx].bounds, extractor.label_grid, view.width, view.height);
                geo.version = region.version;
                geometry_cache[id] = std::move(geo);
            }
        }
    }

    void cleanup_dead_geometry() {
        const auto& active = tracker.active_regions;
        for (auto it = geometry_cache.begin(); it != geometry_cache.end();)
            it = (active.find(it->first) == active.end()) ? geometry_cache.erase(it) : std::next(it);
    }
};

} // namespace rigid
