#include "RegionTracker.hpp"
#include "regionScratch.hpp"
#include <unordered_map>

namespace rigid {

static RegionID generate_id(TrackerState& state) {
    RegionID id = state.next_id++;
    state.generations[id] = 1;
    return id;
}

void tracker_process_frame(
    TrackerState& state,
    const std::vector<RegionIndex>& label_grid,
    const std::vector<RegionBuildRecord>& records,
    int width, int height)
{
    const size_t num_current = records.size();
    if (num_current == 0) {
        state.active_regions.clear();
        state.index_to_id.clear();
        state.prev_label_grid.clear();
        state.prev_index_to_id.clear();
        return;
    }

    state.events.clear();

    // 1. Find the best previous-frame parent for each current region
    struct BestParent { RegionID id = InvalidRegionID; uint32_t count = 0; };
    std::vector<BestParent> best_parents(num_current);

    if (!state.prev_label_grid.empty()) {
        const RegionIndex* curr = label_grid.data();
        const RegionIndex* prev = state.prev_label_grid.data();
        const size_t prev_count = state.prev_index_to_id.size();

        // Flat counter array: indexed by prev RegionIndex, zero-initialised once per frame
        state.overlap_counts.assign(prev_count, 0);

        for (RegionIndex i = 0; i < (RegionIndex)num_current; ++i) {
            const auto& rect = records[i];
            state.touched_scratch.clear();

            for (int y = rect.bounds.min_y; y <= rect.bounds.max_y; ++y) {
                for (int x = rect.bounds.min_x; x <= rect.bounds.max_x; ++x) {
                    int idx = y * width + x;
                    if (curr[idx] != i) continue;
                    RegionIndex p = prev[idx];
                    if (p == InvalidRegionIndex || p >= prev_count) continue;
                    if (state.overlap_counts[p] == 0)
                        state.touched_scratch.push_back(p);
                    state.overlap_counts[p]++;
                }
            }

            for (RegionIndex p : state.touched_scratch) {
                uint32_t cnt = state.overlap_counts[p];
                if (cnt > best_parents[i].count)
                    best_parents[i] = { state.prev_index_to_id[p], cnt };
                state.overlap_counts[p] = 0; // reset as we go
            }
        }
    }

    // 2. Detect splits: count how many children each parent has
    //    Use a flat map (small N, linear scan is fine)
    state.scratch_index_to_id.assign(num_current, InvalidRegionID);

    // Count children per parent with a lightweight flat map
    static std::vector<std::pair<RegionID, uint32_t>> parent_counts;
    parent_counts.clear();
    for (RegionIndex i = 0; i < (RegionIndex)num_current; ++i) {
        RegionID pid = best_parents[i].id;
        if (pid == InvalidRegionID) continue;
        bool found = false;
        for (auto& [id, cnt] : parent_counts) {
            if (id == pid) { ++cnt; found = true; break; }
        }
        if (!found) parent_counts.push_back({ pid, 1 });
    }

    auto child_count = [&](RegionID pid) -> uint32_t {
        for (auto& [id, cnt] : parent_counts)
            if (id == pid) return cnt;
        return 0;
    };

    // 3. Assign IDs
    for (RegionIndex i = 0; i < (RegionIndex)num_current; ++i) {
        RegionID pid = best_parents[i].id;
        if (pid != InvalidRegionID && child_count(pid) == 1) {
            state.scratch_index_to_id[i] = pid;
        } else {
            RegionID new_id = generate_id(state);
            state.scratch_index_to_id[i] = new_id;
            if (pid != InvalidRegionID) {
                RegionLifecycleEvent ev;
                ev.type = LifecycleType::Split;
                ev.involved_ids = { pid, new_id };
                state.events.push_back(std::move(ev));
            }
        }
    }

    // 4. Build active_regions, carrying over motion/version state
    auto previous = std::move(state.active_regions);
    state.active_regions.clear();

    for (size_t i = 0; i < num_current; ++i) {
        RegionID id         = state.scratch_index_to_id[i];
        const auto& build   = records[i];
        RegionRecord& rec   = state.active_regions[id];

        rec.id             = id;
        rec.generation     = state.generations[id];
        rec.pixel_count    = build.pixel_count;
        rec.bounds         = build.bounds;
        rec.group_id       = build.group_id;
        rec.current_index  = (uint32_t)i;

        auto prev_it = previous.find(id);
        if (prev_it != previous.end()) {
            const RegionRecord& old = prev_it->second;
            rec.center_f           = old.center_f;
            rec.prev_center_f      = old.prev_center_f;
            rec.motion_initialized = old.motion_initialized;
            bool changed = (old.pixel_count != build.pixel_count) ||
                           (old.bounds.min_x != build.bounds.min_x) ||
                           (old.bounds.min_y != build.bounds.min_y);
            rec.version = changed ? old.version + 1 : old.version;
        } else {
            rec.center_f      = { 0.0f, 0.0f };
            rec.prev_center_f = { 0.0f, 0.0f };
            rec.version       = 1;
        }
    }

    // 5. Advance frame buffers
    state.prev_label_grid   = label_grid;
    state.prev_index_to_id  = state.scratch_index_to_id;
    state.index_to_id       = state.scratch_index_to_id;
}

} // namespace rigid
