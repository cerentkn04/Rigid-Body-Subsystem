#pragma once
#include <vector>
#include <unordered_map>
#include <cstdint>
#include "RegionType.hpp"

namespace rigid {

struct RegionBuildRecord; 
using RegionIndex = uint32_t;

enum class LifecycleType { Created, Destroyed, Split, Merged };

struct RegionLifecycleEvent {
    LifecycleType type;
    std::vector<RegionID> involved_ids;
};

// All tracker state flat in one struct — no pimpl, no hidden scratch.
struct TrackerState {
    // ── Cross-frame persistent state ─────────────────────────────────────
    uint32_t next_id = 1;
    std::unordered_map<RegionID, uint32_t> generations;
    std::vector<RegionIndex> prev_label_grid;
    std::vector<RegionID>    prev_index_to_id;

    // ── Current-frame output (read directly by other systems) ─────────────
    std::unordered_map<RegionID, RegionRecord> active_regions;
    std::vector<RegionID>    index_to_id;   // frame-local index -> persistent ID
    std::vector<RegionLifecycleEvent> events;

    // ── Scratch buffers (reused each frame to avoid allocation) ───────────
    std::vector<RegionID>    scratch_index_to_id;
    std::vector<uint32_t>    overlap_counts;  // indexed by prev RegionIndex
    std::vector<RegionIndex> touched_scratch; // prev indices touched by current region
};

void tracker_process_frame(
    TrackerState& state,
    const std::vector<RegionIndex>& label_grid,
    const std::vector<RegionBuildRecord>& records,
    int width, int height);

} // namespace rigid
