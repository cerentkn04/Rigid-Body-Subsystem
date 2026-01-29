#include "RegionTracker.hpp"
#include "regionScratch.hpp" 
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <cassert>

namespace rigid {

struct RegionTracker::Impl {
    std::vector<RegionIndex> prev_label_grid;
    // NOTE: RegionIndex is frame-local; this bridges frame identity
    std::vector<RegionID> prev_index_to_id;
    
    struct OverlapPair {
        RegionIndex curr_idx;
        RegionID prev_id;
        bool operator<(const OverlapPair& other) const {
            if (curr_idx != other.curr_idx) return curr_idx < other.curr_idx;
            return prev_id < other.prev_id;
        }
    };
    std::vector<OverlapPair> overlap_scratch;

    // Option A: Never reuse IDs, keep generations monotonic
    std::unordered_map<RegionID, uint32_t> generations;
};

RegionTracker::RegionTracker() : m_impl(new Impl{}) {}
RegionTracker::~RegionTracker() { delete m_impl; }

RegionID RegionTracker::generate_id() {
    RegionID new_id = m_next_id_sequence++;
    m_impl->generations[new_id] = 1; 
    return new_id;
}

void RegionTracker::process_frame(
    const std::vector<RegionIndex>& current_label_grid,
    const std::vector<RegionBuildRecord>& current_records,
    int width, int height) 
{
    const size_t cell_count = static_cast<size_t>(width * height);
    assert(current_label_grid.size() == cell_count);
    
    m_events.clear();
    const size_t num_current = current_records.size();
    auto& pairs = m_impl->overlap_scratch;
    pairs.clear();

    // 1. DISCOVERY PHASE
    if (m_impl->prev_label_grid.size() == cell_count) {
        for (size_t i = 0; i < cell_count; ++i) {
            RegionIndex curr = current_label_grid[i];
            RegionIndex prev = m_impl->prev_label_grid[i];
            if (curr != InvalidRegionIndex && prev != InvalidRegionIndex) {
                pairs.push_back({curr, m_impl->prev_index_to_id[prev]});
            }
        }
    }

    std::sort(pairs.begin(), pairs.end());

    // Mapping relationships
    std::vector<std::unordered_map<RegionID, uint32_t>> overlaps(num_current);
    std::unordered_map<RegionID, std::unordered_map<RegionIndex, uint32_t>> parent_to_children;

    for (size_t i = 0; i < pairs.size(); ) {
        size_t j = i;
        while (j < pairs.size() && pairs[j].curr_idx == pairs[i].curr_idx && pairs[j].prev_id == pairs[i].prev_id) {
            j++;
        }
        uint32_t count = static_cast<uint32_t>(j - i);
        overlaps[pairs[i].curr_idx][pairs[i].prev_id] = count;
        parent_to_children[pairs[i].prev_id][pairs[i].curr_idx] = count;
        i = j;
    }

    // 2. CLASSIFICATION PHASE
    std::unordered_map<RegionID, RegionRecord> next_active_regions;
    std::vector<RegionID> current_index_to_id(num_current, InvalidRegionID);
    
    // IDs currently assigned to this frame's regions
    std::unordered_set<RegionID> assigned_ids;

    for (RegionIndex i = 0; i < num_current; ++i) {
        const auto& my_parents = overlaps[i];
        const auto& build = current_records[i];
        RegionID final_id = InvalidRegionID;

        if (my_parents.empty()) {
            // NEW: Creation
            final_id = generate_id();
            m_events.push_back({LifecycleType::Created, {final_id}}); 
        } 
        else if (my_parents.size() > 1) {
            // MERGE: Multiple parents -> New Body
            final_id = generate_id();
            m_events.push_back({LifecycleType::Created, {final_id}});
        }
        else {
            // One parent candidate
            RegionID best_parent = my_parents.begin()->first;
            const auto& children_of_parent = parent_to_children[best_parent];

            if (children_of_parent.size() == 1) {
                // CONTINUATION: 1 Parent, 1 Child. Identity survives.
                final_id = best_parent;
                assigned_ids.insert(final_id);
            } else {
                // SPLIT: Parent split into multiple. Per Rule 3: All get new IDs.
                final_id = generate_id();
                m_events.push_back({LifecycleType::Created, {final_id}});
            }
        }

        current_index_to_id[i] = final_id;

        RegionRecord rec;
        rec.id = final_id;
        rec.generation = m_impl->generations[final_id];
        rec.pixel_count = build.pixel_count;
        rec.bounds = { build.min_x, build.min_y, build.max_x, build.max_y };
        next_active_regions[final_id] = rec;
    }

    // 3. CLEANUP: Precise Destruction
    // Any ID active last frame that wasn't assigned (via Continuation) is dead.
    for (auto const& [old_id, old_rec] : m_active_regions) {
        if (assigned_ids.find(old_id) == assigned_ids.end()) {
            m_events.push_back({LifecycleType::Destroyed, {old_id}});
            // generations are kept for Option A consistency
        }
    }

    // 4. STATE SWAP
    m_active_regions = std::move(next_active_regions);
    m_impl->prev_label_grid = current_label_grid;
    m_impl->prev_index_to_id = current_index_to_id; 
    m_index_to_id_map = m_impl->prev_index_to_id;
}

} // namespace rigid

// namespace rigid
/*
#include "RegionTracker.hpp"
#include "regionScratch.hpp" // For RegionBuildRecord
#include <algorithm>
#include <unordered_set>

namespace rigid {

struct RegionTracker::Impl {
    // Spatial memory from the previous frame
    std::vector<RegionIndex> prev_label_grid;
    // Map of Previous Index -> The ID that index was assigned
    std::vector<RegionID> prev_index_to_id;
};

RegionTracker::RegionTracker() : m_impl(new Impl{}) {}

RegionTracker::~RegionTracker() {
    delete m_impl;
}

RegionID RegionTracker::generate_id() {
    return m_next_id_sequence++;
}

void RegionTracker::process_frame(

const std::vector<RegionIndex>& current_label_grid,
    const std::vector<RegionBuildRecord>& current_records,
    int width, int height) 
{
    m_events.clear();
    const size_t num_current = current_records.size();
    const size_t cell_count = static_cast<size_t>(width * height);

    // 1. DISCOVERY PHASE (High Speed)
    struct OverlapPair {
        RegionIndex curr_idx;
        RegionID prev_id;
        bool operator<(const OverlapPair& other) const {
            if (curr_idx != other.curr_idx) return curr_idx < other.curr_idx;
            return prev_id < other.prev_id;
        }
    };

    // Use a pre-allocated scratch buffer for pairs to avoid allocations
    static std::vector<OverlapPair> pairs;
    pairs.clear();
    pairs.reserve(cell_count / 2); // Heuristic

    if (m_impl->prev_label_grid.size() == cell_count) {
        for (size_t i = 0; i < cell_count; ++i) {
            RegionIndex curr = current_label_grid[i];
            RegionIndex prev = m_impl->prev_label_grid[i];
            if (curr != InvalidRegionIndex && prev != InvalidRegionIndex) {
                pairs.push_back({curr, m_impl->prev_index_to_id[prev]});
            }
        }
    }

    // Sort all pairs. This groups identical (Child, Parent) pairs together.
    std::sort(pairs.begin(), pairs.end());

    // Mapping: Current Index -> {Previous ID : Count}
    std::vector<std::unordered_map<RegionID, uint32_t>> overlaps(num_current);
    // Mapping: Previous ID -> {Current Index : Count}
    std::unordered_map<RegionID, std::unordered_map<RegionIndex, uint32_t>> parent_to_children;

    // Now scan the sorted list once (Linear O(N)) to fill the maps.
    // This only populates the maps for ACTUAL relationships, not per-pixel.
    for (size_t i = 0; i < pairs.size(); ) {
        size_t j = i;
        while (j < pairs.size() && pairs[j].curr_idx == pairs[i].curr_idx && pairs[j].prev_id == pairs[i].prev_id) {
            j++;
        }
        uint32_t count = static_cast<uint32_t>(j - i);
        overlaps[pairs[i].curr_idx][pairs[i].prev_id] = count;
        parent_to_children[pairs[i].prev_id][pairs[i].curr_idx] = count;
        i = j;
    }


   
    // 2. CLASSIFICATION PHASE
    std::unordered_map<RegionID, RegionRecord> next_active_regions;
    std::vector<RegionID> current_index_to_id(num_current, InvalidRegionID);
    std::unordered_set<RegionID> consumed_parents;

    for (RegionIndex i = 0; i < num_current; ++i) {
        const auto& my_parents = overlaps[i];
        const auto& build = current_records[i];
        RegionID final_id = InvalidRegionID;

        if (my_parents.empty()) {
            // Truly new region (Created)
            final_id = generate_id();
            m_events.push_back({LifecycleType::Created, {final_id}});
        } 
        else {
            // Find the parent that contributed the most to THIS child (Merge logic)
            RegionID best_parent = InvalidRegionID;
            uint32_t max_overlap_from_parent = 0;
            for (auto const& [id, count] : my_parents) {
                if (count > max_overlap_from_parent) {
                    max_overlap_from_parent = count;
                    best_parent = id;
                }
            }

            // Now, check the "Split" side: Am I the largest child of that parent?
            // Only the largest child gets to inherit the ID.
            RegionIndex largest_child_of_parent = i;
            uint32_t max_overlap_to_child = 0;
            for (auto const& [child_idx, count] : parent_to_children[best_parent]) {
                if (count > max_overlap_to_child) {
                    max_overlap_to_child = count;
                    largest_child_of_parent = child_idx;
                }
            }

            if (i == largest_child_of_parent) {
                // SUCCESS: I inherit the ID
                final_id = best_parent;
                // Mark all parents in a merge as consumed
                for (auto const& [id, count] : my_parents) {
                    consumed_parents.insert(id);
                }
            } else {
                // FAILURE: I am a smaller piece of a split. I get a new ID.
                final_id = generate_id();
                m_events.push_back({LifecycleType::Created, {final_id}});
            }
        }

        current_index_to_id[i] = final_id;

        // Build the Record
        RegionRecord rec;
        rec.id = final_id;
        rec.pixel_count = build.pixel_count;
        rec.bounds = { build.min_x, build.min_y, build.max_x, build.max_y };
        next_active_regions[final_id] = rec;
    }

    // 3. CLEANUP: Find destroyed regions
    for (auto const& [old_id, old_rec] : m_active_regions) {
        if (consumed_parents.find(old_id) == consumed_parents.end()) {
            m_events.push_back({LifecycleType::Destroyed, {old_id}});
        }
    }

    // 4. STATE SWAP
    m_active_regions = std::move(next_active_regions);
    m_impl->prev_label_grid = current_label_grid;
    m_impl->prev_index_to_id = current_index_to_id; 
    m_index_to_id_map = std::move(current_index_to_id);
}
} */// namespace rigid
