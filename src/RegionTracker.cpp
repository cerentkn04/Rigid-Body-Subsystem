#include "RegionTracker.hpp"
#include "regionScratch.hpp" 
#include <algorithm>
#include <vector>
#include <cassert>
#include <unordered_set>
namespace rigid {

struct RegionTracker::Impl {
    std::vector<RegionIndex> prev_label_grid;
    std::vector<RegionID> prev_index_to_id;
    
    struct OverlapPair {
        RegionIndex curr_idx;
        RegionID prev_id;
        bool operator<(const OverlapPair& other) const {
            if (curr_idx != other.curr_idx) return curr_idx < other.curr_idx;
            return prev_id < other.prev_id;
        }
    };
    
    // Scratch buffers to prevent re-allocation every frame
    std::vector<OverlapPair> overlap_scratch;
    std::vector<uint32_t> parent_counts; // Replaces the maps
    std::vector<RegionID> current_index_to_id;
    
    // Using a vector for generations is faster if IDs are sequential
    // If IDs are very sparse, keep the map, but it's rarely the bottleneck
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
    const size_t num_current = current_records.size();
    if (num_current == 0) {
        m_active_regions.clear();
        m_index_to_id_map.clear();
        return;
    }

    m_events.clear();
    
    // We will use a flat array to count overlaps instead of sorting
    // This is MUCH faster for large grids
    static std::vector<uint32_t> overlap_counts;
    // Structure: [current_index][parent_id] -> count
    // But since parent_id can be large, we'll use a smarter approach:
    // Only track the BEST parent for each current region per frame
    struct BestParent {
        RegionID id = InvalidRegionID;
        uint32_t count = 0;
        uint32_t total_overlaps = 0;
    };
    static std::vector<BestParent> best_parents;
    best_parents.assign(num_current, {InvalidRegionID, 0, 0});

    // 1. DISCOVERY (Ultra-fast single pass)
    //
    if (!m_impl->prev_label_grid.empty() && m_impl->prev_label_grid.size() == cell_count) {
        const RegionIndex* curr_grid = current_label_grid.data();
        const RegionIndex* prev_grid = m_impl->prev_label_grid.data();
        const RegionID* prev_id_map = m_impl->prev_index_to_id.data();
        const size_t prev_id_size = m_impl->prev_index_to_id.size();

        // Instead of looping 480,000 times, we only loop over the actual rocks
        for (RegionIndex i = 0; i < (RegionIndex)num_current; ++i) {
            const auto& rect = current_records[i];
            auto& best = best_parents[i];

            // Only scan the rectangle where this specific rock exists
            for (int y = rect.min_y; y <= rect.max_y; ++y) {
                const int row_offset = y * width;
                for (int x = rect.min_x; x <= rect.max_x; ++x) {
                    const int idx = row_offset + x;

                    // If this pixel belongs to the current rock we are analyzing
                    if (curr_grid[idx] == i) {
                        RegionIndex p_idx = prev_grid[idx];
                        if (p_idx != InvalidRegionIndex && p_idx < prev_id_size) {
                            RegionID p_id = prev_id_map[p_idx];
                            best.total_overlaps++;
                            
                            if (best.id == InvalidRegionID || best.id == p_id) {
                                best.id = p_id;
                                best.count++;
                            }
                        }
                    }
                }
            }
        }
    }
    

        // 2. & 3. CLASSIFICATION & ID ASSIGNMENT
    static std::vector<uint32_t> claim_counts;
    if (claim_counts.size() < m_next_id_sequence) claim_counts.resize(m_next_id_sequence + 100, 0);

    m_impl->current_index_to_id.assign(num_current, InvalidRegionID);
    
    // First pass: identify continuations
    for (size_t i = 0; i < num_current; ++i) {
        if (best_parents[i].id != InvalidRegionID) {
            claim_counts[best_parents[i].id]++;
        }
    }

    std::unordered_set<RegionID> assigned_ids;
    for (size_t i = 0; i < num_current; ++i) {
        RegionID p = best_parents[i].id;
        // Continuation: One child claims this parent
        if (p != InvalidRegionID && claim_counts[p] == 1) {
            m_impl->current_index_to_id[i] = p;
            assigned_ids.insert(p);
        } else {
            // Split or New
            m_impl->current_index_to_id[i] = generate_id();
        }
    }

    // Reset claim counts efficiently
    for (size_t i = 0; i < num_current; ++i) {
        if (best_parents[i].id != InvalidRegionID) claim_counts[best_parents[i].id] = 0;
    }

    // 4. STATE UPDATE
    m_active_regions.clear();
    for (size_t i = 0; i < num_current; ++i) {
        RegionID id = m_impl->current_index_to_id[i];
        const auto& build = current_records[i];
        RegionRecord& rec = m_active_regions[id];
        rec.id = id;
        rec.generation = m_impl->generations[id];
        rec.pixel_count = build.pixel_count;
        rec.bounds = { build.min_x, build.min_y, build.max_x, build.max_y };
    }

    m_impl->prev_label_grid = current_label_grid;
    m_impl->prev_index_to_id = m_impl->current_index_to_id;
    m_index_to_id_map = m_impl->prev_index_to_id; // CRITICAL for renderer
}

}
