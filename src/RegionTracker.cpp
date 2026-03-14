#include "RegionTracker.hpp"
#include "regionScratch.hpp" 
#include <vector>
#include <cstdio>
#include <cassert>
#include <unordered_set>
namespace rigid {

struct RegionTracker::Impl {
    std::vector<RegionIndex> prev_label_grid;
    std::vector<RegionID> prev_index_to_id;
   const std::vector<RegionID>& get_index_mapping() const; 
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
const size_t num_current = current_records.size();
    if (num_current == 0) {
        m_active_regions.clear();
        m_index_to_id_map.clear();
        m_impl->prev_label_grid.clear();
        m_impl->prev_index_to_id.clear();
        return;
    }

    m_events.clear();
    
    struct BestParent {
        RegionID id = InvalidRegionID;
        uint32_t overlap_count = 0;
    };
    std::vector<BestParent> best_parents(num_current);

    // 1. DISCOVERY: Find which previous ID each current region overlaps with most
    if (!m_impl->prev_label_grid.empty()) {
        const RegionIndex* curr_grid = current_label_grid.data();
        const RegionIndex* prev_grid = m_impl->prev_label_grid.data();
        const RegionID* prev_id_map = m_impl->prev_index_to_id.data();

        for (RegionIndex i = 0; i < (RegionIndex)num_current; ++i) {
            const auto& rect = current_records[i];
            
            // Map to track all parents overlapping this child
            std::unordered_map<RegionID, uint32_t> local_overlaps;

            for (int y = rect.bounds.min_y; y <= rect.bounds.max_y; ++y) {
                for (int x = rect.bounds.min_x; x <= rect.bounds.max_x; ++x) {
                    const int idx = y * width + x;
                    if (curr_grid[idx] == i) {
                        RegionIndex p_idx = prev_grid[idx];
                        if (p_idx != InvalidRegionIndex && p_idx < m_impl->prev_index_to_id.size()) {
                            RegionID p_id = prev_id_map[p_idx];
                            local_overlaps[p_id]++;
                        }
                    }
                }
            }

            // Pick the parent with the most pixel overlap
            RegionID best_id = InvalidRegionID;
            uint32_t max_overlap = 0;
            for (auto const& [id, count] : local_overlaps) {
                if (count > max_overlap) {
                    max_overlap = count;
                    best_id = id;
                }
            }
            best_parents[i] = { best_id, max_overlap };
        }
    }

    // 2. SPLIT DETECTION: Count how many children want to claim each parent ID
    std::unordered_map<RegionID, std::vector<RegionIndex>> parent_claims;
    for (RegionIndex i = 0; i < (RegionIndex)num_current; ++i) {
        if (best_parents[i].id != InvalidRegionID) {
            parent_claims[best_parents[i].id].push_back(i);
        }
    }
// 3. ID ASSIGNMENT: Handle Splits (Deterministic Clean-Slate Logic)
m_impl->current_index_to_id.assign(num_current, InvalidRegionID);

for (auto& [parent_id, children] : parent_claims) {
    
    if (children.size() == 1) {
        // CONTINUATION: Only one child, so it keeps the identity
        m_impl->current_index_to_id[children[0]] = parent_id;
    } 
    else {
        // SPLIT DETECTED: Parent is destroyed, all children are brand new

        for (size_t j = 0; j < children.size(); ++j) {
            RegionID new_id = generate_id();
            m_impl->current_index_to_id[children[j]] = new_id;

            
            // Log the event as a Split (which implies parent destruction)
            RegionLifecycleEvent ev;
            ev.type = LifecycleType::Split;
            ev.involved_ids = { parent_id, new_id }; 
            m_events.push_back(ev);
        }
    }
}

// Assign IDs to entirely new regions that had no parent (same as before)
for (size_t i = 0; i < num_current; ++i) {
    if (m_impl->current_index_to_id[i] == InvalidRegionID) {
        m_impl->current_index_to_id[i] = generate_id();
    }
}
  
    // The "winner" keeps the ID
  



    // Assign IDs to entirely new regions that had no parent
    for (size_t i = 0; i < num_current; ++i) {
        if (m_impl->current_index_to_id[i] == InvalidRegionID) {
            m_impl->current_index_to_id[i] = generate_id();
        }
    } 

        // 2. & 3. CLASSIFICATION & ID ASSIGNMENT


    // 4. STATE UPDATE
  /* auto previous_regions = m_active_regions;
    m_active_regions.clear();
    for (size_t i = 0; i < num_current; ++i) {
        RegionID id = m_impl->current_index_to_id[i];
        const auto& build = current_records[i];
        RegionRecord& rec = m_active_regions[id];
        rec.id = id;
        rec.generation = m_impl->generations[id];
        rec.pixel_count = build.pixel_count;
        rec.bounds = { build.bounds};
auto prev_it = previous_regions.find(id);
if (prev_it != previous_regions.end()) {
    const RegionRecord& old = prev_it->second;

    bool changed =
        (old.pixel_count != build.pixel_count) ||
        (old.bounds.min_x != build.bounds.min_x) ||
        (old.bounds.min_y != build.bounds.min_y) ||
        (old.bounds.max_x != build.bounds.max_x) ||
        (old.bounds.max_y != build.bounds.max_y);

    rec.version = changed ? old.version + 1 : old.version;
} else {
    // New body
    rec.version = 1;
}








    }

    m_impl->prev_label_grid = current_label_grid;
    m_impl->prev_index_to_id = m_impl->current_index_to_id;
    m_index_to_id_map = m_impl->prev_index_to_id; // CRITICAL for renderer */
                                                  //
                                                  //
    // 4. STATE UPDATE
    //
    //


// 4. STATE UPDATE
    auto previous_regions = m_active_regions;
    m_active_regions.clear();

    for (size_t i = 0; i < num_current; ++i) {
        RegionID id = m_impl->current_index_to_id[i];
        const auto& build = current_records[i];
        
        RegionRecord& rec = m_active_regions[id];
        rec.id = id;
        rec.generation = m_impl->generations[id];
        rec.pixel_count = build.pixel_count;
        rec.bounds = { build.bounds };

        auto prev_it = previous_regions.find(id);
        if (prev_it != previous_regions.end()) {
            const RegionRecord& old = prev_it->second;
            rec.prev_center_f = old.prev_center_f;
            rec.center_f = old.center_f;
            
            bool changed = (old.pixel_count != build.pixel_count) || 
                           (old.bounds.min_x != build.bounds.min_x) ||
                           (old.bounds.min_y != build.bounds.min_y);
            rec.version = changed ? old.version + 1 : old.version;
        } else {
            rec.center_f = { 0.0f, 0.0f };
            rec.prev_center_f = { 0.0f, 0.0f };
            rec.version = 1;
        }
    }

    // --- THE FIX IS HERE ---
    // You must store these so the rest of the engine can read them!
    m_impl->prev_label_grid = current_label_grid;
    m_impl->prev_index_to_id = m_impl->current_index_to_id;
    
    // This is the specific variable RigidPixelSystem and the Renderer read:
    m_index_to_id_map = m_impl->current_index_to_id; 
}



}
