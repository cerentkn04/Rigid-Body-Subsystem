#pragma once
#include <vector>
#include <unordered_map>
#include <cstdint>
#include "RegionType.hpp"

namespace rigid {

// Forward declaration of the Step 4 build record
struct RegionBuildRecord;

// Ensure RegionIndex is available (usually uint32_t)
using RegionIndex = uint32_t;

enum class LifecycleType {
    Created,
    Destroyed,
    Split,   // One parent ID -> Multiple child IDs
    Merged   // Multiple parent IDs -> One child ID
};

struct RegionLifecycleEvent {
    LifecycleType type;
    // For Split/Merge, this contains all involved IDs.
    // For Created/Destroyed, it contains the single relevant ID.
    std::vector<RegionID> involved_ids;
};

class RegionTracker {
public:
    RegionTracker();
    ~RegionTracker();

    /**
     * Step 5: Process the current frame's extraction results.
     * Compares current topology against previous frame to manage persistent IDs.
     */
    void process_frame(
        const std::vector<RegionIndex>& current_label_grid,
        const std::vector<RegionBuildRecord>& current_records,
        int width, int height);
const std::vector<RegionID>& get_index_mapping() const { return m_index_to_id_map; }
    // Accessors for the current state
    const std::vector<RegionLifecycleEvent>& get_events() const { return m_events; }
    const std::unordered_map<RegionID, RegionRecord>& get_active_regions() const { return m_active_regions; }

    std::unordered_map<RegionID, RegionRecord>& get_active_regions() { return m_active_regions; }

private:
    struct Impl;
    Impl* m_impl;
    std::vector<RegionID> m_index_to_id_map;

    std::unordered_map<RegionID, RegionRecord> m_active_regions;
    std::vector<RegionLifecycleEvent> m_events;
    
    uint32_t m_next_id_sequence = 1;
    RegionID generate_id();
};

} // namespace rigid
