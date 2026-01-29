#include "RigidPixelSystem.hpp"
#include "RigidPixelTypes.hpp" 
#include "regionScratch.hpp"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cassert>

namespace rigid {

struct BodyState {
    RigidBodyID id;
    GenerationID generation;
    RegionBuildRecord record;
};
struct RigidPixelSystem::Impl {
    RigidPixelConfig config;

    std::vector<RigidBodyID> prev_id_grid; 
    int grid_width = 0;
    int grid_height = 0;

    std::vector<RegionIndex> current_label_grid;
    std::vector<RegionBuildRecord> current_records;

    RigidBodyID next_id = 1; 
    std::vector<RigidBodyID> free_ids;
    std::unordered_map<RigidBodyID, BodyState> registry;
    std::unordered_map<RigidBodyID, GenerationID> generations;

    std::vector<RigidEvent> event_buffer;
    RigidPixelStats stats;

    BodyHandle acquire_handle() {
        RigidBodyID id;
        if (!free_ids.empty()) {
            id = free_ids.back();
            free_ids.pop_back();
            generations[id]++; // Increment when REUSING
        } else {
            id = next_id++;
            generations[id] = 1; // Start brand new IDs at Gen 1
        }
        return { id, generations[id] };
    }

    void release_id(RigidBodyID id) {
        free_ids.push_back(id);
    }

    void update_lifecycle() {
        event_buffer.clear();
        
        if (current_records.empty()) {
            for (auto const& [id, state] : registry) {
                event_buffer.push_back({RigidEventType::BodyDestroyed, {id, state.generation}});
                release_id(id); // Correctly free IDs
            }
            registry.clear();
            std::fill(prev_id_grid.begin(), prev_id_grid.end(), 0);
            stats.body_count = 0;
            return;
        }

        // --- PART 1: OVERLAP MATCHING ---
        std::vector<std::unordered_map<RigidBodyID, uint32_t>> overlap_map(current_records.size());
        for (int i = 0; i < grid_width * grid_height; ++i) {
            RegionIndex curr_idx = current_label_grid[i];
            if (curr_idx != InvalidRegionIndex) {
                RigidBodyID p_id = prev_id_grid[i];
                if (p_id != 0) overlap_map[curr_idx][p_id]++;
            }
        }

        // --- PART 2: TOPOLOGY RULES ---
        // FIRST: Count usage to detect splits correctly
        std::unordered_map<RigidBodyID, uint32_t> usage_count;
        for (const auto& overlaps : overlap_map) {
            if (overlaps.size() == 1) {
                usage_count[overlaps.begin()->first]++;
            }
        }

        // SECOND: Assign IDs
        std::vector<RigidBodyID> new_assigned_ids(current_records.size(), 0);
        for (size_t i = 0; i < current_records.size(); ++i) {
            auto& overlaps = overlap_map[i];
            if (overlaps.empty() || overlaps.size() > 1) {
                // BIRTH or MERGE
                new_assigned_ids[i] = acquire_handle().id;
            } else {
                RigidBodyID p_id = overlaps.begin()->first;
                if (usage_count[p_id] > 1) {
                    // SPLIT
                    new_assigned_ids[i] = acquire_handle().id;
                } else {
                    // CONTINUATION
                    new_assigned_ids[i] = p_id;
                }
            }
        }

        // --- PART 3: REGISTRY & EVENTS ---
        std::unordered_set<RigidBodyID> alive_this_frame;
        for (RigidBodyID id : new_assigned_ids) alive_this_frame.insert(id);

        // 3a. Emit Destruction
        auto it = registry.begin();
        while (it != registry.end()) {
            if (alive_this_frame.find(it->first) == alive_this_frame.end()) {
                event_buffer.push_back({RigidEventType::BodyDestroyed, {it->first, it->second.generation}});
                release_id(it->first); // Free the ID
                it = registry.erase(it);
            } else {
                ++it;
            }
        }

        // 3b. Emit Creation
        for (size_t i = 0; i < current_records.size(); ++i) {
            RigidBodyID id = new_assigned_ids[i];
            if (registry.find(id) == registry.end()) {
                // It's a new body (Created, Merged, or Split)
                registry[id] = {id, generations[id], current_records[i]};
                event_buffer.push_back({RigidEventType::BodyCreated, {id, generations[id]}});
            } else {
                registry[id].record = current_records[i];
            }
        }

        // --- PART 4: COMMIT GRID ---
        std::fill(prev_id_grid.begin(), prev_id_grid.end(), 0);
        for (int i = 0; i < grid_width * grid_height; ++i) {
            RegionIndex curr_idx = current_label_grid[i];
            if (curr_idx != InvalidRegionIndex) {
                prev_id_grid[i] = new_assigned_ids[curr_idx];
            }
        }
        stats.body_count = static_cast<uint32_t>(registry.size());
    }
}; //end of impl 


// --- API Implementation ---

RigidPixelSystem::RigidPixelSystem(const RigidPixelConfig& config)
    : m_impl(new Impl{}) {
    m_impl->config = config;
}

RigidPixelSystem::~RigidPixelSystem() {
    delete m_impl;
}

void RigidPixelSystem::update(const world::WorldView& world_view, float dt) {
    if (m_impl->grid_width != world_view.width || m_impl->grid_height != world_view.height) {
        m_impl->grid_width = world_view.width;
        m_impl->grid_height = world_view.height;
        m_impl->prev_id_grid.assign(m_impl->grid_width * m_impl->grid_height, 0);
        m_impl->current_label_grid.assign(m_impl->grid_width * m_impl->grid_height, InvalidRegionIndex);
    }

    // Extraction would be called here
    m_impl->update_lifecycle();
}

const RigidEvent* RigidPixelSystem::events() const { 
    return m_impl->event_buffer.empty() ? nullptr : m_impl->event_buffer.data(); 
}

uint32_t RigidPixelSystem::event_count() const { 
    return static_cast<uint32_t>(m_impl->event_buffer.size()); 
}

const RigidPixelStats* RigidPixelSystem::stats() const { 
    return &m_impl->stats; 
}

} // namespace rigid

