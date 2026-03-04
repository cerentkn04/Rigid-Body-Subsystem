#include "StructuralTracker.hpp"
#include <box2d/box2d.h>
#include <cstdio>
namespace rigid {
// Internal helper for neighbor support logic
bool check_neighbor_support(uint32_t index, const StructuralTracker& tracker, const world::WorldView& world) {
    const auto& my_bounds = tracker.influence_bounds[index];
    
    int bx0 = std::max(0, my_bounds.min_x / StructuralTracker::BIN_SIZE);
    int by0 = std::max(0, my_bounds.min_y / StructuralTracker::BIN_SIZE);
    int bx1 = std::min(tracker.bins_x - 1, my_bounds.max_x / StructuralTracker::BIN_SIZE);
    int by1 = std::min(tracker.bins_y - 1, my_bounds.max_y / StructuralTracker::BIN_SIZE);

    for (int by = by0; by <= by1; ++by) {
        for (int bx = bx0; bx <= bx1; ++bx) {
            const auto& bin = tracker.bins[by * tracker.bins_x + bx];
            for (uint32_t other_idx : bin.region_indices) {
                if (index == other_idx) continue;
                
                // If a neighbor is already stable and we touch it, we are supported
                if (tracker.is_stable[other_idx]) {
                    // Note: You can add a more precise 'intersects' check here if needed
                    return true; 
                }
            }
        }
    }
    return false;
}

// Function to actually interface with Box2D or your physics manager
void trigger_falling_physics(rigid::RegionID id) {
    // Implement your logic to wake up the body here
    // e.g., physicsWorld.GetBody(id)->SetType(b2_dynamicBody);
    printf("Stability: Region %u is now DYNAMIC (falling)\n", id);
}

void resolve_stability(StructuralTracker& tracker,const world::WorldView& world) {
    for (uint32_t i = 0; i < tracker.ids.size(); ++i) {
        // Only re-evaluate if the tracker says the area around the region changed
        if (tracker.dirty_flags[i] == 0) continue;

        // Rule 1: Floor support
        bool supported = (tracker.influence_bounds[i].max_y >= world.height - 1);

        // Rule 2: Neighbor support (Structural chain)
        if (!supported) {
            supported = check_neighbor_support(i, tracker, world);
        }

        // Update the stable state in our parallel array
        tracker.is_stable[i] = supported;

        // If still not supported, make it fall
        if (!supported) {
            trigger_falling_physics(tracker.ids[i]);
        }
    }
}
}
