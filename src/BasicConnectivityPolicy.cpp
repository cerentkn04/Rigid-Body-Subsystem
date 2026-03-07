#include "IStabilityPolicy.hpp" // Ensure this has the function declaration
#include "StructuralGraph.hpp"
#include <deque>

namespace rigid {

void apply_basic_connectivity_policy(const StructuralGraph& graph, std::vector<bool>& out_is_stable) {
    // 1. Access the vector 'nodes' (not 'node')
    const auto& nodes = graph.nodes; 
    
    // 2. 'out_is_stable' is the vector itself, not a struct member
    out_is_stable.assign(nodes.size(), false);

    // BFS Queue for stability propagation
    std::deque<uint32_t> queue;

    // 3. Phase 1: Seed Anchors (Floor or Group 1/Walls)
    for (uint32_t i = 0; i < (uint32_t)nodes.size(); ++i) {
        if (nodes[i].group_id == 1 || nodes[i].touches_floor) {
            out_is_stable[i] = true;
            queue.push_back(i);
        }
    }

    while (!queue.empty()) {
        uint32_t curr_idx = queue.front();
        queue.pop_front();

        const auto& supporter = nodes[curr_idx];

        for (uint32_t n_idx : supporter.neighbor_indices) {
            if (out_is_stable[n_idx]) continue;

            const auto& candidate = nodes[n_idx];

            // DIRECTIONAL RULE:
            // The 'supporter' provides stability to the 'candidate' 
            // ONLY if the supporter's bottom (max_y) is at or below the candidate's bottom.
            // This prevents the inner object of an "O" from being supported by the "O" top/sides.
            if (supporter.bounds.max_y >= candidate.bounds.max_y) {
                out_is_stable[n_idx] = true;
                queue.push_back(n_idx);
            }
        }
    }
    // 4. Phase 2: Flood-Fill Connectivity
}

} // namespace rigid
