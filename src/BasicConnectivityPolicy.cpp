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

    // 4. Phase 2: Flood-Fill Connectivity
    while (!queue.empty()) {
        uint32_t curr = queue.front();
        queue.pop_front();
        for (uint32_t neighbor_idx : nodes[curr].neighbor_indices) {
            if (!out_is_stable[neighbor_idx]) {
                out_is_stable[neighbor_idx] = true;
                queue.push_back(neighbor_idx);
            }
        }
    }
}

} // namespace rigid
