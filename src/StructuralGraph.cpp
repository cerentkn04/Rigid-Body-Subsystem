#include "StructuralGraph.hpp"
#include <unordered_map>
#include <set>

namespace rigid {

void StructuralGraph::build(
    const std::unordered_map<RegionID, RegionRecord>& active_regions,
    const std::vector<RegionIndex>& label_grid,
    int world_width, int world_height) 
{
    nodes.clear();
    id_to_node_idx.clear();

    // 1. Initial Pass: Create Nodes
    for (const auto& [id, record] : active_regions) {
        id_to_node_idx[id] = (uint32_t)nodes.size();
        StructuralNode node;
        node.id = id;
        node.pixel_count = (uint32_t)record.pixel_count;
        node.bounds = record.bounds;
        node.group_id = record.group_id;
        node.touches_floor = (record.bounds.max_y >= world_height - 1);
        nodes.push_back(node);
    }

    // 2. Adjacency Pass: Scan for neighbors
    for (uint32_t i = 0; i < nodes.size(); ++i) {
        auto& node = nodes[i];
        std::set<uint32_t> neighbor_set;

        // Inflate bounds by 1 to check the immediate surrounding pixels
        int x0 = std::max(0, node.bounds.min_x - 1);
        int y0 = std::max(0, node.bounds.min_y - 1);
        int x1 = std::min(world_width - 1, node.bounds.max_x + 1);
        int y1 = std::min(world_height - 1, node.bounds.max_y + 1);

        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                RegionIndex other_idx = label_grid[y * world_width + x];
                
                // If this pixel belongs to a different valid region
                if (other_idx != InvalidRegionIndex && other_idx != i) {
                    neighbor_set.insert(other_idx);
                }
            }
        }

        // Convert set to vector for the node
        for (uint32_t neighbor : neighbor_set) {
            node.neighbor_indices.push_back(neighbor);
        }
    }
}

} // namespace rigid
