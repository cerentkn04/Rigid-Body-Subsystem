#include "StructuralGraph.hpp"
#include <algorithm>

namespace rigid {

void graph_build(
    StructuralGraph& graph,
    const std::unordered_map<RegionID, RegionRecord>& active_regions,
    const std::vector<RegionIndex>& label_grid,
    const std::vector<RegionID>& index_to_id,
    int world_width,
    int world_height)
{
    graph.nodes.clear();
    graph.id_to_node_idx.clear();

    // 1. Create one node per active region
    for (const auto& [id, record] : active_regions) {
        graph.id_to_node_idx[id] = (uint32_t)graph.nodes.size();
        StructuralNode node;
        node.id           = id;
        node.pixel_count  = record.pixel_count;
        node.bounds       = record.bounds;
        node.group_id     = record.group_id;
        node.touches_floor = (record.bounds.max_y >= world_height - 1);
        graph.nodes.push_back(std::move(node));
    }

    // 2. Adjacency pass
    // label_grid stores extraction-order indices; convert via index_to_id -> id_to_node_idx
    static std::vector<uint32_t> nbr_scratch;
    for (uint32_t i = 0; i < (uint32_t)graph.nodes.size(); ++i) {
        auto& node = graph.nodes[i];
        nbr_scratch.clear();

        int x0 = std::max(0, node.bounds.min_x - 1);
        int y0 = std::max(0, node.bounds.min_y - 1);
        int x1 = std::min(world_width  - 1, node.bounds.max_x + 1);
        int y1 = std::min(world_height - 1, node.bounds.max_y + 1);

        for (int y = y0; y <= y1; ++y) {
            const RegionIndex* row = label_grid.data() + y * world_width;
            for (int x = x0; x <= x1; ++x) {
                RegionIndex extr_idx = row[x];
                if (extr_idx == InvalidRegionIndex || extr_idx >= index_to_id.size()) continue;
                auto it = graph.id_to_node_idx.find(index_to_id[extr_idx]);
                if (it == graph.id_to_node_idx.end()) continue;
                uint32_t other = it->second;
                if (other != i) nbr_scratch.push_back(other);
            }
        }

        std::sort(nbr_scratch.begin(), nbr_scratch.end());
        nbr_scratch.erase(std::unique(nbr_scratch.begin(), nbr_scratch.end()), nbr_scratch.end());
        node.neighbor_indices = nbr_scratch;
    }
}

} // namespace rigid
