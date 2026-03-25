#include "StructuralGraph.hpp"
#include <algorithm>

namespace rigid {

void StructuralGraph::build(
  const std::unordered_map<RegionID, RegionRecord>& active_regions,
  const std::vector<RegionIndex>& label_grid,
  const std::vector<RegionID>& index_to_id,
  int world_width, int world_height)
{
  nodes.clear();
  id_to_node_idx.clear();

  // 1. Create nodes
  for (const auto& [id, record] : active_regions) {
      id_to_node_idx[id] = (uint32_t)nodes.size();
      StructuralNode node;
      node.id           = id;
      node.pixel_count  = record.pixel_count;
      node.bounds       = record.bounds;
      node.group_id     = record.group_id;
      node.touches_floor = (record.bounds.max_y >= world_height - 1);
      nodes.push_back(std::move(node));
  }

  // 2. Adjacency pass — translate label_grid extraction indices to node indices
  //    label_grid stores extraction-order indices; nodes[] is in unordered_map order.
  //    Use index_to_id + id_to_node_idx to get the correct node index for each pixel.
  static std::vector<uint32_t> nbr_scratch;
  for (uint32_t i = 0; i < (uint32_t)nodes.size(); ++i) {
      auto& node = nodes[i];
      nbr_scratch.clear();

      int x0 = std::max(0, node.bounds.min_x - 1);
      int y0 = std::max(0, node.bounds.min_y - 1);
      int x1 = std::min(world_width  - 1, node.bounds.max_x + 1);
      int y1 = std::min(world_height - 1, node.bounds.max_y + 1);

      for (int y = y0; y <= y1; ++y) {
          const RegionIndex* row = label_grid.data() + y * world_width;
          for (int x = x0; x <= x1; ++x) {
              RegionIndex extraction_idx = row[x];
              if (extraction_idx == InvalidRegionIndex || extraction_idx >= index_to_id.size())
                  continue;
              RegionID other_id = index_to_id[extraction_idx];
              auto it = id_to_node_idx.find(other_id);
              if (it == id_to_node_idx.end()) continue;
              uint32_t other_node_idx = it->second;
              if (other_node_idx != i)
                  nbr_scratch.push_back(other_node_idx);
          }
      }

      std::sort(nbr_scratch.begin(), nbr_scratch.end());
      nbr_scratch.erase(std::unique(nbr_scratch.begin(), nbr_scratch.end()), nbr_scratch.end());
      node.neighbor_indices = nbr_scratch;
  }
}
} // namespace rigid
