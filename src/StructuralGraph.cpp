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
                                                                      
          {                                                                       
            static std::vector<std::vector<uint32_t>> adj_scratch;              
             adj_scratch.assign(graph.nodes.size(), {});

            for (int y = 0; y < world_height; ++y) {  
                   const RegionIndex* row = label_grid.data() + y * world_width;
                 for (int x = 0; x < world_width; ++x) {                         
                      RegionIndex idx_a = row[x]; 
                      if (idx_a == InvalidRegionIndex || idx_a >= (RegionIndex)index_to_id.size()) continue;
                      auto it_a = graph.id_to_node_idx.find(index_to_id[idx_a]);  
                      if (it_a == graph.id_to_node_idx.end()) continue;           
                     uint32_t node_a = it_a->second;                             
                                                                                 
                     // Check right and down neighbors only — covers all pairs on
                                                                       
                    const int nxs[2] = { x + 1, x };                            
                    const int nys[2] = { y,     y + 1 };                        
                     for (int d = 0; d < 2; ++d) {                               
                        int nx = nxs[d], ny = nys[d];                           
                         if (nx >= world_width || ny >= world_height) continue;  
                         RegionIndex idx_b = label_grid.data()[ny * world_width +nx];
                       if (idx_b == InvalidRegionIndex || idx_b == idx_a || idx_b >= (RegionIndex)index_to_id.size()) continue; 
                       auto it_b = graph.id_to_node_idx.find(index_to_id[idx_b]);                                                                          
                         if (it_b == graph.id_to_node_idx.end()) continue;       
                         uint32_t node_b = it_b->second;                         
                        adj_scratch[node_a].push_back(node_b);                  
                       adj_scratch[node_b].push_back(node_a);                  
                     }                             
           }
        }
   for (uint32_t i = 0; i < (uint32_t)graph.nodes.size(); ++i) {       
                 auto& v = adj_scratch[i];                                       
              std::sort(v.begin(), v.end());                                  
                v.erase(std::unique(v.begin(), v.end()), v.end());              
                 graph.nodes[i].neighbor_indices = std::move(v);                 
            } 
       
    }
}

} // namespace rigid
