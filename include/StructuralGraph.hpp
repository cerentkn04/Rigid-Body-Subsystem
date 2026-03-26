#pragma once
#include <vector>
#include <cstdint>
#include <unordered_map>
#include <RigidPixelTypes.hpp>
#include <RegionType.hpp>
#include <regionScratch.hpp>

namespace rigid {

struct StructuralNode {
    RegionID id;
    uint32_t pixel_count;
    CellAABB bounds;
    uint32_t group_id;
    bool touches_floor;
    std::vector<uint32_t> neighbor_indices; // indices into StructuralGraph::nodes
};

struct StructuralGraph {
    std::vector<StructuralNode> nodes;
    std::unordered_map<RegionID, uint32_t> id_to_node_idx;
};

void graph_build(
    StructuralGraph& graph,
    const std::unordered_map<RegionID, RegionRecord>& active_regions,
    const std::vector<RegionIndex>& label_grid,
    const std::vector<RegionID>& index_to_id,
    int world_width,
    int world_height);

} // namespace rigid
