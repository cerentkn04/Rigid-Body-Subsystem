#pragma once
#include <vector>
#include <cstdint>
#include <unordered_map>
#include <RigidPixelTypes.hpp>
#include <RegionType.hpp>
#include <regionScratch.hpp>
namespace rigid {

    // A single "Fact" about a region in the world
    struct StructuralNode {
        RegionID id;
        uint32_t pixel_count;
        CellAABB bounds;
        uint32_t group_id;
        bool touches_floor;

        // Adjacency: Indices of other nodes this node is physically touching
        // This is what allows Noita-style stability propagation
        std::vector<uint32_t> neighbor_indices; 
    };

    // The Engine-level map of how regions connect to each other
    struct StructuralGraph {
        std::vector<StructuralNode> nodes;

        // Map to quickly find a node index if you only have a RegionID
        std::unordered_map<RegionID, uint32_t> id_to_node_idx;

        // Accessor for policies
        const std::vector<StructuralNode>& get_nodes() const { return nodes; }

        // The "Heavy Lifting" function (implemented in .cpp)
        // This scans the label grid to find neighbors
        void build(
            const std::unordered_map<RegionID, RegionRecord>& active_regions,
            const std::vector<RegionIndex>& label_grid,
            int world_width, 
            int world_height
        );
    };

} // namespace rigid
