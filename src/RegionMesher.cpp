#include "RegionMesher.hpp"
#include <array>
namespace rigid {

/**
 * MARCHING SQUARES LOOKUP TABLE
 * Each case (0-15) defines which edges of the 2x2 cell are intersected.
 * Edges: 0:Top, 1:Right, 2:Bottom, 3:Left
 * Vertices are relative to the top-left pixel of the 2x2 block.
 */
struct Edge { float x1, y1, x2, y2; };

// Helper to define midpoints of the 1x1 cell boundaries
// 0: Top, 1: Right, 2: Bottom, 3: Left
static const float M = 0.5f;
static const std::array<std::vector<int>, 16> CASE_TABLE = {{
    {},              // 0000: Empty
    {3, 2},          // 0001: Bottom-Left
    {2, 1},          // 0010: Bottom-Right
    {3, 1},          // 0011: Bottom-Full
    {1, 0},          // 0100: Top-Right
    {3, 0, 1, 2},    // 0101: Bottom-Left + Top-Right (Saddle)
    {2, 0},          // 0110: Right-Full
    {3, 0},          // 0111: Not Top-Left
    {0, 3},          // 1000: Top-Left
    {0, 2},          // 1001: Left-Full
    {0, 3, 2, 1},    // 1010: Top-Left + Bottom-Right (Saddle)
    {0, 1},          // 1011: Not Top-Right
    {1, 3},          // 1100: Top-Full
    {1, 2},          // 1101: Not Bottom-Right
    {2, 3},          // 1110: Not Bottom-Left
    {}               // 1111: Full
}};

RegionMesher::RegionMesher(const RegionMesherConfig& config) {}

void RegionMesher::build_region(
    RegionID id,
    const CellAABB& bounds,
    const std::vector<RegionIndex>& label_grid,
    int grid_width,
    int grid_height,
    std::vector<RegionMesh>& out_meshes) 
{
    RegionMesh mesh;
    mesh.id = id;

    // We scan a 1-pixel padded perimeter around the AABB to close the geometry
    for (int y = bounds.min_y - 1; y <= bounds.max_y; ++y) {
        for (int x = bounds.min_x - 1; x <= bounds.max_x; ++x) {
            
            // 1. Sample the 2x2 grid to form the 4-bit index
            // Index bits: [TL][TR][BR][BL]
            auto is_match = [&](int px, int py) {
                if (px < 0 || px >= grid_width || py < 0 || py >= grid_height) return false;
                return label_grid[py * grid_width + px] == static_cast<RegionIndex>(id);
            };

            int case_idx = 0;
            if (is_match(x, y))     case_idx |= 8; // Top-Left
            if (is_match(x+1, y))   case_idx |= 4; // Top-Right
            if (is_match(x+1, y+1)) case_idx |= 2; // Bottom-Right
            if (is_match(x, y+1))   case_idx |= 1; // Bottom-Left

            if (case_idx == 0 || case_idx == 15) continue;

            // 2. Map edges from the table to world-space coordinates
            const auto& edges = CASE_TABLE[case_idx];
            for (size_t i = 0; i < edges.size(); i += 2) {
                auto get_point = [&](int edge_id) {
                    switch(edge_id) {
                        case 0: return Vec2{x + M, (float)y};     // Top
                        case 1: return Vec2{(float)x + 1, y + M}; // Right
                        case 2: return Vec2{x + M, (float)y + 1}; // Bottom
                        case 3: return Vec2{(float)x, y + M};     // Left
                        default: return Vec2{0, 0};
                    }
                };

                Vec2 p1 = get_point(edges[i]);
                Vec2 p2 = get_point(edges[i+1]);

                mesh.vertices.push_back(p1.x);
                mesh.vertices.push_back(p1.y);
                mesh.vertices.push_back(p2.x);
                mesh.vertices.push_back(p2.y);
            }
        }
    }

    if (!mesh.vertices.empty()) {
        out_meshes.push_back(std::move(mesh));
    }
}

} // namespace rigid
