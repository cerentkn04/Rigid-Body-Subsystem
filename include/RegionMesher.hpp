#pragma once
#include <vector>
#include <cstdint>
#include "RigidPixelTypes.hpp"

namespace rigid {

struct Vertex { float x, y; };

struct Contour {
    std::vector<Vertex> points;
    bool is_hole;
};

struct ConvexPiece {
    std::vector<Vertex> points;
};

struct RegionGeometry {
    uint32_t region_id;
    uint64_t version;
    uint64_t topology_hash = 0;
    std::vector<Contour> contours;
    std::vector<ConvexPiece> convex_pieces;
    struct { float x, y; } center;
    bool empty() const { return contours.empty(); }
};

RegionGeometry build_geometry(
    uint32_t region_id,
    const CellAABB& bounds,
    const std::vector<uint32_t>& label_grid,
    int grid_width,
    int grid_height,
    Vertex stableCenter);

} // namespace rigid
