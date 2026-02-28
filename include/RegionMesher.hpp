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
    std::vector<ConvexPiece> convex_pieces;// Flat vector
    struct { float x, y; } center;
    bool empty() const { return contours.empty(); }
};

class GeometryExtractor {
public:
    static RegionGeometry Build(
        uint32_t regionID, 
        const CellAABB& bounds, 
        const std::vector<uint32_t>& labelGrid, // Flat vector
        int gridWidth,
        int gridHeight
    );
};

} // namespace rigid
