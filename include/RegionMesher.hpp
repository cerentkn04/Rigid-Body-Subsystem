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

struct RegionGeometry {
    uint32_t region_id;
    uint64_t version;
    std::vector<Contour> contours;
    bool empty() const { return contours.empty(); }
};

class GeometryExtractor {
public:
    static RegionGeometry Build(
        uint32_t regionID, 
        const CellAABB& bounds, 
        const std::vector<uint32_t>& labelGrid, // Flat vector
        int gridWidth                           // Pass width for index math
    );
};

} // namespace rigid
