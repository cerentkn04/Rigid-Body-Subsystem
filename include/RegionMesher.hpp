#pragma once
#include <vector>
#include "RigidPixelTypes.hpp"
#include "RegionType.hpp"
#include "regionScratch.hpp"

namespace rigid {

struct RegionMesh {
    RegionID id;
    std::vector<float> vertices;   // x0,y0,x1,y1,...
    std::vector<uint32_t> indices; // optional
};
struct Vec2 {
        float x, y;
    };

struct RegionMesherConfig {
    bool generate_indices = false;
};

class RegionMesher {
public:
    explicit RegionMesher(const RegionMesherConfig&);

    // Build geometry for ONE region
    void build_region(
        RegionID id,
        const CellAABB& bounds,
        const std::vector<RegionIndex>& label_grid,
        int grid_width,
        int grid_height,
        std::vector<RegionMesh>& out_meshes
    );
};

} // namespace rigid

