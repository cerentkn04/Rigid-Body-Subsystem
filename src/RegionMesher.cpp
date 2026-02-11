#include "RegionMesher.hpp"
#include <map>
#include <cmath>
#include <algorithm>

namespace rigid {

static float CalculateSignedArea(const std::vector<Vertex>& points) {
    float area = 0.0f;
    for (size_t i = 0; i < points.size(); i++) {
        const Vertex& v1 = points[i];
        const Vertex& v2 = points[(i + 1) % points.size()];
        area += (v1.x * v2.y) - (v2.x * v1.y);
    }
    return area * 0.5f;
}

static const int CASE_TABLE[16][4] = {
    { -1, -1, -1, -1 }, { 3, 2, -1, -1 }, { 2, 1, -1, -1 }, { 3, 1, -1, -1 },
    { 1, 0, -1, -1 },   { 3, 0, 1, 2 },   { 2, 0, -1, -1 }, { 3, 0, -1, -1 },
    { 0, 3, -1, -1 },   { 0, 2, -1, -1 }, { 0, 1, 2, 3 },   { 0, 1, -1, -1 },
    { 1, 3, -1, -1 },   { 1, 2, -1, -1 }, { 2, 3, -1, -1 }, { -1, -1, -1, -1 }
};

RegionGeometry GeometryExtractor::Build(
    uint32_t regionID, 
    const CellAABB& bounds, 
    const std::vector<uint32_t>& labelGrid,
    int gridWidth) 
{
    RegionGeometry geo;
    geo.region_id = regionID;
    geo.version = 0;

    std::map<std::pair<int, int>, std::pair<int, int>> segments;

    // FIX 1: Correct loop variables. Scan the padded influence bounds.
    // We go from min to max (inclusive of the edge check)
    for (int y = bounds.min_y - 1; y <= bounds.max_y; ++y) {
        for (int x = bounds.min_x - 1; x <= bounds.max_x; ++x) {
            
            auto getLabel = [&](int ix, int iy) {
                if (iy < 0 || ix < 0 || ix >= gridWidth || (iy * gridWidth + ix) >= (int)labelGrid.size()) 
                    return 0u;
                return labelGrid[iy * gridWidth + ix];
            };

            // FIX 2: Standard Marching Squares sampling
            int mask = 0;
            if (getLabel(x, y)         == regionID) mask |= 8; // Top-Left
            if (getLabel(x + 1, y)     == regionID) mask |= 4; // Top-Right
            if (getLabel(x + 1, y + 1) == regionID) mask |= 2; // Bottom-Right
            if (getLabel(x, y + 1)     == regionID) mask |= 1; // Bottom-Left

            if (mask == 0 || mask == 15) continue;

            const int* edges = CASE_TABLE[mask];
            for (int i = 0; i < 4; i += 2) {
                if (edges[i] == -1) break;

                auto getEdgePoint = [&](int edgeIdx) -> std::pair<int, int> {
                    // Coordinates scaled by 2 to keep midpoints as integers
                    if (edgeIdx == 0) return { x * 2 + 1, y * 2 };     // North side
                    if (edgeIdx == 1) return { x * 2 + 2, y * 2 + 1 }; // East side
                    if (edgeIdx == 2) return { x * 2 + 1, y * 2 + 2 }; // South side
                    if (edgeIdx == 3) return { x * 2,     y * 2 + 1 }; // West side
                    return {0, 0};
                };

                segments[getEdgePoint(edges[i])] = getEdgePoint(edges[i+1]);
            }
        }
    }
   
    // STITCH: Join segments into closed loops
    while (!segments.empty()) {
        Contour contour;
        auto it = segments.begin();
        std::pair<int, int> startPt = it->first;
        std::pair<int, int> currentPt = it->second;
        
        // Convert integer 2x space back to cell space floats (+0.5 is handled by the 2x scale)
        contour.points.push_back({currentPt.first * 0.5f + 0.5f, currentPt.second * 0.5f + 0.5f});
        segments.erase(it);

        while (true) {
            auto nextIt = segments.find(currentPt);
            if (nextIt != segments.end()) {
                currentPt = nextIt->second;
                contour.points.push_back({currentPt.first * 0.5f + 0.5f, currentPt.second * 0.5f + 0.5f});
                segments.erase(nextIt);
                if (currentPt == startPt) break;
            } else { break; }
        }

        float area = CalculateSignedArea(contour.points);
        if (std::abs(area) > 0.01f) {
            contour.is_hole = (area < 0); 
            geo.contours.push_back(std::move(contour));
        }
    }
    return geo;
}

} // namespace rigid
