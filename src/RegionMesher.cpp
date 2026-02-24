#include "RegionMesher.hpp"
#include <algorithm>
#include <cmath>

namespace rigid {

namespace {

// =====================
// Internal Stitch Types
// =====================

struct IPoint {
    int x;
    int y;

    bool operator<(const IPoint& other) const {
        return (x < other.x) || (x == other.x && y < other.y);
    }

    bool operator==(const IPoint& other) const {
        return x == other.x && y == other.y;
    }
};

struct Edge {
    IPoint from;
    IPoint to;
    bool used = false;
};

// =====================
// Helpers
// =====================
static void SimplifyContour(std::vector<Vertex>& points) {
    if (points.size() < 3) return;

    std::vector<Vertex> simplified;
    simplified.reserve(points.size());

    for (size_t i = 0; i < points.size(); ++i) {
        const Vertex& prev = (i == 0) ? points.back() : points[i - 1];
        const Vertex& curr = points[i];
        const Vertex& next = points[(i + 1) % points.size()];

        // Calculas only happens when drawing at the bottomte 2D Cross Product of vectors (prev->curr) and (curr->next)
        float dx1 = curr.x - prev.x;
        float dy1 = curr.y - prev.y;
        float dx2 = next.x - curr.x;
        float dy2 = next.y - curr.y;

        // If cross product is ~0, the points are collinear
        float cross = dx1 * dy2 - dy1 * dx2;
        if (std::abs(cross) > 1e-5f) {
            simplified.push_back(curr);
        }
    }
    points = std::move(simplified);
}
static float CalculateSignedArea(const std::vector<Vertex>& points) {
    float area = 0.0f;
    for (size_t i = 0; i < points.size(); ++i) {
        const Vertex& v1 = points[i];
        const Vertex& v2 = points[(i + 1) % points.size()];
        area += (v1.x * v2.y) - (v2.x * v1.y);
    }
    return area * 0.5f;
}

} // anonymous namespace


// =====================
// Marching Squares Table
// =====================

static const int CASE_TABLE[16][4] = {
    { -1, -1, -1, -1 }, { 3, 2, -1, -1 }, { 2, 1, -1, -1 }, { 3, 1, -1, -1 },
    { 1, 0, -1, -1 },   { 3, 0, 1, 2 },   { 2, 0, -1, -1 }, { 3, 0, -1, -1 },
    { 0, 3, -1, -1 },   { 0, 2, -1, -1 }, { 0, 1, 2, 3 },   { 0, 1, -1, -1 },
    { 1, 3, -1, -1 },   { 1, 2, -1, -1 }, { 2, 3, -1, -1 }, { -1, -1, -1, -1 }
};


// =====================
// Geometry Builder
// =====================

RegionGeometry GeometryExtractor::Build(
    uint32_t regionID,
    const CellAABB& bounds,
    const std::vector<uint32_t>& labelGrid,
    int gridWidth,
    int gridHeight)
{
    RegionGeometry geo;
    geo.region_id = regionID;
    geo.version = 0;

    std::vector<Edge> edges;
    edges.reserve(256); // optional small reserve

    auto getLabel = [&](int ix, int iy) {
        if (iy < 0 || ix < 0 || ix >= gridWidth || iy >= gridHeight)
             return std::numeric_limits<uint32_t>::max();
        return labelGrid[iy * gridWidth + ix];
    };

    auto getEdgePoint = [&](int x, int y, int edgeIdx) -> IPoint {
        if (edgeIdx == 0) return { x * 2 + 1, y * 2 };     // North
        if (edgeIdx == 1) return { x * 2 + 2, y * 2 + 1 }; // East
        if (edgeIdx == 2) return { x * 2 + 1, y * 2 + 2 }; // South
        if (edgeIdx == 3) return { x * 2,     y * 2 + 1 }; // West
        return {0, 0};
    };

    // =====================
    // 1️⃣ Collect Edges
    // =====================

    for (int y = bounds.min_y - 1; y <= bounds.max_y; ++y) {
        for (int x = bounds.min_x - 1; x <= bounds.max_x; ++x) {

            int mask = 0;
            if (getLabel(x, y)         == regionID) mask |= 8;
            if (getLabel(x + 1, y)     == regionID) mask |= 4;
            if (getLabel(x + 1, y + 1) == regionID) mask |= 2;
            if (getLabel(x, y + 1)     == regionID) mask |= 1;

            if (mask == 0 || mask == 15)
                continue;

            const int* caseEdges = CASE_TABLE[mask];

            for (int i = 0; i < 4; i += 2) {
                if (caseEdges[i] == -1)
                    break;

                IPoint a = getEdgePoint(x, y, caseEdges[i]);
                IPoint b = getEdgePoint(x, y, caseEdges[i + 1]);

                edges.push_back({a, b, false});
            }
        }
    }

    if (edges.empty())
        return geo;

    // =====================
    // 2️⃣ Sort by "from"
    // =====================

    std::sort(edges.begin(), edges.end(),
        [](const Edge& a, const Edge& b) {
            return a.from < b.from;
        });

    auto find_first_edge = [&](const IPoint& p) {
        return std::lower_bound(edges.begin(), edges.end(), p,
            [](const Edge& e, const IPoint& value) {
                return e.from < value;
            });
    };

    // =====================
    // 3️⃣ Stitch Loops
    // =====================

    for (size_t i = 0; i < edges.size(); ++i) {

        if (edges[i].used)
            continue;

        Contour contour;

        IPoint start = edges[i].from;
        IPoint current = edges[i].to;
        edges[i].used = true;

        contour.points.push_back({
            current.x * 0.5f ,
            current.y * 0.5f 
        });

        while (true) {

            auto it = find_first_edge(current);

            bool found = false;

            for (; it != edges.end() && it->from == current; ++it) {
                if (!it->used) {
                    it->used = true;
                    current = it->to;

                    contour.points.push_back({
                        current.x * 0.5f,
                        current.y * 0.5f
                    });

                    found = true;
                    break;
                }
            }

            if (!found || current == start)
                break;
        }

        if (contour.points.size() >= 3)
            geo.contours.push_back(std::move(contour));
    }

    // =====================
    // 4️⃣ Normalize Winding
    // =====================
// Inside Step 4, after winding/reversing:
    if (!geo.contours.empty()) {

        float maxAbsArea = 0.0f;
        int outerIndex = -1;

        for (size_t i = 0; i < geo.contours.size(); ++i) {
            float area = CalculateSignedArea(geo.contours[i].points);
            float absArea = std::abs(area);

            if (absArea > maxAbsArea) {
                maxAbsArea = absArea;
                outerIndex = static_cast<int>(i);
            }
        }

        for (size_t i = 0; i < geo.contours.size(); ++i) {
            auto& c = geo.contours[i];
            float area = CalculateSignedArea(c.points);

            bool isOuter = (static_cast<int>(i) == outerIndex);

            if (isOuter) {
                c.is_hole = false;
                if (area < 0.0f)
                    std::reverse(c.points.begin(), c.points.end());
            } else {
                c.is_hole = true;
                if (area > 0.0f)
                    std::reverse(c.points.begin(), c.points.end());
            }
            SimplifyContour(c.points);
        }
    }

    return geo;
}

} // namespace rigind
