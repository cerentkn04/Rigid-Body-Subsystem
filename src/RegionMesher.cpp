#include "RegionMesher.hpp"
#include <algorithm>
#include <cmath>

namespace rigid {

namespace {
static float PerpendicularDistance(const Vertex& p, const Vertex& a, const Vertex& b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float mag = std::sqrt(dx * dx + dy * dy);
    if (mag < 1e-5f) return std::sqrt(std::pow(p.x - a.x, 2) + std::pow(p.y - a.y, 2));
    return std::abs(dx * (a.y - p.y) - (a.x - p.x) * dy) / mag;
}

static void DouglasPeucker(const std::vector<Vertex>& points, float epsilon, std::vector<Vertex>& out) {
    if (points.size() < 3) {
        out = points;
        return;
    }

    size_t index = 0;
    float maxDist = 0;
    for (size_t i = 1; i < points.size() - 1; ++i) {
        float dist = PerpendicularDistance(points[i], points.front(), points.back());
        if (dist > maxDist) {
            index = i;
            maxDist = dist;
        }
    }

    if (maxDist > epsilon) {
        std::vector<Vertex> res1, res2;
        std::vector<Vertex> firstPart(points.begin(), points.begin() + index + 1);
        std::vector<Vertex> secondPart(points.begin() + index, points.end());
        DouglasPeucker(firstPart, epsilon, res1);
        DouglasPeucker(secondPart, epsilon, res2);
        out.assign(res1.begin(), res1.end() - 1);
        out.insert(out.end(), res2.begin(), res2.end());
    } else {
        out = { points.front(), points.back() };
    }
}


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
//
//
//


static float CalculateSignedArea(const std::vector<Vertex>& points) {
    float area = 0.0f;
    for (size_t i = 0; i < points.size(); ++i) {
        const Vertex& v1 = points[i];
        const Vertex& v2 = points[(i + 1) % points.size()];
        area += (v1.x * v2.y) - (v2.x * v1.y);
    }
    return area * 0.5f;
}
bool IsReflex(const std::vector<Vertex>& poly, int i) {
    int prev = (i == 0) ? (int)poly.size() - 1 : i - 1;
    int next = (i + 1) % poly.size();
    
    // Use a small epsilon to ignore "nearly flat" corners 
    // created by floating point noise or marching squares.
    float val = (poly[i].x - poly[prev].x) * (poly[next].y - poly[i].y) - 
                (poly[i].y - poly[prev].y) * (poly[next].x - poly[i].x);
    return val < -0.001f; // Adjust based on your coordinate scale
}
static void RemoveCollinearPoints(std::vector<Vertex>& points) {
    if (points.size() < 3) return;
    std::vector<Vertex> result;
    for (size_t i = 0; i < points.size(); ++i) {
        const Vertex& prev = points[(i == 0) ? points.size() - 1 : i - 1];
        const Vertex& curr = points[i];
        const Vertex& next = points[(i + 1) % points.size()];

        // Calculate the cross product of (curr-prev) and (next-curr)
        float area = (curr.x - prev.x) * (next.y - curr.y) - (curr.y - prev.y) * (next.x - curr.x);
        
        // If area is nearly 0, the points are collinear
        if (std::abs(area) > 1e-4f) {
            result.push_back(curr);
        }
    }
    points = std::move(result);
}
namespace bayazit {
    using namespace rigid;

// Version A: Check visibility between two arbitrary points
    bool IsVisible(const std::vector<Vertex>& poly, const Vertex& p1, const Vertex& p2) {
        for (size_t i = 0; i < poly.size(); ++i) {
            size_t next = (i + 1) % poly.size();
            
            Vertex v1 = poly[i];
            Vertex v2 = poly[next];

            // Ignore edges that share a vertex with the segment endpoints to avoid false positives
            if ((std::abs(v1.x - p1.x) < 1e-4f && std::abs(v1.y - p1.y) < 1e-4f) ||
                (std::abs(v2.x - p1.x) < 1e-4f && std::abs(v2.y - p1.y) < 1e-4f) ||
                (std::abs(v1.x - p2.x) < 1e-4f && std::abs(v1.y - p2.y) < 1e-4f) ||
                (std::abs(v2.x - p2.x) < 1e-4f && std::abs(v2.y - p2.y) < 1e-4f)) continue;

            float denominator = (p2.x - p1.x) * (v2.y - v1.y) - (p2.y - p1.y) * (v2.x - v1.x);
            if (std::abs(denominator) < 1e-6f) continue;

            float ua = ((v2.x - v1.x) * (p1.y - v1.y) - (v2.y - v1.y) * (p1.x - v1.x)) / denominator;
            float ub = ((p2.x - p1.x) * (p1.y - v1.y) - (p2.y - p1.y) * (p1.x - v1.x)) / denominator;

            if (ua > 0.0f && ua < 1.0f && ub > 0.0f && ub < 1.0f) return false;
        }
        return true;
    }

    // Version B: Keep your index-based version for Decompose
    bool IsVisible(const std::vector<Vertex>& poly, int a, int b) {
        return IsVisible(poly, poly[a], poly[b]);
    }



    bool Left(const Vertex& a, const Vertex& b, const Vertex& c) {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) > 0;
    }

    bool IsReflex(const std::vector<Vertex>& poly, int i) {
        int prev = (i == 0) ? (int)poly.size() - 1 : i - 1;
        int next = (i + 1) % poly.size();
        return !Left(poly[prev], poly[i], poly[next]);
    }

    void Decompose(std::vector<Vertex> poly, std::vector<std::vector<Vertex>>& output) {
        if (poly.size() < 3) return;

        int reflexIdx = -1;
        for (int i = 0; i < (int)poly.size(); ++i) {
            if (IsReflex(poly, i)) {
                reflexIdx = i;
                break;
            }
        }

        // Base Case: Polygon is already convex
        if (reflexIdx == -1) {
            output.push_back(poly);
            return;
        }

        // Choose split vertex: Simplification of Bayazit
        // We find a vertex that is visible and furthest from the reflex point to resolve the concavity
int bestSplitIdx = -1;
        float minDist = 1e10f; // Use a high value for finding the closest split
        Vertex p = poly[reflexIdx];

        for (int i = 0; i < (int)poly.size(); ++i) {
            // Don't split with self or immediate neighbors
            if (i == reflexIdx || i == (reflexIdx + 1) % poly.size() || 
                i == (reflexIdx - 1 + (int)poly.size()) % poly.size()) continue;

            if (IsVisible(poly, reflexIdx, i)) {
                float dst = std::pow(poly[i].x - p.x, 2) + std::pow(poly[i].y - p.y, 2);
                if (dst < minDist) {
                    minDist = dst;
                    bestSplitIdx = i;
                }
            }
        }

        if (bestSplitIdx != -1) {
            std::vector<Vertex> poly1, poly2;
            // Build first sub-polygon
            for (int i = reflexIdx; i != (bestSplitIdx + 1) % poly.size(); i = (i + 1) % poly.size()) {
                poly1.push_back(poly[i]);
            }
            // Build second sub-polygon
            for (int i = bestSplitIdx; i != (reflexIdx + 1) % poly.size(); i = (i + 1) % poly.size()) {
                poly2.push_back(poly[i]);
            }

            // Safety check: ensure we actually split the polygon
            if (poly1.size() < poly.size() && poly2.size() < poly.size()) {
                Decompose(poly1, output);
                Decompose(poly2, output);
            } else {
                output.push_back(poly); // Fallback to avoid infinite loop
            }
        } else {
            output.push_back(poly);
        }
    }
}


static void BridgeHoles(std::vector<Vertex>& mainPoly, const std::vector<Contour>& contours) {
    for (const auto& hole : contours) {
        if (!hole.is_hole) continue;

        // 1. Find the rightmost point in the hole
        size_t holeIdx = 0;
        float maxHoleX = -1e10f;
        for (size_t i = 0; i < hole.points.size(); ++i) {
            if (hole.points[i].x > maxHoleX) {
                maxHoleX = hole.points[i].x;
                holeIdx = i;
            }
        }

        // 2. Find the best bridge point on mainPoly
        int bridgeIdx = -1;
        float minDistSq = 1e10f;
        Vertex hP = hole.points[holeIdx];

        for (size_t i = 0; i < mainPoly.size(); ++i) {
            // Basic visibility heuristic: bridge to the right
            if (mainPoly[i].x >= hP.x) {

              // ✅ Use your existing visibility check to ensure the bridge is valid
                if (bayazit::IsVisible(mainPoly, hP, mainPoly[i])) { 
                    float d2 = std::pow(mainPoly[i].x - hP.x, 2) + std::pow(mainPoly[i].y - hP.y, 2);
                    if (d2 < minDistSq) {
                        minDistSq = d2;
                        bridgeIdx = (int)i;
                    }
                }
            }
        }

        if (bridgeIdx != -1) {
            // 3. Prepare the Hole points (Ensuring CW winding for a CCW outer poly)
            std::vector<Vertex> holePoints = hole.points;
            
            // Logic: If outer is CCW (Area > 0), hole must be CW (Area < 0)
            float holeArea = CalculateSignedArea(holePoints);
            float mainArea = CalculateSignedArea(mainPoly);
            
            // If they have the same sign, reverse the hole
            if ((mainArea > 0 && holeArea > 0) || (mainArea < 0 && holeArea < 0)) {
                std::reverse(holePoints.begin(), holePoints.end());
                // After reversal, we must find the new index of our rightmost point
                maxHoleX = -1e10f;
                for (size_t i = 0; i < holePoints.size(); ++i) {
                    if (holePoints[i].x > maxHoleX) {
                        maxHoleX = holePoints[i].x;
                        holeIdx = i;
                    }
                }
            }

            // 4. Inject the bridge
            std::vector<Vertex> newPath;
            // a. Main up to bridge point
            for (int i = 0; i <= bridgeIdx; ++i) newPath.push_back(mainPoly[i]);
            
            // b. Hole loop (starting from holeIdx)
            for (size_t i = 0; i < holePoints.size(); ++i) {
                newPath.push_back(holePoints[(holeIdx + i) % holePoints.size()]);
            }
            
            // c. Close hole and bridge back
            newPath.push_back(holePoints[holeIdx]); // Back to hole start
            newPath.push_back(mainPoly[bridgeIdx]); // Back to main bridge point
            
            // d. Rest of main
            for (size_t i = bridgeIdx + 1; i < mainPoly.size(); ++i) {
                newPath.push_back(mainPoly[i]);
            }
            
            mainPoly = std::move(newPath);
        }
    }
}




static void SimplifyContour(std::vector<Vertex>& points) {
    if (points.size() < 5) return; 

    std::vector<Vertex> simplified;
    simplified.push_back(points[0]);

    for (size_t i = 1; i < points.size(); ++i) {
        const Vertex& last = simplified.back();
        const Vertex& curr = points[i];

        // If the point is closer than 3.0 units, Nuke it.
        // This is the "Detail" slider. Increase 9.0f to 16.0f for even less detail.
        float distSq = std::pow(curr.x - last.x, 2) + std::pow(curr.y - last.y, 2);
        if (distSq < 9.0f) continue; 

        simplified.push_back(curr);
    }

    if (simplified.size() >= 3) {
        points = std::move(simplified);
    }
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

RegionGeometry build_geometry(
    uint32_t regionID,
    const CellAABB& bounds,
    const std::vector<uint32_t>& labelGrid,
    int gridWidth,
    int gridHeight)
{
    RegionGeometry geo;
    geo.region_id = regionID;
    geo.version = 0;

    uint64_t hash = 0x811C9DC5;

    double centerX = 0.0;
    double centerY = 0.0;
    uint32_t count = 0;

    std::vector<Edge> edges;

    auto getLabel = [&](int ix, int iy) {
        if (iy < 0 || ix < 0 || ix >= gridWidth || iy >= gridHeight)
            return std::numeric_limits<uint32_t>::max();
        return labelGrid[iy * gridWidth + ix];
    };

    auto getEdgePoint = [&](int x, int y, int edgeIdx) -> IPoint {
        if (edgeIdx == 0) return { x * 2 + 1, y * 2 };
        if (edgeIdx == 1) return { x * 2 + 2, y * 2 + 1 };
        if (edgeIdx == 2) return { x * 2 + 1, y * 2 + 2 };
        if (edgeIdx == 3) return { x * 2,     y * 2 + 1 };
        return {0,0};
    };

    // =====================================================
    // 1️⃣ SCAN CELLS (center + hash + marching squares edges)
    // =====================================================
    for (int y = bounds.min_y - 1; y <= bounds.max_y +1; ++y) {
        for (int x = bounds.min_x - 1; x <= bounds.max_x +1; ++x) {
            if (getLabel(x, y) == regionID) {
              centerX += x;
                centerY += y;
                count++;
              geo.topology_hash = hash;
              if (count > 0) {
                geo.center.x = (float)(centerX / count);
                geo.center.y = (float)(centerY / count);
              } 
                uint32_t localX = x - bounds.min_x;
                uint32_t localY = y - bounds.min_y;
                uint32_t combined = (localX << 16) | localY;
                hash ^= combined;
                hash *= 0x01000193;
                geo.topology_hash = hash;

        }

             
                // Accumulate center
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
        if (edges[i].used) continue;
        Contour contour;
        IPoint start = edges[i].from;
        IPoint current = edges[i].to;
        edges[i].used = true;
        
        contour.points.push_back({
                start.x * 0.5f,
                start.y * 0.5f
            });
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

    // --- Inside GeometryExtractor::Build ---

// ... (Step 1-4: Marching Squares and Stitching) ...

std::vector<std::vector<Vertex>> convexPolygons;
auto outerIt = std::find_if(geo.contours.begin(), geo.contours.end(), 
                            [](const Contour& c) { return !c.is_hole; });

if (outerIt != geo.contours.end()) {
    std::vector<Vertex> mainPoly = outerIt->points;

    BridgeHoles(mainPoly, geo.contours);  
   // for (auto& hole : geo.contours) {
   //     if (!hole.is_hole) continue;
   // }

    std::vector<Vertex> optimizedPoly;
DouglasPeucker(mainPoly, 0.25f, optimizedPoly); 
RemoveCollinearPoints(optimizedPoly);
if (optimizedPoly.size() >= 3) {
        bayazit::Decompose(optimizedPoly, convexPolygons);
} else if (mainPoly.size() >= 3) {
        // Fallback to the unoptimized version so Box2D doesn't crash
        bayazit::Decompose(mainPoly, convexPolygons);
    }

}

// CRITICAL: Copy the resulting convex polygons into the geo record
for (const auto& polyPoints : convexPolygons) {
    if (polyPoints.size() >= 3) {
        std::vector<Vertex> localPoints = polyPoints;
        for (auto& pt : localPoints) {
            // Subtract ONCE here
            pt.x -= geo.center.x; 
            pt.y -= geo.center.y;
        }
        geo.convex_pieces.push_back({ localPoints });
    }
}

return geo;
   }

} // namespace rigid
