#include "RegionExtractor.hpp"
#include "RigidPixelWorldView.hpp"
#include <algorithm>

namespace rigid {

struct DSU {
    std::vector<RegionIndex> parent;
    void reset(size_t n) {
        if (parent.size() < n) parent.resize(n);
        for (size_t i = 0; i < n; ++i) parent[i] = (RegionIndex)i;
    }
    inline RegionIndex find(RegionIndex i) {
        while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; }
        return i;
    }
    inline void unite(RegionIndex i, RegionIndex j) {
        RegionIndex ri = find(i), rj = find(j);
        if (ri != rj) { if (ri < rj) parent[rj] = ri; else parent[ri] = rj; }
    }
};

static DSU s_dsu;
static std::vector<RegionIndex> s_root_to_final;

void extractor_extract(
    ExtractorState& state,
    const world::WorldView& world,
    std::vector<RegionBuildRecord>& out_records)
{
    const int width  = world.width;
    const int height = world.height;
    const int total  = width * height;

    state.label_grid.resize(total);
    std::fill(state.label_grid.begin(), state.label_grid.end(), InvalidRegionIndex);
    out_records.clear();

    s_dsu.reset(total / 2 + 1);
    RegionIndex next_label = 0;

    // Returns true if two solid cells should be considered the same region.
    // Authored cells (object_id > 0) connect to any neighbour with the same object_id.
    // Runtime cells (object_id == 0) connect only to neighbours with the same group_id.
    auto can_connect = [&](int ax, int ay, int bx, int by) -> bool {
        world::ObjectID oa = world.object_id_at(ax, ay);
        world::ObjectID ob = world.object_id_at(bx, by);
        if (oa != 0 && oa == ob) return true;               // same authored object
        if (oa == 0 && ob == 0 && state.merge_same_type)
            return world.group_id_at(ax, ay) == world.group_id_at(bx, by); // same runtime material
        return false;
    };

    // Pass 1: DSU labeling (left, NW, N, NE neighbors already processed)
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (world.solidity_at(x, y) != world::CellSolidity::Solid) continue;
            const int idx = y * width + x;

            RegionIndex L  = (x > 0           && can_connect(x, y, x-1, y  )) ? state.label_grid[idx-1]         : InvalidRegionIndex;
            RegionIndex NW = (x > 0 && y > 0  && can_connect(x, y, x-1, y-1)) ? state.label_grid[idx-width-1]   : InvalidRegionIndex;
            RegionIndex N  = (y > 0           && can_connect(x, y, x,   y-1)) ? state.label_grid[idx-width]      : InvalidRegionIndex;
            RegionIndex NE = (x < width-1 && y > 0 && can_connect(x, y, x+1, y-1)) ? state.label_grid[idx-width+1] : InvalidRegionIndex;

            RegionIndex neighbors[] = { L, NW, N, NE };
            RegionIndex target = InvalidRegionIndex;
            for (auto n : neighbors)
                if (n != InvalidRegionIndex && (target == InvalidRegionIndex || n < target)) target = n;

            if (target == InvalidRegionIndex) {
                state.label_grid[idx] = next_label++;
            } else {
                state.label_grid[idx] = target;
                for (auto n : neighbors)
                    if (n != InvalidRegionIndex) s_dsu.unite(target, n);
            }
        }
    }

    // Pass 2: flatten DSU roots to dense indices, build records
    if (s_root_to_final.size() < next_label) s_root_to_final.resize(next_label);
    std::fill(s_root_to_final.begin(), s_root_to_final.begin() + next_label, InvalidRegionIndex);
    out_records.reserve(std::max((size_t)16, (size_t)next_label / 4));

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int idx = y * width + x;
            if (state.label_grid[idx] == InvalidRegionIndex) continue;

            RegionIndex root = s_dsu.find(state.label_grid[idx]);
            if (s_root_to_final[root] == InvalidRegionIndex) {
                s_root_to_final[root] = (RegionIndex)out_records.size();
                // Capture group_id from this first pixel of the region
                RegionBuildRecord rec{};
                rec.bounds    = { x, y, x, y };
                rec.group_id  = (uint32_t)world.group_id_at(x, y);
                out_records.push_back(rec);
            }

            RegionIndex fi = s_root_to_final[root];
            state.label_grid[idx] = fi;

            auto& rec = out_records[fi];
            rec.pixel_count++;
            if (x < rec.bounds.min_x) rec.bounds.min_x = x;
            if (x > rec.bounds.max_x) rec.bounds.max_x = x;
            if (y > rec.bounds.max_y) rec.bounds.max_y = y;
        }
    }
}

} // namespace rigid
