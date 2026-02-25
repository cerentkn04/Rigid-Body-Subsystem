#include "RegionExtractor.hpp"
#include "regionScratch.hpp"
#include <RegionType.hpp>
#include "RigidPixelWorldView.hpp"

#include <algorithm>
#include <cassert>

namespace rigid {

// ------------------------------------------------------------
// Implementation
// ------------------------------------------------------------
struct DSU {
    std::vector<RegionIndex> parent;
    void reset(size_t n) {
        if (parent.size() < n) parent.resize(n);
        // Use faster manual loop or memset if available
        for (size_t i = 0; i < n; ++i) parent[i] = (RegionIndex)i;
    }
    
    // Non-recursive find with path halving is faster for high-density grids
    inline RegionIndex find(RegionIndex i) {
        while (parent[i] != i) {
            parent[i] = parent[parent[i]]; // Path halving
            i = parent[i];
        }
        return i;
    }

    inline void unite(RegionIndex i, RegionIndex j) {
        RegionIndex root_i = find(i);
        RegionIndex root_j = find(j);
        if (root_i != root_j) {
            // Union by rank is overkill here; simple assignment is fine
            if (root_i < root_j) parent[root_j] = root_i;
            else parent[root_i] = root_j;
        }
    }
};


struct RegionExtractor::Impl {
    std::vector<RegionIndex> label_grid;
    DSU dsu;

    void extract(const world::WorldView& world, std::vector<RegionBuildRecord>& out_records) {
        const int width = world.width;
        const int height = world.height;
        const int cell_count = width * height;

        if (label_grid.size() != (size_t)cell_count) label_grid.resize(cell_count);
        std::fill(label_grid.begin(), label_grid.end(), InvalidRegionIndex);
        out_records.clear();
        
        // Reset DSU with a safe upper bound of potential regions
        dsu.reset(cell_count / 2 + 1); 
        RegionIndex next_label = 0;

        // --- PASS 1: LINEAR SCAN (Cache Friendly) ---
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                auto current_group = world.group_id_at(x, y);
                if (world.solidity_at(x, y) != world::CellSolidity::Solid) continue;
    
                const int idx = y * width + x;
                // Check 4 neighbors (West, North-West, North, North-East)
                // These are already processed in a top-down scan
                //
if (x > 0 && world.solidity_at(x-1, y) == world::CellSolidity::Solid) {
    if (world.group_id_at(x-1, y) != current_group) {
    printf("[Extractor] Prevented merge at (%d,%d): Group %u vs %u\n", x, y, current_group, world.group_id_at(x-1, y));
}
}
                          RegionIndex L  = (x > 0 && world.group_id_at(x - 1, y) == current_group) 
                         ? label_grid[idx - 1] : InvalidRegionIndex;
                         
        RegionIndex NW = (x > 0 && y > 0 && world.group_id_at(x - 1, y - 1) == current_group) 
                         ? label_grid[idx - width - 1] : InvalidRegionIndex;
                         
        RegionIndex N  = (y > 0 && world.group_id_at(x, y - 1) == current_group) 
                         ? label_grid[idx - width] : InvalidRegionIndex;
                         
        RegionIndex NE = (x < width - 1 && y > 0 && world.group_id_at(x + 1, y - 1) == current_group) 
                         ? label_grid[idx - width + 1] : InvalidRegionIndex;

        RegionIndex neighbors[] = { L, NW, N, NE };
        RegionIndex target = InvalidRegionIndex;
                


 

                // Find the smallest valid label among neighbors
                for (auto n : neighbors) {
                    if (n != InvalidRegionIndex) {
                        if (target == InvalidRegionIndex || n < target) target = n;
                    }
                }

                if (target == InvalidRegionIndex) {
                    label_grid[idx] = next_label++;
                } else {
                    label_grid[idx] = target;
                    // Record that all neighbor labels are actually the same region
                    for (auto n : neighbors) {
                        if (n != InvalidRegionIndex) dsu.unite(target, n);
                    }
                }
            }
        }

        // --- PASS 2: FLATTEN & RECORD ---
        // Map DSU roots to dense out_records indices
        //
static std::vector<RegionIndex> root_to_final_idx;
    if (root_to_final_idx.size() < next_label) root_to_final_idx.resize(next_label);
    std::fill(root_to_final_idx.begin(), root_to_final_idx.begin() + next_label, InvalidRegionIndex);

    out_records.clear();
    // Pre-reserve based on typical rock density to prevent multiple reallocs
    out_records.reserve(std::max((size_t)100, (size_t)next_label / 4));

    for (int y = 0; y < height; ++y) {
        const int row_offset = y * width;
        for (int x = 0; x < width; ++x) {
            const int idx = row_offset + x;
            if (label_grid[idx] == InvalidRegionIndex) continue;

            RegionIndex root = dsu.find(label_grid[idx]);
            
            if (root_to_final_idx[root] == InvalidRegionIndex) {
                root_to_final_idx[root] = (RegionIndex)out_records.size();
                RegionBuildRecord rec{x, y, x, y, 0}; 
                out_records.push_back(rec);
            }

            RegionIndex final_idx = root_to_final_idx[root];
            label_grid[idx] = final_idx;

            auto& rec = out_records[final_idx];
            rec.pixel_count++;
            if (x < rec.bounds.min_x) rec.bounds.min_x = x;
            if (x > rec.bounds.max_x) rec.bounds.max_x = x;
            if (y < rec.bounds.min_y) rec.bounds.min_y = y; // y is always >= rec.min_y in this scan
            if (y > rec.bounds.max_y) rec.bounds.max_y = y;
        }
    }


            }
};


;

// ------------------------------------------------------------
// Public API
// ------------------------------------------------------------

RegionExtractor::RegionExtractor()
    : m_impl(new Impl{}) {}

RegionExtractor::~RegionExtractor() {
    delete m_impl;
}

void RegionExtractor::extract(const world::WorldView& world,
                              std::vector<RegionBuildRecord>& out_records) {
    m_impl->extract(world, out_records);
}

const std::vector<RegionIndex>&
RegionExtractor::label_grid() const {
    return m_impl->label_grid;
}

} // namespace rigid

