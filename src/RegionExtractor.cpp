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

struct RegionExtractor::Impl {
    std::vector<RegionIndex> label_grid;
    std::vector<RegionPixel> stack;

    void extract(const world::WorldView& world,
                 std::vector<RegionBuildRecord>& out_records) {
        const int width  = world.width;
        const int height = world.height;
        const int cell_count = width * height;

        // --- Scratch setup ---
        if (label_grid.size() != static_cast<size_t>(cell_count)) {
            label_grid.resize(cell_count);
        }

        std::fill(label_grid.begin(), label_grid.end(), InvalidRegionIndex);
        out_records.clear();

        // --- Deterministic scan ---
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                const int idx = y * width + x;

                if (world.solidity_at(x, y) != world::CellSolidity::Solid)
                    continue;

                if (label_grid[idx] != InvalidRegionIndex)
                    continue;

                const RegionIndex region =
                    static_cast<RegionIndex>(out_records.size());

                flood_fill(world, x, y, region, out_records);
            }
        }
    }

private:
    void flood_fill(const world::WorldView& world,
                    int seed_x, int seed_y,
                    RegionIndex region,
                    std::vector<RegionBuildRecord>& out_records) {
        const int width  = world.width;
        const int height = world.height;

        RegionBuildRecord record{};
        record.min_x = record.max_x = seed_x;
        record.min_y = record.max_y = seed_y;
        record.pixel_count = 0;

        stack.clear();
        stack.push_back(RegionPixel{ seed_x, seed_y });
        label_grid[seed_y * width + seed_x] = region;

        // --- 8-connected flood fill ---
        while (!stack.empty()) {
          RegionPixel c = stack.back(); 
          stack.pop_back();

            record.pixel_count++;

            record.min_x = std::min(record.min_x, c.x);
            record.max_x = std::max(record.max_x, c.x);
            record.min_y = std::min(record.min_y, c.y);
            record.max_y = std::max(record.max_y, c.y);

            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;

                    const int nx = c.x + dx;
                    const int ny = c.y + dy;

                    if (nx < 0 || ny < 0 || nx >= width || ny >= height)
                        continue;

                    const int nidx = ny * width + nx;

                    if (label_grid[nidx] != InvalidRegionIndex)
                        continue;

                    if (world.solidity_at(nx, ny) != world::CellSolidity::Solid)
                        continue;

                    label_grid[nidx] = region;
                    stack.push_back(RegionPixel{ nx, ny });
                }
            }
        }

        // Finalize record
        out_records.push_back(record);
    }
};

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

