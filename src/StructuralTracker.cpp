#include "StructuralTracker.hpp"
#include <algorithm>
#include <deque>


static bool intersects(const rigid::CellAABB& a, const rigid::CellAABB& b) {
    return (a.min_x <= b.max_x && a.max_x >= b.min_x) &&
           (a.min_y <= b.max_y && a.max_y >= b.min_y);
}

void StructuralTracker::init_bins(int world_width, int world_height) {
    bins_x = (world_width + BIN_SIZE - 1) / BIN_SIZE;
    bins_y = (world_height + BIN_SIZE - 1) / BIN_SIZE;
    bins.clear();
    bins.resize(bins_x * bins_y);
}

void StructuralTracker::rebuild_bins() {
    for (auto& bin : bins) bin.region_indices.clear();

    for (uint32_t i = 0; i < ids.size(); ++i) {
        const auto& b = influence_bounds[i];
        int bx0 = std::max(0, b.min_x / BIN_SIZE);
        int by0 = std::max(0, b.min_y / BIN_SIZE);
        int bx1 = std::min(bins_x - 1, b.max_x / BIN_SIZE);
        int by1 = std::min(bins_y - 1, b.max_y / BIN_SIZE);

        for (int by = by0; by <= by1; ++by) {
            for (int bx = bx0; bx <= bx1; ++bx) {
                bins[by * bins_x + bx].region_indices.push_back(i);
            }
        }
    }
}

void StructuralTracker::propagate_dirt() {
    std::deque<uint32_t> work_queue;

    for (uint32_t i = 0; i < dirty_flags.size(); ++i) {
        if (dirty_flags[i] == 1) work_queue.push_back(i);
    }

    while (!work_queue.empty()) {
        uint32_t dirty_idx = work_queue.front();
        work_queue.pop_front();

        const auto& d_bounds = influence_bounds[dirty_idx];
        int bx0 = std::max(0, d_bounds.min_x / BIN_SIZE);
        int by0 = std::max(0, d_bounds.min_y / BIN_SIZE);
        int bx1 = std::min(bins_x - 1, d_bounds.max_x / BIN_SIZE);
        int by1 = std::min(bins_y - 1, d_bounds.max_y / BIN_SIZE);

        for (int by = by0; by <= by1; ++by) {
            for (int bx = bx0; bx <= bx1; ++bx) {
                const auto& bin = bins[by * bins_x + bx];
                for (uint32_t neighbor_idx : bin.region_indices) {
                    if (dirty_flags[neighbor_idx]) continue;

                    if (intersects(d_bounds, influence_bounds[neighbor_idx])) {
                        dirty_flags[neighbor_idx] = 1;
                        work_queue.push_back(neighbor_idx);
                    }
                }
            }
        }
    }
}
