#include "StructuralTracker.hpp"
#include <algorithm>
#include <deque>

static bool intersects(const rigid::CellAABB& a, const rigid::CellAABB& b) {
    return (a.min_x <= b.max_x && a.max_x >= b.min_x) &&
           (a.min_y <= b.max_y && a.max_y >= b.min_y);
}

void tracker_init_bins(StructuralTracker& st, int world_width, int world_height) {
    st.bins_x = (world_width  + StructuralTracker::BIN_SIZE - 1) / StructuralTracker::BIN_SIZE;
    st.bins_y = (world_height + StructuralTracker::BIN_SIZE - 1) / StructuralTracker::BIN_SIZE;
    st.bins.clear();
    st.bins.resize(st.bins_x * st.bins_y);
}

void tracker_propagate_dirt(StructuralTracker& st) {
    std::deque<uint32_t> queue;
    for (uint32_t i = 0; i < (uint32_t)st.dirty_flags.size(); ++i)
        if (st.dirty_flags[i] == 1) queue.push_back(i);

    while (!queue.empty()) {
        uint32_t di = queue.front(); queue.pop_front();
        const auto& db = st.influence_bounds[di];

        int bx0 = std::max(0, db.min_x / StructuralTracker::BIN_SIZE);
        int by0 = std::max(0, db.min_y / StructuralTracker::BIN_SIZE);
        int bx1 = std::min(st.bins_x - 1, db.max_x / StructuralTracker::BIN_SIZE);
        int by1 = std::min(st.bins_y - 1, db.max_y / StructuralTracker::BIN_SIZE);

        for (int by = by0; by <= by1; ++by) {
            for (int bx = bx0; bx <= bx1; ++bx) {
                for (uint32_t ni : st.bins[by * st.bins_x + bx].region_indices) {
                    if (st.dirty_flags[ni]) continue;
                    if (intersects(db, st.influence_bounds[ni])) {
                        st.dirty_flags[ni] = 1;
                        queue.push_back(ni);
                    }
                }
            }
        }
    }
}
