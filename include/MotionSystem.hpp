#pragma once
#include <RigidPixelTypes.hpp>
#include <unordered_map>
#include <RegionType.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <climits>
namespace rigid {


template<typename PixelType>
void ApplyRegionMotion(
    PixelType* pixel_grid,
    uint32_t* label_grid,
    int width, int height,
    uint64_t& world_revision,
    std::unordered_map<uint32_t, RegionRecord>& active_regions,
    PixelType empty_value)
{
    // Snapshot: canonical pixel positions in body-local space, built once per
    // region and reused every frame.  Eliminates per-frame quantization drift.
    struct Snapshot {
        int      lmin_x = 0, lmin_y = 0, lw = 0, lh = 0;
        std::vector<PixelType> grid;   // lw*lh, empty_value where no pixel
        uint32_t pixel_count = 0;
    };

    static std::vector<PixelType>  write_buffer;
    static std::vector<size_t>     src_indices;
    static std::unordered_map<uint32_t, Snapshot> snapshots;

    size_t grid_size = static_cast<size_t>(width) * height;

    if (write_buffer.size() != grid_size)
        write_buffer.resize(grid_size);

    std::copy(pixel_grid, pixel_grid + grid_size, write_buffer.begin());

    // Remove snapshots for dead regions
    for (auto it = snapshots.begin(); it != snapshots.end(); ) {
        it = active_regions.count(it->first) ? std::next(it) : snapshots.erase(it);
    }

    bool moved_any = false;

    for (auto& [id, region] : active_regions) {
        if (!region.is_dynamic) {
            region.prev_center_f.x = region.center_f.x;
            region.prev_center_f.y = region.center_f.y;
            continue;
        }

        int   dx        = region.pending_dx;
        int   dy        = region.pending_dy;
        float abs_angle = region.absolute_angle;
        int   pivot_x   = region.pending_pivot_x;
        int   pivot_y   = region.pending_pivot_y;

        bool has_translation = (dx != 0 || dy != 0);
        bool has_rotation    = (std::abs(region.pending_angle) > 1e-5f);

        if (std::abs(dx) > width || std::abs(dy) > height) {
            region.prev_center_f = region.center_f;
            continue;
        }

        if (!has_translation && !has_rotation) continue;
        moved_any = true;

        int new_cx = pivot_x + dx;
        int new_cy = pivot_y + dy;

        // ── Pass 1: collect source positions and clear write_buffer ───────────
        src_indices.clear();
        for (int y = region.bounds.min_y; y <= region.bounds.max_y; ++y) {
            if (y < 0 || y >= height) continue;
            for (int x = region.bounds.min_x; x <= region.bounds.max_x; ++x) {
                if (x < 0 || x >= width) continue;
                size_t idx = static_cast<size_t>(y) * width + x;
                if (label_grid[idx] != region.current_index) continue;
                src_indices.push_back(idx);
                write_buffer[idx] = empty_value;
            }
        }

        // ── Build snapshot when first seen.
        // Never rebuild from rasterized pixels — doing so lets rounding errors
        // in Pass 2 grow the local AABB by 1 pixel each rebuild, causing the
        // shape to inflate and eventually lose identity during fast rotation.
        auto& snap = snapshots[id];
        bool needs_rebuild = snap.grid.empty();

        if (needs_rebuild && !src_indices.empty()) {
            // Transform current world positions to body-local space using
            // R(-abs_angle) * (world - pivot).
            float cos_r = std::cos(abs_angle);
            float sin_r = std::sin(abs_angle);

            int lmin_x = INT_MAX, lmin_y = INT_MAX;
            int lmax_x = INT_MIN, lmax_y = INT_MIN;

            struct LP { int lx, ly; PixelType val; };
            std::vector<LP> local_pixels;
            local_pixels.reserve(src_indices.size());

            for (size_t src_idx : src_indices) {
                int wx = static_cast<int>(src_idx % static_cast<size_t>(width));
                int wy = static_cast<int>(src_idx / static_cast<size_t>(width));
                float fx = static_cast<float>(wx - pivot_x);
                float fy = static_cast<float>(wy - pivot_y);
                // R(-abs_angle) = [[cos, sin], [-sin, cos]]
                int lx = static_cast<int>(std::round( cos_r * fx + sin_r * fy));
                int ly = static_cast<int>(std::round(-sin_r * fx + cos_r * fy));
                lmin_x = std::min(lmin_x, lx); lmax_x = std::max(lmax_x, lx);
                lmin_y = std::min(lmin_y, ly); lmax_y = std::max(lmax_y, ly);
                local_pixels.push_back({lx, ly, pixel_grid[src_idx]});
            }

            snap.lmin_x     = lmin_x;
            snap.lmin_y     = lmin_y;
            snap.lw         = lmax_x - lmin_x + 1;
            snap.lh         = lmax_y - lmin_y + 1;
            snap.pixel_count = region.pixel_count;
            snap.grid.assign(static_cast<size_t>(snap.lw) * snap.lh, empty_value);
            for (const auto& p : local_pixels) {
                int gi = (p.lx - lmin_x) + (p.ly - lmin_y) * snap.lw;
                snap.grid[gi] = p.val;
            }
        }

        // ── Pass 2a: inverse mapping — scan destination AABB, pull from snapshot.
        // Each destination cell is looked up exactly once → no write collisions.
        // After this pass, run a forward fill (2b) for any source pixels that
        // no destination cell claimed, to close the rare boundary holes.
        if (!snap.grid.empty()) {
            float cos_a = std::cos(abs_angle);
            float sin_a = std::sin(abs_angle);

            float hw = static_cast<float>(
                std::max(std::abs(snap.lmin_x), std::abs(snap.lmin_x + snap.lw - 1)));
            float hh = static_cast<float>(
                std::max(std::abs(snap.lmin_y), std::abs(snap.lmin_y + snap.lh - 1)));
            float abs_c = std::abs(cos_a), abs_s = std::abs(sin_a);
            int new_hw = static_cast<int>(std::ceil(hw * abs_c + hh * abs_s)) + 1;
            int new_hh = static_cast<int>(std::ceil(hw * abs_s + hh * abs_c)) + 1;

            int dst_min_x = std::max(0,        new_cx - new_hw);
            int dst_max_x = std::min(width-1,  new_cx + new_hw);
            int dst_min_y = std::max(0,        new_cy - new_hh);
            int dst_max_y = std::min(height-1, new_cy + new_hh);

            // Use a per-snapshot claimed bitset so Pass 2b can find missed pixels.
            static std::vector<uint8_t> claimed;
            claimed.assign(static_cast<size_t>(snap.lw) * snap.lh, 0);

            for (int ty = dst_min_y; ty <= dst_max_y; ++ty) {
                for (int tx = dst_min_x; tx <= dst_max_x; ++tx) {
                    float dlx = static_cast<float>(tx - new_cx);
                    float dly = static_cast<float>(ty - new_cy);
                    int lx = static_cast<int>(std::round( cos_a * dlx + sin_a * dly));
                    int ly = static_cast<int>(std::round(-sin_a * dlx + cos_a * dly));

                    int gi_x = lx - snap.lmin_x;
                    int gi_y = ly - snap.lmin_y;
                    if (gi_x < 0 || gi_x >= snap.lw || gi_y < 0 || gi_y >= snap.lh) continue;

                    size_t gi = static_cast<size_t>(gi_y) * snap.lw + gi_x;
                    const PixelType& src_val = snap.grid[gi];
                    if (src_val.type == empty_value.type) continue;

                    size_t dst_idx = static_cast<size_t>(ty) * width + tx;
                    if (write_buffer[dst_idx].type == empty_value.type ||
                        label_grid[dst_idx] == region.current_index) {
                        write_buffer[dst_idx] = src_val;
                        label_grid[dst_idx]   = region.current_index;
                        claimed[gi] = 1;
                    }
                }
            }

            // ── Pass 2b: forward fill — place any snapshot pixel that was not
            // claimed by any destination lookup above (rare boundary holes).
            for (int ly_i = 0; ly_i < snap.lh; ++ly_i) {
                int ly = ly_i + snap.lmin_y;
                for (int lx_i = 0; lx_i < snap.lw; ++lx_i) {
                    size_t gi = static_cast<size_t>(ly_i) * snap.lw + lx_i;
                    if (claimed[gi]) continue;
                    const PixelType& src_val = snap.grid[gi];
                    if (src_val.type == empty_value.type) continue;

                    int lx = lx_i + snap.lmin_x;
                    int tx = static_cast<int>(std::round(
                        new_cx + cos_a * lx - sin_a * ly));
                    int ty_fw = static_cast<int>(std::round(
                        new_cy + sin_a * lx + cos_a * ly));

                    if (tx < 0 || tx >= width || ty_fw < 0 || ty_fw >= height) continue;
                    size_t dst_idx = static_cast<size_t>(ty_fw) * width + tx;
                    if (write_buffer[dst_idx].type == empty_value.type ||
                        label_grid[dst_idx] == region.current_index) {
                        write_buffer[dst_idx] = src_val;
                        label_grid[dst_idx]   = region.current_index;
                    }
                }
            }
        }

        // ── Pass 3: clear label_grid at old source positions that are now empty
        for (size_t idx : src_indices) {
            if (write_buffer[idx].type == empty_value.type)
                label_grid[idx] = static_cast<uint32_t>(-1);
        }

        // ── Update bounds to rotated AABB for next frame's topology check ─────
        if (!snap.grid.empty()) {
            float cos_a = std::cos(abs_angle);
            float sin_a = std::sin(abs_angle);
            float hw = static_cast<float>(
                std::max(std::abs(snap.lmin_x), std::abs(snap.lmin_x + snap.lw - 1)));
            float hh = static_cast<float>(
                std::max(std::abs(snap.lmin_y), std::abs(snap.lmin_y + snap.lh - 1)));
            float abs_c = std::abs(cos_a), abs_s = std::abs(sin_a);
            int new_hw = static_cast<int>(std::ceil(hw * abs_c + hh * abs_s));
            int new_hh = static_cast<int>(std::ceil(hw * abs_s + hh * abs_c));
            region.bounds.min_x = new_cx - new_hw;
            region.bounds.max_x = new_cx + new_hw;
            region.bounds.min_y = new_cy - new_hh;
            region.bounds.max_y = new_cy + new_hh;
        }
    }

    if (moved_any) {
        std::copy(write_buffer.begin(), write_buffer.end(), pixel_grid);
        world_revision++;
    }
}
}// namespace rigid
