#pragma once
#include <vector>
#include <algorithm>
#include <RigidPixelWorldView.hpp>

namespace rigid {

/*
  RigidPixelGrid<CellT>
  ─────────────────────
  Owns your pixel grid and all revision tracking.
  Call make_view() to get a WorldView ready for RigidPixelSystem — no globals,
  no manual counters, no boilerplate.

  Usage
  -----
      rigid::RigidPixelGrid<Cell> grid(W, H);

      world::WorldView view = grid.make_view(
          [](const Cell& c){ return c.type == CellType::Rock || c.type == CellType::Wood; },
          [](const Cell& c) -> uint32_t { return c.type == CellType::Wood ? 1 : 2; },
          [](const Cell& c) -> uint32_t { return c.object_id; }   // optional
      );

      // Modifying cells:
      grid.at(x, y).type = CellType::Rock;
      grid.mark_solid_changed(x, y);   // bumps revision so the system picks up the change

      // Clearing the whole grid:
      grid.clear();
*/
template<typename CellT>
struct RigidPixelGrid {

    int width  = 0;
    int height = 0;
    std::vector<CellT>      cells;
    std::vector<uint64_t>   cell_revisions;
    uint64_t                world_revision = 1;

    RigidPixelGrid() = default;
    RigidPixelGrid(int w, int h, CellT empty = {})
        : width(w), height(h),
          cells(w * h, empty),
          cell_revisions(w * h, 0) {}

    bool   in_bounds(int x, int y) const { return x >= 0 && x < width && y >= 0 && y < height; }
    CellT& at(int x, int y)              { return cells[y * width + x]; }
    const CellT& at(int x, int y) const  { return cells[y * width + x]; }
    CellT*       data()                  { return cells.data(); }
    const CellT* data() const            { return cells.data(); }

    // Call after any change to a cell that is (or was) solid.
    // Bumps both the per-cell and world revision so the system knows to re-scan.
    void mark_solid_changed(int x, int y) {
        ++world_revision;
        cell_revisions[y * width + x] = world_revision;
    }

    // Clears every cell to `empty` and bumps the world revision.
    void clear(CellT empty = {}) {
        std::fill(cells.begin(), cells.end(), empty);
        ++world_revision;
        std::fill(cell_revisions.begin(), cell_revisions.end(), world_revision);
    }

    // ── WorldView factory ────────────────────────────────────────────────────
    //
    // solid_fn(cell)     → bool       is this cell solid?
    // group_fn(cell)     → uint32_t   runtime material group
    //                                 (same value = can share a region at runtime)
    // object_fn(cell)    → uint32_t   authored object id
    //                                 (0 = runtime, >0 = multi-material authored body)
    //
    // The returned WorldView captures a pointer to this grid.
    // Keep this grid alive and in place for as long as the view is used.

    template<typename SolidFn, typename GroupFn, typename ObjectFn>
    world::WorldView make_view(SolidFn solid_fn, GroupFn group_fn, ObjectFn object_fn) {
        world::WorldView v;
        v.width  = width;
        v.height = height;
        v.solidity_at = [this, solid_fn](int x, int y) -> world::CellSolidity {
            if (!in_bounds(x, y)) return world::CellSolidity::Empty;
            return solid_fn(at(x, y)) ? world::CellSolidity::Solid : world::CellSolidity::Empty;
        };
        v.world_revision = [this]() -> world::WorldRevision {
            return world_revision;
        };
        v.region_revision = [this](int x, int y) -> world::WorldRevision {
            return in_bounds(x, y) ? cell_revisions[y * width + x] : 0;
        };
        v.group_id_at = [this, group_fn](int x, int y) -> world::GroupID {
            if (!in_bounds(x, y)) return 0;
            return static_cast<world::GroupID>(group_fn(at(x, y)));
        };
        v.object_id_at = [this, object_fn](int x, int y) -> world::ObjectID {
            if (!in_bounds(x, y)) return 0;
            return static_cast<world::ObjectID>(object_fn(at(x, y)));
        };
        return v;
    }

    // Two-predicate overload — all cells are runtime solids (object_id always 0)
    template<typename SolidFn, typename GroupFn>
    world::WorldView make_view(SolidFn solid_fn, GroupFn group_fn) {
        return make_view(solid_fn, group_fn, [](const CellT&) -> uint32_t { return 0; });
    }
};

} // namespace rigid
