# RigidPixelSystem — Integration Guide

A subsystem that gives solid pixel groups (Rock, Wood, etc.) real rigid-body physics.
Solid connected regions are automatically detected, meshed, and handed to Box2D.
When a body moves, the pixel data in your grid moves with it.

---

## How it works (one paragraph)

Every frame the system scans your pixel grid, flood-fills connected groups of solid
pixels into **regions**, and builds a convex-hull mesh for each one. A **stability
policy** decides which regions are static (anchored) and which are dynamic (falling).
Box2D drives the dynamic bodies; the system reads the resulting positions back and
physically relocates the pixels in your grid.

---

## Minimal integration

### 1 — The WorldView  (bridge between your grid and the system)

You tell the system about your world through a set of callbacks in `world::WorldView`.
This is the only place you write game-specific glue code.

```cpp
#include "RigidPixelWorldView.hpp"

world::WorldView view;
view.width  = GRID_WIDTH;
view.height = GRID_HEIGHT;

// Is this cell solid?
view.solidity_at = [](int x, int y) -> world::CellSolidity {
    // return Solid for Rock/Wood/etc, Empty otherwise
};

// Monotonically increasing counter — bump it whenever the grid changes
view.world_revision = []() -> world::WorldRevision {
    return my_revision_counter;
};

// Per-cell revision — used for fine-grained dirty detection
view.region_revision = [](int x, int y) -> world::WorldRevision {
    return my_per_cell_revision[y * W + x];
};

// Material group — determines which cells can share a region at runtime
// Return the same value for cells of the same material, 0 for non-solid
view.group_id_at = [](int x, int y) -> world::GroupID {
    // e.g. Wood → 1, Rock → 2
};

// Authored-object identity — groups cells that belong to the same
// hand-crafted object regardless of material  (see "Authored objects" below)
// Return 0 for anything placed at runtime
view.object_id_at = [](int x, int y) -> world::ObjectID {
    return my_grid[y * W + x].object_id;
};
```

### 2 — System lifecycle

```cpp
#include "RigidPixelSystem.hpp"

rigid::RigidPixelSystem rigidSystem;

// Call once at startup (optionally pass a RigidPixelConfig — see below)
rigidSystem.init(GRID_WIDTH, GRID_HEIGHT);

// --- game loop ---

// Fixed timestep (e.g. inside a 60 Hz accumulator):
rigidSystem.step(fixedDt);

// Once per frame:
rigidSystem.update(view);
rigidSystem.apply_motion(myGrid.data(), EmptyCell{}, myRevisionCounter);

// On shutdown:
rigidSystem.shutdown();
```

`apply_motion` is a template — it works with any cell/pixel type. The second argument
is the value written into cells that a moving body has vacated.

---

## Configuration  (`RigidPixelConfig`)

All tunables live in one struct. You only set what you want to override.

```cpp
#include "RigidPixelConfig.hpp"

rigid::RigidPixelConfig cfg;

// ── World physics ──────────────────────────────────────────────────
cfg.gravity_x       = 0.0f;   // default: 0
cfg.gravity_y       = 9.8f;   // default: 9.8  (positive = downward)
cfg.bodies_can_sleep = true;  // default: true  (better performance)
cfg.velocity_iters  = 4;      // default: 1  (higher = more accurate collisions)

// ── Per-body material ──────────────────────────────────────────────
cfg.density         = 1.0f;   // default: 1.0
cfg.friction        = 0.5f;   // default: 0.5
cfg.restitution     = 0.1f;   // default: 0.1  (0 = no bounce, 1 = elastic)
cfg.linear_damping  = 2.5f;   // default: 2.5  (air resistance)
cfg.angular_damping = 0.1f;   // default: 0.1

// ── Ambient force (applied to every dynamic body every tick) ───────
cfg.idle_force_x    = 0.0f;   // e.g. 0.3 for wind
cfg.idle_force_y    = 0.0f;   // e.g. -2.0 for buoyancy

// ── Stability policy ───────────────────────────────────────────────
cfg.stability_policy = nullptr;  // nullptr = use built-in BFS-from-floor policy

rigidSystem.init(GRID_WIDTH, GRID_HEIGHT, cfg);
```

### Common cfg presets

```cpp
// Zero gravity (space / underwater)
cfg.gravity_y = 0.0f;

// Floaty / slow fall
cfg.gravity_y      = 3.0f;
cfg.linear_damping = 0.5f;

// Bouncy objects
cfg.restitution    = 0.7f;
cfg.linear_damping = 0.2f;

// Wind drift
cfg.idle_force_x = 0.4f;
```

---

## Stability policy

The stability policy is the function that decides which regions are **static** (anchored,
no Box2D dynamics) and which are **dynamic** (fall, bounce, etc.).

### Default policy (`apply_basic_connectivity_policy`)

- Regions that **touch the floor** are seeded as stable.
- Stability **propagates upward** through touching regions (BFS, directional rule).
- Isolated / floating regions are dynamic and fall.

### Writing your own policy

```cpp
#include "StabilityResolver.hpp"  // for StructuralGraph, std::vector

void my_stability_policy(const rigid::StructuralGraph& graph,
                          std::vector<bool>& out_is_stable)
{
    out_is_stable.assign(graph.nodes.size(), false);

    for (uint32_t i = 0; i < graph.nodes.size(); ++i) {
        const auto& node = graph.nodes[i];

        // Available per-node data:
        //   node.id             — persistent RegionID
        //   node.bounds         — CellAABB { min_x, min_y, max_x, max_y }
        //   node.group_id       — material group (1 = Wood, 2 = Rock, ...)
        //   node.touches_floor  — true if any pixel is at y == height-1
        //   node.neighbor_indices — indices of touching regions in graph.nodes

        if (node.touches_floor)
            out_is_stable[i] = true;

        // Example: make all Wood always static
        // if (node.group_id == 1) out_is_stable[i] = true;
    }
}

// Then register it:
cfg.stability_policy = &my_stability_policy;
rigidSystem.init(w, h, cfg);
```

---

## Authored objects  (multi-material rigid bodies)

By default, the region extractor only merges pixels of the **same material** into one
region. This means a Rock blade touching a Wood handle would be detected as two separate
regions.

To make them one permanent region (e.g. a dagger, a character), assign a shared
non-zero `object_id` to all their pixels before the first frame:

```cpp
// object_id > 0  →  authored: merges with any pixel sharing the same object_id
// object_id == 0 →  runtime:  merges only with same-material neighbours

// Example: dagger = one region, blade (Rock) + handle (Wood)
for (auto& cell : blade_pixels)  cell.object_id = 1;
for (auto& cell : handle_pixels) cell.object_id = 1;

// A rock thrown at a wall at runtime keeps object_id = 0,
// so it will never permanently merge with the wall.
```

The `object_id_at` callback in `WorldView` exposes this value to the system.

---

## API summary

| Call | When |
|---|---|
| `rigidSystem.init(w, h)` | Once at startup |
| `rigidSystem.init(w, h, cfg)` | Once at startup, with custom config |
| `rigidSystem.step(dt)` | Every physics tick (fixed timestep) |
| `rigidSystem.update(view)` | Once per frame |
| `rigidSystem.apply_motion(grid, empty, revision)` | Once per frame, after `update` |
| `rigidSystem.shutdown()` | On exit |

---

## Files at a glance

| File | What to read if you want to… |
|---|---|
| `RigidPixelSystem.hpp` | See the full API |
| `RigidPixelConfig.hpp` | See / change all tunables |
| `RigidPixelWorldView.hpp` | Understand the WorldView callbacks |
| `BasicConnectivityPolicy.cpp` | Modify or study the default stability policy |
| `StabilityResolver.hpp` | Write a custom stability policy |
| `MotionSystem.hpp` | Understand how pixel motion works |
