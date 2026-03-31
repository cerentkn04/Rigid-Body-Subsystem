# RigidPixelSystem — Developer Guide

---

## What this system does

Your game world is a grid of pixels (cells). Some of those pixels are solid — rock,
wood, metal, whatever material you define. When a group of solid pixels is connected,
this system treats that group as a **rigid body**: a single object that can fall, slide,
collide, rotate, and move as one piece, driven by real physics.

Concretely, this is what happens every frame:

1. The system scans your grid and finds every connected group of solid pixels.
2. Each group gets a physics body in Box2D. The body's shape is computed automatically
   from the pixel outline.
3. The **stability policy** decides which bodies are anchored (static) and which are
   free to move (dynamic).
4. Box2D simulates the dynamic bodies — they fall under gravity, collide with each
   other, and rotate.
5. The system reads the new positions and angles from Box2D and moves the pixels
   in your grid accordingly.

You do not need to know anything about Box2D, meshing, or collision detection.
You only need to tell the system which pixels are solid, and call three functions
each frame.

---

## Key concepts

### Regions

A **region** is a connected group of solid pixels that the system treats as one object.
The system detects regions automatically every frame. If pixels are added or removed,
regions split or merge, and the system creates or destroys physics bodies accordingly.

Regions are identified by a `RegionID` — a persistent integer that stays the same as
long as the region exists unchanged. If a region splits into two pieces, both pieces
get new IDs. If two regions merge, the merged region gets a new ID.

### Solid pixels and materials

Not all solid pixels can share a region. Two adjacent solid pixels are merged into
the same region only if they have the same **group ID** — a number you assign per
material. Rock next to Rock → same region. Rock next to Wood → two separate regions,
unless you use an authored object (see below).

### Static vs dynamic

A **static** body is pinned in place. It has a Box2D body for collision purposes but
it does not move. Think of a stone floor, a wall, or a platform.

A **dynamic** body falls under gravity, bounces off things, and rotates when hit.
Think of a rock falling through the air or a plank knocked off a ledge.

The **stability policy** is the rule that decides which regions are static and which
are dynamic. The default rule treats anything touching the bottom of the grid (the
floor) as static, and propagates that stability upward through connected touching
regions — just like Noita. Regions that are floating or isolated become dynamic and fall.

### Rotation and spinning

Rotation is fully automatic. When a dynamic body collides with something at an angle,
Box2D gives it angular velocity. The system then rotates the body's pixels around the
body's center each frame — the pixel shape spins exactly as Box2D says it should.

The snapshot of each body's pixels is captured in body-local space the first time that
body is seen. This snapshot is then rotated and rasterized into your grid every frame.
Sand and water pixels that a rotating body sweeps through are pushed aside into
adjacent empty cells.

You do not call any special function for rotation. It is part of `apply_motion` and
happens automatically for every dynamic body.

To control how rotation behaves:
- `cfg.angular_damping` — how fast spinning slows down (higher = stops sooner)
- `cfg.restitution` — how much a collision bounces (higher = more spin-up on impact)
- `cfg.linear_damping` — how fast linear movement slows down (does not affect rotation)

---

## Integration — step by step

### Step 1 — Define your cell type

Your cell type can have any fields you want. The system only asks about three things
per cell: is it solid, what material group is it, and what authored object does it
belong to (if any). You answer those questions through lambdas when you build the
WorldView.

```cpp
struct Cell {
    CellType type          = CellType::Empty;
    uint32_t object_id     = 0;   // see "Authored objects" below; 0 = none
    uint8_t  color_variant = 0;   // your own fields — the system ignores these
    bool     updated       = false;
};
```

### Step 2 — Create a `RigidPixelGrid`

`RigidPixelGrid<CellT>` owns your flat pixel array and handles revision tracking
automatically. Revision tracking is how the system knows when something changed so
it does not re-scan an unchanged grid every frame.

```cpp
#include "RigidPixelGrid.hpp"

constexpr int W = 200, H = 150;
rigid::RigidPixelGrid<Cell> grid(W, H);

// Access and modify cells:
grid.at(x, y).type = CellType::Rock;

// IMPORTANT: after changing any cell that is solid (or was solid),
// call mark_solid_changed so the system picks up the change.
grid.mark_solid_changed(x, y);
```

If you forget to call `mark_solid_changed`, the system will not re-detect the region
until the next time the revision is bumped. This is intentional — it is how the system
avoids expensive re-scans every frame.

### Step 3 — Build a WorldView

The `WorldView` is a small bridge object that tells the system how to read your grid.
`RigidPixelGrid::make_view` builds it for you from three short lambdas:

```cpp
world::WorldView view = grid.make_view(
    // solid_fn: return true for cells that should form rigid bodies
    [](const Cell& c) {
        return c.type == CellType::Rock || c.type == CellType::Wood;
    },

    // group_fn: return a non-zero integer identifying the material group.
    // Cells with different group IDs will NOT share a region.
    // Return 0 for non-solid cells (the system ignores them anyway).
    [](const Cell& c) -> uint32_t {
        switch (c.type) {
            case CellType::Wood: return 1;
            case CellType::Rock: return 2;
            default:             return 0;
        }
    },

    // object_fn: return the authored object ID (see "Authored objects").
    // If you are not using authored objects, omit this argument entirely.
    [](const Cell& c) -> uint32_t { return c.object_id; }
);
```

The view holds a pointer to the grid. Keep the grid alive for as long as the view
is in use.

### Step 4 — Initialize and run the system

```cpp
#include "RigidPixelSystem.hpp"

rigid::RigidPixelSystem rigidSystem;

// Call once at startup.
// Optionally pass a RigidPixelConfig to override defaults — see Configuration.
rigidSystem.init(W, H);

// ── Game loop ──────────────────────────────────────────────────────────────

// Physics sub-step — call this at a fixed rate (e.g. 60 Hz).
// If you run multiple sub-steps per frame, call this multiple times.
rigidSystem.step(fixedDt);

// Sync regions with the grid — call once per frame (not per sub-step).
rigidSystem.update(view);

// Move pixels in your grid to match the new physics positions.
// Call once per frame, after update().
// Second argument: the value used to clear cells a body has moved away from.
rigidSystem.apply_motion(grid.data(), Cell{}, grid.world_revision);

// ── On shutdown ────────────────────────────────────────────────────────────
rigidSystem.shutdown();
```

That is the full integration. Everything else in this document is optional.

---

## Configuration

All tunables are in `RigidPixelConfig`. You only set the fields you want to change —
the rest use their defaults.

```cpp
#include "RigidPixelConfig.hpp"

rigid::RigidPixelConfig cfg;
rigidSystem.init(W, H, cfg);
```

### World physics

| Field | Default | Meaning |
|---|---|---|
| `gravity_x` | `0.0` | Horizontal gravity (positive = rightward) |
| `gravity_y` | `0.0` | Vertical gravity (positive = downward). Set to `9.8` for normal falling. |
| `bodies_can_sleep` | `true` | Let Box2D stop simulating bodies that have come to rest. Improves performance. |
| `velocity_iters` | `1` | Box2D sub-steps per tick. Higher values make collisions more accurate but cost more CPU. |

### Per-body material

These apply to every dynamic body. There is currently no per-material override —
all dynamic bodies share the same material properties.

| Field | Default | Meaning |
|---|---|---|
| `density` | `1.0` | Mass per unit area. Higher = heavier bodies. |
| `friction` | `0.5` | Surface friction. `0` = frictionless, `1` = very grippy. |
| `restitution` | `0.5` | Bounciness. `0` = no bounce (dead stop), `1` = perfectly elastic. Also affects how much spin a collision produces. |
| `linear_damping` | `2.5` | How fast linear velocity bleeds off. Higher = more air resistance on movement. |
| `angular_damping` | `0.5` | How fast spinning slows down. Higher = rotation dies out sooner. |

### Ambient force

An optional constant force applied to every dynamic body every physics tick.
Useful for environmental effects.

| Field | Default | Example use |
|---|---|---|
| `idle_force_x` | `0.0` | `0.4` for a rightward wind |
| `idle_force_y` | `0.0` | `-2.0` for upward buoyancy |

### Common presets

```cpp
// Standard falling gravity
cfg.gravity_y = 9.8f;

// Zero gravity (space / underwater)
cfg.gravity_y = 0.0f;

// Slow floaty fall
cfg.gravity_y      = 3.0f;
cfg.linear_damping = 0.5f;

// Bouncy, spinning objects on collision
cfg.restitution    = 0.8f;
cfg.linear_damping = 0.1f;
cfg.angular_damping = 0.1f;

// Objects spin freely but move through air easily
cfg.linear_damping  = 0.05f;
cfg.angular_damping = 0.02f;

// Heavy, sticky objects that barely spin
cfg.density         = 5.0f;
cfg.restitution     = 0.1f;
cfg.angular_damping = 5.0f;

// Wind drift
cfg.idle_force_x = 0.4f;
```

---

## Stability policy

The stability policy is a function you can provide (or leave as default) that decides
which regions are static and which are dynamic.

### Default policy

The default policy works like this:

- Any region whose pixels touch the bottom row of the grid is **stable** (static).
- Any region that physically touches a stable region is also **stable** (BFS propagation).
- Any region that is floating or isolated is **dynamic** — it will fall.

This produces Noita-style structural physics: a tower of rocks is stable if it stands
on the floor. Knock the bottom out and the tower falls.

### Built-in alternatives

```cpp
#include "StabilityResolver.hpp"

// Everything is always static — useful while testing or for fully hand-placed scenes.
cfg.stability_policy = &rigid::apply_static_everything_policy;
```

### Writing your own policy

A policy is a plain function with this signature:

```cpp
void my_policy(const rigid::StructuralGraph& graph, std::vector<bool>& out_is_stable);
```

`graph.nodes` is a list of every currently active region. You write `true` or `false`
into `out_is_stable[i]` for each region to mark it static or dynamic. The vector is
pre-sized to `graph.nodes.size()` but its initial values are not guaranteed —
always write to every index.

```cpp
#include "StabilityResolver.hpp"
#include "StructuralGraph.hpp"

void my_policy(const rigid::StructuralGraph& graph, std::vector<bool>& out_is_stable)
{
    out_is_stable.assign(graph.nodes.size(), false); // default: everything dynamic

    for (uint32_t i = 0; i < graph.nodes.size(); ++i) {
        const auto& node = graph.nodes[i];

        // node.id               — persistent RegionID for this region
        // node.pixel_count      — number of pixels in this region
        // node.bounds           — CellAABB { min_x, min_y, max_x, max_y } in cell coords
        // node.group_id         — material group (from your group_fn)
        // node.touches_floor    — true if any pixel is in the last row of the grid
        // node.neighbor_indices — indices into graph.nodes for physically touching regions

        // Example: floor-touching regions are stable
        if (node.touches_floor)
            out_is_stable[i] = true;

        // Example: Wood (group_id == 1) is always static regardless of position
        // if (node.group_id == 1) out_is_stable[i] = true;

        // Example: small regions (fewer than 10 pixels) are always dynamic
        // if (node.pixel_count < 10) out_is_stable[i] = false;
    }
}

// Register the policy before init:
cfg.stability_policy = &my_policy;
rigidSystem.init(W, H, cfg);
```

---

## Authored objects (multi-material rigid bodies)

By default the system groups pixels by material. Rock next to Wood becomes two regions —
one Rock region, one Wood region — because they have different group IDs. This is
correct for randomly placed runtime pixels.

But sometimes you want a hand-crafted object — a dagger with a Rock blade and a Wood
handle — to behave as one single rigid body. You do this by giving every pixel of that
object the same non-zero `object_id`.

```cpp
// object_id == 0  →  runtime pixel: groups only with same-material neighbours
// object_id  > 0  →  authored pixel: groups with all pixels sharing the same object_id,
//                    regardless of material

// Example: a dagger (object_id = 1) made of Rock and Wood
for (int x = bx; x < bx + blade_w; ++x)
    for (int y = by; y < by + blade_h; ++y) {
        grid.at(x, y).type      = CellType::Rock;
        grid.at(x, y).object_id = 1;
        grid.mark_solid_changed(x, y);
    }

for (int x = hx; x < hx + handle_w; ++x)
    for (int y = hy; y < hy + handle_h; ++y) {
        grid.at(x, y).type      = CellType::Wood;
        grid.at(x, y).object_id = 1;  // same id → same body
        grid.mark_solid_changed(x, y);
    }
```

A runtime Rock pixel thrown against this dagger will not become part of the dagger
body because its `object_id` is `0`. Authored merging only happens between pixels
that explicitly share the same non-zero `object_id`.

---

## Manual WorldView (advanced)

If you are not using `RigidPixelGrid` — for example, if you have your own grid class
or need custom revision logic — you can build a `WorldView` directly:

```cpp
#include "RigidPixelWorldView.hpp"

world::WorldView view;
view.width  = W;
view.height = H;

// Returns Solid or Empty for each cell.
view.solidity_at = [](int x, int y) -> world::CellSolidity {
    return is_solid(x, y) ? world::CellSolidity::Solid : world::CellSolidity::Empty;
};

// A counter that increases every time any solid cell changes.
// The system uses this to detect when a re-scan is needed.
view.world_revision = []() -> world::WorldRevision {
    return my_global_revision;
};

// The revision value of each individual cell.
// The system uses this for fine-grained dirty detection.
view.region_revision = [](int x, int y) -> world::WorldRevision {
    return my_per_cell_revision[y * W + x];
};

// Material group ID — same rules as group_fn above.
view.group_id_at = [](int x, int y) -> world::GroupID {
    return material_group(x, y);
};

// Authored object ID — return 0 if you are not using authored objects.
view.object_id_at = [](int x, int y) -> world::ObjectID {
    return my_grid[y * W + x].object_id;
};
```

You are responsible for bumping `my_global_revision` and `my_per_cell_revision`
whenever solid cells change. If you forget, the system will not re-detect regions
until the next revision bump.

---

## API summary

### `RigidPixelSystem`

| Method | When to call |
|---|---|
| `init(w, h)` | Once at startup |
| `init(w, h, cfg)` | Once at startup, with custom config |
| `step(dt)` | Every physics tick at a fixed timestep (e.g. 60 Hz) |
| `update(view)` | Once per frame (not per physics sub-step) |
| `apply_motion(grid, empty, revision)` | Once per frame, after `update` |
| `shutdown()` | On exit |

### `RigidPixelGrid<CellT>`

| Method | What it does |
|---|---|
| `at(x, y)` | Read or write a cell |
| `in_bounds(x, y)` | Bounds check |
| `data()` | Raw pointer to the cell array — pass to `apply_motion` |
| `mark_solid_changed(x, y)` | Tell the system a solid cell was added, removed, or changed |
| `clear(empty)` | Fill the entire grid with `empty` and bump the revision |
| `make_view(solid_fn, group_fn)` | Build a WorldView (no authored objects) |
| `make_view(solid_fn, group_fn, object_fn)` | Build a WorldView with authored objects |

---

## Files at a glance

| File | What to read if you want to… |
|---|---|
| `RigidPixelSystem.hpp` | See the full system API |
| `RigidPixelGrid.hpp` | Understand the grid helper and revision tracking |
| `RigidPixelConfig.hpp` | See every configurable field with its default |
| `RigidPixelWorldView.hpp` | Understand the WorldView struct (manual setup) |
| `StabilityResolver.hpp` | Write or register a custom stability policy |
| `StructuralGraph.hpp` | See what per-region data your stability policy receives |
| `BasicConnectivityPolicy.cpp` | Read the default floor-BFS stability policy |
| `MotionSystem.hpp` | Understand how pixel translation and rotation are applied |
