#pragma once

#include <cstdint>

namespace rigid {

//
// Rigid Body Identity
// ------------------
//
// A rigid body represents a connected region of solid cells extracted
// from the cellular automata world at a snapshot in time.
//
// Identity rules (CRITICAL — do not violate):
//
// 1. Translation:
//    If a region moves in space (even by many cells), it is still the same body.
//
// 2. Minor topology change:
//    If a region gains or loses cells but remains a single connected component,
//    it is still the same body.
//
// 3. Split:
//    If a region splits into multiple disconnected regions, ALL resulting
//    regions are considered NEW bodies.
//    The original body ceases to exist.
//
// 4. Merge:
//    If two or more previously separate regions become connected via solid
//    adjacency, the resulting region is considered a NEW body.
//    All original bodies cease to exist.
//
// 5. Touching without connectivity:
//    Mere contact (e.g. edge or corner touch without solid adjacency as defined
//    by the connectivity rule) does NOT merge bodies.
//    Connectivity definition is external but must be consistent.
//
// 6. Gameplay consequences (damage, destruction, material interaction,
//    health, etc.) are NOT handled here.
//    This system only defines existence and spatial identity.
//
// These rules ensure determinism, debuggability, and decoupling from gameplay.
//

// -----------------------------------------------------------------------------
// Opaque identifiers
// -----------------------------------------------------------------------------

  
using RigidBodyID = uint32_t;
using GenerationID = uint32_t;

// A globally unique body handle.
// Generation increments when an ID is reused to prevent stale references.
struct BodyHandle final {
    RigidBodyID id;
    GenerationID generation;
};

// -----------------------------------------------------------------------------
// Coordinate types
// -----------------------------------------------------------------------------

// Cell-space coordinates (cell indices, not world units)
using CellCoord = int32_t;

// Axis-aligned bounding box in cell space.
// Inclusive min, exclusive max: [min, max)
struct CellAABB final {
    CellCoord min_x;
    CellCoord min_y;
    CellCoord max_x;
    CellCoord max_y;
};

// -----------------------------------------------------------------------------
// Versioning
// -----------------------------------------------------------------------------

// Monotonically increasing version for body-local topology changes.
// Incremented when:
// - cell membership changes
// - shape changes
// - split / merge creates or destroys bodies
using BodyVersion = uint64_t;

} // namespace rigid

