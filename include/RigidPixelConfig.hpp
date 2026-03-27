#pragma once
#include <vector>
#include "StabilityResolver.hpp"   // StabilityPolicyFunc, apply_basic_connectivity_policy

namespace rigid {

/*
  ════════════════════════════════════════════════════════════════════════════
  RigidPixelConfig  —  all tunables for RigidPixelSystem in one place.

  Usage
  -----
      RigidPixelConfig cfg;          // all defaults are sensible
      cfg.gravity_y      = 0.0f;    // zero-G
      cfg.idle_force_x   = 0.5f;    // gentle rightward drift on every body
      cfg.linear_damping = 0.5f;    // floaty feel
      rigidSystem.init(width, height, cfg);

  Every field has a default. You only touch what you need.
  ════════════════════════════════════════════════════════════════════════════
*/
struct RigidPixelConfig {

    // ── World physics ─────────────────────────────────────────────────────
    float gravity_x        = 0.0f;  // world gravity, x-axis
    float gravity_y        = 9.0f;  // world gravity, y-axis  (positive = downward)
    bool  bodies_can_sleep = true;  // let Box2D sleep idle bodies (better perf)
    int   velocity_iters   = 1;     // Box2D sub-steps per tick (higher = more accurate)

    // ── Per-body material defaults ────────────────────────────────────────
    float density          = 1.0f;
    float friction         = 0.5f;
    float restitution      = 0.1f;  // bounciness  (0 = no bounce, 1 = elastic)
    float linear_damping   = 2.5f;  // air resistance on linear velocity
    float angular_damping  = 0.1f;  // air resistance on rotation

    // ── Ambient / idle force ──────────────────────────────────────────────
    // Applied to every dynamic body every physics step.
    // Examples:
    //   idle_force_y = -2.0f   →  slight upward buoyancy
    //   idle_force_x =  0.3f   →  constant wind drift
    //   (leave at 0 for normal behaviour)
    float idle_force_x     = 0.0f;
    float idle_force_y     = 0.0f;

    // ── Stability policy ──────────────────────────────────────────────────
    // Controls which regions get a static physics body vs a dynamic (falling) one.
    // nullptr  →  use apply_basic_connectivity_policy  (BFS from floor, directional rule)
    //
    // To write your own, implement:
    //   void my_policy(const StructuralGraph& graph, std::vector<bool>& out_is_stable);
    // then set:
    //   cfg.stability_policy = &my_policy;
    StabilityPolicyFunc stability_policy = nullptr;
};

} // namespace rigid
