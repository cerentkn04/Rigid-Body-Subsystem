#pragma once

#include <cstdint>
#include <cstddef>

// Forward declarations (no heavy includes here)
struct Vec2;          // Your engine's 2D vector
using RegionID = uint32_t;

// ----------------------------
// Body Type
// ----------------------------

enum class BodyType : uint8_t
{
    Static,
    Dynamic
};

// ----------------------------
// Rigid Material Policy
// ----------------------------
// This describes physical behavior.
// It does NOT contain geometry.

struct RigidMaterialPolicy
{
    BodyType type = BodyType::Dynamic;

    bool allowRotation = true;

    float density     = 1.0f;
    float friction    = 0.5f;
    float restitution = 0.0f;
};

// ----------------------------
// Triangle View (Non-owning)
// ----------------------------
// Represents a single triangle in world space.
// No ownership. Just points.

struct TriangleView
{
    const Vec2* v0;
    const Vec2* v1;
    const Vec2* v2;
};

// ----------------------------
// Geometry View (Non-owning)
// ----------------------------
// RigidSystem does NOT own geometry.
// It only reads it when building body.

struct GeometryView
{
    const Vec2* vertices;      // pointer to vertex buffer
    const uint32_t* indices;   // pointer to triangle index buffer
    size_t indexCount;         // must be multiple of 3

    // indexCount / 3 = number of triangles
};

// ----------------------------
// Rigid Body Description
// ----------------------------
// This is the full description passed to backend.
// Still contains NO backend types.

struct RigidBodyDescription
{
    RegionID region;

    RigidMaterialPolicy material;

    GeometryView geometry;
};
