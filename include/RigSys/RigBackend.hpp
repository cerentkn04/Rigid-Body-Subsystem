#pragma once
#include "RigTypes.hpp"

// This tells the compiler "RigidBackend exists," 
// but its members (b2World, etc.) are hidden in the .cpp
struct RigidBackend; 

struct RigidBodyHandle {
    uint32_t id = 0;
    bool isValid() const { return id != 0; }
};

// Lifecycle
void rigidBackend_initialize(RigidBackend** backend); // Note: Pointer to pointer for allocation
void rigidBackend_shutdown(RigidBackend* backend);
void rigidBackend_step(RigidBackend* backend, float dt);

// Body Management
RigidBodyHandle rigidBackend_createBody(RigidBackend* backend, const RigidBodyDescription& desc);
void rigidBackend_destroyBody(RigidBackend* backend, RigidBodyHandle handle);

// Query
void rigidBackend_getTransform(const RigidBackend* backend, RigidBodyHandle handle, Vec2& pos, float& rot);
