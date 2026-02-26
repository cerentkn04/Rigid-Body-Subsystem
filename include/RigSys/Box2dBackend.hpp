#pragma once

#include "RigBackend.hpp"
#include "box2d/id.h"

#include <box2d/box2d.h>
#include <vector>
#include <cstdint>

// --------------------------------------------------------
// Internal Slot Structure
// --------------------------------------------------------

struct Box2DBodySlot
{
    b2BodyId* body = nullptr;
    bool alive   = false;
};

// --------------------------------------------------------
// Backend State
// --------------------------------------------------------

struct RigidBackend
{
    b2WorldId* world = nullptr;

    // Dense array indexed by RigidBodyHandle.id
    std::vector<Box2DBodySlot> bodies;

    // Free list for recycling indices
    std::vector<uint32_t> freeList;
};
