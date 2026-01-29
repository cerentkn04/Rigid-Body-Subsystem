#pragma once

#include <cstdint>

#include "RigidPixelTypes.hpp"
#include "RigidPixelWorldView.hpp" 

namespace rigid {
  
    
// -----------------------------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------------------------

struct RigidPixelConfig;
struct RigidPixelStats;

// -----------------------------------------------------------------------------
// Event types (payloads intentionally empty for now)
// -----------------------------------------------------------------------------


enum class RigidEventType : uint8_t {
    BodyCreated,
    BodyDestroyed,
    BodySplit
};

struct RigidEvent {
    RigidEventType type;
    BodyHandle body;
};

// -----------------------------------------------------------------------------
// Rigid Pixel System
// -----------------------------------------------------------------------------

class RigidPixelSystem final {
public:
    // ---------------------------------------------------------------------
    // Construction / Destruction
    // ---------------------------------------------------------------------

    explicit RigidPixelSystem(const RigidPixelConfig& config);
    ~RigidPixelSystem();

    RigidPixelSystem(const RigidPixelSystem&) = delete;
    RigidPixelSystem& operator=(const RigidPixelSystem&) = delete;

    RigidPixelSystem(RigidPixelSystem&&) = delete;
    RigidPixelSystem& operator=(RigidPixelSystem&&) = delete;

    // ---------------------------------------------------------------------
    // Update
    // ---------------------------------------------------------------------

    /*
        Update the rigid pixel system.

        Invariants (ENFORCED BY ASSERTIONS):
        - world_view functions must be non-null
        - world_view dimensions must be > 0
        - update() must be called only during a stable world snapshot
        - world_view must not be mutated during this call

        This function performs NO CA mutation.
    */
    void update(const world::WorldView& world_view, float dt);

    // ---------------------------------------------------------------------
    // Event access
    // ---------------------------------------------------------------------

    /*
        Returns a pointer to an internal event buffer.
        The buffer may be empty.
        The contents are valid until the next update() call.
    */
    const RigidEvent* events() const;
    std::uint32_t event_count() const;

    // ---------------------------------------------------------------------
    // Debug / Stats (optional, may be empty)
    // ---------------------------------------------------------------------

    const RigidPixelStats* stats() const;

private:
    // Opaque internal state
    struct Impl;
    Impl* m_impl;
};

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

struct RigidPixelConfig {
    // Reserved for future configuration.
    // Must be trivially constructible.
    std::uint32_t reserved = 0;
};

// -----------------------------------------------------------------------------
// Debug statistics (optional, can remain empty)
// -----------------------------------------------------------------------------

struct RigidPixelStats {
    std::uint32_t body_count = 0;
};

} // namespace rigid

