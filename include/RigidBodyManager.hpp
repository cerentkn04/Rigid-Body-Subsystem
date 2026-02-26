#pragma once
#include <vector>
#include <unordered_map>
#include <box2d/box2d.h> // v3.0+
#include "RigidPixelTypes.hpp"
#include "RegionMesher.hpp"
#include "RegionStability.hpp"

namespace rigid {

class RigidBodyManager {
public:
    explicit RigidBodyManager(b2WorldId worldId);
    ~RigidBodyManager();

    void synchronize(
        const StabilitySystem& stability,
        const std::unordered_map<RegionID, RegionGeometry>& geometry_cache,
        const std::unordered_map<RegionID, RegionRecord>& active_regions);

private:
    void ensure_capacity(size_t size);
    void create_body(uint32_t index, RegionID id, const RegionGeometry& geo, bool is_floating);
    void update_fixtures(b2BodyId bodyId, const RegionGeometry& geo);
    void destroy_body(uint32_t index);

    // Parallel component storage (SoA)
    // Box2D v3 handles are small enough to copy/store directly
    std::vector<b2BodyId> m_bodies; 
    std::vector<BodyVersion> m_body_versions;
    std::vector<RegionID> m_active_ids;

    b2WorldId m_world_id;
};

} // namespace rigid
