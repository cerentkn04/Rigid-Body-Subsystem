#pragma once
#include <vector>
#include <unordered_map>
#include <box2d/box2d.h>
#include "RigidPixelTypes.hpp"
#include "RegionMesher.hpp"
#include "RegionStability.hpp"

namespace rigid {

    struct BodyEntry {
        b2BodyId bodyId;
        uint64_t version;
    };

    class RigidBodyManager {
    public:
        explicit RigidBodyManager(b2WorldId worldId);
        ~RigidBodyManager();

        void synchronize(
            const StabilitySystem& stability,
            const std::unordered_map<RegionID, RegionGeometry>& geometry_cache,
            const std::unordered_map<RegionID, RegionRecord>& active_regions);

        void update_region_transforms(std::unordered_map<RegionID, RegionRecord>& active_regions);

    private:
        // Updated signatures to match implementation
        void create_body_for_id(RegionID id, const RegionGeometry& geo, uint64_t version, b2BodyType type);
        void update_fixtures(b2BodyId bodyId, const RegionGeometry& geo);

        std::unordered_map<RegionID, BodyEntry> m_body_map;
        b2WorldId m_world_id;
    };

} // namespace rigid
