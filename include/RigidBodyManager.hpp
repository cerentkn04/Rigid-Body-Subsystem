#pragma once
#include <vector>
#include <unordered_map>
#include <box2d/box2d.h>
#include "RigidPixelTypes.hpp"
#include "RegionMesher.hpp"
#include "RegionStability.hpp"
#include <StabilityResolver.hpp>
#include <StructuralTracker.hpp>
#include <StructuralGraph.hpp>
namespace rigid {
enum CollisionCategory {
    CAT_TERRAIN    = 0x0001,
    CAT_LARGE_CHUNK = 0x0002,
    CAT_SMALL_DEBRIS = 0x0004
};
    struct BodyEntry {
        b2BodyId bodyId;
        uint64_t version;
        uint64_t topology_hash;
        bool is_dirty;
    };

    class RigidBodyManager {
    public:
        explicit RigidBodyManager(b2WorldId worldId);
        ~RigidBodyManager();

        void synchronize(
            const StructuralGraph& graph,            // Change 1
            const std::vector<bool>& is_stable_decisions,
            const std::unordered_map<RegionID, RegionGeometry>& geometry_cache,
            const std::unordered_map<RegionID, RegionRecord>& active_regions);

        void update_region_transforms(std::unordered_map<RegionID, RegionRecord>& active_regions);

    private:
        // Updated signatures to match implementation
        void create_body_for_id(RegionID id, const RegionGeometry& geo, uint64_t version, b2BodyType type, float pixel_count);
        void update_fixtures(b2BodyId bodyId, const RegionGeometry& geo, float pixel_count);

        std::unordered_map<RegionID, BodyEntry> m_body_map;
        b2WorldId m_world_id;
    };

} // namespace rigid
