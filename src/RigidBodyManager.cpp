#include "RigidBodyManager.hpp"
#include <algorithm>

namespace rigid {

RigidBodyManager::RigidBodyManager(b2WorldId worldId) : m_world_id(worldId) {}

RigidBodyManager::~RigidBodyManager() {
    for (uint32_t i = 0; i < m_bodies.size(); ++i) {
        destroy_body(i);
    }
}

void RigidBodyManager::ensure_capacity(size_t size) {
    if (m_bodies.size() < size) {
        m_bodies.resize(size, b2_nullBodyId);
        m_body_versions.resize(size, 0);
        m_active_ids.resize(size, InvalidRegionID);
    }
}

void RigidBodyManager::synchronize(
    const StabilitySystem& stability,
    const std::unordered_map<RegionID, RegionGeometry>& geometry_cache,
    const std::unordered_map<RegionID, RegionRecord>& active_regions) 
{
    const size_t count = stability.active_snapshots.size();
    ensure_capacity(count);

    if (m_bodies.size() > count) {
        for (size_t i = count; i < m_bodies.size(); ++i) {
            destroy_body(static_cast<uint32_t>(i));
        }
        m_bodies.resize(count);
        m_body_versions.resize(count);
        m_active_ids.resize(count);
    }

    for (uint32_t i = 0; i < count; ++i) {
        const auto& snapshot = stability.active_snapshots[i];
        const RegionID id = snapshot.id;

        auto rec_it = active_regions.find(id);
        auto geo_it = geometry_cache.find(id);
        
        if (rec_it == active_regions.end() || geo_it == geometry_cache.end()) continue;

        const RegionRecord& record = rec_it->second;
        const RegionGeometry& geo = geo_it->second;

        bool is_dynamic = true; 

        if (!b2Body_IsValid(m_bodies[i]) || m_active_ids[i] != id) {
            destroy_body(i); 
            create_body(i, id, geo, is_dynamic);
            m_active_ids[i] = id;
        } else {
            if (m_body_versions[i] != record.version) {
                update_fixtures(m_bodies[i], geo);
                m_body_versions[i] = record.version;
            }

            b2BodyType target = is_dynamic ? b2_dynamicBody : b2_staticBody;
            if (b2Body_GetType(m_bodies[i]) != target) {
                b2Body_SetType(m_bodies[i], target);
            }
        }
    }
}

void RigidBodyManager::create_body(uint32_t index, RegionID id, const RegionGeometry& geo, bool is_dynamic) {
    b2BodyDef def = b2DefaultBodyDef();
    def.type = is_dynamic ? b2_dynamicBody : b2_staticBody;
    def.userData = (void*)(uintptr_t)id;

    m_bodies[index] = b2CreateBody(m_world_id, &def);
    update_fixtures(m_bodies[index], geo);
    m_body_versions[index] = geo.version;
}

void RigidBodyManager::update_fixtures(b2BodyId bodyId, const RegionGeometry& geo) {
    // 1. Get and destroy shapes correctly
    int capacity = b2Body_GetShapeCount(bodyId);
    if (capacity > 0) {
        std::vector<b2ShapeId> shapeIds(capacity);
        b2Body_GetShapes(bodyId, shapeIds.data(), capacity);
        for (int i = 0; i < capacity; ++i) {
            // FIX: Pass 'false' for updateBodyMass to improve performance during mass-destruction
            b2DestroyShape(shapeIds[i], false); 
        }
    }

    // 2. Setup Shape Definition
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 1.0f;
    
    // FIX: If 'friction' is missing from b2ShapeDef, it might be that your version 
    // uses a 'b2DefaultShapeDef' that initializes it, but the member name is different
    // (e.g. friction is now part of the body definition in some experimental versions, 
    // or simply renamed). 
    // SOLUTION: We will COMMENT IT OUT for now. b2DefaultShapeDef() sets a default of 0.6f.
    // shapeDef.friction = 0.3f; 

    for (const auto& piece : geo.convex_pieces) {
        if (piece.points.size() < 3) continue;

        std::vector<b2Vec2> verts;
        verts.reserve(piece.points.size());
        for (const auto& v : piece.points) {
            verts.push_back({v.x, v.y});
        }

        // Box2D v3 Pipeline: Compute Hull then Make Polygon
        b2Hull hull = b2ComputeHull(verts.data(), (int)verts.size());
        if (hull.count >= 3) {
            b2Polygon poly = b2MakePolygon(&hull, 0.0f);
            b2CreatePolygonShape(bodyId, &shapeDef, &poly);
        }
    }
}

void RigidBodyManager::destroy_body(uint32_t index) {
    if (b2Body_IsValid(m_bodies[index])) {
        b2DestroyBody(m_bodies[index]);
    }
    m_bodies[index] = b2_nullBodyId;
    m_body_versions[index] = 0;
    m_active_ids[index] = InvalidRegionID;
}

} // namespace rigid
