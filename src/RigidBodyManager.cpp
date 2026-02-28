#include "RigidBodyManager.hpp"
#include <algorithm>

namespace rigid {

RigidBodyManager::RigidBodyManager(b2WorldId worldId) : m_world_id(worldId) {}

RigidBodyManager::~RigidBodyManager() {
    for (auto& [id, entry] : m_body_map) {
        if (b2Body_IsValid(entry.bodyId)) {
            b2DestroyBody(entry.bodyId);
        }
    }
}
void RigidBodyManager::synchronize(
    const StabilitySystem& stability,
    const std::unordered_map<RegionID, RegionGeometry>& geometry_cache,
    const std::unordered_map<RegionID, RegionRecord>& active_regions) 
{
    // 1. Cleanup vanished regions (Keep as is)
    for (auto it = m_body_map.begin(); it != m_body_map.end(); ) {
        if (active_regions.find(it->first) == active_regions.end()) {
            if (b2Body_IsValid(it->second.bodyId)) b2DestroyBody(it->second.bodyId);
            it = m_body_map.erase(it);
        } else { ++it; }
    }

    // 2. Sync State and Geometry
    for (const auto& snapshot : stability.active_snapshots) {
        const RegionID id = snapshot.id;
        auto geo_it = geometry_cache.find(id);
        auto rec_it = active_regions.find(id);
        
        if (geo_it == geometry_cache.end() || rec_it == active_regions.end()) continue;

        const auto& geo = geo_it->second;
        const auto& record = rec_it->second;
        b2BodyType targetType = snapshot.is_stable ? b2_staticBody : b2_dynamicBody;

        auto body_it = m_body_map.find(id);
        if (body_it == m_body_map.end()) {
            create_body_for_id(id, geo, record.version, targetType);
            m_body_map[id].topology_hash = geo.topology_hash;
        } else {
            BodyEntry& entry = body_it->second;
            
            // Transform updates are CHEAP, do them every frame
            if (entry.version != record.version) {
                b2Vec2 newPos = { (float)geo.center.x, (float)geo.center.y };
                b2Body_SetTransform(entry.bodyId, newPos, b2Rot_identity);
                
                // If topology changed, just mark it dirty instead of updating now
                if (entry.topology_hash != geo.topology_hash) {
                    entry.is_dirty = true; 
                }
                entry.version = record.version;
            }

            if (b2Body_GetType(entry.bodyId) != targetType) {
                b2Body_SetType(entry.bodyId, targetType);
            }
        }
    }

// 3. Process the "Dirty" Budget
    // Adjust 'maxUpdates' based on your target CPU overhead
    int updatesProcessed = 0;
    const int maxUpdatesPerFrame = 2; 

    for (auto& [id, entry] : m_body_map) {
        if (entry.is_dirty) {
            auto geo_it = geometry_cache.find(id);
            if (geo_it != geometry_cache.end()) {
                update_fixtures(entry.bodyId, geo_it->second);
                entry.topology_hash = geo_it->second.topology_hash;
                entry.is_dirty = false; // Clean!
                
                updatesProcessed++;
                if (updatesProcessed >= maxUpdatesPerFrame) break;
            }
        }
    }
    int updatesThisFrame = 0;
    const int MAX_COMPLEX_UPDATES = 1; // Only allow ONE big body update per frame

    for (auto& [id, entry] : m_body_map) {
        if (entry.is_dirty) {
            auto geo_it = geometry_cache.find(id);
            if (geo_it != geometry_cache.end()) {
                
                // OPTIONAL: If the body is "Small", update it immediately.
                // If the body is "Huge", only do 1 per frame.
                bool is_huge = geo_it->second.convex_pieces.size() > 50;
                
                if (is_huge && updatesThisFrame >= MAX_COMPLEX_UPDATES) {
                    continue; // Skip for now, do it next frame
                }

                update_fixtures(entry.bodyId, geo_it->second);
                entry.is_dirty = false;
                entry.topology_hash = geo_it->second.topology_hash;
                
                if (is_huge) updatesThisFrame++;
            }
        }
    }



}
void RigidBodyManager::update_region_transforms(std::unordered_map<RegionID, RegionRecord>& active_regions) {
    for (auto& [id, entry] : m_body_map) {
        if (!b2Body_IsValid(entry.bodyId)) continue;

        auto it = active_regions.find(id);
        if (it != active_regions.end()) {
            b2Vec2 pos = b2Body_GetPosition(entry.bodyId);
            it->second.center_f.x = pos.x;
            it->second.center_f.y = pos.y;
        }
    }
}

void RigidBodyManager::create_body_for_id(RegionID id, const RegionGeometry& geo, uint64_t version, b2BodyType type) {
    if (std::isnan(geo.center.x) || std::isnan(geo.center.y) || 
        std::abs(geo.center.x) > 1e6f || std::abs(geo.center.y) > 1e6f) {
        return; // Skip creating this body; it's corrupt
    }

    if (geo.convex_pieces.empty()) return;

    b2BodyDef def = b2DefaultBodyDef();
    def.type = type;
    def.userData = (void*)(uintptr_t)id;
    
    // Position body at centroid
    def.position = { (float)geo.center.x, (float)geo.center.y };
    
    // Top-down camera constraints
    def.fixedRotation = true; 
    def.gravityScale = 0.0f; 
    def.linearDamping = (type == b2_dynamicBody) ? 8.0f : 0.0f;

    b2BodyId bodyId = b2CreateBody(m_world_id, &def);
    update_fixtures(bodyId, geo);

    m_body_map[id] = { bodyId, version };
}
void RigidBodyManager::update_fixtures(b2BodyId bodyId, const RegionGeometry& geo) {
    // 1. Get all shapes currently attached to this body
    int count = b2Body_GetShapeCount(bodyId);
    if (count > 0) {
        // Use a larger buffer to handle the "Big Ball" scenario
        std::vector<b2ShapeId> shapes(count);
        int actualCount = b2Body_GetShapes(bodyId, shapes.data(), count);
        
        for (int i = 0; i < actualCount; ++i) {
            // CRITICAL: Pass 'false' to avoid recalculating mass for every single piece
            b2DestroyShape(shapes[i], false);
        }
    }

    b2ShapeDef shapeDef = b2DefaultShapeDef();
    // High-speed optimization: Disable expensive features for these complex bodies
    shapeDef.density = 1.0f;

    b2Vec2 localVerts[B2_MAX_POLYGON_VERTICES];
    for (const auto& piece : geo.convex_pieces) {
        int vCount = (int)std::min(piece.points.size(), (size_t)B2_MAX_POLYGON_VERTICES);
        if (vCount < 3) continue;

        for (int i = 0; i < vCount; ++i) {
            localVerts[i] = { 
                piece.points[i].x - (float)geo.center.x, 
                piece.points[i].y - (float)geo.center.y 
            };
        }

        b2Hull hull = b2ComputeHull(localVerts, vCount);
        if (hull.count >= 3) {
            b2Polygon poly = b2MakePolygon(&hull, 0.0f);
            // This adds the shape to the body's internal list
            b2CreatePolygonShape(bodyId, &shapeDef, &poly);
        }
    }

    // 2. RECALCULATE MASS ONCE AT THE END
    // This is where the 20fps -> 200fps jump happens.
    b2Body_ApplyMassFromShapes(bodyId);
}
/*void RigidBodyManager::update_fixtures(b2BodyId bodyId, const RegionGeometry& geo) {
    int count = b2Body_GetShapeCount(bodyId);
    if (count > 0) {
        b2ShapeId shapes[64]; // Increased buffer
        int actualCount = b2Body_GetShapes(bodyId, shapes, 64);
        
        for (int i = 0; i < actualCount; ++i) {
            // PASS FALSE: Don't update mass 64 times in a row!
            b2DestroyShape(shapes[i], false);
        }
    }

    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 1.0f;
    
    // We'll use a local buffer to avoid heap churn
    b2Vec2 localVerts[B2_MAX_POLYGON_VERTICES];

    for (const auto& piece : geo.convex_pieces) {
        int vCount = (int)std::min(piece.points.size(), (size_t)B2_MAX_POLYGON_VERTICES);
        if (vCount < 3) continue;

        for (int i = 0; i < vCount; ++i) {
            localVerts[i] = { 
                piece.points[i].x - (float)geo.center.x, 
                piece.points[i].y - (float)geo.center.y 
            };
        }

        b2Hull hull = b2ComputeHull(localVerts, vCount);
        if (hull.count >= 3) {
            b2Polygon poly = b2MakePolygon(&hull, 0.0f);
            b2CreatePolygonShape(bodyId, &shapeDef, &poly);
        }
    }
    
    // FINAL STEP: Now that all shapes are in place, calculate mass ONCE.
    b2Body_ApplyMassFromShapes(bodyId);
}*/
} // namespace rigid
