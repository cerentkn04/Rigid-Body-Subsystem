#include "RigidBodyManager.hpp"
#include <cstdio>
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
    const StructuralGraph& graph,            // Change 1
    const std::vector<bool>& is_stable_decisions,
    const std::unordered_map<RegionID, RegionGeometry>& geometry_cache,
    const std::unordered_map<RegionID, RegionRecord>& active_regions) 
{

// 1. Cleanup vanished regions (Keep your existing cleanup code)
    for (auto it = m_body_map.begin(); it != m_body_map.end(); ) {
        if (active_regions.find(it->first) == active_regions.end()) {
            if (b2Body_IsValid(it->second.bodyId)) b2DestroyBody(it->second.bodyId);
            it = m_body_map.erase(it);
        } else { ++it; }
    }

    // 2. Sync State using the Graph and the Decisions vector
    for (uint32_t i = 0; i < (uint32_t)graph.nodes.size(); ++i) {
        const auto& node = graph.nodes[i];
        const RegionID id = node.id;
        
        auto geo_it = geometry_cache.find(id);
        auto rec_it = active_regions.find(id);
        if (geo_it == geometry_cache.end() || rec_it == active_regions.end()) continue;

        const auto& geo = geo_it->second;
        const auto& record = rec_it->second;
        
        // Use the decision from the Policy!
        b2BodyType targetType = is_stable_decisions[i] ? b2_staticBody : b2_dynamicBody;

        auto body_it = m_body_map.find(id);
        if (body_it == m_body_map.end()) {
            create_body_for_id(id, geo, record.version, targetType, (float)record.pixel_count);
            m_body_map[id].topology_hash = geo.topology_hash;
        } else {
            BodyEntry& entry = body_it->second;
            b2BodyType currentType = b2Body_GetType(entry.bodyId);

            // ... (Keep your existing fixture/version/transform sync logic) ...
            
            if (currentType != targetType) {
                if (targetType == b2_dynamicBody) {
                    b2Vec2 newPos = { (float)geo.center.x, (float)geo.center.y };
                    b2Body_SetTransform(entry.bodyId, newPos, b2Rot_identity);
                }
                b2Body_SetType(entry.bodyId, targetType);
            }
        }
    }

    // 3. Budgeted Dirty Updates (Logic remains the same, just clean up)
    int normalUpdates = 0;
    int complexUpdates = 0;
    for (auto& [id, entry] : m_body_map) {
        if (!entry.is_dirty) continue;

        auto geo_it = geometry_cache.find(id);
        auto rec_it = active_regions.find(id);
        if (geo_it == geometry_cache.end() || rec_it == active_regions.end()) continue;

        const auto& geo = geo_it->second;
        bool is_complex = geo.convex_pieces.size() > 50;

        if (is_complex && complexUpdates >= 1) continue;
        if (!is_complex && normalUpdates >= 2) continue;

        update_fixtures(entry.bodyId, geo, (float)rec_it->second.pixel_count);
        entry.topology_hash = geo.topology_hash;
        entry.is_dirty = false;

        if (is_complex) complexUpdates++; else normalUpdates++;
        if (normalUpdates >= 2 && complexUpdates >= 1) break;
    }
}
/*void RigidBodyManager::synchronize(
    const StabilitySystem& stability,
    const std::unordered_map<RegionID, RegionGeometry>& geometry_cache,
    const std::unordered_map<RegionID, RegionRecord>& active_regions) 
{
    // 1. Cleanup vanished regions
    for (auto it = m_body_map.begin(); it != m_body_map.end(); ) {
        if (active_regions.find(it->first) == active_regions.end()) {
            if (b2Body_IsValid(it->second.bodyId)) b2DestroyBody(it->second.bodyId);
            it = m_body_map.erase(it);
        } else { ++it; }
    }

    // 2. Sync State and Logic
    for (const auto& snapshot : stability.active_snapshots) {
        const RegionID id = snapshot.id;
        auto geo_it = geometry_cache.find(id);
        auto rec_it = active_regions.find(id);
        
        if (geo_it == geometry_cache.end() || rec_it == active_regions.end()) continue;

        const auto& geo = geo_it->second;
        const auto& record = rec_it->second;
        float size = (float)record.pixel_count;
        b2BodyType targetType = snapshot.is_stable ? b2_staticBody : b2_dynamicBody;

        auto body_it = m_body_map.find(id);
        if (body_it == m_body_map.end()) {
            create_body_for_id(id, geo, record.version, targetType,size);
            m_body_map[id].topology_hash = geo.topology_hash;
        } else {
            BodyEntry& entry = body_it->second;
            b2BodyType currentType = b2Body_GetType(entry.bodyId);
            if (entry.is_dirty) {
              update_fixtures(entry.bodyId, geo, size);
              entry.is_dirty = false;
            }
            if (entry.version != record.version) {
                if (currentType == b2_staticBody) {
                    b2Vec2 newPos = { (float)geo.center.x, (float)geo.center.y };
                    b2Body_SetTransform(entry.bodyId, newPos, b2Rot_identity);
                }
                
                if (entry.topology_hash != geo.topology_hash) {
                    entry.is_dirty = true; 
                }
                entry.version = record.version;
            }

            // If switching from Static -> Dynamic, we MUST align once
            if (currentType != targetType) {

              const char* typeStr = (targetType == b2_dynamicBody) ? "DYNAMIC" : "STATIC";
              printf("[Physics] Region %u changed state to %s\n", id, typeStr);
                if (targetType == b2_dynamicBody) {
                    b2Vec2 newPos = { (float)geo.center.x, (float)geo.center.y };
                    printf("  -> Transitioning to Dynamic at: (%.2f, %.2f)\n", newPos.x, newPos.y);
                    printf("  -> Transitioning to Dynamic at: (%.2f, %.2f)\n", newPos.x, newPos.y);
                    b2Body_SetTransform(entry.bodyId, newPos, b2Rot_identity);
                }
                b2Body_SetType(entry.bodyId, targetType);
            }
        }
    }

    int normalUpdates = 0;
    int complexUpdates = 0;
    const int MAX_NORMAL = 2;
    const int MAX_COMPLEX = 1;
// 3. Budgeted Dirty Updates
    for (auto& [id, entry] : m_body_map) {
        if (!entry.is_dirty) continue;

        auto geo_it = geometry_cache.find(id);
        auto rec_it = active_regions.find(id); // Added this to get size
        if (geo_it == geometry_cache.end() || rec_it == active_regions.end()) continue;

        const auto& geo = geo_it->second;
        const auto& record = rec_it->second;
        float size = (float)record.pixel_count;
        
        bool is_complex = geo.convex_pieces.size() > 50;

        // Budget checking
        if (is_complex && complexUpdates >= MAX_COMPLEX) continue;
        if (!is_complex && normalUpdates >= MAX_NORMAL) continue;

        // FIXED: Removed trailing comma and added size
        update_fixtures(entry.bodyId, geo, size);
        
        entry.topology_hash = geo.topology_hash;
        entry.is_dirty = false;

        if (is_complex) complexUpdates++; else normalUpdates++;
        
        if (normalUpdates >= MAX_NORMAL && complexUpdates >= MAX_COMPLEX) break;
     }
 }

*/


void RigidBodyManager::update_region_transforms(std::unordered_map<RegionID, RegionRecord>& active_regions) {
    for (auto& [id, entry] : m_body_map) {
        if (!b2Body_IsValid(entry.bodyId)) continue;
        if (b2Body_GetType(entry.bodyId) != b2_dynamicBody) continue;
        if (b2Body_IsAwake(entry.bodyId) == false) continue;

         b2Vec2 pos = b2Body_GetPosition(entry.bodyId);
        auto it = active_regions.find(id);
        if (it != active_regions.end()) {
          if (std::abs(it->second.center_f.y - pos.y) > 0.05f) {
                printf("Region %u falling: y=%.2f\n", id, pos.y);
            }
            b2Vec2 pos = b2Body_GetPosition(entry.bodyId);
            it->second.center_f.x = pos.x;
            it->second.center_f.y = pos.y;
        }
    }
}

void RigidBodyManager::create_body_for_id(RegionID id, const RegionGeometry& geo, uint64_t version, b2BodyType type, float pixel_count) {
    if (std::isnan(geo.center.x) || std::isnan(geo.center.y) || 
        std::abs(geo.center.x) > 1e6f || std::abs(geo.center.y) > 1e6f) {
        return; // Skip creating this body; it's corrupt
    }

    if (geo.convex_pieces.empty()) return;

    b2BodyDef def = b2DefaultBodyDef();
    def.type = type;
    def.userData = (void*)(uintptr_t)id;

    def.enableSleep = true; 
    def.linearDamping = (type == b2_dynamicBody) ? 1.0f : 0.0f; 
    def.angularDamping = 0.5f;
    
    // Position body at centroid
    def.position = { (float)geo.center.x, (float)geo.center.y };
    
    // Top-down camera constraints
    def.fixedRotation = true; 
    def.gravityScale = 1.0f; 

    b2BodyId bodyId = b2CreateBody(m_world_id, &def);
    update_fixtures(bodyId, geo, pixel_count);

    m_body_map[id] = { bodyId, version };
}
void RigidBodyManager::update_fixtures(b2BodyId bodyId, const RegionGeometry& geo,float pixel_count) {
    int count = b2Body_GetShapeCount(bodyId);
    if (count > 0) {
        std::vector<b2ShapeId> shapes(count);
        int actualCount = b2Body_GetShapes(bodyId, shapes.data(), count);
        for (int i = 0; i < actualCount; ++i) {
            b2DestroyShape(shapes[i], false); 
        }
    }
    const float SMALL_THRESHOLD = 20.0f;
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 1.0f;
    if (pixel_count < SMALL_THRESHOLD) {
        // Small debris: Only hits Terrain or Large Chunks (Debris ignore each other)
        shapeDef.filter.categoryBits = CAT_SMALL_DEBRIS;
        shapeDef.filter.maskBits = CAT_TERRAIN | CAT_LARGE_CHUNK;
    } else {
        // Large chunk / Terrain: Hits everything
        shapeDef.filter.categoryBits = CAT_LARGE_CHUNK;
        shapeDef.filter.maskBits = CAT_TERRAIN | CAT_LARGE_CHUNK | CAT_SMALL_DEBRIS;
    }

    // Temporary buffer for vertex transformation
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

        // Restoring b2ComputeHull to fix winding/collinear points and prevent crashes
        b2Hull hull = b2ComputeHull(localVerts, vCount);
        
        // Only create the polygon if the hull validation passes
        if (hull.count >= 3) {
            b2Polygon poly = b2MakePolygon(&hull, 0.0f);
            b2CreatePolygonShape(bodyId, &shapeDef, &poly);
        }
    }

    // Still using the optimized single mass update
    b2Body_ApplyMassFromShapes(bodyId);
}

} // namespace rigid
