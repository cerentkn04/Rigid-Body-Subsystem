#include "RigidBodyManager.hpp"
#include <cstdio>
#include <algorithm>
const float PTM = 0.02f; // Pixels-To-Meters (1/50)
const float MTP = 50.0f;
namespace rigid {

RigidBodyManager::RigidBodyManager(b2WorldId worldId) : m_world_id(worldId) {}

RigidBodyManager::~RigidBodyManager() {
    for (auto& [id, entry] : m_body_map) {
        if (b2Body_IsValid(entry.bodyId)) {
            b2DestroyBody(entry.bodyId);
        }
    }
}
void RigidBodyManager::render_debug_hulls(
    SDL_Renderer* renderer,
    const std::unordered_map<RegionID, RegionRecord>& active_regions,
    const std::unordered_map<RegionID, RegionGeometry>& geometry_cache,
    float scaleX, float scaleY) 
{
    for (auto const& [id, record] : active_regions) {
        // Look up our internal body mapping
        auto it = m_body_map.find(id);
        if (it == m_body_map.end() || !b2Body_IsValid(it->second.bodyId)) continue;

        // Look up geometry
        auto geo_it = geometry_cache.find(id);
        if (geo_it == geometry_cache.end()) continue;

        // Set color based on body type
        bool isStatic = b2Body_GetType(it->second.bodyId) == b2_staticBody;
        if (isStatic) SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); // Red
        else SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);          // Green

        const auto& geo = geo_it->second;
        for (const auto& piece : geo.convex_pieces) {
            for (size_t i = 0; i < piece.points.size(); ++i) {
                const auto& p1 = piece.points[i];
                const auto& p2 = piece.points[(i + 1) % piece.points.size()];
                SDL_RenderLine(renderer, 
                    (p1.x + record.center_f.x) * scaleX, (p1.y + record.center_f.y) * scaleY,
                    (p2.x + record.center_f.x) * scaleX, (p2.y + record.center_f.y) * scaleY);
            }
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
        
        b2BodyType targetType = is_stable_decisions[i] ? b2_staticBody : b2_dynamicBody;

        auto body_it = m_body_map.find(id);
        if (body_it == m_body_map.end()) {
            create_body_for_id(id, geo, record.version, targetType, (float)record.pixel_count);
            m_body_map[id].topology_hash = geo.topology_hash;
        } else {
            BodyEntry& entry = body_it->second;
            b2BodyType currentType = b2Body_GetType(entry.bodyId);

            
            if (entry.version != record.version) {
    entry.is_dirty = true;
    entry.version = record.version;
}

// Inside RigidBodyManager::synchronize
if (currentType != targetType) {
    // ALWAYS update the type
    b2Body_SetType(entry.bodyId, targetType);

    if (targetType == b2_dynamicBody) {
      b2Body_SetAwake(entry.bodyId, true);
    // Use PTM here as well
    b2Body_SetLinearVelocity(entry.bodyId, {0.0f, 0.1f});
    b2Vec2 newPos = { (float)geo.center.x * PTM, (float)geo.center.y * PTM };

    b2Body_SetTransform(entry.bodyId, newPos, b2Rot_identity);
    b2Body_SetAwake(entry.bodyId, true);
       // b2Vec2 newPos = { (float)geo.center.x, (float)geo.center.y };
       // b2Body_SetTransform(entry.bodyId, newPos, b2Rot_identity);
        b2Body_SetAwake(entry.bodyId, true);
    } 
    else {
        // Transitioning to Static: Zero out velocities to prevent "ghost" movement
        b2Body_SetLinearVelocity(entry.bodyId, {0,0});
        b2Body_SetAngularVelocity(entry.bodyId, 0.0f);
    }
}
else if (currentType == b2_staticBody && entry.version != record.version) {
    // If it's already static but the version changed, snap its position 
    // to the new geometric center so collisions align with pixels.
    //
    b2Vec2 newPos = { (float)geo.center.x * PTM, (float)geo.center.y * PTM };
    b2Body_SetTransform(entry.bodyId, newPos, b2Rot_identity);
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

void RigidBodyManager::update_region_transforms(std::unordered_map<RegionID, RegionRecord>& active_regions) {
for (auto& [id, record] : active_regions) {
        record.is_dynamic = false;
    }
  for (auto& [id, entry] : m_body_map) {
        if (!b2Body_IsValid(entry.bodyId)) continue;

         b2Vec2 pos = b2Body_GetPosition(entry.bodyId);
        auto it = active_regions.find(id);
        if (it != active_regions.end()) {
          if (std::abs(it->second.center_f.y - pos.y) > 0.05f) {
            }
          it->second.center_f.x = pos.x * MTP;
            it->second.center_f.y = pos.y * MTP;
            it->second.is_dynamic = (b2Body_GetType(entry.bodyId) == b2_dynamicBody);
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
    def.linearDamping = (type == b2_dynamicBody) ? 2.5f : 0.0f; 
    def.angularDamping = 0.1f;
    
    // Position body at centroid
    def.position = { (float)geo.center.x * PTM, (float)geo.center.y * PTM };
    //def.position = { (float)geo.center.x, (float)geo.center.y };
    
    // Top-down camera constraints
    def.fixedRotation = true; 
    def.gravityScale = 1.0f; 



    b2BodyId bodyId = b2CreateBody(m_world_id, &def);
    update_fixtures(bodyId, geo, pixel_count);
    b2Body_ApplyMassFromShapes(bodyId);
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




b2BodyType bodyType = b2Body_GetType(bodyId);
    const float SMALL_THRESHOLD = 20.0f;
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    
    // Set material properties so they don't slide like ice
    shapeDef.density = 1.0f;

    // --- DATA-DRIVEN FILTERING ---
    if (bodyType == b2_staticBody) {
        shapeDef.filter.categoryBits = CAT_TERRAIN;
        shapeDef.filter.maskBits = 0xFFFF;
        //shapeDef.filter.maskBits = CAT_LARGE_CHUNK | CAT_SMALL_DEBRIS;
    } 
    else if (pixel_count < SMALL_THRESHOLD) {
        // Falling small debris
        shapeDef.filter.categoryBits = CAT_SMALL_DEBRIS;
        shapeDef.filter.maskBits = CAT_TERRAIN | CAT_LARGE_CHUNK;
    } 
    else {
        // Falling large chunks
        shapeDef.filter.categoryBits = CAT_LARGE_CHUNK;
        shapeDef.filter.maskBits = CAT_TERRAIN | CAT_LARGE_CHUNK | CAT_SMALL_DEBRIS;
    }



    // Temporary buffer for vertex transformation
    b2Vec2 localVerts[B2_MAX_POLYGON_VERTICES];

    for (const auto& piece : geo.convex_pieces) {
        int vCount = (int)std::min(piece.points.size(), (size_t)B2_MAX_POLYGON_VERTICES);
        if (vCount < 3) continue;

        for (int i = 0; i < vCount; ++i) {
             localVerts[i].x = piece.points[i].x * PTM;
localVerts[i].y = piece.points[i].y * PTM;

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
