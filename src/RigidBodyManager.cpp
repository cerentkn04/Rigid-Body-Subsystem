#include "RigidBodyManager.hpp"
#include <algorithm>
#include <cmath>

static const float PTM = 0.02f; 
static const float MTP = 50.0f; 

namespace rigid {
static void store_remove_slot(BodyStore& store, uint32_t slot) {
    RegionID removed_id = store.ids[slot];
    uint32_t last = (uint32_t)store.ids.size() - 1;
    if (slot != last) {
        RegionID moved_id           = store.ids[last];
        store.ids[slot]             = store.ids[last];
        store.body_ids[slot]        = store.body_ids[last];
        store.versions[slot]        = store.versions[last];
        store.topo_hashes[slot]     = store.topo_hashes[last];
        store.dirty[slot]           = store.dirty[last];
        store.id_to_slot[moved_id]  = slot;
    }
    store.ids.pop_back();
    store.body_ids.pop_back();
    store.versions.pop_back();
    store.topo_hashes.pop_back();
    store.dirty.pop_back();
    store.id_to_slot.erase(removed_id);
}

static void rebuild_fixtures(b2BodyId body_id, const RegionGeometry& geo, float pixel_count) {
    int count = b2Body_GetShapeCount(body_id);
    if (count > 0) {
        std::vector<b2ShapeId> shapes(count);
        int actual = b2Body_GetShapes(body_id, shapes.data(), count);
        for (int i = 0; i < actual; ++i)
            b2DestroyShape(shapes[i], false);
    }

    b2BodyType body_type = b2Body_GetType(body_id);
    b2ShapeDef shape_def = b2DefaultShapeDef();
    shape_def.density    = 1.0f;

    if (body_type == b2_staticBody) {
        shape_def.filter.categoryBits = CAT_TERRAIN;
        shape_def.filter.maskBits     = 0xFFFF;
    } else if (pixel_count < 20.0f) {
        shape_def.filter.categoryBits = CAT_SMALL_DEBRIS;
        shape_def.filter.maskBits     = CAT_TERRAIN | CAT_LARGE_CHUNK;
    } else {
        shape_def.filter.categoryBits = CAT_LARGE_CHUNK;
        shape_def.filter.maskBits     = CAT_TERRAIN | CAT_LARGE_CHUNK | CAT_SMALL_DEBRIS;
    }

    b2Vec2 verts[B2_MAX_POLYGON_VERTICES];
    for (const auto& piece : geo.convex_pieces) {
        int n = (int)std::min(piece.points.size(), (size_t)B2_MAX_POLYGON_VERTICES);
        if (n < 3) continue;
        for (int i = 0; i < n; ++i)
            verts[i] = { piece.points[i].x * PTM, piece.points[i].y * PTM };
        b2Hull hull = b2ComputeHull(verts, n);
        if (hull.count >= 3) {
            b2Polygon poly = b2MakePolygon(&hull, 0.0f);
            b2CreatePolygonShape(body_id, &shape_def, &poly);
        }
    }
    b2Body_ApplyMassFromShapes(body_id);
}

static b2BodyId make_body(b2WorldId world_id, const RegionGeometry& geo,
                           RegionID id, b2BodyType type, float pixel_count) {
    if (std::isnan(geo.center.x) || std::isnan(geo.center.y) ||
        std::abs(geo.center.x) > 1e6f || std::abs(geo.center.y) > 1e6f)
        return b2_nullBodyId;
    if (geo.convex_pieces.empty())
        return b2_nullBodyId;

    b2BodyDef def      = b2DefaultBodyDef();
    def.type           = type;
    def.userData       = (void*)(uintptr_t)id;
    def.enableSleep    = true;
    def.linearDamping  = (type == b2_dynamicBody) ? 2.5f : 0.0f;
    def.angularDamping = 0.1f;
    def.position       = { geo.center.x * PTM, geo.center.y * PTM };
    def.fixedRotation  = true;
    def.gravityScale   = 1.0f;

    b2BodyId body_id = b2CreateBody(world_id, &def);
    rebuild_fixtures(body_id, geo, pixel_count);
    return body_id;
}

// ── Public API ────────────────────────────────────────────────────────────────

void body_store_init(BodyStore& store, b2WorldId world_id) {
    store.world_id = world_id;
}

void body_store_destroy(BodyStore& store) {
    for (b2BodyId bid : store.body_ids)
        if (b2Body_IsValid(bid)) b2DestroyBody(bid);
    store.ids.clear();
    store.body_ids.clear();
    store.versions.clear();
    store.topo_hashes.clear();
    store.dirty.clear();
    store.id_to_slot.clear();
}

void physics_sync(
    BodyStore& store,
    const StructuralGraph& graph,
    const std::vector<bool>& is_stable,
    const std::unordered_map<RegionID, RegionGeometry>& geometry_cache,
    const std::unordered_map<RegionID, RegionRecord>& active_regions)
{
    // 1. Remove slots for bodies whose region no longer exists
    // Iterate backwards so swap-erase doesn't skip elements
    for (uint32_t i = (uint32_t)store.ids.size(); i-- > 0; ) {
        if (active_regions.find(store.ids[i]) == active_regions.end()) {
            if (b2Body_IsValid(store.body_ids[i]))
                b2DestroyBody(store.body_ids[i]);
            store_remove_slot(store, i);
        }
    }

    // 2. Create or update bodies from the structural graph
    for (uint32_t i = 0; i < (uint32_t)graph.nodes.size(); ++i) {
        const RegionID id = graph.nodes[i].id;

        auto geo_it = geometry_cache.find(id);
        auto rec_it = active_regions.find(id);
        if (geo_it == geometry_cache.end() || rec_it == active_regions.end()) continue;

        const auto& geo    = geo_it->second;
        const auto& record = rec_it->second;
        b2BodyType  target = is_stable[i] ? b2_staticBody : b2_dynamicBody;

        auto slot_it = store.id_to_slot.find(id);
        if (slot_it == store.id_to_slot.end()) {
            // New body — append a slot
            b2BodyId bid = make_body(store.world_id, geo, id, target, (float)record.pixel_count);
            if (!b2Body_IsValid(bid)) continue;
            uint32_t slot = (uint32_t)store.ids.size();
            store.ids.push_back(id);
            store.body_ids.push_back(bid);
            store.versions.push_back(record.version);
            store.topo_hashes.push_back(geo.topology_hash);
            store.dirty.push_back(0);
            store.id_to_slot[id] = slot;
        } else {
            uint32_t   slot    = slot_it->second;
            b2BodyId   bid     = store.body_ids[slot];
            b2BodyType current = b2Body_GetType(bid);

            bool version_changed = (store.versions[slot] != record.version);
            if (version_changed) {
                store.dirty[slot]    = 1;
                store.versions[slot] = record.version;
            }

            if (current != target) {
                b2Body_SetType(bid, target);
                if (target == b2_dynamicBody) {
                    b2Vec2 pos = { geo.center.x * PTM, geo.center.y * PTM };
                    b2Body_SetTransform(bid, pos, b2Rot_identity);
                    b2Body_SetLinearVelocity(bid, { 0.0f, 0.1f });
                    b2Body_SetAwake(bid, true);
                } else {
                    b2Body_SetLinearVelocity(bid, { 0.0f, 0.0f });
                    b2Body_SetAngularVelocity(bid, 0.0f);
                }
            } else if (current == b2_staticBody && version_changed) {
                b2Vec2 pos = { geo.center.x * PTM, geo.center.y * PTM };
                b2Body_SetTransform(bid, pos, b2Rot_identity);
            }
        }
    }

    // 3. Budgeted fixture rebuilds — max 2 normal, 1 complex per frame
    int normal_budget = 0, complex_budget = 0;
    for (uint32_t i = 0; i < (uint32_t)store.ids.size(); ++i) {
        if (!store.dirty[i]) continue;

        auto geo_it = geometry_cache.find(store.ids[i]);
        auto rec_it = active_regions.find(store.ids[i]);
        if (geo_it == geometry_cache.end() || rec_it == active_regions.end()) continue;

        const auto& geo = geo_it->second;
        bool complex    = geo.convex_pieces.size() > 50;
        if ( complex && complex_budget >= 1) continue;
        if (!complex && normal_budget  >= 2) continue;

        rebuild_fixtures(store.body_ids[i], geo, (float)rec_it->second.pixel_count);
        store.topo_hashes[i] = geo.topology_hash;
        store.dirty[i]       = 0;
        complex ? ++complex_budget : ++normal_budget;
        if (normal_budget >= 2 && complex_budget >= 1) break;
    }
}

void physics_read_transforms(
    const BodyStore& store,
    std::unordered_map<RegionID, RegionRecord>& active_regions)
{
    for (auto& [id, record] : active_regions)
        record.is_dynamic = false;

    for (uint32_t i = 0; i < (uint32_t)store.ids.size(); ++i) {
        b2BodyId bid = store.body_ids[i];
        if (!b2Body_IsValid(bid)) continue;

        auto it = active_regions.find(store.ids[i]);
        if (it == active_regions.end()) continue;

        RegionRecord& record = it->second;
        b2Vec2 pos = b2Body_GetPosition(bid);

        if (!record.motion_initialized) {
            record.center_f           = { pos.x * MTP, pos.y * MTP };
            record.prev_center_f      = record.center_f;
            record.motion_initialized = true;
        } else {
            record.center_f = { pos.x * MTP, pos.y * MTP };
        }
        record.is_dynamic = (b2Body_GetType(bid) == b2_dynamicBody);
    }
}


void physics_render_debug(
    const BodyStore& store,
    SDL_Renderer* renderer,
    const std::unordered_map<RegionID, RegionRecord>& active_regions,
    const std::unordered_map<RegionID, RegionGeometry>& geometry_cache,
    float scale_x, float scale_y)
{
    for (uint32_t i = 0; i < (uint32_t)store.ids.size(); ++i) {
        b2BodyId bid = store.body_ids[i];
        if (!b2Body_IsValid(bid)) continue;

        auto rec_it = active_regions.find(store.ids[i]);
        auto geo_it = geometry_cache.find(store.ids[i]);
        if (rec_it == active_regions.end() || geo_it == geometry_cache.end()) continue;

        bool is_static = b2Body_GetType(bid) == b2_staticBody;
        if (is_static) SDL_SetRenderDrawColor(renderer, 255,   0, 0, 255);
        else           SDL_SetRenderDrawColor(renderer,   0, 255, 0, 255);

        const auto& record = rec_it->second;
        const auto& geo    = geo_it->second;
        for (const auto& piece : geo.convex_pieces) {
            const auto& pts = piece.points;
            for (size_t j = 0; j < pts.size(); ++j) {
                const auto& p1 = pts[j];
                const auto& p2 = pts[(j + 1) % pts.size()];
                SDL_RenderLine(renderer,
                    (p1.x + record.center_f.x) * scale_x,
                    (p1.y + record.center_f.y) * scale_y,
                    (p2.x + record.center_f.x) * scale_x,
                    (p2.y + record.center_f.y) * scale_y);
            }
        }
    }
}

} // namespace rigid
