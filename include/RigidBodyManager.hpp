#pragma once
#include <vector>
#include <unordered_map>
#include <box2d/box2d.h>
#include "RegionMesher.hpp"
#include <StructuralGraph.hpp>
#include <SDL3/SDL.h>

namespace rigid {

enum CollisionCategory : uint32_t {
  CAT_TERRAIN      = 0x0001,
  CAT_LARGE_CHUNK  = 0x0002,
  CAT_SMALL_DEBRIS = 0x0004
};
struct BodyStore {
  b2WorldId world_id = b2_nullWorldId;
  std::vector<RegionID>   ids;
  std::vector<b2BodyId>   body_ids;
  std::vector<uint64_t>   versions;
  std::vector<uint64_t>   topo_hashes;
  std::vector<uint8_t>    dirty;       // 1 = fixture rebuild pending
  std::unordered_map<RegionID, uint32_t> id_to_slot;
};

void body_store_init   (BodyStore& store, b2WorldId world_id);
void body_store_destroy(BodyStore& store);

void physics_sync(
  BodyStore& store,
  const StructuralGraph& graph,
  const std::vector<bool>& is_stable,
  const std::unordered_map<RegionID, RegionGeometry>& geometry_cache,
  const std::unordered_map<RegionID, RegionRecord>& active_regions);

void physics_read_transforms(
    const BodyStore& store,
    std::unordered_map<RegionID, RegionRecord>& active_regions);

void physics_render_debug(
    const BodyStore& store,
    SDL_Renderer* renderer,
    const std::unordered_map<RegionID, RegionRecord>& active_regions,
    const std::unordered_map<RegionID, RegionGeometry>& geometry_cache,
    float scale_x, float scale_y);

} // namesrace rigid
