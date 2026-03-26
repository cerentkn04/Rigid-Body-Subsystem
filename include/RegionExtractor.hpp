#pragma once
#include <vector>
#include <cstdint>
#include "regionScratch.hpp"
namespace world { struct WorldView; }

namespace rigid {

struct ExtractorState {
    std::vector<RegionIndex> label_grid;
};

void extractor_extract(
    ExtractorState& state,
    const world::WorldView& world,
    std::vector<RegionBuildRecord>& out_records);

} // namespace rigid
