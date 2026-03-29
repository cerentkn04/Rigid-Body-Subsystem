#pragma once
#include <vector>
#include "regionScratch.hpp"
namespace world { struct WorldView; }

namespace rigid {

struct ExtractorState {
    std::vector<RegionIndex> label_grid;
    bool merge_same_type = false;
};

void extractor_extract(
    ExtractorState& state,
    const world::WorldView& world,
    std::vector<RegionBuildRecord>& out_records);

} // namespace rigid
