#pragma once
#include <vector>
#include <cstdint>
namespace world { struct WorldView; }

namespace rigid {

struct RegionBuildRecord;
using RegionIndex = uint32_t;

class RegionExtractor {
public:
    RegionExtractor();
    ~RegionExtractor();

    void extract(const world::WorldView& world,
                 std::vector<RegionBuildRecord>& out_records);

    const std::vector<RegionIndex>& label_grid() const;

private:
    struct Impl;
    Impl* m_impl;
};

}

