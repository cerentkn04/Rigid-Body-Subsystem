#pragma once
#include <cstdint>
#include <functional>

namespace world {

  using WorldRevision = uint64_t;
  using GroupID  = uint32_t;
  using ObjectID = uint32_t; // 0 = runtime solid, >0 = authored object identity

  enum class CellSolidity : uint8_t { Empty = 0, Solid = 1 };

  struct WorldView final {
    int width  = 0;
    int height = 0;

    std::function<CellSolidity(int x, int y)>    solidity_at;
    std::function<WorldRevision()>               world_revision;
    std::function<WorldRevision(int x, int y)>   region_revision;
    std::function<GroupID(int x, int y)>         group_id_at;
    std::function<ObjectID(int x, int y)>        object_id_at;
  };

} // namespace world
