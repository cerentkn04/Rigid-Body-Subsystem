#pragma once
#include <cstdint>

namespace world {

  using WorldRevision = uint64_t;
  using GroupID = uint32_t; 
  enum class CellSolidity : uint8_t {
    Empty = 0,
    Solid = 1
  };
  struct WorldView final {
    // ---- World bounds ----
    int width;
    int height;
    // ---- Query functions ----

    /*
        Returns the solidity of the cell at (x, y).

        Out-of-bounds coordinates are treated as Empty.
    */
    CellSolidity (*solidity_at)(int x, int y);

    /*
        Returns a monotonically increasing revision number representing
        the state of the world.

        Revision semantics:
        - The value must remain constant for the duration of a snapshot.
        - Any mutation that affects solidity must increment the revision.
    */
    WorldRevision (*world_revision)();

    /*
        Returns a revision number associated with the region containing
        (x, y).

        Region revision semantics:
        - Equal values imply no solidity-affecting change in that region.
        - Different values imply at least one such change.
        - Granularity is implementation-defined (cell, chunk, stripe, etc).
    */
    WorldRevision (*region_revision)(int x, int y);
    GroupID (*group_id_at)(int x, int y);
  };
} 

