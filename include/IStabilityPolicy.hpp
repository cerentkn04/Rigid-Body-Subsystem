 #pragma once
#include <vector>
#include "StructuralGraph.hpp"

namespace rigid {
    // This is our DOD "Interface" - a simple function pointer type
    typedef void (*StabilityPolicyFunc)(const StructuralGraph&, std::vector<bool>&);

    // Declaration for the function in the .cpp
    void apply_basic_connectivity_policy(const StructuralGraph& graph, std::vector<bool>& out_is_stable);
}
