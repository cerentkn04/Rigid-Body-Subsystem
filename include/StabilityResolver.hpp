
#pragma once
#include <vector>
#include "StructuralGraph.hpp"

namespace rigid {
    // A Policy is now just a function signature
    typedef void (*StabilityPolicyFunc)(const StructuralGraph&, std::vector<bool>&);

    // Implementation of your connectivity rule
    void apply_basic_connectivity_policy(const StructuralGraph& graph, std::vector<bool>& out_is_stable);
    
    // Implementation of a "Static-Only" rule (for testing)
    void apply_static_everything_policy(const StructuralGraph& graph, std::vector<bool>& out_is_stable);
}
