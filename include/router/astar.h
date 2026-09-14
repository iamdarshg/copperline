// Copperline: single-threaded A* over the sparse routing graph.
//
// Search state = (node, layer is inside node, incoming direction). Cost =
// length + bend penalties + via penalties + preferred-layer bias +
// reservation/congestion penalties (phase-3 hook, zero in phase 1).
// Hard electrical legality is structural (illegal edges are never built);
// costs only order legal alternatives, never override legality.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "router/geometry.h"
#include "router/sparse_graph.h"

namespace copperline {

struct AStarConfig {
    Coord bend_cost_nm = 500000;      // 0.5mm equivalent per bend
    Coord via_cost_nm = 2000000;      // 2mm equivalent per via
    std::int64_t max_expansions = 200000;
};

struct AStarResult {
    bool found = false;
    std::vector<int> node_path;  // src .. dst node ids
    std::vector<int> edge_path;  // edge index per step (edge_path[i] from node_path[i])
    Coord cost_nm = 0;
    std::int64_t expansions = 0;
    // Diagnostics for failure reports.
    int closest_node = -1;
    Coord closest_goal_dist_nm = 0;
    std::string fail_reason;  // "unreachable" | "budget_exhausted"
};

AStarResult astar_route(const SparseRoutingGraph& graph, const std::vector<double>& layer_mult,
                        const AStarConfig& config);

}  // namespace copperline
