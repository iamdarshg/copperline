// Copperline: single-threaded A* over the sparse routing graph.
//
// Search state = (node, layer is inside node, incoming direction). Cost =
// length + bend penalties + via penalties + preferred-layer bias +
// reservation/congestion penalties (phase-3 hook, zero in phase 1).
// Hard electrical legality is structural (illegal edges are never built);
// costs only order legal alternatives, never override legality.
#pragma once

#include <chrono>
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
    // Issue #14: weighted-A* heuristic scale. 1.0 = admissible (legacy).
    // Dense maturity phases raise it (<= cap, default 1.5) to trade
    // optimality for speed. Ordering-only: illegal edges are never built,
    // so legality is structural regardless of the weight.
    double weight_factor = 1.0;
    // Wall-clock budget shared by the whole route command (default none). A
    // search that overruns reports "budget_exhausted" and stops, so one long
    // A* (a deep recovery branch, a large greedy task) can never blow past
    // --timeout by minutes. Ordering-only: no effect on legality.
    std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::time_point::max();
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
    // Issue #23: A*-frontier-only rejection evidence. Aggregated from the
    // per-transition probes stored on the graph, but ONLY for source nodes
    // this search actually expanded -- candidate edges in regions the
    // frontier never reached contribute nothing. Bounded to
    // kMaxFrontierStats, deterministic order, integer-nm. Streams per-node:
    // one pass over each expanded node's probe list, no dense matrix.
    std::vector<GraphFrontierStat> frontier_stats;
};

AStarResult astar_route(const SparseRoutingGraph& graph, const std::vector<double>& layer_mult,
                        const AStarConfig& config);

// Issue #10: windowed/biased exact search for hierarchical guidance.
// `allowed` is a per-node hard window (empty = every node allowed; callers
// always force src/dst allowed). `bias[to]` is a non-negative soft cost
// added when entering `to` (empty = no bias). Hard legality is unchanged:
// illegal edges were never built, so masking/bias only order or skip legal
// alternatives. Unrestricted fallback (empty mask/bias) is bit-identical to
// astar_route.
AStarResult astar_route_masked(const SparseRoutingGraph& graph,
                               const std::vector<double>& layer_mult,
                               const AStarConfig& config,
                               const std::vector<char>& allowed,
                               const std::vector<Coord>& enter_bias);

}  // namespace copperline
