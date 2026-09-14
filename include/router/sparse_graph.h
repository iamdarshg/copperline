// Copperline: sparse routing graph.
//
// The principal routing representation is vector geometry, NOT a uniform
// raster. Per connection task we build a small visibility-style graph whose
// nodes are the task endpoints plus clearance-expanded obstacle corners on
// each layer, connected by Manhattan (straight or single-elbow) edges whose
// centerlines keep full electrical clearance. Layer changes are via edges
// gated on via legality. Hard electrical legality is structural: illegal
// edges are never built, so A* cannot choose them no matter the costs.
#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include "router/board.h"
#include "router/rules.h"

namespace copperline {

struct SparseNode {
    Point p{};
    LayerId layer = 0;
    int base = -1;  // shared geometric point across layer copies
};

struct SparseEdge {
    int to = -1;
    Coord len_nm = 0;
    int dir1 = 4;   // 0:+x 1:+y 2:-x 3:-y 4:none
    int dir2 = -1;  // second leg direction, -1 when straight
    Point elbow{};  // valid when dir2 >= 0
    bool is_via = false;
    Coord penalty_nm = 0;  // reservations/congestion hook (phase 3); 0 in phase 1
};

struct GraphStats {
    int node_count = 0;
    int edge_count = 0;
    int obstacle_count = 0;
};

// Issue #21: bounded summary of rejected frontier transitions. The sparse
// graph builder knows exactly which obstacle killed each candidate edge/node;
// A* only sees legal edges, so this evidence is collected here and forwarded
// through CandidateRoute -> FrontierDiag -> blocker attribution. Aggregated
// per (blocker net, kind, layer) with a deterministic order; memory is
// bounded by top-N truncation (kMaxFrontierStats).
struct GraphFrontierStat {
    NetId blocker_net = -1;  // -1 = keepout / bounds (nothing to rip)
    std::string kind;        // "trace" | "pad" | "via" | "keepout" | "bounds"
    std::string desc;        // "trace:net=X" | "pad:net=..." | "keepout:..." | ...
    LayerId layer = 0;       // query layer the rejection happened on
    Point pos{};             // representative rejection position (first hit)
    int count = 0;           // number of rejected transitions attributed here
};

inline constexpr int kMaxFrontierStats = 8;

class SparseRoutingGraph {
  public:
    const std::vector<SparseNode>& nodes() const { return nodes_; }
    const std::vector<SparseEdge>& edges(int node) const { return adj_[node]; }
    const GraphStats& stats() const { return stats_; }
    // Issue #21: bounded frontier-rejection evidence (sorted, top-N).
    const std::vector<GraphFrontierStat>& frontier_stats() const { return frontier_stats_; }

    // Builds the graph for one point-to-point task against the CURRENT
    // committed board state (pre-routed + already-routed copper).
    static SparseRoutingGraph build(const Board& committed, const RuleResolver& resolver,
                                    NetId net, Point src, Point dst, LayerId src_layer,
                                    LayerId dst_layer, Coord route_width_nm,
                                    const ElectricalContext& ctx);

    int src_node() const { return src_node_; }
    int dst_node() const { return dst_node_; }

    // Adds soft planning costs to edges (congestion/reservations, Prompt 3).
    // Hard legality is untouched: this only biases A* ordering.
    void add_penalties(const std::function<Coord(const SparseNode&, const SparseEdge&)>& fn);

  private:
    std::vector<SparseNode> nodes_;
    std::vector<std::vector<SparseEdge>> adj_;
    GraphStats stats_;
    std::vector<GraphFrontierStat> frontier_stats_;
    int src_node_ = -1;
    int dst_node_ = -1;
};

int direction_of(Point from, Point to);  // 0:+x 1:+y 2:-x 3:-y 4:same

}  // namespace copperline
