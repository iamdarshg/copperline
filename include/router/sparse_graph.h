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

class SparseRoutingGraph {
  public:
    const std::vector<SparseNode>& nodes() const { return nodes_; }
    const std::vector<SparseEdge>& edges(int node) const { return adj_[node]; }
    const GraphStats& stats() const { return stats_; }

    // Builds the graph for one point-to-point task against the CURRENT
    // committed board state (pre-routed + already-routed copper).
    static SparseRoutingGraph build(const Board& committed, const RuleResolver& resolver,
                                    NetId net, Point src, Point dst, LayerId src_layer,
                                    LayerId dst_layer, Coord route_width_nm,
                                    const ElectricalContext& ctx);

    int src_node() const { return src_node_; }
    int dst_node() const { return dst_node_; }

  private:
    std::vector<SparseNode> nodes_;
    std::vector<std::vector<SparseEdge>> adj_;
    GraphStats stats_;
    int src_node_ = -1;
    int dst_node_ = -1;
};

int direction_of(Point from, Point to);  // 0:+x 1:+y 2:-x 3:-y 4:same

}  // namespace copperline
