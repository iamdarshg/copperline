// Copperline: sparse routing graph.
//
// The principal routing representation is vector geometry, NOT a uniform
// raster. Per connection task we build a small visibility-style graph whose
// nodes are the task endpoints plus clearance-expanded obstacle corners on
// each layer, connected by direct line-of-sight edges at arbitrary angles
// (Euclidean length) with Manhattan straight / single-elbow fallbacks whose
// centerlines keep full electrical clearance. Layer changes are via edges
// gated on via legality. Hard electrical legality is structural: illegal
// edges are never built, so A* cannot choose them no matter the costs.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "router/board.h"
#include "router/rules.h"

namespace copperline {

// Issue #4: configurable sparse-graph search budgets. The hard caps that
// used to live in build_multi (384 bases culled by proximity to the direct
// src->dst segments, 16 nearest neighbours per node) are now explicit
// parameters so maturity level / route tasks can spend more on dense boards
// while the defaults keep easy boards fast (bit-identical behaviour).
//   max_bases  - cap on candidate base points after dedup (0 = uncapped:
//                keep every base; src + all dsts always survive).
//   k_nearest  - K nearest neighbours tried per node per layer (<=0 =
//                uncapped: try every same-layer node; aligned pairs are
//                always tried regardless).
// Genuinely expanded scale beyond the ordinary budget: the widest finite
// base set we will ever build. A true "uncapped" base set makes the K-NN
// edge pass O(bases^2); on a large board (thousands of pads/traces) that
// turns a single failing task into minutes of CPU, so the last-resort /
// CLOSURE scale is bounded here instead of being unlimited.
inline constexpr std::size_t kLastResortMaxBases = 2048;
inline constexpr int kLastResortKNearest = 64;

struct SparseGraphBudget {
    std::size_t max_bases = 384;
    int k_nearest = 16;
    static SparseGraphBudget defaults() { return SparseGraphBudget{}; }
    // Genuinely expanded last-resort scale: wide base set + wide K.
    // Consumed by the unreachable fallback in route_candidate_task /
    // build_portfolio before a task is declared unreachable.
    static SparseGraphBudget last_resort() {
        SparseGraphBudget b;
        b.max_bases = kLastResortMaxBases;
        b.k_nearest = kLastResortKNearest;
        return b;
    }
    bool is_last_resort() const {
        return max_bases >= kLastResortMaxBases && k_nearest >= kLastResortKNearest;
    }
};

inline constexpr std::size_t kDefaultMaxBases = 384;
inline constexpr int kDefaultKNearest = 16;

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

// Issue #21: bounded summary of rejected frontier transitions. Aggregated
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

// Issue #23: per-transition rejection metadata carried from sparse-graph
// construction. The builder knows exactly which obstacle killed each
// candidate edge, but global construction-time aggregation biases
// dependency/rip-up ranking toward copper in regions A* never reaches.
// So each rejected candidate transition is stored on its source node and
// only accumulated when A* actually expands that node (see
// AStarResult::frontier_stats). One entry per rejected probe; memory is
// bounded by the probe count (nodes * rejected-degree, small: bases are
// capped) and every aggregation truncates to kMaxFrontierStats.
struct SparseRejectedProbe {
    NetId blocker_net = -1;  // -1 = keepout / bounds (nothing to rip)
    std::string kind;        // "trace" | "pad" | "via" | "keepout" | "bounds" | "plane"
    std::string desc;        // "trace:net=X" | "pad:net=..." | "keepout:..." | ...
    LayerId layer = 0;       // query layer the rejection happened on
    Point pos{};             // representative rejection position for this probe
};

struct SparseTarget {
    Point p{};
    LayerId layer = 0;
};

class SparseRoutingGraph {
  public:
    const std::vector<SparseNode>& nodes() const { return nodes_; }
    const std::vector<SparseEdge>& edges(int node) const { return adj_[node]; }
    // Issue #23: rejected candidate transitions out of `node`, one entry per
    // probe (parallel to edges()). Only meaningful aggregated over the nodes
    // A* actually expanded; never aggregate globally for attribution.
    const std::vector<SparseRejectedProbe>& rejected(int node) const { return rejected_[node]; }
    const GraphStats& stats() const { return stats_; }
    // Issue #21 legacy: construction-wide aggregation over ALL candidate
    // probes (reachable or not). Diagnostics only; blocker attribution must
    // use AStarResult::frontier_stats (A*-frontier-only). Computed on demand:
    // production never reads it, so the hot build path no longer pays to
    // aggregate every probe (string-carrying) on every graph.
    std::vector<GraphFrontierStat> frontier_stats() const;
    // Issue #23: aggregate per-node probes over an expanded set only.
    // `expanded` is per-node (nonzero = A* expanded the source node).
    // Deterministic, bounded to kMaxFrontierStats.
    std::vector<GraphFrontierStat> frontier_stats_for_expanded(
        const std::vector<char>& expanded) const;
    // Shared bounded aggregation: (count desc, net, kind, layer, desc).
    // Representative position is the min (x, y) hit per key, so the result
    // is independent of visit/expansion order. Integer-nm throughout.
    static std::vector<GraphFrontierStat> aggregate_probes(
        const std::vector<SparseRejectedProbe>& probes);
    // Same bounded aggregation over per-node probe sets directly (avoids a
    // full copy of every probe). Identical result: per-key counts and the
    // min-(x, y) representative are order-independent.
    static std::vector<GraphFrontierStat> aggregate_probe_sets(
        const std::vector<std::vector<SparseRejectedProbe>>& sets);

    // Builds the graph for one point-to-point task against the CURRENT
    // committed board state (pre-routed + already-routed copper).
    // exempt_net: coupled pair sibling treated as own-net (gap-governed at
    // materialization, not voltage-governed here). -1 disables the exemption.
    static SparseRoutingGraph build(const Board& committed, const RuleResolver& resolver,
                                    NetId net, Point src, Point dst, LayerId src_layer,
                                    LayerId dst_layer, Coord route_width_nm,
                                    const ElectricalContext& ctx,
                                    const SparseGraphBudget& budget = SparseGraphBudget{},
                                    NetId exempt_net = -1);
    // Issue #4: multi-target build. src is the unconnected terminal; dsts
    // are legal contact points on committed same-net copper (+/- the
    // representative pad). A* succeeds when ANY dst node is reached, which
    // is what lets later terminals T-junction mid-copper.
    // Issue #10: optional corridor clip. When clip_path is non-null, base
    // points farther than clip_half_width_nm (Manhattan) from the polyline
    // are dropped before node creation, except src and every dst (always
    // kept). The clipped graph is a strict subset of the full legal graph,
    // so callers retry unclipped on failure and legality is unchanged.
    static SparseRoutingGraph build_multi(const Board& committed, const RuleResolver& resolver,
                                          NetId net, Point src, LayerId src_layer,
                                          const std::vector<SparseTarget>& dsts,
                                          Coord route_width_nm, const ElectricalContext& ctx,
                                          const std::vector<Point>* clip_path = nullptr,
                                          Coord clip_half_width_nm = 0,
                                          const SparseGraphBudget& budget = SparseGraphBudget{},
                                          NetId exempt_net = -1);

    int src_node() const { return src_node_; }
    int dst_node() const { return dst_node_; }
    const std::vector<int>& dst_nodes() const { return dst_nodes_; }

    // Adds soft planning costs to edges (congestion/reservations, Prompt 3).
    // Hard legality is untouched: this only biases A* ordering.
    void add_penalties(const std::function<Coord(const SparseNode&, const SparseEdge&)>& fn);

    // Shallow geometry clone for search-variant exploration: copies nodes,
    // edges, endpoints and stats but NOT the per-node rejected-probe ledger
    // (attribution-only, and potentially very large: one string-carrying
    // probe per rejected candidate leg). The portfolio holds one graph per
    // alternative, so cloning geometry instead of the full graph removes the
    // dominant copy cost. A* still behaves identically; only failure-frontier
    // attribution over a clone is empty (never consumed on that path).
    SparseRoutingGraph clone_for_search() const;

  private:
    std::vector<SparseNode> nodes_;
    std::vector<std::vector<SparseEdge>> adj_;
    std::vector<std::vector<SparseRejectedProbe>> rejected_;
    GraphStats stats_;
    int src_node_ = -1;
    int dst_node_ = -1;
    std::vector<int> dst_nodes_;
};

// Diagnostics: cumulative per-section wall time inside
// SparseRoutingGraph::build_multi (process-wide, summed across threads), plus
// the build call count. For --time-stages profiling only; never affects
// routing. Reset before a measured run.
struct SparseBuildPhases {
    std::int64_t obstacles_ns = 0;
    std::int64_t bases_ns = 0;
    std::int64_t nodes_ns = 0;
    std::int64_t edges_ns = 0;
    std::int64_t via_ns = 0;
    std::int64_t finalize_ns = 0;
    std::int64_t calls = 0;
    std::int64_t nodes_total = 0;  // summed node_count across builds
    std::int64_t edges_total = 0;  // summed edge_count across builds
};
SparseBuildPhases sparse_graph_build_phases();
void sparse_graph_build_phases_reset();

int direction_of(Point from, Point to);  // 0:+x 1:+y 2:-x 3:-y 4:same

}  // namespace copperline
