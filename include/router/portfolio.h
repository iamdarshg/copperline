// Copperline: diverse A* route portfolio per task (issue #8).
//
// Generates up to K (default 12, cap 15) deterministic diverse alternatives
// for one connection task. The first path is the normal exact A* on the
// shared sparse graph; alternatives rerun A* on copies of the SAME graph
// object with deterministic deviation penalties/masks/biases (bottleneck
// thirds, forbidden middle edge, via-cost layer strategies, first-via
// early/late, corridor pull-away, bend strategies, layer bans). Graph
// construction (obstacle collection + via-bundle gates, the dominant cost)
// happens exactly once per portfolio; per-alternative work is copy + A* +
// materialize, streamed with at most K stored candidates.
//
// Route signature (dedup + sort key), computed from FINAL copper
// (issue #9): quantized bbox of actual trace/via extents (2mm grid) +
// layer set, 8-way principal direction runs (>=1mm), first via from actual
// via positions (L{a}->{b}@early|mid|late by copper-length fraction),
// bottleneck cells from actual via positions + copper midpoint (2mm).
// Tiny geometric perturbations (<2mm jogs) share a signature and collapse.
//
// Costs (issue #8): CandidateRoute carries search_cost_nm (raw A*,
// debug-only) and materialized_cost_nm (recomputed from final Euclidean
// copper + via/bend/layer costs). Portfolio dedup + ordering use the
// materialized cost; cost_nm mirrors it for compatibility.
//
// Every candidate is exactly legalized via candidate_legal_vs_board before
// entering the portfolio. Costs are integer-nm A* costs; ordering is
// (cost, signature) deterministic. Search work (expansions, coarse work,
// graph builds) and effective K/threads are exposed in JSON.
//
// Consumes: EffectiveSearchBudget route-K (#14, via budget_route_k),
// hierarchy guidance (#10, via HierarchyCache bias + coarse accounting) and
// interference batching (#3, via reservations/congestion inputs plus the
// K x batch memory bound below).
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "router/astar.h"
#include "router/board.h"
#include "router/hierarchy.h"
#include "router/json.h"
#include "router/parallel.h"
#include "router/rules.h"
#include "router/sparse_graph.h"

namespace copperline {

// Default / hard cap for the portfolio width.
inline constexpr int kPortfolioDefaultK = 12;
inline constexpr int kPortfolioMaxK = 15;
inline constexpr int kPortfolioMinK = 1;
// Per-alternative planning overhead for the K x batch memory bound.
// The shared graph itself is charged once per task (per_task_bytes);
// each extra alternative streams copy + A* + materialize scratch.
inline constexpr std::size_t kPerPortfolioAltBytes = 8ULL * 1024ULL * 1024ULL;

struct RouteSignature {
    std::string corridor_class;  // "WxH:L0+L1" quantized to 2mm
    std::string dir_seq;         // "E-N-E" principal runs >= 1mm
    std::string first_via;       // "none" | "L0->L1@early|mid|late"
    std::string bottlenecks;     // "c(x,y,l),..." quantized 2mm cells
    std::string to_string() const;
    JsonValue to_json() const;
};

struct PortfolioCandidate {
    CandidateRoute route;
    RouteSignature sig;
    std::string sig_str;
};

struct PortfolioOptions {
    int requested_k = kPortfolioDefaultK;  // clamped to [1, max_k]
    int max_k = kPortfolioMaxK;            // hard cap (agents control)
    std::size_t memory_budget_bytes = kRouterMemoryBudgetBytes;
    std::size_t per_task_bytes = kPerCandidateBytes;  // shared graph charge
    std::size_t per_alt_bytes = kPerPortfolioAltBytes;
    int batch_width = kParallelBatchSize;  // epoch batch for K x batch bound
    int budget_route_k = -1;  // EffectiveSearchBudget route_k (<0 = no budget)
    int threads_requested = 0;  // 0 = auto (resolve_worker_threads)
    AStarConfig astar;
    HierarchyConfig hier;
    // Issue #4: sparse-graph budget for the shared build (from the maturity
    // EffectiveSearchBudget). has_graph_budget=false = legacy 384/16.
    bool has_graph_budget = false;
    SparseGraphBudget graph_budget;
};

struct PortfolioResult {
    int requested_k = kPortfolioDefaultK;
    int effective_k = 1;
    int threads_effective = 1;
    int graph_builds = 0;  // always 1 on attempted ordinary tasks (reuse proof)
    std::int64_t total_expansions = 0;  // exact A* across all attempts
    std::int64_t coarse_expansions = 0;  // hierarchy coarse work (once)
    std::vector<PortfolioCandidate> candidates;  // sorted by (cost, sig)
    std::string fail_reason;  // set when empty ("unreachable"|"budget_exhausted"|...)
    std::size_t size() const { return candidates.size(); }
    const PortfolioCandidate* best() const {
        return candidates.empty() ? nullptr : &candidates.front();
    }
    JsonValue to_json() const;
};

// Clamp a requested K into [1, max_k] (max_k itself clamped to [1, 64]).
int clamp_portfolio_k(int requested_k, int max_k = kPortfolioMaxK);

// Effective K after the maturity budget and the K x batch memory bound:
//   min(requested, max_k, budget_route_k?, floor((budget/batch - graph)/alt)).
// Always >= 1. Deterministic pure function.
int effective_portfolio_k(int requested_k, int max_k, int budget_route_k,
                          std::size_t memory_budget_bytes, int batch_width,
                          std::size_t per_task_bytes, std::size_t per_alt_bytes);

// Signature of one A* path on its graph. Pure + deterministic.
// Legacy path kept for unit tests; the portfolio itself uses the
// copper-based overload below (issue #9).
RouteSignature compute_route_signature(const SparseRoutingGraph& graph,
                                       const AStarResult& res);

// Issue #9: signature from final materialized copper (authoritative for
// dedup). Quantized bbox from actual trace/via extents, 8-way direction
// runs from actual segments, first via from actual via positions,
// bottlenecks from actual copper. Pure + deterministic.
RouteSignature compute_route_signature(const std::vector<TraceSeg>& traces,
                                       const std::vector<Via>& vias);
RouteSignature compute_route_signature(const CandidateRoute& cand);

// Full portfolio for one ordinary task. Pair-corridor tasks fall back to a
// single route_candidate_task result wrapped as K=1 (pair materialization
// stays atomic in the global path). Every returned candidate is exactly
// legal vs `snapshot`. Deterministic for equal inputs.
PortfolioResult build_portfolio(const Board& snapshot, const RuleResolver& resolver,
                                const ConnectionTask& task, std::size_t task_index,
                                double difficulty, const ElectricalContext& ctx,
                                const std::vector<double>& layer_mult,
                                const AStarConfig& astar_cfg,
                                const CongestionMap& congestion,
                                const ReservationSet& reservations,
                                const HierarchyConfig& hier_cfg,
                                const HierarchyCache* hier_cache,
                                const PortfolioOptions& opts);

}  // namespace copperline
