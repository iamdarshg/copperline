// Copperline: parallel global routing support (Prompt 3).
//
// Multicore without nondeterministic board mutation:
//
//   Workers operate on an immutable committed snapshot, never commit copper
//   themselves, and return candidate routes only. A deterministic central
//   arbiter revalidates every candidate, builds conflicts, selects a
//   deterministic compatible subset, and commits it as one epoch (atomic
//   batch commit). Worker completion order never affects the commit order:
//   candidates are collected into indexed slots and arbitrated in
//   (difficulty, net, a, b) order.
//
//   Reservations are soft costs, never hard locks: they bias A* edge costs
//   but illegal edges are still never built and a unique legal route is never
//   rejected for predicted use. Congestion follows the Pathfinder split:
//   present_cost (current-epoch corridor pressure) vs history_cost (persistent
//   penalty for resources implicated in conflicts/failures). Negotiation
//   affects planning pressure only; final DRC legality is always exact.
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "router/astar.h"
#include "router/board.h"
#include "router/hierarchy.h"
#include "router/json.h"
#include "router/route_tree.h"
#include "router/rules.h"
#include "router/sparse_graph.h"

namespace copperline {

// Fixed batch width. Batch membership is a pure function of the deterministic
// scheduler order and interference weights, therefore independent of
// --threads, which only controls how many candidates are computed
// concurrently. This is what keeps the committed geometry identical at 1, 2,
// 4, ... workers. The width is additionally bounded by the router memory
// budget (see memory_bounded_batch_width): effective_width =
// min(kParallelBatchSize, floor(budget/per_task)). With the default
// 2048MB/64MB bound this stays 8, so determinism tests are unaffected.
inline constexpr int kParallelBatchSize = 8;
// Epochs with zero accepted candidates tolerated before giving up.
inline constexpr int kMaxStalledEpochs = 4;

// Issue #3: interference-aware batch scheduling defaults + memory bounds.
inline constexpr double kDefaultInterferenceThreshold = 1.0;
inline constexpr double kInterferenceRelaxFactor = 2.0;
inline constexpr int kBatchMaxRelaxSteps = 8;
// Sparse-interference cap: at most this many positive pairs are materialized
// for diagnostics. Batch formation itself streams rows (O(batch * remaining))
// and never allocates a dense N^2 double matrix.
inline constexpr std::size_t kMaxStoredInterferencePairs = 8192;
// Router memory budget (dev default, mirrors build/test cap): 2048MB total.
// Per-candidate planning overhead estimate used to bound batch width.
inline constexpr std::size_t kRouterMemoryBudgetBytes = 2048ULL * 1024ULL * 1024ULL;
inline constexpr std::size_t kPerCandidateBytes = 64ULL * 1024ULL * 1024ULL;
// Starvation bonus per deferred epoch added to the scheduling value so
// repeatedly deferred tasks rise to the front deterministically.
inline constexpr double kStarvationBonusPerEpoch = 1.0;

// ---- Route difficulty vector (Prompt 3, section "Route difficulty") ----

struct DifficultyVector {
    double span_mm = 0;              // endpoint Manhattan distance
    double endpoint_density = 0;     // max local pin density at endpoints
    double free_space = 0;           // fraction of corridor free of obstacles
    double corridor_count = 0;       // distinct layer x elbow options
    double corridor_scarcity = 0;    // 1 - free, scaled by obstacle coverage
    double width_mm = 0;             // required electrical trace width
    double clearance_mm = 0;         // max voltage clearance burden
    double layer_restriction = 0;    // costly-layer fraction (0..1-ish)
    double via_restriction = 0;      // extra vias / restricted via class
    double prev_failures = 0;        // previous failed attempts for this task
    double fine_pitch_depth = 0;     // max centre depth at endpoints
    double total = 0;                // weighted sum used for ordering
};

DifficultyVector compute_difficulty(const Board& board, const RuleResolver& resolver,
                                    const ConnectionTask& task, const ElectricalContext& ctx,
                                    const std::vector<double>& terminal_density,
                                    const std::map<TermId, int>& centre_depth,
                                    int prev_failures);

// ---- Probable corridors + interference graph ----

struct Corridor {
    Rect rect{};  // endpoint bbox expanded by width/2 + max clearance
    Coord width_nm = 0;
    Coord clear_nm = 0;
    // Issue #3: electrical resource scarcity carried into the interference
    // weight (not pure bbox overlap). layer_scarcity mirrors the difficulty
    // costly-layer fraction; via_scarcity mirrors the via-restriction burden.
    // electrical_weight = 1 + layer + via (>= 1); the pairwise weight scales
    // by the mean of the two endpoints' weights.
    double layer_scarcity = 0;
    double via_scarcity = 0;
    double electrical_weight = 1.0;
};

Corridor probable_corridor(const Board& board, const RuleResolver& resolver,
                           const ConnectionTask& task, const ElectricalContext& ctx);

// Issue #3: single pairwise interference weight (symmetric, deterministic).
// Zero when corridors are disjoint; otherwise overlap area scaled by the
// electrical resource demand (width + clearance + layer/via scarcity).
double interference_weight(const Corridor& a, const Corridor& b);

// Symmetric pairwise interference weights, parallel to the task vector.
std::vector<std::vector<double>> build_interference(const std::vector<ConnectionTask>& tasks,
                                                    const std::vector<Corridor>& corridors);

// Deterministic batch scheduler: stable order is (difficulty desc, net, a,
// b); batches are consecutive slices of that order with fixed width.
// Legacy path kept for unit tests; the engine uses the interference-aware
// select_interference_batch() below (issue #3).
std::vector<std::vector<int>> schedule_batches(const std::vector<ConnectionTask>& tasks,
                                                int batch_size = kParallelBatchSize);

// ---- Issue #3: interference-aware greedy batch selection ----

struct BatchSchedOptions {
    int batch_width = kParallelBatchSize;
    double interference_threshold = kDefaultInterferenceThreshold;
    double relax_factor = kInterferenceRelaxFactor;
    int max_relax_steps = kBatchMaxRelaxSteps;
    std::size_t memory_budget_bytes = kRouterMemoryBudgetBytes;
    std::size_t per_task_bytes = kPerCandidateBytes;
};

struct BatchSelection {
    std::vector<int> selected;  // global task indices, priority order
    // Pairwise interference among selected (i<j in selected order), bounded
    // to batch_width^2 entries (<= 28 for width 8). Never a dense N^2 matrix.
    std::vector<std::pair<std::pair<int, int>, double>> pair_scores;
    double threshold_used = kDefaultInterferenceThreshold;
    int relax_steps = 0;
    double max_interference = 0;
    double mean_interference = 0;
    int effective_width = kParallelBatchSize;
    std::size_t pairs_stored = 0;
    std::size_t pairs_capped = 0;  // pairs dropped by the sparse cap (0 for batches)
};

// Memory-bounded batch width: min(requested, floor(budget/per_task)), >= 1.
int memory_bounded_batch_width(int requested_width, std::size_t budget_bytes,
                               std::size_t per_task_bytes);
// Worker resolution: explicit --threads wins; 0/negative means "all CPUs".
int resolve_worker_threads(int requested_threads);

// Greedy batch from a deterministic priority order (engine scheduler order).
// Starts from the highest-value eligible task, then adds each later task in
// order when its max interference vs the selected set stays below threshold.
// One task per net per epoch. When the batch cannot fill, the threshold is
// progressively relaxed (x relax_factor per pass); a final infinite-threshold
// pass guarantees workers never idle and deferred tasks eventually run.
// Pure function of (ordered, tasks, corridors, options): independent of the
// worker count, so --threads never changes the committed geometry.
BatchSelection select_interference_batch(const std::vector<int>& ordered_global_idx,
                                         const std::vector<ConnectionTask>& tasks,
                                         const std::vector<Corridor>& corridors,
                                         const BatchSchedOptions& opts);

// ---- Congestion (Pathfinder-style present/history split) ----

struct Hotspot {
    double x_mm = 0;
    double y_mm = 0;
    double present = 0;
    double history = 0;
};

class CongestionMap {
  public:
    CongestionMap() = default;
    void init(const Board& board, int cells_per_side = 32);
    void reset_present();
    void add_present_corridor(const Rect& rect, double weight);
    void add_history_rect(const Rect& rect, double amount);
    void add_history_segment(const Segment& seg, double amount);
    // Soft A* penalty for a centerline segment (capped; never a hard lock).
    Coord penalty_for_segment(const Segment& seg) const;
    std::vector<Hotspot> hotspots(int top_k = 8) const;
    double present_at(Point p) const;
    double history_at(Point p) const;

  private:
    int cell_of_x(Coord x) const;
    int cell_of_y(Coord y) const;
    Rect bounds_{};
    Coord cell_w_ = 1;
    Coord cell_h_ = 1;
    int n_ = 32;
    std::vector<double> present_;
    std::vector<double> history_;
};

// Soft reservations from probable corridors of not-yet-routed tasks.
// Stronger for high-difficulty tasks. Pure cost: never rejects the only
// legal route for predicted use (callers only add it to edge penalties).
class ReservationSet {
  public:
    void build(const std::vector<ConnectionTask>& tasks, const std::vector<Corridor>& corridors,
               const std::vector<double>& difficulties);
    Coord penalty_for_segment(std::size_t self_task, const Segment& seg) const;
    // Issue #14: maturity-driven reservation strength (1.0 = legacy).
    // Soft cost only: scales the penalty, never hard legality.
    void set_strength(double s) { strength_ = (s > 0) ? s : 1.0; }
    double strength() const { return strength_; }

  private:
    std::vector<Corridor> corridors_;
    std::vector<double> weights_;
    double strength_ = 1.0;
};

// ---- Candidates (worker outputs; committed state is never touched) ----

// Issue #21: bounded frontier-rejection evidence forwarded from the sparse
// graph. Each entry aggregates rejected transitions for one
// (blocker net, kind, layer): what actually stopped expansion, not a
// rectangular corridor guess. Bounded to kMaxFrontierStats entries.
struct FrontierBlockerStat {
    NetId blocker_net = -1;
    std::string kind;  // "trace" | "pad" | "via" | "keepout" | "bounds"
    std::string desc;  // stable label, e.g. "trace:net=SEAL"
    LayerId layer = 0;
    Point pos{};
    int count = 0;
};

struct CandidateRoute {
    ConnectionTask task;
    std::size_t task_index = 0;  // index into the epoch task vector
    double difficulty = 0;
    bool found = false;
    std::string fail_reason;  // "unreachable" | "budget_exhausted" | ...
                              // "via_bundle_infeasible" | "no_via_class" | ...
    std::int64_t expansions = 0;
    Coord cost_nm = 0;
    // Issue #8: search vs materialized cost split. search_cost_nm is the raw
    // A* integer cost (pre-simplification, Manhattan + bend/via penalties);
    // kept for debugging/attribution. materialized_cost_nm is recomputed
    // from final copper (Euclidean trace length x layer mult + via + bend
    // costs); cost_nm mirrors it for backward compatibility and is the
    // ordering key everywhere (portfolio, impact).
    Coord search_cost_nm = 0;
    Coord materialized_cost_nm = 0;
    std::vector<TraceSeg> traces;
    std::vector<Via> vias;
    Point gate_a{};  // candidate endpoints (== task terminals on success)
    Point gate_b{};
    int closest_node = -1;
    Coord closest_goal_dist_nm = 0;
    // Issue #21: actual frontier rejections from graph construction.
    std::vector<FrontierBlockerStat> frontier_blockers;
    // Parallel-via diagnostics (issue #5): the current the transition must
    // carry, the style selected for it, the parallel count and the bundle
    // outcome ("ok" | "bundle_blocked" | "no_via_class"). Set for every
    // candidate, including failures, so the engine can report them.
    double required_current_a = 0.0;
    std::string via_style;
    int vias_required = 1;
    std::string via_reason = "ok";
    // Issue #10: hierarchical guidance diagnostics (levels used, fallback
    // status, exact-expansion count). Always filled, including failures.
    HierarchyDiag hierarchy;
    // Issue #9: future-obstruction score for this candidate (filled by the
    // engine when the impact scorer runs; -1 = not scored). Carried so
    // multi-ply recovery (#22) can consume it without recomputation.
    // has_impact_score=false reverts selection to base-cost order.
    double impact_obstruction = -1.0;
    bool has_impact_score = false;
    JsonValue impact_detail;  // per-feature contributions + final score
};

// Issue #8: recomputed cost of final copper. Sum over traces of
// llround(euclid_len x layer_mult[layer]) + vias.size()*cfg.via_cost_nm +
// num_bends*cfg.bend_cost_nm, where num_bends counts direction changes
// between consecutively chained same-layer segments (8-way directions).
// Deterministic pure function; layer_mult out-of-range layers use 1.0.
Coord materialized_route_cost(const std::vector<TraceSeg>& traces,
                              const std::vector<Via>& vias,
                              const std::vector<double>& layer_mult,
                              const AStarConfig& cfg);

// Route one task against an immutable snapshot. Reads snapshot/resolver only;
// all scratch state is local, so any number of threads may call this
// concurrently on the same snapshot. Issue #10: hierarchical guidance runs
// per task through the shared (self-invalidating, thread-safe) cache; a null
// cache disables guidance for the call. Defaults keep direct unit-test and
// recovery call sites compiling.
// Issue #4: graph_budget overrides the sparse-graph caps (max bases /
// K nearest) for this task, e.g. from the maturity EffectiveSearchBudget
// (higher maturity -> larger budgets). Null = legacy defaults (384/16) for
// speed. On an unreachable miss the task always gets one genuinely
// expanded last-resort rebuild (uncapped bases, K=64) before it is
// declared unreachable, no matter the incoming budget.
CandidateRoute route_candidate_task(const Board& snapshot, const RuleResolver& resolver,
                                    const ConnectionTask& task, std::size_t task_index,
                                    double difficulty, const ElectricalContext& ctx,
                                    const std::vector<double>& layer_mult,
                                    const AStarConfig& astar_cfg, const CongestionMap& congestion,
                                    const ReservationSet& reservations,
                                    const HierarchyConfig& hier_cfg = HierarchyConfig{},
                                    const HierarchyCache* hier_cache = nullptr,
                                    const SparseGraphBudget* graph_budget = nullptr);

// ---- Conflict graph + deterministic central arbiter ----

// True when the two candidates' geometries would violate pair clearance.
bool candidates_conflict(const CandidateRoute& a, const CandidateRoute& b,
                         const RuleResolver& resolver, const ElectricalContext& ctx);

// True when the candidate's geometry is exactly legal against the given
// committed copper (bounds + width + pair clearance + via rules).
bool candidate_legal_vs_board(const CandidateRoute& cand, const Board& committed,
                              const RuleResolver& resolver, const ElectricalContext& ctx,
                              std::string& reason_out);

struct ArbiterResult {
    std::vector<std::size_t> accepted;  // indices into the candidate vector
    std::vector<std::size_t> rejected;
    std::vector<std::string> reject_reason;  // parallel to rejected
};

struct RouteStats {
    int nets_total = 0;
    int nets_routed = 0;
    int tasks_total = 0;
    int tasks_routed = 0;
    int via_count = 0;
    Coord length_nm = 0;
    std::int64_t expansions_total = 0;
    std::int64_t time_ms = 0;
    int threads_requested = 1;
    int threads_used = 1;
    int candidates_total = 0;
    int candidates_accepted = 0;
    int candidates_rejected = 0;
    int epochs_count = 0;
    // Issue #10: hierarchical-guidance aggregates over greedy-epoch
    // candidates (recovery branches report exact expansions only).
    int hierarchy_guided_tasks = 0;
    int hierarchy_fallback_tasks = 0;
    std::int64_t hierarchy_coarse_expansions = 0;
};

// Greedy deterministic selection in (difficulty desc, net, a, b) order:
// accept when legal vs committed+already-accepted and conflict-free vs
// already-accepted; otherwise reject with a machine-readable reason.
ArbiterResult arbitrate(const std::vector<CandidateRoute>& candidates, const Board& committed,
                        const RuleResolver& resolver, const ElectricalContext& ctx);

// Apply accepted candidates to the board as one atomic epoch commit.
void commit_candidates(Board& committed, const std::vector<CandidateRoute>& candidates,
                       const ArbiterResult& arb, RouteStats& stats);

// ---- Epoch plumbing ----

struct EpochInfo {
    int epoch = 0;
    int batch_size = 0;
    int candidates = 0;
    int accepted = 0;
    int rejected = 0;
    std::int64_t expansions = 0;
    int workers = 0;
    std::int64_t time_ms = 0;
    // Issue #3: interference-aware batch diagnostics (bounded: batch width
    // <= 8, so <= 28 pairs). Always emitted so --progress and the route
    // report carry selected batch IDs + pairwise scores + stats.
    std::vector<int> batch_task_ids;  // global task indices, selection order
    std::vector<std::pair<std::pair<int, int>, double>> interference_pairs;
    double interference_threshold_used = kDefaultInterferenceThreshold;
    int interference_relax_steps = 0;
    double interference_max = 0;
    double interference_mean = 0;
    int effective_batch_width = kParallelBatchSize;
    // Issue #14: maturity phase + effective budget snapshot for this epoch.
    // phase is one of OPEN_BOARD/MID_ROUTE/DENSE_ROUTE/CLOSURE; maturity and
    // budget are the to_json() objects of BoardMaturityState /
    // EffectiveSearchBudget (kept as JsonValue to avoid a parallel->maturity
    // include cycle; the engine fills them).
    std::string maturity_phase = "OPEN_BOARD";
    JsonValue maturity;
    JsonValue budget;
    JsonValue to_json() const;
};

// FNV-1a hash of committed trace/via geometry; stable across runs for equal
// (board, rules, seed, worker count). Agents use it for determinism checks.
std::string geometry_hash(const Board& board);

// Progress events for agents: {"event":"epoch",...} per epoch plus
// {"event":"done",...}. The engine invokes the callback synchronously from
// run(); the CLI wires it to stderr NDJSON under --progress.
using ProgressCallback = std::function<void(const JsonValue&)>;

}  // namespace copperline
