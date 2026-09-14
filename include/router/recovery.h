// Copperline: rip-up/reroute + chess-engine-style meta-search (Prompt 4).
//
// Greedy epoch A* can strand connections: an early route seals the only
// corridor a later net needed. This module provides the machinery that
// prevents such decisions from becoming permanent:
//
//   StallDetector        - epoch stall accounting (no accepted candidates).
//   FrontierDiag         - A* rejection/frontier diagnostics per failed task.
//   BlockerAttribution   - which committed copper blocks a failed task.
//   DependencyGraph      - failed-task -> blocker-net edges with weights.
//   Route protection     - escape stubs + stable difficult routes cost more
//                          to rip; fixed user copper is never ripped.
//   RipupMove generation - selective, protection-weighted, deterministic.
//   StateHash128         - 128-bit incremental/Zobrist-style board hash.
//   TranspositionTable   - prune revisited (hash, remaining) states.
//   HistoryHeuristic     - reward moves that historically reconnect.
//   Principal variation  - reuse the best branch's move first next gen.
//   Iterative deepening  - generations widen branch count + rip breadth +
//                          A* budgets; modes escalate FAST -> RECOVERY ->
//                          EXHAUSTIVE_LOCAL_RECOVERY.
//   Parallel speculative branches - evaluate candidate moves concurrently;
//                          selection stays deterministic (indexed slots +
//                          lexicographic objective), so --threads never
//                          changes committed copper.
//
// There is deliberately NO minimax / alpha-beta here: there is no adversary.
// The tree represents alternative routing decisions (which escape bundle owns
// a channel, which layer strategy, which routes to rip, what order to retry).
// A* owns detailed geometry; meta-search owns high-impact decisions.
// State comparison keeps strict connectivity priority: a shorter board with
// one unconnected pad NEVER beats a longer fully-connected legal board.
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "router/astar.h"
#include "router/board.h"
#include "router/json.h"
#include "router/parallel.h"
#include "router/route_tree.h"
#include "router/rules.h"

namespace copperline {

// ---- Recovery modes (escalating) ----

enum class RecoveryMode { FAST = 0, RECOVERY = 1, EXHAUSTIVE_LOCAL = 2 };

std::string recovery_mode_name(RecoveryMode m);
RecoveryMode recovery_mode_for_generation(int generation);

// A* budgets per mode. RECOVERY multiplies expansions, EXHAUSTIVE multiplies
// further and is the only mode allowed the largest rip neighborhoods.
// No unsafe probabilistic pruning in EXHAUSTIVE: budgets only grow.
AStarConfig astar_config_for_mode(const AStarConfig& base, RecoveryMode mode);

// How many speculative branches to evaluate in a generation (widening).
int branch_width_for_generation(int generation);
// How many owned routes a single move may rip (deepening).
int max_rip_breadth_for_generation(int generation);

// ---- Stall detection ----

struct StallDetector {
    int stalled_epochs = 0;
    void note_epoch(int accepted);
    bool stalled() const { return stalled_epochs > kMaxStalledEpochs; }
    void reset() { stalled_epochs = 0; }
};

// ---- A* rejection / frontier diagnostics ----

struct FrontierDiag {
    NetId net = -1;
    TermId a = -1;
    TermId b = -1;
    std::string fail_reason;  // "unreachable" | "budget_exhausted" | ...
    std::int64_t expansions = 0;
    int closest_node = -1;
    Coord closest_goal_dist_nm = 0;
    JsonValue to_json() const;
};

FrontierDiag diagnose_task(const ConnectionTask& task, const CandidateRoute& last);

// ---- Blocker attribution ----

struct BlockerHit {
    std::string desc;  // "trace:net=X" | "pad:net=X:U1.A3" | "keepout:..." | ...
    NetId net = -1;    // blocker net id, -1 for keepouts/frontier gaps
    std::string kind;  // "trace" | "pad" | "keepout" | "frontier"
    Coord area = 0;    // overlap area proxy for ordering (larger first)
};

std::vector<BlockerHit> attribute_blockers_detailed(const Board& board,
                                                     const ConnectionTask& task,
                                                     Coord width_nm,
                                                     const CandidateRoute* last);

// ---- Blocked-net dependency graph ----

struct DependencyEdge {
    int failed_pos = -1;  // index into the failed-task vector
    NetId failed_net = -1;
    NetId blocker_net = -1;  // -1 = keepout / frontier (no net to rip)
    std::string blocker_desc;
    double weight = 0;  // overlap-area proxy, deterministic
};

struct DependencyGraph {
    std::vector<ConnectionTask> failed;
    std::vector<DependencyEdge> edges;
    JsonValue to_json() const;
};

DependencyGraph build_dependency_graph(const Board& board, const RuleResolver& resolver,
                                        const ElectricalContext& ctx,
                                        const std::vector<ConnectionTask>& tasks,
                                        const std::vector<int>& remaining,
                                        const std::map<std::pair<NetId, std::pair<TermId, TermId>>,
                                                       CandidateRoute>& last_attempt);

// ---- Route protection ----

// Fixed user copper (present before routing started) is never ripped.
inline constexpr double kFixedProtection = 1e18;

double route_protection_score(bool is_escape_stub, double difficulty, int stable_epochs,
                              bool is_fixed, RecoveryMode mode);

// ---- Owned routes (which task owns which committed copper) ----

struct OwnedRoute {
    ConnectionTask task;
    int task_pos = -1;  // index into the global task vector
    std::vector<TraceSeg> traces;
    std::vector<Via> vias;
    int epoch_committed = 0;
    int stable_epochs = 0;
    bool is_escape_stub = false;
    double protection = 1.0;
};

void rebuild_board_from_owned(Board& board, const std::vector<TraceSeg>& fixed_traces,
                              const std::vector<Via>& fixed_vias,
                              const std::vector<OwnedRoute>& owned);

// ---- 128-bit incremental / Zobrist-style state hash ----

struct StateHash128 {
    std::uint64_t lo = 0;
    std::uint64_t hi = 0;
    bool operator==(const StateHash128& o) const { return lo == o.lo && hi == o.hi; }
    std::string to_hex() const;  // 32 hex chars, stable
};

// Hash committed copper (in order) plus the sorted remaining-task keys.
// Combine is xor-based (Zobrist-style): each element contributes an
// independent sub-hash so rip/commit updates are incremental in spirit.
StateHash128 state_hash128(const Board& board, const std::vector<ConnectionTask>& tasks,
                           const std::vector<int>& remaining);

// ---- Transposition table ----

// Single-threaded only: all should_prune/record calls must happen on the
// deterministic arbiter thread after speculative workers join, in move-index
// order. Never call from worker threads (unsynchronized unordered_map).
class TranspositionTable {
  public:
    // True when this (hash, remaining-count) was already reached with an
    // equal-or-better (smaller-or-equal remaining) state: prune the branch.
    bool should_prune(const StateHash128& h, int remaining) const;
    void record(const StateHash128& h, int remaining);
    int hits() const { return hits_; }
    std::size_t size() const { return table_.size(); }

  private:
    mutable int hits_ = 0;
    std::unordered_map<std::string, int> table_;
};

// ---- History heuristic ----

class HistoryHeuristic {
  public:
    void reward(const std::string& key, double amount = 1.0);
    double bonus(const std::string& key) const;
    static std::string move_key(const ConnectionTask& failed, NetId blocker_net);

  private:
    std::unordered_map<std::string, double> scores_;
};

// ---- Selective rip-up moves ----

struct RipupMove {
    int failed_pos = -1;  // index into the failed-task vector
    ConnectionTask failed_task;
    NetId blocker_net = -1;
    std::vector<int> owned_idx;  // indices into the owned-route vector
    std::string reason;
    double gain = 0;   // blocker overlap benefit
    double cost = 0;   // summed protection of ripped routes
    double score = 0;  // gain/(1+cost) + history bonus (+ PV bonus)
};

// Deterministic: sorted by (score desc, failed net/a/b, owned nets...).
// Skips keepout-only failures (nothing to rip) and caps the move count.
std::vector<RipupMove> generate_ripup_moves(
    const std::vector<ConnectionTask>& failed_tasks, const DependencyGraph& graph,
    const std::vector<OwnedRoute>& owned, const HistoryHeuristic& history,
    const std::string& pv_key, RecoveryMode mode, int max_moves, int max_breadth);

// ---- Branch evaluation (speculative, parallel-safe) ----

struct BranchResult {
    bool evaluated = false;
    bool pruned = false;
    // Local diagnostics: tasks routed inside this branch attempt (includes
    // reconnected ripped routes). NOT used for ranking (see issue #19):
    // reconnecting already-connected copper is work, not improvement.
    int connected_tasks = 0;  // tasks routed inside this branch attempt
    int total_tasks = 0;
    Coord length_nm = 0;  // global: over surviving + rerouted owned copper
    int via_count = 0;    // global: over surviving + rerouted owned copper
    StateHash128 hash;  // exact state hash over (copper, remaining_task_ids)
    std::string mode;
    RipupMove move;
    // Resulting copper when this branch wins (fixed + surviving + rerouted).
    Board board;
    std::vector<OwnedRoute> owned;
    std::vector<int> newly_done;  // task positions routed in this branch
    // Exact unfinished task positions after this branch
    // (sorted; (gen_remaining ∪ to_route) \ newly_done). State identity is
    // (hash, remaining_task_ids): two branches with identical copper but
    // different unfinished sets must NOT alias. Filled by reroute_branch;
    // consumed serially by the arbiter for transposition pruning.
    std::vector<int> remaining_task_ids;
    std::int64_t expansions = 0;
    // ---- Issue #19: explicit global outcome fields ----
    // Ranking must measure the resulting global board, not the amount of
    // work performed inside the branch. All fields below are global.
    int global_connected_tasks = 0;  // tasks.size() - remaining_task_ids.size()
    int hard_violations = 0;         // branches are built legal-only; nonzero if ever violated
    int resource_overuse = 0;        // unresolved resource overuse (0: none tracked yet)
    int newly_connected_global = 0;  // |newly_done ∩ gen_remaining|: previously-unrouted tasks
                                     // connected by this branch. Reconnecting an
                                     // already-connected ripped route does NOT count.
    int disrupted_routes = 0;        // ripped-route count (move.owned_idx.size())
};

// Lexicographic global objective (issue #19): fewer unconnected tasks
// (smaller remaining_task_ids) always wins; then fewer hard violations,
// then less overuse, then more newly-connected previously-unrouted tasks,
// then lower disruption (fewer ripped routes), then fewer vias, then shorter
// length, then stable hash order. Local connected_tasks is deliberately NOT
// compared: a branch that rips 3 and reconnects 3+failed improves the global
// board by the same single task as one that rips 1 and reconnects 1+failed,
// so the less disruptive branch must win.
// Returns true when `a` is strictly better than `b`.
bool branch_better(const BranchResult& a, const BranchResult& b);

// Reroute one speculative branch sequentially (deterministic, single-thread
// body; branches run concurrently in the driver via indexed slots).
// `surviving` + `fixed_*` form the base copper; `to_route` are task positions
// (failed + ripped) retried under `astar_cfg`. `gen_remaining` is the full
// unfinished set at generation start, used to derive the exact
// remaining_task_ids ((gen_remaining ∪ to_route) \ newly_done) and the exact
// state hash. Performs NO transposition-table access: the worker only returns
// the branch outcome + exact unfinished IDs; the arbiter hashes (already done
// here, purely) and prunes/records serially in move-index order after join.
// Order change after stalls is itself a meta-search decision: the failed
// task (`failed_ti`) goes first, then difficulty order with stable
// (net, a, b) tie-breaks.
BranchResult reroute_branch(const Board& base_template, const std::vector<TraceSeg>& fixed_traces,
                            const std::vector<Via>& fixed_vias,
                            const std::vector<OwnedRoute>& surviving,
                            const std::vector<int>& to_route, int failed_ti,
                            const std::vector<ConnectionTask>& tasks,
                            const std::vector<int>& gen_remaining,
                            const std::vector<Corridor>& corridors,
                            const RuleResolver& resolver, const ElectricalContext& ctx,
                            const std::vector<double>& layer_mult, const AStarConfig& astar_cfg,
                            const CongestionMap& congestion_tpl,
                            const std::string& mode_name, const RipupMove& move);

// ---- Recovery summary for route-report JSON ----

struct RecoveryInfo {
    int generations = 0;
    int branches_evaluated = 0;
    int branches_pruned = 0;
    int ripups = 0;
    int transposition_hits = 0;
    std::vector<std::string> modes_attempted;
    DependencyGraph last_graph;
    JsonValue to_json() const;
};

// Result-category strings for agents (stable, machine-readable).
// COMPLETE stays COMPLETE; VIOLATION maps to HARD_RULE_VIOLATION; budget
// exhaustion maps to SEARCH_BUDGET_EXHAUSTED_WITH_UNROUTED_CONNECTIONS;
// other incompletes map to UNROUTABLE_UNDER_CONFIGURED_CONSTRAINTS_AND_BUDGET.
std::string result_category(const std::string& status);

}  // namespace copperline
