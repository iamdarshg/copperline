// Copperline: parallel global routing engine (Prompt 3 + Prompt 4 recovery).
//
// Epoch pipeline: connection tasks from RouteTrees are scored with difficulty
// vectors, ordered by the deterministic batch scheduler, and routed in fixed
// batches. A worker pool generates candidates against an immutable committed
// snapshot (workers never mutate copper); the deterministic central arbiter
// revalidates, builds conflicts, selects a compatible subset, and commits it
// atomically as one epoch. Pathfinder-style present/history congestion plus
// soft reservations bias planning pressure only — never final DRC legality.
//
// Prompt 4: when greedy epochs stall with unrouted tasks, the engine runs
// rip-up/reroute generations driven by blocker attribution, a blocked-net
// dependency graph, protection-weighted selective rip-up sets, 128-bit state
// hashing, a transposition table, history heuristic, PV reuse, iterative
// deepening/widening and parallel speculative branches across escalating
// FAST -> RECOVERY -> EXHAUSTIVE_LOCAL_RECOVERY modes. Connectivity keeps
// strict priority over length/vias/congestion in every comparison.
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "router/astar.h"
#include "router/board.h"
#include "router/diffpair.h"
#include "router/hierarchy.h"
#include "router/impact.h"
#include "router/json.h"
#include "router/maturity.h"
#include "router/optimizer.h"
#include "router/parallel.h"
#include "router/recovery.h"
#include "router/rules.h"
#include "router/tuning.h"
#include "router/verifier.h"

namespace copperline {

struct RouteFailure {
    NetId net = -1;
    std::string net_name;
    TermId a = -1;
    TermId b = -1;
    std::string reason;  // "unreachable" | "budget_exhausted" | "timeout" |
                         // "conflict" | "illegal_overlap:..." | "unattempted" |
                         // "no_via" | "bad_task" | ...
    std::vector<std::string> blockers;
    std::int64_t expansions = 0;
    double required_width_mm = 0;
    std::string width_source;
    // Ampacity accounting (issue #7, mirrors analyze nets[]).
    std::string width_model;
    double copper_weight_oz = 1.0;
    double temp_rise_c = 20.0;
    // Parallel-via diagnostics (issue #5, mirrors CandidateRoute).
    double required_current_a = 0.0;
    std::string via_style;
    int vias_required = 1;
    std::string via_reason = "ok";
    int ripup_attempts = 0;
    std::vector<std::string> modes_attempted;
    FrontierDiag frontier;
    bool has_frontier = false;
    // Prompt 5 agent-stable failure attribution (stable IDs for iteration):
    // source/target component+pin, centre depth (fine-pitch, -1 when n/a),
    // endpoint pin density, failed candidate count, best partial state and
    // the per-connection reason category.
    std::string src_component;
    std::string src_pin;
    std::string dst_component;
    std::string dst_pin;
    int centre_depth = -1;
    double pin_density = 0.0;
    int candidate_count = 0;
    std::string best_partial;
    // SEARCH_BUDGET_EXHAUSTED_WITH_UNROUTED_CONNECTIONS |
    // UNROUTABLE_UNDER_CONFIGURED_CONSTRAINTS_AND_BUDGET
    std::string category = "UNROUTABLE_UNDER_CONFIGURED_CONSTRAINTS_AND_BUDGET";
    // Prompt 5: attempted layers / via classes for this connection.
    std::vector<LayerId> attempted_layers;
    std::vector<std::string> attempted_via_classes;
    // Issue #11: controlled-impedance accounting for this task's net.
    bool has_impedance = false;
    double target_impedance_ohms = 0.0;
    double impedance_tolerance_pct = 0.0;
    LayerId impedance_layer = -1;
    double impedance_width_mm = 0.0;
    std::string impedance_model;
    double estimated_impedance_ohms = 0.0;
    double impedance_error_pct = 0.0;
    bool impedance_conflict = false;
    std::string impedance_detail;
    // Issue #16: plane target this failure belongs to (when the task is a
    // plane-access task). Ordinary tasks carry plane_id == -1.
    bool has_plane_target = false;
    int plane_id = -1;
    LayerId plane_layer = 0;
    int plane_island = 0;
    // Issue #12: pair-corridor attribution. Corridor failures name the pair
    // and both members so agents can tell a coupled-resource failure from
    // an individual-net failure.
    bool is_pair_corridor = false;
    int pair_id = -1;
    NetId pair_other_net = -1;
    std::string pair_name;
    // Issue #10: hierarchical-guidance diagnostics for this task's last
    // attempt (levels used, fallback status, coarse expansions). Exact
    // expansions are already in `expansions`.
    std::vector<double> hierarchy_levels_mm;
    bool hierarchy_fallback = false;
    std::string hierarchy_reason = "none";
    std::int64_t hierarchy_coarse_expansions = 0;
    int hierarchy_window_attempts = 0;
};

// Issue #16: per-terminal plane-access record. Reports which plane, layer
// and island a power terminal entered, the entry geometry, the via bundle
// used and the current-capacity margin of that entry.
struct PlaneAccessInfo {
    NetId net = -1;
    std::string net_name;
    TermId terminal = -1;
    int plane_id = -1;
    LayerId plane_layer = 0;
    int island = 0;
    Point entry{};
    int via_count = 0;
    double required_current_a = 0.0;
    double via_capacity_a = 0.0;
    double current_margin_a = 0.0;
    std::string via_style;
    JsonValue to_json() const;
};

struct EscapeStageInfo {
    // Post-P4 issue #1: centre-out escape stage summary. Runs before global
    // epochs using the same real EscapePlanner as `router escape`.
    int pads_total = 0;
    int pads_escaped = 0;
    int pads_infeasible = 0;
    std::vector<TermId> escaped_terminals;    // sorted, deterministic
    std::vector<TermId> unresolved_terminals;  // escape-infeasible pads, sorted
    // Per-terminal infeasibility reasons for unresolved pads, parallel to
    // unresolved_terminals.
    std::vector<std::string> unresolved_reasons;
    JsonValue to_json() const;
};

struct RouteReport {
    // COMPLETE | INCOMPLETE | BUDGET_EXHAUSTED | TIMEOUT
    std::string status = "INCOMPLETE";
    // Agent-stable category: COMPLETE |
    // SEARCH_BUDGET_EXHAUSTED_WITH_UNROUTED_CONNECTIONS |
    // UNROUTABLE_UNDER_CONFIGURED_CONSTRAINTS_AND_BUDGET | TIMEOUT
    std::string result_category = "UNROUTABLE_UNDER_CONFIGURED_CONSTRAINTS_AND_BUDGET";
    int connected_terminals = 0;
    int total_terminals = 0;
    RouteStats stats;
    std::vector<RouteFailure> failures;
    std::vector<EpochInfo> epochs;
    std::vector<Hotspot> hotspots;
    std::string board_hash;  // geometry_hash() of committed copper
    StateHash128 state_hash{};
    RecoveryInfo recovery;
    EscapeStageInfo escape_stage;  // Post-P4 issue #1: escape summary
    std::vector<PlaneAccessInfo> plane_access;  // issue #16: plane entries
    std::vector<ImpedanceResolution> impedance;  // issue #11: per-net Z report
    std::vector<PairReport> diffpairs;  // issue #12: corridor + materialization
    TuningSummary tuning;  // issue #15: post-route length/skew tuning stage
    OptimizerReport optimizer;  // Prompt 5: transactional cleanup (COMPLETE only)
    VerifyResult verification;  // independent BoardVerifier gate on final copper
    // Issue #14: maturity/budget schedule over the run (one entry per greedy
    // epoch plus one per recovery generation) + the final effective state.
    // Agents observe phase transitions and the exact hyperparams consumed.
    std::vector<MaturityLogEntry> maturity_log;
    BoardMaturityState maturity;
    EffectiveSearchBudget budget;
    bool has_budget = false;
    JsonValue to_json() const;
};

struct EngineOptions {
    AStarConfig astar;
    HierarchyConfig hierarchy;  // issue #10: coarse-to-fine guidance (on by default)
    unsigned seed = 42;
    int threads = 0;  // issue #3: 0 = auto (all CPUs via hardware_concurrency)
    double timeout_s = 0;  // 0 = none
    int max_epochs = 4096;
    ProgressCallback progress;  // optional per-epoch NDJSON events
    bool enable_ripup = true;
    int max_ripup_generations = 6;
    int max_ripup_branches = 8;
    // Issue #22: bounded multi-ply high-level recovery search. Depth 1
    // reproduces one-ply exactly; defaults stay small (depth 2, beam 4).
    // Values are clamped per generation to maturity allowance + explicit
    // caps (maturity.caps.max_recovery_depth/max_beam) + memory budget.
    // 0 = auto (defaults). max_multiply_nodes bounds deeper branch
    // evaluations per generation; exhaustion falls back to one-ply.
    int recovery_depth = 2;
    int recovery_beam = 4;
    std::int64_t max_multiply_nodes = 64;
    // Issue #3: interference-aware batch scheduling (greedy + relaxation).
    double batch_interference_threshold = kDefaultInterferenceThreshold;
    double batch_relax_factor = kInterferenceRelaxFactor;
    int batch_max_relax_steps = kBatchMaxRelaxSteps;
    // Router memory bound (default 2048MB dev cap). Batch width is
    // min(kParallelBatchSize, floor(budget/per_task)); interference stays
    // sparse/streamed (no dense N^2 doubles).
    std::size_t memory_budget_bytes = kRouterMemoryBudgetBytes;
    std::size_t per_task_bytes = kPerCandidateBytes;
    // Issue #14: board-maturity adaptive search budgets (on by default).
    // Disabled -> legacy fixed budgets (always OPEN_BOARD params).
    MaturityOptions maturity;
    // Issue #9: future-obstruction scoring of portfolio candidates (on by
    // default, heuristic weighted scorer; --no-impact disables to base-cost
    // order, --impact-mlp selects the tiny fixed-weight MLP with fallback).
    ImpactOptions impact;
    // Issue #15: post-route length/skew tuning (on by default; runs only
    // after global closure + pair materialization, only when targets
    // exist). --no-tuning or tuning.enabled=false disables the stage.
    TuningConfig tuning;
    // Prompt 5: transactional cleanup optimizer (on by default; runs only
    // after COMPLETE + independent verifier pass). --no-optimizer disables.
    OptimizerOptions optimizer;
    // Measurement: emit per-stage wall ms as one stderr JSON line
    // ({"event":"stage_timings",...}) at the end of run(). Default off:
    // zero behavior change (one clock read per stage boundary only).
    bool time_stages = false;
};

class RouterEngine {
  public:
    RouterEngine(Board board, RuleResolver resolver, EngineOptions options);

    RouteReport run();
    const Board& committed() const { return board_; }

  private:
    Board board_;
    RuleResolver resolver_;
    EngineOptions options_;
};

// Post-P4 issue #2: gate COMPLETE on BoardVerifier. Downgrades a
// bookkeeping-COMPLETE report when independent verification of the committed
// copper disagrees, and escalates any illegal final board to VIOLATION so the
// CLI maps it to the hard-rule-violation exit code. Never upgrades a report.
void apply_verifier_gate(RouteReport& report, const VerifyResult& vr);

}  // namespace copperline
