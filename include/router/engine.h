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
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "router/astar.h"
#include "router/board.h"
#include "router/json.h"
#include "router/parallel.h"
#include "router/recovery.h"
#include "router/rules.h"

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
    int ripup_attempts = 0;
    std::vector<std::string> modes_attempted;
    FrontierDiag frontier;
    bool has_frontier = false;
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
    JsonValue to_json() const;
};

struct EngineOptions {
    AStarConfig astar;
    unsigned seed = 42;
    int threads = 1;
    double timeout_s = 0;  // 0 = none
    int max_epochs = 4096;
    ProgressCallback progress;  // optional per-epoch NDJSON events
    bool enable_ripup = true;
    int max_ripup_generations = 6;
    int max_ripup_branches = 8;
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

}  // namespace copperline
