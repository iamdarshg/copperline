// Copperline: single-threaded routing engine (phase 1).
//
// Routes every RouteTree connection task sequentially in deterministic
// difficulty order against the live committed board. Each task builds a fresh
// sparse graph (phase-3 will add epochs/batches); committed copper is the
// only shared state and only this thread mutates it.
#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "router/astar.h"
#include "router/board.h"
#include "router/json.h"
#include "router/rules.h"

namespace copperline {

struct RouteFailure {
    NetId net = -1;
    std::string net_name;
    TermId a = -1;
    TermId b = -1;
    std::string reason;  // "unreachable" | "budget_exhausted" | "timeout" | "no_via" | ...
    std::vector<std::string> blockers;
    std::int64_t expansions = 0;
    double required_width_mm = 0;
    std::string width_source;
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
    int threads_used = 1;  // phase 1 is single-threaded by design
};

struct RouteReport {
    // COMPLETE | INCOMPLETE | BUDGET_EXHAUSTED | TIMEOUT
    std::string status = "INCOMPLETE";
    int connected_terminals = 0;
    int total_terminals = 0;
    RouteStats stats;
    std::vector<RouteFailure> failures;
    JsonValue to_json() const;
};

struct EngineOptions {
    AStarConfig astar;
    unsigned seed = 42;
    int threads = 1;
    double timeout_s = 0;  // 0 = none
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
