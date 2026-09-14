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
#include "router/json.h"
#include "router/route_tree.h"
#include "router/rules.h"
#include "router/sparse_graph.h"

namespace copperline {

// Fixed batch width. Batch membership is a pure function of the deterministic
// scheduler order and therefore independent of --threads, which only controls
// how many candidates are computed concurrently. This is what keeps the
// committed geometry identical at 1, 2, 4, ... workers.
inline constexpr int kParallelBatchSize = 8;
// Epochs with zero accepted candidates tolerated before giving up.
inline constexpr int kMaxStalledEpochs = 4;

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
};

Corridor probable_corridor(const Board& board, const RuleResolver& resolver,
                           const ConnectionTask& task, const ElectricalContext& ctx);

// Symmetric pairwise interference weights, parallel to the task vector.
std::vector<std::vector<double>> build_interference(const std::vector<ConnectionTask>& tasks,
                                                    const std::vector<Corridor>& corridors);

// Deterministic batch scheduler: stable order is (difficulty desc, net, a,
// b); batches are consecutive slices of that order with fixed width.
std::vector<std::vector<int>> schedule_batches(const std::vector<ConnectionTask>& tasks,
                                                int batch_size = kParallelBatchSize);

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

  private:
    std::vector<Corridor> corridors_;
    std::vector<double> weights_;
};

// ---- Candidates (worker outputs; committed state is never touched) ----

struct CandidateRoute {
    ConnectionTask task;
    std::size_t task_index = 0;  // index into the epoch task vector
    double difficulty = 0;
    bool found = false;
    std::string fail_reason;  // "unreachable" | "budget_exhausted" | ...
    std::int64_t expansions = 0;
    Coord cost_nm = 0;
    std::vector<TraceSeg> traces;
    std::vector<Via> vias;
    Point gate_a{};  // candidate endpoints (== task terminals on success)
    Point gate_b{};
    int closest_node = -1;
    Coord closest_goal_dist_nm = 0;
};

// Route one task against an immutable snapshot. Reads snapshot/resolver only;
// all scratch state is local, so any number of threads may call this
// concurrently on the same snapshot.
CandidateRoute route_candidate_task(const Board& snapshot, const RuleResolver& resolver,
                                    const ConnectionTask& task, std::size_t task_index,
                                    double difficulty, const ElectricalContext& ctx,
                                    const std::vector<double>& layer_mult,
                                    const AStarConfig& astar_cfg, const CongestionMap& congestion,
                                    const ReservationSet& reservations);

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
