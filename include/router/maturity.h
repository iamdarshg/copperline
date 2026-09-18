// Copperline: board-maturity adaptive search budgets (issue #14).
//
// Search effort adapts to board maturity. Early/open boards route cheaply;
// late copper-dense closure automatically spends more exploring alternatives,
// finer guidance, deeper rip-up and more meta-search.
//
//   BoardMaturityState - recomputed each epoch/recovery generation from
//                        committed copper occupancy, congestion hotspots,
//                        remaining-task difficulty, recent acceptance/stall
//                        rate, rip-up count, fine-pitch completion and
//                        remaining-task count.
//   MaturityPhase      - deterministic OPEN_BOARD/MID_ROUTE/DENSE_ROUTE/
//                        CLOSURE classification via configurable thresholds,
//                        with hysteresis: escalation is immediate (repeated
//                        stalls/closure escalate aggressively) while
//                        de-escalation requires measurable board-state
//                        improvement (remaining-task drop >= improvement_frac).
//   EffectiveSearchBudget - single object consumed by A*, hierarchy,
//                        portfolio/scheduler and recovery instead of scattered
//                        constants. Route-K (#8), portfolio and multi-ply
//                        beam (#22) fields are provisioned here so those
//                        features consume dynamic budgets when they land.
//                        Every field is clamped to explicit user caps and the
//                        router memory budget, so agents keep hard control of
//                        runtime/memory.
//
// Determinism: every mapping is a pure function of its inputs. No RNG, no
// thread dependence, integer-nm geometry in, stable JSON out.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "router/astar.h"
#include "router/board.h"
#include "router/hierarchy.h"
#include "router/json.h"

namespace copperline {

// ---- Maturity phases (deterministic, ordered cheap -> expensive) ----

enum class MaturityPhase {
    OPEN_BOARD = 0,
    MID_ROUTE = 1,
    DENSE_ROUTE = 2,
    CLOSURE = 3,
};

std::string maturity_phase_name(MaturityPhase p);
MaturityPhase maturity_phase_from_name(const std::string& name, bool& ok_out);

// ---- Configurable classification thresholds ----

struct MaturityThresholds {
    // Committed-copper occupancy fraction gates.
    double mid_occupancy = 0.10;
    double dense_occupancy = 0.30;
    double closure_occupancy = 0.55;
    // Remaining-task fraction gates (progress-driven cheap escalation).
    double mid_remaining_frac = 0.75;    // <= this -> at least MID_ROUTE
    double dense_remaining_frac = 0.35;  // <= this (+pressure) -> DENSE_ROUTE
    double closure_remaining_frac = 0.15;  // <= this (+pressure) -> CLOSURE
    // Stall / hotspot / recovery pressure gates.
    int mid_stalled_epochs = 1;
    int dense_stalled_epochs = 2;
    int closure_stalled_epochs = 4;
    int mid_hotspots = 2;
    int dense_hotspots = 6;
    int closure_hotspots = 8;
    // Hotspot score floor: only cells with a history-weighted contention
    // score (3 x history) at or above this count toward hotspot_count /
    // pressure. History accumulates only on rejections, failures and
    // recovery, so the default 3.0 counts cells where routing actually
    // fought (≈ one rejected route through the cell), not healthy
    // disjoint corridors (which paint present pressure only).
    double hotspot_score_floor = 3.0;
    int mid_recovery_gens = 1;
    int dense_recovery_gens = 2;
    int closure_recovery_gens = 4;
    int dense_ripups = 6;  // total ripped routes -> at least DENSE_ROUTE
    // Mean remaining-difficulty gates (difficulty units of compute_difficulty,
    // dominated by span_mm: a 16mm task scores ~30, so the MID gate sits
    // above ordinary single-task difficulty and only sustained hard/failing
    // remainders escalate). Both gates require routing to be underway
    // (progress, stalls, hotspots, recovery or imperfect acceptance): a
    // fresh untouched board stays cheap no matter the span.
    double mid_mean_difficulty = 35.0;
    double dense_mean_difficulty = 60.0;
    // Fine-pitch completion below this (+any remaining work) -> at least MID.
    double fine_pitch_mid_frac = 0.8;
};

// ---- Maturity inputs: one snapshot per epoch / recovery generation ----

struct MaturityInput {
    double occupancy_frac = 0.0;       // committed copper / board area
    int hotspot_count = 0;             // congestion hotspots (top-K, score > 0)
    double hotspot_pressure = 0.0;     // summed present+history over hotspots
    double mean_remaining_difficulty = 0.0;
    double acceptance_rate = 1.0;      // recent accepted/candidates (0..1)
    int stalled_epochs = 0;            // consecutive zero-accept epochs
    int ripup_count = 0;               // total ripped routes so far
    int recovery_generations = 0;      // recovery generations completed
    double fine_pitch_done_frac = 1.0;  // escaped/total pads (1 when none)
    int remaining_count = 0;
    int total_tasks = 1;
    double remaining_frac = 1.0;
};

// ---- Classified maturity state (with de-escalation hysteresis) ----

struct BoardMaturityState {
    MaturityPhase phase = MaturityPhase::OPEN_BOARD;
    MaturityInput metrics;
    // Hysteresis bookkeeping: remaining count when the current phase was
    // entered; a step-down requires remaining <= entry * (1-improvement).
    int phase_entry_remaining = 0;
    bool escalated_by_stall = false;
    JsonValue to_json() const;
};

// Raw (hysteresis-free) classification. Pure function of input+thresholds.
MaturityPhase classify_maturity_raw(const MaturityInput& in,
                                     const MaturityThresholds& th);

// Hysteretic update: escalation applies immediately; a lower raw phase only
// takes effect after measurable improvement (remaining-task drop of at
// least improvement_frac since phase entry). A null prev starts fresh.
// improvement_frac is clamped to [0, 0.9].
BoardMaturityState update_maturity(const MaturityInput& in,
                                    const MaturityThresholds& th,
                                    const BoardMaturityState* prev,
                                    double improvement_frac = 0.10);

// Committed-copper occupancy fraction in [0,1]: integer-nm trace
// (width x Manhattan length) + via (outer-diameter squared) areas over the
// board area. Deterministic; 0 for degenerate boards.
double copper_occupancy_frac(const Board& board);

// Board-scale backlog threshold: at or above this many connection tasks the
// coarse-to-fine guidance grid is itself congested, so its per-task search
// costs far more than the exact search it steers, and graph construction is
// the dominant cost (measured ~2900s of edge construction vs ~8s of A* on a
// 2458-task board). Both board-scale policies are LATCHED from the INITIAL
// backlog for the whole run (see effective_budget_for_phase): keying on the
// draining backlog made them flip back off exactly as the board densified.
// Small boards keep the legacy behaviour exactly.
inline constexpr int kGuidanceDisableRemaining = 2048;

// Default board-scale graph clamp target: the legacy OPEN/MID floor (384/16),
// so a board-scale run keeps the well-tested small graph instead of escalating
// to DENSE 1024/32 and CLOSURE 2048/64 -- the escalation is what dominates the
// build cost at this scale. The clamp only ever LOWERS the phase budget, and
// any miss still gets the last-resort rebuild before "unreachable", so this
// trades quality, never correctness. Tunable via MaturityCaps::board_scale_*.
//
// Chosen from a sweep on the 2458-task ESC (14 threads, 2 GB memory budget):
// 96/4 routed clearly worst; 384/16 matched the best observed count;
// 1024/32 and 2048/64 were worse (throughput lost faster than quality gained).
// Fine differences across 256..1024 sit inside run-to-run timing noise (the
// wall-clock deadline makes A* cuts timing-dependent), so the legacy floor is
// preferred over over-fitting a noisy surface.
inline constexpr std::size_t kBoardScaleGraphBases = 384;
inline constexpr int kBoardScaleGraphK = 16;

// ---- Explicit agent caps (floors/ceilings always win over phase maps) ----

struct MaturityCaps {
    // 0 = auto (base * 16). Otherwise an absolute ceiling on A* expansions.
    std::int64_t max_astar_expansions = 0;
    // Ceiling for the backlog-adaptive epoch batch width (see
    // effective_budget_for_phase). Small boards stay at the legacy width 8;
    // large boards widen up to this cap, subject to the memory bound.
    int max_batch_width = 64;
    int max_route_k = 15;  // issue #8 hook: 10-15 diverse alternatives
    int max_recovery_branches = 8;
    int max_rip_breadth = 4;
    int max_recovery_depth = 10;  // issue #22 hook: multi-ply depth allowance
    int max_beam = 8;             // issue #22 hook: multi-ply beam
    int max_hierarchy_window_attempts = 4;
    // 0 = keep the configured HierarchyConfig limit.
    std::size_t max_hierarchy_grid_cells = 0;
    // Weighted-A* factor ceiling (floor is always 1.0 = admissible).
    double max_weight_factor = 2.0;
    // Issue #4: sparse-graph budget ceilings. 0 = auto (phase map wins,
    // so CLOSURE may go uncapped for max_bases / 64 for k_nearest).
    // Otherwise an absolute ceiling: the phase want is clamped down to it
    // (an uncapped want of 0 resolves to the cap value).
    std::size_t max_graph_bases = 0;
    int max_graph_k_nearest = 0;
    // Board-scale graph clamp target (tuning / A-B sweep). The clamp lowers the
    // phase budget to at most these; shipped defaults are kBoardScaleGraph*.
    std::size_t board_scale_graph_bases = kBoardScaleGraphBases;
    int board_scale_graph_k = kBoardScaleGraphK;
    // Escape hatch / A-B testing: force the legacy (non-latched) board-scale
    // policy OFF, so guidance and the graph budget follow the phase map even
    // when the initial backlog is board-scale. Default false = the latched
    // board-scale policy applies.
    bool disable_board_scale_policy = false;
};

// ---- Effective search budget: the single consumed object ----

struct EffectiveSearchBudget {
    std::int64_t astar_max_expansions = 200000;
    // Coarse-to-fine guidance for this phase (false once the backlog is
    // board-scale; see kGuidanceDisableRemaining).
    bool hier_enabled = true;
    double weight_factor = 1.0;  // weighted-A* heuristic scale (1 = exact)
    std::int64_t hier_max_coarse_expansions = 200000;
    int hier_window_attempts = 2;
    Coord hier_tube_half_nm = 5000000;
    std::size_t hier_max_grid_cells = 8000000;
    int route_k = 1;  // issue #8 hook (provisioned; single-candidate today)
    double reservation_strength = 1.0;
    int batch_width = 8;
    double history_growth = 1.0;
    int ripup_breadth = 1;
    int recovery_branches = 2;
    int recovery_depth = 4;  // issue #22 hook (advisory; user cap wins)
    int recovery_beam = 1;   // issue #22 hook (provisioned)
    double timeout_share_per_task_s = 0.0;  // <=0 = no timeout configured
    int threads_effective = 1;
    // Issue #4: sparse-graph budgets for this phase. max_bases 0 = uncapped
    // (last-resort completeness); k_nearest <= 0 = try every same-layer
    // node. OPEN/MID keep the legacy 384/16 defaults for speed.
    std::size_t graph_max_bases = 384;
    int graph_k_nearest = 16;
    JsonValue to_json() const;
};

// Phase -> bounded params, clamped to caps + memory budget. Pure function.
// threads_effective is passed through (resolved by the caller via
// resolve_worker_threads: 0/auto -> hardware_concurrency, explicit wins).
// timeout_remaining_s < 0 means no deadline.
// `board_scale` is a RUN-LEVEL judgment that this board's backlog is
// board-scale (see kGuidanceDisableRemaining): the caller must LATCH it once,
// from the initial backlog, and pass the same value for every epoch. Keying
// the policy on the draining `remaining_count` instead makes it flip back OFF
// as work succeeds -- exactly when the board is densest and per-task cost is
// highest (measured: guidance re-enabled for 4 CPU-hours on a board-scale
// run, and graph sizes returned to full 2048/64). `remaining_count` is still
// the input for backlog-adaptive batch width, which SHOULD drain.
EffectiveSearchBudget effective_budget_for_phase(
    const BoardMaturityState& state, const AStarConfig& base_astar,
    const HierarchyConfig& base_hier, const MaturityCaps& caps,
    std::size_t memory_budget_bytes, std::size_t per_task_bytes,
    int threads_effective, double timeout_remaining_s, int remaining_count,
    bool board_scale = false);

// ---- Engine-facing options ----

struct MaturityOptions {
    MaturityThresholds thresholds;
    MaturityCaps caps;
    bool enabled = true;  // false = legacy fixed budgets (always OPEN_BOARD)
    double improvement_frac = 0.10;  // de-escalation needs this remaining drop
};

// Apply an optional parsed "maturity" config object (from --config JSON).
// Unknown keys are ignored for forward compatibility. Returns false + err on
// out-of-range values (negative numbers, NaN, empty-impossible ranges).
bool apply_maturity_json(MaturityOptions& out, const JsonValue& node,
                          std::string& err_out);

// ---- Maturity log entries for route reports / NDJSON ----

struct MaturityLogEntry {
    int epoch = 0;  // greedy epoch index, or recovery log index
    bool is_recovery = false;
    int generation = -1;  // recovery generation, -1 for greedy epochs
    BoardMaturityState state;
    EffectiveSearchBudget budget;
    JsonValue to_json() const;
};

}  // namespace copperline
