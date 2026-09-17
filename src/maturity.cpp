// Copperline: board-maturity adaptive search budgets (issue #14).
#include "router/maturity.h"

#include <algorithm>
#include <cmath>

#include "router/sparse_graph.h"

namespace copperline {

std::string maturity_phase_name(MaturityPhase p) {
    switch (p) {
        case MaturityPhase::OPEN_BOARD: return "OPEN_BOARD";
        case MaturityPhase::MID_ROUTE: return "MID_ROUTE";
        case MaturityPhase::DENSE_ROUTE: return "DENSE_ROUTE";
        case MaturityPhase::CLOSURE: return "CLOSURE";
    }
    return "OPEN_BOARD";
}

MaturityPhase maturity_phase_from_name(const std::string& name, bool& ok_out) {
    ok_out = true;
    if (name == "OPEN_BOARD") return MaturityPhase::OPEN_BOARD;
    if (name == "MID_ROUTE") return MaturityPhase::MID_ROUTE;
    if (name == "DENSE_ROUTE") return MaturityPhase::DENSE_ROUTE;
    if (name == "CLOSURE") return MaturityPhase::CLOSURE;
    ok_out = false;
    return MaturityPhase::OPEN_BOARD;
}

double copper_occupancy_frac(const Board& board) {
    long double area = static_cast<long double>(board.width_nm) * board.height_nm;
    if (!(area > 0)) return 0.0;
    __int128 copper = 0;
    for (const auto& t : board.traces) {
        Coord len = manhattan(t.a, t.b);
        copper += (__int128)t.width_nm * len;
    }
    for (const auto& v : board.vias) copper += (__int128)v.outer_d_nm * v.outer_d_nm;
    long double frac = static_cast<long double>(copper) / area;
    if (!(frac >= 0)) return 0.0;
    if (frac > 1.0) frac = 1.0;
    return static_cast<double>(frac);
}

MaturityPhase classify_maturity_raw(const MaturityInput& in,
                                     const MaturityThresholds& th) {
    // Highest phase first; every comparison is deterministic.
    //
    // Difficulty gates only fire once routing is underway (progress made,
    // stalls, hotspots, recovery or imperfect acceptance): a fresh board
    // with long spans is still OPEN — early/open boards route cheaply, and
    // difficulty describes the *remainder*, not the untouched board.
    bool underway = in.remaining_frac < 1.0 || in.stalled_epochs > 0 ||
                    in.hotspot_count > 0 || in.recovery_generations > 0 ||
                    in.ripup_count > 0 || in.acceptance_rate < 1.0;
    bool dense_pressure = in.stalled_epochs >= th.dense_stalled_epochs ||
                          in.hotspot_count >= th.dense_hotspots ||
                          in.recovery_generations >= th.dense_recovery_gens ||
                          in.ripup_count >= th.dense_ripups ||
                          (underway &&
                           in.mean_remaining_difficulty >= th.dense_mean_difficulty);
    bool closure_pressure = in.stalled_epochs >= th.closure_stalled_epochs ||
                            in.hotspot_count >= th.closure_hotspots ||
                            in.recovery_generations >= th.closure_recovery_gens;
    if (in.occupancy_frac >= th.closure_occupancy || closure_pressure ||
        (in.remaining_count > 0 && in.remaining_frac <= th.closure_remaining_frac &&
         (in.stalled_epochs >= th.dense_stalled_epochs ||
          in.hotspot_count >= th.dense_hotspots))) {
        return MaturityPhase::CLOSURE;
    }
    if (in.occupancy_frac >= th.dense_occupancy || dense_pressure ||
        (in.remaining_count > 0 && in.remaining_frac <= th.dense_remaining_frac &&
         (in.stalled_epochs >= th.mid_stalled_epochs ||
          in.hotspot_count >= th.mid_hotspots))) {
        return MaturityPhase::DENSE_ROUTE;
    }
    bool mid_pressure = in.stalled_epochs >= th.mid_stalled_epochs ||
                        in.hotspot_count >= th.mid_hotspots ||
                        in.recovery_generations >= th.mid_recovery_gens ||
                        (underway &&
                         in.mean_remaining_difficulty >= th.mid_mean_difficulty) ||
                        (in.fine_pitch_done_frac < th.fine_pitch_mid_frac &&
                         in.remaining_count > 0);
    if (in.occupancy_frac >= th.mid_occupancy || mid_pressure ||
        (in.remaining_count > 0 && in.remaining_frac <= th.mid_remaining_frac)) {
        return MaturityPhase::MID_ROUTE;
    }
    return MaturityPhase::OPEN_BOARD;
}

BoardMaturityState update_maturity(const MaturityInput& in,
                                    const MaturityThresholds& th,
                                    const BoardMaturityState* prev,
                                    double improvement_frac) {
    MaturityPhase raw = classify_maturity_raw(in, th);
    BoardMaturityState out;
    out.metrics = in;
    if (improvement_frac < 0) improvement_frac = 0;
    if (improvement_frac > 0.9) improvement_frac = 0.9;
    if (!prev) {
        out.phase = raw;
        out.phase_entry_remaining = in.remaining_count;
        out.escalated_by_stall = in.stalled_epochs >= th.mid_stalled_epochs;
        return out;
    }
    if (raw > prev->phase) {
        // Escalation is immediate (repeated stalls/closure escalate fast).
        out.phase = raw;
        out.phase_entry_remaining = in.remaining_count;
        out.escalated_by_stall = in.stalled_epochs >= th.mid_stalled_epochs;
        return out;
    }
    if (raw == prev->phase) {
        out.phase = prev->phase;
        out.phase_entry_remaining = prev->phase_entry_remaining;
        out.escalated_by_stall = in.stalled_epochs >= th.mid_stalled_epochs;
        return out;
    }
    // raw < prev->phase: de-escalate only after measurable improvement, i.e.
    // the remaining-task count dropped by improvement_frac since phase entry.
    // A fresh board (remaining grew via tree regen) never de-escalates.
    double bar = prev->phase_entry_remaining * (1.0 - improvement_frac);
    if (in.remaining_count <= bar) {
        out.phase = raw;
        out.phase_entry_remaining = in.remaining_count;
    } else {
        out.phase = prev->phase;
        out.phase_entry_remaining = prev->phase_entry_remaining;
    }
    out.escalated_by_stall = in.stalled_epochs >= th.mid_stalled_epochs;
    return out;
}

namespace {

// Memory bounds shared by K/beam/grid params: total router memory stays
// within the 2048MB-style budget. With default budgets the bounds sit above
// every cap, so easy-board behavior is bit-identical to legacy constants.
int bound_by_mem(int want, std::size_t budget_bytes, std::size_t per_unit_bytes,
                  int floor_v = 1) {
    if (want < floor_v) want = floor_v;
    if (budget_bytes == 0 || per_unit_bytes == 0) return want;
    std::size_t bound = budget_bytes / per_unit_bytes;
    if (bound < static_cast<std::size_t>(floor_v)) bound = floor_v;
    if (static_cast<std::size_t>(want) > bound) want = static_cast<int>(bound);
    return want;
}

}  // namespace

EffectiveSearchBudget effective_budget_for_phase(
    const BoardMaturityState& state, const AStarConfig& base_astar,
    const HierarchyConfig& base_hier, const MaturityCaps& caps,
    std::size_t memory_budget_bytes, std::size_t per_task_bytes,
    int threads_effective, double timeout_remaining_s, int remaining_count) {
    EffectiveSearchBudget b;
    int lvl = static_cast<int>(state.phase);  // 0..3, deterministic

    // Expansion multipliers: 1/2/4/8x. MID stays geometry-identical to OPEN
    // whenever the base budget sufficed (A* is optimal; extra headroom only
    // matters on exhaustion). Clamped to the explicit agent ceiling.
    static const std::int64_t kExpMult[4] = {1, 2, 4, 8};
    __int128 want_exp = (__int128)base_astar.max_expansions * kExpMult[lvl];
    std::int64_t ceil_exp =
        caps.max_astar_expansions > 0 ? caps.max_astar_expansions
                                      : base_astar.max_expansions * 16;
    if (ceil_exp < base_astar.max_expansions) ceil_exp = base_astar.max_expansions;
    if (want_exp > ceil_exp) want_exp = ceil_exp;
    if (want_exp < base_astar.max_expansions) want_exp = base_astar.max_expansions;
    b.astar_max_expansions = static_cast<std::int64_t>(want_exp);

    // Weighted-A* factor: OPEN/MID stay admissible (1.0); dense phases trade
    // optimality for speed. Ordering-only: legality is structural.
    static const double kWeight[4] = {1.0, 1.0, 1.25, 1.5};
    double wmax = caps.max_weight_factor;
    if (!(wmax >= 1.0)) wmax = 1.0;
    b.weight_factor = std::min(kWeight[lvl], wmax);
    if (!(b.weight_factor >= 1.0)) b.weight_factor = 1.0;

    // Hierarchy: coarser levels get the same headroom scaling; dense phases
    // retry windows more often inside a wider handoff tube (finer effective
    // guidance). Grid cells stay memory-bounded (streamed levels).
    __int128 want_coarse = (__int128)base_hier.max_coarse_expansions * kExpMult[lvl];
    if (want_coarse > ceil_exp) want_coarse = ceil_exp;
    if (want_coarse < base_hier.max_coarse_expansions)
        want_coarse = base_hier.max_coarse_expansions;
    b.hier_max_coarse_expansions = static_cast<std::int64_t>(want_coarse);
    static const int kWinBonus[4] = {0, 0, 1, 2};
    int want_win = base_hier.max_window_attempts + kWinBonus[lvl];
    if (want_win < 1) want_win = 1;
    if (want_win > caps.max_hierarchy_window_attempts)
        want_win = caps.max_hierarchy_window_attempts;
    if (want_win < base_hier.max_window_attempts) want_win = base_hier.max_window_attempts;
    b.hier_window_attempts = want_win;
    static const std::int64_t kTubeMult[4] = {1, 1, 2, 4};
    __int128 want_tube = (__int128)base_hier.tube_half_nm * kTubeMult[lvl];
    if (want_tube < base_hier.tube_half_nm) want_tube = base_hier.tube_half_nm;
    // Tube growth is exact-search work, not memory: still cap runaway at 8x.
    __int128 tube_cap = (__int128)base_hier.tube_half_nm * 8;
    if (want_tube > tube_cap) want_tube = tube_cap;
    b.hier_tube_half_nm = static_cast<Coord>(want_tube);
    std::size_t want_cells = base_hier.max_grid_cells;
    if (caps.max_hierarchy_grid_cells > 0 && want_cells > caps.max_hierarchy_grid_cells)
        want_cells = caps.max_hierarchy_grid_cells;
    // Memory bound: ~64B per coarse cell; streamed, never dense-allocated.
    if (memory_budget_bytes > 0) {
        std::size_t mem_cells = memory_budget_bytes / 64;
        if (mem_cells < 1) mem_cells = 1;
        if (want_cells > mem_cells) want_cells = mem_cells;
    }
    b.hier_max_grid_cells = want_cells;

    // Route-K (#8 hook, provisioned): 1/3/8/12 within the 15 cap and memory
    // (~128MB per additional alternative corridor set).
    static const int kRouteK[4] = {1, 3, 8, 12};
    int want_k = std::min(kRouteK[lvl], caps.max_route_k);
    b.route_k = bound_by_mem(want_k, memory_budget_bytes, 128ULL * 1024 * 1024);

    // Reservation strength: OPEN/MID keep legacy pressure (geometry-stable);
    // dense phases push harder around predicted corridors (soft cost only).
    static const double kResStrength[4] = {1.0, 1.0, 1.5, 2.0};
    b.reservation_strength = kResStrength[lvl];

    // Batch width scales with the live backlog. Small boards keep the legacy
    // width 8 (nothing to gain, and it keeps the small-fixture schedules
    // byte-identical); large boards widen so the worker pool is saturated and
    // per-epoch scheduler/arbiter overhead is amortized over more work. This
    // is a pure function of remaining_count, so it stays thread- and timing-
    // independent (the determinism contract is unchanged). Memory-bounded
    // below: min(want, floor(budget/per_task)). No dense N^2 anywhere.
    int want_batch = 8;
    if (remaining_count >= 2048) want_batch = 64;
    else if (remaining_count >= 512) want_batch = 48;
    else if (remaining_count >= 128) want_batch = 32;
    if (want_batch > caps.max_batch_width) want_batch = caps.max_batch_width;
    if (per_task_bytes == 0) per_task_bytes = 64ULL * 1024 * 1024;
    if (memory_budget_bytes > 0) {
        std::size_t bound = memory_budget_bytes / per_task_bytes;
        if (bound < 1) bound = 1;
        if (static_cast<std::size_t>(want_batch) > bound)
            want_batch = static_cast<int>(bound);
    }
    if (want_batch < 1) want_batch = 1;
    b.batch_width = want_batch;

    // Coarse-to-fine guidance is a net loss once the backlog is board-scale:
    // the coarse grid is congested too, so per-task guidance costs more than
    // the exact search it steers. Measured on a 2458-task board, disabling it
    // routed strictly more connections in less wall time. Small boards keep
    // the legacy enabled behaviour exactly (their schedules stay pinned).
    b.hier_enabled = remaining_count < kGuidanceDisableRemaining;

    // History growth scales Pathfinder history increments (pressure memory).
    static const double kHist[4] = {1.0, 1.0, 1.5, 2.0};
    b.history_growth = kHist[lvl];

    // Rip-up breadth / recovery branches: phase floors; the engine takes
    // max(generation default, floor) so escalation only ever deepens search
    // on dense boards while OPEN boards keep legacy generation schedules.
    static const int kBreadth[4] = {1, 1, 2, 3};
    static const int kBranches[4] = {2, 2, 4, 8};
    b.ripup_breadth = std::min(kBreadth[lvl], caps.max_rip_breadth);
    b.recovery_branches = std::min(kBranches[lvl], caps.max_recovery_branches);
    if (b.ripup_breadth < 1) b.ripup_breadth = 1;
    if (b.recovery_branches < 1) b.recovery_branches = 1;

    // Recovery depth/beam (#22 hooks, advisory): phase allowance within caps
    // and memory (~256MB per beam member). The engine's explicit
    // max_ripup_generations cap always wins for actual generations.
    static const int kDepth[4] = {4, 6, 8, 10};
    static const int kBeam[4] = {1, 2, 4, 8};
    b.recovery_depth = std::min(kDepth[lvl], caps.max_recovery_depth);
    int want_beam = std::min(kBeam[lvl], caps.max_beam);
    b.recovery_beam = bound_by_mem(want_beam, memory_budget_bytes,
                                    256ULL * 1024 * 1024);
    if (b.recovery_depth < 1) b.recovery_depth = 1;

    // Issue #4: sparse-graph budgets per phase. OPEN/MID keep the legacy
    // 384/16 defaults for speed; dense boards spend more exploring
    // alternatives; CLOSURE widens to the bounded last-resort scale (wide
    // base set, K=64) so completeness, not the proximity-to-direct-segment
    // cull, decides -- without the O(bases^2) blow-up of a truly uncapped
    // base set on a large board.
    static const std::size_t kGraphBases[4] = {384, 512, 1024, kLastResortMaxBases};
    static const int kGraphK[4] = {16, 24, 32, kLastResortKNearest};
    std::size_t want_bases = kGraphBases[lvl];
    int want_gk = kGraphK[lvl];
    if (caps.max_graph_bases > 0) {
        if (want_bases == 0 || want_bases > caps.max_graph_bases)
            want_bases = caps.max_graph_bases;
    }
    if (caps.max_graph_k_nearest > 0) {
        if (want_gk <= 0 || want_gk > caps.max_graph_k_nearest)
            want_gk = caps.max_graph_k_nearest;
    }
    if (want_gk < 0) want_gk = 0;
    b.graph_max_bases = want_bases;
    b.graph_k_nearest = want_gk;

    // Timeout share per remaining task: equal split of the remaining overall
    // deadline (<=0 = no timeout configured = unlimited).
    if (timeout_remaining_s >= 0 && remaining_count > 0)
        b.timeout_share_per_task_s = timeout_remaining_s / remaining_count;
    else
        b.timeout_share_per_task_s = 0.0;

    b.threads_effective = threads_effective >= 1 ? threads_effective : 1;
    return b;
}

JsonValue BoardMaturityState::to_json() const {
    JsonValue o = JsonValue::object();
    o["phase"] = maturity_phase_name(phase);
    o["phase_level"] = static_cast<double>(static_cast<int>(phase));
    o["occupancy_frac"] = metrics.occupancy_frac;
    o["hotspot_count"] = static_cast<double>(metrics.hotspot_count);
    o["hotspot_pressure"] = metrics.hotspot_pressure;
    o["mean_remaining_difficulty"] = metrics.mean_remaining_difficulty;
    o["acceptance_rate"] = metrics.acceptance_rate;
    o["stalled_epochs"] = static_cast<double>(metrics.stalled_epochs);
    o["ripup_count"] = static_cast<double>(metrics.ripup_count);
    o["recovery_generations"] = static_cast<double>(metrics.recovery_generations);
    o["fine_pitch_done_frac"] = metrics.fine_pitch_done_frac;
    o["remaining_count"] = static_cast<double>(metrics.remaining_count);
    o["total_tasks"] = static_cast<double>(metrics.total_tasks);
    o["remaining_frac"] = metrics.remaining_frac;
    o["phase_entry_remaining"] = static_cast<double>(phase_entry_remaining);
    o["escalated_by_stall"] = escalated_by_stall;
    return o;
}

JsonValue EffectiveSearchBudget::to_json() const {
    JsonValue o = JsonValue::object();
    o["astar_max_expansions"] = static_cast<double>(astar_max_expansions);
    o["hier_enabled"] = hier_enabled;
    o["weight_factor"] = weight_factor;
    o["hier_max_coarse_expansions"] = static_cast<double>(hier_max_coarse_expansions);
    o["hier_window_attempts"] = static_cast<double>(hier_window_attempts);
    json_add_mm(o, "hier_tube_half_mm", nm_to_mm(hier_tube_half_nm));
    o["hier_max_grid_cells"] = static_cast<double>(hier_max_grid_cells);
    o["route_k"] = static_cast<double>(route_k);
    o["reservation_strength"] = reservation_strength;
    o["batch_width"] = static_cast<double>(batch_width);
    o["history_growth"] = history_growth;
    o["ripup_breadth"] = static_cast<double>(ripup_breadth);
    o["recovery_branches"] = static_cast<double>(recovery_branches);
    o["recovery_depth"] = static_cast<double>(recovery_depth);
    o["recovery_beam"] = static_cast<double>(recovery_beam);
    o["timeout_share_per_task_s"] = timeout_share_per_task_s;
    o["threads_effective"] = static_cast<double>(threads_effective);
    o["graph_max_bases"] = static_cast<double>(graph_max_bases);
    o["graph_k_nearest"] = static_cast<double>(graph_k_nearest);
    return o;
}

JsonValue MaturityLogEntry::to_json() const {
    JsonValue o = JsonValue::object();
    o["epoch"] = static_cast<double>(epoch);
    o["is_recovery"] = is_recovery;
    o["generation"] = static_cast<double>(generation);
    o["maturity"] = state.to_json();
    o["budget"] = budget.to_json();
    return o;
}

namespace {

bool get_num(const JsonValue& node, const std::string& key, double& out) {
    const JsonValue* v = node.find(key);
    if (!v) return true;  // absent = keep default
    if (!v->is_number()) return false;
    out = v->as_number();
    return true;
}

bool get_int(const JsonValue& node, const std::string& key, long long& out) {
    double d = 0;
    if (!get_num(node, key, d)) return false;
    const JsonValue* v = node.find(key);
    if (!v) return true;
    out = static_cast<long long>(d);
    return true;
}

}  // namespace

bool apply_maturity_json(MaturityOptions& out, const JsonValue& node,
                          std::string& err_out) {
    if (!node.is_object()) {
        err_out = "maturity: expected an object";
        return false;
    }
    if (const JsonValue* v = node.find("enabled")) {
        if (!v->is_bool()) {
            err_out = "maturity.enabled: expected a boolean";
            return false;
        }
        out.enabled = v->as_bool(true);
    }
    double d = 0;
    if (!get_num(node, "improvement_frac", d)) {
        err_out = "maturity.improvement_frac: expected a number";
        return false;
    }
    if (node.has("improvement_frac")) {
        if (!(d >= 0) || !(d <= 0.9)) {
            err_out = "maturity.improvement_frac: expected a number in [0, 0.9]";
            return false;
        }
        out.improvement_frac = d;
    }
    if (const JsonValue* t = node.find("thresholds")) {
        if (!t->is_object()) {
            err_out = "maturity.thresholds: expected an object";
            return false;
        }
        MaturityThresholds& th = out.thresholds;
        auto num = [&](const char* k, double& field, double lo, double hi) {
            double v = 0;
            if (!get_num(*t, k, v)) {
                err_out = std::string("maturity.thresholds.") + k + ": expected a number";
                return false;
            }
            if (t->has(k)) {
                if (!(v >= lo) || !(v <= hi)) {
                    err_out = std::string("maturity.thresholds.") + k + ": out of range";
                    return false;
                }
                field = v;
            }
            return true;
        };
        auto cnt = [&](const char* k, int& field) {
            long long v = 0;
            if (!get_int(*t, k, v)) {
                err_out = std::string("maturity.thresholds.") + k + ": expected a number";
                return false;
            }
            if (t->has(k)) {
                if (v < 0 || v > 1000000) {
                    err_out = std::string("maturity.thresholds.") + k + ": out of range";
                    return false;
                }
                field = static_cast<int>(v);
            }
            return true;
        };
        if (!num("mid_occupancy", th.mid_occupancy, 0, 1)) return false;
        if (!num("dense_occupancy", th.dense_occupancy, 0, 1)) return false;
        if (!num("closure_occupancy", th.closure_occupancy, 0, 1)) return false;
        if (!num("mid_remaining_frac", th.mid_remaining_frac, 0, 1)) return false;
        if (!num("dense_remaining_frac", th.dense_remaining_frac, 0, 1)) return false;
        if (!num("closure_remaining_frac", th.closure_remaining_frac, 0, 1)) return false;
        if (!cnt("mid_stalled_epochs", th.mid_stalled_epochs)) return false;
        if (!cnt("dense_stalled_epochs", th.dense_stalled_epochs)) return false;
        if (!cnt("closure_stalled_epochs", th.closure_stalled_epochs)) return false;
        if (!cnt("mid_hotspots", th.mid_hotspots)) return false;
        if (!cnt("dense_hotspots", th.dense_hotspots)) return false;
        if (!cnt("closure_hotspots", th.closure_hotspots)) return false;
        if (!num("hotspot_score_floor", th.hotspot_score_floor, 0, 1e9)) return false;
        if (!cnt("mid_recovery_gens", th.mid_recovery_gens)) return false;
        if (!cnt("dense_recovery_gens", th.dense_recovery_gens)) return false;
        if (!cnt("closure_recovery_gens", th.closure_recovery_gens)) return false;
        if (!cnt("dense_ripups", th.dense_ripups)) return false;
        if (!num("mid_mean_difficulty", th.mid_mean_difficulty, 0, 1e9)) return false;
        if (!num("dense_mean_difficulty", th.dense_mean_difficulty, 0, 1e9)) return false;
        if (!num("fine_pitch_mid_frac", th.fine_pitch_mid_frac, 0, 1)) return false;
        if (!(th.mid_occupancy <= th.dense_occupancy &&
              th.dense_occupancy <= th.closure_occupancy)) {
            err_out = "maturity.thresholds: occupancy gates must be non-decreasing";
            return false;
        }
    }
    if (const JsonValue* c = node.find("caps")) {
        if (!c->is_object()) {
            err_out = "maturity.caps: expected an object";
            return false;
        }
        MaturityCaps& cp = out.caps;
        auto capi = [&](const char* k, long long& field, long long lo, long long hi,
                        bool allow_zero = true) {
            long long v = 0;
            if (!get_int(*c, k, v)) {
                err_out = std::string("maturity.caps.") + k + ": expected a number";
                return false;
            }
            if (c->has(k)) {
                if (v < (allow_zero ? 0 : 1) || v > hi) {
                    err_out = std::string("maturity.caps.") + k + ": out of range";
                    return false;
                }
                if (v < lo && !(allow_zero && v == 0)) {
                    err_out = std::string("maturity.caps.") + k + ": out of range";
                    return false;
                }
                field = v;
            }
            return true;
        };
        long long v = 0;
        if (!capi("max_astar_expansions", v, 1, 1000000000)) return false;
        if (c->has("max_astar_expansions")) cp.max_astar_expansions = v;
        if (!capi("max_batch_width", v, 1, 1024, false)) return false;
        if (c->has("max_batch_width")) cp.max_batch_width = static_cast<int>(v);
        if (!capi("max_route_k", v, 1, 64, false)) return false;
        if (c->has("max_route_k")) cp.max_route_k = static_cast<int>(v);
        if (!capi("max_recovery_branches", v, 1, 64, false)) return false;
        if (c->has("max_recovery_branches"))
            cp.max_recovery_branches = static_cast<int>(v);
        if (!capi("max_rip_breadth", v, 1, 64, false)) return false;
        if (c->has("max_rip_breadth")) cp.max_rip_breadth = static_cast<int>(v);
        if (!capi("max_recovery_depth", v, 1, 64, false)) return false;
        if (c->has("max_recovery_depth"))
            cp.max_recovery_depth = static_cast<int>(v);
        if (!capi("max_beam", v, 1, 64, false)) return false;
        if (c->has("max_beam")) cp.max_beam = static_cast<int>(v);
        if (!capi("max_hierarchy_window_attempts", v, 1, 16, false)) return false;
        if (c->has("max_hierarchy_window_attempts"))
            cp.max_hierarchy_window_attempts = static_cast<int>(v);
        if (!capi("max_hierarchy_grid_cells", v, 0, 1000000000)) return false;
        if (c->has("max_hierarchy_grid_cells"))
            cp.max_hierarchy_grid_cells = static_cast<std::size_t>(v);
        if (!capi("max_graph_bases", v, 0, 100000000)) return false;
        if (c->has("max_graph_bases"))
            cp.max_graph_bases = static_cast<std::size_t>(v);
        if (!capi("max_graph_k_nearest", v, 0, 1000000)) return false;
        if (c->has("max_graph_k_nearest"))
            cp.max_graph_k_nearest = static_cast<int>(v);
        double w = 0;
        if (!get_num(*c, "max_weight_factor", w)) {
            err_out = "maturity.caps.max_weight_factor: expected a number";
            return false;
        }
        if (c->has("max_weight_factor")) {
            if (!(w >= 1.0) || !(w <= 4.0)) {
                err_out = "maturity.caps.max_weight_factor: expected a number in [1, 4]";
                return false;
            }
            cp.max_weight_factor = w;
        }
    }
    return true;
}

}  // namespace copperline
