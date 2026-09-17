// Issue #14: board-maturity adaptive search budgets.
//
// Same board transitions to larger budgets as occupancy/stalls grow; easy
// boards stay cheap; late dense boards get larger K / finer hierarchy /
// deeper recovery within configured caps; the schedule is deterministic.
#include <string>

#include "helpers.h"
#include "router/engine.h"
#include "router/maturity.h"
#include "router/parallel.h"

using namespace copperline;
using namespace copperline::test;

namespace {

MaturityInput easy_input() {
    MaturityInput in;
    in.occupancy_frac = 0.01;
    in.hotspot_count = 0;
    in.hotspot_pressure = 0.0;
    in.mean_remaining_difficulty = 5.0;
    in.acceptance_rate = 1.0;
    in.stalled_epochs = 0;
    in.ripup_count = 0;
    in.recovery_generations = 0;
    in.fine_pitch_done_frac = 1.0;
    in.remaining_count = 8;
    in.total_tasks = 8;
    in.remaining_frac = 1.0;
    return in;
}

MaturityInput dense_input() {
    MaturityInput in;
    in.occupancy_frac = 0.50;
    in.hotspot_count = 8;
    in.hotspot_pressure = 40.0;
    in.mean_remaining_difficulty = 50.0;
    in.acceptance_rate = 0.0;
    in.stalled_epochs = 3;
    in.ripup_count = 7;
    in.recovery_generations = 2;
    in.fine_pitch_done_frac = 1.0;
    in.remaining_count = 6;
    in.total_tasks = 8;
    in.remaining_frac = 0.75;
    return in;
}

AStarConfig base_astar() {
    AStarConfig c;
    c.max_expansions = 200000;
    return c;
}

}  // namespace

CT_TEST(maturity_easy_board_stays_open_and_cheap) {
    MaturityThresholds th;
    BoardMaturityState st = update_maturity(easy_input(), th, nullptr, 0.10);
    CT_CHECK(st.phase == MaturityPhase::OPEN_BOARD);
    CT_CHECK(maturity_phase_name(st.phase) == "OPEN_BOARD");

    MaturityCaps caps;
    EffectiveSearchBudget b = effective_budget_for_phase(
        st, base_astar(), HierarchyConfig{}, caps, kRouterMemoryBudgetBytes,
        kPerCandidateBytes, 4, -1.0, 8);
    CT_CHECK(b.astar_max_expansions == 200000);
    CT_CHECK_NEAR(b.weight_factor, 1.0, 1e-12);
    CT_CHECK(b.route_k == 1);
    CT_CHECK_NEAR(b.reservation_strength, 1.0, 1e-12);
    CT_CHECK(b.batch_width == 8);
    CT_CHECK_NEAR(b.history_growth, 1.0, 1e-12);
    CT_CHECK(b.ripup_breadth == 1);
    CT_CHECK(b.recovery_branches == 2);
    CT_CHECK(b.hier_window_attempts == HierarchyConfig{}.max_window_attempts);
    CT_CHECK(b.hier_max_coarse_expansions == 200000);
    CT_CHECK(b.threads_effective == 4);
    // Small boards keep coarse-to-fine guidance enabled.
    CT_CHECK(b.hier_enabled);
}

CT_TEST(maturity_guidance_disabled_once_backlog_is_board_scale) {
    // Board-scale backlogs turn guidance off (it costs more than it steers);
    // everything below the threshold is untouched. Pure function of the
    // backlog, so it stays thread- and timing-independent.
    MaturityThresholds th;
    BoardMaturityState st = update_maturity(easy_input(), th, nullptr, 0.10);
    MaturityCaps caps;
    auto hier_for = [&](int remaining) {
        return effective_budget_for_phase(st, base_astar(), HierarchyConfig{}, caps,
                                          kRouterMemoryBudgetBytes, kPerCandidateBytes, 16,
                                          -1.0, remaining)
            .hier_enabled;
    };
    CT_CHECK(hier_for(8));
    CT_CHECK(hier_for(512));
    CT_CHECK(hier_for(kGuidanceDisableRemaining - 1));
    CT_CHECK(!hier_for(kGuidanceDisableRemaining));
    CT_CHECK(!hier_for(4096));
    // JSON surface carries the decision for agents.
    EffectiveSearchBudget b = effective_budget_for_phase(
        st, base_astar(), HierarchyConfig{}, caps, kRouterMemoryBudgetBytes,
        kPerCandidateBytes, 16, -1.0, 4096);
    JsonValue j = b.to_json();
    CT_CHECK(j["hier_enabled"].as_bool() == false);
}

CT_TEST(maturity_batch_width_scales_with_backlog_and_is_memory_bounded) {
    MaturityThresholds th;
    BoardMaturityState st = update_maturity(easy_input(), th, nullptr, 0.10);
    MaturityCaps caps;
    auto width_for = [&](int remaining, const MaturityCaps& c) {
        return effective_budget_for_phase(st, base_astar(), HierarchyConfig{}, c,
                                          kRouterMemoryBudgetBytes, kPerCandidateBytes,
                                          16, -1.0, remaining)
            .batch_width;
    };
    // Small/medium backlog keeps the legacy width 8: small-fixture schedules
    // (and their pinned hashes) are untouched.
    CT_CHECK(width_for(8, caps) == 8);
    CT_CHECK(width_for(32, caps) == 8);
    CT_CHECK(width_for(64, caps) == 8);
    // Larger backlog widens toward the worker count, then the memory bound
    // (2048MB / 64MB = 32) and the explicit cap, whichever is smaller.
    CT_CHECK(width_for(128, caps) == 32);
    CT_CHECK(width_for(4096, caps) == 32);  // 64 wanted, memory-bounded to 32
    MaturityCaps capped;
    capped.max_batch_width = 12;
    CT_CHECK(width_for(4096, capped) == 12);  // explicit cap always wins
}

CT_TEST(maturity_same_board_escalates_with_occupancy_and_stalls) {
    MaturityThresholds th;
    MaturityCaps caps;
    BoardMaturityState open = update_maturity(easy_input(), th, nullptr, 0.10);
    BoardMaturityState dense = update_maturity(dense_input(), th, &open, 0.10);
    CT_CHECK(dense.phase == MaturityPhase::DENSE_ROUTE ||
             dense.phase == MaturityPhase::CLOSURE);

    EffectiveSearchBudget bo = effective_budget_for_phase(
        open, base_astar(), HierarchyConfig{}, caps, kRouterMemoryBudgetBytes,
        kPerCandidateBytes, 4, -1.0, 8);
    EffectiveSearchBudget bd = effective_budget_for_phase(
        dense, base_astar(), HierarchyConfig{}, caps, kRouterMemoryBudgetBytes,
        kPerCandidateBytes, 4, -1.0, 6);
    CT_CHECK(bd.astar_max_expansions >= 4 * bo.astar_max_expansions);
    CT_CHECK(bd.weight_factor > bo.weight_factor);
    CT_CHECK(bd.route_k > bo.route_k);
    CT_CHECK(bd.reservation_strength > bo.reservation_strength);
    CT_CHECK(bd.hier_max_coarse_expansions > bo.hier_max_coarse_expansions);
    CT_CHECK(bd.hier_window_attempts > bo.hier_window_attempts);
    CT_CHECK(bd.ripup_breadth > bo.ripup_breadth);
    CT_CHECK(bd.recovery_branches > bo.recovery_branches);
    CT_CHECK(bd.recovery_depth > bo.recovery_depth);
    CT_CHECK(bd.recovery_beam >= bo.recovery_beam);
    CT_CHECK(bd.history_growth > bo.history_growth);
}

CT_TEST(maturity_easy_board_never_pays_closure) {
    // Zero pressure, zero occupancy: draining the remaining queue may reach
    // MID_ROUTE on progress alone, but never dense/closure search cost.
    MaturityThresholds th;
    MaturityCaps caps;
    // Chained updates (with hysteresis): draining the queue may reach
    // MID_ROUTE on progress alone, but never dense/closure search cost.
    BoardMaturityState chained = update_maturity(easy_input(), th, nullptr, 0.10);
    for (int remaining = 7; remaining >= 0; --remaining) {
        MaturityInput in = easy_input();
        in.remaining_count = remaining;
        in.remaining_frac = remaining / 8.0;
        chained = update_maturity(in, th, &chained, 0.10);
        CT_CHECK(chained.phase == MaturityPhase::OPEN_BOARD ||
                 chained.phase == MaturityPhase::MID_ROUTE);
        EffectiveSearchBudget b = effective_budget_for_phase(
            chained, base_astar(), HierarchyConfig{}, caps,
            kRouterMemoryBudgetBytes, kPerCandidateBytes, 4, -1.0, remaining);
        CT_CHECK(b.astar_max_expansions <= 2 * 200000);
        CT_CHECK_NEAR(b.weight_factor, 1.0, 1e-12);
        CT_CHECK_NEAR(b.reservation_strength, 1.0, 1e-12);
    }
}

CT_TEST(maturity_late_dense_capped_within_agent_caps) {
    MaturityThresholds th;
    MaturityCaps caps;
    caps.max_astar_expansions = 300000;
    caps.max_route_k = 5;
    caps.max_recovery_branches = 3;
    caps.max_rip_breadth = 2;
    caps.max_recovery_depth = 6;
    caps.max_beam = 2;
    caps.max_hierarchy_window_attempts = 3;
    caps.max_weight_factor = 1.25;

    BoardMaturityState open = update_maturity(easy_input(), th, nullptr, 0.10);
    BoardMaturityState dense = update_maturity(dense_input(), th, &open, 0.10);
    EffectiveSearchBudget bo = effective_budget_for_phase(
        open, base_astar(), HierarchyConfig{}, caps, kRouterMemoryBudgetBytes,
        kPerCandidateBytes, 4, -1.0, 8);
    EffectiveSearchBudget bd = effective_budget_for_phase(
        dense, base_astar(), HierarchyConfig{}, caps, kRouterMemoryBudgetBytes,
        kPerCandidateBytes, 4, -1.0, 6);
    // Hard ceilings hold ...
    CT_CHECK(bd.astar_max_expansions <= 300000);
    CT_CHECK(bd.route_k <= 5);
    CT_CHECK(bd.recovery_branches <= 3);
    CT_CHECK(bd.ripup_breadth <= 2);
    CT_CHECK(bd.recovery_depth <= 6);
    CT_CHECK(bd.recovery_beam <= 2);
    CT_CHECK(bd.hier_window_attempts <= 3);
    CT_CHECK(bd.weight_factor <= 1.25 + 1e-12);
    // ... while dense still strictly outspends the cheap open schedule.
    CT_CHECK(bd.astar_max_expansions > bo.astar_max_expansions);
    CT_CHECK(bd.route_k > bo.route_k);
    CT_CHECK(bd.hier_window_attempts >= bo.hier_window_attempts);
    CT_CHECK(bd.hier_max_coarse_expansions > bo.hier_max_coarse_expansions);
    CT_CHECK(bd.recovery_branches > bo.recovery_branches);
    CT_CHECK(bd.ripup_breadth >= bo.ripup_breadth);
}

CT_TEST(maturity_schedule_is_deterministic) {
    MaturityThresholds th;
    MaturityCaps caps;
    BoardMaturityState a = update_maturity(dense_input(), th, nullptr, 0.10);
    BoardMaturityState b = update_maturity(dense_input(), th, nullptr, 0.10);
    CT_CHECK(maturity_phase_name(a.phase) == maturity_phase_name(b.phase));
    EffectiveSearchBudget ba = effective_budget_for_phase(
        a, base_astar(), HierarchyConfig{}, caps, kRouterMemoryBudgetBytes,
        kPerCandidateBytes, 4, 10.0, 6);
    EffectiveSearchBudget bb = effective_budget_for_phase(
        b, base_astar(), HierarchyConfig{}, caps, kRouterMemoryBudgetBytes,
        kPerCandidateBytes, 4, 10.0, 6);
    CT_CHECK(serialize_json(ba.to_json()) == serialize_json(bb.to_json()));
    CT_CHECK(serialize_json(a.to_json()) == serialize_json(b.to_json()));
    // Timeout share splits the remaining deadline deterministically.
    CT_CHECK_NEAR(ba.timeout_share_per_task_s, 10.0 / 6, 1e-9);

    // Worsening stalls walk the phases monotonically (escalation is fast).
    BoardMaturityState chained = update_maturity(easy_input(), th, nullptr, 0.10);
    int last_level = 0;
    for (int stall = 0; stall <= 6; ++stall) {
        MaturityInput in = easy_input();
        in.stalled_epochs = stall;
        chained = update_maturity(in, th, &chained, 0.10);
        int lvl = static_cast<int>(chained.phase);
        CT_CHECK(lvl >= last_level);
        last_level = lvl;
    }
    CT_CHECK(chained.phase == MaturityPhase::CLOSURE);
}

CT_TEST(maturity_hotspot_gates) {
    // Contested-cell counts (post score-floor) escalate on their own.
    MaturityThresholds th;
    MaturityInput d = easy_input();
    d.hotspot_count = th.dense_hotspots;
    d.hotspot_pressure = 25.0;
    CT_CHECK(classify_maturity_raw(d, th) == MaturityPhase::DENSE_ROUTE);
    MaturityInput c = easy_input();
    c.hotspot_count = th.closure_hotspots;
    c.hotspot_pressure = 40.0;
    CT_CHECK(classify_maturity_raw(c, th) == MaturityPhase::CLOSURE);
    // Below the MID count: no escalation from hotspots.
    MaturityInput q = easy_input();
    q.hotspot_count = th.mid_hotspots - 1;
    CT_CHECK(classify_maturity_raw(q, th) == MaturityPhase::OPEN_BOARD);
}

CT_TEST(maturity_difficulty_gate) {
    // Difficulty alone (no occupancy/stalls/hotspots) escalates once routing
    // is underway: a board whose remainders are all hard routes deserves
    // deeper search. A fresh untouched board stays cheap no matter the span.
    MaturityThresholds th;
    MaturityInput fresh = easy_input();
    fresh.mean_remaining_difficulty = th.dense_mean_difficulty + 10.0;
    CT_CHECK(classify_maturity_raw(fresh, th) == MaturityPhase::OPEN_BOARD);
    MaturityInput mid = easy_input();
    mid.mean_remaining_difficulty = th.mid_mean_difficulty + 5.0;
    mid.remaining_count = 4;
    mid.remaining_frac = 0.5;
    CT_CHECK(classify_maturity_raw(mid, th) == MaturityPhase::MID_ROUTE);
    MaturityInput dense = easy_input();
    dense.mean_remaining_difficulty = th.dense_mean_difficulty + 10.0;
    dense.remaining_count = 4;
    dense.remaining_frac = 0.5;
    CT_CHECK(classify_maturity_raw(dense, th) == MaturityPhase::DENSE_ROUTE);
    // Ordinary single-task difficulty (~30 for a 16mm span) never drives
    // dense search by itself: fresh it stays OPEN, half-drained it is MID
    // on progress alone (cheap headroom, identical ordering).
    MaturityInput ordinary_fresh = easy_input();
    ordinary_fresh.mean_remaining_difficulty = 30.0;
    CT_CHECK(classify_maturity_raw(ordinary_fresh, th) == MaturityPhase::OPEN_BOARD);
    MaturityInput ordinary = easy_input();
    ordinary.mean_remaining_difficulty = 30.0;
    ordinary.remaining_count = 4;
    ordinary.remaining_frac = 0.5;
    CT_CHECK(classify_maturity_raw(ordinary, th) == MaturityPhase::MID_ROUTE);
}

CT_TEST(maturity_deescalation_needs_measurable_improvement) {
    MaturityThresholds th;
    BoardMaturityState open = update_maturity(easy_input(), th, nullptr, 0.10);
    // Drive into DENSE_ROUTE via stalls (remaining stays 8).
    MaturityInput bad = easy_input();
    bad.stalled_epochs = 3;
    bad.hotspot_count = 7;
    BoardMaturityState dense = update_maturity(bad, th, &open, 0.10);
    CT_CHECK(dense.phase == MaturityPhase::DENSE_ROUTE ||
             dense.phase == MaturityPhase::CLOSURE);

    // Pressure gone but remaining barely moved (8 -> 8): hold the phase.
    MaturityInput mild = easy_input();
    mild.remaining_count = 8;
    mild.remaining_frac = 1.0;
    BoardMaturityState held = update_maturity(mild, th, &dense, 0.10);
    CT_CHECK(held.phase == dense.phase);

    // Measurable improvement (8 -> 2, a 75% drop): step down allowed.
    MaturityInput better = easy_input();
    better.remaining_count = 2;
    better.remaining_frac = 0.25;
    BoardMaturityState down = update_maturity(better, th, &dense, 0.10);
    CT_CHECK(static_cast<int>(down.phase) < static_cast<int>(dense.phase));
}

CT_TEST(maturity_occupancy_helper) {
    Board empty = base_2layer(20.0, 20.0);
    CT_CHECK_NEAR(copper_occupancy_frac(empty), 0.0, 1e-12);
    Board b = base_2layer(20.0, 20.0);
    TraceSeg t;
    t.net = 0;
    t.layer = 0;
    t.a = {0, 0};
    t.b = {mm_to_nm(20.0), 0};
    t.width_nm = mm_to_nm(0.2);
    b.traces.push_back(t);
    double occ = copper_occupancy_frac(b);
    CT_CHECK(occ > 0.0 && occ < 1.0);
    CT_CHECK_NEAR(occ, 0.2 * 20.0 / 400.0, 1e-9);
    // Deterministic across calls.
    CT_CHECK(copper_occupancy_frac(b) == occ);
}

CT_TEST(maturity_config_json_round_trip) {
    MaturityOptions opt;
    std::string err;
    CT_CHECK(apply_maturity_json(opt, parse_json(R"({"enabled": false})"), err));
    CT_CHECK(!opt.enabled);
    CT_CHECK(apply_maturity_json(
        opt, parse_json(R"({"enabled": true, "improvement_frac": 0.2,
                             "thresholds": {"dense_occupancy": 0.4},
                             "caps": {"max_route_k": 7, "max_weight_factor": 1.5}})"),
        err));
    CT_CHECK(opt.enabled);
    CT_CHECK_NEAR(opt.improvement_frac, 0.2, 1e-12);
    CT_CHECK_NEAR(opt.thresholds.dense_occupancy, 0.4, 1e-12);
    CT_CHECK(opt.caps.max_route_k == 7);
    CT_CHECK_NEAR(opt.caps.max_weight_factor, 1.5, 1e-12);
    // Unknown keys are ignored (forward compatibility).
    CT_CHECK(apply_maturity_json(opt, parse_json(R"({"future_knob": 3})"), err));
    // Out-of-range values are rejected, never silently clamped.
    CT_CHECK(!apply_maturity_json(opt, parse_json(R"({"improvement_frac": 5})"), err));
    CT_CHECK(!apply_maturity_json(opt, parse_json(R"({"caps": {"max_route_k": 0}})"), err));
    CT_CHECK(!apply_maturity_json(
        opt, parse_json(R"({"thresholds": {"mid_occupancy": 0.9, "dense_occupancy": 0.1}})"),
        err));
}

CT_TEST(maturity_engine_report_carries_phase_and_budget) {
    Board b = base_2layer(20.0, 20.0);
    NetInfo n0 = make_net(0, "A");
    n0.has_current = true;
    n0.current_a = 0.1;
    b.nets.push_back(n0);
    NetInfo n1 = make_net(1, "B");
    n1.has_current = true;
    n1.current_a = 0.1;
    b.nets.push_back(n1);
    add_terminal(b, 0, 2.0, 10.0);
    add_terminal(b, 0, 18.0, 10.0);
    add_terminal(b, 1, 2.0, 12.0);
    add_terminal(b, 1, 18.0, 12.0);

    auto run = [&]() {
        Board bb = b;
        RuleResolver r = RuleResolver::defaults_for(bb);
        EngineOptions opt;
        opt.threads = 1;
        RouterEngine eng(std::move(bb), std::move(r), opt);
        return eng.run();
    };
    RouteReport r1 = run();
    RouteReport r2 = run();
    CT_CHECK(r1.status == "COMPLETE");
    CT_CHECK(r1.has_budget);
    CT_CHECK(!r1.maturity_log.empty());
    // An easy open board starts (and stays) in cheap phases.
    CT_CHECK(r1.maturity_log.front().state.phase == MaturityPhase::OPEN_BOARD);
    for (const auto& e : r1.maturity_log) {
        CT_CHECK(e.state.phase == MaturityPhase::OPEN_BOARD ||
                 e.state.phase == MaturityPhase::MID_ROUTE);
    }
    // Epoch NDJSON/JSON carries phase, metrics and effective hyperparams.
    CT_CHECK(!r1.epochs.empty());
    for (const auto& e : r1.epochs) {
        CT_CHECK(!e.maturity_phase.empty());
        CT_CHECK(e.maturity.has("phase"));
        CT_CHECK(e.budget.has("astar_max_expansions"));
        CT_CHECK(e.budget.has("route_k"));
        CT_CHECK(e.budget.has("threads_effective"));
    }
    JsonValue rj = r1.to_json();
    CT_CHECK(rj.has("maturity_log"));
    CT_CHECK(rj.has("maturity"));
    CT_CHECK(rj.has("budget"));
    CT_CHECK(rj.find("maturity")->get_string("phase") == r1.maturity_log.back().state.to_json().get_string("phase"));
    // Deterministic schedule: same board, same phases and hash.
    CT_CHECK(r1.board_hash == r2.board_hash);
    CT_CHECK(r1.maturity_log.size() == r2.maturity_log.size());
    for (std::size_t i = 0; i < r1.maturity_log.size(); ++i) {
        CT_CHECK(maturity_phase_name(r1.maturity_log[i].state.phase) ==
                 maturity_phase_name(r2.maturity_log[i].state.phase));
        CT_CHECK(serialize_json(r1.maturity_log[i].budget.to_json()) ==
                 serialize_json(r2.maturity_log[i].budget.to_json()));
    }
}

CT_TEST(maturity_memory_budget_bounds_batch_k_beam) {
    MaturityThresholds th;
    // A tiny memory budget clamps every memory-hungry param; the schedule
    // stays deterministic and >= 1 everywhere.
    BoardMaturityState dense = update_maturity(dense_input(), th, nullptr, 0.10);
    MaturityCaps caps;
    EffectiveSearchBudget b = effective_budget_for_phase(
        dense, base_astar(), HierarchyConfig{}, caps,
        256ULL * 1024 * 1024, 64ULL * 1024 * 1024, 4, -1.0, 6);
    CT_CHECK(b.batch_width == 4);  // floor(256/64)
    CT_CHECK(b.route_k >= 1 && b.route_k <= 12);
    CT_CHECK(b.recovery_beam >= 1);
    CT_CHECK(b.threads_effective == 4);
    // Explicit --threads-style resolution is honored (0 = auto handled by
    // resolve_worker_threads at the engine boundary).
    CT_CHECK(resolve_worker_threads(0) >= 1);
    CT_CHECK(resolve_worker_threads(3) == 3);
}

int main() { return copperline::test::run_all_tests(); }
