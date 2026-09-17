// Prompt 4 tests: rip-up/reroute, blocker attribution, dependency graph,
// protection, state hashing, transposition, move ordering, history/PV,
// iterative modes, parallel speculative branches and escalating recovery.
//
// Centrepiece: a single-layer U-enclosure board where greedy A* seals the
// only exit (SEAL straight across the slot mouth), stranding TRAPPED. The
// engine must attribute the blocker, rip SEAL, reroute TRAPPED first + SEAL
// via a different allocation, connect everything and pass the verifier.
#include <algorithm>
#include <map>
#include <set>
#include <thread>

#include "helpers.h"
#include "router/astar.h"
#include "router/board.h"
#include "router/engine.h"
#include "router/parallel.h"
#include "router/recovery.h"
#include "router/sparse_graph.h"
#include "router/verifier.h"

using namespace copperline;
using namespace copperline::test;

#ifndef FIXTURE_DIR
#define FIXTURE_DIR "fixtures"
#endif

namespace {

// Single-layer U-enclosure: SEAL (2,9)-(18,9) straight seals the slot mouth
// y=9; TRAPPED (10,5)-(10,15) lives inside the U (walls to y=8.6 leave SEAL
// legal but plug every sideways bypass). Mirrors fixtures/forced_ripup.json.
Board trap_board() {
    Board b;
    b.source_format = "test";
    b.width_nm = mm_to_nm(20.0);
    b.height_nm = mm_to_nm(20.0);
    b.layers.push_back({0, "Top"});
    NetInfo seal = make_net(0, "SEAL");
    seal.has_current = true;
    seal.current_a = 0.1;
    NetInfo trapped = make_net(1, "TRAPPED");
    trapped.has_current = true;
    trapped.current_a = 0.1;
    b.nets.push_back(seal);
    b.nets.push_back(trapped);
    add_terminal(b, 0, 2.0, 9.0);
    add_terminal(b, 0, 18.0, 9.0);
    add_terminal(b, 1, 10.0, 5.0);
    add_terminal(b, 1, 10.0, 15.0);
    auto wall = [&](double x1, double y1, double x2, double y2, const char* reason) {
        Keepout k;
        k.rect = {mm_to_nm(x1), mm_to_nm(y1), mm_to_nm(x2), mm_to_nm(y2)};
        k.layer = kAllLayers;
        k.reason = reason;
        b.keepouts.push_back(k);
    };
    wall(8, 3, 9, 8.6, "u_left");
    wall(11, 3, 12, 8.6, "u_right");
    wall(8, 3, 12, 4, "u_bottom");
    return b;
}

RouteReport run_board(Board b, int threads, bool ripup) {
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = threads;
    opt.enable_ripup = ripup;
    RouterEngine engine(std::move(b), std::move(r), opt);
    return engine.run();
}

const RouteFailure* find_failure(const RouteReport& rep, const std::string& net) {
    for (const auto& f : rep.failures)
        if (f.net_name == net) return &f;
    return nullptr;
}

bool verify_legal(const Board& board) {
    BoardVerifier v;
    ElectricalContext ctx;
    RuleResolver r = RuleResolver::defaults_for(board);
    return v.verify(board, r, ctx).legal;
}

bool verify_ok(const Board& board) {
    BoardVerifier v;
    ElectricalContext ctx;
    RuleResolver r = RuleResolver::defaults_for(board);
    return v.verify(board, r, ctx).ok;
}

}  // namespace

CT_TEST(greedy_stall_dead_end_without_ripup) {
    // 1. initial route reaches a dead end: SEAL connects, TRAPPED strands.
    RouteReport rep = run_board(trap_board(), 1, /*ripup=*/false);
    CT_CHECK(rep.status == "INCOMPLETE");
    CT_CHECK(rep.stats.tasks_routed == 1);
    const RouteFailure* f = find_failure(rep, "TRAPPED");
    CT_CHECK(f != nullptr);
    CT_CHECK(f->ripup_attempts == 0);
}

CT_TEST(blocker_attribution_identifies_obstructing_route) {
    // 2. blocker attribution identifies the obstructing route.
    Board b = trap_board();
    // Simulate the greedy dead end: SEAL straight committed.
    TraceSeg seal{0, 0, {mm_to_nm(2.0), mm_to_nm(9.0)}, {mm_to_nm(18.0), mm_to_nm(9.0)},
                  mm_to_nm(0.2)};
    b.traces.push_back(seal);
    ConnectionTask trapped{1, b.nets[1].terminals[0], b.nets[1].terminals[1], 0, 0};
    std::vector<BlockerHit> hits =
        attribute_blockers_detailed(b, trapped, mm_to_nm(0.2), nullptr);
    CT_CHECK(!hits.empty());
    CT_CHECK(hits[0].kind == std::string("trace"));
    CT_CHECK(hits[0].net == 0);
    CT_CHECK(hits[0].desc.find("SEAL") != std::string::npos);
}

CT_TEST(dependency_graph_links_blocked_to_blocker) {
    Board b = trap_board();
    TraceSeg seal{0, 0, {mm_to_nm(2.0), mm_to_nm(9.0)}, {mm_to_nm(18.0), mm_to_nm(9.0)},
                  mm_to_nm(0.2)};
    b.traces.push_back(seal);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    ConnectionTask t_seal{0, b.nets[0].terminals[0], b.nets[0].terminals[1], 0, 1.0};
    ConnectionTask t_trapped{1, b.nets[1].terminals[0], b.nets[1].terminals[1], 1, 2.0};
    std::vector<ConnectionTask> tasks = {t_seal, t_trapped};
    std::map<std::pair<NetId, std::pair<TermId, TermId>>, CandidateRoute> last;
    DependencyGraph g = build_dependency_graph(b, r, ctx, tasks, {1}, last);
    CT_CHECK(g.failed.size() == 1);
    bool found = false;
    for (const auto& e : g.edges) {
        if (e.failed_net == 1 && e.blocker_net == 0) {
            found = true;
            CT_CHECK(e.weight > 0);
        }
    }
    CT_CHECK(found);
}

CT_TEST(protection_escape_stable_fixed) {
    double normal = route_protection_score(false, 5.0, 0, false, RecoveryMode::FAST);
    double escape = route_protection_score(true, 5.0, 0, false, RecoveryMode::FAST);
    CT_CHECK(escape > normal);  // escape stubs cost more to rip initially
    double stable = route_protection_score(false, 5.0, 4, false, RecoveryMode::FAST);
    CT_CHECK(stable > normal);  // stable difficult routes accumulate protection
    CT_CHECK(route_protection_score(false, 5.0, 0, true, RecoveryMode::FAST) >= kFixedProtection);
    CT_CHECK(route_protection_score(true, 5.0, 9, true, RecoveryMode::EXHAUSTIVE_LOCAL) >=
             kFixedProtection);  // fixed user copper is never ripped
    double exh_escape = route_protection_score(true, 5.0, 0, false, RecoveryMode::EXHAUSTIVE_LOCAL);
    CT_CHECK(exh_escape < escape);  // exhaustive may reconsider escape bundles
}

CT_TEST(state_hash_stable_sensitive_order_free) {
    Board b = trap_board();
    ConnectionTask t0{0, b.nets[0].terminals[0], b.nets[0].terminals[1], 0, 0};
    ConnectionTask t1{1, b.nets[1].terminals[0], b.nets[1].terminals[1], 1, 0};
    std::vector<ConnectionTask> tasks = {t0, t1};
    StateHash128 h1 = state_hash128(b, tasks, {0, 1});
    StateHash128 h2 = state_hash128(b, tasks, {1, 0});
    CT_CHECK(h1 == h2);  // remaining order must not matter
    CT_CHECK(h1.to_hex().size() == 32);
    Board moved = b;
    moved.traces.push_back(
        {0, 0, {mm_to_nm(2.0), mm_to_nm(9.0)}, {mm_to_nm(18.0), mm_to_nm(9.0)}, mm_to_nm(0.2)});
    CT_CHECK(!(state_hash128(moved, tasks, {0, 1}) == h1));  // copper changes the hash
    CT_CHECK(!(state_hash128(b, tasks, {0}) == h1));  // remaining set changes the hash
}

CT_TEST(transposition_prunes_revisited_states) {
    TranspositionTable tt;
    Board b = trap_board();
    ConnectionTask t0{0, b.nets[0].terminals[0], b.nets[0].terminals[1], 0, 0};
    std::vector<ConnectionTask> tasks = {t0};
    StateHash128 h = state_hash128(b, tasks, {0});
    CT_CHECK(!tt.should_prune(h, 1));
    tt.record(h, 1);
    CT_CHECK(tt.should_prune(h, 1));  // same state: prune
    CT_CHECK(tt.should_prune(h, 2));  // reached before with fewer remaining: prune
    CT_CHECK(!tt.should_prune(h, 0));  // strictly better: do not prune
    CT_CHECK(tt.hits() >= 2);
}

CT_TEST(history_heuristic_rewards_and_keys) {
    HistoryHeuristic hh;
    ConnectionTask f{1, 10, 11, 0, 0};
    std::string k = HistoryHeuristic::move_key(f, 0);
    CT_CHECK(hh.bonus(k) == 0.0);
    hh.reward(k, 2.0);
    CT_CHECK(hh.bonus(k) == 2.0);
    CT_CHECK(HistoryHeuristic::move_key(f, 0) == k);
    CT_CHECK(!(HistoryHeuristic::move_key(f, 1) == k));
}

CT_TEST(move_generation_deterministic_selective) {
    Board b = trap_board();
    b.traces.push_back(
        {0, 0, {mm_to_nm(2.0), mm_to_nm(9.0)}, {mm_to_nm(18.0), mm_to_nm(9.0)}, mm_to_nm(0.2)});
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    ConnectionTask t_seal{0, b.nets[0].terminals[0], b.nets[0].terminals[1], 0, 27.0};
    ConnectionTask t_trapped{1, b.nets[1].terminals[0], b.nets[1].terminals[1], 1, 21.0};
    std::vector<ConnectionTask> tasks = {t_seal, t_trapped};
    std::map<std::pair<NetId, std::pair<TermId, TermId>>, CandidateRoute> last;
    DependencyGraph g = build_dependency_graph(b, r, ctx, tasks, {1}, last);
    OwnedRoute o;
    o.task = t_seal;
    o.task_pos = 0;
    o.traces = b.traces;
    o.protection = route_protection_score(false, 27.0, 1, false, RecoveryMode::FAST);
    HistoryHeuristic hh;
    auto m1 = generate_ripup_moves(g.failed, g, {o}, hh, "", RecoveryMode::FAST, 4, 2);
    auto m2 = generate_ripup_moves(g.failed, g, {o}, hh, "", RecoveryMode::FAST, 4, 2);
    CT_CHECK(!m1.empty());
    CT_CHECK(m1.size() == m2.size());
    for (std::size_t i = 0; i < m1.size(); ++i) {
        CT_CHECK(m1[i].failed_task.net == m2[i].failed_task.net);
        CT_CHECK(m1[i].owned_idx == m2[i].owned_idx);
        CT_CHECK(m1[i].score == m2[i].score);
    }
    // Selective: the move rips exactly the obstructing SEAL route.
    CT_CHECK(m1[0].blocker_net == 0);
    CT_CHECK(m1[0].owned_idx.size() == 1);
}

CT_TEST(branch_better_never_prefers_short_unconnected) {
    // Global connectivity dominates vias/length (issue #19): fewer
    // remaining (unconnected) tasks wins even when longer / more vias.
    BranchResult short_bad, long_good;
    short_bad.evaluated = true;
    short_bad.connected_tasks = 1;  // local-only; ignored by ranking
    short_bad.remaining_task_ids = {1};
    short_bad.newly_connected_global = 0;
    short_bad.disrupted_routes = 0;
    short_bad.length_nm = 1000;
    short_bad.via_count = 0;
    long_good.evaluated = true;
    long_good.connected_tasks = 2;  // local-only; ignored by ranking
    long_good.remaining_task_ids = {};
    long_good.newly_connected_global = 1;
    long_good.disrupted_routes = 1;
    long_good.length_nm = 100000000;
    long_good.via_count = 10;
    // 0.1% shorter + one unconnected pad must NEVER beat full connectivity.
    CT_CHECK(branch_better(long_good, short_bad));
    CT_CHECK(!branch_better(short_bad, long_good));
}

CT_TEST(branch_better_equal_connectivity_prefers_less_disruption) {
    // Issue #19 regression: one branch rips 1 route, another rips 3; both
    // connect the same previously-failed task (same global remaining). The
    // less disruptive branch must win even though the big branch did more
    // local work (higher connected_tasks).
    BranchResult small, big;
    small.evaluated = true;
    small.connected_tasks = 2;  // failed + 1 ripped
    small.remaining_task_ids = {6};
    small.global_connected_tasks = 5;
    small.newly_connected_global = 1;
    small.disrupted_routes = 1;
    small.via_count = 2;
    small.length_nm = 5000;
    big.evaluated = true;
    big.connected_tasks = 4;  // failed + 3 ripped: more local work, same global gain
    big.remaining_task_ids = {6};
    big.global_connected_tasks = 5;
    big.newly_connected_global = 1;
    big.disrupted_routes = 3;
    big.via_count = 2;
    big.length_nm = 5000;
    CT_CHECK(branch_better(small, big));
    CT_CHECK(!branch_better(big, small));
}

CT_TEST(branch_better_does_not_reward_reconnected_ripped) {
    // Newly-connected previously-unrouted count dominates local churn:
    // a branch connecting a genuinely new task beats one that only
    // reconnects ripped copper, at equal remaining... and when remaining is
    // equal the newly_global field breaks the tie before disruption.
    BranchResult reconnect_only, new_progress;
    reconnect_only.evaluated = true;
    reconnect_only.connected_tasks = 3;
    reconnect_only.remaining_task_ids = {5};
    reconnect_only.newly_connected_global = 0;
    reconnect_only.disrupted_routes = 2;
    reconnect_only.via_count = 1;
    reconnect_only.length_nm = 1000;
    new_progress.evaluated = true;
    new_progress.connected_tasks = 1;
    new_progress.remaining_task_ids = {5};
    new_progress.newly_connected_global = 1;
    new_progress.disrupted_routes = 1;
    new_progress.via_count = 5;
    new_progress.length_nm = 9000;
    CT_CHECK(branch_better(new_progress, reconnect_only));
    CT_CHECK(!branch_better(reconnect_only, new_progress));
}

CT_TEST(recovery_tasks_routed_never_exceeds_total) {
    // Recovery must not double-count rerouted ripped tasks (issue #19).
    RouteReport rep = run_board(trap_board(), 1, /*ripup=*/true);
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.stats.tasks_routed <= rep.stats.tasks_total);
    CT_CHECK(rep.stats.tasks_routed == rep.stats.tasks_total);
    RouteReport rep4 = run_board(trap_board(), 4, /*ripup=*/true);
    CT_CHECK(rep4.stats.tasks_routed <= rep4.stats.tasks_total);
}

CT_TEST(modes_escalate_budgets_widen) {
    CT_CHECK(recovery_mode_for_generation(0) == RecoveryMode::FAST);
    CT_CHECK(recovery_mode_for_generation(2) == RecoveryMode::RECOVERY);
    CT_CHECK(recovery_mode_for_generation(5) == RecoveryMode::EXHAUSTIVE_LOCAL);
    CT_CHECK(recovery_mode_name(RecoveryMode::FAST) == "FAST");
    CT_CHECK(recovery_mode_name(RecoveryMode::EXHAUSTIVE_LOCAL) == "EXHAUSTIVE_LOCAL_RECOVERY");
    AStarConfig base;
    base.max_expansions = 1000;
    CT_CHECK(astar_config_for_mode(base, RecoveryMode::FAST).max_expansions == 1000);
    CT_CHECK(astar_config_for_mode(base, RecoveryMode::RECOVERY).max_expansions == 4000);
    CT_CHECK(astar_config_for_mode(base, RecoveryMode::EXHAUSTIVE_LOCAL).max_expansions >= 16000);
    CT_CHECK(branch_width_for_generation(0) <= branch_width_for_generation(1));
    CT_CHECK(branch_width_for_generation(1) <= branch_width_for_generation(5));
    CT_CHECK(max_rip_breadth_for_generation(0) <= max_rip_breadth_for_generation(5));
}

CT_TEST(ripup_reroute_reaches_full_connectivity) {
    // 3+4+5. rip appropriate geometry, reroute to a different allocation,
    // connect all nets; 6. verifier passes (checked in the next test too).
    RouteReport rep = run_board(trap_board(), 1, /*ripup=*/true);
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.connected_terminals == rep.total_terminals);
    CT_CHECK(rep.recovery.generations >= 1);
    CT_CHECK(rep.recovery.ripups >= 1);
    CT_CHECK(!rep.recovery.modes_attempted.empty());
    CT_CHECK(rep.recovery.modes_attempted[0] == "FAST");
    CT_CHECK(rep.result_category == "COMPLETE");
    CT_CHECK(rep.state_hash.to_hex().size() == 32);
}

CT_TEST(recovered_geometry_differs_and_verifies) {
    Board b = trap_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = 1;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    const Board& routed = engine.committed();
    // TRAPPED owns the straight slot; SEAL took a multi-segment detour.
    int seal_traces = 0, trapped_traces = 0;
    for (const auto& t : routed.traces) {
        if (t.net == 0) ++seal_traces;
        if (t.net == 1) ++trapped_traces;
    }
    CT_CHECK(trapped_traces == 1);  // straight vertical through the slot
    CT_CHECK(seal_traces > 1);      // ripped straight, rerouted around
    CT_CHECK(verify_ok(routed));    // 6. verifier passes
}

CT_TEST(recovery_deterministic_across_thread_counts) {
    RouteReport r1 = run_board(trap_board(), 1, true);
    RouteReport r4 = run_board(trap_board(), 4, true);
    CT_CHECK(r1.status == "COMPLETE");
    CT_CHECK(r4.status == "COMPLETE");
    CT_CHECK(r1.board_hash == r4.board_hash);
    CT_CHECK(r1.state_hash.to_hex() == r4.state_hash.to_hex());
    CT_CHECK(r1.recovery.generations == r4.recovery.generations);
    CT_CHECK(r1.recovery.ripups == r4.recovery.ripups);
}

CT_TEST(frontier_diag_recorded_for_unreachable) {
    RouteReport rep = run_board(trap_board(), 1, /*ripup=*/false);
    const RouteFailure* f = find_failure(rep, "TRAPPED");
    CT_CHECK(f != nullptr);
    CT_CHECK(f->has_frontier);
    CT_CHECK(!f->blockers.empty());
    bool names_seal = false;
    for (const auto& bl : f->blockers)
        if (bl.find("SEAL") != std::string::npos) names_seal = true;
    CT_CHECK(names_seal);
}

CT_TEST(fine_pitch_escape_bundle_reconsidered_not_trashed) {
    // Escape stubs carry extra protection: on a fine-pitch board recovery
    // must not rip them speculatively, and committed copper stays legal.
    JsonBoardImporter importer;
    ImportResult ir = importer.import_file(std::string(FIXTURE_DIR) + "/bga_4x4.json");
    Board b = std::move(ir.board);
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = 2;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(verify_legal(engine.committed()));
    CT_CHECK(rep.state_hash.to_hex().size() == 32);
    CT_CHECK(!rep.result_category.empty());
    // Recovery section is always present for agents, even at zero generations.
    CT_CHECK(rep.recovery.generations >= 0);
}

CT_TEST(impossible_board_reports_budget_category_honestly) {
    // No false success: an impassable wall reports the budget-exhausted (or
    // unroutable-under-constraints) category with evidence, never COMPLETE.
    JsonBoardImporter importer;
    ImportResult ir = importer.import_file(std::string(FIXTURE_DIR) + "/blocked_impossible.json");
    Board b = std::move(ir.board);
    RouteReport rep = run_board(std::move(b), 2, true);
    CT_CHECK(!(rep.status == "COMPLETE"));
    CT_CHECK(rep.result_category == "SEARCH_BUDGET_EXHAUSTED_WITH_UNROUTED_CONNECTIONS" ||
             rep.result_category == "UNROUTABLE_UNDER_CONFIGURED_CONSTRAINTS_AND_BUDGET");
    CT_CHECK(!rep.failures.empty());
    CT_CHECK(!rep.failures[0].blockers.empty());
}

// ---- Issue #18: deterministic single-thread transposition + exact state ----

CT_TEST(transposition_same_count_different_set_no_alias) {
    // Identical copper + same remaining COUNT but different unfinished sets
    // must hash differently and must not mutually prune.
    Board b = trap_board();
    ConnectionTask t0{0, b.nets[0].terminals[0], b.nets[0].terminals[1], 0, 0};
    ConnectionTask t1{1, b.nets[1].terminals[0], b.nets[1].terminals[1], 1, 0};
    std::vector<ConnectionTask> tasks = {t0, t1};
    StateHash128 h0 = state_hash128(b, tasks, {0});
    StateHash128 h1 = state_hash128(b, tasks, {1});
    CT_CHECK(!(h0 == h1));  // same count (1), different sets: no alias
    TranspositionTable tt;
    tt.record(h0, 1);
    CT_CHECK(!tt.should_prune(h1, 1));  // different state: keep
    CT_CHECK(tt.should_prune(h0, 1));   // identical state: prune
}

CT_TEST(branch_result_carries_exact_remaining_ids) {
    // reroute_branch must report the exact unfinished set
    // ((gen_remaining union to_route) minus newly_done, sorted) and hash
    // exactly that set, never an empty placeholder.
    Board b = trap_board();
    ConnectionTask t0{0, b.nets[0].terminals[0], b.nets[0].terminals[1], 0, 1.0};
    ConnectionTask t1{1, b.nets[1].terminals[0], b.nets[1].terminals[1], 1, 2.0};
    std::vector<ConnectionTask> tasks = {t0, t1};
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    std::vector<Corridor> corridors = {probable_corridor(b, r, tasks[0], ctx),
                                       probable_corridor(b, r, tasks[1], ctx)};
    std::vector<double> layer_mult = {1.0};
    AStarConfig cfg;
    CongestionMap congestion;
    congestion.init(b);
    RipupMove m;
    m.failed_pos = 0;
    m.failed_task = t1;
    std::vector<int> gen_remaining = {1};
    std::vector<int> to_route = {0, 1};
    BranchResult out = reroute_branch(b, {}, {}, {}, to_route, 1, tasks, gen_remaining,
                                      corridors, r, ctx, layer_mult, cfg, congestion,
                                      "FAST", m);
    CT_CHECK(out.evaluated);
    std::set<int> done(out.newly_done.begin(), out.newly_done.end());
    std::set<int> uni(gen_remaining.begin(), gen_remaining.end());
    for (int ti : to_route) uni.insert(ti);
    std::vector<int> expect;
    for (int ti : uni)
        if (!done.count(ti)) expect.push_back(ti);
    std::sort(expect.begin(), expect.end());
    CT_CHECK(out.remaining_task_ids == expect);
    CT_CHECK(out.hash == state_hash128(out.board, tasks, out.remaining_task_ids));
    if (!out.remaining_task_ids.empty())
        CT_CHECK(!(out.hash == state_hash128(out.board, tasks, {})));
}

CT_TEST(transposition_deterministic_across_thread_counts) {
    // TT is arbiter-serial in move-index order: threads 1/2/4 yield identical
    // copper, state hashes and transposition accounting.
    RouteReport r1 = run_board(trap_board(), 1, true);
    RouteReport r2 = run_board(trap_board(), 2, true);
    RouteReport r4 = run_board(trap_board(), 4, true);
    CT_CHECK(r1.status == "COMPLETE");
    CT_CHECK(r2.status == "COMPLETE");
    CT_CHECK(r4.status == "COMPLETE");
    CT_CHECK(r1.board_hash == r2.board_hash);
    CT_CHECK(r1.board_hash == r4.board_hash);
    CT_CHECK(r1.state_hash.to_hex() == r2.state_hash.to_hex());
    CT_CHECK(r1.state_hash.to_hex() == r4.state_hash.to_hex());
    CT_CHECK(r1.recovery.generations == r2.recovery.generations);
    CT_CHECK(r1.recovery.generations == r4.recovery.generations);
    CT_CHECK(r1.recovery.transposition_hits == r2.recovery.transposition_hits);
    CT_CHECK(r1.recovery.transposition_hits == r4.recovery.transposition_hits);
    CT_CHECK(r1.recovery.branches_pruned == r4.recovery.branches_pruned);
}

CT_TEST(speculative_branches_concurrent_no_race) {
    // The branch worker body performs no shared mutation: 8 threads x 4
    // distinct branches must match serial results exactly (no race/crash).
    Board b = trap_board();
    ConnectionTask t0{0, b.nets[0].terminals[0], b.nets[0].terminals[1], 0, 1.0};
    ConnectionTask t1{1, b.nets[1].terminals[0], b.nets[1].terminals[1], 1, 2.0};
    std::vector<ConnectionTask> tasks = {t0, t1};
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    std::vector<Corridor> corridors = {probable_corridor(b, r, tasks[0], ctx),
                                       probable_corridor(b, r, tasks[1], ctx)};
    std::vector<double> layer_mult = {1.0};
    AStarConfig cfg;
    CongestionMap congestion;
    congestion.init(b);
    const std::vector<std::vector<int>> to_routes = {{1}, {0, 1}, {0}, {1}};
    const std::vector<int> failed = {1, 1, 0, 1};
    const std::vector<int> gen_remaining = {0, 1};
    std::vector<BranchResult> ref(4);
    for (int i = 0; i < 4; ++i) {
        RipupMove m;
        m.failed_task = tasks[failed[i]];
        ref[i] = reroute_branch(b, {}, {}, {}, to_routes[i], failed[i], tasks,
                                gen_remaining, corridors, r, ctx, layer_mult, cfg,
                                congestion, "FAST", m);
    }
    std::vector<std::vector<BranchResult>> par(8, std::vector<BranchResult>(4));
    std::vector<std::thread> pool;
    for (int w = 0; w < 8; ++w)
        pool.emplace_back([&, w] {
            for (int i = 0; i < 4; ++i) {
                RipupMove m;
                m.failed_task = tasks[failed[i]];
                par[w][i] = reroute_branch(b, {}, {}, {}, to_routes[i], failed[i],
                                           tasks, gen_remaining, corridors, r, ctx,
                                           layer_mult, cfg, congestion, "FAST", m);
            }
        });
    for (auto& th : pool) th.join();
    for (int w = 0; w < 8; ++w)
        for (int i = 0; i < 4; ++i) {
            CT_CHECK(par[w][i].hash == ref[i].hash);
            CT_CHECK(par[w][i].connected_tasks == ref[i].connected_tasks);
            CT_CHECK(par[w][i].newly_done == ref[i].newly_done);
            CT_CHECK(par[w][i].remaining_task_ids == ref[i].remaining_task_ids);
        }
}

// ---- Issue #21: frontier-evidence blocker attribution ----

CT_TEST(frontier_stats_record_real_blocker) {
    Board b = trap_board();
    b.traces.push_back(
        {0, 0, {mm_to_nm(2.0), mm_to_nm(9.0)}, {mm_to_nm(18.0), mm_to_nm(9.0)}, mm_to_nm(0.2)});
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    ConnectionTask trapped{1, b.nets[1].terminals[0], b.nets[1].terminals[1], 1, 2.0};
    CongestionMap congestion;
    congestion.init(b);
    ReservationSet reservations;
    CandidateRoute cand = route_candidate_task(b, r, trapped, 0, 2.0, ctx, {1.0},
                                               AStarConfig{}, congestion, reservations);
    CT_CHECK(!cand.found);  // SEAL seals the slot: TRAPPED must fail
    CT_CHECK(!cand.frontier_blockers.empty());
    // The real (rippable) blocker must be among the bounded evidence, even
    // though static keepouts may legitimately outrank it by raw count.
    {
        bool found_seal = false;
        for (const auto& f : cand.frontier_blockers)
            if (f.blocker_net == 0) found_seal = true;
        CT_CHECK(found_seal);
    }
    CT_CHECK((int)cand.frontier_blockers.size() <= kMaxFrontierStats);
    FrontierDiag d = diagnose_task(trapped, cand);
    CT_CHECK(!d.top_blockers.empty());
    {
        bool found_seal = false;
        for (const auto& f : d.top_blockers)
            if (f.blocker_net == 0) found_seal = true;
        CT_CHECK(found_seal);
    }
    JsonValue j = d.to_json();
    CT_CHECK(j["top_blockers"].as_array().size() > 0);
}

CT_TEST(frontier_outranks_irrelevant_bbox_copper) {
    // Huge irrelevant trace inside the bbox must not outrank the real
    // frontier blocker (net 0).
    Board b = trap_board();
    NetInfo irr = make_net(2, "IRRELEVANT");
    irr.has_current = true;
    irr.current_a = 0.1;
    b.nets.push_back(irr);
    b.traces.push_back(
        {0, 0, {mm_to_nm(2.0), mm_to_nm(9.0)}, {mm_to_nm(18.0), mm_to_nm(9.0)}, mm_to_nm(0.2)});
    // Wide irrelevant slab covering much of the TRAPPED corridor.
    b.traces.push_back(
        {2, 0, {mm_to_nm(9.0), mm_to_nm(5.0)}, {mm_to_nm(11.0), mm_to_nm(14.0)}, mm_to_nm(1.0)});
    ConnectionTask trapped{1, b.nets[1].terminals[0], b.nets[1].terminals[1], 1, 2.0};
    CandidateRoute last;
    last.task = trapped;
    last.fail_reason = "unreachable";
    last.closest_node = 0;
    FrontierBlockerStat f;
    f.blocker_net = 0;
    f.kind = "trace";
    f.desc = "trace:net=SEAL";
    f.layer = 0;
    f.pos = {mm_to_nm(10.0), mm_to_nm(9.0)};
    f.count = 50;
    last.frontier_blockers.push_back(f);
    std::vector<BlockerHit> hits =
        attribute_blockers_detailed(b, trapped, mm_to_nm(0.2), &last);
    CT_CHECK(!hits.empty());
    CT_CHECK(hits[0].net == 0);  // real frontier evidence first
}

CT_TEST(keepout_only_failure_nonrippable) {
    // Horizontal keepout wall seals the slot; no traces to rip.
    Board b;
    b.source_format = "test";
    b.width_nm = mm_to_nm(20.0);
    b.height_nm = mm_to_nm(20.0);
    b.layers.push_back({0, "Top"});
    NetInfo t = make_net(1, "TRAPPED");
    t.has_current = true;
    t.current_a = 0.1;
    b.nets.push_back(t);
    add_terminal(b, 1, 10.0, 5.0);
    add_terminal(b, 1, 10.0, 15.0);
    Keepout k;
    k.rect = {mm_to_nm(0.0), mm_to_nm(9.0), mm_to_nm(20.0), mm_to_nm(9.5)};
    k.layer = kAllLayers;
    k.reason = "wall";
    b.keepouts.push_back(k);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    ConnectionTask task{1, b.nets[0].terminals[0], b.nets[0].terminals[1], 0, 1.0};
    CongestionMap congestion;
    congestion.init(b);
    ReservationSet reservations;
    CandidateRoute cand = route_candidate_task(b, r, task, 0, 1.0, ctx, {1.0},
                                               AStarConfig{}, congestion, reservations);
    CT_CHECK(!cand.found);
    CT_CHECK(!cand.frontier_blockers.empty());
    // Top rejection must be the keepout (nothing to rip), not a net.
    CT_CHECK(cand.frontier_blockers[0].blocker_net == -1);
    std::vector<BlockerHit> hits =
        attribute_blockers_detailed(b, task, mm_to_nm(0.2), &cand);
    bool has_keepout = false;
    for (const auto& h : hits)
        if (h.kind == "keepout" && h.net == -1) has_keepout = true;
    CT_CHECK(has_keepout);
    // Dependency graph carries only non-rippable edges -> no rip moves.
    std::vector<ConnectionTask> tasks = {task};
    std::map<std::pair<NetId, std::pair<TermId, TermId>>, CandidateRoute> last;
    last[{task.net, {std::min(task.a, task.b), std::max(task.a, task.b)}}] = cand;
    DependencyGraph g = build_dependency_graph(b, r, ctx, tasks, {0}, last);
    for (const auto& e : g.edges) CT_CHECK(e.blocker_net == -1);
    HistoryHeuristic hh;
    auto moves = generate_ripup_moves(g.failed, g, {}, hh, "", RecoveryMode::RECOVERY, 4, 2);
    CT_CHECK(moves.empty());
}

CT_TEST(multilayer_frontier_reports_correct_layer) {
    // Blocker lives on layer 1 (same as the task); the graph must report
    // layer 1, not layer 0.
    Board b;
    b.source_format = "test";
    b.width_nm = mm_to_nm(20.0);
    b.height_nm = mm_to_nm(20.0);
    b.layers.push_back({0, "Top"});
    b.layers.push_back({1, "Bottom"});
    NetInfo blk = make_net(0, "BLOCKER");
    blk.has_current = true;
    blk.current_a = 0.1;
    NetInfo tgt = make_net(1, "TARGET");
    tgt.has_current = true;
    tgt.current_a = 0.1;
    b.nets.push_back(blk);
    b.nets.push_back(tgt);
    TermId ta = add_terminal(b, 1, 2.0, 10.0, 1);
    TermId tb = add_terminal(b, 1, 18.0, 10.0, 1);
    (void)ta;
    (void)tb;
    TraceSeg wall{0, 1, {mm_to_nm(10.0), mm_to_nm(0.0)}, {mm_to_nm(10.0), mm_to_nm(20.0)},
                  mm_to_nm(0.2)};
    b.traces.push_back(wall);
    // Full-height keepouts left/right force all detours through the wall.
    auto wall_ko = [&](double x1, double x2) {
        Keepout k;
        k.rect = {mm_to_nm(x1), mm_to_nm(0.0), mm_to_nm(x2), mm_to_nm(20.0)};
        k.layer = kAllLayers;
        k.reason = "side";
        b.keepouts.push_back(k);
    };
    wall_ko(0.0, 1.5);
    wall_ko(18.5, 20.0);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    const Terminal* pa = b.find_terminal(b.nets[1].terminals[0]);
    const Terminal* pb = b.find_terminal(b.nets[1].terminals[1]);
    SparseRoutingGraph g = SparseRoutingGraph::build(
        b, r, 1, pa->pos, pb->pos, 1, 1, mm_to_nm(0.2), ctx);
    CT_CHECK(!g.frontier_stats().empty());
    // The layer-1 trace blocker must be present with its correct layer, even
    // if large static keepouts outrank it by raw rejection count.
    {
        bool found = false;
        for (const auto& s : g.frontier_stats())
            if (s.blocker_net == 0 && s.layer == 1) found = true;
        CT_CHECK(found);
    }
    CT_CHECK((int)g.frontier_stats().size() <= kMaxFrontierStats);
}

CT_TEST(frontier_expanded_filter_matches_flat_aggregate) {
    // #23 equivalence pin: aggregating the per-node probe ledger in place
    // behind an expanded mask must equal aggregating exactly those nodes'
    // probes (the previous flat-copy implementation), and an all-ones mask
    // must equal the construction-wide stats. Guards the copy-free
    // aggregation against drift.
    Board b;
    b.source_format = "test";
    b.width_nm = mm_to_nm(20.0);
    b.height_nm = mm_to_nm(20.0);
    b.layers.push_back({0, "Top"});
    b.layers.push_back({1, "Bottom"});
    NetInfo blk = make_net(0, "BLOCKER");
    blk.has_current = true;
    blk.current_a = 0.1;
    NetInfo tgt = make_net(1, "TARGET");
    tgt.has_current = true;
    tgt.current_a = 0.1;
    b.nets.push_back(blk);
    b.nets.push_back(tgt);
    add_terminal(b, 1, 2.0, 10.0, 1);
    add_terminal(b, 1, 18.0, 10.0, 1);
    b.traces.push_back({0, 1, {mm_to_nm(10.0), mm_to_nm(0.0)},
                        {mm_to_nm(10.0), mm_to_nm(20.0)}, mm_to_nm(0.2)});
    b.traces.push_back({0, 0, {mm_to_nm(2.0), mm_to_nm(4.0)},
                        {mm_to_nm(18.0), mm_to_nm(4.0)}, mm_to_nm(0.2)});
    b.traces.push_back({0, 0, {mm_to_nm(2.0), mm_to_nm(16.0)},
                        {mm_to_nm(18.0), mm_to_nm(16.0)}, mm_to_nm(0.2)});
    auto wall_ko = [&](double x1, double x2) {
        Keepout k;
        k.rect = {mm_to_nm(x1), mm_to_nm(0.0), mm_to_nm(x2), mm_to_nm(20.0)};
        k.layer = kAllLayers;
        k.reason = "side";
        b.keepouts.push_back(k);
    };
    wall_ko(0.0, 1.5);
    wall_ko(18.5, 20.0);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    const Terminal* pa = b.find_terminal(b.nets[1].terminals[0]);
    const Terminal* pb = b.find_terminal(b.nets[1].terminals[1]);
    SparseRoutingGraph g = SparseRoutingGraph::build(b, r, 1, pa->pos, pb->pos, 1, 1,
                                                     mm_to_nm(0.2), ctx);
    const std::size_t n = g.nodes().size();
    CT_CHECK(n > 0);
    // The equivalence is only meaningful with a non-empty probe ledger.
    std::size_t probe_total = 0;
    for (std::size_t i = 0; i < n; ++i)
        probe_total += g.rejected(static_cast<int>(i)).size();
    CT_CHECK(probe_total > 0);

    auto same_stats = [](const std::vector<GraphFrontierStat>& a,
                         const std::vector<GraphFrontierStat>& c) {
        if (a.size() != c.size()) return false;
        for (std::size_t i = 0; i < a.size(); ++i) {
            if (a[i].blocker_net != c[i].blocker_net) return false;
            if (a[i].kind != c[i].kind) return false;
            if (a[i].desc != c[i].desc) return false;
            if (a[i].layer != c[i].layer) return false;
            if (a[i].pos.x != c[i].pos.x || a[i].pos.y != c[i].pos.y) return false;
            if (a[i].count != c[i].count) return false;
        }
        return true;
    };

    // All expanded == construction-wide stats.
    std::vector<char> all(n, 1);
    CT_CHECK(same_stats(g.frontier_stats_for_expanded(all), g.frontier_stats()));

    // Partial mask == flat concatenation of exactly those nodes' probes.
    // Pick probe-carrying nodes (every other one) so the mask is a genuine
    // subset and the flat reference is non-empty.
    std::vector<std::size_t> with_probes;
    for (std::size_t i = 0; i < n; ++i)
        if (!g.rejected(static_cast<int>(i)).empty()) with_probes.push_back(i);
    CT_CHECK(!with_probes.empty());
    std::vector<char> mask(n, 0);
    for (std::size_t k = 0; k < with_probes.size(); k += 2) mask[with_probes[k]] = 1;
    std::vector<SparseRejectedProbe> flat;
    for (std::size_t i = 0; i < n; ++i)
        if (mask[i])
            for (const auto& p : g.rejected(static_cast<int>(i))) flat.push_back(p);
    CT_CHECK(!flat.empty());
    CT_CHECK(same_stats(g.frontier_stats_for_expanded(mask),
                        SparseRoutingGraph::aggregate_probes(flat)));
}

CT_TEST(frontier_memory_bounded_top_n) {
    // Even on a dense board the evidence stays bounded.
    JsonBoardImporter importer;
    ImportResult ir = importer.import_file(std::string(FIXTURE_DIR) + "/bga_4x4.json");
    Board b = std::move(ir.board);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    const Terminal* pa = b.find_terminal(b.nets[0].terminals[0]);
    const Terminal* pb = b.find_terminal(b.nets[0].terminals[1]);
    if (!pa || !pb) return;  // fixture shape changed: nothing to assert
    SparseRoutingGraph g = SparseRoutingGraph::build(b, r, b.nets[0].id, pa->pos, pb->pos,
                                                     pa->layer, pb->layer, mm_to_nm(0.2), ctx);
    CT_CHECK((int)g.frontier_stats().size() <= kMaxFrontierStats);
}

// ---- Issue #20: bounded multi-blocker rip-up sets ----

namespace {
DependencyGraph dual_blocker_graph() {
    DependencyGraph g;
    ConnectionTask failed{9, 100, 101, 0, 5.0};
    g.failed.push_back(failed);
    DependencyEdge e0, e1;
    e0.failed_pos = 0;
    e0.failed_net = 9;
    e0.blocker_net = 0;
    e0.blocker_desc = "trace:net=A";
    e0.weight = 100.0;
    e1.failed_pos = 0;
    e1.failed_net = 9;
    e1.blocker_net = 1;
    e1.blocker_desc = "trace:net=B";
    e1.weight = 90.0;
    g.edges.push_back(e0);
    g.edges.push_back(e1);
    return g;
}

std::vector<OwnedRoute> dual_blocker_owned() {
    std::vector<OwnedRoute> owned;
    ConnectionTask ta{0, 10, 11, 0, 1.0};
    ConnectionTask tb{1, 20, 21, 1, 1.0};
    OwnedRoute oa, ob;
    oa.task = ta;
    oa.task_pos = 0;
    oa.protection = 2.0;
    ob.task = tb;
    ob.task_pos = 1;
    ob.protection = 2.0;
    owned.push_back(oa);
    owned.push_back(ob);
    return owned;
}
}  // namespace

CT_TEST(multiblocker_fast_stays_single) {
    DependencyGraph g = dual_blocker_graph();
    std::vector<OwnedRoute> owned = dual_blocker_owned();
    HistoryHeuristic hh;
    auto moves =
        generate_ripup_moves(g.failed, g, owned, hh, "", RecoveryMode::FAST, 8, 2);
    CT_CHECK(!moves.empty());
    for (const auto& m : moves) CT_CHECK(m.owned_idx.size() == 1);
}

CT_TEST(multiblocker_recovery_generates_combo) {
    DependencyGraph g = dual_blocker_graph();
    std::vector<OwnedRoute> owned = dual_blocker_owned();
    HistoryHeuristic hh;
    auto m1 =
        generate_ripup_moves(g.failed, g, owned, hh, "", RecoveryMode::RECOVERY, 8, 2);
    auto m2 =
        generate_ripup_moves(g.failed, g, owned, hh, "", RecoveryMode::RECOVERY, 8, 2);
    CT_CHECK(!m1.empty());
    CT_CHECK(m1.size() == m2.size());  // deterministic
    bool found_combo = false;
    std::set<std::string> keys;
    for (std::size_t i = 0; i < m1.size(); ++i) {
        CT_CHECK(m1[i].owned_idx == m2[i].owned_idx);
        CT_CHECK(m1[i].score == m2[i].score);
        std::string k;
        for (int oi : m1[i].owned_idx) k += std::to_string(oi) + ",";
        std::string dk = std::to_string(m1[i].failed_pos) + ":" + k;
        CT_CHECK(keys.count(dk) == 0);  // deduplicated
        keys.insert(dk);
        if (m1[i].owned_idx.size() == 2) {
            std::set<int> nets;
            for (int oi : m1[i].owned_idx) nets.insert(owned[oi].task.net);
            if (nets.count(0) && nets.count(1)) found_combo = true;
        }
    }
    CT_CHECK(found_combo);  // A+B combined set exists
}

CT_TEST(multiblocker_exhaustive_widens_to_three) {
    CT_CHECK(max_blocker_nets_for_mode(RecoveryMode::FAST) == 1);
    CT_CHECK(max_blocker_nets_for_mode(RecoveryMode::RECOVERY) == 2);
    CT_CHECK(max_blocker_nets_for_mode(RecoveryMode::EXHAUSTIVE_LOCAL) == 3);
    DependencyGraph g;
    ConnectionTask failed{9, 100, 101, 0, 5.0};
    g.failed.push_back(failed);
    for (int n = 0; n < 3; ++n) {
        DependencyEdge e;
        e.failed_pos = 0;
        e.failed_net = 9;
        e.blocker_net = n;
        e.blocker_desc = "trace:net=" + std::to_string(n);
        e.weight = 100.0 - n * 10.0;
        g.edges.push_back(e);
    }
    std::vector<OwnedRoute> owned;
    for (int n = 0; n < 3; ++n) {
        OwnedRoute o;
        o.task = ConnectionTask{n, 10 + n, 20 + n, n, 1.0};
        o.task_pos = n;
        o.protection = 1.0;
        owned.push_back(o);
    }
    HistoryHeuristic hh;
    auto rec = generate_ripup_moves(g.failed, g, owned, hh, "", RecoveryMode::RECOVERY,
                                    16, 4);
    auto exh = generate_ripup_moves(g.failed, g, owned, hh, "",
                                    RecoveryMode::EXHAUSTIVE_LOCAL, 16, 4);
    bool rec_has3 = false, exh_has3 = false;
    for (const auto& m : rec)
        if (m.owned_idx.size() == 3) rec_has3 = true;
    for (const auto& m : exh)
        if (m.owned_idx.size() == 3) exh_has3 = true;
    CT_CHECK(!rec_has3);  // RECOVERY capped at 2 nets
    CT_CHECK(exh_has3);   // EXHAUSTIVE reaches 3 nets
}

CT_TEST(multiblocker_never_rips_fixed) {
    DependencyGraph g = dual_blocker_graph();
    std::vector<OwnedRoute> owned = dual_blocker_owned();
    owned[0].protection = kFixedProtection;  // fixed: must never be ripped
    HistoryHeuristic hh;
    for (auto mode : {RecoveryMode::FAST, RecoveryMode::RECOVERY,
                      RecoveryMode::EXHAUSTIVE_LOCAL}) {
        auto moves = generate_ripup_moves(g.failed, g, owned, hh, "", mode, 8, 4);
        for (const auto& m : moves)
            for (int oi : m.owned_idx) CT_CHECK(oi != 0);
    }
}

CT_TEST(multiblocker_single_rip_insufficient_combo_succeeds) {    // Dual-seal board: TRAPPED vertical must cross two parallel SEAL lines.
    // Ripping either seal alone still leaves the other crossing -> reroute
    // fails; ripping both opens the straight corridor -> reroute succeeds.
    // The move generator must propose that A+B set in RECOVERY.
    Board b;
    b.source_format = "test";
    b.width_nm = mm_to_nm(20.0);
    b.height_nm = mm_to_nm(20.0);
    b.layers.push_back({0, "Top"});
    NetInfo sa = make_net(0, "SEAL_A");
    sa.has_current = true;
    sa.current_a = 0.1;
    NetInfo sb = make_net(1, "SEAL_B");
    sb.has_current = true;
    sb.current_a = 0.1;
    NetInfo tr = make_net(2, "TRAPPED");
    tr.has_current = true;
    tr.current_a = 0.1;
    b.nets.push_back(sa);
    b.nets.push_back(sb);
    b.nets.push_back(tr);
    add_terminal(b, 0, 2.0, 8.0);
    add_terminal(b, 0, 18.0, 8.0);
    add_terminal(b, 1, 2.0, 10.0);
    add_terminal(b, 1, 18.0, 10.0);
    add_terminal(b, 2, 10.0, 5.0);
    add_terminal(b, 2, 10.0, 15.0);
    auto wall = [&](double x1, double y1, double x2, double y2, const char* reason) {
        Keepout k;
        k.rect = {mm_to_nm(x1), mm_to_nm(y1), mm_to_nm(x2), mm_to_nm(y2)};
        k.layer = kAllLayers;
        k.reason = reason;
        b.keepouts.push_back(k);
    };
    wall(8, 3, 9, 7.6, "u_left");
    wall(11, 3, 12, 7.6, "u_right");
    wall(8, 3, 12, 4, "u_bottom");
    // Side walls above the seals force every detour through y=8/10 lines.
    wall(0, 7.0, 7.9, 11.0, "side_l");
    wall(12.1, 7.0, 20.0, 11.0, "side_r");
    // Commit both seals straight.
    b.traces.push_back(
        {0, 0, {mm_to_nm(2.0), mm_to_nm(8.0)}, {mm_to_nm(18.0), mm_to_nm(8.0)}, mm_to_nm(0.2)});
    b.traces.push_back(
        {1, 0, {mm_to_nm(2.0), mm_to_nm(10.0)}, {mm_to_nm(18.0), mm_to_nm(10.0)}, mm_to_nm(0.2)});
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    ConnectionTask t_trapped{2, b.nets[2].terminals[0], b.nets[2].terminals[1], 2, 2.0};
    CongestionMap congestion;
    congestion.init(b);
    ReservationSet reservations;
    // Single-rip boards: remove one seal, keep the other -> still blocked.
    for (int keep = 0; keep < 2; ++keep) {
        Board single = b;
        single.traces.erase(
            std::remove_if(single.traces.begin(), single.traces.end(),
                           [&](const TraceSeg& t) { return t.net == keep; }),
            single.traces.end());
        CandidateRoute c = route_candidate_task(single, r, t_trapped, 0, 2.0, ctx,
                                                {1.0}, AStarConfig{}, congestion,
                                                reservations);
        CT_CHECK(!c.found);
    }
    // Double-rip board: both seals gone -> straight corridor opens.
    {
        Board none = b;
        none.traces.clear();
        CandidateRoute c = route_candidate_task(none, r, t_trapped, 0, 2.0, ctx,
                                                {1.0}, AStarConfig{}, congestion,
                                                reservations);
        CT_CHECK(c.found);
    }
    // Move generator proposes the combined set in RECOVERY, not FAST.
    ConnectionTask t_sa{0, b.nets[0].terminals[0], b.nets[0].terminals[1], 0, 1.0};
    ConnectionTask t_sb{1, b.nets[1].terminals[0], b.nets[1].terminals[1], 1, 1.0};
    std::vector<ConnectionTask> tasks = {t_sa, t_sb, t_trapped};
    std::map<std::pair<NetId, std::pair<TermId, TermId>>, CandidateRoute> last;
    DependencyGraph g = build_dependency_graph(b, r, ctx, tasks, {2}, last);
    OwnedRoute oa, ob;
    oa.task = t_sa;
    oa.task_pos = 0;
    oa.traces = {b.traces[0]};
    oa.protection = 1.0;
    ob.task = t_sb;
    ob.task_pos = 1;
    ob.traces = {b.traces[1]};
    ob.protection = 1.0;
    HistoryHeuristic hh;
    auto fast = generate_ripup_moves(g.failed, g, {oa, ob}, hh, "",
                                     RecoveryMode::FAST, 8, 2);
    auto rec = generate_ripup_moves(g.failed, g, {oa, ob}, hh, "",
                                    RecoveryMode::RECOVERY, 8, 2);
    for (const auto& m : fast) CT_CHECK(m.owned_idx.size() == 1);
    bool combo = false;
    for (const auto& m : rec)
        if (m.owned_idx.size() == 2) combo = true;
    CT_CHECK(combo);
}

// ---- Issue #23: frontier evidence from the A* frontier, not construction ----

// ---- Issue #23: frontier evidence from the A* frontier, not construction ----

CT_TEST(frontier_ranks_reached_blocker_above_unreached) {
    // #21 regression (#23): many candidate edges hit decoy net A (DECOY
    // cluster above the seal) in regions A* never reaches, while the
    // reachable frontier is stopped by blocker B (SEAL straight across the
    // slot mouth). B must rank above A in the candidate evidence and in
    // blocker attribution; the construction-wide aggregate ranks A first
    // (which is exactly what the old impl forwarded).
    Board b;
    b.source_format = "test";
    b.width_nm = mm_to_nm(20.0);
    b.height_nm = mm_to_nm(20.0);
    b.layers.push_back({0, "Top"});
    NetInfo seal = make_net(0, "SEAL");
    seal.has_current = true;
    seal.current_a = 0.1;
    NetInfo tgt = make_net(1, "TARGET");
    tgt.has_current = true;
    tgt.current_a = 0.1;
    NetInfo decoy = make_net(2, "DECOY");
    decoy.has_current = true;
    decoy.current_a = 0.1;
    b.nets.push_back(seal);
    b.nets.push_back(tgt);
    b.nets.push_back(decoy);
    add_terminal(b, 0, 2.0, 9.0);
    add_terminal(b, 0, 18.0, 9.0);
    add_terminal(b, 1, 10.0, 5.0);
    add_terminal(b, 1, 10.0, 15.0);
    add_terminal(b, 2, 14.0, 12.0);
    add_terminal(b, 2, 14.0, 14.0);
    auto wall = [&](double x1, double y1, double x2, double y2, const char* reason) {
        Keepout k;
        k.rect = {mm_to_nm(x1), mm_to_nm(y1), mm_to_nm(x2), mm_to_nm(y2)};
        k.layer = kAllLayers;
        k.reason = reason;
        b.keepouts.push_back(k);
    };
    wall(8, 3, 9, 8.6, "u_left");
    wall(11, 3, 12, 8.6, "u_right");
    wall(8, 3, 12, 4, "u_bottom");
    // Reachable blocker: seals the slot mouth on the search side.
    b.traces.push_back(
        {0, 0, {mm_to_nm(2.0), mm_to_nm(9.0)}, {mm_to_nm(18.0), mm_to_nm(9.0)}, mm_to_nm(0.2)});
    // Unreached decoys: dense grid above the seal, inside the graph
    // corridor (so construction probes hit it often) but beyond the sealed
    // frontier (so A* never expands there).
    for (int i = 0; i < 3; ++i) {
        double y = 11.0 + i;
        b.traces.push_back({2, 0, {mm_to_nm(7.0), mm_to_nm(y)}, {mm_to_nm(13.0), mm_to_nm(y)},
                            mm_to_nm(0.3)});
    }
    for (int i = 0; i < 3; ++i) {
        double x = 9.0 + i;
        b.traces.push_back({2, 0, {mm_to_nm(x), mm_to_nm(10.5)}, {mm_to_nm(x), mm_to_nm(13.5)},
                            mm_to_nm(0.3)});
    }
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    ConnectionTask task{1, b.nets[1].terminals[0], b.nets[1].terminals[1], 1, 2.0};
    CongestionMap congestion;
    congestion.init(b);
    ReservationSet reservations;
    auto route_once = [&]() {
        return route_candidate_task(b, r, task, 0, 2.0, ctx, {1.0}, AStarConfig{},
                                    congestion, reservations);
    };
    CandidateRoute cand = route_once();
    CT_CHECK(!cand.found);  // SEAL seals the slot: TARGET must fail
    CT_CHECK(!cand.frontier_blockers.empty());
    CT_CHECK((int)cand.frontier_blockers.size() <= kMaxFrontierStats);
    auto rank_of = [&](const auto& v, NetId n) {
        for (std::size_t i = 0; i < v.size(); ++i)
            if (v[i].blocker_net == n) return (int)i;
        return 1 << 30;  // absent ranks last
    };
    // The reached blocker (SEAL, net 0) outranks the unreached decoy (net 2).
    CT_CHECK(rank_of(cand.frontier_blockers, 0) < rank_of(cand.frontier_blockers, 2));
    // End to end: dependency/rip-up attribution names SEAL first among
    // rippable (net) blockers. Keepout hits (net -1, nothing to rip) may
    // lead; what matters is SEAL outranks the DECOY copper.
    std::vector<BlockerHit> hits =
        attribute_blockers_detailed(b, task, mm_to_nm(0.2), &cand);
    CT_CHECK(!hits.empty());
    int first_rip = -1, rank0 = 1 << 30, rank2 = 1 << 30;
    for (std::size_t i = 0; i < hits.size(); ++i) {
        if (hits[i].net == 0) rank0 = std::min(rank0, (int)i);
        if (hits[i].net == 2) rank2 = std::min(rank2, (int)i);
        if (first_rip < 0 && hits[i].net >= 0) first_rip = (int)i;
    }
    CT_CHECK(first_rip >= 0 && hits[(std::size_t)first_rip].net == 0);
    CT_CHECK(rank0 < rank2);
    // The board really exercises the regression: construction-wide evidence
    // ranks the unreached decoy at least as high as the seal (old impl
    // forwarded exactly this and failed the ordering above).
    const Terminal* pa = b.find_terminal(b.nets[1].terminals[0]);
    const Terminal* pb = b.find_terminal(b.nets[1].terminals[1]);
    SparseRoutingGraph g =
        SparseRoutingGraph::build(b, r, 1, pa->pos, pb->pos, 0, 0, mm_to_nm(0.2), ctx);
    CT_CHECK(rank_of(g.frontier_stats(), 2) <= rank_of(g.frontier_stats(), 0));
    // Determinism: repeated routing yields identical frontier evidence.
    CandidateRoute again = route_once();
    CT_CHECK(again.frontier_blockers.size() == cand.frontier_blockers.size());
    for (std::size_t i = 0; i < cand.frontier_blockers.size(); ++i) {
        CT_CHECK(again.frontier_blockers[i].blocker_net == cand.frontier_blockers[i].blocker_net);
        CT_CHECK(again.frontier_blockers[i].kind == cand.frontier_blockers[i].kind);
        CT_CHECK(again.frontier_blockers[i].desc == cand.frontier_blockers[i].desc);
        CT_CHECK(again.frontier_blockers[i].layer == cand.frontier_blockers[i].layer);
        CT_CHECK(again.frontier_blockers[i].count == cand.frontier_blockers[i].count);
        CT_CHECK(again.frontier_blockers[i].pos.x == cand.frontier_blockers[i].pos.x);
        CT_CHECK(again.frontier_blockers[i].pos.y == cand.frontier_blockers[i].pos.y);
    }
}

int main() { return copperline::test::run_all_tests(); }
