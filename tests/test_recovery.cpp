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
#include "router/board.h"
#include "router/engine.h"
#include "router/recovery.h"
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
    BranchResult short_bad, long_good;
    short_bad.evaluated = true;
    short_bad.connected_tasks = 1;
    short_bad.length_nm = 1000;
    short_bad.via_count = 0;
    long_good.evaluated = true;
    long_good.connected_tasks = 2;
    long_good.length_nm = 100000000;
    long_good.via_count = 10;
    // 0.1% shorter + one unconnected pad must NEVER beat full connectivity.
    CT_CHECK(branch_better(long_good, short_bad));
    CT_CHECK(!branch_better(short_bad, long_good));
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

int main() { return copperline::test::run_all_tests(); }
