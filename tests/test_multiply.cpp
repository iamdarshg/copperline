// Issue #22 tests: bounded multi-ply high-level recovery search.
//
// A* keeps detailed geometry; only RipupMove/reroute outcomes are tree
// nodes. The beam search looks ahead 2-3 plies over immutable branch
// states, ranks leaves by the global board objective (connectivity-first,
// issue #19, with issue #9 impact as a late tie-break), commits only the
// first action of the best PV and replans. Timeout/node budgets fall back
// to one-ply. Determinism holds for equal seed/workers.
#include <algorithm>
#include <chrono>
#include <set>

#include "helpers.h"
#include "router/board.h"
#include "router/engine.h"
#include "router/recovery.h"
#include "router/verifier.h"

using namespace copperline;
using namespace copperline::test;

namespace {

// Single-layer U-enclosure (mirrors test_recovery trap_board).
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

RouteReport run_board(Board b, int threads, int depth, int beam,
                      std::int64_t multiply_nodes = 64) {
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = threads;
    opt.enable_ripup = true;
    opt.recovery_depth = depth;
    opt.recovery_beam = beam;
    opt.max_multiply_nodes = multiply_nodes;
    RouterEngine engine(std::move(b), std::move(r), opt);
    return engine.run();
}

bool verify_ok(const Board& board) {
    BoardVerifier v;
    ElectricalContext ctx;
    RuleResolver r = RuleResolver::defaults_for(board);
    return v.verify(board, r, ctx).ok;
}

BranchResult leaf(int remaining_count, int newly_global, int disrupted,
                  Coord len, int vias, double impact, bool has_impact,
                  const std::string& hash_lo) {
    BranchResult b;
    b.evaluated = true;
    for (int i = 0; i < remaining_count; ++i) b.remaining_task_ids.push_back(100 + i);
    b.newly_connected_global = newly_global;
    b.disrupted_routes = disrupted;
    b.length_nm = len;
    b.via_count = vias;
    b.impact_obstruction = impact;
    b.has_impact = has_impact;
    // Stable hash from the tag string.
    Board tmp;
    tmp.source_format = "test";
    tmp.width_nm = mm_to_nm(20.0);
    tmp.height_nm = mm_to_nm(20.0);
    tmp.layers.push_back({0, "Top"});
    ConnectionTask t0{0, 0, 1, 0, 0};
    std::vector<ConnectionTask> tasks = {t0};
    StateHash128 h = state_hash128(tmp, tasks, {});
    // Derive distinct hashes deterministically from the tag.
    for (char c : hash_lo) {
        h.lo ^= static_cast<std::uint64_t>(c) * 1099511628211ULL;
        h.lo *= 1099511628211ULL;
    }
    b.hash = h;
    return b;
}

}  // namespace

CT_TEST(multiply_greedy_second_ranked_wins_in_two_plies) {
    // Greedy one-ply ranking prefers A (less disruption, shorter), but A's
    // only child dead-ends while B's child completes. Leaf ranking must
    // prefer B's PV by connectivity-first, so the search commits B first.
    BranchResult first_a = leaf(1, 0, 1, 5000, 1, 0.1, true, "firstA");
    BranchResult first_b = leaf(1, 0, 1, 9000, 2, 0.2, true, "firstB");
    // One-ply: A wins (shorter, fewer vias at equal remaining).
    CT_CHECK(branch_better(first_a, first_b));
    CT_CHECK(!branch_better(first_b, first_a));
    // Two-ply leaves: A-child still strands one task, B-child completes.
    BranchResult leaf_a = leaf(1, 0, 2, 4000, 0, 0.0, true, "leafA");
    BranchResult leaf_b = leaf(0, 1, 2, 50000, 6, 0.9, true, "leafB");
    // Connectivity-first: complete-but-ugly beats short-but-stranded even
    // though leaf_a is shorter, via-free and lower-obstruction.
    CT_CHECK(branch_better(leaf_b, leaf_a));
    CT_CHECK(!branch_better(leaf_a, leaf_b));
}

CT_TEST(multiply_depth1_reproduces_one_ply) {
    // Depth=1 skips lookahead: trap board completes identically to the
    // default depth=2 lookahead on this single-move fixture, and the
    // committed copper verifies.
    RouteReport d1 = run_board(trap_board(), 1, /*depth=*/1, /*beam=*/4);
    RouteReport d2 = run_board(trap_board(), 1, /*depth=*/2, /*beam=*/4);
    CT_CHECK(d1.status == "COMPLETE");
    CT_CHECK(d2.status == "COMPLETE");
    CT_CHECK(d1.board_hash == d2.board_hash);
    CT_CHECK(d1.state_hash.to_hex() == d2.state_hash.to_hex());
    CT_CHECK(d1.recovery.generations >= 1);
    // Depth=1 still records the one-ply PV diagnostics.
    CT_CHECK(!d1.recovery.multiply_pv.empty());
    CT_CHECK(d1.recovery.multiply_best_hash.size() == 32);
}

CT_TEST(multiply_deterministic_pv_and_hash) {
    RouteReport a = run_board(trap_board(), 2, 2, 4);
    RouteReport b = run_board(trap_board(), 2, 2, 4);
    CT_CHECK(a.status == "COMPLETE");
    CT_CHECK(b.status == "COMPLETE");
    CT_CHECK(a.board_hash == b.board_hash);
    CT_CHECK(a.state_hash.to_hex() == b.state_hash.to_hex());
    CT_CHECK(a.recovery.multiply_pv == b.recovery.multiply_pv);
    CT_CHECK(a.recovery.multiply_best_hash == b.recovery.multiply_best_hash);
    CT_CHECK(a.recovery.multiply_depth_effective == b.recovery.multiply_depth_effective);
    CT_CHECK(a.recovery.multiply_beam_effective == b.recovery.multiply_beam_effective);
    // Cross-thread determinism: indexed slots + serial TT keep copper/PV
    // identical at 1 vs 4 workers.
    RouteReport t1 = run_board(trap_board(), 1, 2, 4);
    RouteReport t4 = run_board(trap_board(), 4, 2, 4);
    CT_CHECK(t1.board_hash == t4.board_hash);
    CT_CHECK(t1.state_hash.to_hex() == t4.state_hash.to_hex());
    CT_CHECK(t1.recovery.multiply_pv == t4.recovery.multiply_pv);
}

CT_TEST(multiply_connectivity_first_preserved) {
    // Ugly complete leaf beats beautiful stranded leaf even when the
    // stranded leaf has minimal future obstruction (impact never outranks
    // connectivity).
    BranchResult stranded = leaf(1, 0, 0, 1000, 0, 0.0, true, "stranded");
    BranchResult complete = leaf(0, 1, 3, 100000000, 10, 1.0, true, "complete");
    CT_CHECK(branch_better(complete, stranded));
    CT_CHECK(!branch_better(stranded, complete));
    // Among equally-connected boards lower obstruction wins; ties fall to
    // the stable hash (deterministic).
    BranchResult low = leaf(1, 1, 1, 5000, 1, 0.15, true, "lowX");
    BranchResult high = leaf(1, 1, 1, 5000, 1, 0.85, true, "highX");
    CT_CHECK(branch_better(low, high));
    CT_CHECK(!branch_better(high, low));
    // Missing scores never win on impact alone (falls through to hash).
    BranchResult noscore = leaf(1, 1, 1, 5000, 1, -1.0, false, "noscore");
    BranchResult scored = leaf(1, 1, 1, 5000, 1, 0.0, true, "scored");
    bool s_better = branch_better(scored, noscore);
    bool n_better = branch_better(noscore, scored);
    CT_CHECK(s_better != n_better);  // total order, no crash on missing data
}

CT_TEST(multiply_budget_exhaustion_falls_back_to_one_ply) {
    // Zero deeper node budget forces fallback: the engine still completes
    // via the one-ply best and reports the fallback.
    RouteReport rep = run_board(trap_board(), 1, 2, 4, /*multiply_nodes=*/0);
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.recovery.multiply_fallback_to_one_ply);
    CT_CHECK(!rep.recovery.multiply_pv.empty());
    // Direct search-level fallback: max_nodes=0 with real first-ply states.
    Board b = trap_board();
    b.traces.push_back(
        {0, 0, {mm_to_nm(2.0), mm_to_nm(9.0)}, {mm_to_nm(18.0), mm_to_nm(9.0)}, mm_to_nm(0.2)});
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    ConnectionTask t_seal{0, b.nets[0].terminals[0], b.nets[0].terminals[1], 0, 1.0};
    ConnectionTask t_trapped{1, b.nets[1].terminals[0], b.nets[1].terminals[1], 1, 2.0};
    std::vector<ConnectionTask> tasks = {t_seal, t_trapped};
    std::map<std::pair<NetId, std::pair<TermId, TermId>>, CandidateRoute> last;
    DependencyGraph g = build_dependency_graph(b, r, ctx, tasks, {1}, last);
    OwnedRoute o;
    o.task = t_seal;
    o.task_pos = 0;
    o.traces = b.traces;
    o.protection = 1.0;
    HistoryHeuristic hh;
    std::vector<RipupMove> moves =
        generate_ripup_moves(g.failed, g, {o}, hh, "", RecoveryMode::FAST, 4, 2);
    CT_CHECK(!moves.empty());
    std::vector<Corridor> corridors = {probable_corridor(b, r, tasks[0], ctx),
                                       probable_corridor(b, r, tasks[1], ctx)};
    std::vector<double> layer_mult = {1.0};
    CongestionMap congestion;
    congestion.init(b);
    std::vector<BranchResult> results;
    for (const auto& m : moves) {
        std::vector<int> to_route = {1, 0};
        BranchResult br = reroute_branch(b, {}, {}, {}, to_route, 1, tasks,
                                         {1}, corridors, r, ctx, layer_mult,
                                         AStarConfig{}, congestion, "FAST", m);
        results.push_back(br);
    }
    TranspositionTable tt;
    for (const auto& br : results) tt.record(br.hash, (int)br.remaining_task_ids.size());
    MultiPlyContext mctx;
    mctx.tasks = &tasks;
    mctx.corridors = &corridors;
    mctx.resolver = &r;
    mctx.ectx = &ctx;
    mctx.layer_mult = &layer_mult;
    mctx.congestion_tpl = congestion;
    mctx.mode_name = "FAST";
    mctx.last_attempt = last;
    mctx.history = hh;
    mctx.mode = RecoveryMode::FAST;
    mctx.max_moves = 4;
    mctx.max_breadth = 2;
    MultiPlyConfig cfg;
    cfg.depth = 2;
    cfg.beam = 2;
    cfg.max_nodes = 0;  // no deeper budget
    cfg.threads = 1;
    MultiPlyResult pres = multiply_beam_search(
        moves, results, mctx, cfg, tt, std::chrono::steady_clock::now() + std::chrono::seconds(5));
    CT_CHECK(pres.searched);
    CT_CHECK(pres.fallback_to_one_ply);
    // Fallback still names a valid one-ply move.
    CT_CHECK(pres.best_first_move >= 0);
    CT_CHECK(pres.best_first_move < (int)moves.size());
}

CT_TEST(multiply_effective_depth_beam_threads_in_diagnostics) {
    RouteReport rep = run_board(trap_board(), 2, 2, 4);
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.recovery.multiply_depth_requested == 2);
    CT_CHECK(rep.recovery.multiply_beam_requested == 4);
    CT_CHECK(rep.recovery.multiply_depth_effective >= 1);
    CT_CHECK(rep.recovery.multiply_beam_effective >= 1);
    CT_CHECK(rep.recovery.multiply_beam_effective <= 8);
    CT_CHECK(rep.recovery.multiply_threads_effective >= 1);
    CT_CHECK(rep.recovery.multiply_nodes_evaluated >= 0);
    CT_CHECK(!rep.recovery.multiply_pv.empty());
    CT_CHECK(rep.recovery.multiply_best_hash.size() == 32);
    JsonValue j = rep.to_json();
    CT_CHECK(j["recovery"]["multiply_depth_effective"].as_number() >= 1);
    CT_CHECK(j["recovery"]["multiply_beam_effective"].as_number() >= 1);
    CT_CHECK(j["recovery"]["multiply_pv"].as_array().size() > 0);
    CT_CHECK(rep.verification.ok);
    CT_CHECK(rep.result_category == "COMPLETE");
    // Maturity caps clamp the effective values (pure helpers).
    MaturityCaps caps;
    caps.max_recovery_depth = 1;
    caps.max_beam = 2;
    EffectiveSearchBudget budget;
    budget.recovery_depth = 10;
    budget.recovery_beam = 8;
    CT_CHECK(effective_multiply_depth(5, budget, caps) == 1);
    CT_CHECK(effective_multiply_beam(8, budget, caps, 2048ULL * 1024ULL * 1024ULL) <= 2);
    // Memory bound: tiny budget squeezes the beam to 1.
    CT_CHECK(effective_multiply_beam(4, budget, caps, 256ULL * 1024ULL * 1024ULL) == 1);
}

int main() { return copperline::test::run_all_tests(); }
