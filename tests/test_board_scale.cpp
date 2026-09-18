// Opt-in board-scale test regime.
//
// Board-scale = the initial backlog is >= kGuidanceDisableRemaining, which is
// where the router now latches its board-scale policy (guidance off, sparse
// graph clamped to kBoardScaleGraphBases / kBoardScaleGraphK). These runs are
// minutes long and build ~2000+ task boards, so they are SKIPPED unless the
// environment variable COPPERLINE_BOARD_SCALE=1 is set. Default `ctest` stays
// fast; the board-scale path is exercised explicitly:
//
//   set COPPERLINE_BOARD_SCALE=1
//   build/test_board_scale.exe
//
// What is covered:
//  1. the generator really is board-scale (so the policy actually latches);
//  2. the latched policy does not cost connectivity vs the un-latched legacy
//     policy (the "sticky quality loss" downside) -- measured, not asserted
//     blindly;
//  3. an independent correctness proof: verify, then a rip-up/re-route repair
//     pass on the routed board, then verify again (connectivity must not
//     regress and committed copper must stay legal).
#include <cstdio>
#include <cstdlib>
#include <string>

#include "helpers.h"
#include "router/engine.h"
#include "router/maturity.h"
#include "router/verifier.h"

using namespace copperline;
using namespace copperline::test;

namespace {

// Sizing: one 8-terminal net yields ~5 connection tasks, so kCols*kRows cells
// must clear the kGuidanceDisableRemaining threshold with margin (48*10 = 480
// nets -> ~2400 tasks).
constexpr int kCols = 48;
constexpr int kRows = 10;
constexpr int kPadsPerNet = 8;

// Board-scale fixture: kCols*kRows independent 8-pad nets, each split by a
// full-height wall so the two pad rows must detour around its ends. Every net
// is routable in isolation, so the flood of tasks both latches the policy
// (>= kGuidanceDisableRemaining tasks) and can actually complete.
Board board_scale_board() {
    reset_term_ids();
    Board b;
    b.source_format = "test";
    const double cw = 12.0, ch = 8.0;
    b.width_nm = mm_to_nm(kCols * cw);
    b.height_nm = mm_to_nm(kRows * ch);
    b.layers.push_back({0, "Top"});
    b.layers.push_back({1, "Bottom"});
    int net = 0;
    for (int r = 0; r < kRows; ++r) {
        for (int c = 0; c < kCols; ++c) {
            const double x0 = c * cw, y0 = r * ch;
            NetInfo n = make_net(net, "N" + std::to_string(net));
            n.has_current = true;
            n.current_a = 0.1;
            b.nets.push_back(n);
            // Two rows of kPadsPerNet/2 pads (2mm pitch).
            for (int i = 0; i < kPadsPerNet / 2; ++i) {
                add_terminal(b, net, x0 + 2.0 + 2.0 * i, y0 + 2.0);
                add_terminal(b, net, x0 + 2.0 + 2.0 * i, y0 + 6.0);
            }
            // Wall between the rows, all layers: cross-row connections must
            // escape around its ends.
            Keepout k;
            k.rect = {mm_to_nm(x0 + 1.0), mm_to_nm(y0 + 3.5), mm_to_nm(x0 + 11.0),
                      mm_to_nm(y0 + 4.5)};
            k.layer = kAllLayers;
            k.reason = "wall";
            b.keepouts.push_back(k);
            ++net;
        }
    }
    return b;
}

struct Outcome {
    RouteReport rep;
    Board board;
};

Outcome run_scale(Board b, bool disable_policy, int threads) {
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = threads;
    opt.enable_ripup = true;
    // Latch toggled via the caps escape hatch: false = latched board-scale
    // policy (guidance off + clamped graph), true = legacy phase map.
    opt.maturity.caps.disable_board_scale_policy = disable_policy;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    Board out = engine.committed();
    return Outcome{std::move(rep), std::move(out)};
}

int connected(const RouteReport& rep) { return rep.connected_terminals; }

}  // namespace

CT_TEST(boardsize_fixture_actually_latches) {
    Outcome o = run_scale(board_scale_board(), /*disable_policy=*/false, 8);
    std::printf("  [scale] fixture tasks_total=%d (threshold=%d) routed=%d/%d\n",
                o.rep.stats.tasks_total, kGuidanceDisableRemaining,
                (int)o.rep.stats.tasks_routed, (int)o.rep.stats.tasks_total);
    // Premise of the whole regime: the initial backlog must be board-scale.
    CT_CHECK(o.rep.stats.tasks_total >= kGuidanceDisableRemaining);
    // Committed copper is always legal, whether or not the board completed.
    CT_CHECK(o.rep.verification.legal);
    // The latched policy must be visible in the log.
    CT_CHECK(!o.rep.maturity_log.empty());
    CT_CHECK(o.rep.maturity_log[0].budget.graph_max_bases <= kBoardScaleGraphBases);
    CT_CHECK(o.rep.maturity_log[0].budget.graph_k_nearest <= kBoardScaleGraphK);
    CT_CHECK(!o.rep.maturity_log[0].budget.hier_enabled);
}

CT_TEST(boardsize_latched_does_not_cost_connectivity) {
    // The only downside that could flip the decision: latching keeps the cheap
    // settings for the whole run, so a board that starts board-scale and then
    // drains might connect FEWER of the late hard tasks than the legacy policy
    // (which would re-enable guidance + rich graphs). Measure both; the latched
    // run must not connect fewer terminals.
    Outcome latched = run_scale(board_scale_board(), /*disable_policy=*/false, 16);
    Outcome legacy = run_scale(board_scale_board(), /*disable_policy=*/true, 16);
    std::printf("  [scale] latched connected=%d  legacy connected=%d  (of %d)\n",
                connected(latched.rep), connected(legacy.rep),
                latched.rep.total_terminals);
    CT_CHECK(latched.rep.verification.legal);
    CT_CHECK(legacy.rep.verification.legal);
    CT_CHECK(connected(latched.rep) >= connected(legacy.rep));
}

CT_TEST(boardsize_repair_pass_proves_correctness) {
    // Independent correctness proof. Pass 1 routes. If anything is still
    // unconnected, pass 2 feeds the routed board back so the engine can rip up
    // the blocking allocations and re-route the remainder. The final copper
    // must be legal and connectivity must not regress -- proving the "sticky"
    // policy leaves the router able to repair, not just abandon, work.
    Outcome p1 = run_scale(board_scale_board(), /*disable_policy=*/false, 16);
    const int c1 = connected(p1.rep);
    const auto u1 = p1.rep.verification.unconnected.size();
    std::printf("  [scale] repair pass 1: connected=%d unconnected=%zu legal=%d\n", c1, u1,
                p1.rep.verification.legal ? 1 : 0);
    CT_CHECK(p1.rep.verification.legal);

    Outcome p2 = run_scale(p1.board, /*disable_policy=*/false, 16);
    const int c2 = connected(p2.rep);
    const auto u2 = p2.rep.verification.unconnected.size();
    std::printf("  [scale] repair pass 2: connected=%d unconnected=%zu legal=%d\n", c2, u2,
                p2.rep.verification.legal ? 1 : 0);
    // The repair pass must never make things worse, and must keep the copper
    // legal; if pass 1 already connected everything, pass 2 must stay complete.
    CT_CHECK(p2.rep.verification.legal);
    CT_CHECK(c2 >= c1);
    CT_CHECK(u2 <= u1);
    if (u1 == 0) CT_CHECK(u2 == 0);
    // Full correctness when the board is routable end to end.
    if (p2.rep.verification.connected) CT_CHECK(p2.rep.verification.ok);
}

int main() {
    if (std::getenv("COPPERLINE_BOARD_SCALE") == nullptr) {
        std::cout << "SKIP: board-scale tests are opt-in; set COPPERLINE_BOARD_SCALE=1 "
                     "to run them\n";
        return 0;
    }
    return copperline::test::run_all_tests();
}
