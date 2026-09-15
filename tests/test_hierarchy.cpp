// Issue #10: hierarchical coarse-to-fine A* guidance tests.
//
//   maze_reduces_expansions   - dense street maze: guided exact expansions
//                               and wall time beat unguided; identical
//                               copper across thread counts; verifier clean.
//   narrow_01mm_path          - 0.12mm slot: finest (0.1mm) guidance level
//                               resolves it; end-to-end route threads the
//                               slot and verifies clean.
//   misleading_corridor_fallback - coarse-impenetrable channel: coarse
//                               guidance fails, the unrestricted exact
//                               fallback still finds the legal path
//                               (guidance never removes a solution).
//   no_grid_staircase         - open diagonal: committed copper is one exact
//                               diagonal segment (#17 simplifier
//                               authoritative; no grid steps leak in).
#include <chrono>
#include <cstdio>

#include "helpers.h"
#include "router/board.h"
#include "router/engine.h"
#include "router/hierarchy.h"
#include "router/parallel.h"
#include "router/route_tree.h"
#include "router/verifier.h"

using namespace copperline;
using namespace copperline::test;

namespace {

Board maze_board() {
    // Long-travel street maze on ONE layer: 116mm tasks through a dense
    // uniform field (1mm blocks, 0.8mm streets). Plain builds pay the
    // full obstacle scan per edge while the guided build only ever sees
    // the tube band, so guidance wins expansions and wall time together.
    // Rows run through block rows (forcing jogs) on disjoint corridors,
    // so no contention pollutes the comparison. Corner count exceeds
    // the sparse cap, but truncation keeps the nearest band and all
    // detours are local, so every task stays routable.
    Board b;
    b.source_format = "test";
    b.width_nm = mm_to_nm(120.0);
    b.height_nm = mm_to_nm(30.0);
    b.layers.push_back({0, "Top"});
    // Dense uniform field: 1mm blocks on 1.8mm pitch (0.8mm streets).
    // Plain builds pay the full obstacle scan per edge; the guided build
    // only ever sees the tube band. Rows run through block rows.
    for (int i = 0; i < 60; ++i) {
        for (int j = 0; j < 14; ++j) {
            double x0 = 6.0 + i * 1.8, y0 = 4.0 + j * 1.8;
            Keepout k{{mm_to_nm(x0), mm_to_nm(y0), mm_to_nm(x0 + 1.0),
                       mm_to_nm(y0 + 1.0)},
                      kAllLayers, "block"};
            b.keepouts.push_back(k);
        }
    }
    for (int i = 0; i < 8; ++i) {
        double y = 4.5 + i * 3.0;
        NetInfo n = make_net(i, "M" + std::to_string(i));
        n.has_current = true;
        n.current_a = 0.1;
        b.nets.push_back(n);
        add_terminal(b, i, 2.0, y);
        add_terminal(b, i, 118.0, y);
    }
    return b;
}

RouteReport run_cfg(Board b, bool hierarchy, int threads, double& wall_s,
                    Board* committed_out = nullptr) {
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = threads;
    opt.hierarchy.enabled = hierarchy;
    auto t0 = std::chrono::steady_clock::now();
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    auto t1 = std::chrono::steady_clock::now();
    wall_s = std::chrono::duration<double>(t1 - t0).count();
    if (committed_out != nullptr) *committed_out = engine.committed();
    return rep;
}

bool verify_clean(const Board& committed) {
    BoardVerifier v;
    ElectricalContext ctx;
    RuleResolver r = RuleResolver::defaults_for(committed);
    return v.verify(committed, r, ctx).ok;
}

}  // namespace

CT_TEST(maze_reduces_expansions) {
    // Min-of-3 walls: routing is deterministic, so expansions are compared
    // exactly while walls use the stable minimum against CI noise.
    double w_guided = 0, w_plain = 0;
    RouteReport guided, plain;
    for (int r = 0; r < 3; ++r) {
        double w = 0;
        RouteReport rep = run_cfg(maze_board(), true, 1, w);
        if (r == 0 || w < w_guided) {
            w_guided = w;
            guided = rep;
        }
    }
    for (int r = 0; r < 3; ++r) {
        double w = 0;
        RouteReport rep = run_cfg(maze_board(), false, 1, w);
        if (r == 0 || w < w_plain) {
            w_plain = w;
            plain = rep;
        }
    }
    std::printf("  [hier] maze guided: %lld exp / %.3fs %s routed %d/%d | unguided: %lld exp / %.3fs %s routed %d/%d\n",
                (long long)guided.stats.expansions_total, w_guided,
                guided.status.c_str(), guided.stats.tasks_routed, guided.stats.tasks_total,
                (long long)plain.stats.expansions_total, w_plain,
                plain.status.c_str(), plain.stats.tasks_routed, plain.stats.tasks_total);
    for (const auto& f : guided.failures)
        std::printf("    guided fail: net=%d reason=%s hier=%s\n", f.net, f.reason.c_str(),
                    f.hierarchy_reason.c_str());
    for (const auto& f : plain.failures)
        std::printf("    plain fail: net=%d reason=%s\n", f.net, f.reason.c_str());
    CT_CHECK(guided.status == "COMPLETE");
    CT_CHECK(plain.status == "COMPLETE");
    CT_CHECK(guided.stats.tasks_routed == plain.stats.tasks_routed);
    // The guided window prunes dead-end sparse corners: strictly fewer
    // exact expansions (deterministic values, no timing flake).
    CT_CHECK(guided.stats.expansions_total < plain.stats.expansions_total);
    std::printf("  [hier] guided=%lld plain=%lld ratio=%.3f coarse=%lld fallbacks=%d epochs=%d/%d\n",
                (long long)guided.stats.expansions_total,
                (long long)plain.stats.expansions_total,
                (double)guided.stats.expansions_total /
                    (double)std::max<std::int64_t>(1, plain.stats.expansions_total),
                (long long)guided.stats.hierarchy_coarse_expansions,
                guided.stats.hierarchy_fallback_tasks,
                guided.stats.epochs_count, plain.stats.epochs_count);
    // Guidance must beat unguided end to end (search + build savings
    // dwarf coarse generation on this workload).
    CT_CHECK(w_guided <= w_plain);
    // Thread count never changes guided copper (determinism preserved).
    double w_mt = 0;
    Board committed_mt;
    RouteReport guided_mt = run_cfg(maze_board(), true, 4, w_mt, &committed_mt);
    CT_CHECK(guided_mt.board_hash == guided.board_hash);
    CT_CHECK(verify_clean(committed_mt));
}

CT_TEST(narrow_01mm_path) {
    // 0.12mm vertical slot at x in [6.00, 6.12], walls y in [2, 4].
    // Width 0.05mm + clearance 0.02mm needs 0.045mm/side. Grid centers:
    // 0.8/0.4/0.2mm pitches all land inside the walls (blocked), while the
    // 0.1mm pitch has free center columns (5.95/6.05), so only the finest
    // guidance level resolves the slot.
    Board b = base_2layer(12.0, 6.0);
    b.defaults.trace_width_nm = mm_to_nm(0.05);
    b.defaults.clearance_nm = mm_to_nm(0.02);
    NetInfo n = make_net(0, "SIG");
    b.nets.push_back(n);
    add_terminal(b, 0, 6.06, 1.0);
    add_terminal(b, 0, 6.06, 5.0);
    Keepout kl{{mm_to_nm(4.0), mm_to_nm(2.0), mm_to_nm(6.0), mm_to_nm(4.0)},
               kAllLayers, "left"};
    Keepout kr{{mm_to_nm(6.12), mm_to_nm(2.0), mm_to_nm(8.0), mm_to_nm(4.0)},
               kAllLayers, "right"};
    b.keepouts.push_back(kl);
    b.keepouts.push_back(kr);

    // Guidance level: the finest path must thread the slot.
    {
        RuleResolver r = RuleResolver::defaults_for(b);
        ElectricalContext ctx;
        const Terminal* ta = b.find_terminal(b.nets[0].terminals[0]);
        const Terminal* tb = b.find_terminal(b.nets[0].terminals[1]);
        HierarchyRequest req;
        req.net = 0;
        req.src = ta->pos;
        req.src_layer = ta->layer;
        req.dsts.push_back({tb->pos, tb->layer});
        req.width_nm = r.requiredTraceWidth(0, 0, ctx);
        std::string cs;
        req.clearance_nm = r.requiredClearance(0, 0, 0, ctx, &cs);
        req.layer_mult = {1.0, 1.0};
        req.astar_cfg = AStarConfig{};
        HierarchyCache cache;
        GuidanceResult g = cache.build_guidance(b, r, ctx, req, HierarchyConfig{});
        CT_CHECK(g.found);
        CT_CHECK(g.levels_used_mm.size() == 4);  // all levels ran
        CT_CHECK(!g.level_paths.empty());
        const auto& fine = g.level_paths.back();
        bool through_slot = false;
        for (const auto& p : fine) {
            double x = nm_to_mm(p.x), y = nm_to_mm(p.y);
            if (x > 5.9 && x < 6.2 && y > 2.0 && y < 4.0) through_slot = true;
        }
        std::printf("  [hier] slot levels=%d finest_through_slot=%d\n",
                    (int)g.levels_used_mm.size(), (int)through_slot);
        CT_CHECK(through_slot);
    }

    // End to end: the committed route threads the slot and verifies clean.
    {
        double wall = 0;
        Board committed;
        RouteReport rep = run_cfg(std::move(b), true, 1, wall, &committed);
        CT_CHECK(rep.status == "COMPLETE");
        CT_CHECK(verify_clean(committed));
        bool through_slot = false;
        for (const auto& t : committed.traces) {
            Rect slot{mm_to_nm(5.95), mm_to_nm(2.0), mm_to_nm(6.17), mm_to_nm(4.0)};
            if (seg_intersects_rect(t.segment(), slot)) through_slot = true;
        }
        CT_CHECK(through_slot);
        (void)wall;
    }
}

CT_TEST(misleading_corridor_fallback) {
    // Full-height channel: walls x in [0, 6.03] and [6.11, 12] span the
    // whole board height, leaving a 0.08mm vertical channel with no
    // around-the-ends detour. Single-net keepout distance is half width
    // (0.025mm): every coarse pitch has all in-channel centers within
    // 0.025mm of copper (verified by grid-phase arithmetic), so all
    // guidance levels fail; the unrestricted exact fallback still routes
    // the aligned straight edge and connects.
    Board b = base_2layer(12.0, 6.0);
    b.defaults.trace_width_nm = mm_to_nm(0.05);
    b.defaults.clearance_nm = mm_to_nm(0.02);
    NetInfo n = make_net(0, "SIG");
    b.nets.push_back(n);
    add_terminal(b, 0, 6.07, 1.0, 0, 0.02);
    add_terminal(b, 0, 6.07, 5.0, 0, 0.02);
    Keepout kl{{mm_to_nm(0.0), mm_to_nm(0.0), mm_to_nm(6.03), mm_to_nm(6.0)},
               kAllLayers, "left"};
    Keepout kr{{mm_to_nm(6.11), mm_to_nm(0.0), mm_to_nm(12.0), mm_to_nm(6.0)},
               kAllLayers, "right"};
    b.keepouts.push_back(kl);
    b.keepouts.push_back(kr);

    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    RouteTree tree = build_route_tree(b, 0);
    CT_CHECK(!tree.tasks.empty());
    ConnectionTask task = tree.tasks.front();
    std::vector<double> lm = {1.0, 1.0};
    CongestionMap cg;
    cg.init(b);
    ReservationSet rs;
    std::vector<ConnectionTask> one = {task};
    std::vector<Corridor> corr = {probable_corridor(b, r, task, ctx)};
    rs.build(one, corr, {0.0});
    HierarchyCache cache;
    HierarchyConfig hcfg;
    CandidateRoute cand =
        route_candidate_task(b, r, task, 0, 0.0, ctx, lm, AStarConfig{}, cg, rs,
                             hcfg, &cache);
    std::printf("  [hier] channel found=%d fail=%s fallback=%d reason=%s coarse=%lld exact=%lld\n",
                (int)cand.found, cand.fail_reason.c_str(), (int)cand.hierarchy.fallback,
                cand.hierarchy.fallback_reason.c_str(),
                (long long)cand.hierarchy.coarse_expansions,
                (long long)cand.hierarchy.exact_expansions);
    // Guidance must report the miss, yet the legal path is still found.
    CT_CHECK(cand.hierarchy.attempted);
    CT_CHECK(cand.hierarchy.fallback);
    CT_CHECK(cand.hierarchy.fallback_reason == "coarse_fail");
    CT_CHECK(cand.found);
    // End to end the channel routes and verifies clean.
    {
        double wall = 0;
        Board committed;
        RouteReport rep = run_cfg(std::move(b), true, 1, wall, &committed);
        std::printf("  [hier] channel engine: %s routed %d/%d\n", rep.status.c_str(),
                    rep.stats.tasks_routed, rep.stats.tasks_total);
        for (const auto& f : rep.failures)
            std::printf("    channel fail: net=%d (%d,%d) reason=%s hier=%s\n", f.net,
                        f.a, f.b, f.reason.c_str(), f.hierarchy_reason.c_str());
        CT_CHECK(rep.status == "COMPLETE");
        CT_CHECK(verify_clean(committed));
        (void)wall;
    }
}

CT_TEST(no_grid_staircase) {
    // Open board diagonal: the simplifier must collapse guidance to one
    // exact diagonal segment (no Manhattan/grid steps in copper).
    Board b = base_2layer(20.0, 10.0);
    NetInfo n = make_net(0, "SIG");
    n.has_current = true;
    n.current_a = 0.1;
    b.nets.push_back(n);
    TermId ta = add_terminal(b, 0, 2.0, 2.0);
    TermId tb = add_terminal(b, 0, 18.0, 8.0);
    double wall = 0;
    RuleResolver r0 = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.hierarchy.enabled = true;
    RouterEngine engine(std::move(b), std::move(r0), opt);
    auto t0 = std::chrono::steady_clock::now();
    RouteReport rep = engine.run();
    auto t1 = std::chrono::steady_clock::now();
    wall = std::chrono::duration<double>(t1 - t0).count();
    (void)wall;
    CT_CHECK(rep.status == "COMPLETE");
    const Board& c = engine.committed();
    CT_CHECK(verify_clean(c));
    const Terminal* pa = c.find_terminal(ta);
    const Terminal* pb = c.find_terminal(tb);
    CT_CHECK(pa != nullptr && pb != nullptr);
    // Exactly one diagonal segment tieing the pads (escape adds nothing on
    // isolated pads; the simplifier owns the geometry).
    CT_CHECK(c.traces.size() == 1);
    const TraceSeg& s = c.traces.front();
    CT_CHECK((s.a == pa->pos && s.b == pb->pos) || (s.a == pb->pos && s.b == pa->pos));
    CT_CHECK(!s.segment().axis_aligned());  // arbitrary angle, not grid steps
}

int main() { return copperline::test::run_all_tests(); }
