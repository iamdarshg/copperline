// Issue #8: diverse A* route portfolios per task (10-15 alternatives).
#include <chrono>
#include <set>

#include "helpers.h"
#include "router/engine.h"
#include "router/maturity.h"
#include "router/parallel.h"
#include "router/portfolio.h"
#include "router/verifier.h"

using namespace copperline;
using namespace copperline::test;

namespace {

Board open_2layer() {
    Board b = base_2layer(20.0, 20.0);
    NetInfo n = make_net(0, "SIG");
    n.has_current = true;
    n.current_a = 0.1;
    b.nets.push_back(n);
    add_terminal(b, 0, 2.0, 10.0, 0);
    add_terminal(b, 0, 18.0, 10.0, 0);
    return b;
}

Board multilayer_board() {
    Board b = base_2layer(20.0, 20.0);
    NetInfo n = make_net(0, "SIG");
    n.has_current = true;
    n.current_a = 0.1;
    b.nets.push_back(n);
    add_terminal(b, 0, 2.0, 10.0, 0);
    add_terminal(b, 0, 18.0, 10.0, 1);  // must change layer -> via
    return b;
}

Board bottleneck_board() {
    Board b = base_2layer(20.0, 20.0);
    NetInfo n = make_net(0, "SIG");
    n.has_current = true;
    n.current_a = 0.1;
    b.nets.push_back(n);
    add_terminal(b, 0, 2.0, 10.0, 0);
    add_terminal(b, 0, 18.0, 10.0, 0);
    // Central wall with a narrow gap top and bottom: two distinct corridors.
    Keepout mid;
    mid.rect = {mm_to_nm(9.0), mm_to_nm(4.0), mm_to_nm(11.0), mm_to_nm(16.0)};
    mid.layer = kAllLayers;
    mid.reason = "wall";
    b.keepouts.push_back(mid);
    // Distant second net: raises the worst-case keepout clearance above zero
    // so wall-hugging detours stand off the wall and verify clean (single-net
    // boards touch exactly, which the verifier flags while the arbiter
    // allows; the engine commits on the arbiter predicate).
    NetInfo dummy = make_net(1, "DUMMY");
    dummy.has_current = true;
    dummy.current_a = 0.1;
    b.nets.push_back(dummy);
    add_terminal(b, 1, 2.0, 18.5, 0);
    add_terminal(b, 1, 6.0, 18.5, 0);
    return b;
}

PortfolioOptions opts_k(int k) {
    PortfolioOptions o;
    o.requested_k = k;
    o.max_k = kPortfolioMaxK;
    o.memory_budget_bytes = kRouterMemoryBudgetBytes;
    o.per_task_bytes = kPerCandidateBytes;
    o.per_alt_bytes = kPerPortfolioAltBytes;
    o.batch_width = kParallelBatchSize;
    o.budget_route_k = -1;
    o.threads_requested = 1;
    return o;
}

std::vector<double> layer_mult_for(const Board& b) {
    int max_id = 0;
    for (const auto& l : b.layers) max_id = std::max(max_id, l.id);
    std::vector<double> lm(max_id + 1, 1.0);
    for (const auto& l : b.layers) lm[l.id] = 1.0;
    return lm;
}

}  // namespace

CT_TEST(portfolio_open_multilayer_no_via_vs_via) {
    // Open board: ban-dominant-layer forces a via alternative distinct
    // from the no-via base. Multilayer board: base itself carries a via.
    {
        Board b = open_2layer();
        RuleResolver r = RuleResolver::defaults_for(b);
        ElectricalContext ctx;
        RouteTree tree = build_route_tree(b, 0);
        CT_CHECK(!tree.tasks.empty());
        ConnectionTask task = tree.tasks.front();
        std::vector<double> lm = layer_mult_for(b);
        CongestionMap cg;
        cg.init(b);
        ReservationSet rs;
        Corridor c = probable_corridor(b, r, task, ctx);
        rs.build({task}, {c}, {1.0});
        HierarchyConfig hier;
        HierarchyCache cache;
        PortfolioResult pf = build_portfolio(b, r, task, 0, 1.0, ctx, lm,
                                             AStarConfig{}, cg, rs, hier, &cache,
                                             opts_k(12));
        CT_CHECK(pf.size() >= 2);
        CT_CHECK(pf.graph_builds == 1);  // reuse: one shared graph
        bool has_novia = false, has_via = false;
        std::set<std::string> sigs;
        for (const auto& cand : pf.candidates) {
            sigs.insert(cand.sig_str);
            CT_CHECK(!cand.sig_str.empty());
            CT_CHECK(!cand.sig.corridor_class.empty());
            CT_CHECK(!cand.sig.dir_seq.empty());
            if (cand.route.vias.empty()) has_novia = true;
            if (!cand.route.vias.empty()) has_via = true;
            std::string why;
            CT_CHECK(candidate_legal_vs_board(cand.route, b, r, ctx, why));
        }
        CT_CHECK(has_novia);
        CT_CHECK(has_via);  // layer-ban deviation forces the via alternative
        CT_CHECK(sigs.size() == pf.size());  // deduped
        // Deterministic sort by (cost, signature).
        for (std::size_t i = 1; i < pf.candidates.size(); ++i) {
            bool ok = pf.candidates[i].route.cost_nm > pf.candidates[i - 1].route.cost_nm ||
                      (pf.candidates[i].route.cost_nm == pf.candidates[i - 1].route.cost_nm &&
                       pf.candidates[i].sig_str >= pf.candidates[i - 1].sig_str);
            CT_CHECK(ok);
        }
        JsonValue j = pf.to_json();
        CT_CHECK(j.has("size") && j.has("candidates"));
        CT_CHECK(j.has("total_expansions") && j.has("effective_k"));
        CT_CHECK(j.has("threads_effective"));
    }
    {
        Board b = multilayer_board();
        RuleResolver r = RuleResolver::defaults_for(b);
        ElectricalContext ctx;
        RouteTree tree = build_route_tree(b, 0);
        CT_CHECK(!tree.tasks.empty());
        ConnectionTask task = tree.tasks.front();
        std::vector<double> lm = layer_mult_for(b);
        CongestionMap cg;
        cg.init(b);
        ReservationSet rs;
        Corridor c = probable_corridor(b, r, task, ctx);
        rs.build({task}, {c}, {1.0});
        HierarchyConfig hier;
        HierarchyCache cache;
        PortfolioResult pf = build_portfolio(b, r, task, 0, 1.0, ctx, lm,
                                             AStarConfig{}, cg, rs, hier, &cache,
                                             opts_k(12));
        CT_CHECK(pf.size() >= 1);
        bool any_via = false;
        for (const auto& cand : pf.candidates) {
            if (!cand.route.vias.empty()) any_via = true;
            CT_CHECK(cand.sig.first_via != "none" || cand.route.vias.empty());
        }
        CT_CHECK(any_via);
    }
}

CT_TEST(portfolio_bottleneck_different_corridors) {
    Board b = bottleneck_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    RouteTree tree = build_route_tree(b, 0);
    CT_CHECK(!tree.tasks.empty());
    ConnectionTask task = tree.tasks.front();
    std::vector<double> lm = layer_mult_for(b);
    CongestionMap cg;
    cg.init(b);
    ReservationSet rs;
    Corridor c = probable_corridor(b, r, task, ctx);
    rs.build({task}, {c}, {1.0});
    HierarchyConfig hier;
    HierarchyCache cache;
    PortfolioResult pf = build_portfolio(b, r, task, 0, 1.0, ctx, lm, AStarConfig{},
                                         cg, rs, hier, &cache, opts_k(12));
    CT_CHECK(pf.size() >= 2);
    // At least two distinct corridor classes or direction sequences around
    // the wall (north vs south detour).
    std::set<std::string> corridors, dirs;
    for (const auto& cand : pf.candidates) {
        corridors.insert(cand.sig.corridor_class);
        dirs.insert(cand.sig.dir_seq);
        std::string why;
        CT_CHECK(candidate_legal_vs_board(cand.route, b, r, ctx, why));
    }
    CT_CHECK(corridors.size() + dirs.size() >= 3 || corridors.size() >= 2 || dirs.size() >= 2);
    // Every candidate verifies exactly legal on the snapshot.
    BoardVerifier v;
    for (const auto& cand : pf.candidates) {
        Board tmp = b;
        for (const auto& s : cand.route.traces) tmp.traces.push_back(s);
        for (const auto& vv : cand.route.vias) tmp.vias.push_back(vv);
        RuleResolver rr = RuleResolver::defaults_for(tmp);
        VerifyResult vr = v.verify(tmp, rr, ctx);
        CT_CHECK(vr.legal);
    }
}

CT_TEST(portfolio_tiny_perturbations_collapse) {
    Board b = open_2layer();
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    RouteTree tree = build_route_tree(b, 0);
    ConnectionTask task = tree.tasks.front();
    std::vector<double> lm = layer_mult_for(b);
    CongestionMap cg;
    cg.init(b);
    ReservationSet rs;
    Corridor c = probable_corridor(b, r, task, ctx);
    rs.build({task}, {c}, {1.0});
    HierarchyConfig hier;
    HierarchyCache cache;
    PortfolioResult pf = build_portfolio(b, r, task, 0, 1.0, ctx, lm, AStarConfig{},
                                         cg, rs, hier, &cache, opts_k(15));
    // Cap respected and dedup holds: signatures unique, size <= 15.
    CT_CHECK(pf.size() <= 15);
    std::set<std::string> sigs;
    for (const auto& cand : pf.candidates) sigs.insert(cand.sig_str);
    CT_CHECK(sigs.size() == pf.size());
    // Direct signature check: two near-identical paths share a signature.
    {
        const Terminal* ta = b.find_terminal(task.a);
        const Terminal* tb = b.find_terminal(task.b);
        std::string ws;
        Coord w = r.requiredTraceWidth(0, 0, ctx, &ws);
        SparseRoutingGraph g = SparseRoutingGraph::build(b, r, 0, ta->pos, tb->pos,
                                                         ta->layer, tb->layer, w, ctx);
        AStarResult r1 = astar_route(g, lm, AStarConfig{});
        std::vector<Coord> bias(g.nodes().size(), 0);
        for (std::size_t i = 0; i < g.nodes().size(); ++i) {
            // 0.1mm deterministic jog on odd nodes only (tiny perturbation).
            if (i % 2 == 1) bias[i] = 100000;
        }
        AStarResult r2 = astar_route_masked(g, lm, AStarConfig{}, {}, bias);
        CT_CHECK(r1.found && r2.found);
        RouteSignature s1 = compute_route_signature(g, r1);
        RouteSignature s2 = compute_route_signature(g, r2);
        CT_CHECK(s1.to_string() == s2.to_string());
    }
}

CT_TEST(portfolio_k1_reproduces_single_path) {
    Board b = open_2layer();
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    RouteTree tree = build_route_tree(b, 0);
    ConnectionTask task = tree.tasks.front();
    std::vector<double> lm = layer_mult_for(b);
    CongestionMap cg;
    cg.init(b);
    ReservationSet rs;
    Corridor c = probable_corridor(b, r, task, ctx);
    rs.build({task}, {c}, {1.0});
    HierarchyConfig hier;
    HierarchyCache cache;
    CandidateRoute single = route_candidate_task(b, r, task, 0, 1.0, ctx, lm,
                                                 AStarConfig{}, cg, rs, hier, &cache);
    CT_CHECK(single.found);
    PortfolioResult pf = build_portfolio(b, r, task, 0, 1.0, ctx, lm, AStarConfig{},
                                         cg, rs, hier, &cache, opts_k(1));
    CT_CHECK(pf.size() == 1);
    CT_CHECK(pf.effective_k == 1);
    CT_CHECK(pf.best()->route.cost_nm == single.cost_nm);
    CT_CHECK(pf.best()->route.traces.size() == single.traces.size());
    CT_CHECK(pf.best()->route.vias.size() == single.vias.size());
}

CT_TEST(portfolio_k_effective_budget_memory_threads) {
    // Maturity route-K flows in as the default width; explicit request above
    // the budget is clamped; memory bound collapses K to 1 under pressure.
    CT_CHECK(clamp_portfolio_k(12) == 12);
    CT_CHECK(clamp_portfolio_k(99) == kPortfolioMaxK);
    CT_CHECK(clamp_portfolio_k(0) == 1);
    // Budget route_k from EffectiveSearchBudget wins over a larger request.
    CT_CHECK(effective_portfolio_k(12, 15, 3, kRouterMemoryBudgetBytes, 8,
                                   kPerCandidateBytes, kPerPortfolioAltBytes) == 3);
    // No budget: default memory keeps K=12.
    CT_CHECK(effective_portfolio_k(12, 15, -1, kRouterMemoryBudgetBytes, 8,
                                   kPerCandidateBytes, kPerPortfolioAltBytes) == 12);
    // Tiny memory budget: K collapses to 1 (bounded overhead, never 0).
    CT_CHECK(effective_portfolio_k(12, 15, -1, 64ULL * 1024 * 1024, 8,
                                   kPerCandidateBytes, kPerPortfolioAltBytes) == 1);
    // Threads: explicit wins, 0 = auto (all CPUs).
    CT_CHECK(resolve_worker_threads(2) == 2);
    CT_CHECK(resolve_worker_threads(0) >= 1);
    Board b = open_2layer();
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    RouteTree tree = build_route_tree(b, 0);
    ConnectionTask task = tree.tasks.front();
    std::vector<double> lm = layer_mult_for(b);
    CongestionMap cg;
    cg.init(b);
    ReservationSet rs;
    Corridor cc = probable_corridor(b, r, task, ctx);
    rs.build({task}, {cc}, {1.0});
    HierarchyConfig hier;
    HierarchyCache cache;
    PortfolioOptions o = opts_k(12);
    o.budget_route_k = 3;
    PortfolioResult pf = build_portfolio(b, r, task, 0, 1.0, ctx, lm, AStarConfig{},
                                         cg, rs, hier, &cache, o);
    CT_CHECK(pf.effective_k == 3);
    CT_CHECK(pf.size() <= 3);
    JsonValue j = pf.to_json();
    CT_CHECK(j.get_number("effective_k", -1) == 3);
    CT_CHECK(j.get_number("threads_effective", 0) >= 1);
}

CT_TEST(portfolio_runtime_reuse_bounded_overhead) {
    // K=12 portfolio (one shared graph) vs 12 independent single-path runs
    // (12 graph rebuilds): reuse must show bounded overhead, and total search
    // work must stay within a linear bound of one single run.
    Board b = bottleneck_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    RouteTree tree = build_route_tree(b, 0);
    ConnectionTask task = tree.tasks.front();
    std::vector<double> lm = layer_mult_for(b);
    CongestionMap cg;
    cg.init(b);
    ReservationSet rs;
    Corridor cc = probable_corridor(b, r, task, ctx);
    rs.build({task}, {cc}, {1.0});
    HierarchyConfig hier;
    HierarchyCache cache;
    auto single_run = [&]() {
        auto t0 = std::chrono::steady_clock::now();
        CandidateRoute cr = route_candidate_task(b, r, task, 0, 1.0, ctx, lm,
                                                 AStarConfig{}, cg, rs, hier, &cache);
        auto t1 = std::chrono::steady_clock::now();
        double s = std::chrono::duration<double>(t1 - t0).count();
        return std::make_pair(cr, s);
    };
    auto [c0, s0] = single_run();
    CT_CHECK(c0.found);
    double indep = 0;
    for (int i = 0; i < 12; ++i) {
        auto [cr, s] = single_run();
        indep += s;
        CT_CHECK(cr.found);
    }
    auto p0 = std::chrono::steady_clock::now();
    PortfolioResult pf = build_portfolio(b, r, task, 0, 1.0, ctx, lm, AStarConfig{},
                                         cg, rs, hier, &cache, opts_k(12));
    auto p1 = std::chrono::steady_clock::now();
    double psec = std::chrono::duration<double>(p1 - p0).count();
    std::printf("  [portfolio] K=12 %.3fs vs 12x-independent %.3fs (single %.3fs)\n", psec,
                indep, s0);
    std::printf("  [portfolio] size=%d builds=%d total_exp=%lld single_exp=%lld\n",
                (int)pf.size(), pf.graph_builds, (long long)pf.total_expansions,
                (long long)c0.expansions);
    CT_CHECK(pf.size() >= 2);
    CT_CHECK(pf.graph_builds == 1);  // structural reuse, not 12 rebuilds
    // Bounded overhead: portfolio faster than 12 full rebuilds (generous 0.95x
    // to tolerate timer noise; reuse typically lands ~0.3-0.6x).
    CT_CHECK(psec < indep * 0.95 + 0.05);
    // Search work bounded: total expansions within 20x one single run.
    CT_CHECK(pf.total_expansions <= std::max<std::int64_t>(20 * c0.expansions, 20));
    JsonValue j = pf.to_json();
    CT_CHECK(j.has("size") && j.has("candidates"));
}

int main() { return copperline::test::run_all_tests(); }
