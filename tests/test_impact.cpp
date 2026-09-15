// Issue #9: future-obstruction scoring for route portfolios.
#include <cmath>
#include <limits>
#include <vector>

#include "helpers.h"
#include "router/density.h"
#include "router/impact.h"
#include "router/parallel.h"
#include "router/route_tree.h"

using namespace copperline;
using namespace copperline::test;

namespace {

Board two_net_board(bool high_current_second = false) {
    Board b = base_2layer(20.0, 20.0);
    NetInfo n0 = make_net(0, "SIG");
    n0.has_current = true;
    n0.current_a = 0.1;
    b.nets.push_back(n0);
    add_terminal(b, 0, 2.0, 10.0, 0);
    add_terminal(b, 0, 18.0, 10.0, 0);
    NetInfo n1 = make_net(1, "LATER");
    n1.has_current = true;
    n1.current_a = high_current_second ? 5.0 : 0.1;
    b.nets.push_back(n1);
    add_terminal(b, 1, 10.0, 2.0, 0);
    add_terminal(b, 1, 10.0, 18.0, 0);
    return b;
}

CandidateRoute make_route(NetId net, TermId a, TermId b,
                          const std::vector<std::pair<Point, Point>>& segs,
                          Coord width_nm, Coord cost_nm) {
    CandidateRoute c;
    c.task.net = net;
    c.task.a = a;
    c.task.b = b;
    c.task_index = 0;
    c.found = true;
    c.cost_nm = cost_nm;
    for (const auto& [p, q] : segs) c.traces.push_back({net, 0, p, q, width_nm});
    if (!segs.empty()) {
        c.gate_a = segs.front().first;
        c.gate_b = segs.back().second;
    }
    return c;
}

Point pt(double x_mm, double y_mm) { return {mm_to_nm(x_mm), mm_to_nm(y_mm)}; }

ImpactContext base_ctx(const Board& board, const RuleResolver& resolver,
                       const ElectricalContext& ectx, const CongestionMap& cg,
                       const DensityResult& dens,
                       const std::vector<ConnectionTask>& rem_tasks,
                       const std::vector<Corridor>& rem_corr) {
    ImpactContext c;
    c.board = &board;
    c.resolver = &resolver;
    c.ctx = &ectx;
    c.remaining_tasks = rem_tasks;
    c.remaining_corridors = rem_corr;
    c.terminal_density = dens.terminal_density;
    c.congestion = &cg;
    return c;
}

}  // namespace

CT_TEST(impact_bottleneck_prefers_detour) {
    // Only-corridor preservation: the cheap straight thief covers the later
    // net's only corridor; the slightly longer detour avoids it and must win
    // despite higher base cost.
    Board b = two_net_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    DensityEstimator est;
    DensityResult dens = est.analyze(b);
    CongestionMap cg;
    cg.init(b);

    ConnectionTask later;
    later.net = 1;
    later.a = b.nets[1].terminals[0];
    later.b = b.nets[1].terminals[1];
    Corridor corr;
    corr.rect = {mm_to_nm(9.0), mm_to_nm(9.0), mm_to_nm(11.0), mm_to_nm(11.0)};
    corr.width_nm = mm_to_nm(0.2);
    corr.clear_nm = mm_to_nm(0.15);

    TermId a = b.nets[0].terminals[0], bb = b.nets[0].terminals[1];
    Coord w = mm_to_nm(0.2);
    CandidateRoute cheap =
        make_route(0, a, bb, {{{pt(2, 10), pt(18, 10)}}}, w, mm_to_nm(16.0));
    CandidateRoute detour =
        make_route(0, a, bb,
                   {{pt(2, 10), pt(2, 7)}, {pt(2, 7), pt(18, 7)}, {pt(18, 7), pt(18, 10)}},
                   w, mm_to_nm(22.0));
    CT_CHECK(detour.cost_nm > cheap.cost_nm);  // slightly longer, must still win
    std::string why;
    CT_CHECK(candidate_legal_vs_board(cheap, b, r, ctx, why));
    CT_CHECK(candidate_legal_vs_board(detour, b, r, ctx, why));

    ImpactContext ictx = base_ctx(b, r, ctx, cg, dens, {later}, {corr});
    WeightedImpactScorer scorer;
    ImpactScore s_cheap = scorer.score(cheap, ictx);
    ImpactScore s_det = scorer.score(detour, ictx);
    // Overlap + scarcity fire only for the thief.
    CT_CHECK(s_cheap.features.future_corridor_overlap > 0.0);
    CT_CHECK(s_det.features.future_corridor_overlap == 0.0);
    CT_CHECK(s_cheap.features.bottleneck_scarcity == 1.0);
    CT_CHECK(s_det.features.bottleneck_scarcity == 0.0);
    CT_CHECK(s_det.total < s_cheap.total);

    std::vector<CandidateRoute> cands = {cheap, detour};
    std::vector<ImpactScore> scores = {s_cheap, s_det};
    std::vector<int> order = rank_candidates_by_impact(cands, scores);
    CT_CHECK(order.size() == 2);
    CT_CHECK(order[0] == 1);  // detour first despite higher cost
    ImpactOptions opts;
    CT_CHECK(select_best_candidate(cands, scores, opts) == 1);
    // Per-feature contributions + final score are logged.
    JsonValue j = s_det.to_json();
    CT_CHECK(j.has("features") && j.has("contributions") && j.has("total"));
    CT_CHECK(j.has("scorer"));
}

CT_TEST(impact_electrical_footprint) {
    // High-current / high-clearance nets reflect a larger footprint in the
    // normalized electrical features and total.
    Board b = base_2layer(20.0, 20.0);
    NetInfo low = make_net(0, "SIG");
    low.has_current = true;
    low.current_a = 0.1;
    b.nets.push_back(low);
    add_terminal(b, 0, 2.0, 10.0, 0);
    add_terminal(b, 0, 18.0, 10.0, 0);
    NetInfo high = make_net(1, "PWR");
    high.has_current = true;
    high.current_a = 5.0;
    high.has_min_clearance = true;
    high.min_clearance_nm = mm_to_nm(0.5);
    b.nets.push_back(high);
    add_terminal(b, 1, 2.0, 12.0, 0);
    add_terminal(b, 1, 18.0, 12.0, 0);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    DensityEstimator est;
    DensityResult dens = est.analyze(b);
    CongestionMap cg;
    cg.init(b);

    TermId la = b.nets[0].terminals[0], lb = b.nets[0].terminals[1];
    TermId ha = b.nets[1].terminals[0], hb = b.nets[1].terminals[1];
    std::string src;
    Coord wl = r.requiredTraceWidth(0, 0, ctx, &src);
    Coord wh = r.requiredTraceWidth(1, 0, ctx, &src);
    CT_CHECK(wh > wl);  // high current forces wider copper
    CandidateRoute c_low =
        make_route(0, la, lb, {{{pt(2, 10), pt(18, 10)}}}, wl, mm_to_nm(16.0));
    CandidateRoute c_high =
        make_route(1, ha, hb, {{{pt(2, 12), pt(18, 12)}}}, wh, mm_to_nm(16.0));

    ImpactContext xl = base_ctx(b, r, ctx, cg, dens, {}, {});
    ImpactContext xh = base_ctx(b, r, ctx, cg, dens, {}, {});
    WeightedImpactScorer scorer;
    ImpactScore sl = scorer.score(c_low, xl);
    ImpactScore sh = scorer.score(c_high, xh);
    CT_CHECK(sh.features.current_width > sl.features.current_width);
    CT_CHECK(sh.features.consumed_footprint > sl.features.consumed_footprint);
    CT_CHECK(sh.features.voltage_clearance >= sl.features.voltage_clearance);
    CT_CHECK(sh.total > sl.total);
    for (int i = 0; i < kImpactFeatureCount; ++i) {
        CT_CHECK(sl.features.at(i) >= 0.0 && sl.features.at(i) <= 1.0);
        CT_CHECK(sh.features.at(i) >= 0.0 && sh.features.at(i) <= 1.0);
    }
}

CT_TEST(impact_deterministic_ranking) {
    Board b = two_net_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    DensityEstimator est;
    DensityResult dens = est.analyze(b);
    CongestionMap cg;
    cg.init(b);
    ConnectionTask later;
    later.net = 1;
    Corridor corr;
    corr.rect = {mm_to_nm(9.0), mm_to_nm(9.0), mm_to_nm(11.0), mm_to_nm(11.0)};
    ImpactContext ictx = base_ctx(b, r, ctx, cg, dens, {later}, {corr});
    TermId a = b.nets[0].terminals[0], bb = b.nets[0].terminals[1];
    Coord w = mm_to_nm(0.2);
    CandidateRoute c0 = make_route(0, a, bb, {{{pt(2, 10), pt(18, 10)}}}, w, mm_to_nm(16.0));
    CandidateRoute c1 = make_route(
        0, a, bb, {{pt(2, 10), pt(2, 7)}, {pt(2, 7), pt(18, 7)}, {pt(18, 7), pt(18, 10)}}, w,
        mm_to_nm(22.0));
    std::vector<CandidateRoute> cands = {c0, c1};
    ImpactOptions opts;
    std::vector<ImpactScore> s1, s2;
    int b1 = score_and_select(cands, ictx, opts, s1);
    int b2 = score_and_select(cands, ictx, opts, s2);
    CT_CHECK(b1 == b2);
    CT_CHECK(s1.size() == 2 && s2.size() == 2);
    for (std::size_t i = 0; i < s1.size(); ++i) {
        CT_CHECK(s1[i].total == s2[i].total);
        for (int k = 0; k < kImpactFeatureCount; ++k)
            CT_CHECK(s1[i].contributions[k] == s2[i].contributions[k]);
    }
    CT_CHECK(rank_candidates_by_impact(cands, s1) == rank_candidates_by_impact(cands, s2));
    // MLP scorer with the explicit fixed config is equally deterministic.
    ImpactOptions mopts;
    mopts.use_mlp = true;
    std::vector<ImpactScore> m1, m2;
    CT_CHECK(score_and_select(cands, ictx, mopts, m1) ==
             score_and_select(cands, ictx, mopts, m2));
    CT_CHECK(m1[0].total == m2[0].total && m1[1].total == m2[1].total);
}

CT_TEST(impact_disabled_reverts_to_base_cost) {
    Board b = two_net_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    DensityEstimator est;
    DensityResult dens = est.analyze(b);
    CongestionMap cg;
    cg.init(b);
    ConnectionTask later;
    later.net = 1;
    Corridor corr;
    corr.rect = {mm_to_nm(9.0), mm_to_nm(9.0), mm_to_nm(11.0), mm_to_nm(11.0)};
    ImpactContext ictx = base_ctx(b, r, ctx, cg, dens, {later}, {corr});
    TermId a = b.nets[0].terminals[0], bb = b.nets[0].terminals[1];
    Coord w = mm_to_nm(0.2);
    CandidateRoute cheap =
        make_route(0, a, bb, {{{pt(2, 10), pt(18, 10)}}}, w, mm_to_nm(16.0));
    CandidateRoute detour = make_route(
        0, a, bb, {{pt(2, 10), pt(2, 7)}, {pt(2, 7), pt(18, 7)}, {pt(18, 7), pt(18, 10)}}, w,
        mm_to_nm(22.0));
    std::vector<CandidateRoute> cands = {cheap, detour};
    ImpactOptions on;
    std::vector<ImpactScore> scores;
    CT_CHECK(score_and_select(cands, ictx, on, scores) == 1);  // detour wins
    ImpactOptions off;
    off.enabled = false;
    // Same legality gate: both legal, so removal picks the cheapest.
    CT_CHECK(scores[0].legal && scores[1].legal);
    CT_CHECK(select_best_candidate(cands, scores, off) == 0);
}

CT_TEST(impact_mlp_fallback_and_bounds) {
    Board b = two_net_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    DensityEstimator est;
    DensityResult dens = est.analyze(b);
    CongestionMap cg;
    cg.init(b);
    ImpactContext ictx = base_ctx(b, r, ctx, cg, dens, {}, {});
    TermId a = b.nets[0].terminals[0], bb = b.nets[0].terminals[1];
    CandidateRoute c =
        make_route(0, a, bb, {{{pt(2, 10), pt(18, 10)}}}, mm_to_nm(0.2), mm_to_nm(16.0));
    // Valid MLP: bounded total, legal-first ranking key preserved.
    MlpImpactScorer mlp;
    ImpactScore s = mlp.score(c, ictx);
    CT_CHECK(s.total >= 0.0 && s.total <= 1.0);
    CT_CHECK(s.scorer == "mlp");
    CT_CHECK(s.legal);
    // Invalid config (NaN) falls back to the weighted baseline deterministically.
    MlpImpactConfig bad = MlpImpactConfig::fixed_default();
    bad.w1[0] = std::numeric_limits<double>::quiet_NaN();
    CT_CHECK(!bad.valid());
    MlpImpactScorer fb(bad);
    ImpactScore sf = fb.score(c, ictx);
    CT_CHECK(sf.fallback);
    WeightedImpactScorer w;
    ImpactScore sw = w.score(c, ictx);
    CT_CHECK(sf.total == sw.total);
    // Threads: explicit wins, 0 = auto; memory bound lives on the options.
    CT_CHECK(resolve_worker_threads(2) == 2);
    CT_CHECK(resolve_worker_threads(0) >= 1);
    ImpactOptions o;
    CT_CHECK(o.memory_budget_bytes == kRouterMemoryBudgetBytes);
    CT_CHECK(o.weights.sum() > 0.99 && o.weights.sum() < 1.01);
}

int main() { return copperline::test::run_all_tests(); }
