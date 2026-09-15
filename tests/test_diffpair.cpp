// Issue #12: reserve differential-pair corridors, then materialize coupled
// pairs post-route.
//
// Covers: pair metadata, single atomic PairCorridorTask (not two routes),
// occupied-width sizing, materialization legality through arbitrary-angle
// bends, paired vias, atomic one-member-only rejection, rip-up atomicity
// and determinism across thread counts.
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

#include "helpers.h"

#include "router/analyze.h"
#include "router/diffpair.h"
#include "router/engine.h"
#include "router/parallel.h"
#include "router/recovery.h"
#include "router/verifier.h"

using namespace copperline;
using namespace copperline::test;

#ifndef FIXTURE_DIR
#define FIXTURE_DIR "fixtures"
#endif

namespace {

Board load_fixture_board(const std::string& name) {
    ImportResult r = import_board_auto(std::string(FIXTURE_DIR) + "/" + name);
    return std::move(r.board);
}

bool verify_ok(const Board& board) {
    BoardVerifier v;
    ElectricalContext ctx;
    RuleResolver r = RuleResolver::defaults_for(board);
    return v.verify(board, r, ctx).ok;
}

int count_traces(const Board& b, NetId net) {
    int n = 0;
    for (const auto& t : b.traces)
        if (t.net == net) ++n;
    return n;
}

int count_vias(const Board& b, NetId net) {
    int n = 0;
    for (const auto& v : b.vias)
        if (v.net == net) ++n;
    return n;
}

// Minimum edge-to-edge gap between P/N trace sets on shared layers.
double min_edge_gap_mm(const Board& b, NetId p, NetId n) {
    double best = 1e9;
    for (const auto& a : b.traces) {
        if (a.net != p) continue;
        for (const auto& c : b.traces) {
            if (c.net != n || c.layer != a.layer) continue;
            double d = std::sqrt(static_cast<double>(seg_seg_dist2(a.segment(), c.segment())));
            double edge = (d - (a.width_nm + c.width_nm) / 2.0) / 1e6;
            best = std::min(best, edge);
        }
    }
    return best;
}

int bend_count(const Board& b, NetId net) {
    std::map<LayerId, std::vector<TraceSeg>> by_layer;
    for (const auto& t : b.traces) {
        if (t.net == net) by_layer[t.layer].push_back(t);
    }
    int bends = 0;
    for (auto& [layer, segs] : by_layer) {
        std::sort(segs.begin(), segs.end(), [](const TraceSeg& a, const TraceSeg& b) {
            if (a.a.x != b.a.x) return a.a.x < b.a.x;
            if (a.a.y != b.a.y) return a.a.y < b.a.y;
            if (a.b.x != b.b.x) return a.b.x < b.b.x;
            return a.b.y < b.b.y;
        });
        for (std::size_t i = 0; i + 1 < segs.size(); ++i) {
            Coord ux = segs[i].b.x - segs[i].a.x, uy = segs[i].b.y - segs[i].a.y;
            Coord vx = segs[i + 1].b.x - segs[i + 1].a.x,
                  vy = segs[i + 1].b.y - segs[i + 1].a.y;
            if ((__int128)ux * vy != (__int128)uy * vx) ++bends;
        }
    }
    return bends;
}

}  // namespace

CT_TEST(pair_metadata_parses_and_validates) {
    Board b = load_fixture_board("diffpair_basic.json");
    CT_CHECK(b.diffpairs.size() == 1);
    const DiffPair& pr = b.diffpairs.front();
    CT_CHECK(pr.id == 0);
    CT_CHECK(pr.name == "USB");
    CT_CHECK(pr.net_p == 0 && pr.net_n == 1);
    CT_CHECK(pr.gap_nm == mm_to_nm(0.2));
    CT_CHECK(pr.gap_tol_nm == mm_to_nm(0.05));
    CT_CHECK(pr.has_width && pr.width_nm == mm_to_nm(0.2));
    CT_CHECK(pr.has_max_skew && pr.max_skew_nm == mm_to_nm(1.0));
    CT_CHECK(pr.via_policy == "paired");
    std::string reason;
    CT_CHECK(diffpair_valid(b, pr, reason));
    TermId pa, pb, na, nb;
    CT_CHECK(diffpair_endpoints(b, pr, pa, pb, na, nb, reason));
    CT_CHECK(pa == 0 && pb == 1 && na == 2 && nb == 3);
    // Round-trip through native JSON.
    JsonValue back = board_to_json(b);
    const JsonValue* dp = back.find("diffpairs");
    CT_CHECK(dp && dp->is_array() && dp->as_array().size() == 1);
    CT_CHECK(dp->as_array()[0].get_number("gap_mm", 0) > 0.19);
    // Analyze reports the pair inventory.
    RuleResolver r = RuleResolver::defaults_for(b);
    AnalysisResult a = analyze_board(b, r, r.defaultContext(), {});
    const JsonValue* adp = a.data.find("diffpairs");
    CT_CHECK(adp && adp->is_array() && adp->as_array().size() == 1);
    CT_CHECK(adp->as_array()[0].get_bool("valid", false));
}

CT_TEST(occupied_width_is_both_traces_plus_gap_plus_clearance) {
    Board b = load_fixture_board("diffpair_basic.json");
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    const DiffPair& pr = b.diffpairs.front();
    // 0.2 (P) + 0.2 (N) + 0.2 (gap) + 2 * 0.15 (external) = 0.9 mm.
    CT_CHECK(diffpair_occupied_width(b, r, pr, ctx) == mm_to_nm(0.9));
    ConnectionTask t;
    std::string reason;
    CT_CHECK(make_pair_corridor_task(b, pr, 0, t, reason));
    CT_CHECK(t.is_pair_corridor && t.pair_id == 0);
    CT_CHECK(t.net == 0 && t.pair_other_net == 1);
    // Corridor problem width matches the occupied envelope.
    Corridor c = probable_corridor(b, r, t, ctx);
    CT_CHECK(c.width_nm == mm_to_nm(0.9));
}

CT_TEST(global_stage_produces_one_corridor_not_two_routes) {
    Board b = load_fixture_board("diffpair_basic.json");
    std::vector<std::string> reasons;
    std::vector<int> bad;
    std::vector<ConnectionTask> tasks = build_global_tasks_with_pairs(b, reasons, bad);
    CT_CHECK(bad.empty());
    // 1 ordinary SIG task + 1 atomic pair corridor (never 2 pair members).
    CT_CHECK(tasks.size() == 2);
    int corridors = 0, sig_tasks = 0, member_tasks = 0;
    for (const auto& t : tasks) {
        if (t.is_pair_corridor) {
            ++corridors;
            CT_CHECK(t.net == 0 && t.pair_other_net == 1);
        } else if (t.net == 2) {
            ++sig_tasks;
        } else if (t.net == 0 || t.net == 1) {
            ++member_tasks;
        }
    }
    CT_CHECK(corridors == 1);
    CT_CHECK(sig_tasks == 1);
    CT_CHECK(member_tasks == 0);
    // Deterministic construction.
    std::vector<ConnectionTask> again = build_global_tasks_with_pairs(b, reasons, bad);
    CT_CHECK(again.size() == tasks.size());
    for (std::size_t i = 0; i < tasks.size(); ++i) {
        CT_CHECK(again[i].net == tasks[i].net);
        CT_CHECK(again[i].is_pair_corridor == tasks[i].is_pair_corridor);
    }
}

CT_TEST(route_basic_pair_complete_and_legal) {
    Board b = load_fixture_board("diffpair_basic.json");
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = 1;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.verification.ok);
    CT_CHECK(verify_ok(engine.committed()));
    // Both members committed, none left dangling.
    CT_CHECK(count_traces(engine.committed(), 0) > 0);
    CT_CHECK(count_traces(engine.committed(), 1) > 0);
    CT_CHECK(rep.verification.unconnected.empty());
    // Gap respected: 0.2 +/- 0.05 -> min edge >= 0.15.
    double gmin = min_edge_gap_mm(engine.committed(), 0, 1);
    CT_CHECK(gmin >= 0.15 - 1e-9);
    // Report carries the materialized pair.
    CT_CHECK(rep.diffpairs.size() == 1);
    CT_CHECK(rep.diffpairs.front().materialized);
    CT_CHECK(rep.diffpairs.front().status == "MATERIALIZED");
    JsonValue j = rep.to_json();
    const JsonValue* dp = j.find("diffpairs");
    CT_CHECK(dp && dp->is_array() && dp->as_array().size() == 1);
    CT_CHECK(dp->as_array()[0].get_bool("materialized", false));
}

CT_TEST(materialization_legal_through_bends) {
    Board b = load_fixture_board("diffpair_bend.json");
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(verify_ok(engine.committed()));
    // The keepout forces a detour: the pair bends but stays coupled.
    CT_CHECK(bend_count(engine.committed(), 0) >= 1);
    CT_CHECK(bend_count(engine.committed(), 1) >= 1);
    // No 45-degree dogleg explosion: each member uses few segments.
    CT_CHECK(count_traces(engine.committed(), 0) <= 6);
    CT_CHECK(count_traces(engine.committed(), 1) <= 6);
    double gmin = min_edge_gap_mm(engine.committed(), 0, 1);
    CT_CHECK(gmin >= 0.15 - 1e-9);
    // Skew within the declared 2 mm budget.
    CT_CHECK(rep.diffpairs.size() == 1);
    CT_CHECK(rep.diffpairs.front().skew_mm <= 2.0 + 1e-9);
}

CT_TEST(paired_vias_share_span_and_count) {
    Board b = load_fixture_board("diffpair_via.json");
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(verify_ok(engine.committed()));
    int vp = count_vias(engine.committed(), 0);
    int vn = count_vias(engine.committed(), 1);
    CT_CHECK(vp > 0 && vn > 0);
    CT_CHECK(vp == vn);  // paired transitions, never solo
    // Spans match pairwise (sorted by position for determinism).
    std::vector<Via> p, n;
    for (const auto& v : engine.committed().vias) {
        if (v.net == 0) p.push_back(v);
        if (v.net == 1) n.push_back(v);
    }
    auto by_pos = [](const Via& a, const Via& b) {
        if (a.pos.x != b.pos.x) return a.pos.x < b.pos.x;
        return a.pos.y < b.pos.y;
    };
    std::sort(p.begin(), p.end(), by_pos);
    std::sort(n.begin(), n.end(), by_pos);
    CT_CHECK(p.size() == n.size());
    for (std::size_t i = 0; i < p.size(); ++i) {
        CT_CHECK(p[i].top_layer == n[i].top_layer);
        CT_CHECK(p[i].bottom_layer == n[i].bottom_layer);
        CT_CHECK(p[i].via_class == n[i].via_class);
    }
    CT_CHECK(rep.diffpairs.size() == 1);
    CT_CHECK(rep.diffpairs.front().via_pairs >= 1);
}

CT_TEST(one_member_only_rejected_atomically) {
    // Hand-built straight corridor with a thin keepout sliver exactly on the
    // N member's path: P stays legal, N cannot. Materialization must fail
    // with empty outputs (never a P-only commit).
    Board b = load_fixture_board("diffpair_basic.json");
    // Strip y 10.40..10.50 sits inside N copper (10.35..10.55) but clear of
    // P copper (9.95..10.15).
    Keepout sliver;
    sliver.rect = {mm_to_nm(5.0), mm_to_nm(0.40 + 10.0), mm_to_nm(15.0),
                   mm_to_nm(0.50 + 10.0)};
    sliver.layer = kAllLayers;
    sliver.reason = "n_sliver";
    b.keepouts.push_back(sliver);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    const DiffPair& pr = b.diffpairs.front();
    PairCorridor corr;
    corr.pair_id = pr.id;
    corr.net_p = pr.net_p;
    corr.net_n = pr.net_n;
    corr.occupied_width_nm = diffpair_occupied_width(b, r, pr, ctx);
    corr.center_traces.push_back(
        {pr.net_p, 0, {mm_to_nm(2.0), mm_to_nm(10.25)}, {mm_to_nm(18.0), mm_to_nm(10.25)},
         corr.occupied_width_nm});
    MaterializedPair mat = materialize_pair(b, r, ctx, pr, corr);
    CT_CHECK(!mat.ok);
    CT_CHECK(mat.traces_p.empty() && mat.traces_n.empty());
    CT_CHECK(mat.vias_p.empty() && mat.vias_n.empty());
    // End-to-end the same board never completes with P-only copper: the
    // corridor is revoked and both members report failures.
    EngineOptions opt;
    opt.enable_ripup = false;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status != "COMPLETE");
    bool saw_pair_fail = false;
    for (const auto& f : rep.failures) {
        if (f.is_pair_corridor) saw_pair_fail = true;
    }
    CT_CHECK(saw_pair_fail);
    // Atomicity: no board where P is connected and N is not.
    int p_traces = count_traces(engine.committed(), 0);
    int n_traces = count_traces(engine.committed(), 1);
    CT_CHECK(!(p_traces > 0 && n_traces == 0));
}

CT_TEST(pair_ripup_is_atomic_and_deterministic) {
    // Rip-up atomicity: a move against a pair corridor rips the single
    // owned object (both members together), never half the pair.
    Board b = load_fixture_board("diffpair_basic.json");
    ConnectionTask corr;
    std::string reason;
    CT_CHECK(make_pair_corridor_task(b, b.diffpairs.front(), 0, corr, reason));
    corr.difficulty = 5.0;
    OwnedRoute o_pair;
    o_pair.task = corr;
    o_pair.task_pos = 0;
    o_pair.is_pair_corridor = true;
    o_pair.pair_id = 0;
    o_pair.traces.push_back({0, 0, {0, 0}, {1000000, 0}, mm_to_nm(0.9)});
    o_pair.protection = route_protection_score(false, 5.0, 0, false, RecoveryMode::FAST);
    ConnectionTask t_norm{2, 4, 5, 1, 1.0};
    OwnedRoute o_norm;
    o_norm.task = t_norm;
    o_norm.task_pos = 1;
    o_norm.protection = route_protection_score(false, 1.0, 0, false, RecoveryMode::FAST);
    ConnectionTask t_fail{2, 4, 5, 2, 9.0};
    DependencyGraph g;
    g.failed.push_back(t_fail);
    DependencyEdge e;
    e.failed_pos = 0;
    e.failed_net = 2;
    e.blocker_net = 0;
    e.blocker_desc = "trace:net=USB_P";
    e.weight = 100.0;
    g.edges.push_back(e);
    HistoryHeuristic hh;
    auto moves = generate_ripup_moves(g.failed, g, {o_pair, o_norm}, hh, "",
                                      RecoveryMode::FAST, 4, 2);
    CT_CHECK(!moves.empty());
    // The pair corridor is ripped as exactly one owned entry.
    bool rips_pair = false;
    for (const auto& m : moves) {
        if (m.owned_idx.size() == 1 && m.owned_idx[0] == 0) rips_pair = true;
        // Never a fractional pair rip: owned_idx entries are whole routes.
        for (int oi : m.owned_idx) CT_CHECK(oi == 0 || oi == 1);
    }
    CT_CHECK(rips_pair);
    // Determinism across thread counts: identical copper and hash.
    std::string h0;
    for (int threads : {1, 2, 4}) {
        Board bb = load_fixture_board("diffpair_basic.json");
        RuleResolver rr = RuleResolver::defaults_for(bb);
        EngineOptions opt;
        opt.threads = threads;
        RouterEngine eng(std::move(bb), std::move(rr), opt);
        RouteReport rep = eng.run();
        CT_CHECK(rep.status == "COMPLETE");
        if (h0.empty()) {
            h0 = rep.board_hash;
        } else {
            CT_CHECK(rep.board_hash == h0);
        }
    }
}

CT_TEST(pair_gap_band_enforced_with_location) {
    // Issue #26: coupled sections are banded [gap-tol, gap+tol], not just
    // floored. Bare straight members, nominal 0.20 tol 0.05, width 0.2.
    auto straight = [](double n_y_mm, LayerId layer = 0) {
        std::vector<TraceSeg> tp, tn;
        tp.push_back({0, layer, {mm_to_nm(2.0), mm_to_nm(10.0)},
                      {mm_to_nm(18.0), mm_to_nm(10.0)}, mm_to_nm(0.2)});
        tn.push_back({1, layer, {mm_to_nm(2.0), mm_to_nm(n_y_mm)},
                      {mm_to_nm(18.0), mm_to_nm(n_y_mm)}, mm_to_nm(0.2)});
        return std::pair<std::vector<TraceSeg>, std::vector<TraceSeg>>(tp, tn);
    };
    const Coord gap = mm_to_nm(0.2), tol = mm_to_nm(0.05);
    {
        // Nominal 0.20 edge: pass, zero error.
        auto [tp, tn] = straight(10.4);
        Coord worst = -1;
        Point at{0, 0};
        CT_CHECK(pair_gap_legal(tp, tn, gap, tol, worst, at));
        CT_CHECK(worst == 0);
    }
    {
        // Upper-bound edge 0.25: inclusive pass.
        auto [tp, tn] = straight(10.45);
        Coord worst = -1;
        Point at{0, 0};
        CT_CHECK(pair_gap_legal(tp, tn, gap, tol, worst, at));
        CT_CHECK(worst == mm_to_nm(0.05));
    }
    {
        // 0.26: first step past the band fails with a location.
        auto [tp, tn] = straight(10.46);
        Coord worst = 0;
        Point at{0, 0};
        CT_CHECK(!pair_gap_legal(tp, tn, gap, tol, worst, at));
        CT_CHECK(worst == mm_to_nm(0.06));
        Point want{mm_to_nm(2.0),
                   (mm_to_nm(10.0) + mm_to_nm(10.46)) / 2};
        CT_CHECK(at == want);
    }
    {
        // Issue example: 0.20+-0.05 at 0.50 separation fails.
        auto [tp, tn] = straight(10.7);
        Coord worst = 0;
        Point at{0, 0};
        CT_CHECK(!pair_gap_legal(tp, tn, gap, tol, worst, at));
        CT_CHECK(worst == mm_to_nm(0.30));
    }
    {
        // Different layers: no coupled section, bare check passes
        // (never-coupled uncoupling is the verifier's path, not this one).
        auto [tp, tn] = straight(10.7, 0);
        for (auto& s : tn) s.layer = 1;
        Coord worst = 0;
        Point at{0, 0};
        CT_CHECK(pair_gap_legal(tp, tn, gap, tol, worst, at));
    }
    {
        // Fanout exemption: the same 0.26 drift passes when both segments
        // terminate at member pads (#12 pad-column stitching envelope),
        // floor-checked only. Exemption is per segment: a bare trunk with
        // no pad context stays strict (cases above).
        auto [tp, tn] = straight(10.46);
        std::vector<Point> fanout{tp.front().a, tn.front().a};
        Coord worst = 0;
        Point at{0, 0};
        CT_CHECK(pair_gap_legal_fanout(tp, tn, gap, tol, fanout, worst, at));
        // ...while the trunk-only middle of a longer run still fails (the
        // verifier covers this end to end; here just the API contract).
        CT_CHECK(!pair_gap_legal(tp, tn, gap, tol, worst, at));
    }
}

int main() { return copperline::test::run_all_tests(); }
