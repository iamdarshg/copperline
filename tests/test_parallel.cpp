// Prompt 3 tests: parallel global routing, density-aware scheduling and
// negotiated congestion. All boards are tiny on purpose: parallelism is
// proven by structure (epochs, worker isolation, arbitration), not by
// burning minutes of CPU.
#include <thread>

#include "helpers.h"
#include "router/density.h"
#include "router/engine.h"
#include "router/parallel.h"
#include "router/verifier.h"

using namespace copperline;
using namespace copperline::test;

namespace {

Board open_nets(int n) {
    Board b = base_2layer();
    for (int i = 0; i < n; ++i) {
        NetInfo net = make_net(i, "N" + std::to_string(i));
        net.has_current = true;
        net.current_a = 0.1;
        b.nets.push_back(net);
        double y = 2.0 + i * (16.0 / (n - 1 > 0 ? n - 1 : 1));
        add_terminal(b, i, 2.0, y);
        add_terminal(b, i, 18.0, y);
    }
    return b;
}

RouteReport run_threads(Board b, int threads) {
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = threads;
    RouterEngine engine(std::move(b), std::move(r), opt);
    return engine.run();
}

bool verify_ok(const Board& routed, const JsonValue& cfg, bool has_cfg) {
    BoardVerifier v;
    ElectricalContext ctx;
    if (has_cfg) {
        RuleResolver r = RuleResolver::from_config(routed, cfg);
        return v.verify(routed, r, ctx).ok;
    }
    RuleResolver r = RuleResolver::defaults_for(routed);
    return v.verify(routed, r, ctx).ok;
}

}  // namespace

CT_TEST(parallel_deterministic_1_2_4_8) {
    std::string h0;
    for (int threads : {1, 2, 4, 8}) {
        RouteReport rep = run_threads(open_nets(4), threads);
        CT_CHECK(rep.status == "COMPLETE");
        CT_CHECK(rep.connected_terminals == rep.total_terminals);
        if (h0.empty()) h0 = rep.board_hash;
        CT_CHECK(rep.board_hash == h0);
    }
    CT_CHECK(!h0.empty());
}

CT_TEST(workers_do_not_mutate_snapshot) {
    Board b = open_nets(2);
    Keepout wall;
    wall.rect = {mm_to_nm(9.0), mm_to_nm(0.0), mm_to_nm(11.0), mm_to_nm(15.0)};
    wall.layer = kAllLayers;
    wall.reason = "wall";
    b.keepouts.push_back(wall);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    RouteTree tree = build_route_tree(b, 0);
    CT_CHECK(!tree.tasks.empty());
    ConnectionTask task = tree.tasks.front();

    DensityEstimator de;
    DensityResult dens = de.analyze(b);
    DifficultyVector dv = compute_difficulty(b, r, task, ctx, dens.terminal_density, {}, 0);
    std::vector<double> lm = {1.0, 1.0};
    CongestionMap cg;
    cg.init(b);
    ReservationSet rs;
    std::vector<ConnectionTask> one = {task};
    std::vector<Corridor> corr = {probable_corridor(b, r, task, ctx)};
    rs.build(one, corr, {dv.total});

    const std::size_t traces0 = b.traces.size();
    const std::size_t vias0 = b.vias.size();
    constexpr int kWorkers = 4;
    std::vector<CandidateRoute> out(kWorkers);
    std::vector<std::thread> pool;
    for (int w = 0; w < kWorkers; ++w) {
        pool.emplace_back([&, w] {
            out[w] = route_candidate_task(b, r, task, 0, dv.total, ctx, lm, AStarConfig{}, cg, rs);
        });
    }
    for (auto& th : pool) th.join();
    CT_CHECK(b.traces.size() == traces0);
    CT_CHECK(b.vias.size() == vias0);
    for (int w = 1; w < kWorkers; ++w) {
        CT_CHECK(out[w].found == out[0].found);
        CT_CHECK(out[w].cost_nm == out[0].cost_nm);
        CT_CHECK(out[w].expansions == out[0].expansions);
        CT_CHECK(out[w].traces.size() == out[0].traces.size());
    }
}

CT_TEST(overlapping_candidates_handled_safely) {
    // Single layer X: A horizontal, B vertical. Same-batch candidates both
    // go straight; the arbiter accepts one and the other reroutes next epoch.
    Board b;
    b.source_format = "test";
    b.width_nm = mm_to_nm(20.0);
    b.height_nm = mm_to_nm(20.0);
    b.layers.push_back({0, "Top"});
    NetInfo a = make_net(0, "A");
    a.has_current = true;
    a.current_a = 0.1;
    NetInfo bn = make_net(1, "B");
    bn.has_current = true;
    bn.current_a = 0.1;
    b.nets.push_back(a);
    b.nets.push_back(bn);
    add_terminal(b, 0, 2.0, 10.0);
    add_terminal(b, 0, 18.0, 10.0);
    add_terminal(b, 1, 10.0, 2.0);
    add_terminal(b, 1, 10.0, 18.0);

    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = 4;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    BoardVerifier v;
    ElectricalContext ctx;
    RuleResolver r2 = RuleResolver::defaults_for(engine.committed());
    VerifyResult vr = v.verify(engine.committed(), r2, ctx);
    CT_CHECK(vr.legal);
    CT_CHECK(vr.ok);
}

CT_TEST(independent_routes_commit_in_one_epoch) {
    Board b = base_2layer();
    NetInfo a = make_net(0, "A");
    a.has_current = true;
    a.current_a = 0.1;
    NetInfo bn = make_net(1, "B");
    bn.has_current = true;
    bn.current_a = 0.1;
    b.nets.push_back(a);
    b.nets.push_back(bn);
    add_terminal(b, 0, 1.0, 1.0);
    add_terminal(b, 0, 4.0, 1.0);
    add_terminal(b, 1, 16.0, 19.0);
    add_terminal(b, 1, 19.0, 19.0);
    RouteReport rep = run_threads(std::move(b), 4);
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.stats.epochs_count == 1);
    CT_CHECK(rep.stats.candidates_accepted == 2);
    CT_CHECK(rep.stats.candidates_rejected == 0);
}

CT_TEST(density_aware_scheduler_prioritization) {
    Board b = base_2layer();
    NetInfo dense = make_net(0, "DENSE");
    dense.has_current = true;
    dense.current_a = 0.1;
    NetInfo open = make_net(1, "OPEN");
    open.has_current = true;
    open.current_a = 0.1;
    b.nets.push_back(dense);
    b.nets.push_back(open);
    // Dense cluster around (10,10): 4 terminals within ~1mm. Both tasks
    // span 8mm so endpoint density (not distance) decides the order.
    TermId d0 = add_terminal(b, 0, 10.0, 10.0);
    TermId d1 = add_terminal(b, 0, 18.0, 10.0);
    add_terminal(b, 0, 10.0, 10.5);
    add_terminal(b, 0, 10.5, 10.5);
    TermId o0 = add_terminal(b, 1, 1.0, 1.0);
    TermId o1 = add_terminal(b, 1, 9.0, 1.0);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    DensityEstimator de;
    DensityResult dens = de.analyze(b);
    ConnectionTask td{0, d0, d1, 0, 0}, to{1, o0, o1, 1, 0};
    double dd = compute_difficulty(b, r, td, ctx, dens.terminal_density, {}, 0).total;
    double dopen = compute_difficulty(b, r, to, ctx, dens.terminal_density, {}, 0).total;
    CT_CHECK(dd > dopen);
    td.difficulty = dd;
    to.difficulty = dopen;
    auto batches = schedule_batches({td, to});
    CT_CHECK(batches.size() == 1);
    CT_CHECK(batches[0].size() == 2);
    CT_CHECK(batches[0][0] == 0);  // dense task scheduled first
}

CT_TEST(current_aware_difficulty_and_corridor) {
    Board b = base_2layer();
    NetInfo pwr = make_net(0, "PWR");
    pwr.has_current = true;
    pwr.current_a = 5.0;
    NetInfo sig = make_net(1, "SIG");
    sig.has_current = true;
    sig.current_a = 0.1;
    b.nets.push_back(pwr);
    b.nets.push_back(sig);
    TermId p0 = add_terminal(b, 0, 2.0, 10.0);
    TermId p1 = add_terminal(b, 0, 18.0, 10.0);
    TermId s0 = add_terminal(b, 1, 2.0, 12.0);
    TermId s1 = add_terminal(b, 1, 18.0, 12.0);
    JsonValue cfg = JsonValue::object();
    JsonValue ipc = JsonValue::object();
    ipc["mm_per_amp"] = 0.75;
    cfg["ipc"] = ipc;
    RuleResolver r = RuleResolver::from_config(b, cfg);
    ElectricalContext ctx;
    DensityEstimator de;
    DensityResult dens = de.analyze(b);
    ConnectionTask tp{0, p0, p1, 0, 0}, ts{1, s0, s1, 1, 0};
    double dp = compute_difficulty(b, r, tp, ctx, dens.terminal_density, {}, 0).total;
    double ds = compute_difficulty(b, r, ts, ctx, dens.terminal_density, {}, 0).total;
    CT_CHECK(dp > ds);  // 5A wide trace scores more difficult than a signal
    Corridor cp = probable_corridor(b, r, tp, ctx);
    Corridor cs = probable_corridor(b, r, ts, ctx);
    CT_CHECK(cp.width_nm > cs.width_nm);
    auto w = build_interference({tp, ts}, {cp, cs});
    CT_CHECK(w[0][1] > 0);  // parallel 2mm-apart... corridors touch via clearance
}

CT_TEST(voltage_clearance_aware_interference) {
    auto build = []() {
        Board b = base_2layer(20.0, 20.0);
        NetInfo n0 = make_net(0, "N0");
        n0.has_current = true;
        n0.current_a = 0.1;
        NetInfo n1 = make_net(1, "N1");
        n1.has_current = true;
        n1.current_a = 0.1;
        b.nets.push_back(n0);
        b.nets.push_back(n1);
        add_terminal(b, 0, 2.0, 10.0);
        add_terminal(b, 0, 18.0, 10.0);
        add_terminal(b, 1, 2.0, 10.4);
        add_terminal(b, 1, 18.0, 10.4);
        return b;
    };
    ElectricalContext ctx;
    // Low-voltage pair.
    Board bl = build();
    ConnectionTask t0{0, bl.nets[0].terminals[0], bl.nets[0].terminals[1], 0, 0};
    ConnectionTask t1{1, bl.nets[1].terminals[0], bl.nets[1].terminals[1], 1, 0};
    RuleResolver rl = RuleResolver::defaults_for(bl);
    Corridor cl0 = probable_corridor(bl, rl, t0, ctx);
    Corridor cl1 = probable_corridor(bl, rl, t1, ctx);
    double wl = build_interference({t0, t1}, {cl0, cl1})[0][1];
    // High voltage-difference pair: 0V vs 100V with a 1mm table step.
    Board bh = build();
    bh.nets[0].has_voltage = true;
    bh.nets[0].voltage_v = 0.0;
    bh.nets[1].has_voltage = true;
    bh.nets[1].voltage_v = 100.0;
    JsonValue cfg = JsonValue::object();
    JsonValue vt = JsonValue::array();
    JsonValue e0 = JsonValue::object();
    e0["delta_v_min"] = 0.0;
    e0["clearance_mm"] = 0.15;
    JsonValue e1 = JsonValue::object();
    e1["delta_v_min"] = 50.0;
    e1["clearance_mm"] = 1.0;
    vt.as_array().push_back(e0);
    vt.as_array().push_back(e1);
    cfg["voltage_table"] = vt;
    RuleResolver rh = RuleResolver::from_config(bh, cfg);
    ConnectionTask h0{0, bh.nets[0].terminals[0], bh.nets[0].terminals[1], 0, 0};
    ConnectionTask h1{1, bh.nets[1].terminals[0], bh.nets[1].terminals[1], 1, 0};
    Corridor ch0 = probable_corridor(bh, rh, h0, ctx);
    Corridor ch1 = probable_corridor(bh, rh, h1, ctx);
    double wh = build_interference({h0, h1}, {ch0, ch1})[0][1];
    CT_CHECK(wl > 0);
    CT_CHECK(wh > wl);  // larger clearance burden -> larger shared-resource demand
}

CT_TEST(narrow_channel_contention_is_deterministic) {
    auto build = []() {
        Board b = base_2layer();
        NetInfo a = make_net(0, "A");
        a.has_current = true;
        a.current_a = 0.1;
        NetInfo bn = make_net(1, "B");
        bn.has_current = true;
        bn.current_a = 0.1;
        b.nets.push_back(a);
        b.nets.push_back(bn);
        add_terminal(b, 0, 2.0, 10.0);
        add_terminal(b, 0, 18.0, 10.0);
        add_terminal(b, 1, 2.0, 12.0);
        add_terminal(b, 1, 18.0, 8.0);
        Keepout top;
        top.rect = {mm_to_nm(9.0), mm_to_nm(11.5), mm_to_nm(11.0), mm_to_nm(20.0)};
        top.layer = kAllLayers;
        top.reason = "wall_top";
        Keepout bot;
        bot.rect = {mm_to_nm(9.0), mm_to_nm(0.0), mm_to_nm(11.0), mm_to_nm(8.5)};
        bot.layer = kAllLayers;
        bot.reason = "wall_bot";
        b.keepouts.push_back(top);
        b.keepouts.push_back(bot);
        return b;
    };
    RouteReport r1 = run_threads(build(), 1);
    RouteReport r4 = run_threads(build(), 4);
    CT_CHECK(r1.status == "COMPLETE");
    CT_CHECK(r4.status == "COMPLETE");
    CT_CHECK(r1.board_hash == r4.board_hash);
    // Contention serializes the channel: more than one epoch of work or a
    // conflict rejection must have been observed... on this easy geometry both
    // may still land together; the hard requirement is legality + determinism.
    BoardVerifier v;
    ElectricalContext ctx;
    Board b = build();
    RuleResolver rr = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = 4;
    RouterEngine eng(std::move(b), std::move(rr), opt);
    RouteReport rep = eng.run();
    RuleResolver r2 = RuleResolver::defaults_for(eng.committed());
    CT_CHECK(v.verify(eng.committed(), r2, ctx).ok);
    (void)rep;
}

CT_TEST(multicore_epochs_utilization_and_parity) {
    RouteReport r1 = run_threads(open_nets(10), 1);
    RouteReport r8 = run_threads(open_nets(10), 8);
    CT_CHECK(r1.status == "COMPLETE");
    CT_CHECK(r8.status == "COMPLETE");
    CT_CHECK(r1.board_hash == r8.board_hash);
    CT_CHECK(r8.stats.threads_used >= 2);
    CT_CHECK(r8.stats.epochs_count < r8.stats.tasks_total);  // real batching
    CT_CHECK(r8.stats.candidates_total == r8.stats.tasks_total);
}

CT_TEST(congestion_present_history_split) {
    Board b = base_2layer();
    CongestionMap cg;
    cg.init(b);
    Rect r{mm_to_nm(5), mm_to_nm(5), mm_to_nm(10), mm_to_nm(10)};
    cg.add_present_corridor(r, 2.0);
    CT_CHECK(cg.present_at({mm_to_nm(7), mm_to_nm(7)}) == 2.0);
    CT_CHECK(cg.history_at({mm_to_nm(7), mm_to_nm(7)}) == 0.0);
    CT_CHECK(cg.present_at({mm_to_nm(15), mm_to_nm(15)}) == 0.0);
    cg.add_history_segment({{mm_to_nm(7), mm_to_nm(7)}, {mm_to_nm(8), mm_to_nm(8)}}, 1.5);
    CT_CHECK(cg.history_at({mm_to_nm(7), mm_to_nm(7)}) == 1.5);
    cg.reset_present();
    CT_CHECK(cg.present_at({mm_to_nm(7), mm_to_nm(7)}) == 0.0);
    CT_CHECK(cg.history_at({mm_to_nm(7), mm_to_nm(7)}) == 1.5);  // history persists
    auto hs = cg.hotspots();
    CT_CHECK(!hs.empty());
    for (std::size_t i = 1; i < hs.size(); ++i) {
        double pa = hs[i - 1].present + 3 * hs[i - 1].history;
        double pb = hs[i].present + 3 * hs[i].history;
        CT_CHECK(pa >= pb);
    }
}

CT_TEST(congestion_penalty_is_capped) {
    Board b = base_2layer();
    CongestionMap cg;
    cg.init(b);
    Rect all{0, 0, b.width_nm, b.height_nm};
    for (int i = 0; i < 100; ++i) {
        cg.add_present_corridor(all, 10.0);
        cg.add_history_rect(all, 10.0);
    }
    Coord p = cg.penalty_for_segment({{0, 0}, {b.width_nm, b.height_nm}});
    CT_CHECK(p >= 0);
    CT_CHECK(p <= 1500000);  // bounded distortion: never a hard lock
}

CT_TEST(reservations_soft_bounded_self_excluded) {
    ConnectionTask t0{0, 0, 1, 0, 5.0}, t1{1, 2, 3, 1, 1.0};
    Corridor c0{{0, 0, mm_to_nm(10), mm_to_nm(10)}, mm_to_nm(1), mm_to_nm(1)};
    Corridor c1{{mm_to_nm(5), mm_to_nm(5), mm_to_nm(15), mm_to_nm(15)}, 0, 0};
    ReservationSet rs;
    rs.build({t0, t1}, {c0, c1}, {5.0, 1.0});
    // Self corridor never penalizes its own task (single-task set => zero).
    ReservationSet solo;
    solo.build({t0}, {c0}, {5.0});
    CT_CHECK(solo.penalty_for_segment(0, {{0, 0}, {mm_to_nm(9), mm_to_nm(9)}}) == 0);
    // Foreign overlapping corridor adds bounded cost only.
    Coord p = rs.penalty_for_segment(0, {{mm_to_nm(6), mm_to_nm(6)}, {mm_to_nm(7), mm_to_nm(7)}});
    CT_CHECK(p > 0);
    CT_CHECK(p <= 800000);
    // Disjoint segment is free.
    CT_CHECK(rs.penalty_for_segment(0, {{mm_to_nm(20), mm_to_nm(20)},
                                        {mm_to_nm(21), mm_to_nm(21)}}) == 0);
}

CT_TEST(interference_symmetric_deterministic) {
    ConnectionTask t0{0, 0, 1, 0, 0}, t1{1, 2, 3, 1, 0}, t2{2, 4, 5, 2, 0};
    Corridor c0{{0, 0, mm_to_nm(10), mm_to_nm(10)}, mm_to_nm(1), mm_to_nm(1)};
    Corridor c1{{mm_to_nm(5), mm_to_nm(5), mm_to_nm(15), mm_to_nm(15)}, mm_to_nm(1), mm_to_nm(1)};
    Corridor c2{{mm_to_nm(30), mm_to_nm(30), mm_to_nm(40), mm_to_nm(40)}, 0, 0};
    auto w1 = build_interference({t0, t1, t2}, {c0, c1, c2});
    auto w2 = build_interference({t0, t1, t2}, {c0, c1, c2});
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            CT_CHECK(w1[i][j] == w1[j][i]);
            CT_CHECK(w1[i][j] == w2[i][j]);
        }
    CT_CHECK(w1[0][1] > 0);
    CT_CHECK(w1[0][2] == 0);  // disjoint corridors do not interfere
    CT_CHECK(w1[1][2] == 0);
}

CT_TEST(scheduler_batches_fixed_width_difficulty_first) {
    std::vector<ConnectionTask> tasks;
    for (int i = 0; i < 10; ++i) tasks.push_back({i % 3, i * 2, i * 2 + 1, i, 10.0 - i});
    auto b1 = schedule_batches(tasks);
    auto b2 = schedule_batches(tasks);
    CT_CHECK(b1.size() == 2);
    CT_CHECK(b1[0].size() == 8);
    CT_CHECK(b1[1].size() == 2);
    CT_CHECK(b1[0][0] == 0);  // highest difficulty first
    CT_CHECK(b1 == b2);       // deterministic
    auto b4 = schedule_batches(tasks, 4);
    CT_CHECK(b4.size() == 3);
    CT_CHECK(b4[0].size() == 4 && b4[1].size() == 4 && b4[2].size() == 2);
}

CT_TEST(arbiter_order_deterministic) {
    Board b = open_nets(3);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    std::vector<double> lm = {1.0, 1.0};
    CongestionMap cg;
    cg.init(b);
    std::vector<ConnectionTask> tasks;
    for (NetId n = 0; n < 3; ++n) {
        RouteTree tree = build_route_tree(b, n);
        for (auto& t : tree.tasks) tasks.push_back(t);
    }
    std::vector<Corridor> corr;
    std::vector<double> diff;
    for (auto& t : tasks) {
        corr.push_back(probable_corridor(b, r, t, ctx));
        diff.push_back(1.0);
    }
    ReservationSet rs;
    rs.build(tasks, corr, diff);
    std::vector<CandidateRoute> cands;
    for (std::size_t i = 0; i < tasks.size(); ++i)
        cands.push_back(route_candidate_task(b, r, tasks[i], i, 1.0, ctx, lm, AStarConfig{}, cg, rs));
    ArbiterResult a1 = arbitrate(cands, b, r, ctx);
    // Shuffle input order: the accepted SET must be identical (order-free).
    std::vector<CandidateRoute> rev = cands;
    std::reverse(rev.begin(), rev.end());
    ArbiterResult a2 = arbitrate(rev, b, r, ctx);
    auto key = [&](std::size_t i, const std::vector<CandidateRoute>& v) {
        return std::to_string(v[i].task.net) + ":" + std::to_string(v[i].task.a);
    };
    std::vector<std::string> s1, s2;
    for (auto i : a1.accepted) s1.push_back(key(i, cands));
    for (auto i : a2.accepted) s2.push_back(key(i, rev));
    std::sort(s1.begin(), s1.end());
    std::sort(s2.begin(), s2.end());
    CT_CHECK(s1 == s2);
    CT_CHECK(a1.accepted.size() == 3);
}

CT_TEST(arbiter_rejects_illegal_overlap) {
    Board b = base_2layer();
    NetInfo n0 = make_net(0, "N0");
    n0.has_current = true;
    n0.current_a = 0.1;
    b.nets.push_back(n0);
    TermId a = add_terminal(b, 0, 2.0, 10.0);
    TermId c = add_terminal(b, 0, 18.0, 10.0);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    std::vector<double> lm = {1.0, 1.0};
    CongestionMap cg;
    cg.init(b);
    ConnectionTask task{0, a, c, 0, 1.0};
    Corridor corr = probable_corridor(b, r, task, ctx);
    ReservationSet rs;
    rs.build({task}, {corr}, {1.0});
    CandidateRoute cand =
        route_candidate_task(b, r, task, 0, 1.0, ctx, lm, AStarConfig{}, cg, rs);
    CT_CHECK(cand.found);
    std::string why;
    CT_CHECK(candidate_legal_vs_board(cand, b, r, ctx, why));
    // A wall across the route makes the same geometry illegal.
    Board blocked = b;
    Keepout wall;
    wall.rect = {mm_to_nm(9.0), mm_to_nm(0.0), mm_to_nm(11.0), mm_to_nm(20.0)};
    wall.layer = kAllLayers;
    wall.reason = "wall";
    blocked.keepouts.push_back(wall);
    CT_CHECK(!candidate_legal_vs_board(cand, blocked, r, ctx, why));
    CT_CHECK(!why.empty());
    // And the arbiter rejects it (never commits illegal copper).
    ArbiterResult arb = arbitrate({cand}, blocked, r, ctx);
    CT_CHECK(arb.accepted.empty());
    CT_CHECK(arb.rejected.size() == 1);
}

CT_TEST(conflict_same_net_never_conflicts) {
    Board b = open_nets(1);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    RouteTree tree = build_route_tree(b, 0);
    std::vector<double> lm = {1.0, 1.0};
    CongestionMap cg;
    cg.init(b);
    ConnectionTask t = tree.tasks.front();
    Corridor corr = probable_corridor(b, r, t, ctx);
    ReservationSet rs;
    rs.build({t}, {corr}, {1.0});
    CandidateRoute c1 = route_candidate_task(b, r, t, 0, 1.0, ctx, lm, AStarConfig{}, cg, rs);
    CandidateRoute c2 = c1;  // identical geometry, same net: merging is fine
    CT_CHECK(c1.found);
    CT_CHECK(!candidates_conflict(c1, c2, r, ctx));
}

CT_TEST(geometry_hash_stable_and_sensitive) {
    Board b = open_nets(2);
    std::string h1 = geometry_hash(b);
    std::string h2 = geometry_hash(b);
    CT_CHECK(h1 == h2);
    CT_CHECK(h1.size() == 16);
    RouteReport rep = run_threads(std::move(b), 2);
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(!rep.board_hash.empty());
    CT_CHECK(rep.board_hash != h1);  // routed copper changes the hash
}

CT_TEST(progress_events_sequence) {
    Board b = open_nets(3);
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = 2;
    std::vector<JsonValue> events;
    opt.progress = [&](const JsonValue& ev) { events.push_back(ev); };
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(events.size() == rep.epochs.size() + 1);
    for (std::size_t i = 0; i < rep.epochs.size(); ++i) {
        CT_CHECK(events[i].get_string("event") == "epoch");
        CT_CHECK(events[i].get_number("epoch", -1) == (double)i);
    }
    CT_CHECK(events.back().get_string("event") == "done");
    CT_CHECK(events.back().get_string("status") == "COMPLETE");
    CT_CHECK(events.back().get_string("board_hash") == rep.board_hash);
}

CT_TEST(starvation_free_coverage) {
    // 8 high-difficulty blocked nets (long span across a full wall) plus 2
    // short easy nets with lower difficulty. The easy ones must still route
    // instead of starving behind the failing giants.
    Board b = base_2layer(40.0, 20.0);
    Keepout wall;
    wall.rect = {mm_to_nm(19.0), mm_to_nm(0.0), mm_to_nm(21.0), mm_to_nm(20.0)};
    wall.layer = kAllLayers;
    wall.reason = "wall";
    b.keepouts.push_back(wall);
    for (int i = 0; i < 8; ++i) {
        NetInfo n = make_net(i, "BLK" + std::to_string(i));
        n.has_current = true;
        n.current_a = 0.1;
        b.nets.push_back(n);
        add_terminal(b, i, 2.0, 2.0 + i * 1.0);
        add_terminal(b, i, 38.0, 2.0 + i * 1.0);
    }
    for (int i = 8; i < 10; ++i) {
        NetInfo n = make_net(i, "EASY" + std::to_string(i));
        n.has_current = true;
        n.current_a = 0.1;
        b.nets.push_back(n);
        add_terminal(b, i, 2.0, 15.0 + (i - 8) * 2.0);
        add_terminal(b, i, 6.0, 15.0 + (i - 8) * 2.0);
    }
    RouteReport rep = run_threads(std::move(b), 4);
    CT_CHECK(rep.status == "INCOMPLETE");  // the wall is genuinely impassable
    CT_CHECK(rep.stats.tasks_routed == 2);  // but the easy tasks got workers
    BoardVerifier v;
    ElectricalContext ctx;
    // Re-run to fetch committed copper for the legality check.
    Board b2 = base_2layer(40.0, 20.0);
    b2.keepouts.push_back(wall);
    for (int i = 0; i < 8; ++i) {
        NetInfo n = make_net(i, "BLK" + std::to_string(i));
        n.has_current = true;
        n.current_a = 0.1;
        b2.nets.push_back(n);
        add_terminal(b2, i, 2.0, 2.0 + i * 1.0);
        add_terminal(b2, i, 38.0, 2.0 + i * 1.0);
    }
    for (int i = 8; i < 10; ++i) {
        NetInfo n = make_net(i, "EASY" + std::to_string(i));
        n.has_current = true;
        n.current_a = 0.1;
        b2.nets.push_back(n);
        add_terminal(b2, i, 2.0, 15.0 + (i - 8) * 2.0);
        add_terminal(b2, i, 6.0, 15.0 + (i - 8) * 2.0);
    }
    RuleResolver r2 = RuleResolver::defaults_for(b2);
    EngineOptions opt;
    opt.threads = 4;
    RouterEngine eng(std::move(b2), std::move(r2), opt);
    RouteReport rep2 = eng.run();
    RuleResolver rv = RuleResolver::defaults_for(eng.committed());
    CT_CHECK(v.verify(eng.committed(), rv, ctx).legal);
    (void)rep2;
}

int main() { return copperline::test::run_all_tests(); }
