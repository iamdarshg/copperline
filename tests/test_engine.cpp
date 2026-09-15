#include <algorithm>
#include <map>
#include <set>

#include "helpers.h"

#include "router/density.h"
#include "router/engine.h"
#include "router/escape.h"
#include "router/parallel.h"
#include "router/recovery.h"
#include "router/route_tree.h"
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

Board open_board() {
    Board b = base_2layer();
    NetInfo s = make_net(0, "SIG1");
    s.has_current = true;
    s.current_a = 0.1;
    b.nets.push_back(s);
    NetInfo g = make_net(1, "GND");
    g.has_current = true;
    g.current_a = 0.2;
    b.nets.push_back(g);
    add_terminal(b, 0, 2.0, 10.0);
    add_terminal(b, 0, 18.0, 10.0);
    add_terminal(b, 1, 2.0, 5.0);
    add_terminal(b, 1, 18.0, 5.0);
    return b;
}

void expect_clean(const Board& routed, const RuleResolver& prototype) {
    BoardVerifier v;
    ElectricalContext ctx;
    RuleResolver r = RuleResolver::defaults_for(routed);
    (void)prototype;
    VerifyResult vr = v.verify(routed, r, ctx);
    CT_CHECK(vr.ok);
}

}  // namespace

CT_TEST(open_2layer_routes) {
    Board b = open_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.connected_terminals == rep.total_terminals);
    CT_CHECK(rep.total_terminals == 4);
    CT_CHECK(rep.stats.tasks_routed == 2);
    RuleResolver proto = RuleResolver::defaults_for(engine.committed());
    expect_clean(engine.committed(), proto);
}

CT_TEST(obstacle_detour_routes) {
    Board b = open_board();
    Keepout wall;
    wall.rect = {mm_to_nm(9.0), mm_to_nm(0.0), mm_to_nm(11.0), mm_to_nm(15.0)};
    wall.layer = kAllLayers;
    wall.reason = "wall";
    b.keepouts.push_back(wall);
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    // The y=10 net must detour around the wall top (gap y=15..20).
    Coord total = 0;
    for (const auto& t : engine.committed().traces) total += manhattan(t.a, t.b);
    CT_CHECK(total > mm_to_nm(32.0));  // both nets straight would be exactly 32mm
    RuleResolver proto = RuleResolver::defaults_for(engine.committed());
    expect_clean(engine.committed(), proto);
}

CT_TEST(high_current_consumes_width) {
    Board b = base_2layer();
    NetInfo pwr = make_net(0, "PWR");
    pwr.has_current = true;
    pwr.current_a = 5.0;
    b.nets.push_back(pwr);
    NetInfo sig = make_net(1, "SIG");
    sig.has_current = true;
    sig.current_a = 0.1;
    b.nets.push_back(sig);
    add_terminal(b, 0, 2.0, 12.0);
    add_terminal(b, 0, 18.0, 12.0);
    add_terminal(b, 1, 2.0, 4.0);
    add_terminal(b, 1, 18.0, 4.0);
    JsonValue cfg = JsonValue::object();
    JsonValue ipc = JsonValue::object();
    ipc["mm_per_amp"] = 0.75;
    cfg["ipc"] = ipc;
    RuleResolver r = RuleResolver::from_config(b, cfg);
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    Coord w_pwr = 0, w_sig = 0;
    for (const auto& t : engine.committed().traces) {
        if (t.net == 0) w_pwr = std::max(w_pwr, t.width_nm);
        if (t.net == 1) w_sig = std::max(w_sig, t.width_nm);
    }
    CT_CHECK(w_pwr == mm_to_nm(3.75));
    CT_CHECK(w_pwr > w_sig * 5);
    BoardVerifier v;
    ElectricalContext ctx;
    RuleResolver r2 = RuleResolver::from_config(engine.committed(), cfg);
    CT_CHECK(v.verify(engine.committed(), r2, ctx).ok);
}

CT_TEST(blocked_board_reports_incomplete) {
    Board b = open_board();
    Keepout wall;
    wall.rect = {mm_to_nm(9.0), mm_to_nm(0.0), mm_to_nm(11.0), mm_to_nm(20.0)};
    wall.layer = kAllLayers;
    wall.reason = "full wall";
    b.keepouts.push_back(wall);
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status != "COMPLETE");
    CT_CHECK(!rep.failures.empty());
    CT_CHECK(!rep.failures.front().blockers.empty());
    // The impossible board must NOT verify clean.
    BoardVerifier v;
    ElectricalContext ctx;
    RuleResolver r2 = RuleResolver::defaults_for(engine.committed());
    VerifyResult vr = v.verify(engine.committed(), r2, ctx);
    CT_CHECK(!vr.ok);
    CT_CHECK(!vr.unconnected.empty());
}

CT_TEST(single_terminal_net_trivial) {
    Board b = base_2layer();
    NetInfo s = make_net(0, "SOLO");
    b.nets.push_back(s);
    add_terminal(b, 0, 5.0, 5.0);
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.connected_terminals == 1);
}

CT_TEST(route_tree_mst_task_count_and_determinism) {
    Board b = base_2layer();
    NetInfo s = make_net(0, "MULTI");
    b.nets.push_back(s);
    add_terminal(b, 0, 1.0, 1.0);
    add_terminal(b, 0, 5.0, 1.0);
    add_terminal(b, 0, 9.0, 1.0);
    add_terminal(b, 0, 13.0, 1.0);
    RouteTree t1 = build_route_tree(b, 0);
    RouteTree t2 = build_route_tree(b, 0);
    CT_CHECK(t1.tasks.size() == 3);  // N terminals -> N-1 MST tasks
    CT_CHECK(t2.tasks.size() == 3);
    for (std::size_t i = 0; i < 3; ++i) {
        CT_CHECK(t1.tasks[i].a == t2.tasks[i].a);
        CT_CHECK(t1.tasks[i].b == t2.tasks[i].b);
    }
    // Unknown net yields no tasks (never crashes the scheduler).
    RouteTree empty = build_route_tree(b, 99);
    CT_CHECK(empty.tasks.empty());
}

CT_TEST(route_tree_sort_is_deterministic) {
    std::vector<ConnectionTask> v1 = {{1, 5, 6, 0, 3.0}, {0, 1, 2, 0, 3.0}, {0, 3, 4, 1, 9.0}};
    std::vector<ConnectionTask> v2 = v1;
    // Reverse input, sort both: identical order.
    std::reverse(v1.begin(), v1.end());
    sort_tasks_deterministic(v1);
    sort_tasks_deterministic(v2);
    for (std::size_t i = 0; i < v1.size(); ++i) {
        CT_CHECK(v1[i].net == v2[i].net);
        CT_CHECK(v1[i].a == v2[i].a);
        CT_CHECK(v1[i].b == v2[i].b);
    }
    CT_CHECK(v1[0].difficulty == 9.0);  // hardest first
    CT_CHECK(v1[1].net == 0);           // tie broken by net id
}

CT_TEST(engine_timeout_reports_timeout) {
    Board b = open_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.timeout_s = 1e-9;  // already expired: every task times out
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "TIMEOUT");
    CT_CHECK(!rep.failures.empty());
    CT_CHECK(rep.failures.front().reason == "timeout");
}

CT_TEST(engine_report_carries_stats_and_hash) {
    Board b = open_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    JsonValue j = rep.to_json();
    CT_CHECK(j.get_string("schema") == "copperline/route-report/1");
    CT_CHECK(!j.get_string("board_hash").empty());
    CT_CHECK(j.has("epoch_log"));
    CT_CHECK(j.has("congestion_hotspots"));
    CT_CHECK(j.has("remaining_terminals"));
    CT_CHECK(j.find("stats")->get_number("candidates_total", 0) == 2);
    CT_CHECK(j.find("stats")->get_number("epochs", 0) >= 1);
}

CT_TEST(verifier_gate_keeps_clean_complete) {
    Board b = open_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.verification.ok);
    CT_CHECK(rep.verification.connected);
    CT_CHECK(rep.verification.legal);
    JsonValue j = rep.to_json();
    CT_CHECK(j.has("verification"));
    CT_CHECK(j.get_bool("verifier_ok", false));
}

CT_TEST(verifier_gate_refuses_bookkeeping_lie_disconnected) {
    // Inject a COMPLETE report whose bookkeeping claims success while the
    // independent verifier reports a disconnected pad: success must be refused.
    RouteReport rep;
    rep.status = "COMPLETE";
    rep.result_category = "COMPLETE";
    rep.total_terminals = 2;
    rep.connected_terminals = 2;
    VerifyResult vr;
    vr.ok = false;
    vr.connected = false;
    vr.legal = true;
    Unconnected u;
    u.net = 0;
    u.net_name = "SIG";
    u.terminal = 1;
    u.component = "U1";
    u.pin = "2";
    vr.unconnected.push_back(u);
    apply_verifier_gate(rep, vr);
    CT_CHECK(rep.status == "INCOMPLETE");
    CT_CHECK(!rep.failures.empty());
    CT_CHECK(rep.failures.front().reason == "verifier_unconnected");
    CT_CHECK(rep.connected_terminals == 1);
    CT_CHECK(!rep.verification.ok);
}

CT_TEST(verifier_gate_escalates_hard_violation) {
    // A bookkeeping-COMPLETE report with illegal copper must become VIOLATION
    // with a hard-rule-violation category, never stay COMPLETE.
    RouteReport rep;
    rep.status = "COMPLETE";
    rep.result_category = "COMPLETE";
    rep.total_terminals = 2;
    rep.connected_terminals = 2;
    VerifyResult vr;
    vr.ok = false;
    vr.connected = true;
    vr.legal = false;
    Violation v;
    v.type = "clearance";
    v.net_a = 0;
    v.net_b = 1;
    v.rule = "board_default";
    v.detail = "injected clearance hit";
    vr.violations.push_back(v);
    apply_verifier_gate(rep, vr);
    CT_CHECK(rep.status == "VIOLATION");
    CT_CHECK(rep.result_category == "HARD_RULE_VIOLATION");
    CT_CHECK(!rep.failures.empty());
}

CT_TEST(engine_with_illegal_fixed_copper_never_complete) {
    // Fixed user copper already violates clearance; even if routing tasks
    // succeed the final report must not claim COMPLETE.
    Board b = base_2layer();
    NetInfo a = make_net(0, "A");
    b.nets.push_back(a);
    NetInfo n1 = make_net(1, "B");
    b.nets.push_back(n1);
    add_terminal(b, 0, 1.0, 5.0);
    add_terminal(b, 0, 5.0, 5.0);
    add_terminal(b, 1, 1.0, 5.1);
    add_terminal(b, 1, 5.0, 5.1);
    // Pre-existing illegal pair: 0.1mm apart with 0.15mm default clearance.
    b.traces.push_back({0, 0, b.terminals[0].pos, b.terminals[1].pos, mm_to_nm(0.2)});
    b.traces.push_back({1, 0, b.terminals[2].pos, b.terminals[3].pos, mm_to_nm(0.2)});
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.enable_ripup = false;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status != "COMPLETE");
    CT_CHECK(!rep.verification.ok);
    CT_CHECK(!rep.verification.legal);
    CT_CHECK(!rep.verification.violations.empty());
    JsonValue j = rep.to_json();
    CT_CHECK(j.has("verification"));
    CT_CHECK(j.get_bool("verifier_ok", true) == false);
}

// ---- Post-P4 issue #1: EscapePlanner integrated into `router route` ----

CT_TEST(route_logs_escape_before_global_epochs) {
    // `router route` on BGA fixtures must run the real centre-out escape
    // stage before global epochs (progress: escape, epochs..., done).
    for (const char* fx : {"bga_4x4.json", "bga_8x8.json"}) {
        Board b = load_fixture_board(fx);
        RuleResolver r = RuleResolver::defaults_for(b);
        EngineOptions opt;
        opt.threads = 1;
        std::vector<JsonValue> events;
        opt.progress = [&](const JsonValue& ev) { events.push_back(ev); };
        RouterEngine engine(std::move(b), std::move(r), opt);
        RouteReport rep = engine.run();
        CT_CHECK(rep.escape_stage.pads_total > 0);
        CT_CHECK(!events.empty());
        CT_CHECK(events.front().get_string("event") == "escape");
        CT_CHECK(events.back().get_string("event") == "done");
        // Every middle event is a global epoch.
        for (std::size_t i = 1; i + 1 < events.size(); ++i)
            CT_CHECK(events[i].get_string("event") == "epoch");
        // Epoch log counts only global epochs; escape is separate.
        CT_CHECK(events.size() == rep.epochs.size() + 2);
        JsonValue j = rep.to_json();
        CT_CHECK(j.has("escape"));
        CT_CHECK(j.find("escape")->get_number("pads_total", 0) > 0);
    }
    // Open board: no fine-pitch => no escape event (existing epoch/done
    // sequence unchanged).
    {
        Board b = open_board();
        RuleResolver r = RuleResolver::defaults_for(b);
        EngineOptions opt;
        std::vector<JsonValue> events;
        opt.progress = [&](const JsonValue& ev) { events.push_back(ev); };
        RouterEngine engine(std::move(b), std::move(r), opt);
        RouteReport rep = engine.run();
        CT_CHECK(rep.escape_stage.pads_total == 0);
        CT_CHECK(events.size() == rep.epochs.size() + 1);
        CT_CHECK(events.back().get_string("event") == "done");
    }
}

CT_TEST(route_global_tasks_originate_at_escape_portals) {
    // After the escape stage commits pad->portal stubs, the global
    // RouteTree for an escaped 2-terminal net must target committed escape
    // copper (has_copper_target) instead of generating a second raw-pad
    // task, and the candidate must terminate at that portal point.
    Board b = load_fixture_board("bga_4x4.json");
    RuleResolver r0 = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    EscapePlanner planner;
    EscapeResult esc = planner.plan(b, r0, ctx);
    CT_CHECK(esc.pads_with_candidates > 0);
    // Commit best stubs exactly like RouterEngine::run (eligibility order).
    Board work = b;
    std::map<TermId, Point> portal_of;
    for (const auto& fp : esc.footprints) {
        std::map<TermId, const PadEscapeResult*> by_term;
        for (const auto& p : fp.pads) by_term[p.terminal] = &p;
        for (TermId tid : fp.commit_order) {
            auto it = by_term.find(tid);
            if (it == by_term.end() || !it->second->has_viable) continue;
            const EscapeCandidate& best = it->second->candidates.front();
            for (const auto& s : best.traces) work.traces.push_back(s);
            for (const auto& v : best.vias) work.vias.push_back(v);
            portal_of[tid] = best.portal_pos;
        }
    }
    CT_CHECK(!portal_of.empty());
    // Every escaped net's fresh tree must be portal-anchored.
    int anchored = 0;
    for (const auto& net : work.nets) {
        if (net.terminals.size() != 2) continue;
        TermId pad = -1;
        for (TermId t : net.terminals) {
            const Terminal* term = work.find_terminal(t);
            if (term && term->component == "U1") pad = t;
        }
        if (pad < 0 || !portal_of.count(pad)) continue;
        RouteTree tree = build_route_tree(work, net.id);
        CT_CHECK(tree.tasks.size() == 1);
        const ConnectionTask& task = tree.tasks[0];
        CT_CHECK(task.has_copper_target);
        CT_CHECK(task.copper_point == portal_of[pad]);
        // The global candidate terminates at the portal, not the raw pad.
        RuleResolver r = RuleResolver::defaults_for(work);
        DensityEstimator de;
        DensityResult dens = de.analyze(work);
        std::map<TermId, int> depth;
        DifficultyVector dv = compute_difficulty(work, r, task, ctx,
                                                 dens.terminal_density, depth, 0);
        std::vector<double> lm = {1.0, 1.0};
        CongestionMap cg;
        cg.init(work);
        ReservationSet rs;
        Corridor corr = probable_corridor(work, r, task, ctx);
        rs.build({task}, {corr}, {dv.total});
        CandidateRoute cand =
            route_candidate_task(work, r, task, 0, dv.total, ctx, lm,
                                 AStarConfig{}, cg, rs);
        if (cand.found) {
            CT_CHECK(cand.gate_b == task.copper_point);
            CT_CHECK(cand.gate_b == portal_of[pad]);
        }
        ++anchored;
    }
    CT_CHECK(anchored > 0);
}

CT_TEST(route_outer_ring_cannot_leapfrog_deeper_unresolved) {
    // 8x8 has infeasible interior escapes. The engine must carry those
    // records into the report and must preserve the centre-out eligibility
    // invariant (a shallower pad is never committed while a deeper eligible
    // pad has neither a candidate nor a record).
    Board b = load_fixture_board("bga_8x8.json");
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    EscapePlanner planner;
    EscapeResult esc = planner.plan(b, r, ctx);
    CT_CHECK(esc.pads_infeasible > 0);
    // Planner invariant holds (same real stage the engine runs).
    for (const auto& fp : esc.footprints) {
        std::map<TermId, const PadEscapeResult*> by_term;
        for (const auto& p : fp.pads) by_term[p.terminal] = &p;
        std::set<TermId> committed(fp.commit_order.begin(), fp.commit_order.end());
        for (std::size_t i = 0; i < fp.eligibility_order.size(); ++i) {
            TermId shallow = fp.eligibility_order[i];
            if (!committed.count(shallow)) continue;
            for (std::size_t k = 0; k < i; ++k) {
                TermId deeper = fp.eligibility_order[k];
                const PadEscapeResult* dp = by_term[deeper];
                CT_CHECK(dp->has_viable || dp->infeasibility.recorded);
            }
        }
    }
    // Engine carries the same unresolved set; failures name the escape
    // record so shallower success cannot erase deeper infeasibility.
    EngineOptions opt;
    opt.threads = 1;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(!rep.escape_stage.unresolved_terminals.empty());
    CT_CHECK(rep.escape_stage.unresolved_terminals.size() ==
             rep.escape_stage.unresolved_reasons.size());
    std::set<TermId> planner_unres;
    for (const auto& fp : esc.footprints)
        for (const auto& p : fp.pads)
            if (!p.has_viable) planner_unres.insert(p.terminal);
    std::set<TermId> engine_unres(rep.escape_stage.unresolved_terminals.begin(),
                                  rep.escape_stage.unresolved_terminals.end());
    CT_CHECK(engine_unres == planner_unres);
    bool saw_escape_blocker = false;
    for (const auto& f : rep.failures)
        for (const auto& bl : f.blockers)
            if (bl.rfind("escape_infeasible:", 0) == 0) saw_escape_blocker = true;
    CT_CHECK(saw_escape_blocker);
}

CT_TEST(route_ripup_protects_real_escape_stubs) {
    // Protection must apply to actual escape-owned geometry (is_escape_stub
    // from committed stubs), not depth inference: a real stub outranks a
    // normal route at equal difficulty, and the generator rips the normal
    // route first.
    double normal = route_protection_score(false, 5.0, 0, false, RecoveryMode::FAST);
    double stub = route_protection_score(true, 5.0, 0, false, RecoveryMode::FAST);
    CT_CHECK(stub > normal);
    // Move generation with one real stub + one normal route blocking the
    // same failed task must prefer the normal route.
    Board b = base_2layer();
    NetInfo n0 = make_net(0, "STUB_NET");
    NetInfo n1 = make_net(1, "NORMAL_NET");
    NetInfo nf = make_net(9, "FAILED");
    b.nets.push_back(n0);
    b.nets.push_back(n1);
    b.nets.push_back(nf);
    TermId s0 = add_terminal(b, 0, 2.0, 10.0);
    TermId s1 = add_terminal(b, 0, 18.0, 10.0);
    TermId m0 = add_terminal(b, 1, 2.0, 12.0);
    TermId m1 = add_terminal(b, 1, 18.0, 12.0);
    TermId f0 = add_terminal(b, 9, 2.0, 11.0);
    TermId f1 = add_terminal(b, 9, 18.0, 11.0);
    ConnectionTask t_stub{0, s0, s1, 0, 5.0};
    ConnectionTask t_norm{1, m0, m1, 1, 5.0};
    ConnectionTask t_fail{9, f0, f1, 2, 5.0};
    OwnedRoute o_stub, o_norm;
    o_stub.task = t_stub;
    o_stub.task_pos = 0;
    o_stub.is_escape_stub = true;  // real committed stub
    o_stub.protection = route_protection_score(true, 5.0, 0, false, RecoveryMode::FAST);
    o_norm.task = t_norm;
    o_norm.task_pos = 1;
    o_norm.is_escape_stub = false;
    o_norm.protection = route_protection_score(false, 5.0, 0, false, RecoveryMode::FAST);
    DependencyGraph g;
    g.failed.push_back(t_fail);
    for (NetId bn : {0, 1}) {
        DependencyEdge e;
        e.failed_pos = 0;
        e.failed_net = 9;
        e.blocker_net = bn;
        e.blocker_desc = "trace:net=" + std::to_string(bn);
        e.weight = (bn == 0 ? 100.0 : 99.0);  // stub blocks slightly more...
        g.edges.push_back(e);
    }
    HistoryHeuristic hh;
    auto moves = generate_ripup_moves(g.failed, g, {o_stub, o_norm}, hh, "",
                                      RecoveryMode::FAST, 4, 2);
    CT_CHECK(!moves.empty());
    // ...yet the cheaper normal route must rank first despite lower gain.
    CT_CHECK(moves.front().owned_idx.size() == 1);
    CT_CHECK(moves.front().owned_idx[0] == 1);
    // Engine end-to-end: 4x4 commits real stubs (escaped set non-empty).
    {
        Board fb = load_fixture_board("bga_4x4.json");
        RuleResolver fr = RuleResolver::defaults_for(fb);
        EngineOptions fopt;
        fopt.threads = 1;
        RouterEngine feng(std::move(fb), std::move(fr), fopt);
        RouteReport frep = feng.run();
        CT_CHECK(frep.escape_stage.pads_escaped == 16);
        CT_CHECK(frep.escape_stage.pads_infeasible == 0);
        CT_CHECK(frep.status == "COMPLETE");
    }
}

int main() { return copperline::test::run_all_tests(); }
