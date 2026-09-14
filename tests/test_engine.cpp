#include "helpers.h"

#include "router/engine.h"
#include "router/verifier.h"

using namespace copperline;
using namespace copperline::test;

namespace {

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

int main() { return copperline::test::run_all_tests(); }
