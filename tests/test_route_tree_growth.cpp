#include <algorithm>

#include "helpers.h"

#include "router/engine.h"
#include "router/route_tree.h"
#include "router/verifier.h"

using namespace copperline;
using namespace copperline::test;

namespace {

Board star_board() {
    Board b = base_2layer();
    NetInfo s = make_net(0, "STAR");
    b.nets.push_back(s);
    add_terminal(b, 0, 5.0, 10.0);
    add_terminal(b, 0, 15.0, 10.0);
    add_terminal(b, 0, 10.0, 5.0);
    add_terminal(b, 0, 10.0, 15.0);
    return b;
}

Coord total_trace_length(const Board& b, NetId net = -1) {
    Coord total = 0;
    for (const auto& t : b.traces) {
        if (net >= 0 && t.net != net) continue;
        total += manhattan(t.a, t.b);
    }
    return total;
}

RouteReport route_copy(const Board& b, int threads) {
    Board c = b;
    RuleResolver r = RuleResolver::defaults_for(c);
    EngineOptions opt;
    opt.threads = threads;
    RouterEngine engine(std::move(c), std::move(r), opt);
    return engine.run();
}

}  // namespace

CT_TEST(star_trunk_shared_not_redundant_mst) {
    Board b = star_board();
    // Initial tree: N terminals -> N-1 tasks.
    RouteTree tree = build_route_tree(b, 0);
    CT_CHECK(tree.tasks.size() == 3);

    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.verification.ok);
    CT_CHECK(rep.verification.connected);

    // Naive terminal-pair MST over this symmetric star is 30mm (3x10mm).
    // Shared trunk via centre (10,10) is 20mm. Growth from committed copper
    // must beat redundant point-to-point edges.
    Coord total = total_trace_length(engine.committed(), 0);
    CT_CHECK(total < mm_to_nm(30.0));
    CT_CHECK(total <= mm_to_nm(21.0));

    // All four terminals in one copper component.
    auto comps = net_terminal_components(engine.committed(), 0);
    CT_CHECK(comps.size() == 1);
    CT_CHECK(comps[0].size() == 4);
}

CT_TEST(star_trunk_deterministic_across_threads) {
    Board b1 = star_board();
    Board b2 = b1;
    // Same committed geometry independent of worker count.
    RouteReport r1 = route_copy(b1, 1);
    RouteReport r2 = route_copy(b2, 4);
    CT_CHECK(r1.status == "COMPLETE");
    CT_CHECK(r2.status == "COMPLETE");
    CT_CHECK(r1.board_hash == r2.board_hash);
}

CT_TEST(terminal_attaches_mid_copper) {
    Board b = base_2layer();
    NetInfo s = make_net(0, "TNET");
    b.nets.push_back(s);
    TermId t0 = add_terminal(b, 0, 5.0, 10.0);
    TermId t1 = add_terminal(b, 0, 15.0, 10.0);
    TermId t2 = add_terminal(b, 0, 10.0, 5.0);
    // Pre-committed trunk between t0-t1 (as if first tree edge already grew).
    TraceSeg trunk{0, 0, b.find_terminal(t0)->pos, b.find_terminal(t1)->pos,
                   mm_to_nm(0.2)};
    b.traces.push_back(trunk);

    // Components: {t0,t1} + {t2}.
    auto comps = net_terminal_components(b, 0);
    CT_CHECK(comps.size() == 2);
    const std::vector<TermId>* main = nullptr;
    for (auto& g : comps)
        if (std::find(g.begin(), g.end(), t0) != g.end()) main = &g;
    CT_CHECK(main != nullptr);
    CT_CHECK(main->size() == 2);

    // Nearest contact on the trunk to t2 is the midpoint (10,10).
    auto contacts = copper_contacts_for(b, 0, t2, *main, 8);
    CT_CHECK(!contacts.empty());
    Point expect{mm_to_nm(10.0), mm_to_nm(10.0)};
    CT_CHECK(contacts[0].p == expect);

    // Fresh tree grows from copper: one task with a copper target.
    RouteTree tree = build_route_tree(b, 0);
    CT_CHECK(tree.tasks.size() == 1);
    CT_CHECK(tree.tasks[0].has_copper_target);
    CT_CHECK(tree.tasks[0].copper_point == expect);

    // Routing completes by T-junctioning mid-copper.
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.enable_ripup = false;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    BoardVerifier v;
    ElectricalContext ctx;
    RuleResolver r2 = RuleResolver::defaults_for(engine.committed());
    CT_CHECK(v.verify(engine.committed(), r2, ctx).ok);
    // New copper must touch the trunk midpoint (vertical stub T-junction).
    Rect mid_pt{expect.x, expect.y, expect.x, expect.y};
    bool touches_mid = false;
    for (const auto& t : engine.committed().traces) {
        if (t.net != 0) continue;
        if (seg_intersects_rect(t.segment(), mid_pt)) touches_mid = true;
    }
    CT_CHECK(touches_mid);
}

CT_TEST(ripup_changes_components_and_tasks_regen) {
    Board b = base_2layer();
    NetInfo s = make_net(0, "RNET");
    b.nets.push_back(s);
    TermId t0 = add_terminal(b, 0, 5.0, 10.0);
    TermId t1 = add_terminal(b, 0, 15.0, 10.0);
    TermId t2 = add_terminal(b, 0, 10.0, 5.0);
    // Simulate fully routed: trunk + stub connects all three.
    Point mid{mm_to_nm(10.0), mm_to_nm(10.0)};
    b.traces.push_back({0, 0, b.find_terminal(t0)->pos, b.find_terminal(t1)->pos,
                        mm_to_nm(0.2)});
    b.traces.push_back({0, 0, b.find_terminal(t2)->pos, mid, mm_to_nm(0.2)});

    CT_CHECK(terminals_connected(b, 0, t0, t1));
    CT_CHECK(terminals_connected(b, 0, t0, t2));
    CT_CHECK(net_terminal_components(b, 0).size() == 1);
    CT_CHECK(build_route_tree(b, 0).tasks.empty());

    // Simulate rip-up: remove the stub (second trace).
    b.traces.pop_back();
    CT_CHECK(terminals_connected(b, 0, t0, t1));
    CT_CHECK(!terminals_connected(b, 0, t0, t2));
    auto comps = net_terminal_components(b, 0);
    CT_CHECK(comps.size() == 2);
    RouteTree regen = build_route_tree(b, 0);
    CT_CHECK(regen.tasks.size() == 1);
    CT_CHECK(regen.tasks[0].has_copper_target);
    // Regenerated task reflects the new topology: source is the isolated
    // terminal, target component is the trunk pair.
    CT_CHECK(regen.tasks[0].a == t2);
    CT_CHECK((regen.tasks[0].b == t0 || regen.tasks[0].b == t1));

    // Full rip: clear all copper -> three isolated terminals -> 2 tasks.
    b.traces.clear();
    CT_CHECK(net_terminal_components(b, 0).size() == 3);
    RouteTree cleared = build_route_tree(b, 0);
    CT_CHECK(cleared.tasks.size() == 2);

    // Determinism: repeated queries agree exactly.
    RouteTree again = build_route_tree(b, 0);
    CT_CHECK(again.tasks.size() == cleared.tasks.size());
    for (std::size_t i = 0; i < again.tasks.size(); ++i) {
        CT_CHECK(again.tasks[i].a == cleared.tasks[i].a);
        CT_CHECK(again.tasks[i].b == cleared.tasks[i].b);
        CT_CHECK(again.tasks[i].has_copper_target == cleared.tasks[i].has_copper_target);
        CT_CHECK(again.tasks[i].copper_point == cleared.tasks[i].copper_point);
    }
}

CT_TEST(two_terminal_net_keeps_simple_path) {
    Board b = base_2layer();
    NetInfo s = make_net(0, "PAIR");
    b.nets.push_back(s);
    TermId t0 = add_terminal(b, 0, 2.0, 10.0);
    TermId t1 = add_terminal(b, 0, 18.0, 10.0);
    RouteTree tree = build_route_tree(b, 0);
    CT_CHECK(tree.tasks.size() == 1);
    CT_CHECK(!tree.tasks[0].has_copper_target);
    CT_CHECK(tree.tasks[0].a == t0);
    CT_CHECK(tree.tasks[0].b == t1);
}

int main() { return copperline::test::run_all_tests(); }
