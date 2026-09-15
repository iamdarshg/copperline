#include "helpers.h"

#include "router/engine.h"
#include "router/parallel.h"
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

Coord net_length(const Board& b, NetId net) {
    Coord total = 0;
    for (const auto& t : b.traces)
        if (t.net == net) total += manhattan(t.a, t.b);
    return total;
}

int net_vias(const Board& b, NetId net) {
    int n = 0;
    for (const auto& v : b.vias)
        if (v.net == net) ++n;
    return n;
}

}  // namespace

CT_TEST(plane_import_and_roundtrip) {
    Board b = load_fixture_board("plane_power.json");
    CT_CHECK(b.planes.size() == 2);
    CT_CHECK(b.planes[0].id == 0);
    CT_CHECK(b.planes[0].net == 0);
    CT_CHECK(b.planes[0].layer == 1);
    CT_CHECK(b.planes[0].island == 0);
    CT_CHECK(b.planes[0].routable);
    CT_CHECK(b.planes[0].poly.size() == 4);
    CT_CHECK(b.planes[1].net == 2);
    // Round-trip through board_to_json preserves every plane field.
    JsonValue j = board_to_json(b);
    JsonBoardImporter imp;
    ImportResult r = imp.import_value(j, "roundtrip");
    CT_CHECK(r.board.planes.size() == 2);
    CT_CHECK(r.board.planes[0].id == 0);
    CT_CHECK(r.board.planes[0].net == 0);
    CT_CHECK(r.board.planes[0].layer == 1);
    CT_CHECK(r.board.planes[0].island == 0);
    CT_CHECK(r.board.planes[0].routable);
    CT_CHECK(r.board.planes[0].poly == b.planes[0].poly);
    CT_CHECK(r.board.planes[1].poly == b.planes[1].poly);
}

CT_TEST(plane_island_contact_needs_correct_net_and_layer) {
    Board b = load_fixture_board("plane_split.json");
    int pid = -1, island = -1;
    // VCC pad over its own pour on the plane layer: proven contact.
    CT_CHECK(plane_island_at(b, 0, {mm_to_nm(15), mm_to_nm(10)}, 1, pid, island));
    CT_CHECK(pid == 0);
    CT_CHECK(island == 0);
    // Same geometry, wrong net: geometric overlap proves nothing.
    CT_CHECK(!plane_island_at(b, 1, {mm_to_nm(15), mm_to_nm(10)}, 1, pid, island));
    // VCC terminal sitting over the GND pour: not VCC contact.
    CT_CHECK(!plane_island_at(b, 0, {mm_to_nm(5), mm_to_nm(10)}, 1, pid, island));
    // Right net and place, wrong layer: the pour lives on layer 1.
    CT_CHECK(!plane_island_at(b, 0, {mm_to_nm(15), mm_to_nm(10)}, 0, pid, island));
    // Open field: no contact at all.
    CT_CHECK(!plane_island_at(b, 0, {mm_to_nm(10), mm_to_nm(10)}, 1, pid, island));
}

CT_TEST(nearest_plane_target_prefers_short_then_same_layer) {
    Board b = base_2layer();
    NetInfo pwr = make_net(0, "PWR");
    b.nets.push_back(pwr);
    add_terminal(b, 0, 5.0, 10.0, 0);
    // Near plane on another layer vs far plane on the terminal layer.
    PlaneZone near;
    near.id = 0;
    near.net = 0;
    near.layer = 1;
    near.island = 0;
    near.routable = true;
    near.poly = {{mm_to_nm(5.4), mm_to_nm(9)}, {mm_to_nm(6.4), mm_to_nm(9)},
                 {mm_to_nm(6.4), mm_to_nm(11)}, {mm_to_nm(5.4), mm_to_nm(11)}};
    PlaneZone far;
    far.id = 1;
    far.net = 0;
    far.layer = 0;
    far.island = 1;
    far.routable = true;
    far.poly = {{mm_to_nm(14), mm_to_nm(9)}, {mm_to_nm(16), mm_to_nm(9)},
                {mm_to_nm(16), mm_to_nm(11)}, {mm_to_nm(14), mm_to_nm(11)}};
    b.planes.push_back(near);
    b.planes.push_back(far);
    int pid = -1, island = -1;
    Point entry{};
    LayerId layer = -1;
    CT_CHECK(nearest_plane_target(b, 0, {mm_to_nm(5.0), mm_to_nm(10.0)}, 0, pid, entry,
                                  layer, island));
    CT_CHECK(pid == 0);  // shorter access wins over same-layer
    CT_CHECK(layer == 1);
    CT_CHECK(entry == Point({mm_to_nm(5.4), mm_to_nm(10.0)}));
    // Tie on distance: same-layer direct copper wins, then smallest id.
    b.planes.clear();
    PlaneZone a;
    a.id = 5;
    a.net = 0;
    a.layer = 1;
    a.island = 0;
    a.routable = true;
    a.poly = {{mm_to_nm(7), mm_to_nm(10)}, {mm_to_nm(9), mm_to_nm(10)},
              {mm_to_nm(9), mm_to_nm(12)}, {mm_to_nm(7), mm_to_nm(12)}};
    PlaneZone c;
    c.id = 3;
    c.net = 0;
    c.layer = 0;
    c.island = 1;
    c.routable = true;
    c.poly = {{mm_to_nm(7), mm_to_nm(8)}, {mm_to_nm(9), mm_to_nm(8)},
              {mm_to_nm(9), mm_to_nm(10)}, {mm_to_nm(7), mm_to_nm(10)}};
    b.planes.push_back(a);
    b.planes.push_back(c);
    CT_CHECK(nearest_plane_target(b, 0, {mm_to_nm(5.0), mm_to_nm(10.0)}, 0, pid, entry,
                                  layer, island));
    CT_CHECK(pid == 3);  // equal distance (2mm), same-layer entry wins
    CT_CHECK(layer == 0);
    // Non-routable reference copper is never eligible.
    b.planes.clear();
    PlaneZone ref = near;
    ref.routable = false;
    b.planes.push_back(ref);
    CT_CHECK(!net_has_routable_planes(b, 0));
    CT_CHECK(!nearest_plane_target(b, 0, {mm_to_nm(5.0), mm_to_nm(10.0)}, 0, pid, entry,
                                   layer, island));
}

CT_TEST(power_routes_into_nearby_plane_not_across_board) {
    Board b = load_fixture_board("plane_power.json");
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = 1;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.verification.ok);
    const Board& routed = engine.committed();
    // PWR terminals sit directly over their pour: two via-down stubs
    // instead of a 16mm cross-board trace.
    CT_CHECK(net_length(routed, 0) < mm_to_nm(8.0));
    // Ordinary non-plane net still runs pad to pad (16mm straight).
    CT_CHECK(net_length(routed, 1) > mm_to_nm(15.5));
    // 3A through a 2A STD via needs parallel bundles at both entries.
    CT_CHECK(net_vias(routed, 0) >= 4);
    // High-current entry: 6A needs 3-via bundles at both HPC entries.
    CT_CHECK(net_vias(routed, 2) >= 6);
    // JSON plane access: chosen plane/layer/island, entry, vias, margin.
    CT_CHECK(rep.plane_access.size() == 4);
    for (const auto& p : rep.plane_access) {
        CT_CHECK(p.plane_layer == 1);
        CT_CHECK(p.island == 0);
        CT_CHECK(p.via_count >= 2);
        CT_CHECK(p.current_margin_a >= 0.0);
        CT_CHECK(!p.via_style.empty());
    }
    JsonValue j = rep.to_json();
    CT_CHECK(j.has("plane_access"));
    CT_CHECK(j.find("plane_access")->as_array().size() == 4);
    // Determinism: identical copper at 1 and 4 workers.
    Board b2 = load_fixture_board("plane_power.json");
    RuleResolver r2 = RuleResolver::defaults_for(b2);
    EngineOptions opt2;
    opt2.threads = 4;
    RouterEngine eng2(std::move(b2), std::move(r2), opt2);
    RouteReport rep2 = eng2.run();
    CT_CHECK(rep2.status == "COMPLETE");
    CT_CHECK(rep2.board_hash == rep.board_hash);
}

CT_TEST(split_plane_cannot_connect_wrong_island) {
    Board b = load_fixture_board("plane_split.json");
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.verification.ok);
    const Board& routed = engine.committed();
    // VCC terminal 1 sat over the GND pour; it must reach its OWN island
    // at (11,10) instead of touching the pour beneath it.
    bool saw_vcc_entry = false;
    for (const auto& p : rep.plane_access) {
        if (p.net != 0) continue;
        CT_CHECK(p.plane_id == 0);
        CT_CHECK(p.island == 0);
        if (p.terminal == 1) {
            saw_vcc_entry = true;
            CT_CHECK(p.entry == Point({mm_to_nm(11), mm_to_nm(10)}));
        }
    }
    CT_CHECK(saw_vcc_entry);
    // No VCC copper on the plane layer ever enters the GND polygon.
    const PlaneZone* gnd = find_plane(routed, 1);
    CT_CHECK(gnd != nullptr);
    for (const auto& t : routed.traces) {
        if (t.net != 0 || t.layer != 1) continue;
        CT_CHECK(!plane_seg_hits_poly(t.segment(), gnd->poly));
    }
    for (const auto& v : routed.vias) {
        if (v.net != 0) continue;
        if (v.top_layer > 1 || v.bottom_layer < 1) continue;
        CT_CHECK(!plane_rect_hits_poly(
            Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm), gnd->poly));
    }
}

CT_TEST(isolated_island_does_not_satisfy_connectivity) {
    // Same net, two islands, no stitching: a pad on each island with no
    // routed copper must NOT verify as connected.
    Board b = base_2layer();
    NetInfo pwr = make_net(0, "PWR");
    b.nets.push_back(pwr);
    TermId ta = add_terminal(b, 0, 2.0, 10.0, 0);
    TermId tc = add_terminal(b, 0, 18.0, 10.0, 0);
    (void)ta;
    (void)tc;
    PlaneZone left;
    left.id = 0;
    left.net = 0;
    left.layer = 0;
    left.island = 0;
    left.routable = true;
    left.poly = {{mm_to_nm(1), mm_to_nm(9)}, {mm_to_nm(3), mm_to_nm(9)},
                 {mm_to_nm(3), mm_to_nm(11)}, {mm_to_nm(1), mm_to_nm(11)}};
    PlaneZone right = left;
    right.id = 1;
    right.island = 1;
    right.poly = {{mm_to_nm(17), mm_to_nm(9)}, {mm_to_nm(19), mm_to_nm(9)},
                  {mm_to_nm(19), mm_to_nm(11)}, {mm_to_nm(17), mm_to_nm(11)}};
    b.planes.push_back(left);
    b.planes.push_back(right);
    // Island identity is exact: each pad proves its own island only.
    int pid = -1, island = -1;
    CT_CHECK(plane_island_at(b, 0, {mm_to_nm(2), mm_to_nm(10)}, 0, pid, island));
    CT_CHECK(pid == 0 && island == 0);
    CT_CHECK(plane_island_at(b, 0, {mm_to_nm(18), mm_to_nm(10)}, 0, pid, island));
    CT_CHECK(pid == 1 && island == 1);
    BoardVerifier v;
    ElectricalContext ctx;
    RuleResolver res = RuleResolver::defaults_for(b);
    VerifyResult vr = v.verify(b, res, ctx);
    CT_CHECK(!vr.ok);
    CT_CHECK(!vr.connected);
    CT_CHECK(vr.unconnected.size() == 1);
    // Declared stitching (shared island id) DOES connect across a gap.
    b.planes[1].island = 0;
    VerifyResult vr2 = v.verify(b, res, ctx);
    CT_CHECK(vr2.connected);
    CT_CHECK(vr2.ok);
    // And the route tree agrees: pads already on copper (even an isolated
    // island) need no access stub -- the verifier above still refuses to
    // call them connected, so nothing is falsely satisfied. Terminals off
    // every plane get one plane-access task each.
    b.planes[1].island = 1;
    CT_CHECK(build_route_tree(b, 0).tasks.empty());
    Board c = base_2layer();
    NetInfo q = make_net(0, "PWR");
    c.nets.push_back(q);
    add_terminal(c, 0, 2.0, 10.0, 0);
    add_terminal(c, 0, 18.0, 10.0, 0);
    PlaneZone pour;
    pour.id = 0;
    pour.net = 0;
    pour.layer = 1;
    pour.island = 0;
    pour.routable = true;
    pour.poly = {{mm_to_nm(1), mm_to_nm(8)}, {mm_to_nm(19), mm_to_nm(8)},
                 {mm_to_nm(19), mm_to_nm(12)}, {mm_to_nm(1), mm_to_nm(12)}};
    c.planes.push_back(pour);
    RouteTree pt = build_route_tree(c, 0);
    CT_CHECK(pt.tasks.size() == 2);
    for (const auto& t : pt.tasks) {
        CT_CHECK(t.has_plane_target);
        CT_CHECK(t.plane_id == 0);
        CT_CHECK(t.plane_layer == 1);
        CT_CHECK(t.a == t.b);
    }
}

CT_TEST(routed_plane_board_survives_save_verify_roundtrip) {
    // Plane entries land via-bundles; the saved board must keep via classes
    // so standalone `router verify` still sees full parallel clusters.
    Board b = load_fixture_board("plane_power.json");
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    JsonValue saved = board_to_json(engine.committed());
    CT_CHECK(saved.has("planes"));
    CT_CHECK(saved.find("planes")->as_array().size() == 2);
    JsonBoardImporter imp;
    ImportResult re = imp.import_value(saved, "saved");
    BoardVerifier v;
    ElectricalContext ctx;
    RuleResolver r2 = RuleResolver::defaults_for(re.board);
    VerifyResult vr = v.verify(re.board, r2, ctx);
    CT_CHECK(vr.ok);
    CT_CHECK(vr.connected);
    CT_CHECK(vr.legal);
}

CT_TEST(non_plane_nets_unchanged) {
    // A board without planes keeps the exact legacy behavior: direct
    // pad-to-pad tasks, no plane access records.
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
    CT_CHECK(!net_has_routable_planes(b, 0));
    CT_CHECK(build_route_tree(b, 0).tasks.size() == 1);
    CT_CHECK(!build_route_tree(b, 0).tasks.front().has_plane_target);
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.stats.tasks_total == 2);
    CT_CHECK(rep.plane_access.empty());
    CT_CHECK(rep.verification.ok);
}

int main() { return copperline::test::run_all_tests(); }
