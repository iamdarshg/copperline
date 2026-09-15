// Issue #11: practical impedance-aware routing.
//
// Covers: documented microstrip/stripline estimator trends, per-layer
// solve/select within tolerance, router layer/width choice, explicit
// high-current + impedance conflicts (never silent), and unchanged
// behavior for nets without impedance metadata.
#include <cmath>

#include "helpers.h"

#include "router/analyze.h"
#include "router/engine.h"
#include "router/verifier.h"

using namespace copperline;
using namespace copperline::test;

#ifndef FIXTURE_DIR
#define FIXTURE_DIR "fixtures"
#endif

namespace {

// 2 signal layers with a 0.2 mm FR-4 (er 4.4) stackup to the opposite plane.
Board stackup_2layer() {
    Board b = base_2layer();
    for (auto& l : b.layers) {
        l.has_dielectric_thickness = true;
        l.dielectric_thickness_nm = mm_to_nm(0.2);
        l.has_dielectric_er = true;
        l.dielectric_er = 4.4;
    }
    b.layers[0].has_ref_plane = true;
    b.layers[0].ref_plane_layer = 1;
    b.layers[1].has_ref_plane = true;
    b.layers[1].ref_plane_layer = 0;
    return b;
}

// 4-layer stackup: outer microstrip (h=0.2), inner stripline (h=0.15).
Board stackup_4layer() {
    Board b = base_2layer();
    b.layers.clear();
    Layer top;
    top.id = 0;
    top.name = "Top";
    top.has_dielectric_thickness = true;
    top.dielectric_thickness_nm = mm_to_nm(0.2);
    top.has_dielectric_er = true;
    top.dielectric_er = 4.4;
    top.has_ref_plane = true;
    top.ref_plane_layer = 1;
    Layer gnd;
    gnd.id = 1;
    gnd.name = "GND";
    gnd.layer_type = "plane";
    Layer sig;
    sig.id = 2;
    sig.name = "SIG2";
    sig.has_dielectric_thickness = true;
    sig.dielectric_thickness_nm = mm_to_nm(0.15);
    sig.has_dielectric_er = true;
    sig.dielectric_er = 4.4;
    Layer bot;
    bot.id = 3;
    bot.name = "Bottom";
    bot.has_dielectric_thickness = true;
    bot.dielectric_thickness_nm = mm_to_nm(0.2);
    bot.has_dielectric_er = true;
    bot.dielectric_er = 4.4;
    bot.has_ref_plane = true;
    bot.ref_plane_layer = 2;
    b.layers.push_back(top);
    b.layers.push_back(gnd);
    b.layers.push_back(sig);
    b.layers.push_back(bot);
    return b;
}

NetInfo impedance_net(NetId id, const std::string& name, double target_ohms,
                      double current_a = 0.1) {
    NetInfo n = make_net(id, name);
    n.has_current = true;
    n.current_a = current_a;
    n.has_impedance = true;
    n.target_impedance_ohms = target_ohms;
    return n;
}

Board routed_impedance_board() {
    Board b = stackup_2layer();
    b.nets.push_back(impedance_net(0, "USB_DP", 50.0, 0.1));
    NetInfo sig = make_net(1, "SIG");
    b.nets.push_back(sig);
    add_terminal(b, 0, 2.0, 12.0);
    add_terminal(b, 0, 18.0, 12.0);
    add_terminal(b, 1, 2.0, 5.0);
    add_terminal(b, 1, 18.0, 5.0);
    return b;
}

bool verify_ok(const Board& board) {
    BoardVerifier v;
    ElectricalContext ctx;
    RuleResolver r = RuleResolver::defaults_for(board);
    return v.verify(board, r, ctx).ok;
}

}  // namespace

CT_TEST(microstrip_trends) {
    MicrostripModel m;
    // Wider trace -> lower impedance.
    CT_CHECK(m.estimate_ohms(0.4, 0.2, 4.4, 0.035) < m.estimate_ohms(0.2, 0.2, 4.4, 0.035));
    // Taller dielectric -> higher impedance.
    CT_CHECK(m.estimate_ohms(0.3, 0.3, 4.4, 0.035) > m.estimate_ohms(0.3, 0.1, 4.4, 0.035));
    // Higher permittivity -> lower impedance.
    CT_CHECK(m.estimate_ohms(0.3, 0.2, 6.0, 0.035) < m.estimate_ohms(0.3, 0.2, 3.0, 0.035));
    // Thicker foil -> slightly lower impedance.
    CT_CHECK(m.estimate_ohms(0.3, 0.2, 4.4, 0.070) < m.estimate_ohms(0.3, 0.2, 4.4, 0.018));
    // Degenerate inputs are out of domain, never silent garbage.
    CT_CHECK(m.estimate_ohms(0.0, 0.2, 4.4, 0.035) < 0);
    CT_CHECK(m.estimate_ohms(0.3, 0.0, 4.4, 0.035) < 0);
    CT_CHECK(std::string(m.name()) == "microstrip");
}

CT_TEST(stripline_trends) {
    StriplineModel s;
    CT_CHECK(s.estimate_ohms(0.3, 0.2, 4.4, 0.035) < s.estimate_ohms(0.15, 0.2, 4.4, 0.035));
    CT_CHECK(s.estimate_ohms(0.2, 0.3, 4.4, 0.035) > s.estimate_ohms(0.2, 0.1, 4.4, 0.035));
    CT_CHECK(s.estimate_ohms(0.2, 0.2, 6.0, 0.035) < s.estimate_ohms(0.2, 0.2, 3.0, 0.035));
    // Stripline (two reference planes) sits below microstrip for the same
    // width/height/er.
    MicrostripModel m;
    CT_CHECK(s.estimate_ohms(0.3, 0.2, 4.4, 0.035) < m.estimate_ohms(0.3, 0.2, 4.4, 0.035));
    CT_CHECK(std::string(s.name()) == "stripline");
}

CT_TEST(solver_hits_target_within_tolerance) {
    Board b = stackup_2layer();
    b.nets.push_back(impedance_net(0, "USB_DP", 50.0, 0.1));
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    ImpedanceResolution res = r.impedanceResolution(0, ctx);
    CT_CHECK(res.has_target);
    CT_CHECK(res.feasible);
    CT_CHECK(!res.conflict);
    CT_CHECK(res.selected_layer == 0 || res.selected_layer == 1);
    CT_CHECK(std::string(res.selected_model) == "microstrip");
    CT_CHECK(res.rel_error <= 0.10 + 1e-9);
    CT_CHECK(res.selected_width_nm > 0);
    // ~0.33 mm for 50 ohms on 0.2 mm FR-4 microstrip.
    CT_CHECK(res.selected_width_nm > mm_to_nm(0.2));
    CT_CHECK(res.selected_width_nm < mm_to_nm(0.6));
    CT_CHECK(res.options.size() == 2);
}

CT_TEST(inner_layers_solve_as_stripline) {
    Board b = stackup_4layer();
    b.nets.push_back(impedance_net(0, "DDR_DQ", 50.0, 0.1));
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    ImpedanceResolution res = r.impedanceResolution(0, ctx);
    CT_CHECK(res.has_target);
    CT_CHECK(res.feasible);
    // Plane layer 1 is ineligible; 0/3 microstrip, 2 stripline. The 0.15 mm
    // ampacity clamp floor sits above the narrow stripline tolerance band,
    // so layer 2 reports a per-layer current conflict while the outer
    // layers solve cleanly -- and selection avoids the conflicted layer.
    CT_CHECK(res.options.size() == 3);
    bool saw_stripline = false, saw_microstrip = false;
    for (const auto& op : res.options) {
        if (op.layer == 2) {
            CT_CHECK(op.model == "stripline");
            CT_CHECK(op.current_conflict);
            CT_CHECK(!op.in_tolerance);
            saw_stripline = true;
        } else {
            CT_CHECK(op.model == "microstrip");
            CT_CHECK(op.in_tolerance);
            CT_CHECK(!op.current_conflict);
            saw_microstrip = true;
        }
    }
    CT_CHECK(saw_stripline && saw_microstrip);
    CT_CHECK(res.feasible && !res.conflict);
    CT_CHECK(res.selected_layer == 0 || res.selected_layer == 3);
    // Stripline needs a narrower trace than microstrip for the same target
    // (compare raw solver widths, free of the current floor).
    Coord w_strip = -1, w_micro = -1;
    for (const auto& op : res.options) {
        if (op.layer == 2) w_strip = op.raw_width_nm;
        if (op.layer == 0) w_micro = op.raw_width_nm;
    }
    CT_CHECK(w_strip > 0 && w_micro > 0 && w_strip < w_micro);
}

CT_TEST(preferred_layers_restrict_eligibility) {
    Board b = stackup_2layer();
    NetInfo n = impedance_net(0, "USB_DP", 50.0, 0.1);
    n.impedance_layers = {1};
    b.nets.push_back(n);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    ImpedanceResolution res = r.impedanceResolution(0, ctx);
    CT_CHECK(res.feasible);
    CT_CHECK(res.options.size() == 1);
    CT_CHECK(res.selected_layer == 1);
}

CT_TEST(no_stackup_means_no_target_change) {
    // Nets with impedance metadata but no stackup fall back to current
    // widths; nets without metadata are byte-for-byte the old behavior.
    Board b = base_2layer();
    b.nets.push_back(impedance_net(0, "USB_DP", 50.0, 0.1));
    NetInfo bare = make_net(1, "BARE");
    b.nets.push_back(bare);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    ImpedanceResolution res = r.impedanceResolution(0, ctx);
    CT_CHECK(res.has_target);
    CT_CHECK(!res.feasible);
    CT_CHECK(!res.conflict);
    std::string source;
    Coord w = r.requiredTraceWidth(0, 0, ctx, &source);
    CT_CHECK(source == "ampacity");  // 0.1 A still sizes by current
    CT_CHECK(w == mm_to_nm(0.15));   // ampacity floor clamp
    ImpedanceResolution bare_res = r.impedanceResolution(1, ctx);
    CT_CHECK(!bare_res.has_target);
    Coord wb = r.requiredTraceWidth(1, 0, ctx, &source);
    CT_CHECK(source == "board_default");
    CT_CHECK(wb == b.defaults.trace_width_nm);
    CT_CHECK(r.impedanceLayerMultiplier(1, 0) == 1.0);
}

CT_TEST(router_selects_layer_and_width_in_tolerance) {
    Board b = routed_impedance_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(verify_ok(engine.committed()));
    // Every routed trace of the controlled net estimates inside tolerance.
    RuleResolver r2 = RuleResolver::defaults_for(engine.committed());
    ElectricalContext ctx;
    bool saw_controlled = false;
    for (const auto& t : engine.committed().traces) {
        if (t.net != 0) continue;
        saw_controlled = true;
        std::string model;
        double est = r2.impedanceEstimate(0, t.layer, t.width_nm, ctx, &model);
        CT_CHECK(est > 0);
        double err = std::fabs(est - 50.0) / 50.0;
        CT_CHECK(err <= 0.10 + 1e-9);
    }
    CT_CHECK(saw_controlled);
    // Report carries selected layer/width/model/target/estimate/error.
    CT_CHECK(rep.impedance.size() == 1);
    const ImpedanceResolution& zir = rep.impedance.front();
    CT_CHECK(zir.target_ohms == 50.0);
    CT_CHECK(zir.feasible && !zir.conflict);
    CT_CHECK(zir.selected_width_nm > 0);
    CT_CHECK(zir.rel_error <= 0.10 + 1e-9);
    const JsonValue& j = rep.to_json();
    const JsonValue* iz = j.find("impedance");
    CT_CHECK(iz && iz->is_array() && !iz->as_array().empty());
    const JsonValue& e = iz->as_array().front();
    CT_CHECK(e.get_number("target_ohms", 0) == 50.0);
    CT_CHECK(e.get_number("selected_width_mm", 0) > 0.2);
    CT_CHECK(!e.get_string("model").empty());
    CT_CHECK(e.get_number("estimated_ohms", 0) > 40.0);
}

CT_TEST(router_result_is_deterministic) {
    Board b1 = routed_impedance_board();
    RuleResolver r1 = RuleResolver::defaults_for(b1);
    RouterEngine e1(std::move(b1), std::move(r1), EngineOptions{});
    RouteReport r1rep = e1.run();
    Board b2 = routed_impedance_board();
    RuleResolver r2 = RuleResolver::defaults_for(b2);
    RouterEngine e2(std::move(b2), std::move(r2), EngineOptions{});
    RouteReport r2rep = e2.run();
    CT_CHECK(r1rep.status == "COMPLETE");
    CT_CHECK(r1rep.board_hash == r2rep.board_hash);
}

CT_TEST(high_current_impedance_conflict_is_explicit) {
    // 5 A needs ~1.8 mm by ampacity; 50 ohms needs ~0.33 mm. The router
    // must report the conflict instead of silently narrowing the trace.
    Board b = stackup_2layer();
    b.nets.push_back(impedance_net(0, "PWR50", 50.0, 5.0));
    add_terminal(b, 0, 2.0, 10.0);
    add_terminal(b, 0, 18.0, 10.0);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    ImpedanceResolution res = r.impedanceResolution(0, ctx);
    CT_CHECK(res.has_target);
    CT_CHECK(!res.feasible);
    CT_CHECK(res.conflict);
    CT_CHECK(!res.conflict_detail.empty());
    // Final width never drops below the ampacity floor.
    std::string source;
    Coord w = r.requiredTraceWidth(0, 0, ctx, &source);
    CT_CHECK(source == "impedance_conflict");
    CT_CHECK(w > mm_to_nm(1.0));
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status != "COMPLETE");
    bool saw_conflict = false;
    for (const auto& f : rep.failures) {
        if (f.net == 0 &&
            (f.reason == "impedance_current_conflict" || f.impedance_conflict)) {
            saw_conflict = true;
            CT_CHECK(f.has_impedance);
            CT_CHECK(f.target_impedance_ohms == 50.0);
        }
    }
    CT_CHECK(saw_conflict);
    // Fail-fast: no undersized copper committed for the conflicted net.
    for (const auto& t : engine.committed().traces) CT_CHECK(t.net != 0);
}

CT_TEST(verifier_flags_wrong_width_impedance_trace) {
    Board b = routed_impedance_board();
    // Hand-place an ampacity-floor trace far from the 50-ohm width.
    TraceSeg bad;
    bad.net = 0;
    bad.layer = 0;
    bad.a = {mm_to_nm(2.0), mm_to_nm(14.0)};
    bad.b = {mm_to_nm(18.0), mm_to_nm(14.0)};
    bad.width_nm = mm_to_nm(0.15);
    b.traces.push_back(bad);
    BoardVerifier v;
    ElectricalContext ctx;
    RuleResolver r = RuleResolver::defaults_for(b);
    VerifyResult vr = v.verify(b, r, ctx);
    bool saw_z = false;
    for (const auto& x : vr.violations)
        if (x.type == "impedance") saw_z = true;
    CT_CHECK(saw_z);
    CT_CHECK(!vr.ok);
}

CT_TEST(config_model_override_and_bad_config) {
    Board b = stackup_2layer();
    b.nets.push_back(impedance_net(0, "USB_DP", 50.0, 0.1));
    JsonValue cfg = JsonValue::object();
    JsonValue iz = JsonValue::object();
    iz["model"] = "stripline";
    cfg["impedance"] = iz;
    RuleResolver r = RuleResolver::from_config(b, cfg);
    ElectricalContext ctx;
    ImpedanceResolution res = r.impedanceResolution(0, ctx);
    CT_CHECK(res.feasible);
    for (const auto& op : res.options) CT_CHECK(op.model == "stripline");
    JsonValue bad = JsonValue::object();
    JsonValue biz = JsonValue::object();
    biz["model"] = "field_solver";
    bad["impedance"] = biz;
    bool threw = false;
    try {
        RuleResolver rb = RuleResolver::from_config(b, bad);
        (void)rb;
    } catch (const BoardError& e) {
        threw = e.kind == InputKind::kRule;
    }
    CT_CHECK(threw);
    // Disabled system leaves widths on the current path.
    JsonValue dis = JsonValue::object();
    JsonValue diz = JsonValue::object();
    diz["enabled"] = false;
    dis["impedance"] = diz;
    RuleResolver rd = RuleResolver::from_config(b, dis);
    ImpedanceResolution dr = rd.impedanceResolution(0, ctx);
    CT_CHECK(!dr.has_target);
}

CT_TEST(fixture_parses_and_routes) {
    ImportResult r = import_board_auto(std::string(FIXTURE_DIR) + "/impedance_demo.json");
    CT_CHECK(r.board.layers.size() == 2);
    const NetInfo* zn = r.board.find_net_by_name("USB_DP");
    CT_CHECK(zn && zn->has_impedance && zn->target_impedance_ohms == 50.0);
    CT_CHECK(r.board.layers[0].has_dielectric_thickness);
    RuleResolver res = RuleResolver::defaults_for(r.board);
    EngineOptions opt;
    RouterEngine engine(std::move(r.board), std::move(res), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(verify_ok(engine.committed()));
    // Native JSON round-trips the stackup + impedance intent.
    JsonValue back = board_to_json(engine.committed());
    const JsonValue* layers = back.find("layers");
    CT_CHECK(layers && layers->is_array() && layers->as_array().size() == 2);
    CT_CHECK(layers->as_array()[0].get_number("dielectric_er", 0) == 4.4);
}

CT_TEST(analyze_reports_impedance) {
    Board b = routed_impedance_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    AnalysisResult a = analyze_board(b, r, r.defaultContext(), {});
    const JsonValue* nets = a.data.find("nets");
    CT_CHECK(nets && nets->is_array() && nets->as_array().size() == 2);
    const JsonValue& z = nets->as_array()[0];
    const JsonValue* iz = z.find("impedance");
    CT_CHECK(iz && iz->is_object());
    CT_CHECK(iz->get_number("target_ohms", 0) == 50.0);
    CT_CHECK(iz->get_bool("feasible", false));
    CT_CHECK(iz->get_number("selected_width_mm", 0) > 0.2);
    const JsonValue& plain = nets->as_array()[1];
    const JsonValue* piz = plain.find("impedance");
    CT_CHECK(piz && piz->is_object() && !piz->get_bool("has_target", true));
}

int main() { return copperline::test::run_all_tests(); }
