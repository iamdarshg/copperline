#include "helpers.h"

#include "router/engine.h"
#include "router/verifier.h"
#include "router/via_bundle.h"

using namespace copperline;
using namespace copperline::test;

namespace {

JsonValue small_only_config() {
    JsonValue cfg = JsonValue::object();
    JsonValue styles = JsonValue::array();
    JsonValue s = JsonValue::object();
    s["name"] = "SMALL";
    s["outer_mm"] = 0.4;
    s["hole_mm"] = 0.2;
    s["max_current_a"] = 1.0;
    styles.as_array().push_back(s);
    cfg["via_classes"] = styles;
    return cfg;
}

Board high_current_layer_swap_board() {
    Board b = base_2layer();
    NetInfo pwr = make_net(0, "PWR");
    pwr.has_current = true;
    pwr.current_a = 5.0;  // 5x the 1A SMALL via: needs 5 in parallel
    b.nets.push_back(pwr);
    add_terminal(b, 0, 2.0, 10.0, 0);
    add_terminal(b, 0, 18.0, 10.0, 1);
    return b;
}

}  // namespace

CT_TEST(bundle_planner_picks_parallel_count) {
    Board b = high_current_layer_swap_board();
    JsonValue cfg = small_only_config();
    RuleResolver r = RuleResolver::from_config(b, cfg);
    ElectricalContext ctx;
    ViaStyle style;
    CT_CHECK(r.lookup_via_style("SMALL", style));
    int need = r.current().vias_required(style, *b.find_net(0), b.defaults, ctx);
    CT_CHECK(need == 5);
    LayerSpan span{0, 1};
    // Open board: the full 5-via bundle fits around the transition point.
    ViaBundle bundle =
        ViaBundlePlanner::plan(b, r, 0, {mm_to_nm(10.0), mm_to_nm(10.0)}, span,
                               mm_to_nm(0.5), ctx);
    CT_CHECK(bundle.feasible);
    CT_CHECK(bundle.reason == "ok");
    CT_CHECK(bundle.count == 5);
    CT_CHECK(bundle.positions.size() == 5);
    CT_CHECK(bundle.style.name == "SMALL");
    // Same-net barrels stay disjoint.
    for (std::size_t i = 0; i < bundle.positions.size(); ++i)
        for (std::size_t j = i + 1; j < bundle.positions.size(); ++j) {
            Rect a = Rect::from_center_size(bundle.positions[i], style.outer_nm,
                                            style.outer_nm);
            Rect c = Rect::from_center_size(bundle.positions[j], style.outer_nm,
                                            style.outer_nm);
            CT_CHECK(rect_gap(a, c) > 0);
        }
}

CT_TEST(high_current_routes_with_expected_bundle_count) {
    Board b = high_current_layer_swap_board();
    JsonValue cfg = small_only_config();
    RuleResolver r = RuleResolver::from_config(b, cfg);
    EngineOptions opt;
    opt.threads = 1;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.failures.empty());
    const Board& routed = engine.committed();
    CT_CHECK(routed.vias.size() == 5);  // one atomic 5-via bundle
    for (const auto& v : routed.vias) CT_CHECK(v.via_class == "SMALL");
    // Star stubs keep every barrel on the same copper network.
    CT_CHECK(!routed.traces.empty());
    // Independent verification with the same rule set passes: the bundle
    // shares the current instead of violating the per-via limit.
    RuleResolver rv = RuleResolver::from_config(routed, cfg);
    ElectricalContext ctx;
    BoardVerifier verifier;
    VerifyResult vr = verifier.verify(routed, rv, ctx);
    CT_CHECK(vr.legal);
    CT_CHECK(vr.ok);
    // Deterministic across worker counts.
    Board b2 = high_current_layer_swap_board();
    RuleResolver r2 = RuleResolver::from_config(b2, cfg);
    EngineOptions opt2;
    opt2.threads = 4;
    RouterEngine engine2(std::move(b2), std::move(r2), opt2);
    RouteReport rep2 = engine2.run();
    CT_CHECK(rep2.status == "COMPLETE");
    CT_CHECK(rep2.board_hash == rep.board_hash);
}

CT_TEST(low_current_transition_still_uses_single_via) {
    Board b = base_2layer();
    NetInfo sig = make_net(0, "SIG");
    sig.has_current = true;
    sig.current_a = 0.1;
    b.nets.push_back(sig);
    add_terminal(b, 0, 2.0, 10.0, 0);
    add_terminal(b, 0, 18.0, 10.0, 1);
    JsonValue cfg = small_only_config();  // 1A per via is plenty here
    RuleResolver r = RuleResolver::from_config(b, cfg);
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(engine.committed().vias.size() == 1);
    CT_CHECK(engine.committed().vias[0].via_class == "SMALL");
}

CT_TEST(constrained_bundle_reports_explicit_infeasibility) {
    Board b = base_2layer();
    NetInfo pwr = make_net(0, "PWR");
    pwr.has_current = true;
    pwr.current_a = 5.0;
    b.nets.push_back(pwr);
    add_terminal(b, 0, 2.0, 10.0, 0);
    add_terminal(b, 0, 18.0, 10.0, 1);
    // Windows of 0.6mm around each terminal; everything else is keepout.
    // A single 0.4mm barrel fits in a window, but no 5-via bundle
    // (2.2mm row) fits anywhere, and no trace can cross the middle block.
    auto block = [&](double x1, double y1, double x2, double y2) {
        Keepout ko;
        ko.rect = {mm_to_nm(x1), mm_to_nm(y1), mm_to_nm(x2), mm_to_nm(y2)};
        ko.layer = kAllLayers;
        ko.reason = "choke";
        b.keepouts.push_back(ko);
    };
    block(0.0, 0.0, 20.0, 9.6);
    block(0.0, 10.4, 20.0, 20.0);
    block(0.0, 9.6, 1.6, 10.4);
    block(2.4, 9.6, 17.5, 10.4);
    block(18.5, 9.6, 20.0, 10.4);
    JsonValue cfg = small_only_config();
    RuleResolver r = RuleResolver::from_config(b, cfg);
    ElectricalContext ctx;
    // Planner level: the bundle around the source terminal is blocked...
    ViaBundle probe = ViaBundlePlanner::plan(
        b, r, 0, {mm_to_nm(2.0), mm_to_nm(10.0)}, LayerSpan{0, 1}, mm_to_nm(0.5),
        ctx);
    CT_CHECK(!probe.feasible);
    CT_CHECK(probe.reason == "bundle_blocked");
    CT_CHECK(probe.count == 5);
    // ...while a lone barrel at the same point would be legal, proving the
    // failure is the bundle footprint and not the transition itself.
    ViaStyle style;
    CT_CHECK(r.lookup_via_style("SMALL", style));
    Rect single = Rect::from_center_size({mm_to_nm(2.0), mm_to_nm(10.0)},
                                         style.outer_nm, style.outer_nm);
    CT_CHECK(b.bounds().contains(single));
    // Engine level: explicit infeasibility, never silent success.
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status != "COMPLETE");
    CT_CHECK(!rep.failures.empty());
    bool explicit_bundle = false;
    for (const auto& f : rep.failures) {
        if (f.reason == "via_bundle_infeasible") {
            explicit_bundle = true;
            CT_CHECK(f.vias_required == 5);
            CT_CHECK(f.via_style == "SMALL");
            CT_CHECK(f.required_current_a > 4.9);
        }
    }
    CT_CHECK(explicit_bundle);
    // Machine-readable diagnostics carry the bundle accounting.
    JsonValue j = rep.to_json();
    const JsonValue* fails = j.find("failures");
    CT_CHECK(fails && fails->is_array() && !fails->as_array().empty());
    bool json_bundle = false;
    for (const auto& f : fails->as_array()) {
        if (f.get_string("reason") == "via_bundle_infeasible") {
            json_bundle = true;
            CT_CHECK(f.get_string("via_style") == "SMALL");
            CT_CHECK(f.get_number("vias_required", 0) == 5.0);
            CT_CHECK(f.get_number("required_current_a", 0) > 4.9);
        }
    }
    CT_CHECK(json_bundle);
}

CT_TEST(voltage_clearance_applies_to_full_bundle) {
    Board b = base_2layer();
    NetInfo hv = make_net(0, "HV");
    hv.has_current = true;
    hv.current_a = 5.0;
    hv.has_voltage = true;
    hv.voltage_v = 100.0;
    b.nets.push_back(hv);
    NetInfo lv = make_net(1, "LV");
    lv.has_voltage = true;
    lv.voltage_v = 3.3;
    b.nets.push_back(lv);
    // Neighbour pad 1mm east of the transition centre.
    add_terminal(b, 1, 11.0, 10.0, 0);
    JsonValue cfg = small_only_config();
    // Board default clearance is 0.15mm: the column layout threads past it.
    {
        RuleResolver r = RuleResolver::from_config(b, cfg);
        ElectricalContext ctx;
        ViaBundle bundle = ViaBundlePlanner::plan(
            b, r, 0, {mm_to_nm(10.0), mm_to_nm(10.0)}, LayerSpan{0, 1},
            mm_to_nm(0.2), ctx);
        CT_CHECK(bundle.feasible);
    }
    // A 1mm rule for >= 50V deltas blocks every layout: row satellites and
    // stubs close in on the pad, and the column sits 0.55mm away.
    {
        JsonValue vt = JsonValue::array();
        JsonValue e0 = JsonValue::object();
        e0["delta_v_min"] = 0.0;
        e0["clearance_mm"] = 0.15;
        vt.as_array().push_back(e0);
        JsonValue e1 = JsonValue::object();
        e1["delta_v_min"] = 50.0;
        e1["clearance_mm"] = 1.0;
        vt.as_array().push_back(e1);
        cfg["voltage_table"] = vt;
        RuleResolver r = RuleResolver::from_config(b, cfg);
        ElectricalContext ctx;
        std::string cs;
        CT_CHECK(r.requiredClearance(0, 1, 0, ctx, &cs) == mm_to_nm(1.0));
        ViaBundle bundle = ViaBundlePlanner::plan(
            b, r, 0, {mm_to_nm(10.0), mm_to_nm(10.0)}, LayerSpan{0, 1},
            mm_to_nm(0.2), ctx);
        CT_CHECK(!bundle.feasible);
        CT_CHECK(bundle.reason == "bundle_blocked");
    }
}

CT_TEST(width_class_ceiling_enforced) {
    Board b = base_2layer();
    NetInfo hot = make_net(0, "HOT");
    hot.has_current = true;
    hot.current_a = 5.0;
    hot.width_class = "THIN";
    b.nets.push_back(hot);
    NetInfo sig = make_net(1, "SIG");
    sig.has_current = true;
    sig.current_a = 0.1;
    sig.width_class = "THIN";
    b.nets.push_back(sig);
    JsonValue cfg = JsonValue::object();
    JsonValue classes = JsonValue::array();
    JsonValue thin = JsonValue::object();
    thin["name"] = "THIN";
    thin["min_width_mm"] = 0.2;
    thin["max_current_a"] = 0.5;  // ceiling below HOT's 5A
    classes.as_array().push_back(thin);
    cfg["width_classes"] = classes;
    JsonValue ipc = JsonValue::object();
    ipc["mm_per_amp"] = 0.75;
    cfg["ipc"] = ipc;
    RuleResolver r = RuleResolver::from_config(b, cfg);
    ElectricalContext ctx;
    std::string source;
    // 5A exceeds the class ceiling: the class must not be selected.
    Coord w_hot = r.requiredTraceWidth(0, 0, ctx, &source);
    CT_CHECK(source != "width_class");
    CT_CHECK(w_hot == mm_to_nm(5.0 * 0.75));
    // 0.1A sits under the ceiling: the class still applies.
    Coord w_sig = r.requiredTraceWidth(1, 0, ctx, &source);
    CT_CHECK(source == "width_class");
    CT_CHECK(w_sig == mm_to_nm(0.2));
}

CT_TEST(bundle_prefers_named_via_class) {
    Board b = high_current_layer_swap_board();
    NetInfo* pwr = b.find_net(0);
    pwr->via_class = "SMALL";
    JsonValue cfg = JsonValue::object();
    JsonValue styles = JsonValue::array();
    JsonValue small = JsonValue::object();
    small["name"] = "SMALL";
    small["outer_mm"] = 0.4;
    small["hole_mm"] = 0.2;
    small["max_current_a"] = 1.0;
    styles.as_array().push_back(small);
    JsonValue big = JsonValue::object();
    big["name"] = "BIG";
    big["outer_mm"] = 1.0;
    big["hole_mm"] = 0.5;
    big["max_current_a"] = 8.0;
    styles.as_array().push_back(big);
    cfg["via_classes"] = styles;
    RuleResolver r = RuleResolver::from_config(b, cfg);
    ElectricalContext ctx;
    auto ordered =
        ViaBundlePlanner::ordered_styles(r, 0, LayerSpan{0, 1});
    CT_CHECK(!ordered.empty());
    CT_CHECK(ordered.front().name == "SMALL");  // preference outranks size
    ViaBundle bundle =
        ViaBundlePlanner::plan(b, r, 0, {mm_to_nm(10.0), mm_to_nm(10.0)},
                               LayerSpan{0, 1}, mm_to_nm(0.5), ctx);
    CT_CHECK(bundle.feasible);
    CT_CHECK(bundle.style.name == "SMALL");
    CT_CHECK(bundle.count == 5);
}

int main() { return copperline::test::run_all_tests(); }
