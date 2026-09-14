// Issue #7: IPC-2221 ampacity model replaces the linear mm_per_amp heuristic
// as the default inferred current-width estimator.
#include <cmath>

#include "helpers.h"

#include "router/analyze.h"
#include "router/engine.h"
#include "router/verifier.h"

using namespace copperline;
using namespace copperline::test;

namespace {

Board amp_board() {
    Board b = base_2layer();
    NetInfo pwr = make_net(0, "PWR");
    pwr.has_current = true;
    pwr.current_a = 5.0;
    b.nets.push_back(pwr);
    return b;
}

void set_current(Board& b, NetId net, double amps) {
    NetInfo* n = b.find_net(net);
    n->has_current = true;
    n->current_a = amps;
    n->has_peak = false;
}

JsonValue ampacity_config(double dt = -1, double oz = -1) {
    JsonValue cfg = JsonValue::object();
    JsonValue amp = JsonValue::object();
    amp["model"] = "ipc2221";
    if (dt > 0) amp["temp_rise_c"] = dt;
    if (oz > 0) amp["copper_weight_oz"] = oz;
    cfg["ampacity"] = amp;
    return cfg;
}

}  // namespace

CT_TEST(default_inferred_model_is_ampacity) {
    Board b = amp_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    CT_CHECK(r.current().uses_ampacity());
    ElectricalContext ctx;
    std::string source;
    Coord w = r.requiredTraceWidth(0, 0, ctx, &source);
    CT_CHECK(source == "ampacity");
    // 5 A / 1 oz / 20 C external ~= 1.83 mm (IPC-2221 inversion), clamped sane.
    CT_CHECK(w > mm_to_nm(1.0));
    CT_CHECK(w < mm_to_nm(3.0));
}

CT_TEST(width_increases_monotonically_with_current) {
    Board b = amp_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    // Sub-clamp currents share the minimum width; above the clamp the
    // IPC-2221 inversion is strictly increasing in current.
    Coord prev = -1;
    for (double amps : {0.1, 0.5, 1.0, 2.0, 5.0, 10.0}) {
        set_current(b, 0, amps);
        RuleResolver rr = RuleResolver::defaults_for(b);
        Coord w = rr.requiredTraceWidth(0, 0, ctx);
        CT_CHECK(w >= prev);
        prev = w;
    }
    prev = -1;
    for (double amps : {1.0, 2.0, 5.0, 10.0}) {
        set_current(b, 0, amps);
        RuleResolver rr = RuleResolver::defaults_for(b);
        Coord w = rr.requiredTraceWidth(0, 0, ctx);
        CT_CHECK(w > prev);
        prev = w;
    }
    (void)r;
}

CT_TEST(thinner_copper_needs_more_width) {
    Board b = amp_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext c2, c1, c05;
    c2.copper_weight_oz = 2.0;
    c1.copper_weight_oz = 1.0;
    c05.copper_weight_oz = 0.5;
    Coord w2 = r.requiredTraceWidth(0, 0, c2);
    Coord w1 = r.requiredTraceWidth(0, 0, c1);
    Coord w05 = r.requiredTraceWidth(0, 0, c05);
    CT_CHECK(w2 < w1);
    CT_CHECK(w1 < w05);
}

CT_TEST(smaller_temp_rise_needs_more_width) {
    Board b = amp_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext c40, c20, c10;
    c40.temp_rise_c = 40.0;
    c20.temp_rise_c = 20.0;
    c10.temp_rise_c = 10.0;
    Coord w40 = r.requiredTraceWidth(0, 0, c40);
    Coord w20 = r.requiredTraceWidth(0, 0, c20);
    Coord w10 = r.requiredTraceWidth(0, 0, c10);
    CT_CHECK(w40 < w20);
    CT_CHECK(w20 < w10);
}

CT_TEST(internal_layer_needs_more_width_than_external) {
    // Same current on a 4-layer stackup: middle layer (~2.6x) > outer layers.
    Board b = base_2layer();
    b.layers.clear();
    b.layers.push_back({0, "Top"});
    b.layers.push_back({1, "GND"});
    b.layers.push_back({2, "VCC"});
    b.layers.push_back({3, "Bottom"});
    NetInfo pwr = make_net(0, "PWR");
    pwr.has_current = true;
    pwr.current_a = 3.0;
    b.nets.push_back(pwr);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    WidthDetails outer = r.widthDetails(0, 0, ctx);
    WidthDetails inner = r.widthDetails(0, 1, ctx);
    WidthDetails inner2 = r.widthDetails(0, 2, ctx);
    WidthDetails bottom = r.widthDetails(0, 3, ctx);
    CT_CHECK(!outer.internal_layer);
    CT_CHECK(inner.internal_layer);
    CT_CHECK(inner2.internal_layer);
    CT_CHECK(!bottom.internal_layer);
    CT_CHECK(inner.width_nm > outer.width_nm);
    CT_CHECK(inner2.width_nm > bottom.width_nm);
    // Static helper agrees: internal derating ratio ~= (0.048/0.024)^(1/0.725).
    double ext = AmpacityModel::width_mm_for(3.0, 1.0, 20.0, false);
    double in = AmpacityModel::width_mm_for(3.0, 1.0, 20.0, true);
    CT_CHECK_NEAR(in / ext, std::pow(2.0, 1.0 / 0.725), 0.05);
}

CT_TEST(explicit_width_still_overrides_ampacity) {
    Board b = amp_board();
    NetInfo* pwr = b.find_net(0);
    pwr->has_min_width = true;
    pwr->min_width_nm = mm_to_nm(0.3);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    std::string source;
    Coord w = r.requiredTraceWidth(0, 0, ctx, &source);
    CT_CHECK(w == mm_to_nm(0.3));
    CT_CHECK(source == "explicit");
}

CT_TEST(width_class_still_overrides_ampacity) {
    Board b = amp_board();
    b.find_net(0)->width_class = "SIGNAL";
    JsonValue cfg = ampacity_config();
    JsonValue classes = JsonValue::array();
    JsonValue c = JsonValue::object();
    c["name"] = "SIGNAL";
    c["min_width_mm"] = 0.2;
    classes.as_array().push_back(c);
    cfg["width_classes"] = classes;
    RuleResolver r = RuleResolver::from_config(b, cfg);
    ElectricalContext ctx;
    std::string source;
    CT_CHECK(r.requiredTraceWidth(0, 0, ctx, &source) == mm_to_nm(0.2));
    CT_CHECK(source == "width_class");
}

CT_TEST(legacy_linear_model_when_ipc_explicitly_configured) {
    // Backward compatibility: an old sidecar with only an "ipc" block keeps
    // the legacy linear estimator instead of the ampacity default.
    Board b = amp_board();
    JsonValue cfg = JsonValue::object();
    JsonValue ipc = JsonValue::object();
    ipc["mm_per_amp"] = 0.75;
    ipc["min_mm"] = 0.15;
    ipc["max_mm"] = 10.0;
    cfg["ipc"] = ipc;
    RuleResolver r = RuleResolver::from_config(b, cfg);
    CT_CHECK(!r.current().uses_ampacity());
    ElectricalContext ctx;
    std::string source;
    CT_CHECK(r.requiredTraceWidth(0, 0, ctx, &source) == mm_to_nm(3.75));
    CT_CHECK(source == "ipc_estimate");
}

CT_TEST(ampacity_wins_when_both_blocks_configured) {
    Board b = amp_board();
    JsonValue cfg = JsonValue::object();
    JsonValue ipc = JsonValue::object();
    ipc["mm_per_amp"] = 0.75;
    cfg["ipc"] = ipc;
    JsonValue amp = JsonValue::object();
    amp["model"] = "ipc2221";
    cfg["ampacity"] = amp;
    RuleResolver r = RuleResolver::from_config(b, cfg);
    CT_CHECK(r.current().uses_ampacity());
    ElectricalContext ctx;
    std::string source;
    r.requiredTraceWidth(0, 0, ctx, &source);
    CT_CHECK(source == "ampacity");
}

CT_TEST(ampacity_disabled_falls_back_to_legacy_ipc) {
    Board b = amp_board();
    JsonValue cfg = JsonValue::object();
    JsonValue ipc = JsonValue::object();
    ipc["mm_per_amp"] = 0.75;
    cfg["ipc"] = ipc;
    JsonValue amp = JsonValue::object();
    amp["enabled"] = false;
    cfg["ampacity"] = amp;
    RuleResolver r = RuleResolver::from_config(b, cfg);
    ElectricalContext ctx;
    std::string source;
    CT_CHECK(r.requiredTraceWidth(0, 0, ctx, &source) == mm_to_nm(3.75));
    CT_CHECK(source == "ipc_estimate");
}

CT_TEST(sidecar_thermal_overrides_reach_width_details) {
    Board b = amp_board();
    RuleResolver r = RuleResolver::from_config(b, ampacity_config(40.0, 2.0));
    ElectricalContext ctx;  // sentinel: inherits sidecar values
    WidthDetails d = r.widthDetails(0, 0, ctx);
    CT_CHECK(d.model == "ampacity");
    CT_CHECK_NEAR(d.temp_rise_c, 40.0, 1e-9);
    CT_CHECK_NEAR(d.copper_weight_oz, 2.0, 1e-9);
    CT_CHECK(d.current_a == 5.0);
    // Explicit ctx values beat the sidecar.
    ElectricalContext hot;
    hot.temp_rise_c = 10.0;
    WidthDetails d2 = r.widthDetails(0, 0, hot);
    CT_CHECK_NEAR(d2.temp_rise_c, 10.0, 1e-9);
    CT_CHECK(d2.width_nm > d.width_nm);
}

CT_TEST(layer_copper_override_beats_board_default) {
    Board b = amp_board();
    b.layers[0].copper_weight_oz = 2.0;
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    WidthDetails d0 = r.widthDetails(0, 0, ctx);
    WidthDetails d1 = r.widthDetails(0, 1, ctx);
    CT_CHECK_NEAR(d0.copper_weight_oz, 2.0, 1e-9);
    CT_CHECK_NEAR(d1.copper_weight_oz, 1.0, 1e-9);
    CT_CHECK(d0.width_nm < d1.width_nm);
}

CT_TEST(board_defaults_supply_thermal_baseline) {
    Board b = amp_board();
    b.defaults.copper_weight_oz = 2.0;
    b.defaults.temp_rise_c = 40.0;
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext dctx = r.defaultContext();
    CT_CHECK_NEAR(dctx.temp_rise_c, 40.0, 1e-9);
    CT_CHECK_NEAR(dctx.copper_weight_oz, 2.0, 1e-9);
    WidthDetails d = r.widthDetails(0, 0, dctx);
    CT_CHECK_NEAR(d.temp_rise_c, 40.0, 1e-9);
    CT_CHECK_NEAR(d.copper_weight_oz, 2.0, 1e-9);
}

CT_TEST(analyze_reports_ampacity_accounting) {
    Board b = amp_board();
    add_terminal(b, 0, 2.0, 10.0);
    add_terminal(b, 0, 18.0, 10.0);
    RuleResolver r = RuleResolver::defaults_for(b);
    AnalysisResult a = analyze_board(b, r, r.defaultContext(), {});
    const JsonValue* nets = a.data.find("nets");
    CT_CHECK(nets && nets->is_array() && !nets->as_array().empty());
    const JsonValue& n0 = nets->as_array().front();
    CT_CHECK(n0.get_string("width_model") == "ampacity");
    CT_CHECK(n0.get_string("width_source") == "ampacity");
    CT_CHECK(n0.get_number("copper_weight_oz", 0) > 0);
    CT_CHECK(n0.get_number("temp_rise_c", 0) > 0);
    CT_CHECK(n0.get_number("required_width_mm", 0) > 1.0);
    CT_CHECK(n0.get_number("current_a", 0) == 5.0);
}

CT_TEST(low_current_board_routes_under_ampacity_default) {
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
    RuleResolver r = RuleResolver::defaults_for(b);  // ampacity default
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    BoardVerifier v;
    ElectricalContext ctx;
    RuleResolver r2 = RuleResolver::defaults_for(engine.committed());
    CT_CHECK(v.verify(engine.committed(), r2, ctx).ok);
}

CT_TEST(high_current_board_routes_under_ampacity_default) {
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
    RuleResolver r = RuleResolver::defaults_for(b);  // ampacity default
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    Coord w_pwr = 0, w_sig = 0;
    for (const auto& t : engine.committed().traces) {
        if (t.net == 0) w_pwr = std::max(w_pwr, t.width_nm);
        if (t.net == 1) w_sig = std::max(w_sig, t.width_nm);
    }
    CT_CHECK(w_pwr > w_sig * 5);  // 5 A still consumes far more width
    BoardVerifier v;
    ElectricalContext ctx;
    RuleResolver r2 = RuleResolver::defaults_for(engine.committed());
    CT_CHECK(v.verify(engine.committed(), r2, ctx).ok);
}

int main() { return copperline::test::run_all_tests(); }
