#include "helpers.h"

using namespace copperline;
using namespace copperline::test;

namespace {

Board current_board() {
    Board b = base_2layer();
    NetInfo sig = make_net(0, "SIG");
    sig.has_current = true;
    sig.current_a = 0.1;
    b.nets.push_back(sig);
    NetInfo pwr = make_net(1, "PWR");
    pwr.has_current = true;
    pwr.current_a = 5.0;
    b.nets.push_back(pwr);
    NetInfo exp = make_net(2, "EXPLICIT");
    exp.has_current = true;
    exp.current_a = 5.0;
    exp.has_min_width = true;
    exp.min_width_nm = mm_to_nm(0.3);
    b.nets.push_back(exp);
    NetInfo bare = make_net(3, "BARE");  // no current metadata at all
    b.nets.push_back(bare);
    return b;
}

JsonValue ipc_config(bool enabled = true) {
    JsonValue cfg = JsonValue::object();
    JsonValue ipc = JsonValue::object();
    ipc["enabled"] = enabled;
    ipc["mm_per_amp"] = 0.75;
    ipc["min_mm"] = 0.15;
    ipc["max_mm"] = 10.0;
    cfg["ipc"] = ipc;
    return cfg;
}

}  // namespace

CT_TEST(explicit_minimum_width) {
    Board b = current_board();
    RuleResolver r = RuleResolver::from_config(b, ipc_config());
    ElectricalContext ctx;
    std::string source;
    Coord w = r.requiredTraceWidth(2, 0, ctx, &source);
    CT_CHECK(w == mm_to_nm(0.3));
    CT_CHECK(source == "explicit");
}

CT_TEST(higher_current_consumes_more_width) {
    Board b = current_board();
    RuleResolver r = RuleResolver::from_config(b, ipc_config());
    ElectricalContext ctx;
    Coord sig = r.requiredTraceWidth(0, 0, ctx);
    Coord pwr = r.requiredTraceWidth(1, 0, ctx);
    CT_CHECK(pwr == mm_to_nm(5.0 * 0.75));  // 3.75mm IPC estimate
    CT_CHECK(sig == mm_to_nm(0.15));        // clamped to IPC minimum
    CT_CHECK(pwr > sig * 10);
}

CT_TEST(illegal_neckdown_rejected) {
    Board b = current_board();
    RuleResolver r = RuleResolver::from_config(b, ipc_config());
    ElectricalContext ctx;
    const NetInfo* pwr = b.find_net(1);
    CT_CHECK(pwr && !pwr->allow_neckdown);
    // 0.1mm neck on a 3.75mm-required net: must be rejected.
    CT_CHECK(!r.current().neckdown_legal(*pwr, mm_to_nm(0.1), mm_to_nm(1.0), b.defaults, ctx));
}

CT_TEST(legal_explicit_neckdown_accepted) {
    Board b = current_board();
    NetInfo* pwr = b.find_net(1);
    pwr->allow_neckdown = true;
    pwr->neck_width_nm = mm_to_nm(0.5);
    pwr->neck_max_len_nm = mm_to_nm(2.0);
    RuleResolver r = RuleResolver::from_config(b, ipc_config());
    ElectricalContext ctx;
    // Permitted neck geometry accepted...
    CT_CHECK(r.current().neckdown_legal(*pwr, mm_to_nm(0.6), mm_to_nm(1.0), b.defaults, ctx));
    // ...but a narrower-than-allowed neck is still rejected...
    CT_CHECK(!r.current().neckdown_legal(*pwr, mm_to_nm(0.4), mm_to_nm(1.0), b.defaults, ctx));
    // ...as is an over-long one.
    CT_CHECK(!r.current().neckdown_legal(*pwr, mm_to_nm(0.6), mm_to_nm(5.0), b.defaults, ctx));
}

CT_TEST(via_class_rejected_for_current) {
    Board b = current_board();
    JsonValue cfg = ipc_config();
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
    LayerSpan span{0, 1};
    auto allowed = r.allowedVias(1 /* PWR 5A */, span);
    CT_CHECK(allowed.size() == 1);
    CT_CHECK(allowed[0].name == "BIG");
    // Low-current net may use either.
    auto sig_allowed = r.allowedVias(0, span);
    CT_CHECK(sig_allowed.size() == 2);
}

CT_TEST(board_default_when_current_absent) {
    Board b = current_board();
    RuleResolver r = RuleResolver::from_config(b, ipc_config(false));
    ElectricalContext ctx;
    std::string source;
    Coord w = r.requiredTraceWidth(3, 0, ctx, &source);
    CT_CHECK(w == b.defaults.trace_width_nm);
    CT_CHECK(source == "board_default");
    CT_CHECK(r.default_current_used(3));
    CT_CHECK(!r.default_current_used(0));
}

CT_TEST(explicit_overrides_inferred) {
    Board b = current_board();
    RuleResolver r = RuleResolver::from_config(b, ipc_config());
    ElectricalContext ctx;
    std::string source;
    // 5A would infer 3.75mm, but the explicit 0.3mm rule wins (safety rule).
    Coord w = r.requiredTraceWidth(2, 0, ctx, &source);
    CT_CHECK(w == mm_to_nm(0.3));
    CT_CHECK(source == "explicit");
}

CT_TEST(width_class_model) {
    Board b = current_board();
    NetInfo* sig = b.find_net(0);
    sig->width_class = "SIGNAL";
    JsonValue cfg = ipc_config();
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

CT_TEST(parallel_vias_required_for_high_current) {
    Board b = current_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    ViaStyle std_style;
    CT_CHECK(r.lookup_via_style("STD", std_style));
    // 5A through a 2A STD via needs at least 3 in parallel.
    int n = r.current().vias_required(std_style, *b.find_net(1), b.defaults, ctx);
    CT_CHECK(n >= 3);
    int n_sig = r.current().vias_required(std_style, *b.find_net(0), b.defaults, ctx);
    CT_CHECK(n_sig == 1);
}

CT_TEST(allowed_vias_prefers_named_class) {
    Board b = current_board();
    NetInfo* sig = b.find_net(0);
    sig->via_class = "BIG";
    JsonValue cfg = ipc_config();
    JsonValue styles = JsonValue::array();
    JsonValue small = JsonValue::object();
    small["name"] = "STD";
    small["outer_mm"] = 0.6;
    small["hole_mm"] = 0.3;
    small["max_current_a"] = 2.0;
    styles.as_array().push_back(small);
    JsonValue big = JsonValue::object();
    big["name"] = "BIG";
    big["outer_mm"] = 1.0;
    big["hole_mm"] = 0.5;
    big["max_current_a"] = 8.0;
    styles.as_array().push_back(big);
    cfg["via_classes"] = styles;
    RuleResolver r = RuleResolver::from_config(b, cfg);
    LayerSpan span{0, 1};
    auto allowed = r.allowedVias(0, span);
    CT_CHECK(!allowed.empty());
    CT_CHECK(allowed[0].name == "BIG");  // preferred class first
    ViaStyle picked;
    CT_CHECK(r.select_via(0, span, picked));
    CT_CHECK(picked.name == "BIG");
}

CT_TEST(peak_current_drives_effective_current) {
    Board b = current_board();
    NetInfo* sig = b.find_net(0);
    sig->has_peak = true;
    sig->peak_a = 4.0;  // peak dominates the 0.1A continuous rating
    RuleResolver r = RuleResolver::from_config(b, ipc_config());
    ElectricalContext ctx;
    CT_CHECK(r.requiredTraceWidth(0, 0, ctx) == mm_to_nm(4.0 * 0.75));
}

int main() { return copperline::test::run_all_tests(); }
