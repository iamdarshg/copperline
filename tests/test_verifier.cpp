#include "helpers.h"

#include "router/verifier.h"

using namespace copperline;
using namespace copperline::test;

namespace {

Board two_traces_board() {
    Board b = base_2layer();
    NetInfo a = make_net(0, "A");
    b.nets.push_back(a);
    NetInfo c = make_net(1, "B");
    b.nets.push_back(c);
    TermId a0 = add_terminal(b, 0, 1.0, 5.0);
    TermId a1 = add_terminal(b, 0, 5.0, 5.0);
    TermId b0 = add_terminal(b, 1, 1.0, 6.0);
    TermId b1 = add_terminal(b, 1, 5.0, 6.0);
    (void)a0;
    (void)a1;
    (void)b0;
    (void)b1;
    const Terminal* ta0 = b.find_terminal(a0);
    const Terminal* ta1 = b.find_terminal(a1);
    const Terminal* tb0 = b.find_terminal(b0);
    const Terminal* tb1 = b.find_terminal(b1);
    b.traces.push_back({0, 0, ta0->pos, ta1->pos, mm_to_nm(0.2)});
    b.traces.push_back({1, 0, tb0->pos, tb1->pos, mm_to_nm(0.2)});
    return b;
}

}  // namespace

CT_TEST(connected_legal_board_passes) {
    Board b = two_traces_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    BoardVerifier v;
    VerifyResult vr = v.verify(b, r, ctx);
    CT_CHECK(vr.connected);
    CT_CHECK(vr.legal);
    CT_CHECK(vr.ok);
}

CT_TEST(keepout_violation_detected) {
    Board b = two_traces_board();
    Keepout ko;
    ko.rect = {mm_to_nm(2.0), mm_to_nm(4.0), mm_to_nm(4.0), mm_to_nm(6.0)};
    ko.layer = 0;
    ko.reason = "test-zone";
    b.keepouts.push_back(ko);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    BoardVerifier v;
    VerifyResult vr = v.verify(b, r, ctx);
    CT_CHECK(!vr.legal);
    CT_CHECK(!vr.ok);
    bool found = false;
    for (const auto& x : vr.violations)
        if (x.rule == "keepout") found = true;
    CT_CHECK(found);
}

CT_TEST(off_board_violation_detected) {
    Board b = two_traces_board();
    // A trace hanging 5mm past the 20mm board edge.
    b.traces.push_back({0, 0, {mm_to_nm(18.0), mm_to_nm(5.0)}, {mm_to_nm(25.0), mm_to_nm(5.0)},
                        mm_to_nm(0.2)});
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    BoardVerifier v;
    VerifyResult vr = v.verify(b, r, ctx);
    CT_CHECK(!vr.legal);
    bool found = false;
    for (const auto& x : vr.violations)
        if (x.type == "off_board") found = true;
    CT_CHECK(found);
}

CT_TEST(unknown_via_class_detected) {
    Board b = two_traces_board();
    Via v;
    v.net = 0;
    v.pos = {mm_to_nm(3.0), mm_to_nm(5.0)};
    v.top_layer = 0;
    v.bottom_layer = 1;
    v.outer_d_nm = mm_to_nm(0.6);
    v.hole_d_nm = mm_to_nm(0.3);
    v.via_class = "NOPE";
    b.vias.push_back(v);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    BoardVerifier vrf;
    VerifyResult vr = vrf.verify(b, r, ctx);
    CT_CHECK(!vr.legal);
    bool found = false;
    for (const auto& x : vr.violations)
        if (x.type == "via_class") found = true;
    CT_CHECK(found);
}

CT_TEST(missing_trace_reports_unconnected) {
    Board b = two_traces_board();
    b.traces.pop_back();  // net B loses its link
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    BoardVerifier v;
    VerifyResult vr = v.verify(b, r, ctx);
    CT_CHECK(!vr.connected);
    CT_CHECK(!vr.ok);
    // Both B pads are stranded; the report lists terminals outside the net's
    // reference component (first terminal), i.e. exactly one here.
    CT_CHECK(vr.unconnected.size() == 1);
    CT_CHECK(vr.unconnected[0].net_name == "B");
}

CT_TEST(width_violation_detected) {
    Board b = two_traces_board();
    NetInfo* a = b.find_net(0);
    a->has_min_width = true;
    a->min_width_nm = mm_to_nm(0.5);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    BoardVerifier v;
    VerifyResult vr = v.verify(b, r, ctx);
    CT_CHECK(!vr.legal);
    bool found = false;
    for (const auto& x : vr.violations)
        if (x.type == "width" && x.net_a == 0) found = true;
    CT_CHECK(found);
}

CT_TEST(clearance_violation_detected) {
    Board b = two_traces_board();
    // Default clearance 0.15mm; traces are 1mm apart -> force a 2mm rule.
    NetInfo* a = b.find_net(0);
    a->has_min_clearance = true;
    a->min_clearance_nm = mm_to_nm(2.0);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    BoardVerifier v;
    VerifyResult vr = v.verify(b, r, ctx);
    CT_CHECK(!vr.legal);
    bool found = false;
    for (const auto& x : vr.violations)
        if (x.type == "clearance" && x.rule == "net_floor") found = true;
    CT_CHECK(found);
}

CT_TEST(via_current_violation_detected) {
    Board b = base_2layer();
    NetInfo p = make_net(0, "PWR");
    p.has_current = true;
    p.current_a = 5.0;
    p.via_class = "SMALL";
    b.nets.push_back(p);
    add_terminal(b, 0, 5.0, 5.0, 0);
    add_terminal(b, 0, 6.0, 5.0, 1);
    Via v;
    v.net = 0;
    v.pos = {mm_to_nm(5.5), mm_to_nm(5.0)};
    v.top_layer = 0;
    v.bottom_layer = 1;
    v.outer_d_nm = mm_to_nm(0.4);
    v.hole_d_nm = mm_to_nm(0.2);
    v.via_class = "SMALL";
    b.vias.push_back(v);
    JsonValue cfg = JsonValue::object();
    JsonValue styles = JsonValue::array();
    JsonValue s = JsonValue::object();
    s["name"] = "SMALL";
    s["outer_mm"] = 0.4;
    s["hole_mm"] = 0.2;
    s["max_current_a"] = 1.0;
    styles.as_array().push_back(s);
    cfg["via_classes"] = styles;
    RuleResolver r = RuleResolver::from_config(b, cfg);
    ElectricalContext ctx;
    BoardVerifier ver;
    VerifyResult vr = ver.verify(b, r, ctx);
    bool found = false;
    for (const auto& x : vr.violations)
        if (x.type == "via_current") found = true;
    CT_CHECK(found);
}

// Issue #25: proximity is not current sharing. A second same-class via
// inside the bundle radius but stitched to the transition on neither layer
// must still report via_current; adding the planner's top+bottom star
// stubs clears the violation.
CT_TEST(nearby_unstitched_via_still_fails_via_current) {
    Board b = base_2layer();
    NetInfo p = make_net(0, "PWR");
    p.has_current = true;
    p.current_a = 1.5;  // SMALL carries 1A: needs 2 in parallel
    b.nets.push_back(p);
    add_terminal(b, 0, 2.0, 10.0, 0);
    add_terminal(b, 0, 18.0, 10.0, 1);
    JsonValue cfg = JsonValue::object();
    JsonValue styles = JsonValue::array();
    JsonValue s = JsonValue::object();
    s["name"] = "SMALL";
    s["outer_mm"] = 0.4;
    s["hole_mm"] = 0.2;
    s["max_current_a"] = 1.0;
    styles.as_array().push_back(s);
    cfg["via_classes"] = styles;
    RuleResolver r0 = RuleResolver::from_config(b, cfg);
    ElectricalContext ctx;
    std::string src;
    Coord w = r0.requiredTraceWidth(0, 0, ctx, &src);
    Point v1pos{mm_to_nm(10.0), mm_to_nm(10.0)};
    Point v2pos{mm_to_nm(10.5), mm_to_nm(10.0)};  // 0.5mm east: well inside
                                                 // the 1.35mm bundle window
    auto push_via = [&](Point pos) {
        Via v;
        v.net = 0;
        v.pos = pos;
        v.top_layer = 0;
        v.bottom_layer = 1;
        v.outer_d_nm = mm_to_nm(0.4);
        v.hole_d_nm = mm_to_nm(0.2);
        v.via_class = "SMALL";
        b.vias.push_back(v);
    };
    push_via(v1pos);
    push_via(v2pos);
    const Terminal& ta = b.terminals.front();
    const Terminal& tb = b.terminals.back();
    // Main path through via1 only; via2 floats unstitched.
    b.traces.push_back({0, 0, ta.pos, v1pos, w});
    b.traces.push_back({0, 1, v1pos, tb.pos, w});
    {
        RuleResolver r = RuleResolver::from_config(b, cfg);
        BoardVerifier ver;
        VerifyResult vr = ver.verify(b, r, ctx);
        CT_CHECK(vr.connected);
        bool found = false;
        for (const auto& x : vr.violations)
            if (x.type == "via_current") found = true;
        CT_CHECK(found);  // nearby != sharing: must still fail
    }
    // Planner star stubs tie via2 to the transition on both layers.
    b.traces.push_back({0, 0, v1pos, v2pos, w});
    b.traces.push_back({0, 1, v1pos, v2pos, w});
    {
        RuleResolver r = RuleResolver::from_config(b, cfg);
        BoardVerifier ver;
        VerifyResult vr = ver.verify(b, r, ctx);
        bool found = false;
        for (const auto& x : vr.violations)
            if (x.type == "via_current") found = true;
        CT_CHECK(!found);
        CT_CHECK(vr.legal);
        CT_CHECK(vr.ok);
    }
}

int main() { return copperline::test::run_all_tests(); }
