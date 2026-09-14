#include "helpers.h"

#include "router/engine.h"
#include "router/verifier.h"

using namespace copperline;
using namespace copperline::test;

namespace {

Board voltage_board() {
    Board b = base_2layer(20.0, 10.0);
    NetInfo lv = make_net(0, "LOGIC");
    lv.has_voltage = true;
    lv.voltage_v = 3.3;
    b.nets.push_back(lv);
    NetInfo hv = make_net(1, "HV");
    hv.has_voltage = true;
    hv.voltage_v = 100.0;
    b.nets.push_back(hv);
    return b;
}

JsonValue voltage_config() {
    JsonValue cfg = JsonValue::object();
    JsonValue table = JsonValue::array();
    JsonValue e0 = JsonValue::object();
    e0["delta_v_min"] = 0.0;
    e0["clearance_mm"] = 0.15;
    table.as_array().push_back(e0);
    JsonValue e1 = JsonValue::object();
    e1["delta_v_min"] = 50.0;
    e1["clearance_mm"] = 1.0;
    table.as_array().push_back(e1);
    cfg["voltage_table"] = table;
    return cfg;
}

}  // namespace

CT_TEST(same_voltage_low_clearance) {
    Board b = voltage_board();
    // Second low-voltage net at the same potential.
    NetInfo lv2 = make_net(2, "LOGIC2");
    lv2.has_voltage = true;
    lv2.voltage_v = 3.3;
    b.nets.push_back(lv2);
    RuleResolver r = RuleResolver::from_config(b, voltage_config());
    ElectricalContext ctx;
    std::string source;
    Coord c = r.requiredClearance(0, 2, 0, ctx, &source);
    CT_CHECK(c == mm_to_nm(0.15));
    CT_CHECK(source == "voltage_table");
}

CT_TEST(large_delta_requires_more_clearance) {
    Board b = voltage_board();
    RuleResolver r = RuleResolver::from_config(b, voltage_config());
    ElectricalContext ctx;
    std::string source;
    Coord c = r.requiredClearance(0, 1, 0, ctx, &source);
    CT_CHECK(c == mm_to_nm(1.0));
    CT_CHECK(source == "voltage_table");
}

CT_TEST(pair_rule_overrides_table) {
    Board b = voltage_board();
    JsonValue cfg = voltage_config();
    JsonValue pairs = JsonValue::array();
    JsonValue p = JsonValue::object();
    p["a"] = "HV";
    p["b"] = "LOGIC";
    p["clearance_mm"] = 2.0;
    pairs.as_array().push_back(p);
    cfg["pair_rules"] = pairs;
    RuleResolver r = RuleResolver::from_config(b, cfg);
    ElectricalContext ctx;
    std::string source;
    Coord c = r.requiredClearance(0, 1, 0, ctx, &source);
    CT_CHECK(c == mm_to_nm(2.0));
    CT_CHECK(source == "pair_rule");
}

CT_TEST(astar_avoids_electrically_illegal_route) {
    // HV net must cross the board past a LOGIC pad. Geometrically a straight
    // shot is shortest, but 1mm voltage clearance makes it illegal, so the
    // router must detour (longer than Manhattan) and still verify clean.
    Board b = voltage_board();
    add_terminal(b, 1, 1.0, 5.0, 0, 0.5, "J1", "HV1");
    add_terminal(b, 1, 19.0, 5.0, 0, 0.5, "J2", "HV2");
    add_terminal(b, 0, 10.0, 5.0, 0, 1.0, "U1", "L1");
    add_terminal(b, 0, 10.0, 8.0, 0, 1.0, "U1", "L2");
    RuleResolver r = RuleResolver::from_config(b, voltage_config());
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    // Avoidance proof: the straight 18mm layer-0 shot is electrically illegal,
    // so the router must either detour (longer copper) or hop layers (vias).
    Coord hv_len = 0;
    for (const auto& t : engine.committed().traces) {
        if (t.net == 1) hv_len += manhattan(t.a, t.b);
    }
    CT_CHECK(hv_len > mm_to_nm(18.0) || rep.stats.via_count > 0);
    BoardVerifier v;
    ElectricalContext ctx;
    RuleResolver r2 = RuleResolver::from_config(engine.committed(), voltage_config());
    VerifyResult vr = v.verify(engine.committed(), r2, ctx);
    CT_CHECK(vr.ok);
}

CT_TEST(verifier_catches_voltage_clearance) {
    Board b = voltage_board();
    add_terminal(b, 0, 1.0, 5.0);
    add_terminal(b, 0, 3.0, 5.0);
    add_terminal(b, 1, 1.0, 6.0);
    add_terminal(b, 1, 3.0, 6.0);
    // Two parallel traces 0.2mm apart: fine for 0.15 default, illegal for the
    // 1mm HV/LOGIC requirement.
    TraceSeg s0{0, 0, {mm_to_nm(1.0), mm_to_nm(5.0)}, {mm_to_nm(3.0), mm_to_nm(5.0)}, mm_to_nm(0.15)};
    TraceSeg s1{1, 0, {mm_to_nm(1.0), mm_to_nm(5.2)}, {mm_to_nm(3.0), mm_to_nm(5.2)}, mm_to_nm(0.15)};
    b.traces.push_back(s0);
    b.traces.push_back(s1);
    RuleResolver r = RuleResolver::from_config(b, voltage_config());
    ElectricalContext ctx;
    BoardVerifier v;
    VerifyResult vr = v.verify(b, r, ctx);
    CT_CHECK(!vr.ok);
    bool found = false;
    for (const auto& x : vr.violations) {
        if (x.type == "clearance" && x.rule == "voltage_table") found = true;
    }
    CT_CHECK(found);
}

CT_TEST(class_pair_overrides_table) {
    Board b = voltage_board();
    b.nets[0].voltage_class = "LOGIC";
    b.nets[1].voltage_class = "HV";
    JsonValue cfg = voltage_config();
    JsonValue pairs = JsonValue::array();
    JsonValue p = JsonValue::object();
    p["a"] = "HV";
    p["b"] = "LOGIC";
    p["clearance_mm"] = 2.0;
    pairs.as_array().push_back(p);
    cfg["class_pairs"] = pairs;
    RuleResolver r = RuleResolver::from_config(b, cfg);
    ElectricalContext ctx;
    std::string source;
    Coord c = r.requiredClearance(0, 1, 0, ctx, &source);
    CT_CHECK(c == mm_to_nm(2.0));
    CT_CHECK(source == "class_pair");
}

CT_TEST(net_floor_max_wins) {
    Board b = voltage_board();
    // KiCad-style per-net floor above the table value: max wins.
    b.nets[0].has_min_clearance = true;
    b.nets[0].min_clearance_nm = mm_to_nm(1.5);
    RuleResolver r = RuleResolver::from_config(b, voltage_config());
    ElectricalContext ctx;
    std::string source;
    Coord c = r.requiredClearance(0, 1, 0, ctx, &source);
    CT_CHECK(c == mm_to_nm(1.5));
    CT_CHECK(source == "net_floor");
}

CT_TEST(unknown_voltage_falls_back_to_default) {
    Board b = voltage_board();
    NetInfo bare = make_net(2, "BARE");  // no voltage metadata
    b.nets.push_back(bare);
    RuleResolver r = RuleResolver::from_config(b, voltage_config());
    ElectricalContext ctx;
    std::string source;
    // Table needs BOTH voltages; without them the board default applies.
    Coord c = r.requiredClearance(0, 2, 0, ctx, &source);
    CT_CHECK(c == b.defaults.clearance_nm);
    CT_CHECK(source == "board_default");
}

CT_TEST(pair_rule_below_floor_raised_to_floor) {
    // Issue #6: explicit pair rule must not undercut either net's floor.
    Board b = voltage_board();
    b.nets[0].has_min_clearance = true;
    b.nets[0].min_clearance_nm = mm_to_nm(1.5);
    JsonValue cfg = voltage_config();
    JsonValue pairs = JsonValue::array();
    JsonValue p = JsonValue::object();
    p["a"] = "HV";
    p["b"] = "LOGIC";
    p["clearance_mm"] = 0.2;  // below the 1.5mm floor
    pairs.as_array().push_back(p);
    cfg["pair_rules"] = pairs;
    RuleResolver r = RuleResolver::from_config(b, cfg);
    ElectricalContext ctx;
    std::string source;
    Coord c = r.requiredClearance(0, 1, 0, ctx, &source);
    CT_CHECK(c == mm_to_nm(1.5));
    CT_CHECK(source == "net_floor");
    ClearanceResolution res = r.clearanceResolution(0, 1, 0, ctx);
    CT_CHECK(res.value_nm == mm_to_nm(1.5));
    CT_CHECK(res.candidate_nm == mm_to_nm(0.2));
    CT_CHECK(res.candidate_source == "pair_rule");
    CT_CHECK(res.floor_nm == mm_to_nm(1.5));
    CT_CHECK(res.floor_applied);
}

CT_TEST(pair_rule_below_both_floors_raised_to_larger) {
    Board b = voltage_board();
    b.nets[0].has_min_clearance = true;
    b.nets[0].min_clearance_nm = mm_to_nm(1.0);
    b.nets[1].has_min_clearance = true;
    b.nets[1].min_clearance_nm = mm_to_nm(2.0);
    JsonValue cfg = voltage_config();
    JsonValue pairs = JsonValue::array();
    JsonValue p = JsonValue::object();
    p["a"] = "HV";
    p["b"] = "LOGIC";
    p["clearance_mm"] = 0.2;
    pairs.as_array().push_back(p);
    cfg["pair_rules"] = pairs;
    RuleResolver r = RuleResolver::from_config(b, cfg);
    ElectricalContext ctx;
    Coord c = r.requiredClearance(0, 1, 0, ctx, nullptr);
    CT_CHECK(c == mm_to_nm(2.0));
    ClearanceResolution res = r.clearanceResolution(0, 1, 0, ctx);
    CT_CHECK(res.candidate_source == "pair_rule");
    CT_CHECK(res.floor_applied);
    CT_CHECK(res.floor_nm == mm_to_nm(2.0));
}

CT_TEST(pair_rule_above_floors_unchanged) {
    Board b = voltage_board();
    b.nets[0].has_min_clearance = true;
    b.nets[0].min_clearance_nm = mm_to_nm(0.5);
    JsonValue cfg = voltage_config();
    JsonValue pairs = JsonValue::array();
    JsonValue p = JsonValue::object();
    p["a"] = "HV";
    p["b"] = "LOGIC";
    p["clearance_mm"] = 2.0;  // above the floor
    pairs.as_array().push_back(p);
    cfg["pair_rules"] = pairs;
    RuleResolver r = RuleResolver::from_config(b, cfg);
    ElectricalContext ctx;
    std::string source;
    Coord c = r.requiredClearance(0, 1, 0, ctx, &source);
    CT_CHECK(c == mm_to_nm(2.0));
    CT_CHECK(source == "pair_rule");
    ClearanceResolution res = r.clearanceResolution(0, 1, 0, ctx);
    CT_CHECK(!res.floor_applied);
    CT_CHECK(res.candidate_source == "pair_rule");
}

CT_TEST(floor_enforced_verifier_and_astar_agree) {
    // Same resolved clearance must gate both search legality and verification.
    Board b = voltage_board();
    b.nets[0].has_min_clearance = true;
    b.nets[0].min_clearance_nm = mm_to_nm(1.0);
    JsonValue cfg = voltage_config();
    JsonValue pairs = JsonValue::array();
    JsonValue p = JsonValue::object();
    p["a"] = "HV";
    p["b"] = "LOGIC";
    p["clearance_mm"] = 0.2;  // floored to 1.0mm
    pairs.as_array().push_back(p);
    cfg["pair_rules"] = pairs;
    // Two parallel traces 0.5mm apart: legal under the raw pair rule (0.2mm)
    // but illegal once the 1.0mm floor is enforced.
    add_terminal(b, 0, 1.0, 5.0);
    add_terminal(b, 0, 3.0, 5.0);
    add_terminal(b, 1, 1.0, 6.0);
    add_terminal(b, 1, 3.0, 6.0);
    TraceSeg s0{0, 0, {mm_to_nm(1.0), mm_to_nm(5.0)}, {mm_to_nm(3.0), mm_to_nm(5.0)}, mm_to_nm(0.15)};
    TraceSeg s1{1, 0, {mm_to_nm(1.0), mm_to_nm(5.5)}, {mm_to_nm(3.0), mm_to_nm(5.5)}, mm_to_nm(0.15)};
    b.traces.push_back(s0);
    b.traces.push_back(s1);
    RuleResolver r = RuleResolver::from_config(b, cfg);
    ElectricalContext ctx;
    // Resolver reports the floored value with both stages visible.
    ClearanceResolution res = r.clearanceResolution(0, 1, 0, ctx);
    CT_CHECK(res.value_nm == mm_to_nm(1.0));
    CT_CHECK(res.candidate_source == "pair_rule");
    CT_CHECK(res.floor_applied);
    // Verifier uses the same resolver result, so it must flag the pair.
    BoardVerifier v;
    VerifyResult vr = v.verify(b, r, ctx);
    CT_CHECK(!vr.ok);
    bool found = false;
    for (const auto& x : vr.violations) {
        if (x.type == "clearance" && x.rule == "net_floor") found = true;
    }
    CT_CHECK(found);
}

int main() { return copperline::test::run_all_tests(); }
