// Prompt 2 tests: fine-pitch detection, centre-out escape, K-best diversity,
// electrical constraints inside footprints, golden fixtures.
#include <algorithm>
#include <map>
#include <set>

#include "helpers.h"
#include "router/board.h"
#include "router/escape.h"
#include "router/verifier.h"

using namespace copperline;
using namespace copperline::test;

#ifndef FIXTURE_DIR
#define FIXTURE_DIR "fixtures"
#endif

namespace {

Board load_fixture(const std::string& name) {
    ImportResult r = import_board_auto(std::string(FIXTURE_DIR) + "/" + name);
    return std::move(r.board);
}

RuleResolver defaults_for_board(const Board& b) { return RuleResolver::defaults_for(b); }

const FootprintEscapeResult* find_fp(const EscapeResult& r, const std::string& comp) {
    for (const auto& fp : r.footprints) {
        if (fp.footprint.component == comp) return &fp;
    }
    return nullptr;
}

const PadEscapeResult* find_pad(const FootprintEscapeResult& fp, TermId tid) {
    for (const auto& p : fp.pads) {
        if (p.terminal == tid) return &p;
    }
    return nullptr;
}

}  // namespace

CT_TEST(detector_flags_4x4_bga) {
    Board b = load_fixture("bga_4x4.json");
    RuleResolver r = defaults_for_board(b);
    ElectricalContext ctx;
    FinePitchDetector det;
    auto fps = det.detect(b, r, ctx);
    CT_CHECK(!fps.empty());
    bool u1 = false;
    for (const auto& fp : fps) {
        if (fp.component == "U1") {
            u1 = true;
            CT_CHECK(fp.pad_count == 16);
            CT_CHECK(fp.is_fine_pitch);
        }
    }
    CT_CHECK(u1);
}

CT_TEST(detector_ignores_open_board) {
    Board b = base_2layer();
    NetInfo s = make_net(0, "SIG1");
    b.nets.push_back(s);
    add_terminal(b, 0, 2.0, 10.0);
    add_terminal(b, 0, 18.0, 10.0);
    RuleResolver r = defaults_for_board(b);
    ElectricalContext ctx;
    FinePitchDetector det;
    CT_CHECK(det.detect(b, r, ctx).empty());
}

CT_TEST(centre_depth_4x4_rings) {
    Board b = load_fixture("bga_4x4.json");
    RuleResolver r = defaults_for_board(b);
    ElectricalContext ctx;
    FinePitchDetector det;
    auto fps = det.detect(b, r, ctx);
    CT_CHECK(!fps.empty());
    CentreDepthAnalyzer cda;
    auto depth = cda.analyze(b, fps.front());
    int max_d = 0, zero = 0;
    for (const auto& [tid, d] : depth) {
        max_d = std::max(max_d, d);
        if (d == 0) ++zero;
    }
    CT_CHECK(max_d == 1);   // 4x4: perimeter 0, inner 2x2 depth 1
    CT_CHECK(zero == 12);   // outer ring
}

CT_TEST(centre_depth_8x8_rings) {
    Board b = load_fixture("bga_8x8.json");
    RuleResolver r = defaults_for_board(b);
    ElectricalContext ctx;
    FinePitchDetector det;
    auto fps = det.detect(b, r, ctx);
    CT_CHECK(!fps.empty());
    CentreDepthAnalyzer cda;
    auto depth = cda.analyze(b, fps.front());
    int max_d = 0;
    for (const auto& [tid, d] : depth) max_d = std::max(max_d, d);
    CT_CHECK(max_d == 3);  // 8x8: depths 0..3, centre deepest
    // Perimeter pads are all depth 0.
    int zero = 0;
    for (const auto& [tid, d] : depth) {
        if (d == 0) ++zero;
    }
    CT_CHECK(zero == 28);  // 8*4-4 perimeter pads
}

CT_TEST(eligibility_is_centre_out) {
    Board b = load_fixture("bga_8x8.json");
    RuleResolver r = defaults_for_board(b);
    ElectricalContext ctx;
    EscapePlanner planner;
    EscapeResult res = planner.plan(b, r, ctx);
    const auto* fp = find_fp(res, "U1");
    CT_CHECK(fp != nullptr);
    // Eligibility order must be non-increasing in centre depth.
    int prev = 1 << 30;
    for (TermId tid : fp->eligibility_order) {
        const PadEscapeResult* p = find_pad(*fp, tid);
        CT_CHECK(p != nullptr);
        CT_CHECK(p->centre_depth <= prev);
        prev = p->centre_depth;
    }
    // First eligible pad is a deepest central pad.
    const PadEscapeResult* first = find_pad(*fp, fp->eligibility_order.front());
    CT_CHECK(first->centre_depth == 3);
}

CT_TEST(centre_out_invariant_holds) {
    Board b = load_fixture("bga_4x4.json");
    RuleResolver r = defaults_for_board(b);
    ElectricalContext ctx;
    EscapePlanner planner;
    EscapeResult res = planner.plan(b, r, ctx);
    const auto* fp = find_fp(res, "U1");
    CT_CHECK(fp != nullptr);
    // Invariant: a shallower terminal is never committed while a deeper
    // eligible terminal has neither a viable candidate nor a record.
    std::map<TermId, const PadEscapeResult*> by_term;
    for (const auto& p : fp->pads) by_term[p.terminal] = &p;
    std::set<TermId> committed(fp->commit_order.begin(), fp->commit_order.end());
    for (std::size_t i = 0; i < fp->eligibility_order.size(); ++i) {
        TermId shallow = fp->eligibility_order[i];
        if (!committed.count(shallow)) continue;
        for (std::size_t j = 0; j < i; ++j) {
            TermId deeper = fp->eligibility_order[j];
            const PadEscapeResult* dp = by_term[deeper];
            CT_CHECK(dp->has_viable || dp->infeasibility.recorded);
        }
    }
}

CT_TEST(kbest_signatures_are_distinct) {
    Board b = load_fixture("bga_4x4.json");
    RuleResolver r = defaults_for_board(b);
    ElectricalContext ctx;
    EscapePlanner planner;
    EscapeResult res = planner.plan(b, r, ctx);
    const auto* fp = find_fp(res, "U1");
    CT_CHECK(fp != nullptr);
    bool saw_multi = false;
    for (const auto& p : fp->pads) {
        if (p.candidates.size() > 1) {
            saw_multi = true;
            std::set<std::string> sigs;
            for (const auto& c : p.candidates) sigs.insert(c.signature);
            CT_CHECK(sigs.size() == p.candidates.size());  // genuinely different
        }
    }
    CT_CHECK(saw_multi);  // at least one pad has real alternatives
}

CT_TEST(qfn_detected_and_ordered) {
    Board b = load_fixture("dense_qfn.json");
    RuleResolver r = defaults_for_board(b);
    ElectricalContext ctx;
    FinePitchDetector det;
    auto fps = det.detect(b, r, ctx);
    bool uq = false;
    for (const auto& fp : fps) {
        if (fp.component == "UQ") {
            uq = true;
            CT_CHECK(fp.pad_count == 32);
        }
    }
    CT_CHECK(uq);
    EscapePlanner planner;
    EscapeResult res = planner.plan(b, r, ctx);
    const auto* fp = find_fp(res, "UQ");
    CT_CHECK(fp != nullptr);
    CT_CHECK(fp->eligibility_order.size() == 32);
}

CT_TEST(irregular_array_detected) {
    Board b = load_fixture("irregular_array.json");
    RuleResolver r = defaults_for_board(b);
    ElectricalContext ctx;
    EscapePlanner planner;
    EscapeResult res = planner.plan(b, r, ctx);
    const auto* fp = find_fp(res, "U2");
    CT_CHECK(fp != nullptr);
    CT_CHECK(fp->footprint.pad_count == 13);
}

CT_TEST(neckdown_legal_model) {
    Board b = base_2layer();
    NetInfo n = make_net(0, "PWR");
    n.has_min_width = true;
    n.min_width_nm = mm_to_nm(0.5);
    n.allow_neckdown = true;
    n.neck_width_nm = mm_to_nm(0.25);
    n.neck_max_len_nm = mm_to_nm(2.0);
    b.nets.push_back(n);
    RuleResolver r = defaults_for_board(b);
    ElectricalContext ctx;
    CT_CHECK(r.current().neckdown_legal(n, mm_to_nm(0.25), mm_to_nm(1.0), b.defaults, ctx));
    CT_CHECK(!r.current().neckdown_legal(n, mm_to_nm(0.1), mm_to_nm(1.0), b.defaults, ctx));
    NetInfo strict = make_net(1, "STRICT");
    strict.has_min_width = true;
    strict.min_width_nm = mm_to_nm(0.5);
    b.nets.push_back(strict);
    CT_CHECK(!r.current().neckdown_legal(strict, mm_to_nm(0.25), mm_to_nm(1.0), b.defaults, ctx));
}

CT_TEST(high_current_bga_honors_width) {
    Board b = load_fixture("high_current_bga.json");
    RuleResolver r = defaults_for_board(b);
    ElectricalContext ctx;
    EscapePlanner planner;
    EscapeResult res = planner.plan(b, r, ctx);
    const auto* fp = find_fp(res, "UP");
    CT_CHECK(fp != nullptr);
    // No candidate may silently narrow below the required width: every trace
    // is either full width or a legal neckdown.
    for (const auto& p : fp->pads) {
        const Terminal* t = b.find_terminal(p.terminal);
        std::string ws;
        Coord need = r.requiredTraceWidth(t->net, t->layer, ctx, &ws);
        for (const auto& c : p.candidates) {
            if (c.width_nm < need) {
                CT_CHECK(c.use_neckdown);
                const NetInfo* n = b.find_net(t->net);
                CT_CHECK(r.current().neckdown_legal(*n, c.width_nm, fp->footprint.pitch_nm,
                                                   b.defaults, ctx));
            }
        }
    }
}

CT_TEST(via_class_rejected_for_current) {
    Board b = base_2layer();
    NetInfo n = make_net(0, "HOT");
    n.has_current = true;
    n.current_a = 10.0;
    b.nets.push_back(n);
    add_terminal(b, 0, 5.0, 5.0);
    RuleResolver r = defaults_for_board(b);  // STD max 2A
    CT_CHECK(r.allowedVias(0, {0, 1}).empty());
    ViaStyle s;
    CT_CHECK(!r.select_via(0, {0, 1}, s));
}

CT_TEST(voltage_clearance_pair_rule) {
    Board b = load_fixture("mixed_voltage_bga.json");
    JsonValue cfg = JsonValue::object();
    JsonValue vt = JsonValue::array();
    JsonValue e0 = JsonValue::object();
    e0["delta_v_min"] = 0.0;
    e0["clearance_mm"] = 0.15;
    JsonValue e1 = JsonValue::object();
    e1["delta_v_min"] = 50.0;
    e1["clearance_mm"] = 1.0;
    vt.as_array().push_back(e0);
    vt.as_array().push_back(e1);
    cfg["voltage_table"] = vt;
    RuleResolver r = RuleResolver::from_config(b, cfg);
    ElectricalContext ctx;
    // HV net vs LOGIC net: delta 56.7V -> 1.0mm clearance.
    NetId hv = -1, logic = -1;
    for (const auto& n : b.nets) {
        if (n.voltage_class == "HV" && hv < 0) hv = n.id;
        if (n.voltage_class == "LOGIC" && logic < 0) logic = n.id;
    }
    CT_CHECK(hv >= 0 && logic >= 0);
    std::string src;
    Coord c = r.requiredClearance(hv, logic, 0, ctx, &src);
    CT_CHECK(c == mm_to_nm(1.0));
    CT_CHECK(src == "voltage_table");
    // Escape under this rule: candidates must exist or carry explicit records.
    EscapePlanner planner;
    EscapeResult res = planner.plan(b, r, ctx);
    const auto* fp = find_fp(res, "UH");
    CT_CHECK(fp != nullptr);
    for (const auto& p : fp->pads) {
        CT_CHECK(p.has_viable || p.infeasibility.recorded);
    }
}

CT_TEST(greedy_fixture_centre_first) {
    Board b = load_fixture("greedy_outside_first.json");
    RuleResolver r = defaults_for_board(b);
    ElectricalContext ctx;
    EscapePlanner planner;
    EscapeResult res = planner.plan(b, r, ctx);
    const auto* fp = find_fp(res, "UG");
    CT_CHECK(fp != nullptr);
    // Centre pad (deepest) leads the eligibility order; perimeter is last.
    const PadEscapeResult* first = find_pad(*fp, fp->eligibility_order.front());
    const PadEscapeResult* last = find_pad(*fp, fp->eligibility_order.back());
    CT_CHECK(first->centre_depth > last->centre_depth);
    CT_CHECK(last->centre_depth == 0);
}

CT_TEST(impossible_escape_reports_infeasible) {
    Board b = load_fixture("impossible_escape.json");
    RuleResolver r = defaults_for_board(b);
    ElectricalContext ctx;
    EscapePlanner planner;
    EscapeResult res = planner.plan(b, r, ctx);
    CT_CHECK(res.pads_infeasible > 0);
    const auto* fp = find_fp(res, "UX");
    CT_CHECK(fp != nullptr);
    bool saw_reason = false;
    for (const auto& p : fp->pads) {
        if (!p.has_viable) {
            CT_CHECK(p.infeasibility.recorded);
            CT_CHECK(!p.infeasibility.reason.empty());
            CT_CHECK(!p.infeasibility.blockers.empty());
            saw_reason = true;
        }
    }
    CT_CHECK(saw_reason);
}

CT_TEST(escape_json_schema) {
    Board b = load_fixture("bga_4x4.json");
    RuleResolver r = defaults_for_board(b);
    ElectricalContext ctx;
    EscapePlanner planner;
    EscapeResult res = planner.plan(b, r, ctx);
    JsonValue j = res.to_json();
    CT_CHECK(j.get_string("schema") == "copperline/escape-report/1");
    CT_CHECK(j.has("footprints"));
    CT_CHECK(!j.find("footprints")->as_array().empty());
    const JsonValue& fp = j.find("footprints")->as_array().front();
    for (const char* k :
         {"component", "eligibility_order", "commit_order", "pads", "boundary"}) {
        CT_CHECK(fp.has(k));
    }
    const JsonValue& pad = fp.find("pads")->as_array().front();
    for (const char* k : {"terminal", "centre_depth", "density_per_mm2", "eligibility_index",
                          "candidate_count", "candidate_portals", "via_decisions"}) {
        CT_CHECK(pad.has(k));
    }
}

CT_TEST(escape_is_deterministic) {
    Board b = load_fixture("bga_4x4.json");
    RuleResolver r = defaults_for_board(b);
    ElectricalContext ctx;
    EscapePlanner planner;
    JsonValue j1 = planner.plan(b, r, ctx).to_json();
    JsonValue j2 = planner.plan(b, r, ctx).to_json();
    CT_CHECK(serialize_json(j1) == serialize_json(j2));
}

CT_TEST(via_fixture_inner_rings_need_vias) {
    Board b = load_fixture("bga_8x8_via.json");
    RuleResolver r = defaults_for_board(b);
    ElectricalContext ctx;
    // Targets sit on Bottom while pads are on Top. Perimeter pads escape on
    // the surface layer; inner rings (fully surrounded, zero same-layer
    // channels) must change layers, i.e. use a via.
    FinePitchDetector det;
    auto fps = det.detect(b, r, ctx);
    CT_CHECK(!fps.empty());
    CentreDepthAnalyzer cda;
    auto depth = cda.analyze(b, fps.front());
    EscapePlanner planner;
    EscapeResult res = planner.plan(b, r, ctx);
    const auto* fp = find_fp(res, "U1");
    CT_CHECK(fp != nullptr);
    int inner_viable = 0;
    for (const auto& p : fp->pads) {
        int d = depth.count(p.terminal) ? depth.at(p.terminal) : 0;
        if (!p.has_viable || d < 2) continue;
        ++inner_viable;
        bool any_via = false;
        for (const auto& c : p.candidates) {
            if (c.via_count > 0) any_via = true;
        }
        CT_CHECK(any_via);
    }
    CT_CHECK(inner_viable > 0);
}

CT_TEST(multilayer_strategies_appear) {
    // 4x4 already exercises real layer changes (inner pads escape via-first
    // while perimeter pads stay on the surface layer); the 8x8via golden
    // fixture is covered by via_fixture_inner_rings_need_vias.
    Board b = load_fixture("bga_4x4.json");
    RuleResolver r = defaults_for_board(b);
    ElectricalContext ctx;
    EscapePlanner planner;
    EscapeResult res = planner.plan(b, r, ctx);
    std::set<std::string> strategies;
    for (const auto& fp : res.footprints) {
        for (const auto& p : fp.pads) {
            for (const auto& c : p.candidates) strategies.insert(c.layer_strategy);
        }
    }
    // Same-layer-only would be a degenerate escape router.
    CT_CHECK(strategies.size() > 1);
}

CT_TEST(neckdown_rejected_on_internal_thin_layer) {
    // Issue #24 regression: escape neckdown legality must use the
    // layer-aware ampacity width (per-layer copper + internal derating),
    // never the external/default width. 3 A on 1 oz external needs ~0.9 mm;
    // the same current on the 0.5 oz internal layer needs ~4.7 mm. A 1.0 mm
    // neck with a 0.1 mm max length passes the old external/default check
    // (early "not a neckdown" return, length ignored) but is a true
    // over-length neckdown on the internal layer and must be rejected there.
    Board b;
    b.source_format = "test";
    b.width_nm = mm_to_nm(20.0);
    b.height_nm = mm_to_nm(20.0);
    b.layers.push_back({0, "Top"});
    b.layers.push_back({1, "In1"});
    b.layers.push_back({2, "In2"});
    b.layers.push_back({3, "Bottom"});
    b.layers[1].copper_weight_oz = 0.5;  // thin internal copper
    NetInfo pwr = make_net(0, "PWR");
    pwr.has_current = true;
    pwr.current_a = 3.0;
    pwr.allow_neckdown = true;
    pwr.neck_width_nm = mm_to_nm(1.0);
    pwr.neck_max_len_nm = mm_to_nm(0.1);
    b.nets.push_back(pwr);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;

    const NetInfo* n = b.find_net(0);
    std::string src;
    Coord req_ext = r.current().required_min_width(*n, b, 0, ctx, src);
    Coord req_int = r.current().required_min_width(*n, b, 1, ctx, src);
    CT_CHECK(req_int > req_ext);  // internal/thin needs strictly more width
    CT_CHECK(mm_to_nm(1.0) >= req_ext);
    CT_CHECK(mm_to_nm(1.0) < req_int);

    // Old external/default overload accepts (the #24 bypass)...
    CT_CHECK(r.current().neckdown_legal(*n, mm_to_nm(1.0), mm_to_nm(0.5),
                                        b.defaults, ctx));
    // ...while the layer-aware overload accepts on the external layer but
    // rejects on the internal thin-copper layer (over max length).
    CT_CHECK(r.current().neckdown_legal(*n, mm_to_nm(1.0), mm_to_nm(0.5),
                                        b, 0, ctx));
    CT_CHECK(!r.current().neckdown_legal(*n, mm_to_nm(1.0), mm_to_nm(0.5),
                                         b, 1, ctx));

    // End to end: 4x4 fine-pitch grid on the internal layer. The escape
    // planner must not emit any necked candidate for these pads.
    for (int ix = 0; ix < 4; ++ix) {
        for (int iy = 0; iy < 4; ++iy) {
            add_terminal(b, 0, 9.25 + 0.5 * ix, 9.25 + 0.5 * iy,
                         /*layer=*/1, /*pad_mm=*/0.3, "U1");
        }
    }
    RuleResolver r2 = RuleResolver::defaults_for(b);
    EscapePlanner planner;
    EscapeResult res = planner.plan(b, r2, ctx);
    const auto* fp = find_fp(res, "U1");
    CT_CHECK(fp != nullptr);
    CT_CHECK(fp->footprint.pad_count == 16);
    bool saw_viable = false;
    for (const auto& p : fp->pads) {
        if (p.has_viable) saw_viable = true;
        for (const auto& c : p.candidates) {
            CT_CHECK(!c.use_neckdown);  // no internal neckdown escapes
        }
    }
    CT_CHECK(saw_viable);  // full-width escapes exist: the check is not vacuous
}

int main() { return copperline::test::run_all_tests(); }
