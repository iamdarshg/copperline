// Issue #13: independent pair-aware verification from committed copper.
//
// The verifier derives everything from traces/vias/pads (never router
// metadata): per-member lengths, skew, coupled-section gap (exact integer
// math, arbitrary-angle aware), compatible layers, paired-via symmetry and
// one-member-only detection. Each failure mode fails independently; the
// route JSON carries P/N lengths, skew, worst gap error, gap location,
// via mismatch and pair status; COMPLETE requires the verifier clean.
#include <algorithm>
#include <cmath>

#include "helpers.h"

#include "router/diffpair.h"
#include "router/engine.h"
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

Board route_fixture(const std::string& name) {
    Board b = load_fixture_board(name);
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = 1;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.verification.ok);
    return engine.committed();
}

VerifyResult verify_board(const Board& board) {
    RuleResolver r = RuleResolver::defaults_for(board);
    BoardVerifier v;
    return v.verify(board, r, r.defaultContext());
}

bool has_violation_type(const VerifyResult& vr, const std::string& type) {
    for (const auto& v : vr.violations) {
        if (v.type == type) return true;
    }
    return false;
}

bool violation_detail_has(const VerifyResult& vr, const std::string& type,
                          const std::string& frag) {
    for (const auto& v : vr.violations) {
        if (v.type == type && v.detail.find(frag) != std::string::npos)
            return true;
    }
    return false;
}

const PairVerifyDetail* find_pair(const VerifyResult& vr, int pair_id) {
    for (const auto& p : vr.pairs) {
        if (p.pair_id == pair_id) return &p;
    }
    return nullptr;
}

// Synthetic diagonal (arbitrary-angle) pair board. N pads/traces shift with
// (0, dy_mm) so the test controls the coupled edge gap while keeping both
// members connected pad-to-pad. Base geometry: P (4,4)-(16,13), 3-4-5 slope,
// N offset by 0.4*normal = (-0.24,+0.32) for a nominal 0.2 edge gap.
Board diagonal_pair_board(double dy_mm) {
    Board b = base_2layer(20.0, 20.0);
    b.defaults.trace_width_nm = mm_to_nm(0.2);
    b.defaults.clearance_nm = mm_to_nm(0.15);
    b.defaults.via_outer_nm = mm_to_nm(0.6);
    b.defaults.via_hole_nm = mm_to_nm(0.3);
    b.defaults.default_current_a = 0.5;
    NetInfo p = make_net(0, "P");
    p.current_a = 0.1;
    p.has_current = true;
    p.voltage_v = 3.3;
    p.has_voltage = true;
    NetInfo n = make_net(1, "N");
    n.current_a = 0.1;
    n.has_current = true;
    n.voltage_v = 3.3;
    n.has_voltage = true;
    b.nets.push_back(p);
    b.nets.push_back(n);
    TermId p0 = add_terminal(b, 0, 4.0, 4.0, 0, 0.1, "J1", "1");
    TermId p1 = add_terminal(b, 0, 16.0, 13.0, 0, 0.1, "J1", "2");
    TermId n0 = add_terminal(b, 1, 3.76, 4.32 + dy_mm, 0, 0.1, "J1", "3");
    TermId n1 = add_terminal(b, 1, 15.76, 13.32 + dy_mm, 0, 0.1, "J1", "4");
    (void)p0;
    (void)p1;
    (void)n0;
    (void)n1;
    const Terminal* tp0 = b.find_terminal(p0);
    const Terminal* tp1 = b.find_terminal(p1);
    const Terminal* tn0 = b.find_terminal(n0);
    const Terminal* tn1 = b.find_terminal(n1);
    b.traces.push_back({0, 0, tp0->pos, tp1->pos, mm_to_nm(0.2)});
    b.traces.push_back({1, 0, tn0->pos, tn1->pos, mm_to_nm(0.2)});
    DiffPair pr;
    pr.id = 0;
    pr.name = "DIAG";
    pr.net_p = 0;
    pr.net_n = 1;
    pr.gap_nm = mm_to_nm(0.2);
    pr.gap_tol_nm = mm_to_nm(0.05);
    pr.has_width = true;
    pr.width_nm = mm_to_nm(0.2);
    pr.has_max_skew = true;
    pr.max_skew_nm = mm_to_nm(1.0);
    pr.via_policy = "paired";
    b.diffpairs.push_back(pr);
    return b;
}

// Straight 3-segment pair board with nominal pad pitch. Pads sit at the
// nominal 0.2 edge (P row y=10, N row y=10.4, 0.1 pads, 0.2 traces) so the
// end segments are nominal fans; the middle sections (touching no pad) are
// trunk and strictly banded. dy_mid_mm shifts only the N middle vertices,
// controlling the trunk edge gap while keeping both members connected
// pad-to-pad. The P middle (x 9..11) sits fully inside the N middle span
// (x 8..12) with far (>1mm) corners, so the trunk nearest is unique: the
// reported drift location never depends on a corner tiebreak.
Board straight_trunk_board(double dy_mid_mm) {
    Board b = base_2layer(20.0, 20.0);
    b.defaults.trace_width_nm = mm_to_nm(0.2);
    b.defaults.clearance_nm = mm_to_nm(0.15);
    b.defaults.via_outer_nm = mm_to_nm(0.6);
    b.defaults.via_hole_nm = mm_to_nm(0.3);
    b.defaults.default_current_a = 0.5;
    NetInfo p = make_net(0, "P");
    p.current_a = 0.1;
    p.has_current = true;
    p.voltage_v = 3.3;
    p.has_voltage = true;
    NetInfo n = make_net(1, "N");
    n.current_a = 0.1;
    n.has_current = true;
    n.voltage_v = 3.3;
    n.has_voltage = true;
    b.nets.push_back(p);
    b.nets.push_back(n);
    TermId p0 = add_terminal(b, 0, 2.0, 10.0, 0, 0.1, "J1", "1");
    TermId p1 = add_terminal(b, 0, 18.0, 10.0, 0, 0.1, "J1", "2");
    TermId n0 = add_terminal(b, 1, 2.0, 10.4, 0, 0.1, "J1", "3");
    TermId n1 = add_terminal(b, 1, 18.0, 10.4, 0, 0.1, "J1", "4");
    (void)p0;
    (void)p1;
    (void)n0;
    (void)n1;
    Coord w = mm_to_nm(0.2);
    Point pp0{mm_to_nm(2.0), mm_to_nm(10.0)}, pp1{mm_to_nm(9.0), mm_to_nm(10.0)};
    Point pp2{mm_to_nm(11.0), mm_to_nm(10.0)}, pp3{mm_to_nm(18.0), mm_to_nm(10.0)};
    Point np0{mm_to_nm(2.0), mm_to_nm(10.4)}, np3{mm_to_nm(18.0), mm_to_nm(10.4)};
    Point np1{mm_to_nm(8.0), mm_to_nm(10.4 + dy_mid_mm)};
    Point np2{mm_to_nm(12.0), mm_to_nm(10.4 + dy_mid_mm)};
    b.traces.push_back({0, 0, pp0, pp1, w});
    b.traces.push_back({0, 0, pp1, pp2, w});
    b.traces.push_back({0, 0, pp2, pp3, w});
    b.traces.push_back({1, 0, np0, np1, w});
    b.traces.push_back({1, 0, np1, np2, w});
    b.traces.push_back({1, 0, np2, np3, w});
    DiffPair pr;
    pr.id = 0;
    pr.name = "STRUNK";
    pr.net_p = 0;
    pr.net_n = 1;
    pr.gap_nm = mm_to_nm(0.2);
    pr.gap_tol_nm = mm_to_nm(0.05);
    pr.has_width = true;
    pr.width_nm = mm_to_nm(0.2);
    pr.has_max_skew = true;
    pr.max_skew_nm = mm_to_nm(1.0);
    pr.via_policy = "paired";
    b.diffpairs.push_back(pr);
    return b;
}

}  // namespace

CT_TEST(legal_pair_passes_with_full_metrics) {
    Board routed = route_fixture("diffpair_basic.json");
    VerifyResult vr = verify_board(routed);
    CT_CHECK(vr.ok);
    CT_CHECK(vr.legal && vr.connected);
    CT_CHECK(!has_violation_type(vr, "diffpair_gap"));
    CT_CHECK(!has_violation_type(vr, "diffpair_skew"));
    CT_CHECK(!has_violation_type(vr, "diffpair_via"));
    CT_CHECK(!has_violation_type(vr, "diffpair_one_sided"));
    CT_CHECK(!has_violation_type(vr, "diffpair_layer"));
    CT_CHECK(vr.pairs.size() == 1);
    const PairVerifyDetail* d = find_pair(vr, 0);
    CT_CHECK(d && d->ok && d->status == "OK");
    CT_CHECK(d->length_p_mm > 10.0 && d->length_n_mm > 10.0);
    CT_CHECK_NEAR(d->length_p_mm - d->length_n_mm, d->skew_mm > 0 ? d->skew_mm : 0.0, 1e-9);
    // skew field equals |P-N| in mm.
    CT_CHECK(std::fabs(std::fabs(d->length_p_mm - d->length_n_mm) - d->skew_mm) < 1e-9);
    CT_CHECK(d->skew_mm <= 1.0 + 1e-9);
    // The straight corridor collapses to pad pitch (0.3 edge vs 0.2
    // nominal): both single-segment runs terminate at member pads, so the
    // #26 pad-column fanout exemption floor-checks them while the error
    // stays informational. Trunk copper with no pad context stays strictly
    // banded (see trunk_drift_fails_as_coupled_too_far).
    CT_CHECK_NEAR(d->worst_gap_err_mm, 0.1, 1e-6);
    CT_CHECK(d->has_gap_location);
    CT_CHECK(d->via_mismatch.empty());
    CT_CHECK(d->vias_p == d->vias_n);
}

CT_TEST(excessive_gap_fails_independently) {
    Board routed = route_fixture("diffpair_basic.json");
    // Uncouple the members: every N trace moves to layer 1 while N pads
    // stay on layer 0. No shared-layer coupled section remains, so the gap
    // is effectively unbounded. Lengths/vias are untouched.
    for (auto& t : routed.traces) {
        if (t.net == 1) t.layer = 1;
    }
    VerifyResult vr = verify_board(routed);
    CT_CHECK(!vr.ok && !vr.legal);
    CT_CHECK(has_violation_type(vr, "diffpair_gap"));
    CT_CHECK(violation_detail_has(vr, "diffpair_gap", "uncoupled"));
    const PairVerifyDetail* d = find_pair(vr, 0);
    CT_CHECK(d && !d->ok);
    CT_CHECK(d->status.find("too_far") != std::string::npos);
    CT_CHECK(d->status.find("too_close") == std::string::npos);
    // Independent: no skew/via/one-sided failure rides along. (The disjoint
    // layers also trip the compatible-layer check for the same root cause.)
    CT_CHECK(!has_violation_type(vr, "diffpair_skew"));
    CT_CHECK(!has_violation_type(vr, "diffpair_via"));
    CT_CHECK(!has_violation_type(vr, "diffpair_one_sided"));
}

CT_TEST(preferred_layer_violation_fails) {
    Board routed = route_fixture("diffpair_basic.json");
    // Both members route on layer 0; restricting the pair to layer 1 makes
    // every committed trace off-limits while the gap itself still measures
    // fine (same-layer minimum unchanged).
    routed.diffpairs.front().preferred_layers = {1};
    VerifyResult vr = verify_board(routed);
    CT_CHECK(!vr.ok && !vr.legal);
    CT_CHECK(has_violation_type(vr, "diffpair_layer"));
    const PairVerifyDetail* d = find_pair(vr, 0);
    CT_CHECK(d && !d->ok);
    CT_CHECK(d->status.find("LAYER_MISMATCH") != std::string::npos);
    CT_CHECK(!has_violation_type(vr, "diffpair_skew"));
    CT_CHECK(!has_violation_type(vr, "diffpair_via"));
    CT_CHECK(!has_violation_type(vr, "diffpair_one_sided"));
}

CT_TEST(too_small_gap_fails_independently) {
    Board routed = route_fixture("diffpair_basic.json");
    // Declared gap far above the committed copper: the trunk reads too close.
    routed.diffpairs.front().gap_nm = mm_to_nm(0.6);
    routed.diffpairs.front().gap_tol_nm = mm_to_nm(0.05);
    VerifyResult vr = verify_board(routed);
    CT_CHECK(!vr.ok && !vr.legal);
    CT_CHECK(has_violation_type(vr, "diffpair_gap"));
    CT_CHECK(violation_detail_has(vr, "diffpair_gap", "below"));
    const PairVerifyDetail* d = find_pair(vr, 0);
    CT_CHECK(d && !d->ok);
    CT_CHECK(d->status.find("too_close") != std::string::npos);
    CT_CHECK(d->status.find("too_far") == std::string::npos);
    CT_CHECK(!has_violation_type(vr, "diffpair_skew"));
    CT_CHECK(!has_violation_type(vr, "diffpair_via"));
    CT_CHECK(!has_violation_type(vr, "diffpair_one_sided"));
    CT_CHECK(!has_violation_type(vr, "diffpair_layer"));
}

CT_TEST(excessive_skew_fails_independently) {
    Board routed = route_fixture("diffpair_basic.json");
    VerifyResult base = verify_board(routed);
    const PairVerifyDetail* bd = find_pair(base, 0);
    CT_CHECK(bd && bd->ok);
    // Extra P stub west of the source pad: connectivity preserved (touches
    // the pad), foreign clearance kept (0.25mm edge to N copper), but the
    // P member grows 1.5mm past the 1.0mm skew budget.
    TraceSeg stub;
    stub.net = 0;
    stub.layer = 0;
    stub.a = {mm_to_nm(2.0), mm_to_nm(10.0)};
    stub.b = {mm_to_nm(0.5), mm_to_nm(10.0)};
    stub.width_nm = mm_to_nm(0.2);
    routed.traces.push_back(stub);
    VerifyResult vr = verify_board(routed);
    CT_CHECK(!vr.ok && !vr.legal);
    CT_CHECK(has_violation_type(vr, "diffpair_skew"));
    const PairVerifyDetail* d = find_pair(vr, 0);
    CT_CHECK(d && !d->ok);
    CT_CHECK(d->status.find("SKEW_EXCEEDED") != std::string::npos);
    // Lengths from committed copper: P grew by exactly the stub.
    CT_CHECK_NEAR(d->length_p_mm - bd->length_p_mm, 1.5, 1e-9);
    CT_CHECK_NEAR(d->length_n_mm, bd->length_n_mm, 1e-9);
    CT_CHECK(d->skew_mm > 1.0);
    CT_CHECK(!has_violation_type(vr, "diffpair_gap"));
    CT_CHECK(!has_violation_type(vr, "diffpair_via"));
    CT_CHECK(!has_violation_type(vr, "diffpair_one_sided"));
    CT_CHECK(!has_violation_type(vr, "diffpair_layer"));
}

CT_TEST(mismatched_vias_fail_independently) {
    Board routed = route_fixture("diffpair_via.json");
    int vp = 0, vn = 0;
    const Via* first_n = nullptr;
    for (const auto& v : routed.vias) {
        if (v.net == 0) ++vp;
        if (v.net == 1) {
            if (!first_n) first_n = &v;
            ++vn;
        }
    }
    CT_CHECK(vp > 0 && vp == vn);
    // Duplicate one N barrel in place: connectivity and foreign clearance
    // unchanged (identical site), only the paired count breaks symmetry.
    routed.vias.push_back(*first_n);
    VerifyResult vr = verify_board(routed);
    CT_CHECK(!vr.ok && !vr.legal);
    CT_CHECK(has_violation_type(vr, "diffpair_via"));
    CT_CHECK(violation_detail_has(vr, "diffpair_via", "count:"));
    const PairVerifyDetail* d = find_pair(vr, 0);
    CT_CHECK(d && !d->ok);
    CT_CHECK(d->status.find("VIA_MISMATCH") != std::string::npos);
    CT_CHECK(!d->via_mismatch.empty());
    CT_CHECK(d->vias_p != d->vias_n);
    CT_CHECK(!has_violation_type(vr, "diffpair_gap"));
    CT_CHECK(!has_violation_type(vr, "diffpair_skew"));
    CT_CHECK(!has_violation_type(vr, "diffpair_one_sided"));
    CT_CHECK(!has_violation_type(vr, "diffpair_layer"));
}

CT_TEST(one_member_only_is_hard_failure) {
    Board routed = route_fixture("diffpair_basic.json");
    Board stripped;
    stripped = routed;
    stripped.traces.clear();
    stripped.vias.clear();
    for (const auto& t : routed.traces) {
        if (t.net != 1) stripped.traces.push_back(t);
    }
    for (const auto& v : routed.vias) {
        if (v.net != 1) stripped.vias.push_back(v);
    }
    int left_n = 0;
    for (const auto& t : stripped.traces) {
        if (t.net == 1) ++left_n;
    }
    CT_CHECK(left_n == 0);
    VerifyResult vr = verify_board(stripped);
    CT_CHECK(!vr.ok && !vr.legal);
    CT_CHECK(has_violation_type(vr, "diffpair_one_sided"));
    const PairVerifyDetail* d = find_pair(vr, 0);
    CT_CHECK(d && !d->ok);
    CT_CHECK(d->status.find("ONE_SIDED") != std::string::npos);
    // No gap/via/layer noise: with N gone there is nothing to couple.
    CT_CHECK(!has_violation_type(vr, "diffpair_gap"));
    CT_CHECK(!has_violation_type(vr, "diffpair_via"));
    CT_CHECK(!has_violation_type(vr, "diffpair_layer"));
}

CT_TEST(arbitrary_angle_gap_measured_correctly) {
    // Legal 3-4-5 diagonal: exact 0.2 edge gap, 15mm members, zero skew.
    Board legal = diagonal_pair_board(0.0);
    VerifyResult vr = verify_board(legal);
    CT_CHECK(vr.ok);
    const PairVerifyDetail* d = find_pair(vr, 0);
    CT_CHECK(d && d->ok && d->status == "OK");
    CT_CHECK_NEAR(d->length_p_mm, 15.0, 0.01);
    CT_CHECK_NEAR(d->length_n_mm, 15.0, 0.01);
    CT_CHECK(d->skew_mm < 1e-6);
    CT_CHECK(d->worst_gap_err_mm < 0.005);
    // Copper-level too close: whole N net shifts south, pads follow so both
    // members stay connected pad-to-pad.
    Board close = diagonal_pair_board(-0.15);
    VerifyResult vc = verify_board(close);
    CT_CHECK(!vc.ok);
    CT_CHECK(has_violation_type(vc, "diffpair_gap"));
    CT_CHECK(violation_detail_has(vc, "diffpair_gap", "below"));
    // Copper-level too far as uncoupling: N traces rise to layer 1 while N
    // pads stay on layer 0, so no shared-layer coupled section remains.
    Board far = diagonal_pair_board(0.0);
    for (auto& t : far.traces) {
        if (t.net == 1) t.layer = 1;
    }
    VerifyResult vf = verify_board(far);
    CT_CHECK(!vf.ok);
    CT_CHECK(has_violation_type(vf, "diffpair_gap"));
    CT_CHECK(violation_detail_has(vf, "diffpair_gap", "uncoupled"));
}

CT_TEST(pair_constraints_hold_after_cleanup_and_json) {
    Board b = load_fixture_board("diffpair_basic.json");
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = 1;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.verification.ok);
    CT_CHECK(rep.diffpairs.size() == 1);
    const PairReport& pr = rep.diffpairs.front();
    CT_CHECK(pr.materialized);
    CT_CHECK(pr.status == "MATERIALIZED");
    // Route JSON carries verifier-measured values (issue #13 schema).
    JsonValue j = rep.to_json();
    const JsonValue* dp = j.find("diffpairs");
    CT_CHECK(dp && dp->is_array() && dp->as_array().size() == 1);
    const JsonValue& o = dp->as_array()[0];
    CT_CHECK(o.has("length_p_mm") && o.has("length_n_mm"));
    CT_CHECK(o.has("skew_mm") && o.has("worst_gap_err_mm"));
    CT_CHECK(o.has("gap_x_mm") && o.has("gap_y_mm") && o.has("gap_layer"));
    CT_CHECK(o.has("via_mismatch") && o.has("status"));
    CT_CHECK(o.get_string("status") == "MATERIALIZED");
    CT_CHECK(o.get_string("via_mismatch").empty());
    // Independent re-verification of the committed copper agrees exactly.
    VerifyResult vr = verify_board(engine.committed());
    CT_CHECK(vr.ok);
    const PairVerifyDetail* d = find_pair(vr, 0);
    CT_CHECK(d && d->ok);
    CT_CHECK_NEAR(pr.length_p_mm, d->length_p_mm, 1e-9);
    CT_CHECK_NEAR(pr.length_n_mm, d->length_n_mm, 1e-9);
    CT_CHECK_NEAR(pr.skew_mm, d->skew_mm, 1e-9);
    CT_CHECK_NEAR(pr.worst_gap_err_mm, d->worst_gap_err_mm, 1e-9);
    // Verify JSON mirrors the same pair schema.
    JsonValue vj = vr.to_json();
    const JsonValue* vpj = vj.find("diffpairs");
    CT_CHECK(vpj && vpj->is_array() && vpj->as_array().size() == 1);
    CT_CHECK(vpj->as_array()[0].has("length_p_mm"));
    CT_CHECK(vpj->as_array()[0].has("skew_mm"));
    CT_CHECK(vpj->as_array()[0].has("via_mismatch"));
    CT_CHECK(vpj->as_array()[0].has("status"));
}

CT_TEST(trunk_drift_fails_as_coupled_too_far) {
    // Issue #26: coupled-but-drifting is a gap violation, not the uncoupled
    // path. Trunk edge 0.26 (> 0.25) fails with an "above" detail and a
    // trunk location; the uncoupled wording must not appear and no
    // too_close/skew/via/one-sided failure rides along.
    Board b = straight_trunk_board(0.06);
    VerifyResult vr = verify_board(b);
    CT_CHECK(!vr.ok && !vr.legal);
    CT_CHECK(has_violation_type(vr, "diffpair_gap"));
    CT_CHECK(violation_detail_has(vr, "diffpair_gap", "above"));
    CT_CHECK(!violation_detail_has(vr, "diffpair_gap", "uncoupled"));
    CT_CHECK(!violation_detail_has(vr, "diffpair_gap", "below"));
    const PairVerifyDetail* d = find_pair(vr, 0);
    CT_CHECK(d && !d->ok);
    CT_CHECK(d->status.find("too_far") != std::string::npos);
    CT_CHECK(d->status.find("too_close") == std::string::npos);
    CT_CHECK(d->status.find("uncoupled") == std::string::npos);
    CT_CHECK(d->has_gap_location);
    CT_CHECK(d->gap_layer == 0);
    // Unique trunk nearest: P middle (9..11, 10) fully overlapped by the N
    // middle (8..12, 10.46); end corners sit >1mm away, so no tiebreak is
    // involved: midpoint of the middles = (10, 10.23).
    CT_CHECK_NEAR(d->gap_x_mm, 10.0, 1e-9);
    CT_CHECK_NEAR(d->gap_y_mm, 10.23, 1e-9);
    CT_CHECK_NEAR(d->worst_gap_err_mm, 0.06, 1e-9);
    CT_CHECK(!has_violation_type(vr, "diffpair_skew"));
    CT_CHECK(!has_violation_type(vr, "diffpair_via"));
    CT_CHECK(!has_violation_type(vr, "diffpair_one_sided"));
    CT_CHECK(!has_violation_type(vr, "diffpair_layer"));
}

CT_TEST(trunk_band_edges_pass) {
    // Nominal 0.20 and upper-bound 0.25 trunk edges pass cleanly.
    for (double dy : {0.0, 0.05}) {
        Board b = straight_trunk_board(dy);
        VerifyResult vr = verify_board(b);
        CT_CHECK(vr.ok);
        CT_CHECK(!has_violation_type(vr, "diffpair_gap"));
        const PairVerifyDetail* d = find_pair(vr, 0);
        CT_CHECK(d && d->ok && d->status == "OK");
    }
}

CT_TEST(pad_column_fanout_exempt_from_ceiling) {
    // Issue #26 exemption fixture: single pad-to-pad segments at pad pitch
    // (0.3 edge vs 0.2 nominal, above gap+tol) still verify OK because both
    // segments terminate at member pads -- the #12 pad-column stitching
    // envelope, floor-checked only. The same bare copper with no pad
    // context stays strictly banded (pair_gap_legal rejects it), pinning
    // the exemption boundary: pads in scope = fanout, bare trunk = strict.
    Board b = base_2layer(20.0, 20.0);
    b.defaults.trace_width_nm = mm_to_nm(0.2);
    b.defaults.clearance_nm = mm_to_nm(0.15);
    b.defaults.default_current_a = 0.5;
    NetInfo p = make_net(0, "P");
    p.current_a = 0.1;
    p.has_current = true;
    p.voltage_v = 3.3;
    p.has_voltage = true;
    NetInfo n = make_net(1, "N");
    n.current_a = 0.1;
    n.has_current = true;
    n.voltage_v = 3.3;
    n.has_voltage = true;
    b.nets.push_back(p);
    b.nets.push_back(n);
    add_terminal(b, 0, 2.0, 10.0, 0, 0.3, "J1", "1");
    add_terminal(b, 0, 18.0, 10.0, 0, 0.3, "J1", "2");
    add_terminal(b, 1, 2.0, 10.5, 0, 0.3, "J1", "3");
    add_terminal(b, 1, 18.0, 10.5, 0, 0.3, "J1", "4");
    Coord w = mm_to_nm(0.2);
    b.traces.push_back(
        {0, 0, {mm_to_nm(2.0), mm_to_nm(10.0)}, {mm_to_nm(18.0), mm_to_nm(10.0)}, w});
    b.traces.push_back(
        {1, 0, {mm_to_nm(2.0), mm_to_nm(10.5)}, {mm_to_nm(18.0), mm_to_nm(10.5)}, w});
    DiffPair pr;
    pr.id = 0;
    pr.name = "FAN";
    pr.net_p = 0;
    pr.net_n = 1;
    pr.gap_nm = mm_to_nm(0.2);
    pr.gap_tol_nm = mm_to_nm(0.05);
    pr.has_width = true;
    pr.width_nm = mm_to_nm(0.2);
    pr.via_policy = "paired";
    b.diffpairs.push_back(pr);
    VerifyResult vr = verify_board(b);
    CT_CHECK(vr.ok);
    CT_CHECK(!has_violation_type(vr, "diffpair_gap"));
    const PairVerifyDetail* d = find_pair(vr, 0);
    CT_CHECK(d && d->ok && d->status == "OK");
    CT_CHECK_NEAR(d->worst_gap_err_mm, 0.1, 1e-9);
    // Same copper, no pad context: strictly banded, rejects.
    std::vector<TraceSeg> tp, tn;
    for (const auto& t : b.traces) {
        if (t.net == 0) tp.push_back(t);
        if (t.net == 1) tn.push_back(t);
    }
    Coord worst = 0;
    Point at{0, 0};
    CT_CHECK(!pair_gap_legal(tp, tn, pr.gap_nm, pr.gap_tol_nm, worst, at));
}

int main() { return copperline::test::run_all_tests(); }
