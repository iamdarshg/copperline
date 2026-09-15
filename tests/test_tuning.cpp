// Issue #15: post-route meander/length-tuning stage.
//
// Covers: dedicated LengthTuner after closure + pair materialization,
// single-ended target met inside a legal window, mismatched pair brought
// within skew, insufficient-space infeasible without topology change,
// never runs before closure, tuning_exempt survives #17 simplification,
// verifier-clean output and thread-count determinism.
#include <algorithm>
#include <cmath>
#include <string>

#include "helpers.h"

#include "router/board.h"
#include "router/diffpair.h"
#include "router/engine.h"
#include "router/simplify.h"
#include "router/tuning.h"
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

RouteReport route_board(Board b, int threads) {
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = threads;
    RouterEngine engine(std::move(b), std::move(r), opt);
    return engine.run();
}

VerifyResult verify_board(const Board& board) {
    RuleResolver r = RuleResolver::defaults_for(board);
    BoardVerifier v;
    return v.verify(board, r, r.defaultContext());
}

Coord net_len(const Board& b, NetId net) {
    Coord total = 0;
    for (const auto& t : b.traces)
        if (t.net == net) total += euclid_len_nm(t.a, t.b);
    return total;
}

int count_traces(const Board& b, NetId net) {
    int n = 0;
    for (const auto& t : b.traces)
        if (t.net == net) ++n;
    return n;
}

std::string board_serial(const Board& b) { return serialize_json(board_to_json(b)); }

// Copper-geometry serial: terminal ids come from a process-wide counter in
// helpers.h, so two identically built boards differ there; compare only the
// committed traces/vias that tuning may change.
std::string copper_serial(const Board& b) {
    struct Key {
        NetId net;
        LayerId layer;
        Coord ax, ay, bx, by, w;
        bool operator<(const Key& o) const {
            if (net != o.net) return net < o.net;
            if (layer != o.layer) return layer < o.layer;
            if (ax != o.ax) return ax < o.ax;
            if (ay != o.ay) return ay < o.ay;
            if (bx != o.bx) return bx < o.bx;
            if (by != o.by) return by < o.by;
            return w < o.w;
        }
    };
    std::vector<Key> keys;
    for (const auto& t : b.traces)
        keys.push_back({t.net, t.layer, t.a.x, t.a.y, t.b.x, t.b.y, t.width_nm});
    std::sort(keys.begin(), keys.end());
    std::string s;
    for (const auto& k : keys) {
        s += std::to_string(k.net) + "," + std::to_string(k.layer) + "," +
             std::to_string(k.ax) + "," + std::to_string(k.ay) + "," +
             std::to_string(k.bx) + "," + std::to_string(k.by) + "," +
             std::to_string(k.w) + ";";
    }
    s += "vias:" + std::to_string(b.vias.size());
    return s;
}

const TuningRecord* find_record(const TuningSummary& s, const std::string& kind, int pair_id,
                                NetId net) {
    for (const auto& r : s.records) {
        if (r.kind != kind) continue;
        if (kind == "pair" && r.pair_id == pair_id) return &r;
        if (kind == "single" && r.net == net) return &r;
    }
    return nullptr;
}

// Hand-built committed pair with 4 mm skew: P is 10 mm, N is 6 mm, both on
// layer 0 with a nominal 0.3 mm edge gap (0.5 mm centerline). Mimics
// post-materialization copper so the LengthTuner can be exercised directly.
Board skewed_committed_pair() {
    Board b = base_2layer(20.0, 20.0);
    b.defaults.trace_width_nm = mm_to_nm(0.2);
    b.defaults.clearance_nm = mm_to_nm(0.15);
    b.defaults.via_outer_nm = mm_to_nm(0.6);
    b.defaults.via_hole_nm = mm_to_nm(0.3);
    b.defaults.default_current_a = 0.5;
    NetInfo p = make_net(0, "P");
    p.has_current = true;
    p.current_a = 0.1;
    p.has_voltage = true;
    p.voltage_v = 3.3;
    NetInfo n = make_net(1, "N");
    n.has_current = true;
    n.current_a = 0.1;
    n.has_voltage = true;
    n.voltage_v = 3.3;
    b.nets.push_back(p);
    b.nets.push_back(n);
    TermId p0 = add_terminal(b, 0, 2.0, 10.0, 0, 0.3, "J1", "1");
    TermId p1 = add_terminal(b, 0, 12.0, 10.0, 0, 0.3, "J1", "2");
    TermId n0 = add_terminal(b, 1, 2.0, 10.5, 0, 0.3, "J1", "3");
    TermId n1 = add_terminal(b, 1, 8.0, 10.5, 0, 0.3, "J1", "4");
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
    pr.name = "SKEWED";
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

CT_TEST(length_intent_parses_and_round_trips) {
    Board b = load_fixture_board("tuning_single.json");
    const NetInfo* sig = b.find_net_by_name("SIG");
    CT_CHECK(sig && sig->has_target_length);
    CT_CHECK(sig->target_length_nm == mm_to_nm(20.0));
    CT_CHECK(sig->length_tol_nm == mm_to_nm(1.5));
    const NetInfo* gnd = b.find_net_by_name("GND");
    CT_CHECK(gnd && !gnd->has_target_length);
    JsonValue back = board_to_json(b);
    const JsonValue* nets = back.find("nets");
    CT_CHECK(nets && nets->is_array());
    bool saw_target = false;
    for (const auto& n : nets->as_array()) {
        if (n.get_string("name") == "SIG") {
            CT_CHECK(n.get_number("target_length_mm", 0) > 19.9);
            CT_CHECK(n.get_number("target_length_mm", 0) < 20.1);
            saw_target = true;
        }
    }
    CT_CHECK(saw_target);
    // Symmetric pair mode round-trips.
    Board pb = load_fixture_board("tuning_pair.json");
    CT_CHECK(pb.diffpairs.size() == 1);
    CT_CHECK(pb.diffpairs.front().max_skew_nm == mm_to_nm(0.5));
}

CT_TEST(tuning_config_parses_and_rejects_bad_style) {
    TuningConfig cfg;
    std::string err;
    CT_CHECK(tuning_config_from_json(parse_json("{}"), cfg, err));
    CT_CHECK(cfg.enabled && cfg.style == "trombone");
    CT_CHECK(cfg.amplitude_nm == mm_to_nm(1.0));
    JsonValue good = parse_json(
        "{\"tuning\": {\"amplitude_mm\": 0.8, \"pitch_mm\": 0.5, "
        "\"max_added_mm\": 10, \"max_candidates\": 8, \"symmetric_pairs\": true}}");
    CT_CHECK(tuning_config_from_json(good, cfg, err));
    CT_CHECK(cfg.amplitude_nm == mm_to_nm(0.8));
    CT_CHECK(cfg.pitch_nm == mm_to_nm(0.5));
    CT_CHECK(cfg.max_added_nm == mm_to_nm(10.0));
    CT_CHECK(cfg.max_candidates == 8);
    CT_CHECK(cfg.symmetric_pairs);
    JsonValue bad = parse_json("{\"tuning\": {\"style\": \"arc\"}}");
    CT_CHECK(!tuning_config_from_json(bad, cfg, err));
    JsonValue bad2 = parse_json("{\"tuning\": {\"amplitude_mm\": -1}}");
    CT_CHECK(!tuning_config_from_json(bad2, cfg, err));
}

CT_TEST(single_ended_target_met_in_window) {
    RouteReport rep = route_board(load_fixture_board("tuning_single.json"), 1);
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.verification.ok);
    CT_CHECK(rep.tuning.stage == "TUNING_COMPLETE");
    const TuningRecord* rec = find_record(rep.tuning, "single", -1, 0);
    CT_CHECK(rec && rec->status == "TUNED");
    CT_CHECK(rec->tuning_exempt);
    CT_CHECK(rec->added_nm > mm_to_nm(2.4) && rec->added_nm < mm_to_nm(5.6));
    CT_CHECK(rep.verification.ok);
}

CT_TEST(single_ended_final_length_inside_window) {
    Board b = load_fixture_board("tuning_single.json");
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = 1;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    const Board& tuned = engine.committed();
    double len_mm = nm_to_mm(net_len(tuned, 0));
    CT_CHECK(len_mm >= 18.5 - 1e-9 && len_mm <= 21.5 + 1e-9);
    // Unrelated net untouched: GND stays a single straight segment.
    CT_CHECK(count_traces(tuned, 1) == 1);
    CT_CHECK(net_len(tuned, 1) == mm_to_nm(16.0));
    VerifyResult vr = verify_board(tuned);
    CT_CHECK(vr.ok);
}

CT_TEST(mismatched_pair_tuned_within_skew) {
    Board b = skewed_committed_pair();
    // Precondition: 4 mm skew exceeds the 1 mm max (and nothing else fails).
    {
        VerifyResult pre = verify_board(b);
        CT_CHECK(!pre.ok);
        bool saw_skew = false;
        for (const auto& v : pre.violations) {
            if (v.type == "diffpair_skew") saw_skew = true;
            else CT_CHECK(v.type == "diffpair_skew");
        }
        CT_CHECK(saw_skew);
    }
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx = r.defaultContext();
    TuningConfig cfg;  // defaults: 1.0 mm amplitude, 0.6 mm pitch
    LengthTuner tuner(&b, &r, &ctx, cfg);
    TuningSummary sum = tuner.run(true);
    CT_CHECK(sum.stage == "TUNING_COMPLETE");
    CT_CHECK(sum.records.size() == 1);
    CT_CHECK(sum.records[0].kind == "pair");
    CT_CHECK(sum.records[0].status == "TUNED");
    CT_CHECK(sum.records[0].tuning_exempt);
    // Skew from committed copper is within max; N grew, P untouched.
    std::vector<TraceSeg> tp, tn;
    for (const auto& t : b.traces) {
        if (t.net == 0) tp.push_back(t);
        if (t.net == 1) tn.push_back(t);
    }
    Coord lp = pair_total_length(tp), ln = pair_total_length(tn);
    Coord skew = lp >= ln ? lp - ln : ln - lp;
    CT_CHECK(skew <= mm_to_nm(1.0));
    CT_CHECK(lp == mm_to_nm(10.0));
    CT_CHECK(ln >= mm_to_nm(9.0) && ln <= mm_to_nm(11.0));
    // Paired-via policy preserved: no vias introduced.
    CT_CHECK(b.vias.empty());
    VerifyResult vr = verify_board(b);
    CT_CHECK(vr.ok);
    // Determinism: identical input tunes to identical copper.
    Board b2 = skewed_committed_pair();
    RuleResolver r2 = RuleResolver::defaults_for(b2);
    ElectricalContext ctx2 = r2.defaultContext();
    LengthTuner tuner2(&b2, &r2, &ctx2, cfg);
    TuningSummary sum2 = tuner2.run(true);
    CT_CHECK(sum2.records[0].status == "TUNED");
    CT_CHECK(copper_serial(b) == copper_serial(b2));
}

CT_TEST(pair_pipeline_materializes_then_tunes) {
    RouteReport rep = route_board(load_fixture_board("tuning_pair.json"), 1);
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.verification.ok);
    CT_CHECK(!rep.diffpairs.empty());
    CT_CHECK(rep.diffpairs[0].status == "MATERIALIZED");
    // Tuning ran after materialization (not skipped for closure).
    CT_CHECK(rep.tuning.stage == "TUNING_COMPLETE");
    const TuningRecord* rec = find_record(rep.tuning, "pair", 0, -1);
    CT_CHECK(rec);
    CT_CHECK(rec->status == "TUNED" || rec->status == "ALREADY_WITHIN_WINDOW");
    CT_CHECK(rec->skew_after_mm <= 0.5 + 1e-9);
    CT_CHECK(rep.verification.pairs.size() == 1);
    CT_CHECK(rep.verification.pairs[0].ok);
}

CT_TEST(insufficient_space_reports_infeasible_untouched) {
    Board b = load_fixture_board("tuning_blocked.json");
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = 1;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    // Single-ended shortfall is a soft diagnostic: routing still COMPLETE
    // and verifier-clean, but tuning reports infeasible explicitly.
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.verification.ok);
    const TuningRecord* rec = find_record(rep.tuning, "single", -1, 0);
    CT_CHECK(rec);
    CT_CHECK(rec->status.rfind("INFEASIBLE", 0) == 0);
    CT_CHECK(rec->added_nm == 0);
    // No topology change: SIG stays one straight 16 mm segment, GND intact.
    const Board& tuned = engine.committed();
    CT_CHECK(net_len(tuned, 0) == mm_to_nm(16.0));
    CT_CHECK(count_traces(tuned, 0) == 1);
    CT_CHECK(count_traces(tuned, 1) == 1);
    CT_CHECK(net_len(tuned, 1) == mm_to_nm(16.0));
}

CT_TEST(tuning_never_runs_before_closure) {
    RouteReport rep = route_board(load_fixture_board("tuning_unclosed.json"), 1);
    CT_CHECK(rep.status != "COMPLETE");
    CT_CHECK(rep.tuning.stage == "SKIPPED_NOT_CLOSED");
    CT_CHECK(rep.tuning.records.empty());
    // LengthTuner gate in isolation also touches nothing when open.
    Board b = load_fixture_board("tuning_single.json");
    std::string before = board_serial(b);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx = r.defaultContext();
    LengthTuner tuner(&b, &r, &ctx, TuningConfig{});
    TuningSummary sum = tuner.run(false);
    CT_CHECK(sum.stage == "SKIPPED_NOT_CLOSED");
    CT_CHECK(board_serial(b) == before);
}

CT_TEST(tuning_exempt_survives_simplification) {
    Board b = load_fixture_board("tuning_single.json");
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = 1;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    const Board& tuned = engine.committed();
    // The tuned net owns meander bends now.
    CT_CHECK(count_traces(tuned, 0) > 1);
    // #17 simplifier with the tuning_exempt flag leaves them verbatim.
    std::vector<TraceSeg> traces = tuned.traces;
    RuleResolver r2 = RuleResolver::defaults_for(tuned);
    ElectricalContext ctx2 = r2.defaultContext();
    std::vector<char> no_stub(traces.size(), 0);
    SimplifyStats st = simplify_candidate_traces(tuned, r2, ctx2, 0, traces, no_stub,
                                                 /*tuning_exempt=*/true);
    CT_CHECK(st.skipped_exempt);
    CT_CHECK(traces.size() == tuned.traces.size());
    ConnectionTask exempt_task;
    exempt_task.tuning_exempt = true;
    CT_CHECK(simplify_exempt_task(exempt_task));
    ConnectionTask plain_task;
    CT_CHECK(!simplify_exempt_task(plain_task));
}

CT_TEST(tuning_deterministic_across_thread_counts) {
    std::string h0;
    for (int threads : {1, 4}) {
        RouteReport rep = route_board(load_fixture_board("tuning_single.json"), threads);
        CT_CHECK(rep.status == "COMPLETE");
        CT_CHECK(rep.verification.ok);
        if (h0.empty()) {
            h0 = rep.board_hash;
        } else {
            CT_CHECK(rep.board_hash == h0);
        }
    }
}

CT_TEST(pair_tuning_teeth_marked_and_ceiling_exempt) {
    // Issue #26 interplay with #15: trombone teeth are specified skew jogs
    // that legitimately leave the gap band, so pair tuning marks them and
    // the verifier floor-checks (never ceiling-checks) them. The skewed
    // fixture tunes cleanly, the accordion carries the mark, re-verification
    // passes, and native JSON round-trips the annotation.
    Board b = skewed_committed_pair();
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx = r.defaultContext();
    TuningConfig cfg;
    LengthTuner tuner(&b, &r, &ctx, cfg);
    TuningSummary sum = tuner.run(true);
    CT_CHECK(sum.stage == "TUNING_COMPLETE");
    int teeth = 0;
    for (const auto& t : b.traces) {
        if (t.net == 1) {
            if (t.tuning_tooth) ++teeth;
        } else {
            CT_CHECK(!t.tuning_tooth);  // P untouched and unmarked
        }
    }
    CT_CHECK(teeth > 0);
    CT_CHECK(verify_board(b).ok);
    // The mark is load-bearing, not vacuous: the same copper with marks
    // stripped fails the strict bare check (teeth genuinely leave the band
    // while the verifier accepts the marked accordion).
    {
        std::vector<TraceSeg> tp, tn;
        for (const auto& t : b.traces) {
            TraceSeg c = t;
            c.tuning_tooth = false;
            if (c.net == 0) tp.push_back(c);
            if (c.net == 1) tn.push_back(c);
        }
        Coord worst = 0;
        Point at{0, 0};
        CT_CHECK(!pair_gap_legal(tp, tn, mm_to_nm(0.2), mm_to_nm(0.05), worst, at));
    }
    // Native JSON round-trip preserves the mark and still verifies.
    JsonBoardImporter imp;
    ImportResult rt = imp.import_value(board_to_json(b), "roundtrip");
    int rt_teeth = 0;
    for (const auto& t : rt.board.traces) {
        if (t.tuning_tooth) ++rt_teeth;
    }
    CT_CHECK(rt_teeth == teeth);
    CT_CHECK(verify_board(rt.board).ok);
}

int main() { return copperline::test::run_all_tests(); }
