// Prompt 5 golden suite: end-to-end fixtures, determinism, optimizer,
// DSN/SES adapters and sidecar nets. Routable goldens must reach 100%
// connectivity with an independent verifier pass; the impossible fixture
// must NOT falsely succeed.
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "helpers.h"
#include "router/board.h"
#include "router/dsn.h"
#include "router/engine.h"
#include "router/optimizer.h"
#include "router/verifier.h"

using namespace copperline;

#ifndef FIXTURE_DIR
#define FIXTURE_DIR "fixtures"
#endif

namespace {

std::string fixture(const std::string& name) { return std::string(FIXTURE_DIR) + "/" + name; }

struct GoldenCase {
    std::string file;
    std::string config;  // "" = defaults
    bool threads_sweep = false;
};

RouteReport run_fixture(const std::string& file, const std::string& config, int threads,
                        unsigned seed = 42) {
    ImportResult ir = import_board_auto(fixture(file));
    Board b = std::move(ir.board);
    RuleResolver r = RuleResolver::defaults_for(b);
    if (!config.empty()) {
        std::string cfg_text;
        {
            // Reuse the importer path for configs: plain file read.
            std::ifstream f(fixture(config), std::ios::binary);
            if (!f) throw std::runtime_error("cannot open config " + config);
            std::ostringstream ss;
            ss << f.rdbuf();
            cfg_text = ss.str();
        }
        JsonValue cfg = parse_json(cfg_text);
        apply_sidecar_nets(b, cfg);
        r = RuleResolver::from_config(b, cfg);
    }
    EngineOptions opt;
    opt.seed = seed;
    opt.threads = threads;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    // Independent gate (mirrors the CLI): never trust bookkeeping alone.
    RuleResolver vr = RuleResolver::defaults_for(engine.committed());
    if (!config.empty()) {
        std::ifstream f(fixture(config), std::ios::binary);
        std::ostringstream ss;
        ss << f.rdbuf();
        JsonValue cfg = parse_json(ss.str());
        apply_sidecar_nets(const_cast<Board&>(engine.committed()), cfg);
        vr = RuleResolver::from_config(engine.committed(), cfg);
    }
    BoardVerifier verifier;
    VerifyResult independent =
        verifier.verify(engine.committed(), vr, vr.defaultContext());
    apply_verifier_gate(rep, independent);
    return rep;
}

void check_complete(const RouteReport& rep, const std::string& name) {
    if (rep.status != "COMPLETE")
        throw std::runtime_error(name + ": status=" + rep.status + " category=" +
                                 rep.result_category);
    if (!rep.verification.ok)
        throw std::runtime_error(name + ": verifier not ok after COMPLETE");
    if (rep.connected_terminals != rep.total_terminals)
        throw std::runtime_error(name + ": partial connectivity marked COMPLETE");
    if (!rep.optimizer.ran)
        throw std::runtime_error(name + ": optimizer did not run after COMPLETE (" +
                                 rep.optimizer.gate_reason + ")");
}

}  // namespace

// Every routable golden fixture: 100% connectivity + verifier pass.
CT_TEST(golden_routable_suite) {
    const std::vector<GoldenCase> goldens = {
        {"open_2layer.json", ""},
        {"obstacle_detour.json", ""},
        {"high_current.json", "rules_demo.json"},
        {"voltage_clearance.json", ""},
        {"narrow_channel.json", ""},
        {"bga_4x4.json", ""},
        {"multi_terminal_tree.json", ""},
        {"forced_ripup.json", ""},
        {"dense_mcu_4layer.json", "rules_sidecar_nets.json"},
        {"golden_demo.dsn", ""},
    };
    for (const auto& g : goldens) {
        RouteReport rep = run_fixture(g.file, g.config, 2);
        check_complete(rep, g.file);
    }
}

// The intentionally impossible fixture must NOT falsely report success.
CT_TEST(golden_impossible_must_not_succeed) {
    RouteReport rep = run_fixture("blocked_impossible.json", "", 2);
    CT_CHECK(rep.status != "COMPLETE");
    CT_CHECK(!rep.failures.empty());
    CT_CHECK(!rep.result_category.empty());
    // ... and explain-failure inputs exist for every failure.
    for (const auto& f : rep.failures) {
        CT_CHECK(!f.net_name.empty());
        CT_CHECK(!f.category.empty());
        CT_CHECK(f.candidate_count >= 0);
    }
}

// No false success on dense fine-pitch boards either: whenever the engine
// claims COMPLETE the independent verifier must agree (gate invariant).
CT_TEST(golden_no_false_success_dense) {
    for (const char* fx : {"irregular_array.json", "dense_qfn.json"}) {
        RouteReport rep = run_fixture(fx, "", 1);
        if (rep.status == "COMPLETE") CT_CHECK(rep.verification.ok);
    }
}

// Determinism: identical (board, rules, workers, seed) -> identical hashes;
// worker timing never affects output (threads 1 vs 4 identical copper).
CT_TEST(golden_determinism) {
    RouteReport t1 = run_fixture("obstacle_detour.json", "", 1, 7);
    RouteReport t4 = run_fixture("obstacle_detour.json", "", 4, 7);
    CT_CHECK(t1.status == "COMPLETE");
    CT_CHECK(t4.status == "COMPLETE");
    CT_CHECK(t1.board_hash == t4.board_hash);
    RouteReport again = run_fixture("obstacle_detour.json", "", 4, 7);
    CT_CHECK(again.board_hash == t4.board_hash);
    // DSN board determinism across thread counts too.
    RouteReport d1 = run_fixture("golden_demo.dsn", "", 1);
    RouteReport d4 = run_fixture("golden_demo.dsn", "", 4);
    CT_CHECK(d1.board_hash == d4.board_hash);
}

// Optimizer is transactional: never harms connectivity/legality, never
// grows length, always re-verifies.
CT_TEST(optimizer_transactional) {
    ImportResult ir = import_board_auto(fixture("multi_terminal_tree.json"));
    Board b = std::move(ir.board);
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = 1;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.optimizer.ran);
    CT_CHECK(rep.optimizer.length_after_mm <= rep.optimizer.length_before_mm + 1e-9);
    CT_CHECK(rep.optimizer.vias_after <= rep.optimizer.vias_before);
    CT_CHECK(rep.optimizer.bends_after <= rep.optimizer.bends_before);
    CT_CHECK(rep.verification.ok);
    // Disabled optimizer leaves copper untouched but reports honestly.
    ImportResult ir2 = import_board_auto(fixture("open_2layer.json"));
    Board b2 = std::move(ir2.board);
    RuleResolver r2 = RuleResolver::defaults_for(b2);
    EngineOptions opt2;
    opt2.threads = 1;
    opt2.optimizer.enabled = false;
    RouterEngine engine2(std::move(b2), std::move(r2), opt2);
    RouteReport rep2 = engine2.run();
    CT_CHECK(rep2.status == "COMPLETE");
    CT_CHECK(!rep2.optimizer.ran);
}

// Issue #37: Pass 1/2 restarted the deterministic scan unconditionally after
// attempt(...), even when the mutation was reverted. The rejected first
// candidate retried until the guard expired, burning the max_candidates
// budget while later legal candidates starved. Only accepted mutations
// (rep.applied advance, as the via pass already does) may restart the scan.
CT_TEST(optimizer_rejected_candidate_does_not_starve_later_ones) {
    Board b = test::base_2layer();
    b.nets.push_back(test::make_net(0, "N0"));
    b.nets.push_back(test::make_net(1, "N1"));
    test::add_terminal(b, 0, 1.0, 5.0);
    test::add_terminal(b, 0, 3.0, 7.0);
    test::add_terminal(b, 1, 10.0, 5.0);
    test::add_terminal(b, 1, 12.0, 7.0);
    const Coord w = mm_to_nm(0.2);
    // Net 0 corner first in scan order; its diagonal shortcut crosses the
    // keepout below, so the verifier must reject it.
    b.traces.push_back({0, 0, {mm_to_nm(1.0), mm_to_nm(5.0)}, {mm_to_nm(3.0), mm_to_nm(5.0)}, w});
    b.traces.push_back({0, 0, {mm_to_nm(3.0), mm_to_nm(5.0)}, {mm_to_nm(3.0), mm_to_nm(7.0)}, w});
    // Net 1 corner second; its shortcut is in clear copper and must apply.
    b.traces.push_back({1, 0, {mm_to_nm(10.0), mm_to_nm(5.0)}, {mm_to_nm(12.0), mm_to_nm(5.0)}, w});
    b.traces.push_back({1, 0, {mm_to_nm(12.0), mm_to_nm(5.0)}, {mm_to_nm(12.0), mm_to_nm(7.0)}, w});
    Keepout ko;
    ko.rect = {mm_to_nm(1.8), mm_to_nm(5.8), mm_to_nm(2.2), mm_to_nm(6.2)};
    ko.layer = 0;
    ko.reason = "issue-37-blocker";
    b.keepouts.push_back(ko);

    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    BoardVerifier v;
    CT_CHECK(v.verify(b, r, ctx).ok);  // gate precondition: legal input

    OptimizerOptions opt;
    opt.collinear_merge = false;
    opt.bend_removal = true;
    opt.via_elimination = false;
    opt.preferred_layer = false;
    CleanupOptimizer oz(&b, &r, ctx, opt);
    OptimizerReport rep = oz.run();
    CT_CHECK(rep.ran);
    // The legal second corner applied exactly once...
    CT_CHECK(rep.applied == 1);
    // ...without retrying the rejected first corner until guard expiry
    // (buggy code burned 8 candidates / 8 reverts and applied nothing).
    CT_CHECK(rep.candidates <= 4);
    CT_CHECK(rep.reverted <= 2);
    CT_CHECK(rep.bends_after == rep.bends_before - 1);
    CT_CHECK(rep.length_after_mm < rep.length_before_mm);
    // Geometry: net 1 is now a direct shortcut, net 0's L is untouched.
    int n0_segs = 0, n1_segs = 0;
    bool n1_shortcut = false;
    for (const auto& s : b.traces) {
        if (s.net == 0) ++n0_segs;
        if (s.net == 1) {
            ++n1_segs;
            if (s.a == Point{mm_to_nm(10.0), mm_to_nm(5.0)} &&
                s.b == Point{mm_to_nm(12.0), mm_to_nm(7.0)})
                n1_shortcut = true;
        }
    }
    CT_CHECK(n0_segs == 2);
    CT_CHECK(n1_segs == 1);
    CT_CHECK(n1_shortcut);
    CT_CHECK(v.verify(b, r, ctx).ok);
}

// DSN import preserves net names/pads/layers/classes/widths; SES export
// round-trips copper back through verify.
CT_TEST(adapter_dsn_ses_roundtrip) {
    DsnImporter dsn;
    ImportResult ir = dsn.import_file(fixture("golden_demo.dsn"));
    CT_CHECK(ir.board.nets.size() == 2);
    CT_CHECK(ir.board.find_net_by_name("SIG1") != nullptr);
    CT_CHECK(ir.board.find_net_by_name("VBAT") != nullptr);
    const NetInfo* vbat = ir.board.find_net_by_name("VBAT");
    CT_CHECK(vbat->has_min_width);  // PWR class width preserved
    CT_CHECK(!vbat->via_class.empty());  // use_via preserved
    CT_CHECK(ir.board.terminals.size() == 4);
    for (const auto& t : ir.board.terminals) {
        CT_CHECK(!t.component.empty());
        CT_CHECK(!t.pin.empty());
    }
    // Route, export SES, re-import copper into a fresh board, verify.
    Board b = std::move(ir.board);
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = 1;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    std::string ses = board_to_ses(engine.committed(), "golden_demo");
    CT_CHECK(ses.find("(session") != std::string::npos);
    CT_CHECK(ses.find("SIG1") != std::string::npos);
    CT_CHECK(ses.find("VBAT") != std::string::npos);
    ImportResult fresh = dsn.import_file(fixture("golden_demo.dsn"));
    std::vector<std::string> warns =
        ses_merge_into(fresh.board, ses, "roundtrip.ses");
    (void)warns;
    RuleResolver vr = RuleResolver::defaults_for(fresh.board);
    BoardVerifier verifier;
    VerifyResult res = verifier.verify(fresh.board, vr, vr.defaultContext());
    CT_CHECK(res.ok);
}

// Copper pours are never silently dropped: unresolvable copper warns.
CT_TEST(adapter_pour_warning) {
    const std::string text =
        "(pcb \"w\" (unit mm)\n"
        "  (structure (layer Top (type signal)) (layer Bottom (type signal))\n"
        "    (boundary (rect 0 0 10 10)) (copper_pour (layer Top)))\n"
        "  (placement (component U1 (place P 1 1 front 0)) (component U2 (place P 9 9 front 0)))\n"
        "  (library (image P (pin 1 0 0)))\n"
        "  (network (net N1 (pins U1-1 U2-1))))\n";
    DsnImporter dsn;
    ImportResult ir = dsn.import_text(text, "pour.dsn");
    bool warned = false;
    for (const auto& w : ir.warnings)
        if (w.find("pour") != std::string::npos && w.find("warning") != std::string::npos)
            warned = true;
    CT_CHECK(warned);
}

// Sidecar nets extend configs: current/voltage intent lands on DSN nets;
// unknown nets are hard errors (no silent typos).
CT_TEST(adapter_sidecar_nets) {
    DsnImporter dsn;
    ImportResult ir = dsn.import_file(fixture("golden_demo.dsn"));
    JsonValue cfg = parse_json(
        "{\"nets\": {\"VBAT\": {\"current_a\": 8.0, \"voltage_v\": 16.8, "
        "\"trace_width_min_mm\": 1.2, \"via_class\": \"POWER\"}}}");
    apply_sidecar_nets(ir.board, cfg);
    const NetInfo* vbat = ir.board.find_net_by_name("VBAT");
    CT_CHECK(vbat->has_current && vbat->current_a == 8.0);
    CT_CHECK(vbat->has_voltage && vbat->voltage_v == 16.8);
    CT_CHECK(vbat->via_class == "POWER");
    JsonValue bad = parse_json("{\"nets\": {\"NOPE\": {\"current_a\": 1.0}}}");
    bool threw = false;
    try {
        apply_sidecar_nets(ir.board, bad);
    } catch (const BoardError&) {
        threw = true;
    }
    CT_CHECK(threw);
}

int main() { return copperline::test::run_all_tests(); }
