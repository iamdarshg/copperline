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
