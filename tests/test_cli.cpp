// End-to-end CLI tests: drive the built `router` binary on fixtures and
// assert exit codes, JSON purity and stable schemas.
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "helpers.h"
#include "router/dsn.h"
#include "router/json.h"
#include "router/parallel.h"

using namespace copperline;

#ifndef ROUTER_BIN
#define ROUTER_BIN "router"
#endif
#ifndef FIXTURE_DIR
#define FIXTURE_DIR "fixtures"
#endif

namespace {

// cmd.exe cannot execute a quoted program path with forward slashes, so every
// path baked into a system() command is normalized to backslashes.
std::string win_path(std::string p) {
    for (char& c : p)
        if (c == '/') c = '\\';
    return p;
}

struct CliResult {
    int rc = 0;
    std::string out;  // stdout (pure JSON under --json)
    std::string err;  // stderr (warnings, --progress, --time-stages)
};

CliResult run_cli_full(const std::string& args) {
    namespace fs = std::filesystem;
    fs::path tmp = fs::temp_directory_path() / "copperline_cli_out.txt";
    fs::path err = fs::temp_directory_path() / "copperline_cli_err.txt";
    std::string prog = win_path(ROUTER_BIN);
    std::string redir = " > \"" + win_path(tmp.string()) + "\" 2> \"" + win_path(err.string()) + "\"";
    std::string cmd;
    if (prog.find(' ') == std::string::npos && prog.find('&') == std::string::npos) {
        cmd = prog + " " + args + redir;
    } else {
        // msvcrt system() mangles a leading quoted program path (cmd strips
        // the outer quotes), so use the documented cmd /c ""exe" args" form.
        cmd = "cmd /c \"\"" + prog + "\" " + args + redir + "\"";
    }
    CliResult r;
    r.rc = std::system(cmd.c_str());
    {
        std::ifstream f(tmp, std::ios::binary);
        std::ostringstream ss;
        ss << f.rdbuf();
        r.out = ss.str();
    }
    {
        std::ifstream f(err, std::ios::binary);
        std::ostringstream ss;
        ss << f.rdbuf();
        r.err = ss.str();
    }
    return r;
}

std::string run_cli(const std::string& args, int& rc) {
    CliResult r = run_cli_full(args);
    rc = r.rc;
    return r.out;
}

std::string fixture(const std::string& name) {
    return win_path(std::string(FIXTURE_DIR) + "/" + name);
}

JsonValue must_parse(const std::string& text) {
    try {
        return parse_json(text);
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("stdout is not pure JSON: ") + e.what() +
                                 " :: " + text.substr(0, 200));
    }
}

// S3 perf gate: first stderr line whose JSON carries {"event": <name>}.
// Plain-text diagnostics (import warnings) are skipped line by line.
JsonValue find_event(const std::string& err, const std::string& name) {
    std::istringstream in(err);
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        try {
            JsonValue v = parse_json(line);
            if (v.get_string("event") == name) return v;
        } catch (...) {
            continue;
        }
    }
    throw std::runtime_error("stderr event not found: " + name +
                             " in: " + err.substr(0, 300));
}

std::string temp_path(const std::string& name) {
    return win_path((std::filesystem::temp_directory_path() / name).string());
}

}  // namespace

CT_TEST(sidecar_defaults_override_board_clearance) {
    Board b = copperline::test::base_2layer();
    const Coord old_clearance = b.defaults.clearance_nm;
    JsonValue cfg = JsonValue::object();
    JsonValue defaults = JsonValue::object();
    defaults["clearance_mm"] = 0.125;
    cfg["defaults"] = defaults;
    apply_sidecar_nets(b, cfg);
    CT_CHECK(old_clearance != b.defaults.clearance_nm);
    CT_CHECK(b.defaults.clearance_nm == mm_to_nm(0.125));
}

CT_TEST(capabilities_json) {
    int rc = 0;
    std::string out = run_cli("capabilities --json", rc);
    CT_CHECK(rc == 0);
    JsonValue v = must_parse(out);
    CT_CHECK(v.get_string("name") == "copperline");
    CT_CHECK(v.get_string("schema") == "copperline/capabilities/1");
    CT_CHECK(v.has("exit_codes"));
    CT_CHECK(v.has("formats"));
}

CT_TEST(capabilities_parallel_phase) {
    int rc = 0;
    std::string out = run_cli("capabilities --json", rc);
    CT_CHECK(rc == 0);
    JsonValue v = must_parse(out);
    CT_CHECK(v.get_string("phase") == "prompt-5-release");
    CT_CHECK(v.find("features")->get_bool("parallel_routing", false));
    CT_CHECK(v.find("features")->get_bool("ripup_reroute", false));
    bool has_benchmark = false;
    for (const auto& c : v.find("commands")->as_array())
        if (c.as_string() == "benchmark") has_benchmark = true;
    CT_CHECK(has_benchmark);
}

CT_TEST(route_forced_ripup_recovers_and_reports) {
    int rc = 0;
    std::string routed = temp_path("routed_ripup.json");
    std::string out = run_cli("route " + fixture("forced_ripup.json") +
                                  " --json --output \"" + routed + "\" --seed 42",
                              rc);
    CT_CHECK(rc == 0);
    JsonValue v = must_parse(out);
    CT_CHECK(v.get_string("status") == "COMPLETE");
    CT_CHECK(v.get_string("result_category") == "COMPLETE");
    CT_CHECK(v.has("recovery"));
    CT_CHECK(v.find("recovery")->get_number("generations", 0) >= 1);
    CT_CHECK(v.find("recovery")->get_number("ripups", 0) >= 1);
    CT_CHECK(v.has("state_hash"));
    // The recovered board verifies independently.
    std::string vout = run_cli("verify \"" + routed + "\" --json", rc);
    CT_CHECK(rc == 0);
    JsonValue vv = must_parse(vout);
    CT_CHECK(vv.get_bool("ok", false));
}

CT_TEST(analyze_open_json) {
    int rc = 0;
    std::string out = run_cli("analyze " + fixture("open_2layer.json") + " --json", rc);
    CT_CHECK(rc == 0);
    JsonValue v = must_parse(out);
    CT_CHECK(v.get_string("schema") == "copperline/analyze-report/1");
    CT_CHECK(v.find("nets")->as_array().size() == 2);
    CT_CHECK(v.has("likely_bottlenecks"));
}

CT_TEST(verify_unrouted_reports_incomplete) {
    int rc = 0;
    std::string out = run_cli("verify " + fixture("open_2layer.json") + " --json", rc);
    CT_CHECK(rc == 4);  // routing incomplete: nothing routed yet
    JsonValue v = must_parse(out);
    CT_CHECK(v.get_bool("connected", true) == false);
}

CT_TEST(verify_violation_exit_5) {
    int rc = 0;
    std::string out = run_cli("verify " + fixture("violation_board.json") + " --json", rc);
    CT_CHECK(rc == 5);
    JsonValue v = must_parse(out);
    CT_CHECK(v.get_bool("legal", true) == false);
    CT_CHECK(!v.find("violations")->as_array().empty());
}

CT_TEST(route_open_complete_and_verify) {
    int rc = 0;
    std::string routed = temp_path("routed_open.json");
    std::string out = run_cli("route " + fixture("open_2layer.json") + " --json --output \"" +
                                  routed + "\" --seed 7",
                              rc);
    CT_CHECK(rc == 0);
    JsonValue v = must_parse(out);
    CT_CHECK(v.get_string("status") == "COMPLETE");
    // The file written via --output must independently verify clean.
    int rc2 = 0;
    std::string out2 = run_cli("verify \"" + routed + "\" --json", rc2);
    CT_CHECK(rc2 == 0);
    JsonValue v2 = must_parse(out2);
    CT_CHECK(v2.get_bool("ok", false));
}

CT_TEST(route_blocked_reports_incomplete) {
    int rc = 0;
    std::string out = run_cli("route " + fixture("blocked_impossible.json") + " --json", rc);
    CT_CHECK(rc == 4);
    JsonValue v = must_parse(out);
    CT_CHECK(v.get_string("status") == "INCOMPLETE");
    CT_CHECK(!v.find("failures")->as_array().empty());
    CT_CHECK(!v.find("failures")->as_array()[0].find("blockers")->as_array().empty());
}

CT_TEST(route_budget_exhausted_exit_7) {
    // Pin maturity escalation off so the tiny search cap cannot be
    // outgrown across epochs: without this, issue-#14 via-disc legality
    // admits extra transitions and the escalated attempt completes.
    std::string cfg = temp_path("no_maturity.json");
    {
        std::ofstream f(cfg, std::ios::binary | std::ios::trunc);
        f << "{\"maturity\":{\"enabled\":false}}";
    }
    int rc = 0;
    std::string out = run_cli("route " + fixture("obstacle_detour.json") +
                                  " --json --max-search-nodes 1 --config \"" + cfg + "\"",
                              rc);
    CT_CHECK(rc == 7);
    JsonValue v = must_parse(out);
    CT_CHECK(v.get_string("status") == "BUDGET_EXHAUSTED");
}

CT_TEST(route_obstacle_with_config) {
    int rc = 0;
    std::string out = run_cli("route " + fixture("high_current.json") + " --json --config " +
                                  fixture("rules_demo.json"),
                              rc);
    CT_CHECK(rc == 0);
    JsonValue v = must_parse(out);
    CT_CHECK(v.get_string("status") == "COMPLETE");
}

CT_TEST(invalid_input_exit_2) {
    int rc = 0;
    run_cli("route " + fixture("does_not_exist.json") + " --json", rc);
    CT_CHECK(rc == 2);
    int rc2 = 0;
    run_cli("frobnicate --json", rc2);
    CT_CHECK(rc2 == 2);
}

CT_TEST(malformed_rules_exit_3) {
    std::string bad = temp_path("bad_rules.json");
    {
        std::ofstream f(bad, std::ios::binary | std::ios::trunc);
        f << "{\"ipc\": {\"mm_per_amp\": -1}}";
    }
    int rc = 0;
    run_cli("route " + fixture("open_2layer.json") + " --json --config \"" + bad + "\"", rc);
    CT_CHECK(rc == 3);
}

CT_TEST(kicad_ingest) {
    int rc = 0;
    std::string out = run_cli("analyze " + fixture("minimal.kicad_pcb") + " --json", rc);
    CT_CHECK(rc == 0);
    JsonValue v = must_parse(out);
    CT_CHECK(v.find("board")->get_number("terminals", 0) == 4);
    // Net-class rules from KiCad must surface as explicit widths.
    bool vcc_wide = false;
    for (const auto& n : v.find("nets")->as_array()) {
        if (n.get_string("name") == "VCC") {
            vcc_wide = n.get_number("required_width_mm", 0) == 0.8;
        }
    }
    CT_CHECK(vcc_wide);
}

CT_TEST(route_is_deterministic) {
    int rc1 = 0, rc2 = 0;
    std::string o1 = run_cli("route " + fixture("obstacle_detour.json") + " --json --seed 7", rc1);
    std::string o2 = run_cli("route " + fixture("obstacle_detour.json") + " --json --seed 7", rc2);
    CT_CHECK(rc1 == 0 && rc2 == 0);
    // time_ms is wall-clock by definition; everything else must be identical.
    JsonValue v1 = must_parse(o1), v2 = must_parse(o2);
    v1["stats"]["time_ms"] = 0.0;
    v2["stats"]["time_ms"] = 0.0;
    // Per-epoch time_ms is measured wall-clock by design; mask it too.
    for (JsonValue* v : {&v1, &v2}) {
        if (const JsonValue* log = v->find("epoch_log")) {
            for (auto& e : const_cast<JsonArray&>(log->as_array())) e["time_ms"] = 0.0;
        }
    }
    CT_CHECK(serialize_json(v1) == serialize_json(v2));
}

CT_TEST(escape_bga_json) {
    int rc = 0;
    std::string out = run_cli("escape " + fixture("bga_4x4.json") + " --json", rc);
    CT_CHECK(rc == 0);
    JsonValue v = must_parse(out);
    CT_CHECK(v.get_string("schema") == "copperline/escape-report/1");
    CT_CHECK(!v.find("footprints")->as_array().empty());
    const JsonValue& fp = v.find("footprints")->as_array().front();
    CT_CHECK(fp.has("eligibility_order"));
    CT_CHECK(fp.has("commit_order"));
    CT_CHECK(!fp.find("pads")->as_array().empty());
}

CT_TEST(route_threads_parity) {
    int rc1 = 0, rc4 = 0;
    std::string o1 = run_cli("route " + fixture("obstacle_detour.json") + " --json --seed 7 --threads 1", rc1);
    std::string o4 = run_cli("route " + fixture("obstacle_detour.json") + " --json --seed 7 --threads 4", rc4);
    CT_CHECK(rc1 == 0 && rc4 == 0);
    JsonValue v1 = must_parse(o1), v4 = must_parse(o4);
    CT_CHECK(v1.get_string("status") == "COMPLETE");
    CT_CHECK(v4.get_string("status") == "COMPLETE");
    CT_CHECK(v1.get_string("board_hash") == v4.get_string("board_hash"));
    CT_CHECK(serialize_json(*v1.find("board")) == serialize_json(*v4.find("board")));
    CT_CHECK(v4.find("stats")->get_number("threads_used", 0) >= 2);
    CT_CHECK(v1.find("stats")->get_number("epochs", 0) >= 1);
    CT_CHECK(v1.has("epoch_log"));
    CT_CHECK(v1.has("congestion_hotspots"));
}

CT_TEST(route_progress_json_pure) {
    int rc = 0;
    // --progress emits NDJSON on stderr; stdout must stay pure JSON.
    std::string out =
        run_cli("route " + fixture("open_2layer.json") + " --json --seed 7 --progress", rc);
    CT_CHECK(rc == 0);
    JsonValue v = must_parse(out);
    CT_CHECK(v.get_string("status") == "COMPLETE");
}

CT_TEST(benchmark_reports_real_numbers) {
    int rc = 0;
    std::string out = run_cli("benchmark " + fixture("open_2layer.json") + " --json", rc);
    CT_CHECK(rc == 0);
    JsonValue v = must_parse(out);
    CT_CHECK(v.get_string("schema") == "copperline/benchmark-report/1");
    CT_CHECK(v.has("single"));
    CT_CHECK(v.has("parallel"));
    CT_CHECK(v.has("speedup"));
    CT_CHECK(v.get_bool("identical_geometry", false));
    CT_CHECK(v.find("single")->get_string("board_hash") ==
              v.find("parallel")->get_string("board_hash"));
}

CT_TEST(escape_impossible_reports_incomplete) {
    int rc = 0;
    std::string out = run_cli("escape " + fixture("impossible_escape.json") + " --json", rc);
    CT_CHECK(rc == 4);
    JsonValue v = must_parse(out);
    CT_CHECK(v.get_string("status") == "INCOMPLETE");
}

CT_TEST(escape_threads_parity) {
    // `escape --threads` wiring: explicit counts parallelize the fallback
    // fan-out but must not change the committed geometry. The escape report
    // carries no timing fields, so the JSON reports are byte-identical.
    int rc1 = 0, rc16 = 0;
    std::string o1 = run_cli("escape " + fixture("bga_8x8.json") + " --json --threads 1", rc1);
    std::string o16 =
        run_cli("escape " + fixture("bga_8x8.json") + " --json --threads 16", rc16);
    CT_CHECK(rc1 == rc16);
    JsonValue v1 = must_parse(o1), v16 = must_parse(o16);
    CT_CHECK(v1.get_string("schema") == "copperline/escape-report/1");
    CT_CHECK(o1 == o16);
}

CT_TEST(route_illegal_copper_refuses_success) {
    // Issue #2: pre-existing clearance violation must not route COMPLETE.
    // The route report must carry the independent verification and the
    // process must exit with the hard-rule-violation code, never 0.
    int rc = 0;
    std::string out = run_cli("route " + fixture("violation_board.json") + " --json", rc);
    CT_CHECK(rc == 5);
    JsonValue v = must_parse(out);
    CT_CHECK(v.get_string("status") == "VIOLATION");
    CT_CHECK(v.get_bool("verifier_ok", true) == false);
    CT_CHECK(v.get_bool("verifier_legal", true) == false);
    CT_CHECK(v.has("verification"));
    CT_CHECK(!v.find("verification")->find("violations")->as_array().empty());
}

CT_TEST(route_complete_carries_verifier_ok) {
    int rc = 0;
    std::string out =
        run_cli("route " + fixture("open_2layer.json") + " --json --seed 7", rc);
    CT_CHECK(rc == 0);
    JsonValue v = must_parse(out);
    CT_CHECK(v.get_string("status") == "COMPLETE");
    CT_CHECK(v.get_bool("verifier_ok", false));
    CT_CHECK(v.has("verification"));
    CT_CHECK(v.find("verification")->get_bool("ok", false));
}

CT_TEST(explain_failure_renders_report) {
    int rc = 0;
    std::string rep = temp_path("explain_report.json");
    run_cli("route " + fixture("blocked_impossible.json") + " --json --report \"" + rep +
                "\" --seed 42",
            rc);
    CT_CHECK(rc == 4);
    int rc2 = 0;
    std::string out = run_cli("explain-failure \"" + rep + "\" --json", rc2);
    CT_CHECK(rc2 == 0);
    JsonValue v = must_parse(out);
    CT_CHECK(v.get_string("schema") == "copperline/failure-explanation/1");
    CT_CHECK(v.get_number("failure_count", 0) >= 1);
    const JsonValue& fl = v.find("failures")->as_array().front();
    CT_CHECK(fl.get_string("net_name") == "SIG1");
    CT_CHECK(!fl.get_string("category").empty());
    CT_CHECK(fl.has("top_blockers"));
    CT_CHECK(fl.has("suggestion"));
    CT_CHECK(fl.get_number("candidate_count", -1) >= 0);
}

CT_TEST(threads_zero_means_auto) {
    int rc = 0;
    std::string out = run_cli("route " + fixture("open_2layer.json") +
                                  " --json --seed 7 --threads 0",
                              rc);
    CT_CHECK(rc == 0);
    JsonValue v = must_parse(out);
    CT_CHECK(v.get_string("status") == "COMPLETE");
    CT_CHECK(v.find("params")->get_number("threads_effective", 0) >= 1);
    // Derived from the shipped constant so the assertion tracks the budget.
    CT_CHECK(v.find("params")->get_number("memory_budget_mb", 0) ==
             static_cast<double>(kRouterMemoryBudgetBytes / (1024ULL * 1024ULL)));
}

CT_TEST(dsn_ses_workflow) {
    int rc = 0;
    std::string ses = temp_path("golden_demo_cli.ses");
    std::string out = run_cli("route " + fixture("golden_demo.dsn") + " --json --seed 42 " +
                                  "--output \"" + ses + "\"",
                              rc);
    CT_CHECK(rc == 0);
    JsonValue v = must_parse(out);
    CT_CHECK(v.get_string("status") == "COMPLETE");
    CT_CHECK(v.get_string("output_format") == "ses");
    int rc2 = 0;
    std::string vout = run_cli("verify " + fixture("golden_demo.dsn") + " --routes \"" +
                                   ses + "\" --json",
                               rc2);
    CT_CHECK(rc2 == 0);
    JsonValue vv = must_parse(vout);
    CT_CHECK(vv.get_bool("ok", false));
}

CT_TEST(route_optimizer_reported) {
    int rc = 0;
    std::string out = run_cli("route " + fixture("open_2layer.json") + " --json --seed 7", rc);
    CT_CHECK(rc == 0);
    JsonValue v = must_parse(out);
    CT_CHECK(v.has("optimizer"));
    CT_CHECK(v.find("optimizer")->get_bool("ran", false));
    CT_CHECK(v.find("optimizer")->get_string("schema") == "copperline/optimizer-report/1");
}

CT_TEST(capabilities_unsupported_empty) {
    int rc = 0;
    std::string out = run_cli("capabilities --json", rc);
    CT_CHECK(rc == 0);
    JsonValue v = must_parse(out);
    CT_CHECK(v.find("unsupported")->as_array().empty());
    CT_CHECK(v.find("features")->get_bool("kicad_export", false));
    CT_CHECK(v.find("features")->get_bool("gerber_import", false));
    CT_CHECK(v.find("features")->get_bool("ipc2581_import", false));
    CT_CHECK(v.find("features")->get_bool("copper_pours", false));
    bool has_gerber = false, has_ipc = false;
    for (const auto& f : v.find("formats")->find("supported")->as_array()) {
        if (f.as_string() == "gerber") has_gerber = true;
        if (f.as_string() == "ipc-2581") has_ipc = true;
    }
    CT_CHECK(has_gerber && has_ipc);
    CT_CHECK(v.find("formats")->find("planned")->as_array().empty());
}

CT_TEST(route_kicad_export_verify) {
    int rc = 0;
    std::string routed = temp_path("routed_kicad.kicad_pcb");
    std::string out = run_cli("route " + fixture("minimal.kicad_pcb") +
                                  " --json --seed 42 --output \"" + routed + "\"",
                              rc);
    CT_CHECK(rc == 0);
    JsonValue v = must_parse(out);
    CT_CHECK(v.get_string("status") == "COMPLETE");
    CT_CHECK(v.get_string("output_format") == "kicad_pcb");
    int rc2 = 0;
    std::string vout = run_cli("verify \"" + routed + "\" --json", rc2);
    CT_CHECK(rc2 == 0);
    JsonValue vv = must_parse(vout);
    CT_CHECK(vv.get_bool("ok", false));
}

CT_TEST(analyze_gerber_and_ipc2581) {
    int rc = 0;
    std::string gout = run_cli("analyze " + fixture("gerber_copper.gbr") + " --json", rc);
    CT_CHECK(rc == 0);
    JsonValue g = must_parse(gout);
    CT_CHECK(g.get_string("schema") == "copperline/analyze-report/1");
    int rc2 = 0;
    std::string iout = run_cli("analyze " + fixture("ipc2581_demo.xml") + " --json", rc2);
    CT_CHECK(rc2 == 0);
    JsonValue iv = must_parse(iout);
    CT_CHECK(iv.find("board")->get_number("terminals", 0) == 4);
}

CT_TEST(route_no_board_echo_report_only) {
    // S3 (I3): --no-board-echo drops the full board object from route
    // stdout (agent-loop I/O hygiene); everything else is identical.
    int rc1 = 0, rc2 = 0;
    std::string o1 = run_cli("route " + fixture("open_2layer.json") + " --json --seed 42 --threads 1", rc1);
    std::string o2 = run_cli("route " + fixture("open_2layer.json") +
                                 " --json --seed 42 --threads 1 --no-board-echo",
                             rc2);
    CT_CHECK(rc1 == 0 && rc2 == 0);
    JsonValue v1 = must_parse(o1), v2 = must_parse(o2);
    CT_CHECK(v1.get_string("status") == "COMPLETE");
    CT_CHECK(v2.get_string("status") == "COMPLETE");
    CT_CHECK(v1.has("board"));
    CT_CHECK(!v2.has("board"));
    CT_CHECK(v1.get_string("board_hash") == v2.get_string("board_hash"));
    // Wall-clock times vary run to run by design; mask them like the
    // determinism test, then every non-board key must match exactly.
    v1["stats"]["time_ms"] = 0.0;
    v2["stats"]["time_ms"] = 0.0;
    for (JsonValue* v : {&v1, &v2}) {
        if (const JsonValue* log = v->find("epoch_log")) {
            for (auto& e : const_cast<JsonArray&>(log->as_array())) e["time_ms"] = 0.0;
        }
    }
    for (const auto& kv : v1.as_object()) {
        if (kv.first == "board") continue;
        CT_CHECK(v2.has(kv.first));
        CT_CHECK(serialize_json(kv.second) == serialize_json(*v2.find(kv.first)));
    }
    CT_CHECK(o2.size() < o1.size());
    std::printf("  [io] route stdout %llu -> %llu bytes (board echo saved %llu)\n",
                (unsigned long long)o1.size(), (unsigned long long)o2.size(),
                (unsigned long long)(o1.size() - o2.size()));
}

CT_TEST(route_time_stages_schema_and_budgets) {
    // S3 perf gate (--time-stages surface): stage_timings + cli_timings
    // events keep their schema (S1/S2 depend on it) and every stage stays
    // inside generous regression ceilings on a 2-task board.
    CliResult r = run_cli_full("route " + fixture("open_2layer.json") +
                               " --json --seed 42 --threads 1 --time-stages");
    CT_CHECK(r.rc == 0);
    JsonValue v = must_parse(r.out);
    CT_CHECK(v.get_string("status") == "COMPLETE");
    JsonValue st = find_event(r.err, "stage_timings");
    JsonValue cl = find_event(r.err, "cli_timings");
    const char* stage_ms[] = {"escape_ms",  "taskgen_ms", "batch_ms",   "workers_ms",
                              "arbiter_ms", "recovery_ms", "materialize_ms", "tuning_ms",
                              "attribution_ms", "verify_ms", "optimizer_ms", "hash_ms"};
    for (const char* k : stage_ms) {
        CT_CHECK(st.has(k));
        double ms = st.get_number(k, -1.0);
        CT_CHECK(ms >= 0.0 && ms < 60000.0);
    }
    const char* cli_ms[] = {"import_ms", "cli_verify_ms", "report_json_ms",
                            "serialize_write_ms"};
    for (const char* k : cli_ms) {
        CT_CHECK(cl.has(k));
        double ms = cl.get_number(k, -1.0);
        CT_CHECK(ms >= 0.0 && ms < 60000.0);
    }
    CT_CHECK(st.get_number("threads", 0) == 1);
    CT_CHECK(st.get_number("epochs", 0) >= 1);
    CT_CHECK(st.get_number("tasks_total", 0) == 2);
    CT_CHECK(cl.get_number("threads", 0) == 1);
    CT_CHECK(cl.get_number("board_bytes", 0) > 0);
    CT_CHECK(cl.get_number("output_bytes", -1) == 0);  // no --output/--report
    std::printf("  [stages] workers %.3fms arbiter %.3fms report_json %.3fms cli_verify %.3fms\n",
                st.get_number("workers_ms", -1), st.get_number("arbiter_ms", -1),
                cl.get_number("report_json_ms", -1), cl.get_number("cli_verify_ms", -1));
    // Default runs stay silent: no timing events without the flag.
    CliResult quiet = run_cli_full("route " + fixture("open_2layer.json") + " --json --seed 42");
    CT_CHECK(quiet.rc == 0);
    CT_CHECK(quiet.err.find("stage_timings") == std::string::npos);
    CT_CHECK(quiet.err.find("cli_timings") == std::string::npos);
}

CT_TEST(route_time_stages_geometry_parity_1_vs_16) {
    // S3 perf gate (determinism across worker counts): --threads 1 and 16
    // produce identical copper; both runs still emit well-formed timings
    // with matching epoch/task accounting.
    CliResult r1 = run_cli_full("route " + fixture("open_2layer.json") +
                                " --json --seed 42 --threads 1 --time-stages");
    CliResult rN = run_cli_full("route " + fixture("open_2layer.json") +
                                " --json --seed 42 --threads 16 --time-stages");
    CT_CHECK(r1.rc == 0 && rN.rc == 0);
    JsonValue v1 = must_parse(r1.out), vN = must_parse(rN.out);
    CT_CHECK(v1.get_string("status") == "COMPLETE");
    CT_CHECK(vN.get_string("status") == "COMPLETE");
    CT_CHECK(v1.get_string("board_hash") == vN.get_string("board_hash"));
    CT_CHECK(serialize_json(*v1.find("board")) == serialize_json(*vN.find("board")));
    JsonValue s1 = find_event(r1.err, "stage_timings");
    JsonValue sN = find_event(rN.err, "stage_timings");
    CT_CHECK(s1.get_number("threads", 0) == 1);
    CT_CHECK(sN.get_number("threads", 0) == 16);
    CT_CHECK(s1.get_number("epochs", -1) == sN.get_number("epochs", -1));
    CT_CHECK(s1.get_number("tasks_total", -1) == sN.get_number("tasks_total", -1));
    std::printf("  [parity] hash %s identical at 1T/16T (%d epochs)\n",
                v1.get_string("board_hash").c_str(),
                (int)s1.get_number("epochs", -1));
}

int main() { return copperline::test::run_all_tests(); }
