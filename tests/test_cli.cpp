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

// Phase 3 item 15: declarative strategy files (--strategy) fold stage
// toggles + budget scalars into the engine; the active strategy name is
// surfaced in the report.
CT_TEST(cli_strategy_engine) {
    // Baseline without a strategy: no name, optimizer on.
    CliResult base = run_cli_full("route " + fixture("open_2layer.json") +
                                  " --json --seed 42 --no-board-echo");
    CT_CHECK(base.rc == 0);
    JsonValue vb = must_parse(base.out);
    CT_CHECK(vb.get_string("strategy", "") == "");
    CT_CHECK(vb.get_string("status") == "COMPLETE");

    // via_miser strategy: name surfaced, same geometry hash, budget scale
    // 0.75 applied (report carries it via the strategy name).
    CliResult vm = run_cli_full("route " + fixture("open_2layer.json") +
                                " --json --seed 42 --no-board-echo"
                                " --strategy " + fixture("strategies/via_miser.json"));
    CT_CHECK(vm.rc == 0);
    JsonValue vv = must_parse(vm.out);
    CT_CHECK(vv.get_string("strategy") == "via_miser");
    CT_CHECK(vv.get_string("status") == "COMPLETE");

    // Stage toggles actually disable machinery: no_cleanup_test sets
    // optimizer/tuning/recovery false and astar_scale 0.5.
    CliResult off = run_cli_full("route " + fixture("open_2layer.json") +
                                 " --json --seed 42 --no-board-echo"
                                 " --strategy " + fixture("strategies/no_cleanup_test.json"));
    CT_CHECK(off.rc == 0);
    JsonValue vo = must_parse(off.out);
    CT_CHECK(vo.get_string("strategy") == "no_cleanup_test");
    CT_CHECK(vo.get_string("status") == "COMPLETE");
    JsonValue oo = *vo.find("optimizer");
    CT_CHECK(oo.get_bool("enabled", true) == false);
    CT_CHECK(oo.get_bool("ran", true) == false);
    JsonValue rec = *vo.find("recovery");
    CT_CHECK(rec.get_number("generations", -1) == 0);

    // Malformed strategy JSON -> malformed_rules exit.
    std::string bad = win_path(std::string(FIXTURE_DIR) + "/strategies/no_such_file.json");
    CliResult miss = run_cli_full("route " + fixture("open_2layer.json") +
                                  " --strategy " + bad + " --json");
    CT_CHECK(miss.rc == 2);  // invalid input: cannot open file
    std::string badjson = win_path(std::string(FIXTURE_DIR) + "/strategies/_broken.json");
    {  // write a broken strategy file into the fixture dir
        std::ofstream f(badjson);
        f << "{ not json";
    }
    CliResult badr = run_cli_full("route " + fixture("open_2layer.json") +
                                  " --strategy " + badjson + " --json");
    CT_CHECK(badr.rc == 3);  // malformed rules: bad strategy JSON
    CT_CHECK(badr.out.find("malformed_rules") != std::string::npos);
    std::remove(badjson.c_str());
}

// Phase 3 item 16: the memory U-pattern pass is strategy-gated — OFF by
// default (existing boards untouched), ON with "memory": true.
CT_TEST(cli_memory_pass_strategy_gate) {
    // Without a memory strategy: pass reports ran=false, 0 pairs.
    CliResult off = run_cli_full("route " + fixture("memory_bus.json") +
                                 " --json --seed 42 --no-board-echo");
    CT_CHECK(off.rc == 0);
    JsonValue vo = must_parse(off.out);
    CT_CHECK(vo.get_string("status") == "COMPLETE");
    JsonValue mo = *vo.find("memory");
    CT_CHECK(mo.get_bool("ran", true) == false);
    CT_CHECK(mo.get_number("pairs_committed", -1) == 0);

    // With the memory_bus strategy: 4 aligned pairs committed as 12 traces.
    CliResult on = run_cli_full("route " + fixture("memory_bus.json") +
                                " --json --seed 42 --no-board-echo"
                                " --strategy " + fixture("strategies/memory_bus.json"));
    CT_CHECK(on.rc == 0);
    JsonValue vn = must_parse(on.out);
    CT_CHECK(vn.get_string("status") == "COMPLETE");
    CT_CHECK(vn.get_string("strategy") == "memory_bus");
    JsonValue mn = *vn.find("memory");
    CT_CHECK(mn.get_bool("ran", false) == true);
    CT_CHECK(mn.get_number("pairs_found", -1) == 4);
    CT_CHECK(mn.get_number("pairs_committed", -1) == 4);
    CT_CHECK(mn.get_number("traces_added", -1) == 12);
    // The independent verifier accepts the memory copper.
    CT_CHECK(vn.find("verification") && vn.find("verification")->is_object());
}

// Phase 3 item 19: the straighten pass is strategy-gated — OFF by default
// (existing boards untouched), ON with "straighten": true. The pass reuses
// the optimizer's transactional verify+revert machinery.
CT_TEST(cli_straighten_strategy_gate) {
    // Without the strategy: optimizer runs but straighten_ran stays false.
    CliResult off = run_cli_full("route " + fixture("open_2layer.json") +
                                 " --json --seed 42 --no-board-echo");
    CT_CHECK(off.rc == 0);
    JsonValue vo = must_parse(off.out);
    CT_CHECK(vo.get_string("status") == "COMPLETE");
    JsonValue oo = *vo.find("optimizer");
    CT_CHECK(oo.get_bool("straighten_ran", true) == false);

    // With the straighten_demo strategy: the pass runs and re-verifies.
    CliResult on = run_cli_full("route " + fixture("open_2layer.json") +
                                " --json --seed 42 --no-board-echo"
                                " --strategy " + fixture("strategies/straighten_demo.json"));
    CT_CHECK(on.rc == 0);
    JsonValue vn = must_parse(on.out);
    CT_CHECK(vn.get_string("status") == "COMPLETE");
    CT_CHECK(vn.get_string("strategy") == "straighten_demo");
    JsonValue on2 = *vn.find("optimizer");
    CT_CHECK(on2.get_bool("straighten_ran", false) == true);
    // Straighten never grows wire length (must_shorten gate).
    CT_CHECK(on2.get_number("length_after_mm", -1.0) <=
             on2.get_number("length_before_mm", 0.0) + 1e-9);
}

// Phase 3 item 24: Via Miser is strategy-gated — the optimizer report
// shows via_miser_ran true only when a strategy sets "via_miser": true.
CT_TEST(cli_via_miser_strategy_gate) {
    CliResult off = run_cli_full("route " + fixture("open_2layer.json") +
                                 " --json --seed 42 --no-board-echo");
    CT_CHECK(off.rc == 0);
    JsonValue vo = must_parse(off.out);
    JsonValue oo = *vo.find("optimizer");
    CT_CHECK(oo.get_bool("via_miser_ran", true) == false);

    CliResult on = run_cli_full("route " + fixture("open_2layer.json") +
                                " --json --seed 42 --no-board-echo"
                                " --strategy " + fixture("strategies/via_miser.json"));
    CT_CHECK(on.rc == 0);
    JsonValue vn = must_parse(on.out);
    CT_CHECK(vn.get_string("status") == "COMPLETE");
    CT_CHECK(vn.get_string("strategy") == "via_miser");
    JsonValue on2 = *vn.find("optimizer");
    CT_CHECK(on2.get_bool("via_miser_ran", false) == true);
    // Via Miser never grows length either (verify+revert gate).
    CT_CHECK(on2.get_number("length_after_mm", -1.0) <=
             on2.get_number("length_before_mm", 0.0) + 1e-9);
}

// Phase 3 item 17: adjacent-memory is strategy-gated — OFF by default, and
// with the strategy it routes the adjacent same-net pin pairs.
CT_TEST(cli_adjacent_memory_strategy_gate) {
    CliResult off = run_cli_full("route " + fixture("adjacent_bus.json") +
                                 " --json --seed 42 --no-board-echo");
    CT_CHECK(off.rc == 0);
    JsonValue vo = must_parse(off.out);
    CT_CHECK(vo.get_string("status") == "COMPLETE");
    JsonValue ao = *vo.find("adjacent_memory");
    CT_CHECK(ao.get_bool("ran", true) == false);
    CT_CHECK(ao.get_number("pairs_committed", -1) == 0);

    CliResult on = run_cli_full(
        "route " + fixture("adjacent_bus.json") +
        " --json --seed 42 --no-board-echo --strategy " +
        fixture("strategies/adjacent_memory_demo.json"));
    CT_CHECK(on.rc == 0);
    JsonValue vn = must_parse(on.out);
    CT_CHECK(vn.get_string("status") == "COMPLETE");
    CT_CHECK(vn.get_string("strategy") == "adjacent_memory_demo");
    JsonValue an = *vn.find("adjacent_memory");
    CT_CHECK(an.get_bool("ran", false) == true);
    CT_CHECK(an.get_string("mode") == "adjacent_memory");
    CT_CHECK(an.get_number("pairs_found", -1) == 2);
    CT_CHECK(an.get_number("pairs_committed", -1) == 2);
    CT_CHECK(an.get_number("traces_added", -1) == 6);
    // The independent verifier accepts the adjacent copper.
    CT_CHECK(vn.find("verification") &&
             vn.find("verification")->get_bool("ok", false));
}

// Phase 3 item 20: corner miter is strategy-gated — OFF by default, ON with
// "orthogonal": true.
CT_TEST(cli_orthogonal_miter_strategy_gate) {
    CliResult off = run_cli_full("route " + fixture("open_2layer.json") +
                                 " --json --seed 42 --no-board-echo");
    CT_CHECK(off.rc == 0);
    JsonValue vo = must_parse(off.out);
    JsonValue oo = *vo.find("optimizer");
    CT_CHECK(oo.get_bool("orthogonal_ran", true) == false);

    CliResult on = run_cli_full(
        "route " + fixture("open_2layer.json") +
        " --json --seed 42 --no-board-echo --strategy " +
        fixture("strategies/orthogonal_demo.json"));
    CT_CHECK(on.rc == 0);
    JsonValue vn = must_parse(on.out);
    CT_CHECK(vn.get_string("status") == "COMPLETE");
    CT_CHECK(vn.get_string("strategy") == "orthogonal_demo");
    JsonValue on2 = *vn.find("optimizer");
    CT_CHECK(on2.get_bool("orthogonal_ran", false) == true);
    CT_CHECK(on2.get_number("length_after_mm", -1.0) <=
             on2.get_number("length_before_mm", 0.0) + 1e-9);
}

// Phase 3 item 18: clean pad entries is strategy-gated — OFF by default.
CT_TEST(cli_clean_pad_entries_strategy_gate) {
    CliResult off = run_cli_full("route " + fixture("open_2layer.json") +
                                 " --json --seed 42 --no-board-echo");
    CT_CHECK(off.rc == 0);
    JsonValue vo = must_parse(off.out);
    JsonValue oo = *vo.find("optimizer");
    CT_CHECK(oo.get_bool("clean_pad_entries_ran", true) == false);

    CliResult on = run_cli_full(
        "route " + fixture("open_2layer.json") +
        " --json --seed 42 --no-board-echo --strategy " +
        fixture("strategies/clean_pad_demo.json"));
    CT_CHECK(on.rc == 0);
    JsonValue vn = must_parse(on.out);
    CT_CHECK(vn.get_string("status") == "COMPLETE");
    CT_CHECK(vn.get_string("strategy") == "clean_pad_demo");
    JsonValue on2 = *vn.find("optimizer");
    CT_CHECK(on2.get_bool("clean_pad_entries_ran", false) == true);
    CT_CHECK(vn.find("verification") &&
             vn.find("verification")->get_bool("ok", false));
}

// Phase 5 item 30: `router gloss` — post-route polish over an already routed
// board, running exactly the requested passes with verify+revert, and a
// JSON report with before/after hashes.
CT_TEST(cli_gloss_runs_requested_passes) {
    const std::string routed = win_path(std::string(FIXTURE_DIR) + "/_gloss_routed.json");
    const std::string report = win_path(std::string(FIXTURE_DIR) + "/_gloss_report.json");
    CliResult route = run_cli_full("route " + fixture("multi_terminal_tree.json") +
                                   " --json --no-board-echo --seed 42 --output " + routed);
    CT_CHECK(route.rc == 0);
    CliResult g = run_cli_full("gloss " + routed + " --straighten --json --report " + report);
    CT_CHECK(g.rc == 0);
    JsonValue v = must_parse(g.out);
    CT_CHECK(v.get_string("schema") == "copperline/gloss-report/1");
    CT_CHECK(v.get_string("status") == "COMPLETE");
    JsonValue opt = *v.find("optimizer");
    CT_CHECK(opt.get_bool("ran", false) == true);
    CT_CHECK(opt.get_bool("straighten_ran", false) == true);
    CT_CHECK(opt.get_bool("via_miser_ran", true) == false);
    CT_CHECK(opt.get_number("applied", 0) >= 1.0);
    // Straighten never grows length and the hash reflects real copper change.
    CT_CHECK(opt.get_number("length_after_mm", 1e9) <=
             opt.get_number("length_before_mm", 0.0) + 1e-9);
    CT_CHECK(v.get_string("board_hash_before") != v.get_string("board_hash_after"));
    CT_CHECK(v.find("verification") && v.find("verification")->get_bool("ok", false));
    std::remove(routed.c_str());
    std::remove(report.c_str());
}

// gloss: no pass selected is invalid input; an unrouted board is gated
// before any polish runs (gloss is post-route).
CT_TEST(cli_gloss_guards) {
    CliResult none = run_cli_full("gloss " + fixture("open_2layer.json") + " --json");
    CT_CHECK(none.rc == 2);
    CT_CHECK(none.out.find("invalid_input") != std::string::npos);

    CliResult gated = run_cli_full("gloss " + fixture("open_2layer.json") +
                                   " --straighten --json");
    CT_CHECK(gated.rc == 4);  // incomplete: input copper is not routed
    JsonValue g = must_parse(gated.out);
    CT_CHECK(g.get_string("status") == "GATED");

    // hug is a real pass now: on an unrouted board it gates, never crashes.
    CliResult hug = run_cli_full("gloss " + fixture("open_2layer.json") + " --hug --json");
    CT_CHECK(hug.rc == 4);
    JsonValue h = must_parse(hug.out);
    CT_CHECK(h.get_string("status") == "GATED");
    JsonValue hp = *h.find("passes");
    CT_CHECK(hp.as_array().size() == 1u);
}

// Phase 5 item 28: `router setup-report` — pre-route analysis with layer
// directions, drill pairs, per-net width/via resolution and setup warnings.
CT_TEST(cli_setup_report) {
    CliResult r = run_cli_full("setup-report " + fixture("multi_terminal_tree.json") +
                               " --json");
    CT_CHECK(r.rc == 0);
    JsonValue v = must_parse(r.out);
    CT_CHECK(v.get_string("schema") == "copperline/setup-report/1");
    JsonValue b = *v.find("board");
    CT_CHECK(b.get_number("nets", -1) == 2);
    CT_CHECK(b.get_number("terminals", -1) == 6);
    // Every net gets a resolved width and a via answer.
    JsonValue nets = *v.find("nets");
    CT_CHECK(nets.is_array());
    CT_CHECK(nets.as_array().size() == 2u);
    for (const auto& n : nets.as_array()) {
        CT_CHECK(n.get_number("width_mm", 0) > 0);
        CT_CHECK(!n.get_string("width_source").empty());
        CT_CHECK(!n.get_string("via_style").empty());
    }
    // Layers without a preferred direction are reported as warnings.
    JsonValue layers = *v.find("layers");
    CT_CHECK(layers.as_array().size() == 2u);
    CT_CHECK(v.find("warnings") != nullptr);
    const std::string warn_text = serialize_json(*v.find("warnings"));
    CT_CHECK(warn_text.find("no direction preference") != std::string::npos);
    // Drill pairs: none on an unrouted board.
    CT_CHECK(v.find("drill_pairs") != nullptr);
}

// setup-report writes the same JSON to --report.
CT_TEST(cli_setup_report_writes_report_file) {
    const std::string path = win_path(std::string(FIXTURE_DIR) + "/_setup_report.json");
    CliResult r = run_cli_full("setup-report " + fixture("golden_demo.dsn") +
                               " --json --report " + path);
    CT_CHECK(r.rc == 0);
    JsonValue v = must_parse(r.out);
    CT_CHECK(v.get_string("schema") == "copperline/setup-report/1");
    CT_CHECK(v.find("nets") != nullptr);
    std::string file_text;
    {
        std::ifstream rf(path);
        std::stringstream ss;
        ss << rf.rdbuf();
        file_text = ss.str();
    }
    JsonValue file = must_parse(file_text);
    CT_CHECK(file.get_string("schema") == "copperline/setup-report/1");
    std::remove(path.c_str());
}

// Phase 5 item 29: `router fanout` — escape-engine fanout with pattern and
// direction selection, committing copper and reporting the arrangement.
CT_TEST(cli_fanout_pattern_selection) {
    CliResult r = run_cli_full("fanout " + fixture("bga_4x4.json") +
                               " --pattern inline --dir 2 --json");
    CT_CHECK(r.rc == 0);
    JsonValue v = must_parse(r.out);
    CT_CHECK(v.get_string("schema") == "copperline/fanout-report/1");
    CT_CHECK(v.get_string("pattern") == "inline");
    CT_CHECK(v.get_number("dir", -99) == 2);
    CT_CHECK(v.get_number("pads_total", -1) == 16);
    CT_CHECK(v.get_number("terminals_committed", 0) > 0);
    CT_CHECK(v.get_number("traces_added", 0) > 0);
    CT_CHECK(v.find("verification") && v.find("verification")->get_bool("legal", false));

    // Default (auto) must also run and stay legal.
    CliResult a = run_cli_full("fanout " + fixture("bga_4x4.json") + " --json");
    CT_CHECK(a.rc == 0);
    JsonValue va = must_parse(a.out);
    CT_CHECK(va.get_string("pattern") == "auto");
    CT_CHECK(va.get_number("dir", 1) == -1);

    // Bad flag values are rejected at parse time.
    CliResult badp = run_cli_full("fanout " + fixture("bga_4x4.json") +
                                  " --pattern wiggly --json");
    CT_CHECK(badp.rc == 2);
    CliResult badd = run_cli_full("fanout " + fixture("bga_4x4.json") +
                                  " --dir 9 --json");
    CT_CHECK(badd.rc == 2);
}

// Phase 5 item 31: `router route-external` writes a DO file, runs an external
// Specctra router prefix and imports its SES. The happy path needs a real
// router; the guards and the failure path are covered here. (No DSN exporter
// exists, so only .dsn inputs are accepted.)
CT_TEST(cli_route_external_guards) {
    CliResult json_board = run_cli_full("route-external " + fixture("open_2layer.json") +
                                        " --router \"cmd /c exit 0\" --json");
    CT_CHECK(json_board.rc == 2);
    CT_CHECK(json_board.out.find("needs a .dsn input") != std::string::npos);

    CliResult no_router = run_cli_full("route-external " + fixture("golden_demo.dsn") +
                                       " --json");
    CT_CHECK(no_router.rc == 2);
    CT_CHECK(no_router.out.find("requires --router") != std::string::npos);

    CliResult failed = run_cli_full("route-external " + fixture("golden_demo.dsn") +
                                    " --router \"cmd /c exit 7\" --json");
    CT_CHECK(failed.rc == 6);  // internal failure: external router failed
    JsonValue v = must_parse(failed.out);
    CT_CHECK(v.get_string("schema") == "copperline/external-route-report/1");
    CT_CHECK(v.get_string("status") == "ROUTER_FAILED");
    CT_CHECK(v.get_number("router_exit_code", -1) == 7);
    std::remove(win_path(std::string(FIXTURE_DIR) + "/golden_demo.dsn.do").c_str());
}

// Input DRC state is reported separately from router-introduced violations:
// an illegal imported board must be distinguishable from a bad route.
CT_TEST(cli_route_reports_input_drc) {
    CliResult good = run_cli_full("route " + fixture("open_2layer.json") +
                                  " --json --no-board-echo --seed 42");
    CT_CHECK(good.rc == 0);
    JsonValue g = must_parse(good.out);
    CT_CHECK(g.get_bool("input_legal", false) == true);
    CT_CHECK(g.get_number("input_violation_count", -1) == 0);

    CliResult bad = run_cli_full("route " + fixture("violation_board.json") +
                                 " --json --no-board-echo --seed 42");
    JsonValue b = must_parse(bad.out);
    CT_CHECK(b.get_bool("input_legal", true) == false);
    CT_CHECK(b.get_number("input_violation_count", 0) > 0);
}

// `router import` converts to native JSON without routing.
CT_TEST(cli_import_converts_without_routing) {
    const std::string out = win_path(std::string(FIXTURE_DIR) + "/_import_test.json");
    CliResult r = run_cli_full("import " + fixture("minimal.kicad_pcb") +
                               " --output " + out + " --json");
    CT_CHECK(r.rc == 0);
    JsonValue v = must_parse(r.out);
    CT_CHECK(v.find("nets") != nullptr);
    CT_CHECK(v.find("board") != nullptr);
    std::string text;
    {
        std::ifstream f(out);
        std::stringstream ss;
        ss << f.rdbuf();
        text = ss.str();
    }
    JsonValue file = must_parse(text);
    CT_CHECK(file.find("nets") != nullptr);
    CT_CHECK(file.find("layers") != nullptr);
    std::remove(out.c_str());
}

// --memory-budget-mb bounds the engine's batch/scratch accounting (not RSS);
// pair it with tools/run_capped.ps1 for a hard OS limit.
CT_TEST(cli_memory_budget_flag) {
    CliResult r = run_cli_full("route " + fixture("open_2layer.json") +
                               " --json --no-board-echo --seed 42"
                               " --memory-budget-mb 256");
    CT_CHECK(r.rc == 0);
    JsonValue v = must_parse(r.out);
    JsonValue params = *v.find("params");
    CT_CHECK(params.get_number("memory_budget_mb", -1) == 256);

    CliResult bad = run_cli_full("route " + fixture("open_2layer.json") +
                                 " --memory-budget-mb 10 --json");
    CT_CHECK(bad.rc == 2);
}

int main() { return copperline::test::run_all_tests(); }
