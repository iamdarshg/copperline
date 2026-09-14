// End-to-end CLI tests: drive the built `router` binary on fixtures and
// assert exit codes, JSON purity and stable schemas.
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "helpers.h"
#include "router/json.h"

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

std::string run_cli(const std::string& args, int& rc) {
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
    rc = std::system(cmd.c_str());
    std::ifstream f(tmp, std::ios::binary);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
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

std::string temp_path(const std::string& name) {
    return win_path((std::filesystem::temp_directory_path() / name).string());
}

}  // namespace

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
    CT_CHECK(v.get_string("phase") == "prompt-3-parallel");
    CT_CHECK(v.find("features")->get_bool("parallel_routing", false));
    bool has_benchmark = false;
    for (const auto& c : v.find("commands")->as_array())
        if (c.as_string() == "benchmark") has_benchmark = true;
    CT_CHECK(has_benchmark);
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
    int rc = 0;
    std::string out = run_cli("route " + fixture("obstacle_detour.json") +
                                  " --json --max-search-nodes 1",
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

int main() { return copperline::test::run_all_tests(); }
