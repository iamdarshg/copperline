// Copperline CLI: the only user interface (headless, agent-first).
//
// Every command supports --json (stdout = pure machine-readable JSON,
// diagnostics on stderr) and --quiet (errors only). Exit codes are stable
// and documented under `router capabilities`:
//
//   0  success (route complete / verify clean / analyze ok)
//   2  invalid input (missing file, bad JSON, bad flags, unknown command)
//   3  malformed design rules (bad net rules or --config content)
//   4  routing incomplete (finished, but terminals remain unconnected)
//   5  hard-rule violation (verify found clearance/width/via/keepout hits)
//   6  internal failure (unexpected exception)
//   7  search budget exhausted (node budget or --timeout hit with work left)
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "router/analyze.h"
#include "router/board.h"
#include "router/engine.h"
#include "router/escape.h"
#include "router/rules.h"
#include "router/verifier.h"

#ifndef COPPERLINE_VERSION
#define COPPERLINE_VERSION "0.1.0"
#endif

namespace copperline {
namespace {

constexpr int kOk = 0;
constexpr int kInvalidInput = 2;
constexpr int kBadRules = 3;
constexpr int kIncomplete = 4;
constexpr int kViolation = 5;
constexpr int kInternal = 6;
constexpr int kBudget = 7;

struct Flags {
    bool json = false;
    bool quiet = false;
    bool pretty = false;
    bool progress = false;  // NDJSON epoch events on stderr
    std::string config;
    unsigned seed = 42;
    int threads = 1;
    double timeout_s = 0;
    std::int64_t max_search_nodes = 200000;
    std::string output;
    std::string report;
    std::string board;
};

void warn(const Flags& f, const std::string& msg) {
    if (!f.quiet) std::cerr << "router: " << msg << "\n";
}

bool read_file(const std::string& path, std::string& out, std::string& err) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        err = "cannot open file: " + path;
        return false;
    }
    std::ostringstream ss;
    ss << file.rdbuf();
    out = ss.str();
    return true;
}

bool write_file(const std::string& path, const std::string& data, std::string& err) {
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file) {
        err = "cannot write file: " + path;
        return false;
    }
    file << data;
    if (!file) {
        err = "failed writing file: " + path;
        return false;
    }
    return true;
}

void emit_json(const Flags& f, const JsonValue& v) {
    std::cout << serialize_json(v, f.pretty);
    if (!f.pretty) std::cout << "\n";
}

JsonValue error_json(const std::string& code, const std::string& message) {
    JsonValue o = JsonValue::object();
    o["schema"] = "copperline/error/1";
    o["code"] = code;
    o["message"] = message;
    return o;
}

int fail(const Flags& f, int code, const std::string& err_code, const std::string& msg) {
    if (f.json) emit_json(f, error_json(err_code, msg));
    else std::cerr << "router: error: " << msg << "\n";
    return code;
}

bool parse_flags(const std::vector<std::string>& args, std::size_t start, Flags& f,
                 std::string& err) {
    auto need_value = [&](std::size_t i, const std::string& name, std::string& out) -> bool {
        if (i + 1 >= args.size()) {
            err = name + " needs a value";
            return false;
        }
        out = args[i + 1];
        return true;
    };
    for (std::size_t i = start; i < args.size();) {
        const std::string& a = args[i];
        std::string v;
        if (a == "--json") f.json = true;
        else if (a == "--quiet") f.quiet = true;
        else if (a == "--pretty") f.pretty = true;
        else if (a == "--progress") f.progress = true;
        else if (a == "--config") {
            if (!need_value(i, "--config", v)) return false;
            f.config = v;
            ++i;
        } else if (a == "--seed") {
            if (!need_value(i, "--seed", v)) return false;
            try {
                f.seed = static_cast<unsigned>(std::stoul(v));
            } catch (...) {
                err = "bad --seed value";
                return false;
            }
            ++i;
        } else if (a == "--threads") {
            if (!need_value(i, "--threads", v)) return false;
            try {
                f.threads = std::stoi(v);
            } catch (...) {
                err = "bad --threads value";
                return false;
            }
            if (f.threads < 1) {
                err = "--threads must be >= 1";
                return false;
            }
            ++i;
        } else if (a == "--timeout") {
            if (!need_value(i, "--timeout", v)) return false;
            try {
                f.timeout_s = std::stod(v);
            } catch (...) {
                err = "bad --timeout value";
                return false;
            }
            if (f.timeout_s < 0) {
                err = "--timeout must be >= 0";
                return false;
            }
            ++i;
        } else if (a == "--max-search-nodes") {
            if (!need_value(i, "--max-search-nodes", v)) return false;
            try {
                f.max_search_nodes = std::stoll(v);
            } catch (...) {
                err = "bad --max-search-nodes value";
                return false;
            }
            if (f.max_search_nodes < 1) {
                err = "--max-search-nodes must be >= 1";
                return false;
            }
            ++i;
        } else if (a == "--output") {
            if (!need_value(i, "--output", v)) return false;
            f.output = v;
            ++i;
        } else if (a == "--report") {
            if (!need_value(i, "--report", v)) return false;
            f.report = v;
            ++i;
        } else if (!a.empty() && a[0] == '-') {
            err = "unknown flag: " + a;
            return false;
        } else {
            if (!f.board.empty()) {
                err = "unexpected argument: " + a;
                return false;
            }
            f.board = a;
        }
        ++i;
    }
    return true;
}

struct LoadedBoard {
    Board board;
    std::vector<std::string> warnings;
};

bool load_board(const Flags& f, LoadedBoard& out, int& code, std::string& err_code,
                std::string& msg) {
    if (f.board.empty()) {
        code = kInvalidInput;
        err_code = "missing_board";
        msg = "no board file given";
        return false;
    }
    try {
        ImportResult r = import_board_auto(f.board);
        out.board = std::move(r.board);
        out.warnings = std::move(r.warnings);
        return true;
    } catch (const BoardError& e) {
        code = e.kind == InputKind::kRule ? kBadRules : kInvalidInput;
        err_code = e.kind == InputKind::kRule ? "malformed_rules" : "invalid_input";
        msg = e.what();
        return false;
    }
}

bool load_resolver(const Flags& f, const Board& board, RuleResolver& out, std::string& msg) {
    try {
        if (f.config.empty()) {
            out = RuleResolver::defaults_for(board);
        } else {
            std::string text, err;
            if (!read_file(f.config, text, err)) {
                msg = err;
                return false;
            }
            JsonValue cfg;
            try {
                cfg = parse_json(text);
            } catch (const std::exception& e) {
                msg = std::string("bad config JSON: ") + e.what();
                return false;
            }
            out = RuleResolver::from_config(board, cfg);
        }
        return true;
    } catch (const BoardError& e) {
        msg = e.what();
        return false;
    }
}

JsonValue capabilities_json() {
    JsonValue r = JsonValue::object();
    r["schema"] = "copperline/capabilities/1";
    r["name"] = "copperline";
    r["version"] = COPPERLINE_VERSION;
    r["phase"] = "prompt-4-recovery";
    JsonValue cmds = JsonValue::array();
    for (const char* c : {"capabilities", "analyze", "verify", "route", "escape", "benchmark"})
        cmds.as_array().push_back(JsonValue(c));
    r["commands"] = cmds;
    JsonValue planned = JsonValue::array();
    for (const char* c : {"explain-failure"})
        planned.as_array().push_back(JsonValue(c));
    r["planned_commands"] = planned;
    JsonValue formats = JsonValue::object();
    JsonValue sup = JsonValue::array();
    for (const auto& s : supported_formats()) sup.as_array().push_back(JsonValue(s));
    formats["supported"] = sup;
    JsonValue fut = JsonValue::array();
    for (const char* c : {"dsn", "ses", "ipc-2581", "gerber"})
        fut.as_array().push_back(JsonValue(c));
    formats["planned"] = fut;
    r["formats"] = formats;
    JsonValue codes = JsonValue::object();
    codes["0"] = "success";
    codes["2"] = "invalid_input";
    codes["3"] = "malformed_rules";
    codes["4"] = "routing_incomplete";
    codes["5"] = "hard_rule_violation";
    codes["6"] = "internal_failure";
    codes["7"] = "search_budget_exhausted";
    r["exit_codes"] = codes;
    JsonValue feat = JsonValue::object();
    feat["single_thread_astar"] = true;
    feat["current_aware"] = true;
    feat["voltage_aware"] = true;
    feat["pin_density"] = true;
    feat["parallel_routing"] = true;
    feat["deterministic_epochs"] = true;
    feat["congestion_negotiation"] = true;
    feat["fine_pitch_escape"] = true;
    feat["ripup_reroute"] = true;
    feat["meta_search"] = true;
    feat["recovery_modes"] = true;
    feat["optimizer"] = false;
    r["features"] = feat;
    return r;
}

int cmd_capabilities(const Flags& f) {
    if (f.json) {
        emit_json(f, capabilities_json());
    } else if (!f.quiet) {
        std::cout << "copperline " << COPPERLINE_VERSION << " (prompt-4 recovery)\n"
                  << "commands: capabilities, analyze, verify, route, escape, benchmark\n"
                  << "planned: explain-failure\n"
                  << "formats: json, kicad_pcb (.kicad_pcb)\n"
                  << "exit codes: 0 ok, 2 invalid input, 3 malformed rules, 4 incomplete,\n"
                  << "            5 rule violation, 6 internal failure, 7 budget exhausted\n";
    }
    return kOk;
}

int cmd_analyze(const Flags& f) {
    LoadedBoard lb;
    int code = 0;
    std::string err_code, msg;
    if (!load_board(f, lb, code, err_code, msg)) return fail(f, code, err_code, msg);
    RuleResolver resolver = RuleResolver::defaults_for(lb.board);
    if (!f.config.empty()) {
        if (!load_resolver(f, lb.board, resolver, msg))
            return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                        "malformed_rules", msg);
    }
    ElectricalContext ctx;
    AnalysisResult a = analyze_board(lb.board, resolver, ctx, lb.warnings);
    if (f.json) {
        emit_json(f, a.data);
    } else if (!f.quiet) {
        const Board& b = lb.board;
        std::cout << "board: " << nm_to_mm(b.width_nm) << " x " << nm_to_mm(b.height_nm) << "mm, "
                  << b.layers.size() << " layers, " << b.nets.size() << " nets, "
                  << b.terminals.size() << " terminals\n";
        std::cout << serialize_json(a.data, true);
    }
    for (const auto& w : lb.warnings) warn(f, "import: " + w);
    return kOk;
}

int cmd_verify(const Flags& f) {
    LoadedBoard lb;
    int code = 0;
    std::string err_code, msg;
    if (!load_board(f, lb, code, err_code, msg)) return fail(f, code, err_code, msg);
    RuleResolver resolver = RuleResolver::defaults_for(lb.board);
    if (!f.config.empty()) {
        if (!load_resolver(f, lb.board, resolver, msg))
            return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                        "malformed_rules", msg);
    }
    ElectricalContext ctx;
    BoardVerifier verifier;
    VerifyResult r = verifier.verify(lb.board, resolver, ctx);
    if (f.json) {
        emit_json(f, r.to_json());
    } else if (!f.quiet) {
        std::cout << (r.ok ? "PASS" : "FAIL") << ": "
                  << (r.connected ? "connected" : "NOT connected") << ", "
                  << (r.legal ? "legal" : "VIOLATIONS") << " (" << r.unconnected.size()
                  << " unconnected, " << r.violations.size() << " violations)\n";
        for (const auto& u : r.unconnected)
            std::cout << "  unconnected: net " << u.net_name << " terminal " << u.terminal << "\n";
        for (const auto& v : r.violations) std::cout << "  " << v.type << ": " << v.detail << "\n";
    }
    for (const auto& w : lb.warnings) warn(f, "import: " + w);
    if (!r.legal) return kViolation;
    if (!r.connected) return kIncomplete;
    return kOk;
}

int cmd_escape(const Flags& f) {
    LoadedBoard lb;
    int code = 0;
    std::string err_code, msg;
    if (!load_board(f, lb, code, err_code, msg)) return fail(f, code, err_code, msg);
    RuleResolver resolver = RuleResolver::defaults_for(lb.board);
    if (!f.config.empty()) {
        if (!load_resolver(f, lb.board, resolver, msg))
            return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                        "malformed_rules", msg);
    }
    for (const auto& w : lb.warnings) warn(f, "import: " + w);
    ElectricalContext ctx;
    EscapePlanner planner;
    EscapeResult r = planner.plan(lb.board, resolver, ctx);
    JsonValue rj = r.to_json();
    if (!f.report.empty()) {
        std::string err;
        if (!write_file(f.report, serialize_json(rj, true), err))
            return fail(f, kInvalidInput, "write_failed", err);
        rj["report"] = f.report;
    }
    if (f.json) {
        emit_json(f, rj);
    } else if (!f.quiet) {
        std::cout << "escape: " << rj.get_string("status") << " (" << r.pads_with_candidates
                  << "/" << r.pads_total << " pads, " << r.footprints.size() << " footprints)\n";
        for (const auto& fp : r.footprints) {
            std::cout << "  footprint " << fp.footprint.component << " (" << fp.footprint.pad_count
                      << " pads, pitch " << nm_to_mm(fp.footprint.pitch_nm) << "mm): ";
            for (TermId tid : fp.eligibility_order) std::cout << tid << " ";
            std::cout << "\n";
        }
    }
    if (r.pads_total == 0) return kOk;  // no fine-pitch: nothing to escape
    return r.pads_infeasible == 0 ? kOk : kIncomplete;
}

int cmd_route(const Flags& f) {
    LoadedBoard lb;
    int code = 0;
    std::string err_code, msg;
    if (!load_board(f, lb, code, err_code, msg)) return fail(f, code, err_code, msg);
    RuleResolver resolver = RuleResolver::defaults_for(lb.board);
    if (!f.config.empty()) {
        if (!load_resolver(f, lb.board, resolver, msg))
            return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                        "malformed_rules", msg);
    }
    for (const auto& w : lb.warnings) warn(f, "import: " + w);

    EngineOptions opt;
    opt.seed = f.seed;
    opt.threads = f.threads;
    opt.timeout_s = f.timeout_s;
    opt.astar.max_expansions = f.max_search_nodes;
    if (f.progress) {
        opt.progress = [](const JsonValue& ev) {
            std::cerr << serialize_json(ev) << "\n";
        };
    }
    RouterEngine engine(std::move(lb.board), std::move(resolver), opt);
    RouteReport report = engine.run();

    // Issue #2: independent CLI-side gate. No `router route` may exit 0
    // unless BoardVerifier.ok on the final committed copper, even if the
    // engine's bookkeeping claimed COMPLETE.
    {
        RuleResolver verify_resolver = RuleResolver::defaults_for(engine.committed());
        if (!f.config.empty()) {
            std::string cfg_text, cfg_err;
            if (read_file(f.config, cfg_text, cfg_err)) {
                try {
                    verify_resolver =
                        RuleResolver::from_config(engine.committed(), parse_json(cfg_text));
                } catch (...) {
                    // Config already validated above; keep defaults on
                    // unexpected re-parse failure (engine gate already ran).
                }
            }
        }
        ElectricalContext verify_ctx;
        BoardVerifier verifier;
        VerifyResult independent = verifier.verify(engine.committed(), verify_resolver, verify_ctx);
        apply_verifier_gate(report, independent);
    }

    JsonValue rj = report.to_json();
    JsonValue params = JsonValue::object();
    params["seed"] = static_cast<double>(f.seed);
    params["threads_requested"] = static_cast<double>(f.threads);
    params["timeout_s"] = f.timeout_s;
    params["max_search_nodes"] = static_cast<double>(f.max_search_nodes);
    rj["params"] = params;

    const Board& routed = engine.committed();
    if (!f.output.empty()) {
        std::string err;
        if (!write_file(f.output, serialize_json(board_to_json(routed), true), err))
            return fail(f, kInvalidInput, "write_failed", err);
        rj["output"] = f.output;
    } else {
        rj["board"] = board_to_json(routed);
    }
    if (!f.report.empty()) {
        std::string err;
        if (!write_file(f.report, serialize_json(rj, true), err))
            return fail(f, kInvalidInput, "write_failed", err);
    }
    if (f.json) {
        emit_json(f, rj);
    } else if (!f.quiet) {
        std::cout << "route: " << report.status << " (" << report.connected_terminals << "/"
                  << report.total_terminals << " terminals, " << report.stats.tasks_routed << "/"
                  << report.stats.tasks_total << " tasks, " << report.stats.via_count << " vias, "
                  << nm_to_mm(report.stats.length_nm) << "mm, " << report.stats.expansions_total
                  << " expansions, " << report.stats.time_ms << "ms, "
                  << report.stats.epochs_count << " epochs, " << report.stats.threads_used
                  << "/" << f.threads << " workers, hash " << report.board_hash << ")\n";
        for (const auto& fl : report.failures)
            std::cout << "  failed: net " << fl.net_name << " (" << fl.reason << ")\n";
    }

    if (report.status == "COMPLETE") {
        // Belt-and-braces: the report already carries the engine + CLI gates,
        // but never trust bookkeeping alone for the process exit code.
        if (!report.verification.ok) {
            if (!report.verification.legal) return kViolation;
            return kIncomplete;
        }
        return kOk;
    }
    if (report.status == "VIOLATION") return kViolation;
    // A non-COMPLETE report can still carry illegal copper (e.g. pre-existing
    // user copper): surface the hard-rule-violation code first, matching
    // `router verify` precedence (legal before connected).
    if (!report.verification.legal) return kViolation;
    if (report.status == "BUDGET_EXHAUSTED" || report.status == "TIMEOUT") return kBudget;
    return kIncomplete;
}

int cmd_benchmark(const Flags& f) {
    LoadedBoard lb;
    int code = 0;
    std::string err_code, msg;
    if (!load_board(f, lb, code, err_code, msg)) return fail(f, code, err_code, msg);
    RuleResolver base = RuleResolver::defaults_for(lb.board);
    if (!f.config.empty()) {
        if (!load_resolver(f, lb.board, base, msg))
            return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                        "malformed_rules", msg);
    }
    unsigned hw = std::thread::hardware_concurrency();
    if (hw == 0) hw = 4;
    int par_threads = f.threads > 1 ? f.threads : static_cast<int>(hw);

    auto run_once = [&](int threads) {
        Board b = lb.board;  // immutable input snapshot per run
        RuleResolver r = base;
        r.rebind(&b);  // point at this run's copy (engine rebinds to owned board anyway)
        EngineOptions opt;
        opt.seed = f.seed;
        opt.threads = threads;
        opt.timeout_s = f.timeout_s;
        opt.astar.max_expansions = f.max_search_nodes;
        auto t0 = std::chrono::steady_clock::now();
        RouterEngine engine(std::move(b), std::move(r), opt);
        RouteReport rep = engine.run();
        auto t1 = std::chrono::steady_clock::now();
        std::int64_t wall =
            std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        return std::make_pair(rep, wall);
    };

    auto [rep1, wall1] = run_once(1);
    auto [repN, wallN] = run_once(par_threads);

    JsonValue r = JsonValue::object();
    r["schema"] = "copperline/benchmark-report/1";
    r["board"] = f.board;
    r["seed"] = static_cast<double>(f.seed);
    r["threads_available"] = static_cast<double>(hw);
    auto side = [](const RouteReport& rep, std::int64_t wall, int threads) {
        JsonValue o = JsonValue::object();
        o["threads"] = static_cast<double>(threads);
        o["status"] = rep.status;
        o["wall_ms"] = static_cast<double>(wall);
        o["engine_ms"] = static_cast<double>(rep.stats.time_ms);
        o["expansions"] = static_cast<double>(rep.stats.expansions_total);
        o["tasks_routed"] = static_cast<double>(rep.stats.tasks_routed);
        o["tasks_total"] = static_cast<double>(rep.stats.tasks_total);
        o["epochs"] = static_cast<double>(rep.stats.epochs_count);
        o["threads_used"] = static_cast<double>(rep.stats.threads_used);
        o["board_hash"] = rep.board_hash;
        return o;
    };
    r["single"] = side(rep1, wall1, 1);
    r["parallel"] = side(repN, wallN, par_threads);
    r["identical_geometry"] = rep1.board_hash == repN.board_hash;
    r["speedup"] = wallN > 0 ? static_cast<double>(wall1) / static_cast<double>(wallN) : 0.0;
    if (f.json) {
        emit_json(f, r);
    } else if (!f.quiet) {
        std::cout << "benchmark: single " << wall1 << "ms, parallel(" << par_threads << ") "
                  << wallN << "ms, speedup " << serialize_json(r["speedup"]) << ", geometry "
                  << (rep1.board_hash == repN.board_hash ? "identical" : "DIFFERS") << "\n";
    }
    return kOk;
}

int run(const std::vector<std::string>& args) {
    if (args.size() < 2) {
        std::cerr << "usage: router <capabilities|analyze|verify|route|escape|benchmark> [board] [flags]\n"
                     "       router --help | router --version\n";
        return kInvalidInput;
    }
    if (args[1] == "--help" || args[1] == "-h" || args[1] == "help") {
        std::cout << "copperline " << COPPERLINE_VERSION
                  << " - headless PCB autorouter for agents\n\n"
                     "  router capabilities [--json]\n"
                     "  router analyze <board> [--json] [--config cfg.json]\n"
                     "  router verify <board> [--json] [--config cfg.json]\n"
                      "  router route <board> [--json] [--config cfg.json] [--seed N]\n"
                      "                       [--threads N] [--timeout S] [--max-search-nodes N]\n"
                      "                       [--output routed.json] [--report report.json]\n"
                      "  router escape <board> [--json] [--config cfg.json] [--report report.json]\n"
                      "  router benchmark <board> [--json] [--config cfg.json] [--seed N]\n"
                      "                       [--threads N] [--timeout S] [--max-search-nodes N]\n\n"
                      "route flags: --progress emits NDJSON epoch events on stderr\n"
                      "boards: .json (native) or .kicad_pcb\n";
        return kOk;
    }
    if (args[1] == "--version") {
        std::cout << COPPERLINE_VERSION << "\n";
        return kOk;
    }
    const std::string cmd = args[1];
    Flags f;
    std::string err;
    if (!parse_flags(args, 2, f, err)) {
        Flags jf;
        for (const auto& a : args)
            if (a == "--json") jf.json = true;
        return fail(jf, kInvalidInput, "bad_flags", err);
    }
    try {
        if (cmd == "capabilities") return cmd_capabilities(f);
        if (cmd == "analyze") return cmd_analyze(f);
        if (cmd == "verify") return cmd_verify(f);
        if (cmd == "route") return cmd_route(f);
        if (cmd == "escape") return cmd_escape(f);
        if (cmd == "benchmark") return cmd_benchmark(f);
        if (cmd == "explain-failure")
            return fail(f, kInvalidInput, "not_implemented",
                        "command 'explain-failure' lands in Prompt 5");
        return fail(f, kInvalidInput, "unknown_command",
                    "unknown command '" + cmd + "' (see router --help)");
    } catch (const BoardError& e) {
        return fail(f, e.kind == InputKind::kRule ? kBadRules : kInvalidInput,
                    e.kind == InputKind::kRule ? "malformed_rules" : "invalid_input", e.what());
    } catch (const std::exception& e) {
        return fail(f, kInternal, "internal_failure", e.what());
    }
}

}  // namespace
}  // namespace copperline

int main(int argc, char** argv) {
    std::vector<std::string> args(argv, argv + argc);
    return copperline::run(args);
}
