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
#include "router/dsn.h"
#include "router/engine.h"
#include "router/escape.h"
#include "router/maturity.h"
#include "router/rules.h"
#include "router/tuning.h"
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
    int threads = 0;  // Prompt 5: 0 = auto (hardware_concurrency, bounded)
    int route_k = 0;  // issue #8: 0 = auto (maturity route-K), else 1..15 wins
    bool no_impact = false;  // issue #9: disable obstruction scoring
    bool impact_mlp = false;  // issue #9: tiny fixed-weight MLP scorer
    double timeout_s = 0;
    std::int64_t max_search_nodes = 200000;
    bool no_hierarchy = false;  // issue #10: disable coarse-to-fine guidance
    int recovery_depth = 2;     // issue #22: 1 = one-ply, default 2
    int recovery_beam = 4;      // issue #22: beam width, default 4
    std::int64_t max_multiply_nodes = 64;  // issue #22: deeper node budget
    bool recovery_depth_set = false;
    bool recovery_beam_set = false;
    bool max_multiply_nodes_set = false;
    bool no_tuning = false;  // issue #15: disable post-route length tuning
    bool no_optimizer = false;  // Prompt 5: disable cleanup optimizer
    double tuning_amplitude_mm = 0;  // issue #15: 0 = config default
    double tuning_pitch_mm = 0;      // issue #15: 0 = config default
    double tuning_max_added_mm = 0;  // issue #15: 0 = config default
    std::string output;
    std::string report;
    std::string board;
    std::string routes;  // Prompt 5: `verify <board> --routes <routed>` merge file
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
            // Prompt 5: 0 = auto (all CPUs via hardware_concurrency);
            // explicit N >= 1 wins. Total stays within the 2048MB budget
            // via bounded batch widths (see EngineOptions).
            if (f.threads < 0) {
                err = "--threads must be >= 0 (0 = auto)";
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
        } else if (a == "--route-k") {
            if (!need_value(i, "--route-k", v)) return false;
            try {
                f.route_k = std::stoi(v);
            } catch (...) {
                err = "bad --route-k value";
                return false;
            }
            if (f.route_k < 0 || f.route_k > 15) {
                err = "--route-k must be in [0, 15] (0 = auto)";
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
        } else if (a == "--recovery-depth") {
            if (!need_value(i, "--recovery-depth", v)) return false;
            try {
                f.recovery_depth = std::stoi(v);
            } catch (...) {
                err = "bad --recovery-depth value";
                return false;
            }
            if (f.recovery_depth < 1 || f.recovery_depth > 8) {
                err = "--recovery-depth must be in [1, 8] (1 = one-ply)";
                return false;
            }
            f.recovery_depth_set = true;
            ++i;
        } else if (a == "--recovery-beam") {
            if (!need_value(i, "--recovery-beam", v)) return false;
            try {
                f.recovery_beam = std::stoi(v);
            } catch (...) {
                err = "bad --recovery-beam value";
                return false;
            }
            if (f.recovery_beam < 1 || f.recovery_beam > 8) {
                err = "--recovery-beam must be in [1, 8]";
                return false;
            }
            f.recovery_beam_set = true;
            ++i;
        } else if (a == "--max-multiply-nodes") {
            if (!need_value(i, "--max-multiply-nodes", v)) return false;
            try {
                f.max_multiply_nodes = std::stoll(v);
            } catch (...) {
                err = "bad --max-multiply-nodes value";
                return false;
            }
            if (f.max_multiply_nodes < 0) {
                err = "--max-multiply-nodes must be >= 0";
                return false;
            }
            f.max_multiply_nodes_set = true;
            ++i;
        } else if (a == "--no-hierarchy") {
            f.no_hierarchy = true;
        } else if (a == "--no-tuning") {
            f.no_tuning = true;  // issue #15
        } else if (a == "--no-optimizer") {
            f.no_optimizer = true;  // Prompt 5
        } else if (a == "--tuning-amplitude-mm") {
            if (!need_value(i, "--tuning-amplitude-mm", v)) return false;
            try {
                f.tuning_amplitude_mm = std::stod(v);
            } catch (...) {
                err = "bad --tuning-amplitude-mm value";
                return false;
            }
            if (f.tuning_amplitude_mm <= 0) {
                err = "--tuning-amplitude-mm must be > 0";
                return false;
            }
            ++i;
        } else if (a == "--tuning-pitch-mm") {
            if (!need_value(i, "--tuning-pitch-mm", v)) return false;
            try {
                f.tuning_pitch_mm = std::stod(v);
            } catch (...) {
                err = "bad --tuning-pitch-mm value";
                return false;
            }
            if (f.tuning_pitch_mm <= 0) {
                err = "--tuning-pitch-mm must be > 0";
                return false;
            }
            ++i;
        } else if (a == "--tuning-max-added-mm") {
            if (!need_value(i, "--tuning-max-added-mm", v)) return false;
            try {
                f.tuning_max_added_mm = std::stod(v);
            } catch (...) {
                err = "bad --tuning-max-added-mm value";
                return false;
            }
            if (f.tuning_max_added_mm <= 0) {
                err = "--tuning-max-added-mm must be > 0";
                return false;
            }
            ++i;
        } else if (a == "--no-impact") {
            f.no_impact = true;
        } else if (a == "--impact-mlp") {
            f.impact_mlp = true;
        } else if (a == "--output") {
            if (!need_value(i, "--output", v)) return false;
            f.output = v;
            ++i;
        } else if (a == "--report") {
            if (!need_value(i, "--report", v)) return false;
            f.report = v;
            ++i;
        } else if (a == "--routes") {
            if (!need_value(i, "--routes", v)) return false;
            f.routes = v;
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

// Prompt 5: extend (never replace) the sidecar system. A "nets" object in
// --config carries current/voltage intent for formats (DSN/KiCad) that lack
// it. Applied to the imported board before the resolver is built.
bool apply_sidecar(const Flags& f, Board& board, std::string& msg) {
    if (f.config.empty()) return true;
    std::string text, err;
    if (!read_file(f.config, text, err)) {
        msg = err;
        return false;
    }
    try {
        JsonValue cfg = parse_json(text);
        apply_sidecar_nets(board, cfg);
    } catch (const BoardError& e) {
        msg = e.what();
        return false;
    } catch (const std::exception& e) {
        msg = std::string("bad config JSON: ") + e.what();
        return false;
    }
    return true;
}
// Issue #14: optional "maturity" object inside --config JSON overrides the
// adaptive-budget thresholds/caps. The resolver load already validated the
// file above, so this re-read only applies the maturity section.
bool load_maturity_opt(const Flags& f, EngineOptions& opt, std::string& msg) {
    if (f.config.empty()) return true;
    std::string text, err;
    if (!read_file(f.config, text, err)) return true;
    try {
        JsonValue cfg = parse_json(text);
        if (const JsonValue* m = cfg.find("maturity")) {
            if (!apply_maturity_json(opt.maturity, *m, msg)) return false;
        }
        // Issue #22: optional "recovery" object {depth, beam, max_nodes}.
        if (const JsonValue* r = cfg.find("recovery")) {
            if (!r->is_object()) {
                msg = "recovery: expected an object";
                return false;
            }
            if (r->has("depth")) {
                const JsonValue* v = r->find("depth");
                if (!v->is_number()) {
                    msg = "recovery.depth: expected a number";
                    return false;
                }
                long long d = static_cast<long long>(v->as_number());
                if (d < 1 || d > 8) {
                    msg = "recovery.depth: expected a number in [1, 8]";
                    return false;
                }
                opt.recovery_depth = static_cast<int>(d);
            }
            if (r->has("beam")) {
                const JsonValue* v = r->find("beam");
                if (!v->is_number()) {
                    msg = "recovery.beam: expected a number";
                    return false;
                }
                long long b = static_cast<long long>(v->as_number());
                if (b < 1 || b > 8) {
                    msg = "recovery.beam: expected a number in [1, 8]";
                    return false;
                }
                opt.recovery_beam = static_cast<int>(b);
            }
            if (r->has("max_nodes")) {
                const JsonValue* v = r->find("max_nodes");
                if (!v->is_number()) {
                    msg = "recovery.max_nodes: expected a number";
                    return false;
                }
                long long n = static_cast<long long>(v->as_number());
                if (n < 0 || n > 1000000) {
                    msg = "recovery.max_nodes: out of range";
                    return false;
                }
                opt.max_multiply_nodes = n;
            }
        }
        // Issue #15: optional "tuning" object (amplitude/pitch/max-added,
        // candidate/region caps, style, symmetric_pairs, enabled).
        if (const JsonValue* t = cfg.find("tuning")) {
            if (!tuning_config_from_json(cfg, opt.tuning, msg)) return false;
            (void)t;
        }
    } catch (const std::exception& e) {
        msg = std::string("bad config JSON: ") + e.what();
        return false;
    }
    return true;
}

JsonValue capabilities_json() {
    JsonValue r = JsonValue::object();
    r["schema"] = "copperline/capabilities/1";
    r["name"] = "copperline";
    r["version"] = COPPERLINE_VERSION;
    r["phase"] = "prompt-5-release";
    JsonValue cmds = JsonValue::array();
    for (const char* c : {"capabilities", "analyze", "verify", "route", "escape",
                          "benchmark", "explain-failure"})
        cmds.as_array().push_back(JsonValue(c));
    r["commands"] = cmds;
    JsonValue planned = JsonValue::array();
    r["planned_commands"] = planned;
    JsonValue formats = JsonValue::object();
    JsonValue sup = JsonValue::array();
    for (const auto& s : supported_formats()) sup.as_array().push_back(JsonValue(s));
    formats["supported"] = sup;
    JsonValue fut = JsonValue::array();
    formats["planned"] = fut;
    formats["notes"] =
        "json: native round-trip; "
        "kicad_pcb: ingest + export (outline, nets + net classes, footprints/pads, "
        "tracks, vias, copper zones as planes, keepout rule areas); "
        "dsn: Specctra subset (structure/network/library/placement) + SES route "
        "export; planes + copper_pour map to planes when net+layer resolve, "
        "otherwise skipped with explicit warning (never silent); "
        "gerber: RS-274X subset (apertures D10+, draws/flashes/regions, polarity, "
        "inch/mm, zero-suppression modes) on copper layers into single-net "
        "(COPPER) pads/traces/pours + obstacles; outline from profile layer "
        "when present, else copper bbox + 1mm; no embedded netlist; "
        "ipc-2581: IPC-2581C flat-subset XML (Datum, Layers, Nets, Components/"
        "Pads, Traces/Vias, Profiles, NetClass widths/clearances); see README "
        "honest-limits sections for each format";
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
    feat["ampacity_model"] = true;
    feat["impedance_aware"] = true;
    feat["voltage_aware"] = true;
    feat["pin_density"] = true;
    feat["parallel_routing"] = true;
    feat["deterministic_epochs"] = true;
    feat["congestion_negotiation"] = true;
    feat["fine_pitch_escape"] = true;
    feat["plane_routing"] = true;
    feat["hierarchical_guidance"] = true;  // issue #10: coarse-to-fine A* guidance
    feat["maturity_adaptive_budgets"] = true;  // issue #14: maturity-driven budgets
    feat["route_portfolio"] = true;  // issue #8: diverse K-alternative portfolios
    feat["future_obstruction_scoring"] = true;  // issue #9: impact scorer
    feat["multiply_recovery"] = true;  // issue #22: bounded multi-ply beam search
    feat["length_tuning"] = true;  // issue #15: post-route meander stage
    feat["ripup_reroute"] = true;
    feat["meta_search"] = true;
    feat["recovery_modes"] = true;
    feat["optimizer"] = true;
    feat["cleanup_optimizer"] = true;
    feat["explain_failure"] = true;
    feat["dsn_import"] = true;
    feat["ses_export"] = true;
    feat["kicad_export"] = true;
    feat["gerber_import"] = true;
    feat["ipc2581_import"] = true;
    feat["copper_pours"] = true;
    feat["sidecar_nets"] = true;
    feat["golden_suite"] = true;
    JsonValue unsup = JsonValue::array();
    r["unsupported"] = unsup;
    r["features"] = feat;
    return r;
}

int cmd_capabilities(const Flags& f) {
    if (f.json) {
        emit_json(f, capabilities_json());
    } else if (!f.quiet) {
        std::cout << "copperline " << COPPERLINE_VERSION << " (prompt-5 release)\n"
                  << "commands: capabilities, analyze, verify, route, escape, benchmark,\n"
                  << "          explain-failure\n"
                  << "formats: json, kicad_pcb (.kicad_pcb), dsn (.dsn),\n"
                  << "         gerber (.gbr/.gtl/.gbl/...), ipc-2581 (.xml)\n"
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
    if (!apply_sidecar(f, lb.board, msg))
        return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                    "malformed_rules", msg);
    RuleResolver resolver = RuleResolver::defaults_for(lb.board);
    if (!f.config.empty()) {
        if (!load_resolver(f, lb.board, resolver, msg))
            return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                        "malformed_rules", msg);
    }
    ElectricalContext ctx = resolver.defaultContext();
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
    if (!apply_sidecar(f, lb.board, msg))
        return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                    "malformed_rules", msg);
    // Prompt 5 DSN+SES workflow: `verify board.dsn --routes routed.ses`.
    if (!f.routes.empty()) {
        try {
            for (const auto& w : merge_routes_file(lb.board, f.routes)) warn(f, "routes: " + w);
        } catch (const BoardError& e) {
            return fail(f, e.kind == InputKind::kRule ? kBadRules : kInvalidInput,
                        e.kind == InputKind::kRule ? "malformed_rules" : "invalid_input",
                        e.what());
        }
    }
    RuleResolver resolver = RuleResolver::defaults_for(lb.board);
    if (!f.config.empty()) {
        if (!load_resolver(f, lb.board, resolver, msg))
            return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                        "malformed_rules", msg);
    }
    ElectricalContext ctx = resolver.defaultContext();
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
    if (!apply_sidecar(f, lb.board, msg))
        return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                    "malformed_rules", msg);
    RuleResolver resolver = RuleResolver::defaults_for(lb.board);
    if (!f.config.empty()) {
        if (!load_resolver(f, lb.board, resolver, msg))
            return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                        "malformed_rules", msg);
    }
    for (const auto& w : lb.warnings) warn(f, "import: " + w);
    ElectricalContext ctx = resolver.defaultContext();
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
    if (!apply_sidecar(f, lb.board, msg))
        return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                    "malformed_rules", msg);
    RuleResolver resolver = RuleResolver::defaults_for(lb.board);
    if (!f.config.empty()) {
        if (!load_resolver(f, lb.board, resolver, msg))
            return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                        "malformed_rules", msg);
    }
    for (const auto& w : lb.warnings) warn(f, "import: " + w);

    EngineOptions opt;
    opt.seed = f.seed;
    // Issue #3: default to all CPUs (hardware_concurrency); explicit
    // --threads wins. Batch formation stays thread-independent so geometry
    // is deterministic; memory stays bounded (sparse interference + batch
    // width min(8, floor(2048MB/per-task))).
    {
        int eff = f.threads > 0 ? f.threads : static_cast<int>(std::thread::hardware_concurrency());
        if (eff < 1) eff = 4;
        opt.threads = eff;
    }
    opt.timeout_s = f.timeout_s;
    opt.astar.max_expansions = f.max_search_nodes;
    opt.hierarchy.enabled = !f.no_hierarchy;  // issue #10
    opt.impact.enabled = !f.no_impact;        // issue #9
    opt.impact.use_mlp = f.impact_mlp;        // issue #9: fixed MLP + fallback
    opt.recovery_depth = f.recovery_depth;    // issue #22
    opt.recovery_beam = f.recovery_beam;      // issue #22
    opt.max_multiply_nodes = f.max_multiply_nodes;  // issue #22
    // Issue #15: explicit tuning CLI wins over --config; --no-tuning off.
    if (f.no_tuning) opt.tuning.enabled = false;
    // Prompt 5: cleanup optimizer on by default (gated on COMPLETE inside).
    if (f.no_optimizer) opt.optimizer.enabled = false;
    if (f.tuning_amplitude_mm > 0) opt.tuning.amplitude_nm = mm_to_nm(f.tuning_amplitude_mm);
    if (f.tuning_pitch_mm > 0) opt.tuning.pitch_nm = mm_to_nm(f.tuning_pitch_mm);
    if (f.tuning_max_added_mm > 0) opt.tuning.max_added_nm = mm_to_nm(f.tuning_max_added_mm);
    {
        std::string merr;
        if (!load_maturity_opt(f, opt, merr))
            return fail(f, kBadRules, "malformed_rules", merr);
        // Issue #8: explicit --route-k wins over the maturity cap.
        if (f.route_k >= 1) opt.maturity.caps.max_route_k = f.route_k;
        // Issue #22: explicit CLI depth/beam/nodes win over --config.
        if (f.recovery_depth_set) opt.recovery_depth = f.recovery_depth;
        if (f.recovery_beam_set) opt.recovery_beam = f.recovery_beam;
        if (f.max_multiply_nodes_set) opt.max_multiply_nodes = f.max_multiply_nodes;
    }
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
        ElectricalContext verify_ctx = verify_resolver.defaultContext();
        BoardVerifier verifier;
        VerifyResult independent = verifier.verify(engine.committed(), verify_resolver, verify_ctx);
        apply_verifier_gate(report, independent);
    }

    JsonValue rj = report.to_json();
    JsonValue params = JsonValue::object();
    params["seed"] = static_cast<double>(f.seed);
    params["threads_requested"] = static_cast<double>(opt.threads);
    params["threads_flag"] = static_cast<double>(f.threads);  // 0 = auto
    params["timeout_s"] = f.timeout_s;
    params["max_search_nodes"] = static_cast<double>(f.max_search_nodes);
    params["hierarchy_enabled"] = !f.no_hierarchy;  // issue #10
    params["maturity_enabled"] = opt.maturity.enabled;  // issue #14
    params["route_k_requested"] = static_cast<double>(f.route_k);  // issue #8: 0=auto
    params["route_k_cap"] = static_cast<double>(opt.maturity.caps.max_route_k);
    params["impact_enabled"] = !f.no_impact;  // issue #9
    params["impact_scorer"] = f.impact_mlp ? "mlp" : "weighted";  // issue #9
    params["recovery_depth"] = static_cast<double>(opt.recovery_depth);  // issue #22
    params["recovery_beam"] = static_cast<double>(opt.recovery_beam);    // issue #22
    params["max_multiply_nodes"] = static_cast<double>(opt.max_multiply_nodes);  // #22
    params["tuning_enabled"] = !f.no_tuning && opt.tuning.enabled;  // issue #15
    params["tuning_amplitude_mm"] = nm_to_mm(opt.tuning.amplitude_nm);
    params["tuning_pitch_mm"] = nm_to_mm(opt.tuning.pitch_nm);
    params["tuning_max_added_mm"] = nm_to_mm(opt.tuning.max_added_nm);
    params["optimizer_enabled"] = !f.no_optimizer;  // Prompt 5
    // Prompt 5 diagnostics: effective threads/memory actually used.
    params["threads_effective"] = static_cast<double>(opt.threads);
    params["memory_budget_mb"] =
        static_cast<double>(opt.memory_budget_bytes / (1024ULL * 1024ULL));
    params["optimizer_max_candidates"] =
        static_cast<double>(opt.optimizer.max_candidates);
    rj["params"] = params;

    const Board& routed = engine.committed();
    auto has_suffix = [](const std::string& s, const std::string& suf) {
        return s.size() >= suf.size() && s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
    };
    if (!f.output.empty()) {
        std::string err;
        bool ok = false;
        if (has_suffix(f.output, ".ses")) {
            ok = write_file(f.output, board_to_ses(routed, f.board), err);
        } else if (has_suffix(f.output, ".kicad_pcb")) {
            ok = write_file(f.output, board_to_kicad_pcb(routed), err);
        } else {
            ok = write_file(f.output, serialize_json(board_to_json(routed), true), err);
        }
        if (!ok) return fail(f, kInvalidInput, "write_failed", err);
        rj["output"] = f.output;
        rj["output_format"] = has_suffix(f.output, ".ses")
                                  ? "ses"
                                  : (has_suffix(f.output, ".kicad_pcb") ? "kicad_pcb" : "json");
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
                   << "/" << opt.threads << " workers, hash " << report.board_hash << ")\n";
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
    // Prompt 5: explicit --threads wins; 0/default = all CPUs. Bound by the
    // 2048MB router budget via fixed batch widths (never by thread count).
    int par_threads = f.threads >= 1 ? f.threads : static_cast<int>(hw);
    // Issue #14: benchmark runs share the route maturity config (if any).
    EngineOptions bench_base;
    bench_base.seed = f.seed;
    bench_base.timeout_s = f.timeout_s;
    bench_base.astar.max_expansions = f.max_search_nodes;
    bench_base.hierarchy.enabled = !f.no_hierarchy;  // issue #10
    bench_base.recovery_depth = f.recovery_depth;    // issue #22
    bench_base.recovery_beam = f.recovery_beam;      // issue #22
    bench_base.max_multiply_nodes = f.max_multiply_nodes;  // issue #22
    {
        std::string merr;
        if (!load_maturity_opt(f, bench_base, merr))
            return fail(f, kBadRules, "malformed_rules", merr);
        if (f.recovery_depth_set) bench_base.recovery_depth = f.recovery_depth;
        if (f.recovery_beam_set) bench_base.recovery_beam = f.recovery_beam;
        if (f.max_multiply_nodes_set) bench_base.max_multiply_nodes = f.max_multiply_nodes;
    }

    auto run_once = [&](int threads) {
        Board b = lb.board;  // immutable input snapshot per run
        RuleResolver r = base;
        r.rebind(&b);  // point at this run's copy (engine rebinds to owned board anyway)
        EngineOptions opt = bench_base;
        opt.threads = threads;
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
        o["result_category"] = rep.result_category;
        o["wall_ms"] = static_cast<double>(wall);
        o["engine_ms"] = static_cast<double>(rep.stats.time_ms);
        o["expansions"] = static_cast<double>(rep.stats.expansions_total);
        o["tasks_routed"] = static_cast<double>(rep.stats.tasks_routed);
        o["tasks_total"] = static_cast<double>(rep.stats.tasks_total);
        o["connected_terminals"] = static_cast<double>(rep.connected_terminals);
        o["total_terminals"] = static_cast<double>(rep.total_terminals);
        o["connected_pct"] = rep.total_terminals > 0
                                 ? 100.0 * rep.connected_terminals / rep.total_terminals
                                 : 100.0;
        o["epochs"] = static_cast<double>(rep.stats.epochs_count);
        o["ripup_generations"] = static_cast<double>(rep.recovery.generations);
        o["accepted"] = static_cast<double>(rep.stats.candidates_accepted);
        o["rejected"] = static_cast<double>(rep.stats.candidates_rejected);
        o["threads_used"] = static_cast<double>(rep.stats.threads_used);
        o["via_count"] = static_cast<double>(rep.stats.via_count);
        o["length_mm"] = nm_to_mm(rep.stats.length_nm);
        o["optimizer_applied"] = static_cast<double>(rep.optimizer.applied);
        o["optimizer_reverted"] = static_cast<double>(rep.optimizer.reverted);
        o["board_hash"] = rep.board_hash;
        return o;
    };
    r["single"] = side(rep1, wall1, 1);
    r["parallel"] = side(repN, wallN, par_threads);
    r["identical_geometry"] = rep1.board_hash == repN.board_hash;
    r["speedup"] = wallN > 0 ? static_cast<double>(wall1) / static_cast<double>(wallN) : 0.0;
    // Prompt 5 resource accounting (honest bounds, not sampled RSS).
    r["memory_budget_mb"] =
        static_cast<double>(bench_base.memory_budget_bytes / (1024ULL * 1024ULL));
    r["batch_width_cap"] = 8.0;  // kParallelBatchSize: threads never widen batches
    r["cpu_threads"] = static_cast<double>(hw);
    if (f.json) {
        emit_json(f, r);
    } else if (!f.quiet) {
        std::cout << "benchmark: single " << wall1 << "ms, parallel(" << par_threads << ") "
                  << wallN << "ms, speedup " << serialize_json(r["speedup"]) << ", geometry "
                  << (rep1.board_hash == repN.board_hash ? "identical" : "DIFFERS") << "\n";
    }
    return kOk;
}

int cmd_explain_failure(const Flags& f) {
    // Prompt 5: render a route report's per-connection failures for agents.
    // Input is a route-report.json file (from `route --report` or --output).
    if (f.board.empty())
        return fail(f, kInvalidInput, "missing_report", "no route report file given");
    std::string text, err;
    if (!read_file(f.board, text, err))
        return fail(f, kInvalidInput, "invalid_input", err);
    JsonValue rep;
    try {
        rep = parse_json(text);
    } catch (const std::exception& e) {
        return fail(f, kInvalidInput, "invalid_input",
                    std::string("bad report JSON: ") + e.what());
    }
    if (rep.get_string("schema") != "copperline/route-report/1")
        return fail(f, kInvalidInput, "invalid_input",
                    "not a copperline/route-report/1 file (got '" +
                        rep.get_string("schema") + "')");
    JsonValue out = JsonValue::object();
    out["schema"] = "copperline/failure-explanation/1";
    out["status"] = rep.get_string("status");
    out["result_category"] = rep.get_string("result_category");
    out["connected_terminals"] = rep.get_number("connected_terminals", 0);
    out["total_terminals"] = rep.get_number("total_terminals", 0);
    out["remaining_terminals"] = rep.get_number("remaining_terminals", 0);
    out["board_hash"] = rep.get_string("board_hash");
    const JsonValue* fails = rep.find("failures");
    JsonValue list = JsonValue::array();
    std::size_t nfail = (fails && fails->is_array()) ? fails->as_array().size() : 0;
    if (fails && fails->is_array()) {
        for (const auto& fl : fails->as_array()) {
            JsonValue e = JsonValue::object();
            e["net"] = fl.get_number("net", -1);
            e["net_name"] = fl.get_string("net_name");
            e["terminal_a"] = fl.get_number("terminal_a", -1);
            e["terminal_b"] = fl.get_number("terminal_b", -1);
            e["src_component"] = fl.get_string("src_component");
            e["src_pin"] = fl.get_string("src_pin");
            e["dst_component"] = fl.get_string("dst_component");
            e["dst_pin"] = fl.get_string("dst_pin");
            e["reason"] = fl.get_string("reason");
            e["category"] = fl.get_string("category", rep.get_string("result_category"));
            e["required_width_mm"] = fl.get_number("required_width_mm", 0);
            e["width_source"] = fl.get_string("width_source");
            e["required_current_a"] = fl.get_number("required_current_a", 0);
            e["centre_depth"] = fl.get_number("centre_depth", -1);
            e["pin_density_per_mm2"] = fl.get_number("pin_density_per_mm2", 0);
            e["candidate_count"] = fl.get_number("candidate_count", 0);
            e["expansions"] = fl.get_number("expansions", 0);
            e["ripup_attempts"] = fl.get_number("ripup_attempts", 0);
            e["best_partial"] = fl.get_string("best_partial");
            if (const JsonValue* b = fl.find("blockers")) e["top_blockers"] = *b;
            if (const JsonValue* m = fl.find("modes_attempted")) e["modes_attempted"] = *m;
            if (const JsonValue* al = fl.find("attempted_layers")) e["attempted_layers"] = *al;
            if (const JsonValue* av = fl.find("attempted_via_classes"))
                e["attempted_via_classes"] = *av;
            if (const JsonValue* fr = fl.find("frontier")) e["frontier"] = *fr;
            // Actionable suggestion, deterministic per reason.
            std::string reason = fl.get_string("reason");
            std::string hint;
            if (reason == "budget_exhausted" || reason == "unattempted")
                hint = "raise --max-search-nodes / --timeout, or widen recovery beam";
            else if (reason == "conflict")
                hint = "inspect top_blockers: rip-up already tried; consider rule or placement change";
            else if (reason == "no_via")
                hint = "check via_classes for a style covering required_current_a";
            else if (reason.rfind("hard_violation", 0) == 0)
                hint = "pre-existing illegal copper: fix input or relax the violated rule";
            else if (reason == "impedance_current_conflict" || reason == "impedance_infeasible")
                hint = "ampacity floor vs impedance band conflict: change stackup or targets";
            else
                hint = "corridor blocked: see top_blockers + congestion_hotspots in the report";
            e["suggestion"] = hint;
            list.as_array().push_back(e);
        }
    }
    out["failure_count"] = static_cast<double>(nfail);
    out["failures"] = list;
    if (const JsonValue* hs = rep.find("congestion_hotspots")) out["hotspots"] = *hs;
    if (f.json) {
        emit_json(f, out);
    } else if (!f.quiet) {
        std::cout << "failures: " << nfail << " (" << out.get_string("status") << "/"
                  << out.get_string("result_category") << ")\n";
        for (const auto& e : list.as_array()) {
            std::cout << "  net " << e.get_string("net_name") << " terminals "
                      << e.get_number("terminal_a", -1) << "->" << e.get_number("terminal_b", -1);
            std::string sc = e.get_string("src_component");
            if (!sc.empty())
                std::cout << " (" << sc << "." << e.get_string("src_pin") << " -> "
                          << e.get_string("dst_component") << "." << e.get_string("dst_pin")
                          << ")";
            std::cout << " reason=" << e.get_string("reason")
                      << " candidates=" << e.get_number("candidate_count", 0)
                      << " ripups=" << e.get_number("ripup_attempts", 0) << "\n";
            std::cout << "    suggestion: " << e.get_string("suggestion") << "\n";
        }
    }
    return kOk;
}

int run(const std::vector<std::string>& args) {
    if (args.size() < 2) {
        std::cerr << "usage: router <capabilities|analyze|verify|route|escape|benchmark|explain-failure> [board] [flags]\n"
                     "       router --help | router --version\n";
        return kInvalidInput;
    }
    if (args[1] == "--help" || args[1] == "-h" || args[1] == "help") {
        std::cout << "copperline " << COPPERLINE_VERSION
                  << " - headless PCB autorouter for agents\n\n"
                      "  router capabilities [--json]\n"
                      "  router analyze <board> [--json] [--config cfg.json]\n"
                      "  router verify <board> [--json] [--config cfg.json] [--routes routed.ses]\n"
                      "  router route <board> [--json] [--config cfg.json] [--seed N]\n"
                      "                       [--threads N] [--timeout S] [--max-search-nodes N]\n"
                      "                       [--route-k K] [--output routed.ses] [--report report.json]\n"
                      "                       [--no-hierarchy] [--no-impact] [--impact-mlp]\n"
                      "                       [--recovery-depth D] [--recovery-beam B]\n"
                      "                       [--max-multiply-nodes N] [--no-optimizer]\n"
                      "                       [--no-tuning] [--tuning-amplitude-mm A]\n"
                      "                       [--tuning-pitch-mm P] [--tuning-max-added-mm M]\n"
                      "  router escape <board> [--json] [--config cfg.json] [--report report.json]\n"
                      "  router benchmark <board> [--json] [--config cfg.json] [--seed N]\n"
                      "                       [--threads N] [--timeout S] [--max-search-nodes N]\n"
                      "  router explain-failure <route-report.json> [--json]\n\n"
                      "route flags: --progress emits NDJSON epoch events on stderr\n"
                      "boards: .json (native), .kicad_pcb, .dsn (Specctra subset),\n"
                      "        .gbr/.gtl/.gbl (Gerber RS-274X subset), .xml (IPC-2581C subset)\n"
                      "route --output: .json (native), .ses (Specctra session),\n"
                      "                or .kicad_pcb (KiCad board)\n"
                      "threads: 0 = auto (all CPUs); explicit N wins; memory <= 2048MB\n"
                      "exit codes: 0 ok, 2 invalid input, 3 malformed rules, 4 incomplete,\n"
                      "            5 rule violation, 6 internal failure, 7 budget exhausted\n";
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
        if (cmd == "explain-failure") return cmd_explain_failure(f);
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
