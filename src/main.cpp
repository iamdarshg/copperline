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
#include "router/optimizer.h"
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
    bool time_stages = false;  // stage + CLI wall timings as JSON on stderr
    bool no_board_echo = false;  // S3 (I3): route stdout omits the board object
    std::string config;
    std::string strategy;  // Phase 3 item 15: strategy JSON file
    unsigned seed = 42;
    int threads = 0;  // Prompt 5: 0 = auto (hardware_concurrency, bounded)
    int route_k = 0;  // issue #8: 0 = auto (maturity route-K), else 1..15 wins
    bool no_impact = false;  // issue #9: disable obstruction scoring
    bool impact_mlp = false;  // issue #9: tiny fixed-weight MLP scorer
    double timeout_s = 0;
    std::int64_t max_search_nodes = 200000;
    int max_epochs = 0;  // 0 = engine default; else hard epoch cap (bounded runs)
    // Recovery runs after the greedy epochs and is NOT bounded by
    // --max-epochs. -1 = engine default; 0 disables rip-up recovery.
    int max_ripup_generations = -1;
    bool no_hierarchy = false;  // issue #10: disable coarse-to-fine guidance
    int recovery_depth = 2;     // issue #22: 1 = one-ply, default 2
    int recovery_beam = 4;      // issue #22: beam width, default 4
    std::int64_t max_multiply_nodes = 64;  // issue #22: deeper node budget
    bool recovery_depth_set = false;
    bool recovery_beam_set = false;
    bool max_multiply_nodes_set = false;
    bool no_tuning = false;  // issue #15: disable post-route length tuning
    bool no_optimizer = false;  // Prompt 5: disable cleanup optimizer
    // Phase 5 item 30: `router gloss` pass selection.
    bool gloss_straighten = false;
    bool gloss_recorner = false;
    bool gloss_via_min = false;
    bool gloss_clean_pad = false;
    bool gloss_all = false;
    bool gloss_hug = false;
    bool gloss_spread = false;
    // Phase 5 item 29: `router fanout` arrangement + direction.
    std::string fanout_pattern = "auto";
    int fanout_dir = -1;
    // Phase 5 item 31: external router command prefix for route-external.
    std::string route_router;
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
        else if (a == "--time-stages") f.time_stages = true;
        else if (a == "--no-board-echo") f.no_board_echo = true;
        else if (a == "--config") {
            if (!need_value(i, "--config", v)) return false;
            f.config = v;
            ++i;
        } else if (a == "--strategy") {
            if (!need_value(i, "--strategy", v)) return false;
            f.strategy = v;
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
        } else if (a == "--max-epochs") {
            if (!need_value(i, "--max-epochs", v)) return false;
            try {
                f.max_epochs = std::stoi(v);
            } catch (...) {
                err = "bad --max-epochs value";
                return false;
            }
            if (f.max_epochs < 1) {
                err = "--max-epochs must be >= 1";
                return false;
            }
            ++i;
        } else if (a == "--max-ripup-generations") {
            if (!need_value(i, "--max-ripup-generations", v)) return false;
            try {
                f.max_ripup_generations = std::stoi(v);
            } catch (...) {
                err = "bad --max-ripup-generations value";
                return false;
            }
            if (f.max_ripup_generations < 0) {
                err = "--max-ripup-generations must be >= 0 (0 disables)";
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
        } else if (a == "--straighten") {
            f.gloss_straighten = true;  // item 30
        } else if (a == "--recorner") {
            f.gloss_recorner = true;  // item 30
        } else if (a == "--via-min") {
            f.gloss_via_min = true;  // item 30
        } else if (a == "--clean-pad") {
            f.gloss_clean_pad = true;  // item 30
        } else if (a == "--gloss-all") {
            f.gloss_all = true;  // item 30
        } else if (a == "--hug") {
            f.gloss_hug = true;  // item 30 (not implemented; reported)
        } else if (a == "--spread") {
            f.gloss_spread = true;  // item 30 (not implemented; reported)
        } else if (a == "--pattern") {
            if (!need_value(i, "--pattern", v)) return false;
            if (v != "auto" && v != "inline" && v != "staggered") {
                err = "--pattern must be auto|inline|staggered";
                return false;
            }
            f.fanout_pattern = v;  // item 29
            ++i;
        } else if (a == "--dir") {
            if (!need_value(i, "--dir", v)) return false;
            try {
                f.fanout_dir = std::stoi(v);
            } catch (...) {
                err = "bad --dir value";
                return false;
            }
            if (f.fanout_dir < -1 || f.fanout_dir > 7) {
                err = "--dir must be -1..7 (0:E 1:NE 2:N 3:NW 4:W 5:SW 6:S 7:SE)";
                return false;
            }
            ++i;
        } else if (a == "--router") {
            if (!need_value(i, "--router", v)) return false;
            f.route_router = v;  // item 31
            ++i;
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

// S3 (I1): --config is read+parsed ONCE per command; every consumer shares
// the parsed value. Previously `route` read+parsed the file 4x (sidecar,
// resolver, maturity, CLI verify gate) — pure agent-loop waste.
struct SharedConfig {
    bool has = false;
    JsonValue value = JsonValue::object();
};

bool load_shared_config(const Flags& f, SharedConfig& out, std::string& msg) {
    if (f.config.empty()) {
        out.has = false;
        return true;
    }
    std::string text, err;
    if (!read_file(f.config, text, err)) {
        msg = err;
        return false;
    }
    try {
        out.value = parse_json(text);
    } catch (const std::exception& e) {
        msg = std::string("bad config JSON: ") + e.what();
        return false;
    }
    out.has = true;
    return true;
}

// Phase 3 item 15: load a strategy JSON file and fold it into EngineOptions.
// Strategy schema (copperline/strategy/1):
//   {"strategy": "via_miser",
//    "escape": true, "optimizer": true, "tuning": true, "recovery": true,
//    "max_epochs": 0, "recovery_depth": 0, "recovery_beam": 0,
//    "astar_scale": 1.0}
// Stage toggles default to true (built-in behaviour); zero scalars inherit
// the engine defaults. Returns false with `msg` on malformed JSON.
bool load_strategy(const Flags& f, EngineOptions& opt, std::string& msg) {
    if (f.strategy.empty()) return true;
    std::string text, err;
    if (!read_file(f.strategy, text, err)) {
        msg = err;
        return false;
    }
    JsonValue root;
    try {
        root = parse_json(text);
    } catch (const std::exception& e) {
        msg = std::string("bad strategy JSON: ") + e.what();
        return false;
    }
    if (!root.is_object()) {
        msg = "strategy file must be a JSON object";
        return false;
    }
    opt.strategy = root.get_string("strategy", "custom");
    if (root.has("escape")) opt.strategy_escape = root.get_bool("escape", true);
    if (root.has("memory")) opt.strategy_memory = root.get_bool("memory", false);
    if (root.has("adjacent_memory"))
        opt.strategy_adjacent_memory = root.get_bool("adjacent_memory", false);
    if (root.has("straighten"))
        opt.strategy_straighten = root.get_bool("straighten", false);
    if (root.has("via_miser")) opt.strategy_via_miser = root.get_bool("via_miser", false);
    if (root.has("orthogonal"))
        opt.strategy_orthogonal = root.get_bool("orthogonal", false);
    if (root.has("clean_pad_entries"))
        opt.strategy_clean_pad_entries =
            root.get_bool("clean_pad_entries", false);
    if (root.has("hug")) opt.strategy_hug = root.get_bool("hug", false);
    if (root.has("spread")) opt.strategy_spread = root.get_bool("spread", false);
    if (root.has("fanout_pattern")) {
        const std::string pattern = root.get_string("fanout_pattern", "auto");
        if (pattern != "auto" && pattern != "inline" && pattern != "staggered") {
            msg = "strategy fanout_pattern must be auto|inline|staggered";
            return false;
        }
        opt.strategy_fanout_pattern = pattern;
    }
    if (root.has("fanout_dir")) {
        const double dir = root.get_number("fanout_dir", -1.0);
        if (dir < -1.0 || dir > 7.0) {
            msg = "strategy fanout_dir out of range (-1..7)";
            return false;
        }
        opt.strategy_fanout_dir = static_cast<int>(dir);
    }
    if (root.has("optimizer")) opt.strategy_optimizer = root.get_bool("optimizer", true);
    if (root.has("tuning")) opt.strategy_tuning = root.get_bool("tuning", true);
    if (root.has("recovery")) opt.strategy_recovery = root.get_bool("recovery", true);
    if (root.has("max_epochs")) {
        double me = root.get_number("max_epochs", 0.0);
        if (me < 0 || me > 1e9) {
            msg = "strategy max_epochs out of range";
            return false;
        }
        opt.strategy_max_epochs = static_cast<int>(me);
    }
    if (root.has("recovery_depth")) {
        double rd = root.get_number("recovery_depth", 0.0);
        if (rd < 0 || rd > 64) {
            msg = "strategy recovery_depth out of range";
            return false;
        }
        opt.strategy_recovery_depth = static_cast<int>(rd);
    }
    if (root.has("recovery_beam")) {
        double rb = root.get_number("recovery_beam", 0.0);
        if (rb < 0 || rb > 256) {
            msg = "strategy recovery_beam out of range";
            return false;
        }
        opt.strategy_recovery_beam = static_cast<int>(rb);
    }
    if (root.has("astar_scale")) {
        double s = root.get_number("astar_scale", 1.0);
        if (s < 0.05 || s > 100.0) {
            msg = "strategy astar_scale out of range [0.05, 100]";
            return false;
        }
        opt.strategy_astar_scale = s;
    }
    return true;
}

bool load_resolver(const Board& board, const SharedConfig& cfg, RuleResolver& out,
                   std::string& msg) {
    try {
        if (!cfg.has) {
            out = RuleResolver::defaults_for(board);
        } else {
            out = RuleResolver::from_config(board, cfg.value);
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
bool apply_sidecar(Board& board, const SharedConfig& cfg, std::string& msg) {
    if (!cfg.has) return true;
    try {
        apply_sidecar_nets(board, cfg.value);
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
// adaptive-budget thresholds/caps. The shared parse above already validated
// the file, so this only applies the maturity/recovery/tuning sections.
bool load_maturity_opt(const SharedConfig& cfg, EngineOptions& opt, std::string& msg) {
    if (!cfg.has) return true;
    try {
        const JsonValue& cfgv = cfg.value;
        if (const JsonValue* m = cfgv.find("maturity")) {
            if (!apply_maturity_json(opt.maturity, *m, msg)) return false;
        }
        // Issue #22: optional "recovery" object {depth, beam, max_nodes}.
        if (const JsonValue* r = cfgv.find("recovery")) {
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
        if (const JsonValue* t = cfgv.find("tuning")) {
            if (!tuning_config_from_json(cfgv, opt.tuning, msg)) return false;
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
    r["commands"] = json_string_array(
        std::vector<std::string>{"capabilities", "analyze", "verify", "import",
                                 "route", "gloss", "setup-report", "fanout",
                                 "route-external", "escape", "benchmark",
                                 "explain-failure"});
    JsonValue planned = JsonValue::array();
    r["planned_commands"] = planned;
    JsonValue formats = JsonValue::object();
    formats["supported"] = json_string_array(supported_formats());
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
    feat["no_board_echo"] = true;  // S3 (I3): route --no-board-echo (report only)
    feat["gloss"] = true;          // Phase 5 item 30: post-route polish CLI
    feat["gloss_straighten"] = true;
    feat["gloss_recorner"] = true;
    feat["gloss_via_min"] = true;
    feat["gloss_clean_pad"] = true;
    feat["gloss_hug"] = true;
    feat["gloss_spread"] = true;
    feat["setup_report"] = true;  // Phase 5 item 28
    feat["fanout"] = true;        // Phase 5 item 29
    feat["route_external"] = true;  // Phase 5 item 31 (requires a .dsn input)
    feat["import"] = true;          // import-only conversion to JSON
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
                  << "commands: capabilities, analyze, verify, import, route, gloss,\n"
                  << "          setup-report, fanout, route-external, escape, benchmark,\n"
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
    SharedConfig scfg;
    if (!load_shared_config(f, scfg, msg))
        return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                    "malformed_rules", msg);
    if (!apply_sidecar(lb.board, scfg, msg))
        return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                    "malformed_rules", msg);
    RuleResolver resolver = RuleResolver::defaults_for(lb.board);
    if (scfg.has) {
        if (!load_resolver(lb.board, scfg, resolver, msg))
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

int cmd_setup_report(const Flags& f) {
    LoadedBoard lb;
    int code = 0;
    std::string err_code, msg;
    if (!load_board(f, lb, code, err_code, msg)) return fail(f, code, err_code, msg);
    SharedConfig scfg;
    if (!load_shared_config(f, scfg, msg))
        return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput
                                                                    : kBadRules,
                    "malformed_rules", msg);
    if (!apply_sidecar(lb.board, scfg, msg))
        return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput
                                                                    : kBadRules,
                    "malformed_rules", msg);
    RuleResolver resolver = RuleResolver::defaults_for(lb.board);
    if (scfg.has) {
        if (!load_resolver(lb.board, scfg, resolver, msg))
            return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput
                                                                        : kBadRules,
                        "malformed_rules", msg);
    }
    ElectricalContext ctx = resolver.defaultContext();
    JsonValue rj = setup_report(lb.board, resolver, ctx, lb.warnings);
    if (!f.report.empty()) {
        std::string err;
        if (!write_file(f.report, serialize_json(rj, true), err))
            return fail(f, kInvalidInput, "write_failed", err);
        rj["report"] = f.report;
    }
    if (f.json) {
        emit_json(f, rj);
    } else if (!f.quiet) {
        std::cout << serialize_json(rj, true);
    }
    for (const auto& w : lb.warnings) warn(f, "import: " + w);
    return kOk;
}

int cmd_verify(const Flags& f) {
    LoadedBoard lb;
    int code = 0;
    std::string err_code, msg;
    if (!load_board(f, lb, code, err_code, msg)) return fail(f, code, err_code, msg);
    SharedConfig scfg;
    if (!load_shared_config(f, scfg, msg))
        return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                    "malformed_rules", msg);
    if (!apply_sidecar(lb.board, scfg, msg))
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
    if (scfg.has) {
        if (!load_resolver(lb.board, scfg, resolver, msg))
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

// Phase 5 item 30: `router gloss` — post-route polish passes with per-pass
// verify+revert (the transactional optimizer) and a JSON report. Hug and
// Spread are not implemented; when requested they are reported as
// unsupported rather than silently ignored.
int cmd_gloss(const Flags& f) {
    LoadedBoard lb;
    int code = 0;
    std::string err_code, msg;
    if (!load_board(f, lb, code, err_code, msg)) return fail(f, code, err_code, msg);
    SharedConfig scfg;
    if (!load_shared_config(f, scfg, msg))
        return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput
                                                                    : kBadRules,
                    "malformed_rules", msg);
    if (!apply_sidecar(lb.board, scfg, msg))
        return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput
                                                                    : kBadRules,
                    "malformed_rules", msg);
    RuleResolver resolver = RuleResolver::defaults_for(lb.board);
    if (scfg.has) {
        if (!load_resolver(lb.board, scfg, resolver, msg))
            return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput
                                                                        : kBadRules,
                        "malformed_rules", msg);
    }
    ElectricalContext ctx = resolver.defaultContext();

    const bool all = f.gloss_all;
    const bool want_straighten = all || f.gloss_straighten;
    const bool want_recorner = all || f.gloss_recorner;
    const bool want_via_min = all || f.gloss_via_min;
    const bool want_clean_pad = all || f.gloss_clean_pad;
    const bool want_hug = all || f.gloss_hug;
    const bool want_spread = all || f.gloss_spread;
    if (!want_straighten && !want_recorner && !want_via_min && !want_clean_pad &&
        !want_hug && !want_spread) {
        return fail(f, kInvalidInput, "invalid_input",
                    "gloss: select at least one pass (--straighten --recorner "
                    "--via-min --clean-pad --hug --spread --gloss-all)");
    }
    std::vector<std::string> passes;
    if (want_straighten) passes.push_back("straighten");
    if (want_recorner) passes.push_back("recorner");
    if (want_via_min) passes.push_back("via_min");
    if (want_clean_pad) passes.push_back("clean_pad");
    if (want_hug) passes.push_back("hug");
    if (want_spread) passes.push_back("spread");
    std::vector<std::string> unsupported;  // reserved: no pass is unsupported now
    if (passes.empty()) {
        return fail(f, kInvalidInput, "invalid_input",
                    "gloss: no pass selected");
    }

    // Gloss runs exactly the requested polish passes (baseline merge/bend
    // transforms are not part of gloss).
    OptimizerOptions opt;
    opt.collinear_merge = false;
    opt.bend_removal = false;
    opt.via_elimination = want_via_min;
    opt.preferred_layer = false;
    opt.straighten = want_straighten;
    opt.orthogonal = want_recorner;
    opt.clean_pad_entries = want_clean_pad;
    opt.hug = want_hug;
    opt.spread = want_spread;

    const std::string hash_before = geometry_hash(lb.board);
    CleanupOptimizer polish(&lb.board, &resolver, ctx, opt);
    OptimizerReport reps = polish.run();
    resolver.rebind(&lb.board);
    BoardVerifier verifier;
    VerifyResult vr = verifier.verify(lb.board, resolver, ctx);
    const std::string hash_after = geometry_hash(lb.board);

    JsonValue rj = JsonValue::object();
    rj["schema"] = "copperline/gloss-report/1";
    rj["passes"] = json_string_array(passes);
    rj["unsupported"] = json_string_array(unsupported);
    rj["optimizer"] = reps.to_json();
    rj["verification"] = vr.to_json();
    rj["board_hash_before"] = hash_before;
    rj["board_hash_after"] = hash_after;
    const bool ran = reps.ran;
    rj["status"] = !ran ? JsonValue("GATED")
                        : (vr.ok ? JsonValue("COMPLETE") : JsonValue("VIOLATION"));

    if (!f.output.empty()) {
        std::string err;
        bool ok = false;
        const auto ends_with = [](const std::string& s, const std::string& suffix) {
            return s.size() >= suffix.size() &&
                   s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
        };
        if (ends_with(f.output, ".ses")) {
            ok = write_file(f.output, board_to_ses(lb.board, f.board), err);
        } else if (ends_with(f.output, ".kicad_pcb")) {
            ok = write_file(f.output, board_to_kicad_pcb(lb.board), err);
        } else {
            ok = write_file(f.output, serialize_json(board_to_json(lb.board), true), err);
        }
        if (!ok) return fail(f, kInvalidInput, "write_failed", err);
        rj["output"] = f.output;
    }
    if (!f.report.empty()) {
        std::string err;
        if (!write_file(f.report, serialize_json(rj, true), err))
            return fail(f, kInvalidInput, "write_failed", err);
        rj["report"] = f.report;
    }
    if (f.json) {
        emit_json(f, rj);
    } else if (!f.quiet) {
        std::cout << (vr.ok ? "PASS" : "FAIL") << ": gloss " << passes.size()
                  << " pass(es), applied " << reps.applied << ", reverted "
                  << reps.reverted << ", length " << reps.length_before_mm << " -> "
                  << reps.length_after_mm << " mm\n";
        if (!ran) std::cout << "  gated: " << reps.gate_reason << "\n";
        if (!unsupported.empty()) {
            std::cout << "  unsupported:";
            for (const auto& u : unsupported) std::cout << " " << u;
            std::cout << "\n";
        }
    }
    for (const auto& w : lb.warnings) warn(f, "import: " + w);
    if (!ran) return kIncomplete;
    if (!vr.legal) return kViolation;
    if (!vr.connected) return kIncomplete;
    return kOk;
}

// Phase 5 item 29: `router fanout` — SMD fanout through the escape engine
// with item-21 pattern/direction selection, committing the chosen candidates
// to copper and reporting what was placed.
int cmd_fanout(const Flags& f) {
    LoadedBoard lb;
    int code = 0;
    std::string err_code, msg;
    if (!load_board(f, lb, code, err_code, msg)) return fail(f, code, err_code, msg);
    SharedConfig scfg;
    if (!load_shared_config(f, scfg, msg))
        return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput
                                                                    : kBadRules,
                    "malformed_rules", msg);
    if (!apply_sidecar(lb.board, scfg, msg))
        return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput
                                                                    : kBadRules,
                    "malformed_rules", msg);
    RuleResolver resolver = RuleResolver::defaults_for(lb.board);
    if (scfg.has) {
        if (!load_resolver(lb.board, scfg, resolver, msg))
            return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput
                                                                        : kBadRules,
                        "malformed_rules", msg);
    }
    ElectricalContext ctx = resolver.defaultContext();
    EscapeOptions escape_options;
    escape_options.threads = f.threads;
    escape_options.fanout_pattern = f.fanout_pattern;
    escape_options.fanout_dir = f.fanout_dir;
    EscapePlanner planner(escape_options);
    EscapeResult esc = planner.plan(lb.board, resolver, ctx);

    int committed = 0, traces = 0, vias = 0;
    for (const auto& fp : esc.footprints) {
        std::map<TermId, const PadEscapeResult*> by_term;
        for (const auto& p : fp.pads) by_term[p.terminal] = &p;
        for (TermId tid : fp.commit_order) {
            auto it = by_term.find(tid);
            if (it == by_term.end() || !it->second->has_viable ||
                it->second->candidates.empty())
                continue;
            const EscapeCandidate& best = it->second->candidates.front();
            for (const auto& s : best.traces) {
                lb.board.traces.push_back(s);
                ++traces;
            }
            for (const auto& v : best.vias) {
                lb.board.vias.push_back(v);
                ++vias;
            }
            ++committed;
        }
    }
    resolver.rebind(&lb.board);
    BoardVerifier verifier;
    VerifyResult vr = verifier.verify(lb.board, resolver, ctx);

    JsonValue rj = JsonValue::object();
    rj["schema"] = "copperline/fanout-report/1";
    rj["pattern"] = esc.fanout_pattern;
    rj["dir"] = static_cast<double>(esc.fanout_dir);
    rj["pads_total"] = static_cast<double>(esc.pads_total);
    rj["pads_escaped"] = static_cast<double>(esc.pads_with_candidates);
    rj["pads_infeasible"] = static_cast<double>(esc.pads_infeasible);
    rj["fanout_filtered"] = static_cast<double>(esc.fanout_filtered);
    rj["terminals_committed"] = static_cast<double>(committed);
    rj["traces_added"] = static_cast<double>(traces);
    rj["vias_added"] = static_cast<double>(vias);
    rj["verification"] = vr.to_json();
    rj["status"] = vr.legal ? JsonValue("COMPLETE") : JsonValue("VIOLATION");

    if (!f.output.empty()) {
        std::string err;
        const auto ends_with = [](const std::string& s, const std::string& suffix) {
            return s.size() >= suffix.size() &&
                   s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
        };
        bool ok = false;
        if (ends_with(f.output, ".ses")) {
            ok = write_file(f.output, board_to_ses(lb.board, f.board), err);
        } else if (ends_with(f.output, ".kicad_pcb")) {
            ok = write_file(f.output, board_to_kicad_pcb(lb.board), err);
        } else {
            ok = write_file(f.output, serialize_json(board_to_json(lb.board), true), err);
        }
        if (!ok) return fail(f, kInvalidInput, "write_failed", err);
        rj["output"] = f.output;
    }
    if (!f.report.empty()) {
        std::string err;
        if (!write_file(f.report, serialize_json(rj, true), err))
            return fail(f, kInvalidInput, "write_failed", err);
        rj["report"] = f.report;
    }
    if (f.json) {
        emit_json(f, rj);
    } else if (!f.quiet) {
        std::cout << "fanout " << esc.fanout_pattern << " (dir " << esc.fanout_dir
                  << "): " << committed << " terminal(s), " << traces << " traces, "
                  << vias << " vias, " << esc.pads_infeasible << " infeasible\n";
    }
    for (const auto& w : lb.warnings) warn(f, "import: " + w);
    if (!vr.legal) return kViolation;
    return kOk;
}

// Phase 5 item 31: `router route-external <design.dsn> --router "<cmd prefix>"`
// writes a Specctra DO file, invokes the external router (FreeRouting by
// default) and imports the SES it produces. There is no DSN exporter in the
// project, so the input must already be a .dsn design (forking a JSON board
// through an external router would need one). The external process is not
// bundled; the happy path requires the user to have the router installed.
int cmd_route_external(const Flags& f) {
    const auto ends_with = [](const std::string& s, const std::string& suffix) {
        return s.size() >= suffix.size() &&
               s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
    };
    if (!ends_with(f.board, ".dsn")) {
        return fail(f, kInvalidInput, "invalid_input",
                    "route-external needs a .dsn input (no DSN exporter exists)");
    }
    if (f.route_router.empty()) {
        return fail(f, kInvalidInput, "invalid_input",
                    "route-external requires --router \"<command prefix>\"");
    }
    LoadedBoard lb;
    int code = 0;
    std::string err_code, msg;
    if (!load_board(f, lb, code, err_code, msg)) return fail(f, code, err_code, msg);

    const std::string ses =
        f.output.empty() ? (f.board + ".ses") : f.output;
    const std::string dofile = f.board + ".do";
    {
        std::ostringstream do_content;
        do_content << "(source \"" << f.board << "\")\n"
                   << "(export \"" << ses << "\")\n"
                   << "(host_cad \"copperline\")\n"
                   << "(host_version \"" << COPPERLINE_VERSION << "\")\n";
        std::string err;
        if (!write_file(dofile, do_content.str(), err))
            return fail(f, kInvalidInput, "write_failed", err);
    }

    const std::string cmd = f.route_router + " \"" + dofile + "\"";
    const int rc = std::system(cmd.c_str());
    JsonValue rj = JsonValue::object();
    rj["schema"] = "copperline/external-route-report/1";
    rj["router"] = f.route_router;
    rj["do_file"] = dofile;
    rj["ses_file"] = ses;
    rj["router_exit_code"] = static_cast<double>(rc);
    bool ok = (rc == 0);
    std::ifstream ses_in(ses);
    if (!ok) {
        rj["status"] = "ROUTER_FAILED";
    } else if (!ses_in) {
        rj["status"] = "NO_SES";
        ok = false;
    } else {
        ses_in.close();
        try {
            for (const auto& w : merge_routes_file(lb.board, ses)) warn(f, "ses: " + w);
        } catch (const BoardError& e) {
            return fail(f, kInvalidInput, "invalid_input", e.what());
        }
        RuleResolver resolver = RuleResolver::defaults_for(lb.board);
        ElectricalContext ctx = resolver.defaultContext();
        BoardVerifier verifier;
        VerifyResult vr = verifier.verify(lb.board, resolver, ctx);
        rj["verification"] = vr.to_json();
        rj["status"] = vr.ok ? JsonValue("COMPLETE")
                             : (vr.legal ? JsonValue("INCOMPLETE")
                                         : JsonValue("VIOLATION"));
    }
    if (!f.report.empty()) {
        std::string err;
        if (!write_file(f.report, serialize_json(rj, true), err))
            return fail(f, kInvalidInput, "write_failed", err);
        rj["report"] = f.report;
    }
    if (f.json) {
        emit_json(f, rj);
    } else if (!f.quiet) {
        std::cout << "route-external: " << rj.get_string("status") << " ("
                  << f.route_router << ")\n";
    }
    if (!ok) return kInternal;
    if (rj.get_string("status") == "VIOLATION") return kViolation;
    if (rj.get_string("status") == "INCOMPLETE") return kIncomplete;
    return kOk;
}

int cmd_escape(const Flags& f) {
    LoadedBoard lb;
    int code = 0;
    std::string err_code, msg;
    if (!load_board(f, lb, code, err_code, msg)) return fail(f, code, err_code, msg);
    SharedConfig scfg;
    if (!load_shared_config(f, scfg, msg))
        return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                    "malformed_rules", msg);
    if (!apply_sidecar(lb.board, scfg, msg))
        return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                    "malformed_rules", msg);
    RuleResolver resolver = RuleResolver::defaults_for(lb.board);
    if (scfg.has) {
        if (!load_resolver(lb.board, scfg, resolver, msg))
            return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                        "malformed_rules", msg);
    }
    for (const auto& w : lb.warnings) warn(f, "import: " + w);
    ElectricalContext ctx = resolver.defaultContext();
    // Prompt 5 / issue #3 idiom (same as route): explicit --threads wins,
    // 0 = auto (resolved inside the planner via resolve_worker_threads).
    // Footprint merge + candidate selection are worker-count independent,
    // so --threads never changes the committed geometry.
    EscapeOptions escape_options;
    escape_options.threads = f.threads;
    EscapePlanner planner(escape_options);
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

// `router import <board> --output <file.json>` — convert a supported input to
// the native JSON board WITHOUT routing. Useful for inspecting what the
// importer produced and for strip-and-reroute workflows.
int cmd_import(const Flags& f) {
    LoadedBoard lb;
    int code = 0;
    std::string err_code, msg;
    if (!load_board(f, lb, code, err_code, msg)) return fail(f, code, err_code, msg);
    JsonValue bj = board_to_json(lb.board);
    if (!f.output.empty()) {
        std::string err;
        if (!write_file(f.output, serialize_json(bj, true), err))
            return fail(f, kInvalidInput, "write_failed", err);
        if (!f.quiet && !f.json)
            std::cout << "wrote " << f.output << "\n";
    }
    if (f.json) emit_json(f, bj);
    for (const auto& w : lb.warnings) warn(f, "import: " + w);
    return kOk;
}

int cmd_route(const Flags& f) {
    auto cli_now = []() { return std::chrono::steady_clock::now(); };
    const auto t_cli0 = cli_now();
    double ms_import = 0, ms_cli_verify = 0, ms_report_json = 0, ms_write = 0;
    std::size_t board_bytes = 0, output_bytes = 0;
    LoadedBoard lb;
    int code = 0;
    std::string err_code, msg;
    if (!load_board(f, lb, code, err_code, msg)) return fail(f, code, err_code, msg);
    SharedConfig scfg;
    if (!load_shared_config(f, scfg, msg))
        return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                    "malformed_rules", msg);
    if (!apply_sidecar(lb.board, scfg, msg))
        return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                    "malformed_rules", msg);
    RuleResolver resolver = RuleResolver::defaults_for(lb.board);
    if (scfg.has) {
        if (!load_resolver(lb.board, scfg, resolver, msg))
            return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                        "malformed_rules", msg);
    }
    for (const auto& w : lb.warnings) warn(f, "import: " + w);
    ms_import = std::chrono::duration<double, std::milli>(cli_now() - t_cli0).count();
    {  // board file bytes for the I/O ledger (best effort, read-only stat)
        std::ifstream sz(f.board, std::ios::binary | std::ios::ate);
        if (sz) board_bytes = static_cast<std::size_t>(sz.tellg());
    }

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
    if (f.max_epochs > 0) opt.max_epochs = f.max_epochs;
    if (f.max_ripup_generations >= 0)
        opt.max_ripup_generations = f.max_ripup_generations;
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
        if (!load_maturity_opt(scfg, opt, merr))
            return fail(f, kBadRules, "malformed_rules", merr);
        // Issue #8: explicit --route-k wins over the maturity cap.
        if (f.route_k >= 1) opt.maturity.caps.max_route_k = f.route_k;
        // Issue #22: explicit CLI depth/beam/nodes win over --config.
        if (f.recovery_depth_set) opt.recovery_depth = f.recovery_depth;
        if (f.recovery_beam_set) opt.recovery_beam = f.recovery_beam;
        if (f.max_multiply_nodes_set) opt.max_multiply_nodes = f.max_multiply_nodes;
    }
    // Phase 3 item 15: declarative strategy file folds stage toggles +
    // budget scalars into the options before the engine reads them.
    {
        std::string serr;
        if (!load_strategy(f, opt, serr))
            return fail(f, serr.find("cannot open") != std::string::npos
                               ? kInvalidInput : kBadRules,
                        "malformed_rules", serr);
    }
    if (f.progress) {
        opt.progress = [](const JsonValue& ev) {
            std::cerr << serialize_json(ev) << "\n";
        };
    }
    opt.time_stages = f.time_stages;
    RouterEngine engine(std::move(lb.board), std::move(resolver), opt);
    RouteReport report = engine.run();

    // Issue #2: independent CLI-side gate. No `router route` may exit 0
    // unless BoardVerifier.ok on the final committed copper, even if the
    // engine's bookkeeping claimed COMPLETE.
    {
        const auto t_verify0 = cli_now();
        RuleResolver verify_resolver = RuleResolver::defaults_for(engine.committed());
        if (scfg.has) {
            try {
                verify_resolver = RuleResolver::from_config(engine.committed(), scfg.value);
            } catch (...) {
                // Config already validated above; keep defaults on
                // unexpected re-parse failure (engine gate already ran).
            }
        }
        ElectricalContext verify_ctx = verify_resolver.defaultContext();
        BoardVerifier verifier;
        VerifyResult independent = verifier.verify(engine.committed(), verify_resolver, verify_ctx);
        apply_verifier_gate(report, independent);
        ms_cli_verify = std::chrono::duration<double, std::milli>(cli_now() - t_verify0).count();
    }

    const auto t_json0 = cli_now();
    JsonValue rj = report.to_json();
    ms_report_json = std::chrono::duration<double, std::milli>(cli_now() - t_json0).count();
    const auto t_write0 = cli_now();
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
            std::string payload = board_to_ses(routed, f.board);
            output_bytes += payload.size();
            ok = write_file(f.output, payload, err);
        } else if (has_suffix(f.output, ".kicad_pcb")) {
            std::string payload = board_to_kicad_pcb(routed);
            output_bytes += payload.size();
            ok = write_file(f.output, payload, err);
        } else {
            std::string payload = serialize_json(board_to_json(routed), true);
            output_bytes += payload.size();
            ok = write_file(f.output, payload, err);
        }
        if (!ok) return fail(f, kInvalidInput, "write_failed", err);
        rj["output"] = f.output;
        rj["output_format"] = has_suffix(f.output, ".ses")
                                  ? "ses"
                                  : (has_suffix(f.output, ".kicad_pcb") ? "kicad_pcb" : "json");
    } else if (!f.no_board_echo) {
        // S3 (I3): default keeps the full board echo for piping; agent
        // loops pass --no-board-echo for the report alone (same rj is
        // written to --report, so stdout and file stay consistent).
        rj["board"] = board_to_json(routed);
    }
    if (!f.report.empty()) {
        std::string err;
        std::string payload = serialize_json(rj, true);
        output_bytes += payload.size();
        if (!write_file(f.report, payload, err))
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
    ms_write = std::chrono::duration<double, std::milli>(cli_now() - t_write0).count();
    if (f.time_stages) {
        std::fprintf(stderr,
                     "{\"event\":\"cli_timings\",\"import_ms\":%.3f,"
                     "\"cli_verify_ms\":%.3f,\"report_json_ms\":%.3f,"
                     "\"serialize_write_ms\":%.3f,\"board_bytes\":%llu,"
                     "\"output_bytes\":%llu,\"threads\":%d}\n",
                     ms_import, ms_cli_verify, ms_report_json, ms_write,
                     static_cast<unsigned long long>(board_bytes),
                     static_cast<unsigned long long>(output_bytes), opt.threads);
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
    SharedConfig scfg;
    if (!load_shared_config(f, scfg, msg))
        return fail(f, msg.find("cannot open") != std::string::npos ? kInvalidInput : kBadRules,
                    "malformed_rules", msg);
    RuleResolver base = RuleResolver::defaults_for(lb.board);
    if (scfg.has) {
        if (!load_resolver(lb.board, scfg, base, msg))
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
        if (!load_maturity_opt(scfg, bench_base, merr))
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
    // Backlog-adaptive epoch batch width; threads never change it (it is a
    // pure function of the remaining task count), only the memory bound can.
    r["batch_width_cap"] =
        static_cast<double>(bench_base.maturity.caps.max_batch_width);
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
        std::cerr << "usage: router <capabilities|analyze|verify|import|route|gloss|setup-report|fanout|route-external|escape|benchmark|explain-failure> [board] [flags]\n"
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
                      "                       [--strategy strat.json] [--report report.json]\n"
                      "                       [--max-epochs N] [--max-ripup-generations N]\n"
                      "                       [--route-k K]\n"
                      "                       [--output routed.ses] [--report report.json]\n"
                      "                       [--no-hierarchy] [--no-impact] [--impact-mlp]\n"
                      "                       [--recovery-depth D] [--recovery-beam B]\n"
                      "                       [--max-multiply-nodes N] [--no-optimizer]\n"
                      "                       [--no-tuning] [--tuning-amplitude-mm A]\n"
                      "                       [--tuning-pitch-mm P] [--tuning-max-added-mm M]\n"
                      "                       [--no-board-echo]\n"
                       "  router escape <board> [--json] [--config cfg.json] [--report report.json]\n"
                       "                       [--threads N]\n"
                      "  router benchmark <board> [--json] [--config cfg.json] [--seed N]\n"
                      "                       [--threads N] [--timeout S] [--max-search-nodes N]\n"
                      "  router explain-failure <route-report.json> [--json]\n\n"
                      "route flags: --progress emits NDJSON epoch events on stderr\n"
                      "             --time-stages emits stage_timings + cli_timings JSON on stderr\n"
                      "             --no-board-echo omits board from route stdout (report only)\n"
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
        if (cmd == "import") return cmd_import(f);
        if (cmd == "gloss") return cmd_gloss(f);
        if (cmd == "setup-report") return cmd_setup_report(f);
        if (cmd == "fanout") return cmd_fanout(f);
        if (cmd == "route-external") return cmd_route_external(f);
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
