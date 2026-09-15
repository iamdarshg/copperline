#include "router/engine.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>

#include "router/density.h"
#include "router/escape.h"
#include "router/portfolio.h"
#include "router/route_tree.h"
#include "router/simplify.h"
#include "router/sparse_graph.h"
#include "router/verifier.h"
#include "router/via_bundle.h"

namespace copperline {

JsonValue EscapeStageInfo::to_json() const {
    JsonValue r = JsonValue::object();
    r["pads_total"] = static_cast<double>(pads_total);
    r["pads_escaped"] = static_cast<double>(pads_escaped);
    r["pads_infeasible"] = static_cast<double>(pads_infeasible);
    // D5: shared array helpers (byte-identical numbers/strings).
    r["escaped_terminals"] = json_int_array(
        std::vector<int>(escaped_terminals.begin(), escaped_terminals.end()));
    r["unresolved_terminals"] = json_int_array(
        std::vector<int>(unresolved_terminals.begin(), unresolved_terminals.end()));
    r["unresolved_reasons"] = json_string_array(unresolved_reasons);
    return r;
}

JsonValue PlaneAccessInfo::to_json() const {
    JsonValue o = JsonValue::object();
    o["net"] = static_cast<double>(net);
    o["net_name"] = net_name;
    o["terminal"] = static_cast<double>(terminal);
    o["plane_id"] = static_cast<double>(plane_id);
    o["plane_layer"] = static_cast<double>(plane_layer);
    o["island"] = static_cast<double>(island);
    json_add_point_mm(o, "entry_x_mm", "entry_y_mm", nm_to_mm(entry.x),
                      nm_to_mm(entry.y));
    o["via_count"] = static_cast<double>(via_count);
    o["required_current_a"] = required_current_a;
    o["via_capacity_a"] = via_capacity_a;
    o["current_margin_a"] = current_margin_a;
    o["via_style"] = via_style;
    return o;
}

JsonValue RouteReport::to_json() const {
    JsonValue r = JsonValue::object();
    r["schema"] = "copperline/route-report/1";
    r["status"] = status;
    r["result_category"] = result_category;
    r["connected_terminals"] = static_cast<double>(connected_terminals);
    r["total_terminals"] = static_cast<double>(total_terminals);
    r["remaining_terminals"] = static_cast<double>(total_terminals - connected_terminals);
    r["board_hash"] = board_hash;
    r["state_hash"] = state_hash.to_hex();
    JsonValue st = JsonValue::object();
    st["nets_total"] = static_cast<double>(stats.nets_total);
    st["nets_routed"] = static_cast<double>(stats.nets_routed);
    st["tasks_total"] = static_cast<double>(stats.tasks_total);
    st["tasks_routed"] = static_cast<double>(stats.tasks_routed);
    st["via_count"] = static_cast<double>(stats.via_count);
    st["length_mm"] = nm_to_mm(stats.length_nm);
    st["expansions"] = static_cast<double>(stats.expansions_total);
    st["time_ms"] = static_cast<double>(stats.time_ms);
    st["threads_requested"] = static_cast<double>(stats.threads_requested);
    st["threads_used"] = static_cast<double>(stats.threads_used);
    st["candidates_total"] = static_cast<double>(stats.candidates_total);
    st["candidates_accepted"] = static_cast<double>(stats.candidates_accepted);
    st["candidates_rejected"] = static_cast<double>(stats.candidates_rejected);
    st["epochs"] = static_cast<double>(stats.epochs_count);
    // Issue #10: hierarchical-guidance aggregates.
    st["hierarchy_guided_tasks"] = static_cast<double>(stats.hierarchy_guided_tasks);
    st["hierarchy_fallback_tasks"] = static_cast<double>(stats.hierarchy_fallback_tasks);
    st["hierarchy_coarse_expansions"] =
        static_cast<double>(stats.hierarchy_coarse_expansions);
    r["stats"] = st;
    JsonValue ep = JsonValue::array();
    for (const auto& e : epochs) ep.as_array().push_back(e.to_json());
    r["epoch_log"] = ep;
    JsonValue hs = JsonValue::array();
    for (const auto& h : hotspots) {
        JsonValue o = JsonValue::object();
        o["x_mm"] = h.x_mm;
        o["y_mm"] = h.y_mm;
        o["present"] = h.present;
        o["history"] = h.history;
        hs.as_array().push_back(o);
    }
    r["congestion_hotspots"] = hs;
    JsonValue fails = JsonValue::array();
    for (const auto& f : failures) {
        JsonValue o = JsonValue::object();
        o["net"] = static_cast<double>(f.net);
        o["net_name"] = f.net_name;
        o["terminal_a"] = static_cast<double>(f.a);
        o["terminal_b"] = static_cast<double>(f.b);
        o["reason"] = f.reason;
        o["expansions"] = static_cast<double>(f.expansions);
        o["required_width_mm"] = f.required_width_mm;
        o["width_source"] = f.width_source;
        o["width_model"] = f.width_model.empty() ? f.width_source : f.width_model;
        o["copper_weight_oz"] = f.copper_weight_oz;
        o["temp_rise_c"] = f.temp_rise_c;
        // Parallel-via diagnostics (issue #5).
        o["required_current_a"] = f.required_current_a;
        o["via_style"] = f.via_style;
        o["vias_required"] = static_cast<double>(f.vias_required);
        o["via_reason"] = f.via_reason;
        o["ripup_attempts"] = static_cast<double>(f.ripup_attempts);
        o["modes_attempted"] = json_string_array(f.modes_attempted);
        // Prompt 5 agent-stable attribution (stable IDs for iteration).
        o["src_component"] = f.src_component;
        o["src_pin"] = f.src_pin;
        o["dst_component"] = f.dst_component;
        o["dst_pin"] = f.dst_pin;
        o["centre_depth"] = static_cast<double>(f.centre_depth);
        o["pin_density_per_mm2"] = f.pin_density;
        o["candidate_count"] = static_cast<double>(f.candidate_count);
        o["best_partial"] = f.best_partial;
        o["category"] = f.category;
        o["attempted_layers"] = json_int_array(
            std::vector<int>(f.attempted_layers.begin(), f.attempted_layers.end()));
        o["attempted_via_classes"] = json_string_array(f.attempted_via_classes);
        // Issue #11: controlled-impedance accounting.
        o["has_impedance"] = f.has_impedance;
        if (f.has_impedance) {
            o["target_impedance_ohms"] = f.target_impedance_ohms;
            o["impedance_tolerance_pct"] = f.impedance_tolerance_pct;
            o["impedance_layer"] = static_cast<double>(f.impedance_layer);
            o["impedance_width_mm"] = f.impedance_width_mm;
            o["impedance_model"] = f.impedance_model;
            o["estimated_impedance_ohms"] = f.estimated_impedance_ohms;
            o["impedance_error_pct"] = f.impedance_error_pct;
            o["impedance_conflict"] = f.impedance_conflict;
            if (!f.impedance_detail.empty()) o["impedance_detail"] = f.impedance_detail;
        }
        if (f.has_frontier) o["frontier"] = f.frontier.to_json();
        // Issue #16: plane target attribution for power-access failures.
        o["has_plane_target"] = f.has_plane_target;
        if (f.has_plane_target) {
            o["plane_id"] = static_cast<double>(f.plane_id);
            o["plane_layer"] = static_cast<double>(f.plane_layer);
            o["plane_island"] = static_cast<double>(f.plane_island);
        }
        // Issue #12: pair-corridor attribution.
        o["is_pair_corridor"] = f.is_pair_corridor;
        if (f.is_pair_corridor) {
            o["pair_id"] = static_cast<double>(f.pair_id);
            o["pair_name"] = f.pair_name;
            o["pair_other_net"] = static_cast<double>(f.pair_other_net);
        }
        o["blockers"] = json_string_array(f.blockers);
        // Issue #10: hierarchical-guidance diagnostics for the last attempt.
        JsonValue hier = JsonValue::object();
        hier["levels_used_mm"] = json_double_array(f.hierarchy_levels_mm);
        hier["fallback"] = f.hierarchy_fallback;
        hier["fallback_reason"] = f.hierarchy_reason;
        hier["coarse_expansions"] = static_cast<double>(f.hierarchy_coarse_expansions);
        hier["window_attempts"] = static_cast<double>(f.hierarchy_window_attempts);
        hier["exact_expansions"] = static_cast<double>(f.expansions);
        o["hierarchy"] = hier;
        fails.as_array().push_back(o);
    }
    r["failures"] = fails;
    JsonValue pa = JsonValue::array();
    for (const auto& p : plane_access) pa.as_array().push_back(p.to_json());
    r["plane_access"] = pa;
    // Issue #11: per-net controlled-impedance report (selected layer/width,
    // model, target, estimate, tolerance error). Only nets with targets.
    JsonValue iz = JsonValue::array();
    for (const auto& z : impedance) iz.as_array().push_back(z.to_json());
    r["impedance"] = iz;
    // Issue #12: pair corridor + materialization report.
    JsonValue dp = JsonValue::array();
    for (const auto& p : diffpairs) dp.as_array().push_back(p.to_json());
    r["diffpairs"] = dp;
    // Issue #15: post-route length/skew tuning report.
    r["tuning"] = tuning.to_json();
    // Prompt 5: transactional cleanup optimizer report.
    r["optimizer"] = optimizer.to_json();
    r["recovery"] = recovery.to_json();
    r["escape"] = escape_stage.to_json();
    // Issue #14: maturity/budget schedule + final effective state. Agents
    // see every phase transition and the exact hyperparams consumed.
    {
        JsonValue ml = JsonValue::array();
        for (const auto& e : maturity_log) ml.as_array().push_back(e.to_json());
        r["maturity_log"] = ml;
        r["maturity"] = maturity.to_json();
        if (has_budget) r["budget"] = budget.to_json();
    }
    r["verification"] = verification.to_json();
    r["verifier_ok"] = verification.ok;
    r["verifier_connected"] = verification.connected;
    r["verifier_legal"] = verification.legal;
    r["result_category"] = result_category;
    r["state_hash"] = state_hash.to_hex();
    return r;
}

RouterEngine::RouterEngine(Board board, RuleResolver resolver, EngineOptions options)
    : board_(std::move(board)), resolver_(std::move(resolver)), options_(options) {
    // The resolver was necessarily bound to the caller's board object before
    // the move; rebind it to the owned (moved-in) board.
    resolver_.rebind(&board_);
}

namespace {

// Blocker attribution for failure reports: foreign copper overlapping the
// task corridor, largest area first, plus the A* frontier gap.
std::vector<std::string> attribute_blockers(const Board& board, const ConnectionTask& task,
                                            Coord width, const CandidateRoute* last) {
    std::vector<std::string> out;
    for (const auto& h : attribute_blockers_detailed(board, task, width, last))
        out.push_back(h.desc);
    return out;
}

// Issue #16: carry the plane target into failure records so agents see
// which plane/layer/island a failed power access belonged to.
void fill_plane_fields(RouteFailure& f, const ConnectionTask& task) {
    if (task.has_plane_target) {
        f.has_plane_target = true;
        f.plane_id = task.plane_id;
        f.plane_layer = task.plane_layer;
        f.plane_island = task.plane_island;
    }
}

// Issue #10: copy hierarchical-guidance diagnostics from the last attempt
// into the failure record (levels used, fallback status, coarse work).
void fill_hierarchy_fields(RouteFailure& f, const CandidateRoute* last) {
    if (!last) return;
    f.hierarchy_levels_mm = last->hierarchy.levels_used_mm;
    f.hierarchy_fallback = last->hierarchy.fallback;
    f.hierarchy_reason = last->hierarchy.fallback_reason;
    f.hierarchy_coarse_expansions = last->hierarchy.coarse_expansions;
    f.hierarchy_window_attempts = last->hierarchy.window_attempts;
}

// Issue #12: carry the pair identity into failure records.
void fill_pair_fields(RouteFailure& f, const ConnectionTask& task,
                      const Board& board) {
    if (task.is_pair_corridor) {
        f.is_pair_corridor = true;
        f.pair_id = task.pair_id;
        f.pair_other_net = task.pair_other_net;
        for (const auto& pr : board.diffpairs) {
            if (pr.id == task.pair_id) {
                f.pair_name = pr.name;
                break;
            }
        }
    }
}

}  // namespace

void apply_verifier_gate(RouteReport& report, const VerifyResult& vr) {
    report.verification = vr;
    if (vr.ok) return;
    // Hard legality failure always escalates: illegal copper must surface as
    // the hard-rule-violation status, never as COMPLETE/INCOMPLETE/BUDGET.
    if (!vr.legal) {
        if (report.status != "VIOLATION") {
            report.status = "VIOLATION";
    report.result_category = result_category(report.status);
    for (auto& f : report.failures) f.category = report.result_category;
        }
        // Ensure failures explain the violation even when bookkeeping was
        // COMPLETE (failures empty). One synthetic entry per violation keeps
        // per-net attribution for agents.
        if (report.failures.empty()) {
            for (const auto& v : vr.violations) {
                RouteFailure f;
                f.net = v.net_a;
                f.a = -1;
                f.b = -1;
                f.reason = "hard_violation:" + v.type;
                f.blockers.push_back(v.detail);
                report.failures.push_back(f);
            }
        }
        // Illegal boards are also disconnected in general; keep the terminal
        // count truthful when bookkeeping claimed full connectivity.
        if (!vr.unconnected.empty()) {
            int un = static_cast<int>(vr.unconnected.size());
            report.connected_terminals =
                std::max(0, report.total_terminals - un);
        }
        return;
    }
    // Connectivity-only failure: refuse COMPLETE (or empty-failure success).
    if (report.status == "COMPLETE" || report.failures.empty()) {
        report.status = "INCOMPLETE";
        report.result_category = result_category(report.status);
        if (report.failures.empty()) {
            for (const auto& u : vr.unconnected) {
                RouteFailure f;
                f.net = u.net;
                f.net_name = u.net_name;
                f.a = u.terminal;
                f.b = -1;
                f.reason = "verifier_unconnected";
                report.failures.push_back(f);
            }
        }
        int un = static_cast<int>(vr.unconnected.size());
        report.connected_terminals =
            std::max(0, report.total_terminals - un);
    }
}

RouteReport RouterEngine::run() {
    auto t0 = std::chrono::steady_clock::now();
    const std::chrono::steady_clock::time_point deadline =
        options_.timeout_s > 0
            ? t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                       std::chrono::duration<double>(options_.timeout_s))
            : std::chrono::steady_clock::time_point::max();
    RouteReport report;
    report.total_terminals = static_cast<int>(board_.terminals.size());
    // ---- Lightweight stage timer (EngineOptions::time_stages) ----
    // One clock read per stage boundary only; accumulators stay zero-cost
    // when the flag is off (the final stderr line is the only output).
    auto stage_now = []() { return std::chrono::steady_clock::now(); };
    auto stage_ms = [](const std::chrono::steady_clock::time_point& a,
                       const std::chrono::steady_clock::time_point& b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    double ms_escape = 0, ms_taskgen = 0, ms_batch = 0, ms_workers = 0;
    double ms_arbiter = 0, ms_recovery = 0, ms_materialize = 0, ms_tuning = 0;
    double ms_attribution = 0, ms_verify = 0, ms_optimizer = 0, ms_hash = 0;
    const auto t_escape_start = stage_now();
    // Resolve this before preprocessing so timeout reports still describe
    // the requested worker capacity truthfully. Escape planning is currently
    // deterministic and serial; the global epochs use this full capacity.
    const int effective_threads = resolve_worker_threads(options_.threads);
    report.stats.threads_requested = effective_threads;

    // Fixed user copper: everything committed before routing starts. It is
    // the only copper that is absolutely protected (never ripped).
    const std::vector<TraceSeg> fixed_traces = board_.traces;
    const std::vector<Via> fixed_vias = board_.vias;

    ElectricalContext ctx;

    // ---- Post-P4 issue #1: centre-out escape stage ----
    // Use the same real EscapePlanner as `router escape` BEFORE ordinary
    // connection-task generation. Accepted escape traces/vias are committed
    // to the working board as owned routes flagged is_escape_stub=true, so
    // global routing starts from committed escape geometry (via the
    // copper-target RouteTree growth) instead of raw dense pads.
    std::vector<OwnedRoute> owned;
    std::map<TermId, std::string> escape_infeasible_reason;
    std::map<TermId, int> escape_centre_depth;  // Prompt 5: failure attribution
    {
        EscapeOptions escape_options;
        escape_options.deadline = deadline;
        // S1 escape parallelism: footprints plan on the worker pool and each
        // pad's fallback portal attempts fan out on it (deterministic merge,
        // same commit path below). Explicit --threads wins, 0 = auto.
        escape_options.threads = effective_threads;
        EscapePlanner planner(escape_options);
        EscapeResult esc = planner.plan(board_, resolver_, ctx);
        report.escape_stage.pads_total = esc.pads_total;
        report.escape_stage.pads_escaped = esc.pads_with_candidates;
        report.escape_stage.pads_infeasible = esc.pads_infeasible;
        // Commit in planner eligibility order (per-footprint commit_order is
        // already centre-out). Deterministic: footprints sorted by component
        // (planner guarantee), commit_order preserves eligibility order.
        for (const auto& fp : esc.footprints) {
            std::map<TermId, const PadEscapeResult*> by_term;
            for (const auto& p : fp.pads) by_term[p.terminal] = &p;
            for (TermId tid : fp.commit_order) {
                auto it = by_term.find(tid);
                if (it == by_term.end() || !it->second->has_viable ||
                    it->second->candidates.empty())
                    continue;
                const EscapeCandidate& best = it->second->candidates.front();
                OwnedRoute o;
                const Terminal* term = board_.find_terminal(tid);
                o.task.net = term ? term->net : -1;
                o.task.a = tid;
                o.task.b = -1;  // portal stub: no second terminal
                o.task_pos = -1;  // not a global task; see recovery mapping
                o.traces = best.traces;
                o.vias = best.vias;
                o.epoch_committed = -1;  // pre-global escape epoch
                o.stable_epochs = 0;
                o.is_escape_stub = true;
                o.protection = route_protection_score(true, 0.0, 0,
                                                      /*is_fixed=*/false,
                                                      RecoveryMode::FAST);
                for (const auto& s : best.traces) {
                    board_.traces.push_back(s);
                    report.stats.length_nm += euclid_len_nm(s.a, s.b);
                }
                for (const auto& v : best.vias) {
                    board_.vias.push_back(v);
                    report.stats.via_count++;
                }
                owned.push_back(o);
                report.escape_stage.escaped_terminals.push_back(tid);
            }
            for (const auto& p : fp.pads) {
                escape_centre_depth[p.terminal] = p.centre_depth;
                if (!p.has_viable) {
                    report.escape_stage.unresolved_terminals.push_back(p.terminal);
                    std::string reason = p.infeasibility.recorded
                                             ? p.infeasibility.reason
                                             : "no_candidate";
                    report.escape_stage.unresolved_reasons.push_back(reason);
                    escape_infeasible_reason[p.terminal] = reason;
                }
            }
        }
        std::sort(report.escape_stage.escaped_terminals.begin(),
                  report.escape_stage.escaped_terminals.end());
        {
            std::vector<std::size_t> idx(report.escape_stage.unresolved_terminals.size());
            for (std::size_t i = 0; i < idx.size(); ++i) idx[i] = i;
            std::sort(idx.begin(), idx.end(), [&](std::size_t a, std::size_t b) {
                return report.escape_stage.unresolved_terminals[a] <
                       report.escape_stage.unresolved_terminals[b];
            });
            std::vector<TermId> ut;
            std::vector<std::string> ur;
            for (std::size_t i : idx) {
                ut.push_back(report.escape_stage.unresolved_terminals[i]);
                ur.push_back(report.escape_stage.unresolved_reasons[i]);
            }
            report.escape_stage.unresolved_terminals = std::move(ut);
            report.escape_stage.unresolved_reasons = std::move(ur);
        }
        if (options_.progress && esc.pads_total > 0) {
            JsonValue ev = JsonValue::object();
            ev["event"] = "escape";
            ev["pads_total"] = static_cast<double>(esc.pads_total);
            ev["pads_escaped"] = static_cast<double>(esc.pads_with_candidates);
            ev["pads_infeasible"] = static_cast<double>(esc.pads_infeasible);
            JsonValue un = JsonValue::array();
            for (TermId t : report.escape_stage.unresolved_terminals)
                un.as_array().push_back(JsonValue(static_cast<double>(t)));
            ev["unresolved_terminals"] = un;
            options_.progress(ev);
        }
        resolver_.rebind(&board_);
        if (esc.timed_out) {
            report.status = "TIMEOUT";
            report.result_category = result_category(report.status);
            report.stats.nets_total = static_cast<int>(board_.nets.size());
            report.stats.time_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - t0)
                    .count();
            report.board_hash = geometry_hash(board_);
            RouteFailure f;
            f.reason = "timeout";
            f.blockers.push_back("escape preprocessing deadline reached");
            f.category = report.result_category;
            report.failures.push_back(std::move(f));
            BoardVerifier verifier;
            apply_verifier_gate(report, verifier.verify(board_, resolver_, ctx));
            if (options_.progress) {
                JsonValue ev = JsonValue::object();
                ev["event"] = "preprocessing_timeout";
                ev["phase"] = "escape";
                ev["status"] = report.status;
                options_.progress(ev);
                JsonValue done = JsonValue::object();
                done["event"] = "done";
                done["status"] = report.status;
                done["connected_terminals"] =
                    static_cast<double>(report.connected_terminals);
                done["total_terminals"] = static_cast<double>(report.total_terminals);
                options_.progress(done);
            }
            return report;
        }
    }

    // Rebuild density AFTER escape copper is committed so global routing sees
    // the actual occupied board.
    ms_escape = stage_ms(t_escape_start, stage_now());
    const auto t_taskgen_start = stage_now();
    DensityEstimator density_est;
    DensityResult density = density_est.analyze(board_);

    // ---- Connection tasks from RouteTrees (on the escape-augmented board) --
    // Two-terminal nets with escape copper now yield has_copper_target tasks
    // whose dst is the committed stub endpoint (portal), so no second
    // raw-pad task is generated.
    // Issue #12: pair member nets never produce individual tasks. Their two
    // tasks are replaced by one atomic PairCorridorTask sized for both
    // traces + gap + external clearance. Invalid pairs are reported as
    // failures (no silent fallback to independent routing).
    std::vector<ConnectionTask> tasks;
    std::vector<std::string> pair_invalid_reasons;
    std::vector<int> pair_invalid_ids;
    if (board_.diffpairs.empty()) {
        for (const auto& net : board_.nets) {
            if (net.terminals.size() < 2) continue;
            RouteTree tree = build_route_tree(board_, net.id);
            for (auto& t : tree.tasks) tasks.push_back(t);
        }
    } else {
        tasks = build_global_tasks_with_pairs(board_, pair_invalid_reasons,
                                              pair_invalid_ids);
        for (std::size_t i = 0; i < pair_invalid_ids.size(); ++i) {
            RouteFailure f;
            int pid = pair_invalid_ids[i];
            f.is_pair_corridor = true;
            f.pair_id = pid;
            f.reason = "pair_invalid:" + pair_invalid_reasons[i];
            for (const auto& pr : board_.diffpairs) {
                if (pr.id == pid) {
                    f.net = pr.net_p;
                    const NetInfo* n = board_.find_net(pr.net_p);
                    f.net_name = n ? n->name : "?";
                    f.pair_other_net = pr.net_n;
                    f.pair_name = pr.name;
                    // One entry per member so per-net failure queries work.
                    RouteFailure g = f;
                    g.net = pr.net_n;
                    const NetInfo* nn = board_.find_net(pr.net_n);
                    g.net_name = nn ? nn->name : "?";
                    report.failures.push_back(g);
                    break;
                }
            }
            report.failures.push_back(f);
        }
    }
    // Centre-out boost (Prompt 2): depth outranks density but never legality.
    // NOTE (issue #1): is_escape_stub ownership is real now (escape stubs
    // committed above with task_pos=-1). Global routes must NOT infer escape
    // status from centre depth; only the committed stubs carry the flag.
    std::map<TermId, int> depth_of;
    {
        FinePitchDetector detector;
        CentreDepthAnalyzer cda;
        for (const auto& fp : detector.detect(board_, resolver_, ctx)) {
            for (const auto& [tid, d] : cda.analyze(board_, fp)) depth_of[tid] = d;
        }
    }
    auto task_max_depth = [&](const ConnectionTask& t) {
        int da = 0, db = 0;
        auto it = depth_of.find(t.a);
        if (it != depth_of.end()) da = it->second;
        it = depth_of.find(t.b);
        if (it != depth_of.end()) db = it->second;
        // Issue #12: the deeper member sets the corridor depth.
        if (t.is_pair_corridor) {
            it = depth_of.find(t.pair_a_other);
            if (it != depth_of.end()) da = std::max(da, it->second);
            it = depth_of.find(t.pair_b_other);
            if (it != depth_of.end()) db = std::max(db, it->second);
        }
        return std::max(da, db);
    };
    std::vector<int> fail_count(tasks.size(), 0);
    std::vector<Corridor> corridors(tasks.size());
    // S1: corridor cache per board generation (Stream-1 probable_corridor
    // calls only; cache lives on our side). probable_corridor depends only
    // on terminals/nets/rules (endpoint bbox + width/2 + max clearance +
    // scarcity) — never on committed traces/vias — so a task key hit is
    // valid across epochs/generations. This memoizes the internal
    // width/clearance lookups too. The HierarchyCache clip tube below is
    // likewise shared across epochs/branches (Stream-1 API, self-
    // invalidating on board signature).
    std::unordered_map<std::string, Corridor> corridor_cache;
    auto corridor_key_of = [](const ConnectionTask& t) {
        std::string k = std::to_string(t.net) + ":" + std::to_string(t.a) +
                        ":" + std::to_string(t.b) + ":";
        k += t.has_copper_target ? std::to_string(t.copper_point.x) + "," +
                                       std::to_string(t.copper_point.y) + "," +
                                       std::to_string(t.copper_layer)
                                 : "-";
        k += ":";
        k += t.has_plane_target ? std::to_string(t.plane_id) + "," +
                                      std::to_string(t.plane_point.x) + "," +
                                      std::to_string(t.plane_point.y) + "," +
                                      std::to_string(t.plane_layer) + "," +
                                      std::to_string(t.plane_island)
                                : "-";
        k += t.is_pair_corridor ? ":pair" + std::to_string(t.pair_id) : ":nopair";
        return k;
    };
    auto cached_corridor = [&](const ConnectionTask& t) {
        std::string k = corridor_key_of(t);
        auto it = corridor_cache.find(k);
        if (it != corridor_cache.end()) return it->second;
        Corridor c = probable_corridor(board_, resolver_, t, ctx);
        corridor_cache.emplace(std::move(k), c);
        return c;
    };
    auto refresh_difficulties = [&](const std::vector<int>& idx) {
        for (int i : idx) {
            DifficultyVector dv = compute_difficulty(board_, resolver_, tasks[i], ctx,
                                                     density.terminal_density, depth_of,
                                                     fail_count[i]);
            tasks[i].difficulty = dv.total;
        }
    };
    std::vector<int> all_idx(tasks.size());
    for (std::size_t i = 0; i < tasks.size(); ++i) all_idx[i] = static_cast<int>(i);
    refresh_difficulties(all_idx);
    for (std::size_t i = 0; i < tasks.size(); ++i)
        corridors[i] = cached_corridor(tasks[i]);
    ms_taskgen = stage_ms(t_taskgen_start, stage_now());

    report.stats.tasks_total = static_cast<int>(tasks.size());
    report.stats.nets_total = static_cast<int>(board_.nets.size());
    // Issue #3: 0 = auto (all CPUs); explicit --threads wins. Workers affect
    // concurrency only: batch membership is a pure function of the scheduler
    // order + interference weights, so geometry stays identical at 1/2/4/N.
    // Layer cost multipliers for A*.
    std::vector<double> layer_mult;
    {
        int max_id = 0;
        for (const auto& l : board_.layers) max_id = std::max(max_id, l.id);
        layer_mult.assign(max_id + 1, 1.0);
        for (const auto& l : board_.layers)
            if (l.id >= 0 && l.id < static_cast<int>(layer_mult.size()))
                layer_mult[l.id] = l.cost_multiplier > 0 ? l.cost_multiplier : 1.0;
    }

    CongestionMap congestion;
    congestion.init(board_);
    ReservationSet reservations;
    // Issue #10: one hierarchical-guidance cache for the whole run. The
    // snapshot is constant inside an epoch (workers share it); the cache
    // self-invalidates on board-signature change, so epochs and recovery
    // branches reuse obstacle data exactly where the board permits.
    HierarchyCache hier_cache;

    bool timed_out = false;
    bool budget_hit = false;

    std::vector<char> task_done(tasks.size(), 0);
    // Issue #4: superseded tasks were replaced by regenerated tree growth
    // after a commit/rip-up. They are ignored for failures/connectivity and
    // excluded from the active task count; only active tasks route.
    std::vector<char> task_superseded(tasks.size(), 0);
    std::vector<int> remaining = all_idx;
    // Last epoch in which each task was given a worker attempt (-1 = never).
    std::vector<int> last_epoch(tasks.size(), -1);
    std::map<std::pair<NetId, std::pair<TermId, TermId>>, CandidateRoute> last_attempt;
    std::map<std::pair<NetId, std::pair<TermId, TermId>>, int> attempt_counts;  // P5
    auto attempt_key = [](const ConnectionTask& t) {
        return std::make_pair(t.net, std::make_pair(std::min(t.a, t.b), std::max(t.a, t.b)));
    };

    std::vector<char> net_ok(board_.nets.size(), 1);
    auto net_index = [&](NetId id) -> int {
        for (std::size_t i = 0; i < board_.nets.size(); ++i)
            if (board_.nets[i].id == id) return static_cast<int>(i);
        return -1;
    };

    // (owned was declared with the escape stage above and already holds
    // committed escape stubs; greedy commits append below.)

    // ---- Issue #4: route-tree growth helpers ----
    auto is_multi_net = [&](NetId net) -> bool {
        const NetInfo* n = board_.find_net(net);
        return n && n->terminals.size() > 2;
    };
    auto active_tasks_total = [&]() -> int {
        int c = 0;
        for (std::size_t i = 0; i < tasks.size(); ++i)
            if (!task_superseded[i]) ++c;
        return c;
    };
    // Regenerate one net's tasks from current committed components. Only
    // affected multi-terminal nets are touched (§4). Returns true when the
    // remaining set changed.
    auto regenerate_net = [&](NetId net) -> bool {
        if (!is_multi_net(net)) return false;
        std::vector<int> old_rem;
        for (int ti : remaining)
            if (tasks[ti].net == net) old_rem.push_back(ti);
        RouteTree fresh = build_route_tree(board_, net);
        auto key_of = [](const ConnectionTask& t) {
            std::string key = std::to_string(t.a) + ":" + std::to_string(t.b) + ":";
            key += t.has_copper_target
                       ? std::to_string(t.copper_point.x) + "," +
                             std::to_string(t.copper_point.y) + "," +
                             std::to_string(t.copper_layer)
                       : "-";
            key += ":";
            key += t.has_plane_target
                       ? std::to_string(t.plane_id) + "," +
                             std::to_string(t.plane_point.x) + "," +
                             std::to_string(t.plane_point.y) + "," +
                             std::to_string(t.plane_layer) + "," +
                             std::to_string(t.plane_island)
                       : "-";
            return key;
        };
        std::vector<std::string> old_keys, fresh_keys;
        for (int ti : old_rem) old_keys.push_back(key_of(tasks[ti]));
        for (auto& t : fresh.tasks) {
            if (task_already_connected(board_, t)) continue;
            fresh_keys.push_back(key_of(t));
        }
        std::sort(old_keys.begin(), old_keys.end());
        std::sort(fresh_keys.begin(), fresh_keys.end());
        if (old_keys == fresh_keys) return false;
        // Supersede stale tasks for this net.
        {
            std::set<int> old_set(old_rem.begin(), old_rem.end());
            std::vector<int> kept;
            for (int ti : remaining)
                if (!old_set.count(ti))
                    kept.push_back(ti);
                else {
                    task_superseded[ti] = 1;
                    task_done[ti] = 1;
                }
            remaining = std::move(kept);
        }
        for (auto& t : fresh.tasks) {
            if (task_already_connected(board_, t)) continue;
            ConnectionTask nt = t;
            int pos = static_cast<int>(tasks.size());
            nt.index = static_cast<int>(fresh.tasks.size() > 0 ? pos : pos);
            tasks.push_back(nt);
            task_done.push_back(0);
            task_superseded.push_back(0);
            fail_count.push_back(0);
            last_epoch.push_back(-1);
            DifficultyVector dv = compute_difficulty(board_, resolver_, tasks.back(), ctx,
                                                     density.terminal_density, depth_of, 0);
            tasks.back().difficulty = dv.total;
            // S1: corridor push per commit rebuilds only affected nets (this
            // loop); the cache above dedups identical regenerated tasks. S8:
            // no per-net global aggregate here — tasks_total is refreshed
            // once per affected set by the caller (see both call sites).
            corridors.push_back(cached_corridor(tasks.back()));
            remaining.push_back(pos);
        }
        return true;
    };

    int epoch = 0;
    int stalled = 0;
    int threads_used_max = 1;
    const int workers_cap = std::max(1, effective_threads);

    // ---- Issue #14: board-maturity adaptive search budgets ----
    // Recomputed every greedy epoch and every recovery generation from live
    // board state. A single EffectiveSearchBudget is consumed by A*, the
    // hierarchical guidance, the batch scheduler, reservation/history
    // pressure and recovery (route-K/beam/depth fields are provisioned for
    // #8/#22). Disabled -> legacy fixed budgets (OPEN_BOARD params).
    BoardMaturityState maturity_state;
    EffectiveSearchBudget active_budget;
    bool have_budget = false;
    std::vector<double> recent_accept;  // accepted/candidates, last 4 epochs
    const double escape_done_frac =
        report.escape_stage.pads_total > 0
            ? static_cast<double>(report.escape_stage.pads_escaped) /
                  static_cast<double>(report.escape_stage.pads_total)
            : 1.0;
    auto maturity_timeout_remaining = [&]() -> double {
        if (!(options_.timeout_s > 0)) return -1.0;
        double left = std::chrono::duration<double>(
                          deadline - std::chrono::steady_clock::now())
                          .count();
        return left > 0 ? left : 0.0;
    };
    auto gather_maturity = [&](int stalled_epochs, int ripups, int rec_gens) {
        MaturityInput in;
        in.occupancy_frac =
            options_.maturity.enabled ? copper_occupancy_frac(board_) : 0.0;
        // Issue #14: only observed contention counts as hotspots
        // (history-weighted score at or above the floor). Present-corridor
        // pressure is speculative — every remaining task paints its corridor
        // — while history accumulates only on rejections, failures and
        // recovery, i.e. where routing actually fought. Healthy disjoint
        // routing therefore stays cheap.
        std::vector<Hotspot> hs = congestion.hotspots();
        int nhot = 0;
        double press = 0;
        for (const auto& h : hs) {
            double score = 3.0 * h.history;
            if (score >= options_.maturity.thresholds.hotspot_score_floor) {
                ++nhot;
                press += score;
            }
        }
        in.hotspot_count = nhot;
        in.hotspot_pressure = press;
        double dsum = 0;
        for (int ti : remaining) dsum += tasks[ti].difficulty;
        in.mean_remaining_difficulty =
            remaining.empty() ? 0.0 : dsum / static_cast<double>(remaining.size());
        if (recent_accept.empty()) {
            in.acceptance_rate = 1.0;
        } else {
            double s = 0;
            for (double v : recent_accept) s += v;
            in.acceptance_rate = s / static_cast<double>(recent_accept.size());
        }
        in.stalled_epochs = stalled_epochs;
        in.ripup_count = ripups;
        in.recovery_generations = rec_gens;
        in.fine_pitch_done_frac = escape_done_frac;
        in.remaining_count = static_cast<int>(remaining.size());
        int tot = active_tasks_total();
        in.total_tasks = tot > 0 ? tot : 1;
        in.remaining_frac = remaining.empty()
                                ? 0.0
                                : static_cast<double>(remaining.size()) /
                                      static_cast<double>(in.total_tasks);
        return in;
    };
    auto refresh_budget = [&](int stalled_epochs, int ripups, int rec_gens,
                              int log_epoch, bool is_rec, int gen) {
        if (!options_.maturity.enabled) {
            // Legacy fixed budgets: OPEN_BOARD params over the base config.
            MaturityInput in = gather_maturity(0, 0, 0);
            in.occupancy_frac = 0.0;
            in.hotspot_count = 0;
            in.hotspot_pressure = 0.0;
            in.mean_remaining_difficulty = 0.0;
            in.acceptance_rate = 1.0;
            in.stalled_epochs = 0;
            in.ripup_count = 0;
            in.recovery_generations = 0;
            in.fine_pitch_done_frac = 1.0;
            maturity_state.phase = MaturityPhase::OPEN_BOARD;
            maturity_state.metrics = in;
            maturity_state.phase_entry_remaining = in.remaining_count;
            maturity_state.escalated_by_stall = false;
        } else {
            MaturityInput in = gather_maturity(stalled_epochs, ripups, rec_gens);
            const BoardMaturityState* prev = have_budget ? &maturity_state : nullptr;
            maturity_state = update_maturity(in, options_.maturity.thresholds, prev,
                                             options_.maturity.improvement_frac);
        }
        active_budget = effective_budget_for_phase(
            maturity_state, options_.astar, options_.hierarchy,
            options_.maturity.caps, options_.memory_budget_bytes,
            options_.per_task_bytes, effective_threads,
            maturity_timeout_remaining(), static_cast<int>(remaining.size()));
        have_budget = true;
        MaturityLogEntry e;
        e.epoch = log_epoch;
        e.is_recovery = is_rec;
        e.generation = gen;
        e.state = maturity_state;
        e.budget = active_budget;
        report.maturity_log.push_back(e);
        report.maturity = maturity_state;
        report.budget = active_budget;
        report.has_budget = true;
    };

    while (!remaining.empty() && epoch < options_.max_epochs) {
        if (std::chrono::steady_clock::now() > deadline) {
            timed_out = true;
            for (int i : remaining) {
                RouteFailure f;
                f.net = tasks[i].net;
                const NetInfo* n = board_.find_net(tasks[i].net);
                f.net_name = n ? n->name : "?";
                f.a = tasks[i].a;
                f.b = tasks[i].b;
                f.reason = "timeout";
                fill_plane_fields(f, tasks[i]);
                fill_pair_fields(f, tasks[i], board_);
                if (n) {
                    bool dummy = false;
                    f.required_current_a =
                        resolver_.current().effective_current(*n, board_.defaults, dummy);
                    auto ordered = ViaBundlePlanner::ordered_styles(
                        resolver_, tasks[i].net,
                        LayerSpan{board_.layers.front().id, board_.layers.back().id});
                    if (!ordered.empty()) {
                        f.via_style = ordered.front().name;
                        f.vias_required = ViaBundlePlanner::required_count(
                            resolver_, ordered.front(), tasks[i].net);
                    }
                }
                report.failures.push_back(f);
                int ni = net_index(tasks[i].net);
                if (ni >= 0) net_ok[ni] = 0;
            }
            remaining.clear();
            break;
        }
        const auto t_epoch_start = stage_now();
        refresh_difficulties(remaining);
        // Issue #4: drop tasks whose endpoints are already joined by
        // committed copper (redundant after tree growth). Deterministic.
        {
            std::vector<int> still;
            for (int ti : remaining) {
                if (task_superseded[ti]) continue;
                if (task_already_connected(board_, tasks[ti])) {
                    task_done[ti] = 1;
                } else {
                    still.push_back(ti);
                }
            }
            remaining = std::move(still);
            if (remaining.empty()) break;
            refresh_difficulties(remaining);
        }
        std::vector<ConnectionTask> ordered_tasks;
        std::vector<int> ordered_idx;
        {
            std::vector<int> order = remaining;
            std::sort(order.begin(), order.end(), [&](int a, int b) {
                bool sa = last_epoch[a] < epoch - 1;
                bool sb = last_epoch[b] < epoch - 1;
                if (sa != sb) return sa > sb;
                // Issue #1: centre-out eligibility is structural, not just a
                // difficulty bonus. Deeper fine-pitch tasks order before
                // shallower ones so outer-ring routing cannot leapfrog a
                // deeper unresolved pad.
                int da = task_max_depth(tasks[a]);
                int db = task_max_depth(tasks[b]);
                if (da != db) return da > db;
                if (tasks[a].difficulty != tasks[b].difficulty)
                    return tasks[a].difficulty > tasks[b].difficulty;
                if (tasks[a].net != tasks[b].net) return tasks[a].net < tasks[b].net;
                if (tasks[a].a != tasks[b].a) return tasks[a].a < tasks[b].a;
                if (tasks[a].b != tasks[b].b) return tasks[a].b < tasks[b].b;
                if (tasks[a].has_copper_target != tasks[b].has_copper_target)
                    return tasks[a].has_copper_target < tasks[b].has_copper_target;
                if (tasks[a].copper_point.x != tasks[b].copper_point.x)
                    return tasks[a].copper_point.x < tasks[b].copper_point.x;
                if (tasks[a].copper_point.y != tasks[b].copper_point.y)
                    return tasks[a].copper_point.y < tasks[b].copper_point.y;
                if (tasks[a].copper_layer != tasks[b].copper_layer)
                    return tasks[a].copper_layer < tasks[b].copper_layer;
                // Issue #16: plane targets order deterministically after
                // copper targets (island, plane, entry, layer).
                if (tasks[a].has_plane_target != tasks[b].has_plane_target)
                    return tasks[a].has_plane_target < tasks[b].has_plane_target;
                if (tasks[a].plane_island != tasks[b].plane_island)
                    return tasks[a].plane_island < tasks[b].plane_island;
                if (tasks[a].plane_id != tasks[b].plane_id)
                    return tasks[a].plane_id < tasks[b].plane_id;
                if (tasks[a].plane_point.x != tasks[b].plane_point.x)
                    return tasks[a].plane_point.x < tasks[b].plane_point.x;
                if (tasks[a].plane_point.y != tasks[b].plane_point.y)
                    return tasks[a].plane_point.y < tasks[b].plane_point.y;
                return tasks[a].plane_layer < tasks[b].plane_layer;
            });
            for (int i : order) {
                ordered_idx.push_back(i);
                ordered_tasks.push_back(tasks[i]);
            }
        }
        // Issue #3: interference-aware greedy batch over the scheduler order
        // (starvation/depth/difficulty/net/a/b above, with fail_count age
        // bonuses folded into difficulty by refresh_difficulties). Starts from
        // the highest-value eligible task, then adds each later task in order
        // when its max interference vs the selected set stays below the
        // configurable threshold. One task per net per epoch (issue #4: same-
        // net tasks must see each other's copper). When the batch cannot
        // fill, the threshold relaxes progressively; a final infinite pass
        // guarantees workers never idle and deferred tasks eventually run.
        // Corridor buffers are reused across epochs (built once, appended for
        // regenerated tasks); interference rows stream on demand with no dense
        // N^2 allocation, and the width respects the memory bound.
        //
        // Issue #14: maturity is recomputed from the live board BEFORE batch
        // selection so the single effective budget drives the batch width,
        // the epoch A*/hierarchy configs, reservation strength and history
        // growth below. Greedy epochs never rip, so ripups/gens are 0 here.
        refresh_budget(stalled, 0, 0, epoch, false, -1);
        AStarConfig epoch_astar = options_.astar;
        epoch_astar.max_expansions = active_budget.astar_max_expansions;
        epoch_astar.weight_factor = active_budget.weight_factor;
        HierarchyConfig epoch_hier = options_.hierarchy;
        epoch_hier.max_coarse_expansions = active_budget.hier_max_coarse_expansions;
        epoch_hier.max_window_attempts = active_budget.hier_window_attempts;
        epoch_hier.tube_half_nm = active_budget.hier_tube_half_nm;
        epoch_hier.max_grid_cells = active_budget.hier_max_grid_cells;
        BatchSchedOptions bsched;
        bsched.batch_width = active_budget.batch_width;
        bsched.interference_threshold = options_.batch_interference_threshold;
        bsched.relax_factor = options_.batch_relax_factor;
        bsched.max_relax_steps = options_.batch_max_relax_steps;
        bsched.memory_budget_bytes = options_.memory_budget_bytes;
        bsched.per_task_bytes = options_.per_task_bytes;
        BatchSelection bsel =
            select_interference_batch(ordered_idx, tasks, corridors, bsched);
        std::vector<int> batch_idx = bsel.selected;
        std::size_t batch_n = batch_idx.size();
        for (int ti : batch_idx) last_epoch[ti] = epoch;

        congestion.reset_present();
        for (int i : remaining) {
            double w = 1.0 + tasks[i].difficulty / 20.0;
            congestion.add_present_corridor(corridors[i].rect, w);
        }
        std::vector<Corridor> rem_corr;
        std::vector<double> rem_diff;
        std::map<int, std::size_t> rem_pos;
        for (std::size_t k = 0; k < ordered_idx.size(); ++k) {
            rem_pos[ordered_idx[k]] = k;
            rem_corr.push_back(corridors[ordered_idx[k]]);
            rem_diff.push_back(tasks[ordered_idx[k]].difficulty);
        }
        reservations.build(ordered_tasks, rem_corr, rem_diff);
        // Issue #14: maturity-driven reservation strength (soft cost only).
        reservations.set_strength(active_budget.reservation_strength);

        auto epoch_t0 = std::chrono::steady_clock::now();
        ms_batch += stage_ms(t_epoch_start, epoch_t0);
        std::vector<CandidateRoute> candidates(batch_n);
        const Board& snapshot = board_;
        const std::size_t traces_before = snapshot.traces.size();
        const std::size_t vias_before = snapshot.vias.size();
        int workers = std::max(1, std::min<int>(workers_cap, static_cast<int>(batch_n)));
        threads_used_max = std::max(threads_used_max, workers);
        auto worker_fn = [&](int w) {
            for (std::size_t k = w; k < batch_n; k += workers) {
                int ti = batch_idx[k];
                // Issue #8 + #9: dense phases spend the maturity route-K on
                // a deterministic diverse portfolio. Issue #9 scores every
                // exactly-legal member with the future-obstruction scorer
                // (remaining-task corridors, density, electrical burden) and
                // commits the lexicographic best (legal, obstruction, cost,
                // signature). Disabled (--no-impact) reverts to the cheapest
                // legal member under the identical legality gate. Pair
                // corridors stay single (atomic).
                auto score_single_for_recovery = [&](CandidateRoute& c) {
                    // Expose the obstruction on single-path candidates too
                    // so multi-ply recovery (#22) reads it without recompute.
                    if (!options_.impact.enabled || !c.found) return;
                    ImpactContext sctx;
                    sctx.board = &snapshot;
                    sctx.resolver = &resolver_;
                    sctx.ctx = &ctx;
                    for (int rj : remaining) {
                        if (rj == ti) continue;
                        sctx.remaining_tasks.push_back(tasks[rj]);
                        sctx.remaining_corridors.push_back(corridors[rj]);
                    }
                    sctx.terminal_density = density.terminal_density;
                    sctx.centre_depth = depth_of;
                    sctx.congestion = &congestion;
                    auto scorer = make_impact_scorer(options_.impact);
                    ImpactScore s = scorer->score(c, sctx);
                    c.impact_obstruction = s.total;
                    c.has_impact_score = true;
                    c.impact_detail = s.to_json();
                };
                // Issue #4: maturity graph budget for this epoch (higher
                // maturity -> larger max_bases / k_nearest; OPEN keeps
                // the legacy 384/16 defaults). Lives outside the worker
                // lambda body so every thread shares the identical object.
                SparseGraphBudget epoch_graph_budget;
                epoch_graph_budget.max_bases = active_budget.graph_max_bases;
                epoch_graph_budget.k_nearest = active_budget.graph_k_nearest;
                if (active_budget.route_k > 1 && !tasks[ti].is_pair_corridor) {
                    PortfolioOptions po;
                    po.requested_k = active_budget.route_k;
                    po.max_k = options_.maturity.caps.max_route_k;
                    po.memory_budget_bytes = options_.memory_budget_bytes;
                    po.per_task_bytes = options_.per_task_bytes;
                    po.per_alt_bytes = kPerPortfolioAltBytes;
                    po.batch_width = active_budget.batch_width;
                    po.budget_route_k = active_budget.route_k;
                    po.threads_requested = effective_threads;
                    po.astar = epoch_astar;
                    po.hier = epoch_hier;
                    po.has_graph_budget = true;
                    po.graph_budget = epoch_graph_budget;
                    PortfolioResult pf = build_portfolio(
                        snapshot, resolver_, tasks[ti], rem_pos[ti],
                        tasks[ti].difficulty, ctx, layer_mult, epoch_astar,
                        congestion, reservations, epoch_hier, &hier_cache, po);
                    if (!pf.candidates.empty()) {
                        int best = 0;
                        std::vector<ImpactScore> iscores;
                        if (options_.impact.enabled) {
                            // Streamed scoring: remaining corridors borrowed,
                            // O(1) scratch per candidate, no dense matrices.
                            ImpactContext ictx;
                            ictx.board = &snapshot;
                            ictx.resolver = &resolver_;
                            ictx.ctx = &ctx;
                            for (int rj : remaining) {
                                if (rj == ti) continue;
                                ictx.remaining_tasks.push_back(tasks[rj]);
                                ictx.remaining_corridors.push_back(corridors[rj]);
                            }
                            ictx.terminal_density = density.terminal_density;
                            ictx.centre_depth = depth_of;
                            ictx.congestion = &congestion;
                            ImpactOptions iopt = options_.impact;
                            iopt.threads_requested = effective_threads;
                            iopt.memory_budget_bytes = options_.memory_budget_bytes;
                            std::vector<CandidateRoute> routes;
                            routes.reserve(pf.candidates.size());
                            for (const auto& pc : pf.candidates) routes.push_back(pc.route);
                            best = score_and_select(routes, ictx, iopt, iscores);
                            if (best < 0) best = 0;
                        }
                        candidates[k] = pf.candidates[best].route;
                        // Full portfolio search work, not just the winner.
                        candidates[k].expansions = pf.total_expansions;
                        candidates[k].hierarchy.coarse_expansions =
                            pf.coarse_expansions;
                        candidates[k].hierarchy.attempted = true;
                        if (options_.impact.enabled && !iscores.empty() &&
                            best < (int)iscores.size()) {
                            candidates[k].impact_obstruction = iscores[best].total;
                            candidates[k].has_impact_score = true;
                            candidates[k].impact_detail = iscores[best].to_json();
                        }
                    } else {
                        candidates[k] = route_candidate_task(
                            snapshot, resolver_, tasks[ti], rem_pos[ti],
                            tasks[ti].difficulty, ctx, layer_mult, epoch_astar,
                            congestion, reservations, epoch_hier, &hier_cache,
                            &epoch_graph_budget);
                        score_single_for_recovery(candidates[k]);
                    }
                } else {
                    candidates[k] = route_candidate_task(
                        snapshot, resolver_, tasks[ti], rem_pos[ti],
                        tasks[ti].difficulty, ctx, layer_mult, epoch_astar,
                        congestion, reservations, epoch_hier, &hier_cache,
                        &epoch_graph_budget);
                    score_single_for_recovery(candidates[k]);
                }
            }
        };
        if (workers == 1) {
            worker_fn(0);
        } else {
            std::vector<std::thread> pool;
            for (int w = 0; w < workers; ++w) pool.emplace_back(worker_fn, w);
            for (auto& th : pool) th.join();
        }
        const auto t_workers_end = stage_now();
        ms_workers += stage_ms(epoch_t0, t_workers_end);
        if (snapshot.traces.size() != traces_before || snapshot.vias.size() != vias_before) {
            RouteFailure f;
            f.reason = "internal_worker_mutation";
            f.blockers.push_back("worker mutated committed snapshot");
            report.failures.push_back(f);
            break;
        }

        ArbiterResult arb = arbitrate(candidates, board_, resolver_, ctx);
        commit_candidates(board_, candidates, arb, report.stats);
        // Record ownership for the accepted candidates (deterministic).
        // Issue #1: global routes are never escape stubs; only the
        // pre-committed escape stage owns is_escape_stub=true geometry.
        // Issue #12: accepted pair corridors are owned atomically as one
        // object (both members live or die together).
        for (std::size_t k : arb.accepted) {
            const CandidateRoute& c = candidates[k];
            int ti = batch_idx[k];
            OwnedRoute o;
            o.task = tasks[ti];
            o.task_pos = ti;
            o.traces = c.traces;
            o.vias = c.vias;
            o.epoch_committed = epoch;
            o.stable_epochs = 0;
            o.is_escape_stub = false;
            o.is_pair_corridor = tasks[ti].is_pair_corridor;
            o.pair_id = tasks[ti].pair_id;
            o.protection =
                route_protection_score(false, tasks[ti].difficulty, 0,
                                       /*is_fixed=*/false, RecoveryMode::FAST);
            owned.push_back(o);
        }
        for (auto& o : owned) o.stable_epochs++;
        report.stats.candidates_total += static_cast<int>(batch_n);
        report.stats.candidates_accepted += static_cast<int>(arb.accepted.size());
        report.stats.candidates_rejected += static_cast<int>(arb.rejected.size());

        std::vector<char> in_batch(tasks.size(), 0);
        for (int ti : batch_idx) in_batch[ti] = 1;
        std::map<int, std::string> reject_reason;
        for (std::size_t k = 0; k < arb.rejected.size(); ++k) {
            int ti = batch_idx[arb.rejected[k]];
            reject_reason[ti] = arb.reject_reason[k];
        }
        for (std::size_t k = 0; k < batch_n; ++k) {
            int ti = batch_idx[k];
            last_attempt[attempt_key(tasks[ti])] = candidates[k];
            attempt_counts[attempt_key(tasks[ti])]++;  // Prompt 5: failure report
        }
        for (std::size_t k = 0; k < arb.rejected.size(); ++k) {
            const CandidateRoute& c = candidates[arb.rejected[k]];
            // Issue #14: Pathfinder history growth scales with maturity.
            const double hg = active_budget.history_growth;
            if (c.found) {
                for (const auto& t : c.traces) congestion.add_history_segment(t.segment(), 1.0 * hg);
                for (const auto& vv : c.vias)
                    congestion.add_history_rect(
                        Rect::from_center_size(vv.pos, vv.outer_d_nm, vv.outer_d_nm), 1.0 * hg);
            } else {
                int ti = batch_idx[arb.rejected[k]];
                congestion.add_history_rect(corridors[ti].rect, 0.5 * hg);
                if (c.fail_reason == "budget_exhausted") budget_hit = true;
            }
        }

        std::vector<int> next_remaining;
        for (int ti : remaining) {
            bool accepted = false;
            for (std::size_t k : arb.accepted) {
                if (batch_idx[k] == ti) {
                    accepted = true;
                    break;
                }
            }
            if (accepted) {
                task_done[ti] = 1;
            } else {
                if (ti < (int)in_batch.size() && in_batch[ti]) fail_count[ti]++;
                next_remaining.push_back(ti);
            }
        }
        remaining = std::move(next_remaining);

        // Issue #4: grow affected multi-terminal nets from the new copper.
        // Only nets with an accepted commit are regenerated (§4). S8: the
        // active-task aggregate is refreshed once here, not per net.
        if (!arb.accepted.empty()) {
            std::set<NetId> affected;
            for (std::size_t k : arb.accepted) affected.insert(tasks[batch_idx[k]].net);
            bool grown = false;
            for (NetId net : affected) grown = regenerate_net(net) || grown;
            if (grown) report.stats.tasks_total = active_tasks_total();
        }

        auto epoch_t1 = std::chrono::steady_clock::now();
        ms_arbiter += stage_ms(t_workers_end, epoch_t1);
        EpochInfo info;
        info.epoch = epoch;
        info.batch_size = static_cast<int>(batch_n);
        info.candidates = static_cast<int>(batch_n);
        info.accepted = static_cast<int>(arb.accepted.size());
        info.rejected = static_cast<int>(arb.rejected.size());
        std::int64_t ep_exp = 0;
        for (const auto& c : candidates) ep_exp += c.expansions;
        info.expansions = ep_exp;
        info.workers = workers;
        info.time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(epoch_t1 - epoch_t0).count();
        // Issue #3: batch IDs + pairwise scores + width/threshold stats.
        info.batch_task_ids = bsel.selected;
        info.interference_pairs = bsel.pair_scores;
        info.interference_threshold_used = bsel.threshold_used;
        info.interference_relax_steps = bsel.relax_steps;
        info.interference_max = bsel.max_interference;
        info.interference_mean = bsel.mean_interference;
        info.effective_batch_width = bsel.effective_width;
        // Issue #14: phase + maturity metrics + effective hyperparams.
        info.maturity_phase = maturity_phase_name(maturity_state.phase);
        info.maturity = maturity_state.to_json();
        info.budget = active_budget.to_json();
        report.epochs.push_back(info);
        if (options_.progress) {
            JsonValue ev = info.to_json();
            ev["event"] = "epoch";
            ev["remaining"] = static_cast<double>(remaining.size());
            options_.progress(ev);
        }
        // Issue #14: acceptance history feeds the next epoch's maturity.
        recent_accept.push_back(batch_n > 0 ? static_cast<double>(arb.accepted.size()) /
                                                  static_cast<double>(batch_n)
                                            : 1.0);
        if (recent_accept.size() > 4) recent_accept.erase(recent_accept.begin());

        if (arb.accepted.empty()) {
            if (++stalled > kMaxStalledEpochs) break;
        } else {
            stalled = 0;
        }
        ++epoch;
    }

    // ---- Prompt 4: rip-up/reroute meta-search after greedy stalls ----
    RecoveryInfo rec;
    TranspositionTable transposition;
    HistoryHeuristic history;
    std::string pv_key;
    std::map<std::pair<NetId, std::pair<TermId, TermId>>, int> ripup_attempts;
    // Seed transposition with the post-greedy state.
    transposition.record(state_hash128(board_, tasks, remaining), (int)remaining.size());
    const auto t_recovery_start = stage_now();
    if (!remaining.empty() && options_.enable_ripup && !timed_out) {
        StallDetector stall;
        stall.stalled_epochs = stalled;
        int max_gen = std::max(0, options_.max_ripup_generations);
        for (int gen = 0; gen < max_gen && !remaining.empty(); ++gen) {
            if (std::chrono::steady_clock::now() > deadline) {
                timed_out = true;
                break;
            }
            RecoveryMode mode = recovery_mode_for_generation(gen);
            std::string mode_name = recovery_mode_name(mode);
            if (std::find(rec.modes_attempted.begin(), rec.modes_attempted.end(), mode_name) ==
                rec.modes_attempted.end())
                rec.modes_attempted.push_back(mode_name);
            AStarConfig mode_cfg;
            HierarchyConfig rec_hier;
            int width = 0;
            int breadth = 0;
            {
                // Issue #14: maturity recomputed per recovery generation from
                // live copper/stall/rip-up state. The generation consumes the
                // single effective budget: the mode escalation multiplies the
                // maturity-scaled A* headroom, and maturity floors deepen the
                // branch/rip search on dense boards only (OPEN floors equal
                // the legacy generation schedule). Explicit user caps win.
                refresh_budget(stall.stalled_epochs, rec.ripups, rec.generations,
                               static_cast<int>(report.epochs.size()), true, gen);
                AStarConfig budget_base = options_.astar;
                budget_base.max_expansions = active_budget.astar_max_expansions;
                budget_base.weight_factor = active_budget.weight_factor;
                mode_cfg = astar_config_for_mode(budget_base, mode);
                rec_hier = options_.hierarchy;
                rec_hier.max_coarse_expansions =
                    active_budget.hier_max_coarse_expansions;
                rec_hier.max_window_attempts = active_budget.hier_window_attempts;
                rec_hier.tube_half_nm = active_budget.hier_tube_half_nm;
                rec_hier.max_grid_cells = active_budget.hier_max_grid_cells;
                width = std::min(std::max(branch_width_for_generation(gen),
                                          active_budget.recovery_branches),
                                 options_.max_ripup_branches);
                breadth = std::max(max_rip_breadth_for_generation(gen),
                                   active_budget.ripup_breadth);
                if (breadth > options_.maturity.caps.max_rip_breadth)
                    breadth = options_.maturity.caps.max_rip_breadth;
            }
            // Issue #22: effective multi-ply depth/beam for this generation.
            // Requested values come from EngineOptions (0 = auto = defaults);
            // maturity allowance + explicit caps + memory bound the effective
            // values. Stored boards stay within depth*beam streaming caps.
            int req_depth = options_.recovery_depth <= 0 ? kDefaultMultiplyDepth
                                                         : options_.recovery_depth;
            int req_beam = options_.recovery_beam <= 0 ? kDefaultMultiplyBeam
                                                       : options_.recovery_beam;
            int eff_depth =
                effective_multiply_depth(req_depth, active_budget, options_.maturity.caps);
            int eff_beam = effective_multiply_beam(req_beam, active_budget,
                                                   options_.maturity.caps,
                                                   options_.memory_budget_bytes);
            while (eff_depth * eff_beam > kMaxStoredMultiplyNodes && eff_beam > 1)
                --eff_beam;
            if (rec.multiply_depth_requested == kDefaultMultiplyDepth &&
                rec.multiply_beam_requested == kDefaultMultiplyBeam &&
                rec.generations == 0) {
                rec.multiply_depth_requested = req_depth;
                rec.multiply_beam_requested = req_beam;
            } else {
                rec.multiply_depth_requested = req_depth;
                rec.multiply_beam_requested = req_beam;
            }
            rec.multiply_depth_effective = eff_depth;
            rec.multiply_beam_effective = eff_beam;

            // Refresh difficulties so previous failures + congestion count.
            refresh_difficulties(remaining);
            DependencyGraph graph;
            // S2: blocker attribution (the O(remaining x copper) corridor
            // rescan inside attribute_blockers_detailed) runs in parallel
            // indexed slots; graph assembly stays serial in failed-task
            // order with the identical collapse + sort as
            // build_dependency_graph (recovery.cpp), so the committed branch
            // choice (and the reported dependency_graph) never varies with
            // --threads. workers == 1 keeps the original serial call.
            int attr_workers =
                std::max(1, std::min<int>(workers_cap, static_cast<int>(remaining.size())));
            if (attr_workers <= 1) {
                graph = build_dependency_graph(board_, resolver_, ctx, tasks,
                                               remaining, last_attempt);
            } else {
                threads_used_max = std::max(threads_used_max, attr_workers);
                std::vector<std::vector<BlockerHit>> attr_hits(remaining.size());
                auto attr_fn = [&](int w) {
                    for (std::size_t fi = static_cast<std::size_t>(w);
                         fi < remaining.size();
                         fi += static_cast<std::size_t>(attr_workers)) {
                        int ti = remaining[fi];
                        const ConnectionTask& task = tasks[ti];
                        const Terminal* ta = board_.find_terminal(task.a);
                        TraceRule rule = resolver_.traceRule(
                            task.net, ta ? ta->layer : 0, kAnyRegion);
                        auto it = last_attempt.find(
                            std::make_pair(task.net,
                                           std::make_pair(std::min(task.a, task.b),
                                                          std::max(task.a, task.b))));
                        const CandidateRoute* last =
                            it != last_attempt.end() ? &it->second : nullptr;
                        attr_hits[fi] = attribute_blockers_detailed(
                            board_, task, rule.pref_width_nm, last);
                    }
                };
                std::vector<std::thread> attr_pool;
                for (int w = 0; w < attr_workers; ++w)
                    attr_pool.emplace_back(attr_fn, w);
                for (auto& th : attr_pool) th.join();
                // Serial assembly: verbatim mirror of build_dependency_graph
                // over the precomputed per-task hits (collapse to one edge
                // per blocker net, deterministic; same final sort).
                for (int ti : remaining) graph.failed.push_back(tasks[ti]);
                for (std::size_t fi = 0; fi < remaining.size(); ++fi) {
                    const ConnectionTask& task = tasks[remaining[fi]];
                    std::map<NetId, DependencyEdge> best;
                    for (const auto& h : attr_hits[fi]) {
                        if (h.kind != "trace" && h.kind != "pad" && h.kind != "via") {
                            if (h.kind == "keepout" || h.kind == "frontier" ||
                                h.kind == "bounds") {
                                DependencyEdge e;
                                e.failed_pos = static_cast<int>(fi);
                                e.failed_net = task.net;
                                e.blocker_net = -1;
                                e.blocker_desc = h.desc;
                                e.weight = 0.0;
                                graph.edges.push_back(e);
                            }
                            continue;
                        }
                        auto b = best.find(h.net);
                        double wgt = static_cast<double>(h.area);
                        if (b == best.end() || wgt > b->second.weight) {
                            DependencyEdge e;
                            e.failed_pos = static_cast<int>(fi);
                            e.failed_net = task.net;
                            e.blocker_net = h.net;
                            e.blocker_desc = h.desc;
                            e.weight = wgt;
                            best[h.net] = e;
                        }
                    }
                    for (const auto& kv : best) graph.edges.push_back(kv.second);
                }
                std::sort(graph.edges.begin(), graph.edges.end(),
                          [](const DependencyEdge& a, const DependencyEdge& b) {
                              if (a.weight != b.weight) return a.weight > b.weight;
                              if (a.failed_net != b.failed_net)
                                  return a.failed_net < b.failed_net;
                              return a.blocker_net < b.blocker_net;
                          });
            }
            rec.last_graph = graph;
            if (graph.edges.empty()) break;  // keepout-only: nothing to rip

            std::vector<ConnectionTask> failed_tasks = graph.failed;
            std::vector<RipupMove> moves = generate_ripup_moves(
                failed_tasks, graph, owned, history, pv_key, mode, width, breadth);
            if (moves.empty()) break;

            // Count rip-up attempts per failed task for the failure report.
            for (const auto& m : moves) {
                auto k = std::make_pair(m.failed_task.net, std::make_pair(std::min(m.failed_task.a, m.failed_task.b),
                                                                          std::max(m.failed_task.a, m.failed_task.b)));
                ripup_attempts[k]++;
            }

            // Parallel speculative branches: indexed slots, deterministic pick.
            int branch_n = (int)moves.size();
            int workers = std::max(1, std::min<int>(workers_cap, branch_n));
            threads_used_max = std::max(threads_used_max, workers);
            std::vector<BranchResult> results(branch_n);
            auto branch_fn = [&](int w) {
                for (int b = w; b < branch_n; b += workers) {
                    const RipupMove& m = moves[b];
                    // Surviving owned copper (rip set removed).
                    std::set<int> rip(m.owned_idx.begin(), m.owned_idx.end());
                    std::vector<OwnedRoute> surviving;
                    for (std::size_t i = 0; i < owned.size(); ++i)
                        if (!rip.count((int)i)) surviving.push_back(owned[i]);
                    // Tasks to retry: the failed task + every ripped task.
                    // Issue #1: ripped escape stubs (task_pos=-1) map to the
                    // global tasks covering their terminal/net so the pad can
                    // reconnect; raw -1 indices are never routed.
                    std::vector<int> to_route;
                    to_route.push_back(remaining[m.failed_pos]);
                    for (int oi : m.owned_idx) {
                        int tp = owned[oi].task_pos;
                        if (tp >= 0) {
                            to_route.push_back(tp);
                        } else {
                            TermId stub_term = owned[oi].task.a;
                            NetId stub_net = owned[oi].task.net;
                            for (std::size_t ti = 0; ti < tasks.size(); ++ti) {
                                if (task_superseded[ti]) continue;
                                if (task_done[ti]) continue;
                                if (tasks[ti].net != stub_net) continue;
                                if (tasks[ti].a == stub_term || tasks[ti].b == stub_term)
                                    to_route.push_back(static_cast<int>(ti));
                            }
                        }
                    }
                    std::sort(to_route.begin(), to_route.end());
                    to_route.erase(std::unique(to_route.begin(), to_route.end()),
                                   to_route.end());
                    int failed_ti = remaining[m.failed_pos];
                    BranchResult r = reroute_branch(
                        board_, fixed_traces, fixed_vias, surviving, to_route, failed_ti,
                        tasks, remaining, corridors, resolver_, ctx, layer_mult, mode_cfg,
                        congestion, mode_name, m, rec_hier, &hier_cache,
                        active_budget.reservation_strength);
                    // Issue #18: no transposition access on worker threads.
                    // The worker returns the outcome + exact remaining_task_ids
                    // and exact hash; pruning/recording happens serially on
                    // the arbiter after join, in move-index order.
                    results[b] = r;
                }
            };
            if (workers == 1) {
                branch_fn(0);
            } else {
                std::vector<std::thread> pool;
                for (int w = 0; w < workers; ++w) pool.emplace_back(branch_fn, w);
                for (auto& th : pool) th.join();
            }
            // Deterministic single-thread transposition pass, in move-index
            // order, using the exact (hash, remaining-set) state identity.
            for (int b = 0; b < branch_n; ++b) {
                BranchResult& r = results[b];
                if (!r.evaluated) continue;
                int undone = (int)r.remaining_task_ids.size();
                if (transposition.should_prune(r.hash, undone)) {
                    r.pruned = true;
                } else {
                    transposition.record(r.hash, undone);
                }
            }
            rec.branches_evaluated += branch_n;
            int pruned = 0;
            for (const auto& r : results)
                if (r.pruned) ++pruned;
            rec.branches_pruned += pruned;

            // Deterministic one-ply selection: first strictly-best wins.
            int one_ply_best = -1;
            for (int b = 0; b < branch_n; ++b) {
                if (!results[b].evaluated || results[b].pruned) continue;
                if (one_ply_best < 0 || branch_better(results[b], results[one_ply_best]))
                    one_ply_best = b;
            }
            if (one_ply_best < 0) {
                rec.transposition_hits = transposition.hits();
                break;  // everything pruned: give up deterministically
            }
            // Issue #22: bounded multi-ply lookahead over immutable branch
            // states. Depth=1 reproduces one-ply exactly (no lookahead).
            // Otherwise the beam search ranks leaves by the same global
            // objective and returns the first action of the best PV; only
            // that first action's one-ply branch is committed below, then
            // the engine replans from the new real board. Budget exhaustion
            // falls back to the one-ply best.
            int best = one_ply_best;
            {
                rec.multiply_threads_effective =
                    std::max(rec.multiply_threads_effective, workers);
                if (eff_depth > 1) {
                    MultiPlyContext mctx;
                    mctx.tasks = &tasks;
                    mctx.corridors = &corridors;
                    mctx.resolver = &resolver_;
                    mctx.ectx = &ctx;
                    mctx.layer_mult = &layer_mult;
                    mctx.astar_cfg = mode_cfg;
                    mctx.congestion_tpl = congestion;
                    mctx.mode_name = mode_name;
                    mctx.hier_cfg = rec_hier;
                    mctx.hier_cache = &hier_cache;
                    mctx.reservation_strength = active_budget.reservation_strength;
                    mctx.fixed_traces = fixed_traces;
                    mctx.fixed_vias = fixed_vias;
                    mctx.last_attempt = last_attempt;
                    mctx.history = history;
                    mctx.pv_key = pv_key;
                    mctx.mode = mode;
                    mctx.max_moves = width;
                    mctx.max_breadth = breadth;
                    MultiPlyConfig mcfg;
                    mcfg.depth = eff_depth;
                    mcfg.beam = eff_beam;
                    mcfg.max_nodes = options_.max_multiply_nodes;
                    mcfg.max_moves_per_node = width;
                    mcfg.max_breadth = breadth;
                    mcfg.threads = workers;
                    MultiPlyResult mpres = multiply_beam_search(
                        moves, results, mctx, mcfg, transposition, deadline);
                    report.stats.expansions_total += mpres.expansions_total;
                    rec.multiply_nodes_evaluated += mpres.nodes_evaluated;
                    rec.multiply_nodes_pruned += mpres.nodes_pruned;
                    rec.branches_evaluated += mpres.nodes_evaluated;
                    rec.branches_pruned += mpres.nodes_pruned;
                    if (mpres.searched) {
                        if (!mpres.fallback_to_one_ply &&
                            mpres.best_first_move >= 0 &&
                            mpres.best_first_move < branch_n &&
                            results[mpres.best_first_move].evaluated &&
                            !results[mpres.best_first_move].pruned) {
                            best = mpres.best_first_move;
                            rec.multiply_fallback_to_one_ply = false;
                        } else {
                            best = one_ply_best;
                            rec.multiply_fallback_to_one_ply = true;
                        }
                        // Diagnostics: best PV reasons + leaf hash (last
                        // generation wins; cumulative nodes above).
                        rec.multiply_pv.clear();
                        rec.multiply_best_hash.clear();
                        if (!mpres.fallback_to_one_ply && mpres.has_best_leaf) {
                            for (const auto& mv : mpres.best_pv)
                                rec.multiply_pv.push_back(mv.reason);
                            rec.multiply_best_hash =
                                mpres.best_leaf.hash.to_hex();
                        } else {
                            rec.multiply_pv.push_back(moves[best].reason);
                            rec.multiply_best_hash =
                                results[best].hash.to_hex();
                        }
                    } else {
                        best = one_ply_best;
                        rec.multiply_fallback_to_one_ply = true;
                        rec.multiply_pv.clear();
                        rec.multiply_pv.push_back(moves[best].reason);
                        rec.multiply_best_hash = results[best].hash.to_hex();
                    }
                } else {
                    best = one_ply_best;
                    rec.multiply_pv.clear();
                    rec.multiply_pv.push_back(moves[best].reason);
                    rec.multiply_best_hash = results[best].hash.to_hex();
                }
            }
            const BranchResult& win = results[best];
            // Progress test: the failed task must be among newly_done AND
            // every ripped task must have been rerouted (net remaining
            // strictly decreases). Committing a branch that strands ripped
            // copper would regress connectivity, violating the lexicographic
            // objective.
            int failed_ti = remaining[moves[best].failed_pos];
            bool failed_done = std::find(win.newly_done.begin(), win.newly_done.end(),
                                         failed_ti) != win.newly_done.end();
            int need_total = (int)moves[best].owned_idx.size() + 1;  // ripped + failed
            bool fully_rerouted = win.connected_tasks >= need_total;
            if (!failed_done || !fully_rerouted) {
                // No branch reconnected its failed task: escalate (stronger
                // history pressure on the failed corridors) and widen next gen.
                // Issue #14: history growth scales with maturity.
                for (int ti : remaining)
                    congestion.add_history_rect(corridors[ti].rect,
                                                1.0 * active_budget.history_growth);
                if (mode == RecoveryMode::EXHAUSTIVE_LOCAL) {
                    // Exhaustive still failed: record and stop (report budget).
                    for (const auto& r : results) report.stats.expansions_total += r.expansions;
                    budget_hit = true;
                    break;
                }
                for (const auto& r : results) report.stats.expansions_total += r.expansions;
                stall.note_epoch(0);
                ++rec.generations;
                continue;
            }
            // Commit the winning branch atomically: replace copper + ownership.
            // Issue #4: capture ripped nets BEFORE replacing owned.
            std::set<NetId> ripped_nets;
            for (int oi : moves[best].owned_idx)
                if (oi >= 0 && oi < (int)owned.size()) ripped_nets.insert(owned[oi].task.net);
            board_ = win.board;
            resolver_.rebind(&board_);
            owned = win.owned;
            for (auto& o : owned) {
                // Recompute protection under the winning mode; stable routes
                // accumulate protection over generations. Issue #1: escape
                // stubs (task_pos=-1) keep real escape protection; never
                // index tasks[-1].
                o.stable_epochs++;
                double diff = 0.0;
                if (o.task_pos >= 0 && o.task_pos < static_cast<int>(tasks.size()))
                    diff = tasks[o.task_pos].difficulty;
                o.protection = route_protection_score(o.is_escape_stub, diff,
                                                      o.stable_epochs, false, mode);
            }
            // Update done/remaining deterministically.
            // Issue #19: only false->true transitions count toward
            // tasks_routed. Rerouted ripped routes were already counted in
            // the greedy phase; re-adding them would let tasks_routed exceed
            // tasks_total after multiple generations.
            // Issue #4: superseded tasks never count toward routed.
            for (int ti : win.newly_done) task_done[ti] = 1;
            {
                int routed = 0;
                for (std::size_t i = 0; i < task_done.size(); ++i)
                    if (task_done[i] && !task_superseded[i]) ++routed;
                // Newly routed ripped tasks were already counted: clamp to
                // active total so regen never inflates the counter.
                int active = 0;
                for (char s : task_superseded)
                    if (!s) ++active;
                report.stats.tasks_total = active;
                report.stats.tasks_routed = std::min(routed, active);
            }
            // Fail counts: only the tasks involved in this move that are
            // still not done count another failure (avoids inflating every
            // remaining task's difficulty each generation).
            {
                int failed_pos_ti = remaining[moves[best].failed_pos];
                if (!task_done[failed_pos_ti]) fail_count[failed_pos_ti]++;
            }
            std::vector<int> next;
            for (int ti : remaining)
                if (!task_done[ti]) next.push_back(ti);
            remaining = std::move(next);
            // Issue #4: rip-up changes components; regenerated tasks reflect
            // the new topology. Only affected nets are touched.
            {
                std::set<NetId> affected = ripped_nets;
                for (int ti : win.newly_done) affected.insert(tasks[ti].net);
                for (const auto& o : win.owned) affected.insert(o.task.net);
                affected.insert(moves[best].failed_task.net);
                if (moves[best].blocker_net >= 0) affected.insert(moves[best].blocker_net);
                bool grown = false;
                for (NetId net : affected) grown = regenerate_net(net) || grown;
                if (grown) {
                    int active = active_tasks_total();
                    report.stats.tasks_total = active;
                }
                // Refresh remaining after regen (regenerate_net already
                // rebuilt it); drop any newly-redundant tasks.
                std::vector<int> still;
                for (int ti : remaining) {
                    if (task_superseded[ti]) continue;
                    if (task_already_connected(board_, tasks[ti])) {
                        task_done[ti] = 1;
                    } else {
                        still.push_back(ti);
                    }
                }
                remaining = std::move(still);
            }
            // History reward + PV reuse for the next generation.
            {
                const RipupMove& m = moves[best];
                history.reward(HistoryHeuristic::move_key(m.failed_task, m.blocker_net), 1.0);
                pv_key = HistoryHeuristic::move_key(m.failed_task, m.blocker_net);
            }
            report.stats.expansions_total += win.expansions;
            // Count all branches' search work (not just the winner).
            for (int b = 0; b < branch_n; ++b) {
                if (b == best) continue;
                report.stats.expansions_total += results[b].expansions;
            }
            // S7: incremental stats — the winning branch already measured its
            // global copper (length_nm/via_count over surviving + rerouted
            // owned). Reuse it O(1) instead of rescanning all owned routes.
            // Identical values: win.owned becomes owned verbatim above.
            report.stats.length_nm = win.length_nm;
            report.stats.via_count = win.via_count;
            // Epoch log entry for the generation (agents observe it).
            // Continue numbering past the greedy epochs (no duplicates).
            {
                EpochInfo info;
                info.epoch = static_cast<int>(report.epochs.size());
                epoch = info.epoch + 1;
                info.batch_size = (int)win.newly_done.size();
                info.candidates = branch_n;
                info.accepted = (int)win.newly_done.size();
                info.rejected = branch_n - 1;
                info.expansions = win.expansions;
                info.workers = workers;
                info.time_ms = 0;
                // Issue #14: the generation's maturity/budget snapshot.
                info.maturity_phase = maturity_phase_name(maturity_state.phase);
                info.maturity = maturity_state.to_json();
                info.budget = active_budget.to_json();
                report.epochs.push_back(info);
                if (options_.progress) {
                    JsonValue ev = info.to_json();
                    ev["event"] = "epoch";
                    ev["recovery_mode"] = mode_name;
                    ev["remaining"] = static_cast<double>(remaining.size());
                    options_.progress(ev);
                }
            }
            rec.generations++;
            rec.ripups += (int)moves[best].owned_idx.size();
            stall.reset();
            // Strengthen history around the repaired region so later
            // generations avoid the same corridor fight.
            // Issue #14: history growth scales with maturity.
            for (int ti : win.newly_done)
                congestion.add_history_rect(corridors[ti].rect,
                                            0.5 * active_budget.history_growth);
        }
    }
    rec.transposition_hits = transposition.hits();
    report.recovery = rec;
    ms_recovery = stage_ms(t_recovery_start, stage_now());
    const auto t_materialize_start = stage_now();

    // ---- Issue #12: post-route pair materialization ----
    // Corridors were reserved with the ordinary global search. Now each
    // routed corridor is replaced by its two coupled members, atomically:
    // both commit after exact legality, or neither does and the pair is
    // marked for corridor re-route/recovery (no one-member-only copper).
    // Single-threaded, pair-id order: deterministic across thread counts.
    // P/N never enter cleanup before this gate (there is no optimizer yet,
    // so the guarantee is structural: materializer output is committed
    // verbatim, never passed through generic bend removal).
    report.diffpairs.clear();
    if (!board_.diffpairs.empty()) {
        std::vector<DiffPair> pairs_sorted = board_.diffpairs;
        std::sort(pairs_sorted.begin(), pairs_sorted.end(),
                  [](const DiffPair& a, const DiffPair& b) { return a.id < b.id; });
        // Map pair id -> corridor task position (if any).
        std::map<int, int> pair_task_pos;
        for (std::size_t i = 0; i < tasks.size(); ++i) {
            if (tasks[i].is_pair_corridor) pair_task_pos[tasks[i].pair_id] = (int)i;
        }
        for (const auto& pr : pairs_sorted) {
            PairReport prep;
            prep.pair_id = pr.id;
            prep.name = pr.name;
            prep.net_p = pr.net_p;
            prep.net_n = pr.net_n;
            const NetInfo* np = board_.find_net(pr.net_p);
            const NetInfo* nn = board_.find_net(pr.net_n);
            prep.net_p_name = np ? np->name : "?";
            prep.net_n_name = nn ? nn->name : "?";
            prep.gap_mm = nm_to_mm(pr.gap_nm);
            prep.gap_tol_mm = nm_to_mm(pr.gap_tol_nm);
            prep.occupied_width_mm = nm_to_mm(diffpair_occupied_width(board_, resolver_, pr, ctx));
            prep.corridor_routed = false;
            prep.materialized = false;

            auto tp_it = pair_task_pos.find(pr.id);
            bool invalid = false;
            for (int bad : pair_invalid_ids) {
                if (bad == pr.id) {
                    invalid = true;
                    break;
                }
            }
            if (invalid || tp_it == pair_task_pos.end()) {
                prep.status = "CORRIDOR_FAILED";
                report.diffpairs.push_back(prep);
                continue;
            }
            int ti = tp_it->second;
            if (!task_done[ti] || task_superseded[ti]) {
                prep.status = "CORRIDOR_FAILED";
                report.diffpairs.push_back(prep);
                continue;
            }
            // Locate the owned corridor.
            int owned_idx = -1;
            for (std::size_t i = 0; i < owned.size(); ++i) {
                if (owned[i].task_pos == ti) {
                    owned_idx = (int)i;
                    break;
                }
            }
            if (owned_idx < 0) {
                prep.status = "CORRIDOR_FAILED";
                report.diffpairs.push_back(prep);
                continue;
            }
            prep.corridor_routed = true;
            PairCorridor corr;
            corr.pair_id = pr.id;
            corr.net_p = pr.net_p;
            corr.net_n = pr.net_n;
            corr.center_traces = owned[owned_idx].traces;
            corr.center_vias = owned[owned_idx].vias;
            corr.occupied_width_nm = diffpair_occupied_width(board_, resolver_, pr, ctx);

            // Remove the temporary corridor copper (exact match) so the
            // pair is validated against real foreign copper.
            auto erase_traces = [&](const std::vector<TraceSeg>& rm) {
                for (const auto& s : rm) {
                    for (auto it = board_.traces.begin(); it != board_.traces.end(); ++it) {
                        if (it->net == s.net && it->layer == s.layer &&
                            it->a == s.a && it->b == s.b && it->width_nm == s.width_nm) {
                            board_.traces.erase(it);
                            break;
                        }
                    }
                }
            };
            auto erase_vias = [&](const std::vector<Via>& rm) {
                for (const auto& s : rm) {
                    for (auto it = board_.vias.begin(); it != board_.vias.end(); ++it) {
                        if (it->net == s.net && it->pos == s.pos &&
                            it->top_layer == s.top_layer &&
                            it->bottom_layer == s.bottom_layer &&
                            it->outer_d_nm == s.outer_d_nm &&
                            it->hole_d_nm == s.hole_d_nm) {
                            board_.vias.erase(it);
                            break;
                        }
                    }
                }
            };
            erase_traces(owned[owned_idx].traces);
            erase_vias(owned[owned_idx].vias);
            resolver_.rebind(&board_);

            MaterializedPair mat =
                materialize_pair(board_, resolver_, ctx, pr, corr);
            if (!mat.ok) {
                // Atomic revert: nothing committed. Revoke the corridor
                // task and mark both members for re-route/recovery.
                task_done[ti] = false;
                if (report.stats.tasks_routed > 0) report.stats.tasks_routed--;
                owned.erase(owned.begin() + owned_idx);
                resolver_.rebind(&board_);
                RouteFailure f;
                f.net = pr.net_p;
                f.net_name = prep.net_p_name;
                f.a = tasks[ti].a;
                f.b = tasks[ti].b;
                f.reason = "materialization_failed:" + mat.reason;
                fill_pair_fields(f, tasks[ti], board_);
                f.blockers.push_back("pair_materialization:" + mat.reason);
                if (mat.has_gap_violation_at) {
                    f.blockers.push_back(
                        "gap_violation_at_mm=" +
                        std::to_string(nm_to_mm(mat.gap_violation_at.x)) + "," +
                        std::to_string(nm_to_mm(mat.gap_violation_at.y)));
                }
                f.ripup_attempts = rec.generations > 0 ? rec.generations : 1;
                f.modes_attempted = rec.modes_attempted;
                {
                    const Terminal* tta = board_.find_terminal(tasks[ti].a);
                    TraceRule rule = resolver_.traceRule(
                        pr.net_p, tta ? tta->layer : 0, kAnyRegion);
                    f.required_width_mm =
                        nm_to_mm(diffpair_occupied_width(board_, resolver_, pr, ctx));
                    f.width_source = "pair_corridor";
                    f.width_model = "pair_corridor";
                    WidthDetails wd = resolver_.widthDetails(
                        pr.net_p, tta ? tta->layer : 0, ctx);
                    f.copper_weight_oz = wd.copper_weight_oz;
                    f.temp_rise_c = wd.temp_rise_c;
                    f.blockers = attribute_blockers(board_, tasks[ti],
                                                    rule.pref_width_nm,
                                                    nullptr);
                    f.blockers.push_back("pair_materialization:" + mat.reason);
                }
                report.failures.push_back(f);
                {
                    // Companion entry for the N member (per-net queries).
                    RouteFailure g = f;
                    g.net = pr.net_n;
                    g.net_name = prep.net_n_name;
                    g.a = tasks[ti].pair_a_other;
                    g.b = tasks[ti].pair_b_other;
                    report.failures.push_back(g);
                }
                int ni = net_index(pr.net_p);
                if (ni >= 0) net_ok[ni] = 0;
                ni = net_index(pr.net_n);
                if (ni >= 0) net_ok[ni] = 0;
                prep.status = "MATERIALIZATION_FAILED:" + mat.reason;
                report.diffpairs.push_back(prep);
                continue;
            }
            // Atomic commit of both members.
            OwnedRoute po;
            po.task = tasks[ti];
            po.task_pos = ti;
            po.traces.insert(po.traces.end(), mat.traces_p.begin(), mat.traces_p.end());
            po.traces.insert(po.traces.end(), mat.traces_n.begin(), mat.traces_n.end());
            po.vias.insert(po.vias.end(), mat.vias_p.begin(), mat.vias_p.end());
            po.vias.insert(po.vias.end(), mat.vias_n.begin(), mat.vias_n.end());
            po.epoch_committed = owned[owned_idx].epoch_committed;
            po.stable_epochs = owned[owned_idx].stable_epochs;
            po.is_escape_stub = false;
            po.is_pair_corridor = true;
            po.pair_id = pr.id;
            po.protection = owned[owned_idx].protection;
            for (const auto& s : mat.traces_p) board_.traces.push_back(s);
            for (const auto& s : mat.traces_n) board_.traces.push_back(s);
            for (const auto& v : mat.vias_p) board_.vias.push_back(v);
            for (const auto& v : mat.vias_n) board_.vias.push_back(v);
            owned[owned_idx] = po;
            resolver_.rebind(&board_);
            prep.materialized = true;
            prep.status = "MATERIALIZED";
            prep.length_p_mm = nm_to_mm(mat.length_p_nm);
            prep.length_n_mm = nm_to_mm(mat.length_n_nm);
            prep.skew_mm = nm_to_mm(mat.skew_nm);
            prep.worst_gap_err_mm = nm_to_mm(mat.worst_gap_err_nm);
            prep.has_gap_location = mat.has_gap_violation_at;
            prep.gap_x_mm = nm_to_mm(mat.gap_violation_at.x);
            prep.gap_y_mm = nm_to_mm(mat.gap_violation_at.y);
            prep.gap_layer = mat.traces_p.empty() ? 0 : mat.traces_p.front().layer;
            prep.via_pairs = (int)corr.center_vias.size();
            prep.vias_p = static_cast<int>(mat.vias_p.size());
            prep.vias_n = static_cast<int>(mat.vias_n.size());
            prep.via_mismatch.clear();
            report.diffpairs.push_back(prep);
        }
        // Recompute length/via stats from final ownership (corridor copper
        // replaced by materialized members; failures revoked theirs).
        report.stats.length_nm = 0;
        report.stats.via_count = 0;
        for (const auto& o : owned) {
            for (const auto& t : o.traces) report.stats.length_nm += euclid_len_nm(t.a, t.b);
            report.stats.via_count += (int)o.vias.size();
        }
        {
            int routed = 0;
            for (std::size_t i = 0; i < task_done.size(); ++i)
                if (task_done[i] && !task_superseded[i]) ++routed;
            int active = 0;
            for (char s : task_superseded)
                if (!s) ++active;
            report.stats.tasks_total = active;
            report.stats.tasks_routed = std::min(routed, active);
        }
    }

    // ---- Issue #15: post-route length/skew tuning ----
    // Dedicated LengthTuner stage, invoked ONLY after global routing
    // closure and #12 pair materialization above. Single-threaded
    // post-phase in (pair-id, net-id) order: deterministic across thread
    // counts. Each accepted transaction re-verifies the full BoardVerifier
    // gate; infeasible targets yield explicit diagnostics and leave
    // unrelated copper untouched. Committed tuning geometry is marked
    // tuning_exempt so #17 simplification/cleanup must not collapse it.
    {
        bool closed = true;
        for (std::size_t i = 0; i < tasks.size(); ++i) {
            if (!task_superseded[i] && !task_done[i]) {
                closed = false;
                break;
            }
        }
        const auto t_tuning_start = stage_now();
        LengthTuner tuner(&board_, &resolver_, &ctx, options_.tuning);
        report.tuning = tuner.run(closed);
        report.tuning.effective_threads = options_.threads > 0 ? options_.threads : 1;
        if (!report.tuning.tuned_nets.empty()) {
            for (auto& task : tasks) {
                bool touched = false;
                for (NetId n : report.tuning.tuned_nets) {
                    if (task.net == n || task.pair_other_net == n) {
                        touched = true;
                        break;
                    }
                }
                if (touched) task.tuning_exempt = true;
            }
            resolver_.rebind(&board_);
            // Recompute copper stats from committed geometry (tuning adds
            // trace length without new tasks/vias).
            report.stats.length_nm = 0;
            for (const auto& t : board_.traces)
                report.stats.length_nm += euclid_len_nm(t.a, t.b);
            report.stats.via_count = static_cast<int>(board_.vias.size());
        }
        ms_materialize = stage_ms(t_materialize_start, t_tuning_start);
        ms_tuning = stage_ms(t_tuning_start, stage_now());
    }

    report.stats.epochs_count = static_cast<int>(report.epochs.size());
    report.stats.threads_used = threads_used_max;

    // Issue #16: plane-access report from committed ownership. Each routed
    // plane task contributes its chosen plane/layer/island, entry geometry,
    // via bundle size and current-capacity margin. Sorted by (net,
    // terminal) so identical inputs hash identically.
    report.plane_access.clear();
    for (const auto& o : owned) {
        if (!o.task.has_plane_target) continue;
        PlaneAccessInfo p;
        p.net = o.task.net;
        const NetInfo* ninfo = board_.find_net(o.task.net);
        p.net_name = ninfo ? ninfo->name : "?";
        p.terminal = o.task.a;
        p.plane_id = o.task.plane_id;
        p.plane_layer = o.task.plane_layer;
        p.island = o.task.plane_island;
        p.entry = o.task.plane_point;
        p.via_count = static_cast<int>(o.vias.size());
        bool dummy = false;
        p.required_current_a =
            ninfo ? resolver_.current().effective_current(*ninfo, board_.defaults, dummy)
                  : 0.0;
        if (!o.vias.empty()) {
            p.via_style = o.vias.front().via_class;
            ViaStyle st;
            if (!p.via_style.empty() && resolver_.lookup_via_style(p.via_style, st)) {
                p.via_capacity_a =
                    st.max_current_a * std::max(1, static_cast<int>(o.vias.size()));
            }
        } else {
            // Direct same-layer copper entry: no via bottleneck.
            p.via_style = "direct_copper";
            p.via_capacity_a = p.required_current_a;
        }
        p.current_margin_a = p.via_capacity_a - p.required_current_a;
        report.plane_access.push_back(p);
    }
    std::sort(report.plane_access.begin(), report.plane_access.end(),
              [](const PlaneAccessInfo& a, const PlaneAccessInfo& b) {
                  if (a.net != b.net) return a.net < b.net;
                  return a.terminal < b.terminal;
              });

    // Issue #11: per-net controlled-impedance resolutions for the report.
    // Only nets with targets appear; deterministic net-id order.
    report.impedance.clear();
    for (const auto& net : board_.nets) {
        if (!resolver_.impedance().has_target(net)) continue;
        report.impedance.push_back(resolver_.impedanceResolution(net.id, ctx));
    }

    // Failures for everything left unrouted.
    const auto t_attribution_start = stage_now();
    // B5: memoize the endpoint pin-density scan per task midpoint. The
    // count depends only on (midpoint, terminals); terminals are fixed for
    // the whole attribution loop, so repeated midpoints share one
    // O(terminals) scan. Values are identical to the inline scan.
    std::map<std::pair<Coord, Coord>, int> pin_density_cache;
    for (std::size_t i = 0; i < tasks.size(); ++i) {
        if (task_done[i]) continue;
        if (task_superseded[i]) continue;
        const ConnectionTask& task = tasks[i];
        bool already = false;
        for (const auto& f : report.failures) {
            if (f.net == task.net && ((f.a == task.a && f.b == task.b) ||
                                      (f.a == task.b && f.b == task.a))) {
                already = true;
                break;
            }
        }
        if (already) continue;
        RouteFailure f;
        f.net = task.net;
        const NetInfo* n = board_.find_net(task.net);
        f.net_name = n ? n->name : "?";
        f.a = task.a;
        f.b = task.b;
        fill_plane_fields(f, task);
        fill_pair_fields(f, task, board_);
        auto it = last_attempt.find(attempt_key(task));
        const CandidateRoute* last = it != last_attempt.end() ? &it->second : nullptr;
        if (!last) {
            f.reason = "unattempted";
            if (n) {
                bool dummy = false;
                f.required_current_a =
                    resolver_.current().effective_current(*n, board_.defaults, dummy);
                auto ordered = ViaBundlePlanner::ordered_styles(
                    resolver_, task.net,
                    LayerSpan{board_.layers.front().id, board_.layers.back().id});
                if (!ordered.empty()) {
                    f.via_style = ordered.front().name;
                    f.vias_required =
                        ViaBundlePlanner::required_count(resolver_, ordered.front(),
                                                         task.net);
                }
            }
        } else if (last && !last->found) {
            f.reason = last->fail_reason.empty() ? "unreachable" : last->fail_reason;
            if (f.reason == "budget_exhausted") budget_hit = true;
            f.expansions = last->expansions;
            fill_hierarchy_fields(f, last);
            f.required_current_a = last->required_current_a;
            f.via_style = last->via_style;
            f.vias_required = last->vias_required;
            f.via_reason = last->via_reason;
            f.frontier = diagnose_task(task, *last);
            f.has_frontier = true;
        } else if (last && last->found) {
            f.reason = "conflict";
            fill_hierarchy_fields(f, last);
            f.required_current_a = last->required_current_a;
            f.via_style = last->via_style;
            f.vias_required = last->vias_required;
            f.via_reason = last->via_reason;
            f.frontier = diagnose_task(task, *last);
            f.has_frontier = true;
        } else {
            f.reason = "unreachable";
        }
        // If recovery ran but this task stayed unrouted, record attempts.
        auto rk = std::make_pair(task.net, std::make_pair(std::min(task.a, task.b),
                                                          std::max(task.a, task.b)));
        auto rit = ripup_attempts.find(rk);
        f.ripup_attempts = rit != ripup_attempts.end() ? rit->second : (rec.generations > 0 ? 1 : 0);
        f.modes_attempted = rec.modes_attempted;
        // Prompt 5 agent-stable attribution: endpoints, density, attempts.
        {
            const Terminal* ta = board_.find_terminal(task.a);
            const Terminal* tb = board_.find_terminal(task.b);
            if (ta) {
                f.src_component = ta->component;
                f.src_pin = ta->pin;
            }
            if (tb) {
                f.dst_component = tb->component;
                f.dst_pin = tb->pin;
            }
            int depth = -1;
            auto da = escape_centre_depth.find(task.a);
            if (da != escape_centre_depth.end()) depth = std::max(depth, da->second);
            auto db = escape_centre_depth.find(task.b);
            if (db != escape_centre_depth.end()) depth = std::max(depth, db->second);
            f.centre_depth = depth;
            // Endpoint pin density: terminals within 5 mm of the task
            // midpoint, per mm^2. Deterministic, integer-geometry based.
            if (ta && tb) {
                Coord mx = (ta->pos.x + tb->pos.x) / 2;
                Coord my = (ta->pos.y + tb->pos.y) / 2;
                auto mkey = std::make_pair(mx, my);
                auto mhit = pin_density_cache.find(mkey);
                int n;
                if (mhit != pin_density_cache.end()) {
                    n = mhit->second;
                } else {
                    Coord r = mm_to_nm(5.0);
                    __int128 r2 = (__int128)r * r;
                    n = 0;
                    for (const auto& t : board_.terminals) {
                        __int128 dx = (__int128)t.pos.x - mx;
                        __int128 dy = (__int128)t.pos.y - my;
                        if (dx * dx + dy * dy <= r2) n++;
                    }
                    pin_density_cache[mkey] = n;
                }
                f.pin_density = n / (3.14159265358979 * 25.0);
            }
            auto cit = attempt_counts.find(attempt_key(task));
            int greedy_tries = cit != attempt_counts.end() ? cit->second : 0;
            f.candidate_count = greedy_tries + f.ripup_attempts;
            for (const auto& l : board_.layers) f.attempted_layers.push_back(l.id);
            for (const auto& vs :
                 resolver_.allowedVias(task.net, LayerSpan{board_.layers.front().id,
                                                          board_.layers.back().id}))
                f.attempted_via_classes.push_back(vs.name);
            if (f.has_frontier) {
                char buf[128];
                std::snprintf(buf, sizeof(buf), "frontier_gap_mm=%.4f expansions=%lld",
                              nm_to_mm(f.frontier.closest_goal_dist_nm),
                              (long long)f.expansions);
                f.best_partial = buf;
            } else if (!last) {
                f.best_partial = "unattempted: no worker candidate produced";
            } else {
                f.best_partial = "candidate_rejected_by_arbiter";
            }
        }
        const Terminal* ta = board_.find_terminal(task.a);
        TraceRule rule = resolver_.traceRule(
            task.net, ta ? ta->layer : 0, kAnyRegion);
        f.required_width_mm = nm_to_mm(rule.pref_width_nm);
        f.width_source = rule.width_source;
        f.width_model = rule.width_source;
        if (task.is_pair_corridor) {
            // Issue #12: report the atomic envelope, not one member.
            for (const auto& pr : board_.diffpairs) {
                if (pr.id == task.pair_id) {
                    f.required_width_mm =
                        nm_to_mm(diffpair_occupied_width(board_, resolver_, pr, ctx));
                    f.width_source = "pair_corridor";
                    f.width_model = "pair_corridor";
                    break;
                }
            }
        }
        {
            WidthDetails wd = resolver_.widthDetails(task.net, ta ? ta->layer : 0, ctx);
            f.width_model = wd.model;
            f.copper_weight_oz = wd.copper_weight_oz;
            f.temp_rise_c = wd.temp_rise_c;
        }
        // Issue #11: controlled-impedance accounting on every failure so
        // agents see target/selection/error/conflict without extra calls.
        if (const NetInfo* fn = board_.find_net(task.net);
            fn && resolver_.impedance().has_target(*fn)) {
            ImpedanceResolution zir = resolver_.impedanceResolution(task.net, ctx);
            f.has_impedance = true;
            f.target_impedance_ohms = zir.target_ohms;
            f.impedance_tolerance_pct = zir.tolerance_frac * 100.0;
            f.impedance_layer = zir.selected_layer;
            f.impedance_width_mm = nm_to_mm(zir.selected_width_nm);
            f.impedance_model = zir.selected_model;
            f.estimated_impedance_ohms = zir.estimated_ohms;
            f.impedance_error_pct = zir.rel_error * 100.0;
            f.impedance_conflict = zir.conflict;
            f.impedance_detail = zir.conflict_detail;
            if (zir.conflict && f.reason != "impedance_current_conflict" &&
                f.reason != "impedance_infeasible") {
                f.blockers.push_back("impedance_current_conflict:" +
                                     zir.conflict_detail);
            }
        }
        f.blockers = attribute_blockers(board_, task, rule.pref_width_nm, last);
        // Issue #1: carry unresolved escape records into the route report so
        // agents see why a dense pad never escaped. Prevents silent bypass:
        // a shallower pad's success never erases a deeper pad's record
        // (escape_stage.unresolved_* is authoritative and append-only).
        auto ea = escape_infeasible_reason.find(task.a);
        if (ea != escape_infeasible_reason.end())
            f.blockers.push_back("escape_infeasible:" + ea->second);
        auto eb = escape_infeasible_reason.find(task.b);
        if (eb != escape_infeasible_reason.end() && eb->first != task.a)
            f.blockers.push_back("escape_infeasible:" + eb->second);
        report.failures.push_back(f);
        int ni = net_index(task.net);
        if (ni >= 0) net_ok[ni] = 0;
        // Issue #12: a corridor failure strands both members.
        if (task.is_pair_corridor) {
            ni = net_index(task.pair_other_net);
            if (ni >= 0) net_ok[ni] = 0;
        }
    }

    for (char ok : net_ok)
        if (ok) report.stats.nets_routed++;

    report.connected_terminals = 0;
    for (const auto& net : board_.nets) {
        if (net.terminals.size() < 2) {
            report.connected_terminals += static_cast<int>(net.terminals.size());
            continue;
        }
        bool all_ok = true;
        for (std::size_t i = 0; i < tasks.size(); ++i) {
            if (task_superseded[i]) continue;
            // Issue #12: pair members share their corridor task.
            bool covers = tasks[i].net == net.id ||
                          (tasks[i].is_pair_corridor &&
                           tasks[i].pair_other_net == net.id);
            if (!covers || task_done[i]) continue;
            all_ok = false;
            break;
        }
        if (all_ok) report.connected_terminals += static_cast<int>(net.terminals.size());
    }

    auto t1 = std::chrono::steady_clock::now();
    report.stats.time_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    report.hotspots = congestion.hotspots();
    // S7: single final hash — deferred until after the optimizer stage
    // below. Neither status classification nor the verifier gate consumes
    // board_hash/state_hash, so hashing once at the end is identical and
    // saves a full copper+task rescan on COMPLETE boards. Per-epoch report
    // JSON stays lazy: EpochInfo JSON is built once per epoch for the
    // final log, and progress NDJSON only when a callback is set.

    if (report.failures.empty()) {
        report.status = "COMPLETE";
    } else if (timed_out) {
        report.status = "TIMEOUT";
    } else if (budget_hit) {
        report.status = "BUDGET_EXHAUSTED";
    } else {
        report.status = "INCOMPLETE";
    }
    report.result_category = result_category(report.status);
    ms_attribution = stage_ms(t_attribution_start, stage_now());
    // Issue #2: independently verify committed copper before returning
    // COMPLETE. Bookkeeping alone must never declare success.
    // Issue #13: the route JSON pair entries carry verifier-measured values
    // (lengths/skew/gap/location/vias from committed copper, not router
    // metadata) so post-cleanup constraint drift is visible. A verifier
    // pair failure downgrades a MATERIALIZED entry and the gate above
    // already refused COMPLETE.
    {
        BoardVerifier verifier;
        const auto t_verify_start = stage_now();
        VerifyResult vr = verifier.verify(board_, resolver_, ctx);
        ms_verify = stage_ms(t_verify_start, stage_now());
        for (auto& prep : report.diffpairs) {
            for (const auto& vd : vr.pairs) {
                if (vd.pair_id != prep.pair_id) continue;
                prep.length_p_mm = vd.length_p_mm;
                prep.length_n_mm = vd.length_n_mm;
                prep.skew_mm = vd.skew_mm;
                prep.worst_gap_err_mm = vd.worst_gap_err_mm;
                prep.has_gap_location = vd.has_gap_location;
                prep.gap_x_mm = vd.gap_x_mm;
                prep.gap_y_mm = vd.gap_y_mm;
                prep.gap_layer = vd.gap_layer;
                prep.vias_p = vd.vias_p;
                prep.vias_n = vd.vias_n;
                prep.via_mismatch = vd.via_mismatch;
                if (!vd.ok && prep.status == "MATERIALIZED")
                    prep.status = "VERIFIER_FAILED:" + vd.status;
                else if (!vd.ok && prep.status.rfind("MATERIALIZATION_FAILED",
                                                     0) != 0 &&
                           prep.status.rfind("CORRIDOR_FAILED", 0) != 0 &&
                           prep.status.rfind("VERIFIER_FAILED", 0) != 0)
                    prep.status = "VERIFIER_FAILED:" + vd.status;
                break;
            }
        }
        apply_verifier_gate(report, vr);
    }
    // Prompt 5: transactional cleanup optimizer. Runs ONLY after complete
    // legal connectivity (engine + independent verifier agree). Every
    // transform re-verifies and reverts on harm; stats/hashes refresh after.
    const auto t_optimizer_start = stage_now();
    if (report.status == "COMPLETE" && report.verification.ok) {
        OptimizerOptions oo = options_.optimizer;
        // Bound optimizer scratch by the router memory budget.
        oo.max_candidates = std::min<std::size_t>(
            oo.max_candidates, options_.memory_budget_bytes / (4 * options_.per_task_bytes));
        if (oo.max_candidates < 8) oo.max_candidates = 8;
        CleanupOptimizer opt(&board_, &resolver_, ctx, oo);
        report.optimizer = opt.run();
        resolver_.rebind(&board_);
        if (report.optimizer.ran) {
            report.stats.length_nm = 0;
            for (const auto& s : board_.traces)
                report.stats.length_nm += euclid_len_nm(s.a, s.b);
            report.stats.via_count = static_cast<int>(board_.vias.size());
            BoardVerifier recheck;
            VerifyResult vr2 = recheck.verify(board_, resolver_, ctx);
            apply_verifier_gate(report, vr2);
            for (auto& f : report.failures) f.category = report.result_category;
        }
    } else {
        report.optimizer.enabled = options_.optimizer.enabled;
        report.optimizer.gate_reason =
            std::string("gated: status=") + report.status +
            (report.verification.ok ? "" : " (verifier not ok)");
    }
    // S7: the single final hash (covers both optimizer-ran and gated-off
    // paths; verifier gating semantics above are unchanged).
    ms_optimizer = stage_ms(t_optimizer_start, stage_now());
    const auto t_hash_start = stage_now();
    report.board_hash = geometry_hash(board_);
    report.state_hash = state_hash128(board_, tasks, remaining);
    ms_hash = stage_ms(t_hash_start, stage_now());
    if (options_.progress) {
        JsonValue done = JsonValue::object();
        done["event"] = "done";
        done["status"] = report.status;
        done["result_category"] = report.result_category;
        done["board_hash"] = report.board_hash;
        done["state_hash"] = report.state_hash.to_hex();
        done["connected_terminals"] = static_cast<double>(report.connected_terminals);
        done["remaining_terminals"] =
            static_cast<double>(report.total_terminals - report.connected_terminals);
        options_.progress(done);
    }
    (void)options_.seed;
    if (options_.time_stages) {
        std::fprintf(stderr,
                     "{\"event\":\"stage_timings\",\"escape_ms\":%.3f,"
                     "\"taskgen_ms\":%.3f,\"batch_ms\":%.3f,\"workers_ms\":%.3f,"
                     "\"arbiter_ms\":%.3f,\"recovery_ms\":%.3f,"
                     "\"materialize_ms\":%.3f,\"tuning_ms\":%.3f,"
                     "\"attribution_ms\":%.3f,\"verify_ms\":%.3f,"
                     "\"optimizer_ms\":%.3f,\"hash_ms\":%.3f,"
                     "\"threads\":%d,\"epochs\":%d,\"tasks_total\":%d}\n",
                     ms_escape, ms_taskgen, ms_batch, ms_workers, ms_arbiter,
                     ms_recovery, ms_materialize, ms_tuning, ms_attribution,
                     ms_verify, ms_optimizer, ms_hash, effective_threads,
                     static_cast<int>(report.epochs.size()),
                     report.stats.tasks_total);
    }
    return report;
}

}  // namespace copperline
