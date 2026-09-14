#include "router/engine.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <thread>

#include "router/density.h"
#include "router/escape.h"
#include "router/route_tree.h"
#include "router/sparse_graph.h"

namespace copperline {

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
        o["ripup_attempts"] = static_cast<double>(f.ripup_attempts);
        JsonValue ma = JsonValue::array();
        for (const auto& m : f.modes_attempted) ma.as_array().push_back(JsonValue(m));
        o["modes_attempted"] = ma;
        if (f.has_frontier) o["frontier"] = f.frontier.to_json();
        JsonValue bl = JsonValue::array();
        for (const auto& b : f.blockers) bl.as_array().push_back(JsonValue(b));
        o["blockers"] = bl;
        fails.as_array().push_back(o);
    }
    r["failures"] = fails;
    r["recovery"] = recovery.to_json();
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

}  // namespace

RouteReport RouterEngine::run() {
    auto t0 = std::chrono::steady_clock::now();
    RouteReport report;
    report.total_terminals = static_cast<int>(board_.terminals.size());

    // Fixed user copper: everything committed before routing starts. It is
    // the only copper that is absolutely protected (never ripped).
    const std::vector<TraceSeg> fixed_traces = board_.traces;
    const std::vector<Via> fixed_vias = board_.vias;

    DensityEstimator density_est;
    DensityResult density = density_est.analyze(board_);
    ElectricalContext ctx;

    // ---- Connection tasks from RouteTrees ----
    std::vector<ConnectionTask> tasks;
    for (const auto& net : board_.nets) {
        if (net.terminals.size() < 2) continue;
        RouteTree tree = build_route_tree(board_, net.id);
        for (auto& t : tree.tasks) tasks.push_back(t);
    }
    // Centre-out boost (Prompt 2): depth outranks density but never legality.
    std::map<TermId, int> depth_of;
    {
        FinePitchDetector detector;
        CentreDepthAnalyzer cda;
        for (const auto& fp : detector.detect(board_, resolver_, ctx)) {
            for (const auto& [tid, d] : cda.analyze(board_, fp)) depth_of[tid] = d;
        }
    }
    auto is_escape_task = [&](const ConnectionTask& t) {
        auto it = depth_of.find(t.a);
        int da = it != depth_of.end() ? it->second : 0;
        it = depth_of.find(t.b);
        int db = it != depth_of.end() ? it->second : 0;
        return std::max(da, db) > 0;
    };
    std::vector<int> fail_count(tasks.size(), 0);
    std::vector<Corridor> corridors(tasks.size());
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
        corridors[i] = probable_corridor(board_, resolver_, tasks[i], ctx);

    report.stats.tasks_total = static_cast<int>(tasks.size());
    report.stats.nets_total = static_cast<int>(board_.nets.size());
    report.stats.threads_requested = options_.threads;

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

    auto deadline = options_.timeout_s > 0
                        ? t0 + std::chrono::duration<double>(options_.timeout_s)
                        : std::chrono::steady_clock::time_point::max();
    bool timed_out = false;
    bool budget_hit = false;

    std::vector<char> task_done(tasks.size(), 0);
    std::vector<int> remaining = all_idx;
    // Last epoch in which each task was given a worker attempt (-1 = never).
    std::vector<int> last_epoch(tasks.size(), -1);
    std::map<std::pair<NetId, std::pair<TermId, TermId>>, CandidateRoute> last_attempt;
    auto attempt_key = [](const ConnectionTask& t) {
        return std::make_pair(t.net, std::make_pair(std::min(t.a, t.b), std::max(t.a, t.b)));
    };

    std::vector<char> net_ok(board_.nets.size(), 1);
    auto net_index = [&](NetId id) -> int {
        for (std::size_t i = 0; i < board_.nets.size(); ++i)
            if (board_.nets[i].id == id) return static_cast<int>(i);
        return -1;
    };

    // Ownership: which greedy task owns which committed copper. Needed so
    // rip-up can remove selective routes and rebuild deterministically.
    std::vector<OwnedRoute> owned;

    int epoch = 0;
    int stalled = 0;
    int threads_used_max = 1;
    const int workers_cap = std::max(1, options_.threads);

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
                report.failures.push_back(f);
                int ni = net_index(tasks[i].net);
                if (ni >= 0) net_ok[ni] = 0;
            }
            remaining.clear();
            break;
        }
        refresh_difficulties(remaining);
        std::vector<ConnectionTask> ordered_tasks;
        std::vector<int> ordered_idx;
        {
            std::vector<int> order = remaining;
            std::sort(order.begin(), order.end(), [&](int a, int b) {
                bool sa = last_epoch[a] < epoch - 1;
                bool sb = last_epoch[b] < epoch - 1;
                if (sa != sb) return sa > sb;
                if (tasks[a].difficulty != tasks[b].difficulty)
                    return tasks[a].difficulty > tasks[b].difficulty;
                if (tasks[a].net != tasks[b].net) return tasks[a].net < tasks[b].net;
                if (tasks[a].a != tasks[b].a) return tasks[a].a < tasks[b].a;
                return tasks[a].b < tasks[b].b;
            });
            for (int i : order) {
                ordered_idx.push_back(i);
                ordered_tasks.push_back(tasks[i]);
            }
        }
        std::size_t batch_n = std::min<std::size_t>(ordered_idx.size(), kParallelBatchSize);
        std::vector<int> batch_idx(ordered_idx.begin(), ordered_idx.begin() + batch_n);
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

        auto epoch_t0 = std::chrono::steady_clock::now();
        std::vector<CandidateRoute> candidates(batch_n);
        const Board& snapshot = board_;
        const std::size_t traces_before = snapshot.traces.size();
        const std::size_t vias_before = snapshot.vias.size();
        int workers = std::max(1, std::min<int>(workers_cap, static_cast<int>(batch_n)));
        threads_used_max = std::max(threads_used_max, workers);
        auto worker_fn = [&](int w) {
            for (std::size_t k = w; k < batch_n; k += workers) {
                int ti = batch_idx[k];
                candidates[k] =
                    route_candidate_task(snapshot, resolver_, tasks[ti], rem_pos[ti],
                                         tasks[ti].difficulty, ctx, layer_mult, options_.astar,
                                         congestion, reservations);
            }
        };
        if (workers == 1) {
            worker_fn(0);
        } else {
            std::vector<std::thread> pool;
            for (int w = 0; w < workers; ++w) pool.emplace_back(worker_fn, w);
            for (auto& th : pool) th.join();
        }
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
            o.is_escape_stub = is_escape_task(tasks[ti]);
            o.protection =
                route_protection_score(o.is_escape_stub, tasks[ti].difficulty, 0,
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
        }
        for (std::size_t k = 0; k < arb.rejected.size(); ++k) {
            const CandidateRoute& c = candidates[arb.rejected[k]];
            if (c.found) {
                for (const auto& t : c.traces) congestion.add_history_segment(t.segment(), 1.0);
                for (const auto& vv : c.vias)
                    congestion.add_history_rect(
                        Rect::from_center_size(vv.pos, vv.outer_d_nm, vv.outer_d_nm), 1.0);
            } else {
                int ti = batch_idx[arb.rejected[k]];
                congestion.add_history_rect(corridors[ti].rect, 0.5);
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
                if (in_batch[ti]) fail_count[ti]++;
                next_remaining.push_back(ti);
            }
        }
        remaining = std::move(next_remaining);

        auto epoch_t1 = std::chrono::steady_clock::now();
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
        report.epochs.push_back(info);
        if (options_.progress) {
            JsonValue ev = info.to_json();
            ev["event"] = "epoch";
            ev["remaining"] = static_cast<double>(remaining.size());
            options_.progress(ev);
        }

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
            AStarConfig mode_cfg = astar_config_for_mode(options_.astar, mode);

            // Refresh difficulties so previous failures + congestion count.
            refresh_difficulties(remaining);
            DependencyGraph graph = build_dependency_graph(board_, resolver_, ctx, tasks,
                                                           remaining, last_attempt);
            rec.last_graph = graph;
            if (graph.edges.empty()) break;  // keepout-only: nothing to rip

            std::vector<ConnectionTask> failed_tasks = graph.failed;
            int width = std::min(branch_width_for_generation(gen), options_.max_ripup_branches);
            int breadth = max_rip_breadth_for_generation(gen);
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
                    std::vector<int> to_route;
                    to_route.push_back(remaining[m.failed_pos]);
                    for (int oi : m.owned_idx) to_route.push_back(owned[oi].task_pos);
                    std::sort(to_route.begin(), to_route.end());
                    to_route.erase(std::unique(to_route.begin(), to_route.end()),
                                   to_route.end());
                    int failed_ti = remaining[m.failed_pos];
                    BranchResult r = reroute_branch(
                        board_, fixed_traces, fixed_vias, surviving, to_route, failed_ti,
                        tasks, corridors, resolver_, ctx, layer_mult, mode_cfg, congestion,
                        mode_name, m);
                    // Transposition prune inside the slot (counted, deterministic:
                    // pruned branches simply never win).
                    int left_after = (int)remaining.size() - r.connected_tasks +
                                     (int)m.owned_idx.size();
                    // Remaining after = old remaining - newly connected (failed
                    // tasks reconnect) ... approximate by tasks still undone:
                    // total undone = remaining + ripped - done.
                    int undone = (int)remaining.size() + (int)m.owned_idx.size() -
                                 r.connected_tasks;
                    if (transposition.should_prune(r.hash, undone)) {
                        r.pruned = true;
                    } else {
                        transposition.record(r.hash, undone);
                    }
                    results[b] = r;
                    (void)left_after;
                }
            };
            if (workers == 1) {
                branch_fn(0);
            } else {
                std::vector<std::thread> pool;
                for (int w = 0; w < workers; ++w) pool.emplace_back(branch_fn, w);
                for (auto& th : pool) th.join();
            }
            rec.branches_evaluated += branch_n;
            int pruned = 0;
            for (const auto& r : results)
                if (r.pruned) ++pruned;
            rec.branches_pruned += pruned;

            // Deterministic selection in move order: first strictly-best wins.
            int best = -1;
            for (int b = 0; b < branch_n; ++b) {
                if (!results[b].evaluated || results[b].pruned) continue;
                if (best < 0 || branch_better(results[b], results[best])) best = b;
            }
            if (best < 0) {
                rec.transposition_hits = transposition.hits();
                break;  // everything pruned: give up deterministically
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
                for (int ti : remaining) congestion.add_history_rect(corridors[ti].rect, 1.0);
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
            board_ = win.board;
            resolver_.rebind(&board_);
            owned = win.owned;
            for (auto& o : owned) {
                // Recompute protection under the winning mode; stable routes
                // accumulate protection over generations.
                o.stable_epochs++;
                o.protection = route_protection_score(o.is_escape_stub, tasks[o.task_pos].difficulty,
                                                      o.stable_epochs, false, mode);
            }
            // Update done/remaining deterministically.
            for (int ti : win.newly_done) task_done[ti] = 1;
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
            // History reward + PV reuse for the next generation.
            {
                const RipupMove& m = moves[best];
                history.reward(HistoryHeuristic::move_key(m.failed_task, m.blocker_net), 1.0);
                pv_key = HistoryHeuristic::move_key(m.failed_task, m.blocker_net);
            }
            report.stats.tasks_routed += (int)win.newly_done.size();
            report.stats.expansions_total += win.expansions;
            // Count all branches' search work (not just the winner).
            for (int b = 0; b < branch_n; ++b) {
                if (b == best) continue;
                report.stats.expansions_total += results[b].expansions;
            }
            // Recompute length/via stats from committed copper.
            {
                report.stats.length_nm = 0;
                report.stats.via_count = 0;
                for (const auto& o : owned) {
                    for (const auto& t : o.traces) report.stats.length_nm += manhattan(t.a, t.b);
                    report.stats.via_count += (int)o.vias.size();
                }
            }
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
            for (int ti : win.newly_done) congestion.add_history_rect(corridors[ti].rect, 0.5);
        }
    }
    rec.transposition_hits = transposition.hits();
    report.recovery = rec;

    report.stats.epochs_count = static_cast<int>(report.epochs.size());
    report.stats.threads_used = threads_used_max;

    // Failures for everything left unrouted.
    for (std::size_t i = 0; i < tasks.size(); ++i) {
        if (task_done[i]) continue;
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
        auto it = last_attempt.find(attempt_key(task));
        const CandidateRoute* last = it != last_attempt.end() ? &it->second : nullptr;
        if (!last) {
            f.reason = "unattempted";
        } else if (last && !last->found) {
            f.reason = last->fail_reason.empty() ? "unreachable" : last->fail_reason;
            if (f.reason == "budget_exhausted") budget_hit = true;
            f.expansions = last->expansions;
            f.frontier = diagnose_task(task, *last);
            f.has_frontier = true;
        } else if (last && last->found) {
            f.reason = "conflict";
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
        const Terminal* ta = board_.find_terminal(task.a);
        TraceRule rule = resolver_.traceRule(
            task.net, ta ? ta->layer : 0, kAnyRegion);
        f.required_width_mm = nm_to_mm(rule.pref_width_nm);
        f.width_source = rule.width_source;
        f.blockers = attribute_blockers(board_, task, rule.pref_width_nm, last);
        report.failures.push_back(f);
        int ni = net_index(task.net);
        if (ni >= 0) net_ok[ni] = 0;
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
            if (tasks[i].net != net.id || task_done[i]) continue;
            all_ok = false;
            break;
        }
        if (all_ok) report.connected_terminals += static_cast<int>(net.terminals.size());
    }

    auto t1 = std::chrono::steady_clock::now();
    report.stats.time_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    report.hotspots = congestion.hotspots();
    report.board_hash = geometry_hash(board_);
    report.state_hash = state_hash128(board_, tasks, remaining);

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
    return report;
}

}  // namespace copperline
