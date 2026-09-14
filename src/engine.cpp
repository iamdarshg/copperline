#include "router/engine.h"

#include <algorithm>
#include <cmath>
#include <map>
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
    r["connected_terminals"] = static_cast<double>(connected_terminals);
    r["total_terminals"] = static_cast<double>(total_terminals);
    r["remaining_terminals"] = static_cast<double>(total_terminals - connected_terminals);
    r["board_hash"] = board_hash;
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
        JsonValue bl = JsonValue::array();
        for (const auto& b : f.blockers) bl.as_array().push_back(JsonValue(b));
        o["blockers"] = bl;
        fails.as_array().push_back(o);
    }
    r["failures"] = fails;
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
    const Terminal* ta = board.find_terminal(task.a);
    const Terminal* tb = board.find_terminal(task.b);
    std::vector<std::string> out;
    if (!ta || !tb) {
        out.push_back("terminal id not found on board");
        return out;
    }
    Rect corridor = Rect::from_points(ta->pos, tb->pos).expanded(width);
    struct Hit {
        std::string desc;
        Coord area;
    };
    std::vector<Hit> hits;
    for (const auto& ko : board.keepouts) {
        if (!ko.rect.intersects(corridor)) continue;
        Rect inter{std::max(ko.rect.x1, corridor.x1), std::max(ko.rect.y1, corridor.y1),
                   std::min(ko.rect.x2, corridor.x2), std::min(ko.rect.y2, corridor.y2)};
        __int128 area = (__int128)(inter.x2 - inter.x1) * (inter.y2 - inter.y1);
        hits.push_back({"keepout:" + ko.reason, static_cast<Coord>(area)});
    }
    for (const auto& t : board.traces) {
        if (t.net == task.net) continue;
        if (!t.segment().bounds().expanded(t.width_nm).intersects(corridor)) continue;
        const NetInfo* on = board.find_net(t.net);
        hits.push_back({"trace:net=" + std::string(on ? on->name : "?"),
                        t.width_nm * manhattan(t.a, t.b)});
    }
    for (const auto& t : board.terminals) {
        if (t.net == task.net) continue;
        if (!t.pad_rect().intersects(corridor)) continue;
        const NetInfo* on = board.find_net(t.net);
        hits.push_back({"pad:net=" + std::string(on ? on->name : "?") +
                            (t.component.empty() ? "" : ":" + t.component + "." + t.pin),
                        t.pad_w_nm * t.pad_h_nm});
    }
    std::sort(hits.begin(), hits.end(), [](const Hit& a, const Hit& b) { return a.area > b.area; });
    for (std::size_t i = 0; i < hits.size() && i < 5; ++i) out.push_back(hits[i].desc);
    if (last && last->closest_node >= 0) {
        out.push_back("frontier_gap_mm=" + std::to_string(nm_to_mm(last->closest_goal_dist_nm)));
    }
    return out;
}

}  // namespace

RouteReport RouterEngine::run() {
    auto t0 = std::chrono::steady_clock::now();
    RouteReport report;
    report.total_terminals = static_cast<int>(board_.terminals.size());

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
    // The scheduler always prefers tasks that sat out the previous epoch, so
    // hard tasks that keep failing cannot starve easy ones: every remaining
    // task is attempted at least once per coverage round. Deterministic.
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
        // Deterministic scheduler order over the remaining set: tasks that
        // sat out the previous epoch come first (starvation-free coverage),
        // then difficulty desc, then stable (net, a, b) tie-breaks.
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

        // Present congestion + reservations from all remaining corridors.
        congestion.reset_present();
        for (int i : remaining) {
            double w = 1.0 + tasks[i].difficulty / 20.0;
            congestion.add_present_corridor(corridors[i].rect, w);
        }
        std::vector<Corridor> rem_corr;
        std::vector<double> rem_diff;
        std::map<int, std::size_t> rem_pos;  // task idx -> position in ordered set
        for (std::size_t k = 0; k < ordered_idx.size(); ++k) {
            rem_pos[ordered_idx[k]] = k;
            rem_corr.push_back(corridors[ordered_idx[k]]);
            rem_diff.push_back(tasks[ordered_idx[k]].difficulty);
        }
        reservations.build(ordered_tasks, rem_corr, rem_diff);

        // ---- Worker pool: candidates only, snapshot is read-only ----
        auto epoch_t0 = std::chrono::steady_clock::now();
        std::vector<CandidateRoute> candidates(batch_n);
        // Snapshot aliases: workers hold a const reference and only build
        // thread-local graphs; committed copper is untouched until the
        // arbiter commits (verified by test: sizes equal before/after).
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
        // Workers must not mutate committed state (paranoia check in debug;
        // the type system already guarantees it: const Board&).
        if (snapshot.traces.size() != traces_before || snapshot.vias.size() != vias_before) {
            RouteFailure f;
            f.reason = "internal_worker_mutation";
            f.blockers.push_back("worker mutated committed snapshot");
            report.failures.push_back(f);
            break;
        }

        // ---- Deterministic central arbiter + atomic batch commit ----
        ArbiterResult arb = arbitrate(candidates, board_, resolver_, ctx);
        commit_candidates(board_, candidates, arb, report.stats);
        report.stats.candidates_total += static_cast<int>(batch_n);
        report.stats.candidates_accepted += static_cast<int>(arb.accepted.size());
        report.stats.candidates_rejected += static_cast<int>(arb.rejected.size());

        std::vector<char> in_batch(tasks.size(), 0);
        for (int ti : batch_idx) in_batch[ti] = 1;
        std::map<int, std::string> reject_reason;  // task idx -> reason
        for (std::size_t k = 0; k < arb.rejected.size(); ++k) {
            int ti = batch_idx[arb.rejected[k]];
            reject_reason[ti] = arb.reject_reason[k];
        }
        for (std::size_t k = 0; k < batch_n; ++k) {
            int ti = batch_idx[k];
            last_attempt[attempt_key(tasks[ti])] = candidates[k];
            if (!in_batch[ti]) continue;
        }
        // History: conflicts/failures make resources less attractive.
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
        // A task that was not in this batch keeps its failure count.
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

    report.stats.epochs_count = static_cast<int>(report.epochs.size());
    report.stats.threads_used = threads_used_max;

    // Failures for everything left unrouted.
    for (std::size_t i = 0; i < tasks.size(); ++i) {
        if (task_done[i]) continue;
        const ConnectionTask& task = tasks[i];
        // Skip tasks already reported (timeout path).
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
        } else if (last && last->found) {
            f.reason = "conflict";
        } else {
            f.reason = "unreachable";
        }
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

    // Connected terminals: single-terminal nets count as connected; others
    // need all their tree tasks routed. (Verifier gives the independent word.)
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

    if (report.failures.empty()) {
        report.status = "COMPLETE";
    } else if (timed_out) {
        report.status = "TIMEOUT";
    } else if (budget_hit) {
        report.status = "BUDGET_EXHAUSTED";
    } else {
        report.status = "INCOMPLETE";
    }
    if (options_.progress) {
        JsonValue done = JsonValue::object();
        done["event"] = "done";
        done["status"] = report.status;
        done["board_hash"] = report.board_hash;
        done["connected_terminals"] = static_cast<double>(report.connected_terminals);
        done["remaining_terminals"] =
            static_cast<double>(report.total_terminals - report.connected_terminals);
        options_.progress(done);
    }
    (void)options_.seed;  // recorded in CLI output; ordering is by stable IDs.
    return report;
}

}  // namespace copperline
