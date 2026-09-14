#include "router/engine.h"

#include <algorithm>
#include <cmath>

#include "router/density.h"
#include "router/route_tree.h"
#include "router/sparse_graph.h"

namespace copperline {

JsonValue RouteReport::to_json() const {
    JsonValue r = JsonValue::object();
    r["schema"] = "copperline/route-report/1";
    r["status"] = status;
    r["connected_terminals"] = static_cast<double>(connected_terminals);
    r["total_terminals"] = static_cast<double>(total_terminals);
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
    r["stats"] = st;
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

RouteReport RouterEngine::run() {
    auto t0 = std::chrono::steady_clock::now();
    RouteReport report;
    report.total_terminals = static_cast<int>(board_.terminals.size());

    DensityEstimator density_est;
    DensityResult density = density_est.analyze(board_);
    ElectricalContext ctx;

    // Collect tasks from every net's tree.
    std::vector<ConnectionTask> tasks;
    for (const auto& net : board_.nets) {
        if (net.terminals.size() < 2) continue;
        RouteTree tree = build_route_tree(board_, net.id);
        for (auto& t : tree.tasks) {
            t.difficulty = task_difficulty(board_, resolver_, t, ctx, density.terminal_density);
            tasks.push_back(t);
        }
    }
    sort_tasks_deterministic(tasks);
    report.stats.tasks_total = static_cast<int>(tasks.size());
    report.stats.nets_total = static_cast<int>(board_.nets.size());
    report.stats.threads_requested = options_.threads;
    report.stats.threads_used = 1;

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

    auto deadline = options_.timeout_s > 0
                        ? t0 + std::chrono::duration<double>(options_.timeout_s)
                        : std::chrono::steady_clock::time_point::max();
    bool timed_out = false;
    bool budget_hit = false;

    std::vector<char> net_ok(board_.nets.size(), 1);
    auto net_index = [&](NetId id) -> int {
        for (std::size_t i = 0; i < board_.nets.size(); ++i)
            if (board_.nets[i].id == id) return static_cast<int>(i);
        return -1;
    };

    for (const auto& task : tasks) {
        if (std::chrono::steady_clock::now() > deadline) {
            timed_out = true;
            RouteFailure f;
            f.net = task.net;
            const NetInfo* n = board_.find_net(task.net);
            f.net_name = n ? n->name : "?";
            f.a = task.a;
            f.b = task.b;
            f.reason = "timeout";
            report.failures.push_back(f);
            int ni = net_index(task.net);
            if (ni >= 0) net_ok[ni] = 0;
            continue;
        }
        const Terminal* ta = board_.find_terminal(task.a);
        const Terminal* tb = board_.find_terminal(task.b);
        if (!ta || !tb) {
            RouteFailure f;
            f.net = task.net;
            const NetInfo* nx = board_.find_net(task.net);
            f.net_name = nx ? nx->name : "?";
            f.a = task.a;
            f.b = task.b;
            f.reason = "bad_task";
            f.blockers.push_back("terminal id not found on board");
            report.failures.push_back(f);
            int ni0 = net_index(task.net);
            if (ni0 >= 0) net_ok[ni0] = 0;
            continue;
        }
        TraceRule rule = resolver_.traceRule(task.net, ta->layer, kAnyRegion);
        Coord width = rule.pref_width_nm;

        SparseRoutingGraph graph = SparseRoutingGraph::build(board_, resolver_, task.net, ta->pos,
                                                             tb->pos, ta->layer, tb->layer, width,
                                                             ctx);
        AStarResult res = astar_route(graph, layer_mult, options_.astar);
        report.stats.expansions_total += res.expansions;
        if (!res.found) {
            RouteFailure f;
            f.net = task.net;
            const NetInfo* n = board_.find_net(task.net);
            f.net_name = n ? n->name : "?";
            f.a = task.a;
            f.b = task.b;
            f.reason = res.fail_reason == "budget_exhausted" ? "budget_exhausted" : "unreachable";
            if (f.reason == "budget_exhausted") budget_hit = true;
            f.expansions = res.expansions;
            f.required_width_mm = nm_to_mm(width);
            f.width_source = rule.width_source;
            // Blocker attribution: obstacles overlapping the task corridor.
            Rect corridor = Rect::from_points(ta->pos, tb->pos).expanded(width);
            struct Hit {
                std::string desc;
                Coord area;
            };
            std::vector<Hit> hits;
            for (const auto& ko : board_.keepouts) {
                if (!ko.rect.intersects(corridor)) continue;
                Rect inter{std::max(ko.rect.x1, corridor.x1), std::max(ko.rect.y1, corridor.y1),
                           std::min(ko.rect.x2, corridor.x2), std::min(ko.rect.y2, corridor.y2)};
                __int128 area = (__int128)(inter.x2 - inter.x1) * (inter.y2 - inter.y1);
                hits.push_back({"keepout:" + ko.reason, static_cast<Coord>(area)});
            }
            for (const auto& t : board_.traces) {
                if (t.net == task.net) continue;
                if (!t.segment().bounds().expanded(t.width_nm).intersects(corridor)) continue;
                const NetInfo* on = board_.find_net(t.net);
                hits.push_back({"trace:net=" + std::string(on ? on->name : "?"),
                                t.width_nm * manhattan(t.a, t.b)});
            }
            for (const auto& t : board_.terminals) {
                if (t.net == task.net) continue;
                if (!t.pad_rect().intersects(corridor)) continue;
                const NetInfo* on = board_.find_net(t.net);
                hits.push_back({"pad:net=" + std::string(on ? on->name : "?") +
                                    (t.component.empty() ? "" : ":" + t.component + "." + t.pin),
                                t.pad_w_nm * t.pad_h_nm});
            }
            std::sort(hits.begin(), hits.end(),
                      [](const Hit& a, const Hit& b) { return a.area > b.area; });
            for (std::size_t i = 0; i < hits.size() && i < 5; ++i)
                f.blockers.push_back(hits[i].desc);
            if (res.closest_node >= 0) {
                f.blockers.push_back("frontier_gap_mm=" +
                                     std::to_string(nm_to_mm(res.closest_goal_dist_nm)));
            }
            report.failures.push_back(f);
            int ni = net_index(task.net);
            if (ni >= 0) net_ok[ni] = 0;
            continue;
        }
        // Commit the path.
        ViaStyle style;
        LayerSpan full{board_.layers.front().id, board_.layers.back().id};
        resolver_.select_via(task.net, full, style);
        for (std::size_t i = 0; i < res.edge_path.size(); ++i) {
            int u = res.node_path[i];
            const SparseEdge& e = graph.edges(u)[res.edge_path[i]];
            const SparseNode& nu = graph.nodes()[u];
            const SparseNode& nv = graph.nodes()[e.to];
            if (e.is_via) {
                Via v;
                v.net = task.net;
                v.pos = nu.p;
                v.top_layer = std::min(nu.layer, nv.layer);
                v.bottom_layer = std::max(nu.layer, nv.layer);
                v.outer_d_nm = style.outer_nm;
                v.hole_d_nm = style.hole_nm;
                v.via_class = style.name;
                board_.vias.push_back(v);
                report.stats.via_count++;
            } else if (e.dir2 >= 0) {
                TraceSeg s1{task.net, nu.layer, nu.p, e.elbow, width};
                TraceSeg s2{task.net, nu.layer, e.elbow, nv.p, width};
                report.stats.length_nm += manhattan(nu.p, e.elbow) + manhattan(e.elbow, nv.p);
                board_.traces.push_back(s1);
                if (!(e.elbow == nv.p)) board_.traces.push_back(s2);
            } else {
                TraceSeg s{task.net, nu.layer, nu.p, nv.p, width};
                report.stats.length_nm += manhattan(nu.p, nv.p);
                if (!(nu.p == nv.p)) board_.traces.push_back(s);
            }
        }
        report.stats.tasks_routed++;
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
        for (const auto& task : tasks) {
            if (task.net != net.id) continue;
            bool routed = true;
            for (const auto& f : report.failures) {
                if (f.net == task.net && ((f.a == task.a && f.b == task.b) ||
                                          (f.a == task.b && f.b == task.a))) {
                    routed = false;
                    break;
                }
            }
            if (!routed) {
                all_ok = false;
                break;
            }
        }
        if (all_ok) report.connected_terminals += static_cast<int>(net.terminals.size());
    }

    auto t1 = std::chrono::steady_clock::now();
    report.stats.time_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    if (report.failures.empty()) {
        report.status = "COMPLETE";
    } else if (timed_out) {
        report.status = "TIMEOUT";
    } else if (budget_hit) {
        report.status = "BUDGET_EXHAUSTED";
    } else {
        report.status = "INCOMPLETE";
    }
    (void)options_.seed;  // seed threads the deterministic pipeline (phase 3 uses it
                           // for tie-break shuffling); recorded in CLI output.
    return report;
}

}  // namespace copperline
