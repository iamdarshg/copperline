#include "router/astar.h"

#include <algorithm>
#include <limits>
#include <queue>
#include <tuple>
#include <vector>

namespace copperline {

namespace {

// Directions: 0:+x 1:+y 2:-x 3:-y 4:none(start)
int edge_exit_dirs(const SparseEdge& e, int& first, int& second) {
    first = e.dir1;
    second = e.dir2;
    return e.dir2 >= 0 ? 2 : 1;
}

}  // namespace

AStarResult astar_route(const SparseRoutingGraph& graph, const std::vector<double>& layer_mult,
                        const AStarConfig& config) {
    return astar_route_masked(graph, layer_mult, config, {}, {});
}

AStarResult astar_route_masked(const SparseRoutingGraph& graph,
                               const std::vector<double>& layer_mult,
                               const AStarConfig& config,
                               const std::vector<char>& allowed,
                               const std::vector<Coord>& enter_bias) {
    AStarResult out;
    int n = static_cast<int>(graph.nodes().size());
    int src = graph.src_node();
    const std::vector<int>& dsts = graph.dst_nodes();
    int dst = graph.dst_node();
    if (src < 0 || dst < 0 || src >= n || dst >= n) {
        out.fail_reason = "unreachable";
        return out;
    }
    // Multi-target (issue #4): any dst node completes the route. Membership
    // is by node id; dsts is tiny (<=8) so linear scan is cheapest.
    auto is_goal = [&](int node) {
        for (int d : dsts)
            if (node == d) return true;
        return false;
    };
    if (is_goal(src)) {
        out.found = true;
        out.node_path = {src};
        return out;
    }

    // Hoisted invariants: cheapest layer multiplier (admissible scaling) and
    // the per-node layer cost factor, so the inner loop does no scans.
    double min_mult = 1.0;
    if (!layer_mult.empty()) {
        min_mult = layer_mult[0];
        for (double v : layer_mult) min_mult = std::min(min_mult, v);
    }
    std::vector<double> node_layer_cost(n, 1.0);
    for (int i = 0; i < n; ++i) {
        LayerId l = graph.nodes()[i].layer;
        if (l >= 0 && l < static_cast<int>(layer_mult.size()) && layer_mult[l] > 0)
            node_layer_cost[i] = layer_mult[l];
    }

    // Issue #14: weighted-A* heuristic scale (1.0 = legacy admissible).
    // Ordering-only: legality is structural (illegal edges never built).
    double weight = config.weight_factor;
    if (!(weight >= 1.0)) weight = 1.0;
    if (weight > 4.0) weight = 4.0;

    constexpr Coord kInf = std::numeric_limits<Coord>::max() / 4;
    // dist[node][dir]
    std::vector<std::vector<Coord>> dist(n, std::vector<Coord>(5, kInf));
    struct Came {
        int node = -1;
        int dir = 4;
        int edge = -1;
    };
    std::vector<std::vector<Came>> came(n, std::vector<Came>(5));

    auto heuristic = [&](int node) -> Coord {
        Coord best = std::numeric_limits<Coord>::max();
        for (int d : dsts) {
            Coord h = manhattan(graph.nodes()[node].p, graph.nodes()[d].p);
            if (h < best) best = h;
        }
        if (best == std::numeric_limits<Coord>::max()) best = 0;
        return static_cast<Coord>(best * min_mult);
    };

    using Entry = std::tuple<Coord, Coord, int, int>;  // (f, g, node, dir)
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
    dist[src][4] = 0;
    pq.push({static_cast<Coord>(heuristic(src) * weight), 0, src, 4});

    auto goal_dist = [&](int node) -> Coord {
        Coord best = std::numeric_limits<Coord>::max();
        for (int d : dsts) {
            Coord h = manhattan(graph.nodes()[node].p, graph.nodes()[d].p);
            if (h < best) best = h;
        }
        return best == std::numeric_limits<Coord>::max() ? 0 : best;
    };
    out.closest_node = src;
    out.closest_goal_dist_nm = goal_dist(src);

    std::int64_t expansions = 0;
    int found_dir = -1;
    int found_dst = -1;
    // Issue #23: per-node expansion ledger. A node contributes its stored
    // rejection probes once -- on first expansion -- so (node, dir)
    // re-expansions never double-count and unreached regions contribute
    // nothing. Aggregation is a single bounded pass at exit via
    // frontier_stats_for_expanded (top-N, deterministic); the ledger itself
    // is one byte per node, so memory stays flat.
    std::vector<char> expanded(n, 0);
    auto finish_frontier = [&]() {
        out.frontier_stats = graph.frontier_stats_for_expanded(expanded);
    };
    while (!pq.empty()) {
        auto [f, g, u, dir] = pq.top();
        pq.pop();
        if (g != dist[u][dir]) continue;
        if (is_goal(u)) {
            found_dir = dir;
            found_dst = u;
            break;
        }
        if (++expansions > config.max_expansions) {
            out.expansions = expansions;
            out.fail_reason = "budget_exhausted";
            finish_frontier();
            return out;
        }
        // Wall-clock deadline (checked every 4096 expansions: cheap, and only
        // ever aborts searches that were going to be cut off anyway).
        if ((expansions & 4095) == 0 &&
            std::chrono::steady_clock::now() > config.deadline) {
            out.expansions = expansions;
            out.fail_reason = "budget_exhausted";
            finish_frontier();
            return out;
        }
        expanded[u] = 1;
        Coord gd = goal_dist(u);
        if (gd < out.closest_goal_dist_nm) {
            out.closest_goal_dist_nm = gd;
            out.closest_node = u;
        }
        const auto& edges = graph.edges(u);
        for (std::size_t ei = 0; ei < edges.size(); ++ei) {
            const SparseEdge& e = edges[ei];
            // Issue #10: hard guidance window. Src/dst are force-allowed
            // by the caller, so a valid solution is never masked away
            // without the unrestricted fallback below it.
            if (!allowed.empty() &&
                (e.to < 0 || e.to >= static_cast<int>(allowed.size()) || !allowed[e.to]))
                continue;
            int ndir = dir;
            // Issue #13: the impedance layer multiplier prices physical
            // copper length only. Soft ordering costs (congestion /
            // reservation penalty_nm, enter_bias) and bend/via costs stay
            // unmultiplied.
            Coord step =
                static_cast<Coord>(e.len_nm * node_layer_cost[e.to]) + e.penalty_nm;
            if (!enter_bias.empty() && e.to >= 0 &&
                e.to < static_cast<int>(enter_bias.size()) && enter_bias[e.to] > 0)
                step += enter_bias[e.to];
            if (e.is_via) {
                step += config.via_cost_nm;
                // Via preserves incoming direction (no bend).
            } else {
                int first = 4, second = -1;
                edge_exit_dirs(e, first, second);
                if (dir != 4 && first != 4 && first != dir) step += config.bend_cost_nm;
                if (second >= 0 && second != first) step += config.bend_cost_nm;
                ndir = second >= 0 ? second : first;
            }
            Coord ng = g + step;
            if (ng < dist[e.to][ndir]) {
                dist[e.to][ndir] = ng;
                came[e.to][ndir] = {u, dir, static_cast<int>(ei)};
                pq.push({static_cast<Coord>(ng + heuristic(e.to) * weight), ng, e.to,
                         ndir});
            }
        }
    }
    out.expansions = expansions;
    if (found_dir < 0 || found_dst < 0) {
        out.fail_reason = "unreachable";
        finish_frontier();
        return out;
    }
    out.found = true;
    out.cost_nm = dist[found_dst][found_dir];
    finish_frontier();
    // Reconstruct.
    std::vector<int> nodes, edges;
    int u = found_dst, d = found_dir;
    nodes.push_back(u);
    while (!(u == src && d == 4)) {
        Came c = came[u][d];
        edges.push_back(c.edge);
        u = c.node;
        d = c.dir;
        nodes.push_back(u);
    }
    std::reverse(nodes.begin(), nodes.end());
    std::reverse(edges.begin(), edges.end());
    out.node_path = std::move(nodes);
    out.edge_path = std::move(edges);
    return out;
}

}  // namespace copperline
