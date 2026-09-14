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
    AStarResult out;
    int n = static_cast<int>(graph.nodes().size());
    int src = graph.src_node();
    int dst = graph.dst_node();
    if (src < 0 || dst < 0 || src >= n || dst >= n) {
        out.fail_reason = "unreachable";
        return out;
    }
    if (src == dst) {
        out.found = true;
        out.node_path = {src};
        return out;
    }
    const Point goal = graph.nodes()[dst].p;

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
        Coord h = manhattan(graph.nodes()[node].p, goal);
        double m = 1.0;
        if (!layer_mult.empty()) {
            m = layer_mult[0];
            for (double v : layer_mult) m = std::min(m, v);
        }
        return static_cast<Coord>(h * m);
    };

    using Entry = std::tuple<Coord, Coord, int, int>;  // (f, g, node, dir)
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
    dist[src][4] = 0;
    pq.push({heuristic(src), 0, src, 4});

    out.closest_node = src;
    out.closest_goal_dist_nm = manhattan(graph.nodes()[src].p, goal);

    auto layer_cost = [&](int node) -> double {
        LayerId l = graph.nodes()[node].layer;
        if (l >= 0 && l < static_cast<int>(layer_mult.size()) && layer_mult[l] > 0)
            return layer_mult[l];
        return 1.0;
    };

    std::int64_t expansions = 0;
    int found_dir = -1;
    while (!pq.empty()) {
        auto [f, g, u, dir] = pq.top();
        pq.pop();
        if (g != dist[u][dir]) continue;
        if (u == dst) {
            found_dir = dir;
            break;
        }
        if (++expansions > config.max_expansions) {
            out.expansions = expansions;
            out.fail_reason = "budget_exhausted";
            return out;
        }
        Coord gd = manhattan(graph.nodes()[u].p, goal);
        if (gd < out.closest_goal_dist_nm) {
            out.closest_goal_dist_nm = gd;
            out.closest_node = u;
        }
        const auto& edges = graph.edges(u);
        for (std::size_t ei = 0; ei < edges.size(); ++ei) {
            const SparseEdge& e = edges[ei];
            int ndir = dir;
            Coord step = e.len_nm + e.penalty_nm;
            if (e.is_via) {
                step += config.via_cost_nm;
                // Via preserves incoming direction (no bend).
            } else {
                int first = 4, second = -1;
                edge_exit_dirs(e, first, second);
                step = static_cast<Coord>(step * layer_cost(e.to));
                if (dir != 4 && first != 4 && first != dir) step += config.bend_cost_nm;
                if (second >= 0 && second != first) step += config.bend_cost_nm;
                ndir = second >= 0 ? second : first;
            }
            Coord ng = g + step;
            if (ng < dist[e.to][ndir]) {
                dist[e.to][ndir] = ng;
                came[e.to][ndir] = {u, dir, static_cast<int>(ei)};
                pq.push({ng + heuristic(e.to), ng, e.to, ndir});
            }
        }
    }
    out.expansions = expansions;
    if (found_dir < 0) {
        out.fail_reason = "unreachable";
        return out;
    }
    out.found = true;
    out.cost_nm = dist[dst][found_dir];
    // Reconstruct.
    std::vector<int> nodes, edges;
    int u = dst, d = found_dir;
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
