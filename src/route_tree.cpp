#include "router/route_tree.h"

#include <algorithm>
#include <limits>

namespace copperline {

RouteTree build_route_tree(const Board& board, NetId net) {
    RouteTree tree;
    tree.net = net;
    const NetInfo* n = board.find_net(net);
    if (!n || n->terminals.size() < 2) return tree;

    // Deterministic Prim over terminal ids sorted ascending.
    std::vector<TermId> tids = n->terminals;
    std::sort(tids.begin(), tids.end());
    auto pos = [&](TermId tid) -> Point {
        const Terminal* t = board.find_terminal(tid);
        return t ? t->pos : Point{};
    };

    std::vector<bool> in_tree(tids.size(), false);
    in_tree[0] = true;
    std::vector<Coord> best(tids.size(), std::numeric_limits<Coord>::max());
    std::vector<int> parent(tids.size(), 0);  // best[] is seeded from tids[0]
    for (std::size_t i = 1; i < tids.size(); ++i) best[i] = manhattan(pos(tids[0]), pos(tids[i]));

    for (std::size_t count = 1; count < tids.size(); ++count) {
        int pick = -1;
        for (std::size_t i = 0; i < tids.size(); ++i) {
            if (in_tree[i]) continue;
            if (pick < 0 || best[i] < best[pick] ||
                (best[i] == best[pick] && tids[i] < tids[pick]))
                pick = i;
        }
        in_tree[pick] = true;
        ConnectionTask task;
        task.net = net;
        task.a = tids[parent[pick]];
        task.b = tids[pick];
        task.index = static_cast<int>(tree.tasks.size());
        tree.tasks.push_back(task);
        for (std::size_t i = 0; i < tids.size(); ++i) {
            if (in_tree[i]) continue;
            Coord d = manhattan(pos(tids[pick]), pos(tids[i]));
            // Strict improvement only: picks are already deterministic, so the
            // parent assignment stays deterministic without extra tie-breaks.
            if (d < best[i]) {
                best[i] = d;
                parent[i] = pick;
            }
        }
    }
    return tree;
}

double task_difficulty(const Board& board, const RuleResolver& resolver,
                       const ConnectionTask& task, const ElectricalContext& ctx,
                       const std::vector<double>& terminal_density) {
    const Terminal* ta = board.find_terminal(task.a);
    const Terminal* tb = board.find_terminal(task.b);
    if (!ta || !tb) return 0.0;
    const NetInfo* net = board.find_net(task.net);

    double span_mm = nm_to_mm(manhattan(ta->pos, tb->pos));
    std::string wsource;
    Coord width = resolver.requiredTraceWidth(task.net, ta->layer, ctx, &wsource);
    double width_mm = nm_to_mm(width);

    // Voltage clearance burden: max clearance this net demands vs anyone.
    Coord max_clear = 0;
    for (const auto& other : board.nets) {
        if (other.id == task.net) continue;
        std::string cs;
        Coord c = resolver.requiredClearance(task.net, other.id, ta->layer, ctx, &cs);
        max_clear = std::max(max_clear, c);
    }
    double clear_mm = nm_to_mm(max_clear);

    double dens = 0.0;
    for (std::size_t i = 0; i < board.terminals.size(); ++i) {
        if (board.terminals[i].id == task.a || board.terminals[i].id == task.b) {
            if (i < terminal_density.size()) dens = std::max(dens, terminal_density[i]);
        }
    }

    double difficulty = 0.0;
    difficulty += span_mm;                        // distance
    difficulty += 40.0 * width_mm;                // wide traces consume channels
    difficulty += 30.0 * clear_mm;                // clearance burden
    difficulty += 0.5 * dens;                     // dense endpoints
    if (wsource == "ipc_estimate" || wsource == "ampacity")
        difficulty += 0.5;  // inferred electrics = risk
    if (net && net->terminals.size() > 2) difficulty += 0.25 * net->terminals.size();
    return difficulty;
}

void sort_tasks_deterministic(std::vector<ConnectionTask>& tasks) {
    std::sort(tasks.begin(), tasks.end(), [](const ConnectionTask& a, const ConnectionTask& b) {
        if (a.difficulty != b.difficulty) return a.difficulty > b.difficulty;
        if (a.net != b.net) return a.net < b.net;
        if (a.a != b.a) return a.a < b.a;
        return a.b < b.b;
    });
}

}  // namespace copperline
