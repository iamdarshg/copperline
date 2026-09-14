// Copperline: RouteTree + connection tasks.
//
// A multi-terminal net is decomposed into point-to-point connection tasks via
// a deterministic Prim MST over Manhattan terminal distance. The engine routes
// tasks in difficulty order and commits each before the next, so later tasks
// see earlier copper as obstacles.
#pragma once

#include <string>
#include <vector>

#include "router/board.h"
#include "router/rules.h"

namespace copperline {

struct ConnectionTask {
    NetId net = -1;
    TermId a = -1;
    TermId b = -1;
    int index = 0;          // position inside the tree (deterministic)
    double difficulty = 0;  // filled by score_tasks()
};

struct RouteTree {
    NetId net = -1;
    std::vector<ConnectionTask> tasks;
};

RouteTree build_route_tree(const Board& board, NetId net);

// Difficulty incorporates span, electrical width burden, voltage clearance
// burden and endpoint pin density (phase-1 subset of the Prompt-3 vector).
double task_difficulty(const Board& board, const RuleResolver& resolver,
                       const ConnectionTask& task, const ElectricalContext& ctx,
                       const std::vector<double>& terminal_density);

// Deterministic order: difficulty desc, then (net, a, b).
void sort_tasks_deterministic(std::vector<ConnectionTask>& tasks);

}  // namespace copperline
