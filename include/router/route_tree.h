// Copperline: RouteTree + connection tasks.
//
// A multi-terminal net is decomposed into point-to-point connection tasks.
// Two-terminal nets use a single direct task. Multi-terminal nets grow from
// the actual committed route tree: connectivity components are derived from
// committed same-net copper + terminals, and each unconnected component gets
// one task attaching to the main (largest) component via the nearest legal
// contact point on its copper -- not a frozen terminal-pair MST. After every
// commit/rip-up the affected nets' tasks are regenerated, so later terminals
// share trunks (including T-junctions mid-copper) and rip-up never leaves
// stale task topology.
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "router/board.h"
#include "router/rules.h"

namespace copperline {

struct CopperTarget {
    Point p{};
    LayerId layer = 0;
};

struct ConnectionTask {
    NetId net = -1;
    TermId a = -1;
    TermId b = -1;  // representative terminal in the target component
    int index = 0;          // position inside the tree (deterministic)
    double difficulty = 0;  // filled by score_tasks()
    // Issue #4: when true, routing terminates on committed same-net copper
    // at copper_point/copper_layer (a T-junction) instead of terminal b's
    // pad. b remains the stable representative terminal for reporting,
    // hashing and deterministic ordering.
    bool has_copper_target = false;
    Point copper_point{};
    LayerId copper_layer = 0;
    // Issue #16: when true, routing terminates by entering a declared
    // plane/zone at plane_point/plane_layer (plane_id, island) instead of
    // running point-to-point to terminal b. For plane tasks b == a (one
    // task per unconnected terminal); connectivity is satisfied when the
    // terminal's copper joins the target island's electrical network.
    // Plane targets take precedence over copper targets in dst resolution.
    bool has_plane_target = false;
    int plane_id = -1;
    Point plane_point{};
    LayerId plane_layer = 0;
    int plane_island = 0;
    // Issue #17 (#15 hook): intentional tuning regions (e.g. length-tuning
    // meanders) are exempt from arbitrary-angle simplification. The flag
    // exists and is honored by the simplifier; no producer sets it yet.
    bool tuning_exempt = false;
    // Issue #12: pair-corridor task. Replaces the two individual P/N net
    // tasks with one atomic corridor object. net/a/b carry the P member
    // (representative for ordering/hashing); pair_* carry the N member.
    // Routing endpoints are the P/N pad midpoints (see task_src/dst_point).
    // P/N must not route independently while this task is active.
    bool is_pair_corridor = false;
    int pair_id = -1;
    NetId pair_other_net = -1;  // N net id
    TermId pair_a_other = -1;   // N source terminal
    TermId pair_b_other = -1;   // N destination terminal
};

struct RouteTree {
    NetId net = -1;
    std::vector<ConnectionTask> tasks;
};

RouteTree build_route_tree(const Board& board, NetId net);

// Issue #4: connectivity derived from actual committed geometry + pads.
// Terminals of one net grouped by copper connectivity (pads + traces + vias
// touching on overlapping layers). Deterministic order: groups sorted by
// minimum terminal id, terminals inside each group sorted ascending.
std::vector<std::vector<TermId>> net_terminal_components(const Board& board, NetId net);
bool terminals_connected(const Board& board, NetId net, TermId a, TermId b);
bool task_already_connected(const Board& board, const ConnectionTask& task);

// Nearest legal contact points on the target component's committed copper
// (trace endpoints/midpoints, via positions, member pads) to the source
// terminal. Sorted by (manhattan, x, y, layer), truncated to max_n.
// Used to build the A* multi-target set so the router can T-junction
// mid-copper instead of running redundant point-to-point edges.
std::vector<CopperTarget> copper_contacts_for(const Board& board, NetId net,
                                              TermId source,
                                              const std::vector<TermId>& target_component,
                                              int max_n = 8);

// Resolved routing endpoints honoring plane/copper targets. src is always
// terminal a; dst is the plane entry point when present, else the copper
// point when present, else terminal b.
Point task_src_point(const Board& board, const ConnectionTask& task);
Point task_dst_point(const Board& board, const ConnectionTask& task);
LayerId task_src_layer(const Board& board, const ConnectionTask& task);
LayerId task_dst_layer(const Board& board, const ConnectionTask& task);

// Issue #16: plane-connectivity queries. Geometric overlap alone never
// proves contact: the caller must name the expected net, and the match must
// agree on net + layer (+ island for island_at). Deterministic: overlapping
// planes resolve to the smallest plane id.
const PlaneZone* find_plane(const Board& board, int plane_id);
// True when pos lies on a plane of `net` on `layer`; reports the plane and
// its island. False for wrong-net/wrong-layer overlap.
bool plane_island_at(const Board& board, NetId net, Point pos, LayerId layer,
                     int& plane_id_out, int& island_out);
// True when the net declares at least one routable plane target.
bool net_has_routable_planes(const Board& board, NetId net);
// Nearest eligible (same-net, routable) plane entry for a source point.
// Prefers the shortest Manhattan access; ties prefer same-layer entry,
// then smallest plane id. Reports the entry point inside/on the polygon.
bool nearest_plane_target(const Board& board, NetId net, Point src, LayerId src_layer,
                          int& plane_id_out, Point& entry_out, LayerId& layer_out,
                          int& island_out);

// Difficulty incorporates span, electrical width burden, voltage clearance
// burden and endpoint pin density (phase-1 subset of the Prompt-3 vector).
double task_difficulty(const Board& board, const RuleResolver& resolver,
                       const ConnectionTask& task, const ElectricalContext& ctx,
                       const std::vector<double>& terminal_density);

// Deterministic order: difficulty desc, then (net, a, b).
void sort_tasks_deterministic(std::vector<ConnectionTask>& tasks);

}  // namespace copperline
