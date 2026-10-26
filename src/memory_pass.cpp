// Copperline: Memory + Adjacent Memory routing passes (Phase 3 items 16/17).
//
// See include/router/memory_pass.h for the design. Both passes are
// deterministic pattern routers that commit legal same-net copper before the
// global router runs. Legality is exact-integer via the SpatialIndex; any
// illegal leg aborts the pair (never partial-commits, never rips others).
#include "router/memory_pass.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>

#include "router/board_stats.h"
#include "router/geometry.h"
#include "router/verifier.h"

namespace copperline {

JsonValue MemoryPassReport::to_json() const {
    JsonValue o = JsonValue::object();
    o["ran"] = ran;
    o["mode"] = mode;
    o["pairs_found"] = static_cast<double>(pairs_found);
    o["pairs_committed"] = static_cast<double>(pairs_committed);
    o["pairs_skipped_illegal"] = static_cast<double>(pairs_skipped_illegal);
    o["traces_added"] = static_cast<double>(traces_added);
    o["vias_added"] = static_cast<double>(vias_added);
    JsonValue sk = JsonValue::array();
    for (const auto& s : skipped_nets) sk.as_array().push_back(s);
    o["skipped_nets"] = sk;
    return o;
}

namespace {

// Does segment (a,b) on `layer` with `width` clear any foreign copper?
// The SpatialIndex holds all board copper (pads/traces/vias/keepouts); a
// query on the swept rect (segment bbox inflated by width/2 + margin) that
// returns a foreign-net item OR a keepout aborts the leg.
bool leg_legal(const Board& board, const SpatialIndex& si, Point a, Point b,
               LayerId layer, Coord width_nm, NetId net, Coord margin) {
    (void)board;  // the index holds the live copper; the board is unused here
    Rect sweep = Rect::from_points(a, b).expanded(width_nm / 2 + margin);
    for (int id : si.query(sweep)) {
        const IndexedRect& ir = si.item(static_cast<std::size_t>(id));
        if (ir.layer != kAllLayers && ir.layer != layer) continue;
        if (ir.kind == CopperKind::kKeepout) return false;
        if (ir.net != net) return false;  // foreign copper
    }
    return true;
}

// Is there committed copper of `net` terminating at `p` on `layer`?
bool has_copper_at(const Board& board, NetId net, LayerId layer, Point p) {
    for (const auto& t : board.traces)
        if (t.net == net && t.layer == layer && (t.a == p || t.b == p))
            return true;
    return false;
}

// Build the SpatialIndex over the CURRENT committed copper (defined below).
SpatialIndex build_index(const Board& board);

// Legally commit a polyline. Every leg must clear foreign copper and
// keepouts; the committed copper must also pass the independent verifier's
// local legality check (width/clearance rules), otherwise it is rolled back.
// Returns the number of traces added (0 = aborted), setting `illegal`.
int commit_polyline(Board& board, SpatialIndex& si, const RuleResolver& resolver,
                    const ElectricalContext& ctx, NetId net, LayerId layer,
                    Coord width, Coord clear, const std::vector<Point>& pts,
                    bool* illegal) {
    if (pts.size() < 2) return 0;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        if (pts[i] == pts[i + 1]) continue;  // degenerate leg
        if (!leg_legal(board, si, pts[i], pts[i + 1], layer, width, net, clear)) {
            if (illegal) *illegal = true;
            return 0;
        }
    }
    const std::size_t before = board.traces.size();
    int added = 0;
    Rect area = Rect::from_points(pts.front(), pts.front());
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        area = rect_union(area, Rect::from_points(pts[i], pts[i + 1]));
        if (pts[i] == pts[i + 1]) continue;
        TraceSeg s{net, layer, pts[i], pts[i + 1], width};
        board.traces.push_back(s);
        si.insert({Rect::from_points(s.a, s.b), net, layer, CopperKind::kTrace,
                   -1});
        ++added;
    }
    // Rule-level gate: a pattern leg must satisfy the net's own width and
    // clearance rules too (a board-default width is not enough).
    BoardVerifier verifier;
    if (verifier.has_local_violation(board, resolver, ctx, net,
                                     area.expanded(width))) {
        board.traces.resize(before);
        si = build_index(board);  // drop the rejected entries
        if (illegal) *illegal = true;
        return 0;
    }
    return added;
}

// Deterministic 3-leg U for a pair aligned on X or Y. The connecting run
// sits a jog OUTSIDE both pads (perpendicular to the alignment), so no leg
// degenerates:
//   aligned_x (shared Y): A up -> run across -> B down
//   aligned_y (shared X): A right -> run across -> B left
std::vector<Point> u_points(const Terminal& a, const Terminal& b, Coord width,
                            Coord clear) {
    const Coord clear_pad = width / 2 + clear;
    const Coord jog = std::max((a.pad_h_nm + 1) / 2, (b.pad_h_nm + 1) / 2) +
                      clear_pad;  // opening direction: Y
    const Coord jog_x = std::max((a.pad_w_nm + 1) / 2, (b.pad_w_nm + 1) / 2) +
                        clear_pad;  // opening direction: X
    if (a.pos.y == b.pos.y) {  // aligned on X -> U opens in +Y
        const Coord run_y = std::max(a.pos.y, b.pos.y) + jog;
        return {a.pos, Point{a.pos.x, run_y}, Point{b.pos.x, run_y}, b.pos};
    }
    // aligned on Y -> U opens in +X
    const Coord run_x = std::max(a.pos.x, b.pos.x) + jog_x;
    return {a.pos, Point{run_x, a.pos.y}, Point{run_x, b.pos.y}, b.pos};
}

// Build the SpatialIndex over the CURRENT committed copper.
SpatialIndex build_index(const Board& board) {
    SpatialIndex si(mm_to_nm(1.0));
    for (const auto& t : board.terminals)
        si.insert({t.pad_rect(), t.net, t.layer, CopperKind::kPad, t.id});
    for (const auto& tr : board.traces)
        si.insert({Rect::from_points(tr.a, tr.b), tr.net, tr.layer,
                   CopperKind::kTrace, -1});
    for (const auto& v : board.vias)
        si.insert({Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm),
                   v.net, kAllLayers, CopperKind::kVia, -1});
    for (const auto& k : board.keepouts)
        si.insert({k.rect, -1, k.layer, CopperKind::kKeepout, -1});
    return si;
}

// Group terminals by net with a deterministic order.
std::map<NetId, std::vector<const Terminal*>> terminals_by_net(
    const Board& board) {
    std::map<NetId, std::vector<const Terminal*>> by_net;
    for (const auto& t : board.terminals) by_net[t.net].push_back(&t);
    for (auto& [net, terms] : by_net) {
        std::sort(terms.begin(), terms.end(),
                  [](const Terminal* a, const Terminal* b) {
                      if (a->pos.x != b->pos.x) return a->pos.x < b->pos.x;
                      if (a->pos.y != b->pos.y) return a->pos.y < b->pos.y;
                      return a->id < b->id;
                  });
    }
    return by_net;
}

// Board default trace width + clearance (fallbacks when the resolver has no
// explicit rule for the net).
void pattern_rules(const Board& board, Coord* width, Coord* clear) {
    *width = board.defaults.trace_width_nm;
    if (*width <= 0) *width = mm_to_nm(0.2);
    *clear = board.defaults.clearance_nm;
    if (*clear <= 0) *clear = mm_to_nm(0.15);
}

// Per-net pattern geometry rules: the resolver's required width (explicit
// class/min-width/ampacity rules win over the board default) and clearance.
void net_pattern_rules(const Board& board, const RuleResolver& resolver,
                       const ElectricalContext& ctx, NetId net, LayerId layer,
                       Coord* width, Coord* clear) {
    pattern_rules(board, width, clear);
    const Coord w = resolver.requiredTraceWidth(net, layer, ctx, nullptr);
    if (w > 0) *width = w;
    const Coord c = resolver.requiredClearance(net, net, layer, ctx, nullptr);
    if (c > *clear) *clear = c;
}

}  // namespace

MemoryPassReport run_memory_pass(Board& board, const RuleResolver& resolver,
                                 const ElectricalContext& ctx,
                                 const std::vector<std::string>& via_names) {
    MemoryPassReport rep;
    rep.ran = true;
    rep.mode = "memory";
    (void)via_names;
    if (board.terminals.size() < 2) return rep;

    SpatialIndex si = build_index(board);

    auto by_net = terminals_by_net(board);
    for (auto& [net, terms] : by_net) {
        for (std::size_t i = 0; i < terms.size(); ++i) {
            for (std::size_t j = i + 1; j < terms.size(); ++j) {
                const Terminal* a = terms[i];
                const Terminal* b = terms[j];
                // Memory pattern: different components, same layer, aligned
                // on X or Y.
                if (a->component.empty() || b->component.empty()) continue;
                if (a->component == b->component) continue;
                if (a->layer != b->layer) continue;
                const bool aligned_x = (a->pos.y == b->pos.y);
                const bool aligned_y = (a->pos.x == b->pos.x);
                if (!aligned_x && !aligned_y) continue;
                Coord width = 0, clear = 0;
                net_pattern_rules(board, resolver, ctx, net, a->layer, &width,
                                  &clear);
                // Already joined by committed copper (e.g. an earlier pass)?
                if (has_copper_at(board, net, a->layer, a->pos) &&
                    has_copper_at(board, net, a->layer, b->pos))
                    continue;
                ++rep.pairs_found;
                bool illegal = false;
                const int added = commit_polyline(
                    board, si, resolver, ctx, net, a->layer, width, clear,
                    u_points(*a, *b, width, clear), &illegal);
                if (added == 0) {
                    ++rep.pairs_skipped_illegal;
                    const NetInfo* n = board.find_net(net);
                    if (n && std::find(rep.skipped_nets.begin(),
                                       rep.skipped_nets.end(),
                                       n->name) == rep.skipped_nets.end())
                        rep.skipped_nets.push_back(n->name);
                    continue;
                }
                rep.traces_added += added;
                ++rep.pairs_committed;
            }
        }
    }
    return rep;
}

MemoryPassReport run_adjacent_memory_pass(
    Board& board, const RuleResolver& resolver, const ElectricalContext& ctx,
    const std::vector<std::string>& via_names, Coord max_pitch_nm) {
    MemoryPassReport rep;
    rep.ran = true;
    rep.mode = "adjacent_memory";
    (void)via_names;
    if (board.terminals.size() < 2 || max_pitch_nm <= 0) return rep;

    SpatialIndex si = build_index(board);

    auto by_net = terminals_by_net(board);
    for (auto& [net, terms] : by_net) {
        for (std::size_t i = 0; i < terms.size(); ++i) {
            for (std::size_t j = i + 1; j < terms.size(); ++j) {
                const Terminal* a = terms[i];
                const Terminal* b = terms[j];
                if (a->layer != b->layer) continue;
                Coord width = 0, clear = 0;
                net_pattern_rules(board, resolver, ctx, net, a->layer, &width,
                                  &clear);
                // Adjacency: same net, same layer, Chebyshev pin distance
                // within the pitch bound (components may be equal).
                const Coord dx = a->pos.x > b->pos.x ? a->pos.x - b->pos.x
                                                    : b->pos.x - a->pos.x;
                const Coord dy = a->pos.y > b->pos.y ? a->pos.y - b->pos.y
                                                    : b->pos.y - a->pos.y;
                if (dx == 0 && dy == 0) continue;
                if (dx > max_pitch_nm || dy > max_pitch_nm) continue;
                if (has_copper_at(board, net, a->layer, a->pos) &&
                    has_copper_at(board, net, a->layer, b->pos))
                    continue;
                ++rep.pairs_found;
                std::vector<Point> pts;
                if (a->pos.x == b->pos.x || a->pos.y == b->pos.y) {
                    pts = u_points(*a, *b, width, clear);
                } else {
                    // Offset adjacent pins: deterministic 2-leg L dogleg
                    // (exit along X from A, then along Y into B).
                    pts = {a->pos, Point{b->pos.x, a->pos.y}, b->pos};
                }
                bool illegal = false;
                const int added =
                    commit_polyline(board, si, resolver, ctx, net, a->layer,
                                    width, clear, pts, &illegal);
                if (added == 0) {
                    ++rep.pairs_skipped_illegal;
                    const NetInfo* n = board.find_net(net);
                    if (n && std::find(rep.skipped_nets.begin(),
                                       rep.skipped_nets.end(),
                                       n->name) == rep.skipped_nets.end())
                        rep.skipped_nets.push_back(n->name);
                    continue;
                }
                rep.traces_added += added;
                ++rep.pairs_committed;
            }
        }
    }
    return rep;
}

}  // namespace copperline
