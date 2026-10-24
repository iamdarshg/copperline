// Copperline: Memory routing pass (Phase 3 item 16).
//
// See include/router/memory_pass.h for the design. This is a deterministic
// pattern router: same-net terminal pairs in different components that share
// an X or Y coordinate on the same layer get a legal U-pattern route before
// the global router runs. Legality is exact-integer via the SpatialIndex;
// any illegal leg aborts the pair (never partial-commits, never rips others).
#include "router/memory_pass.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

#include "router/geometry.h"

namespace copperline {

JsonValue MemoryPassReport::to_json() const {
    JsonValue o = JsonValue::object();
    o["ran"] = ran;
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

// Clearance-expanded rect of a pad. Both pads are expanded by the net's
// required clearance so the pattern legs never violate same-net spacing.
Rect pad_clear_rect(const Terminal& t, Coord clear_nm) {
    Rect r = t.pad_rect();
    return r.expanded(clear_nm);
}

// Does segment (a,b) on `layer` with `width` clear any foreign copper?
// The SpatialIndex holds all board copper (pads/traces/vias/keepouts); a
// query on the swept rect (segment bbox inflated by width/2 + margin) that
// returns a foreign-net item OR a keepout aborts the leg.
bool leg_legal(const Board& board, const SpatialIndex& si, Point a, Point b,
               LayerId layer, Coord width_nm, NetId net, Coord margin) {
    Rect sweep = Rect::from_points(a, b).expanded(width_nm / 2 + margin);
    for (int id : si.query(sweep)) {
        const IndexedRect& ir = si.item(static_cast<std::size_t>(id));
        if (ir.layer != kAllLayers && ir.layer != layer) continue;
        if (ir.kind == CopperKind::kKeepout) return false;
        if (ir.net != net) return false;  // foreign copper
    }
    return true;
}

}  // namespace

MemoryPassReport run_memory_pass(Board& board, const RuleResolver& resolver,
                                 const ElectricalContext& ctx,
                                 const std::vector<std::string>& via_names) {
    MemoryPassReport rep;
    rep.ran = true;
    // resolver/ctx are reserved for future per-net width/clearance
    // resolution (pattern legs currently use the board floor).
    (void)resolver;
    (void)ctx;
    (void)via_names;
    if (board.terminals.size() < 2) return rep;

    // Spatial index over the CURRENT committed copper (pads + any pre-route).
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

    // Default trace width + clearance for the pattern geometry (board
    // defaults; the resolver path needs a real net id, and pattern legs use
    // the board floor conservatively).
    Coord width = board.defaults.trace_width_nm;
    if (width <= 0) width = mm_to_nm(0.2);
    Coord clear = board.defaults.clearance_nm;
    if (clear <= 0) clear = mm_to_nm(0.15);

    // Group terminals by net; sort pairs deterministically.
    std::map<NetId, std::vector<const Terminal*>> by_net;
    for (const auto& t : board.terminals)
        by_net[t.net].push_back(&t);
    for (auto& [net, terms] : by_net) {
        std::sort(terms.begin(), terms.end(),
                  [](const Terminal* a, const Terminal* b) {
                      if (a->pos.x != b->pos.x) return a->pos.x < b->pos.x;
                      if (a->pos.y != b->pos.y) return a->pos.y < b->pos.y;
                      return a->id < b->id;
                  });
        for (std::size_t i = 0; i < terms.size(); ++i) {
            for (std::size_t j = i + 1; j < terms.size(); ++j) {
                const Terminal* a = terms[i];
                const Terminal* b = terms[j];
                // Memory pattern: different components, same layer, aligned
                // on X or Y.
                if (a->component.empty() || b->component.empty()) continue;
                if (a->component == b->component) continue;
                if (a->layer != b->layer) continue;
                bool aligned_x = (a->pos.y == b->pos.y);  // shared Y -> U opens above
                bool aligned_y = (a->pos.x == b->pos.x);  // shared X -> U opens right
                if (!aligned_x && !aligned_y) continue;
                ++rep.pairs_found;

                // Deterministic 3-leg U pattern. The connecting run sits a
                // jog OUTSIDE both pads (perpendicular to the alignment), so
                // no leg degenerates:
                //   aligned_x (shared Y): A up -> run across -> B down
                //   aligned_y (shared X): A right -> run across -> B left
                // The jog clears the pad by width/2 + clearance beyond the
                // pad half-extent in the opening axis.
                Coord clear_pad = width / 2 + clear;
                Coord jog = std::max((a->pad_h_nm + 1) / 2, (b->pad_h_nm + 1) / 2) +
                            clear_pad;                 // opening direction: Y
                Coord jog_x = std::max((a->pad_w_nm + 1) / 2, (b->pad_w_nm + 1) / 2) +
                              clear_pad;               // opening direction: X
                Point c1, c2;
                if (aligned_x) {
                    Coord run_y = std::max(a->pos.y, b->pos.y) + jog;
                    c1 = Point{a->pos.x, run_y};
                    c2 = Point{b->pos.x, run_y};
                } else {
                    Coord run_x = std::max(a->pos.x, b->pos.x) + jog_x;
                    c1 = Point{run_x, a->pos.y};
                    c2 = Point{run_x, b->pos.y};
                }
                // Legs: A->c1, c1->c2 (the run), c2->B.
                bool ok = leg_legal(board, si, a->pos, c1, a->layer,
                                    width, net, clear) &&
                          leg_legal(board, si, c1, c2, a->layer,
                                    width, net, clear) &&
                          leg_legal(board, si, c2, b->pos, a->layer,
                                    width, net, clear);
                if (!ok) {
                    ++rep.pairs_skipped_illegal;
                    const NetInfo* n = board.find_net(net);
                    if (n && std::find(rep.skipped_nets.begin(),
                                       rep.skipped_nets.end(),
                                       n->name) == rep.skipped_nets.end())
                        rep.skipped_nets.push_back(n->name);
                    continue;
                }
                // Commit: three trace segments + refresh the index.
                TraceSeg s1{net, a->layer, a->pos, c1, width};
                TraceSeg s2{net, a->layer, c1, c2, width};
                TraceSeg s3{net, a->layer, c2, b->pos, width};
                board.traces.push_back(s1);
                board.traces.push_back(s2);
                board.traces.push_back(s3);
                si.insert({Rect::from_points(s1.a, s1.b), net, a->layer,
                           CopperKind::kTrace, -1});
                si.insert({Rect::from_points(s2.a, s2.b), net, a->layer,
                           CopperKind::kTrace, -1});
                si.insert({Rect::from_points(s3.a, s3.b), net, a->layer,
                           CopperKind::kTrace, -1});
                rep.traces_added += 3;
                ++rep.pairs_committed;
            }
        }
    }
    return rep;
}

}  // namespace copperline