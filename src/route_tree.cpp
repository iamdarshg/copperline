#include "router/route_tree.h"

#include <algorithm>
#include <limits>
#include <map>
#include <numeric>
#include <set>

#include "router/connectivity.h"

namespace copperline {

namespace {

// Shared connectivity (D1): single DSU + touch implementation.
using DSU = conn::DSU;

struct Elem {
    enum class Kind { kPad, kTrace, kVia, kPlane } kind;
    LayerId layer = 0;
    LayerId lo = 0, hi = 0;
    Rect rect{};
    Segment seg{};
    TermId term = -1;
    int term_idx = -1;  // index into the net's terminal list (pads only)
    int plane_id = -1;  // planes only
    int plane_island = 0;
    const std::vector<Point>* poly = nullptr;  // planes only (board-owned)
};

// Copper contact between two same-net elements. Planes join by declared
// island (same island id = stitched, regardless of layer) or by visible
// geometric bridging on a shared layer; copper touches a plane polygon on
// an overlapping layer. Wrong-net/wrong-layer overlap never unites here
// because collection is per-net (issue #16: overlap alone is not proof).
bool elem_touch(const Elem& a, const Elem& b) {
    auto is_plane = [](const Elem& e) { return e.kind == Elem::Kind::kPlane; };
    auto is_trace = [](const Elem& e) { return e.kind == Elem::Kind::kTrace; };
    auto lo_of = [](const Elem& e) -> LayerId {
        return e.kind == Elem::Kind::kVia ? e.lo : e.layer;
    };
    auto hi_of = [](const Elem& e) -> LayerId {
        return e.kind == Elem::Kind::kVia ? e.hi : e.layer;
    };
    auto layer_of = [](const Elem& e) { return e.layer; };
    auto rect_of = [](const Elem& e) -> const Rect& { return e.rect; };
    auto seg_of = [](const Elem& e) -> const Segment& { return e.seg; };
    auto island_of = [](const Elem& e) { return e.plane_island; };
    auto poly_of = [](const Elem& e) { return e.poly; };
    return conn::copper_touch_generic(a, b, is_plane, is_trace, lo_of, hi_of,
                                      layer_of, rect_of, seg_of, island_of,
                                      poly_of, /*same_island_unites=*/true);
}

// All same-net elements (pads + traces + vias) with DSU roots.
struct NetCopper {
    std::vector<Elem> elems;
    DSU dsu{0};
};

NetCopper collect_net_copper(const Board& board, NetId net) {
    NetCopper out;
    const NetInfo* n = board.find_net(net);
    std::vector<TermId> tids = n ? n->terminals : std::vector<TermId>{};
    std::sort(tids.begin(), tids.end());
    for (std::size_t i = 0; i < tids.size(); ++i) {
        const Terminal* t = board.find_terminal(tids[i]);
        if (!t) continue;
        Elem e;
        e.kind = Elem::Kind::kPad;
        e.layer = t->layer;
        e.rect = t->pad_rect();
        e.term = t->id;
        e.term_idx = static_cast<int>(i);
        out.elems.push_back(e);
    }
    for (const auto& t : board.traces) {
        if (t.net != net) continue;
        Elem e;
        e.kind = Elem::Kind::kTrace;
        e.layer = t.layer;
        e.seg = t.segment();
        out.elems.push_back(e);
    }
    for (const auto& v : board.vias) {
        if (v.net != net) continue;
        Elem e;
        e.kind = Elem::Kind::kVia;
        e.lo = std::min(v.top_layer, v.bottom_layer);
        e.hi = std::max(v.top_layer, v.bottom_layer);
        e.rect = Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm);
        out.elems.push_back(e);
    }
    // Issue #16: declared planes participate as fixed same-net copper.
    for (const auto& z : board.planes) {
        if (z.net != net) continue;
        Elem e;
        e.kind = Elem::Kind::kPlane;
        e.layer = z.layer;
        e.rect = z.bounds();
        e.plane_id = z.id;
        e.plane_island = z.island;
        e.poly = &z.poly;
        out.elems.push_back(e);
    }
    out.dsu = DSU(static_cast<int>(out.elems.size()));
    for (std::size_t i = 0; i < out.elems.size(); ++i) {
        for (std::size_t j = i + 1; j < out.elems.size(); ++j) {
            if (elem_touch(out.elems[i], out.elems[j])) out.dsu.unite((int)i, (int)j);
        }
    }
    return out;
}

}  // namespace

std::vector<std::vector<TermId>> net_terminal_components(const Board& board, NetId net) {
    const NetInfo* n = board.find_net(net);
    if (!n || n->terminals.empty()) return {};
    std::vector<TermId> tids = n->terminals;
    std::sort(tids.begin(), tids.end());
    bool has_planes = false;
    for (const auto& z : board.planes) {
        if (z.net == net) {
            has_planes = true;
            break;
        }
    }
    if (board.traces.empty() && board.vias.empty() && !has_planes) {
        // Fast path: no copper, every terminal isolated.
        std::vector<std::vector<TermId>> out;
        for (TermId t : tids) out.push_back({t});
        return out;
    }
    NetCopper copper = collect_net_copper(board, net);
    // Map terminal id -> element index.
    std::map<TermId, int> elem_of;
    for (std::size_t i = 0; i < copper.elems.size(); ++i) {
        if (copper.elems[i].kind == Elem::Kind::kPad) elem_of[copper.elems[i].term] = (int)i;
    }
    std::map<int, std::vector<TermId>> groups;
    for (TermId t : tids) {
        auto it = elem_of.find(t);
        int root = it != elem_of.end() ? copper.dsu.find(it->second) : t;
        groups[root].push_back(t);
    }
    std::vector<std::vector<TermId>> out;
    for (auto& [r, v] : groups) {
        std::sort(v.begin(), v.end());
        out.push_back(v);
    }
    // Stable order: by minimum terminal id.
    std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) {
        return a.front() < b.front();
    });
    return out;
}

bool terminals_connected(const Board& board, NetId net, TermId a, TermId b) {
    if (a == b) return true;
    for (const auto& g : net_terminal_components(board, net)) {
        bool ha = std::find(g.begin(), g.end(), a) != g.end();
        bool hb = std::find(g.begin(), g.end(), b) != g.end();
        if (ha && hb) return true;
    }
    return false;
}

const PlaneZone* find_plane(const Board& board, int plane_id) {
    for (const auto& z : board.planes) {
        if (z.id == plane_id) return &z;
    }
    return nullptr;
}

bool plane_island_at(const Board& board, NetId net, Point pos, LayerId layer,
                     int& plane_id_out, int& island_out) {
    // Issue #16: proves contact with the CORRECT electrical island. A point
    // overlapping a wrong-net or wrong-layer pour reports false: geometric
    // overlap alone is not connectivity.
    int best_plane = -1, best_island = 0;
    for (const auto& z : board.planes) {
        if (z.net != net || z.layer != layer) continue;
        if (!plane_poly_contains(z.poly, pos)) continue;
        if (best_plane < 0 || z.id < best_plane) {
            best_plane = z.id;
            best_island = z.island;
        }
    }
    if (best_plane < 0) return false;
    plane_id_out = best_plane;
    island_out = best_island;
    return true;
}

bool net_has_routable_planes(const Board& board, NetId net) {
    for (const auto& z : board.planes) {
        if (z.net == net && z.routable) return true;
    }
    return false;
}

bool nearest_plane_target(const Board& board, NetId net, Point src, LayerId src_layer,
                          int& plane_id_out, Point& entry_out, LayerId& layer_out,
                          int& island_out) {
    // Eligible = same net + routable. Shortest Manhattan access wins; ties
    // prefer a same-layer (direct copper, no via) entry, then smallest id.
    bool found = false;
    Coord best_d = 0;
    int best_same = 0, best_id = 0;
    Point best_entry{};
    LayerId best_layer = 0;
    int best_island = 0;
    for (const auto& z : board.planes) {
        if (z.net != net || !z.routable) continue;
        if (!board.valid_layer(z.layer)) continue;
        Point entry = plane_poly_nearest(z.poly, src);
        if (entry.x < 0 || entry.y < 0 || entry.x > board.width_nm ||
            entry.y > board.height_nm)
            continue;
        Coord d = manhattan(src, entry);
        int same = (z.layer == src_layer) ? 1 : 0;
        if (!found || d < best_d || (d == best_d && same > best_same) ||
            (d == best_d && same == best_same && z.id < best_id)) {
            found = true;
            best_d = d;
            best_same = same;
            best_id = z.id;
            best_entry = entry;
            best_layer = z.layer;
            best_island = z.island;
        }
    }
    if (!found) return false;
    plane_id_out = best_id;
    entry_out = best_entry;
    layer_out = best_layer;
    island_out = best_island;
    return true;
}

// True when a plane-target task is satisfied: terminal a already joins the
// target island's electrical network through committed copper/planes.
bool plane_task_satisfied(const Board& board, const ConnectionTask& task) {
    NetCopper copper = collect_net_copper(board, task.net);
    int pad_elem = -1, plane_elem = -1;
    for (std::size_t i = 0; i < copper.elems.size(); ++i) {
        const Elem& e = copper.elems[i];
        if (e.kind == Elem::Kind::kPad && e.term == task.a) pad_elem = (int)i;
        if (e.kind == Elem::Kind::kPlane && e.plane_id == task.plane_id)
            plane_elem = (int)i;
    }
    if (pad_elem < 0 || plane_elem < 0) return false;
    return copper.dsu.find(pad_elem) == copper.dsu.find(plane_elem);
}

bool task_already_connected(const Board& board, const ConnectionTask& task) {
    if (task.is_pair_corridor) {
        // Issue #12: a corridor task is satisfied only when BOTH members
        // are fully connected (never one-member-only).
        if (!terminals_connected(board, task.net, task.a, task.b)) return false;
        if (task.pair_other_net < 0) return false;
        return terminals_connected(board, task.pair_other_net, task.pair_a_other,
                                   task.pair_b_other);
    }
    if (task.has_plane_target) return plane_task_satisfied(board, task);
    if (task.a == task.b && !task.has_copper_target &&
        net_has_routable_planes(board, task.net))
        return false;  // issue #16: target-less plane task (no eligible plane
                       // existed at build time); never trivially satisfied.
    return terminals_connected(board, task.net, task.a, task.b);
}

std::vector<CopperTarget> copper_contacts_for(const Board& board, NetId net, TermId source,
                                              const std::vector<TermId>& target_component,
                                              int max_n) {
    const Terminal* src = board.find_terminal(source);
    Point sp = src ? src->pos : Point{};
    // Roots of the target component to select its copper.
    NetCopper copper = collect_net_copper(board, net);
    std::map<TermId, int> elem_of;
    for (std::size_t i = 0; i < copper.elems.size(); ++i) {
        if (copper.elems[i].kind == Elem::Kind::kPad) elem_of[copper.elems[i].term] = (int)i;
    }
    int target_root = -1;
    for (TermId t : target_component) {
        auto it = elem_of.find(t);
        if (it != elem_of.end()) {
            target_root = copper.dsu.find(it->second);
            break;
        }
    }
    std::vector<CopperTarget> cands;
    auto push = [&](Point p, LayerId layer) {
        // Must be on the board; integer-nm preserved.
        if (p.x < 0 || p.y < 0 || p.x > board.width_nm || p.y > board.height_nm) return;
        cands.push_back({p, layer});
    };
    // Member pads always count (even with no traces yet).
    for (TermId t : target_component) {
        const Terminal* term = board.find_terminal(t);
        if (term) push(term->pos, term->layer);
    }
    if (target_root >= 0) {
        for (std::size_t i = 0; i < copper.elems.size(); ++i) {
            if (copper.dsu.find((int)i) != target_root) continue;
            const Elem& e = copper.elems[i];
            if (e.kind == Elem::Kind::kTrace) {
                push(e.seg.a, e.layer);
                push(e.seg.b, e.layer);
                push({(e.seg.a.x + e.seg.b.x) / 2, (e.seg.a.y + e.seg.b.y) / 2}, e.layer);
            } else if (e.kind == Elem::Kind::kVia) {
                push(e.rect.center(), e.lo);
                push(e.rect.center(), e.hi);
            }
        }
    }
    std::sort(cands.begin(), cands.end(), [](const CopperTarget& a, const CopperTarget& b) {
        if (a.p.x != b.p.x) return a.p.x < b.p.x;
        if (a.p.y != b.p.y) return a.p.y < b.p.y;
        return a.layer < b.layer;
    });
    cands.erase(std::unique(cands.begin(), cands.end(),
                            [](const CopperTarget& a, const CopperTarget& b) {
                                return a.p == b.p && a.layer == b.layer;
                            }),
                cands.end());
    // Stable nearest-first: (manhattan, x, y, layer).
    std::sort(cands.begin(), cands.end(), [&](const CopperTarget& a, const CopperTarget& b) {
        Coord da = manhattan(sp, a.p), db = manhattan(sp, b.p);
        if (da != db) return da < db;
        if (a.p.x != b.p.x) return a.p.x < b.p.x;
        if (a.p.y != b.p.y) return a.p.y < b.p.y;
        return a.layer < b.layer;
    });
    if (max_n > 0 && (int)cands.size() > max_n) cands.resize(max_n);
    return cands;
}

// Issue #5: far (portal-side) endpoint of committed escape copper attached
// to one terminal. Returns false when the terminal has no committed
// trace/via beyond its pad (no escape stub yet). Deterministic: the farthest
// same-net copper point from the pad (manhattan, then x, y, layer); ties are
// stable. Via centers count (multilayer escapes end in a via).
static bool pair_stub_endpoint(const Board& board, TermId tid, Point& pt_out,
                               LayerId& layer_out) {
    const Terminal* t = board.find_terminal(tid);
    if (!t) return false;
    NetCopper copper = collect_net_copper(board, t->net);
    int pad_elem = -1;
    for (std::size_t i = 0; i < copper.elems.size(); ++i) {
        if (copper.elems[i].kind == Elem::Kind::kPad && copper.elems[i].term == tid) {
            pad_elem = static_cast<int>(i);
            break;
        }
    }
    if (pad_elem < 0) return false;
    int root = copper.dsu.find(pad_elem);
    bool have = false;
    Point best{};
    LayerId best_layer = t->layer;
    Coord best_d = -1;
    auto consider = [&](Point p, LayerId layer) {
        if (p == t->pos && layer == t->layer) return;
        Coord d = manhattan(t->pos, p);
        if (!have || d > best_d || (d == best_d && (p.x < best.x ||
                     (p.x == best.x && (p.y < best.y ||
                      (p.y == best.y && layer < best_layer)))))) {
            have = true;
            best = p;
            best_layer = layer;
            best_d = d;
        }
    };
    for (std::size_t i = 0; i < copper.elems.size(); ++i) {
        if (copper.dsu.find(static_cast<int>(i)) != root) continue;
        const Elem& e = copper.elems[i];
        if (e.kind == Elem::Kind::kTrace) {
            consider(e.seg.a, e.layer);
            consider(e.seg.b, e.layer);
        } else if (e.kind == Elem::Kind::kVia) {
            Point c = e.rect.center();
            consider(c, e.lo);
            consider(c, e.hi);
        }
    }
    if (!have) return false;
    pt_out = best;
    layer_out = best_layer;
    return true;
}

Point task_src_point(const Board& board, const ConnectionTask& task) {
    if (task.is_pair_corridor) {
        // Issue #5: when escape geometry is committed for both members of a
        // side, the corridor starts at the paired stub endpoints (midpoint of
        // the two portal ends), not at the raw pad midpoint. Falls back to
        // the pad midpoint when either side lacks escape copper.
        Point ep;
        LayerId el = 0;
        Point ep2;
        LayerId el2 = 0;
        if (pair_stub_endpoint(board, task.a, ep, el) &&
            pair_stub_endpoint(board, task.pair_a_other, ep2, el2)) {
            return {(ep.x + ep2.x) / 2, (ep.y + ep2.y) / 2};
        }
        // Issue #12: corridor sources at the P/N pad midpoint so the
        // reserved envelope covers both members symmetrically.
        const Terminal* tp = board.find_terminal(task.a);
        const Terminal* tn = board.find_terminal(task.pair_a_other);
        if (tp && tn) return {(tp->pos.x + tn->pos.x) / 2, (tp->pos.y + tn->pos.y) / 2};
        if (tp) return tp->pos;
        if (tn) return tn->pos;
        return Point{};
    }
    const Terminal* t = board.find_terminal(task.a);
    return t ? t->pos : Point{};
}

Point task_dst_point(const Board& board, const ConnectionTask& task) {
    if (task.is_pair_corridor) {
        if (task.has_plane_target) return task.plane_point;
        if (task.has_copper_target) return task.copper_point;
        // Issue #5: paired escape-stub endpoints when both members escaped.
        Point ep;
        LayerId el = 0;
        Point ep2;
        LayerId el2 = 0;
        if (pair_stub_endpoint(board, task.b, ep, el) &&
            pair_stub_endpoint(board, task.pair_b_other, ep2, el2)) {
            return {(ep.x + ep2.x) / 2, (ep.y + ep2.y) / 2};
        }
        const Terminal* tp = board.find_terminal(task.b);
        const Terminal* tn = board.find_terminal(task.pair_b_other);
        if (tp && tn) return {(tp->pos.x + tn->pos.x) / 2, (tp->pos.y + tn->pos.y) / 2};
        if (tp) return tp->pos;
        if (tn) return tn->pos;
        return Point{};
    }
    if (task.has_plane_target) return task.plane_point;
    if (task.has_copper_target) return task.copper_point;
    const Terminal* t = board.find_terminal(task.b);
    return t ? t->pos : Point{};
}

LayerId task_src_layer(const Board& board, const ConnectionTask& task) {
    if (task.is_pair_corridor) {
        // Issue #5: corridor starts on the escape layer when both member
        // stubs agree; otherwise the terminal layer.
        Point ep;
        LayerId el = 0;
        Point ep2;
        LayerId el2 = 0;
        if (pair_stub_endpoint(board, task.a, ep, el) &&
            pair_stub_endpoint(board, task.pair_a_other, ep2, el2) && el == el2)
            return el;
    }
    const Terminal* t = board.find_terminal(task.a);
    return t ? t->layer : 0;
}

LayerId task_dst_layer(const Board& board, const ConnectionTask& task) {
    if (task.is_pair_corridor) {
        if (task.has_plane_target) return task.plane_layer;
        if (task.has_copper_target) return task.copper_layer;
        Point ep;
        LayerId el = 0;
        Point ep2;
        LayerId el2 = 0;
        if (pair_stub_endpoint(board, task.b, ep, el) &&
            pair_stub_endpoint(board, task.pair_b_other, ep2, el2) && el == el2)
            return el;
        const Terminal* t = board.find_terminal(task.b);
        return t ? t->layer : 0;
    }
    if (task.has_plane_target) return task.plane_layer;
    if (task.has_copper_target) return task.copper_layer;
    const Terminal* t = board.find_terminal(task.b);
    return t ? t->layer : 0;
}

RouteTree build_route_tree(const Board& board, NetId net) {
    RouteTree tree;
    tree.net = net;
    const NetInfo* n = board.find_net(net);
    if (!n || n->terminals.size() < 2) return tree;
    std::vector<TermId> tids = n->terminals;
    std::sort(tids.begin(), tids.end());

    // Issue #16: plane-backed power/ground nets route each unconnected
    // terminal into the plane (short low-impedance access) instead of
    // running long point-to-point traces. One task per terminal outside the
    // main island network, targeting the nearest eligible plane entry;
    // split-plane islands are never bridged (each task serves its local
    // island and the verifier judges global connectivity by island).
    if (net_has_routable_planes(board, net)) {
        // One task per terminal whose copper has not yet joined any
        // same-net plane island. Terminals already on a plane (even an
        // isolated island) need no access stub; the verifier judges whether
        // all islands join into one electrical network, so wrong-island or
        // isolated contact can never falsely satisfy connectivity.
        NetCopper copper = collect_net_copper(board, net);
        std::map<TermId, int> pad_elem;
        for (std::size_t i = 0; i < copper.elems.size(); ++i) {
            if (copper.elems[i].kind == Elem::Kind::kPad)
                pad_elem[copper.elems[i].term] = (int)i;
        }
        auto on_plane = [&](TermId tid) {
            auto it = pad_elem.find(tid);
            if (it == pad_elem.end()) return false;
            int root = copper.dsu.find(it->second);
            for (std::size_t i = 0; i < copper.elems.size(); ++i) {
                if (copper.elems[i].kind == Elem::Kind::kPlane &&
                    copper.dsu.find((int)i) == root)
                    return true;
            }
            return false;
        };
        for (TermId tid : tids) {
            if (on_plane(tid)) continue;
            const Terminal* term = board.find_terminal(tid);
            if (!term) continue;
            int pid = -1, island = 0;
            Point entry{};
            LayerId elayer = 0;
            ConnectionTask task;
            task.net = net;
            task.a = tid;
            task.b = tid;  // plane task: one terminal, plane is the target
            task.index = static_cast<int>(tree.tasks.size());
            if (nearest_plane_target(board, net, term->pos, term->layer, pid, entry,
                                     elayer, island)) {
                task.has_plane_target = true;
                task.plane_id = pid;
                task.plane_point = entry;
                task.plane_layer = elayer;
                task.plane_island = island;
            }
            // Without an eligible plane the task stays target-less so the
            // failure is explicit; the pad is never silently dropped.
            tree.tasks.push_back(task);
        }
        return tree;
    }

    // Two-terminal nets: direct pad-to-pad while no same-net copper exists
    // (preserves the simple initial topology). Once committed copper exists
    // (notably Post-P4 issue #1 centre-out escape stubs pad->portal), attach
    // to the main component's copper so the global task originates at the
    // committed escape endpoint instead of generating a second raw-pad task.
    if (tids.size() == 2) {
        if (terminals_connected(board, net, tids[0], tids[1])) return tree;
        bool net_has_copper = false;
        for (const auto& t : board.traces)
            if (t.net == net) {
                net_has_copper = true;
                break;
            }
        if (!net_has_copper)
            for (const auto& v : board.vias)
                if (v.net == net) {
                    net_has_copper = true;
                    break;
                }
        if (!net_has_copper) {
            ConnectionTask task;
            task.net = net;
            task.a = tids[0];
            task.b = tids[1];
            task.index = 0;
            tree.tasks.push_back(task);
            return tree;
        }
        std::vector<std::vector<TermId>> comps2 = net_terminal_components(board, net);
        if (comps2.size() <= 1) {
            ConnectionTask task;
            task.net = net;
            task.a = tids[0];
            task.b = tids[1];
            task.index = 0;
            tree.tasks.push_back(task);
            return tree;
        }
        std::sort(comps2.begin(), comps2.end(), [](const auto& a, const auto& b) {
            if (a.size() != b.size()) return a.size() > b.size();
            return a.front() < b.front();
        });
        const std::vector<TermId>& main2 = comps2.front();
        // Two terminals => exactly one non-main singleton.
        TermId src2 = comps2.back().front();
        TermId repr2 = main2.front();
        std::vector<CopperTarget> contacts2 = copper_contacts_for(board, net, src2, main2, 1);
        ConnectionTask task;
        task.net = net;
        task.a = src2;
        task.b = repr2;
        task.index = 0;
        if (!contacts2.empty()) {
            const Terminal* rt = board.find_terminal(repr2);
            bool is_pad = rt && contacts2[0].p == rt->pos &&
                          contacts2[0].layer == rt->layer;
            bool main_has_copper = false;
            for (const auto& t : board.traces)
                if (t.net == net) {
                    main_has_copper = true;
                    break;
                }
            if (!main_has_copper)
                for (const auto& v : board.vias)
                    if (v.net == net) {
                        main_has_copper = true;
                        break;
                    }
            // No copper yet => plain pad-to-pad (no flag, preserves the
            // simple initial star). With escape copper, flag only when the
            // nearest contact leaves the representative pad (i.e. the
            // escape portal/stub endpoint).
            if (main_has_copper && !is_pad) {
                task.has_copper_target = true;
                task.copper_point = contacts2[0].p;
                task.copper_layer = contacts2[0].layer;
            }
        }
        tree.tasks.push_back(task);
        return tree;
    }

    // Multi-terminal: grow from actual committed components. One task per
    // non-main component attaching to the main (largest, tie-break smallest
    // minimum terminal id). Deterministic star that collapses to shared
    // trunks as copper lands because contacts are recomputed from geometry.
    std::vector<std::vector<TermId>> comps = net_terminal_components(board, net);
    if (comps.size() <= 1) return tree;  // already fully connected
    // Main = most terminals, tie-break smallest head terminal id.
    std::sort(comps.begin(), comps.end(), [](const auto& a, const auto& b) {
        if (a.size() != b.size()) return a.size() > b.size();
        return a.front() < b.front();
    });
    const std::vector<TermId>& main = comps.front();
    std::vector<std::vector<TermId>> others(comps.begin() + 1, comps.end());
    std::sort(others.begin(), others.end(),
              [](const auto& a, const auto& b) { return a.front() < b.front(); });
    for (const auto& grp : others) {
        TermId src = grp.front();  // stable: smallest in component
        TermId repr = main.front();
        // Nearest contact on the main component's copper to the source.
        std::vector<CopperTarget> contacts = copper_contacts_for(board, net, src, main, 1);
        ConnectionTask task;
        task.net = net;
        task.a = src;
        task.b = repr;
        if (!contacts.empty()) {
            // Attach to copper when there is committed copper to share;
            // with no copper yet contacts[0] is the representative pad
            // itself, which degenerates to terminal-to-terminal.
            Point mp = contacts[0].p;
            const Terminal* rt = board.find_terminal(repr);
            bool is_pad = rt && mp == rt->pos && contacts[0].layer == rt->layer;
            // Only flag copper targets when they are NOT exactly the
            // representative pad: that preserves the simple initial star
            // while enabling mid-copper T-junctions once trunks exist.
            // Heuristic: if main has any committed trace/via, any contact
            // (including pad-adjacent) is a genuine tree attach.
            bool main_has_copper = false;
            for (const auto& t : board.traces)
                if (t.net == net) {
                    main_has_copper = true;
                    break;
                }
            if (!main_has_copper)
                for (const auto& v : board.vias)
                    if (v.net == net) {
                        main_has_copper = true;
                        break;
                    }
            if (main_has_copper && !(mp == task_src_point(board, task))) {
                task.has_copper_target = true;
                task.copper_point = mp;
                task.copper_layer = contacts[0].layer;
            } else if (!is_pad) {
                task.has_copper_target = true;
                task.copper_point = mp;
                task.copper_layer = contacts[0].layer;
            }
        }
        task.index = static_cast<int>(tree.tasks.size());
        tree.tasks.push_back(task);
    }
    return tree;
}

double task_difficulty(const Board& board, const RuleResolver& resolver,
                       const ConnectionTask& task, const ElectricalContext& ctx,
                       const std::vector<double>& terminal_density) {
    const Terminal* ta = board.find_terminal(task.a);
    if (!ta) return 0.0;
    Point dst = task_dst_point(board, task);
    const NetInfo* net = board.find_net(task.net);

    double span_mm = nm_to_mm(manhattan(ta->pos, dst));
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
        if (a.b != b.b) return a.b < b.b;
        // Issue #12: pair corridors order deterministically after plain
        // tasks on the same (net, a, b) key.
        if (a.is_pair_corridor != b.is_pair_corridor) return a.is_pair_corridor < b.is_pair_corridor;
        if (a.pair_id != b.pair_id) return a.pair_id < b.pair_id;
        if (a.pair_other_net != b.pair_other_net) return a.pair_other_net < b.pair_other_net;
        if (a.pair_a_other != b.pair_a_other) return a.pair_a_other < b.pair_a_other;
        if (a.pair_b_other != b.pair_b_other) return a.pair_b_other < b.pair_b_other;
        if (a.has_copper_target != b.has_copper_target) return a.has_copper_target < b.has_copper_target;
        if (a.copper_point.x != b.copper_point.x) return a.copper_point.x < b.copper_point.x;
        if (a.copper_point.y != b.copper_point.y) return a.copper_point.y < b.copper_point.y;
        if (a.copper_layer != b.copper_layer) return a.copper_layer < b.copper_layer;
        // Issue #16: plane targets order after copper targets, by island,
        // plane, entry point, layer (all deterministic).
        if (a.has_plane_target != b.has_plane_target) return a.has_plane_target < b.has_plane_target;
        if (a.plane_island != b.plane_island) return a.plane_island < b.plane_island;
        if (a.plane_id != b.plane_id) return a.plane_id < b.plane_id;
        if (a.plane_point.x != b.plane_point.x) return a.plane_point.x < b.plane_point.x;
        if (a.plane_point.y != b.plane_point.y) return a.plane_point.y < b.plane_point.y;
        return a.plane_layer < b.plane_layer;
    });
}

}  // namespace copperline
