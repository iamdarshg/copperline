#include "router/sparse_graph.h"

#include <algorithm>
#include <map>
#include <string>

#include "router/via_bundle.h"

namespace copperline {

int direction_of(Point from, Point to) {
    if (to.x > from.x) return 0;
    if (to.y > from.y) return 1;
    if (to.x < from.x) return 2;
    if (to.y < from.y) return 3;
    return 4;
}

namespace {

// Raw (unexpanded) obstacle with the exact centerline keep-distance.
struct Obstacle {
    Rect raw{};
    LayerId layer = kAllLayers;
    Coord dist_min_nm = 0;  // required centerline distance from raw copper
    // Issue #21: identity for frontier-rejection evidence.
    NetId net = -1;      // -1 = keepout (nothing to rip)
    std::string kind;    // "keepout" | "pad" | "trace" | "via"
    std::string desc;    // stable human/agent-readable label
};

bool layer_match(LayerId obstacle_layer, LayerId query_layer) {
    return obstacle_layer == kAllLayers || obstacle_layer == query_layer;
}

// Exact legality: squared centerline distance >= dist_min^2. Fast path: when
// the segment bbox expanded by dist_min does not even touch the raw rect,
// the exact distance must exceed dist_min (closed-interval touch would still
// be caught, so falling through on touch keeps the predicate exact).
bool seg_legal_vs(const Segment& s, const Obstacle& o) {
    if (!s.bounds().expanded(o.dist_min_nm).intersects(o.raw)) return true;
    __int128 d2 = seg_rect_dist2(s, o.raw);
    __int128 need = (__int128)o.dist_min_nm * o.dist_min_nm;
    return d2 >= need;
}

bool seg_in_bounds(const Segment& s, const Rect& bounds, Coord half_width) {
    Rect r = s.bounds().expanded(half_width);
    return bounds.contains(r);
}

}  // namespace

void SparseRoutingGraph::add_penalties(
    const std::function<Coord(const SparseNode&, const SparseEdge&)>& fn) {
    for (std::size_t i = 0; i < nodes_.size(); ++i)
        for (auto& e : adj_[i]) e.penalty_nm += fn(nodes_[i], e);
}

SparseRoutingGraph SparseRoutingGraph::build(const Board& committed, const RuleResolver& resolver,
                                              NetId net, Point src, Point dst, LayerId src_layer,
                                              LayerId dst_layer, Coord route_width_nm,
                                              const ElectricalContext& ctx) {
    SparseRoutingGraph g;
    const Coord half_w = route_width_nm / 2;

    // ---- 1. Collect raw obstacles with per-obstacle keep distances ----
    // Clearance is layer-independent (RuleResolver ignores layer/ctx), so one
    // lookup per foreign net is exact and avoids repeat linear scans.
    std::map<NetId, Coord> clear_cache;
    auto clearance_to = [&](NetId other, LayerId layer) -> Coord {
        auto it = clear_cache.find(other);
        if (it != clear_cache.end()) return it->second;
        std::string cs;
        Coord c = resolver.requiredClearance(net, other, layer, ctx, &cs);
        clear_cache[other] = c;
        return c;
    };
    Coord max_clear = 0;
    for (const auto& other : committed.nets) {
        if (other.id == net) continue;
        max_clear = std::max(max_clear, clearance_to(other.id, 0));
    }
    std::vector<Obstacle> obstacles;
    obstacles.reserve(committed.keepouts.size() + committed.terminals.size() +
                      committed.traces.size() + committed.vias.size());
    auto push_obstacle = [&](Rect raw, LayerId layer, Coord dist_min, NetId onet,
                             std::string kind, std::string desc) {
        if (raw.x2 < raw.x1 || raw.y2 < raw.y1) return;
        obstacles.push_back({raw, layer, dist_min, onet, std::move(kind), std::move(desc)});
    };
    for (const auto& ko : committed.keepouts) {
        push_obstacle(ko.rect, ko.layer, max_clear + half_w, -1, "keepout",
                      "keepout:" + ko.reason);
    }
    for (const auto& t : committed.terminals) {
        if (t.net == net) continue;  // own copper is connectable, not an obstacle
        const NetInfo* on = committed.find_net(t.net);
        std::string nm = on ? on->name : "?";
        std::string desc = "pad:net=" + nm +
                           (t.component.empty() ? "" : ":" + t.component + "." + t.pin);
        push_obstacle(t.pad_rect(), t.layer, clearance_to(t.net, t.layer) + half_w, t.net,
                      "pad", std::move(desc));
    }
    for (const auto& t : committed.traces) {
        if (t.net == net) continue;
        const NetInfo* on = committed.find_net(t.net);
        std::string nm = on ? on->name : "?";
        Rect raw = t.segment().bounds().expanded(t.width_nm / 2);
        push_obstacle(raw, t.layer, clearance_to(t.net, t.layer) + half_w, t.net, "trace",
                      "trace:net=" + nm);
    }
    for (const auto& v : committed.vias) {
        if (v.net == net) continue;
        const NetInfo* on = committed.find_net(v.net);
        std::string nm = on ? on->name : "?";
        Coord c = clearance_to(v.net, v.top_layer);
        Rect raw = Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm);
        for (const auto& l : committed.layers) {
            bool in_span = (l.id >= std::min(v.top_layer, v.bottom_layer) &&
                            l.id <= std::max(v.top_layer, v.bottom_layer));
            if (in_span)
                push_obstacle(raw, l.id, c + half_w, v.net, "via", "via:net=" + nm);
        }
    }
    g.stats_.obstacle_count = static_cast<int>(obstacles.size());

    // Issue #21: bounded frontier-rejection aggregation. Keyed by
    // (blocker net, kind, layer) so memory stays bounded no matter how many
    // candidate edges are attempted; the representative position is the first
    // rejection site (deterministic: graph build order is deterministic).
    struct RejAgg {
        NetId net = -1;
        std::string kind;
        std::string desc;
        LayerId layer = 0;
        Point pos{};
        int count = 0;
    };
    std::map<std::string, RejAgg> rej;
    auto rej_key = [](NetId n, const std::string& k, LayerId l) {
        return std::to_string(n) + "|" + k + "|" + std::to_string(l);
    };
    auto note_rejection = [&](NetId bn, const std::string& kind, const std::string& desc,
                              LayerId layer, Point rep) {
        std::string key = rej_key(bn, kind, layer);
        auto it = rej.find(key);
        if (it == rej.end()) {
            RejAgg a;
            a.net = bn;
            a.kind = kind;
            a.desc = desc;
            a.layer = layer;
            a.pos = rep;
            a.count = 1;
            rej.emplace(key, std::move(a));
        } else {
            it->second.count++;
        }
    };

    auto first_blocker = [&](const Segment& s, LayerId layer) -> const Obstacle* {
        for (const auto& o : obstacles) {
            if (!layer_match(o.layer, layer)) continue;
            if (!seg_legal_vs(s, o)) return &o;
        }
        return nullptr;
    };

    auto seg_legal = [&](const Segment& s, LayerId layer) -> bool {
        if (!seg_in_bounds(s, committed.bounds(), half_w)) return false;
        return first_blocker(s, layer) == nullptr;
    };
    // Recording variant: on rejection, attributes the actual blocker (or
    // bounds) with a representative position. Used for every legality probe
    // during graph construction so the evidence reflects what A* could not
    // traverse -- not a rectangular corridor guess.
    auto seg_legal_record = [&](const Segment& s, LayerId layer, Point rep) -> bool {
        if (!seg_in_bounds(s, committed.bounds(), half_w)) {
            note_rejection(-1, "bounds", "off_board", layer, rep);
            return false;
        }
        if (const Obstacle* o = first_blocker(s, layer)) {
            note_rejection(o->net, o->kind, o->desc, layer, rep);
            return false;
        }
        return true;
    };
    (void)seg_legal;

    // ---- 2. Candidate base points ----
    // Node-count optimisation: expanded corners are only emitted for obstacles
    // near the task corridor (generous margin, so detours survive). ALL
    // obstacles still participate in legality, so no illegal edge can ever be
    // built; at worst a far detour is missed and the arbiter reports it.
    const Coord span = manhattan(src, dst);
    const Coord margin = std::max(mm_to_nm(6.0), span / 2);
    const Rect corridor = Rect::from_points(src, dst).expanded(margin);
    std::vector<Point> bases;
    bases.reserve(2 + obstacles.size());
    bases.push_back(src);
    bases.push_back(dst);
    for (const auto& o : obstacles) {
        if (!o.raw.expanded(o.dist_min_nm).intersects(corridor)) continue;
        Rect exp = o.raw.expanded(o.dist_min_nm);
        Point corners[4] = {{exp.x1, exp.y1}, {exp.x2, exp.y1}, {exp.x2, exp.y2}, {exp.x1, exp.y2}};
        for (auto c : corners) bases.push_back(c);
    }
    // Board corners (inset by half width) give border-hugging corridors.
    Rect inner = committed.bounds().expanded(-half_w);
    if (inner.x2 >= inner.x1 && inner.y2 >= inner.y1) {
        bases.push_back({inner.x1, inner.y1});
        bases.push_back({inner.x2, inner.y1});
        bases.push_back({inner.x2, inner.y2});
        bases.push_back({inner.x1, inner.y2});
    }
    std::sort(bases.begin(), bases.end());
    bases.erase(std::unique(bases.begin(), bases.end(),
                            [](const Point& a, const Point& b) { return a.x == b.x && a.y == b.y; }),
                bases.end());
    // Deterministic safety cap: keep src/dst plus the bases closest to the
    // task segment (distance, then x, then y tie-breaks).
    constexpr std::size_t kMaxBases = 384;
    if (bases.size() > kMaxBases) {
        Segment task_seg{src, dst};
        std::vector<std::pair<__int128, Point>> ranked;
        ranked.reserve(bases.size());
        for (auto p : bases) {
            if (p == src || p == dst) continue;
            ranked.push_back({seg_rect_dist2(task_seg, Rect{p.x, p.y, p.x, p.y}), p});
        }
        std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
            if (a.first != b.first) return a.first < b.first;
            if (a.second.x != b.second.x) return a.second.x < b.second.x;
            return a.second.y < b.second.y;
        });
        bases.clear();
        bases.push_back(src);
        bases.push_back(dst);
        for (std::size_t i = 0; i < ranked.size() && bases.size() < kMaxBases; ++i)
            bases.push_back(ranked[i].second);
        std::sort(bases.begin(), bases.end());
    }
    // Clamp into the legal centerline region.
    std::vector<Point> clipped;
    clipped.reserve(bases.size());
    for (auto p : bases) {
        if (!inner.contains(p)) continue;
        clipped.push_back(p);
    }
    bases = std::move(clipped);

    // ---- 3. Nodes: layer copies, filtered to legal wire positions ----
    // base index -> per-layer node id
    std::map<int, std::map<LayerId, int>> node_of;
    for (std::size_t bi = 0; bi < bases.size(); ++bi) {
        for (const auto& l : committed.layers) {
            // A wire point is legal when a zero-length segment there is legal.
            Segment pt{bases[bi], bases[bi]};
            if (!seg_legal_record(pt, l.id, bases[bi])) continue;
            // Always keep src/dst on their own layers even if the pad interior
            // is marked (endpoint copper belongs to us and was excluded above,
            // so this only triggers on genuine foreign overlap -> then the
            // task is truly blocked; keeping the node lets A* report it).
            SparseNode n;
            n.p = bases[bi];
            n.layer = l.id;
            n.base = static_cast<int>(bi);
            int id = static_cast<int>(g.nodes_.size());
            g.nodes_.push_back(n);
            node_of[static_cast<int>(bi)][l.id] = id;
        }
    }
    // Guarantee src/dst nodes exist on their layers (even if illegal, so the
    // failure is explicit rather than a missing-node internal error).
    auto ensure_node = [&](Point p, LayerId layer) -> int {
        int bi = -1;
        for (std::size_t i = 0; i < bases.size(); ++i) {
            if (bases[i].x == p.x && bases[i].y == p.y) {
                bi = static_cast<int>(i);
                break;
            }
        }
        if (bi < 0) return -1;
        auto it = node_of.find(bi);
        if (it != node_of.end()) {
            auto jt = it->second.find(layer);
            if (jt != it->second.end()) return jt->second;
        }
        SparseNode n;
        n.p = p;
        n.layer = layer;
        n.base = bi;
        int id = static_cast<int>(g.nodes_.size());
        g.nodes_.push_back(n);
        node_of[bi][layer] = id;
        return id;
    };
    g.src_node_ = ensure_node(src, src_layer);
    g.dst_node_ = ensure_node(dst, dst_layer);
    g.adj_.resize(g.nodes_.size());

    // Per-layer node lists: edge search never scans other layers.
    std::map<LayerId, std::vector<int>> layer_nodes;
    for (int i = 0; i < static_cast<int>(g.nodes_.size()); ++i)
        layer_nodes[g.nodes_[i].layer].push_back(i);

    // ---- 4. Manhattan edges: aligned pairs + K nearest per node ----
    constexpr int kNearest = 16;
    auto try_edge = [&](int from, int to) {
        if (from == to) return;
        const Point a = g.nodes_[from].p;
        const Point b = g.nodes_[to].p;
        if (g.nodes_[from].layer != g.nodes_[to].layer) return;
        LayerId layer = g.nodes_[from].layer;
        // Candidate paths: straight (if aligned) else both elbow orders.
        struct Cand {
            Point elbow;
            bool has_elbow;
        };
        std::vector<Cand> cands;
        if (a.x == b.x || a.y == b.y) {
            cands.push_back({{}, false});
        } else {
            cands.push_back({{b.x, a.y}, true});
            cands.push_back({{a.x, b.y}, true});
        }
        for (const auto& c : cands) {
            Segment s1{a, c.has_elbow ? c.elbow : b};
            Segment s2{c.has_elbow ? Segment{c.elbow, b} : Segment{b, b}};
            bool s2_empty = !c.has_elbow;
            Point rep1{(a.x + (c.has_elbow ? c.elbow.x : b.x)) / 2,
                       (a.y + (c.has_elbow ? c.elbow.y : b.y)) / 2};
            if (!seg_legal_record(s1, layer, rep1)) continue;
            if (!s2_empty) {
                Point rep2{(c.elbow.x + b.x) / 2, (c.elbow.y + b.y) / 2};
                if (!seg_legal_record(s2, layer, rep2)) continue;
            }
            SparseEdge e;
            e.to = to;
            e.len_nm = manhattan(a, c.has_elbow ? c.elbow : b) +
                       (c.has_elbow ? manhattan(c.elbow, b) : 0);
            e.dir1 = direction_of(a, c.has_elbow ? c.elbow : b);
            e.dir2 = c.has_elbow ? direction_of(c.elbow, b) : -1;
            e.elbow = c.elbow;
            e.is_via = false;
            g.adj_[from].push_back(e);
            break;  // first legal elbow order wins (deterministic)
        }
    };
    for (const auto& [layer, ids] : layer_nodes) {
        // Aligned pairs (shared x or y) preserve corridors at any distance.
        std::map<Coord, std::vector<int>> by_x, by_y;
        for (int i : ids) {
            by_x[g.nodes_[i].p.x].push_back(i);
            by_y[g.nodes_[i].p.y].push_back(i);
        }
        for (int i : ids) {
            for (int j : by_x[g.nodes_[i].p.x]) try_edge(i, j);
            for (int j : by_y[g.nodes_[i].p.y]) try_edge(i, j);
        }
        // K nearest on the same layer (deterministic: full sort by
        // (distance, node id); lists are small after corridor clipping).
        for (int i : ids) {
            std::vector<std::pair<Coord, int>> near;
            near.reserve(ids.size());
            for (int j : ids) {
                if (j == i) continue;
                near.push_back({manhattan(g.nodes_[i].p, g.nodes_[j].p), j});
            }
            std::sort(near.begin(), near.end());
            for (int k = 0; k < static_cast<int>(near.size()) && k < kNearest; ++k) {
                try_edge(i, near[k].second);
            }
        }
    }

    // ---- 5. Via edges between layer copies of the same base ----
    // Bundle-aware gate (issue #5): a transition exists only when the full
    // parallel-via bundle for this net fits legally around the base point
    // (all barrels + star stubs clear foreign copper under pair clearance).
    // The planner is deterministic, so candidate materialization replays the
    // identical bundle for the committed edge.
    if (!committed.layers.empty()) {
        for (const auto& [bi, per_layer] : node_of) {
            std::vector<std::pair<LayerId, int>> copies(per_layer.begin(), per_layer.end());
            for (std::size_t i = 0; i < copies.size(); ++i) {
                for (std::size_t j = 0; j < copies.size(); ++j) {
                    if (i == j) continue;
                    Point p = g.nodes_[copies[i].second].p;
                    LayerSpan span{copies[i].first, copies[j].first};
                    ViaBundle bundle = ViaBundlePlanner::plan(
                        committed, resolver, net, p, span, route_width_nm, ctx);
                    if (!bundle.feasible) continue;
                    SparseEdge e;
                    e.to = copies[j].second;
                    e.len_nm = 0;
                    e.dir1 = 4;
                    e.dir2 = -1;
                    e.is_via = true;
                    g.adj_[copies[i].second].push_back(e);
                }
            }
        }
    }

    // Deterministic edge order.
    for (auto& vec : g.adj_) {
        std::sort(vec.begin(), vec.end(), [](const SparseEdge& a, const SparseEdge& b) {
            if (a.is_via != b.is_via) return a.is_via < b.is_via;
            if (a.to != b.to) return a.to < b.to;
            return a.len_nm < b.len_nm;
        });
    }
    int edges = 0;
    for (const auto& vec : g.adj_) edges += static_cast<int>(vec.size());
    g.stats_.node_count = static_cast<int>(g.nodes_.size());
    g.stats_.edge_count = edges;
    // Issue #21: finalize bounded frontier evidence (count desc, then stable
    // net/kind/layer/desc order). Deterministic across runs and threads.
    {
        std::vector<RejAgg> all;
        all.reserve(rej.size());
        for (auto& [k, v] : rej) all.push_back(v);
        std::sort(all.begin(), all.end(), [](const RejAgg& a, const RejAgg& b) {
            if (a.count != b.count) return a.count > b.count;
            if (a.net != b.net) return a.net < b.net;
            if (a.kind != b.kind) return a.kind < b.kind;
            if (a.layer != b.layer) return a.layer < b.layer;
            return a.desc < b.desc;
        });
        for (std::size_t i = 0; i < all.size() && (int)g.frontier_stats_.size() < kMaxFrontierStats;
             ++i) {
            GraphFrontierStat s;
            s.blocker_net = all[i].net;
            s.kind = all[i].kind;
            s.desc = all[i].desc;
            s.layer = all[i].layer;
            s.pos = all[i].pos;
            s.count = all[i].count;
            g.frontier_stats_.push_back(s);
        }
    }
    return g;
}

}  // namespace copperline
