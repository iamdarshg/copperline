#include "router/sparse_graph.h"

#include <algorithm>
#include <map>

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
};

std::string net_name(const Board& b, NetId net) {
    const NetInfo* n = b.find_net(net);
    return n ? n->name : ("#" + std::to_string(net));
}

bool layer_match(LayerId obstacle_layer, LayerId query_layer) {
    return obstacle_layer == kAllLayers || obstacle_layer == query_layer;
}

// Exact legality: squared centerline distance >= dist_min^2.
bool seg_legal_vs(const Segment& s, const Obstacle& o) {
    __int128 d2 = seg_rect_dist2(s, o.raw);
    __int128 need = (__int128)o.dist_min_nm * o.dist_min_nm;
    return d2 >= need;
}

bool seg_in_bounds(const Segment& s, const Rect& bounds, Coord half_width) {
    Rect r = s.bounds().expanded(half_width);
    return bounds.contains(r);
}

}  // namespace

SparseRoutingGraph SparseRoutingGraph::build(const Board& committed, const RuleResolver& resolver,
                                             NetId net, Point src, Point dst, LayerId src_layer,
                                             LayerId dst_layer, Coord route_width_nm,
                                             const ElectricalContext& ctx) {
    SparseRoutingGraph g;
    const Coord half_w = route_width_nm / 2;

    // ---- 1. Collect raw obstacles with per-obstacle keep distances ----
    std::vector<Obstacle> obstacles;
    // Max clearance of this net vs any other net: used for keepouts (no net).
    Coord max_clear = 0;
    for (const auto& other : committed.nets) {
        if (other.id == net) continue;
        std::string cs;
        max_clear = std::max(max_clear, resolver.requiredClearance(net, other.id, 0, ctx, &cs));
    }
    auto push_obstacle = [&](Rect raw, LayerId layer, Coord dist_min) {
        if (raw.x2 < raw.x1 || raw.y2 < raw.y1) return;
        obstacles.push_back({raw, layer, dist_min});
    };
    for (const auto& ko : committed.keepouts) {
        push_obstacle(ko.rect, ko.layer, max_clear + half_w);
    }
    for (const auto& t : committed.terminals) {
        if (t.net == net) continue;  // own copper is connectable, not an obstacle
        std::string cs;
        Coord c = resolver.requiredClearance(net, t.net, t.layer, ctx, &cs);
        push_obstacle(t.pad_rect(), t.layer, c + half_w);
    }
    for (const auto& t : committed.traces) {
        if (t.net == net) continue;
        std::string cs;
        Coord c = resolver.requiredClearance(net, t.net, t.layer, ctx, &cs);
        Rect raw = t.segment().bounds().expanded(t.width_nm / 2);
        push_obstacle(raw, t.layer, c + half_w);
    }
    for (const auto& v : committed.vias) {
        if (v.net == net) continue;
        std::string cs;
        Coord c = resolver.requiredClearance(net, v.net, v.top_layer, ctx, &cs);
        Rect raw = Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm);
        for (const auto& l : committed.layers) {
            bool in_span = (l.id >= std::min(v.top_layer, v.bottom_layer) &&
                            l.id <= std::max(v.top_layer, v.bottom_layer));
            if (in_span) push_obstacle(raw, l.id, c + half_w);
        }
    }
    (void)net_name;
    g.stats_.obstacle_count = static_cast<int>(obstacles.size());

    auto seg_legal = [&](const Segment& s, LayerId layer) -> bool {
        if (!seg_in_bounds(s, committed.bounds(), half_w)) return false;
        for (const auto& o : obstacles) {
            if (!layer_match(o.layer, layer)) continue;
            if (!seg_legal_vs(s, o)) return false;
        }
        return true;
    };

    // ---- 2. Candidate base points ----
    std::vector<Point> bases;
    bases.push_back(src);
    bases.push_back(dst);
    for (const auto& o : obstacles) {
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
    // Clamp into the legal centerline region.
    std::vector<Point> clipped;
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
            if (!seg_legal(pt, l.id)) continue;
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

    // ---- 4. Manhattan edges: aligned pairs + K nearest per node ----
    constexpr int kNearest = 24;
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
            if (!seg_legal(s1, layer)) continue;
            if (!s2_empty && !seg_legal(s2, layer)) continue;
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
    int ncount = static_cast<int>(g.nodes_.size());
    // Aligned pairs (shared x or y) preserve corridors at any distance.
    std::map<Coord, std::vector<int>> by_x, by_y;
    for (int i = 0; i < ncount; ++i) {
        by_x[g.nodes_[i].p.x].push_back(i);
        by_y[g.nodes_[i].p.y].push_back(i);
    }
    for (int i = 0; i < ncount; ++i) {
        for (int j : by_x[g.nodes_[i].p.x]) try_edge(i, j);
        for (int j : by_y[g.nodes_[i].p.y]) try_edge(i, j);
        // K nearest on the same layer.
        std::vector<std::pair<Coord, int>> near;
        for (int j = 0; j < ncount; ++j) {
            if (j == i || g.nodes_[j].layer != g.nodes_[i].layer) continue;
            near.push_back({manhattan(g.nodes_[i].p, g.nodes_[j].p), j});
        }
        std::sort(near.begin(), near.end());
        for (int k = 0; k < static_cast<int>(near.size()) && k < kNearest; ++k) {
            try_edge(i, near[k].second);
        }
    }

    // ---- 5. Via edges between layer copies of the same base ----
    ViaStyle style;
    LayerSpan full{committed.layers.front().id, committed.layers.back().id};
    bool have_via = resolver.select_via(net, full, style);
    if (have_via) {
        for (const auto& [bi, per_layer] : node_of) {
            std::vector<std::pair<LayerId, int>> copies(per_layer.begin(), per_layer.end());
            for (std::size_t i = 0; i < copies.size(); ++i) {
                for (std::size_t j = 0; j < copies.size(); ++j) {
                    if (i == j) continue;
                    Point p = g.nodes_[copies[i].second].p;
                    // Via disc must clear foreign copper on every spanned layer.
                    Rect via_rect =
                        Rect::from_center_size(p, style.outer_nm, style.outer_nm);
                    bool ok = true;
                    Rect bnds = committed.bounds();
                    if (!bnds.contains(via_rect)) {
                        ok = false;
                    } else {
                        LayerId lo = std::min(copies[i].first, copies[j].first);
                        LayerId hi = std::max(copies[i].first, copies[j].first);
                        for (const auto& o : obstacles) {
                            if (o.layer != kAllLayers && (o.layer < lo || o.layer > hi)) continue;
                            if (rect_gap(via_rect, o.raw) < o.dist_min_nm - half_w) {
                                ok = false;
                                break;
                            }
                        }
                    }
                    if (!ok) continue;
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
    g.stats_.node_count = ncount;
    g.stats_.edge_count = edges;
    return g;
}

}  // namespace copperline
