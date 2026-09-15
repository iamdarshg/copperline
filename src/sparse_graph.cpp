#include "router/sparse_graph.h"

#include <algorithm>
#include <limits>
#include <map>
#include <string>
#include <tuple>

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
    std::string kind;    // "keepout" | "pad" | "trace" | "via" | "plane"
    std::string desc;    // stable human/agent-readable label
    // Issue #16: foreign planes keep exact polygon geometry (raw is the
    // bbox, used only for corridor pre-checks); legality uses the polygon.
    const std::vector<Point>* poly = nullptr;
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
    __int128 d2;
    if (o.poly) {
        // Issue #16: exact centerline distance to the pour polygon, not its
        // bbox, so legal corridors beside plane corners survive.
        d2 = plane_seg_poly_dist2(s, *o.poly);
    } else {
        d2 = seg_rect_dist2(s, o.raw);
    }
    __int128 need = (__int128)o.dist_min_nm * o.dist_min_nm;
    return d2 >= need;
}

bool seg_in_bounds(const Segment& s, const Rect& bounds, Coord half_width) {
    Rect r = s.bounds().expanded(half_width);
    return bounds.contains(r);
}

// S2: uniform bucket index over the per-task expanded obstacles. An obstacle
// can only fail seg_legal_vs(s) when s.bounds() touches
// raw.expanded(dist_min) (box-expansion symmetry with the bbox precheck), so
// filing each obstacle under raw.expanded(dist_min) and querying with the
// bare segment bbox prunes exactly the obstacles the precheck would pass.
// Exact predicates still decide everything; callers re-scan candidates in
// original obstacle order, so first-blocker identity and rejected-probe
// evidence are bit-identical to the linear scan. Buckets own their member
// lists outright: a shared head/next chain cannot work here because one
// obstacle spans many buckets and a single next[] link per obstacle would
// orphan earlier buckets' chains on every re-file.
struct ObstacleIndex {
    Coord bucket = 1000000;  // 1mm
    int nx = 0, ny = 0;
    Rect bounds{};
    std::vector<std::vector<int>> cells;  // nx*ny buckets, filing order

    template <typename Obs>
    void build(const std::vector<Obs>& obs, const Rect& b) {
        bounds = b;
        Coord w = std::max<Coord>(1, b.width());
        Coord h = std::max<Coord>(1, b.height());
        nx = std::max(1, static_cast<int>((w + bucket - 1) / bucket));
        ny = std::max(1, static_cast<int>((h + bucket - 1) / bucket));
        cells.assign(static_cast<std::size_t>(nx) * ny, {});
        for (std::size_t oi = 0; oi < obs.size(); ++oi) {
            Rect r = obs[oi].raw.expanded(obs[oi].dist_min_nm);
            int ix0 = std::clamp(static_cast<int>((r.x1 - b.x1) / bucket), 0, nx - 1);
            int ix1 = std::clamp(static_cast<int>((r.x2 - b.x1) / bucket), 0, nx - 1);
            int iy0 = std::clamp(static_cast<int>((r.y1 - b.y1) / bucket), 0, ny - 1);
            int iy1 = std::clamp(static_cast<int>((r.y2 - b.y1) / bucket), 0, ny - 1);
            for (int iy = iy0; iy <= iy1; ++iy)
                for (int ix = ix0; ix <= ix1; ++ix)
                    cells[static_cast<std::size_t>(iy) * nx + ix].push_back(
                        static_cast<int>(oi));
        }
    }

    // Collects candidate obstacle indices overlapped by q (duplicates
    // possible; the caller sorts/uniques into original order).
    void query(const Rect& q, std::vector<int>& out) const {
        if (nx <= 0 || ny <= 0) return;
        int ix0 = std::clamp(static_cast<int>((q.x1 - bounds.x1) / bucket), 0, nx - 1);
        int ix1 = std::clamp(static_cast<int>((q.x2 - bounds.x1) / bucket), 0, nx - 1);
        int iy0 = std::clamp(static_cast<int>((q.y1 - bounds.y1) / bucket), 0, ny - 1);
        int iy1 = std::clamp(static_cast<int>((q.y2 - bounds.y1) / bucket), 0, ny - 1);
        for (int iy = iy0; iy <= iy1; ++iy)
            for (int ix = ix0; ix <= ix1; ++ix) {
                const std::vector<int>& cell =
                    cells[static_cast<std::size_t>(iy) * nx + ix];
                out.insert(out.end(), cell.begin(), cell.end());
            }
    }
};

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
    SparseTarget single{dst, dst_layer};
    return build_multi(committed, resolver, net, src, src_layer, {single}, route_width_nm,
                       ctx);
}

SparseRoutingGraph SparseRoutingGraph::build_multi(
    const Board& committed, const RuleResolver& resolver, NetId net, Point src,
    LayerId src_layer, const std::vector<SparseTarget>& dsts, Coord route_width_nm,
    const ElectricalContext& ctx, const std::vector<Point>* clip_path,
    Coord clip_half_width_nm) {
    SparseRoutingGraph g;
    const Coord half_w = route_width_nm / 2;
    (void)dsts.empty();

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
                             std::string kind, std::string desc,
                             const std::vector<Point>* poly = nullptr) {
        if (raw.x2 < raw.x1 || raw.y2 < raw.y1) return;
        obstacles.push_back(
            {raw, layer, dist_min, onet, std::move(kind), std::move(desc), poly});
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
    // Issue #16: foreign pours are obstacles with exact polygon legality;
    // own-net pours are connectable copper, never obstacles.
    for (const auto& z : committed.planes) {
        if (z.net == net) continue;
        const NetInfo* on = committed.find_net(z.net);
        std::string nm = on ? on->name : "?";
        push_obstacle(z.bounds(), z.layer, clearance_to(z.net, z.layer) + half_w, z.net,
                      "plane", "plane:net=" + nm + "#" + std::to_string(z.id), &z.poly);
    }
    g.stats_.obstacle_count = static_cast<int>(obstacles.size());

    // Issue #23: no global construction-time aggregation here. Rejected
    // candidate transitions are stored per source node (see rej_per_node
    // below) and only accumulated for nodes A* actually expands. Aggregating
    // every probe globally is what biased dependency/rip-up ranking toward
    // copper in regions the search never reached (#21 regression).
    // S2: the linear scan is served by the obstacle index. Candidates are
    // re-scanned in original obstacle order, so the returned first blocker
    // (and every recorded rejected probe) is identical to the linear scan;
    // the index only skips obstacles the bbox precheck would have passed.
    ObstacleIndex obs_index;
    obs_index.build(obstacles, committed.bounds());
    std::vector<int> scratch;  // per-build scratch (no sharing across threads:
                               // every build_multi call owns its locals)
    auto first_blocker = [&](const Segment& s, LayerId layer) -> const Obstacle* {
        scratch.clear();
        obs_index.query(s.bounds(), scratch);
        std::sort(scratch.begin(), scratch.end());
        scratch.erase(std::unique(scratch.begin(), scratch.end()), scratch.end());
        for (int oi : scratch) {
            const Obstacle& o = obstacles[static_cast<std::size_t>(oi)];
            if (!layer_match(o.layer, layer)) continue;
            if (!seg_legal_vs(s, o)) return &o;
        }
        return nullptr;
    };

    auto seg_legal = [&](const Segment& s, LayerId layer) -> bool {
        if (!seg_in_bounds(s, committed.bounds(), half_w)) return false;
        return first_blocker(s, layer) == nullptr;
    };

    // ---- 2. Candidate base points ----
    // Node-count optimisation: expanded corners are only emitted for obstacles
    // near the task corridor (generous margin, so detours survive). ALL
    // obstacles still participate in legality, so no illegal edge can ever be
    // built; at worst a far detour is missed and the arbiter reports it.
    // Issue #4: corridor covers src + every dst contact so T-junction
    // targets all survive clipping.
    Coord span = 0;
    Rect corridor{src.x, src.y, src.x, src.y};
    for (const auto& d : dsts) {
        span = std::max(span, manhattan(src, d.p));
        corridor = {std::min(corridor.x1, d.p.x), std::min(corridor.y1, d.p.y),
                    std::max(corridor.x2, d.p.x), std::max(corridor.y2, d.p.y)};
    }
    const Coord margin = std::max(mm_to_nm(6.0), span / 2);
    corridor = corridor.expanded(margin);
    std::vector<Point> bases;
    bases.reserve(2 + obstacles.size());
    bases.push_back(src);
    for (const auto& d : dsts) bases.push_back(d.p);
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
    // Deterministic safety cap: keep src/all-dsts plus the bases closest
    // to any task segment src->dst_i (distance, then x, then y tie-breaks).
    constexpr std::size_t kMaxBases = 384;
    if (bases.size() > kMaxBases) {
        std::vector<Segment> task_segs;
        for (const auto& d : dsts) task_segs.push_back({src, d.p});
        if (task_segs.empty()) task_segs.push_back({src, src});
        auto is_protected = [&](Point p) {
            if (p == src) return true;
            for (const auto& d : dsts)
                if (p == d.p) return true;
            return false;
        };
        std::vector<std::pair<__int128, Point>> ranked;
        ranked.reserve(bases.size());
        for (auto p : bases) {
            if (is_protected(p)) continue;
            __int128 best = -1;
            for (const auto& sg : task_segs) {
                __int128 dd = seg_rect_dist2(sg, Rect{p.x, p.y, p.x, p.y});
                if (best < 0 || dd < best) best = dd;
            }
            ranked.push_back({best, p});
        }
        std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
            if (a.first != b.first) return a.first < b.first;
            if (a.second.x != b.second.x) return a.second.x < b.second.x;
            return a.second.y < b.second.y;
        });
        bases.clear();
        bases.push_back(src);
        for (const auto& d : dsts) bases.push_back(d.p);
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
    // Issue #10: corridor clip. Drops base points outside the guidance
    // tube before node creation (src + every dst always survive), so the
    // graph, its edges and the exact search all stay inside the corridor.
    // Null/empty clip_path disables the filter (bit-identical to before).
    if (clip_path != nullptr && !clip_path->empty() && clip_half_width_nm > 0) {
        std::vector<Point> kept;
        kept.reserve(bases.size());
        for (auto p : bases) {
            bool prot = (p == src);
            if (!prot) {
                for (const auto& d : dsts) {
                    if (p == d.p) {
                        prot = true;
                        break;
                    }
                }
            }
            if (prot) {
                kept.push_back(p);
                continue;
            }
            Coord best = std::numeric_limits<Coord>::max();
            for (const auto& q : *clip_path) {
                Coord dd = manhattan(p, q);
                if (dd < best) best = dd;
                if (best <= clip_half_width_nm) break;
            }
            if (best <= clip_half_width_nm) kept.push_back(p);
        }
        bases = std::move(kept);
    }

    // ---- 3. Nodes: layer copies, filtered to legal wire positions ----
    // base index -> per-layer node id. Issue #23: point-legality probes for
    // bases that never become nodes are NOT evidence (A* never expands a
    // node that was never built), so they are checked without recording.
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
    g.dst_nodes_.clear();
    for (const auto& d : dsts) g.dst_nodes_.push_back(ensure_node(d.p, d.layer));
    g.dst_node_ = g.dst_nodes_.empty() ? -1 : g.dst_nodes_.front();
    // Adjacency must cover nodes added by ensure_node for extra dsts.
    g.adj_.resize(g.nodes_.size());
    // Issue #23: per-source-node rejected-transition store. try_edge pushes
    // one probe per failed candidate leg here; A* later accumulates only the
    // nodes it actually expands. Sized with the nodes above (ensure_node may
    // already have added forced src/dst nodes).
    std::vector<std::vector<SparseRejectedProbe>> rej_per_node(g.nodes_.size());

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
            // Issue #23: record the actual blocker on the SOURCE node only.
            // A* accumulates this probe iff it expands `from`.
            if (!seg_in_bounds(s1, committed.bounds(), half_w)) {
                rej_per_node[from].push_back({-1, "bounds", "off_board", layer, rep1});
                continue;
            }
            if (const Obstacle* o1 = first_blocker(s1, layer)) {
                rej_per_node[from].push_back({o1->net, o1->kind, o1->desc, layer, rep1});
                continue;
            }
            if (!s2_empty) {
                Point rep2{(c.elbow.x + b.x) / 2, (c.elbow.y + b.y) / 2};
                if (!seg_in_bounds(s2, committed.bounds(), half_w)) {
                    rej_per_node[from].push_back({-1, "bounds", "off_board", layer, rep2});
                    continue;
                }
                if (const Obstacle* o2 = first_blocker(s2, layer)) {
                    rej_per_node[from].push_back({o2->net, o2->kind, o2->desc, layer, rep2});
                    continue;
                }
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
        // K nearest on the same layer (deterministic: (distance, node id)
        // order). partial_sort yields EXACTLY the full-sort head by
        // construction (nth_element + resize + sort would cut ties at the
        // boundary arbitrarily and change the tried-pair set on grid
        // layouts with massive distance ties — maze regression).
        for (int i : ids) {
            std::vector<std::pair<Coord, int>> near;
            near.reserve(ids.size());
            for (int j : ids) {
                if (j == i) continue;
                near.push_back({manhattan(g.nodes_[i].p, g.nodes_[j].p), j});
            }
            if (static_cast<int>(near.size()) > kNearest) {
                std::partial_sort(near.begin(), near.begin() + kNearest,
                                  near.end());
                near.resize(kNearest);
            } else {
                std::sort(near.begin(), near.end());
            }
            for (const auto& pr : near) {
                try_edge(i, pr.second);
            }
        }
    }

    // ---- 5. Via edges between layer copies of the same base ----
    // Bundle-aware gate (issue #5): a transition exists only when the full
    // parallel-via bundle for this net fits legally around the base point
    // (all barrels + star stubs clear foreign copper under pair clearance).
    // The planner is deterministic, so candidate materialization replays the
    // identical bundle for the committed edge.
    //
    // S2: plan() is pure in (center, span) within one build (board, resolver,
    // net, width and ctx are fixed), so each copy-pair plans once and both
    // directions share it. Map key order keeps the result deterministic.
    std::map<std::tuple<Coord, Coord, LayerId, LayerId>, ViaBundle> via_memo;
    auto bundle_for = [&](Point p, LayerId a, LayerId b) -> const ViaBundle& {
        LayerId lo = std::min(a, b), hi = std::max(a, b);
        auto key = std::make_tuple(p.x, p.y, lo, hi);
        auto it = via_memo.find(key);
        if (it != via_memo.end()) return it->second;
        LayerSpan span{a, b};
        ViaBundle bundle = ViaBundlePlanner::plan(committed, resolver, net, p, span,
                                                  route_width_nm, ctx);
        return via_memo.emplace(key, std::move(bundle)).first->second;
    };
    if (!committed.layers.empty()) {
        for (const auto& [bi, per_layer] : node_of) {
            std::vector<std::pair<LayerId, int>> copies(per_layer.begin(), per_layer.end());
            for (std::size_t i = 0; i < copies.size(); ++i) {
                for (std::size_t j = 0; j < copies.size(); ++j) {
                    if (i == j) continue;
                    Point p = g.nodes_[copies[i].second].p;
                    const ViaBundle& bundle =
                        bundle_for(p, copies[i].first, copies[j].first);
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
    g.rejected_ = std::move(rej_per_node);
    // Issue #21 legacy (diagnostics only): construction-wide aggregation over
    // all per-node probes. Attribution must use the A*-frontier-only path
    // (AStarResult::frontier_stats via frontier_stats_for_expanded).
    {
        std::vector<SparseRejectedProbe> all;
        for (const auto& vec : g.rejected_)
            for (const auto& p : vec) all.push_back(p);
        g.frontier_stats_ = aggregate_probes(all);
    }
    return g;
}

std::vector<GraphFrontierStat> SparseRoutingGraph::aggregate_probes(
    const std::vector<SparseRejectedProbe>& probes) {
    struct Agg {
        NetId net = -1;
        std::string kind;
        std::string desc;
        LayerId layer = 0;
        Point pos{};
        int count = 0;
    };
    std::map<std::string, Agg> agg;
    for (const auto& p : probes) {
        std::string key =
            std::to_string(p.blocker_net) + "|" + p.kind + "|" + std::to_string(p.layer);
        auto it = agg.find(key);
        if (it == agg.end()) {
            Agg a;
            a.net = p.blocker_net;
            a.kind = p.kind;
            a.desc = p.desc;
            a.layer = p.layer;
            a.pos = p.pos;
            a.count = 1;
            agg.emplace(key, std::move(a));
        } else {
            it->second.count++;
            // Order-independent representative: min (x, y) hit per key.
            if (p.pos.x < it->second.pos.x ||
                (p.pos.x == it->second.pos.x && p.pos.y < it->second.pos.y))
                it->second.pos = p.pos;
        }
    }
    std::vector<Agg> all;
    all.reserve(agg.size());
    for (auto& [k, v] : agg) all.push_back(v);
    std::sort(all.begin(), all.end(), [](const Agg& a, const Agg& b) {
        if (a.count != b.count) return a.count > b.count;
        if (a.net != b.net) return a.net < b.net;
        if (a.kind != b.kind) return a.kind < b.kind;
        if (a.layer != b.layer) return a.layer < b.layer;
        return a.desc < b.desc;
    });
    std::vector<GraphFrontierStat> out;
    for (std::size_t i = 0; i < all.size() && (int)out.size() < kMaxFrontierStats; ++i) {
        GraphFrontierStat s;
        s.blocker_net = all[i].net;
        s.kind = all[i].kind;
        s.desc = all[i].desc;
        s.layer = all[i].layer;
        s.pos = all[i].pos;
        s.count = all[i].count;
        out.push_back(s);
    }
    return out;
}

std::vector<GraphFrontierStat> SparseRoutingGraph::frontier_stats_for_expanded(
    const std::vector<char>& expanded) const {
    std::vector<SparseRejectedProbe> probes;
    for (std::size_t i = 0; i < rejected_.size() && i < expanded.size(); ++i) {
        if (!expanded[i]) continue;  // A* never reached this source: not evidence
        for (const auto& p : rejected_[i]) probes.push_back(p);
    }
    return aggregate_probes(probes);
}

}  // namespace copperline
