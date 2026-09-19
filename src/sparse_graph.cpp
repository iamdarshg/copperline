#include "router/sparse_graph.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>

#include "router/via_bundle.h"
#include "router/simplify.h"

namespace copperline {

namespace {
// Diagnostics: cumulative per-section wall time inside build_multi (summed
// across threads via relaxed atomics; only ever added to, never read on the
// hot path). Exposed through sparse_graph_build_phases().
struct BuildPhaseAcc {
    std::atomic<std::int64_t> obstacles{0};
    std::atomic<std::int64_t> bases{0};
    std::atomic<std::int64_t> nodes{0};
    std::atomic<std::int64_t> edges{0};
    std::atomic<std::int64_t> via{0};
    std::atomic<std::int64_t> finalize{0};
    std::atomic<std::int64_t> calls{0};
    std::atomic<std::int64_t> nodes_total{0};
    std::atomic<std::int64_t> edges_total{0};
    std::atomic<std::int64_t> aligned_edges{0};
    std::atomic<std::int64_t> knn_edges{0};
    std::atomic<std::int64_t> via_edges{0};
};
BuildPhaseAcc& build_phase_acc() {
    static BuildPhaseAcc a;
    return a;
}
}  // namespace

SparseBuildPhases sparse_graph_build_phases() {
    BuildPhaseAcc& a = build_phase_acc();
    SparseBuildPhases out;
    out.obstacles_ns = a.obstacles.load(std::memory_order_relaxed);
    out.bases_ns = a.bases.load(std::memory_order_relaxed);
    out.nodes_ns = a.nodes.load(std::memory_order_relaxed);
    out.edges_ns = a.edges.load(std::memory_order_relaxed);
    out.via_ns = a.via.load(std::memory_order_relaxed);
    out.finalize_ns = a.finalize.load(std::memory_order_relaxed);
    out.calls = a.calls.load(std::memory_order_relaxed);
    out.nodes_total = a.nodes_total.load(std::memory_order_relaxed);
    out.edges_total = a.edges_total.load(std::memory_order_relaxed);
    out.aligned_edges = a.aligned_edges.load(std::memory_order_relaxed);
    out.knn_edges = a.knn_edges.load(std::memory_order_relaxed);
    out.via_edges = a.via_edges.load(std::memory_order_relaxed);
    return out;
}

void sparse_graph_build_phases_reset() {
    BuildPhaseAcc& a = build_phase_acc();
    a.obstacles.store(0, std::memory_order_relaxed);
    a.bases.store(0, std::memory_order_relaxed);
    a.nodes.store(0, std::memory_order_relaxed);
    a.edges.store(0, std::memory_order_relaxed);
    a.via.store(0, std::memory_order_relaxed);
    a.finalize.store(0, std::memory_order_relaxed);
    a.calls.store(0, std::memory_order_relaxed);
    a.nodes_total.store(0, std::memory_order_relaxed);
    a.edges_total.store(0, std::memory_order_relaxed);
    a.aligned_edges.store(0, std::memory_order_relaxed);
    a.knn_edges.store(0, std::memory_order_relaxed);
    a.via_edges.store(0, std::memory_order_relaxed);
}

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
    // Issue #1: traces are first-class Segment+width obstacles. raw stays
    // the copper bbox (trace bounds expanded by floor(trace_width/2)) for
    // broadphase/index filing; exact legality uses the centerline segment
    // with the verifier-exact 4*d2 >= (2*clear+cand_width+trace_width)^2
    // predicate (see SegLegalityCtx::segment_legal in simplify.cpp).
    bool is_trace = false;
    Segment trace_seg{{0, 0}, {0, 0}};
    Coord trace_width_nm = 0;  // full foreign trace width
    Coord trace_clear_nm = 0;  // pair clearance c (without half width)
    Coord cand_width_nm = 0;   // candidate (route) full width
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
    if (o.is_trace) {
        // Issue #1: verifier-exact trace predicate. The bbox precheck above
        // (raw = trace bbox expanded by floor(trace_width/2), dist_min =
        // clear + floor(cand_width/2)) is conservative on the integer grid:
        // any centerline pair violating 4*d2 < rhs^2 has per-axis bbox gaps
        // within the truncated need, so it always reaches the exact check.
        __int128 rhs = (__int128)2 * o.trace_clear_nm + o.cand_width_nm +
                       o.trace_width_nm;
        d2 = seg_seg_dist2(s, o.trace_seg);
        __int128 need = rhs * rhs;
        return (__int128)4 * d2 >= need;
    }
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
            // Issue #1: trace filing is raw.expanded(dist_min+1). raw already
            // holds floor(trace_width/2) and dist_min holds clear +
            // floor(cand_width/2); the exact capsule need from the trace
            // centerline is clear + (cand+trace)/2 which can exceed the
            // truncated sum by 1nm when both widths are odd. The +1nm keeps
            // filing conservative (superset, exact predicate still decides).
            Rect r = obs[oi].is_trace
                         ? obs[oi].raw.expanded(obs[oi].dist_min_nm + 1)
                         : obs[oi].raw.expanded(obs[oi].dist_min_nm);
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

// Issue #21/#23: shared bounded aggregation over per-node probe sets.
// `expanded` (optional) filters which sets contribute: null = all sets
// (construction-wide diagnostics), non-null = only nodes A* actually
// expanded. The result is independent of which sets contribute and in what
// order: counts are additive and the per-key representative is the min
// (x, y) hit, while the tail ordering is a strict total order over
// (count, net, kind, layer, desc). The (net, kind, layer) key is injective
// after the integer formatting the previous string key used, so keying by
// tuple is output-identical while dropping a string build+alloc per probe.
std::vector<GraphFrontierStat> aggregate_probe_sets_filtered(
    const std::vector<std::vector<SparseRejectedProbe>>& sets,
    const std::vector<char>* expanded) {
    struct Agg {
        std::string kind;
        std::string desc;
        Point pos{};
        int count = 0;
    };
    std::map<std::tuple<NetId, std::string, LayerId>, Agg> agg;
    for (std::size_t si = 0; si < sets.size(); ++si) {
        if (expanded != nullptr && (si >= expanded->size() || !(*expanded)[si]))
            continue;  // A* never reached this source: not evidence
        for (const auto& p : sets[si]) {
            auto key = std::make_tuple(p.blocker_net, p.kind, p.layer);
            auto it = agg.find(key);
            if (it == agg.end()) {
                Agg a;
                a.kind = p.kind;
                a.desc = p.desc;
                a.pos = p.pos;
                a.count = 1;
                agg.emplace(std::move(key), std::move(a));
            } else {
                it->second.count++;
                // Order-independent representative: min (x, y) hit per key.
                if (p.pos.x < it->second.pos.x ||
                    (p.pos.x == it->second.pos.x && p.pos.y < it->second.pos.y))
                    it->second.pos = p.pos;
            }
        }
    }
    struct Row {
        NetId net;
        std::string kind;
        std::string desc;
        LayerId layer;
        Point pos;
        int count;
    };
    std::vector<Row> rows;
    rows.reserve(agg.size());
    for (const auto& [key, a] : agg)
        rows.push_back(Row{std::get<0>(key), a.kind, a.desc, std::get<2>(key), a.pos, a.count});
    std::sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) {
        if (a.count != b.count) return a.count > b.count;
        if (a.net != b.net) return a.net < b.net;
        if (a.kind != b.kind) return a.kind < b.kind;
        if (a.layer != b.layer) return a.layer < b.layer;
        return a.desc < b.desc;
    });
    std::vector<GraphFrontierStat> out;
    out.reserve(std::min<std::size_t>(rows.size(), kMaxFrontierStats));
    for (const auto& r : rows) {
        if (static_cast<int>(out.size()) >= kMaxFrontierStats) break;
        GraphFrontierStat s;
        s.blocker_net = r.net;
        s.kind = r.kind;
        s.desc = r.desc;
        s.layer = r.layer;
        s.pos = r.pos;
        s.count = r.count;
        out.push_back(std::move(s));
    }
    return out;
}

}  // namespace

void SparseRoutingGraph::add_penalties(
    const std::function<Coord(const SparseNode&, const SparseEdge&)>& fn) {
    for (std::size_t i = 0; i < nodes_.size(); ++i)
        for (auto& e : adj_[i]) e.penalty_nm += fn(nodes_[i], e);
}

SparseRoutingGraph SparseRoutingGraph::clone_for_search() const {
    SparseRoutingGraph g;
    g.nodes_ = nodes_;
    g.adj_ = adj_;
    // Attribution-only: sized so frontier_stats_for_expanded can index it,
    // but with no probes copied (the expensive, string-carrying part).
    g.rejected_.resize(nodes_.size());
    g.stats_ = stats_;
    g.src_node_ = src_node_;
    g.dst_node_ = dst_node_;
    g.dst_nodes_ = dst_nodes_;
    return g;
}

SparseRoutingGraph SparseRoutingGraph::build(const Board& committed, const RuleResolver& resolver,
                                              NetId net, Point src, Point dst, LayerId src_layer,
                                              LayerId dst_layer, Coord route_width_nm,
                                              const ElectricalContext& ctx,
                                              const SparseGraphBudget& budget,
                                              NetId exempt_net) {
    SparseTarget single{dst, dst_layer};
    return build_multi(committed, resolver, net, src, src_layer, {single}, route_width_nm,
                       ctx, nullptr, 0, budget, exempt_net);
}

SparseRoutingGraph SparseRoutingGraph::build_multi(
    const Board& committed, const RuleResolver& resolver, NetId net, Point src,
    LayerId src_layer, const std::vector<SparseTarget>& dsts, Coord route_width_nm,
    const ElectricalContext& ctx, const std::vector<Point>* clip_path,
    Coord clip_half_width_nm, const SparseGraphBudget& budget, NetId exempt_net) {
    SparseRoutingGraph g;
    const Coord half_w = route_width_nm / 2;
    (void)dsts.empty();
    BuildPhaseAcc& bpa = build_phase_acc();
    auto pt = []() { return std::chrono::steady_clock::now(); };
    auto tick = [&](std::atomic<std::int64_t>& acc, std::chrono::steady_clock::time_point t0) {
        acc.fetch_add(std::chrono::duration_cast<std::chrono::nanoseconds>(pt() - t0).count(),
                      std::memory_order_relaxed);
    };
    bpa.calls.fetch_add(1, std::memory_order_relaxed);
    auto t_obstacles = pt();

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
    // Keepout standoff stays worst-case over ALL foreign nets (including a
    // paired sibling): keepouts are net-agnostic and the arbiter's exact
    // gate (ClearanceCache::max_clear) holds corridor copper to the same
    // margin. Only sibling-net *copper* is exempt (gap-governed).
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
        if (t.net == net || t.net == exempt_net) continue;  // own + sibling exempt
        const NetInfo* on = committed.find_net(t.net);
        std::string nm = on ? on->name : "?";
        std::string desc = "pad:net=" + nm +
                           (t.component.empty() ? "" : ":" + t.component + "." + t.pin);
        push_obstacle(t.pad_rect(), t.layer, clearance_to(t.net, t.layer) + half_w, t.net,
                      "pad", std::move(desc));
    }
    for (const auto& t : committed.traces) {
        if (t.net == net || t.net == exempt_net) continue;
        const NetInfo* on = committed.find_net(t.net);
        std::string nm = on ? on->name : "?";
        Segment bseg = t.segment();
        Rect raw = bseg.bounds().expanded(t.width_nm / 2);
        if (raw.x2 < raw.x1 || raw.y2 < raw.y1) continue;
        Coord c = clearance_to(t.net, t.layer);
        Obstacle o;
        o.raw = raw;
        o.layer = t.layer;
        o.dist_min_nm = c + half_w;
        o.net = t.net;
        o.kind = "trace";
        o.desc = "trace:net=" + nm;
        o.poly = nullptr;
        o.is_trace = true;
        o.trace_seg = bseg;
        o.trace_width_nm = t.width_nm;
        o.trace_clear_nm = c;
        o.cand_width_nm = route_width_nm;
        obstacles.push_back(std::move(o));
    }
    for (const auto& v : committed.vias) {
        if (v.net == net || v.net == exempt_net) continue;
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
        if (z.net == net || z.net == exempt_net) continue;
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
    tick(bpa.obstacles, t_obstacles);
    auto t_bases = pt();
    std::vector<int> scratch;  // per-build scratch (no sharing across threads:
                               // every build_multi call owns its locals)
    std::vector<int> query_raw;  // raw bucket hits (duplicates) before dedupe
    std::vector<int> seen(obstacles.size(), 0);
    int seen_gen = 0;
    // Perf (exact): first_blocker(s, layer) depends only on the *unordered*
    // segment geometry and the layer -- seg_legal_vs is direction-independent
    // (seg_seg_dist2 / seg_rect_dist2 / plane polygon distance are all
    // symmetric) and seg_in_bounds is not consulted here. decide_edge probes
    // every unordered pair from both ends (aligned pairs explicitly in both
    // directions, K-nearest because each end lists the other), so memoising on
    // the canonical (min-endpoint, max-endpoint, layer) key removes the second
    // and every later identical probe while replaying the IDENTICAL blocker
    // (same obstacle index => same kind/desc/net/pos), so rejected-probe
    // evidence is bit-identical.
    struct SegKeyHash {
        std::size_t operator()(
            const std::tuple<Coord, Coord, Coord, Coord, LayerId>& k) const {
            std::uint64_t h = 1469598103934665603ULL;
            auto mix = [&](std::uint64_t v) {
                h ^= v;
                h *= 1099511628211ULL;
            };
            mix(static_cast<std::uint64_t>(std::get<0>(k)));
            mix(static_cast<std::uint64_t>(std::get<1>(k)));
            mix(static_cast<std::uint64_t>(std::get<2>(k)));
            mix(static_cast<std::uint64_t>(std::get<3>(k)));
            mix(static_cast<std::uint64_t>(std::get<4>(k) + 2));
            return static_cast<std::size_t>(h);
        }
    };
    std::unordered_map<std::tuple<Coord, Coord, Coord, Coord, LayerId>, int, SegKeyHash>
        blocker_memo;
    blocker_memo.reserve(4096);
    auto first_blocker = [&](const Segment& s, LayerId layer) -> const Obstacle* {
        Point a = s.a, b = s.b;
        if (b.x < a.x || (b.x == a.x && b.y < a.y)) std::swap(a, b);
        const auto key = std::make_tuple(a.x, a.y, b.x, b.y, layer);
        auto mit = blocker_memo.find(key);
        if (mit != blocker_memo.end())
            return mit->second < 0
                       ? nullptr
                       : &obstacles[static_cast<std::size_t>(mit->second)];
        query_raw.clear();
        obs_index.query(s.bounds(), query_raw);
        // Linear dedupe (one hit per covering bucket) before the sort: the
        // ascending unique set is identical, the sort input is typically
        // several times smaller.
        ++seen_gen;
        scratch.clear();
        for (int oi : query_raw) {
            if (seen[static_cast<std::size_t>(oi)] == seen_gen) continue;
            seen[static_cast<std::size_t>(oi)] = seen_gen;
            scratch.push_back(oi);
        }
        std::sort(scratch.begin(), scratch.end());
        for (int oi : scratch) {
            const Obstacle& o = obstacles[static_cast<std::size_t>(oi)];
            if (!layer_match(o.layer, layer)) continue;
            if (!seg_legal_vs(s, o)) {
                blocker_memo.emplace(key, oi);
                return &o;
            }
        }
        blocker_memo.emplace(key, -1);
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
        if (o.is_trace) {
            // Issue #1: diagonal traces must not be over-culled by their
            // bbox. Keep the obstacle when EITHER the legacy bbox check hits
            // (superset, preserves all previously emitted corners) OR the
            // exact segment capsule (ceil need, covers odd-width 1nm band)
            // reaches the corridor. Emission stays bbox corners; legality is
            // unaffected (all obstacles participate regardless).
            bool keep = o.raw.expanded(o.dist_min_nm).intersects(corridor);
            if (!keep) {
                Coord need_ceil =
                    o.trace_clear_nm + (o.cand_width_nm + o.trace_width_nm + 1) / 2;
                __int128 d2 = seg_rect_dist2(o.trace_seg, corridor);
                keep = d2 <= (__int128)need_ceil * need_ceil;
            }
            if (!keep) continue;
        } else {
            if (!o.raw.expanded(o.dist_min_nm).intersects(corridor)) continue;
        }
        Rect exp = o.raw.expanded(o.dist_min_nm);
        Point corners[4] = {{exp.x1, exp.y1}, {exp.x2, exp.y1}, {exp.x2, exp.y2}, {exp.x1, exp.y2}};
        for (auto c : corners) bases.push_back(c);
        // Issue #10: plane legality is polygon-exact but discovery used bbox
        // corners only. Emit expanded polygon vertices (radial offset by
        // dist_min) alongside the bbox fallback; the inner-contains clamp
        // below keeps everything on the legal centerline region.
        if (o.poly != nullptr && !o.poly->empty() && o.dist_min_nm >= 0) {
            const std::vector<Point>& poly = *o.poly;
            long double cx = 0, cy = 0;
            for (const auto& v : poly) {
                cx += static_cast<long double>(v.x);
                cy += static_cast<long double>(v.y);
            }
            cx /= static_cast<long double>(poly.size());
            cy /= static_cast<long double>(poly.size());
            for (const auto& v : poly) {
                Point q = v;
                long double dx = static_cast<long double>(v.x) - cx;
                long double dy = static_cast<long double>(v.y) - cy;
                long double len = std::sqrt(dx * dx + dy * dy);
                if (len > 0.5L && o.dist_min_nm > 0) {
                    long double s = static_cast<long double>(o.dist_min_nm) / len;
                    q.x = v.x + static_cast<Coord>(std::llround(dx * s));
                    q.y = v.y + static_cast<Coord>(std::llround(dy * s));
                }
                bases.push_back(q);
            }
        }
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
    // Issue #4: the cap is budget.max_bases (default 384); 0 = uncapped
    // (last-resort completeness: keep every base).
    const std::size_t max_bases = budget.max_bases;
    if (max_bases > 0 && bases.size() > max_bases) {
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
        for (std::size_t i = 0; i < ranked.size() && bases.size() < max_bases; ++i)
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

    tick(bpa.bases, t_bases);
    auto t_nodes = pt();
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

    tick(bpa.nodes, t_nodes);
    auto t_edges = pt();
    // ---- 4. Manhattan edges: aligned pairs + K nearest per node ----
    // Issue #4: K is budget.k_nearest (default 16); <=0 = uncapped (try
    // every same-layer node). Aligned pairs are always tried regardless.
    const int kNearest = budget.k_nearest;
    // Probe-cut memo + colinear-chain inference (perf follow-up, exact):
    // edge legality is a pure function of the segment geometry (board,
    // resolver, net and width are fixed within one build), so a decided
    // directed pair is never re-probed: the stored outcome is replayed
    // bit-identically (same probes appended to the same source node, same
    // edge appended). Non-adjacent aligned pairs additionally skip probing
    // when every colinear chain link between them probed legal: a Manhattan
    // segment is the point-set union of its contiguous colinear halves, so an
    // obstacle (or the board bounds) violates the whole iff it violates a
    // half, under the exact same predicates. Failures are always probed
    // normally, so rejected-probe evidence (#23), determinism and the
    // committed node/edge sets are unchanged. All scratch state is per-build
    // (bounded, thread-local) and the memo is only ever queried by key, never
    // iterated, so thread count cannot affect the result.
    struct DecidedEdge {
        // Issue #3: parallel elbow edges. A diagonal pair can yield up to 3
        // legal edges (direct diagonal LOS + both L elbows); an aligned pair
        // yields exactly 1 (direct == straight). A* picks among them by cost
        // (length + penalty), so decide_edge must not stop at the first legal
        // elbow. Adjacency is vector<SparseEdge> per node, i.e. multi-edges
        // are native: apply_edge pushes every entry.
        std::vector<SparseEdge> edges;
        bool has_edge() const { return !edges.empty(); }
        int n_probes = 0;
        // Issue #2: <=1 for straight pairs, <=2 with elbows, +1 for the
        // direct diagonal LOS attempt (max 3 total when diagonal fails and
        // both elbow orders fail).
        SparseRejectedProbe probes[3]{};
    };
    auto decide_edge = [&](int from, int to, DecidedEdge& out) {
        out = DecidedEdge{};
        const Point a = g.nodes_[from].p;
        const Point b = g.nodes_[to].p;
        if (g.nodes_[from].layer != g.nodes_[to].layer) return;
        LayerId layer = g.nodes_[from].layer;
        const bool is_aligned = (a.x == b.x || a.y == b.y);
        // Issue #2: direct arbitrary-angle line-of-sight first. Probes the
        // exact Segment(a,b) with the same seg_in_bounds + first_blocker
        // (seg_legal_vs) predicates the Manhattan legs use. On success the
        // edge carries Euclidean length (euclid_len_nm, == manhattan for
        // axis-aligned pairs) with the straight-edge encoding (dir2 = -1,
        // elbow unused); materialization already emits dir2 < 0 as one
        // direct Segment(nu.p, nv.p), and the simplifier/verifier accept
        // arbitrary angles, so no other stage changes.
        // Issue #3: the direct edge no longer suppresses the elbows. For an
        // aligned pair direct == straight so we return with the single edge
        // (emitting the straight candidate again would duplicate it); for a
        // diagonal pair we keep the direct edge AND probe both elbows below.
        {
            Segment direct{a, b};
            Point rep{(a.x + b.x) / 2, (a.y + b.y) / 2};
            bool direct_ok = false;
            if (!seg_in_bounds(direct, committed.bounds(), half_w)) {
                out.probes[out.n_probes++] = {-1, "bounds", "off_board", layer, rep};
            } else if (const Obstacle* od = first_blocker(direct, layer)) {
                out.probes[out.n_probes++] = {od->net, od->kind, od->desc, layer, rep};
            } else {
                direct_ok = true;
            }
            if (direct_ok) {
                SparseEdge e;
                e.to = to;
                e.len_nm = euclid_len_nm(a, b);
                e.dir1 = direction_of(a, b);
                e.dir2 = -1;
                e.elbow = Point{};
                e.is_via = false;
                out.edges.push_back(e);
                if (is_aligned) return;
            }
        }
        // Candidate paths: straight (if aligned) else both elbow orders.
        struct Cand {
            Point elbow;
            bool has_elbow;
        };
        Cand cands[2];
        int n_cands = 0;
        if (is_aligned) {
            cands[n_cands++] = {{}, false};
        } else {
            cands[n_cands++] = {{b.x, a.y}, true};
            cands[n_cands++] = {{a.x, b.y}, true};
        }
        for (int ci = 0; ci < n_cands; ++ci) {
            const Cand& c = cands[ci];
            Segment s1{a, c.has_elbow ? c.elbow : b};
            Segment s2{c.has_elbow ? Segment{c.elbow, b} : Segment{b, b}};
            bool s2_empty = !c.has_elbow;
            Point rep1{(a.x + (c.has_elbow ? c.elbow.x : b.x)) / 2,
                       (a.y + (c.has_elbow ? c.elbow.y : b.y)) / 2};
            // Issue #23: record the actual blocker on the SOURCE node only.
            // A* accumulates this probe iff it expands `from`.
            if (!seg_in_bounds(s1, committed.bounds(), half_w)) {
                out.probes[out.n_probes++] = {-1, "bounds", "off_board", layer, rep1};
                continue;
            }
            if (const Obstacle* o1 = first_blocker(s1, layer)) {
                out.probes[out.n_probes++] = {o1->net, o1->kind, o1->desc, layer, rep1};
                continue;
            }
            if (!s2_empty) {
                Point rep2{(c.elbow.x + b.x) / 2, (c.elbow.y + b.y) / 2};
                if (!seg_in_bounds(s2, committed.bounds(), half_w)) {
                    out.probes[out.n_probes++] = {-1, "bounds", "off_board", layer, rep2};
                    continue;
                }
                if (const Obstacle* o2 = first_blocker(s2, layer)) {
                    out.probes[out.n_probes++] = {o2->net, o2->kind, o2->desc, layer, rep2};
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
            out.edges.push_back(e);
            // Issue #3: no break -- collect BOTH legal elbows so later
            // congestion/reservation penalties in A* can choose between them.
        }
    };
    // Diagnostics only: attribute committed edges to the pass that made them
    // (aligned pairs vs K-nearest vs via bundles) so the edge cost can be
    // targeted. Never affects the graph.
    std::int64_t aligned_edges_local = 0, knn_edges_local = 0, via_edges_local = 0;
    std::int64_t* edge_ctr = nullptr;
    auto apply_edge = [&](int from, const DecidedEdge& d) {
        if (edge_ctr != nullptr) *edge_ctr += static_cast<std::int64_t>(d.edges.size());
        for (int k = 0; k < d.n_probes; ++k) rej_per_node[from].push_back(d.probes[k]);
        for (const auto& e : d.edges) g.adj_[from].push_back(e);
    };
    auto try_edge = [&](int from, int to) {
        if (from == to) return;
        DecidedEdge d;
        decide_edge(from, to, d);
        apply_edge(from, d);
    };
    auto directed_key = [](int from, int to) {
        return (static_cast<unsigned long long>(static_cast<unsigned int>(from)) << 32) |
               static_cast<unsigned int>(to);
    };
    // Every decided aligned directed pair (probed or inferred). Consulted by
    // the K-nearest pass so shared pairs replay instead of re-probing.
    // Single-allocation open-addressing map: no per-entry mallocs that would
    // contend across worker threads. find() results are consumed (applied)
    // before the next insert, so outs growth never invalidates a live use.
    struct OutcomeMap {
        std::vector<unsigned long long> keys;  // stored key+1, 0 = empty
        std::vector<unsigned int> slots;       // index into outs
        std::vector<DecidedEdge> outs;
        std::vector<unsigned long long> out_keys;  // key+1 per out index
        std::size_t mask = 0;
        void init(std::size_t want) {
            std::size_t cap = 8;
            while (cap < want * 2) cap *= 2;
            keys.assign(cap, 0);
            slots.assign(cap, 0);
            outs.clear();
            out_keys.clear();
            outs.reserve(want);
            out_keys.reserve(want);
            mask = cap - 1;
        }
        void grow() {
            std::size_t cap = keys.size() * 2;
            keys.assign(cap, 0);
            slots.assign(cap, 0);
            mask = cap - 1;
            // Re-insert in outs order: deterministic for a given insertion
            // sequence (all inserts happen on this thread anyway).
            for (std::size_t s = 0; s < outs.size(); ++s) {
                std::size_t i = hash(out_keys[s]) & mask;
                while (keys[i] != 0) i = (i + 1) & mask;
                keys[i] = out_keys[s];
                slots[i] = static_cast<unsigned int>(s);
            }
        }
        static unsigned long long hash(unsigned long long k) {
            k ^= k >> 33;
            k *= 0xff51afd7ed558ccdULL;
            k ^= k >> 33;
            k *= 0xc4ceb9fe1a85ec53ULL;
            k ^= k >> 33;
            return k;
        }
        const DecidedEdge* find(unsigned long long key) const {
            if (mask == 0) return nullptr;
            unsigned long long k = key + 1;
            std::size_t i = hash(k) & mask;
            for (;;) {
                if (keys[i] == 0) return nullptr;
                if (keys[i] == k) return &outs[slots[i]];
                i = (i + 1) & mask;
            }
        }
        void insert(unsigned long long key, const DecidedEdge& d) {
            // Keep load under 0.7 so find() always terminates quickly; growth
            // re-inserts in outs order (deterministic for one insertion seq).
            if ((outs.size() + 1) * 10 >= keys.size() * 7) grow();
            unsigned long long k = key + 1;
            std::size_t i = hash(k) & mask;
            for (;;) {
                if (keys[i] == 0) {
                    keys[i] = k;
                    slots[i] = static_cast<unsigned int>(outs.size());
                    outs.push_back(d);
                    out_keys.push_back(k);
                    return;
                }
                // Keys are unique per build (each directed pair is decided
                // exactly once), so a hit here is unreachable.
                i = (i + 1) & mask;
            }
        }
    };
    OutcomeMap aligned_outcome;
    aligned_outcome.init(g.nodes_.size() * 40 + 64);
    for (const auto& [layer, ids] : layer_nodes) {
        // Aligned pairs (shared x or y) preserve corridors at any distance.
        std::map<Coord, std::vector<int>> by_x, by_y;
        for (int i : ids) {
            by_x[g.nodes_[i].p.x].push_back(i);
            by_y[g.nodes_[i].p.y].push_back(i);
        }
        // Colinear chains for inference: members sorted along the line,
        // link_ok[t] = both directed probes of (members[t], members[t+1])
        // built an edge. Node ids are dense (0..N-1, all nodes exist by now),
        // so chain slots live in plain vectors (no hash maps): xchain[node]
        // is the node's x-chain index (-1 = none), xidx[node] its slot.
        struct Chain {
            std::vector<int> members;
            std::vector<char> link_ok;
        };
        std::vector<Chain> chains;
        chains.reserve(by_x.size() + by_y.size());
        const int n_all_nodes = static_cast<int>(g.nodes_.size());
        std::vector<int> xchain(n_all_nodes, -1), xidx(n_all_nodes, -1);
        std::vector<int> ychain(n_all_nodes, -1), yidx(n_all_nodes, -1);
        auto add_chain = [&](const std::map<Coord, std::vector<int>>& by, bool sort_by_y,
                             std::vector<int>& chain_of, std::vector<int>& idx_of) {
            for (const auto& [c, group] : by) {
                (void)c;
                if (group.size() < 2) continue;
                Chain ch;
                ch.members = group;
                std::sort(ch.members.begin(), ch.members.end(), [&](int u, int v) {
                    Coord au = sort_by_y ? g.nodes_[u].p.y : g.nodes_[u].p.x;
                    Coord av = sort_by_y ? g.nodes_[v].p.y : g.nodes_[v].p.x;
                    if (au != av) return au < av;
                    return u < v;
                });
                ch.link_ok.assign(ch.members.size() - 1, 0);
                int ci = static_cast<int>(chains.size());
                for (std::size_t t = 0; t < ch.members.size(); ++t) {
                    chain_of[ch.members[t]] = ci;
                    idx_of[ch.members[t]] = static_cast<int>(t);
                }
                chains.push_back(std::move(ch));
            }
        };
        add_chain(by_x, true, xchain, xidx);    // shared x: order along y
        add_chain(by_y, false, ychain, yidx);   // shared y: order along x
        // Pass 1: probe every adjacent chain link in both directions (pure
        // decide, no mutation yet) so pass 2 appends in legacy order.
        for (auto& ch : chains) {
            for (std::size_t t = 0; t < ch.link_ok.size(); ++t) {
                int u = ch.members[t], v = ch.members[t + 1];
                DecidedEdge duv, dvu;
                decide_edge(u, v, duv);
                decide_edge(v, u, dvu);
                // Success is symmetric (same segments); AND keeps the
                // inference conservative under any future asymmetry.
                // Issue #3: aligned chain links carry exactly one edge (direct
                // == straight), so non-empty is the success test.
                ch.link_ok[t] = (!duv.edges.empty() && !dvu.edges.empty()) ? 1 : 0;
                aligned_outcome.insert(directed_key(u, v), duv);
                aligned_outcome.insert(directed_key(v, u), dvu);
            }
        }
        auto chain_clean = [&](int from, int to, bool share_x) {
            const std::vector<int>& chain_of = share_x ? xchain : ychain;
            const std::vector<int>& idx_of = share_x ? xidx : yidx;
            int cf = chain_of[from], ct = chain_of[to];
            if (cf < 0 || cf != ct) return false;
            const Chain& ch = chains[static_cast<std::size_t>(cf)];
            int a = idx_of[from], b = idx_of[to];
            if (a == b) return false;
            if (a > b) std::swap(a, b);
            if (b == a + 1) return false;  // adjacent: decided in pass 1
            for (int t = a; t < b; ++t)
                if (!ch.link_ok[static_cast<std::size_t>(t)]) return false;
            return true;
        };
        auto aligned_attempt = [&](int from, int to) {
            if (from == to) return;
            unsigned long long key = directed_key(from, to);
            if (const DecidedEdge* hit = aligned_outcome.find(key)) {
                apply_edge(from, *hit);
                return;
            }
            const Point a = g.nodes_[from].p, b = g.nodes_[to].p;
            bool share_x = (a.x == b.x), share_y = (a.y == b.y);
            if ((share_x != share_y) && chain_clean(from, to, share_x)) {
                // Exact inference (see memo header comment): the whole chain
                // probed legal with straight links, so this straight pair is
                // legal with zero probes and an identical edge.
                DecidedEdge d;
                SparseEdge ie;
                ie.to = to;
                ie.len_nm = manhattan(a, b);
                ie.dir1 = direction_of(a, b);
                ie.dir2 = -1;
                ie.elbow = Point{};
                ie.is_via = false;
                d.edges.push_back(ie);
                apply_edge(from, d);
                aligned_outcome.insert(key, d);
                return;
            }
            DecidedEdge d;
            decide_edge(from, to, d);
            apply_edge(from, d);
            aligned_outcome.insert(key, d);
        };
        edge_ctr = &aligned_edges_local;
        for (int i : ids) {
            for (int j : by_x[g.nodes_[i].p.x]) aligned_attempt(i, j);
            for (int j : by_y[g.nodes_[i].p.y]) aligned_attempt(i, j);
        }
        // K nearest on the same layer (deterministic: (distance, node id)
        // order). partial_sort yields EXACTLY the full-sort head by
        // construction (nth_element + resize + sort would cut ties at the
        // boundary arbitrarily and change the tried-pair set on grid
        // layouts with massive distance ties — maze regression).
        edge_ctr = &knn_edges_local;
        for (int i : ids) {
            std::vector<std::pair<Coord, int>> near;
            near.reserve(ids.size());
            for (int j : ids) {
                if (j == i) continue;
                near.push_back({manhattan(g.nodes_[i].p, g.nodes_[j].p), j});
            }
            if (kNearest > 0 && static_cast<int>(near.size()) > kNearest) {
                std::partial_sort(near.begin(), near.begin() + kNearest,
                                  near.end());
                near.resize(kNearest);
            } else {
                std::sort(near.begin(), near.end());
            }
            for (const auto& pr : near) {
                unsigned long long key = directed_key(i, pr.second);
                if (const DecidedEdge* hit = aligned_outcome.find(key)) {
                    apply_edge(i, *hit);
                } else {
                    try_edge(i, pr.second);
                }
            }
        }
        edge_ctr = nullptr;
    }

    tick(bpa.edges, t_edges);
    auto t_via = pt();
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
    //
    // Perf follow-up (exact): style selection is pure in (resolver, net,
    // span) within one build, so the ordered (style, count) list is computed
    // once per distinct layer pair instead of once per copy-pair plan. The
    // per-pair loop replays ViaBundlePlanner::plan's sequence exactly
    // (styles in order, first feasible wins, last failure kept, no-style
    // fallback preserved), so bundles are bit-identical.
    const NetInfo* task_net = committed.find_net(net);
    // Perf (exact): via legality (via_pos_legal / stub_seg_legal) scans ALL
    // board copper per barrel, per layout candidate, per style, and every base
    // for this task lies inside the (already clipped) corridor. Any bundle or
    // star stub reaches at most `via_reach` from its centre, so ONE filtered
    // board covering the corridor expanded by that reach is a conservative
    // superset for every plan in this build: the same predicates decide on a
    // strictly smaller candidate set, so bundles are bit-identical.
    Coord max_via_outer = 0, max_via_extent = 0;
    if (task_net != nullptr) {
        for (const auto& s : resolver.via_styles()) {
            if (s.outer_nm <= 0) continue;
            ElectricalContext c2 = ctx;
            int need = resolver.current().vias_required(s, *task_net, committed.defaults, c2);
            if (need < 1) need = 1;
            const Coord pitch = s.outer_nm + mm_to_nm(0.15);
            const Coord ext = (static_cast<Coord>(need) - 1) * pitch / 2 + s.outer_nm;
            max_via_outer = std::max(max_via_outer, s.outer_nm);
            max_via_extent = std::max(max_via_extent, ext);
        }
    }
    const Coord via_reach =
        max_clear + route_width_nm + 2 * max_via_extent + max_via_outer + mm_to_nm(0.5);
    Board via_board;
    const Board* via_scan = &committed;
    if (max_via_extent > 0) {
        const Rect box = corridor.expanded(via_reach);
        via_board = committed;  // scalars, nets, defaults, layers, bounds
        auto keep_rect = [&](const Rect& r) { return r.intersects(box); };
        via_board.keepouts.clear();
        for (const auto& k : committed.keepouts)
            if (keep_rect(k.rect)) via_board.keepouts.push_back(k);
        via_board.terminals.clear();
        for (const auto& t : committed.terminals)
            if (keep_rect(t.pad_rect())) via_board.terminals.push_back(t);
        via_board.traces.clear();
        for (const auto& t : committed.traces)
            if (keep_rect(t.segment().bounds().expanded(t.width_nm / 2)))
                via_board.traces.push_back(t);
        via_board.vias.clear();
        for (const auto& v : committed.vias)
            if (keep_rect(Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm)))
                via_board.vias.push_back(v);
        via_board.planes.clear();
        for (const auto& z : committed.planes)
            if (keep_rect(z.bounds())) via_board.planes.push_back(z);
        via_scan = &via_board;
    }
    // Perf follow-up (exact): one shared per-net clearance memo for every via
    // bundle plan in this build. Clearances are pure in (board, net pair) —
    // the resolver ignores layer/ctx — so the shared values are identical to
    // per-call refills; the memo is per-build (thread-local) and only read
    // after it is warm. Previously each of the ~hundreds of bundle plans
    // refilled N nets × requiredClearance (2 linear find_net scans each).
    ClearanceCache shared_cc(committed, resolver, ctx, net);
    std::map<std::pair<LayerId, LayerId>, std::vector<std::pair<ViaStyle, int>>> style_cache;
    auto styles_for = [&](LayerId a, LayerId b)
        -> const std::vector<std::pair<ViaStyle, int>>& {
        auto key = std::make_pair(a, b);
        auto it = style_cache.find(key);
        if (it != style_cache.end()) return it->second;
        std::vector<std::pair<ViaStyle, int>> v;
        if (task_net != nullptr) {
            LayerSpan span{a, b};
            for (const auto& style : ViaBundlePlanner::ordered_styles(resolver, net, span)) {
                ElectricalContext c2 = ctx;
                int need = resolver.current().vias_required(style, *task_net,
                                                            committed.defaults, c2);
                v.push_back({style, need});
            }
        }
        return style_cache.emplace(key, std::move(v)).first->second;
    };
    auto plan_bundle = [&](Point p, LayerSpan span,
                           const std::vector<std::pair<ViaStyle, int>>& styles) {
        ViaBundle fail;
        fail.reason = "no_via_class";
        if (task_net == nullptr) return fail;
        bool tried = false;
        for (const auto& [style, need] : styles) {
            tried = true;
            ViaBundle b = ViaBundlePlanner::plan_with_style(*via_scan, resolver, net, p, span,
                                                            style, need, route_width_nm, ctx,
                                                            shared_cc, exempt_net);
            if (b.feasible) return b;
            fail = b;  // keep the last deterministic reason (bundle_blocked)
        }
        if (!tried) {
            fail.reason = "no_via_class";
            fail.count = 0;
        }
        return fail;
    };
    std::map<std::tuple<Coord, Coord, LayerId, LayerId>, ViaBundle> via_memo;
    auto bundle_for = [&](Point p, LayerId a, LayerId b) -> const ViaBundle& {
        LayerId lo = std::min(a, b), hi = std::max(a, b);
        auto key = std::make_tuple(p.x, p.y, lo, hi);
        auto it = via_memo.find(key);
        if (it != via_memo.end()) return it->second;
        LayerSpan span{a, b};
        ViaBundle bundle = plan_bundle(p, span, styles_for(a, b));
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
                    ++via_edges_local;
                    g.adj_[copies[i].second].push_back(e);
                }
            }
        }
    }

    tick(bpa.via, t_via);
    auto t_finalize = pt();
    // Deterministic edge order. Issue #3: parallel elbow edges share
    // (is_via, to, len) (both elbows have identical Manhattan length), so
    // tie-break on elbow/dir to keep the order total and stable.
    for (auto& vec : g.adj_) {
        std::sort(vec.begin(), vec.end(), [](const SparseEdge& a, const SparseEdge& b) {
            if (a.is_via != b.is_via) return a.is_via < b.is_via;
            if (a.to != b.to) return a.to < b.to;
            if (a.len_nm != b.len_nm) return a.len_nm < b.len_nm;
            if (a.dir1 != b.dir1) return a.dir1 < b.dir1;
            if (a.dir2 != b.dir2) return a.dir2 < b.dir2;
            if (a.elbow.x != b.elbow.x) return a.elbow.x < b.elbow.x;
            return a.elbow.y < b.elbow.y;
        });
    }
    int edges = 0;
    for (const auto& vec : g.adj_) edges += static_cast<int>(vec.size());
    g.stats_.node_count = static_cast<int>(g.nodes_.size());
    g.stats_.edge_count = edges;
    bpa.nodes_total.fetch_add(static_cast<std::int64_t>(g.nodes_.size()),
                              std::memory_order_relaxed);
    bpa.edges_total.fetch_add(static_cast<std::int64_t>(edges),
                              std::memory_order_relaxed);
    bpa.aligned_edges.fetch_add(aligned_edges_local, std::memory_order_relaxed);
    bpa.knn_edges.fetch_add(knn_edges_local, std::memory_order_relaxed);
    bpa.via_edges.fetch_add(via_edges_local, std::memory_order_relaxed);
    g.rejected_ = std::move(rej_per_node);
    // Issue #21 legacy aggregation is intentionally NOT computed here: it is
    // diagnostics-only (production reads the A*-frontier-only path), so it is
    // computed on demand in frontier_stats(). Previously it ran on every
    // build and aggregated every string-carrying probe across the whole graph.
    tick(bpa.finalize, t_finalize);
    return g;
}

std::vector<GraphFrontierStat> SparseRoutingGraph::frontier_stats() const {
    return aggregate_probe_sets(rejected_);
}

std::vector<GraphFrontierStat> SparseRoutingGraph::aggregate_probe_sets(
    const std::vector<std::vector<SparseRejectedProbe>>& sets) {
    return aggregate_probe_sets_filtered(sets, nullptr);
}

std::vector<GraphFrontierStat> SparseRoutingGraph::aggregate_probes(
    const std::vector<SparseRejectedProbe>& probes) {
    return aggregate_probe_sets({probes});
}

std::vector<GraphFrontierStat> SparseRoutingGraph::frontier_stats_for_expanded(
    const std::vector<char>& expanded) const {
    // #23: aggregate the ledger in place behind the expanded mask instead of
    // copying every reached probe into a flat vector (and then into a
    // one-element vector-of-vectors). Output is identical: the aggregation
    // is order-independent and the final ordering is a strict total order.
    return aggregate_probe_sets_filtered(rejected_, &expanded);
}

}  // namespace copperline
