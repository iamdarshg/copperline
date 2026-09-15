// Copperline: hierarchical coarse-to-fine A* guidance (issue #10).
//
// The guidance grid is a separate lightweight raster over (cell x layer)
// states. It never decides legality: all blockage tests here are advisory
// (conservative or sampling-based), while the exact sparse graph plus the
// issue-#17 simplifier remain the sole source of committed copper.
#include "router/hierarchy.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <queue>
#include <tuple>

#include "router/astar.h"

namespace copperline {

JsonValue HierarchyDiag::to_json() const {
    JsonValue o = JsonValue::object();
    o["attempted"] = attempted;
    o["guided"] = guided;
    o["fallback"] = fallback;
    o["fallback_reason"] = fallback_reason;
    JsonValue lv = JsonValue::array();
    for (double m : levels_used_mm) lv.as_array().push_back(JsonValue(m));
    o["levels_used_mm"] = lv;
    o["coarse_expansions"] = static_cast<double>(coarse_expansions);
    o["exact_expansions"] = static_cast<double>(exact_expansions);
    o["window_attempts"] = static_cast<double>(window_attempts);
    return o;
}

namespace {

// Exact integer point-to-rect gap (0 when inside/touching). Guidance-only
// sampling; determinism comes from integer inputs (llround on the diagonal
// is stable for a fixed binary).
Coord point_rect_gap(Point c, const Rect& r) {
    Coord dx = 0, dy = 0;
    if (c.x < r.x1) dx = r.x1 - c.x;
    else if (c.x > r.x2) dx = c.x - r.x2;
    if (c.y < r.y1) dy = r.y1 - c.y;
    else if (c.y > r.y2) dy = c.y - r.y2;
    if (dx == 0 && dy == 0) return 0;
    if (dx == 0) return dy;
    if (dy == 0) return dx;
    long double d2 = static_cast<long double>(dx) * dx + static_cast<long double>(dy) * dy;
    return static_cast<Coord>(std::llround(std::sqrt(d2)));
}

bool layer_match(LayerId obstacle_layer, LayerId query_layer) {
    return obstacle_layer == kAllLayers || obstacle_layer == query_layer;
}

// FNV-1a/64 over committed copper + structural counts. Independent of
// parallel.h's geometry_hash (which covers the same copper); identical
// boards always produce identical keys, so cache reuse is deterministic.
std::string snapshot_key(const Board& b) {
    std::uint64_t h = 1469598103934665603ULL;
    auto mix = [&](std::uint64_t v) {
        h ^= v;
        h *= 1099511628211ULL;
    };
    auto mix_coord = [&](Coord v) { mix(static_cast<std::uint64_t>(v)); };
    mix_coord(b.width_nm);
    mix_coord(b.height_nm);
    mix(static_cast<std::uint64_t>(b.layers.size()));
    mix(static_cast<std::uint64_t>(b.keepouts.size()));
    mix(static_cast<std::uint64_t>(b.terminals.size()));
    mix(static_cast<std::uint64_t>(b.traces.size()));
    mix(static_cast<std::uint64_t>(b.vias.size()));
    mix(static_cast<std::uint64_t>(b.planes.size()));
    for (const auto& t : b.traces) {
        mix(static_cast<std::uint64_t>(t.net));
        mix(static_cast<std::uint64_t>(t.layer));
        mix_coord(t.a.x);
        mix_coord(t.a.y);
        mix_coord(t.b.x);
        mix_coord(t.b.y);
        mix_coord(t.width_nm);
    }
    for (const auto& v : b.vias) {
        mix(static_cast<std::uint64_t>(v.net));
        mix_coord(v.pos.x);
        mix_coord(v.pos.y);
        mix(static_cast<std::uint64_t>(v.top_layer));
        mix(static_cast<std::uint64_t>(v.bottom_layer));
        mix_coord(v.outer_d_nm);
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%016llx", (unsigned long long)h);
    return std::string(buf);
}

struct ExpObs {
    Rect raw{};
    LayerId layer = kAllLayers;
    Coord dist_min_nm = 0;
};

// Uniform bucket index over the per-task expanded obstacles. Queries
// return candidate obstacle indices whose raw rect may interact; exact
// predicates still decide. Order-independent results (boolean OR), so
// sharing across threads is safe.
struct ObstacleIndex {
    Coord bucket = 1000000;  // 1mm
    int nx = 0, ny = 0;
    Rect bounds{};
    std::vector<int> head;  // nx*ny bucket heads, -1 empty
    std::vector<int> next;  // per-obstacle chain link

    void build(const std::vector<ExpObs>& obs, const Rect& b, Coord max_dist) {
        bounds = b;
        bucket = std::max<Coord>(1000000, max_dist);  // covers any query fan-out
        Coord w = std::max<Coord>(1, b.width());
        Coord h = std::max<Coord>(1, b.height());
        nx = std::max(1, static_cast<int>((w + bucket - 1) / bucket));
        ny = std::max(1, static_cast<int>((h + bucket - 1) / bucket));
        head.assign(static_cast<std::size_t>(nx) * ny, -1);
        next.assign(obs.size(), -1);
        for (std::size_t oi = 0; oi < obs.size(); ++oi) {
            const Rect& r = obs[oi].raw;
            int ix0 = std::clamp(static_cast<int>((r.x1 - b.x1) / bucket), 0, nx - 1);
            int ix1 = std::clamp(static_cast<int>((r.x2 - b.x1) / bucket), 0, nx - 1);
            int iy0 = std::clamp(static_cast<int>((r.y1 - b.y1) / bucket), 0, ny - 1);
            int iy1 = std::clamp(static_cast<int>((r.y2 - b.y1) / bucket), 0, ny - 1);
            for (int iy = iy0; iy <= iy1; ++iy)
                for (int ix = ix0; ix <= ix1; ++ix) {
                    std::size_t bi = static_cast<std::size_t>(iy) * nx + ix;
                    next[oi] = head[bi];
                    head[bi] = static_cast<int>(oi);
                }
        }
    }

    // Calls fn(obstacle_index) for every obstacle filed in a bucket
    // touched by q. Duplicates across buckets are possible; all uses are
    // boolean-OR so repeats are harmless.
    template <typename F>
    void for_each(const Rect& q, F&& fn) const {
        if (nx <= 0 || ny <= 0) return;
        int ix0 = std::clamp(static_cast<int>((q.x1 - bounds.x1) / bucket), 0, nx - 1);
        int ix1 = std::clamp(static_cast<int>((q.x2 - bounds.x1) / bucket), 0, nx - 1);
        int iy0 = std::clamp(static_cast<int>((q.y1 - bounds.y1) / bucket), 0, ny - 1);
        int iy1 = std::clamp(static_cast<int>((q.y2 - bounds.y1) / bucket), 0, ny - 1);
        for (int iy = iy0; iy <= iy1; ++iy)
            for (int ix = ix0; ix <= ix1; ++ix) {
                for (int oi = head[static_cast<std::size_t>(iy) * nx + ix]; oi >= 0;
                     oi = next[oi])
                    fn(oi);
            }
    }
};

struct LevelGrid {
    Coord pitch = 0;
    int nx = 0, ny = 0;
    Rect bounds{};
    // blocked[(li * ny + iy) * nx + ix], li = layer position in board order
    std::vector<char> blocked;
};

Point cell_center(const LevelGrid& g, int ix, int iy) {
    return {g.bounds.x1 + ix * g.pitch + g.pitch / 2,
            g.bounds.y1 + iy * g.pitch + g.pitch / 2};
}

Rect cell_rect(const LevelGrid& g, int ix, int iy) {
    Coord x1 = g.bounds.x1 + ix * g.pitch;
    Coord y1 = g.bounds.y1 + iy * g.pitch;
    return {x1, y1, std::min(x1 + g.pitch, g.bounds.x2),
            std::min(y1 + g.pitch, g.bounds.y2)};
}

}  // namespace

const HierarchyCache::Snapshot& HierarchyCache::snapshot_for(const Board& board) const {
    std::string key = snapshot_key(board);
    std::lock_guard<std::mutex> lock(mutex_);
    if (key == cached_key_) return cached_;
    Snapshot s;
    s.key = key;
    s.bounds = board.bounds();
    s.obstacles.reserve(board.keepouts.size() + board.terminals.size() +
                        board.traces.size() + board.vias.size() + board.planes.size());
    for (const auto& ko : board.keepouts)
        s.obstacles.push_back({ko.rect, ko.layer, -1});
    for (const auto& t : board.terminals)
        s.obstacles.push_back({t.pad_rect(), t.layer, t.net});
    for (const auto& t : board.traces) {
        Rect raw = t.segment().bounds().expanded(t.width_nm / 2);
        s.obstacles.push_back({raw, t.layer, t.net});
    }
    for (const auto& v : board.vias) {
        Rect raw = Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm);
        // One entry per spanned layer keeps layer_match() exact.
        for (const auto& l : board.layers) {
            bool in_span = (l.id >= std::min(v.top_layer, v.bottom_layer) &&
                            l.id <= std::max(v.top_layer, v.bottom_layer));
            if (in_span) s.obstacles.push_back({raw, l.id, v.net});
        }
    }
    for (const auto& z : board.planes)
        s.obstacles.push_back({z.bounds(), z.layer, z.net});
    cached_ = std::move(s);
    cached_key_ = key;
    return cached_;
}

GuidanceResult HierarchyCache::build_guidance(const Board& board,
                                              const RuleResolver& resolver,
                                              const ElectricalContext& ctx,
                                              const HierarchyRequest& req,
                                              const HierarchyConfig& cfg) const {
    GuidanceResult out;
    if (cfg.level_pitch_nm.empty() || req.dsts.empty()) return out;
    const Snapshot& snap = snapshot_for(board);
    const Coord half_w = req.width_nm / 2;

    // Per-task keep distances. Clearance is layer-independent, so one lookup
    // per foreign net (mirrors the sparse graph builder).
    std::map<NetId, Coord> clear_cache;
    auto clearance_to = [&](NetId other) -> Coord {
        auto it = clear_cache.find(other);
        if (it != clear_cache.end()) return it->second;
        std::string cs;
        Coord c = resolver.requiredClearance(req.net, other, 0, ctx, &cs);
        clear_cache[other] = c;
        return c;
    };
    Coord max_clear = 0;
    for (const auto& other : board.nets) {
        if (other.id == req.net) continue;
        max_clear = std::max(max_clear, clearance_to(other.id));
    }
    std::vector<ExpObs> obstacles;
    obstacles.reserve(snap.obstacles.size());
    Coord max_dist = half_w;  // query fan-out bound for the bucket index
    for (const auto& o : snap.obstacles) {
        if (o.net == req.net) continue;  // own copper is connectable
        Coord dist = (o.net < 0 ? max_clear : clearance_to(o.net)) + half_w;
        obstacles.push_back({o.raw, o.layer, dist});
        max_dist = std::max(max_dist, dist);
    }
    ObstacleIndex index;
    index.build(obstacles, board.bounds(), max_dist);

    // Layer order is board order (deterministic). Via edges connect every
    // pair of layers at one cell with via_cost_nm per transition.
    std::vector<LayerId> layers;
    for (const auto& l : board.layers) layers.push_back(l.id);
    if (layers.empty()) return out;
    auto layer_pos = [&](LayerId id) -> int {
        for (std::size_t i = 0; i < layers.size(); ++i)
            if (layers[i] == id) return static_cast<int>(i);
        return -1;
    };
    double min_mult = 1.0;
    for (double v : req.layer_mult) min_mult = std::min(min_mult, v);
    auto layer_cost = [&](LayerId id) -> double {
        if (id >= 0 && id < static_cast<int>(req.layer_mult.size()) &&
            req.layer_mult[id] > 0)
            return req.layer_mult[id];
        return 1.0;
    };

    Rect inner = board.bounds().expanded(-half_w);
    auto cell_of = [](const LevelGrid& g, Point p) -> std::pair<int, int> {
        int ix = static_cast<int>((p.x - g.bounds.x1) / g.pitch);
        int iy = static_cast<int>((p.y - g.bounds.y1) / g.pitch);
        ix = std::clamp(ix, 0, g.nx - 1);
        iy = std::clamp(iy, 0, g.ny - 1);
        return {ix, iy};
    };

    Rect window{};
    bool has_window = false;
    constexpr Coord kInf = std::numeric_limits<Coord>::max() / 4;

    for (Coord pitch : cfg.level_pitch_nm) {
        if (pitch <= 0) continue;
        Coord bw = std::max<Coord>(1, board.bounds().width());
        Coord bh = std::max<Coord>(1, board.bounds().height());
        long double fnx = static_cast<long double>(bw) / pitch;
        long double fny = static_cast<long double>(bh) / pitch;
        if (fnx * fny > static_cast<long double>(cfg.max_grid_cells)) continue;  // skip level
        // Windowed refinement on a huge window costs more than it saves;
        // skip the level (not recorded as used) and keep the coarser
        // corridor. Full-board levels (no window yet) always run.
        if (has_window) {
            long double fwx =
                static_cast<long double>(window.width()) / pitch + 2.0L;
            long double fwy =
                static_cast<long double>(window.height()) / pitch + 2.0L;
            if (fwx * fwy > static_cast<long double>(cfg.max_window_cells)) continue;
        }
        LevelGrid g;
        g.pitch = pitch;
        g.bounds = board.bounds();
        g.nx = std::max(1, static_cast<int>(std::ceil(fnx)));
        g.ny = std::max(1, static_cast<int>(std::ceil(fny)));
        const std::size_t nl = layers.size();
        g.blocked.assign(nl * g.ny * g.nx, 0);
        // Cell-driven fill through the bucket index: each center is tested
        // against nearby obstacles only (exact integer point distance).
        for (std::size_t li = 0; li < nl; ++li) {
            for (int iy = 0; iy < g.ny; ++iy) {
                for (int ix = 0; ix < g.nx; ++ix) {
                    Point ctr = cell_center(g, ix, iy);
                    if (!inner.contains(ctr)) {
                        g.blocked[(li * g.ny + iy) * g.nx + ix] = 1;
                        continue;
                    }
                    bool hit = false;
                    index.for_each(cell_rect(g, ix, iy).expanded(max_dist),
                                   [&](int oi) {
                                       if (hit) return;
                                       const ExpObs& o = obstacles[oi];
                                       if (!layer_match(o.layer, layers[li])) return;
                                       if (point_rect_gap(ctr, o.raw) < o.dist_min_nm)
                                           hit = true;
                                   });
                    if (hit) g.blocked[(li * g.ny + iy) * g.nx + ix] = 1;
                }
            }
        }

        // Soft planning-cost table, sampled once per cell (point
        // segment at the center). Moves below do O(1) lookups instead of
        // re-sampling congestion/reservations per edge. Guidance only.
        std::vector<Coord> soft;
        if (req.soft_cost) {
            soft.assign(static_cast<std::size_t>(g.nx) * g.ny, 0);
            for (int iy = 0; iy < g.ny; ++iy)
                for (int ix = 0; ix < g.nx; ++ix) {
                    Point ctr = cell_center(g, ix, iy);
                    soft[static_cast<std::size_t>(iy) * g.nx + ix] =
                        req.soft_cost(Segment{ctr, ctr});
                }
        }

        auto [six, siy] = cell_of(g, req.src);
        int sli = layer_pos(req.src_layer);
        if (sli < 0) sli = 0;
        struct GoalCell {
            int ix, iy, li;
        };
        std::vector<GoalCell> goals;
        for (const auto& d : req.dsts) {
            auto [gx, gy] = cell_of(g, d.p);
            int gli = layer_pos(d.layer);
            if (gli < 0) gli = sli;
            goals.push_back({gx, gy, gli});
        }
        // Endpoints are always traversable (mirrors ensure_node in the
        // sparse builder: a blocked endpoint is an explicit failure, not a
        // missing-node internal error).
        g.blocked[(sli * g.ny + siy) * g.nx + six] = 0;
        for (const auto& gl : goals)
            g.blocked[(gl.li * g.ny + gl.iy) * g.nx + gl.ix] = 0;

        auto is_goal = [&](int ix, int iy, int li) {
            for (const auto& gl : goals)
                if (gl.ix == ix && gl.iy == iy && gl.li == li) return true;
            return false;
        };
        // Finer levels search only inside/near the coarser window.
        auto cell_allowed = [&](int ix, int iy) {
            if (!has_window) return true;
            if (window.contains(cell_center(g, ix, iy))) return true;
            if (ix == six && iy == siy) return true;
            for (const auto& gl : goals)
                if (gl.ix == ix && gl.iy == iy) return true;
            return false;
        };

        auto heuristic = [&](int ix, int iy) -> Coord {
            Coord best = kInf;
            for (const auto& gl : goals) {
                Coord h = std::llabs(static_cast<long long>(ix - gl.ix)) * pitch +
                          std::llabs(static_cast<long long>(iy - gl.iy)) * pitch;
                if (h < best) best = h;
            }
            if (best == kInf) best = 0;
            return static_cast<Coord>(best * min_mult);
        };

        const std::size_t ncell = static_cast<std::size_t>(g.nx) * g.ny;
        std::vector<Coord> dist(nl * ncell, kInf);
        struct Came {
            int ix = -1, iy = -1, li = -1;
        };
        std::vector<Came> came(nl * ncell, {-1, -1, -1});
        auto idx = [&](int ix, int iy, int li) {
            return (static_cast<std::size_t>(li) * g.ny + iy) * g.nx + ix;
        };
        using Entry = std::tuple<Coord, Coord, int, int, int>;  // f,g,iy,ix,li
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
        dist[idx(six, siy, sli)] = 0;
        pq.push({heuristic(six, siy), 0, siy, six, sli});
        std::int64_t expansions = 0;
        int fix = -1, fiy = -1, fli = -1;
        constexpr int kDx[4] = {1, 0, -1, 0};
        constexpr int kDy[4] = {0, 1, 0, -1};
        while (!pq.empty()) {
            auto [f, gg, iy, ix, li] = pq.top();
            pq.pop();
            if (gg != dist[idx(ix, iy, li)]) continue;
            if (is_goal(ix, iy, li)) {
                fix = ix;
                fiy = iy;
                fli = li;
                break;
            }
            if (++expansions > cfg.max_coarse_expansions) {
                fix = -1;
                break;
            }
            // Planar moves (fixed order +x,+y,-x,-y). The centerline
            // between the two cells must keep clearance (exact integer
            // segment test): without this, coarse cells tunnel through
            // thin walls and the refinement window is meaningless.
            for (int d = 0; d < 4; ++d) {
                int nx = ix + kDx[d], ny = iy + kDy[d];
                if (nx < 0 || ny < 0 || nx >= g.nx || ny >= g.ny) continue;
                if (!cell_allowed(nx, ny)) continue;
                if (g.blocked[idx(nx, ny, li)]) continue;
                Segment move{cell_center(g, ix, iy), cell_center(g, nx, ny)};
                bool crosses = false;
                index.for_each(move.bounds().expanded(max_dist), [&](int oi) {
                    if (crosses) return;
                    const ExpObs& o = obstacles[oi];
                    if (!layer_match(o.layer, layers[li])) return;
                    if (!move.bounds().expanded(o.dist_min_nm).intersects(o.raw))
                        return;
                    __int128 d2 = seg_rect_dist2(move, o.raw);
                    __int128 need = (__int128)o.dist_min_nm * o.dist_min_nm;
                    if (d2 < need) crosses = true;
                });
                if (crosses) continue;
                Coord step = static_cast<Coord>(pitch * layer_cost(layers[li]));
                if (!soft.empty()) {
                    std::size_t a = static_cast<std::size_t>(iy) * g.nx + ix;
                    std::size_t bb = static_cast<std::size_t>(ny) * g.nx + nx;
                    step += (soft[a] + soft[bb]) / 2;
                }
                Coord ng = gg + step;
                if (ng < dist[idx(nx, ny, li)]) {
                    dist[idx(nx, ny, li)] = ng;
                    came[idx(nx, ny, li)] = {ix, iy, li};
                    Coord h = heuristic(nx, ny);
                    pq.push({ng + h, ng, ny, nx, li});
                }
            }
            // Via transitions (layer order, every pair like the sparse via fan).
            for (std::size_t lj = 0; lj < nl; ++lj) {
                if ((int)lj == li) continue;
                if (!cell_allowed(ix, iy)) continue;
                if (g.blocked[idx(ix, iy, (int)lj)]) continue;
                Coord step = req.astar_cfg.via_cost_nm;
                if (!soft.empty())
                    step += soft[static_cast<std::size_t>(iy) * g.nx + ix];
                Coord ng = gg + step;
                if (ng < dist[idx(ix, iy, (int)lj)]) {
                    dist[idx(ix, iy, (int)lj)] = ng;
                    came[idx(ix, iy, (int)lj)] = {ix, iy, li};
                    Coord h = heuristic(ix, iy);
                    pq.push({ng + h, ng, iy, ix, (int)lj});
                }
            }
        }
        out.coarse_expansions += expansions;
        out.levels_used_mm.push_back(nm_to_mm(pitch));
        // A failed level seeds no window, but finer levels still run: when
        // no window exists they search full-board, so a corridor narrower
        // than the coarse pitch (e.g. a 0.1mm slot) is discovered by the
        // level that resolves it. Only all-level failure falls back.
        if (fix < 0) continue;
        // Reconstruct cell-center path.
        std::vector<Point> path;
        std::vector<int> path_layers;
        int cx = fix, cy = fiy, cl = fli;
        path.push_back(cell_center(g, cx, cy));
        path_layers.push_back(layers[cl]);
        while (!(cx == six && cy == siy && cl == sli)) {
            Came c = came[idx(cx, cy, cl)];
            if (c.ix < 0) break;
            cx = c.ix;
            cy = c.iy;
            cl = c.li;
            path.push_back(cell_center(g, cx, cy));
            path_layers.push_back(layers[cl]);
        }
        std::reverse(path.begin(), path.end());
        out.level_paths.push_back(std::move(path));
        out.level_layers = std::move(path_layers);  // finest so far wins below
        out.found = true;
        out.finest_pitch_nm = pitch;
        // Refinement window around this corridor for the next finer level.
        Rect w = cell_rect(g, six, siy);
        const auto& done_path = out.level_paths.back();
        for (const auto& p : done_path) {
            Rect cell{p.x - pitch / 2, p.y - pitch / 2, p.x + pitch / 2,
                      p.y + pitch / 2};
            w = {std::min(w.x1, cell.x1), std::min(w.y1, cell.y1),
                 std::max(w.x2, cell.x2), std::max(w.y2, cell.y2)};
        }
        Coord pad = cfg.window_margin_cells * pitch + req.clearance_nm + half_w;
        window = w.expanded(pad).clamped(board.bounds());
        has_window = true;
    }
    // Only the finest successful path (level_paths.back) feeds the exact
    // window/bias; earlier paths are corridor history.
    if (out.found && has_window) {
        out.final_window = window;
        out.has_window = true;
    }
    return out;
}

}  // namespace copperline
