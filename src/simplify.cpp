#include "router/simplify.h"

#include <algorithm>

namespace copperline {

namespace {

Coord max_clear_for_net(const Board& board, const RuleResolver& resolver, NetId net,
                        const ElectricalContext& ctx) {
    Coord m = 0;
    std::string cs;
    for (const auto& o : board.nets) {
        if (o.id == net) continue;
        m = std::max(m, resolver.requiredClearance(net, o.id, 0, ctx, &cs));
    }
    return m;
}

bool all_legal(const Board& board, const RuleResolver& resolver, NetId net, LayerId layer,
               Coord width_nm, const std::vector<Point>& pts, const ElectricalContext& ctx) {
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        if (pts[i] == pts[i + 1]) continue;
        if (!simplify_segment_legal(board, resolver, net, layer, width_nm,
                                    {pts[i], pts[i + 1]}, ctx))
            return false;
    }
    return true;
}

// Exact collinearity of b on the ray a->c (cross == 0, same direction).
bool collinear_middle(Point a, Point b, Point c) {
    Coord ux = b.x - a.x, uy = b.y - a.y;
    Coord vx = c.x - b.x, vy = c.y - b.y;
    if ((__int128)ux * vy != (__int128)uy * vx) return false;
    if ((__int128)ux * vx + (__int128)uy * vy <= 0) return false;
    return b.x >= std::min(a.x, c.x) && b.x <= std::max(a.x, c.x) &&
           b.y >= std::min(a.y, c.y) && b.y <= std::max(a.y, c.y);
}

}  // namespace

bool simplify_segment_legal(const Board& board, const RuleResolver& resolver, NetId net,
                            LayerId layer, Coord width_nm, const Segment& s,
                            const ElectricalContext& ctx) {
    return simplify_segment_legal_except(board, resolver, net, layer, width_nm, s, ctx, -1);
}

bool simplify_segment_legal_except(const Board& board, const RuleResolver& resolver,
                                   NetId net, LayerId layer, Coord width_nm,
                                   const Segment& s, const ElectricalContext& ctx,
                                   NetId exempt_net) {
    Coord hw = width_nm / 2;
    if (!board.bounds().contains(s.bounds().expanded(hw))) return false;
    std::string cs;
    Coord max_clear = max_clear_for_net(board, resolver, net, ctx);
    // Keepouts: two parts. (1) Verifier-exact: the verifier flags even
    // closed-interval touch of the copper bbox, so the hw-expanded bbox
    // must be strictly disjoint from the raw rect. (2) Policy margin: the
    // centerline additionally keeps max_clear + half width, with touch at
    // exactly the margin allowed (copper then stands max_clear off the
    // keepout, which verifies clean). Part 2 mirrors the graph/arbiter
    // predicate so raw Manhattan legs stay simplifiable.
    for (const auto& ko : board.keepouts) {
        if (ko.layer != kAllLayers && ko.layer != layer) continue;
        if (s.bounds().expanded(hw).intersects(ko.rect)) return false;
        if (max_clear > 0) {
            Coord need = max_clear + hw;
            if (!s.bounds().expanded(need).intersects(ko.rect)) continue;
            if (seg_rect_dist2(s, ko.rect) < (__int128)need * need) return false;
        }
    }
    // Foreign copper: verifier-exact centerline math (4*d2 >= rhs^2 with full
    // widths folded in, no half-width truncation) so simplified diagonals
    // always re-verify clean, even for odd-nm widths.
    for (const auto& z : board.planes) {
        if (z.net == net || z.net == exempt_net || z.layer != layer) continue;
        Coord c = resolver.requiredClearance(net, z.net, layer, ctx, &cs);
        __int128 rhs = (__int128)2 * c + width_nm;
        if (s.bounds().expanded(c + hw).intersects(z.bounds())) {
            if ((__int128)4 * plane_seg_poly_dist2(s, z.poly) < rhs * rhs) return false;
        }
    }
    for (const auto& t : board.terminals) {
        if (t.net == net || t.net == exempt_net || t.layer != layer) continue;
        Coord c = resolver.requiredClearance(net, t.net, layer, ctx, &cs);
        __int128 rhs = (__int128)2 * c + width_nm;
        Rect pr = t.pad_rect();
        if (s.bounds().expanded(c + hw).intersects(pr)) {
            if ((__int128)4 * seg_rect_dist2(s, pr) < rhs * rhs) return false;
        }
    }
    for (const auto& t : board.traces) {
        if (t.net == net || t.net == exempt_net || t.layer != layer) continue;
        Coord c = resolver.requiredClearance(net, t.net, layer, ctx, &cs);
        Segment b = t.segment();
        __int128 rhs = (__int128)2 * c + width_nm + t.width_nm;
        if (!s.bounds().expanded(c + hw + t.width_nm / 2).intersects(b.bounds()))
            continue;
        if ((__int128)4 * seg_seg_dist2(s, b) < rhs * rhs) return false;
    }
    for (const auto& v : board.vias) {
        if (v.net == net || v.net == exempt_net) continue;
        if (layer < std::min(v.top_layer, v.bottom_layer) ||
            layer > std::max(v.top_layer, v.bottom_layer))
            continue;
        Coord c = resolver.requiredClearance(net, v.net, layer, ctx, &cs);
        __int128 rhs = (__int128)2 * c + width_nm;
        Rect vr = Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm);
        if (s.bounds().expanded(c + hw).intersects(vr)) {
            if ((__int128)4 * seg_rect_dist2(s, vr) < rhs * rhs) return false;
        }
    }
    return true;
}

std::vector<Point> simplify_visibility_corners(const Board& board,
                                               const RuleResolver& resolver, NetId net,
                                               LayerId layer, Coord width_nm,
                                               const Rect& corridor,
                                               const ElectricalContext& ctx,
                                               std::size_t max_corners) {
    Coord hw = width_nm / 2;
    std::string cs;
    std::vector<Point> out;
    auto emit_expanded = [&](const Rect& raw, Coord dist) {
        if (!raw.expanded(dist).intersects(corridor)) return;
        Rect e = raw.expanded(dist);
        out.push_back({e.x1, e.y1});
        out.push_back({e.x2, e.y1});
        out.push_back({e.x2, e.y2});
        out.push_back({e.x1, e.y2});
    };
    Coord max_clear = max_clear_for_net(board, resolver, net, ctx);
    for (const auto& ko : board.keepouts) {
        if (ko.layer != kAllLayers && ko.layer != layer) continue;
        emit_expanded(ko.rect, max_clear + hw);
    }
    for (const auto& t : board.terminals) {
        if (t.net == net || t.layer != layer) continue;
        emit_expanded(t.pad_rect(), resolver.requiredClearance(net, t.net, layer, ctx, &cs) + hw);
    }
    for (const auto& t : board.traces) {
        if (t.net == net || t.layer != layer) continue;
        Coord need = resolver.requiredClearance(net, t.net, layer, ctx, &cs) + hw +
                     t.width_nm / 2;
        emit_expanded(t.segment().bounds(), need);
    }
    for (const auto& v : board.vias) {
        if (v.net == net) continue;
        if (layer < std::min(v.top_layer, v.bottom_layer) ||
            layer > std::max(v.top_layer, v.bottom_layer))
            continue;
        emit_expanded(Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm),
                      resolver.requiredClearance(net, v.net, layer, ctx, &cs) + hw);
    }
    for (const auto& z : board.planes) {
        if (z.net == net || z.layer != layer) continue;
        emit_expanded(z.bounds(), resolver.requiredClearance(net, z.net, layer, ctx, &cs) + hw);
    }
    Rect inner = board.bounds().expanded(-hw);
    std::vector<Point> kept;
    kept.reserve(out.size());
    for (auto p : out) {
        if (!inner.contains(p)) continue;
        if (!corridor.expanded(hw).contains(p)) continue;
        kept.push_back(p);
    }
    std::sort(kept.begin(), kept.end());
    kept.erase(std::unique(kept.begin(), kept.end(),
                           [](const Point& a, const Point& b) {
                               return a.x == b.x && a.y == b.y;
                           }),
               kept.end());
    if (kept.size() > max_corners) kept.resize(max_corners);
    return kept;
}

std::vector<Point> simplify_polyline(const Board& board, const RuleResolver& resolver,
                                     NetId net, LayerId layer, Coord width_nm,
                                     const std::vector<Point>& pts,
                                     const ElectricalContext& ctx,
                                     const SimplifyOptions& opt) {
    // Clean: drop consecutive duplicates (zero-length legs).
    std::vector<Point> clean;
    clean.reserve(pts.size());
    for (auto p : pts) {
        if (clean.empty() || !(clean.back() == p)) clean.push_back(p);
    }
    if (clean.size() <= 2) return clean;

    auto legal = [&](Point a, Point b) {
        if (a == b) return true;
        return simplify_segment_legal(board, resolver, net, layer, width_nm, {a, b}, ctx);
    };

    const Point src = clean.front(), dst = clean.back();
    std::vector<Point> best;

    // 1. Prefer the direct source->target segment when exactly legal.
    if (legal(src, dst)) {
        best = {src, dst};
    } else {
        // 2. Shortest legal single-bend path through a visibility vertex.
        // Deterministic: integer-rounded lengths ascending, then (x, y).
        if (opt.use_visibility) {
            Rect corridor = Rect::from_points(src, dst).expanded(width_nm / 2 + mm_to_nm(0.5));
            std::vector<Point> corners =
                simplify_visibility_corners(board, resolver, net, layer, width_nm,
                                            corridor, ctx, opt.max_corners);
            Coord best_len = 0;
            Point best_c{};
            bool have = false;
            for (auto c : corners) {
                if (c == src || c == dst) continue;
                if (!legal(src, c) || !legal(c, dst)) continue;
                Coord len = euclid_len_nm(src, c) + euclid_len_nm(c, dst);
                if (!have || len < best_len || (len == best_len && c < best_c)) {
                    have = true;
                    best_len = len;
                    best_c = c;
                }
            }
            if (have) best = {src, best_c, dst};
        }
        // 3. Greedy farthest-reach string-pull over the A* waypoints.
        if (best.empty()) {
            best.push_back(src);
            std::size_t i = 0;
            while (i + 1 < clean.size()) {
                std::size_t j = clean.size() - 1;
                while (j > i + 1 && !legal(clean[i], clean[j])) --j;
                best.push_back(clean[j]);
                i = j;
            }
        } else {
            // A single visibility bend may still shortcut further against
            // the waypoint set: pull each half independently.
            std::vector<Point> refined;
            for (std::size_t k = 0; k + 1 < best.size(); ++k) {
                // Collect waypoints inside this half's bbox for pulling.
                std::vector<Point> half{best[k]};
                Rect box = Rect::from_points(best[k], best[k + 1]);
                for (auto p : clean) {
                    if (p == best[k] || p == best[k + 1]) continue;
                    if (box.contains(p)) half.push_back(p);
                }
                half.push_back(best[k + 1]);
                std::vector<Point> pulled{half.front()};
                std::size_t i = 0;
                while (i + 1 < half.size()) {
                    std::size_t j = half.size() - 1;
                    while (j > i + 1 && !legal(half[i], half[j])) --j;
                    pulled.push_back(half[j]);
                    i = j;
                }
                for (std::size_t q = (refined.empty() ? 0 : 1); q < pulled.size(); ++q)
                    refined.push_back(pulled[q]);
            }
            if (all_legal(board, resolver, net, layer, width_nm, refined, ctx)) best = refined;
        }
    }

    // 4a. Merge exactly-collinear middles (integer-exact cross product).
    bool changed = true;
    while (changed) {
        changed = false;
        for (std::size_t i = 0; i + 2 < best.size(); ++i) {
            if (collinear_middle(best[i], best[i + 1], best[i + 2])) {
                std::vector<Point> trial = best;
                trial.erase(trial.begin() + (i + 1));
                if (all_legal(board, resolver, net, layer, width_nm, trial, ctx)) {
                    best = std::move(trial);
                    changed = true;
                    break;
                }
            }
        }
    }
    // 4b. Remove tiny jogs: a middle point dies when either adjacent leg is
    // shorter than the tiny threshold and the shortcut stays exactly legal.
    {
        Coord tiny = opt.tiny_jog_nm > 0
                         ? opt.tiny_jog_nm
                         : std::max<Coord>(width_nm / 4, mm_to_nm(0.005));
        __int128 tiny2 = (__int128)tiny * tiny;
        bool jog_changed = true;
        while (jog_changed) {
            jog_changed = false;
            for (std::size_t i = 0; i + 2 < best.size(); ++i) {
                __int128 dx1 = (__int128)best[i + 1].x - best[i].x;
                __int128 dy1 = (__int128)best[i + 1].y - best[i].y;
                __int128 dx2 = (__int128)best[i + 2].x - best[i + 1].x;
                __int128 dy2 = (__int128)best[i + 2].y - best[i + 1].y;
                if (dx1 * dx1 + dy1 * dy1 > tiny2 && dx2 * dx2 + dy2 * dy2 > tiny2)
                    continue;
                std::vector<Point> trial = best;
                trial.erase(trial.begin() + (i + 1));
                if (all_legal(board, resolver, net, layer, width_nm, trial, ctx)) {
                    best = std::move(trial);
                    jog_changed = true;
                    break;
                }
            }
        }
    }

    // 5. Exact gate: any illegal leg retains the prior legal geometry.
    if (!all_legal(board, resolver, net, layer, width_nm, best, ctx)) return clean;
    return best;
}

SimplifyStats simplify_candidate_traces(const Board& snapshot, const RuleResolver& resolver,
                                        const ElectricalContext& ctx, NetId net,
                                        std::vector<TraceSeg>& traces,
                                        const std::vector<char>& is_stub, bool tuning_exempt,
                                        const SimplifyOptions& opt) {
    SimplifyStats st;
    st.segments_before = static_cast<int>(traces.size());
    {
        int b = 0;
        for (std::size_t i = 0; i + 1 < traces.size(); ++i) {
            if (traces[i].layer != traces[i + 1].layer) {
                ++b;  // layer transition counts as a bend for reporting
                continue;
            }
            Coord ux = traces[i].b.x - traces[i].a.x, uy = traces[i].b.y - traces[i].a.y;
            Coord vx = traces[i + 1].b.x - traces[i + 1].a.x,
                  vy = traces[i + 1].b.y - traces[i + 1].a.y;
            if ((__int128)ux * vy != (__int128)uy * vx) ++b;
        }
        st.bends_before = b;
    }
    if (tuning_exempt) {
        st.segments_after = st.segments_before;
        st.bends_after = st.bends_before;
        st.skipped_exempt = true;
        return st;
    }
    // Split route geometry (non-stub) into contiguous same-layer runs.
    // Stubs keep their positions: rebuilt in path order as route + stubs.
    std::vector<TraceSeg> route_only;
    std::vector<TraceSeg> stubs_only;
    for (std::size_t i = 0; i < traces.size(); ++i) {
        bool stub = i < is_stub.size() && is_stub[i];
        if (stub)
            stubs_only.push_back(traces[i]);
        else
            route_only.push_back(traces[i]);
    }
    std::vector<TraceSeg> simplified;
    simplified.reserve(route_only.size() + stubs_only.size());
    std::size_t k = 0;
    while (k < route_only.size()) {
        std::size_t m = k + 1;
        while (m < route_only.size() && route_only[m].layer == route_only[k].layer &&
               route_only[m].a == route_only[m - 1].b)
            ++m;
        // Run [k, m): one layer, contiguous.
        std::vector<Point> pts{route_only[k].a};
        for (std::size_t q = k; q < m; ++q) pts.push_back(route_only[q].b);
        Coord w = route_only[k].width_nm;
        LayerId layer = route_only[k].layer;
        std::vector<Point> out = simplify_polyline(snapshot, resolver, net, layer, w, pts,
                                                   ctx, opt);
        if (out.size() == 2 && pts.size() > 2 &&
            !(out.front() == pts.front() && out.back() == pts.back())) {
            // Unreachable: endpoints are fixed by contract; kept for safety.
        }
        if (pts.size() == 2 && out.size() == 2 && out.front() == out.back()) {
            // Degenerate; drop (original code skipped zero-length legs).
        } else {
            for (std::size_t q = 0; q + 1 < out.size(); ++q) {
                if (out[q] == out[q + 1]) continue;
                simplified.push_back({net, layer, out[q], out[q + 1], w});
            }
        }
        if (out.size() == 2 && pts.size() > 2) st.direct_used = true;
        k = m;
    }
    // Exact gate over the merged result (route simplified + stubs verbatim):
    // any illegal route leg restores the input verbatim.
    std::vector<TraceSeg> merged = simplified;
    merged.insert(merged.end(), stubs_only.begin(), stubs_only.end());
    bool ok = true;
    for (const auto& t : simplified) {
        if (!simplify_segment_legal(snapshot, resolver, net, t.layer, t.width_nm,
                                    t.segment(), ctx)) {
            ok = false;
            break;
        }
    }
    if (!ok) {
        st.segments_after = st.segments_before;
        st.bends_after = st.bends_before;
        st.retained_prior = true;
        return st;
    }
    traces = std::move(merged);
    st.segments_after = static_cast<int>(traces.size());
    {
        int b = 0;
        for (std::size_t i = 0; i + 1 < simplified.size(); ++i) {
            if (simplified[i].layer != simplified[i + 1].layer) {
                ++b;
                continue;
            }
            Coord ux = simplified[i].b.x - simplified[i].a.x,
                  uy = simplified[i].b.y - simplified[i].a.y;
            Coord vx = simplified[i + 1].b.x - simplified[i + 1].a.x,
                  vy = simplified[i + 1].b.y - simplified[i + 1].a.y;
            if ((__int128)ux * vy != (__int128)uy * vx) ++b;
        }
        st.bends_after = b;
    }
    return st;
}

bool simplify_escape_traces(const Board& work, const RuleResolver& resolver,
                            const ElectricalContext& ctx, NetId net,
                            std::vector<TraceSeg>& traces, const SimplifyOptions& opt) {
    if (traces.size() <= 1) return true;
    std::vector<char> no_stub(traces.size(), 0);
    SimplifyStats st =
        simplify_candidate_traces(work, resolver, ctx, net, traces, no_stub,
                                  /*tuning_exempt=*/false, opt);
    return !st.retained_prior;
}

}  // namespace copperline
