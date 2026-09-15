#include "router/simplify.h"

#include <algorithm>
#include <cmath>
#include <map>

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
    // S4: one shared clearance cache for the whole run, not one scan per leg.
    SegLegalityCtx leg(board, resolver, ctx, net, layer, width_nm);
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        if (pts[i] == pts[i + 1]) continue;
        if (!leg.segment_legal({pts[i], pts[i + 1]})) return false;
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
    // D3: the public entry is the shared verifier-exact probe (same predicate,
    // per-call cache). Hot paths build one SegLegalityCtx per task instead.
    SegLegalityCtx leg(board, resolver, ctx, net, layer, width_nm, exempt_net);
    return leg.segment_legal(s);
}

bool SegLegalityCtx::segment_legal(const Segment& s, std::string* why) const {
    const Board& board = *cc.board;
    NetId net = cc.net;
    Coord hw = half();
    auto fail = [&](const std::string& r) {
        if (why) *why = r;
        return false;
    };
    if (!board.bounds().contains(s.bounds().expanded(hw))) return fail("off_board");
    Coord max_clear = cc.max_clear();
    // Keepouts: two parts. (1) Verifier-exact: the verifier flags even
    // closed-interval touch of the copper bbox, so the hw-expanded bbox
    // must be strictly disjoint from the raw rect. (2) Policy margin: the
    // centerline additionally keeps max_clear + half width, with touch at
    // exactly the margin allowed (copper then stands max_clear off the
    // keepout, which verifies clean). Part 2 mirrors the graph/arbiter
    // predicate so raw Manhattan legs stay simplifiable.
    for (const auto& ko : board.keepouts) {
        if (ko.layer != kAllLayers && ko.layer != layer) continue;
        if (s.bounds().expanded(hw).intersects(ko.rect)) return fail("keepout:" + ko.reason);
        if (max_clear > 0) {
            Coord need = max_clear + hw;
            if (!s.bounds().expanded(need).intersects(ko.rect)) continue;
            if (seg_rect_dist2(s, ko.rect) < (__int128)need * need)
                return fail("keepout:" + ko.reason);
        }
    }
    // Foreign copper: verifier-exact centerline math (4*d2 >= rhs^2 with full
    // widths folded in, no half-width truncation) so simplified diagonals
    // always re-verify clean, even for odd-nm widths.
    for (const auto& z : board.planes) {
        if (z.net == net || z.net == exempt_net || z.layer != layer) continue;
        Coord c = cc.get(z.net, layer);
        __int128 rhs = (__int128)2 * c + width_nm;
        if (s.bounds().expanded(c + hw).intersects(z.bounds())) {
            if ((__int128)4 * plane_seg_poly_dist2(s, z.poly) < rhs * rhs)
                return fail("clearance:plane");
        }
    }
    for (const auto& t : board.terminals) {
        if (t.net == net || t.net == exempt_net || t.layer != layer) continue;
        Coord c = cc.get(t.net, layer);
        __int128 rhs = (__int128)2 * c + width_nm;
        Rect pr = t.pad_rect();
        if (s.bounds().expanded(c + hw).intersects(pr)) {
            if ((__int128)4 * seg_rect_dist2(s, pr) < rhs * rhs)
                return fail("clearance:pad");
        }
    }
    for (const auto& t : board.traces) {
        if (t.net == net || t.net == exempt_net || t.layer != layer) continue;
        Coord c = cc.get(t.net, layer);
        Segment b = t.segment();
        __int128 rhs = (__int128)2 * c + width_nm + t.width_nm;
        if (!s.bounds().expanded(c + hw + t.width_nm / 2).intersects(b.bounds()))
            continue;
        if ((__int128)4 * seg_seg_dist2(s, b) < rhs * rhs)
            return fail("clearance:trace");
    }
    for (const auto& v : board.vias) {
        if (v.net == net || v.net == exempt_net) continue;
        if (layer < std::min(v.top_layer, v.bottom_layer) ||
            layer > std::max(v.top_layer, v.bottom_layer))
            continue;
        Coord c = cc.get(v.net, layer);
        __int128 rhs = (__int128)2 * c + width_nm;
        Rect vr = Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm);
        if (s.bounds().expanded(c + hw).intersects(vr)) {
            if ((__int128)4 * seg_rect_dist2(s, vr) < rhs * rhs)
                return fail("clearance:via");
        }
    }
    return true;
}

std::vector<Point> simplify_visibility_corners(const Board& board,
                                               const RuleResolver& resolver, NetId net,
                                               LayerId layer, Coord width_nm,
                                               const Rect& corridor,
                                               const ElectricalContext& ctx,
                                               std::size_t max_corners, Point src,
                                               Point dst) {
    Coord hw = width_nm / 2;
    // S4: one shared clearance cache for all corner emissions.
    ClearanceCache cc(board, resolver, ctx, net);
    // (point, obstacle group). Groups give the per-obstacle guarantee in
    // the cap below; every emission below assigns a fresh group id.
    std::vector<std::pair<Point, int>> out;
    auto emit_expanded = [&](const Rect& raw, Coord dist, int gid) {
        if (!raw.expanded(dist).intersects(corridor)) return;
        Rect e = raw.expanded(dist);
        out.push_back({{e.x1, e.y1}, gid});
        out.push_back({{e.x2, e.y1}, gid});
        out.push_back({{e.x2, e.y2}, gid});
        out.push_back({{e.x1, e.y2}, gid});
    };
    // Issue #10: plane legality is polygon-exact; emit expanded polygon
    // vertices (radial offset by dist) alongside the bbox-corner fallback.
    auto emit_expanded_poly = [&](const std::vector<Point>& poly, Coord dist, int gid) {
        if (poly.empty()) return;
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
            if (len > 0.5L && dist > 0) {
                long double s = static_cast<long double>(dist) / len;
                q.x = v.x + static_cast<Coord>(std::llround(dx * s));
                q.y = v.y + static_cast<Coord>(std::llround(dy * s));
            }
            out.push_back({q, gid});
        }
    };
    Coord max_clear = cc.max_clear();
    int gid = 0;
    for (const auto& ko : board.keepouts) {
        if (ko.layer != kAllLayers && ko.layer != layer) continue;
        emit_expanded(ko.rect, max_clear + hw, gid++);
    }
    for (const auto& t : board.terminals) {
        if (t.net == net || t.layer != layer) continue;
        emit_expanded(t.pad_rect(), cc.get(t.net, layer) + hw, gid++);
    }
    for (const auto& t : board.traces) {
        if (t.net == net || t.layer != layer) continue;
        Coord need = cc.get(t.net, layer) + hw + t.width_nm / 2;
        emit_expanded(t.segment().bounds(), need, gid++);
    }
    for (const auto& v : board.vias) {
        if (v.net == net) continue;
        if (layer < std::min(v.top_layer, v.bottom_layer) ||
            layer > std::max(v.top_layer, v.bottom_layer))
            continue;
        emit_expanded(Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm),
                      cc.get(v.net, layer) + hw, gid++);
    }
    for (const auto& z : board.planes) {
        if (z.net == net || z.layer != layer) continue;
        Coord dist = cc.get(z.net, layer) + hw;
        if (!z.bounds().expanded(dist).intersects(corridor)) continue;
        int g = gid++;
        emit_expanded(z.bounds(), dist, g);
        emit_expanded_poly(z.poly, dist, g);
    }
    Rect inner = board.bounds().expanded(-hw);
    // Per-group member indices after the inner/corridor filter + dedup, so
    // the cap can guarantee representation even when groups share points.
    std::map<Point, std::size_t, bool (*)(const Point&, const Point&)> index_of(
        [](const Point& a, const Point& b) { return a < b; });
    std::vector<Point> kept;
    std::vector<int> kept_gid;
    std::vector<std::vector<std::size_t>> members(gid);
    for (auto& [p, g] : out) {
        if (!inner.contains(p)) continue;
        if (!corridor.expanded(hw).contains(p)) continue;
        auto it = index_of.find(p);
        std::size_t idx;
        if (it == index_of.end()) {
            idx = kept.size();
            index_of.emplace(p, idx);
            kept.push_back(p);
            kept_gid.push_back(g);
        } else {
            idx = it->second;
            if (g < kept_gid[idx]) kept_gid[idx] = g;
        }
        if (g >= 0 && g < (int)members.size()) members[(std::size_t)g].push_back(idx);
    }
    // Dedup member lists (one obstacle can emit the same point twice).
    for (auto& m : members) {
        std::sort(m.begin(), m.end());
        m.erase(std::unique(m.begin(), m.end()), m.end());
    }
    if (kept.size() <= max_corners) return kept;
    // Issue #11: rank by detour cost / distance to Segment(src, dst),
    // not lexicographic order. Score is (detour, perpendicular dist2,
    // x, y); detour is the single-bend extra length via the corner.
    Segment task{src, dst};
    Coord base_len = euclid_len_nm(src, dst);
    struct Score {
        Coord detour = 0;
        Coord d2 = 0;
        Point p{};
        std::size_t idx = 0;
    };
    std::vector<Score> scores;
    scores.reserve(kept.size());
    for (std::size_t i = 0; i < kept.size(); ++i) {
        Coord detour = euclid_len_nm(src, kept[i]) + euclid_len_nm(kept[i], dst);
        detour = (detour >= base_len) ? (detour - base_len) : 0;
        scores.push_back({detour, point_seg_dist2(kept[i], task), kept[i], i});
    }
    auto cmp = [](const Score& a, const Score& b) {
        if (a.detour != b.detour) return a.detour < b.detour;
        if (a.d2 != b.d2) return a.d2 < b.d2;
        if (a.p.x != b.p.x) return a.p.x < b.p.x;
        return a.p.y < b.p.y;
    };
    std::vector<char> taken(kept.size(), 0);
    std::vector<Score> chosen;
    chosen.reserve(max_corners);
    // Guarantee: best corner of each locally relevant obstacle first.
    // Groups are ordered by their best score for determinism.
    std::vector<Score> group_best;
    for (auto& m : members) {
        if (m.empty()) continue;
        const Score* best = nullptr;
        for (auto idx : m) {
            const Score& s = scores[idx];
            if (best == nullptr || cmp(s, *best)) best = &s;
        }
        if (best != nullptr) group_best.push_back(*best);
    }
    std::sort(group_best.begin(), group_best.end(), cmp);
    for (auto& s : group_best) {
        if (chosen.size() >= max_corners) break;
        if (taken[s.idx]) continue;
        taken[s.idx] = 1;
        chosen.push_back(s);
    }
    if (chosen.size() < max_corners) {
        std::vector<Score> rest = scores;
        std::sort(rest.begin(), rest.end(), cmp);
        for (auto& s : rest) {
            if (chosen.size() >= max_corners) break;
            if (taken[s.idx]) continue;
            taken[s.idx] = 1;
            chosen.push_back(s);
        }
    }
    std::vector<Point> result;
    result.reserve(chosen.size());
    for (auto& s : chosen) result.push_back(s.p);
    return result;
}

std::vector<Point> simplify_visibility_corners(const Board& board,
                                               const RuleResolver& resolver, NetId net,
                                               LayerId layer, Coord width_nm,
                                               const Rect& corridor,
                                               const ElectricalContext& ctx,
                                               std::size_t max_corners) {
    // Corridor-only fallback: score against the corridor diagonal so the
    // cap still ranks by task-line proximity instead of lexicographic
    // order. The polyline path below passes the exact endpoints.
    return simplify_visibility_corners(board, resolver, net, layer, width_nm,
                                       corridor, ctx, max_corners,
                                       {corridor.x1, corridor.y1},
                                       {corridor.x2, corridor.y2});
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

    // S4: one shared clearance cache for all probes of this run.
    SegLegalityCtx leg(board, resolver, ctx, net, layer, width_nm);
    auto legal = [&](Point a, Point b) {
        if (a == b) return true;
        return leg.segment_legal({a, b});
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
                                            corridor, ctx, opt.max_corners, src, dst);
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
    {
        // S4: one shared memo for the gate; legs may vary in (layer, width)
        // so each gets a light ctx view over the same cache.
        ClearanceCache cc(snapshot, resolver, ctx, net);
        for (const auto& t : simplified) {
            SegLegalityCtx leg;
            leg.cc = cc;  // shares the memo table
            leg.exempt_net = -1;
            leg.layer = t.layer;
            leg.width_nm = t.width_nm;
            if (!leg.segment_legal(t.segment())) {
                ok = false;
                break;
            }
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
