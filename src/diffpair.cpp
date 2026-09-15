#include "router/diffpair.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

#include "router/simplify.h"
#include "router/via_bundle.h"

namespace copperline {

namespace {

// Exact pair-aware segment legality: mirrors simplify_segment_legal but
// exempts the coupled sibling net (its copper is governed by gap_nm, not
// by the voltage table). Foreign copper of every other net keeps full
// voltage clearance. Same-net copper is connectable.
bool pair_segment_legal(const Board& board, const RuleResolver& resolver, NetId net,
                        NetId sibling, LayerId layer, Coord width_nm, const Segment& s,
                        const ElectricalContext& ctx) {
    Coord hw = width_nm / 2;
    if (!board.bounds().contains(s.bounds().expanded(hw))) return false;
    std::string cs;
    Coord max_clear = 0;
    for (const auto& o : board.nets) {
        if (o.id == net || o.id == sibling) continue;
        max_clear = std::max(max_clear, resolver.requiredClearance(net, o.id, 0, ctx, &cs));
    }
    for (const auto& ko : board.keepouts) {
        if (ko.layer != kAllLayers && ko.layer != layer) continue;
        if (s.bounds().expanded(hw).intersects(ko.rect)) return false;
        if (max_clear > 0) {
            Coord need = max_clear + hw;
            if (!s.bounds().expanded(need).intersects(ko.rect)) continue;
            if (seg_rect_dist2(s, ko.rect) < (__int128)need * need) return false;
        }
    }
    for (const auto& z : board.planes) {
        if (z.net == net || z.net == sibling || z.layer != layer) continue;
        Coord c = resolver.requiredClearance(net, z.net, layer, ctx, &cs);
        __int128 rhs = (__int128)2 * c + width_nm;
        if (s.bounds().expanded(c + hw).intersects(z.bounds())) {
            if ((__int128)4 * plane_seg_poly_dist2(s, z.poly) < rhs * rhs) return false;
        }
    }
    for (const auto& t : board.terminals) {
        if (t.net == net || t.net == sibling || t.layer != layer) continue;
        Coord c = resolver.requiredClearance(net, t.net, layer, ctx, &cs);
        __int128 rhs = (__int128)2 * c + width_nm;
        Rect pr = t.pad_rect();
        if (s.bounds().expanded(c + hw).intersects(pr)) {
            if ((__int128)4 * seg_rect_dist2(s, pr) < rhs * rhs) return false;
        }
    }
    for (const auto& t : board.traces) {
        if (t.net == net || t.net == sibling || t.layer != layer) continue;
        Coord c = resolver.requiredClearance(net, t.net, layer, ctx, &cs);
        Segment b = t.segment();
        __int128 rhs = (__int128)2 * c + width_nm + t.width_nm;
        if (!s.bounds().expanded(c + hw + t.width_nm / 2).intersects(b.bounds())) continue;
        if ((__int128)4 * seg_seg_dist2(s, b) < rhs * rhs) return false;
    }
    for (const auto& v : board.vias) {
        if (v.net == net || v.net == sibling) continue;
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

bool pair_via_legal(const Board& board, const RuleResolver& resolver, NetId net,
                    NetId sibling, const Via& vv, const std::vector<Via>& extra_self,
                    const std::vector<Via>& extra_sibling,
                    const ElectricalContext& ctx) {
    Rect vr = Rect::from_center_size(vv.pos, vv.outer_d_nm, vv.outer_d_nm);
    if (!board.bounds().contains(vr)) return false;
    std::string cs;
    Coord max_clear = 0;
    for (const auto& o : board.nets) {
        if (o.id == net || o.id == sibling) continue;
        max_clear = std::max(max_clear, resolver.requiredClearance(net, o.id, 0, ctx, &cs));
    }
    auto gap_rect = [&](const Rect& raw, Coord need) {
        if (!vr.expanded(need).intersects(raw)) return true;
        Coord dx = 0, dy = 0;
        if (vr.x2 < raw.x1) dx = raw.x1 - vr.x2;
        else if (raw.x2 < vr.x1) dx = vr.x1 - raw.x2;
        if (vr.y2 < raw.y1) dy = raw.y1 - vr.y2;
        else if (raw.y2 < vr.y1) dy = vr.y1 - raw.y2;
        __int128 d2 = (__int128)dx * dx + (__int128)dy * dy;
        return d2 >= (__int128)need * need;
    };
    for (const auto& ko : board.keepouts) {
        bool span_hit = ko.layer == kAllLayers ||
                        (ko.layer >= std::min(vv.top_layer, vv.bottom_layer) &&
                         ko.layer <= std::max(vv.top_layer, vv.bottom_layer));
        if (!span_hit) continue;
        if (!gap_rect(ko.rect, max_clear)) return false;
    }
    for (const auto& z : board.planes) {
        if (z.net == net || z.net == sibling) continue;
        if (z.layer < std::min(vv.top_layer, vv.bottom_layer) ||
            z.layer > std::max(vv.top_layer, vv.bottom_layer))
            continue;
        Coord c = resolver.requiredClearance(net, z.net, z.layer, ctx, &cs);
        if (vr.expanded(c).intersects(z.bounds())) {
            if (plane_rect_poly_dist2(vr, z.poly) < (__int128)c * c) return false;
        }
    }
    for (const auto& t : board.terminals) {
        if (t.net == net || t.net == sibling) continue;
        bool span_hit = t.layer >= std::min(vv.top_layer, vv.bottom_layer) &&
                        t.layer <= std::max(vv.top_layer, vv.bottom_layer);
        if (!span_hit) continue;
        Coord c = resolver.requiredClearance(net, t.net, t.layer, ctx, &cs);
        if (!gap_rect(t.pad_rect(), c)) return false;
    }
    auto check_trace = [&](const TraceSeg& t) {
        if (t.net == net || t.net == sibling) return true;
        if (t.layer < std::min(vv.top_layer, vv.bottom_layer) ||
            t.layer > std::max(vv.top_layer, vv.bottom_layer))
            return true;
        Coord c = resolver.requiredClearance(net, t.net, t.layer, ctx, &cs);
        __int128 d2 = seg_rect_dist2(t.segment(), vr);
        Coord need = c + t.width_nm / 2;
        return d2 >= (__int128)need * need;
    };
    for (const auto& t : board.traces)
        if (!check_trace(t)) return false;
    auto check_via = [&](const Via& o) {
        if (o.net == net || o.net == sibling) return true;
        bool overlap = !(o.bottom_layer < std::min(vv.top_layer, vv.bottom_layer) ||
                         o.top_layer > std::max(vv.top_layer, vv.bottom_layer));
        if (!overlap) return true;
        Rect orr = Rect::from_center_size(o.pos, o.outer_d_nm, o.outer_d_nm);
        Coord c = resolver.requiredClearance(net, o.net, vv.top_layer, ctx, &cs);
        return gap_rect(orr, c);
    };
    for (const auto& o : board.vias)
        if (!check_via(o)) return false;
    for (const auto& o : extra_self)
        if (!check_via(o)) return false;
    for (const auto& o : extra_sibling) {
        // Sibling vias are governed by the pair gap, checked separately;
        // here only ensure they do not overlap exactly (checked by gap).
        (void)o;
    }
    return true;
}

// Chain unordered same-layer segments into one polyline (path order).
// Returns points; empty when chaining fails (caller falls back to raw order).
std::vector<Point> chain_run(const std::vector<TraceSeg>& segs) {
    if (segs.empty()) return {};
    if (segs.size() == 1) return {segs[0].a, segs[0].b};
    std::multimap<std::pair<Coord, Coord>, std::size_t> at;
    for (std::size_t i = 0; i < segs.size(); ++i) {
        at.insert({{segs[i].a.x, segs[i].a.y}, i});
        at.insert({{segs[i].b.x, segs[i].b.y}, i});
    }
    // Start at an endpoint with degree 1 (deterministic: smallest point).
    std::map<std::pair<Coord, Coord>, int> deg;
    for (const auto& s : segs) {
        deg[{s.a.x, s.a.y}]++;
        deg[{s.b.x, s.b.y}]++;
    }
    std::pair<Coord, Coord> start = {segs[0].a.x, segs[0].a.y};
    for (const auto& [p, d] : deg) {
        if (d == 1) {
            start = p;
            break;
        }
    }
    std::vector<Point> pts{{start.first, start.second}};
    std::vector<char> used(segs.size(), 0);
    std::pair<Coord, Coord> cur = start;
    for (std::size_t k = 0; k < segs.size(); ++k) {
        bool found = false;
        for (std::size_t i = 0; i < segs.size(); ++i) {
            if (used[i]) continue;
            const auto& s = segs[i];
            if (s.a.x == cur.first && s.a.y == cur.second) {
                pts.push_back(s.b);
                cur = {s.b.x, s.b.y};
                used[i] = 1;
                found = true;
                break;
            }
            if (s.b.x == cur.first && s.b.y == cur.second) {
                pts.push_back(s.a);
                cur = {s.a.x, s.a.y};
                used[i] = 1;
                found = true;
                break;
            }
        }
        if (!found) return {};  // forked/disjoint: caller keeps raw order
    }
    return pts;
}

}  // namespace

bool diffpair_valid(const Board& board, const DiffPair& pair, std::string& reason_out) {
    if (!board.find_net(pair.net_p)) {
        reason_out = "unknown_p_net";
        return false;
    }
    if (!board.find_net(pair.net_n)) {
        reason_out = "unknown_n_net";
        return false;
    }
    if (pair.net_p == pair.net_n) {
        reason_out = "p_equals_n";
        return false;
    }
    if (pair.gap_nm <= 0) {
        reason_out = "gap_must_be_positive";
        return false;
    }
    if (pair.gap_tol_nm < 0 || pair.gap_tol_nm > pair.gap_nm) {
        reason_out = "bad_gap_tolerance";
        return false;
    }
    if (pair.has_width && pair.width_nm <= 0) {
        reason_out = "bad_pair_width";
        return false;
    }
    for (LayerId l : pair.preferred_layers) {
        if (!board.valid_layer(l)) {
            reason_out = "unknown_preferred_layer";
            return false;
        }
    }
    if (pair.has_impedance && pair.target_impedance_ohms <= 0) {
        reason_out = "bad_impedance_target";
        return false;
    }
    if (pair.has_max_skew && pair.max_skew_nm < 0) {
        reason_out = "bad_max_skew";
        return false;
    }
    if (pair.via_policy != "paired" && pair.via_policy != "independent") {
        reason_out = "bad_via_policy";
        return false;
    }
    reason_out = "ok";
    return true;
}

bool diffpair_endpoints(const Board& board, const DiffPair& pair, TermId& pa,
                        TermId& pb, TermId& na, TermId& nb, std::string& reason_out) {
    const NetInfo* p = board.find_net(pair.net_p);
    const NetInfo* n = board.find_net(pair.net_n);
    if (!p || !n) {
        reason_out = "unknown_net";
        return false;
    }
    if (p->terminals.size() != 2 || n->terminals.size() != 2) {
        reason_out = "pair_members_must_be_2_terminal";
        return false;
    }
    std::vector<TermId> pt = p->terminals, nt = n->terminals;
    std::sort(pt.begin(), pt.end());
    std::sort(nt.begin(), nt.end());
    pa = pt[0];
    pb = pt[1];
    na = nt[0];
    nb = nt[1];
    reason_out = "ok";
    return true;
}

Coord diffpair_member_width(const Board& board, const RuleResolver& resolver,
                            const DiffPair& pair, NetId member, LayerId layer,
                            const ElectricalContext& ctx) {
    (void)board;
    std::string src;
    Coord w_req = resolver.requiredTraceWidth(member, layer, ctx, &src);
    if (pair.has_width) return std::max(pair.width_nm, w_req);
    return w_req;
}

Coord diffpair_external_clearance(const Board& board, const RuleResolver& resolver,
                                  const DiffPair& pair, const ElectricalContext& ctx) {
    Coord m = 0;
    std::string cs;
    for (const auto& o : board.nets) {
        if (o.id == pair.net_p || o.id == pair.net_n) continue;
        m = std::max(m, resolver.requiredClearance(pair.net_p, o.id, 0, ctx, &cs));
        m = std::max(m, resolver.requiredClearance(pair.net_n, o.id, 0, ctx, &cs));
    }
    return m;
}

Coord diffpair_occupied_width(const Board& board, const RuleResolver& resolver,
                              const DiffPair& pair, const ElectricalContext& ctx) {
    // Size on the source layer; impedance-controlled members use the
    // conservative max-across-layers width so the corridor fits everywhere.
    const Terminal* tp = nullptr;
    TermId pa = -1, pb = -1, na = -1, nb = -1;
    std::string dummy;
    LayerId layer = 0;
    if (diffpair_endpoints(board, pair, pa, pb, na, nb, dummy)) {
        const Terminal* t = board.find_terminal(pa);
        if (t) layer = t->layer;
        tp = t;
    } else if (!board.layers.empty()) {
        layer = board.layers.front().id;
    }
    (void)tp;
    const NetInfo* p = board.find_net(pair.net_p);
    const NetInfo* n = board.find_net(pair.net_n);
    Coord wp = 0, wn = 0;
    if (p && resolver.impedance().has_target(*p))
        wp = resolver.maxRequiredWidth(pair.net_p, ctx);
    else
        wp = diffpair_member_width(board, resolver, pair, pair.net_p, layer, ctx);
    if (n && resolver.impedance().has_target(*n))
        wn = resolver.maxRequiredWidth(pair.net_n, ctx);
    else
        wn = diffpair_member_width(board, resolver, pair, pair.net_n, layer, ctx);
    if (pair.has_width) {
        // Explicit pair width still floors at the reconciled member widths.
        wp = std::max(wp, pair.width_nm);
        wn = std::max(wn, pair.width_nm);
        // ...and never below either member's electrical minimum (already in
        // diffpair_member_width when has_width is set, kept explicit here).
    }
    Coord ext = diffpair_external_clearance(board, resolver, pair, ctx);
    return wp + wn + pair.gap_nm + 2 * ext;
}

bool make_pair_corridor_task(const Board& board, const DiffPair& pair, int index,
                             ConnectionTask& out, std::string& reason_out) {
    std::string why;
    if (!diffpair_valid(board, pair, why)) {
        reason_out = why;
        return false;
    }
    TermId pa, pb, na, nb;
    if (!diffpair_endpoints(board, pair, pa, pb, na, nb, why)) {
        reason_out = why;
        return false;
    }
    const Terminal* tp = board.find_terminal(pa);
    const Terminal* tn = board.find_terminal(na);
    const Terminal* tp2 = board.find_terminal(pb);
    const Terminal* tn2 = board.find_terminal(nb);
    if (!tp || !tn || !tp2 || !tn2) {
        reason_out = "missing_terminal";
        return false;
    }
    if (tp->layer != tn->layer || tp2->layer != tn2->layer) {
        reason_out = "pair_layer_mismatch";
        return false;
    }
    ConnectionTask t;
    t.net = pair.net_p;
    t.a = pa;
    t.b = pb;
    t.index = index;
    t.is_pair_corridor = true;
    t.pair_id = pair.id;
    t.pair_other_net = pair.net_n;
    t.pair_a_other = na;
    t.pair_b_other = nb;
    out = t;
    reason_out = "ok";
    return true;
}

std::vector<ConnectionTask> build_global_tasks_with_pairs(
    const Board& board, std::vector<std::string>& invalid_reasons,
    std::vector<int>& invalid_pair_ids) {
    std::set<NetId> paired_nets;
    std::map<NetId, int> net_to_pair;
    for (const auto& pr : board.diffpairs) {
        paired_nets.insert(pr.net_p);
        paired_nets.insert(pr.net_n);
    }
    std::vector<ConnectionTask> out;
    // Ordinary nets first (deterministic net-id order).
    std::vector<NetId> nets;
    for (const auto& n : board.nets) nets.push_back(n.id);
    std::sort(nets.begin(), nets.end());
    for (NetId nid : nets) {
        if (paired_nets.count(nid)) continue;
        const NetInfo* n = board.find_net(nid);
        if (!n || n->terminals.size() < 2) continue;
        RouteTree tree = build_route_tree(board, nid);
        for (auto& t : tree.tasks) out.push_back(t);
    }
    // One atomic corridor task per pair, in pair-id order.
    std::vector<DiffPair> pairs = board.diffpairs;
    std::sort(pairs.begin(), pairs.end(),
              [](const DiffPair& a, const DiffPair& b) { return a.id < b.id; });
    // Disjointness: a net in two pairs invalidates the later pair.
    std::set<NetId> claimed;
    for (const auto& pr : pairs) {
        std::string why;
        if (claimed.count(pr.net_p) || claimed.count(pr.net_n)) {
            invalid_pair_ids.push_back(pr.id);
            invalid_reasons.push_back("net_shared_between_pairs");
            continue;
        }
        ConnectionTask t;
        if (!make_pair_corridor_task(board, pr, static_cast<int>(out.size()), t, why)) {
            invalid_pair_ids.push_back(pr.id);
            invalid_reasons.push_back(why);
            continue;
        }
        claimed.insert(pr.net_p);
        claimed.insert(pr.net_n);
        net_to_pair[pr.net_p] = pr.id;
        net_to_pair[pr.net_n] = pr.id;
        out.push_back(t);
    }
    // Stable indices.
    for (std::size_t i = 0; i < out.size(); ++i) out[i].index = static_cast<int>(i);
    return out;
}

Coord pair_total_length(const std::vector<TraceSeg>& traces) {
    Coord total = 0;
    for (const auto& t : traces) total += euclid_len_nm(t.a, t.b);
    return total;
}

bool pair_gap_legal(const std::vector<TraceSeg>& traces_p,
                    const std::vector<TraceSeg>& traces_n, Coord gap_nm,
                    Coord tol_nm, Coord& worst_err_out, Point& at_out) {
    worst_err_out = 0;
    bool have = false;
    Point worst_at{};
    // Minimum edge gap over all same-layer P/N pairs must clear gap - tol.
    for (std::size_t ai = 0; ai < traces_p.size(); ++ai) {
        const auto& a = traces_p[ai];
        for (std::size_t bi = 0; bi < traces_n.size(); ++bi) {
            const auto& b = traces_n[bi];
            if (a.layer != b.layer) continue;
            __int128 d2 = seg_seg_dist2(a.segment(), b.segment());
            // Edge gap G = center - (wa+wb)/2 >= gap - tol  <=>
            // 4*d2 >= (2*(gap-tol) + wa + wb)^2 ; negative RHS always passes.
            __int128 rhs = (__int128)2 * (gap_nm - tol_nm) + a.width_nm + b.width_nm;
            if (rhs < 0) continue;
            if ((__int128)4 * d2 < rhs * rhs) {
                // Report worst shortfall location (midpoint of segment starts).
                worst_err_out = gap_nm - tol_nm;  // marker: violation
                at_out = {(a.a.x + b.a.x) / 2, (a.a.y + b.a.y) / 2};
                return false;
            }
            // Track worst deviation from nominal for reporting.
            long double d = std::sqrt((long double)d2);
            long double edge = d - (a.width_nm + b.width_nm) / 2.0L;
            long double err = fabsl(edge - (long double)gap_nm);
            Coord erri = static_cast<Coord>(llround((double)err));
            if (!have || erri > worst_err_out) {
                worst_err_out = erri;
                worst_at = {(a.a.x + b.a.x) / 2, (a.a.y + b.a.y) / 2};
                have = true;
            }
        }
    }
    if (have) at_out = worst_at;
    return true;
}

MaterializedPair materialize_pair(const Board& board_without_corridor,
                                  const RuleResolver& resolver,
                                  const ElectricalContext& ctx,
                                  const DiffPair& pair,
                                  const PairCorridor& corridor) {
    MaterializedPair out;
    out.pair_id = pair.id;
    out.net_p = pair.net_p;
    out.net_n = pair.net_n;
    auto fail = [&](const std::string& r) {
        out.ok = false;
        out.reason = r;
        out.traces_p.clear();
        out.traces_n.clear();
        out.vias_p.clear();
        out.vias_n.clear();
        return out;
    };
    std::string why;
    if (!diffpair_valid(board_without_corridor, pair, why)) return fail(why);
    TermId pa, pb, na, nb;
    if (!diffpair_endpoints(board_without_corridor, pair, pa, pb, na, nb, why))
        return fail(why);
    const Terminal* tpa = board_without_corridor.find_terminal(pa);
    const Terminal* tna = board_without_corridor.find_terminal(na);
    const Terminal* tpb = board_without_corridor.find_terminal(pb);
    const Terminal* tnb = board_without_corridor.find_terminal(nb);
    if (!tpa || !tna || !tpb || !tnb) return fail("missing_terminal");
    if (tpa->layer != tna->layer || tpb->layer != tnb->layer)
        return fail("pair_layer_mismatch");
    if (corridor.center_traces.empty()) return fail("empty_corridor");

    // Member widths: per-layer reconciled (corridor may span layers).
    // Use each run's layer width; precompute per layer present.
    std::set<LayerId> layers;
    for (const auto& s : corridor.center_traces) layers.insert(s.layer);
    for (const auto& v : corridor.center_vias) {
        layers.insert(v.top_layer);
        layers.insert(v.bottom_layer);
    }
    std::map<LayerId, Coord> wp_of, wn_of;
    for (LayerId l : layers) {
        wp_of[l] = diffpair_member_width(board_without_corridor, resolver, pair,
                                         pair.net_p, l, ctx);
        wn_of[l] = diffpair_member_width(board_without_corridor, resolver, pair,
                                         pair.net_n, l, ctx);
        if (wp_of[l] <= 0 || wn_of[l] <= 0) return fail("bad_pair_width");
    }
    // Preferred-layer gate: corridor must stay within the pair's layers.
    if (!pair.preferred_layers.empty()) {
        for (const auto& s : corridor.center_traces) {
            if (std::find(pair.preferred_layers.begin(), pair.preferred_layers.end(),
                          s.layer) == pair.preferred_layers.end())
                return fail("corridor_off_preferred_layer");
        }
    }

    // Group centerline traces into per-layer runs and chain each run.
    std::map<LayerId, std::vector<TraceSeg>> by_layer;
    for (const auto& s : corridor.center_traces) by_layer[s.layer].push_back(s);
    struct Run {
        LayerId layer = 0;
        std::vector<Point> pts;
    };
    std::vector<Run> runs;
    for (auto& [layer, segs] : by_layer) {
        std::vector<Point> pts = chain_run(segs);
        if (pts.size() < 2) {
            // Fallback: raw path order (should not happen for clean corridors).
            pts.clear();
            pts.push_back(segs.front().a);
            for (const auto& s : segs) pts.push_back(s.b);
            std::vector<Point> clean;
            for (auto p : pts) {
                if (clean.empty() || !(clean.back() == p)) clean.push_back(p);
            }
            pts = clean;
        }
        if (pts.size() < 2) return fail("empty_corridor_run");
        runs.push_back({layer, pts});
    }
    std::sort(runs.begin(), runs.end(),
              [](const Run& a, const Run& b) { return a.layer < b.layer; });

    auto via_at_point = [&](Point p) -> bool {
        for (const auto& v : corridor.center_vias) {
            if (v.pos == p) return true;
        }
        return false;
    };

    Point src_mid = {(tpa->pos.x + tna->pos.x) / 2, (tpa->pos.y + tna->pos.y) / 2};
    Point dst_mid = {(tpb->pos.x + tnb->pos.x) / 2, (tpb->pos.y + tnb->pos.y) / 2};

    // Fanout insertion: when a pad-end leg leaves along (rather than across)
    // the pad-pair axis, perpendicular offsets would force the two fans
    // into a pinched V that cannot hold the gap. Insert one short
    // perpendicular jog at such ends so the pair fans out before turning
    // down the trunk. Gated by exact corridor-width legality; skipped
    // honestly when the jog does not fit (downstream checks then decide).
    {
        long double ax = (long double)tna->pos.x - tpa->pos.x;
        long double ay = (long double)tna->pos.y - tpa->pos.y;
        long double al = std::sqrt(ax * ax + ay * ay);
        Coord occupied = diffpair_occupied_width(board_without_corridor, resolver,
                                                 pair, ctx);
        for (auto& run : runs) {
            if (run.pts.size() < 2) continue;
            for (int end = 0; end < 2; ++end) {
                Point ep = (end == 0) ? run.pts.front() : run.pts.back();
                // Only true pad ends (exactly a pad midpoint, no
                // co-located corridor via).
                if (!(ep == src_mid || ep == dst_mid)) continue;
                if (via_at_point(ep)) continue;
                if (al <= 0) continue;
                Point next = (end == 0) ? run.pts[1] : run.pts[run.pts.size() - 2];
                long double dx = (long double)next.x - ep.x;
                long double dy = (long double)next.y - ep.y;
                long double L = std::sqrt(dx * dx + dy * dy);
                if (L <= 0) continue;
                long double dot = std::fabs((dx * ax + dy * ay) / (L * al));
                if (dot <= 0.7L) continue;  // already across the axis
                Coord wp = wp_of[run.layer], wn = wn_of[run.layer];
                Coord center_dist = pair.gap_nm + (wp + wn) / 2;
                // Jog perpendicular to the pad axis (i.e. across the row),
                // one center-distance long: [ep, J, next, ...].
                // Deterministic sign order; exact-gated.
                long double jx = -ay / al, jy = ax / al;
                for (int s = 0; s < 2; ++s) {
                    long double sx = (s == 0) ? jx : -jx;
                    long double sy = (s == 0) ? jy : -jy;
                    Point J{static_cast<Coord>(llround((long double)ep.x + sx * center_dist)),
                            static_cast<Coord>(llround((long double)ep.y + sy * center_dist))};
                    if (J == ep || J == next) continue;
                    Segment s1{ep, J}, s2{J, next};
                    if (!pair_segment_legal(board_without_corridor, resolver,
                                            pair.net_p, pair.net_n, run.layer,
                                            occupied, s1, ctx))
                        continue;
                    if (!pair_segment_legal(board_without_corridor, resolver,
                                            pair.net_p, pair.net_n, run.layer,
                                            occupied, s2, ctx))
                        continue;
                    if (end == 0) run.pts.insert(run.pts.begin() + 1, J);
                    else run.pts.insert(run.pts.end() - 1, J);
                    break;
                }
            }
        }
    }
    std::vector<TraceSeg> tp_all, tn_all;
    std::vector<Via> vp_all, vn_all;

    // Materialize each run independently, then stitch via pairs.
    // Offset convention: pp[] is the P member, np[] the N member. Widths
    // stay net-bound (pp segments always use the P width); the side flip
    // only swaps which array P takes, never the widths.
    struct RunPair {
        LayerId layer = 0;
        std::vector<Point> pp;
        std::vector<Point> np;
    };
    std::vector<RunPair> run_pairs;

    for (const auto& run : runs) {
        const std::vector<Point>& c = run.pts;
        Coord wp = wp_of[run.layer], wn = wn_of[run.layer];
        Coord center_dist = pair.gap_nm + (wp + wn) / 2;
        // Integer-center rounding: keep symmetric; the 1nm remainder (odd
        // sums) lands on the N side deterministically.
        Coord off_p = center_dist / 2;
        Coord off_n = center_dist - off_p;
        std::size_t n = c.size();
        std::vector<std::pair<long double, long double>> normals(n);
        auto seg_normal = [](Point a, Point b) {
            long double dx = (long double)b.x - a.x;
            long double dy = (long double)b.y - a.y;
            long double L = std::sqrt(dx * dx + dy * dy);
            if (L <= 0) return std::pair<long double, long double>(0, 0);
            return std::pair<long double, long double>(-dy / L, dx / L);
        };
        for (std::size_t i = 0; i < n; ++i) {
            if (i == 0) {
                normals[i] = seg_normal(c[0], c[1]);
            } else if (i + 1 == n) {
                normals[i] = seg_normal(c[n - 2], c[n - 1]);
            } else {
                auto n1 = seg_normal(c[i - 1], c[i]);
                auto n2 = seg_normal(c[i], c[i + 1]);
                long double mx = n1.first + n2.first;
                long double my = n1.second + n2.second;
                long double ml = std::sqrt(mx * mx + my * my);
                if (ml <= 1e-12) {
                    normals[i] = n2;  // 180-degree reversal: bevel
                } else {
                    mx /= ml;
                    my /= ml;
                    long double dot = mx * n1.first + my * n1.second;
                    if (fabsl(dot) < 0.25L) {
                        // Sharp spike: clamp miter (bevel fallback).
                        normals[i] = n2;
                    } else {
                        long double scale = 1.0L / dot;
                        if (scale > 4.0L) scale = 4.0L;
                        normals[i] = {mx * scale, my * scale};
                    }
                }
            }
        }
        std::vector<Point> pp(n), np(n);
        for (std::size_t i = 0; i < n; ++i) {
            long double nx = normals[i].first, ny = normals[i].second;
            pp[i] = {static_cast<Coord>(llround((long double)c[i].x - nx * (long double)off_p)),
                     static_cast<Coord>(llround((long double)c[i].y - ny * (long double)off_p))};
            np[i] = {static_cast<Coord>(llround((long double)c[i].x + nx * (long double)off_n)),
                     static_cast<Coord>(llround((long double)c[i].y + ny * (long double)off_n))};
        }
        // Global side check on the run containing src_mid (or any run):
        // ensure pp[0]/pp.back() side matches P pads. Use total distance to
        // P pads vs N pads for both orientations; flip when N is nearer.
        {
            auto dist2 = [](Point a, Point b) {
                __int128 dx = (__int128)a.x - b.x, dy = (__int128)a.y - b.y;
                return dx * dx + dy * dy;
            };
            // Associate run ends with src/dst by proximity to midpoints.
            __int128 d_start_src = dist2(pp.front(), src_mid) + dist2(np.front(), src_mid);
            __int128 d_end_src = dist2(pp.back(), src_mid) + dist2(np.back(), src_mid);
            bool start_is_src = d_start_src <= d_end_src;
            Point p_end = start_is_src ? pp.front() : pp.back();
            Point n_end = start_is_src ? np.front() : np.back();
            Point p_pad = start_is_src ? tpa->pos : tpb->pos;
            Point n_pad = start_is_src ? tna->pos : tnb->pos;
            __int128 p_to_p = dist2(p_end, p_pad) + dist2(n_end, n_pad);
            __int128 p_to_n = dist2(p_end, n_pad) + dist2(n_end, p_pad);
            if (p_to_n < p_to_p) {
                // Swap which array P takes (no twisting: the whole run).
                // Widths are net-bound and stay put (fixed below at emit).
                pp.swap(np);
            }
        }
        // Pad-end stitching: a run end exactly at a pad midpoint that does
        // NOT co-locate a corridor via drops its perpendicular offset jog
        // (the jog pinches the fan pair when the corridor leaves along the
        // pad axis) and starts/ends directly at the member pads. Via ends
        // keep their offsets for barrel rewiring below.
        {
            bool front_pad = false, back_pad = false;
            Point front_pp, front_np, back_pp, back_np;
            bool front_is_src = (c.front() == src_mid);
            bool front_is_dst = (c.front() == dst_mid);
            bool back_is_src = (c.back() == src_mid);
            bool back_is_dst = (c.back() == dst_mid);
            if ((front_is_src || front_is_dst) && !via_at_point(c.front())) {
                front_pad = true;
                front_pp = front_is_src ? tpa->pos : tpb->pos;
                front_np = front_is_src ? tna->pos : tnb->pos;
            }
            if ((back_is_src || back_is_dst) && !via_at_point(c.back())) {
                back_pad = true;
                back_pp = back_is_src ? tpa->pos : tpb->pos;
                back_np = back_is_src ? tna->pos : tnb->pos;
            }
            // Rebuild without the dropped perpendicular jogs (a 2-point
            // pad-to-pad run collapses to exactly the two pads).
            std::vector<Point> npp, nnp;
            if (front_pad) {
                npp.push_back(front_pp);
                nnp.push_back(front_np);
            }
            std::size_t lo = front_pad ? 1 : 0;
            std::size_t hi = pp.size() - (back_pad ? 1 : 0);
            for (std::size_t i = lo; i < hi; ++i) {
                npp.push_back(pp[i]);
                nnp.push_back(np[i]);
            }
            if (back_pad) {
                npp.push_back(back_pp);
                nnp.push_back(back_np);
            }
            // Deduplicate consecutive points (a dropped jog may coincide
            // with its pad when the corridor meets the pads head-on).
            auto dedup = [](std::vector<Point>& v) {
                std::vector<Point> o;
                for (auto p : v) {
                    if (o.empty() || !(o.back() == p)) o.push_back(p);
                }
                v.swap(o);
            };
            dedup(npp);
            dedup(nnp);
            if (npp.size() < 2 || nnp.size() < 2) return fail("empty_corridor_run");
            pp.swap(npp);
            np.swap(nnp);
        }
        run_pairs.push_back({run.layer, pp, np});
    }

    // Emit happens after via rewiring below (endpoints snap to barrel
    // bases); entry stubs are collected per via first.
    std::vector<TraceSeg> stub_p, stub_n;

    // Paired vias: one P+N pair per corridor via, symmetric stagger when
    // the trace pitch cannot fit both barrels.
    for (const auto& cv : corridor.center_vias) {
        LayerSpan span{std::min(cv.top_layer, cv.bottom_layer),
                       std::max(cv.top_layer, cv.bottom_layer)};
        ViaStyle sp, sn;
        bool okp = resolver.select_via(pair.net_p, span, sp);
        bool okn = resolver.select_via(pair.net_n, span, sn);
        if (!okp || !okn) return fail("via_infeasible");
        // Shared style: first P-ordered style legal for both nets keeps
        // barrels identical (symmetric pair). Falls back to P's style when
        // it also carries N's current.
        ViaStyle use = sp;
        {
            auto ordered = ViaBundlePlanner::ordered_styles(resolver, pair.net_p, span);
            bool found = false;
            for (const auto& st : ordered) {
                bool dummy = false;
                (void)dummy;
                const NetInfo* np = board_without_corridor.find_net(pair.net_p);
                const NetInfo* nn = board_without_corridor.find_net(pair.net_n);
                if (!np || !nn) break;
                ElectricalContext c0;
                if (!resolver.current().via_style_ok(st, *np,
                                                     board_without_corridor.defaults, c0))
                    continue;
                if (!resolver.current().via_style_ok(st, *nn,
                                                     board_without_corridor.defaults, c0))
                    continue;
                use = st;
                found = true;
                break;
            }
            if (!found) {
                // Different currents force different barrels: still pair
                // them geometrically (atomic commit), sizes differ.
                use = sp;
            }
        }
        // Corridor direction at the via from the CENTERLINE (stable under
        // the side flip): the first run holding V as an endpoint gives the
        // local direction. Deterministic: lowest layer first.
        long double ux = 1, uy = 0;
        {
            bool found = false;
            for (const auto& run : runs) {
                if (found) break;
                if (run.pts.size() < 2) continue;
                if (run.pts.front() == cv.pos && !(run.pts[1] == cv.pos)) {
                    long double dx = (long double)run.pts[1].x - cv.pos.x;
                    long double dy = (long double)run.pts[1].y - cv.pos.y;
                    long double L = std::sqrt(dx * dx + dy * dy);
                    if (L > 0) {
                        ux = dx / L;
                        uy = dy / L;
                        found = true;
                    }
                } else if (run.pts.back() == cv.pos &&
                           !(run.pts[run.pts.size() - 2] == cv.pos)) {
                    std::size_t m = run.pts.size();
                    long double dx = (long double)cv.pos.x - run.pts[m - 2].x;
                    long double dy = (long double)cv.pos.y - run.pts[m - 2].y;
                    long double L = std::sqrt(dx * dx + dy * dy);
                    if (L > 0) {
                        ux = dx / L;
                        uy = dy / L;
                        found = true;
                    }
                }
            }
            (void)found;
        }
        long double px = -uy, py = ux;
        Coord wp = wp_of.count(cv.top_layer) ? wp_of[cv.top_layer]
                                             : wp_of.begin()->second;
        Coord wn = wn_of.count(cv.top_layer) ? wn_of[cv.top_layer]
                                             : wn_of.begin()->second;
        Coord center_dist = pair.gap_nm + (wp + wn) / 2;
        Coord off_p = center_dist / 2, off_n = center_dist - off_p;
        std::string cs;
        Coord clr_pn = resolver.requiredClearance(pair.net_p, pair.net_n, cv.top_layer,
                                                  ctx, &cs);
        Coord need = std::max(pair.gap_nm - pair.gap_tol_nm, clr_pn);
        if (need < 0) need = 0;
        Coord outer = use.outer_nm;
        // Perpendicular barrel separation (square-barrel model shared with
        // the verifier/arbiter): the barrels fan apart perpendicular to the
        // corridor instead of staggering along-track. Along-track stagger
        // parks one barrel over the opposite member's run (which spans the
        // whole corridor), while perpendicular separation keeps every
        // barrel clear of opposite copper. Needed center distance D satisfies
        // D - outer >= need, i.e. D >= outer + need; extra Total = D - pitch
        // splits evenly (deterministic 1nm remainder to N).
        Coord perp_dist = off_p + off_n;
        Coord want = outer + need;
        Coord extra_total = want > perp_dist ? want - perp_dist : 0;
        Coord extra_p = extra_total / 2, extra_n = extra_total - extra_p;
        // Unstaggered bases keep the runs parallel (no crossing); the
        // barrels connect through short parallel entry stubs.
        Point bp{static_cast<Coord>(llround((long double)cv.pos.x - px * off_p)),
                 static_cast<Coord>(llround((long double)cv.pos.y - py * off_p))};
        Point bn{static_cast<Coord>(llround((long double)cv.pos.x + px * off_n)),
                 static_cast<Coord>(llround((long double)cv.pos.y + py * off_n))};
        Point pv{static_cast<Coord>(llround((long double)cv.pos.x - px * (off_p + extra_p))),
                 static_cast<Coord>(llround((long double)cv.pos.y - py * (off_p + extra_p)))};
        Point nv{static_cast<Coord>(llround((long double)cv.pos.x + px * (off_n + extra_n))),
                 static_cast<Coord>(llround((long double)cv.pos.y + py * (off_n + extra_n)))};
        Via vvp, vvn;
        vvp.net = pair.net_p;
        vvp.pos = pv;
        vvp.top_layer = span.top;
        vvp.bottom_layer = span.bottom;
        vvp.outer_d_nm = use.outer_nm;
        vvp.hole_d_nm = use.hole_nm;
        vvp.via_class = use.name;
        vvn.net = pair.net_n;
        vvn.pos = nv;
        vvn.top_layer = span.top;
        vvn.bottom_layer = span.bottom;
        vvn.outer_d_nm = use.outer_nm;
        vvn.hole_d_nm = use.hole_nm;
        vvn.via_class = use.name;
        // Rewire run endpoints at the corridor via to the unstaggered
        // bases (member pads are never rewired), then add parallel entry
        // stubs base->barrel on every spanned layer that owns runs. Layers
        // of the span without runs (endpoint vias) get via-to-pad stubs.
        auto is_pad = [&](Point p) {
            return p == tpa->pos || p == tpb->pos || p == tna->pos ||
                   p == tnb->pos;
        };
        std::set<LayerId> run_layers;
        for (const auto& rp : run_pairs) run_layers.insert(rp.layer);
        for (auto& rp : run_pairs) {
            if (rp.layer < span.top || rp.layer > span.bottom) continue;
            if (!rp.pp.empty() && !(is_pad(rp.pp.front())) &&
                manhattan(rp.pp.front(), cv.pos) <= center_dist)
                rp.pp.front() = bp;
            if (!rp.pp.empty() && !(is_pad(rp.pp.back())) &&
                manhattan(rp.pp.back(), cv.pos) <= center_dist)
                rp.pp.back() = bp;
            if (!rp.np.empty() && !(is_pad(rp.np.front())) &&
                manhattan(rp.np.front(), cv.pos) <= center_dist)
                rp.np.front() = bn;
            if (!rp.np.empty() && !(is_pad(rp.np.back())) &&
                manhattan(rp.np.back(), cv.pos) <= center_dist)
                rp.np.back() = bn;
        }
        for (LayerId L = span.top; L <= span.bottom; ++L) {
            Coord wp = wp_of.count(L) ? wp_of[L] : wp_of.begin()->second;
            Coord wn = wn_of.count(L) ? wn_of[L] : wn_of.begin()->second;
            if (run_layers.count(L)) {
                if (!(bp == pv))
                    stub_p.push_back({pair.net_p, L, bp, pv, wp});
                if (!(bn == nv))
                    stub_n.push_back({pair.net_n, L, bn, nv, wn});
            } else {
                // Endpoint-via layer without centerline: stub the barrels
                // directly to the member pads on this layer.
                const Terminal* ppad = nullptr;
                const Terminal* npad = nullptr;
                for (const Terminal* t : {tpa, tpb}) {
                    if (t->layer == L &&
                        (!ppad || manhattan(t->pos, cv.pos) <
                                        manhattan(ppad->pos, cv.pos) ||
                         (manhattan(t->pos, cv.pos) ==
                              manhattan(ppad->pos, cv.pos) &&
                          t->id < ppad->id)))
                        ppad = t;
                }
                for (const Terminal* t : {tna, tnb}) {
                    if (t->layer == L &&
                        (!npad || manhattan(t->pos, cv.pos) <
                                        manhattan(npad->pos, cv.pos) ||
                         (manhattan(t->pos, cv.pos) ==
                              manhattan(npad->pos, cv.pos) &&
                          t->id < npad->id)))
                        npad = t;
                }
                if (!ppad || !npad) return fail("via_infeasible");
                if (!(pv == ppad->pos))
                    stub_p.push_back({pair.net_p, L, pv, ppad->pos, wp});
                if (!(nv == npad->pos))
                    stub_n.push_back({pair.net_n, L, nv, npad->pos, wn});
            }
        }
        vp_all.push_back(vvp);
        vn_all.push_back(vvn);
    }

    // Final emit from rewired polylines plus collected stubs.
    for (const auto& rp : run_pairs) {
        Coord wp = wp_of[rp.layer], wn = wn_of[rp.layer];
        for (std::size_t i = 0; i + 1 < rp.pp.size(); ++i) {
            if (rp.pp[i] == rp.pp[i + 1]) continue;
            tp_all.push_back({pair.net_p, rp.layer, rp.pp[i], rp.pp[i + 1], wp});
        }
        for (std::size_t i = 0; i + 1 < rp.np.size(); ++i) {
            if (rp.np[i] == rp.np[i + 1]) continue;
            tn_all.push_back({pair.net_n, rp.layer, rp.np[i], rp.np[i + 1], wn});
        }
    }
    for (const auto& s : stub_p) tp_all.push_back(s);
    for (const auto& s : stub_n) tn_all.push_back(s);
    if (tp_all.empty() || tn_all.empty()) return fail("empty_materialization");

    // Exact legality: every member segment vs foreign copper (sibling
    // exempt, gap-governed), then pair gap, then vias, then skew.
    for (const auto& s : tp_all) {
        if (!pair_segment_legal(board_without_corridor, resolver, pair.net_p,
                                pair.net_n, s.layer, s.width_nm, s.segment(), ctx))
            return fail("illegal:p_trace");
    }
    for (const auto& s : tn_all) {
        if (!pair_segment_legal(board_without_corridor, resolver, pair.net_n,
                                pair.net_p, s.layer, s.width_nm, s.segment(), ctx))
            return fail("illegal:n_trace");
    }
    // Inter-member gap (exact): min edge gap >= gap - tol.
    {
        Coord worst = 0;
        Point at{};
        // Incremental check including sibling committed traces being built:
        // check P vs N final sets (both complete here).
        if (!pair_gap_legal(tp_all, tn_all, pair.gap_nm, pair.gap_tol_nm, worst, at)) {
            out.worst_gap_err_nm = worst;
            out.gap_violation_at = at;
            out.has_gap_violation_at = true;
            return fail("gap_violation");
        }
        out.worst_gap_err_nm = worst;
        if (worst > 0) {
            out.gap_violation_at = at;
            out.has_gap_violation_at = true;
        }
    }
    // Sibling-pad gap: member traces keep pair gap from the sibling's pads
    // (adjacent pair pins are gap-governed, not voltage-governed).
    {
        for (const auto& s : tp_all) {
            for (TermId tid : {na, nb}) {
                const Terminal* t = board_without_corridor.find_terminal(tid);
                if (!t || t->layer != s.layer) continue;
                // Own connectivity: P traces touch P pads only; sibling pads
                // must keep the gap. Pad-touching fan endpoints are near the
                // sibling pad by construction (shared pitch); enforce the
                // same edge rule as trace-trace.
                __int128 d2 = seg_rect_dist2(s.segment(), t->pad_rect());
                __int128 rhs = (__int128)2 * (pair.gap_nm - pair.gap_tol_nm) + s.width_nm;
                if (rhs < 0) continue;
                if ((__int128)4 * d2 < rhs * rhs) return fail("gap_violation:pad");
            }
        }
        for (const auto& s : tn_all) {
            for (TermId tid : {pa, pb}) {
                const Terminal* t = board_without_corridor.find_terminal(tid);
                if (!t || t->layer != s.layer) continue;
                __int128 d2 = seg_rect_dist2(s.segment(), t->pad_rect());
                __int128 rhs = (__int128)2 * (pair.gap_nm - pair.gap_tol_nm) + s.width_nm;
                if (rhs < 0) continue;
                if ((__int128)4 * d2 < rhs * rhs) return fail("gap_violation:pad");
            }
        }
    }
    // Member traces vs sibling via barrels (and vice versa): the barrel
    // edge keeps the pair gap too, on overlapping layers.
    {
        auto trace_via_ok = [&](const TraceSeg& s, const Via& v) {
            if (s.layer < std::min(v.top_layer, v.bottom_layer) ||
                s.layer > std::max(v.top_layer, v.bottom_layer))
                return true;
            Rect vr = Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm);
            __int128 d2 = seg_rect_dist2(s.segment(), vr);
            __int128 rhs = (__int128)2 * (pair.gap_nm - pair.gap_tol_nm) + s.width_nm;
            if (rhs < 0) return true;
            return (__int128)4 * d2 >= rhs * rhs;
        };
        for (const auto& s : tp_all)
            for (const auto& v : vn_all)
                if (!trace_via_ok(s, v)) return fail("gap_violation:trace_via");
        for (const auto& s : tn_all)
            for (const auto& v : vp_all)
                if (!trace_via_ok(s, v)) return fail("gap_violation:via_trace");
    }
    // Via legality (foreign) + pair via gap.
    for (const auto& v : vp_all) {
        if (!pair_via_legal(board_without_corridor, resolver, pair.net_p, pair.net_n,
                            v, vp_all, vn_all, ctx))
            return fail("illegal:p_via");
    }
    for (const auto& v : vn_all) {
        if (!pair_via_legal(board_without_corridor, resolver, pair.net_n, pair.net_p,
                            v, vn_all, vp_all, ctx))
            return fail("illegal:n_via");
    }
    for (const auto& a : vp_all) {
        for (const auto& b : vn_all) {
            bool overlap = !(b.bottom_layer < std::min(a.top_layer, a.bottom_layer) ||
                             b.top_layer > std::max(a.top_layer, a.bottom_layer));
            if (!overlap) continue;
            Rect ra = Rect::from_center_size(a.pos, a.outer_d_nm, a.outer_d_nm);
            Rect rb = Rect::from_center_size(b.pos, b.outer_d_nm, b.outer_d_nm);
            Coord dx = 0, dy = 0;
            if (ra.x2 < rb.x1) dx = rb.x1 - ra.x2;
            else if (rb.x2 < ra.x1) dx = ra.x1 - rb.x2;
            if (ra.y2 < rb.y1) dy = rb.y1 - ra.y2;
            else if (rb.y2 < ra.y1) dy = ra.y1 - rb.y2;
            __int128 d2 = (__int128)dx * dx + (__int128)dy * dy;
            std::string cs;
            Coord clr = resolver.requiredClearance(pair.net_p, pair.net_n, a.top_layer,
                                                   ctx, &cs);
            Coord need = std::max(pair.gap_nm - pair.gap_tol_nm, clr);
            if (d2 < (__int128)need * need) return fail("gap_violation:via");
        }
    }
    // Skew gate.
    Coord lp = pair_total_length(tp_all), ln = pair_total_length(tn_all);
    Coord skew = lp >= ln ? lp - ln : ln - lp;
    if (pair.has_max_skew && skew > pair.max_skew_nm) return fail("skew_exceeded");

    out.ok = true;
    out.reason = "ok";
    out.traces_p = tp_all;
    out.traces_n = tn_all;
    out.vias_p = vp_all;
    out.vias_n = vn_all;
    out.length_p_nm = lp;
    out.length_n_nm = ln;
    out.skew_nm = skew;
    return out;
}

JsonValue PairReport::to_json() const {
    JsonValue o = JsonValue::object();
    o["pair_id"] = static_cast<double>(pair_id);
    o["name"] = name;
    o["net_p"] = static_cast<double>(net_p);
    o["net_n"] = static_cast<double>(net_n);
    o["net_p_name"] = net_p_name;
    o["net_n_name"] = net_n_name;
    o["corridor_routed"] = corridor_routed;
    o["occupied_width_mm"] = occupied_width_mm;
    o["gap_mm"] = gap_mm;
    o["gap_tol_mm"] = gap_tol_mm;
    o["materialized"] = materialized;
    o["status"] = status;
    o["length_p_mm"] = length_p_mm;
    o["length_n_mm"] = length_n_mm;
    o["skew_mm"] = skew_mm;
    o["worst_gap_err_mm"] = worst_gap_err_mm;
    o["has_gap_location"] = has_gap_location;
    o["gap_x_mm"] = gap_x_mm;
    o["gap_y_mm"] = gap_y_mm;
    o["gap_layer"] = static_cast<double>(gap_layer);
    o["via_pairs"] = static_cast<double>(via_pairs);
    o["vias_p"] = static_cast<double>(vias_p);
    o["vias_n"] = static_cast<double>(vias_n);
    o["via_mismatch"] = via_mismatch;
    return o;
}

}  // namespace copperline
