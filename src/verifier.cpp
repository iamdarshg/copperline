#include "router/verifier.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>

#include "router/diffpair.h"
#include "router/simplify.h"
#include "router/spatial_index.h"
#include "router/via_bundle.h"

namespace copperline {

JsonValue PairVerifyDetail::to_json() const {
    JsonValue o = JsonValue::object();
    o["pair_id"] = static_cast<double>(pair_id);
    o["name"] = name;
    o["net_p"] = static_cast<double>(net_p);
    o["net_n"] = static_cast<double>(net_n);
    o["net_p_name"] = net_p_name;
    o["net_n_name"] = net_n_name;
    o["length_p_mm"] = length_p_mm;
    o["length_n_mm"] = length_n_mm;
    o["skew_mm"] = skew_mm;
    o["worst_gap_err_mm"] = worst_gap_err_mm;
    o["has_gap_location"] = has_gap_location;
    o["gap_x_mm"] = gap_x_mm;
    o["gap_y_mm"] = gap_y_mm;
    o["gap_layer"] = static_cast<double>(gap_layer);
    o["vias_p"] = static_cast<double>(vias_p);
    o["vias_n"] = static_cast<double>(vias_n);
    o["via_pairs"] = static_cast<double>(via_pairs);
    o["via_mismatch"] = via_mismatch;
    o["status"] = status;
    o["ok"] = ok;
    return o;
}

JsonValue VerifyResult::to_json() const {
    JsonValue r = JsonValue::object();
    r["schema"] = "copperline/verify-report/1";
    r["ok"] = ok;
    r["connected"] = connected;
    r["legal"] = legal;
    JsonValue u = JsonValue::array();
    for (const auto& e : unconnected) {
        JsonValue o = JsonValue::object();
        o["net"] = static_cast<double>(e.net);
        o["net_name"] = e.net_name;
        o["terminal"] = static_cast<double>(e.terminal);
        o["component"] = e.component;
        o["pin"] = e.pin;
        u.as_array().push_back(o);
    }
    r["unconnected"] = u;
    JsonValue v = JsonValue::array();
    for (const auto& x : violations) {
        JsonValue o = JsonValue::object();
        o["type"] = x.type;
        o["net_a"] = static_cast<double>(x.net_a);
        o["net_b"] = static_cast<double>(x.net_b);
        o["rule"] = x.rule;
        o["detail"] = x.detail;
        o["x_mm"] = x.x_mm;
        o["y_mm"] = x.y_mm;
        o["layer"] = static_cast<double>(x.layer);
        v.as_array().push_back(o);
    }
    r["violations"] = v;
    JsonValue dp = JsonValue::array();
    for (const auto& p : pairs) dp.as_array().push_back(p.to_json());
    r["diffpairs"] = dp;
    // Alias kept for agent convenience (issue #13 route/verify JSON).
    r["pairs"] = dp;
    return r;
}

namespace {

struct DSU {
    std::vector<int> p;
    explicit DSU(int n) : p(n) {
        std::iota(p.begin(), p.end(), 0);
    }
    int find(int x) { return p[x] == x ? x : p[x] = find(p[x]); }
    void unite(int a, int b) { p[find(a)] = find(b); }
};

// Copper element for connectivity (per net) and clearance (global).
struct Element {
    enum class Kind { kPad, kTrace, kVia, kPlane } kind;
    NetId net = -1;
    LayerId layer = 0;  // pads/traces/planes; vias use lo/hi
    LayerId lo = 0, hi = 0;
    Rect rect{};        // pads + via discs (square approx) + plane bboxes
    Segment seg{};      // traces
    Coord width = 0;    // traces
    int term_id = -1;   // pads
    int plane_id = -1;  // planes
    int plane_island = 0;
    const std::vector<Point>* poly = nullptr;  // planes (board-owned)
};

Rect elem_bounds(const Element& e) {
    if (e.kind == Element::Kind::kTrace) return e.seg.bounds().expanded(e.width / 2 + 1);
    return e.rect.expanded(1);
}

bool elem_layer_overlap(const Element& a, const Element& b) {
    LayerId a_lo = a.kind == Element::Kind::kVia ? a.lo : a.layer;
    LayerId a_hi = a.kind == Element::Kind::kVia ? a.hi : a.layer;
    LayerId b_lo = b.kind == Element::Kind::kVia ? b.lo : b.layer;
    LayerId b_hi = b.kind == Element::Kind::kVia ? b.hi : b.layer;
    return a_lo <= b_hi && b_lo <= a_hi;
}

// Issue #16: planes join connectivity by declared island (same island id =
// stitched) or by visible geometric bridging; copper touches a plane
// polygon on an overlapping layer. Cross-net overlap never unites because
// the DSU loop only pairs same-net elements.
bool elem_touch(const Element& a, const Element& b) {
    const bool a_plane = a.kind == Element::Kind::kPlane;
    const bool b_plane = b.kind == Element::Kind::kPlane;
    if (a_plane && b_plane) {
        if (a.plane_island == b.plane_island) return true;
        if (!elem_layer_overlap(a, b) || !a.poly || !b.poly) return false;
        for (const auto& p : *a.poly) {
            if (plane_poly_contains(*b.poly, p)) return true;
        }
        for (const auto& p : *b.poly) {
            if (plane_poly_contains(*a.poly, p)) return true;
        }
        std::size_t n = a.poly->size(), m = b.poly->size();
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < m; ++j)
                if (seg_intersects_seg({(*a.poly)[i], (*a.poly)[(i + 1) % n]},
                                       {(*b.poly)[j], (*b.poly)[(j + 1) % m]}))
                    return true;
        return false;
    }
    if (a_plane || b_plane) {
        const Element& pl = a_plane ? a : b;
        const Element& other = a_plane ? b : a;
        if (!elem_layer_overlap(pl, other) || !pl.poly) return false;
        if (other.kind == Element::Kind::kTrace)
            return plane_seg_hits_poly(other.seg, *pl.poly);
        return plane_rect_hits_poly(other.rect, *pl.poly);
    }
    if (!elem_layer_overlap(a, b)) return false;
    if (a.kind == Element::Kind::kTrace && b.kind == Element::Kind::kTrace)
        return seg_intersects_seg(a.seg, b.seg);
    if (a.kind == Element::Kind::kTrace)
        return seg_intersects_rect(a.seg, b.rect);
    if (b.kind == Element::Kind::kTrace)
        return seg_intersects_rect(b.seg, a.rect);
    return a.rect.intersects(b.rect);
}

// Exact squared edge-to-edge distance between two polygons (0 on hit).
__int128 plane_poly_dist2(const std::vector<Point>& a, const std::vector<Point>& b) {
    for (const auto& p : a) {
        if (plane_poly_contains(b, p)) return 0;
    }
    for (const auto& p : b) {
        if (plane_poly_contains(a, p)) return 0;
    }
    __int128 best = -1;
    for (std::size_t i = 0; i < a.size(); ++i)
        for (std::size_t j = 0; j < b.size(); ++j) {
            __int128 d = seg_seg_dist2({a[i], a[(i + 1) % a.size()]},
                                       {b[j], b[(j + 1) % b.size()]});
            if (best < 0 || d < best) best = d;
        }
    return best < 0 ? 0 : best;
}

// Exact copper-edge to copper-edge check: gap >= need, all in integers.
// For centerline-based primitives the half widths are folded into the
// comparison (4*d2 >= (2*need + w)^2) instead of the geometry, so odd-nm
// widths stay exact.
bool elem_gap_ok(const Element& a, const Element& b, Coord need) {
    const bool a_plane = a.kind == Element::Kind::kPlane;
    const bool b_plane = b.kind == Element::Kind::kPlane;
    // Issue #16: foreign pours keep exact polygon clearance. Trace
    // centerlines fold half width into the comparison; rects compare edges.
    if (a_plane && b_plane) {
        if (!a.poly || !b.poly) return true;
        __int128 d2 = plane_poly_dist2(*a.poly, *b.poly);
        return d2 >= (__int128)need * need;
    }
    if (a_plane || b_plane) {
        const Element& pl = a_plane ? a : b;
        const Element& other = a_plane ? b : a;
        if (!pl.poly) return true;
        if (other.kind == Element::Kind::kTrace) {
            __int128 d2 = plane_seg_poly_dist2(other.seg, *pl.poly);
            __int128 rhs = (__int128)2 * need + other.width;
            return (__int128)4 * d2 >= rhs * rhs;
        }
        __int128 d2 = plane_rect_poly_dist2(other.rect, *pl.poly);
        return d2 >= (__int128)need * need;
    }
    if (a.kind == Element::Kind::kTrace && b.kind == Element::Kind::kTrace) {
        __int128 d2 = seg_seg_dist2(a.seg, b.seg);
        __int128 rhs = (__int128)2 * need + a.width + b.width;
        return (__int128)4 * d2 >= rhs * rhs;
    }
    if (a.kind == Element::Kind::kTrace) {
        __int128 d2 = seg_rect_dist2(a.seg, b.rect);
        __int128 rhs = (__int128)2 * need + a.width;
        return (__int128)4 * d2 >= rhs * rhs;
    }
    if (b.kind == Element::Kind::kTrace) {
        __int128 d2 = seg_rect_dist2(b.seg, a.rect);
        __int128 rhs = (__int128)2 * need + b.width;
        return (__int128)4 * d2 >= rhs * rhs;
    }
    return rect_gap(a.rect, b.rect) >= need;
}

// Approximate gap in mm for human detail strings (reporting only).
double elem_gap_mm(const Element& a, const Element& b) {
    // Issue #16: polygon-aware approximations for pour pairs (reporting only;
    // legality stays in exact integer predicates above).
    auto poly_gap_mm = [](const std::vector<Point>* pa, const std::vector<Point>* pb,
                          const Element& oa, const Element& ob) -> double {
        if (!pa || !pb) return 0.0;
        __int128 d2 = plane_poly_dist2(*pa, *pb);
        (void)oa;
        (void)ob;
        return std::sqrt(static_cast<double>(d2)) / 1e6;
    };
    const bool a_plane = a.kind == Element::Kind::kPlane;
    const bool b_plane = b.kind == Element::Kind::kPlane;
    if (a_plane && b_plane) return poly_gap_mm(a.poly, b.poly, a, b);
    if (a_plane || b_plane) {
        const Element& pl = a_plane ? a : b;
        const Element& other = a_plane ? b : a;
        if (!pl.poly) return 0.0;
        if (other.kind == Element::Kind::kTrace) {
            double d = std::sqrt(static_cast<double>(plane_seg_poly_dist2(other.seg, *pl.poly)));
            return (d - other.width / 2.0) / 1e6;
        }
        double d = std::sqrt(static_cast<double>(plane_rect_poly_dist2(other.rect, *pl.poly)));
        return d / 1e6;
    }
    if (a.kind == Element::Kind::kTrace && b.kind == Element::Kind::kTrace) {
        double d = std::sqrt(static_cast<double>(seg_seg_dist2(a.seg, b.seg)));
        return (d - (a.width + b.width) / 2.0) / 1e6;
    }
    if (a.kind == Element::Kind::kTrace) {
        double d = std::sqrt(static_cast<double>(seg_rect_dist2(a.seg, b.rect)));
        return (d - a.width / 2.0) / 1e6;
    }
    if (b.kind == Element::Kind::kTrace) {
        double d = std::sqrt(static_cast<double>(seg_rect_dist2(b.seg, a.rect)));
        return (d - b.width / 2.0) / 1e6;
    }
    return nm_to_mm(rect_gap(a.rect, b.rect));
}

Point elem_rep(const Element& a, const Element& b) {
    Point pa = a.kind == Element::Kind::kTrace ? a.seg.a : a.rect.center();
    Point pb = b.kind == Element::Kind::kTrace ? b.seg.a : b.rect.center();
    return {(pa.x + pb.x) / 2, (pa.y + pb.y) / 2};
}

// ---- Issue #13: pair-aware verification from committed copper ----

// True when the board declares (p, n) as a differential pair (either order).
bool pair_declared(const Board& board, NetId a, NetId b) {
    if (a == b) return false;
    for (const auto& pr : board.diffpairs) {
        if ((pr.net_p == a && pr.net_n == b) || (pr.net_p == b && pr.net_n == a))
            return true;
    }
    return false;
}

Point seg_mid(const Segment& s) {
    return {(s.a.x + s.b.x) / 2, (s.a.y + s.b.y) / 2};
}

// Verify one pair from committed copper only. Appends hard violations to
// `violations` and returns the per-pair measurement for VerifyResult::pairs
// and the route JSON. Deterministic: board order iteration, sorted vias.
PairVerifyDetail verify_one_pair(const Board& board, const DiffPair& pr,
                                 std::vector<Violation>& violations) {
    PairVerifyDetail d;
    d.pair_id = pr.id;
    d.name = pr.name;
    d.net_p = pr.net_p;
    d.net_n = pr.net_n;
    const NetInfo* np = board.find_net(pr.net_p);
    const NetInfo* nn = board.find_net(pr.net_n);
    d.net_p_name = np ? np->name : "?";
    d.net_n_name = nn ? nn->name : "?";

    std::string why;
    if (!diffpair_valid(board, pr, why)) {
        d.status = "INVALID:" + why;
        d.ok = false;
        Violation v;
        v.type = "diffpair_invalid";
        v.net_a = pr.net_p;
        v.net_b = pr.net_n;
        v.rule = "diffpair:" + pr.name;
        v.detail = "diffpair " + pr.name + " invalid: " + why;
        v.layer = 0;
        violations.push_back(v);
        return d;
    }

    std::vector<TraceSeg> tp, tn;
    std::vector<Via> vp, vn;
    std::vector<Terminal> pads_p, pads_n;
    for (const auto& t : board.traces) {
        if (t.net == pr.net_p) tp.push_back(t);
        if (t.net == pr.net_n) tn.push_back(t);
    }
    for (const auto& v : board.vias) {
        if (v.net == pr.net_p) vp.push_back(v);
        if (v.net == pr.net_n) vn.push_back(v);
    }
    for (const auto& t : board.terminals) {
        if (t.net == pr.net_p) pads_p.push_back(t);
        if (t.net == pr.net_n) pads_n.push_back(t);
    }
    d.vias_p = static_cast<int>(vp.size());
    d.vias_n = static_cast<int>(vn.size());
    d.via_pairs = std::min(d.vias_p, d.vias_n);

    Coord lp = pair_total_length(tp), ln = pair_total_length(tn);
    Coord skew = lp >= ln ? lp - ln : ln - lp;
    d.length_p_mm = nm_to_mm(lp);
    d.length_n_mm = nm_to_mm(ln);
    d.skew_mm = nm_to_mm(skew);

    const bool has_p = !tp.empty() || !vp.empty();
    const bool has_n = !tn.empty() || !vn.empty();
    std::vector<std::string> parts;

    auto first_copper_rep = [&]() -> std::pair<Point, LayerId> {
        if (!tp.empty()) return {seg_mid(tp.front().segment()), tp.front().layer};
        if (!tn.empty()) return {seg_mid(tn.front().segment()), tn.front().layer};
        if (!vp.empty()) return {vp.front().pos, vp.front().top_layer};
        if (!vn.empty()) return {vn.front().pos, vn.front().top_layer};
        if (!pads_p.empty()) return {pads_p.front().pos, pads_p.front().layer};
        if (!pads_n.empty()) return {pads_n.front().pos, pads_n.front().layer};
        return {{board.width_nm / 2, board.height_nm / 2}, 0};
    };

    if (has_p != has_n) {
        parts.push_back("ONE_SIDED");
        auto [rep, layer] = first_copper_rep();
        Violation v;
        v.type = "diffpair_one_sided";
        v.net_a = pr.net_p;
        v.net_b = pr.net_n;
        v.rule = "diffpair:" + pr.name;
        v.detail = "diffpair " + pr.name + " has copper on only one member (P traces=" +
                   std::to_string(tp.size()) + " vias=" + std::to_string(vp.size()) +
                   ", N traces=" + std::to_string(tn.size()) + " vias=" +
                   std::to_string(vn.size()) + ")";
        v.x_mm = nm_to_mm(rep.x);
        v.y_mm = nm_to_mm(rep.y);
        v.layer = layer;
        violations.push_back(v);
    }

    // Compatible layers where coupling is required.
    {
        std::string layer_detail;
        Point layer_at{0, 0};
        LayerId layer_id = 0;
        bool bad = false;
        if (!pr.preferred_layers.empty()) {
            std::set<LayerId> pref(pr.preferred_layers.begin(), pr.preferred_layers.end());
            for (const auto& t : tp) {
                if (!pref.count(t.layer)) {
                    bad = true;
                    layer_detail = "P trace on layer " + std::to_string(t.layer) +
                                   " outside preferred layers";
                    layer_at = seg_mid(t.segment());
                    layer_id = t.layer;
                    break;
                }
            }
            if (!bad) {
                for (const auto& t : tn) {
                    if (!pref.count(t.layer)) {
                        bad = true;
                        layer_detail = "N trace on layer " + std::to_string(t.layer) +
                                       " outside preferred layers";
                        layer_at = seg_mid(t.segment());
                        layer_id = t.layer;
                        break;
                    }
                }
            }
        } else if (!tp.empty() && !tn.empty()) {
            std::set<LayerId> set_p, set_n;
            for (const auto& t : tp) set_p.insert(t.layer);
            for (const auto& t : tn) set_n.insert(t.layer);
            bool shared = false;
            for (LayerId l : set_p) {
                if (set_n.count(l)) {
                    shared = true;
                    break;
                }
            }
            if (!shared) {
                bad = true;
                layer_detail = "P/N share no coupled layer";
                layer_at = seg_mid(tp.front().segment());
                layer_id = tp.front().layer;
            }
        }
        if (bad) {
            parts.push_back("LAYER_MISMATCH");
            Violation v;
            v.type = "diffpair_layer";
            v.net_a = pr.net_p;
            v.net_b = pr.net_n;
            v.rule = "diffpair:" + pr.name;
            v.detail = "diffpair " + pr.name + " layer mismatch: " + layer_detail;
            v.x_mm = nm_to_mm(layer_at.x);
            v.y_mm = nm_to_mm(layer_at.y);
            v.layer = layer_id;
            violations.push_back(v);
        }
    }

    // Coupled-section gap: exact integer math, arbitrary-angle aware.
    // min_tt = closest trace-trace edge on a shared layer (coupling trunk).
    // min_all = closest edge over every same-span P/N copper pair
    // (trace-trace, trace-pad, trace-via, via-via, pad-pad).
    // Too close (min_all < gap-tol) and too far (min_tt > gap+tol) fail
    // independently; fanout divergence never masks the trunk minimum.
    if (has_p && has_n) {
        const Coord lo = pr.gap_nm - pr.gap_tol_nm;
        bool have_tt = false, have_all = false;
        long double best_tt = 0, best_all = 0;  // edge approx, nm
        Point at_tt{0, 0}, at_all{0, 0};
        LayerId layer_tt = 0, layer_all = 0;
        bool exact_too_close = false;
        Point close_at{0, 0};
        LayerId close_layer = 0;

        auto note_all = [&](long double edge, Point at, LayerId layer) {
            if (!have_all || edge < best_all) {
                have_all = true;
                best_all = edge;
                at_all = at;
                layer_all = layer;
            }
        };
        auto note_tt = [&](long double edge, Point at, LayerId layer) {
            if (!have_tt || edge < best_tt) {
                have_tt = true;
                best_tt = edge;
                at_tt = at;
                layer_tt = layer;
            }
            note_all(edge, at, layer);
        };

        for (const auto& a : tp) {
            for (const auto& b : tn) {
                if (a.layer != b.layer) continue;
                __int128 d2 = seg_seg_dist2(a.segment(), b.segment());
                __int128 rhs_lo =
                    (__int128)2 * lo + a.width_nm + b.width_nm;
                if (rhs_lo >= 0 && (__int128)4 * d2 < rhs_lo * rhs_lo) {
                    exact_too_close = true;
                    Point m1 = seg_mid(a.segment()), m2 = seg_mid(b.segment());
                    close_at = {(m1.x + m2.x) / 2, (m1.y + m2.y) / 2};
                    close_layer = a.layer;
                }
                long double dc = std::sqrt((long double)d2);
                long double edge = dc - (a.width_nm + b.width_nm) / 2.0L;
                Point m1 = seg_mid(a.segment()), m2 = seg_mid(b.segment());
                note_tt((double)edge, {(m1.x + m2.x) / 2, (m1.y + m2.y) / 2},
                        a.layer);
            }
        }
        for (const auto& s : tp) {
            for (const auto& p : pads_n) {
                if (p.layer != s.layer) continue;
                __int128 d2 = seg_rect_dist2(s.segment(), p.pad_rect());
                __int128 rhs_lo = (__int128)2 * lo + s.width_nm;
                if (rhs_lo >= 0 && (__int128)4 * d2 < rhs_lo * rhs_lo) {
                    exact_too_close = true;
                    Point m = seg_mid(s.segment());
                    close_at = {(m.x + p.pos.x) / 2, (m.y + p.pos.y) / 2};
                    close_layer = s.layer;
                }
                long double edge =
                    std::sqrt((long double)d2) - s.width_nm / 2.0L;
                Point m = seg_mid(s.segment());
                note_all((double)edge, {(m.x + p.pos.x) / 2, (m.y + p.pos.y) / 2},
                         s.layer);
            }
        }
        for (const auto& s : tn) {
            for (const auto& p : pads_p) {
                if (p.layer != s.layer) continue;
                __int128 d2 = seg_rect_dist2(s.segment(), p.pad_rect());
                __int128 rhs_lo = (__int128)2 * lo + s.width_nm;
                if (rhs_lo >= 0 && (__int128)4 * d2 < rhs_lo * rhs_lo) {
                    exact_too_close = true;
                    Point m = seg_mid(s.segment());
                    close_at = {(m.x + p.pos.x) / 2, (m.y + p.pos.y) / 2};
                    close_layer = s.layer;
                }
                long double edge =
                    std::sqrt((long double)d2) - s.width_nm / 2.0L;
                Point m = seg_mid(s.segment());
                note_all((double)edge, {(m.x + p.pos.x) / 2, (m.y + p.pos.y) / 2},
                         s.layer);
            }
        }
        auto trace_via_gap = [&](const TraceSeg& s, const Via& v) {
            LayerId v_lo = std::min(v.top_layer, v.bottom_layer);
            LayerId v_hi = std::max(v.top_layer, v.bottom_layer);
            if (s.layer < v_lo || s.layer > v_hi) return false;
            Rect vr = Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm);
            __int128 d2 = seg_rect_dist2(s.segment(), vr);
            __int128 rhs_lo = (__int128)2 * lo + s.width_nm;
            if (rhs_lo >= 0 && (__int128)4 * d2 < rhs_lo * rhs_lo) {
                exact_too_close = true;
                Point m = seg_mid(s.segment());
                close_at = {(m.x + v.pos.x) / 2, (m.y + v.pos.y) / 2};
                close_layer = s.layer;
            }
            long double edge = std::sqrt((long double)d2) - s.width_nm / 2.0L;
            Point m = seg_mid(s.segment());
            note_all((double)edge, {(m.x + v.pos.x) / 2, (m.y + v.pos.y) / 2},
                     s.layer);
            return true;
        };
        for (const auto& s : tp) {
            for (const auto& v : vn) trace_via_gap(s, v);
        }
        for (const auto& s : tn) {
            for (const auto& v : vp) trace_via_gap(s, v);
        }
        for (const auto& a : vp) {
            for (const auto& b : vn) {
                bool overlap =
                    !(b.bottom_layer < std::min(a.top_layer, a.bottom_layer) ||
                      b.top_layer > std::max(a.top_layer, a.bottom_layer));
                if (!overlap) continue;
                Rect ra = Rect::from_center_size(a.pos, a.outer_d_nm, a.outer_d_nm);
                Rect rb = Rect::from_center_size(b.pos, b.outer_d_nm, b.outer_d_nm);
                Coord g = rect_gap(ra, rb);
                if (g < lo) {
                    exact_too_close = true;
                    close_at = {(a.pos.x + b.pos.x) / 2, (a.pos.y + b.pos.y) / 2};
                    close_layer = a.top_layer;
                }
                Point at{(a.pos.x + b.pos.x) / 2, (a.pos.y + b.pos.y) / 2};
                note_all((double)g, at, a.top_layer);
            }
        }
        for (const auto& a : pads_p) {
            for (const auto& b : pads_n) {
                if (a.layer != b.layer) continue;
                Coord g = rect_gap(a.pad_rect(), b.pad_rect());
                if (g < lo) {
                    exact_too_close = true;
                    close_at = {(a.pos.x + b.pos.x) / 2, (a.pos.y + b.pos.y) / 2};
                    close_layer = a.layer;
                }
                Point at{(a.pos.x + b.pos.x) / 2, (a.pos.y + b.pos.y) / 2};
                note_all((double)g, at, a.layer);
            }
        }

        // Worst error + location track the trunk minimum when measurable.
        long double ref_edge = have_tt ? best_tt : (have_all ? best_all : 0);
        Point ref_at = have_tt ? at_tt : at_all;
        LayerId ref_layer = have_tt ? layer_tt : layer_all;
        if (have_tt || have_all) {
            long double err = fabsl(ref_edge - (long double)pr.gap_nm);
            d.worst_gap_err_mm = (double)(err / 1e6L);
            d.has_gap_location = true;
            d.gap_x_mm = nm_to_mm(ref_at.x);
            d.gap_y_mm = nm_to_mm(ref_at.y);
            d.gap_layer = ref_layer;
        }

        if (exact_too_close) {
            parts.push_back("GAP_VIOLATION:too_close");
            Violation v;
            v.type = "diffpair_gap";
            v.net_a = pr.net_p;
            v.net_b = pr.net_n;
            v.rule = "diffpair:" + pr.name;
            v.detail = "diffpair " + pr.name + " edge gap below " +
                       std::to_string(nm_to_mm(lo)) + "mm (gap " +
                       std::to_string(nm_to_mm(pr.gap_nm)) + "mm tol " +
                       std::to_string(nm_to_mm(pr.gap_tol_nm)) + "mm)";
            v.x_mm = nm_to_mm(close_at.x);
            v.y_mm = nm_to_mm(close_at.y);
            v.layer = close_layer;
            violations.push_back(v);
        } else if (!have_tt && !tp.empty() && !tn.empty()) {
            // Excessive gap as uncoupling: both members own trace copper
            // but share no layer, so no coupled section exists at all.
            // (A finite same-layer minimum is governed by the gap floor
            // above, matching the materializer's one-sided tolerance that
            // the routed suite is built on: pad-pitch fanout legitimately
            // rests above nominal.)
            parts.push_back("GAP_VIOLATION:too_far");
            Point rep = seg_mid(tp.front().segment());
            Violation v;
            v.type = "diffpair_gap";
            v.net_a = pr.net_p;
            v.net_b = pr.net_n;
            v.rule = "diffpair:" + pr.name;
            v.detail = "diffpair " + pr.name +
                       " members uncoupled: no shared-layer traces (gap effectively "
                       "unbounded, nominal " +
                       std::to_string(nm_to_mm(pr.gap_nm)) + "mm tol " +
                       std::to_string(nm_to_mm(pr.gap_tol_nm)) + "mm)";
            v.x_mm = nm_to_mm(rep.x);
            v.y_mm = nm_to_mm(rep.y);
            v.layer = tp.front().layer;
            violations.push_back(v);
        }
    }

    if (pr.has_max_skew && skew > pr.max_skew_nm) {
        parts.push_back("SKEW_EXCEEDED");
        auto [rep, layer] = first_copper_rep();
        Violation v;
        v.type = "diffpair_skew";
        v.net_a = pr.net_p;
        v.net_b = pr.net_n;
        v.rule = "diffpair:" + pr.name;
        v.detail = "diffpair " + pr.name + " skew " +
                   std::to_string(nm_to_mm(skew)) + "mm exceeds " +
                   std::to_string(nm_to_mm(pr.max_skew_nm)) + "mm (P " +
                   std::to_string(nm_to_mm(lp)) + "mm N " +
                   std::to_string(nm_to_mm(ln)) + "mm)";
        v.x_mm = nm_to_mm(rep.x);
        v.y_mm = nm_to_mm(rep.y);
        v.layer = layer;
        violations.push_back(v);
    }

    // Paired-via count/style/span symmetry (deterministic sorted compare).
    {
        std::string mismatch;
        Point mat{0, 0};
        LayerId mlayer = 0;
        auto by_key = [](const Via& a, const Via& b) {
            if (a.pos.x != b.pos.x) return a.pos.x < b.pos.x;
            if (a.pos.y != b.pos.y) return a.pos.y < b.pos.y;
            if (a.top_layer != b.top_layer) return a.top_layer < b.top_layer;
            if (a.bottom_layer != b.bottom_layer)
                return a.bottom_layer < b.bottom_layer;
            if (a.via_class != b.via_class) return a.via_class < b.via_class;
            if (a.outer_d_nm != b.outer_d_nm) return a.outer_d_nm < b.outer_d_nm;
            return a.hole_d_nm < b.hole_d_nm;
        };
        std::vector<Via> sp = vp, sn = vn;
        std::sort(sp.begin(), sp.end(), by_key);
        std::sort(sn.begin(), sn.end(), by_key);
        if (sp.size() != sn.size()) {
            mismatch = "count:" + std::to_string(sp.size()) + "v" +
                       std::to_string(sn.size());
            if (!sp.empty()) {
                mat = sp.front().pos;
                mlayer = sp.front().top_layer;
            } else if (!sn.empty()) {
                mat = sn.front().pos;
                mlayer = sn.front().top_layer;
            } else {
                auto [rep, layer] = first_copper_rep();
                mat = rep;
                mlayer = layer;
            }
        } else {
            for (std::size_t i = 0; i < sp.size(); ++i) {
                if (sp[i].top_layer != sn[i].top_layer ||
                    sp[i].bottom_layer != sn[i].bottom_layer) {
                    mismatch =
                        "span:P[" + std::to_string(sp[i].top_layer) + "," +
                        std::to_string(sp[i].bottom_layer) + "]vN[" +
                        std::to_string(sn[i].top_layer) + "," +
                        std::to_string(sn[i].bottom_layer) + "]";
                    mat = {(sp[i].pos.x + sn[i].pos.x) / 2,
                           (sp[i].pos.y + sn[i].pos.y) / 2};
                    mlayer = sp[i].top_layer;
                    break;
                }
                if (sp[i].via_class != sn[i].via_class ||
                    sp[i].outer_d_nm != sn[i].outer_d_nm ||
                    sp[i].hole_d_nm != sn[i].hole_d_nm) {
                    mismatch = "style:'" + sp[i].via_class + "'v'" +
                               sn[i].via_class + "'";
                    mat = {(sp[i].pos.x + sn[i].pos.x) / 2,
                           (sp[i].pos.y + sn[i].pos.y) / 2};
                    mlayer = sp[i].top_layer;
                    break;
                }
            }
        }
        d.via_mismatch = mismatch;
        if (!mismatch.empty()) {
            parts.push_back("VIA_MISMATCH:" + mismatch);
            Violation v;
            v.type = "diffpair_via";
            v.net_a = pr.net_p;
            v.net_b = pr.net_n;
            v.rule = "diffpair:" + pr.name;
            v.detail =
                "diffpair " + pr.name + " via mismatch " + mismatch;
            v.x_mm = nm_to_mm(mat.x);
            v.y_mm = nm_to_mm(mat.y);
            v.layer = mlayer;
            violations.push_back(v);
        }
    }

    if (parts.empty()) {
        d.status = "OK";
        d.ok = true;
    } else {
        d.status.clear();
        for (std::size_t i = 0; i < parts.size(); ++i) {
            if (i) d.status += "+";
            d.status += parts[i];
        }
        d.ok = false;
    }
    return d;
}

}  // namespace

VerifyResult BoardVerifier::verify(const Board& board, const RuleResolver& resolver,
                                   const ElectricalContext& ctx) const {
    VerifyResult out;

    // ---- Collect elements ----
    std::vector<Element> elems;
    for (const auto& t : board.terminals) {
        Element e;
        e.kind = Element::Kind::kPad;
        e.net = t.net;
        e.layer = t.layer;
        e.rect = t.pad_rect();
        e.term_id = t.id;
        elems.push_back(e);
    }
    for (const auto& t : board.traces) {
        Element e;
        e.kind = Element::Kind::kTrace;
        e.net = t.net;
        e.layer = t.layer;
        e.seg = t.segment();
        e.width = t.width_nm;
        elems.push_back(e);
    }
    for (const auto& v : board.vias) {
        Element e;
        e.kind = Element::Kind::kVia;
        e.net = v.net;
        e.lo = std::min(v.top_layer, v.bottom_layer);
        e.hi = std::max(v.top_layer, v.bottom_layer);
        e.rect = Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm);
        elems.push_back(e);
    }
    // Issue #16: declared pours are fixed copper. Same-net contact counts
    // toward connectivity (valid island contact satisfies the net);
    // foreign-net pours participate in clearance like any other copper.
    for (const auto& z : board.planes) {
        Element e;
        e.kind = Element::Kind::kPlane;
        e.net = z.net;
        e.layer = z.layer;
        e.rect = z.bounds();
        e.plane_id = z.id;
        e.plane_island = z.island;
        e.poly = &z.poly;
        elems.push_back(e);
    }

    // ---- Connectivity per net ----
    DSU dsu(static_cast<int>(elems.size()));
    SpatialIndex index(mm_to_nm(1.0));
    for (std::size_t i = 0; i < elems.size(); ++i) {
        IndexedRect r;
        r.rect = elem_bounds(elems[i]);
        r.net = elems[i].net;
        r.layer = elems[i].layer;
        r.index = static_cast<int>(i);
        index.insert(r);
    }
    for (std::size_t i = 0; i < elems.size(); ++i) {
        for (int j : index.query(elem_bounds(elems[i]))) {
            if (j <= static_cast<int>(i)) continue;
            if (elems[i].net != elems[j].net) continue;
            if (elem_touch(elems[i], elems[j])) dsu.unite(static_cast<int>(i), j);
        }
    }
    // Issue #16: declared island stitching. Same-(net, island) pours are one
    // electrical network even with no geometric proximity (stitched through
    // other layers), so they unite directly instead of via spatial overlap.
    for (std::size_t i = 0; i < elems.size(); ++i) {
        if (elems[i].kind != Element::Kind::kPlane) continue;
        for (std::size_t j = i + 1; j < elems.size(); ++j) {
            if (elems[j].kind != Element::Kind::kPlane) continue;
            if (elems[i].net != elems[j].net) continue;
            if (elems[i].plane_island == elems[j].plane_island)
                dsu.unite(static_cast<int>(i), static_cast<int>(j));
        }
    }
    out.connected = true;
    for (const auto& net : board.nets) {
        if (net.terminals.size() < 2) continue;
        // Find element indices of this net's pads.
        std::vector<int> pad_elems;
        for (std::size_t i = 0; i < elems.size(); ++i) {
            if (elems[i].kind == Element::Kind::kPad && elems[i].net == net.id)
                pad_elems.push_back(static_cast<int>(i));
        }
        if (pad_elems.empty()) continue;
        int root = dsu.find(pad_elems[0]);
        for (std::size_t k = 1; k < pad_elems.size(); ++k) {
            if (dsu.find(pad_elems[k]) != root) {
                out.connected = false;
                // Attribute to the disconnected pad's terminal.
                for (TermId tid : net.terminals) {
                    const Terminal* t = board.find_terminal(tid);
                    if (!t) continue;
                    for (int ei : pad_elems) {
                        if (elems[ei].term_id == tid && dsu.find(ei) != root) {
                            Unconnected u;
                            u.net = net.id;
                            u.net_name = net.name;
                            u.terminal = tid;
                            u.component = t->component;
                            u.pin = t->pin;
                            out.unconnected.push_back(u);
                            break;
                        }
                    }
                }
                break;
            }
        }
    }

    // ---- Width + via rules ----
    for (const auto& t : board.traces) {
        std::string source;
        Coord need = resolver.requiredTraceWidth(t.net, t.layer, ctx, &source);
        if (t.width_nm < need) {
            const NetInfo* n = board.find_net(t.net);
            Violation v;
            v.type = "width";
            v.net_a = t.net;
            v.rule = source;
            v.detail = "trace width " + std::to_string(nm_to_mm(t.width_nm)) + "mm below " +
                       std::to_string(nm_to_mm(need)) + "mm required for net " +
                       (n ? n->name : "?");
            v.x_mm = nm_to_mm((t.a.x + t.b.x) / 2);
            v.y_mm = nm_to_mm((t.a.y + t.b.y) / 2);
            v.layer = t.layer;
            out.violations.push_back(v);
        }
        Rect r = t.segment().bounds().expanded(t.width_nm / 2);
        if (!board.bounds().contains(r)) {
            Violation v;
            v.type = "off_board";
            v.net_a = t.net;
            v.rule = "board_bounds";
            v.detail = "trace copper leaves the board outline";
            v.x_mm = nm_to_mm((t.a.x + t.b.x) / 2);
            v.y_mm = nm_to_mm((t.a.y + t.b.y) / 2);
            v.layer = t.layer;
            out.violations.push_back(v);
        }
    }
    // ---- Issue #11: controlled-impedance check ----
    // Every trace of an impedance-controlled net must sit on an eligible
    // layer with an estimated impedance inside target +/- tolerance. The
    // width floor above already enforces the ampacity minimum, so this
    // predicate is purely the impedance half of the reconciliation.
    for (const auto& t : board.traces) {
        const NetInfo* n = board.find_net(t.net);
        if (!n || !resolver.impedance().has_target(*n)) continue;
        double tol = resolver.impedance().tolerance_for(*n);
        std::string model;
        double est = resolver.impedanceEstimate(t.net, t.layer, t.width_nm, ctx,
                                                &model);
        auto rep = [&]() {
            Violation v;
            v.net_a = t.net;
            v.x_mm = nm_to_mm((t.a.x + t.b.x) / 2);
            v.y_mm = nm_to_mm((t.a.y + t.b.y) / 2);
            v.layer = t.layer;
            return v;
        };
        if (est < 0) {
            Violation v = rep();
            v.type = "impedance";
            v.rule = "impedance_ineligible_layer";
            v.detail = "net " + n->name + " trace on layer " +
                       std::to_string(t.layer) +
                       " has no impedance stackup (no dielectric/er or non-signal)";
            out.violations.push_back(v);
            continue;
        }
        double err = std::fabs(est - n->target_impedance_ohms) / n->target_impedance_ohms;
        if (err > tol + 1e-9) {
            Violation v = rep();
            v.type = "impedance";
            v.rule = model.empty() ? "impedance" : ("impedance_" + model);
            v.detail = "net " + n->name + " estimated " + std::to_string(est) +
                       " ohms vs target " + std::to_string(n->target_impedance_ohms) +
                       " ohms (+/-" + std::to_string(tol * 100.0) + "%, error " +
                       std::to_string(err * 100.0) + "%) on layer " +
                       std::to_string(t.layer) + " [" + model + "]";
            out.violations.push_back(v);
        }
    }
    for (const auto& v : board.vias) {
        const NetInfo* n = board.find_net(v.net);
        if (!n) continue;
        bool dummy = false;
        double current = resolver.current().effective_current(*n, board.defaults, dummy);
        // Resolve the style actually used: named class wins when present so a
        // legal-but-small class reports over-current instead of "unknown".
        ViaStyle style;
        bool have_style = false;
        bool named_unknown = false;
        if (!v.via_class.empty()) {
            have_style = resolver.lookup_via_style(v.via_class, style);
            named_unknown = !have_style;
        } else {
            LayerSpan span{v.top_layer, v.bottom_layer};
            have_style = resolver.select_via(v.net, span, style);
        }
        if (named_unknown) {
            Violation x;
            x.type = "via_class";
            x.net_a = v.net;
            x.rule = "via_class_unknown";
            x.detail = "via uses unknown class '" + v.via_class + "'";
            x.x_mm = nm_to_mm(v.pos.x);
            x.y_mm = nm_to_mm(v.pos.y);
            out.violations.push_back(x);
        } else if (!have_style) {
            // No single via carries this current. Routed bundles always name
            // their class (see the cluster check below), so class-less copper
            // here is user input: report the parallel count the net needs.
            int need = 1;
            std::string best;
            for (const auto& s : resolver.via_styles()) {
                bool dummy2 = false;
                double ic = resolver.current().effective_current(*n, board.defaults, dummy2);
                int k = s.max_current_a > 0
                            ? std::max(1, static_cast<int>(
                                              std::ceil(ic / s.max_current_a)))
                            : 1;
                if (best.empty() || k < need) {
                    need = k;
                    best = s.name;
                }
            }
            Violation x;
            x.type = "via_current";
            x.net_a = v.net;
            x.rule = "via_current_class";
            x.detail = "no via class meets " + std::to_string(current) + "A for net " +
                       n->name + " (needs " + std::to_string(need) +
                       " parallel vias of '" + best + "')";
            x.x_mm = nm_to_mm(v.pos.x);
            x.y_mm = nm_to_mm(v.pos.y);
            out.violations.push_back(x);
        } else if (current > style.max_current_a) {
            // Parallel-bundle allowance (issue #5): a via over its single
            // limit still passes when it sits inside a full same-net bundle
            // of its class (enough neighbours within the bundle diameter to
            // share the current). An isolated over-current via still fails.
            bool dummy3 = false;
            double ic = resolver.current().effective_current(*n, board.defaults, dummy3);
            int need = style.max_current_a > 0
                           ? std::max(1, static_cast<int>(
                                             std::ceil(ic / style.max_current_a)))
                           : 1;
            int nearby = 0;
            if (need > 1) {
                Coord window = 2 * via_bundle_radius(style, need);
                LayerId lo = std::min(v.top_layer, v.bottom_layer);
                LayerId hi = std::max(v.top_layer, v.bottom_layer);
                for (const auto& w : board.vias) {
                    if (w.net != v.net) continue;
                    if (w.via_class != v.via_class) continue;
                    if (w.bottom_layer < lo || w.top_layer > hi) continue;
                    if (manhattan(w.pos, v.pos) <= window) ++nearby;
                }
            }
            if (need <= 1 || nearby < need) {
                Violation x;
                x.type = "via_current";
                x.net_a = v.net;
                x.rule = "via_current_class";
                x.detail = "via class '" + style.name + "' carries " +
                           std::to_string(current) + "A over " +
                           std::to_string(style.max_current_a) + "A limit";
                if (need > 1)
                    x.detail += " (needs " + std::to_string(need) + " in parallel, found " +
                                std::to_string(nearby) + " nearby)";
                x.x_mm = nm_to_mm(v.pos.x);
                x.y_mm = nm_to_mm(v.pos.y);
                out.violations.push_back(x);
            }
        }
    }

    // ---- Clearance between foreign copper ----
    Coord max_clear = board.defaults.clearance_nm;
    for (std::size_t i = 0; i < board.nets.size(); ++i) {
        for (std::size_t j = i + 1; j < board.nets.size(); ++j) {
            std::string cs;
            max_clear = std::max(
                max_clear, resolver.requiredClearance(board.nets[i].id, board.nets[j].id, 0, ctx, &cs));
        }
    }
    for (std::size_t i = 0; i < elems.size(); ++i) {
        Rect area = elem_bounds(elems[i]).expanded(max_clear + 1);
        for (int j : index.query(area)) {
            if (j <= static_cast<int>(i)) continue;
            const Element& a = elems[i];
            const Element& b = elems[j];
            if (a.net == b.net) continue;
            // Issue #13: P/N coupling is governed by the pair gap, not by
            // the voltage table. Generic clearance skips declared pairs;
            // verify_one_pair enforces the gap exactly below.
            if (pair_declared(board, a.net, b.net)) continue;
            if (!elem_layer_overlap(a, b)) continue;
            std::string source;
            Coord need = resolver.requiredClearance(a.net, b.net, 0, ctx, &source);
            if (!elem_gap_ok(a, b, need)) {
                const NetInfo* na = board.find_net(a.net);
                const NetInfo* nb = board.find_net(b.net);
                Violation v;
                v.type = "clearance";
                v.net_a = a.net;
                v.net_b = b.net;
                v.rule = source;
                double gap_mm = elem_gap_mm(a, b);
                v.detail = "clearance " + std::to_string(gap_mm) + "mm below " +
                           std::to_string(nm_to_mm(need)) + "mm between " +
                           (na ? na->name : "?") + " and " + (nb ? nb->name : "?");
                // Explainability (issue #6): when a per-net floor raised the
                // stage-1 candidate, report both so agents can see why.
                ClearanceResolution res = resolver.clearanceResolution(a.net, b.net, 0, ctx);
                if (res.floor_applied) {
                    v.detail += " [candidate=" + res.candidate_source + " " +
                                std::to_string(nm_to_mm(res.candidate_nm)) + "mm floor=" +
                                std::to_string(nm_to_mm(res.floor_nm)) + "mm]";
                }
                Point rep = elem_rep(a, b);
                v.x_mm = nm_to_mm(rep.x);
                v.y_mm = nm_to_mm(rep.y);
                v.layer = a.layer;
                out.violations.push_back(v);
            }
        }
    }

    // ---- Keepout intrusions ----
    for (const auto& ko : board.keepouts) {
        for (const auto& t : board.traces) {
            if (ko.layer != kAllLayers && ko.layer != t.layer) continue;
            Rect copper = t.segment().bounds().expanded(t.width_nm / 2);
            if (copper.intersects(ko.rect)) {
                Violation v;
                v.type = "overlap";
                v.net_a = t.net;
                v.rule = "keepout";
                v.detail = "trace enters keepout" +
                           (ko.reason.empty() ? "" : " (" + ko.reason + ")");
                v.x_mm = nm_to_mm((t.a.x + t.b.x) / 2);
                v.y_mm = nm_to_mm((t.a.y + t.b.y) / 2);
                v.layer = t.layer;
                out.violations.push_back(v);
            }
        }
        for (const auto& t : board.terminals) {
            if (ko.layer != kAllLayers && ko.layer != t.layer) continue;
            // Pads landing in keepouts: flag only when the pad center is inside
            // (pads are fixed input; full DRC of imports is Prompt-5 work).
            if (ko.rect.contains(t.pos)) {
                Violation v;
                v.type = "overlap";
                v.net_a = t.net;
                v.rule = "keepout";
                v.detail = "pad inside keepout" +
                           (ko.reason.empty() ? "" : " (" + ko.reason + ")");
                v.x_mm = nm_to_mm(t.pos.x);
                v.y_mm = nm_to_mm(t.pos.y);
                v.layer = t.layer;
                out.violations.push_back(v);
            }
        }
    }

    // ---- Issue #13: pair-aware verification from committed copper ----
    // Pair declarations sorted by id for determinism. Uses traces/vias/pads
    // only, never router metadata. Each failing aspect is a hard violation
    // so the final success gate (COMPLETE requires verifier clean) refuses
    // bad pairs.
    {
        std::vector<DiffPair> pairs_sorted = board.diffpairs;
        std::sort(pairs_sorted.begin(), pairs_sorted.end(),
                  [](const DiffPair& a, const DiffPair& b) { return a.id < b.id; });
        for (const auto& pr : pairs_sorted) {
            out.pairs.push_back(verify_one_pair(board, pr, out.violations));
        }
    }

    out.legal = out.violations.empty();
    out.ok = out.connected && out.legal;
    return out;
}

}  // namespace copperline
