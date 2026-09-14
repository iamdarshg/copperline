#include "router/verifier.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "router/spatial_index.h"

namespace copperline {

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
    enum class Kind { kPad, kTrace, kVia } kind;
    NetId net = -1;
    LayerId layer = 0;  // pads/traces; vias use lo/hi
    LayerId lo = 0, hi = 0;
    Rect rect{};        // pads + via discs (square approx)
    Segment seg{};      // traces
    Coord width = 0;    // traces
    int term_id = -1;   // pads
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

bool elem_touch(const Element& a, const Element& b) {
    if (!elem_layer_overlap(a, b)) return false;
    if (a.kind == Element::Kind::kTrace && b.kind == Element::Kind::kTrace)
        return seg_intersects_seg(a.seg, b.seg);
    if (a.kind == Element::Kind::kTrace)
        return seg_intersects_rect(a.seg, b.rect);
    if (b.kind == Element::Kind::kTrace)
        return seg_intersects_rect(b.seg, a.rect);
    return a.rect.intersects(b.rect);
}

// Exact copper-edge to copper-edge check: gap >= need, all in integers.
// For centerline-based primitives the half widths are folded into the
// comparison (4*d2 >= (2*need + w)^2) instead of the geometry, so odd-nm
// widths stay exact.
bool elem_gap_ok(const Element& a, const Element& b, Coord need) {
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
            Violation x;
            x.type = "via_current";
            x.net_a = v.net;
            x.rule = "via_current_class";
            x.detail = "no via class meets " + std::to_string(current) + "A for net " + n->name;
            x.x_mm = nm_to_mm(v.pos.x);
            x.y_mm = nm_to_mm(v.pos.y);
            out.violations.push_back(x);
        } else if (current > style.max_current_a) {
            Violation x;
            x.type = "via_current";
            x.net_a = v.net;
            x.rule = "via_current_class";
            x.detail = "via class '" + style.name + "' carries " + std::to_string(current) +
                       "A over " + std::to_string(style.max_current_a) + "A limit";
            x.x_mm = nm_to_mm(v.pos.x);
            x.y_mm = nm_to_mm(v.pos.y);
            out.violations.push_back(x);
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

    out.legal = out.violations.empty();
    out.ok = out.connected && out.legal;
    return out;
}

}  // namespace copperline
