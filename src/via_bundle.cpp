#include "router/via_bundle.h"

#include <algorithm>
#include <cmath>

#include "router/simplify.h"

namespace copperline {

Coord via_bundle_pitch(const ViaStyle& style) {
    // Barrel plus one default-clearance manufacturing step (0.15 mm). Keeps
    // same-net barrels disjoint with a reproducible gap while staying compact
    // enough to fit beside the transition point.
    return style.outer_nm + mm_to_nm(0.15);
}

Coord via_bundle_radius(const ViaStyle& style, int count) {
    if (count <= 1) return style.outer_nm;
    Coord pitch = via_bundle_pitch(style);
    // Longest layout is the row: half length plus one barrel for the rect.
    return ((static_cast<Coord>(count) - 1) * pitch) / 2 + style.outer_nm;
}

namespace {

bool span_hits(LayerId layer, LayerSpan span) {
    return layer >= std::min(span.top, span.bottom) && layer <= std::max(span.top, span.bottom);
}

// D2: rect_gap2/gap_ok_rect/seg_ok_rect live in geometry.h (single definition).
// D4/S4: worst-case clearance comes from the shared ClearanceCache memo.

// One directed legality probe for a via barrel at pos.
// S4: clearances come from the plan-level memo, not per-probe resolver scans.
bool via_pos_legal(const Board& board, NetId net, Point pos, LayerSpan span,
                   const ViaStyle& style, const ClearanceCache& cc,
                   const std::vector<Rect>& sibling_rects, NetId exempt_net = -1) {
    Rect vr = Rect::from_center_size(pos, style.outer_nm, style.outer_nm);
    if (!board.bounds().contains(vr)) return false;
    for (const auto& s : sibling_rects) {
        if (rect_gap(vr, s) <= 0) return false;  // same-net barrels stay disjoint
    }
    Coord max_clear = cc.max_clear();
    for (const auto& ko : board.keepouts) {
        bool hit = ko.layer == kAllLayers || span_hits(ko.layer, span);
        if (!hit) continue;
        if (!gap_ok_rect(vr, ko.rect, max_clear)) return false;
    }
    for (const auto& t : board.terminals) {
        if (t.net == net || t.net == exempt_net) continue;
        if (!span_hits(t.layer, span)) continue;
        Coord c = cc.get(t.net, t.layer);
        if (!gap_ok_rect(vr, t.pad_rect(), c)) return false;
    }
    for (const auto& t : board.traces) {
        if (t.net == net || t.net == exempt_net) continue;
        if (!span_hits(t.layer, span)) continue;
        Coord c = cc.get(t.net, t.layer);
        Coord need = c + t.width_nm / 2;
        if (!seg_ok_rect(t.segment(), vr, need)) return false;
    }
    for (const auto& v : board.vias) {
        if (v.net == net || v.net == exempt_net) continue;
        bool overlap = !(v.bottom_layer < std::min(span.top, span.bottom) ||
                         v.top_layer > std::max(span.top, span.bottom));
        if (!overlap) continue;
        Coord c = cc.get(v.net, v.top_layer);
        Rect orr = Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm);
        if (!gap_ok_rect(vr, orr, c)) return false;
    }
    // Issue #16: bundle barrels keep clearance from foreign pours on spanned
    // layers; own-net pours are connectable and never block.
    for (const auto& z : board.planes) {
        if (z.net == net || z.net == exempt_net) continue;
        if (z.layer < std::min(span.top, span.bottom) ||
            z.layer > std::max(span.top, span.bottom))
            continue;
        Coord c = cc.get(z.net, z.layer);
        if (vr.expanded(c).intersects(z.bounds())) {
            if (plane_rect_poly_dist2(vr, z.poly) < (__int128)c * c) return false;
        }
    }
    return true;
}

// One directed legality probe for a star stub centerline on one layer.
// D3: the shared verifier-exact predicate (exact 4*d2 >= rhs^2, bbox
// prechecks, keepout/plane/pad/trace/via loops). width_nm = 2 * half_w, so
// the folded rhs matches the old need-based predicate exactly for even-nm
// widths; odd-nm razor cases follow the verifier exactly.
bool stub_seg_legal(const Board& board, NetId net, const Segment& s, LayerId layer,
                    Coord half_w, const ClearanceCache& cc, NetId exempt_net = -1) {
    SegLegalityCtx leg;
    leg.cc = cc;  // shares the plan-level memo table
    leg.exempt_net = exempt_net;
    leg.layer = layer;
    leg.width_nm = 2 * half_w;
    return leg.segment_legal(s);
}

// Deterministic offset sets, fixed try order: row, column, compact grid.
std::vector<std::vector<Point>> layout_candidates(Point center, Coord pitch, int count) {
    std::vector<std::vector<Point>> out;
    if (count <= 0) return out;
    // Row (x-major line through center).
    {
        std::vector<Point> row;
        Coord span = (static_cast<Coord>(count) - 1) * pitch;
        for (int i = 0; i < count; ++i)
            row.push_back({center.x + i * pitch - span / 2, center.y});
        out.push_back(row);
    }
    if (count > 1) {
        // Column (y-major line through center).
        std::vector<Point> col;
        Coord span = (static_cast<Coord>(count) - 1) * pitch;
        for (int i = 0; i < count; ++i)
            col.push_back({center.x, center.y + i * pitch - span / 2});
        out.push_back(col);
    }
    if (count > 2) {
        // Compact grid: cols = ceil(sqrt(n)), centered.
        int cols = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(count))));
        int rows = (count + cols - 1) / cols;
        std::vector<Point> grid;
        for (int k = 0; k < count; ++k) {
            int ix = k % cols, iy = k / cols;
            Coord x = center.x + static_cast<Coord>(ix) * pitch -
                      (static_cast<Coord>(cols) - 1) * pitch / 2;
            Coord y = center.y + static_cast<Coord>(iy) * pitch -
                      (static_cast<Coord>(rows) - 1) * pitch / 2;
            grid.push_back({x, y});
        }
        out.push_back(grid);
    }
    return out;
}

}  // namespace

std::vector<ViaStyle> ViaBundlePlanner::ordered_styles(const RuleResolver& resolver, NetId net,
                                                       LayerSpan span) {
    (void)span;
    const Board* board = resolver.board();
    const NetInfo* n = board ? board->find_net(net) : nullptr;
    std::vector<ViaStyle> preferred, rest;
    for (const auto& s : resolver.via_styles()) {
        if (n && !n->via_class.empty() && s.name == n->via_class)
            preferred.push_back(s);
        else
            rest.push_back(s);
    }
    auto rank = [&](const ViaStyle& s) {
        ElectricalContext ctx;
        int need = 1;
        if (board && n) need = resolver.current().vias_required(s, *n, board->defaults, ctx);
        return need;
    };
    std::sort(rest.begin(), rest.end(), [&](const ViaStyle& a, const ViaStyle& b) {
        int na = rank(a), nb = rank(b);
        if (na != nb) return na < nb;
        if (a.outer_nm != b.outer_nm) return a.outer_nm < b.outer_nm;
        return a.name < b.name;
    });
    preferred.insert(preferred.end(), rest.begin(), rest.end());
    return preferred;
}

int ViaBundlePlanner::required_count(const RuleResolver& resolver, const ViaStyle& style,
                                     NetId net) {
    const Board* board = resolver.board();
    if (!board) return 1;
    const NetInfo* n = board->find_net(net);
    if (!n) return 1;
    ElectricalContext ctx;
    return resolver.current().vias_required(style, *n, board->defaults, ctx);
}

ViaBundle ViaBundlePlanner::plan_with_style(const Board& board, const RuleResolver& resolver,
                                             NetId net, Point center, LayerSpan span,
                                             const ViaStyle& style, int count,
                                             Coord route_width_nm,
                                             const ElectricalContext& ctx,
                                             NetId exempt_net) {
    // D4/S4: one plan-level clearance memo for every barrel + stub probe.
    ClearanceCache cc(board, resolver, ctx, net);
    return plan_with_style(board, resolver, net, center, span, style, count, route_width_nm,
                           ctx, cc, exempt_net);
}

ViaBundle ViaBundlePlanner::plan_with_style(const Board& board, const RuleResolver& resolver,
                                             NetId net, Point center, LayerSpan span,
                                             const ViaStyle& style, int count,
                                             Coord route_width_nm,
                                             const ElectricalContext& ctx,
                                             const ClearanceCache& cc, NetId exempt_net) {
    ViaBundle out;
    out.style = style;
    out.count = std::max(1, count);
    out.via_class = style.name;
    const NetInfo* n = board.find_net(net);
    if (!n) {
        out.reason = "no_via_class";
        return out;
    }
    bool dummy = false;
    out.required_current_a = resolver.current().effective_current(*n, board.defaults, dummy);
    if (style.max_current_a <= 0 || style.outer_nm <= 0) {
        out.reason = "no_via_class";
        return out;
    }
    // cc is the caller's shared per-net clearance memo (pure in (board, net
    // pair)); barrel + stub probes below hit it instead of refilling per call.
    Coord pitch = via_bundle_pitch(style);
    Coord half_w = route_width_nm / 2;
    LayerId top = std::min(span.top, span.bottom);
    LayerId bottom = std::max(span.top, span.bottom);
    // Span endpoints for the star stubs: the transition's own layers.
    for (const auto& positions : layout_candidates(center, pitch, out.count)) {
        bool ok = true;
        std::vector<Rect> placed;
        placed.reserve(positions.size());
        for (auto p : positions) {
            if (!via_pos_legal(board, net, p, span, style, cc, placed, exempt_net)) {
                ok = false;
                break;
            }
            placed.push_back(Rect::from_center_size(p, style.outer_nm, style.outer_nm));
        }
        if (!ok) continue;
        // Star stubs: center -> each satellite on both transition layers.
        // The center barrel itself needs no stub.
        std::vector<TraceSeg> stubs;
        for (auto p : positions) {
            if (p == center) continue;
            for (LayerId layer : {top, bottom}) {
                if (top == bottom && layer != top) continue;
                Segment s{center, p};
                if (!stub_seg_legal(board, net, s, layer, half_w, cc, exempt_net)) {
                    ok = false;
                    break;
                }
                stubs.push_back({net, layer, center, p, route_width_nm});
            }
            if (!ok) break;
            if (top == bottom) continue;
        }
        if (!ok) continue;
        // Same-layer single-span transitions only need one stub layer.
        out.feasible = true;
        out.reason = "ok";
        out.positions = positions;
        out.stubs = stubs;
        return out;
    }
    out.reason = "bundle_blocked";
    return out;
}

ViaBundle ViaBundlePlanner::plan(const Board& board, const RuleResolver& resolver, NetId net,
                                 Point center, LayerSpan span, Coord route_width_nm,
                                 const ElectricalContext& ctx, NetId exempt_net) {
    ViaBundle fail;
    fail.reason = "no_via_class";
    const NetInfo* n = board.find_net(net);
    if (!n) return fail;
    bool tried = false;
    for (const auto& style : ordered_styles(resolver, net, span)) {
        tried = true;
        ElectricalContext c2 = ctx;
        int need = resolver.current().vias_required(style, *n, board.defaults, c2);
        ViaBundle b =
            plan_with_style(board, resolver, net, center, span, style, need,
                            route_width_nm, ctx, exempt_net);
        if (b.feasible) return b;
        fail = b;  // keep the last deterministic reason (bundle_blocked)
    }
    if (!tried) {
        fail.reason = "no_via_class";
        fail.count = 0;
    }
    return fail;
}

}  // namespace copperline
