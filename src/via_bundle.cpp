#include "router/via_bundle.h"

#include <algorithm>
#include <cmath>

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

__int128 rect_gap2(const Rect& a, const Rect& b) {
    Coord dx = 0, dy = 0;
    if (a.x2 < b.x1) dx = b.x1 - a.x2;
    else if (b.x2 < a.x1) dx = a.x1 - b.x2;
    if (a.y2 < b.y1) dy = b.y1 - a.y2;
    else if (b.y2 < a.y1) dy = a.y1 - b.y2;
    return (__int128)dx * dx + (__int128)dy * dy;
}

bool gap_ok(const Rect& a, const Rect& b, Coord need) {
    if (!a.expanded(need).intersects(b)) return true;
    return rect_gap2(a, b) >= (__int128)need * need;
}

bool seg_ok_rect(const Segment& s, const Rect& raw, Coord need) {
    if (!s.bounds().expanded(need).intersects(raw)) return true;
    return seg_rect_dist2(s, raw) >= (__int128)need * need;
}

// One directed legality probe for a via barrel at pos.
bool via_pos_legal(const Board& board, const RuleResolver& resolver, NetId net, Point pos,
                   LayerSpan span, const ViaStyle& style, Coord max_clear,
                   const ElectricalContext& ctx, const std::vector<Rect>& sibling_rects) {
    Rect vr = Rect::from_center_size(pos, style.outer_nm, style.outer_nm);
    if (!board.bounds().contains(vr)) return false;
    for (const auto& s : sibling_rects) {
        if (rect_gap(vr, s) <= 0) return false;  // same-net barrels stay disjoint
    }
    std::string cs;
    for (const auto& ko : board.keepouts) {
        bool hit = ko.layer == kAllLayers || span_hits(ko.layer, span);
        if (!hit) continue;
        if (!gap_ok(vr, ko.rect, max_clear)) return false;
    }
    for (const auto& t : board.terminals) {
        if (t.net == net) continue;
        if (!span_hits(t.layer, span)) continue;
        Coord c = resolver.requiredClearance(net, t.net, t.layer, ctx, &cs);
        if (!gap_ok(vr, t.pad_rect(), c)) return false;
    }
    for (const auto& t : board.traces) {
        if (t.net == net) continue;
        if (!span_hits(t.layer, span)) continue;
        Coord c = resolver.requiredClearance(net, t.net, t.layer, ctx, &cs);
        Coord need = c + t.width_nm / 2;
        if (!seg_ok_rect(t.segment(), vr, need)) return false;
    }
    for (const auto& v : board.vias) {
        if (v.net == net) continue;
        bool overlap = !(v.bottom_layer < std::min(span.top, span.bottom) ||
                         v.top_layer > std::max(span.top, span.bottom));
        if (!overlap) continue;
        Coord c = resolver.requiredClearance(net, v.net, v.top_layer, ctx, &cs);
        Rect orr = Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm);
        if (!gap_ok(vr, orr, c)) return false;
    }
    return true;
}

// One directed legality probe for a star stub centerline on one layer.
bool stub_seg_legal(const Board& board, const RuleResolver& resolver, NetId net,
                    const Segment& s, LayerId layer, Coord half_w, Coord max_clear,
                    const ElectricalContext& ctx) {
    if (!board.bounds().contains(s.bounds().expanded(half_w))) return false;
    std::string cs;
    for (const auto& ko : board.keepouts) {
        if (ko.layer != kAllLayers && ko.layer != layer) continue;
        if (!seg_ok_rect(s, ko.rect, max_clear + half_w)) return false;
    }
    for (const auto& t : board.terminals) {
        if (t.net == net || t.layer != layer) continue;
        Coord c = resolver.requiredClearance(net, t.net, layer, ctx, &cs);
        if (!seg_ok_rect(s, t.pad_rect(), c + half_w)) return false;
    }
    for (const auto& t : board.traces) {
        if (t.net == net || t.layer != layer) continue;
        Coord c = resolver.requiredClearance(net, t.net, layer, ctx, &cs);
        Segment b = t.segment();
        if (!s.bounds().expanded(c + half_w + t.width_nm / 2).intersects(b.bounds()))
            continue;
        Coord need = c + half_w + t.width_nm / 2;
        if (seg_seg_dist2(s, b) < (__int128)need * need) return false;
    }
    for (const auto& v : board.vias) {
        if (v.net == net) continue;
        if (layer < std::min(v.top_layer, v.bottom_layer) ||
            layer > std::max(v.top_layer, v.bottom_layer))
            continue;
        Coord c = resolver.requiredClearance(net, v.net, layer, ctx, &cs);
        Rect vr = Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm);
        if (!seg_ok_rect(s, vr, c + half_w)) return false;
    }
    return true;
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
                                            const ElectricalContext& ctx) {
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
    Coord max_clear = max_clear_for_net(board, resolver, net, ctx);
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
            if (!via_pos_legal(board, resolver, net, p, span, style, max_clear, ctx,
                               placed)) {
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
                if (!stub_seg_legal(board, resolver, net, s, layer, half_w, max_clear,
                                    ctx)) {
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
                                 const ElectricalContext& ctx) {
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
                            route_width_nm, ctx);
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
