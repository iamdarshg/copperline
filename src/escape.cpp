#include "router/escape.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <sstream>

#include "router/astar.h"
#include "router/density.h"
#include "router/simplify.h"
#include "router/sparse_graph.h"

namespace copperline {
namespace {

double euclid_nm(Point a, Point b) {
    double dx = static_cast<double>(a.x - b.x);
    double dy = static_cast<double>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}

constexpr int kExitDx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
constexpr int kExitDy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

inline Coord pitch_fallback(const FinePitchFootprint& fp) {
    return fp.pitch_nm > 0 ? fp.pitch_nm : mm_to_nm(0.8);
}

// Local window board: escape search is local (pad -> boundary portal), so the
// sparse graph only needs copper intersecting the pad->portal corridor window
// (probe-verified: identical A* outcomes at ~1/5 the nodes). Nets, layers,
// defaults and bounds are kept intact (the rule engine iterates nets by id);
// only geometric obstacles are filtered. Finalists are always re-checked
// against the full board by escape_candidate_legal.
inline Board local_escape_board(const Board& work, const Rect& window) {
    Board local;
    local.width_nm = work.width_nm;
    local.height_nm = work.height_nm;
    local.layers = work.layers;
    local.nets = work.nets;
    local.defaults = work.defaults;
    local.source_format = work.source_format;
    local.source_file = work.source_file;
    for (const auto& t : work.terminals) {
        if (t.pad_rect().intersects(window)) local.terminals.push_back(t);
    }
    for (const auto& s : work.traces) {
        if (s.segment().bounds().expanded(s.width_nm).intersects(window))
            local.traces.push_back(s);
    }
    for (const auto& v : work.vias) {
        if (Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm).intersects(window))
            local.vias.push_back(v);
    }
    for (const auto& k : work.keepouts) {
        if (k.rect.intersects(window)) local.keepouts.push_back(k);
    }
    return local;
}

}  // namespace

FinePitchDetector::FinePitchDetector(Options options) : options_(options) {
    if (options_.pitch_threshold_nm <= 0)
        options_.pitch_threshold_nm = mm_to_nm(1.27);
    if (options_.min_pins <= 0) options_.min_pins = 8;
}

std::vector<FinePitchFootprint> FinePitchDetector::detect(
    const Board& board, const RuleResolver& resolver, const ElectricalContext& ctx) const {
    std::map<std::string, std::vector<std::size_t>> groups;
    for (std::size_t i = 0; i < board.terminals.size(); ++i) {
        const auto& t = board.terminals[i];
        if (!t.component.empty()) {
            groups["comp:" + t.component].push_back(i);
        } else {
            long long bx = static_cast<long long>(t.pos.x / mm_to_nm(4.0));
            long long by = static_cast<long long>(t.pos.y / mm_to_nm(4.0));
            groups["cell:" + std::to_string(bx) + "," + std::to_string(by)].push_back(i);
        }
    }

    DensityEstimator density_est;
    DensityResult density = density_est.analyze(board);

    std::vector<FinePitchFootprint> out;
    for (const auto& [key, members] : groups) {
        if (members.size() < 4) continue;
        FinePitchFootprint fp;
        if (key.rfind("comp:", 0) == 0)
            fp.component = key.substr(5);
        else
            fp.component = key;
        for (std::size_t i : members) fp.members.push_back(board.terminals[i].id);
        std::sort(fp.members.begin(), fp.members.end());
        fp.pad_count = static_cast<int>(fp.members.size());

        Coord x1 = board.terminals[members[0]].pos.x, y1 = board.terminals[members[0]].pos.y;
        Coord x2 = x1, y2 = y1;
        Coord pw = 0, ph = 0;
        double peak_dens = 0;
        for (std::size_t i : members) {
            const auto& t = board.terminals[i];
            x1 = std::min(x1, t.pos.x);
            y1 = std::min(y1, t.pos.y);
            x2 = std::max(x2, t.pos.x);
            y2 = std::max(y2, t.pos.y);
            pw = std::max(pw, t.pad_w_nm);
            ph = std::max(ph, t.pad_h_nm);
            peak_dens = std::max(peak_dens, density.terminal_density[i]);
        }
        fp.bbox = {x1, y1, x2, y2};
        fp.centroid = {(x1 + x2) / 2, (y1 + y2) / 2};
        Coord dx = x2 - x1, dy = y2 - y1;
        fp.radius_nm = std::max(dx, dy) / 2;
        fp.pad_w_nm = pw;
        fp.pad_h_nm = ph;

        // Nearest-neighbour pitch (min over members).
        double min_pitch = std::numeric_limits<double>::max();
        for (std::size_t a = 0; a < members.size(); ++a) {
            for (std::size_t b = a + 1; b < members.size(); ++b) {
                double d = euclid_nm(board.terminals[members[a]].pos,
                                     board.terminals[members[b]].pos);
                if (d > 0) min_pitch = std::min(min_pitch, d);
            }
        }
        if (min_pitch == std::numeric_limits<double>::max()) min_pitch = 0;
        fp.pitch_nm = static_cast<Coord>(std::llround(min_pitch));

        // Electrical burden: max width / clearance over member nets.
        Coord wmax = 0, cmax = 0;
        for (TermId tid : fp.members) {
            const Terminal* t = board.find_terminal(tid);
            if (!t) continue;
            std::string ws;
            wmax = std::max(wmax, resolver.requiredTraceWidth(t->net, t->layer, ctx, &ws));
        }
        for (std::size_t a = 0; a < fp.members.size(); ++a) {
            const Terminal* ta = board.find_terminal(fp.members[a]);
            if (!ta) continue;
            for (std::size_t b = a + 1; b < fp.members.size(); ++b) {
                const Terminal* tb = board.find_terminal(fp.members[b]);
                if (!tb) continue;
                std::string cs;
                cmax = std::max(cmax, resolver.requiredClearance(ta->net, tb->net, ta->layer,
                                                                ctx, &cs));
            }
        }
        if (cmax == 0) cmax = board.defaults.clearance_nm;
        fp.trace_width_nm = wmax;
        fp.clearance_nm = cmax;

        double w_mm = nm_to_mm(x2 - x1), h_mm = nm_to_mm(y2 - y1);
        double area = std::max(w_mm * h_mm, 0.25);
        fp.pins_per_mm2 = fp.members.size() / area;
        fp.local_density = peak_dens;

        Coord pad_ext = std::max(pw, ph);
        Coord denom = wmax + cmax;
        if (denom > 0 && min_pitch > 0) {
            double avail = min_pitch - pad_ext - 2.0 * static_cast<double>(cmax);
            fp.channel_count =
                avail <= 0 ? 0 : static_cast<int>(std::floor(avail / static_cast<double>(denom)));
        } else {
            fp.channel_count = 0;
        }

        // Flag decision (deterministic, documented in reason).
        std::string why;
        bool flag = false;
        if (fp.pad_count >= options_.min_pins) {
            if (fp.pitch_nm > 0 && fp.pitch_nm <= options_.pitch_threshold_nm) {
                flag = true;
                why = "pitch_below_threshold";
            } else if (fp.channel_count <= 1) {
                flag = true;
                why = "no_routing_channel";
            } else if (fp.pins_per_mm2 >= 4.0) {
                flag = true;
                why = "high_pin_density";
            } else {
                why = "not_fine_pitch";
            }
        } else {
            why = "too_few_pins";
        }
        fp.is_fine_pitch = flag;
        fp.reason = why;
        if (flag) out.push_back(fp);
    }
    std::sort(out.begin(), out.end(), [](const FinePitchFootprint& a, const FinePitchFootprint& b) {
        return a.component < b.component;
    });
    return out;
}

std::map<TermId, int> CentreDepthAnalyzer::analyze(
    const Board& board, const FinePitchFootprint& footprint) const {
    std::map<TermId, int> depth;
    if (footprint.members.empty()) return depth;

    // Cluster x/y with tolerance derived from pitch.
    Coord tol = footprint.pitch_nm > 0 ? footprint.pitch_nm * 2 / 5 : mm_to_nm(0.2);
    if (tol <= 0) tol = mm_to_nm(0.2);
    std::vector<Coord> xs, ys;
    for (TermId tid : footprint.members) {
        const Terminal* t = board.find_terminal(tid);
        if (!t) continue;
        xs.push_back(t->pos.x);
        ys.push_back(t->pos.y);
    }
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    std::vector<Coord> xclusters, yclusters;
    for (Coord x : xs) {
        if (xclusters.empty() || x - xclusters.back() > tol) xclusters.push_back(x);
    }
    for (Coord y : ys) {
        if (yclusters.empty() || y - yclusters.back() > tol) yclusters.push_back(y);
    }
    auto nearest_cluster = [](Coord v, const std::vector<Coord>& clusters) {
        int best = 0;
        Coord bd = std::numeric_limits<Coord>::max();
        for (std::size_t i = 0; i < clusters.size(); ++i) {
            Coord d = v >= clusters[i] ? v - clusters[i] : clusters[i] - v;
            if (d < bd) {
                bd = d;
                best = static_cast<int>(i);
            }
        }
        return best;
    };
    int ncols = static_cast<int>(xclusters.size());
    int nrows = static_cast<int>(yclusters.size());
    bool grid_like =
        (ncols >= 2 && nrows >= 2 &&
         static_cast<int>(footprint.members.size()) >= (ncols * nrows * 7 / 10));

    Coord pitch = footprint.pitch_nm > 0 ? footprint.pitch_nm : mm_to_nm(0.5);
    for (TermId tid : footprint.members) {
        const Terminal* t = board.find_terminal(tid);
        if (!t) {
            depth[tid] = 0;
            continue;
        }
        if (grid_like) {
            int c = nearest_cluster(t->pos.x, xclusters);
            int r = nearest_cluster(t->pos.y, yclusters);
            int d = std::min(std::min(c, r), std::min(ncols - 1 - c, nrows - 1 - r));
            depth[tid] = std::max(0, d);
        } else {
            Coord edge = std::min(std::min(t->pos.x - footprint.bbox.x1, footprint.bbox.x2 - t->pos.x),
                                  std::min(t->pos.y - footprint.bbox.y1, footprint.bbox.y2 - t->pos.y));
            if (edge < 0) edge = 0;
            depth[tid] = static_cast<int>(edge / pitch);
        }
    }
    return depth;
}

EscapeBoundary build_escape_boundary(const Board& board, const FinePitchFootprint& footprint,
                                     Coord margin_nm) {
    EscapeBoundary b;
    Coord margin = margin_nm;
    if (margin <= 0) {
        Coord pitch = footprint.pitch_nm > 0 ? footprint.pitch_nm : mm_to_nm(0.8);
        margin = std::max(pitch * 2, mm_to_nm(1.0));
        margin = std::max(margin, footprint.clearance_nm + footprint.trace_width_nm + mm_to_nm(0.5));
    }
    Rect r = footprint.bbox.expanded(margin);
    // Clamp inside the board so portals are always routable targets.
    Rect bounds = board.bounds();
    r.x1 = std::max(r.x1, bounds.x1);
    r.y1 = std::max(r.y1, bounds.y1);
    r.x2 = std::min(r.x2, bounds.x2);
    r.y2 = std::min(r.y2, bounds.y2);
    b.rect = r;

    Coord pitch = footprint.pitch_nm > 0 ? footprint.pitch_nm : mm_to_nm(0.5);
    Coord step = std::max(pitch, mm_to_nm(0.4));
    if (step <= 0) step = mm_to_nm(0.5);
    int id = 0;
    // Side 0: E (x=x2), Side 1: N (y=y2), Side 2: W (x=x1), Side 3: S (y=y1).
    for (Coord y = r.y1; y <= r.y2; y += step) {
        b.portals.push_back({id++, {r.x2, y}, 0});
    }
    for (Coord x = r.x1; x <= r.x2; x += step) {
        b.portals.push_back({id++, {x, r.y2}, 1});
    }
    for (Coord y = r.y2; y >= r.y1; y -= step) {
        b.portals.push_back({id++, {r.x1, y}, 2});
    }
    for (Coord x = r.x2; x >= r.x1; x -= step) {
        b.portals.push_back({id++, {x, r.y1}, 3});
    }
    // Ensure corners exist exactly (loop steps may skip x2/y2).
    return b;
}

std::vector<int> legal_exit_sectors(const Board& board, const RuleResolver& resolver,
                                    const ElectricalContext& ctx, const FinePitchFootprint& fp,
                                    TermId terminal, Coord route_width_nm) {
    std::vector<int> out;
    const Terminal* t = board.find_terminal(terminal);
    if (!t) return out;
    Coord half_w = route_width_nm / 2;
    Coord pitch = fp.pitch_nm > 0 ? fp.pitch_nm : mm_to_nm(0.8);
    Coord len = std::max(pitch * 3, half_w + fp.clearance_nm + std::max(t->pad_w_nm, t->pad_h_nm));

    for (int s = 0; s < 8; ++s) {
        double inv = 1.0;
        double dx = kExitDx[s], dy = kExitDy[s];
        double norm = std::sqrt(dx * dx + dy * dy);
        dx /= norm;
        dy /= norm;
        Point dst{static_cast<Coord>(std::llround(t->pos.x + dx * static_cast<double>(len))),
                  static_cast<Coord>(std::llround(t->pos.y + dy * static_cast<double>(len)))};
        Segment seg{t->pos, dst};
        // Must stay on the board (centreline + half width).
        if (!board.bounds().contains(seg.bounds().expanded(half_w))) continue;
        bool ok = true;
        for (const auto& o : board.terminals) {
            if (o.id == terminal) continue;
            std::string cs;
            Coord c = resolver.requiredClearance(t->net, o.net, t->layer, ctx, &cs);
            Coord need = c + half_w;
            __int128 d2 = seg_rect_dist2(seg, o.pad_rect());
            __int128 need2 = (__int128)need * need;
            if (d2 < need2) {
                ok = false;
                break;
            }
        }
        if (!ok) continue;
        for (const auto& ko : board.keepouts) {
            if (ko.layer != kAllLayers && ko.layer != t->layer) continue;
            __int128 d2 = seg_rect_dist2(seg, ko.rect);
            Coord need = fp.clearance_nm + half_w;
            if (d2 < (__int128)need * need) {
                ok = false;
                break;
            }
        }
        if (ok) {
            (void)inv;
            out.push_back(s);
        }
    }
    return out;
}

int via_site_count(const Board& board, const RuleResolver& resolver, const FinePitchFootprint& fp,
                   TermId terminal) {
    const Terminal* t = board.find_terminal(terminal);
    if (!t) return 0;
    ViaStyle style;
    LayerSpan full{board.layers.front().id, board.layers.back().id};
    if (!resolver.select_via(t->net, full, style)) return 0;
    Coord dist = std::max(t->pad_w_nm, t->pad_h_nm) / 2 + style.outer_nm / 2 + fp.clearance_nm;
    if (dist <= 0) dist = mm_to_nm(0.4);
    int count = 0;
    for (int s = 0; s < 8; ++s) {
        double dx = kExitDx[s], dy = kExitDy[s];
        double norm = std::sqrt(dx * dx + dy * dy);
        dx /= norm;
        dy /= norm;
        Point site{static_cast<Coord>(std::llround(t->pos.x + dx * static_cast<double>(dist))),
                   static_cast<Coord>(std::llround(t->pos.y + dy * static_cast<double>(dist)))};
        Rect via_rect = Rect::from_center_size(site, style.outer_nm, style.outer_nm);
        if (!board.bounds().contains(via_rect)) continue;
        bool ok = true;
        for (const auto& o : board.terminals) {
            if (o.id == terminal) continue;
            if (rect_gap(via_rect, o.pad_rect()) < fp.clearance_nm) {
                ok = false;
                break;
            }
        }
        if (!ok) continue;
        for (const auto& ko : board.keepouts) {
            if (rect_gap(via_rect, ko.rect) < fp.clearance_nm) {
                ok = false;
                break;
            }
        }
        if (ok) ++count;
    }
    return count;
}

EscapeDensityMap build_escape_density_map(const Board& board, const FinePitchFootprint& fp) {
    EscapeDensityMap m;
    Coord pitch = fp.pitch_nm > 0 ? fp.pitch_nm : mm_to_nm(0.5);
    m.cell_nm = std::max(pitch / 2, mm_to_nm(0.2));
    Coord w = fp.bbox.x2 - fp.bbox.x1, h = fp.bbox.y2 - fp.bbox.y1;
    m.nx = static_cast<int>(std::max<Coord>(1, (w + m.cell_nm - 1) / m.cell_nm));
    m.ny = static_cast<int>(std::max<Coord>(1, (h + m.cell_nm - 1) / m.cell_nm));
    m.cells.assign(m.nx * m.ny, 0.0);
    double cell_area = nm_to_mm(m.cell_nm) * nm_to_mm(m.cell_nm);
    if (cell_area <= 0) cell_area = 1.0;
    for (TermId tid : fp.members) {
        const Terminal* t = board.find_terminal(tid);
        if (!t) continue;
        int ix = static_cast<int>((t->pos.x - fp.bbox.x1) / m.cell_nm);
        int iy = static_cast<int>((t->pos.y - fp.bbox.y1) / m.cell_nm);
        ix = std::clamp(ix, 0, m.nx - 1);
        iy = std::clamp(iy, 0, m.ny - 1);
        m.cells[iy * m.nx + ix] += 1.0 / cell_area;
    }
    for (double v : m.cells) m.peak = std::max(m.peak, v);
    return m;
}

int principal_direction(Point from, Point to) {
    Coord dx = to.x - from.x, dy = to.y - from.y;
    if (dx == 0 && dy == 0) return 0;
    double adx = std::fabs(static_cast<double>(dx));
    double ady = std::fabs(static_cast<double>(dy));
    bool ex = dx > 0, wx = dx < 0, ny = dy > 0, sy = dy < 0;
    if (adx > 2 * ady) return ex ? 0 : 4;
    if (ady > 2 * adx) return ny ? 2 : 6;
    if (ex && ny) return 1;
    if (wx && ny) return 3;
    if (wx && sy) return 5;
    return 7;
}

std::string candidate_signature(int portal_id, int principal_dir,
                                const std::string& layer_strategy,
                                const std::string& via_class, int channel_count) {
    std::ostringstream ss;
    ss << "p" << portal_id << "|d" << principal_dir << "|" << layer_strategy << "|" << via_class
       << "|ch" << channel_count;
    return ss.str();
}

// Exact direct-escape legality for one segment (arbitrary angle), mirroring the
// escape_candidate_legal gate (keepouts with worst-case clearance, foreign
// pads/traces/vias with pair clearance, on-board copper). Integer-exact.
bool direct_seg_legal(const Board& work, const RuleResolver& resolver,
                      const ElectricalContext& ctx, NetId net, const Segment& s, Coord half_w,
                      LayerId layer, Coord max_clear) {
    if (!work.bounds().contains(s.bounds().expanded(half_w))) return false;
    for (const auto& ko : work.keepouts) {
        if (ko.layer != kAllLayers && ko.layer != layer) continue;
        __int128 d2 = seg_rect_dist2(s, ko.rect);
        Coord need = max_clear + half_w;
        if (d2 < (__int128)need * need) return false;
    }
    for (const auto& o : work.terminals) {
        if (o.net == net || o.layer != layer) continue;
        std::string cs;
        Coord need = resolver.requiredClearance(net, o.net, layer, ctx, &cs) + half_w;
        if (seg_rect_dist2(s, o.pad_rect()) < (__int128)need * need) return false;
    }
    for (const auto& o : work.traces) {
        if (o.net == net || o.layer != layer) continue;
        std::string cs;
        Coord need = resolver.requiredClearance(net, o.net, layer, ctx, &cs);
        __int128 rhs = (__int128)2 * need + 2 * half_w + o.width_nm;
        if ((__int128)4 * seg_seg_dist2(s, o.segment()) < rhs * rhs) return false;
    }
    for (const auto& v : work.vias) {
        if (v.net == net) continue;
        LayerId lo = std::min(v.top_layer, v.bottom_layer);
        LayerId hi = std::max(v.top_layer, v.bottom_layer);
        if (layer < lo || layer > hi) continue;
        std::string cs;
        Coord need = resolver.requiredClearance(net, v.net, layer, ctx, &cs);
        Rect via_rect = Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm);
        __int128 rhs = (__int128)2 * need + 2 * half_w;
        if ((__int128)4 * seg_rect_dist2(s, via_rect) < rhs * rhs) return false;
    }
    return true;
}

// Direct Manhattan escape attempt: straight when aligned, else the first
// legal elbow order. No graph, no A* (microseconds). False when the pad
// cannot reach this portal without crossing foreign copper.
bool try_direct_candidate(const Board& work, const RuleResolver& resolver,
                          const ElectricalContext& ctx, const FinePitchFootprint& fp,
                          const Terminal& t, const EscapePortal& portal, Coord width,
                          bool is_neck, EscapeCandidate& out) {
    Coord half_w = width / 2;
    Coord max_clear = fp.clearance_nm;
    for (const auto& other : work.nets) {
        if (other.id == t.net) continue;
        std::string cs;
        max_clear = std::max(max_clear, resolver.requiredClearance(t.net, other.id, t.layer,
                                                                   ctx, &cs));
    }
    const Point a = t.pos, b = portal.pos;
    Point elbow{};
    bool has_elbow = false;
    // Issue #17: prefer the direct arbitrary-angle segment when exactly
    // legal (a clear diagonal is one natural segment, never 45-straight-45
    // or an elbow). Manhattan elbows are the fallback only.
    if (a == b) {
        // Degenerate: pad already at the portal; no copper needed.
    } else if (direct_seg_legal(work, resolver, ctx, t.net, {a, b}, half_w, t.layer,
                                max_clear)) {
        // Single natural-angle segment; handled by the shared epilogue.
    } else if (a.x == b.x || a.y == b.y) {
        return false;
    } else {
        Point e1{b.x, a.y}, e2{a.x, b.y};
        if (direct_seg_legal(work, resolver, ctx, t.net, {a, e1}, half_w, t.layer, max_clear) &&
            direct_seg_legal(work, resolver, ctx, t.net, {e1, b}, half_w, t.layer, max_clear)) {
            elbow = e1;
            has_elbow = true;
        } else if (direct_seg_legal(work, resolver, ctx, t.net, {a, e2}, half_w, t.layer,
                                    max_clear) &&
                   direct_seg_legal(work, resolver, ctx, t.net, {e2, b}, half_w, t.layer,
                                    max_clear)) {
            elbow = e2;
            has_elbow = true;
        } else {
            return false;
        }
    }
    out.terminal = t.id;
    out.portal_id = portal.id;
    out.portal_pos = portal.pos;
    out.principal_dir = principal_direction(a, b);
    out.layer_strategy = "same-layer";
    out.traces.clear();
    out.vias.clear();
    if (!has_elbow) {
        if (!(a == b)) out.traces.push_back({t.net, t.layer, a, b, width});
    } else {
        out.traces.push_back({t.net, t.layer, a, elbow, width});
        if (!(elbow == b)) out.traces.push_back({t.net, t.layer, elbow, b, width});
    }
    out.length_nm = euclid_len_nm(a, has_elbow ? elbow : b) +
                    (has_elbow ? euclid_len_nm(elbow, b) : 0);
    out.via_count = 0;
    out.width_nm = width;
    out.use_neckdown = is_neck;
    out.via_class = "";
    out.cost = static_cast<double>(out.length_nm + (has_elbow ? 500000 : 0));
    out.signature = candidate_signature(portal.id, out.principal_dir, out.layer_strategy,
                                        "none", fp.channel_count);
    return true;
}

std::vector<TermId> EscapePlanner::eligibility_order(
    const Board& board, const RuleResolver& resolver, const ElectricalContext& ctx,
    const FinePitchFootprint& fp, const std::map<TermId, int>& depth,
    const std::map<TermId, std::vector<int>>& exit_sectors,
    const std::map<TermId, int>& via_sites, const std::map<TermId, double>& density,
    const std::map<TermId, double>& downstream) {
    (void)board;
    (void)resolver;
    (void)ctx;
    std::vector<TermId> order = fp.members;
    std::sort(order.begin(), order.end(), [&](TermId a, TermId b) {
        int da = depth.count(a) ? depth.at(a) : 0;
        int db = depth.count(b) ? depth.at(b) : 0;
        if (da != db) return da > db;  // centre depth ALWAYS outranks
        std::size_t ea = exit_sectors.count(a) ? exit_sectors.at(a).size() : 8;
        std::size_t eb = exit_sectors.count(b) ? exit_sectors.at(b).size() : 8;
        if (ea != eb) return ea < eb;  // fewest legal exits
        int va = via_sites.count(a) ? via_sites.at(a) : 8;
        int vb = via_sites.count(b) ? via_sites.at(b) : 8;
        if (va != vb) return va < vb;  // fewest via opportunities
        double na = density.count(a) ? density.at(a) : 0;
        double nb = density.count(b) ? density.at(b) : 0;
        if (na != nb) return na > nb;  // highest escape density
        double ca = downstream.count(a) ? downstream.at(a) : 0;
        double cb = downstream.count(b) ? downstream.at(b) : 0;
        if (ca != cb) return ca > cb;  // highest contention/difficulty
        return a < b;                  // stable terminal id
    });
    return order;
}

namespace {

// Full-board legality re-check for a candidate produced on a corridor-filtered
// graph. Corridor filtering makes search cheap; this gate keeps it honest: a
// candidate is committable only when its copper keeps required clearance to
// ALL committed copper (not just the corridor subset), stays on-board, and
// respects keepouts. Same integer-exact semantics as the verifier.
bool escape_candidate_legal(const Board& work, const RuleResolver& resolver,
                            const ElectricalContext& ctx, NetId net,
                            const EscapeCandidate& c) {
    Coord max_clear = 0;
    for (const auto& other : work.nets) {
        if (other.id == net) continue;
        std::string cs;
        max_clear = std::max(max_clear, resolver.requiredClearance(net, other.id, 0, ctx, &cs));
    }
    auto seg_ok = [&](const Segment& s, Coord half_w, LayerId layer) -> bool {
        if (!work.bounds().contains(s.bounds().expanded(half_w))) return false;
        for (const auto& ko : work.keepouts) {
            if (ko.layer != kAllLayers && ko.layer != layer) continue;
            __int128 d2 = seg_rect_dist2(s, ko.rect);
            Coord need = max_clear + half_w;
            if (d2 < (__int128)need * need) return false;
        }
        for (const auto& o : work.terminals) {
            if (o.net == net || o.layer != layer) continue;
            std::string cs;
            Coord need = resolver.requiredClearance(net, o.net, layer, ctx, &cs) + half_w;
            if (seg_rect_dist2(s, o.pad_rect()) < (__int128)need * need) return false;
        }
        for (const auto& o : work.traces) {
            if (o.net == net || o.layer != layer) continue;
            std::string cs;
            Coord need = resolver.requiredClearance(net, o.net, layer, ctx, &cs);
            __int128 rhs = (__int128)2 * need + 2 * half_w + o.width_nm;
            if ((__int128)4 * seg_seg_dist2(s, o.segment()) < rhs * rhs) return false;
        }
        for (const auto& v : work.vias) {
            if (v.net == net) continue;
            LayerId lo = std::min(v.top_layer, v.bottom_layer);
            LayerId hi = std::max(v.top_layer, v.bottom_layer);
            if (layer < lo || layer > hi) continue;
            std::string cs;
            Coord need = resolver.requiredClearance(net, v.net, layer, ctx, &cs);
            Rect via_rect = Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm);
            __int128 rhs = (__int128)2 * need + 2 * half_w;
            if ((__int128)4 * seg_rect_dist2(s, via_rect) < rhs * rhs) return false;
        }
        return true;
    };
    for (const auto& t : c.traces) {
        if (!seg_ok(t.segment(), t.width_nm / 2, t.layer)) return false;
    }
    for (const auto& v : c.vias) {
        Rect via_rect = Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm);
        if (!work.bounds().contains(via_rect)) return false;
        LayerId lo = std::min(v.top_layer, v.bottom_layer);
        LayerId hi = std::max(v.top_layer, v.bottom_layer);
        for (const auto& ko : work.keepouts) {
            if (ko.layer != kAllLayers && (ko.layer < lo || ko.layer > hi)) continue;
            if (rect_gap(via_rect, ko.rect) < max_clear) return false;
        }
        for (const auto& o : work.terminals) {
            if (o.net == net || o.layer < lo || o.layer > hi) continue;
            std::string cs;
            if (rect_gap(via_rect, o.pad_rect()) <
                resolver.requiredClearance(net, o.net, o.layer, ctx, &cs))
                return false;
        }
        for (const auto& o : work.traces) {
            if (o.net == net || o.layer < lo || o.layer > hi) continue;
            std::string cs;
            Coord need = resolver.requiredClearance(net, o.net, o.layer, ctx, &cs);
            __int128 rhs = (__int128)2 * need + o.width_nm;
            if ((__int128)4 * seg_rect_dist2(o.segment(), via_rect) < rhs * rhs) return false;
        }
    }
    return true;
}

// Convert an A* path over a sparse graph into committable escape geometry.
bool path_to_candidate(const SparseRoutingGraph& graph, const AStarResult& res, NetId net,
                       LayerId layer, Coord width, const ViaStyle& style, bool have_via,
                       const std::string& via_class, TermId terminal, const EscapePortal& portal,
                       int channel_count, EscapeCandidate& out) {
    if (!res.found) return false;
    out.terminal = terminal;
    out.portal_id = portal.id;
    out.portal_pos = portal.pos;
    out.width_nm = width;
    out.via_class = have_via ? style.name : "";
    (void)via_class;
    bool saw_via = false;
    Point first_via_pt{};
    bool have_first_via = false;
    Coord total = 0;
    for (std::size_t i = 0; i < res.edge_path.size(); ++i) {
        int u = res.node_path[i];
        const SparseEdge& e = graph.edges(u)[res.edge_path[i]];
        const SparseNode& nu = graph.nodes()[u];
        const SparseNode& nv = graph.nodes()[e.to];
        if (e.is_via) {
            saw_via = true;
            if (!have_first_via) {
                first_via_pt = nu.p;
                have_first_via = true;
            }
            Via v;
            v.net = net;
            v.pos = nu.p;
            v.top_layer = std::min(nu.layer, nv.layer);
            v.bottom_layer = std::max(nu.layer, nv.layer);
            v.outer_d_nm = style.outer_nm;
            v.hole_d_nm = style.hole_nm;
            v.via_class = style.name;
            out.vias.push_back(v);
        } else if (e.dir2 >= 0) {
            out.traces.push_back({net, nu.layer, nu.p, e.elbow, width});
            total += euclid_len_nm(nu.p, e.elbow);
            if (!(e.elbow == nv.p)) {
                out.traces.push_back({net, nu.layer, e.elbow, nv.p, width});
                total += euclid_len_nm(e.elbow, nv.p);
            }
        } else {
            if (!(nu.p == nv.p)) {
                out.traces.push_back({net, nu.layer, nu.p, nv.p, width});
                total += euclid_len_nm(nu.p, nv.p);
            }
        }
    }
    out.length_nm = total;
    out.via_count = static_cast<int>(out.vias.size());
    out.cost = static_cast<double>(res.cost_nm);
    (void)layer;
    // Layer strategy from actual geometry.
    const Terminal* t = nullptr;
    (void)t;
    if (!saw_via) {
        out.layer_strategy = "same-layer";
    } else {
        // Dogbone: first via very close to the pad (short stub).
        // Via-first: via at/near the pad. Multilayer: via deeper in the route.
        // We distinguish dogbone vs via-first by stub length.
        Coord stub = have_first_via ? manhattan(graph.nodes()[res.node_path.front()].p, first_via_pt)
                                    : 0;
        // Pitch-scale threshold is applied by the caller via channel context;
        // here use 1mm as the dogbone stub boundary (documented).
        if (stub <= 1000000LL) {
            // Both via-first and dogbone start at the pad; dogbone implies a
            // short surface stub before the via, via-first implies the via is
            // effectively at the pad. Sparse-graph paths start exactly at the
            // pad centre, so stub==0 reads as via-first.
            out.layer_strategy = stub == 0 ? "via-first" : "dogbone";
        } else {
            out.layer_strategy = "multilayer";
        }
        // Refine: single-via short escapes are dogbone by construction.
        if (out.via_count == 1 && total <= 2000000LL && out.layer_strategy == "via-first")
            out.layer_strategy = "dogbone";
    }
    // Principal direction from pad to portal.
    if (!res.node_path.empty()) {
        Point src = graph.nodes()[res.node_path.front()].p;
        out.principal_dir = principal_direction(src, portal.pos);
    }
    out.signature = candidate_signature(portal.id, out.principal_dir, out.layer_strategy,
                                        have_via ? style.name : "none", channel_count);
    return true;
}

}  // namespace

EscapeResult EscapePlanner::plan(const Board& board, const RuleResolver& resolver,
                                 const ElectricalContext& ctx) const {
    EscapeResult result;
    const auto expired = [&]() {
        return std::chrono::steady_clock::now() >= options_.deadline;
    };
    bool aborted = false;
    FinePitchDetector detector;
    std::vector<FinePitchFootprint> fps = detector.detect(board, resolver, ctx);
    CentreDepthAnalyzer depth_analyzer;
    DensityEstimator density_est;

    // Scratch board accumulates committed escape stubs so later (shallower)
    // pads route around earlier (deeper) copper: the centre-out mechanism.
    Board work = board;

    for (const auto& fp : fps) {
        if (expired()) {
            aborted = true;
            break;
        }
        FootprintEscapeResult fr;
        fr.footprint = fp;
        fr.boundary = build_escape_boundary(board, fp, 0);
        fr.density_map = build_escape_density_map(board, fp);

        std::map<TermId, int> depth = depth_analyzer.analyze(board, fp);
        std::map<TermId, std::vector<int>> exits;
        std::map<TermId, int> vias;
        std::map<TermId, double> dens, downstream;
        for (TermId tid : fp.members) {
            if (expired()) {
                aborted = true;
                break;
            }
            const Terminal* t = board.find_terminal(tid);
            if (!t) continue;
            std::string ws;
            Coord w = resolver.requiredTraceWidth(t->net, t->layer, ctx, &ws);
            exits[tid] = legal_exit_sectors(board, resolver, ctx, fp, tid, w);
            vias[tid] = via_site_count(board, resolver, fp, tid);
            // Terminal-order density index lookup.
            for (std::size_t i = 0; i < board.terminals.size(); ++i) {
                if (board.terminals[i].id == tid) {
                    dens[tid] = density_est.local_density(board, board.terminals[i].pos);
                    break;
                }
            }
            // Downstream difficulty proxy: escape span + electrical burden.
            Coord max_clear = 0;
            for (const auto& other : board.nets) {
                if (other.id == t->net) continue;
                std::string cs;
                max_clear = std::max(max_clear, resolver.requiredClearance(t->net, other.id,
                                                                           t->layer, ctx, &cs));
            }
            double span_mm = nm_to_mm(manhattan(t->pos, fp.centroid) +
                                      manhattan(fp.centroid, fr.boundary.rect.center()));
            downstream[tid] =
                span_mm + 40.0 * nm_to_mm(w) + 30.0 * nm_to_mm(max_clear) + 0.5 * dens[tid];
        }
        if (aborted) {
            result.footprints.push_back(std::move(fr));
            break;
        }

        std::vector<TermId> order =
            eligibility_order(board, resolver, ctx, fp, depth, exits, vias, dens, downstream);
        fr.eligibility_order = order;

        // Layer cost multipliers for A*.
        std::vector<double> layer_mult;
        {
            int max_id = 0;
            for (const auto& l : board.layers) max_id = std::max(max_id, l.id);
            layer_mult.assign(max_id + 1, 1.0);
            for (const auto& l : board.layers)
                if (l.id >= 0 && l.id < static_cast<int>(layer_mult.size()))
                    layer_mult[l.id] = l.cost_multiplier > 0 ? l.cost_multiplier : 1.0;
        }

        // Portals nearest-first per pad are tried; deterministic.
        int elig_idx = 0;
        for (TermId tid : order) {
            if (expired()) {
                aborted = true;
                break;
            }
            PadEscapeResult pr;
            pr.terminal = tid;
            pr.centre_depth = depth.count(tid) ? depth.at(tid) : 0;
            pr.density = dens.count(tid) ? dens.at(tid) : 0;
            pr.eligibility_index = elig_idx++;
            pr.exit_sectors = exits.count(tid) ? exits.at(tid) : std::vector<int>{};
            pr.via_sites = vias.count(tid) ? vias.at(tid) : 0;
            pr.downstream_difficulty = downstream.count(tid) ? downstream.at(tid) : 0;

            const Terminal* t = board.find_terminal(tid);
            if (!t) {
                pr.has_viable = false;
                pr.infeasibility = {tid, "unknown_terminal", {"terminal id not on board"}, true};
                fr.pads.push_back(pr);
                continue;
            }
            std::string wsource;
            Coord full_w = resolver.requiredTraceWidth(t->net, t->layer, ctx, &wsource);
            // Issue #11: per-pad A* layer bias toward the impedance solution.
            std::vector<double> pad_layer_mult = layer_mult;
            if (const NetInfo* eni = board.find_net(t->net);
                eni && resolver.impedance().has_target(*eni)) {
                full_w = resolver.maxRequiredWidth(t->net, ctx);
                for (const auto& l : board.layers) {
                    double b = resolver.impedanceLayerMultiplier(t->net, l.id);
                    if (l.id >= 0 && l.id < static_cast<int>(pad_layer_mult.size()))
                        pad_layer_mult[l.id] *= b;
                }
            }
            const NetInfo* net_info = board.find_net(t->net);
            // Neckdown option: legal explicit neck only.
            Coord neck_w = 0;
            bool have_neck = false;
            if (net_info && net_info->allow_neckdown && net_info->neck_width_nm > 0 &&
                net_info->neck_width_nm < full_w) {
                have_neck = true;
                neck_w = net_info->neck_width_nm;
            }
            ViaStyle style;
            LayerSpan full{board.layers.front().id, board.layers.back().id};
            bool have_via = resolver.select_via(t->net, full, style);

            // Candidate portals: nearest K-window, deterministic.
            std::vector<EscapePortal> portals = fr.boundary.portals;
            std::sort(portals.begin(), portals.end(), [&](const EscapePortal& a, const EscapePortal& b) {
                Coord da = manhattan(t->pos, a.pos), db = manhattan(t->pos, b.pos);
                if (da != db) return da < db;
                return a.id < b.id;
            });
            if (static_cast<int>(portals.size()) > options_.max_portals_per_pad)
                portals.resize(options_.max_portals_per_pad);

            // Alternate target layer for multilayer diversity.
            LayerId alt_layer = t->layer;
            for (const auto& l : board.layers) {
                if (l.id != t->layer) {
                    alt_layer = l.id;
                    break;
                }
            }
            bool multilayer_possible = (alt_layer != t->layer);

            std::map<std::string, EscapeCandidate> by_sig;
            AStarConfig cfg;
            cfg.max_expansions = options_.max_expansions;
            const Coord pitch2 = pitch_fallback(fp);
            const int kNeed = options_.k_best;
            auto note = [&](EscapeCandidate c) {
                auto it = by_sig.find(c.signature);
                if (it == by_sig.end() || c.cost < it->second.cost) by_sig[c.signature] = c;
            };
            // Issue #24: layer-aware neckdown legality. The neck stub lives on
            // the terminal layer for direct/same-layer escapes and may span
            // the alternate layer for multilayer escapes, so the gate uses
            // the per-layer copper/internal width on every layer the neck
            // could occupy (never the external/default width alone).
            auto neck_ok_on = [&](LayerId layer) -> bool {
                return have_neck && net_info &&
                       resolver.current().neckdown_legal(*net_info, neck_w, pitch2,
                                                         board, layer, ctx);
            };
            auto neck_ok = [&]() -> bool {
                if (!neck_ok_on(t->layer)) return false;
                if (multilayer_possible && !neck_ok_on(alt_layer)) return false;
                return true;
            };

            // Phase 1: direct escapes (microseconds each, no graph
            // search). Perimeter pads resolve here. Full width is preferred:
            // note() keeps the cheaper candidate on signature ties and the
            // full-width pass runs first.
            for (int phase = 0; phase < 2; ++phase) {
                if (phase == 1 && !neck_ok()) break;
                Coord w = (phase == 0) ? full_w : neck_w;
                bool is_neck = (w != full_w);
                for (const auto& portal : portals) {
                    if (static_cast<int>(by_sig.size()) >= kNeed) break;
                    EscapeCandidate c;
                    if (try_direct_candidate(work, resolver, ctx, fp, *t, portal, w, is_neck,
                                             c))
                        note(c);
                }
                if (static_cast<int>(by_sig.size()) >= kNeed) break;
            }

            // Phase 2: A* fallback for pads direct cannot escape (deep
            // interior, blocked channels). Corridor-filtered sparse graphs:
            // per (pad, portal) only copper near the pad->portal corridor
            // enters the graph (probe-verified identical outcomes at a
            // fraction of the nodes). Finalists are re-checked against the
            // FULL board below; a full-window pass runs only when the
            // corridor pass finds nothing (recall safety net).
            //
            // Fully surrounded pads (zero legal exit sectors) escape through
            // vias when the stackup allows it: that is the intended
            // multilayer escape for interior rings, and it also bounds the
            // search (no doomed same-layer attempts). Single-layer boards
            // still try same-layer A* as the only option.
            bool interior = pr.exit_sectors.empty();
            auto collect_on = [&](const Board& gb, const EscapePortal& portal, Coord w,
                                  bool is_neck, bool via_only) {
                if (expired()) {
                    aborted = true;
                    return;
                }
                // Same-layer attempt (skipped for via-first interior pads).
                if (!via_only) {
                    SparseRoutingGraph g = SparseRoutingGraph::build(
                        gb, resolver, t->net, t->pos, portal.pos, t->layer, t->layer, w,
                        ctx);
                    AStarResult res = astar_route(g, pad_layer_mult, cfg);
                    if (expired()) {
                        aborted = true;
                        return;
                    }
                    if (res.found) {
                        EscapeCandidate c;
                        if (path_to_candidate(g, res, t->net, t->layer, w, style, have_via,
                                              "", tid, portal, fp.channel_count, c)) {
                            c.use_neckdown = is_neck;
                            // Issue #17: minimum-bend arbitrary-angle stubs.
                            // Legality is decided against the FULL board
                            // (work), not the corridor-filtered graph board.
                            if (simplify_escape_traces(work, resolver, ctx, t->net,
                                                       c.traces)) {
                                c.length_nm = 0;
                                for (const auto& s : c.traces)
                                    c.length_nm += euclid_len_nm(s.a, s.b);
                            }
                            note(c);
                        }
                    }
                }
                // Multilayer attempt, only while more diversity is needed.
                if (multilayer_possible && static_cast<int>(by_sig.size()) < kNeed) {
                    SparseRoutingGraph g = SparseRoutingGraph::build(
                        gb, resolver, t->net, t->pos, portal.pos, t->layer, alt_layer, w,
                        ctx);
                    AStarResult res = astar_route(g, pad_layer_mult, cfg);
                    if (expired()) {
                        aborted = true;
                        return;
                    }
                    if (res.found) {
                        EscapeCandidate c;
                        if (path_to_candidate(g, res, t->net, t->layer, w, style, have_via,
                                              "", tid, portal, fp.channel_count, c)) {
                            c.use_neckdown = is_neck;
                            if (simplify_escape_traces(work, resolver, ctx, t->net,
                                                       c.traces)) {
                                c.length_nm = 0;
                                for (const auto& s : c.traces)
                                    c.length_nm += euclid_len_nm(s.a, s.b);
                            }
                            note(c);
                        }
                    }
                }
            };

            if (static_cast<int>(by_sig.size()) < kNeed) {
                bool via_first = interior && multilayer_possible;
                int tried = 0;
                for (const auto& portal : portals) {
                    if (expired()) {
                        aborted = true;
                        break;
                    }
                    if (tried >= options_.max_fallback_portals ||
                        static_cast<int>(by_sig.size()) >= kNeed)
                        break;
                    ++tried;
                    Rect corridor = Rect::from_points(t->pos, portal.pos)
                                        .expanded(2 * pitch2 + full_w + fp.clearance_nm);
                    Board local = local_escape_board(work, corridor);
                    collect_on(local, portal, full_w, false, via_first);
                    if (aborted) break;
                }
                if (by_sig.empty() && neck_ok()) {
                    for (const auto& portal : portals) {
                        if (expired()) {
                            aborted = true;
                            break;
                        }
                        if (!by_sig.empty()) break;
                        Rect corridor = Rect::from_points(t->pos, portal.pos)
                                            .expanded(2 * pitch2 + neck_w + fp.clearance_nm);
                        Board local = local_escape_board(work, corridor);
                        collect_on(local, portal, neck_w, true, via_first);
                        if (aborted) break;
                    }
                }
            }
            if (aborted) break;
            if (by_sig.empty()) {
                // Recall safety net: corridor filtering may hide a wide detour.
                Board wide = local_escape_board(
                    work, fr.boundary.rect.expanded(2 * pitch2 + full_w));
                int tried = 0;
                for (const auto& portal : portals) {
                    if (expired()) {
                        aborted = true;
                        break;
                    }
                    if (!by_sig.empty() || tried >= 3) break;
                    ++tried;
                    collect_on(wide, portal, full_w, false, false);
                    if (aborted) break;
                }
            }
            if (aborted) break;

            std::vector<EscapeCandidate> cands;
            for (auto& [sig, c] : by_sig) {
                // Corridor search is optimistic: keep only candidates that are
                // legal against the full committed board.
                if (escape_candidate_legal(work, resolver, ctx, t->net, c)) cands.push_back(c);
            }
            std::sort(cands.begin(), cands.end(), [](const EscapeCandidate& a, const EscapeCandidate& b) {
                if (a.cost != b.cost) return a.cost < b.cost;
                return a.signature < b.signature;
            });
            if (static_cast<int>(cands.size()) > options_.k_best)
                cands.resize(options_.k_best);
            pr.candidates = cands;
            pr.has_viable = !cands.empty();
            if (!pr.has_viable) {
                InfeasibilityRecord rec;
                rec.terminal = tid;
                rec.recorded = true;
                if (pr.exit_sectors.empty()) {
                    rec.reason = "no_exit_sectors";
                    rec.blockers.push_back("all 8 exit sectors blocked by neighboring pads");
                } else if (!have_via && multilayer_possible && full_w > fp.pitch_nm && fp.pitch_nm > 0) {
                    rec.reason = "via_current";
                    rec.blockers.push_back("no via class meets net current requirement");
                } else if (have_neck == false && full_w > 0 && fp.pitch_nm > 0 &&
                           full_w > fp.pitch_nm) {
                    rec.reason = "width_conflict";
                    rec.blockers.push_back("required width exceeds pad pitch channel");
                    rec.blockers.push_back("neckdown not permitted (allow_neckdown=false)");
                } else {
                    rec.reason = "blocked_channel";
                    // Corridor attribution: keepouts/pads overlapping the
                    // pad-to-boundary corridor.
                    Rect corridor = Rect::from_points(t->pos, fr.boundary.rect.center())
                                        .expanded(full_w);
                    int added = 0;
                    for (const auto& ko : board.keepouts) {
                        if (!ko.rect.intersects(corridor)) continue;
                        rec.blockers.push_back("keepout:" +
                                               (ko.reason.empty() ? "unnamed" : ko.reason));
                        if (++added >= 4) break;
                    }
                    for (const auto& o : board.terminals) {
                        if (added >= 5) break;
                        if (o.id == tid || o.net == t->net) continue;
                        if (!o.pad_rect().intersects(corridor)) continue;
                        rec.blockers.push_back("pad:net=" + std::to_string(o.net));
                        ++added;
                    }
                    if (rec.blockers.empty())
                        rec.blockers.push_back("no A* path to any portal within budget");
                }
                pr.infeasibility = rec;
            }
            // Centre-out commit: deeper pads' copper is visible to shallower
            // pads. A shallower pad is therefore never committed while a
            // deeper pad lacks both a candidate and a record: deeper pads are
            // always processed first and always leave one or the other.
            if (pr.has_viable) {
                const EscapeCandidate& best = pr.candidates.front();
                for (const auto& s : best.traces) work.traces.push_back(s);
                for (const auto& v : best.vias) work.vias.push_back(v);
                fr.commit_order.push_back(tid);
            }
            fr.pads.push_back(pr);
        }
        result.footprints.push_back(std::move(fr));
        if (aborted) break;
    }

    result.timed_out = aborted;
    for (const auto& fp : result.footprints) {
        for (const auto& p : fp.pads) {
            ++result.pads_total;
            if (p.has_viable)
                ++result.pads_with_candidates;
            else
                ++result.pads_infeasible;
        }
    }
    return result;
}

JsonValue EscapeResult::to_json() const {
    JsonValue r = JsonValue::object();
    r["schema"] = "copperline/escape-report/1";
    r["pads_total"] = static_cast<double>(pads_total);
    r["pads_with_candidates"] = static_cast<double>(pads_with_candidates);
    r["pads_infeasible"] = static_cast<double>(pads_infeasible);
    r["timed_out"] = timed_out;
    r["status"] = timed_out ? JsonValue("TIMEOUT")
                             : (pads_infeasible == 0 ? JsonValue("COMPLETE")
                                                     : JsonValue("INCOMPLETE"));
    JsonValue fps = JsonValue::array();
    for (const auto& fp : footprints) {
        JsonValue o = JsonValue::object();
        o["component"] = fp.footprint.component;
        o["reason"] = fp.footprint.reason;
        o["pitch_mm"] = nm_to_mm(fp.footprint.pitch_nm);
        o["pad_count"] = static_cast<double>(fp.footprint.pad_count);
        o["pins_per_mm2"] = fp.footprint.pins_per_mm2;
        o["channel_count"] = static_cast<double>(fp.footprint.channel_count);
        o["required_width_mm"] = nm_to_mm(fp.footprint.trace_width_nm);
        o["required_clearance_mm"] = nm_to_mm(fp.footprint.clearance_nm);
        JsonValue b = JsonValue::object();
        b["x1_mm"] = nm_to_mm(fp.boundary.rect.x1);
        b["y1_mm"] = nm_to_mm(fp.boundary.rect.y1);
        b["x2_mm"] = nm_to_mm(fp.boundary.rect.x2);
        b["y2_mm"] = nm_to_mm(fp.boundary.rect.y2);
        b["portals"] = static_cast<double>(fp.boundary.portals.size());
        o["boundary"] = b;
        o["escape_density_peak_per_mm2"] = fp.density_map.peak;
        JsonValue eo = JsonValue::array();
        for (TermId tid : fp.eligibility_order) eo.as_array().push_back(JsonValue(static_cast<double>(tid)));
        o["eligibility_order"] = eo;
        JsonValue co = JsonValue::array();
        for (TermId tid : fp.commit_order) co.as_array().push_back(JsonValue(static_cast<double>(tid)));
        o["commit_order"] = co;
        JsonValue pads = JsonValue::array();
        for (const auto& p : fp.pads) {
            JsonValue q = JsonValue::object();
            q["terminal"] = static_cast<double>(p.terminal);
            q["centre_depth"] = static_cast<double>(p.centre_depth);
            q["density_per_mm2"] = p.density;
            q["eligibility_index"] = static_cast<double>(p.eligibility_index);
            q["candidate_count"] = static_cast<double>(p.candidates.size());
            JsonValue portals = JsonValue::array();
            JsonValue vias = JsonValue::array();
            for (const auto& c : p.candidates) {
                portals.as_array().push_back(JsonValue(static_cast<double>(c.portal_id)));
                JsonValue vo = JsonValue::object();
                vo["portal"] = static_cast<double>(c.portal_id);
                vo["strategy"] = c.layer_strategy;
                vo["vias"] = static_cast<double>(c.via_count);
                vo["via_class"] = c.via_class;
                vo["width_mm"] = nm_to_mm(c.width_nm);
                vo["neckdown"] = c.use_neckdown;
                vo["signature"] = c.signature;
                vias.as_array().push_back(vo);
            }
            q["candidate_portals"] = portals;
            q["via_decisions"] = vias;
            q["exit_sectors"] = static_cast<double>(p.exit_sectors.size());
            q["via_sites"] = static_cast<double>(p.via_sites);
            if (p.has_viable) {
                q["status"] = JsonValue("ok");
            } else {
                q["status"] = JsonValue("infeasible");
                q["infeasibility_reason"] = p.infeasibility.reason;
                JsonValue bl = JsonValue::array();
                for (const auto& s : p.infeasibility.blockers) bl.as_array().push_back(JsonValue(s));
                q["infeasibility_blockers"] = bl;
            }
            pads.as_array().push_back(q);
        }
        o["pads"] = pads;
        fps.as_array().push_back(o);
    }
    r["footprints"] = fps;
    return r;
}

}  // namespace copperline
