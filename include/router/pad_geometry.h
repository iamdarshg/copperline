// Copperline: shape-aware pad geometry.
//
// Pads on real boards are circles, capsules (ovals) and rounded rectangles,
// not squares. Treating a 1.3mm round pad as a square makes a diagonal
// neighbour at 1.27mm pitch look like a 0mm clearance violation, so DRC
// parity with real designs needs the true shape.
//
// Representation: a pad is a convex *core* Minkowski-summed with a disc:
//   rect      -> core = rectangle (w,h),              r = 0
//   circle    -> core = point,                        r = w/2
//   oval      -> core = segment along the long axis,  r = short/2
//   roundrect -> core = rectangle shrunk by r,        r = corner radius
// Distance between two pads is distance(coreA, coreB) - rA - rB, exact for
// this representation. Cores are point / segment / rectangle; each is turned
// into its vertices and measured with exact convex distance routines.
// Rotations use long double; the error is far below a nanometre at pad sizes.
#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "router/board.h"
#include "router/geometry.h"

namespace copperline {

namespace padgeom_detail {

inline constexpr double kPi = 3.14159265358979323846;
inline double deg_to_rad(double d) { return d * kPi / 180.0; }

inline double point_seg_dist(double px, double py, double ax, double ay,
                             double bx, double by) {
    const double vx = bx - ax, vy = by - ay;
    const double wx = px - ax, wy = py - ay;
    const double vv = vx * vx + vy * vy;
    if (vv <= 0.0) return std::hypot(wx, wy);
    double t = (wx * vx + wy * vy) / vv;
    t = std::max(0.0, std::min(1.0, t));
    return std::hypot(px - (ax + t * vx), py - (ay + t * vy));
}

inline double seg_seg_dist(double ax, double ay, double bx, double by,
                           double cx, double cy, double dx, double dy) {
    const double d1 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    const double d2 = (bx - ax) * (dy - ay) - (by - ay) * (dx - ax);
    const double d3 = (dx - cx) * (ay - cy) - (dy - cy) * (ax - cx);
    const double d4 = (dx - cx) * (by - cy) - (dy - cy) * (bx - cx);
    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
        ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
        return 0.0;
    return std::min(std::min(point_seg_dist(ax, ay, cx, cy, dx, dy),
                             point_seg_dist(bx, by, cx, cy, dx, dy)),
                    std::min(point_seg_dist(cx, cy, ax, ay, bx, by),
                             point_seg_dist(dx, dy, ax, ay, bx, by)));
}

// Distance from a point/segment to a convex polygon.
inline double seg_poly_dist(const std::vector<std::pair<double, double>>& poly,
                            double ax, double ay, double bx, double by) {
    bool inside_a = poly.size() >= 3, inside_b = poly.size() >= 3;
    if (inside_a || inside_b) {
        auto inside = [](const std::vector<std::pair<double, double>>& p,
                         double x, double y) {
            bool pos = false, neg = false;
            for (std::size_t i = 0; i < p.size(); ++i) {
                const auto& u = p[i];
                const auto& v = p[(i + 1) % p.size()];
                const double cross = (v.first - u.first) * (y - u.second) -
                                     (v.second - u.second) * (x - u.first);
                if (cross > 0) pos = true;
                if (cross < 0) neg = true;
            }
            return !(pos && neg);
        };
        if (inside_a && inside(poly, ax, ay)) return 0.0;
        if (inside_b && inside(poly, bx, by)) return 0.0;
    }
    double best = 1e18;
    for (std::size_t i = 0; i < poly.size(); ++i) {
        const auto& p1 = poly[i];
        const auto& p2 = poly[(i + 1) % poly.size()];
        best = std::min(best, seg_seg_dist(ax, ay, bx, by, p1.first, p1.second,
                                           p2.first, p2.second));
    }
    return best;
}

inline double poly_poly_dist(const std::vector<std::pair<double, double>>& a,
                             const std::vector<std::pair<double, double>>& b) {
    double best = 1e18;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const auto& a1 = a[i];
        const auto& a2 = a[(i + 1) % a.size()];
        best = std::min(best, seg_poly_dist(b, a1.first, a1.second, a2.first,
                                            a2.second));
        if (best == 0.0) return 0.0;
    }
    return best;
}

}  // namespace padgeom_detail

// Core of a pad: a point, a segment (along the long axis in the pad frame) or
// a rectangle, plus a disc radius added on top.
struct PadCore {
    enum class Kind { kPoint, kSegment, kRect } kind = Kind::kRect;
    double hx = 0.0;  // kRect: half width; kSegment: half length
    double hy = 0.0;  // kRect: half height
    bool vertical = false;  // kSegment: long axis along pad-frame Y
    double r = 0.0;
};

inline PadCore pad_core(const Terminal& t) {
    PadCore c;
    const double w = static_cast<double>(t.pad_w_nm);
    const double h = static_cast<double>(t.pad_h_nm);
    switch (t.shape) {
        case PadShape::kCircle:
            c.kind = PadCore::Kind::kPoint;
            c.r = std::max(w, h) / 2.0;
            break;
        case PadShape::kOval: {
            const double lo = std::min(w, h), hi = std::max(w, h);
            c.kind = PadCore::Kind::kSegment;
            c.vertical = h > w;         // long axis follows the larger size
            c.hx = (hi - lo) / 2.0;     // half length along the long axis
            c.hy = 0.0;
            c.r = lo / 2.0;
            break;
        }
        case PadShape::kRoundRect: {
            const double rc = static_cast<double>(t.corner_r_nm);
            c.kind = PadCore::Kind::kRect;
            c.hx = std::max(0.0, w / 2.0 - rc);
            c.hy = std::max(0.0, h / 2.0 - rc);
            c.r = rc;
            break;
        }
        case PadShape::kRect:
        default:
            c.kind = PadCore::Kind::kRect;
            c.hx = w / 2.0;
            c.hy = h / 2.0;
            c.r = 0.0;
            break;
    }
    return c;
}

// Edge-to-edge gap between two pads (0 when they touch or overlap).
inline Coord pad_gap_nm(const Terminal& a, const Terminal& b) {
    using namespace padgeom_detail;
    const PadCore ca = pad_core(a);
    const PadCore cb = pad_core(b);
    const double ang_a = deg_to_rad(a.rotation_deg);
    const double ang_b = deg_to_rad(b.rotation_deg);

    // Express B in A's frame (A's core then shares A's axes).
    const double dx = static_cast<double>(b.pos.x - a.pos.x);
    const double dy = static_cast<double>(b.pos.y - a.pos.y);
    double bx = dx * std::cos(-ang_a) - dy * std::sin(-ang_a);
    double by = dx * std::sin(-ang_a) + dy * std::cos(-ang_a);
    double rel = ang_b - ang_a;
    if (ca.vertical) {  // normalise A's segment onto the X axis
        const double t = bx;
        bx = by;
        by = -t;
        rel -= kPi / 2.0;
    }

    auto core_points = [](const PadCore& c, double cx, double cy, double ang,
                          bool a_vertical) {
        std::vector<std::pair<double, double>> pts;
        switch (c.kind) {
            case PadCore::Kind::kPoint:
                pts.emplace_back(cx, cy);
                break;
            case PadCore::Kind::kSegment: {
                double dir = ang + (c.vertical ? kPi / 2.0 : 0.0);
                if (a_vertical) dir -= kPi / 2.0;
                for (double s : {-1.0, 1.0})
                    pts.emplace_back(cx + s * c.hx * std::cos(dir),
                                     cy + s * c.hx * std::sin(dir));
                break;
            }
            case PadCore::Kind::kRect: {
                const double cs[4][2] = {{c.hx, c.hy}, {-c.hx, c.hy},
                                         {-c.hx, -c.hy}, {c.hx, -c.hy}};
                for (const auto& p : cs)
                    pts.emplace_back(cx + p[0] * std::cos(ang) - p[1] * std::sin(ang),
                                     cy + p[0] * std::sin(ang) + p[1] * std::cos(ang));
                break;
            }
        }
        return pts;
    };

    // A is centred at the origin in its own frame; both cores were normalised
    // so A's segment (if any) runs along X. A's own `vertical` flag has been
    // folded into the scene rotation above, so it must not be applied twice.
    PadCore ca_pts = ca;
    ca_pts.vertical = false;
    const auto pa = core_points(ca_pts, 0.0, 0.0, 0.0, false);
    const auto pb = core_points(cb, bx, by, rel, ca.vertical);

    double dist = 1e18;
    if (pa.size() == 1 && pb.size() == 1) {
        dist = std::hypot(bx, by);
    } else if (pa.size() == 1) {
        dist = pb.size() == 2
                   ? point_seg_dist(0, 0, pb[0].first, pb[0].second, pb[1].first,
                                    pb[1].second)
                   : seg_poly_dist(pb, 0, 0, 0, 0);
    } else if (pb.size() == 1) {
        dist = pa.size() == 2
                   ? point_seg_dist(bx, by, pa[0].first, pa[0].second, pa[1].first,
                                    pa[1].second)
                   : seg_poly_dist(pa, bx, by, bx, by);
    } else if (pa.size() == 2 && pb.size() == 2) {
        dist = seg_seg_dist(pa[0].first, pa[0].second, pa[1].first, pa[1].second,
                            pb[0].first, pb[0].second, pb[1].first, pb[1].second);
    } else if (pa.size() == 2) {
        dist = seg_poly_dist(pb, pa[0].first, pa[0].second, pa[1].first,
                             pa[1].second);
    } else if (pb.size() == 2) {
        dist = seg_poly_dist(pa, pb[0].first, pb[0].second, pb[1].first,
                             pb[1].second);
    } else {
        dist = poly_poly_dist(pa, pb);
    }
    const double gap = dist - ca.r - cb.r;
    if (gap <= 0.0) return 0;
    return static_cast<Coord>(std::llround(gap));
}

}  // namespace copperline
