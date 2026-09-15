// Copperline: integer-nanometre geometry primitives.
//
// Every position and extent in the router core is an integer number of
// nanometres (Coord = int64_t). Floating point appears only at the JSON
// boundary (millimetre I/O) and inside reporters. All legality predicates
// use exact integer arithmetic.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace copperline {

using Coord = std::int64_t;

inline constexpr Coord kNanometresPerMillimetre = 1000000LL;
inline constexpr Coord kNanometresPerMicrometre = 1000LL;

inline Coord mm_to_nm(double mm) { return static_cast<Coord>(std::llround(mm * 1.0e6)); }
inline double nm_to_mm(Coord nm) { return static_cast<double>(nm) / 1.0e6; }

using NetId = int;
using LayerId = int;
using TermId = int;
using RegionId = int;

inline constexpr RegionId kAnyRegion = -1;
inline constexpr LayerId kAllLayers = -1;

struct Point {
    Coord x = 0;
    Coord y = 0;
    bool operator==(const Point& o) const = default;
    bool operator<(const Point& o) const { return x < o.x || (x == o.x && y < o.y); }
    Point operator+(const Point& o) const { return {x + o.x, y + o.y}; }
    Point operator-(const Point& o) const { return {x - o.x, y - o.y}; }
};

inline Coord manhattan(const Point& a, const Point& b) {
    Coord dx = a.x >= b.x ? a.x - b.x : b.x - a.x;
    Coord dy = a.y >= b.y ? a.y - b.y : b.y - a.y;
    return dx + dy;
}

struct Rect {
    Coord x1 = 0, y1 = 0, x2 = 0, y2 = 0;  // invariant: x1 <= x2, y1 <= y2

    static Rect from_points(Point a, Point b) {
        return {std::min(a.x, b.x), std::min(a.y, b.y), std::max(a.x, b.x), std::max(a.y, b.y)};
    }
    static Rect from_center_size(Point c, Coord w, Coord h) {
        return {c.x - w / 2, c.y - h / 2, c.x - w / 2 + w, c.y - h / 2 + h};
    }

    Coord width() const { return x2 - x1; }
    Coord height() const { return y2 - y1; }
    Point center() const { return {(x1 + x2) / 2, (y1 + y2) / 2}; }

    bool contains(Point p) const { return p.x >= x1 && p.x <= x2 && p.y >= y1 && p.y <= y2; }
    bool contains(const Rect& o) const {
        return o.x1 >= x1 && o.x2 <= x2 && o.y1 >= y1 && o.y2 <= y2;
    }
    // Closed-interval intersection: touching counts as intersecting.
    bool intersects(const Rect& o) const {
        return x1 <= o.x2 && o.x1 <= x2 && y1 <= o.y2 && o.y1 <= y2;
    }
    Rect expanded(Coord d) const { return {x1 - d, y1 - d, x2 + d, y2 + d}; }
    Rect clamped(const Rect& bounds) const {
        return {std::max(x1, bounds.x1), std::max(y1, bounds.y1), std::min(x2, bounds.x2),
                std::min(y2, bounds.y2)};
    }
};

// Integer square root rounded to nearest. Exact replacement for
// llround(sqrt(...)): the long-double value is only a seed, the __int128
// correction loops decide the exact result. (No exact .5 case exists for
// integer inputs: k^2+k+1/4 is never integral, so half-up == half-away.)
inline Coord isqrt_nearest(__int128 d2) {
    if (d2 <= 0) return 0;
    __int128 r = static_cast<__int128>(std::sqrt(static_cast<long double>(d2)));
    if (r < 0) r = 0;
    while ((r + 1) * (r + 1) <= d2) ++r;
    while (r * r > d2) --r;
    // sqrt(d2) >= r + 0.5  <=>  d2 >= r^2 + r + 1/4  <=>  4*(d2-r^2) > 4*r
    if (4 * (d2 - r * r) > 4 * r) ++r;
    return static_cast<Coord>(r);
}

// Exact edge-to-edge gap between rects (0 when touching or overlapping).
inline Coord rect_gap(const Rect& a, const Rect& b) {
    Coord dx = 0, dy = 0;
    if (a.x2 < b.x1) dx = b.x1 - a.x2;
    else if (b.x2 < a.x1) dx = a.x1 - b.x2;
    if (a.y2 < b.y1) dy = b.y1 - a.y2;
    else if (b.y2 < a.y1) dy = a.y1 - b.y2;
    if (dx == 0 && dy == 0) return 0;
    if (dx == 0) return dy;
    if (dy == 0) return dx;
    // S9: integer-only diagonal (was double sqrt + llround per call).
    return isqrt_nearest((__int128)dx * dx + (__int128)dy * dy);
}

struct Segment {
    Point a{};
    Point b{};
    bool axis_aligned() const { return a.x == b.x || a.y == b.y; }
    bool is_point() const { return a == b; }
    Rect bounds() const { return Rect::from_points(a, b); }
};

// Integer division rounding half away from zero (matches std::llround
// semantics for the quotient). Denominator must be positive.
inline __int128 div_round_half_away(__int128 num, __int128 den) {
    __int128 q = num / den;  // truncated toward zero
    __int128 r = num % den;  // sign of num (0 when exact)
    __int128 a = r >= 0 ? r : -r;
    if (2 * a >= den) q += (num >= 0 ? 1 : -1);
    return q;
}

// Exact squared distance, point to segment. Uses 128-bit intermediates.
// S9: integer-only projection (was double t + llround per call). The snapped
// residual round(p - (a + t_num*v/len2)) is evaluated exactly in rationals,
// so results are bit-identical to the old code unless the true residual lay
// within ~1ulp of a .5 rounding boundary (0 mismatches in 6M fuzz trials
// spanning board-scale, local, diagonal and degenerate inputs).
inline Coord point_seg_dist2(Point p, Segment s) {
    __int128 vx = (__int128)s.b.x - s.a.x;
    __int128 vy = (__int128)s.b.y - s.a.y;
    __int128 wx = (__int128)p.x - s.a.x;
    __int128 wy = (__int128)p.y - s.a.y;
    __int128 len2 = vx * vx + vy * vy;
    if (len2 == 0) {
        __int128 d2 = wx * wx + wy * wy;
        return static_cast<Coord>(d2);
    }
    __int128 t_num = wx * vx + wy * vy;
    __int128 qx, qy;
    if (t_num <= 0) {
        qx = s.a.x;
        qy = s.a.y;
    } else if (t_num >= len2) {
        qx = s.b.x;
        qy = s.b.y;
    } else {
        // Round the projection residual exactly in rationals:
        //   ix = round(px - (ax + t_num*vx/len2))  (half away from zero)
        // i.e. the old llround(px - cx) without any floating point.
        __int128 ix = div_round_half_away(wx * len2 - t_num * vx, len2);
        __int128 iy = div_round_half_away(wy * len2 - t_num * vy, len2);
        __int128 d2 = ix * ix + iy * iy;
        return static_cast<Coord>(d2);
    }
    __int128 dx = (__int128)p.x - qx;
    __int128 dy = (__int128)p.y - qy;
    __int128 d2 = dx * dx + dy * dy;
    return static_cast<Coord>(d2);
}

inline int orientation(Point p, Point q, Point r) {
    __int128 v = (__int128)(q.y - p.y) * (r.x - q.x) - (__int128)(q.x - p.x) * (r.y - q.y);
    if (v == 0) return 0;
    return v > 0 ? 1 : 2;
}

inline bool on_segment(Point p, Point q, Point r) {
    return q.x >= std::min(p.x, r.x) && q.x <= std::max(p.x, r.x) && q.y >= std::min(p.y, r.y) &&
           q.y <= std::max(p.y, r.y);
}

inline bool seg_intersects_seg(Segment s1, Segment s2) {
    int o1 = orientation(s1.a, s1.b, s2.a);
    int o2 = orientation(s1.a, s1.b, s2.b);
    int o3 = orientation(s2.a, s2.b, s1.a);
    int o4 = orientation(s2.a, s2.b, s1.b);
    if (o1 != o2 && o3 != o4) return true;
    if (o1 == 0 && on_segment(s1.a, s2.a, s1.b)) return true;
    if (o2 == 0 && on_segment(s1.a, s2.b, s1.b)) return true;
    if (o3 == 0 && on_segment(s2.a, s1.a, s2.b)) return true;
    if (o4 == 0 && on_segment(s2.a, s1.b, s2.b)) return true;
    return false;
}

inline bool seg_intersects_rect(Segment s, const Rect& r) {
    if (r.contains(s.a) || r.contains(s.b)) return true;
    Segment edges[4] = {{{r.x1, r.y1}, {r.x2, r.y1}},
                        {{r.x2, r.y1}, {r.x2, r.y2}},
                        {{r.x2, r.y2}, {r.x1, r.y2}},
                        {{r.x1, r.y2}, {r.x1, r.y1}}};
    for (const auto& e : edges)
        if (seg_intersects_seg(s, e)) return true;
    return false;
}

// Exact squared distance, segment to segment.
inline Coord seg_seg_dist2(Segment s1, Segment s2) {
    if (seg_intersects_seg(s1, s2)) return 0;
    Coord d = point_seg_dist2(s1.a, s2);
    d = std::min(d, point_seg_dist2(s1.b, s2));
    d = std::min(d, point_seg_dist2(s2.a, s1));
    d = std::min(d, point_seg_dist2(s2.b, s1));
    return d;
}

// Exact squared edge-to-edge gap between rects (0 when touching/overlapping).
// Shared by the arbiter, the via-bundle planner and conflict checks (D2:
// single definition; the per-file clones are deleted).
inline __int128 rect_gap2(const Rect& a, const Rect& b) {
    Coord dx = 0, dy = 0;
    if (a.x2 < b.x1) dx = b.x1 - a.x2;
    else if (b.x2 < a.x1) dx = a.x1 - b.x2;
    if (a.y2 < b.y1) dy = b.y1 - a.y2;
    else if (b.y2 < a.y1) dy = a.y1 - b.y2;
    return (__int128)dx * dx + (__int128)dy * dy;
}

// Fast conservative test: exact gap >= need? Rectangular pre-check first.
inline bool gap_ok_rect(const Rect& a, const Rect& b, Coord need) {
    if (!a.expanded(need).intersects(b)) return true;
    return rect_gap2(a, b) >= (__int128)need * need;
}

// Exact squared distance, segment to rect (0 when touching).
inline Coord seg_rect_dist2(Segment s, const Rect& r) {
    if (seg_intersects_rect(s, r)) return 0;
    Coord d = point_seg_dist2(s.a, {{r.x1, r.y1}, {r.x1, r.y1}});
    // Distance to each rect edge.
    Segment edges[4] = {{{r.x1, r.y1}, {r.x2, r.y1}},
                        {{r.x2, r.y1}, {r.x2, r.y2}},
                        {{r.x2, r.y2}, {r.x1, r.y2}},
                        {{r.x1, r.y2}, {r.x1, r.y1}}};
    for (const auto& e : edges) d = std::min(d, seg_seg_dist2(s, e));
    return d;
}

inline bool seg_ok_rect(const Segment& s, const Rect& raw, Coord need) {
    if (!s.bounds().expanded(need).intersects(raw)) return true;
    __int128 d2 = seg_rect_dist2(s, raw);
    return d2 >= (__int128)need * need;
}

}  // namespace copperline
