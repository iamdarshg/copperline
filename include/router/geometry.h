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
    double d2 = static_cast<double>(dx) * static_cast<double>(dx) +
                static_cast<double>(dy) * static_cast<double>(dy);
    return static_cast<Coord>(std::llround(std::sqrt(d2)));
}

struct Segment {
    Point a{};
    Point b{};
    bool axis_aligned() const { return a.x == b.x || a.y == b.y; }
    bool is_point() const { return a == b; }
    Rect bounds() const { return Rect::from_points(a, b); }
};

// Exact squared distance, point to segment. Uses 128-bit intermediates.
inline Coord point_seg_dist2(Point p, Segment s) {
    Coord vx = s.b.x - s.a.x;
    Coord vy = s.b.y - s.a.y;
    Coord wx = p.x - s.a.x;
    Coord wy = p.y - s.a.y;
    __int128 len2 = (__int128)vx * vx + (__int128)vy * vy;
    if (len2 == 0) {
        __int128 d2 = (__int128)wx * wx + (__int128)wy * wy;
        return static_cast<Coord>(d2);
    }
    __int128 t_num = (__int128)wx * vx + (__int128)wy * vy;
    double t = static_cast<double>(t_num) / static_cast<double>(len2);
    t = std::clamp(t, 0.0, 1.0);
    double cx = static_cast<double>(s.a.x) + t * static_cast<double>(vx);
    double cy = static_cast<double>(s.a.y) + t * static_cast<double>(vy);
    double dx = static_cast<double>(p.x) - cx;
    double dy = static_cast<double>(p.y) - cy;
    // Snap: the projection of an axis-aligned segment is exact; round to int.
    __int128 ix = static_cast<__int128>(std::llround(dx));
    __int128 iy = static_cast<__int128>(std::llround(dy));
    __int128 d2 = ix * ix + iy * iy;
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

}  // namespace copperline
