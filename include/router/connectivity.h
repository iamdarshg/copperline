// Copperline: shared connectivity primitives (D1).
//
// Single home for the copper-touch DSU machinery previously cloned in
// route_tree.cpp and verifier.cpp (~120 dup lines). Header-only so no
// build re-configure is needed.
//
// Design: fully parameterized on element accessors (functors), so both
// call sites keep their own element structs. The verifier stays
// STANDALONE: this header depends only on board.h/geometry.h (integer-nm
// primitives + plane polygon helpers) and never on engine state.
//
// Plane semantics (issue #16, preserved verbatim from both clones):
//   - plane+plane with the same island id unites by declared stitching,
//     unless same_island_unites=false (per-layer proof for via bundles).
//   - different-island planes unite only through visible same-layer
//     geometric bridging (vertex containment or edge crossing).
//   - copper+plane touches when the copper hits the plane polygon on an
//     overlapping layer (trace -> seg test, else rect test).
//   - plain copper touches on overlapping layers: trace/trace -> segment
//     intersection, trace/other -> segment-rect, else rect overlap.
#pragma once

#include <numeric>
#include <vector>

#include "router/board.h"

namespace copperline {
namespace conn {

// Disjoint-set union for same-net connectivity. Deterministic: union by
// attachment order only (no ranks keyed on data), path compression.
class DSU {
  public:
    explicit DSU(int n) : p_(static_cast<std::size_t>(n)) {
        std::iota(p_.begin(), p_.end(), 0);
    }
    // Const find: path compression mutates only the cache (mutable), so
    // cached per-layer partitions (S5) can be shared read-only.
    int find(int x) const {
        int& slot = p_[static_cast<std::size_t>(x)];
        return slot == x ? x : slot = find(slot);
    }
    void unite(int a, int b) { p_[static_cast<std::size_t>(find(a))] = find(b); }

  private:
    mutable std::vector<int> p_;
};

// Layer-span overlap for via spans vs single layers.
inline bool span_overlap(LayerId a_lo, LayerId a_hi, LayerId b_lo, LayerId b_hi) {
    return a_lo <= b_hi && b_lo <= a_hi;
}

// Generic copper-touch predicate. Element access is entirely through the
// functor parameters, so route_tree's Elem and the verifier's Element (or
// any future store) share one exact implementation.
//
// Functor contracts:
//   is_plane(const E&) -> bool, is_trace(const E&) -> bool
//   lo_of/hi_of(const E&) -> LayerId (via span; single layer returns layer)
//   layer_of(const E&) -> LayerId (single-layer elements)
//   rect_of(const E&) -> const Rect& (pad/via disc/plane bbox)
//   seg_of(const E&) -> const Segment& (traces)
//   island_of(const E&) -> int (planes)
//   poly_of(const E&) -> const std::vector<Point>* (planes, board-owned)
template <typename E, typename IsPlaneFn, typename IsTraceFn, typename LoFn,
          typename HiFn, typename LayerFn, typename RectFn, typename SegFn,
          typename IslandFn, typename PolyFn>
bool copper_touch_generic(const E& a, const E& b, IsPlaneFn is_plane,
                          IsTraceFn is_trace, LoFn lo_of, HiFn hi_of,
                          LayerFn layer_of, RectFn rect_of, SegFn seg_of,
                          IslandFn island_of, PolyFn poly_of,
                          bool same_island_unites = true) {
    (void)layer_of;  // spans (lo/hi) carry single-layer identity; kept for
                     // accessor symmetry across element stores.
    const bool a_plane = is_plane(a);
    const bool b_plane = is_plane(b);
    if (a_plane && b_plane) {
        if (same_island_unites && island_of(a) == island_of(b)) return true;
        LayerId a_lo = lo_of(a), a_hi = hi_of(a);
        LayerId b_lo = lo_of(b), b_hi = hi_of(b);
        if (!span_overlap(a_lo, a_hi, b_lo, b_hi)) return false;
        const std::vector<Point>* pa = poly_of(a);
        const std::vector<Point>* pb = poly_of(b);
        if (!pa || !pb) return false;
        for (const auto& p : *pa) {
            if (plane_poly_contains(*pb, p)) return true;
        }
        for (const auto& p : *pb) {
            if (plane_poly_contains(*pa, p)) return true;
        }
        const std::size_t n = pa->size(), m = pb->size();
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = 0; j < m; ++j) {
                if (seg_intersects_seg({(*pa)[i], (*pa)[(i + 1) % n]},
                                       {(*pb)[j], (*pb)[(j + 1) % m]}))
                    return true;
            }
        }
        return false;
    }
    if (a_plane || b_plane) {
        const E& pl = a_plane ? a : b;
        const E& other = a_plane ? b : a;
        if (!span_overlap(lo_of(pl), hi_of(pl), lo_of(other), hi_of(other)))
            return false;
        const std::vector<Point>* poly = poly_of(pl);
        if (!poly) return false;
        if (is_trace(other)) return plane_seg_hits_poly(seg_of(other), *poly);
        return plane_rect_hits_poly(rect_of(other), *poly);
    }
    if (!span_overlap(lo_of(a), hi_of(a), lo_of(b), hi_of(b))) return false;
    const bool a_trace = is_trace(a);
    const bool b_trace = is_trace(b);
    if (a_trace && b_trace) return seg_intersects_seg(seg_of(a), seg_of(b));
    if (a_trace) return seg_intersects_rect(seg_of(a), rect_of(b));
    if (b_trace) return seg_intersects_rect(seg_of(b), rect_of(a));
    return rect_of(a).intersects(rect_of(b));
}

// Plain-data convenience form (no functors): pass resolved spans/flags.
inline bool copper_touch_resolved(bool a_plane, bool b_plane, bool a_trace,
                                  bool b_trace, LayerId a_lo, LayerId a_hi,
                                  LayerId b_lo, LayerId b_hi,
                                  const Rect& a_rect, const Rect& b_rect,
                                  const Segment& a_seg, const Segment& b_seg,
                                  int a_island, int b_island,
                                  const std::vector<Point>* a_poly,
                                  const std::vector<Point>* b_poly,
                                  bool same_island_unites = true) {
    struct View {
        bool plane, trace;
        LayerId lo, hi, layer;
        const Rect* rect;
        const Segment* seg;
        int island;
        const std::vector<Point>* poly;
    };
    View va{a_plane, a_trace, a_lo, a_hi, a_lo, &a_rect, &a_seg, a_island, a_poly};
    View vb{b_plane, b_trace, b_lo, b_hi, b_lo, &b_rect, &b_seg, b_island, b_poly};
    auto is_plane = [](const View& e) { return e.plane; };
    auto is_trace = [](const View& e) { return e.trace; };
    auto lo_of = [](const View& e) { return e.lo; };
    auto hi_of = [](const View& e) { return e.hi; };
    auto layer_of = [](const View& e) { return e.layer; };
    auto rect_of = [](const View& e) -> const Rect& { return *e.rect; };
    auto seg_of = [](const View& e) -> const Segment& { return *e.seg; };
    auto island_of = [](const View& e) { return e.island; };
    auto poly_of = [](const View& e) { return e.poly; };
    return copper_touch_generic(va, vb, is_plane, is_trace, lo_of, hi_of,
                                layer_of, rect_of, seg_of, island_of, poly_of,
                                same_island_unites);
}

}  // namespace conn
}  // namespace copperline
