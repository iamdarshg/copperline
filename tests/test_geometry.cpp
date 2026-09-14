#include "helpers.h"

#include "router/geometry.h"

using namespace copperline;

CT_TEST(mm_roundtrip) {
    CT_CHECK(mm_to_nm(1.0) == 1000000);
    CT_CHECK(mm_to_nm(0.001) == 1000);
    CT_CHECK_NEAR(nm_to_mm(mm_to_nm(0.15)), 0.15, 1e-9);
}

CT_TEST(rect_ops) {
    Rect r{0, 0, 10, 10};
    CT_CHECK(r.contains(Point{5, 5}));
    CT_CHECK(r.contains(Point{10, 10}));  // closed intervals
    CT_CHECK(!r.contains(Point{11, 5}));
    Rect o{10, 10, 20, 20};
    CT_CHECK(r.intersects(o));  // touching counts
    Rect far{11, 11, 20, 20};
    CT_CHECK(!r.intersects(far));
    CT_CHECK(rect_gap(r, far) == 1);  // corner gap sqrt(2) rounds to 1
    Rect side{11, 0, 20, 10};
    CT_CHECK(rect_gap(r, side) == 1);
    Rect over{5, 5, 15, 15};
    CT_CHECK(rect_gap(r, over) == 0);
}

CT_TEST(seg_distances) {
    Segment s{{0, 0}, {10, 0}};
    CT_CHECK(point_seg_dist2({5, 3}, s) == 9);
    CT_CHECK(point_seg_dist2({5, 0}, s) == 0);
    Segment v{{5, 1}, {5, 9}};
    CT_CHECK(seg_seg_dist2(s, v) == 1);  // gap of 1 in y
    Segment cross{{5, -5}, {5, 5}};
    CT_CHECK(seg_seg_dist2(s, cross) == 0);
    Segment par{{0, 4}, {10, 4}};
    CT_CHECK(seg_seg_dist2(s, par) == 16);
    Rect r{20, 20, 30, 30};
    CT_CHECK(seg_rect_dist2(s, r) == 100 + 400);  // dx=10, dy=20
}

CT_TEST(orientation_and_segment_predicates) {
    CT_CHECK(orientation({0, 0}, {10, 0}, {10, 10}) == 2);
    CT_CHECK(orientation({0, 0}, {10, 0}, {10, -10}) == 1);
    CT_CHECK(orientation({0, 0}, {5, 0}, {10, 0}) == 0);  // collinear
    CT_CHECK(on_segment({0, 0}, {5, 0}, {10, 0}));
    CT_CHECK(!on_segment({0, 0}, {11, 0}, {10, 0}));
    CT_CHECK(seg_intersects_seg({{0, 0}, {10, 10}}, {{0, 10}, {10, 0}}));
    CT_CHECK(!seg_intersects_seg({{0, 0}, {10, 0}}, {{0, 5}, {10, 5}}));
    CT_CHECK(seg_intersects_rect({{0, 5}, {10, 5}}, {2, 2, 8, 8}));
    CT_CHECK(!seg_intersects_rect({{0, 0}, {10, 0}}, {2, 2, 8, 8}));
}

CT_TEST(rect_gap_exact_axes) {
    Rect a{0, 0, 10, 10};
    Rect touch{10, 0, 20, 10};
    CT_CHECK(rect_gap(a, touch) == 0);  // touching: zero gap
    Rect diag{13, 14, 20, 20};          // dx=3, dy=4 -> 5
    CT_CHECK(rect_gap(a, diag) == 5);
    Rect vonly{2, 15, 8, 25};  // x-overlap, dy=5
    CT_CHECK(rect_gap(a, vonly) == 5);
    // Expanded-bounds fast reject agrees with the exact predicate.
    Segment s{{0, 0}, {10, 0}};
    Rect far{100, 100, 110, 110};
    CT_CHECK(!s.bounds().expanded(50).intersects(far));
    CT_CHECK(s.bounds().expanded(200).intersects(far));
}

CT_TEST(integer_legality_comparison) {
    // Exact integer comparison: gap^2 vs required^2, no float involvement.
    Segment s{{0, 0}, {10, 0}};
    Rect r{0, 5, 10, 5};  // zero-height rect 5nm above
    __int128 d2 = seg_rect_dist2(s, r);
    __int128 need5 = (__int128)5 * 5;
    __int128 need6 = (__int128)6 * 6;
    CT_CHECK(d2 >= need5);
    CT_CHECK(!(d2 >= need6));
}

int main() { return copperline::test::run_all_tests(); }
