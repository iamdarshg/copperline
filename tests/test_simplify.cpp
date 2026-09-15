// Issue #17: arbitrary-angle minimum-bend straight trace geometry.
#include <cmath>

#include "helpers.h"

#include "router/engine.h"
#include "router/parallel.h"
#include "router/simplify.h"
#include "router/verifier.h"

using namespace copperline;
using namespace copperline::test;

namespace {

Board diagonal_board() {
    Board b = base_2layer();
    NetInfo s = make_net(0, "SIG");
    b.nets.push_back(s);
    // Deliberately off-axis AND off-45: slope dy/dx = 3/16.
    add_terminal(b, 0, 2.0, 9.0);
    add_terminal(b, 0, 18.0, 12.0);
    return b;
}

std::vector<TraceSeg> net_traces(const Board& b, NetId net) {
    std::vector<TraceSeg> out;
    for (const auto& t : b.traces)
        if (t.net == net) out.push_back(t);
    return out;
}

bool axis_aligned(const TraceSeg& t) { return t.a.x == t.b.x || t.a.y == t.b.y; }

int run_bends(const std::vector<TraceSeg>& segs) {
    int bends = 0;
    for (std::size_t i = 0; i + 1 < segs.size(); ++i) {
        if (segs[i].layer != segs[i + 1].layer) {
            ++bends;
            continue;
        }
        Coord ux = segs[i].b.x - segs[i].a.x, uy = segs[i].b.y - segs[i].a.y;
        Coord vx = segs[i + 1].b.x - segs[i + 1].a.x, vy = segs[i + 1].b.y - segs[i + 1].a.y;
        if ((__int128)ux * vy != (__int128)uy * vx) ++bends;
    }
    return bends;
}

void expect_verifier_clean(const Board& b) {
    BoardVerifier v;
    ElectricalContext ctx;
    RuleResolver r = RuleResolver::defaults_for(b);
    VerifyResult vr = v.verify(b, r, ctx);
    CT_CHECK(vr.ok);
}

}  // namespace

CT_TEST(clear_diagonal_is_one_natural_segment) {
    Board b = diagonal_board();
    Point src = b.terminals[0].pos, dst = b.terminals[1].pos;
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = 1;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    auto segs = net_traces(engine.committed(), 0);
    CT_CHECK(segs.size() == 1);
    CT_CHECK(segs[0].a == src);
    CT_CHECK(segs[0].b == dst);
    // Natural angle: neither axis-aligned nor snapped to 45 degrees.
    CT_CHECK(!axis_aligned(segs[0]));
    Coord dx = segs[0].b.x - segs[0].a.x, dy = segs[0].b.y - segs[0].a.y;
    CT_CHECK(dx == mm_to_nm(16.0));
    CT_CHECK(dy == mm_to_nm(3.0));
    CT_CHECK(dx != dy);  // not a 45-degree snap
    expect_verifier_clean(engine.committed());
}

CT_TEST(diagonal_not_45_straight_45) {
    // Guidance elbows or 45octilinear staging must never leak into copper:
    // one segment means no 45 lead-in/out is even representable.
    Board b = diagonal_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    auto segs = net_traces(engine.committed(), 0);
    CT_CHECK(segs.size() == 1);
    CT_CHECK(run_bends(segs) == 0);
    // Integer-nm endpoints preserved exactly (no float snapping).
    CT_CHECK(segs[0].a.x == mm_to_nm(2.0));
    CT_CHECK(segs[0].b.y == mm_to_nm(12.0));
}

CT_TEST(obstacle_uses_minimum_visibility_bends) {
    // Short stub wall: the direct line crosses it, but a one-bend tangent
    // cut around the expanded corner is the practical minimum. The
    // simplifier must find it (2 diagonal segments, 1 bend), never a
    // Manhattan staircase or 45-straight-45 staging.
    Board b = base_2layer();
    NetInfo s = make_net(0, "SIG");
    b.nets.push_back(s);
    NetInfo g = make_net(1, "GND");
    b.nets.push_back(g);
    add_terminal(b, 0, 2.0, 10.0);
    add_terminal(b, 0, 18.0, 2.0);
    add_terminal(b, 1, 2.0, 18.0);
    add_terminal(b, 1, 18.0, 18.0);
    Keepout wall;
    wall.rect = {mm_to_nm(9.0), mm_to_nm(0.0), mm_to_nm(11.0), mm_to_nm(8.0)};
    wall.layer = kAllLayers;
    wall.reason = "stub";
    b.keepouts.push_back(wall);
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    auto segs = net_traces(engine.committed(), 0);
    // Minimum practical: over the top via two visibility corners at most.
    CT_CHECK(segs.size() <= 3);
    CT_CHECK(run_bends(segs) <= 2);
    // Shortcuts are diagonal: not pure Manhattan staging.
    bool saw_diagonal = false;
    for (const auto& t : segs)
        if (!axis_aligned(t)) saw_diagonal = true;
    CT_CHECK(saw_diagonal);
    // No three consecutive collinear waypoints survive (merge pass).
    for (std::size_t i = 0; i + 1 < segs.size(); ++i) {
        if (segs[i].layer != segs[i + 1].layer) continue;
        Coord ux = segs[i].b.x - segs[i].a.x, uy = segs[i].b.y - segs[i].a.y;
        Coord vx = segs[i + 1].b.x - segs[i + 1].a.x, vy = segs[i + 1].b.y - segs[i + 1].a.y;
        bool collinear = ((__int128)ux * vy == (__int128)uy * vx) &&
                         ((__int128)ux * vx + (__int128)uy * vy > 0);
        CT_CHECK(!collinear);
    }
    expect_verifier_clean(engine.committed());
}

CT_TEST(staircase_guidance_collapses_to_straight) {
    // Coarse Manhattan waypoint staging must not survive as a staircase.
    Board b = base_2layer();
    NetInfo s = make_net(0, "SIG");
    b.nets.push_back(s);
    TermId ta = add_terminal(b, 0, 1.0, 1.0);
    TermId tb = add_terminal(b, 0, 9.0, 9.0);
    (void)ta;
    (void)tb;
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    std::vector<Point> stairs{{mm_to_nm(1.0), mm_to_nm(1.0)},
                              {mm_to_nm(3.0), mm_to_nm(1.0)},
                              {mm_to_nm(3.0), mm_to_nm(3.0)},
                              {mm_to_nm(5.0), mm_to_nm(3.0)},
                              {mm_to_nm(5.0), mm_to_nm(5.0)},
                              {mm_to_nm(7.0), mm_to_nm(5.0)},
                              {mm_to_nm(7.0), mm_to_nm(7.0)},
                              {mm_to_nm(9.0), mm_to_nm(7.0)},
                              {mm_to_nm(9.0), mm_to_nm(9.0)}};
    std::string ws;
    Coord w = r.requiredTraceWidth(0, 0, ctx, &ws);
    std::vector<Point> out = simplify_polyline(b, r, 0, 0, w, stairs, ctx);
    CT_CHECK(out.size() == 2);
    CT_CHECK(out.front() == stairs.front());
    CT_CHECK(out.back() == stairs.back());
}

CT_TEST(duplicates_collinear_and_tiny_jogs_removed) {
    Board b = base_2layer();
    NetInfo s = make_net(0, "SIG");
    b.nets.push_back(s);
    add_terminal(b, 0, 2.0, 5.0);
    add_terminal(b, 0, 12.0, 5.0);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    std::string ws;
    Coord w = r.requiredTraceWidth(0, 0, ctx, &ws);  // 0.2mm -> tiny = 50um
    // Duplicates + exact collinear middles (interior geometry: the
    // centerline plus half width must stay on the board).
    {
        std::vector<Point> pts{{mm_to_nm(2.0), mm_to_nm(5.0)},
                                {mm_to_nm(2.0), mm_to_nm(5.0)},
                                {mm_to_nm(7.0), mm_to_nm(5.0)},
                                {mm_to_nm(7.0), mm_to_nm(5.0)},
                                {mm_to_nm(12.0), mm_to_nm(5.0)}};
        std::vector<Point> out = simplify_polyline(b, r, 0, 0, w, pts, ctx);
        CT_CHECK(out.size() == 2);
    }
    // Micro-staircase (10um steps, below the 50um tiny threshold) collapses.
    {
        std::vector<Point> pts{{mm_to_nm(2.0), mm_to_nm(5.0)},
                                {mm_to_nm(2.01), mm_to_nm(5.0)},
                                {mm_to_nm(2.01), mm_to_nm(5.01)},
                                {mm_to_nm(12.0), mm_to_nm(5.01)},
                                {mm_to_nm(12.0), mm_to_nm(5.0)}};
        std::vector<Point> out = simplify_polyline(b, r, 0, 0, w, pts, ctx);
        CT_CHECK(out.size() == 2);
        CT_CHECK(out.front() == pts.front());
        CT_CHECK(out.back() == pts.back());
    }
    // Determinism: repeated simplification is a fixpoint.
    {
        std::vector<Point> pts{{mm_to_nm(2.0), mm_to_nm(2.0)},
                                {mm_to_nm(6.0), mm_to_nm(2.0)},
                                {mm_to_nm(6.0), mm_to_nm(6.0)},
                                {mm_to_nm(10.0), mm_to_nm(6.0)},
                                {mm_to_nm(10.0), mm_to_nm(10.0)}};
        std::vector<Point> once = simplify_polyline(b, r, 0, 0, w, pts, ctx);
        std::vector<Point> twice = simplify_polyline(b, r, 0, 0, w, once, ctx);
        CT_CHECK(once.size() == twice.size());
        for (std::size_t i = 0; i < once.size(); ++i) CT_CHECK(once[i] == twice[i]);
    }
}

CT_TEST(tuning_exempt_task_keeps_guidance_geometry) {
    // Issue #15 hook: intentional tuning regions skip simplification.
    Board b = base_2layer();
    NetInfo s = make_net(0, "SIG");
    b.nets.push_back(s);
    add_terminal(b, 0, 1.0, 1.0);
    add_terminal(b, 0, 9.0, 9.0);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    ConnectionTask task;
    task.net = 0;
    task.tuning_exempt = true;
    CT_CHECK(simplify_exempt_task(task));
    std::vector<TraceSeg> elbow = {
        {0, 0, {mm_to_nm(1.0), mm_to_nm(1.0)}, {mm_to_nm(9.0), mm_to_nm(1.0)}, mm_to_nm(0.2)},
        {0, 0, {mm_to_nm(9.0), mm_to_nm(1.0)}, {mm_to_nm(9.0), mm_to_nm(9.0)}, mm_to_nm(0.2)}};
    std::vector<char> mask(2, 0);
    SimplifyStats st =
        simplify_candidate_traces(b, r, ctx, 0, elbow, mask, simplify_exempt_task(task));
    CT_CHECK(st.skipped_exempt);
    CT_CHECK(elbow.size() == 2);  // verbatim
    // Same geometry without the flag collapses to one diagonal.
    task.tuning_exempt = false;
    std::vector<TraceSeg> same = {
        {0, 0, {mm_to_nm(1.0), mm_to_nm(1.0)}, {mm_to_nm(9.0), mm_to_nm(1.0)}, mm_to_nm(0.2)},
        {0, 0, {mm_to_nm(9.0), mm_to_nm(1.0)}, {mm_to_nm(9.0), mm_to_nm(9.0)}, mm_to_nm(0.2)}};
    SimplifyStats st2 =
        simplify_candidate_traces(b, r, ctx, 0, same, mask, simplify_exempt_task(task));
    CT_CHECK(!st2.skipped_exempt);
    CT_CHECK(same.size() == 1);
    CT_CHECK(!axis_aligned(same[0]));
}

CT_TEST(simplified_routing_is_deterministic) {
    auto run_once = []() {
        Board b = diagonal_board();
        RuleResolver r = RuleResolver::defaults_for(b);
        EngineOptions opt;
        opt.threads = 1;
        RouterEngine engine(std::move(b), std::move(r), opt);
        RouteReport rep = engine.run();
        CT_CHECK(rep.status == "COMPLETE");
        return rep.board_hash;
    };
    CT_CHECK(run_once() == run_once());
}

CT_TEST(simplified_parallel_matches_single_thread) {
    Board b = diagonal_board();
    NetInfo g = make_net(1, "GND");
    b.nets.push_back(g);
    add_terminal(b, 1, 2.0, 5.0);
    add_terminal(b, 1, 18.0, 7.0);
    std::string h1, h4;
    for (int threads : {1, 4}) {
        Board bb = b;
        // Rebuild terminals per run is unnecessary: boards copy cleanly.
        RuleResolver r = RuleResolver::defaults_for(bb);
        EngineOptions opt;
        opt.threads = threads;
        RouterEngine engine(std::move(bb), std::move(r), opt);
        RouteReport rep = engine.run();
        CT_CHECK(rep.status == "COMPLETE");
        if (threads == 1)
            h1 = rep.board_hash;
        else
            h4 = rep.board_hash;
    }
    CT_CHECK(h1 == h4);
}

int main() { return copperline::test::run_all_tests(); }
