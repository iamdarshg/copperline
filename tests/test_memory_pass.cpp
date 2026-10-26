// Copperline: Memory pass tests (Phase 3 item 16).
//
// Pins the deterministic U-pattern router: aligned same-net pairs across
// components get legal 3-leg routes; unaligned or same-component pairs do
// not; legality aborts on blocked pairs; the pass never touches the board
// when disabled (strategy gate lives in the engine, tested via CLI).

#include <cstdint>
#include <string>
#include <vector>

#include "router/memory_pass.h"
#include "router/rules.h"
#include "router/verifier.h"

#include "helpers.h"

namespace copperline {
namespace test {

using namespace copperline;

CT_TEST(memory_pass_commits_aligned_pairs) {
    Board b;
    b.width_nm = mm_to_nm(20.0);
    b.height_nm = mm_to_nm(12.0);
    b.layers.push_back({0, "Top"});
    b.layers.push_back({1, "Bottom"});
    b.defaults.trace_width_nm = mm_to_nm(0.2);
    b.defaults.clearance_nm = mm_to_nm(0.15);

    NetInfo n0;
    n0.id = 0;
    n0.name = "MEM_D0";
    // Aligned on Y (same x): A at (2,3), B at (2,9).
    n0.terminals = {0, 1};
    Terminal ta0{0, 0, {mm_to_nm(2), mm_to_nm(3)}, 0,
                  mm_to_nm(0.5), mm_to_nm(0.5), "U1", "A1"};
    Terminal tb0{1, 0, {mm_to_nm(2), mm_to_nm(9)}, 0,
                  mm_to_nm(0.5), mm_to_nm(0.5), "U2", "D0"};
    NetInfo n1;
    n1.id = 1;
    n1.name = "MEM_D1";
    // Same component pair: must NOT be memory-routed.
    n1.terminals = {2, 3};
    Terminal ta1{2, 1, {mm_to_nm(6), mm_to_nm(3)}, 0,
                  mm_to_nm(0.5), mm_to_nm(0.5), "U1", "A1"};
    Terminal tb1{3, 1, {mm_to_nm(6), mm_to_nm(9)}, 0,
                  mm_to_nm(0.5), mm_to_nm(0.5), "U1", "A2"};
    NetInfo n2;
    n2.id = 2;
    n2.name = "MEM_D2";
    // Diagonal pair: not aligned, must NOT be memory-routed.
    n2.terminals = {4, 5};
    Terminal ta2{4, 2, {mm_to_nm(10), mm_to_nm(3)}, 0,
                  mm_to_nm(0.5), mm_to_nm(0.5), "U1", "A1"};
    Terminal tb2{5, 2, {mm_to_nm(14), mm_to_nm(9)}, 0,
                  mm_to_nm(0.5), mm_to_nm(0.5), "U2", "D2"};
    b.nets = {n0, n1, n2};
    b.terminals = {ta0, tb0, ta1, tb1, ta2, tb2};

    RuleResolver resolver = RuleResolver::defaults_for(b);
    ElectricalContext ctx = resolver.defaultContext();
    MemoryPassReport rep = run_memory_pass(b, resolver, ctx, {});
    CT_CHECK(rep.ran);
    CT_CHECK(rep.pairs_found == 1);  // only MEM_D0 qualifies
    CT_CHECK(rep.pairs_committed == 1);
    CT_CHECK(rep.traces_added == 3);  // 3-leg U
    CT_CHECK(b.traces.size() == 3);
    // Same-layer, same-net, correct span.
    CT_CHECK(b.traces[0].net == 0 && b.traces[0].layer == 0);
    CT_CHECK(b.traces[0].width_nm == mm_to_nm(0.2));
    // The pair is X-aligned (same x=2mm, y=3->9), so the U opens in +X:
    // the run leg sits at x = 2 + pad_half(0.25) + w/2(0.1) + clear(0.15)
    // = 2.5mm, which clears the pad's right edge (2.25mm).
    bool run_beyond_pad = false;
    for (const auto& t : b.traces)
        run_beyond_pad = run_beyond_pad || t.a.x > mm_to_nm(2.3) ||
                         t.b.x > mm_to_nm(2.3);
    CT_CHECK(run_beyond_pad);
    // Diagonal/same-component pairs untouched.
    CT_CHECK(rep.skipped_nets.empty());
}

CT_TEST(memory_pass_illegal_pair_aborts) {
    Board b;
    b.width_nm = mm_to_nm(20.0);
    b.height_nm = mm_to_nm(12.0);
    b.layers.push_back({0, "Top"});
    b.layers.push_back({1, "Bottom"});
    b.defaults.trace_width_nm = mm_to_nm(0.2);
    b.defaults.clearance_nm = mm_to_nm(0.15);
    NetInfo n0;
    n0.id = 0;
    n0.name = "MEM";
    n0.terminals = {0, 1};
    Terminal ta0{0, 0, {mm_to_nm(2), mm_to_nm(3)}, 0,
                  mm_to_nm(0.5), mm_to_nm(0.5), "U1", "A1"};
    Terminal tb0{1, 0, {mm_to_nm(2), mm_to_nm(9)}, 0,
                  mm_to_nm(0.5), mm_to_nm(0.5), "U2", "D0"};
    b.nets = {n0};
    b.terminals = {ta0, tb0};
    // Block the U apex region with a keepout spanning x=1..3, y=4..8.
    Keepout k;
    k.rect = Rect{mm_to_nm(1), mm_to_nm(4), mm_to_nm(3), mm_to_nm(8)};
    k.layer = 0;
    b.keepouts.push_back(k);

    RuleResolver resolver = RuleResolver::defaults_for(b);
    ElectricalContext ctx = resolver.defaultContext();
    MemoryPassReport rep = run_memory_pass(b, resolver, ctx, {});
    CT_CHECK(rep.pairs_found == 1);
    CT_CHECK(rep.pairs_committed == 0);
    CT_CHECK(rep.pairs_skipped_illegal == 1);
    CT_CHECK(b.traces.empty());
    CT_CHECK(!rep.skipped_nets.empty());
    CT_CHECK(rep.skipped_nets[0] == "MEM");
}

// Phase 3 item 17: adjacent-memory pass — adjacent same-net pins, INCLUDING
// pins of the same component (the plain memory pass skips those). Aligned
// pairs get the 3-leg U; offset pairs get the 2-leg L dogleg.
CT_TEST(adjacent_memory_commits_same_component_pairs) {
    Board b;
    b.width_nm = mm_to_nm(14.0);
    b.height_nm = mm_to_nm(8.0);
    b.layers.push_back({0, "Top"});
    b.layers.push_back({1, "Bottom"});
    b.defaults.trace_width_nm = mm_to_nm(0.2);
    b.defaults.clearance_nm = mm_to_nm(0.15);

    NetInfo gnd;
    gnd.id = 0;
    gnd.name = "GND";
    gnd.terminals = {0, 1};
    Terminal g1{0, 0, {mm_to_nm(2), mm_to_nm(2)}, 0, mm_to_nm(0.5),
                mm_to_nm(0.5), "J1", "1"};
    Terminal g2{1, 0, {mm_to_nm(2), mm_to_nm(3.5)}, 0, mm_to_nm(0.5),
                mm_to_nm(0.5), "J1", "2"};
    NetInfo vcc;
    vcc.id = 1;
    vcc.name = "VCC";
    vcc.terminals = {2, 3};
    Terminal v1{2, 1, {mm_to_nm(6), mm_to_nm(2)}, 0, mm_to_nm(0.5),
                mm_to_nm(0.5), "J1", "3"};
    Terminal v2{3, 1, {mm_to_nm(6), mm_to_nm(3.5)}, 0, mm_to_nm(0.5),
                mm_to_nm(0.5), "J1", "4"};
    b.nets = {gnd, vcc};
    b.terminals = {g1, g2, v1, v2};

    RuleResolver resolver = RuleResolver::defaults_for(b);
    ElectricalContext ctx = resolver.defaultContext();

    // The plain memory pass must NOT touch same-component pairs.
    MemoryPassReport plain = run_memory_pass(b, resolver, ctx, {});
    CT_CHECK(plain.mode == "memory");
    CT_CHECK(plain.pairs_found == 0);
    CT_CHECK(b.traces.empty());

    // Adjacent memory routes both pairs (aligned on Y -> U opens in +X).
    MemoryPassReport rep = run_adjacent_memory_pass(b, resolver, ctx, {});
    CT_CHECK(rep.ran);
    CT_CHECK(rep.mode == "adjacent_memory");
    CT_CHECK(rep.pairs_found == 2);
    CT_CHECK(rep.pairs_committed == 2);
    CT_CHECK(rep.traces_added == 6);  // 3 legs per U
    CT_CHECK(b.traces.size() == 6);
    CT_CHECK(rep.skipped_nets.empty());
    // Same net/layer and the run sits beyond the pad edge in +X.
    bool run_beyond = false;
    for (const auto& t : b.traces) {
        CT_CHECK(t.layer == 0);
        CT_CHECK(t.net == 0 || t.net == 1);
        run_beyond = run_beyond || t.a.x > mm_to_nm(2.3) ||
                     t.b.x > mm_to_nm(2.3);
    }
    CT_CHECK(run_beyond);
    // Deterministic: a second pass finds nothing new (copper already joins).
    MemoryPassReport again = run_adjacent_memory_pass(b, resolver, ctx, {});
    CT_CHECK(again.pairs_committed == 0);
    CT_CHECK(b.traces.size() == 6);
}

// Offset (diagonal) adjacent pins take the deterministic 2-leg L dogleg
// with the corner at (b.x, a.y).
CT_TEST(adjacent_memory_offset_pair_dogleg) {
    Board b;
    b.width_nm = mm_to_nm(14.0);
    b.height_nm = mm_to_nm(8.0);
    b.layers.push_back({0, "Top"});
    b.layers.push_back({1, "Bottom"});
    b.defaults.trace_width_nm = mm_to_nm(0.2);
    b.defaults.clearance_nm = mm_to_nm(0.15);
    NetInfo n;
    n.id = 0;
    n.name = "SIG";
    n.terminals = {0, 1};
    Terminal a{0, 0, {mm_to_nm(2), mm_to_nm(2)}, 0, mm_to_nm(0.5),
               mm_to_nm(0.5), "U1", "A1"};
    Terminal c{1, 0, {mm_to_nm(3), mm_to_nm(3)}, 0, mm_to_nm(0.5),
               mm_to_nm(0.5), "U2", "B1"};
    b.nets = {n};
    b.terminals = {a, c};
    RuleResolver resolver = RuleResolver::defaults_for(b);
    ElectricalContext ctx = resolver.defaultContext();
    MemoryPassReport rep = run_adjacent_memory_pass(b, resolver, ctx, {});
    CT_CHECK(rep.pairs_found == 1);
    CT_CHECK(rep.pairs_committed == 1);
    CT_CHECK(rep.traces_added == 2);  // 2-leg dogleg
    CT_CHECK(b.traces.size() == 2);
    // Corner at (b.x, a.y) = (3, 2).
    bool corner = false;
    for (const auto& t : b.traces) {
        if (t.a == Point{mm_to_nm(3), mm_to_nm(2)} ||
            t.b == Point{mm_to_nm(3), mm_to_nm(2)})
            corner = true;
    }
    CT_CHECK(corner);
}

// A keepout across the U blocks the pair: nothing commits for that net.
CT_TEST(adjacent_memory_illegal_pair_aborts) {
    Board b;
    b.width_nm = mm_to_nm(14.0);
    b.height_nm = mm_to_nm(8.0);
    b.layers.push_back({0, "Top"});
    b.layers.push_back({1, "Bottom"});
    b.defaults.trace_width_nm = mm_to_nm(0.2);
    b.defaults.clearance_nm = mm_to_nm(0.15);
    NetInfo n;
    n.id = 0;
    n.name = "GND";
    n.terminals = {0, 1};
    Terminal a{0, 0, {mm_to_nm(2), mm_to_nm(2)}, 0, mm_to_nm(0.5),
               mm_to_nm(0.5), "J1", "1"};
    Terminal c{1, 0, {mm_to_nm(2), mm_to_nm(3.5)}, 0, mm_to_nm(0.5),
               mm_to_nm(0.5), "J1", "2"};
    b.nets = {n};
    b.terminals = {a, c};
    Keepout k;
    k.rect = Rect{mm_to_nm(1.5), mm_to_nm(1.5), mm_to_nm(3.5), mm_to_nm(4.0)};
    k.layer = 0;
    b.keepouts.push_back(k);
    RuleResolver resolver = RuleResolver::defaults_for(b);
    ElectricalContext ctx = resolver.defaultContext();
    MemoryPassReport rep = run_adjacent_memory_pass(b, resolver, ctx, {});
    CT_CHECK(rep.pairs_found == 1);
    CT_CHECK(rep.pairs_committed == 0);
    CT_CHECK(rep.pairs_skipped_illegal == 1);
    CT_CHECK(rep.skipped_nets.size() == 1 && rep.skipped_nets[0] == "GND");
    CT_CHECK(b.traces.empty());
}

// Regression: pattern legs must honour the net's required width, not the
// board default. A 0.8 mm min-width net on a 0.2 mm board previously got
// 0.25 mm copper and turned a real board into a VIOLATION.
CT_TEST(adjacent_memory_uses_net_required_width) {
    Board b;
    b.width_nm = mm_to_nm(14.0);
    b.height_nm = mm_to_nm(8.0);
    b.layers.push_back({0, "Top"});
    b.layers.push_back({1, "Bottom"});
    b.defaults.trace_width_nm = mm_to_nm(0.2);
    b.defaults.clearance_nm = mm_to_nm(0.15);
    NetInfo n;
    n.id = 0;
    n.name = "VCC";
    n.has_min_width = true;
    n.min_width_nm = mm_to_nm(0.8);
    n.terminals = {0, 1};
    Terminal a{0, 0, {mm_to_nm(2), mm_to_nm(2)}, 0, mm_to_nm(0.9),
               mm_to_nm(0.9), "J1", "1"};
    Terminal c{1, 0, {mm_to_nm(2), mm_to_nm(4)}, 0, mm_to_nm(0.9),
               mm_to_nm(0.9), "J1", "2"};
    b.nets = {n};
    b.terminals = {a, c};
    RuleResolver resolver = RuleResolver::defaults_for(b);
    ElectricalContext ctx = resolver.defaultContext();
    BoardVerifier verifier;
    CT_CHECK(verifier.verify(b, resolver, ctx).legal);
    MemoryPassReport rep = run_adjacent_memory_pass(b, resolver, ctx, {});
    CT_CHECK(rep.pairs_found == 1);
    // Either the pair is skipped (width does not fit) or it commits at the
    // required width. It must never commit narrow copper.
    for (const auto& t : b.traces) CT_CHECK(t.width_nm >= mm_to_nm(0.8));
    const VerifyResult vr = verifier.verify(b, resolver, ctx);
    CT_CHECK(vr.legal);
    if (rep.pairs_committed > 0) CT_CHECK(b.traces.size() == 3);
}

}  // namespace test
}  // namespace copperline

int main() { return copperline::test::run_all_tests(); }