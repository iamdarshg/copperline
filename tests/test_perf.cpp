// Prompt 3 performance gates: minimal throughput numbers.
//
// These are regression floors, not benchmarks-of-record: margins are wide
// (typically 10-50x below observed Release throughput on a 16-core dev
// machine) so they only trip on genuine algorithmic regressions, never on
// CI noise. Every test prints its measured numbers; `router benchmark`
// remains the tool for exact single-vs-parallel comparisons.
//
// Observed reference (Release, 16 cores, 2026-09-14):
//   perf32 (32 tasks, maze cells): single ~270ms / ~2000 exp,
//                                  parallel-16 ~61ms, speedup ~4.4x,
//                                  identical geometry hash.
#include <chrono>
#include <cstdio>
#include <thread>

#include "helpers.h"
#include "router/engine.h"
#include "router/parallel.h"
#include "router/sparse_graph.h"
#include "router/verifier.h"

using namespace copperline;
using namespace copperline::test;

namespace {

// 32 independent maze-cell tasks (mirrors the reference workload). Each cell
// forces a detour around 3 keepout blocks, so graphs are non-trivial but the
// board is fully routable and contention-free.
Board perf_board() {
    Board b;
    b.source_format = "test";
    b.width_nm = mm_to_nm(160.0);
    b.height_nm = mm_to_nm(40.0);
    b.layers.push_back({0, "Top"});
    b.layers.push_back({1, "Bottom"});
    NetId nid = 0;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 8; ++col) {
            double x0 = col * 20.0, y0 = row * 10.0;
            NetInfo n = make_net(nid, "N" + std::to_string(nid));
            n.has_current = true;
            n.current_a = 0.1;
            b.nets.push_back(n);
            add_terminal(b, nid, x0 + 2, y0 + 5);
            add_terminal(b, nid, x0 + 18, y0 + 5);
            Keepout k1{{mm_to_nm(x0 + 5), mm_to_nm(y0), mm_to_nm(x0 + 7), mm_to_nm(y0 + 7)},
                       0,
                       "ka"};
            Keepout k2{{mm_to_nm(x0 + 9), mm_to_nm(y0 + 3), mm_to_nm(x0 + 11), mm_to_nm(y0 + 10)},
                       0,
                       "kb"};
            Keepout k3{{mm_to_nm(x0 + 13), mm_to_nm(y0), mm_to_nm(x0 + 15), mm_to_nm(y0 + 7)},
                       0,
                       "kc"};
            b.keepouts.push_back(k1);
            b.keepouts.push_back(k2);
            b.keepouts.push_back(k3);
            ++nid;
        }
    }
    return b;
}

RouteReport run_n(Board b, int threads, double& wall_s) {
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = threads;
    auto t0 = std::chrono::steady_clock::now();
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    auto t1 = std::chrono::steady_clock::now();
    wall_s = std::chrono::duration<double>(t1 - t0).count();
    return rep;
}

}  // namespace

CT_TEST(perf_single_thread_throughput_floor) {
    double wall = 0;
    RouteReport rep = run_n(perf_board(), 1, wall);
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.stats.tasks_routed == 32);
    double tasks_per_s = 32.0 / std::max(wall, 1e-3);
    double exp_per_s = (double)rep.stats.expansions_total / std::max(wall, 1e-3);
    std::printf("  [perf] single: %.3fs wall, %.1f tasks/s, %.0f expansions/s\n", wall,
                tasks_per_s, exp_per_s);
    CT_CHECK(wall < 15.0);        // completes a 32-task board quickly
    CT_CHECK(tasks_per_s > 5.0);  // minimal routing throughput
    CT_CHECK(exp_per_s > 100.0);  // minimal search throughput
    // The result must independently verify clean.
    BoardVerifier v;
    ElectricalContext ctx;
    Board b = perf_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    RouterEngine eng(std::move(b), std::move(r), opt);
    RouteReport rep2 = eng.run();
    RuleResolver rv = RuleResolver::defaults_for(eng.committed());
    CT_CHECK(v.verify(eng.committed(), rv, ctx).ok);
    (void)rep2;
}

CT_TEST(perf_parallel_speedup_floor) {
    double w1 = 0, wN = 0;
    RouteReport r1 = run_n(perf_board(), 1, w1);
    unsigned hw = std::thread::hardware_concurrency();
    if (hw < 2) hw = 2;
    RouteReport rN = run_n(perf_board(), (int)hw, wN);
    CT_CHECK(r1.status == "COMPLETE");
    CT_CHECK(rN.status == "COMPLETE");
    CT_CHECK(r1.board_hash == rN.board_hash);  // worker count never changes copper
    CT_CHECK(rN.stats.threads_used >= 2);
    double speedup = w1 / std::max(wN, 1e-6);
    std::printf("  [perf] single %.3fs vs parallel(%u) %.3fs -> speedup %.2fx\n", w1, hw, wN,
                speedup);
    CT_CHECK(speedup > 1.5);  // materially useful multicore execution
    CT_CHECK(wN < 15.0);
}

CT_TEST(perf_graph_build_floor) {
    // Dense pad field: one task graph against ~200 foreign pads.
    Board b = base_2layer(30.0, 30.0);
    NetInfo n0 = make_net(0, "SIG");
    n0.has_current = true;
    n0.current_a = 0.1;
    b.nets.push_back(n0);
    NetInfo n1 = make_net(1, "FIELD");
    n1.has_current = true;
    n1.current_a = 0.1;
    b.nets.push_back(n1);
    add_terminal(b, 0, 1.0, 15.0);
    add_terminal(b, 0, 29.0, 15.0);
    for (int i = 0; i < 14; ++i)
        for (int j = 0; j < 14; ++j) add_terminal(b, 1, 3.0 + i * 1.8, 3.0 + j * 1.8);
    RuleResolver r = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    const Terminal* ta = b.find_terminal(b.nets[0].terminals[0]);
    const Terminal* tb = b.find_terminal(b.nets[0].terminals[1]);
    auto t0 = std::chrono::steady_clock::now();
    SparseRoutingGraph g =
        SparseRoutingGraph::build(b, r, 0, ta->pos, tb->pos, ta->layer, tb->layer,
                                  r.requiredTraceWidth(0, 0, ctx), ctx);
    auto t1 = std::chrono::steady_clock::now();
    double s = std::chrono::duration<double>(t1 - t0).count();
    std::printf("  [perf] graph build: %.3fs, %d nodes, %d edges\n", s,
                (int)g.nodes().size(), g.stats().edge_count);
    CT_CHECK(g.src_node() >= 0 && g.dst_node() >= 0);
    CT_CHECK(s < 5.0);  // corridor clipping keeps dense fields tractable
}

CT_TEST(perf_verifier_floor) {
    double wall = 0;
    Board b = perf_board();
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    RouterEngine eng(std::move(b), std::move(r), opt);
    RouteReport rep = eng.run();
    CT_CHECK(rep.status == "COMPLETE");
    RuleResolver rv = RuleResolver::defaults_for(eng.committed());
    ElectricalContext ctx;
    BoardVerifier v;
    auto t0 = std::chrono::steady_clock::now();
    VerifyResult vr = v.verify(eng.committed(), rv, ctx);
    auto t1 = std::chrono::steady_clock::now();
    double s = std::chrono::duration<double>(t1 - t0).count();
    std::printf("  [perf] verifier: %.3fs for %d traces\n", s,
                (int)eng.committed().traces.size());
    CT_CHECK(vr.ok);
    CT_CHECK(s < 5.0);
    (void)wall;
}

int main() { return copperline::test::run_all_tests(); }
