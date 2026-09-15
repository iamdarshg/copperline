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
// Post issue #10 (hierarchical guidance, same machine class): exact
// expansions collapse (~100 for 32 tasks, all guided first-try) while
// coarse guidance adds ~70k expansions; wall ~0.9s single / ~0.2s
// parallel, speedup ~5x, identical geometry. The exp_per_s floor below
// therefore counts total search work (exact + coarse).
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
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

// Graph-identity hash (perf probe-audit gate): FNV-1a over every committed
// node, edge (in stored order) and per-node rejected probe. Any change to the
// built graph — nodes, edges, fields, or #23 frontier evidence — flips it.
std::uint64_t graph_identity_hash(const SparseRoutingGraph& g) {
    std::uint64_t h = 1469598103934665603ULL;
    auto mix_u64 = [&](std::uint64_t v) {
        h ^= v;
        h *= 1099511628211ULL;
    };
    auto mix_str = [&](const std::string& s) {
        for (unsigned char ch : s) {
            h ^= ch;
            h *= 1099511628211ULL;
        }
        mix_u64(s.size());
    };
    mix_u64(static_cast<std::uint64_t>(g.src_node()));
    mix_u64(static_cast<std::uint64_t>(g.dst_node()));
    for (int d : g.dst_nodes()) mix_u64(static_cast<std::uint64_t>(d + 2));
    for (const auto& n : g.nodes()) {
        mix_u64(static_cast<std::uint64_t>(n.p.x));
        mix_u64(static_cast<std::uint64_t>(n.p.y));
        mix_u64(static_cast<std::uint64_t>(n.layer));
        mix_u64(static_cast<std::uint64_t>(n.base + 2));
    }
    for (std::size_t i = 0; i < g.nodes().size(); ++i) {
        for (const auto& e : g.edges(static_cast<int>(i))) {
            mix_u64(static_cast<std::uint64_t>(e.to));
            mix_u64(static_cast<std::uint64_t>(e.len_nm));
            mix_u64(static_cast<std::uint64_t>(e.dir1 + 8));
            mix_u64(static_cast<std::uint64_t>(e.dir2 + 8));
            mix_u64(static_cast<std::uint64_t>(e.elbow.x));
            mix_u64(static_cast<std::uint64_t>(e.elbow.y));
            mix_u64(e.is_via ? 1ULL : 0ULL);
            mix_u64(static_cast<std::uint64_t>(e.penalty_nm));
        }
        for (const auto& p : g.rejected(static_cast<int>(i))) {
            mix_u64(static_cast<std::uint64_t>(p.blocker_net + 2));
            mix_str(p.kind);
            mix_str(p.desc);
            mix_u64(static_cast<std::uint64_t>(p.layer));
            mix_u64(static_cast<std::uint64_t>(p.pos.x));
            mix_u64(static_cast<std::uint64_t>(p.pos.y));
        }
    }
    return h;
}

}  // namespace

CT_TEST(perf_single_thread_throughput_floor) {
    double wall = 0;
    RouteReport rep = run_n(perf_board(), 1, wall);
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.stats.tasks_routed == 32);
    double tasks_per_s = 32.0 / std::max(wall, 1e-3);
    // Issue #10: hierarchical guidance moves search work into the coarse
    // levels, so exact expansions drop by design. Throughput counts all
    // pathfinding work (exact + coarse); the tasks/s floor above still
    // guards end-to-end routing independently.
    std::int64_t search_total =
        rep.stats.expansions_total + rep.stats.hierarchy_coarse_expansions;
    double exp_per_s = (double)search_total / std::max(wall, 1e-3);
    std::printf("  [perf] single: %.3fs wall, %.1f tasks/s, %.0f expansions/s\n", wall,
                tasks_per_s, exp_per_s);
    std::printf("  [perf]   exact=%lld coarse=%lld guided=%d fallbacks=%d\n",
                (long long)rep.stats.expansions_total,
                (long long)rep.stats.hierarchy_coarse_expansions,
                rep.stats.hierarchy_guided_tasks, rep.stats.hierarchy_fallback_tasks);
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
    std::uint64_t ident = graph_identity_hash(g);
    std::printf("  [perf] graph build: %.3fs, %d nodes, %d edges, ident=0x%016llx\n", s,
                (int)g.nodes().size(), g.stats().edge_count,
                (unsigned long long)ident);
    CT_CHECK(g.src_node() >= 0 && g.dst_node() >= 0);
    // Probe-audit identity gate: the committed graph (nodes, edges, #23
    // rejected-probe evidence) must be bit-identical across probe-cut work.
    CT_CHECK((int)g.nodes().size() == 768);
    CT_CHECK(g.stats().edge_count == 42236);
    // Probe-audit identity gate: pre-cut baseline ident=0x060c2fdf3f4510f3.
    CT_CHECK(ident == 0x060c2fdf3f4510f3ULL);
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
