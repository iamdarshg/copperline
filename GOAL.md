# ESC routing goal — the metric that matters

Target board (read-only, never commit to drone-arm):

    D:\CodeProjects\drone-arm\.worktrees\main-route-20260910\reports\esc-candidates\fd1ead6\esc_rev_b.kicad_pcb

4-layer, ~3451 terminals, ~2400 connection tasks. It does **not** fully route;
we are pushing the frontier, not chasing 100%.

## Objective (in priority order)

1. **Connections per second** — wall-clock throughput.
2. **Total connections after ~60 greedy epochs.**

"Connections" is counted both ways and both are reported:

- tasks routed: `stats.tasks_routed`
- terminals connected: `connected_terminals / total_terminals`

Everything else (stage timings, graph sizes, hashes, DRC counts) is a *means*
to those two numbers, not a goal.

## Canonical measurement

    router route <board> --json --threads 14 --seed 42 \
      --max-epochs 60 --timeout 1500 --time-stages --report <out>.json

- `--threads 14`: always leave 2 logical CPUs (1 physical core) free for the
  machine; do not run the router at 16 threads.
- Report: epochs completed, wall seconds, tasks routed, terminals connected,
  and both rates (tasks/s, terminals/s).

## Baselines

| date       | code                                  | epochs | wall s | tasks      | terminals   | terminals/s |
|------------|---------------------------------------|--------|--------|------------|-------------|-------------|
| 2026-09-18 | latched 384/16                        | 54     | 934    | 746 / 2387 | 1208 / 3451 | 1.29 |
| 2026-09-18 | + row-clip penalty, 384/16            | 61     | 1524   | 791 / 2392 | 1246 / 3451 | 0.82 |
| 2026-09-18 | + 192/8                               | 60     | 1462   | 827 / 2457 | 1260 / 3451 | 0.862 |
| 2026-09-18 | + route_k=1 at board scale                | 60 | 781  | 827 / 2457 | 1260 / 3451 | 1.613 |
| 2026-09-18 | + exact graph-build opts                  | 60 | 720  | 827 / 2457 | 1260 / 3451 | 1.750 |
| 2026-09-18 | **+ faster predicates + batch-estimate fix (current best, 8T/600MB)** | **60** | **603** | **995 / 2463** | **1502 / 3451** | **2.493** |

The batch-estimate fix matters most: `kPerCandidateBytes` was 64 MB, which
capped the epoch batch at `floor(budget/64)` for no real memory benefit (peak
RSS is transient and batch-independent: batch 9 -> 1931 MB, batch 32 -> 2039 MB,
~5 MB marginal per task). Correcting it to a still-conservative 16 MB restores
the batch to 37 at a 600 MB budget, taking 60 epochs from 386 tasks / 873
terminals to 995 / 1502.

Exact graph-build optimizations (identical graph counters, identical copper):
inline `DecidedEdge` storage (no heap alloc per probe), row-clipped obstacle
query (`gb_edges_ms` 2211s -> 1729s CPU), dense per-build clearance/node
caches, and resetting only the live edge/probe counts instead of reconstructing
the struct (avoids 6 string ctor/dtor per `decide_edge`).

The route_k=1 policy (single cheapest legal path instead of the K-corridor
portfolio + per-alternative future-obstruction scoring) produced **identical
connections at +87% throughput**. Same pattern as guidance and the graph clamp:
a phase-escalated quality feature that costs more than it delivers at board
scale.

Fixed-60-epoch sweep (14 threads, 2 GB), scoring both metrics:

| bases/k | tasks | terminals | terminals/s |
|---------|-------|-----------|-------------|
| 128/8   | 715   | 1181      | 0.78 |
| **192/8** | **827** | **1260** | **0.862** |
| 384/16  | 791   | 1246      | 0.818 |
| 1024/32 | worse | worse     | -   |
| 2048/64 | worse | worse     | -   |

Method note: sweep at a **fixed epoch count**, not a wall-clock timeout — the
deadline makes A* cuts timing-dependent, so timeout-based sweeps are noise.

## Known constraints (measured)

- Peak RSS on the ESC is a *transient* ~1.9 GB (43 MB at start -> ~1930 MB peak
  -> 59 MB at exit) and is essentially independent of the batch width. The
  memory budget is a batch-width model, NOT a cap on actual RSS: at a 600 MB
  budget, peak RSS is still ~1.9 GB. So a small budget buys no real footprint
  reduction, only a smaller batch (hence the estimate fix above).
- Graph construction is ~70% of CPU (`gb_edges_ms` dominates), and A* exhausts
  the graph, so lazy/late binding of edges or penalties saves nothing.
- Exactness gate: graph counters (`gb_edges_total`, and the aligned/knn/via
  split) must be byte-identical across a change. The small perf identity hash is
  NOT sufficient — it missed nothing here, but always confirm on the ESC with an
  untruncated run (a short `--timeout` truncates and fakes a difference).

## Rules

- Never commit anything to the drone-arm repo (the board is read-only input).
- Keep the 2 GB router memory budget (`kRouterMemoryBudgetBytes`) intact.
- Correctness still gates: `ctest` must stay green (golden hashes, perf
  graph-identity, thread determinism). Throughput wins must not trade legality.
