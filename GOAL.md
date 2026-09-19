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
| 2026-09-18 | **+ 192/8 (current best)**            | **60** | **1462** | **827 / 2457** | **1260 / 3451** | **0.862** |

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

- Peak RSS on the ESC is ~2039 MB, i.e. the 2 GB budget is real and binding.
  The epoch batch is capped at 32 candidates by `2048 MB / 64 MB`; it cannot be
  widened without exceeding the cap. Reducing per-graph memory is therefore a
  *throughput* lever, not just a footprint one.
- Graph construction is ~70% of CPU (`gb_edges_ms` dominates), and A* exhausts
  the graph (5712 expansions over 1101 nodes), so lazy/late binding of edges or
  penalties saves nothing.

## Rules

- Never commit anything to the drone-arm repo (the board is read-only input).
- Keep the 2 GB router memory budget (`kRouterMemoryBudgetBytes`) intact.
- Correctness still gates: `ctest` must stay green (golden hashes, perf
  graph-identity, thread determinism). Throughput wins must not trade legality.
