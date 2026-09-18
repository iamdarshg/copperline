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

| date       | code                                   | epochs | wall s | tasks      | terminals  | terminals/s |
|------------|----------------------------------------|--------|--------|------------|------------|-------------|
| 2026-09-18 | 15fd2af (latched, 384/16)              | 54     | 934    | 746 / 2387 | 1208 / 3451 | 1.29 |
| 2026-09-18 | 15fd2af + row-clip penalty (uncommitted)| 61    | 1524   | 791 / 2392 | 1246 / 3451 | 0.82 |

The row-clip improved per-task graph time (~2.67 s -> ~1.75 s per epoch task)
but the later epochs get slower as the board densifies, so total time rose with
the extra epochs. The frontier is still graph construction.

## Rules

- Never commit anything to the drone-arm repo (the board is read-only input).
- Keep the 2 GB router memory budget (`kRouterMemoryBudgetBytes`) intact.
- Correctness still gates: `ctest` must stay green (golden hashes, perf
  graph-identity, thread determinism). Throughput wins must not trade legality.
