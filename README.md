# Copperline — headless PCB autorouter for agents

Copperline is a **CLI-only, headless** PCB autorouter. There is no GUI, no
viewer, no desktop shell. The consumers are AI agents, scripts, CI systems and
EDA pipelines: every important operation has a stable machine-readable JSON
representation, documented nonzero exit codes, and deterministic output for a
given `(board, rules, seed)`.

> Status: **Prompt 4 recovery** — rip-up/reroute + chess-engine-style
> meta-search (stall detection, blocker attribution, dependency graph,
> protection-weighted selective rip-up, 128-bit state hashing, transposition
> table, history heuristic, PV reuse, iterative deepening/widening, parallel
> speculative branches, `FAST → RECOVERY → EXHAUSTIVE_LOCAL_RECOVERY`) on top
> of Prompt 1 foundation + Prompt 2 escape + Prompt 3 parallel epochs.
> Adapters/optimizer/release (P5) are explicitly planned and reported as
> such by `router capabilities`.

## Quick start (agents: copy/paste)

```bash
# Build (C++20, CMake 3.20+, no third-party dependencies)
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build

# Run the full test suite (123 cases, 11 binaries, ~21 s)
ctest --test-dir build --output-on-failure

# Route a board, machine-readable
./build/router route fixtures/open_2layer.json --json --seed 42 \
  --output routed.json --report report.json

# Route with electrical sidecar rules (multicore: identical copper at any --threads)
./build/router route fixtures/high_current.json --config fixtures/rules_demo.json \
  --threads 16 --seed 42 --output routed.json --report report.json --json

# Greedy dead end repaired by meta-search (see report "recovery")
./build/router route fixtures/forced_ripup.json --json --seed 42 \
  --output routed.json --report report.json

# Watch epoch progress as NDJSON on stderr (stdout stays pure JSON)
./build/router route fixtures/narrow_channel.json --json --threads 8 --progress

# Compare single-thread vs multicore with real measured numbers
./build/router benchmark fixtures/obstacle_detour.json --json --threads 8

# Verify committed copper independently of the router's search state
./build/router verify routed.json --json

# Analyze difficulty, density, bottlenecks
./build/router analyze board.json --json

# Plan fine-pitch escapes (centre-out, K-best, JSON diagnostics)
./build/router escape fixtures/bga_4x4.json --json
./build/router escape fixtures/bga_8x8.json --json --report escape.json
# rc 4 + "INCOMPLETE" when any pad carries an infeasibility record

# Ingest a KiCad board directly
./build/router analyze fixtures/minimal.kicad_pcb --json
./build/router route  my_board.kicad_pcb --json --output routed.json

# What can this build do?
./build/router capabilities --json
```

`--json` mode: stdout contains **only** JSON. Diagnostics go to stderr.
`--quiet` suppresses everything but errors. Schemas are versioned, e.g.
`"schema": "copperline/route-report/1"`.

## Exit codes

| Code | Meaning |
| ---- | ------- |
| 0 | success (route COMPLETE / verify clean / analyze ok) |
| 2 | invalid input (missing file, bad JSON/flags, unknown command) |
| 3 | malformed design rules (bad net rules or `--config` content) |
| 4 | routing incomplete (finished; terminals remain unconnected) |
| 5 | hard-rule violation (verify found clearance/width/via/keepout hits) |
| 6 | internal failure (unexpected exception) |
| 7 | search budget exhausted (`--max-search-nodes` / `--timeout` hit) |

Future commands (`benchmark`, `explain-failure`) exit 2 with
`"code": "not_implemented"` until their prompt lands. `escape` returns 4
when any pad ends infeasible (records included in the JSON report).

## Electrical awareness

Nets may state continuous/peak current, min/preferred widths, width/via
classes, neckdown permission, voltages and voltage classes. Absent current
falls back to explicit board defaults and is **reported**
(`default_current_used`, `defaults_used_for_current`) — never silently
invented.

Width resolution (first hit wins, i.e. explicit always beats inference):

1. `explicit` — net `min_width_mm`
2. `width_class` — user width/current class
3. `ipc_estimate` — configurable `width = I · k` clamped to `[min_mm, max_mm]`
4. `board_default` — `defaults.trace_width_mm`

Clearance between two nets resolves from **both** nets (pair rule >
class pair > voltage-difference table > board default), plus conservative
per-net floors (KiCad net-class semantics: max wins). The model lives in
`CurrentCapacityModel` / `VoltageClearanceModel` behind `RuleResolver` —
never inside A*.

Sidecar `--config` (JSON) example — see `fixtures/rules_demo.json`:

```json
{
  "ipc": {"enabled": true, "mm_per_amp": 0.75, "min_mm": 0.15, "max_mm": 10.0},
  "voltage_table": [
    {"delta_v_min": 0, "clearance_mm": 0.15},
    {"delta_v_min": 50, "clearance_mm": 1.0}
  ],
  "class_pairs": [{"a": "HV", "b": "LOGIC", "clearance_mm": 1.5}],
  "pair_rules": [{"a": "MAINS_L", "b": "SELV", "clearance_mm": 3.0}],
  "width_classes": [{"name": "SIGNAL", "min_width_mm": 0.15}],
  "via_classes": [{"name": "POWER", "outer_mm": 1.2, "hole_mm": 0.6, "max_current_a": 8.0}]
}
```

## Board formats

| Format | Status |
| ------ | ------ |
| Native JSON (`copperline/board/1`) | full round-trip (input + `--output`) |
| KiCad `.kicad_pcb` | **ingest**: outline, copper layers, nets + net classes (width/clearance/via rules), footprints/pads, tracks, vias, keepout rule areas |
| Specctra DSN / SES, IPC-2581, Gerber | Prompt 5 (importer interface `BoardImporter` is ready) |

KiCad phase-1 approximations (all surfaced in `import_warnings`): pad shapes
reduce to rotated bounding boxes; thru-hole pads fan to one terminal per
copper layer; non-Manhattan tracks import as-is (verify-only); copper zones
are **ignored with a warning** (full pour support is Prompt 5); coordinate
translation to a `(0,0)` origin is internal only.

## Architecture

All geometry is integer nanometres; legality uses exact integer arithmetic
(`__int128` intermediates — GCC/Clang; MSVC port is Prompt-5 work). The
principal routing representation is vector geometry + a per-task **sparse
visibility-style graph** (endpoints + clearance-expanded obstacle corners +
border corridors, Manhattan edges, via edges) — never a fine raster. Hard
electrical legality is structural (illegal edges are never built); costs only
order legal alternatives.

```
include/router  geometry.h  json.h  sexpr.h  board.h  rules.h
                spatial_index.h  density.h  route_tree.h
                sparse_graph.h  astar.h  escape.h  parallel.h  recovery.h
                engine.h  verifier.h  analyze.h
src             json/sexpr/board/kicad/rules/spatial_index/density/...
                route_tree/sparse_graph/astar/escape/parallel/recovery/...
                engine/verifier/analyze/main(CLI)
tests           11 binaries, 123 cases (no third-party framework)
fixtures        6 Prompt-1/3 JSON boards (open_2layer, obstacle_detour,
                high_current, voltage_clearance, narrow_channel, ...)
                + 9 fine-pitch golden boards
                (bga_4x4, bga_8x8, bga_8x8_via, irregular_array, dense_qfn,
                high_current_bga, mixed_voltage_bga, greedy_outside_first,
                impossible_escape) + forced_ripup (Prompt 4) + rules sidecar
                + KiCad sample + violation board
```

Global acceptance is lexicographic (connectivity > hard violations >
resource overuse > electrical geometry > vias > length > bends >
congestion) and enforced by the engine/verifier split.

## Fine-pitch escape (Prompt 2)

BGA/LGA/dense-array routing is structurally **centre-out**: deeper pads are
planned before shallower ones — not a centrality bonus, but an eligibility
order enforced by the planner. A shallower pad is never committed while a
deeper eligible pad has neither a viable candidate nor an explicit
temporary-infeasibility record (`router escape --json` reports
`eligibility_order`, `commit_order`, per-pad `centre_depth`, `candidate_count`,
`candidate_portals`, `via_decisions` and `infeasibility_reason`/`blockers`
under schema `copperline/escape-report/1`; exit 4 when any pad is infeasible).

Within equal centre depth: fewest legal exits → fewest via sites → highest
density → highest downstream difficulty → stable terminal id. Candidates are
K-best with distinct signatures
(`portal|direction|layer-strategy|via-class|bottleneck`); strategies cover
`same-layer`, `dogbone`, `via-first` (legal neckdown honoured, never invented)
and `multilayer`. The engine routes fine-pitch tasks centre-out via a depth
difficulty boost. `router analyze --json` gains a `fine_pitch` section.

## Parallel global routing (Prompt 3)

Workers route connection tasks against an **immutable committed snapshot**
and return candidates only; a deterministic central arbiter revalidates,
builds the conflict graph, selects a compatible subset in
`(difficulty, net, a, b)` order, and commits it as one atomic epoch.
Worker completion order cannot change the result: batch membership is a
pure function of scheduler order with a fixed width, so
`--threads 1/2/4/16` produce bit-identical copper (see `board_hash`).

- Difficulty vectors: span, endpoint density, free space, corridor
  count/scarcity, trace width, voltage-clearance burden, layer/via
  restrictions, previous failures, fine-pitch depth.
- Pathfinder-style congestion: `present_cost` per epoch vs persistent
  `history_cost`; soft reservations from probable corridors (bounded
  costs, never hard locks — the only legal route is never rejected for
  predicted use). Final DRC legality is always exact integer geometry.
- Starvation-free scheduler: tasks that sat out the previous epoch go
  first, so failing giants cannot block easy tasks forever.
- Agent observables: `epoch_log`, `candidates_accepted/rejected`,
  `congestion_hotspots`, `board_hash`, `--progress` NDJSON on stderr,
  `router benchmark` (real wall-clock single-vs-parallel + speedup).

Reference throughput (Release, 16 cores): 32-task maze board COMPLETE in
~270 ms single-thread (~260 tasks/s) vs ~60 ms parallel (~4.4x speedup,
identical hash). `tests/test_perf.cpp` enforces generous floors
(>5 tasks/s, >1.5x speedup) so regressions trip loudly.

## Rip-up / meta-search recovery (Prompt 4)

Greedy epochs stall when an early route seals the only corridor a later net
needed (`fixtures/forced_ripup.json`: SEAL straight traps TRAPPED in a
single-layer U-enclosure). Recovery repairs this without touching legality:

- Stall detection (epoch acceptance accounting) + per-task A* frontier
  diagnostics (`closest_node`, `closest_goal_dist_mm`, `expansions`).
- Blocker attribution (`trace:net=SEAL …`) → blocked-net dependency graph
  (`failed → blocker_net` edges, in `report.recovery.dependency_graph`).
- Protection-weighted selective rip-up: escape stubs +5, stable routes +1/gen,
  fixed user copper never ripped (infinite protection in every mode).
- 128-bit incremental/Zobrist-style state hash (`report.state_hash`) +
  transposition table (prunes revisited states) + history heuristic +
  principal-variation reuse across generations.
- Iterative deepening/widening over `FAST → RECOVERY → EXHAUSTIVE_LOCAL_RECOVERY`
  (A* budgets ×1/×4/×16, branch widths 2/4/8, rip breadth 1/2/4).
- Parallel speculative branches evaluated concurrently but committed
  deterministically: `--threads 1/2/4` yield identical `board_hash`.
- Strict connectivity priority: `branch_better` never prefers a shorter
  board with an unconnected pad. Unexhausted dead ends report
  `SEARCH_BUDGET_EXHAUSTED_WITH_UNROUTED_CONNECTIONS`, otherwise
  `UNROUTABLE_UNDER_CONFIGURED_CONSTRAINTS_AND_BUDGET` (see
  `report.result_category`; per-failure `ripup_attempts`, `modes_attempted`,
  `frontier` included). No minimax/alpha-beta: the tree holds routing
  decisions (escape bundle, channel owner, layer strategy, rip set, retry
  order — failed task first), A* keeps detailed geometry.

## Known limitations (Prompt 4)

- No optimizer — cleanup passes (bend/via/length) are Prompt 5; recovery
  targets connectivity, not polish.
- Impossible boards report `INCOMPLETE`/`BUDGET_EXHAUSTED` with blocker hints.
- Escape is complete on 4x4/QFN/irregular fixtures; ultra-dense 8x8
  (0.8 mm pitch, zero same-layer channels) escapes 48/64 with explicit
  infeasibility records for the rest — recovery is Prompt 4 work.
- Escape planner on ultra-dense 8x8 (0.8 mm pitch, zero same-layer channels)
  escapes 48/64 with explicit infeasibility records for the rest —
  recovery is Prompt 4 work. Sparse-graph corridor clipping cut the escape
  suite from ~3 min to ~18 s as a side effect.
- No DSN/SES export yet; routed output is native JSON (Prompt 5).
- `router explain-failure` is a stub by design (Prompt 5).
