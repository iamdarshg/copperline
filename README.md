# Copperline — headless PCB autorouter for agents

Copperline is a **CLI-only, headless** PCB autorouter. There is no GUI, no
viewer, no desktop shell. The consumers are AI agents, scripts, CI systems and
EDA pipelines: every important operation has a stable machine-readable JSON
representation, documented nonzero exit codes, and deterministic output for a
given `(board, rules, seed)`.

> Status: **Prompt 5 release (v0.1.0) + format completion** — DSN import +
> SES export, KiCad export, Gerber RS-274X import, IPC-2581C import,
> DSN/KiCad copper-pour import, sidecar `nets` intent, transactional
> cleanup optimizer, `explain-failure`, golden suite (10 routable +
> impossible), determinism + benchmark evidence, on top of Prompt 4
> recovery + Prompt 3 parallel epochs + Prompt 2 escape.
> `router capabilities` reports exactly what is done (`unsupported` is
> now empty; per-format honest limits live below and in `formats.notes`).

## Quick start (agents: copy/paste)

```bash
# Build (C++20, CMake 3.20+, no third-party dependencies)
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build

# Run the full test suite (303 cases, 27 binaries)
ctest --test-dir build --parallel 4 --output-on-failure

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

# One real PCB workflow: Specctra DSN in, SES out, independent verify
./build/router route fixtures/golden_demo.dsn --json --seed 42 \
  --output routed.ses --report report.json
./build/router verify fixtures/golden_demo.dsn --routes routed.ses --json

# Route with electrical sidecar intent (DSN/KiCad carry no current intent)
./build/router route fixtures/dense_mcu_4layer.json \
  --config fixtures/rules_sidecar_nets.json --json --seed 42 \
  --output routed.json --report report.json

# Explain an incomplete route without a GUI (stable net/terminal/blocker IDs)
./build/router explain-failure report.json --json

# Verify committed copper independently of the router's search state
./build/router verify routed.json --json

# Analyze difficulty, density, bottlenecks
./build/router analyze board.json --json

# Plan fine-pitch escapes (centre-out, K-best, JSON diagnostics)
./build/router escape fixtures/bga_4x4.json --json
./build/router escape fixtures/bga_8x8.json --json --report escape.json
# rc 4 + "INCOMPLETE" when any pad carries an infeasibility record

# Ingest a KiCad board directly, route it, write it back, verify it
./build/router analyze fixtures/minimal.kicad_pcb --json
./build/router route  my_board.kicad_pcb --json --output routed.kicad_pcb
./build/router verify routed.kicad_pcb --json

# Gerber / IPC-2581 ingest (honest subsets, see Board formats)
./build/router analyze fixtures/gerber_copper.gbr --json
./build/router analyze fixtures/ipc2581_demo.xml --json

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

Future commands: none — all of `analyze/route/verify/escape/benchmark/
explain-failure/capabilities` are implemented. `escape` returns 4
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
3. `ampacity` — IPC-2221 §6.2 external/internal inversion
   (`I = k·dT^0.44·A^0.725`, `k = 0.048` external / `0.024` internal,
   `A = w·t`, `t = weight_oz·1.37 mil`), clamped to `[min_mm, max_mm]`
4. `board_default` — `defaults.trace_width_mm`

The legacy linear `ipc_estimate` (`width = I · k`) is kept for backward
compatibility and is selected only when a sidecar config carries an `"ipc"`
block without an `"ampacity"` block. The ampacity model only scales *stated*
net current; nets without current metadata fall back to the board default
and are reported (`default_current_used`). Thermal inputs resolve per layer
as `layer copper_weight_oz` > `--config ampacity` > `defaults`, and
temperature rise as `--config ampacity` > `defaults`; middle layers of a
3+-layer stackup use the internal (derated) constant unless `is_internal`
is set explicitly. This is an IPC-2221-style estimate, not an IPC-2152
qualified rating.

Clearance between two nets resolves from **both** nets (pair rule >
class pair > voltage-difference table > board default), plus conservative
per-net floors (KiCad net-class semantics: max wins). The model lives in
`CurrentCapacityModel` / `VoltageClearanceModel` behind `RuleResolver` —
never inside A*.

Sidecar `--config` (JSON) example — see `fixtures/rules_demo.json`.
A `"nets"` object adds current/voltage intent for formats that lack it
(DSN, KiCad) — see `fixtures/rules_sidecar_nets.json`. It extends, never
replaces, the blocks below (unknown net names are hard errors):

```json
{
  "ampacity": {"model": "ipc2221", "temp_rise_c": 20.0, "copper_weight_oz": 1.0,
               "min_mm": 0.15, "max_mm": 10.0},
  "voltage_table": [
    {"delta_v_min": 0, "clearance_mm": 0.15},
    {"delta_v_min": 50, "clearance_mm": 1.0}
  ],
  "class_pairs": [{"a": "HV", "b": "LOGIC", "clearance_mm": 1.5}],
  "pair_rules": [{"a": "MAINS_L", "b": "SELV", "clearance_mm": 3.0}],
  "width_classes": [{"name": "SIGNAL", "min_width_mm": 0.15}],
  "via_classes": [{"name": "POWER", "outer_mm": 1.2, "hole_mm": 0.6, "max_current_a": 8.0}],
  "nets": {
    "VBAT": {"current_a": 8.0, "voltage_v": 16.8, "trace_width_min_mm": 1.2,
             "class": "HIGH_CURRENT", "via_class": "POWER"},
    "HV_SW": {"voltage_v": 48.0, "voltage_class": "HV", "clearance_mm": 0.5}
  }
}
```

## Board formats

| Format | Status |
| ------ | ------ |
| Native JSON (`copperline/board/1`) | full round-trip (input + `--output`) |
| Specctra DSN (documented subset) | **import**: unit/structure/boundary/vias/rules, library+placement pads, network nets+pins+classes, planes + `copper_pour` as plane zones, keepout shapes; `--output *.ses` exports the session |
| Specctra SES | **export** from `route --output routed.ses`; **import** via `verify --routes routed.ses` |
| KiCad `.kicad_pcb` | **ingest + export**: outline, copper layers, nets + net classes (width/clearance/via rules), footprints/pads, tracks, vias, keepout rule areas, copper zones as plane zones; `route --output routed.kicad_pcb` re-verifies clean |
| Gerber RS-274X (documented subset) | **import**: apertures D10+ (C/R/O/P + circle/rect macro subset), draws/flashes/regions, dark/clear polarity, inch/mm, leading/trailing-zero + absolute/incremental modes; copper lands on one net (`COPPER`) + obstacles; outline from profile layer when present |
| IPC-2581C XML (flat subset) | **import**: `Datum`, `Layer`s, `Net`s, `Component`/`Pad`s, `Trace`/`Via`s, `Profile`, `NetClass` widths/clearances, net names/classes preserved |

KiCad phase-1 approximations (all surfaced in `import_warnings`): pad shapes
reduce to rotated bounding boxes; thru-hole pads fan to one terminal per
copper layer; non-Manhattan tracks import as-is (verify-only). Copper zones
import as `PlaneZone` entries (owning net + layer + polygon, own island id
each — no silent stitching across zones; overlaps bridge geometrically);
zones without a resolvable net/layer/polygon are skipped with an explicit
warning, never silently. `board_to_kicad_pcb` writes the board back
losslessly for everything modeled (pads re-fan identically, zones keep
polygons, net classes keep width/clearance rules); import → export →
re-import preserves nets/pads/tracks/vias/zones. Board-wide keepouts
expand to one keepout zone per copper layer on re-import (same
enforcement); component-less terminals group into virtual footprints
(geometry exact, refs synthesized).

DSN subset notes: coordinates carry the file `(unit ...)`; pads resolve via
`library (image ...)` + `placement`; net classes map to min-width /
min-clearance / via-class; `(plane ...)` and `(copper_pour ...)` map to a
`PlaneZone` (own island each) when net + layer resolve, otherwise skipped
with an explicit warning (never silent). Pour shapes: `polygon` exact,
`rect` exact, `circle` as octagon (warned), `path` closed into a polygon
(warned), `(window ...)` cutouts ignored (warned); polygons over 16384
vertices are skipped with a warning. `(wire|via|place|bend|elongate)_keepout`
shapes become rect keepouts (polygons by bbox, warned). `(wiring ...)`
pre-routes are re-routed from pads (warned).

Gerber honest limits: no embedded netlist exists in RS-274X, so ALL dark
copper lands on a single net (`COPPER`, id 0) — flashes become pads, draws
become traces (width = min aperture extent), dark regions become routable
pours (own island each); clear-polarity features become rect keepouts
(regions by bbox, warned). Polygon apertures reduce to bounding boxes;
macro apertures honor only circle/rect primitives (bbox, warned); arcs are
linearized into ≤256 chords (warned); step-repeat is replicated (bounded,
warned when clamped). Non-copper `FileFunction`s parse as copper with a
warning; inner copper layers collapse onto Bottom (2-layer board model,
warned); the board is always Top+Bottom signal layers. Without a profile
(`FileFunction,Profile`, profile-ish filename, or `G04 COPPERLINE:PROFILE`)
the outline is the copper bbox + 1mm margin (warned). Streaming 64KB parse;
region vertices capped at 32768 (truncated, warned); primitives capped at
2M (hard error).

IPC-2581C honest limits: flat-subset only — `Datum` (else profile bbox,
else copper bbox + 1mm, warned), layers (absent ⇒ Top/Bottom), nets (else
derived from pad refs, warned), component pads (unknown net ⇒ warn+skip,
unknown layer ⇒ default Top + warn), traces/vias (same policy),
`NetClass` width/clearance floors, profile polygons/rects/points (capped at
32768 vertices, truncated + warned). Tag/attribute matching is
case-insensitive. NOT modeled (warned once): stackup dielectrics, padstack
libraries, embedded parts, hierarchy, DFM/BOM, microvias, multi-step panels
(first `<Step>` only). Streaming 64KB scan, no DOM; primitives capped at
2M (hard error).

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
                dsn.h  gerber.h  ipc2581.h  optimizer.h  engine.h
                verifier.h  analyze.h
src             json/sexpr/board/kicad/kicad_export/dsn/gerber/ipc2581/...
                rules/spatial_index/density/...
                route_tree/sparse_graph/astar/escape/parallel/recovery/...
                optimizer/engine/verifier/analyze/main(CLI)
tests           27 binaries (no third-party framework)
fixtures        6 Prompt-1/3 JSON boards (open_2layer, obstacle_detour,
                high_current, voltage_clearance, narrow_channel, ...)
                + 9 fine-pitch golden boards
                (bga_4x4, bga_8x8, bga_8x8_via, irregular_array, dense_qfn,
                high_current_bga, mixed_voltage_bga, greedy_outside_first,
                impossible_escape) + forced_ripup (Prompt 4)
                + multi_terminal_tree, dense_mcu_4layer, golden_demo.dsn,
                rules_sidecar_nets.json (Prompt 5) + rules sidecar
                + KiCad samples (minimal, pour_zone) + violation board
                + pour_test.dsn, gerber_copper.gbr, gerber_profile.gko,
                ipc2581_demo.xml (format completion)
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

## Cleanup optimizer (Prompt 5)

After COMPLETE + an independent verifier pass only (`--no-optimizer`
disables), transactional passes run in deterministic order: collinear merge,
bend removal, via elimination, preferred-layer. Each candidate re-verifies
the whole board and reverts on any connectivity/legality/electrical harm
(length-tuned and pair-coupled nets are structurally untouched). The report
carries `optimizer` (`copperline/optimizer-report/1`: applied/reverted,
bends/vias/length deltas). Stats and `board_hash` refresh after, with a
second verifier gate.

## Failure reports (Prompt 5)

Every route failure carries stable agent IDs: net, `terminal_a/b`,
`src_component/src_pin`, `dst_component/dst_pin`, electrical constraints,
`centre_depth`, `pin_density_per_mm2`, `candidate_count`, `top_blockers`,
`modes_attempted`, `attempted_layers`, `attempted_via_classes`,
`best_partial`, per-connection `category`
(`SEARCH_BUDGET_EXHAUSTED_WITH_UNROUTED_CONNECTIONS` |
`UNROUTABLE_UNDER_CONFIGURED_CONSTRAINTS_AND_BUDGET`) plus hotspots from
the report. `router explain-failure report.json --json` renders schema
`copperline/failure-explanation/1` with a per-reason suggestion — no GUI.

## Golden suite (Prompt 5)

| Fixture | Result | Verifier |
| ------- | ------ | -------- |
| open_2layer | COMPLETE 4/4 | ok |
| obstacle_detour | COMPLETE 4/4 | ok |
| high_current (+rules_demo) | COMPLETE | ok |
| voltage_clearance | COMPLETE 4/4 | ok |
| narrow_channel | COMPLETE 4/4 | ok |
| bga_4x4 | COMPLETE 32/32 | ok |
| multi_terminal_tree | COMPLETE 6/6 | ok |
| forced_ripup | COMPLETE | ok |
| dense_mcu_4layer (+sidecar nets) | COMPLETE 16/16 | ok |
| golden_demo.dsn → routed.ses | COMPLETE 4/4 | ok |
| blocked_impossible | INCOMPLETE (must NOT succeed) | n/a |

`tests/test_golden.cpp` enforces this end-to-end (engine + independent
verifier gate), plus determinism (threads 1 vs 4 identical `board_hash`),
optimizer monotonicity and the DSN→SES round-trip.

## Determinism & benchmarks (Prompt 5)

Identical `(board, rules, workers, seed, version)` → identical
committed-geometry hashes; batch membership is thread-independent so worker
timing never affects output. Evidence: `dense_mcu_4layer` routes to hash
`e6f6e653757cf635` at both `--threads 1` and `--threads 4`.

Measured (Release, Ryzen 7 7735HS, 16 logical CPUs, seed 42):

| Board | 1 thread | 4 threads | 8 threads | Geometry |
| ----- | -------- | --------- | --------- | -------- |
| dense_mcu_4layer (8 tasks) | 89 ms wall | 44 ms (2.0x) | 40 ms (2.2x) | identical |
| obstacle_detour (2 tasks) | 7 ms | 9 ms (0.8x) | — | identical |

Small boards are overhead-dominated (honestly <1x); the 8-task board shows
a real ~2x at 4 threads with bit-identical copper. `router benchmark`
reports wall/engine time, expansions, accepted/rejected, epochs, rip-up
generations, connected %, vias, length, optimizer counts and speedup.

## Known limitations (v0.1.0)

- Impossible boards report `INCOMPLETE`/`BUDGET_EXHAUSTED` with blocker
  hints and per-connection categories — never false success.
- Escape is complete on 4x4/QFN/irregular fixtures; ultra-dense 8x8
  (0.8 mm pitch, zero same-layer channels) escapes 48/64 with explicit
  infeasibility records for the rest.
- Copper pours: native JSON `planes[]`, DSN `(plane ...)` /
  `(copper_pour ...)` and KiCad copper zones all route plane-aware via the
  issue-#16 machinery (own-net targets, foreign-pour obstacles at exact
  polygon clearance); unresolvable copper warns explicitly, never silently.
- MSVC port still assumes `__int128` (GCC/Clang exact integer clearance
  math).
- Threads: `--threads 0` (default) = all CPUs; total stays within the
  2048 MB router budget via bounded batch widths/candidates
  (`memory_budget_mb`, `optimizer_max_candidates` in report params).
