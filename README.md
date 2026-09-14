# Copperline — headless PCB autorouter for agents

Copperline is a **CLI-only, headless** PCB autorouter. There is no GUI, no
viewer, no desktop shell. The consumers are AI agents, scripts, CI systems and
EDA pipelines: every important operation has a stable machine-readable JSON
representation, documented nonzero exit codes, and deterministic output for a
given `(board, rules, seed)`.

> Status: **Prompt 1 foundation** (`0.1.0`) — core geometry, electrical rules,
> CLI contract and single-threaded router. Fine-pitch escape (P2), parallel
> routing (P3), rip-up/meta-search (P4) and adapters/optimizer/release (P5)
> are explicitly planned and reported as such by `router capabilities`.

## Quick start (agents: copy/paste)

```bash
# Build (C++20, CMake 3.20+, no third-party dependencies)
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build

# Run the full test suite (42 cases, 7 binaries)
ctest --test-dir build --output-on-failure

# Route a board, machine-readable
./build/router route fixtures/open_2layer.json --json --seed 42 \
  --output routed.json --report report.json

# Route with electrical sidecar rules
./build/router route fixtures/high_current.json --config fixtures/rules_demo.json \
  --threads 16 --seed 42 --output routed.json --report report.json --json

# Verify committed copper independently of the router's search state
./build/router verify routed.json --json

# Analyze difficulty, density, bottlenecks
./build/router analyze board.json --json

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

Future commands (`escape`, `benchmark`, `explain-failure`) exit 2 with
`"code": "not_implemented"` until their prompt lands.

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
                sparse_graph.h  astar.h  engine.h  verifier.h  analyze.h
src             json/sexpr/board/kicad/rules/spatial_index/density/...
                route_tree/sparse_graph/astar/engine/verifier/analyze/main(CLI)
tests           7 binaries, 42 cases (no third-party framework)
fixtures        5 JSON boards + rules sidecar + KiCad sample + violation board
```

Global acceptance is lexicographic (connectivity > hard violations >
resource overuse > electrical geometry > vias > length > bends >
congestion) and enforced by the engine/verifier split.

## Known limitations (Prompt 1)

- Single-threaded engine (`--threads N>1` is accepted, logged, and runs 1).
- No fine-pitch escape stage, no rip-up/reroute, no optimizer — greedy
  sequential A*; impossible boards report `INCOMPLETE` with blocker hints.
- No DSN/SES export yet; routed output is native JSON (Prompt 5).
- `router escape|benchmark|explain-failure` are stubs by design.
