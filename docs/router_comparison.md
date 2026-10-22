# Copperline ↔ Electra ↔ Altium/Specctra — three-way routing comparison

Compares copperline's implemented router (v0.1.0) against the two
commercial autorouting systems reverse-engineered in this repo:

- **Electra** (DipTrace's bundled shape router) — `docs/reverse/diptrace/autorouter_modules.md`
- **Altium/Specctra** (CircuitMaker: Situs topological + Routecct external) — `docs/reverse/circuitmaker/autorouter_modules.md`

Legend: ✅ implemented · 🟡 partial / needs work · ❌ absent · 🆕 **should add** (high value) · 💡 **could add** (nice to have)

---

## 1. What we already have (✅ — no action)

| Capability | copperline module | Electra | Situs/Routecct |
|---|---|---|---|
| DSN import | `DsnImporter` (src/dsn.cpp) | engine-parses DSN itself | Tds_File codec |
| SES export / net-matched merge | `board_to_ses`, `ses_merge_into` | same dialect | Tds_File + FreeRouting dialect |
| Sparse/vector graph search (not raster) | `sparse_graph` + `astar` | shape-based expansion | Situs: topological (any-angle) |
| Net classes, width, clearance rules | `rules.h` RuleResolver | netclass + rule pairs | scope hierarchy (12 priorities) |
| Via classes & current-rated vias | `via_bundle` + rules | via styles + via policy knobs | via types (blind/buried/micro) |
| Costed search (length/bends/vias/layer) | `astar` costs | `cost <= total_cost` | strategy-pass costs |
| Fanout / fine-pitch escape (centre-out) | `escape.h` | fanout passes (no centre-out) | BGA fanout (dog-bone, blind-via) |
| Rip-up / recovery | `recovery.h` (dependency graph, transposition, PV, history) | protection + filter passes | push-and-shove + rip-up-after |
| Post-route cleanup (transactional, verify+revert) | `optimizer` + `simplify` + `verifier` | init/final clean | Gloss (straighten/recorner/spread/hug) |
| Differential pairs | `diffpair` + `tuning` | DSN diff-pair nets | Differential Pairs Routing rule |
| Planes/pours/keepouts as obstacles | `board` PlaneZone/Keepout + `spatial_index` | shapes + keepout types | plane pours + keepout regions |
| Deterministic parallel epochs | `parallel` (thread-invariant) | — (single-threaded) | — (single-threaded) |
| Independent verifier gate | `verifier` | DRC object-pair checks | DRC tie-in |

**Bottom line:** copperline is functionally complete against both
commercial routers on the *core* routing problem, and strictly ahead on
determinism, parallel scaling, fine-pitch centre-out escape, meta-search
recovery and verified transactionality.

---

## 2. What we're missing (the diff)

### 🆕 A. Interop correctness — real-world files (SHOULD ADD, highest ROI)

These are *cheap, high-value* fixes that unlock real DipTrace/CircuitMaker/
FreeRouting files; they are bugs of completeness in `src/dsn.cpp`:

| # | Gap | Source finding | Effort | Value |
|---|---|---|---|---|
| A1 | **`(resolution <unit> <digits>)` scaling** — copperline only reads `(unit …)`; real files (Altium mil-10, KiCad um-10, FreeRouting um-10) carry `(resolution …)` and coordinates must scale by `unit*10^-digits` | Altium M1 (confirmed gap), Electra E1 | S (~1 day + tests) | Unlocks all real DSN files |
| A2 | **Padstack `(shapes (circle|rect|octagon|polygon …))`** — copperline treats pads as squares; Altium exports shape lists for non-circular pads | Altium M1/M10 | S–M | DRC-correct import |
| A3 | **Quoted-name normalization** — `"Top Layer"` vs `TOP` vs `TopLayer` must unify; `(parser (string_quote ") (space_in_quoted_tokens on))` must not fatal | Altium M1 | S | Real files parse |
| A4 | **Polygon board boundary** — accept `(boundary (polygon …))`, not only rect | Altium M1 | S | Non-rect boards |
| A5 | **FreeRouting SES dialect** — `network_out`/`library_out` nesting, padstack-name resolution from `library_out` | Altium M2 | S–M | Use FreeRouting as oracle |
| A6 | **Per-layer `(direction)`/`(grid)`** in structure — Electra emits these; copperline imports layers but not direction/grid constraints | Electra E1/E4 | S | Layer-direction-aware routing |

### 🆕 B. Routing capabilities (SHOULD ADD — agent-visible wins)

| # | Capability | Where it comes from | copperline add | Effort |
|---|---|---|---|---|
| B1 | **Strategy files** — declarative pass-ordered strategy (Electra pass families / Situs strategies: Via Miser, 2-Layer, Multi-Layer, Orthogonal). Agents configure routing *policy* not just rules. | Situs M8, Electra E6 | `Strategy` struct + `run_strategy` + JSON strategy format; defaults shipped as fixtures | M |
| B2 | **Memory routing pass** — route same-net pins sharing X/Y in "U" patterns (classic memory) | Situs M7 | `memory_route` pass on connectivity model | S–M |
| B3 | **Pad-entry cleanup** — reroute pad exits along the pad's long axis | Situs M7 | geometry sweep pass in optimizer | S |
| B4 | **Straighten/Recorner passes** — H/V/45 corner-probe shortening + mitering (copperline `simplify` partially covers) | Situs M7, Electra E9 | explicit passes over optimizer | S |
| B5 | **Via-share / via-at-SMD-grid policy** — Electra's `via_share`, `sharing_via`, `via_at_smd_grid`, `via_testpoint`; affects density-constrained fanout | Electra E3/E4 | rules knobs + via placement policy | M |

### 💡 C. Research-grade (COULD ADD — new algorithm, measure first)

| # | Capability | Source | Notes | Effort |
|---|---|---|---|---|
| C1 | **Topological mapping** — constrained triangulation of free space ("spiders' web"), any-angle routing, fewer vias on staggered/dense boards | Situs M5/M6 | New `TopoMap` module over `spatial_index`; benchmark vs current on dense BGA fixtures before committing | L (experimental) |
| C2 | **Thermal-relief spokes** for plane-connected pads/vias (CONNECTSTYLE relief/solid, AIRGAP, spoke count) | Situs M10, formats.md | needed for power boards; affects obstacle model + DRC | M |
| C3 | **Gloss "Hug"/"Spread"** passes — pack routes to open channels / equalize spacing | Situs M7/M12 | post-optimizer polish | M |
| C4 | **`--route-external` wrapper** — write DO-file, launch FreeRouting as an oracle/integration test | Altium M3/M4 | not routing code; CI/validation harness | S |

### ❌ D. Deliberately NOT adding (with justification)

| Item | Why not |
|---|---|
| Grid router engine (Electra's 2nd, DipTrace's simple router) | copperline's sparse-graph A* is a strict superset; rip-up-and-retry already in `recovery` |
| COM/OLE host transport (RouteService/RouteLayer) | copperline is headless CLI; files are the contract |
| DRC-pair post-check (Electra style) | copperline does exact-integer structural legality — stronger |
| Interactive Walk-Around/Push GUI | out of scope (headless) |
| Situs.INI/padinit.dat exact formats | only needed for bit-exact round-trips of AD boards |

---

## 3. Recommended implementation order (1–6)

1. **A1–A4** (resolution, padstack shapes, quoted names, polygon boundary) —
   DSN interop correctness; ~2–3 days incl. tests; validate against
   KiCad-exported + FreeRouting-exported DSNs as oracles.
2. **A5** SES dialect tolerance → then wire FreeRouting as CI oracle.
3. **B1** Strategy file format + engine (agents get routing *policy*).
4. **B2+B3+B4** pass library on top of optimizer/connectivity (Memory,
   Pad-entry, Straighten, Recorner).
5. **B5** via-share / via-at-grid policy knobs.
6. **C1** experimental TopoMap benchmark (only if dense-BGA results warrant).

---

## 4. Files

- This file: `docs/router_comparison.md`
- Electra decomposition: `docs/reverse/diptrace/autorouter_modules.md`
- Altium decomposition: `docs/reverse/circuitmaker/autorouter_modules.md`
- Installer/format RE: `docs/reverse/{circuitmaker,diptrace}/README.md`, `formats.md`
