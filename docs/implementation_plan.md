# Copperline implementation plan — the long list

Source: `docs/router_comparison.md` (three-way diff vs Electra + Situs/
Routecct). Each item states the change, the file(s), acceptance criteria
and priority. Implement in priority order; every item must keep
`ctest` green (30 tests) and follow repo style (integer-nm, JSON
schemas, honest import warnings, deterministic output).

Legend: [interop] correctness with real files · [capability] new router
behaviour · [em] EM validation · [orch] CLI/orchestration · [🆕] net-new

## Status (updated 2026-10-26)

| Item | Status | Commit / note |
|---|---|---|
| 9, 12, 13 | done | `f091a27` layer-scoped rules, via policy, relief import |
| 10 | done | `b15bc05` min/pref/max width triplets |
| 15 strategy engine | done | `fd9db4d` `--strategy`, toggles + budgets |
| 16 memory pass | done | `b74ab2f` U-pattern, strategy-gated OFF by default |
| 17 adjacent memory | done | same-component/adjacent pins, 2-leg dogleg for offset pairs |
| 18 clean pad entries | done | asymmetric pads exit along the long axis |
| 19 straighten | done | `ce89080` orientation-independent corner probe + verify/revert |
| 20 recorner/miter | done | 45° chamfer on degree-2 axis-aligned corners (`orthogonal: true`) |
| 21 fanout pattern | done | escape-stage inline/staggered preference + direction control |
| 24 via-minimization | done | `0519043` Via Miser mode (via elimination only) |
| 22 Hug, 23 Spread | done | pass 8: shift a straight parallel run with perpendicular stubs; hug tightens to same-net spacing, spread equalises the two neighbour gaps; verify+revert, bounded shift |
| 28 setup-report | done | pre-route rules/layer/drill-pair/width/via JSON (`copperline/setup-report/1`) |
| 29 fanout | done | `router fanout --pattern --dir` commits escape fanout copper (`copperline/fanout-report/1`) |
| 30 gloss | done | `router gloss` runs requested polish passes with verify+revert and a JSON report |
| 31 route-external | done (guarded) | `.dsn`-only wrapper: DO file + external router + SES import; no DSN exporter exists, so JSON boards are rejected |
| 25–27 EM | partial | analytical tier validated; full-wave relief NOT validated |

All new passes are strategy-gated and OFF by default: a pristine build at
`ce89080` and the current build produce identical default `board_hash` on
all 16 real board fixtures (see `docs/validation/phase3_realboard.md`).

---

## Phase 1 — DSN/SES interop correctness (highest ROI, ~3 days)

1. **[interop] `(resolution <unit> <digits>)` scaling** — `src/dsn.cpp`
   `DsnImporter`. Parse `(resolution mil 10)` / `(resolution um 10)`,
   scale every numeric literal by `unit_size * 10^-digits`; keep
   `(unit …)` as fallback; both-present ⇒ resolution wins. Tests: scale
   a known-coordinate DSN (e.g. a 10 mm trace in um-10 → 10,000,000 nm).
2. **[interop] Padstack shapes** — parse `(shapes (circle …) (rect …)
   (octagon …) (polygon …))` on padstacks inside `(library (image …))`;
   drive Terminal/pad expansion from the shape (not square).
   Tests: DSN with a rounded-rect pad → pad geometry respected by
   verifier.
3. **[interop] Quoted-name normalization** — `"Top Layer"` == `TopLayer`
   == `TOP`; tokenizer must survive `(parser (string_quote ")
   (space_in_quoted_tokens on))` (quoted tokens may contain spaces).
4. **[interop] Polygon board boundary** — accept
   `(boundary (polygon pcb …))` (and `(rect …)`); non-rect outlines
   become the board outline. Currently rect-only.
5. **[interop] FreeRouting SES dialect** — read both classic
   `(route (network …))` and FreeRouting `network_out`/`library_out`
   nesting; resolve padstack names from `library_out`; validate against
   real FreeRouting output as an oracle then wire it into CI.
6. **[interop] Layer direction/grid** — import per-layer
   `(direction in|out|horizontal|vertical|45)` + `(grid on|off)` from
   `(structure)` and feed layer direction preference into route costs
   (soft, never hard unless documented).
7. **[interop] Keepout types** — `(wire_keepout)`, `(via_keepout)`,
   `(place_keepout)`, `(bend_keepout)`, `(elongate_keepout)` →
   rect keepouts with correct enforcement kind (warn on polygon → bbox).
8. **[interop] microvia / via_rule** — parse and record + warn (honest
   subset), never fatal.

## Phase 2 — Rules & capability additions

9.  **[capability] Layer-scoped net-class rules** — `DsnClassRule` gains
    layer-scope variants (InNetClass+OnLayer) and the 12-priority
    scope flattening from Situs M9; resolve by Specctra priority order.
10. **[capability] min/pref/max width triplets** — rule uses Preferred,
    falls back to Min; expose chosen source in reports (agent-facing).
11. **[capability] Via-type sets per net class** —
    `(via_rule)`/allowed-vias set per class; preferred via style
    fallback; via legality from the set.
12. **[capability] Via-share policy knobs** —
    `via_share`, `sharing_via`, `via_at_smd`, `via_at_smd_grid`,
    `via_at_smd_fit`, `via_testpoint`, `smd_to_via_gap`,
    `buried_via_gap`, `via_to_via_gap` — rules-level via placement
    policy (shared vias between same-net pins, via-on-grid spacing).
13. **[capability] Thermal-relief / PolygonConnect rule import** —
    CONNECTSTYLE (relief|solid|none), AIRGAPWIDTH,
    RELIEFCONDUCTORWIDTH, RELIEFENTRIES (spoke count) from
    Altium `.PcbDoc` rules (formats.md Rules6 `PolygonConnect`) and
    DSN rule fragments; plane-connected pad/via objects honor spoke
    geometry for current/clearance.
14. **[capability] Differential-pair gap/width params** —
    extend `diffpair`/`tuning` to read gap + width per pair from
    rule tables (DSN `(differential_pair …)`, KiCad net-class).

## Phase 3 — Pass library & strategy engine

15. **[🆕] Strategy format + engine** — JSON strategy file
    (`copperline/strategy/1`): ordered passes with params; defaults
    shipped as fixtures: `default_2layer`, `default_multilayer`,
    `via_miser`, `general_orthogonal`; `router route --strategy x`.
    Engine: iterate passes in order over the committed board.
16. **[🆕] Memory pass** — route same-net pins sharing X/Y in the
    classic "memory" U pattern (adjacent-pair fanout).
17. **[🆕] Adjacent Memory pass** — U-pattern fanout for adjacent
    same-net pins.
18. **[🆕] Clean Pad Entries pass** — reroute each pad's exit along its
    longest axis (asymmetric-pad cleanliness).
19. **[🆕] Straighten pass** — corner-probe (H/V/45) shortening; keep
    only if it reduces length; gate on verify+revert (reuse optimizer
    machinery).
20. **[🆕] Recorner/miter pass** — miter 90° corners when strategy has
    `orthogonal: true`.
21. **[🆕] Fanout pattern selection** — inline/staggered arrangement per
    Fanout Control rule (pattern check + clearance + width + via
    style); direction control (`cbFanoutDir` analogue).
22. **[🆕] Hug pass** — reroute connections hugging existing copper at
    minimum clearance to free channels (post-route polish).
23. **[🆕] Spread pass** — equalize spacing between fixed objects.
24. **[🆕] Via-minimization pass** — batch via removal on legal nets
    (extend optimizer's via elimination into a strategy pass); the
    "Via Miser" behaviour.

## Phase 4 — EM validation (thermal relief)

25. **[em] Thermal-relief spoke model** — `em/rf_models.py`: annular-gap
    + N-spoke connection model → equivalent resistance of a
    plane-connected pad vs solid connection; ampacity derating
    (I_derated = I * sqrt(R_solid/R_total), documented first-order);
    spoke parameter validation.
26. **[em] Solid-vs-spoke openEMS driver** — `em/gen_openems.py` gains a
    `--thermal-relief` mode generating two FDTD models (solid pad-to-
    plane vs 4-spoke relief) for the same net; extract S11/S21 +
    resistance proxy; compare.
27. **[em] validate_rf thermal flag** — report per-net spoke geometry,
    relief loss (dB/Ω), derating factor, and the solid-vs-spoke verdict
    vs target current. Schema `copperline/em-report/1` extended.

## Phase 5 — Orchestration & agent surfaces

28. **[orch] `router setup-report`** — pre-route analysis: rules
    collected, layer directions, drill pairs, width/via resolution per
    net, warnings (layers set to any, unconnected nets). Agent-facing
    JSON (schema `copperline/setup-report/1`).
29. **[orch] `router fanout`** — expose SMD fanout as a subcommand
    (uses escape engine with fanout pattern selection).
30. **[orch] `router gloss`** — run post-route polish passes
    (straighten/recorner/hug/spread/via-min) with per-pass
    verify+revert and JSON report.
31. **[orch] `--route-external <router>`** — optional wrapper writing a
    DO-file and invoking an external Specctra-compatible router
    (FreeRouting default) as an oracle; CI integration after #5.

---

## Merge gate (all phases)

- `ctest --test-dir build --parallel 4` = 30/30 (new tests added per item)
- Determinism: threads 1/4 identical `board_hash` where routing behaviour
  changed.
- New JSON schemas versioned `copperline/<name>/1`; stdout pure JSON in
  `--json`.
- No GUI. No third-party runtime deps in the C++ core (EM/Python stays
  in `em/`).