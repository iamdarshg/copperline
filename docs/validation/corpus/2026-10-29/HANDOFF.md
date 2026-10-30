# Handoff - make the corpus campaign pass (COMPLETE, not just legal)

Status: **two gates, one of them red on purpose.** The boards in
`tests/corpus_manifest.json` are covered by two ctest tests (label `campaign`):

```
ctest -R test_corpus_legal       # PASS: 4/4 boards legal, 0 violations
ctest -R test_corpus_complete    # FAIL: 0/4 COMPLETE - this is the work list
ctest -LE campaign               # everything else, 33/33 green
```

The legality gate is the hard safety property (no board may ever commit illegal
copper) and is green today. The completion gate requires every board to be fully
routed and is red; it is the thing this document is about. Both run the same
boards, and `--cache` lets the second reuse the first's reports.

## Definition of done

1. `python tests/test_corpus_boards.py --gate complete --tier short` -> **4/4 COMPLETE**.
2. `python tests/test_corpus_boards.py --gate complete --tier long` -> **10/10 COMPLETE**.
3. The legality gate stays green throughout (`--gate legal`): every board legal,
   no tolerance. Non-negotiable: no loosening of the verifier, the clearance
   rules, or the gates to buy connectivity.
4. The recorded floors and escape-infeasible caps still hold or improve.
5. `ctest` is green with the campaign included (35/35).

Do **not** reach green by: deleting boards from the manifest, raising the budget
until it hides the problem, `WILL_FAIL`, lowering the `infeasible` caps, or
recording floors (`--record` only refreshes the progress ratchet; the COMPLETE
gate is independent of it).

## The gate budget

All boards run at **12 epochs / 200k search nodes** (`tests/corpus_manifest.json`
`budget`), with `--timeout 1800` as a pathological-run guard only. Budgets are
work based, so the copper is identical across machines and thread counts (both
verified). If a board needs a bigger budget, that is a real decision to record in
the manifest - not a way to make the gate pass.

The tier split exists because `tinytapeout-demo` (1611 terminals) and
`OpenJBOD-RP2040` (852) spend minutes just escaping and building the graph:
`short` = the four cheap boards (ctest, ~2 min), `long` = all ten (CI, ~25 min).

## Measured state at the gate budget

From `gate/summary.json` (full report for the cheapest board in
`gate/rp2040_game_con.rep.json`):

| board | connected | tasks routed | failed task-routes | of which no candidate | searched+blocked | gap 0 |
|---|---|---|---|---|---|---|
| `pico-radio-9-kotoldalas` | 217/340 | 177/238 | 61 | **56** | 5 | 3 |
| `KiCAD_Board_3` | 94/198 | 85/138 | 53 | **44** | 9 | 3 |
| `rp2040_nema` | 94/184 | 58/110 | 52 | **47** | 5 | 2 |
| `rp2040_game_con` | 79/152 | 49/85 | 36 | **34** | 2 | 0 |

Long tier (same budget, from the last CI-shaped run): `interf_u` 398/684,
`tinytapeout` 171/1611, `Pico 2 C` 111/327, `RPI-PICO` 89/282, `Pico WH` 238/517,
`OpenJBOD` 364/852. All legal.

Across the 9 incomplete boards at a 180 s wall clock (187 real search failures):
**95 (51 %) produced no candidate**, 35 far (>2 mm), 32 mid (0.5-2 mm),
13 near (<=0.5 mm), 12 reached the goal node (`frontier_gap_mm=0.0000`) but did
not commit. Blockers: **93 % traces** (rippable copper), 6 % pads, 1 % vias.
Only **36 terminals in the whole corpus (1.7 %)** sit on pads with no legal
escape at all.

## Workstream 1 - "no candidate produced" (the dominant cause)

At the gate budget this is 86-92 % of all failed task-routes; at 180 s it is
51 %. The engine records `best_partial = "unattempted: no worker candidate
produced"` when the worker returns no `CandidateRoute` for a scheduled task
(`src/engine.cpp` failure attribution), i.e. the task ran, A*/portfolio/
hierarchical produced **nothing**.

Start with `rp2040_game_con`: 34 of its 36 failures are this, on a 152-terminal
board that takes ~27 s to run, and it needs only 36 task-routes.

```
python tests/test_corpus_boards.py --tier short --only rp2040_game_con \
  --keep-reports /tmp/gate --router build/router.exe
```

Where to look:
* `src/parallel.cpp` - worker entry for a task (`route_task`/`route_one`), the
  portfolio/`route_k` loop, `GuidanceResult` and the hierarchical window.
* `src/astar.cpp` - the search only returns a path when it reaches the target
  copper; an empty result is indistinguishable from "no path" today.
* `src/route_tree.cpp` - how a task's source/destination copper targets are
  resolved.

Hypotheses to test, cheapest first:
1. **Task attempted before its target copper exists.** The net's other pad has
   nothing committed yet, so there is no copper target to reach and the search
   returns empty. The task is then counted as a failure and only retried a few
   times. Check whether these tasks ever get a `last_attempt` entry with a real
   `closest_goal_dist_nm`; if not, they never really searched.
2. **Endpoint prefilter.** Dense QFN pads may fail the width/clearance or
   corridor prefilter before A* starts (the pad cannot even be departed).
3. **Hierarchical window.** `hier_window_attempts` / `hier_tube_half_mm` may
   return an empty window for pads inside a dense cluster.

Instrument the worker to log *why* the candidate list is empty (no source node /
no target node / prefilter / window / budget) and the count will collapse into a
specific bug. Acceptance: no-candidate count 0 on `rp2040_game_con`.

## Workstream 2 - rip-up rips but cannot re-place the victim

`recovery.ripups = 0` on every incomplete board, even when `EXHAUSTIVE_LOCAL`
runs (it does now, after the escalation fix). 93 % of attributed failures name a
**trace** as the blocker, i.e. movable copper, so rip-up is the right tool - but
no branch survives.

Why: `src/engine.cpp` only commits a recovery branch when the failed task is
newly done **and** every ripped victim is re-routed (`failed_done &&
fully_rerouted`). `src/recovery.cpp::reroute_branch` re-routes the victims with
the same A* config that just failed them, in the same congested region.

Ideas, in order of expected value:
* Negotiated congestion: before re-routing the victim, add history cost in the
  ripped region so the *failed* task's corridor becomes cheap and the victim
  detours around it (`congestion.add_history_rect` already exists).
* Rip a **segment** of the blocking route near the conflict instead of the whole
  route, so the victim is a short stub that can be re-placed.
* Let the victim change layer / via class (the branch currently inherits the
  parent's mode config).
* Only then widen breadth (`ripup_breadth`) - more blocking traces at once.

Constraint: a branch that strands copper must never commit. Connectivity is
checked by the verifier, not by bookkeeping.

## Workstream 3 - throughput on the three giants

`interf_u` (684 terminals), `tinytapeout-demo` (1611), `OpenJBOD-RP2040` (852)
produce **zero** real search failures - all their records are timeout
bookkeeping, because the run never reaches the failing tasks. `tinytapeout`
spends ~50 s in epoch 0 alone.

Where: `src/sparse_graph.cpp` graph construction and the per-epoch rebuild in
`src/engine.cpp`. Profiling on `sonde` showed 71 s of 120 s in graph edge
construction (11.3 M edges over 785 rebuilds).

Fix: cache the graph per board revision (invalidate on commit), or rebuild only
the nets touched by the last batch. This is the difference between "needs a
bigger budget" and "completes".

## Workstream 4 - `frontier_gap_mm=0.0000` (12 failures at 180 s)

The search reaches the target node and still nothing commits. That is a commit
path question, not a search question: candidate produced -> epoch gate ->
`has_local_violation`. Reproduce with stage provenance:

```
COPPERLINE_TRACE_PROVENANCE=1 build/router.exe route <board> --json \
  --no-board-echo --seed 42 --threads 4 --max-epochs 12 --max-search-node 200000 \
  --report /tmp/r.json
```

Look for `PROV gate_escape ... rejected=1` or `epoch_illegal` rejections on those
nets. If the gate is right, the search's endpoint is landing inside foreign
copper and the endpoint adjustment is wrong.

## Workstream 5 - geometry (last, smallest)

36 terminals corpus-wide: `Pico 2 C` 26/211 pads unescapable, `RPI-PICO` 17/181,
`tinytapeout` 7/471, `OpenJBOD` 6/449 (`escape.unresolved_reasons` ->
`no_exit_sectors`). These are 0 mm-clearance / 0.5 mm-pitch boards on 2 layers.

Where: `src/escape.cpp` (exit sector search), via style selection (the smallest
via on the board is already adopted as the default, see `src/kicad.cpp`).

Options: via-in-pad or microvia styles, a narrower width class where the board's
rules allow it, or a 4-layer stackup. If a pad is provably unescapable at the
declared rules, that is a documented exception with the geometry in the
manifest - not a silent skip.

## Guard rails and conventions

* **The verifier is authoritative.** Every commit path is gated by
  `has_local_violation` (copper extents, zero-clearance safe). Never relax it.
* **Determinism.** Campaign budgets are work based; do not add wall-clock
  assertions, and keep copper identical across thread counts
  (`test_golden` already enforces hash equality 1 vs 4 threads).
* **Exit codes are a status channel**: 0 ok, 4 incomplete, 7 budget exhausted are
  legitimate; 2/3/5/6 are hard failures. The campaign treats 4/7 as results.
* **`stats.epochs` overshoots `--max-epochs`** (recovery generations are logged
  too), so never assert `epochs == max_epochs`.
* When a board completes: run `--tier long --record`, confirm the gate is still
  COMPLETE, and keep the board in the manifest as a completion regression.

## Where everything lives

```
tests/test_corpus_boards.py            the gate + harness
tests/corpus_manifest.json             boards, tiers, budget, ratchet floors
CMakeLists.txt                         ctest registration (label: campaign)
.github/workflows/corpus.yml           CI: short + long tiers
docs/validation/corpus/2026-10-29/
  campaign.md                          how the campaign works
  gate/summary.json                    measured failure buckets per board
  gate/rp2040_game_con.rep.json        full report for the cheapest board
  boards/bNNN_stripped.json            frozen pads-only inputs (no KiCad needed)
  reports/                             the older 180 s reports
docs/validation/corpus.md              benchmark story + root causes
```

Lineage: the zero-clearance gate fix, escape-candidate retry, second wind and
recovery escalation landed on 2026-10-29 (`7491c5f`, `e339347`, `05109c2`,
`45bc987`, `70c2d05`). This handoff is the work that remained after them.
