# Phase 3 — real-board validation (2026-10-26)

Purpose: verify that the new strategy-gated passes (memory, adjacent
memory, straighten, miter, clean pad entries, via miser, fanout pattern)
are **real** additions — they change geometry only when enabled, never
regress default routing, and their tests catch failures.

## Method

Two builds of the same board fixtures:

* **pristine** — worktree at `ce89080` (before items 17/18/20/21/24 work).
* **current** — same tree with the new passes.

Each board was routed with `router route <board> --json --seed 42
--no-board-echo` in three modes: default, `--strategy
fixtures/strategies/straighten_demo.json`, `--strategy
fixtures/strategies/via_miser.json`. Reports were parsed for
`status`, `verification.ok`, `board_hash`, and the optimizer/via metrics.

## Result 1 — default behaviour is unchanged

Default-mode `board_hash` matched the pristine build on **all 16 boards**
(12 JSON boards + DSN/KiCad imports). Status and verifier results were
identical on every board.

```
bga_8x8.json            hash_match=True  INCOMPLETE (pre-existing)
bga_8x8_via.json        hash_match=True  INCOMPLETE (pre-existing)
dense_qfn.json          hash_match=True  COMPLETE verify=True
high_current_bga.json   hash_match=True  COMPLETE verify=True
mixed_voltage_bga.json  hash_match=True  COMPLETE verify=True
irregular_array.json    hash_match=True  COMPLETE verify=True
greedy_outside_first    hash_match=True  VIOLATION  (pre-existing)
open_2layer.json        hash_match=True  COMPLETE verify=True
multi_terminal_tree     hash_match=True  COMPLETE verify=True
obstacle_detour.json    hash_match=True  COMPLETE verify=True
diffpair_basic.json     hash_match=True  COMPLETE verify=True
plane_power.json        hash_match=True  COMPLETE verify=True
golden_demo.dsn         hash_match=True  COMPLETE verify=True
pour_test.dsn           hash_match=True  COMPLETE verify=True
minimal.kicad_pcb       hash_match=True  COMPLETE verify=True
pour_zone.kicad_pcb     hash_match=True  COMPLETE verify=True
```

`bga_8x8*` is INCOMPLETE and `greedy_outside_first` is a VIOLATION in the
pristine build too: pre-existing router limitations, not caused by the new
passes. No board regressed.

## Result 2 — straighten does real work on real boards

| board | default length (mm) | straighten length (mm) | delta |
|---|---|---|---|
| mixed_voltage_bga | 103.768016 | 99.6659674 | **−4.102** |
| irregular_array | 93.1836242 | 92.456764 | **−0.727** |
| multi_terminal_tree | 88.69999 | 88.5641246 | **−0.136** |
| dense_qfn | 162.928632 | 162.859955 | **−0.069** |

No board increased in length. `straighten_ran` was true on every COMPLETE
board, so the pass runs broadly and applies where a legal shortening exists.

## Result 3 — Via Miser on real boards

`via_miser_ran` was true on every COMPLETE board, but **no vias were
removed** on these fixtures (`vias_after == vias_before`). The vias present
are load-bearing: their far ends land on pads whose layer differs from the
joined layer, so elimination is correctly rejected by the verifier. The
mode's removal path is exercised by `optimizer_via_miser_mode` on a via
chain whose far end is another (all-layer) via. This is an honest
limitation: Via Miser is a no-op on boards without eliminable vias.

## Result 4 — determinism

For `bga_8x8.json` and `greedy_outside_first.json`, reruns with
`--threads 1` and `--threads 4` produced identical `board_hash` in all
three modes (default/straighten/via_miser).

## Result 5 — real-board defect found and fixed

Routing `minimal.kicad_pcb` with the adjacent-memory strategy produced a
**VIOLATION** while default routing was clean:

```
trace width 0.250000mm below 0.800000mm required for net VCC
```

Cause: both pattern passes sized legs from the board default width, ignoring
per-net rules. Fix: pattern legs now resolve `requiredTraceWidth` /
`requiredClearance` per net, and every committed leg additionally passes the
verifier's local legality check (`has_local_violation`) or is rolled back.
After the fix the same board is COMPLETE with zero violations, and
`adjacent_memory_uses_net_required_width` pins the behaviour.

## Result 6 — newer passes on real boards

| mode | boards checked | result |
|---|---|---|
| miter (`orthogonal_demo`) | 6 | COMPLETE + verifier OK; shortened `multi_terminal_tree`, `dense_qfn`, `irregular_array` (1–2 corner miters each) |
| clean pad (`clean_pad_demo`) | 6 | COMPLETE + verifier OK; no asymmetric-pad exits on these fixtures (no-op, nothing broken) |
| adjacent memory (`adjacent_memory_demo`) | 6 | COMPLETE + verifier OK after the width fix |

## Result 7 — mutation testing: are the tests real?

Each mutation was applied to a source file, the affected target rebuilt, the
named test run, then the file restored and the tree re-verified green.
A mutation "killed" means the test failed — proof the assertion is load-bearing.

| # | mutation | test | result |
|---|---|---|---|
| 1 | straighten pass disabled (`if (false)`) | optimizer_straighten_reversed_corner | killed |
| 2 | straighten `must_shorten = false` | optimizer_straighten_reversed_corner | equivalent (candidate already requires shortening) |
| 3 | straighten shortening pre-check removed | optimizer_straighten_reverts_illegal_shortcut | equivalent (`must_shorten` enforces it) |
| 4 | via-miser focus lost (collinear pass runs) | optimizer_via_miser_mode | killed (`passes == 1`) |
| 5 | `via_miser_ran` never set | cli_via_miser_strategy_gate | killed |
| 6 | straighten forced on by default | cli_straighten_strategy_gate | killed |
| 7 | via-miser strategy wiring removed | cli_via_miser_strategy_gate | killed |
| 8 | memory jog ignores clearance | memory_pass_commits_aligned_pairs | killed |
| 9 | adjacent memory disabled in engine | cli_adjacent_memory_strategy_gate | killed |
| 10 | adjacent pass excludes same component | adjacent_memory_commits_same_component_pairs | killed |
| 11 | miter pass disabled | optimizer_miter_chamfers_corner | killed |
| 12 | miter degree gate removed | optimizer_miter_requires_degree_two | **survived, then killed** after strengthening the assertion to `rep.candidates == 0` |
| 13 | clean pad entries disabled | optimizer_clean_pad_entry_reroutes_long_axis | killed |
| 14 | clean pad long axis inverted | optimizer_clean_pad_entry_reroutes_long_axis | killed |
| 15 | miter strategy wiring removed | cli_orthogonal_miter_strategy_gate | killed |
| 16 | clean pad strategy wiring removed | cli_clean_pad_entries_strategy_gate | killed |

**Finding:** mutant 12 exposed a genuinely weak assertion — the junction
test only checked `applied == 0`, which the verifier satisfied even with the
degree gate gone. It now also asserts `candidates == 0` and `reverted == 0`,
so the gate itself is required.

### Environment caveat (CLI mutants)

CLI mutation runs were first attempted in a temp tree whose path contains a
space (`C:\Users\Darsh Gupta\...`). Every CLI test failed there regardless of
code, because the harness builds an unquoted command string — so those early
CLI results were invalid and discarded. The set was re-run in a space-free
path (`D:\...\.wt-validate-mut`) with a confirmed 40/40 baseline; the table
above reports the re-run.

## Limitations

* All new passes are OFF by default and require a strategy file.
* Hug (22) and Spread (23) are not implemented; both need route
  re-planning or moving fixed objects and are reported as open.
* EM full-wave relief comparison remains unvalidated (see
  `em/relief_research.md`); the analytical tier is what the tests cover.
