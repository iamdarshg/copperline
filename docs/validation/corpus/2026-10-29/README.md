# Curated corpus run - 2026-10-29 (post zero-clearance fix)

Frozen evidence for the 21-board clean corpus benchmark, captured so it can be
reproduced without the original temporary checkouts (they are gone).

Build measured: `0856188` + the working-tree changes of 2026-10-29
(zero-clearance legality gate, escape-candidate retry, second wind, recovery
escalation). Parameters: `--timeout 180 --threads 4 --memory-budget-mb 512
--seed 42`.

## Result

**11 / 21 COMPLETE + legal. 21 / 21 boards verify with 0 violations.**

| outcome | count | boards |
|---|---|---|
| COMPLETE+legal | 11 | `flat_hierarchy` 476/476, `pic_programmer` 470/470, `pico-radio-9` 357/357, `sonde xilinx` 182/182, `working_with_schematics` 200/200, `laying_out_a_pcb` 200/200, `stamp_and_module` 164/164, `ecc83-pp` 66/66, `h_bridge` 32/32, `test_pads_inside_pads` 27/27, `custom_pads_test` 6/6 |
| TIMEOUT | 5 | `interf_u` 451/684, `tinytapeout-demo` 27/1611, `RPI-PICO-R3a` 127/282, `Pico WH` (killed at cap), `OpenJBOD-RP2040` 345/852 |
| INCOMPLETE (legal) | 4 | `pico-radio-9-kotoldalas` 324/340, `KiCAD_Board_3` 109/198, `rp2040_nema` 103/184, `rp2040_game_con` 79/152 |
| illegal copper | **0** | - |

The previous run (`../bench180.json`) had `Pico 2 C` at **246/327 with 4
violations**; it is now 126/327 with **0**. The connectivity drop is honest:
the rejected escape stubs were connecting terminals through illegal copper.

## Why each incomplete board stopped

| board | result | budget used | cause |
|---|---|---|---|
| `pico-radio-9-kotoldalas` | 324/340 | 72-96 s of 180 | blockers are ordinary traces; reaches `EXHAUSTIVE_LOCAL`, `ripups` 0 |
| `rp2040_game_con` | 79/152 | 107 s | `EXHAUSTIVE_LOCAL` reached, no rip branch survives |
| `KiCAD_Board_3` | 109/198 | 181 s | clock; 3 of 148 pads unescapable |
| `rp2040_nema` | 103/184 | 183 s | clock; 1 of 86 pads unescapable |
| `interf_u` | 451/684 | 181 s | clock (684 terminals) |
| `tinytapeout-demo` | 27/1611 | 185 s | clock; epoch 0 alone ~50 s |
| `Pico WH` | - | cap | clock (517 terminals) |
| `OpenJBOD-RP2040` | 345/852 | 185 s | clock (852 terminals) |
| `RPI-PICO-R3a` | 127/282 | 173 s | 17 of 181 pads unescapable |
| `Pico 2 C` | 126/327 | 180 s | 26 of 211 pads unescapable |

Only `Pico 2 C` (26/211) and `RPI-PICO` (17/181) are dominated by geometry.
Every incomplete board reports `recovery.ripups = 0`: no branch exists where
ripping a blocker *and* re-placing the victim both succeed, even in
`EXHAUSTIVE_LOCAL`. That - not a retry budget - is the open problem.

`classification.txt` / `classification.json` are produced by
`tools/classify_corpus.py` and split the 10 incomplete boards into 5 `clock`,
3 `clock+geometry` and 2 `gave-up`.

## Re-run of the "gave-up" boards with the escalation build

After the recovery-escalation change (the benchmark above used the build from
before the relink), the four early-stopping boards were re-run at the same
180 s cap:

| board | before (this corpus) | after escalation | modes now reached |
|---|---|---|---|
| `pico-radio-9-kotoldalas` | INCOMPLETE 324/340 @72 s | BUDGET_EXHAUSTED 324/340 @96 s, re-armed 8 | `EXHAUSTIVE_LOCAL` |
| `rp2040_game_con` | INCOMPLETE 79/152 @98 s | BUDGET_EXHAUSTED 79/152 @107 s | `EXHAUSTIVE_LOCAL` |
| `KiCAD_Board_3` | INCOMPLETE 109/198 @137 s | TIMEOUT 109/198 @181 s | `RECOVERY` |
| `rp2040_nema` | INCOMPLETE 103/184 @154 s | TIMEOUT 103/184 @183 s | (budget spent in epochs) |

Escalation demonstrably works - the strongest mode now runs, tasks are re-armed
up to 8 times, and the runs use the full budget - but **connectivity did not
move on any of them**. The blocker is that recovery cannot both rip a route and
successfully re-place it, so no branch is committable. That is the next piece
of work, not another retry knob.

## Contents

```
corpus.json                     harness rows for all 21 boards
harness_log.txt                 per-board harness output
boards21.txt                    the board list used (original absolute paths)
boards/bNNN_stripped.json       pads-only inputs for the 13 boards whose source
                                checkout was in temp; the 8 KiCad demo boards
                                are re-importable from Program Files
reports/*.rep.json              full reports for the boards classified above
provenance/pico2c_gate.txt      the 7 provenance lines that localise the
                                zero-clearance escape-gate hole
```

`boards21.txt` still contains the old absolute paths. To re-run any saved
board without the originals:

```
build/router.exe route docs/validation/corpus/2026-10-29/boards/b010_stripped.json \
  --json --no-board-echo --seed 42 --threads 4 --memory-budget-mb 512 \
  --timeout 180 --report /tmp/b010.rep.json
```

Index -> board name is `corpus.json[bNNN].board`.

## How the classification was derived

Each report's `failures[]` carries `blockers` (trace/pad/via nets),
`best_partial` (`frontier_gap_mm=...`), `ripup_attempts` and `modes_attempted`,
and `escape.unresolved_reasons` distinguishes `no_exit_sectors` (geometric)
from `clearance_conflict` (blocked by copper another route owns). A failure
with a large frontier gap and trace blockers is a rippable-congestion failure;
`no_exit_sectors` is a rule/geometry limit.
