# Routing corpus (2026-10-28)

Goal: run the router on many boards — from simple breakouts to dense
multi-layer designs — and publish what actually happens, including failures.

## Method

Per board, via `tools/corpus_harness.py`:
1. `router import <board> --output raw.json` (import fidelity).
2. Strip `traces`, `vias`, `planes` (route from pads + outline).
3. `router verify` the pads-only board → the **input DRC** column. A board
   that is already illegal cannot fairly judge the router.
4. `router route --seed 42 --threads 4 --memory-budget-mb 256 --timeout N`.
5. Record status, connected/total, final DRC violations, wall time.

"COMPLETE+legal" below means `status == COMPLETE` **and** the independent
verifier reports `ok` (connected and legal). Boards whose input DRC is not
clean are reported separately, never counted as router failures.

## Corpus

| set | boards | composition |
|---|---|---|
| synthetic | 15 | `tools/gen_boards.py`: breakouts, grids, dense BGA 6x6/10x10/14x14 (4-12 layers), board-scale detours |
| curated | 52 | 20 KiCad 9 demos + 14 boards from open-hardware repos (`OpenJBOD/rp2040`, `sabogalc/project-piCo`, `bt-soft/pico-radio-9`, `raspberrypipress/raspberry-pi-pico-with-kicad`) + 4 local projects (`drone-arm`, `Code-SDR`, `dev-board`) + 14 synthetic |
| wide | 118 | every unique `.kicad_pcb`/`.dsn` on this machine (deduped by content), including project snapshots and test artifacts |

Machine-readable results: `docs/validation/corpus/{synthetic,curated,wide}.json`.

## Results

Synthetic (15 boards, all with clean input DRC):

| outcome | boards |
|---|---|
| COMPLETE + legal | **12** |
| TIMEOUT (dense BGA 6x6/10x10/14x14 at 20 s) | 3 |
| illegal copper committed | 0 |

Curated (52 boards):

| outcome | boards |
|---|---|
| import failed | 1 (`RoyalBlue54L-Feather`, unsupported KiCad dialect) |
| input DRC already illegal | 20 |
| input legal → **COMPLETE + legal** | **10** |
| input legal → incomplete/timeout with **0 violations** | 13 |
| input legal → **illegal copper committed** | 8 |

Complete and legal on the curated set: `scale_32x6` (384 pads, 2 L, 16.6 s),
`scale_16x4`, `scale_8x2`, `ecc83-pp` (66/66), `h_bridge` (32/32),
`test_pads_inside_pads` (27/27), `grid_8`, `custom_pads_test`, `breakout_2`,
`microwave` (empty).

Large boards that connect without violations but run out of the 20 s budget:
`flat_hierarchy` (116/476), `interf_u` (74/684), `pic_programmer` (110/470),
`pico-radio-9` (235/357), `rp2040_game_con` (83/152), `OpenJBOD-RP2040`
(53/852). With a 300 s budget `pic_programmer` reaches **470/470 COMPLETE,
legal** (see `real_boards.md`).

Wide sweep (118 boards): 17 import failures, 97 inputs already DRC-illegal
(project snapshots with pads closer than their own rules), 4 legal inputs →
1 COMPLETE+legal. This set measures import robustness more than routing.

## Fix that landed after the first run (2026-10-28)

The first curated run had 8 boards ending with overlapping copper. Root
cause: `BoardVerifier::has_local_violation` only examined element pairs where
the **lower-indexed** element belonged to the touched net (`if (j <= i)
continue;`), so both the escape gate and the epoch gate silently passed real
overlaps. Removing that ordering assumption made the gates effective, and
epoch commits are now validated **per candidate** (commit, check, roll back
only that candidate) instead of trusting the arbiter's predicates alone.

Post-fix curated results (same 20 s budget):

| | before | after |
|---|---|---|
| eligible (input legal) | 31 | 26 |
| COMPLETE + legal | 10 | **9** |
| illegal copper committed | **8** | **2** |
| legal but incomplete/timeout | 13 | 15 |

Boards that were VIOLATION with 200/200 connected now finish **COMPLETE and
legal** (`working_with_schematics` 200/200, `laying_out_a_pcb` 200/200,
`sonde xilinx` 182/182). The two remaining illegal cases
(`tinytapeout-demo`, `Pico 2 C`, 4 violations each) come from a commit path
that still bypasses the gate (pair materialization runs after routing).

Fair benchmark: 9/26 = **34.6%** COMPLETE+legal at 20 s per board. 15 of the
26 are timeouts, not failures — `pic_programmer` (470/470) needed ~300 s in
`real_boards.md`.

## Remaining: illegal copper from the pair-materialization path

After the gate fix, two curated boards still end illegal
(`tinytapeout-demo`, `Pico 2 C`, 4 violations each). Both are boards where
diff-pair corridors are materialized: `engine.cpp` writes
`mat.traces_p/n` and `mat.vias_p/n` straight into the board **after** the
routing loop, outside the per-candidate epoch gate. That path still needs the
same transactional treatment.

Reproducer for the fixed path (kept as a regression scenario):

```
router route dev_board.kicad_pcb --threads 4 --timeout 60 \
  --strategy tools/probes/minimal.json   # escape/optimizer/tuning/recovery off
# before the fix: VIOLATION, 6 overlaps
# after:          INCOMPLETE, legal=True, 0 violations
```


## Benchmark: clean corpus at 180 s (2026-10-29)

Corpus built with `tools/build_corpus.py` (signature dedupe, pads >= 4, input
DRC clean): **21 boards**, 180 s each, 4 threads, 512 MB budget.

**11 / 21 = 52.4% COMPLETE + legal.**

| outcome | boards |
|---|---|
| COMPLETE+legal | `flat_hierarchy` 476/476 @89 s, `pic_programmer` 470/470 @78 s, `pico-radio-9` 357/357 @33 s, `sonde xilinx` 182/182 @8.6 s, `working_with_schematics` 200/200, `laying_out_a_pcb` 200/200, `stamp_and_module` 164/164, `ecc83-pp` 66/66, `h_bridge` 32/32, `test_pads` 27/27, `custom_pads` 6/6 |
| timeout | 5 — `interf_u`, `tinytapeout` (1611 pads), `RPI-PICO`, `Pico WH`, `OpenJBOD` (852 pads) |
| incomplete (legal) | 4 — `pico-radio-9-kötoldalas` 324/340, `KiCAD_Board_3` 109/198, `rp2040_nema` 103/184, `rp2040_game_con` 79/152 |
| illegal copper | 1 — `Pico 2 C` (4 violations) |

Progress from two engine fixes on 2026-10-29:

| | before | after |
|---|---|---|
| COMPLETE+legal | 10/21 (47.6%) | **11/21 (52.4%)** |
| `sonde xilinx` | 903 s timeout, 177/182 | **8.6 s, 182/182** |
| `pico-radio-9` | 48 s | 33 s |

The spin fix matters more than the raw rate: a task whose candidate was
accepted but whose terminals were still unconnected never incremented the
failure counter, so it was retried forever and kept resetting the stall
counter. Sonde burned 903 s that way.

Profile of the remaining timeouts (sonde, before the fix): graph edge
construction was 71 s of 120 s (11.3 M edges over 785 rebuilds) — the next
performance lever is reducing per-epoch graph rebuilds, not more timeout.

## Building a fair corpus

## Building a fair corpus

`tools/build_corpus.py` prepares the set before any routing happens:

- collects `.kicad_pcb`/`.dsn` from directories or a list file;
- imports each board and **drops boards with fewer than 4 pads** (their
  netlist lives in an external `.net`, so routing them would trivially
  "succeed" at 0/0 and inflate the score);
- dedupes by *board signature* (board size, layer count, net count, pad
  count) and then by file name, so minor revisions of one board collapse;
- verifies the pads-only board and **keeps only boards whose input DRC is
  clean** — a board that is already illegal cannot judge the router.

```
python tools/build_corpus.py \
  --sources <repo-dirs-or-list> --out build/bench
python tools/corpus_harness.py --boards build/bench/boards.txt \
  --out build/bench/run --timeout 180 --threads 4 --budget-mb 512
python tools/benchmark_corpus.py build/bench/run/corpus.json --target 0.70
```

## How to reproduce

```
python tools/gen_boards.py --out build/corpus-boards
python tools/corpus_harness.py --boards fixtures build/corpus-boards \
  --out build/corpus-out --timeout 20 --threads 4 --memory-budget-mb 256
```

CI: `.github/workflows/corpus.yml` (manual dispatch or weekly) builds the
router, generates the synthetic corpus, runs the harness over `fixtures/` and
the generated boards, prints the status summary and uploads `corpus.json`.
