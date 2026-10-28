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

## Known bug: illegal copper can still be committed

Eight curated boards end with real overlaps after routing even though their
input DRC is clean, e.g.:

```
laying_out_a_pcb.kicad_pcb       conn 200/200  violations 56
working_with_schematics.kicad_pcb conn 200/200  violations 11
Pico 2 C.kicad_pcb               conn  88/327  violations 11
RPI-PICO-R3a-PUBLIC.kicad_pcb    conn 107/282  violations  7
sonde xilinx.kicad_pcb           conn 182/182  violations  6
```

Reproducer (no escape, no recovery, no optimizer, a single epoch):

```
router route dev_board.kicad_pcb --threads 4 --timeout 60 \
  --strategy tools/probes/minimal.json   # escape/optimizer/tuning/recovery off
# -> VIOLATION, 6 overlaps, e.g. "clearance -0.104mm below 0.000mm
#    between /MP_BST and /TPS_ENSS"
```

Raising that board's clearance to 0.15 mm makes it worse (148 violations),
so this is not a zero-clearance edge case: the epoch commit path can place
copper that the arbiter's own predicates accept but the verifier rejects.
Escape commits are already gated this way (see `src/engine.cpp`); the epoch
and pair-materialization commit paths are not. Fixing that is the next task.

## How to reproduce

```
python tools/gen_boards.py --out build/corpus-boards
python tools/corpus_harness.py --boards fixtures build/corpus-boards \
  --out build/corpus-out --timeout 20 --threads 4 --memory-budget-mb 256
```

CI: `.github/workflows/corpus.yml` (manual dispatch or weekly) builds the
router, generates the synthetic corpus, runs the harness over `fixtures/` and
the generated boards, prints the status summary and uploads `corpus.json`.
