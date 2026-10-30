# Corpus campaign

Ten boards from the 2026-10-29 corpus run finish incomplete: five run out of
clock, three are partly blocked by geometry, two used to stop early. They were
outside every test, so the suite was green while a third of the corpus quietly
failed. They are now a campaign whose bar is **COMPLETE**:

```
python tests/test_corpus_boards.py --tier short   # ctest, ~2 min, currently RED
python tests/test_corpus_boards.py --tier long    # CI, ~25 min, currently RED
ctest -LE campaign                                # the other 33 tests, green
```

**The red is intentional.** Until every board in the manifest is fully routed,
this test fails and says which boards are short and why. See
[`HANDOFF.md`](HANDOFF.md) for the work that closes it.

## What each board is held to

| assertion | tolerance |
|---|---|
| **COMPLETE** - fully routed | none; the failure message carries the counts |
| verification legal, 0 violations | none |
| connected terminals >= recorded floor | 5% (progress ratchet) |
| unescapable pads <= recorded count | none |

Budgets are **work based** (`--max-epochs`, `--max-search-nodes`), not wall
clock, so a board produces the same copper on a laptop and on CI (verified at 1
vs 4 threads and under CPU contention). `--timeout` only guards a pathological
run. The gate budget is 12 epochs / 200k nodes for every board.

## Tiers

| tier | boards | where |
|---|---|---|
| short | `pico-radio-9-kotoldalas`, `rp2040_game_con`, `rp2040_nema`, `KiCAD_Board_3` | every ctest (~2 min) |
| long | all ten | CI campaign / nightly |

The six long-only boards include the two giants (`tinytapeout-demo` 1611
terminals, `OpenJBOD-RP2040` 852) which spend minutes just escaping and building
the graph, plus `interf_u`, `Pico 2 C`, `RPI-PICO`, `Pico WH`.

## The boards and why they were incomplete

| board | class | why |
|---|---|---|
| `interf_u` | clock | 684 terminals |
| `tinytapeout-demo` | clock | 1611 terminals; epoch 0 alone ~50 s |
| `Pico WH` | clock | 517 terminals |
| `OpenJBOD-RP2040` | clock | 852 terminals |
| `KiCAD_Board_3` | clock | 3 of 148 pads have no legal escape |
| `rp2040_nema` | clock | 1 of 86 pads have no legal escape |
| `Pico 2 C` | geometry | 26 of 211 pads have no legal escape (0 mm rule) |
| `RPI-PICO-R3a` | geometry | 17 of 181 pads have no legal escape |
| `pico-radio-9-kotoldalas` | gave-up | used to stop at 324/340 with 108 s unused |
| `rp2040_game_con` | gave-up | used to stop at 79/152 with 81 s unused |

Inputs are the frozen pads-only JSON in `boards/`, so the campaign needs no KiCad
install and no network.

## Recording

`--record` refreshes the progress ratchet (`connected`, `infeasible`); it never
relaxes the COMPLETE gate.

```
python tests/test_corpus_boards.py --tier short --record --only rp2040_game_con
python tests/test_corpus_boards.py --tier long  --record
```

Use `--keep-reports DIR` to keep each board's report JSON for debugging, and
`--only <board>` to run one.
