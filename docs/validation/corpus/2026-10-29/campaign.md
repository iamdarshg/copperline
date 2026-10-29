# Corpus campaign

Ten boards from the 2026-10-29 corpus run finish **incomplete**: five run out of
clock, three are partly blocked by geometry, two used to stop early. They are
not "passing" tests just because the router returns legal copper, so they are
now a campaign of their own:

```
python tests/test_corpus_boards.py --tier short      # ctest, ~1 min
python tests/test_corpus_boards.py --tier long       # CI, ~25 min
```

`ctest` runs the **short** tier as `test_corpus_boards`, and
`.github/workflows/corpus.yml` runs both tiers on the weekly/dispatch job.

Not every board fits every tier. The two giants (`tinytapeout-demo` 1611
terminals, `OpenJBOD-RP2040` 852) take ~3 minutes just to escape and build the
graph even at one epoch, so they are long-tier only. The short tier carries the
four boards that are both cheap and diagnostic:

| board | why it is in the short tier |
|---|---|
| `pico-radio-9-kotoldalas` | the 324/340 board that used to stop with 108 s unused |
| `rp2040_game_con` | the 79/152 board that used to stop after 42 s |
| `rp2040_nema` | clock-limited, 103/184 |
| `KiCAD_Board_3` | clock + geometry, 109/198 |

All ten run in the long tier (CI).

## What each board is held to

Budgets are **work based** - `--max-epochs` and `--max-search-nodes` - not wall
clock, so the same board produces the same copper on a laptop and on CI.
`--timeout` is only a guard against a pathological run.

| assertion | tolerance |
|---|---|
| verification legal, 0 violations | none - illegal copper fails immediately |
| connected terminals >= recorded floor | `--slack` (default 5%, CI relaxes it) |
| unescapable pads <= recorded count | none - the escape stage cannot regress |
| an INCOMPLETE run must not stop before its epoch budget | none |

The last one is the point of the exercise: the 2026-10-29 bug was a loop that
quit with 60% of its budget unused and reported the board unroutable. A green
suite hid it because nothing measured the boards that fail.

## The boards

| board | class | why it was incomplete |
|---|---|---|
| `interf_u` | clock | 684 terminals |
| `tinytapeout-demo` | clock | 1611 terminals; one epoch is ~50 s |
| `Pico WH` | clock | 517 terminals |
| `OpenJBOD-RP2040` | clock | 852 terminals |
| `KiCAD_Board_3` | clock | 3 of 148 pads have no legal escape |
| `rp2040_nema` | clock | 1 of 86 pads have no legal escape |
| `Pico 2 C` | geometry | 26 of 211 pads have no legal escape (0 mm rule) |
| `RPI-PICO-R3a` | geometry | 17 of 181 pads have no legal escape |
| `pico-radio-9-kotoldalas` | gave-up | stopped at 324/340 with 108 s unused |
| `rp2040_game_con` | gave-up | stopped at 79/152 with 81 s unused |

Inputs are the frozen pads-only JSON in
`docs/validation/corpus/2026-10-29/boards/`, so the campaign needs no KiCad
install and no network.

## Recording

When a board legitimately improves (or a rule change shifts it), re-record the
floors for that board and review the diff:

```
python tests/test_corpus_boards.py --tier short --record --only rp2040_game_con
python tests/test_corpus_boards.py --tier long  --record
```

`--record` writes the measured `connected` / `infeasible` values into
`tests/corpus_manifest.json`; every assertion reads them from there, so the
manifest doubles as the campaign's ledger.

Recorded values on the reference machine (4 threads), for orientation:

| board | short tier (3 epochs / 50k nodes) | long tier (12 epochs / 200k nodes) |
|---|---|---|
| `pico-radio-9-kotoldalas` | 89/340 | 217/340 |
| `rp2040_game_con` | 61/152 | 79/152 |
| `rp2040_nema` | 49/184 | 94/184 |
| `KiCAD_Board_3` | 38/198 | 94/198 |
| `interf_u` | - | 398/684 |
| `Pico 2 C` | - | 111/327 |
| `RPI-PICO-R3a` | - | 89/282 |
| `Pico WH` | - | 238/517 |
| `tinytapeout-demo` | - | 171/1611 |
| `OpenJBOD-RP2040` | - | 364/852 |

Every one of them reports 0 violations in both tiers.

## Deliberately not asserted

* **COMPLETE status.** These boards are not expected to finish; asserting it
  would just fail forever. The bar is "legal, and no worse than recorded".
* **Exact connectivity on CI.** Thread count and CPU change how much work fits,
  so CI relaxes the floors (`--slack 1.0` in the workflow) and relies on the
  machine-independent checks (legality, escape cap, no early give-up).
