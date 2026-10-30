# Corpus campaign — two gates

Ten boards from the 2026-10-29 corpus run finish incomplete: five run out of
clock, three are partly blocked by geometry, two used to stop early. They were
outside every test, so the suite was green while a third of the corpus quietly
failed. They are now covered by **two independent gates** over one routing pass,
so the suite says which property is broken instead of one big red:

| gate | ctest test | question | status |
|---|---|---|---|
| **legality** | `test_corpus_legal` | did any board commit illegal copper? | **green** |
| **completion** | `test_corpus_complete` | is every board fully routed? | **red** (by design) |

```
ctest -R test_corpus_legal       # passes: 4/4 boards legal, 0 violations
ctest -R test_corpus_complete    # fails: 0/4 COMPLETE, names the shortfall
ctest -LE campaign               # everything else (green)

python tests/test_corpus_boards.py --gate legal|complete|all [--tier short|long]
```

The two tests route the same boards, so `--cache` makes the second one reuse the
first one's reports (0.7 s instead of ~2 min). The key is the board bytes + work
budget + seed + the router binary identity, so a rebuilt router or a changed
board reroutes automatically.

## What each gate asserts

**Legality** (hard safety, no tolerance): zero violations on every board. This is
the property that must never regress — illegal copper is worse than an
unrouted board.

**Completion** (the open work):

| assertion | tolerance |
|---|---|
| **COMPLETE** — fully routed | none; the message carries the counts |
| legal (re-checked) | none |
| connected terminals >= recorded floor | 5% (progress ratchet) |
| unescapable pads <= recorded count | none |

Budgets are **work based** (`--max-epochs`, `--max-search-nodes`), not wall
clock, so a board produces the same copper on a laptop and on CI (verified at 1
vs 4 threads and under CPU contention). `--timeout` only guards a pathological
run. The gate budget is 12 epochs / 200k nodes for every board.

## Tiers

| tier | boards | where |
|---|---|---|
| short | `pico-radio-9-kotoldalas`, `rp2040_game_con`, `rp2040_nema`, `KiCAD_Board_3` | every ctest (~2 min per gate) |
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
install and no network. The work that turns the completion gate green is in
[`HANDOFF.md`](HANDOFF.md).

## Recording

`--record` refreshes the progress ratchet (`connected`, `infeasible`); it never
relaxes the COMPLETE gate.

```
python tests/test_corpus_boards.py --gate complete --record --only rp2040_game_con
python tests/test_corpus_boards.py --tier long --gate complete --record
```

Use `--keep-reports DIR` to keep each board's report JSON for debugging.
