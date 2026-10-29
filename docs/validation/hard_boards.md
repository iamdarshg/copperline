# Hard-board runs (2026-10-27)

Purpose: push the router against the hardest fixtures in the repo and record
what actually happens, including the causes of non-COMPLETE results.

Commands: `router route <board> --json --no-board-echo --seed 42`
(timings are wall-clock on this machine, threads=auto/16).

| board | status | connected | verifier | failures | notes |
|---|---|---|---|---|---|
| bga_8x8.json | INCOMPLETE | 54/128 | false | 24 unattempted, 13 unreachable | 2-layer, 16 interior pads infeasible |
| bga_8x8_via.json | INCOMPLETE | 60/128 | false | 21 unattempted, 12 unreachable, 1 conflict | 4-layer, same 16 infeasible pads |
| greedy_outside_first.json | INCOMPLETE | 30/50 | **true** | 9 unreachable, 1 unattempted | fixture repaired (see below) |
| dense_qfn.json | COMPLETE | 64/64 | true | 0 | |
| impossible_escape.json | INCOMPLETE | 0/18 | false | 9 unreachable | fixture is designed infeasible |
| narrow_channel.json | COMPLETE | 4/4 | true | 0 | |
| blocked_impossible.json | INCOMPLETE | 0/2 | false | 1 unreachable | designed blocked |
| plane_split.json | COMPLETE | 4/4 | true | 0 | |
| high_current_bga.json | COMPLETE | 32/32 | true | 0 | |
| mixed_voltage_bga.json | COMPLETE | 32/32 | true | 0 | |
| irregular_array.json | COMPLETE | 26/26 | true | 0 | |
| dense_mcu_4layer.json | COMPLETE | 16/16 | true | 0 | |
| obstacle_detour.json | COMPLETE | 4/4 | true | 0 | |
| multi_terminal_tree.json | COMPLETE | 6/6 | true | 0 | |

## Why the 8x8 BGAs stop at ~60/128

The escape stage reports `pads_infeasible = 16` with reason
`no_exit_sectors`: the interior 4x4 pads have no legal exit on either layer.
Geometry: 0.8 mm pitch, 0.35 mm pads, 0.2 mm traces, 0.15 mm clearance.
The channel between neighbouring pads is 0.8 - 0.35 = 0.45 mm, and a legal
trace needs 0.2 + 2x0.15 = 0.5 mm. So no planar trace can leave an interior
pad, and the configured via style (0.6 mm outer over a 0.35 mm pad) does not
fit either. This is a stack-up/rules limit, not a search limit.

Evidence that it is not budget: five different configurations
(default, 4M search nodes + 20k epochs, deep recovery, route-k 8, all
combined) produce the exact same 60/128 and 30/64 tasks in 59-81 s.

The codebase already expects this: `test_engine` comments that the 8x8 has
infeasible interior escapes, and asserts the centre-out invariant rather
than a COMPLETE result.

## Fixture bug found and fixed: greedy_outside_first

The fixture's 25 test-point pads (0.5 mm) were placed 0.1-0.1414 mm apart,
below the board's own 0.15 mm clearance. The router was therefore reporting
VIOLATION for a board that never had a chance to verify. The pads are now
0.3 mm (minimum foreign pad gap 0.25 mm), the input verifies clean, and the
board routes to INCOMPLETE/legal 30/50.

## Board-scale run (opt-in regime)

`COPPERLINE_BOARD_SCALE=1 build/test_board_scale.exe` (480 nets, 8 pads each,
~2400 connection tasks, full-height walls forcing detours):

```
[scale] fixture tasks_total=2400 (threshold=2048) routed=2400/2400
ok - boardsize_fixture_actually_latches              (49.8 s)
[scale] latched connected=3840 legacy connected=3840 (of 3840)
ok - boardsize_latched_does_not_cost_connectivity    (434.1 s)
[scale] repair pass 1: connected=3840 unconnected=0 legal=1
[scale] repair pass 2: connected=3840 unconnected=0 legal=1
ok - boardsize_repair_pass_proves_correctness        (71.0 s)
PASS: 3/3 passed
```

Every pad is connected and the board stays legal; the board-scale policy
latches and does **not** cost connectivity versus the legacy policy. The
independent repair pass (rip-up/re-route then verify) finds nothing to fix.

## New report fields

`route` reports now carry `input_legal` and `input_violation_count`
(the DRC state of the imported board before routing). This distinguishes a
bad input from a router-introduced violation:

```
fixtures/violation_board.json  -> status VIOLATION, input_legal false,
                                  input_violation_count 7
fixtures/greedy_outside_first  -> status INCOMPLETE, input_legal true
```

Failures caused by an unescapable pad now report `reason:
"escape_infeasible"` with `best_partial: "pad escape infeasible: <detail>"`
instead of the vague `"unattempted"`.

## Why quick INCOMPLETEs now keep trying (2026-10-29)

Boards used to bail out with the budget barely touched: `rp2040_game_con`
reported INCOMPLETE 79/152 after 42 s of a 150 s cap, and recovery recorded
`generations: 2, ripups: 0` — i.e. it stopped **without ever moving a trace**,
which is the "this trace is in the way, so it must be impossible" behaviour.

Three causes, all fixed:

1. **Deferral gave up.** After 3 failed attempts a task was deferred, and once
   only deferred tasks remained the epoch loop `break`ed. Now a task gets 6
   attempts and then up to 3 bounded "second wind" passes with a reset failure
   count, and the re-arm count is reported as `stats.deferred_retries`.
2. **All-pruned branches ended the search.** When the transposition table
   pruned *every* first-ply branch, recovery `break`ed out immediately. That is
   a repeated-state signal, not proof of infeasibility: the loop now escalates
   to the next recovery mode (widening A*) and presses the failed corridors
   harder, and only `EXHAUSTIVE_LOCAL` may stop it.
3. **Rip-up counts were dishonest.** `recovery.generations` was incremented
   only on commit, so a run that tried four generations and committed nothing
   reported `generations: 1`. It now counts **attempts**, which is what
   `failures[].ripup_attempts` reports.

Effect on `rp2040_game_con` (150 s cap): 42 s -> 84+ s, 46 epochs instead of
~10, ending INCOMPLETE at the honest point (every remaining task has exhausted
6 attempts plus 3 second-wind passes) rather than quitting early. Connectivity
on that board is unchanged (79/152): the remaining terminals are blocked by
`+3.3V`/`GND` copper, and the escapes report a 0.2375 mm frontier gap, so the
next lever is a targeted local rip of those blocker traces, not more retries.
