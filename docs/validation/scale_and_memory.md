# Scale and memory controls (2026-10-28)

## The problem

`route` on a large board used **4.4 GB peak RSS** while reporting a 1024 MB
limit. The engine's memory budget is an *accounting* bound (it caps batch
width and scratch charging), not an OS limit, and the wrapper that was
supposed to enforce a hard cap assigned the job object *after* the child
started — a call that failed silently.

## Controls

| control | kind | what it does |
|---|---|---|
| `--memory-budget-mb <N>` | engine | bounds batch/scratch accounting (64..65536 MB). Smaller budget => narrower batches => less scratch. |
| `--threads <N>` | engine | fewer workers => fewer transient board copies and verifier scratches. |
| `tools/run_capped.ps1 -LimitMB <N>` | OS | Windows job object (`JOB_OBJECT_LIMIT_PROCESS_MEMORY|JOB_MEMORY`). The script puts *itself* in the job before spawning, so the child inherits the limit from its first allocation; it refuses to run if membership cannot be established. Prints `CAPPED_RUN exit=… peak_rss_mb=… wall_s=…`. |

Example:

```
powershell -NoProfile -ExecutionPolicy Bypass -File tools/run_capped.ps1 `
  -LimitMB 1024 -Exe build\router.exe route board.json --json --threads 4 `
  --memory-budget-mb 512 --timeout 900
```

## Measured

| run | settings | peak RSS | wall | outcome |
|---|---|---|---|---|
| bga_8x8_via (4-layer, 128 pads) | cap 1 GB, budget 1 GB | **750 MB** | 100 s | INCOMPLETE (escape-limited) |
| Pi-class synthetic, 12 layers, 842 pads | cap 1 GB, budget 512 MB, 4 threads | **308 MB** | 240 s (timeout) | VIOLATION, 405/842 connected |

The 4.4 GB case was the same Pi-class board without a binding cap.

## Pi-class board (synthetic)

160x100 mm, 12 layers, 403 nets / 842 pads: 20x20 BGA SoC at 0.8 mm pitch,
two 12x8 DRAM chips, 8 DDR4 byte lanes (length-targeted), 8 DQS and 8 PCIe
differential pairs, 280 GPIO runs, power pads. Legal input (0 DRC
violations).

Result: with a 240 s budget the run times out during/after the escape stage
and ends **VIOLATION** — the escape planner commits 170 vias and stubs that
overlap each other on the dense BGA:

```
clearance -0.100000mm below 0.100000mm between DDR4_A4 and DDR4_DQ7_3
clearance -0.100000mm below 0.100000mm between DDR4_A6 and DDR4_A5
```

90 such violations, 405/842 pads connected. So at Pi-class density the
limiter is **escape legality**, not memory: the planner places more copper
than the BGA channel can legally hold. This is a known-class bug to fix in
the escape stage (reject a candidate when the *board* verifier rejects it,
and cap escape commit density per ring).
