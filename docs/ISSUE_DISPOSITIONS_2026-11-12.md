# Issue dispositions #27–#48 (C-G5), 2026-11-12

Branch `audit/copperline` (PR #52). Host: macOS 26.5 arm64. "Code present" means the
fix is in this branch with an `Issue #N` comment at the cited location. "Verified" means
behaviour was observed on this host with the command given. Nothing here is closed:
fixes land when the owner merges PR #52, and the ones without a regression test should
get one first.

| # | Title (short) | Disposition | Evidence |
|---|---|---|---|
| 27–36 | Release readiness (CI matrix, real-board qualification, FreeRouting benchmark, round-trip torture, install/packaging, CLI contract, fuzzing, timeout/cancel, license/changelog, release automation) | **Open — process epics**, not addressed in this PR | — |
| 37 | Cleanup optimizer retries rejected first candidate | Closed upstream before this audit | `tests/test_golden.cpp`, `src/optimizer.cpp` reference it |
| 38 | Hug/spread optimizer stops after first rejected window | **Code present, no regression test** | `src/optimizer.cpp:759-889` cursor advances past a fully rejected window instead of ending the pass |
| 39 | Pad-to-pad gap asymmetric for vertical oval (B rotated twice) | **Open — not addressed** | no reference in code or tests |
| 40 | Fine-pitch escape commits conflicting stubs without board-level legality gate | **Open — not addressed** | no reference |
| 41 | Synthetic corpus generator overlapping different-net pads | Closed upstream | — |
| 42 | Epoch commits can introduce DRC violations despite clean input | **Open — not addressed**. Note: the short and long tier legality gates are 4/4 and 10/10 with 0 violations on this host, which is consistent with but does not prove the absence of the defect | `tests/test_corpus_boards.py --gate legal` |
| 43 | Corpus builder drops distinct legal boards on signature collisions | **Code present, no regression test** | `tools/build_corpus.py:122` signature recorded only for legal boards |
| 44 | Long-run harness reuses stale reports / basename collisions | **Code present per handoff (stale-report cleanup), no `Issue #44` marker found, no test** | needs a marker and a test before it can be dispositioned as fixed |
| 45 | Thermal Joule model underestimates resistance on variable-width traces | **Open — not addressed** | no reference |
| 46 | verify_rules reports missing checks as zero violations | **Code present, no regression test** | `tools/verify_rules.py:44` collects failures and fails the run |
| 47 | Campaign passes when required inputs missing / router exits | **Code present, no regression test** | `tests/test_corpus_boards.py:239` missing input fails both gates |
| 48 | Gate cache reuses reports from a different router binary | **Code present; behaviour observed** | `tests/test_corpus_boards.py:65` cache key hashes the router binary. After rebuilding `build/router` with commit 9f3bc10 the long-tier campaign recomputed every board (real 487 s) instead of reusing cached reports |

## Additional defects found and fixed in this PR (not in #27–#48)

- `route-external` reported POSIX wait statuses as exit codes (7 → 1792); `test_cli` Windows-only assumptions (commit 07e8856).
- Data race in `HierarchyCache::snapshot_for` aborting the router on OpenJBOD-RP2040 (commit 9f3bc10; stress test red 2/3 → green 20/20).

## Suggested next tickets

1. Regression tests for #38, #43, #46, #47 (each can be a small fixture-driven test).
2. #42 and #40 are legality defects; they deserve targeted reproductions before any completion work (C-G1).
