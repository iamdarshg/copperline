"""Regression test for issue #43: an input-illegal board must not shadow a later
legal board that shares its coarse signature in tools/build_corpus.py.

The import, signature and input-DRC steps are stubbed so the dedupe ordering is
tested in isolation (no router, no real boards). Standard library only.
"""
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import build_corpus as bc  # noqa: E402

BOARD = {"nets": [{"terminals": [1, 2]}, {"terminals": [3, 4]}]}


def run(files, legal_by_path, sig_by_path):
    """Run build_corpus.main() over stubbed boards; return (kept list, {path: reason})."""
    bc.collect = lambda sources: list(files)
    bc.import_board = lambda path, work, idx: ((BOARD, path), None)
    order = iter(files)
    bc.signature = lambda b: sig_by_path[next(order)]
    bc.input_drc = lambda raw, work, idx: (legal_by_path[raw], 0 if legal_by_path[raw] else 3, None)
    with tempfile.TemporaryDirectory() as out:
        sys.argv = ["build_corpus.py", "--sources", "/src", "--out", out]
        bc.main()
        kept = [l for l in open(os.path.join(out, "boards.txt")).read().splitlines() if l]
        meta = json.load(open(os.path.join(out, "corpus_meta.json")))
    return kept, [(m["path"], m["reason"]) for m in meta]


failures = []

# Case 1 (the #43 report): illegal rev1 must not shadow legal rev2 with the same signature.
f1 = ["/src/rev1/board.kicad_pcb", "/src/rev2/board_fixed.kicad_pcb"]
kept, reasons = run(f1, {f1[0]: False, f1[1]: True}, {f1[0]: "S", f1[1]: "S"})
print("case1 kept:", kept, "reasons:", reasons)
if kept != [f1[1]] or reasons != [(f1[0], "input_illegal"), (f1[1], "kept")]:
    failures.append("case1: illegal board shadowed the later legal board (#43)")

# Case 2 (ordering over a sequence): the fix must not disable real dedupe.
#   A illegal S1, B legal S1, C legal S2, D legal S1 (true duplicate of B), E illegal S3
f2 = ["/src/a/a.kicad_pcb", "/src/b/b.kicad_pcb", "/src/c/c.kicad_pcb", "/src/d/d.kicad_pcb", "/src/e/e.kicad_pcb"]
legal = dict(zip(f2, [False, True, True, True, False]))
sig = dict(zip(f2, ["S1", "S1", "S2", "S1", "S3"]))
kept, reasons = run(f2, legal, sig)
print("case2 kept:", kept, "reasons:", reasons)
expected = [(f2[0], "input_illegal"), (f2[1], "kept"), (f2[2], "kept"),
            (f2[3], "duplicate_signature"), (f2[4], "input_illegal")]
if kept != [f2[1], f2[2]] or reasons != expected:
    failures.append(f"case2: expected kept [b, c] and reasons {expected}")

print("ok" if not failures else "FAIL: " + "; ".join(failures))
sys.exit(0 if not failures else 1)
