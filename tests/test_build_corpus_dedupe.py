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
files = ["/src/rev1/board.kicad_pcb", "/src/rev2/board_fixed.kicad_pcb"]
legal_by_path = {files[0]: False, files[1]: True}

bc.collect = lambda sources: list(files)
bc.import_board = lambda path, work, idx: ((BOARD, path), None)
bc.signature = lambda b: "SIG-SAME"
bc.input_drc = lambda raw, work, idx: (legal_by_path[raw], 0 if legal_by_path[raw] else 3, None)

with tempfile.TemporaryDirectory() as out:
    sys.argv = ["build_corpus.py", "--sources", "/src", "--out", out]
    bc.main()
    kept = [l for l in open(os.path.join(out, "boards.txt")).read().splitlines() if l]
    meta = json.load(open(os.path.join(out, "corpus_meta.json")))

reasons = {m["path"]: m["reason"] for m in meta}
ok = kept == [files[1]] and reasons[files[0]] == "input_illegal" and reasons[files[1]] == "kept"
print("kept:", kept, "reasons:", reasons)
print("ok" if ok else "FAIL: illegal board shadowed the later legal board (#43)")
sys.exit(0 if ok else 1)
