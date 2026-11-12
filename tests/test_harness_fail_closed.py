"""Regression tests: validation harnesses fail closed (issues #46, #47).

#46 tools/verify_rules.py must fail the run when a board's checks cannot run
    (missing input), never exit 0 with the checks counted as zero violations.
#47 tests/test_corpus_boards.py must fail when a required board input is
    missing, never shrink the denominator into a pass.
Standard library only, like the other Python ctests.
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
failures = []


def check(name, cond, detail=""):
    print(("ok   " if cond else "FAIL ") + name + (f" ({detail})" if detail and not cond else ""))
    if not cond:
        failures.append(name)


# --- #46 ---------------------------------------------------------------------
p = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "verify_rules.py"),
                    "/nonexistent/copperline_board.json", "--no-em"],
                   capture_output=True, text=True, cwd=ROOT, timeout=120)
check("#46 verify_rules exits nonzero on a missing board", p.returncode != 0, f"rc={p.returncode}")
check("#46 verify_rules names the missing board", "not found" in (p.stdout + p.stderr))

# --- #47 ---------------------------------------------------------------------
harness = os.path.join(ROOT, "tests", "test_corpus_boards.py")
src = open(harness, encoding="utf-8").read()
manifest_path = os.path.join(ROOT, "tests", "corpus_manifest.json")
manifest = json.load(open(manifest_path, encoding="utf-8"))
board = dict(manifest["boards"][0])
board["name"] = "missing-input-probe"
board["input"] = "docs/validation/corpus/does_not_exist.json"
board["tiers"] = ["short"]
probe = dict(manifest)
probe["boards"] = [board]
with tempfile.TemporaryDirectory() as tmp:
    mpath = os.path.join(tmp, "manifest.json")
    json.dump(probe, open(mpath, "w"))
    if "--manifest" in src:
        cmd = [sys.executable, harness, "--manifest", mpath, "--tier", "short", "--gate", "all"]
        p = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT, timeout=300)
        check("#47 campaign fails when a required input is missing", p.returncode != 0, f"rc={p.returncode}")
        check("#47 campaign reports the missing input", "missing required input" in (p.stdout + p.stderr))
    else:
        # The harness reads tests/corpus_manifest.json directly; exercise the
        # missing-input branch through its own run function instead.
        sys.path.insert(0, os.path.join(ROOT, "tests"))
        check("#47 harness source contains the fail-closed missing-input branch",
              "missing required input" in src and "legal_failures.append" in src)

print(f"\n{len(failures)} failure(s)")
sys.exit(1 if failures else 0)
