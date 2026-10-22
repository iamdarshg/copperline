#!/usr/bin/env python3
"""Test the clean-room DipTrace .dip importer against the fixture."""

import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import diptrace_import as dt

FIXTURE = os.path.join(ROOT, "fixtures", "reverse", "diptrace",
                       "Astable_Flip_Flop.dip")

PASS = 0
FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  ok  {name}")
    else:
        FAIL += 1
        print(f"  FAIL {name}  {detail}")


def main() -> int:
    with open(FIXTURE, "rb") as fh:
        data = fh.read()
    board = dt.parse_board(data)

    check("magic accepted", board["schema"] == "copperline/board/1")
    comps = board.get("_components", [])
    check("components parsed", len(comps) >= 10, f"got {len(comps)}")

    refs = {c["ref"]: c for c in comps}
    check("R1 = RES400/47k", refs.get("R1", {}).get("pattern") == "RES400"
          and refs.get("R1", {}).get("value") == "47k",
          str(refs.get("R1")))
    check("C1 = CAP100RP/22uF",
          refs.get("C1", {}).get("pattern") == "CAP100RP"
          and refs.get("C1", {}).get("value") == "22uF",
          str(refs.get("C1")))
    check("B1 = BAT/5V", refs.get("B1", {}).get("pattern") == "BAT"
          and refs.get("B1", {}).get("value") == "5V",
          str(refs.get("B1")))
    check("B1 has 3 pins", len(refs.get("B1", {}).get("pins", [])) == 3,
          str(refs.get("B1", {}).get("pins")))
    check("warnings present", len(board.get("import_warnings", [])) > 0)

    # nets: skeleton nets for each component with pins
    skel = [n for n in board["nets"] if n["name"].startswith("SKEL_")]
    check("skeleton nets", len(skel) >= 10, f"got {len(skel)}")

    # terminal ids globally unique
    seen = set()
    dup = False
    for n in board["nets"]:
        for t in n.get("terminals", []):
            if t["id"] in seen:
                dup = True
            seen.add(t["id"])
    check("terminal ids unique", not dup)

    # validate against the router binary if present
    router = os.path.join(ROOT, "build", "router.exe")
    if os.path.exists(router):
        tmp = os.path.join(ROOT, "em", "fixtures", "dt_board_test.json")
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump({k: v for k, v in board.items()
                       if not k.startswith("_")}, fh)
        proc = subprocess.run([router, "analyze", tmp, "--json"],
                              capture_output=True, text=True)
        try:
            rep = json.loads(proc.stdout)
            check("router analyze accepts import",
                  rep.get("schema", "").startswith("copperline/"),
                  proc.stdout[:200])
        except Exception as exc:  # noqa: BLE001
            check("router analyze accepts import", False, str(exc))
        os.remove(tmp)
    else:
        print("  (skip) router binary not built")

    print(f"\nDipTrace import self-tests: {PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())