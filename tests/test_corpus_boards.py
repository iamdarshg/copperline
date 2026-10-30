#!/usr/bin/env python3
"""Corpus campaign: hold the hard boards to a recorded bar.

The published benchmark (`docs/validation/corpus/`) reports boards that finish
incomplete because they run out of clock or because they stop trying. Those
boards used to sit outside every test: ctest was green while ten of them were
quietly incomplete. This campaign puts them under test.

Budgets are **work based** (`--max-epochs`, `--max-search-nodes`), not wall
clock, so a board produces the same copper on a laptop and on CI. Wall time is
carried by a generous `--timeout` that only exists to stop a pathological run.

Per tier, each board asserts:

  * **COMPLETE** - the board is fully routed. This is the bar: an INCOMPLETE
    board fails the campaign, with the unconnected-terminal and failed-task
    counts in the message.
  * LEGAL - 0 violations (absolute, no tolerance)
  * connected terminals >= the recorded floor (progress ratchet while a board
    is still short of COMPLETE)
  * unescapable pads <= the recorded count (the escape stage cannot regress)

Tiers: `short` runs on every ctest (a few minutes); `long` is the CI campaign
and needs ~20 minutes.

THE SUITE IS EXPECTED TO BE RED until every board in the manifest completes.
Run the rest of the suite with `ctest -LE campaign`.

Usage:
  python tests/test_corpus_boards.py --tier short
  python tests/test_corpus_boards.py --tier long --slack 0.05
  python tests/test_corpus_boards.py --tier short --record
"""
import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MANIFEST = os.path.join(HERE, "corpus_manifest.json")


def find_router(explicit):
    if explicit:
        return explicit
    env = os.environ.get("ROUTER_BIN")
    if env:
        return env
    for cand in ("build/router.exe", "build/router", "router.exe", "router"):
        p = os.path.join(ROOT, cand)
        if os.path.exists(p):
            return p
    return "router"


def run_board(router, board_path, tier, threads, report_path):
    """Route one board. Exit codes are a status channel, not a verdict:
    0 ok, 4 incomplete, 7 budget exhausted are legitimate outcomes here;
    2 invalid input, 3 malformed rules, 5 rule violation, 6 internal are not.
    """
    cmd = [router, "route", board_path, "--json", "--no-board-echo",
           "--seed", "42", "--threads", str(threads),
           "--memory-budget-mb", "512",
           "--max-epochs", str(tier["max_epochs"]),
           "--max-search-nodes", str(tier["max_search_nodes"]),
           "--timeout", str(tier.get("timeout", 1800)),
           "--report", report_path]
    t0 = time.time()
    proc = subprocess.run(cmd, stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL)
    wall = time.time() - t0
    hard_fail = {2: "invalid input", 3: "malformed rules",
                 5: "rule violation", 6: "internal failure"}
    rep = None
    if os.path.exists(report_path):
        try:
            with open(report_path) as fh:
                rep = json.load(fh)
        except Exception:
            rep = None
    if rep is None:
        return None, wall, proc.returncode, hard_fail.get(
            proc.returncode, f"no report (exit {proc.returncode})")
    if proc.returncode in hard_fail:
        return rep, wall, proc.returncode, hard_fail[proc.returncode]
    return rep, wall, proc.returncode, None


def measure(rep, wall, tier):
    v = rep.get("verification") or {}
    esc = rep.get("escape") or {}
    stats = rep.get("stats") or {}
    fails = rep.get("failures") or []
    def partial(f):
        return f.get("best_partial") or ""
    # "unattempted: no worker candidate produced" is what the engine writes
    # when a task was scheduled but the worker returned no route at all.
    no_candidate = sum(1 for f in fails if partial(f).startswith("unattempted"))
    rejected = sum(1 for f in fails if "arbiter" in partial(f))
    searched = sum(1 for f in fails if "frontier_gap_mm=" in partial(f))
    return dict(
        status=rep.get("status"),
        connected=rep.get("connected_terminals"),
        total=rep.get("total_terminals"),
        tasks_total=stats.get("tasks_total") or 0,
        tasks_routed=stats.get("tasks_routed") or 0,
        no_candidate=no_candidate,
        rejected_by_arbiter=rejected,
        searched_blocked=searched,
        failures=len(fails),
        violations=len(v.get("violations") or []),
        legal=bool(v.get("legal")),
        infeasible=esc.get("pads_infeasible") or 0,
        pads=esc.get("pads_total") or 0,
        epochs=stats.get("epochs") or 0,
        ripups=(rep.get("recovery") or {}).get("ripups") or 0,
        wall_s=round((stats.get("time_ms") or 0) / 1000.0, 1) or round(wall, 1),
        max_epochs=tier["max_epochs"],
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tier", choices=["short", "long"], default="short")
    ap.add_argument("--router")
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--slack", type=float, default=0.05,
                    help="allowed shortfall vs the recorded connectivity floor")
    ap.add_argument("--record", action="store_true",
                    help="rewrite the manifest from this run instead of asserting")
    ap.add_argument("--only", nargs="*", help="limit to these board names")
    ap.add_argument("--keep-reports", metavar="DIR",
                    help="write each board's report JSON here (debugging)")
    args = ap.parse_args()

    router = find_router(args.router)
    if os.sep in router and not os.path.exists(router):
        print(f"router not found: {router}", file=sys.stderr)
        return 2

    manifest = json.load(open(MANIFEST))
    budget = manifest["budget"]
    only = set(args.only or [])
    failures = []
    updates = {}
    rows = []

    for entry in manifest["boards"]:
        name = entry["name"]
        if only and name not in only:
            continue
        # A board may opt out of a tier (the two 800-1600 terminal boards cost
        # minutes just to escape, so they only run in the CI campaign).
        if args.tier not in entry.get("tiers", ["short", "long"]):
            continue
        board = os.path.join(ROOT, entry["input"])
        if not os.path.exists(board):
            print(f"SKIP {name}: missing input {entry['input']}")
            continue
        tier = budget
        if args.keep_reports:
            os.makedirs(args.keep_reports, exist_ok=True)
            rep_path = os.path.join(args.keep_reports, name.replace("/", "_") + ".rep.json")
            rep, wall, rc, hard = run_board(router, board, tier, args.threads, rep_path)
        else:
            with tempfile.TemporaryDirectory() as tmp:
                rep, wall, rc, hard = run_board(router, board, tier, args.threads,
                                                os.path.join(tmp, "rep.json"))
        if rep is None or hard:
            failures.append(f"{name}: {hard or 'no report'}")
            rows.append((name, "HARD-FAIL", "-", "-", "-", hard or "no report"))
            continue
        m = measure(rep, wall, tier)
        # `--record` refreshes the progress ratchet only; it never relaxes the
        # COMPLETE gate.
        updates[name] = dict(
            connected=m["connected"] if m["connected"] is not None else 0,
            infeasible=m["infeasible"])

        problems = []
        # COMPLETE is the bar. Not "legal", not "no worse than last time": a
        # board that is not fully routed fails this campaign.
        if m["status"] != "COMPLETE":
            unconn = (m["total"] or 0) - (m["connected"] or 0)
            failed = max(0, (m["tasks_total"] or 0) - (m["tasks_routed"] or 0))
            detail = (f"{m['status']} {m['connected']}/{m['total']}, "
                      f"{unconn} terminals unconnected, {failed} task-routes "
                      f"failed: {m['no_candidate']} produced no candidate, "
                      f"{m['searched_blocked']} searched and blocked, "
                      f"{m['rejected_by_arbiter']} rejected by the arbiter")
            if (m["status"] == "INCOMPLETE" and
                    m["epochs"] < max(1, m["max_epochs"] - 1)):
                detail += f"; stopped after {m['epochs']} of {m['max_epochs']} epochs"
            problems.append("NOT COMPLETE (" + detail + ")")
        if not m["legal"] or m["violations"]:
            problems.append(f"ILLEGAL ({m['violations']} violations)")
        floor = entry.get("connected")
        if floor is not None and m["connected"] is not None:
            allowed = int(math.floor(floor * (1.0 - args.slack)))
            if m["connected"] < allowed:
                problems.append(f"connected {m['connected']} < floor {floor} "
                                f"(allowed {allowed})")
        cap = entry.get("infeasible")
        if cap is not None and m["infeasible"] > cap:
            problems.append(f"unescapable pads {m['infeasible']} > {cap}")

        verdict = "ok" if not problems else "FAIL: " + "; ".join(problems)
        if problems:
            failures.append(f"{name}: {verdict}")
        rows.append((name, m["status"], f"{m['connected']}/{m['total']}",
                     f"viol {m['violations']}",
                     f"ep {m['epochs']}/{m['max_epochs']}", verdict))

    width = max((len(r[0]) for r in rows), default=10)
    print(f"corpus campaign ({args.tier} tier, router {os.path.basename(router)})")
    print(f"  {'board':{width}s}  {'status':11s} {'conn':>9s} {'viol':>7s} "
          f"{'epochs':>9s}  verdict")
    for name, status, conn, viol, eps, verdict in rows:
        print(f"  {name:{width}s}  {str(status):11s} {conn:>9s} {viol:>7s} "
              f"{eps:>9s}  {verdict}")

    if args.record:
        for entry in manifest["boards"]:
            if entry["name"] in updates:
                entry.update(updates[entry["name"]])
        json.dump(manifest, open(MANIFEST, "w"), indent=1)
        print(f"\nrecorded {len(updates)} boards into "
              f"{os.path.relpath(MANIFEST, ROOT)}")
        print("note: the COMPLETE gate is unchanged - recording floors never "
              "turns a red campaign green")

    if failures:
        print(f"\nFAIL ({len(failures)}/{len(rows)})")
        for f in failures:
            print("  " + f)
        return 1
    print(f"\nPASS: {len(rows)}/{len(rows)} boards held the bar")
    return 0


if __name__ == "__main__":
    sys.exit(main())
