#!/usr/bin/env python3
"""Corpus campaign: two gates over the same routing pass.

The boards that finish incomplete used to sit outside every test. They are now
checked by two independent gates so the suite says *which* property is broken:

  --gate legal     0 violations on every board. This is the hard safety gate;
                   illegal copper must never be committed. Green today.
  --gate complete  every board reaches COMPLETE (fully routed) *and* stays
                   legal *and* holds its progress floors. Red until the
                   handoff work lands (docs/validation/corpus/.../HANDOFF.md).
  --gate all       evaluate both and report them separately (default).

Both gates route the same way, so `--cache DIR` lets the second gate reuse the
first gate's reports: the key is the board bytes + work budget + seed + the
router binary identity, so a rebuilt router or a changed board reroutes.

Budgets are WORK based (--max-epochs, --max-search-nodes), never wall clock, so
a board produces the same copper on a laptop and on CI. `--timeout` only guards
a pathological run.

Usage:
  python tests/test_corpus_boards.py --gate legal
  python tests/test_corpus_boards.py --gate complete --cache build/gate-cache
  python tests/test_corpus_boards.py --gate all --tier long
  python tests/test_corpus_boards.py --gate complete --record   # refresh ratchet
"""
import argparse
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MANIFEST = os.path.join(HERE, "corpus_manifest.json")
SEED = 42


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


def safe_name(name):
    return name.replace("/", "_").replace("\\", "_").replace(" ", "_")


def cache_key(board_path, budget, router):
    h = hashlib.sha256(open(board_path, "rb").read()).hexdigest()
    st = os.stat(router)
    return (f"{h}:{budget['max_epochs']}:{budget['max_search_nodes']}:"
            f"{SEED}:{st.st_size}:{int(st.st_mtime)}")


def cache_load(cache_dir, name, key):
    if not cache_dir:
        return None
    rep = os.path.join(cache_dir, safe_name(name) + ".rep.json")
    kf = rep + ".key"
    try:
        if open(kf).read().strip() != key:
            return None
        return json.load(open(rep, encoding="utf-8", errors="replace"))
    except Exception:
        return None


def cache_store(cache_dir, name, key, rep_path):
    if not cache_dir or not os.path.exists(rep_path):
        return
    os.makedirs(cache_dir, exist_ok=True)
    dst = os.path.join(cache_dir, safe_name(name) + ".rep.json")
    shutil.copyfile(rep_path, dst)
    with open(dst + ".key", "w") as fh:
        fh.write(key + "\n")


def run_board(router, board_path, budget, threads, report_path):
    """Route one board. Exit codes are a status channel, not a verdict:
    0 ok, 4 incomplete, 7 budget exhausted are legitimate results here;
    2 invalid input, 3 malformed rules, 5 rule violation, 6 internal are not.
    """
    cmd = [router, "route", board_path, "--json", "--no-board-echo",
           "--seed", str(SEED), "--threads", str(threads),
           "--memory-budget-mb", "512",
           "--max-epochs", str(budget["max_epochs"]),
           "--max-search-nodes", str(budget["max_search_nodes"]),
           "--timeout", str(budget.get("timeout", 1800)),
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
            rep = json.load(open(report_path, encoding="utf-8", errors="replace"))
        except Exception:
            rep = None
    if rep is None:
        return None, wall, hard_fail.get(
            proc.returncode, f"no report (exit {proc.returncode})")
    if proc.returncode in hard_fail:
        return rep, wall, hard_fail[proc.returncode]
    return rep, wall, None


def measure(rep, wall, budget):
    v = rep.get("verification") or {}
    esc = rep.get("escape") or {}
    stats = rep.get("stats") or {}
    fails = rep.get("failures") or []

    def partial(f):
        return f.get("best_partial") or ""

    # "unattempted: no worker candidate produced" is what the engine writes when
    # a task was scheduled but the worker returned no route at all.
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
        max_epochs=budget["max_epochs"],
    )


def legality_problems(m):
    out = []
    if not m["legal"] or m["violations"]:
        out.append(f"ILLEGAL ({m['violations']} violations)")
    return out


def completion_problems(m, entry, slack):
    out = []
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
        out.append("NOT COMPLETE (" + detail + ")")
    floor = entry.get("connected")
    if floor is not None and m["connected"] is not None:
        allowed = int(math.floor(floor * (1.0 - slack)))
        if m["connected"] < allowed:
            out.append(f"connected {m['connected']} < floor {floor} "
                       f"(allowed {allowed})")
    cap = entry.get("infeasible")
    if cap is not None and m["infeasible"] > cap:
        out.append(f"unescapable pads {m['infeasible']} > {cap}")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tier", choices=["short", "long"], default="short")
    ap.add_argument("--gate", choices=["legal", "complete", "all"], default="all")
    ap.add_argument("--router")
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--slack", type=float, default=0.05,
                    help="allowed shortfall vs the recorded connectivity floor")
    ap.add_argument("--record", action="store_true",
                    help="refresh the progress ratchet in the manifest")
    ap.add_argument("--only", nargs="*", help="limit to these board names")
    ap.add_argument("--keep-reports", metavar="DIR",
                    help="write each board's report JSON here (debugging)")
    ap.add_argument("--cache", metavar="DIR",
                    help="reuse reports when board+budget+router are unchanged")
    args = ap.parse_args()

    router = find_router(args.router)
    if os.sep in router and not os.path.exists(router):
        print(f"router not found: {router}", file=sys.stderr)
        return 2

    manifest = json.load(open(MANIFEST))
    budget = manifest["budget"]
    only = set(args.only or [])
    legal_failures = []
    complete_failures = []
    updates = {}
    rows = []
    cached = 0

    for entry in manifest["boards"]:
        name = entry["name"]
        if only and name not in only:
            continue
        if args.tier not in entry.get("tiers", ["short", "long"]):
            continue
        board = os.path.join(ROOT, entry["input"])
        if not os.path.exists(board):
            print(f"SKIP {name}: missing input {entry['input']}")
            continue

        key = cache_key(board, budget, router)
        rep = cache_load(args.cache, name, key)
        tmpdir = None
        if rep is not None:
            cached += 1
            wall = 0.0
            hard = None
        else:
            if args.keep_reports:
                os.makedirs(args.keep_reports, exist_ok=True)
                rep_path = os.path.join(args.keep_reports, safe_name(name) + ".rep.json")
            else:
                tmpdir = tempfile.TemporaryDirectory()
                rep_path = os.path.join(tmpdir.name, "rep.json")
            rep, wall, hard = run_board(router, board, budget, args.threads, rep_path)
            if rep is not None and hard is None:
                cache_store(args.cache, name, key, rep_path)
        if rep is None or hard:
            legal_failures.append(f"{name}: {hard or 'no report'}")
            complete_failures.append(f"{name}: {hard or 'no report'}")
            rows.append((name, "HARD-FAIL", "-", "-", "-", hard or "no report"))
            if tmpdir:
                tmpdir.cleanup()
            continue

        m = measure(rep, wall, budget)
        updates[name] = dict(connected=m["connected"] if m["connected"] is not None else 0,
                             infeasible=m["infeasible"])
        lp = legality_problems(m)
        cp = lp + completion_problems(m, entry, args.slack)
        if lp:
            legal_failures.append(f"{name}: " + "; ".join(lp))
        if cp:
            complete_failures.append(f"{name}: " + "; ".join(cp))

        verdicts = []
        if args.gate in ("legal", "all"):
            verdicts.append("legal" if not lp else "ILLEGAL")
        if args.gate in ("complete", "all"):
            verdicts.append("complete" if not cp else "NOT COMPLETE")
        rows.append((name, m["status"], f"{m['connected']}/{m['total']}",
                     f"viol {m['violations']}", " / ".join(verdicts),
                     "; ".join(cp) if cp else "ok"))
        if tmpdir:
            tmpdir.cleanup()

    width = max((len(r[0]) for r in rows), default=10)
    print(f"corpus campaign ({args.tier} tier, gate={args.gate}, "
          f"router {os.path.basename(router)}"
          f"{', %d cached' % cached if cached else ''})")
    print(f"  {'board':{width}s}  {'status':11s} {'conn':>9s} {'viol':>7s} "
          f"{'gates':>20s}  detail")
    for name, status, conn, viol, gates, detail in rows:
        d = detail if len(detail) <= 78 else detail[:75] + "..."
        print(f"  {name:{width}s}  {str(status):11s} {conn:>9s} {viol:>7s} "
              f"{gates:>20s}  {d}")

    if args.record:
        for entry in manifest["boards"]:
            if entry["name"] in updates:
                entry.update(updates[entry["name"]])
        json.dump(manifest, open(MANIFEST, "w"), indent=1)
        print(f"\nrecorded {len(updates)} boards into "
              f"{os.path.relpath(MANIFEST, ROOT)}")
        print("note: --record only refreshes the progress ratchet; the COMPLETE "
              "gate is unchanged")

    failed = False
    print()
    if args.gate in ("legal", "all"):
        ok = len(rows) - len(legal_failures)
        if legal_failures:
            failed = True
            print(f"legality gate: FAIL ({ok}/{len(rows)} boards legal)")
            for f in legal_failures:
                print("  " + f)
        else:
            print(f"legality gate: PASS ({ok}/{len(rows)} boards legal, "
                  f"0 violations)")
    if args.gate in ("complete", "all"):
        ok = len(rows) - len(complete_failures)
        if complete_failures:
            failed = True
            print(f"completion gate: FAIL ({ok}/{len(rows)} boards COMPLETE)")
            for f in complete_failures:
                print("  " + f)
        else:
            print(f"completion gate: PASS ({ok}/{len(rows)} boards COMPLETE)")

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
