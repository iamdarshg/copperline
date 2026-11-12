"""Re-run the unfinished boards with a large cap and record how long each
actually takes (or where it stalls).

Reads a corpus.json from corpus_harness.py, picks every row that is not
COMPLETE+legal, and routes its already-stripped board with --timeout N.
Writes results incrementally to <out>/long_run.json.
"""

import argparse
import json
import os
import subprocess
import time

ROOT = r"D:\CodeProjects\reverse\copperline"
ROUTER = os.path.join(ROOT, "build", "router.exe")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--corpus", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--timeout", type=int, default=900)
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--budget-mb", type=int, default=512)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rows = json.load(open(args.corpus))
    run_dir = os.path.dirname(args.corpus)
    todo = [r for r in rows
            if not (r.get("status") == "COMPLETE" and r.get("legal"))]
    # Easiest first: highest connected/total, so early results are informative.
    def progress(r):
        t = r.get("total") or 0
        return (r.get("connected") or 0) / t if t else -1.0
    todo.sort(key=progress, reverse=True)
    print(f"{len(todo)} unfinished boards, cap {args.timeout}s", flush=True)

    results_path = os.path.join(args.out, "long_run.json")
    results = []
    for r in todo:
        board = r.get("board", "?")
        idx = None
        for f in os.listdir(run_dir):
            if f.endswith("_stripped.json"):
                continue
        # The harness names stripped boards bNNN_stripped.json; map by order.
        # Fall back to re-importing the original path.
        stripped = None
        stem = r.get("path")
        if stem:
            cand = os.path.join(args.out, f"{board}.stripped.json")
            raw = os.path.join(args.out, f"{board}.raw.json")
            p = subprocess.run([ROUTER, "import", stem, "--output", raw,
                                "--json"], capture_output=True, text=True,
                               timeout=900)
            if os.path.exists(raw):
                b = json.load(open(raw))
                b["traces"] = []
                b["vias"] = []
                b["planes"] = []
                json.dump(b, open(cand, "w"))
                stripped = cand
        if not stripped:
            print(f"{board}: no stripped board", flush=True)
            continue
        rep = os.path.join(args.out, f"{board}.rep.json")
        t0 = time.time()
        timed_out = False
        try:
            subprocess.run([ROUTER, "route", stripped, "--json",
                            "--no-board-echo", "--seed", "42",
                            "--threads", str(args.threads),
                            "--memory-budget-mb", str(args.budget_mb),
                            "--timeout", str(args.timeout),
                            "--report", rep],
                           capture_output=True, text=True,
                           timeout=args.timeout + 60)
        except subprocess.TimeoutExpired:
            timed_out = True
        dt = time.time() - t0
        status, conn, total, viol = "NO_REPORT", None, None, None
        if os.path.exists(rep):
            try:
                d = json.load(open(rep))
                status = d.get("status")
                conn = d.get("connected_terminals")
                total = d.get("total_terminals")
                viol = len((d.get("verification") or {}).get("violations", []))
            except Exception:
                pass
        print(f"{board[:34]:34s} {status:10s} {conn}/{total} viol={viol} "
              f"{dt:7.1f}s{' (killed)' if timed_out else ''}", flush=True)
        results.append(dict(board=board, status=status, connected=conn,
                            total=total, violations=viol,
                            wall_s=round(dt, 1), killed=timed_out,
                            cap_s=args.timeout))
        json.dump(results, open(results_path, "w"), indent=1)


if __name__ == "__main__":
    main()
