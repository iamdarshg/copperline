"""Corpus harness: import -> strip copper -> verify -> route -> verify.

Usage:
  python tools/corpus_harness.py --boards <dir-or-file> [...] --out <dir>
        [--timeout 30] [--threads 4] [--budget-mb 256] [--limit 0]

Writes <out>/corpus.json and prints one line per board. Boards that cannot be
imported are recorded with status "IMPORT_FAILED" (honest coverage, not a
silent skip).
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROUTER = os.path.join(ROOT, "build", "router.exe")
if not os.path.exists(ROUTER):
    ROUTER = os.path.join(ROOT, "build", "router")


def seg_len(t):
    return math.hypot(t["x2_mm"] - t["x1_mm"], t["y2_mm"] - t["y1_mm"])


def run(cmd, timeout):
    t0 = time.time()
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                           cwd=ROOT)
        return p, time.time() - t0, False
    except subprocess.TimeoutExpired:
        return None, time.time() - t0, True


def collect(paths):
    out = []
    for p in paths:
        if p.endswith(".txt") and os.path.isfile(p):
            for line in open(p, encoding="utf-8"):
                line = line.strip()
                if line:
                    out.append(line)
            continue
        if os.path.isfile(p):
            out.append(p)
        else:
            for dirpath, _dirnames, names in os.walk(p):
                for n in names:
                    if n.endswith((".kicad_pcb", ".dsn", ".json")):
                        out.append(os.path.join(dirpath, n))
    # Deterministic order; skip our own generated outputs.
    seen = set()
    uniq = []
    for p in sorted(out):
        if "_raw" in p or "_stripped" in p or "_routed" in p or "_report" in p:
            continue
        if p in seen:
            continue
        seen.add(p)
        uniq.append(p)
    return uniq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--boards", nargs="+", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--timeout", type=int, default=30)
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--budget-mb", type=int, default=256)
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    boards = collect(args.boards)
    if args.limit:
        boards = boards[: args.limit]
    print(f"{len(boards)} candidate boards", flush=True)

    rows = []
    for idx, path in enumerate(boards):
        name = os.path.basename(path)
        raw = os.path.join(args.out, f"b{idx:03d}_raw.json")
        p, imp_s, imp_to = run([ROUTER, "import", path, "--output", raw, "--json"],
                               600)
        if imp_to or not os.path.exists(raw):
            rows.append(dict(board=name, path=path, status="IMPORT_FAILED",
                             detail=((p.stdout or p.stderr)[:160] if p else "timeout"),
                             import_s=round(imp_s, 1)))
            print(f"{idx:3d} {name[:34]:34s} IMPORT_FAILED", flush=True)
            continue
        b = json.load(open(raw))
        nets = len(b.get("nets", []))
        pads = sum(len(n.get("terminals", [])) for n in b.get("nets", []))
        layers = len(b.get("layers", []))
        src = dict(traces=len(b.get("traces", [])), vias=len(b.get("vias", [])),
                   length=round(sum(seg_len(t) for t in b.get("traces", [])), 1))
        b["traces"] = []
        b["vias"] = []
        b["planes"] = []
        stripped = os.path.join(args.out, f"b{idx:03d}_stripped.json")
        json.dump(b, open(stripped, "w"))

        pr, ver_s, ver_to = run([ROUTER, "verify", stripped, "--json"], 300)
        input_legal = None
        input_viol = None
        if pr and pr.stdout.strip().startswith("{"):
            try:
                vd = json.loads(pr.stdout)
                input_legal = bool(vd.get("legal"))
                input_viol = len(vd.get("violations", []))
            except Exception:
                pass

        rep = os.path.join(args.out, f"b{idx:03d}_report.json")
        routed = os.path.join(args.out, f"b{idx:03d}_routed.json")
        cmd = [ROUTER, "route", stripped, "--json", "--no-board-echo",
               "--seed", "42", "--threads", str(args.threads),
               "--memory-budget-mb", str(args.budget_mb),
               "--timeout", str(args.timeout), "--report", rep,
               "--output", routed]
        rt, route_s, route_to = run(cmd, args.timeout + 60)
        status = "TIMEOUT" if route_to else "NO_REPORT"
        conn = total = None
        legal = None
        violations = None
        if os.path.exists(rep):
            try:
                d = json.load(open(rep))
                status = d.get("status", status)
                conn = d.get("connected_terminals")
                total = d.get("total_terminals")
                v = d.get("verification") or {}
                legal = v.get("ok")
                violations = len(v.get("violations", []))
            except Exception:
                pass
        rows.append(dict(board=name, path=path, nets=nets, pads=pads,
                         layers=layers, src=src, input_legal=input_legal,
                         input_violations=input_viol, status=status,
                         connected=conn, total=total, legal=legal,
                         violations=violations,
                         import_s=round(imp_s, 1), route_s=round(route_s, 1)))
        json.dump(rows, open(os.path.join(args.out, "corpus.json"), "w"),
                  indent=1)
        print(f"{idx:3d} {name[:34]:34s} {str(status):10s} "
              f"{str(conn)}/{str(total)} legal={legal} {route_s:6.1f}s "
              f"in_legal={input_legal}", flush=True)

    ok = sum(1 for r in rows if r.get("status") == "COMPLETE" and r.get("legal"))
    print(f"\nsummary: {len(rows)} boards, {ok} COMPLETE and legal")
    json.dump(rows, open(os.path.join(args.out, "corpus.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
