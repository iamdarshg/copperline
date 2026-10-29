#!/usr/bin/env python3
"""Classify why corpus boards did not reach COMPLETE.

Reads the `corpus.json` written by `tools/corpus_harness.py` (and the
`bNNN_report.json` files next to it when present) and separates:

  ok            COMPLETE and legal
  ILLEGAL       finished with hard-rule violations (never acceptable)
  clock         used the whole wall-clock cap
  search-budget stopped because the engine's expansion/time budget ran out
  gave-up       INCOMPLETE with a large slice of the wall clock unused
  geometry      a meaningful share of pads has no legal escape at all
                (escape reason `no_exit_sectors`)

The last two columns of the detailed view answer the question "is this board
actually impossible, or did the router stop trying?":

  infeas   pads with no legal escape (escape.pads_infeasible)
  ripups   recovery rip-ups actually committed (recovery.ripups)

Usage:
  python tools/classify_corpus.py build/bench/run/corpus.json --timeout 180
"""
import argparse
import glob
import json
import os


def load_reports(out_dir):
    """Index bNNN_report.json by the order they were produced."""
    reports = {}
    for path in sorted(glob.glob(os.path.join(out_dir, "b0*_report.json"))):
        idx = int(os.path.basename(path)[1:4])
        try:
            reports[idx] = json.load(open(path))
        except Exception:
            pass
    return reports


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("corpus", help="corpus.json or the run directory holding it")
    ap.add_argument("--timeout", type=int, default=180, help="per-board cap in seconds")
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    args = ap.parse_args()

    path = args.corpus
    out_dir = os.path.dirname(os.path.abspath(path)) if path.endswith(".json") else path
    if os.path.isdir(path):
        path = os.path.join(path, "corpus.json")
    rows = json.load(open(path))
    reports = load_reports(out_dir)

    result = []
    for i, r in enumerate(rows):
        rep = reports.get(i) or {}
        esc = rep.get("escape") or {}
        rec = rep.get("recovery") or {}
        stats = rep.get("stats") or {}
        pads = esc.get("pads_total") or r.get("pads") or 0
        infeasible = esc.get("pads_infeasible") or 0
        route_s = r.get("route_s") or 0.0
        status = r.get("status") or "?"
        viol = r.get("violations")
        connected, total = r.get("connected"), r.get("total")

        if status == "COMPLETE" and (viol == 0):
            verdict = "ok"
        elif viol:
            verdict = "ILLEGAL"
        elif status == "TIMEOUT" or route_s >= 0.95 * args.timeout:
            verdict = "clock"
        elif status == "BUDGET_EXHAUSTED":
            verdict = "search-budget"
        elif route_s < 0.7 * args.timeout:
            verdict = "gave-up"
        else:
            verdict = "clock"
        # A board whose pads cannot escape is limited by its own rules, not by
        # search effort; report that alongside the timing verdict.
        if pads and infeasible * 100 >= 2 * pads and verdict != "ok":
            verdict += "+geometry"

        result.append(dict(
            board=r.get("board"), status=status, verdict=verdict,
            connected=connected, total=total, violations=viol,
            route_s=route_s, unused_s=round(max(0.0, args.timeout - route_s), 1),
            pads=pads, infeasible=infeasible,
            epochs=stats.get("epochs"), rearm=stats.get("deferred_retries"),
            gens=rec.get("generations"), ripups=rec.get("ripups"),
            modes=rec.get("modes_attempted") or [],
        ))

    if args.json:
        print(json.dumps(result, indent=1))
        return

    print(f"{'board':34s} {'verdict':16s} {'conn':>9s} {'viol':>4s} "
          f"{'wall':>6s} {'unused':>6s} {'infeas':>7s} {'rip':>3s}  modes")
    for x in result:
        conn = f"{x['connected']}/{x['total']}"
        infeas = f"{x['infeasible']}/{x['pads']}" if x["pads"] else "-"
        modes = ",".join(m.replace("_RECOVERY", "") for m in x["modes"])
        print(f"{(x['board'] or '?')[:34]:34s} {x['verdict']:16s} {conn:>9s} "
              f"{str(x['violations']):>4s} {x['route_s']:6.1f} {x['unused_s']:6.1f} "
              f"{infeas:>7s} {str(x['ripups']):>3s}  {modes}")

    counts = {}
    for x in result:
        counts[x["verdict"]] = counts.get(x["verdict"], 0) + 1
    n = len(result)
    ok = counts.get("ok", 0)
    print()
    print(f"{ok}/{n} COMPLETE+legal ({100.0 * ok / max(1, n):.1f}%)")
    for k in sorted(counts):
        if k != "ok":
            print(f"  {k:16s} {counts[k]}")


if __name__ == "__main__":
    main()
