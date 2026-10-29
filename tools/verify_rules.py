"""Verify that each board's own manufacturability rules are followed.

For a board: import -> analyze (required width/clearance/via per net) ->
strip -> route -> compare the committed copper against the rules, then run the
EM/RF and thermal validators on the routed output.

Usage: python tools/verify_rules.py <board> [<board> ...]
        [--timeout 120] [--threads 4] [--budget-mb 512] [--no-em]
"""

import argparse
import glob
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROUTER = os.path.join(ROOT, "build", "router")
if not os.path.exists(ROUTER):
    ROUTER = os.path.join(ROOT, "build", "router.exe")
WORK = os.path.join(ROOT, "build", "rulecheck")


def run(cmd, timeout):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                           cwd=ROOT)
        return p, False
    except subprocess.TimeoutExpired:
        return None, True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("boards", nargs="+")
    ap.add_argument("--timeout", type=int, default=120)
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--budget-mb", type=int, default=512)
    ap.add_argument("--no-em", action="store_true")
    args = ap.parse_args()
    os.makedirs(WORK, exist_ok=True)

    for idx, board in enumerate(args.boards):
        paths = board if os.path.exists(board) else (glob.glob(board) or [None])[0]
        if not paths:
            print(f"{board}: not found")
            continue
        print(f"\n=== {os.path.basename(paths)}")
        raw = os.path.join(WORK, f"r{idx}_raw.json")
        p, to = run([ROUTER, "import", paths, "--output", raw, "--json"], 600)
        if to or not os.path.exists(raw):
            print("  import failed")
            continue
        b = json.load(open(raw))
        defaults = b.get("defaults") or {}
        print(f"  rules: width={defaults.get('trace_width_mm')} "
              f"clearance={defaults.get('clearance_mm')} "
              f"via={defaults.get('via_outer_mm')}/{defaults.get('via_hole_mm')}")

        # Required widths per net from the analyzer (same resolver the router uses).
        pa, _ = run([ROUTER, "analyze", raw, "--json"], 600)
        required = {}
        if pa and pa.stdout.strip().startswith("{"):
            try:
                ad = json.loads(pa.stdout)
                for n in ad.get("nets", []):
                    required[n.get("name")] = n.get("required_width_mm")
            except Exception:
                pass

        b["traces"] = []
        b["vias"] = []
        b["planes"] = []
        strip = os.path.join(WORK, f"r{idx}_strip.json")
        json.dump(b, open(strip, "w"))
        routed = os.path.join(WORK, f"r{idx}_routed.json")
        rep = os.path.join(WORK, f"r{idx}_rep.json")
        pr, to = run([ROUTER, "route", strip, "--json", "--no-board-echo",
                      "--seed", "42", "--threads", str(args.threads),
                      "--memory-budget-mb", str(args.budget_mb),
                      "--timeout", str(args.timeout), "--output", routed,
                      "--report", rep], args.timeout + 90)
        if to or not os.path.exists(rep):
            print("  route: no report (timeout)")
            continue
        d = json.load(open(rep))
        v = d.get("verification") or {}
        names = {n["id"]: n["name"] for n in b["nets"]}
        widths = {}
        if os.path.exists(routed):
            rb = json.load(open(routed))
            for t in rb.get("traces", []):
                names.setdefault(t["net"], str(t["net"]))
                nm = names.get(t["net"])
                widths.setdefault(nm, []).append(t.get("width_mm"))
        # Width compliance vs the analyzer's requirement.
        thin = []
        for nm, ws in widths.items():
            req = required.get(nm)
            if req is None or not ws:
                continue
            if min(ws) < req - 1e-6:
                thin.append((nm, round(min(ws), 3), round(req, 3)))
        print(f"  route: {d.get('status')} {d.get('connected_terminals')}/"
              f"{d.get('total_terminals')} legal={v.get('legal')} "
              f"violations={len(v.get('violations', []))}")
        print(f"  high-current/rules: {len(widths)} nets routed, "
              f"{len(thin)} below required width")
        for t in thin[:5]:
            print(f"    THIN {t[0]} {t[1]}mm < {t[2]}mm")

        pairs = d.get("diffpairs") or []
        if pairs:
            bad = [x for x in pairs if x.get("status") != "MATERIALIZED"]
            worst = max((abs(x.get("worst_gap_err_mm") or 0) for x in pairs),
                        default=0.0)
            print(f"  diff pairs: {len(pairs)} declared, "
                  f"{len(pairs) - len(bad)} materialized, "
                  f"worst gap error {worst:.4f}mm")
            for x in bad[:3]:
                print(f"    UNMATERIALIZED {x.get('name')}: {x.get('status')}")

        if not args.no_em and os.path.exists(routed):
            prf, to = run(["python", os.path.join(ROOT, "em", "validate_rf.py"),
                           routed, "--json"], 900)
            if not to and prf and prf.stdout.strip().startswith("{"):
                rf = json.loads(prf.stdout)
                print(f"  RF: {rf.get('nets_pass')}/{rf.get('nets_checked')} "
                      f"nets pass (quasi-static)")
            pth, to = run(["python", os.path.join(ROOT, "em", "validate_thermal.py"),
                           routed, "--json"], 900)
            if not to and pth and pth.stdout.strip().startswith("{"):
                th = json.loads(pth.stdout)
                worst = 0.0
                for n in th.get("nets", []):
                    worst = max(worst, n.get("junction_c") or 0.0)
                print(f"  thermal: {th.get('nets_pass')}/{th.get('nets_checked')} "
                      f"nets pass, worst junction {worst:.1f}C")


if __name__ == "__main__":
    main()
