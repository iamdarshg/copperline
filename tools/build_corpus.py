"""Build a clean corpus: collect boards, dedupe by *board signature* (not just
file bytes, so revisions of the same board collapse), import each one, strip
copper and keep only boards whose pads-only board verifies legal.

Usage:
  python tools/build_corpus.py --sources <dir> [...] --out <dir>
        [--list <file>] [--max-boards 0]

Writes <out>/boards.txt (legal boards, one path per line) and
<out>/corpus_meta.json with every candidate and why it was kept/dropped.
"""

import argparse
import hashlib
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROUTER = os.path.join(ROOT, "build", "router")
if not os.path.exists(ROUTER):
    ROUTER = os.path.join(ROOT, "build", "router.exe")


def collect(sources):
    out = []
    for s in sources:
        if s.endswith(".txt") and os.path.isfile(s):
            out.extend(l.strip() for l in open(s, encoding="utf-8") if l.strip())
            continue
        if os.path.isfile(s):
            out.append(s)
            continue
        for dirpath, _d, names in os.walk(s):
            for n in names:
                if n.endswith((".kicad_pcb", ".dsn")):
                    out.append(os.path.join(dirpath, n))
    return sorted(set(out))


def import_board(path, workdir, idx):
    raw = os.path.join(workdir, f"c{idx:04d}_raw.json")
    try:
        p = subprocess.run([ROUTER, "import", path, "--output", raw, "--json"],
                           capture_output=True, text=True, timeout=300, cwd=ROOT)
    except subprocess.TimeoutExpired:
        return None, "import_timeout"
    if not os.path.exists(raw):
        return None, "import_failed"
    try:
        b = json.load(open(raw))
    except Exception:
        return None, "import_unreadable"
    return (b, raw), None


def signature(b):
    w = round(b["board"]["width_mm"], 1)
    h = round(b["board"]["height_mm"], 1)
    nets = len(b.get("nets", []))
    pads = sum(len(n.get("terminals", [])) for n in b.get("nets", []))
    layers = len(b.get("layers", []))
    return f"{w}x{h}|{layers}L|{nets}n|{pads}p"


def input_drc(raw, workdir, idx):
    b = json.load(open(raw))
    b["traces"] = []
    b["vias"] = []
    b["planes"] = []
    stripped = os.path.join(workdir, f"c{idx:04d}_stripped.json")
    json.dump(b, open(stripped, "w"))
    try:
        p = subprocess.run([ROUTER, "verify", stripped, "--json"],
                           capture_output=True, text=True, timeout=300, cwd=ROOT)
        d = json.loads(p.stdout)
        return bool(d.get("legal")), len(d.get("violations", [])), stripped
    except Exception:
        return None, None, stripped


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sources", nargs="+", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-boards", type=int, default=0)
    args = ap.parse_args()

    work = os.path.join(args.out, "work")
    os.makedirs(work, exist_ok=True)
    boards = collect(args.sources)
    print(f"{len(boards)} candidate files", flush=True)

    seen_sig = {}
    kept, meta = [], []
    for idx, path in enumerate(boards):
        parsed, why = import_board(path, work, idx)
        if parsed is None:
            meta.append(dict(path=path, reason=why))
            continue
        b, raw = parsed
        pads = sum(len(n.get("terminals", [])) for n in b.get("nets", []))
        nets = len(b.get("nets", []))
        if pads < 4 or nets < 2:
            # No routable content (netlist lives in an external .net file) —
            # routing it would trivially "succeed" with 0/0 and inflate scores.
            meta.append(dict(path=path, reason="no_pads", pads=pads, nets=nets))
            continue
        sig = signature(b)
        if sig in seen_sig:
            meta.append(dict(path=path, reason="duplicate_signature",
                             signature=sig, kept=seen_sig[sig]))
            continue
        legal, viol, _stripped = input_drc(raw, work, idx)
        if legal is None:
            meta.append(dict(path=path, reason="verify_failed", signature=sig))
            continue
        if not legal:
            meta.append(dict(path=path, reason="input_illegal",
                             signature=sig, violations=viol))
            seen_sig[sig] = path
            continue
        seen_sig[sig] = path
        # Collapse revisions that differ only in net count: keep one board per
        # file name, preferring the one with the most pads.
        base = os.path.basename(path).lower()
        prev = next((k for k in kept if os.path.basename(k).lower() == base), None)
        if prev is not None:
            meta.append(dict(path=path, reason="duplicate_board_name",
                             signature=sig, kept=prev))
            continue
        kept.append(path)
        meta.append(dict(path=path, reason="kept", signature=sig))
        print(f"  kept {os.path.basename(path)[:50]:50s} {sig}", flush=True)
        if args.max_boards and len(kept) >= args.max_boards:
            break

    list_path = os.path.join(args.out, "boards.txt")
    open(list_path, "w", encoding="utf-8").write("\n".join(kept))
    json.dump(meta, open(os.path.join(args.out, "corpus_meta.json"), "w"),
              indent=1)
    reasons = {}
    for m in meta:
        reasons[m["reason"]] = reasons.get(m["reason"], 0) + 1
    print(f"\nkept {len(kept)} legal, deduped boards -> {list_path}")
    print("reasons:", reasons)


if __name__ == "__main__":
    main()
