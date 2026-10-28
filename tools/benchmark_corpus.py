"""Score a corpus run: the fair denominator is boards whose pads-only input
already verifies legal (a board that is DRC-illegal before routing cannot
judge the router).

Usage: python tools/benchmark_corpus.py <corpus.json> [--target 0.70]
"""

import argparse
import json
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("corpus", help="corpus.json produced by corpus_harness.py")
    ap.add_argument("--target", type=float, default=0.70)
    args = ap.parse_args()

    rows = json.load(open(args.corpus))
    eligible = [r for r in rows if r.get("input_legal") is True]
    excluded_illegal = sum(1 for r in rows if r.get("input_legal") is False)
    excluded_import = sum(1 for r in rows if r.get("status") == "IMPORT_FAILED")

    complete = [r for r in eligible if r.get("status") == "COMPLETE"
                and r.get("legal")]
    legal = [r for r in eligible if r.get("legal")]
    illegal = [r for r in eligible if r.get("legal") is False
               and (r.get("violations") or 0) > 0]

    print(f"boards recorded:      {len(rows)}")
    print(f"  excluded (import):  {excluded_import}")
    print(f"  excluded (input DRC illegal): {excluded_illegal}")
    print(f"eligible (input legal): {len(eligible)}")
    if not eligible:
        print("no eligible boards")
        return 1
    rate = len(complete) / len(eligible)
    print(f"  COMPLETE + legal:   {len(complete)} / {len(eligible)} "
          f"({rate * 100:.1f}%)")
    print(f"  legal copper (no violations): {len(legal)} / {len(eligible)} "
          f"({len(legal) / len(eligible) * 100:.1f}%)")
    print(f"  illegal copper committed: {len(illegal)}")
    print(f"  target: {args.target * 100:.0f}% -> "
          f"{'PASS' if rate >= args.target else 'FAIL'}")
    if illegal:
        print("  boards with illegal copper:")
        for r in illegal[:10]:
            print(f"    {r['board'][:40]:40s} viol={r.get('violations')}")
    short = [r for r in eligible
             if r.get("status") in ("TIMEOUT", "INCOMPLETE")]
    if short:
        print("  legal but incomplete/timeout:")
        for r in short[:12]:
            print(f"    {r['board'][:40]:40s} {r.get('status'):10s} "
                  f"{r.get('connected')}/{r.get('total')}")
    return 0 if rate >= args.target else 2


if __name__ == "__main__":
    sys.exit(main())
