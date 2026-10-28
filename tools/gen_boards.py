"""Generate a spread of synthetic boards for the corpus harness.

Families: breakout (few nets), mid (grid), dense BGA (0.6-0.8mm pitch),
board-scale (long detours). Deterministic, no external inputs, so CI can run
the same corpus anywhere.

Usage: python tools/gen_boards.py --out <dir> [--count 20]
"""

import argparse
import json
import os


def board_doc(layers, w, h, width, clear, via):
    return {
        "schema": "copperline/board/1",
        "board": {"width_mm": w, "height_mm": h},
        "layers": [{"id": i, "name": ("Top" if i == 0 else
                                      "Bottom" if i == layers - 1
                                      else f"In{i}")}
                   for i in range(layers)],
        "defaults": {"trace_width_mm": width, "clearance_mm": clear,
                     "via_outer_mm": via, "via_hole_mm": 0.2,
                     "default_current_a": 0.5},
        "nets": [],
    }


def add_net(doc, name, terms, **kw):
    n = {"id": len(doc["nets"]), "name": name, "terminals": []}
    n.update(kw)
    doc["nets"].append(n)
    for (x, y, comp, pin, pad) in terms:
        n["terminals"].append({
            "id": 0, "x_mm": round(x, 4), "y_mm": round(y, 4), "layer": 0,
            "pad_w_mm": pad, "pad_h_mm": pad, "shape": "circle",
            "component": comp, "pin": pin})
    return n


def renumber(doc):
    tid = 0
    for n in doc["nets"]:
        for t in n["terminals"]:
            t["id"] = tid
            tid += 1


def breakout(pairs, spacing=2.54, pad=1.2):
    rows = (pairs + 9) // 10
    w = 10 * spacing + 4
    h = rows * spacing + 8
    doc = board_doc(2, w, h, 0.25, 0.2, 0.6)
    for k in range(pairs):
        x0 = 2 + (k % 10) * spacing
        y0 = 4 + (k // 10) * spacing
        add_net(doc, f"N{k}", [(x0, y0, "J1", str(k), pad),
                               (x0 + 6.0, y0, "J2", str(k), pad)])
    renumber(doc)
    return doc


def grid(nets, pitch=1.27, layers=2):
    w = 10 * pitch + 6
    h = ((nets + 9) // 10) * pitch + 6
    doc = board_doc(layers, w, h, 0.15, 0.15, 0.5)
    for k in range(nets):
        x0 = 3 + (k % 10) * pitch
        y0 = 3 + (k // 10) * pitch
        add_net(doc, f"G{k}", [(x0, y0, "U1", str(k), 0.5),
                               (x0 + 0.9 * pitch, y0 + 0.9 * pitch, "U2",
                                str(k), 0.5)])
    renumber(doc)
    return doc


def dense_bga(n, layers=6, pitch=0.8, pad=0.4, width=0.1, clear=0.1,
              via=0.45):
    span = n * 1.5 + 10
    doc = board_doc(layers, span, span, width, clear, via)
    per_row = max(1, (n * n + 7) // 8)
    k = 0
    for row in range(n):
        for col in range(n):
            cx = span / 2 + (col - (n - 1) / 2) * pitch
            cy = span / 2 + (row - (n - 1) / 2) * pitch
            px = 3 + (k % per_row) * 1.5
            py = 3 + (k // per_row) * 1.5
            add_net(doc, f"B{row}_{col}", [(cx, cy, "U1", f"{row}{col}", pad),
                                           (px, py, "J1", str(k), 0.6)])
            k += 1
    renumber(doc)
    return doc


def board_scale(cols, rows, layers=2):
    cw, ch = 12.0, 8.0
    doc = board_doc(layers, cols * cw, rows * ch, 0.2, 0.2, 0.6)
    for r in range(rows):
        for c in range(cols):
            x0, y0 = c * cw, r * ch
            # A wall between the two pads forces a detour.
            add_net(doc, f"S{r}_{c}", [(x0 + 2, y0 + 4, "U1", str(c), 0.8),
                                       (x0 + cw - 2, y0 + 4, "U2", str(c), 0.8)])
    renumber(doc)
    return doc


def write(out, name, doc):
    path = os.path.join(out, name + ".json")
    json.dump(doc, open(path, "w"), indent=0)
    pads = sum(len(n["terminals"]) for n in doc["nets"])
    print(f"{name:24s} {len(doc['nets']):5d} nets {pads:5d} pads "
          f"{len(doc['layers']):2d} layers")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    for k, pairs in enumerate((2, 4, 8, 16, 32)):
        write(args.out, f"breakout_{pairs}", breakout(pairs))
    for k, nets in enumerate((8, 24, 64)):
        write(args.out, f"grid_{nets}", grid(nets))
    for n, layers in ((6, 4), (10, 6), (14, 12)):
        write(args.out, f"bga_{n}x{n}_{layers}l", dense_bga(n, layers))
    for cols, rows in ((8, 2), (16, 4), (32, 6)):
        write(args.out, f"scale_{cols}x{rows}", board_scale(cols, rows))


if __name__ == "__main__":
    main()
