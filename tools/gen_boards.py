"""Generate a spread of synthetic boards for the corpus harness.

Families: breakout (few nets), grid (cells), dense BGA (0.6-0.8mm pitch),
board-scale (long detours). Every board is DRC-legal by construction (pads
are placed on a grid with clearance for the configured pad size), so a
routing failure cannot be blamed on the fixture.

Usage: python tools/gen_boards.py --out <dir>
"""

import argparse
import json
import math
import os


def board_doc(layers, w, h, width, clear, via):
    return {
        "schema": "copperline/board/1",
        "board": {"width_mm": round(w, 3), "height_mm": round(h, 3)},
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


def breakout(pairs, pitch=2.54, pad=1.0):
    """Two vertical columns of pads: J1 on the left, J2 on the right."""
    rows = pairs
    w = 20.0
    h = rows * pitch + 6.0
    doc = board_doc(2, w, h, 0.25, 0.2, 0.6)
    for k in range(pairs):
        y = 3.0 + k * pitch
        add_net(doc, f"N{k}", [(4.0, y, "J1", str(k), pad),
                               (w - 4.0, y, "J2", str(k), pad)])
    renumber(doc)
    return doc


def grid(nets, cell=5.08, pad=0.8, layers=2):
    """Cells on a 5.08mm lattice; pads sit at opposite corners of a cell."""
    cols = max(1, int(math.ceil(math.sqrt(nets))))
    rows = (nets + cols - 1) // cols
    w = cols * cell + 4.0
    h = rows * cell + 4.0
    doc = board_doc(layers, w, h, 0.2, 0.2, 0.5)
    for k in range(nets):
        cx = 3.0 + (k % cols) * cell
        cy = 3.0 + (k // cols) * cell
        add_net(doc, f"G{k}", [(cx, cy, "U1", str(k), pad),
                               (cx + 2.54, cy + 2.54, "U2", str(k), pad)])
    renumber(doc)
    return doc


def dense_bga(n, layers=6, pitch=0.8, pad=0.4, width=0.1, clear=0.1,
              via=0.45):
    """Centred BGA cluster plus a perimeter fan-out field."""
    cluster = n * pitch
    per_row = max(1, (n * n + 7) // 8)
    rows = (n * n + per_row - 1) // per_row
    field_w = (per_row - 1) * 2.0 + 2.0
    field_h = (rows - 1) * 2.0 + 2.0
    w = max(cluster + 16.0, field_w + 8.0)
    h = 6.0 + cluster + 6.0 + field_h
    doc = board_doc(layers, w, h, width, clear, via)
    cx0 = w / 2.0
    cy0 = 6.0 + cluster / 2.0
    k = 0
    for row in range(n):
        for col in range(n):
            px = cx0 + (col - (n - 1) / 2.0) * pitch
            py = cy0 + (row - (n - 1) / 2.0) * pitch
            fx = 4.0 + (k % per_row) * 2.0
            fy = 6.0 + cluster + 6.0 + (k // per_row) * 2.0
            add_net(doc, f"B{row}_{col}", [(px, py, "U1", f"{row}{col}", pad),
                                           (fx, fy, "J1", str(k), 0.6)])
            k += 1
    renumber(doc)
    return doc


def board_scale(cols, rows, layers=2):
    """Independent nets whose two pads are separated by a wall gap."""
    cw, ch = 12.0, 8.0
    doc = board_doc(layers, cols * cw, rows * ch, 0.2, 0.2, 0.6)
    for r in range(rows):
        for c in range(cols):
            x0, y0 = c * cw, r * ch
            add_net(doc, f"S{r}_{c}", [(x0 + 2.0, y0 + 4.0, "U1", str(c), 0.8),
                                       (x0 + cw - 2.0, y0 + 4.0, "U2", str(c), 0.8)])
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
    for pairs in (2, 4, 8, 16, 32):
        write(args.out, f"breakout_{pairs}", breakout(pairs))
    for nets in (8, 24, 64, 128):
        write(args.out, f"grid_{nets}", grid(nets))
    for n, layers in ((6, 4), (10, 6), (14, 12)):
        write(args.out, f"bga_{n}x{n}_{layers}l", dense_bga(n, layers))
    for cols, rows in ((8, 2), (16, 4), (32, 6)):
        write(args.out, f"scale_{cols}x{rows}", board_scale(cols, rows))


if __name__ == "__main__":
    main()
