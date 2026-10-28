# Real open-source boards: strip copper and re-route (2026-10-27)

Goal: take boards that are known to be routable (shipped KiCad demos and real
KiCad projects on this machine), delete their copper, and see whether the
router can put it back.

Method:
1. `router import <board> --output raw.json` (new import-only command).
2. Strip `traces`, `vias` and `planes` from the JSON (planes are stripped
   because the importer maps KiCad zone outlines as solid copper, which
   overlaps foreign pads; see limitations).
3. `router verify stripped.json` to measure the DRC of the pads-only board.
4. `router route stripped.json --seed 42` and compare with the source.

## Results

| board | nets | pads | pads-only DRC | source copper (t/v/mm) | status | re-routed (t/v/mm) | connected | legal |
|---|---|---|---|---|---|---|---|---|
| custom_pads_test | 3 | 6 | ok | 19/0/108.3 | **COMPLETE** | 3/0/132.1 | **6/6** | **true** |
| ecc83-pp | 13 | 66 | 4 viol | 59/0/211.0 | VIOLATION | 41/33/284.8 | **66/66** | false |
| pic_programmer | 111 | 470 | 46 viol | 370/6/1745.6 | VIOLATION | 288/259/3528.6 | 454/470 | false |
| flat_hierarchy | 111 | 476 | 48 viol | 371/7/1752.5 | VIOLATION | 277/245/3495.4 | 445/476 | false |
| interf_u | 173 | 684 | 28 viol | 731/84/5101.5 | VIOLATION | 126/150/2240.4 | 298/684 | false |
| dev_board | 322 | 1951 | 156 viol | 256/17/325.8 | VIOLATION | 35/73/2069.3 | 368/1951 | false |
| microwave | 0 | 0 | ok | - | COMPLETE | - | 0/0 | true |

Every board that imported with a clean pads-only DRC routed to a legal,
fully-connected result (custom_pads_test). On the larger boards the router
connected 66/66 (ecc83), 445-454/476 (pic_programmer, flat_hierarchy) but the
run is reported VIOLATION because the **input board already violates DRC**.

## What blocks real-board parity

1. **Pad geometry fidelity.** The importer approximates non-rectangular pads
   by a bounding box. Circles are now rotation-invariant (fixed in this
   change: a rotated circle was inflated by up to sqrt(2)), but ovals,
   roundrects and rotated rectangles are still AABB approximations of a
   larger shape. Example: pic_programmer has 46 pad pairs closer than the
   0.15 mm board clearance in the *pads-only* board, mostly connector and
   TO-92 pads whose real copper is round/oval.
2. **Per-net-class clearance.** Real designs use per-class clearance (KiCad
   defaults 0.2 mm, RF/power classes differ). The verifier applies the board
   default to every foreign pair, so tolerant footprints (e.g. 1.3 mm pads at
   1.27 mm pitch) are flagged.
3. **Copper zones.** The importer maps zone outlines as solid planes, which
   overlap foreign pads (12 violations on ecc83 before stripping). Zones are
   therefore stripped, which removes the intended GND/power connection path
   and makes plane-dependent nets much harder to route (interf_u, dev_board).
4. **Custom/RF footprints.** `microwave` imported zero pads: its footprints
   are not in the supported subset.

## Importer fidelity work (2026-10-28)

Fixed:
- **Circle pads are rotation invariant.** The importer ran every pad through
  the rotated-rectangle bounding-box formula, inflating a round pad by up to
  sqrt(2). Regression test `kicad_rotated_circle_pad_not_inflated`.
- **Pads keep their true size and shape.** `Terminal` now carries
  `shape` (rect/circle/oval/roundrect), `rotation_deg` and
  `corner_r_nm`; `pad_rect()` is only the broadphase AABB. The board JSON
  round-trips these fields.
- **Shape-aware pad-to-pad DRC.** `pad_gap_nm` models a pad as a convex core
  (point/segment/rect) Minkowski-summed with a disc, so circles, capsules
  and rounded rectangles measure exactly. The verifier uses it for
  pad-vs-pad clearance. Tests: `pad_gap_circles_are_not_squares`,
  `pad_gap_roundrect_uses_corner_radius` (1.3mm round pads at 1.27mm
  diagonal now measure 0.496mm instead of 0.000mm).

Effect on the pads-only DRC of the KiCad demos: pic_programmer 46 -> 26,
flat_hierarchy 48 -> 30, dev_board 156 -> 51 pad pairs flagged (with the
earlier, pre-shape model).

## Open issue: pad orientation on rotated footprints

KiCad's own DRC (`kicad-cli pcb drc`) reports **0 clearance violations** and
0 unconnected items on pic_programmer (its 63 findings are all
`lib_footprint_issues`, i.e. library-link noise). Our importer still reports
pad pairs below the 0.15mm board default on these boards, so some of the
pad transform is still wrong. The two conventions tried disagree:

- pad angle absolute: fewer false positives on these boards, but the DIP-14
  `U2` pads (footprint 90 deg, pad 90 deg, oval 2.4x1.6) end up vertical and
  appear to overlap the neighbouring diode;
- pad angle footprint-relative (`frot + pad`): `U2` is fine, but the P3
  connector pads (oval 1.6x2.8, pad angle 270) end up rotated into their
  neighbours.

Neither is self-consistent, so the bug is probably in how the pad's local
`size` axis and the rotation are combined, or in the pad offset rotation.
Resolving it needs an independent oracle: export F.Cu as Gerber (or
IPC-2581) with `kicad-cli` and compare flash positions/apertures against our
import. That is the next step; until then the importer is **not** claimed to
match KiCad's pad geometry, and re-routing real boards still starts from a
slightly wrong copper model.


## Honest verdict

The router is not yet able to reproduce real KiCad routing on these boards,
and the dominant blocker is **import fidelity**, not the routing engine:
- where the import is clean, routing completes legally;
- where it is not, the router is asked to route a geometrically wrong board.

Next work, in order of impact: finish the pad transform (orientation oracle
above), per-class clearance import, and zone-fill import so planes can be
kept rather than stripped.
