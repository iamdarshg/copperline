# CircuitMaker / Altium PCB Format Notes (clean-room)

These notes describe **observed and publicly-documented facts** about the
file formats that CircuitMaker 2.x (and Altium Designer, CircuitStudio,
SolidWorks PCB) use, relevant to PCB autorouting and to copperline's format
adapters. Everything here is written from documentation (KiCad dev-docs,
BoardRipper, AltiumSharp, altium2kicad issue #28, Specctra DSN spec,
Altium's own interfacing docs) and from **byte-level observations of the
installer manifest** — no proprietary source code is reproduced.

---

## 1. Container: Microsoft Compound File Binary (CFB / OLE2)

All binary Altium-family design files are CFB "structured storage" documents.

| File type | Container | Extension |
|---|---|---|
| Altium Designer PCB | CFB | `.PcbDoc` |
| **CircuitMaker PCB** | **CFB (same format)** | **`.CMPcbDoc`** |
| CircuitStudio PCB | CFB (same format) | `.CSPcbDoc` |
| SolidWorks PCB | CFB (same format) | `.SWPcbDoc` |
| Schematic | CFB **or** ASCII `|HEADER=` text | `.SchDoc` |
| Schematic library | CFB | `.SchLib` |
| Footprint library | CFB | `.PcbLib` |
| Integrated library | CFB (zlib streams) | `.IntLib` |
| Project | ASCII INI-style text | `.PrjPcb` |

**Signature (magic bytes):**

```
D0 CF 11 E0 A1 B1 1A E1     ; CFB header (MS-CFB)
```

Observed Altium specifics (from documentation and fixture analysis):

- Sector size 512 bytes (CFB V3). Altium rejects V4 (4096-byte) files.
- Inside the CFB tree, each object type is a **storage** (directory) named
  `<Type>6` (or legacy plural), containing:
  - `Header` stream — one `uint32` record count
  - `Data` stream — the records
- String encoding inside records: Windows-1252; lengths are block-prefixed
  (uint32) and **include** the trailing `\0`.

Known PCB storages (name → parse style):

| Storage | Style | Contents |
|---|---|---|
| `FileHeader` | special | binary version string (e.g. `PCB 6.0 …`) |
| `Board6` | text props | board outline, stackup, sheet |
| `Components6` | text props | footprint instances (placements) |
| `Nets6` | text props | net definitions |
| `Classes6` | text props | net/component classes |
| `Rules6` | text props | design rules (see §6) |
| `Polygons6` | text props | copper pour definitions |
| `Dimensions6` | text props | dimension annotations |
| `Models` / `ModelsNoEmbed` | text props | 3D model refs / embedded STEP |
| `ExtendedPrimitiveInformation` | text props | per-primitive masks |
| `Tracks6` | binary | track primitives |
| `Arcs6` | binary | arc primitives |
| `Vias6` | binary | via primitives |
| `Pads6` | binary | pad primitives (multi-subrecord) |
| `Fills6` | binary | rectangular fills |
| `Texts6` | binary | text primitives |
| `Regions6` | binary | copper region fill results |
| `ShapeBasedRegions6` | binary | keepouts / cutouts / cavities |
| `BoardRegions` | binary | board region definitions |
| `ComponentBodies6` | binary+props | 3D bodies |
| `WideStrings6` | binary | UTF-16 string table |

(Unused by importers but present: `Connections6`, `FromTos6`,
`DifferentialPairs6`, `PadViaLibrary*`, `PinSwapOptions6`, `SmartUnions`,
`UnionNames`, `UniqueIDPrimitiveInformation`, `Textures`, `Embeddeds6`,
`EmbeddedBoards6`, `EmbeddedFonts6`, `SignalClasses`, `PinPairsSection`,
`Coordinates6`, `FileVersionInfo`.)

Directory tree example (PcbDoc = CFB):

```
+-- FileHeader/
+-- Arcs6/  (Header: u32 n; Data: n binary arc records)
+-- Board6/ (Header; Data: |KEY=VALUE|...| prop records)
+-- Components6/ ...
+-- Nets6/ ...
+-- Pads6/  ...
+-- Tracks6/ ...
+-- Vias6/  ...
+-- WideStrings6/ (Data only: UTF-16 table)
+-- Models/ (Data + numbered compressed STEP streams 0,1,2…)
```

---

## 2. Text property-stream encoding (Board6, Components6, Nets6, Classes6, Rules6, Polygons6, Dimensions6, Models)

- Each record: `uint32 length` followed by ASCII bytes:
  `|KEY=VALUE|KEY=VALUE|…|` with a trailing `\0`.
- Record length includes the NUL terminator.
- Boolean values are literal `TRUE` / `FALSE`.
- Coordinates in text records are **mils as strings** (e.g. `"100mil"`).
- Unicode strings are referenced by index into `WideStrings6`.

Example shapes (publicly documented):

```
|RECORD=Component|SOURCEDESIGNATOR=R1|PATTERN=0603|LAYER=TOP|X=100000|Y=200000|ROTATION=0|
|RECORD=Net|NETHAME=N|...|
```

Schematic variant (`|HEADER=…` or CFB FileHeader): records `|RECORD=<n>|…`,
where 0=HEADER, 1=COMPONENT, 2=PIN, 4=LABEL, 5=BEZIER, 6=POLYLINE,
7=POLYGON, 8=ELLIPSE, 12=ARC, 13=LINE, 14=RECTANGLE, 15=SHEET_SYMBOL,
16=SHEET_ENTRY, 17=POWER_PORT, 18=PORT, 25=NET_LABEL, 26=BUS, 27=WIRE,
28=TEXT_FRAME, 29=JUNCTION, 30=IMAGE, 31=SHEET, 32=SHEET_NAME, 33=FILE_NAME,
34=DESIGNATOR, 37=BUS_ENTRY, 39=TEMPLATE, 41=PARAMETER, 44=IMPLEMENTATION_LIST,
45=IMPLEMENTATION, 48=IMPL_PARAMS, 209=NOTE, 226=HYPERLINK.

---

## 3. Binary record-stream encoding

Each binary record:

```
uint8  record_type
uint32 subrecord_length      ; length of everything after this field
... payload ...
```

Record type IDs (observed in all binary PCB streams):

| ID | Type |
|---|---|
| 1 | ARC |
| 2 | PAD |
| 3 | VIA |
| 4 | TRACK |
| 5 | TEXT |
| 6 | FILL |
| 11 | REGION |
| 12 | MODEL |

### Track (Tracks6) — record_type 4

```
u8   layer_v6            ; legacy layer ID (1 top … 32 bottom, 74 multi)
u8   flags1              ; bit1 set = polygon outline, bit2 clear = locked
u8   flags2              ; 2 = keepout
u16  net                 ; 0xFFFF = none
u16  polygon             ; owning pour, 0xFFFF = none
u16  component           ; owning component, 0xFFFF = none
[4]  padding
i32  start_x, start_y
i32  end_x, end_y
i32  width
u16  subpolyindex
u8   unknown
[V7+]: 5 pad + u32 layer_v7   ; newer files
[V8+]: u8 keepout_restrictions
```

### Arc (Arcs6) — record_type 1

Same header as track, then:

```
i32   center_x, center_y
i32   radius
f64   start_angle(deg)
f64   end_angle(deg)
i32   width
u16   subpolyindex
[V7/V8 extension fields]
```

### Via (Vias6) — record_type 3

```
u8   unknown
u8   flags1              ; bit7 testfab top, bit6 tent bot, bit5 tent top, bit2 clear = locked
u8   flags2
u16  net
[8]  padding
i32  position_x, position_y
i32  diameter
i32  hole_size
u8   layer_start
u8   layer_end
[extended when subrecord_length > 74: thermal relief, solder mask expansion, pad mode, per-layer diameters[32]]
```

### Pad (Pads6) — record_type 2, six subrecords

```
Subrecord 1: pad name (Pascal string)
Subrecord 2-4: unknown (skipped)
Subrecord 5 (>=110 B): main geometry
  u8    layer_v6, u8 flags1, u8 flags2, u16 net
  [2] pad, u16 component, [4] pad
  i32   pos_x, pos_y
  i32   top_size_x/y, mid_size_x/y, bot_size_x/y
  i32   hole_size
  u8    top/mid/bot shape   ; 1=circle, 2=rect, 3=octagon, 9=rounded rect
  f64   direction (deg)
  u8    plated, [1] pad, u8 pad_mode   ; 0=simple,1=top/mid/bot,2=full stack
  [23] pad
  i32   paste_mask_expansion, solder_mask_expansion
  [7] pad, u8 paste_mask_mode, u8 solder_mask_mode
  ... hole rotation, layer span, pad-to-die ...
Subrecord 6 (>=596 B): per-layer arrays
  i32  inner_size_x[29], inner_size_y[29]
  u8   inner_shape[29]
  [1] pad, u8 hole_shape, i32 slot_size, f64 slot_rotation
  i32  hole_offset_x[32], hole_offset_y[32]
  [1] pad, u8 alt_shape[32], u8 corner_radius[32]
```

### Fill (Fills6) — record_type 6

```
u8   layer, u8 flags1, u8 flags2, u16 net, [2] pad, u16 component, [4] pad
i32  pos1_x, pos1_y, pos2_x, pos2_y
f64  rotation (deg)
```

### Text (Texts6) — record_type 5

```
u8   layer_v6, [6] pad, u16 component, [4] pad
i32  pos_x, pos_y, height
u16  stroke_font_type    ; 1=default, 2=sans, 3=serif
f64  rotation; u8 mirrored; i32 stroke_width
[>=123 B adds: font type 0=stroke/1=TrueType/2=barcode, bold/italic,
 64B UTF-16LE font name, inverted flag, box dims+justify,
 widestring table index, barcode params, layer_v7]
```

### WideStrings6 (string table)

```
u32  index
u32  byte_length          ; incl. NUL (0 or 2 for empty)
<byte_length> UTF-16LE bytes
```

---

## 4. Units and coordinate system

- Binary coordinates are **int32** in units of **1/10000 mil** = 2.54 nm.
  → `nm = value * 2.54`; `mm = value / 39370.0787` (see altium-parser
  BINARY_MIL_TO_MM = 1/39.37/10000).
- Text-format coordinates are mil strings (often with `_FRAC` fractional
  counterparts in schematics).
- Origin bottom-left, Y axis **up** (KiCad flips Y on import).
- File resolution (Specctra DSN): `(resolution <unit> <value>)` —
  e.g. `(resolution mil 10)`; a robust DSN importer must scale by it.

## 5. Layer numbering

V6 (1 byte):

| ID | Layer |
|---|---|
| 1 | Top |
| 2..31 | Mid 1..30 |
| 32 | Bottom |
| 33/34 | Top/Bottom Overlay |
| 35/36 | Top/Bottom Paste |
| 37/38 | Top/Bottom Solder |
| 39..54 | Internal Plane 1..16 |
| 55 | Drill Guide |
| 56 | Keep-Out |
| 57..72 | Mechanical 1..16 |
| 73 | Drill Drawing |
| 74 | Multi-Layer |
| 75..82 | System (connections, background, DRC, grids, holes) |

V7 (32-bit): `0x01000000 + n` (1=top, 65535=bottom),
`0x01020000 + n` = Mechanical n (unbounded count).
V8: `0x01030000 + k` for overlay/paste/solder/drill/keepout/multi/system IDs.

Mechanical layer semantic kinds (from `MECHKIND` prop): Board=0x04,
CourtyardTop/Bottom=0x0B/0x0C, DesignatorTop/Bottom=0x0D/0x0E,
FabNotes=0x12, VCut=0x19, RouteToolPath=0x1C, BoardShape=0x1E, etc.

## 6. Design rules (Rules6)

Rules are text-prop records keyed by `RULEKIND`:

| RULEKIND | keys |
|---|---|
| Clearance | `GAP` |
| Width | `MINLIMIT`, `MAXLIMIT`, `PREFERREDWIDTH` |
| RoutingVias | `WIDTH`, `MINWIDTH`, `MAXWIDTH`, `HOLEWIDTH`, `MINHOLEWIDTH`, `MAXHOLEWIDTH` |
| HoleSize | `MINLIMIT`, `MAXLIMIT` |
| HoleToHoleClearance | `GAP` |
| PasteMaskExpansion / SolderMaskExpansion | `EXPANSION` |
| PlaneClearance | `CLEARANCE` |
| PolygonConnect | `CONNECTSTYLE`, `AIRGAPWIDTH`, `RELIEFCONDUCTORWIDTH`, `RELIEFENTRIES` |
| DiffPairsRouting | differential pair params |

Common: `NAME`, `PRIORITY`, `SCOPE1EXPRESSION`, `SCOPE2EXPRESSION`.

## 7. Board outline (Board6)

Keys: `SHEETX`, `SHEETY`, `SHEETWIDTH`, `SHEETHEIGHT`, `LAYERSETSCOUNT`,
`LAYER<n>NAME/COPTHICK/DIELCONST/DIELHEIGHT/DIELMATERIAL/MECHENABLED/MECHKIND`,
outline vertices `VX<n>/VY<n>`, vertex kind `KIND<n>` (0=line, else arc with
`R<n>` radius, `SA<n>/EA<n>` angles, `CX<n>/CY<n>` center). V7: `LAYERV7_*`
prefixed; `LAYER<n>LAYERID` carries the 32-bit layer id.

---

## 8. Specctra DSN ↔ copperline mapping

Specctra DSN (what Altium/CM write for an external autorouter) is an
S-expression ASCII file. Primary sections:

```
(pcb <name>
  (resolution mil 10)
  (structure
    (layer Top (type signal)) ...
    (boundary (rect x1 y1 x2 y2) | (polygon x1 y1 x2 y2 ...))
    (via STD dia hole) ...
    (rule (width w) (clearance c)) ...
    (copper_pour ...) (keepout ...) (plane ...) (wiring ...))
  (placement (component <ref> (place <part> x y front|back rot)) ...)
  (library (image <part> (pin <n> x y) ... (padstack ...)) ...)
  (network (net N1 (pins U1-1 U2-1) (class C)) ... )
)
```

copperline's `src/dsn.cpp` currently imports this subset:

- `(unit mm)` — note: real files use `(resolution …)` too → **gap**.
- `(structure (layer (type…)))` — ✓
- `(boundary rect|polygon)` — ✓
- `(via name dia hole)` — ✓
- `(rule width clearance)` for default + per-class — ✓
- `(plane)`/`(copper_pour)` → `PlaneZone` — ✓ (polygon/rect/circle/path)
- keepouts (`wire_keepout`, `via_keepout`, `place_keepout`, `bend_keepout`,
  `elongate_keepout`, `keepout`) → `Keepout` rect by bbox — ✓ (warned)
- `network/class/net/pins` — ✓; `library/image/pin` + `placement` → terminals — ✓
- `(wiring …)` pre-routes → skipped with warning — documented ✓
- microvia/blind/buried → recorded-warning — honest ✓

SES output (`board_to_ses`) emits:

```
(session name (base_design name) (unit mm)
  (route (library (padstack …)) (network (net … (wire (path layer x1 y1 x2 y2 w)) … (via …)))))
```

Real Specctra SES grammar uses the same turret families (`(library)`,
`(network)`, `(wires …)`, `(via …)`, `(routes …)`). **Verification against a
real router (Freerouting) is recommended** before claiming CM-side round-trip.

### DSN/SES facts from public sources
- DSN = design input (layers, outline, keepouts, footprints, pins, nets,
  rules, pre-routes); SES = routing session output (wires + vias).
- Altium exports **DSN** and imports DSN/**RTE** (`.rte` = routed result);
  the "Specctra Router" feature is enabled by default and is what
  `tds_file.dll`/`RouteSupport.dll` implement inside CircuitMaker.
- FreeRouting accepts `board.dsn` (+ optional `.ses`) and writes `.ses`/`.dsn`/
  `.scr` — a ready-made open oracle for validating copperline SES.

---

## 9. Where CircuitMaker/Altium put each format (observed manifest facts)

From `Installation.Release` (all byte sizes are manifest-reported, factual):

| Artifact | Location | Size |
|---|---|---|
| DSN codecs | `System\tds_file.dll`, `System\tds_dss.dll` | 7,026,688 / 6,965,248 |
| Router | `System\RouteSupport.dll` (+`.Do`,`.OUT`,`.INS`) | 14,873,088 |
| PCB core | `System\Advpcb.dll` etc. | — |
| CFB reader (OLE) | `System\OpenMCDF.dll` | — |
| Situs router cfg | `System\Situs.INI`, `System\SitusTemplate.Bas` | — |
| Routing rules | `System\route.rul`, `System\padinit.dat`, `System\DefaultRules.rhv`, `System\DefaultPadsPinDecals.c` | — |
| PCB templates (CMPcbDoc) | `Templates\A.CMPcbDoc` … `Title Block.CMPcbDoc` | 732,672–744,960 (14 files) |
| SchLib templates | `Templates\SchLib\*.SchLib` + `System\Templates\OrCAD-CAPSYM.SchLib` | 4,608–99,328 |
| CM product data | `System\CircuitMaker.Designspace` | 4,767 |
| CAM outputs | `System\Gerber.OUT`, `GerberX2.OUT`, `Drill.OUT`, `Odb.OUT`, `PADS_ASCII.out`, `PickPlace.OUT` | — |

---

## 10. Clean-room implementation checklist for copperline

1. **CFB reader** — implement `ParseCFB` (header at 512-byte sectors,
   FAT-style) ~200 LOC; iterate directories; for each storage of interest
   read `Header` (u32 count) + `Data`.
2. **Text-prop parser** — `ReadProperties(block) -> map<string,string>`:
   split on `|`, parse `K=V`; remember trailing NUL.
3. **Binary record dispatch** — switch on record_type 1-6,11,12; parse via
   documented fixed offsets, use `subrecord_length` for version tolerance.
4. **Board6 → Board** — outline verts, layer stack, `LAYER<n>NAME` mapping;
   5.**Nets6/Components6 → nets/terminals** — resolve REF-PAD via
   Components6 placements + Pads6 geometry; net list from Nets6.
5. **Rules6 → BoardDefaults/classes** — Width/Clearance/RoutingVias kinds.
6. **Tracks6/Arcs6/Vias6/Fills6/Texts6 → trace objects + obstacles** for the
   routing engine (convert nm via ×2.54).
7. Wire into `main.cpp` as a new `--input *.CMPcbDoc` path that produces the
   same `Board` as DSN — router/verifier/SES-export then work unchanged.

No step copies Altium code; all layouts above are public documentation +
observed manifest file inventory.