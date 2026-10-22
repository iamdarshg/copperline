# DipTrace — Reverse-Engineering Notes (Clean-Room)

**Target:** `diptrace_trial_setup.exe` (258,292,104 bytes, SHA-256-suspect build)
**Scope:** Installer container architecture, file formats, and the Specctra DSN/SES
integration path into the copperline headless autorouter.
**Method:** static, offline analysis only (7-Zip listing, Python parsing of the
overlay, web research). The installer was **never executed**.
**Clean-room status:** All notes are observations of file structure + facts from
Novarm's *public* documentation (the format specs and plugin guide ship *inside*
the installer's Docs folder and are also published by Novarm). No code was
decompiled, and no copyrighted implementation is reproduced here.

---

## 1. Installer architecture — QSetup Installation Suite SFX

### 1.1 PE stub

| Field | Value |
|---|---|
| File size | 258,292,104 bytes (247 MiB) |
| PE type | 32-bit x86 (`machine 0x14c`), Windows GUI |
| Image size | 286,720 bytes (~280 KB) — the PE is a tiny stub |
| Sections | 8: CODE, DATA, BSS, .idata, .tls, .rdata, .reloc, .rsrc |
| Overlay start | file offset `0x3EC00` (257,024) |
| Authenticode | WIN_CERTIFICATE at `0xF650398`, 13,808 bytes |
| Version info | QSetup `12.0.0.5`, FileDescription "DipTrace Setup SFX", CompanyName "Pantaray Research Ltd." / "Novarm Limited", Last Compile 17/05/2014 |
| Runtime | Borland Delphi (resources `DVCLAL`, `PACKAGEINFO` are Delphi markers) |

The version resource carries **two** version pairs — `5.3.5.1` (the DipTrace
product being installed) and `12.0.0.5` (the QSetup composer that built the
media). The "Last Compile" date is the *stub's* build date, not the payload's.

### 1.2 Overlay structure (the "encrypted-looking" 258 MB)

7-Zip treats the whole payload as one opaque member `[0]` (258,021,272 bytes) —
it auto-detects no container format (no ZIP/CAB/RAR/NSIS/Inno signatures;
`PK`, `MSCF`, `Rar!`, `7z` all have **zero** hits in the payload). Early scans
found `ZED`/`BZh`/`ID3` markers at "regular intervals", but counting shows they
occur at **random-coincidence rates** (3 hits of a 3-byte pattern in 64 MiB
≈ 64M/16.7M ≈ 3.8 expected) — they are false positives inside compressed data.

The real structure, recovered by parsing the overlay directly:

```
offset 0x3EC00   plaintext SFX config/strings block (~1 KB):
                 "English\x01Charset=0\x01FontSize=8\x01FontName=Tahoma\x01
                  s10=Downloading DipTrace Files... \x01 s11=<b>INTERNET DOWNLOAD</b>...
                  s0=DipTrace Installation \x01 s1=Extracting Files...
                  BtnText=|&OK|&Cancel|... Attention=Attention Confirm=Confirm Error=Error"
                 (0x01-separated key=value pairs; sN= strings come from English.lng)
offset ~0x3EFxx  "\x02M\x00\x00\x00|http:|.info|.exe|uitftdjsufkwwivjvux...|0|"
                 (download-URL config for the split/download SFX feature)
offset 0x3F002   payload records: 1740 ×
                 [ 4-byte little-endian zlib-stream size ]
                 [ zlib stream ]  → decompresses to:
                   "|NNNNN#filename|unix_timestamp|\x00" + raw file bytes
                 (streams 0–8 use a simpler "|filename|timestamp|\x00" header)
offset 0xF650348  end of records (1740 streams), then Authenticode certificate
```

**Key finding: the payload is *not* encrypted — it is a flat sequence of
zlib-compressed chunks**, each carrying its own file header inline. Any
`zlib.decompress()` call recovers a file. The ~8.0 shannon entropy everywhere
is just zlib's normal output entropy.

### 1.3 Extraction

A 30-line Python script (this project: `docs/reverse/diptrace/tools/qsetup_extract.py`)
walks the records and decompresses every stream:

- **1,740 streams**, 2,529,871,647 bytes (2.53 GB) uncompressed
- **1,731 parsed** with `NNNNN#name` headers; stream 0 is `Engine.exe`
  (the QSetup engine, UPX-packed), stream 1 is `Setup.txt` (the QSetup build
  script = a full manifest of seq# → install path), stream 2 is
  `DipTrace 64.qsp` (the QSetup project file, `FileType=QSetupIniFile`,
  `ComposerVersion=12.0.0.5`, `Language=UKRAINIAN`), streams 3–8 are bitmaps,
  license text and `English.lng`.

**The build script (`Setup.txt`) is the definitive manifest.** It maps every
`NNNNN#` index to a target path (`SET_SUB_DIR` + `SET_COPY_FILES`):

| Seq range | Install path | Contents |
|---|---|---|
| 0–39 | `\converter64` | OpenCascade DLLs (`TKernel.dll`, `TKBRep.dll`, `TKSTEP.dll`, …), `freetype.dll`, OpenSSL (`libeay32`, `ssleay32`), VC++ runtimes — the 3D/STEP/IGES import subsystem |
| 40–51 | `\Data_Unicode` | default component DB, `comstand_std.lib`, `elstand_std.eli`, `compat.db`, language files |
| 52–61 | app folder + `\Docs` | the **public XML format specs** and plugin guide |
| 62–84 | `\Docs\plugin_sdk` | markdown format references + templates (python + xml) |
| 85–107 | `\Examples` | sample boards/schematics/netlists, incl. **BGA_autorouter.dip**, example_routed.dip, example_unrouted.dip, Differential_Pairs.dip, tango_1.net |
| 108–113 | app folder | **CompEdit.exe, PattEdit.exe, Pcb.exe, Schematic.exe, Launcher.exe, DipTrace_DB_Updater.exe** |
| 114–360 | `\images\pattern_generator` | pattern-generator UI images |
| 361–1327 | `\Lib` | **967 library files: 505 .lib pattern libs + 464 .eli component libs** |
| 1336–1342 | `\ngspice64` | bundled ngspice-41 (SPICE simulation engine) |
| 1434–1437 | `\router64` | **RouteService.exe, router.dll, RouteLayer.dll, RouteServiceChecker.vbs — the built-in shape-based autorouter** |
| 1345–1433 | `\Plugins` | example plugins incl. full C# and Delphi *source* (DashDotLine, OpenMaskForTraces), `settings.xml` templates |

### 1.4 Why 7-Zip cannot extract it

QSetup's SFX container is not registered in libarchive's format table. The
payload has no global directory or magic header — file metadata is embedded
per-chunk, and there is no EOCD/cab header to anchor on. The `PK\x03\x04` hits
in the mission brief were zlib false positives (`PK` occurs in compressed
streams by chance). **Extraction is nonetheless trivial** with Python's
`zlib` (see tool script).

---

## 2. DipTrace file formats (clean-room summary)

### 2.1 Native binary formats — the `.dip` assumption is WRONG for native files

The mission brief assumed `.dip` is ASCII. **Observed: native DipTrace files
are BINARY**, with a common record layout:

| Extension | Meaning | Observed magic |
|---|---|---|
| `.dip` | PCB board | `\x07DTBOARD` |
| `.dch` | schematic | `\x07DTSCHEM` |
| `.lib` | pattern (footprint) library | `\x06DTCLIB` |
| `.eli` | component (schematic symbol) library | `\x06DTELIB` |

Verified on the extracted samples (`fixtures/reverse/diptrace/`):
`Astable_Flip_Flop.dip` starts `07 44 54 42 4F 41 52 44` (= `\x07DTBOARD`),
`Astable_Flip_Flop.dch` starts `07 44 54 53 43 48 45 4D` (= `\x07DTSCHEM`),
`_to220_p5.50.lib` starts `06 44 54 43 4C 49 42` (= `\x06DTCLIB`),
`pp_filters_ferrite.eli` starts `06 44 54 45 4C 49 42` (= `\x06DTELIB`).

The binary layout (observed): magic prefix byte (0x06/0x07), 4-char ASCII
type, then a stream of **length-prefixed UTF-16LE strings** interleaved with
**little-endian 4-byte IEEE-754 floats** (coordinates/angles) and small
integers. E.g. the `.dip` header embeds the source `.dch` path as UTF-16LE
(`C:\Program Files\DipTrace\Examples\Astable_Flip_Flop.dch`) followed by
floating-point data. This is a Delphi-style record serialization. **No
public binary-format spec exists** — Novarm's *documented* interchange
formats are XML and DSN/SES (below).

> Digression: DipTrace 5.x also offers "Save/Open As DipTrace XML" and
> markets **native XML for Git workflows** — the XML dialects are the
> text-based, version-control-friendly representation (see 2.2).

### 2.2 DipTrace XML dialects (FULLY public — the golden clean-room source)

Novarm ships four XML format specs *in the installer itself*
(`Docs\DipTrace_{PCB,Schematic,CompEdit,PattEdit}_XML_Specification.pdf`)
plus a plugin developer guide (md+pdf) and a `plugin_sdk\` with markdown
references. These are official Novarm publications describing exactly what
the serializer writes/reads. All four share one convention set
(`02_diptrace_xml_conventions.md`):

- Root shells:
  - project files: `<Source Type="DipTrace-PCB"|"DipTrace-Schematic" Version="5.3.5.0" Units="mm|inch|mil">`, embedding a `<Library>` design-cache (component library **nesting the pattern library**) then the editable `<Board>`/`<Schematic>`.
  - library files: `<Library Type="DipTrace-ComponentLibrary" Name= Hint= Version= UID32= Units=>` (`.eli`/`.elixml`) and `<Library Type="DipTrace-PatternLibrary" …>` (`.lib`).
- Units: root-level `Units` only; **no per-object override**. All Real coords are in those units, dot decimal (reader tolerates comma).
- Frames: project/sheet frame for board objects; **object-center frame** for library internals (pins/pads/shapes relative to part origin).
- Angles: **radians CCW** except discrete orientation enums (0/90/180/270) and 3D-model Rotate (degrees).
- IDs: flat set of lists joined by integer `Id` (`-1` = none). Since 5.3.5.0 references resolve **by the Id the file carries** (identity when Id=position), with fallback + one import-summary dialog.
- Booleans `Y/N`; colors Windows `0x00BBGGRR` (255 = red).
- Defaults are *omitted*: missing attribute ⇒ default, not zero.
- Exchange laws (plug-in path): manifest gates (`<Ratlines>`, `<Layers>`, `<DesignCache>`, `<Group>`), **anti-wipe** (empty container never erases a live list; absent block changes nothing), merge by Id without deletion, delete channel = `Enabled="N"`.

### 2.3 PCB XML routing-relevant elements (from the official spec)

The PCB dialect is the direct map to copperline's `Board`:

| XML | Meaning | copperline analogue |
|---|---|---|
| `<CopperLayers><Lay Id Type=Signal\|Plane NetId PlaneRing>` | copper stack, file order = stack order (Top…Bottom) | `board.layers` |
| `<NonSignals><NonSignal Id Side Top\|Bottom>` | custom layers | non-signal layers |
| `<LayerStackName>/<LayerStackItems><Material Type Conductor\|Dielectric Thickness Constant>` | stack-up + materials | via/stack metadata |
| `<ViaStyles><ViaStyle Id Size HoleSize Lay1 Lay2>` | via styles; **no Through attr — through vs blind/buried derived from Lay1/Lay2 absence** | via geometries |
| `<NetClasses><NetClass Id Type Normal\|Differential Pair><LayProperties><LayProperty MinWidth MaxWidth Clearance>` | net-class rules, per-layer width/clearance | rules/width classes |
| `<NetClass><AllowedVias>` | via restriction per class | via_class |
| `<NetClass><Autorouting RoutePriority RouteMaxVias RouteAllLayers>` | autorouter hints (priority, max vias, layer allow) | router knobs |
| `<ClassToClass><CTC_Cells><Cell NetClass1 NetClass2 Clearance>` | class-to-class clearance matrix (upper triangle) | clearance tables |
| `<DRC AllLayers CheckClearance CheckSize …><LayClearances><LayClearance Lay SameTraceToTrace …>` `<LaySizes><LaySize MinTrace MinDrill MinRing>` | DRC config + per-layer min sizes | design rules |
| `<BoardOutline><Points><Point X Y Arc>` | board edge polygon (Y stored negated) | outline polygon |
| `<Components><Component Id PatternStyle X Y Angle Side Top\|Bottom><RefDes><Name><Value><Pads><Pad Id NetId InternalConnection>` | placements; **pad NetId is the authoritative net membership** | components + pads + net refs |
| `<Ratlines><Ratline Id Comp1 Pad1 Comp2 Pad2 X1 Y1 X2 Y2>` | unrouted connections (auto-derived from pad NetIds — "a plug-in does not hand-build it") | ratnets to route |
| `<Nets><Net Id NetClass HiddenId RouteMode><Name><Pads><Item Comp Pad><Traces>` | nets; net `<Pads>` is a **derived mirror**, rebuilt from pad NetId | nets |
| `<Traces><Trace Id Connected1 Object1 SubObject1 Point1 …><Points><Point Id X Y Lay Width ViaStyle Arc Meander>` | routed segments; per-segment props on the **second point**; connectivity via endpoint quads, not visual position | routed copper |
| `<DifferentialPairs><DifferentialPair Id NetClass PosNet NegNet>` + Pos/Neg trace geometry | diff pairs | diffpair engine |
| `<CopperPours><CopperPour NetId Lay>` | copper pours (fills can be compressed or plain text) | plane/pour zones |
| `<Shapes><Shape NetId LayId>` | free graphics/text | shapes |

Connectivity model (official): *"Electrical membership lives on the pad: each
pad's own NetId is authoritative … to move a pad between nets you set its
NetId, not the `<Pads>` list."*

### 2.4 `.eli` schematic library / CompEdit dialect (brief)

The CompEdit spec (20 pages) covers the component library dialect:
`<Library Type="DipTrace-ComponentLibrary">` → `<Categories><Category>` tree,
`<Components><Component>` with symbol geometry (`<GraphicItems>`, `<Pins>`),
pattern links (`<Pattern Style=…>` matching a PatternStyle in the embedded
pattern library), and part/package info. Pins bind to pads via
`Pin.PadNumber == Pattern Pad.Number`. Component libraries can be stored as
binary `.eli` or XML (`.elixml`).

### 2.5 Pattern library (PattEdit dialect) — brief

Footprints: `<Library Type="DipTrace-PatternLibrary">` → `<Patterns><Pattern
Name Orientation>` with `<Pads><Pad Number PadStyle Side Top|Bottom>`,
`<GraphicItems>` (lines/arcs/circles/text on layers like `Top Silk`),
`<Holes>`, `<Origin>`, `<Courtyard>`. Pads reference `PadStyle` objects that
carry pad shapes (circle/rect/obround/polygon + thermal spokes).

### 2.6 Netlists and import formats

- `tango_1.net` (extracted sample, 7.1 KB) is an ASCII **Protel/Tango-style
  netlist** (bracket component sections `[RefDes, Pattern, Value]`, then
  parenthesised net sections `(NETxxxx <refdes>-<pin> …)`) — matches
  DipTrace's documented netlist importers (Accel, Allegro, Mentor, PADS,
  P-CAD, KiCad, Protel, Tango).
- `outline.dxf` (extracted sample) is plain ASCII **DXF R1009**.
- Import/export per official site: Altium, OrCAD, PADS, P-CAD, Eagle, KiCad,
  DXF exchange; ODB++, PADS VX ASCII designs; Gerber RS-274X/X2, Excellon,
  IPC-2581C, IPC-D-356A output.

### 2.7 Bundled 3rd-party tech (observed, factual)

- `converter64\` = **OpenCascade** (TKernel, TKBRep, TKSTEP, TKIGES, …) →
  STEP/IGES 3D import/export.
- `ngspice64\` = bundled **ngspice** for simulation.
- `router64\` = DipTrace's own shape-based autorouter as a **local
  service** (`RouteService.exe` + `router.dll` + `RouteLayer.dll`).

---

## 3. The DSN/SES golden path to copperline

### 3.1 DipTrace's external-router interface (official)

From diptrace.com (PCB Layout page, quoted):

> "DipTrace high-class shape-based autorouter … is capable of routing complex
> multi-layer boards … **DSN/SES interface provides support for external
> routers (Specctra, Electra, Topor etc.)**"

So DipTrace exports the board as a **Specctra DSN** file, the external router
routes it and returns a **Specctra SES** session. This is *exactly* the pair
copperline speaks:

- `DsnImporter` parses `(pcb …)` DSN: unit, structure/layers, boundary,
  vias, rules, placements, library images, networks (see `include/router/dsn.h`).
- `board_to_ses()` serializes routed copper as `(session … (route …))` SES.
- `ses_merge_into()` re-imports a router's SES into a board (net-matched).

### 3.2 The workflow (concrete)

```
DipTrace (Pcb.exe)                       copperline (headless CLI)
─────────────────────                     ──────────────────────────
File → Export → Specctra DSN   ──────▶   router route board.dsn --json
                                          (DSN import → route → SES export)
File → Import → SES (Specctra) ◀──────   routed.ses
                                          + router verify board.dsn --routes routed.ses
```

Even better: copperline can be packaged as a **DipTrace plug-in** without any
code change on the DipTrace side, using the documented exchange contract
(2.2/2.3): a standalone exe + `settings.xml` manifest
(`Type="DipTrace_Pcb_Plugin"`, `<ExpMode>…</ExpMode>`, `<Net>All</Net>`,
`<Layers>All</Layers>`, `<Ratlines>…</Ratlines>`). DipTrace exports the XML
exchange file, launches the exe with the file path, and imports the result —
the same "file in, file out" shape as DSN/SES but with full object fidelity.

### 3.3 Recommended integration points (ranked)

1. **DSN/SES (today, zero new code)**: `router route board.dsn --json
   --output routed.ses`. DipTrace's "external router" flow already produces
   DSN and consumes SES. Recommended to add a fixture pair
   (`fixtures/reverse/diptrace/*.dsn/*.ses`) generated from DipTrace's own
   example boards (e.g. `example_unrouted.dip` → DSN) once a DSN export
   fixture is captured.
2. **DipTrace XML plug-in (next step)**: implement the exchange-file contract
   (`Source Type="DipTrace-PCB"`) as a new importer/exporter in copperline;
   the official PCB XML spec gives the complete element catalog. This unlocks
   round-tripping **placements, net classes, DRC, via styles, diff pairs,
   pours** — richer than the documented DSN subset.
3. **Netlist import** (`tango_1.net` style) is a small, well-understood
   addition for netlist→placement workflows (Protel/Tango format is simple
   bracket/paren ASCII).

### 3.4 Semantic gaps to note for any adapter

- copperline's DSN subset documents exactly what it reads/writes (see dsn.h
  header comment); DipTrace's DSN export may use syntax outside that subset
  (e.g. `(boundary …)` polygons vs rect, keepouts, `(class …)` circuits) —
  verify on a captured file.
- Coordinates: DSN in mm → copperline quantizes to integer nm. DipTrace DSN
  units are configurable at export.
- Net-class / clearance fidelity is preserved in the XML path but only
  partially in DSN (DSN has `(rule …)`/`(class …)` per net; class-to-class
  matrices and DRC tables are XML-only).

---

## 4. Extracted artifacts

All heavy extraction lives in scratch
(`C:\Users\Darsh Gupta\AppData\Local\Temp\opencode\re-dt\streams\`,
1,740 stream files + `files.json` manifest + `names.json`). Tiny samples
copied into the repo (each < 100 KB, `fixtures/reverse/diptrace/`):

| File | Source stream | Size | Format |
|---|---|---|---|
| `Astable_Flip_Flop.dip` | 110 | 47,443 B | native binary PCB (`DTBOARD`) |
| `Astable_Flip_Flop.dch` | 109 | 51,041 B | native binary schematic (`DTSCHEM`) |
| `_to220_p5.50.lib` | 620 | 2,969 B | pattern library (`DTCLIB`) |
| `pp_filters_ferrite.eli` | 1400 | 3,581 B | component library (`DTELIB`) |
| `tango_1.net` | 130 | 7,129 B | Protel/Tango netlist (ASCII) |
| `outline.dxf` | 120 | 7,719 B | DXF R1009 (ASCII) |

The public XML spec PDFs and plugin SDK docs are *referenced*, not copied
(copyrighted Novarm publications, freely available on diptrace.com; the
installer ships them under `Docs\` and `Docs\plugin_sdk\`).

---

## 5. Extraction tool

`docs/reverse/diptrace/tools/qsetup_extract.py` — clean-room reimplementation
from the observed record format. Usage:

```
python qsetup_extract.py <installer.exe> <outdir>
```

Recovers every file (name, timestamp, bytes) plus `manifest.json`
(offset, csize, size, name). Run it against any future QSetup-based installer.

---

## 6. Sources

- Official DipTrace site (diptrace.com): PCB Layout page (DSN/SES external
  router statement, import/export list, native-XML claim, shape-based
  autorouter).
- Novarm's four XML format specifications + plugin developer guide (shipped
  in the installer; also published by Novarm).
- Observed container structure (this work).
