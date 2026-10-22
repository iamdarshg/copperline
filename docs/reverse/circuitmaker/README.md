# CircuitMaker Installer — Clean-Room Reverse Engineering Report

**Target:** `C:\Users\Darsh Gupta\Downloads\CircuitMakerSetup.exe`
**Size:** 22,482,160 bytes (21.44 MiB) | **Date:** 2022-06-10 build (CircuitMaker 2.2.1.6)
**Date of analysis:** 2026-10-20
**Analyst:** copperline reverse-engineering subagent (clean-room)

---

## 1. Installer Architecture — Framework Identification

### 1.1 Verdict

This is **NOT** an Electron/Squirrel app, **NOT** NSIS, and **NOT** Inno Setup.
It is **Altium's proprietary "Altium Installer" framework** — a native **Delphi
(C++Builder/VCL)** GUI install stub whose second-stage payload is Altium's own
**web-installer**: a 7-Zip SFX-style overlay archive containing
`AltiumInstaller.exe`, the real Delphi installer plus a plugin DLL and the
`Installation.Release` manifest.

The string-scan hints from the mission brief (`'electron'` @ 11,360,651 =
0xAD598B, `'Payload'` @ 6,678,827 = 0x65E92B) are **coincidental substrings**:
- `0x65E92B` = `...fIsPayloadShared...` — an internal *vault/EDMS object
  property* field name (TLIB-style serialized RTTI), unrelated to payloads.
- `0xAD598B` = `...fMCU_ARM_STMicroelectronics...` — a *microcontroller
  classification enum* string. Neither indicates an Electron app.

### 1.2 Evidence (observed facts)

| Observation | Value | Interpretation |
|---|---|---|
| `7z l` on the EXE | `Type = PE ... CPU = x86 ... Linker Version 2.25` | 32-bit PE stub |
| PE version resource | `InternalName=AltiumInstaller`, `ProductName=CircuitMaker`, `ProductVersion=2.2.1.0`, `FileVersion=2.2.1.6`, `CompanyName=Altium Limited`, `LegalCopyright: Copyright 2022 Altium Limited` | Version identity |
| PE exports (`.edata`) | `madTraceProcess`, `TMethodImplementationIntercept`, `dbkFCallWrapperAddr` | **madExcept** (Delphi error-handling library) is statically linked → stub is Delphi |
| PE imports (`.idata`) | `wininet.dll`, `oleaut32.dll`, `advapi32.dll`, `user32.dll`, `kernel32.dll`, `netapi32.dll`, `crypt32.dll`, `shell32.dll`, `gdiplus.dll`, `ActiveDS.dll` | `wininet` = HTTP downloads; COM/OLE for MSXML + CFB |
| Overlay (7z `l`) | `Type = 7z, Method = LZMA:24 BCJ2, Physical Size = 6365828, Tail Size = 4` | 7-Zip LZMA with BCJ2 x86 branch converter, solid archive |
| Overlay contents | `Installation.Release` (802,658 B), `AltiumInstaller.exe` (16,688,168 B), `CircuitMaker.dll` (1,559,552 B), `xdelta.dll` (74,752 B), `Theme/*.png` + Delphi DFM `.layout` forms, 13 EULA RTF/PDF | Second-stage installer + xdelta delta-update engine |
| `xdelta.dll` | — | Delta-update (VCDIFF) support for incremental installs |
| `CircuitMaker.dll` strings | `TRttiPackage`, `TArray<System.Rtti...>`, `InstallationSchema`, `AltiumInstallation`, `Xml.Win.msxmldom`, `TMSXMLDOMDocumentFactory` | Native Delphi RTTI + MSXML DOM → this DLL implements the install schema/update logic |
| `AltiumInstaller.exe` strings | `Altium.System.Zip.TZipHeader`, `TInstallationManager.GetReleasedFile$81`, `GetReleasedExtension$84`, `DownloadEULA$165`, `DownloadHelpID$86`, `GetPRT_PreferencesURLForDownload`, `DownloadItemRevisionURLByRevision` | Download-from-service API used by the web installer |
| Theme DFM files | `object InstallerForm: TInstallerForm`, `TXPBitBtn`, `TXPLabel`, `TXPExtPanel` | Custom VCL XP-themed controls, Delphi DFM layout files (plain in this build: `Form_InstallManager.layout`, `Frame_EULA.layout`, etc.) |

**Framework summary:** "Altium Installer" (internal name) — a custom Delphi web
installer. The EXE you download is a thin stub: its `.rsrc`/PE tail carries a
7-Zip SFX-style self-extracting archive holding a **manifest** describing what
to download, a **bootstrap installer** that reads the manifest, authenticates
against the Altium AppRegistry web service, downloads ~174 `.zip` packages
(≥247 MB total) and installs them (with xdelta patching for updates).

### 1.3 On-disk layout (byte offsets, verified against the file)

```
0x000000   PE/DOS header, 11 sections (stub)                      [16.11 MB]
0x000400   .text .. .rsrc (raw end 0xF5CA00)
0xF5C400   last PE section (.rsrc) tail: version-resource XML
           ('<?xml version="1.0" ...><description>Altium Installer</description>')
           followed by 'PADDINGPADDINGXX...' filler (7-Zip SFX signature gap)
0xF5CA00   7z magic 37 7A BC AF 27 1C  --> LZMA:24 BCJ2 solid archive
           [6,365,828 bytes] -> ends 0x156EC84 (22,473,860)
           tail 4 bytes (00 00 00 00), then Authenticode signature (8,296 B)
0x156EC88  Authenticode (PKCS#7) signature  [8,296 bytes to EOF 0x1570CF0]
```

**Verified arithmetic:** the nested 7z begins at file offset 16,108,032
(0xF5CA00) — exactly at the PE raw-section end — and spans 6,365,828 bytes to
22,473,860 (0x156EC84); + 4 tail bytes = 22,473,864 (0x156EC88, exactly the
PE SECURITY data-directory VA); the remaining 8,296 bytes to EOF (22,482,160 =
0x1570CF0) are the Authenticode signature, matching the SECURITY directory
size (8,296). `7z t` reports the archive "Everything is Ok". ✓

Overlay entry list (from `7z l`, all LZMA-compressed):

```
Installation.Release              802,658   (XML install manifest — the web-installer "download manifest")
AltiumInstaller.exe            16,688,168   (Delphi installer, madExcept-linked)
CircuitMaker.dll                1,559,552   (install schema/plugin, Delphi native)
xdelta.dll                         74,752   (VCDIFF delta engine)
Theme\*.png + *.layout                        (VCL DFM theme + bitmaps)
EULA\RTF, EULA\PDF                            (13 EULAs)
ProductImprovement.rtf           64,388
```

### 1.4 The download manifest (Task 3 — proves it is a web installer)

`Installation.Release` is a hand-written XML doc (~795 KB) with sections:
`<settings>`, `<components>`, `<extensions>`. Key facts:

```xml
<productName>CircuitMaker</productName>
<productVersion>2.2.1</productVersion>
<productBuild>6</productBuild>
<productFolder>Altium\CM2</productFolder>
<mainExeFileName>CircuitMaker.exe</mainExeFileName>
<locatorType>AppRegistry</locatorType>
<locatorParams>AppRegistryUrl=https://appregistry.api.altium.com/AppRegistryWebService.asmx|SecureLogin=False|Timeout=600</locatorParams>
<windowsPlatform>x86</windowsPlatform>
<updateMinVersion>2.0</updateMinVersion>
<platforms>
  <platform name="DXP" version="1.0.16.10"/>
  <platform name="EDP" version="10.0.16.10"/>
</platforms>
```

- **174 downloadable packages** each carrying `filename="<pkg>.zip"`
  `fileid="<GUID>"` `md5="..."` `uncompressedFilesSize`
  `compressedFilesSize`, followed by per-file `<file base="Programs|Documents"
  path="...">` entries — **6,001 files total listed**.
- No absolute download URLs in the manifest: the installer resolves `<fileid>`
  through `AppRegistryWebService.asmx` (SOAP). This is the classical Altium
  AppRegistry web-installer contract.
- `documentationHelpID` = `https://helpidadfiles.s3.amazonaws.com/circuitmaker/CircuitMaker.HelpID.zip`
  (one direct URL — documentation only).

This certifies the stub as a **web installer**: the 22 MB EXE triggers a
~247 MB+ package download at install time.

---

## 2. PCB-Autorouting-Relevant Formats Found (Task 2)

### 2.1 Format inventory observed in the manifest

**Native design-file formats** (CircuitMaker-specific):

| Extension | Meaning | Notes |
|---|---|---|
| `.CMPcbDoc` | CircuitMaker's PCB document | Binary CFB (OLE) container, same internals as Altium `.PcbDoc` — see formats.md. 14 board templates shipped in the `PCB Templates` package (A/A0–A4/B/C/D/E, Legal, Letter, Tabloid, Title Block; 732,672–744,960 B each). |
| `.SchDoc` | Schematic (shared with Altium Designer — CircuitMaker **can open** AD schematics) | CFB, `|RECORD=…|` compound ASCII, or binary stream |
| `.SchLib` | Schematic symbol library | CFB; 13 sample libs in `Templates\SchLib\` (4,608–6,656 B each) + `System\Templates\OrCAD-CAPSYM.SchLib` (99,328 B) |
| `.PrjPCB` | Project file | ASCII INI-style text (per public Altium docs) |
| `.cmplib_template` | Footprint template (`group/resistor/transistor`) | Text template |
| `.Designspace` | `CircuitMaker.Designspace` (4,767 B) | product customisation data (workspace "design space") |
| `.SchDot` | Schematic template (A2/A3/A4, ANSI) | CFB |
| `.config_3d`, `.config_2dsimple`, `.stackup`, `.pcbsyscolors`, `.rtm`, `.OutJob` | Board-view / layer-stack / report / output-job configs | Text-ish |

**Autorouting / exchange formats (the copperline-relevant set):**

| Package (manifest name) | Payload | Purpose |
|---|---|---|
| `Base_PCBUtils - Tds_File` | `System\tds_file.dll` (7.0 MB), `System\tds_dss.dll` (7.0 MB) | **Specctra DSN/SES import-export** ("TDS" = The Design System — Cadence Specctra). The two DLLs: `tds_file.dll` (format codec) + `tds_dss.dll` (design-system session) |
| `Base_PCBUtils - Routecct` | `System\RouteSupport.dll` (14.9 MB), `RouteSupport.INS`, `RouteCct.Do`, `RouteSupport.OUT`, `routecct.ico` | **Specctra-based autorouter integration** ("Routecct" = route via Specctra); the `.Do` file is the Specctra DO-command file |
| `Base_PCB` | `System\Advpcb.dll` (+`AdvPCB.IMP/.ins/.rcs/.xrf`), `Altium.PCB.DataModel.dll`, `Altium.Sch.*` suite, `route.rul`, `Situs.INI`, `padinit.dat`, `SpatialIndex.dll`, `rtree.dll`, `OpenMCDF.dll` | Core PCB editor + **Situs** topological autorouter config (`Situs.INI`, `SitusTemplate.Bas`, `Situs.ico/png`) + **OpenMCDF** (OLE CFB reader — the container layer for .PcbDoc/.CMPcbDoc) |
| `Gerber` | `Gerber.OUT`, `GerberX2.OUT`, `OUT_Gerber.dll`, `Odb.OUT`, `PADS_ASCII.out`, `ExportSTEP.OUT` | CAM output generators |
| `Drill` | `Drill.OUT`, `PickPlace.OUT`, `OUT_Drill.dll` | Excellon drill / pick&place outputs |
| `Importer_ProtelPcbBinary` (+ `ImporterProtelPCB3`, `Importer - ProtelPCB2.8ASCII`) | `LoadBin.dll`, `LoadPCB3.dll`, `LoadText.dll`, `LoadAccel.*`, `PCadB2AConverter.exe` | Legacy Protel/PCAD binary→current importers |
| `Exporter_Protel_28_ASCII` | `SaveText.dll`, `SaveAccel.dll` | Protel 2.8 ASCII text `.PCB` export |
| `Importer_99SE`, `Importer_OrCAD`, `Importer_KiCAD`, `Importer_EAGLE`, `Importer_PADS`, `Importer_Accel`, `Importer_PDIF`, `Importer_DXF`, `Exporter_DXF`, `Exporter_PCADASCII`, `EDIF` | — | import/export suite (KiCad import exists in CM 2.2!) |
| `Importer_Gerber`, `Importer_GerberBatch` | `LoadGerb.dll`, `Loadbach.dll` | Gerber → CM board import |

**Router-adjacent config files observed:** `System\route.rul` (routing rules —
the Situs rule file), `System\Situs.INI`, `System\SitusTemplate.Bas`,
`System\padinit.dat` (pad defaults), `System\DefaultRules.rhv`,
`System\DefaultPadsPinDecals.c`, `System\FabSym.lib`.

### 2.2 The `.CMPcbDoc` / `.PcbDoc` format (documented, public-domain knowledge)

See **formats.md** for the full clean-room notes. Headline facts (corroborated
by public documentation — KiCad dev-docs, BoardRipper, altium-parser,
pcbreader, embedded-society PCBLIB_FORMAT.md):

- **Container:** Microsoft Compound File Binary (CFB/OLE2), magic `D0 CF 11 E0 A1 B1 1A E1`, 512-byte sectors (V3).
- Altium Designer `.PcbDoc`, CircuitMaker `.CMPcbDoc`, CircuitStudio `.CSPcbDoc`, SolidWorks `.SWPcbDoc` are **the same format** (KiCad dev-docs table).
- ~40 named storages, each usually `Header` (uint32 record count) + `Data` stream.
- **Text-format streams** (Board6, Components6, Nets6, Classes6, Rules6, Polygons6, Dimensions6, Models, ExtendedPrimitiveInformation): length-prefixed `|KEY=VALUE|...|` ASCII records; coordinates in mil strings.
- **Binary streams** (Arcs6, Tracks6, Vias6, Pads6, Fills6, Texts6, Regions6, ComponentBodies6, BoardRegions): records = `uint8 type` + `uint32 subrecord-length` + payload. Type IDs: 1=ARC, 2=PAD, 3=VIA, 4=TRACK, 5=TEXT, 6=FILL, 11=REGION, 12=MODEL.
- **Units:** int32 nanocoordinates, 1 unit = 1/10000 mil = 2.54 nm; Y-up Cartesian.
- Layer IDs: V6 = 1 top … 32 bottom, 33/34 overlay, 37/38 solder, 56 keepout, 74 multi; V7/V8 = 32-bit IDs (`0x01000000+1`…).
- **Specctra integration:** Altium exports/imports Specctra **DSN** and **RTE** (`.rte` = result/route file) per official docs; SES is the session file which copperline already targets.

---

## 3. Autorouting Relevance → copperline integration map (Task 3/5)

copperline already implements, in `src/dsn.cpp` (`DsnImporter::import_text`,
`board_to_ses`, `ses_merge_into`):

- DSN **import** of: `(unit)`, `(structure (layer)(boundary)(via)(rule)(plane/copper_pour)
  (wire_keepout|via_keepout|place_keepout|keepout))`, `(network (class)(net))`,
  `(library (image (pin)))`, `(placement (component (place)))`.
- SES **export** (`route --output *.ses`) and **import** (`verify --routes *.ses`).

**Gap analysis vs real CircuitMaker/Altium Specctra flow:**

| copperline today | Real Altium/CM Specctra | Action |
|---|---|---|
| `(pcb "name" (unit mm) ...)` with **rect boundary** | Full Specctra: `(pcb name (unit mm) (structure (boundary (rect ...))) (library (image ... (padstack ...))) (placement (component ... (place ref part x y front 0))) (network (net ... (pins ...)) ... (wiring ...)))` | mostly aligned; add `(padstack ...)`/`(via ...)` inside `(image ...)` and `(wiring ...)` skip already handled |
| SES writer emits `(route (library ...)(network (net ... (wire (path ...) ...))))` | Canonical Specctra SES root is `(session "…" (base_design "…") (unit mm) (routes (wire (path …)) …))` | **Validate SES form against real Specctra output** (KiCad specctra.cpp, LayoutEditor SES, Freerouting); keep `.ses` and `.rte` variants |
| pin token `REF-PAD`, `-` split | Specctra uses `(net "N" (pins U1-1 U2-1))` — same | ✓ |
| pads are 0.3×0.3 mm squares | real padstack shapes matter for DRC; Specctra sends full `(padstack ... (shapes ...))` | add `(shape …)` parse: `circle`, `rect`, `polygon`, `octagon` |
| `(resolution 10)` lines ignored | exported DSNs carry `(resolution mil 10)` … | **must** respect resolution on import (unit + scaling) |
| classes/rules limited to width/clearance/via | Specctra rules: `(rule (width …)(clearance …)(class …)(via …))` + `clearance_class`, `via_at_smd`… | extend `DsnClassRule` |
| no `.D`-style designator mapping | Specctra places carry components; nets reference pins by `ref-pad` | ✓ already |

### Recommended integration points (highest value first)

1. **CMPcbDoc/PcbDoc import adapter** — new `src/cmpcb.cpp` (clean-room):
   - Read CFB container (implement a minimal CFB reader ~200 LOC, or vendor
     a permissively-licensed one; note `OpenMCDF.dll` proves CM itself uses
     CFB); enumerate `*/Header`+`*/Data`.
   - Parse `Board6/Components6/Nets6/Classes6/Rules6` via the documented
     `|KEY=VALUE|` text-prop format; parse `Tracks6/Vias6/Pads6/Arcs6` binary
     records (type IDs above). Deliver a `Board` identical to the DSN path —
     **then the existing router, verifier, and SES exporter all work unchanged.**
   - This gives a clean path: Altium Designer / CircuitStudio / CircuitMaker
     boards in, routed SES out.
2. **Resolution support in `dsn.cpp`** — respect `(resolution ...)` and
   padstack shapes; cheap, makes real exported CM DSNs route correctly.
3. **SES validation** — generate a fixture from a real route, diff against
   Specctra grammar; treat `.rte` as alias for `.ses`.
4. **Schematic (`|RECORD=…|`) parser** — reuse the same property-map approach
   for `.SchDoc`/`.SchLib` (record IDs 0..48 documented publicly) to build
   netlists when only schematics are available.
5. **Gerber Exporter coverage** — copperline already imports Gerber
   (`src/gerber.cpp`); the CAM output packages (`Drill.OUT`, `PickPlace.OUT`)
   give the same board the drill and pick&place exports needed for real fab
   data.

Clean-room note: nothing above requires copying Altium code — it is written
from the documented record layouts (KiCad's public importer docs, BoardRipper,
AltiumSharp, altium2kicad issue #28, and the Specctra DSN specification PDF).

---

## 4. Sources (public documentation, cited)

1. **KiCad Developer Documentation — Altium import format reference**
   `https://dev-docs.kicad.org/en/import-formats/altium/index.html` —
   CFB container, stream list, `Header`/`Data`, record type IDs, track/pad/via/
   text/fill layouts, coordinate system (1/10000 mil = 2.54 nm), layer IDs V6/
   V7/V8, Board6/Rules6 properties, region kinds. (State: same format for
   `.PcbDoc`, `.CMPcbDoc`, `.CSPcbDoc`, `.SWPcbDoc`.)
2. **BoardRipper — ALTIUM_PCB_FORMAT.md** `github.com/AlexeyInwerp/BoardRipper` —
   CFB + `D0 CF 11 E0 A1 B1 1A E1` magic; `|KEY=VALUE|` text stream encoding;
   1/10000-mil int32; ASCII v5.0 variant.
3. **altium-parser (EthanChan1003)** — record types 0x01..0x06; binary mil
   10000 units/mil; BINARY_MIL_TO_MM constant. Also `thesourcerer8/altium2kicad`
   (issue #28), `issus/AltiumSharp` (read/write .PcbDoc).
4. **embedded-society-altium-designer-mcp / PCBLIB_FORMAT.md** — CFB V3
   512-byte sectors; FileHeader `"PCB 6.0 Binary Library File"`; block encoding
   (uint32 length prefix incl. NUL; pipe-delimited params; Windows-1252).
5. **Altium official docs — Interfacing to Other Design Tools**
   `https://www.altium.com/documentation/altium-designer/design-tools-interfacing`
   — Specctra import: `.dsn`, `.rte`; Specctra export `.dsn` ("Ensuring PCB
   Readiness for Specctra-compatible Routers").
6. **Altium official docs — Altium Design Software Import-Export Support**
   `https://www.altium.com/documentation/altium-designer/design-tools-interfacing/altium-design-software-import-export`
   — CircuitMaker `.CMPcbDoc` export/import, Protel PCB 2.8 ASCII, P-CAD PDIF.
7. **Altium CircuitMaker docs — Design Documents / FAQs** — `.CMPcbDoc` is
   CircuitMaker's *own* PCB format; AD schematics can be embedded; projects are
   stored on Altium 365; `.PrjPCB` is ASCII.
8. **LayoutEditor.org — DSN format page** — DSN is the input to a Specctra-
   style autorouter; SES is the router output; ASCII S-expression family;
   `(resolution …)` semantics.
9. **tscircuit/dsnts** — TypeScript DSN/SES toolkit; grammar overview.
10. **freerouting docs — command_line_arguments.md** — `-de board.dsn[:ses]`
    `-do board.ses|dsn|scr`; the canonical open-source Specctra-compatible
    target copperline could interop with or compare against.
11. **rveen/golib pcbreader** — Go decoder for `.PcbDoc` text+binary storages.

---

## 5. Extracted artifacts (Task 6)

No embedded **sample board/netlist files** exist in the stub — the overlay
holds only the installer (theme, EULAs, two DLLs, installer EXE, manifest).
The `.CMPcbDoc` templates and `.SchLib` samples are **not** in the stub; they
are delivered by the 247 MB package download (`PCB Templates.zip`, fileId
`CFF02DD3-E635-4607-8491-BE760CCC3072`).

What is preserved in this repo (small, factual, clean-room):

- `fixtures/reverse/circuitmaker/packages_of_interest.tsv` — manifest-derived
  package list (names, filenames, GUID fileIds, md5, sizes) for the
  autorouting-relevant packages only (facts, not code).
- `docs/reverse/circuitmaker/README.md` — this report.
- `docs/reverse/circuitmaker/formats.md` — clean-room format notes.

Full overlay extraction (50 files, 24.5 MB) is kept in scratch
(`%TEMP%\opencode\re-cm\re-cm-payload\`) and is **not** committed.

---

## 6. Recommendations Summary

1. **Implement `.CMPcbDoc`/`.PcbDoc` import** as a CFB + text-prop + binary
   record reader feeding the existing `Board` model — highest-value interop
   for real CircuitMaker community boards.
2. **Respect `(resolution …)` and padstack shapes in the DSN importer** —
   CircuitMaker/Altium export full Specctra, which copperline's current
   simplified DSN parser only partially covers.
3. **SES writer**: verify emitted output against the Specctra SES grammar
   (root `(session … (routes …))` variant) so AD/CM can read copperline output.
4. **Expose netlist extraction** from `.SchDoc` (`|RECORD=…|` props) for
   schematic-only projects.
5. Consider **Freerouting** as a reference router for SES validation
   (copperline could issue `-de board.dsn` and consume `board.ses`, an
   independent oracle that mirrors CM's own Routecct flow).