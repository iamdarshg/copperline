# Reverse-engineering reports — clean-room integration

This directory holds clean-room reverse-engineering reports for PCB-EDA
installers found in the Downloads directory, integrated into copperline
as **documentation + format intel + fixtures** for future import/export
adapters.

## Scope & method

| Installer | Product | Framework identified | Report |
|---|---|---|---|
| `CircuitMakerSetup.exe` (22 MB) | Altium CircuitMaker 2.2.1 | Altium proprietary Delphi web-installer (7-Zip LZMA overlay + `Installation.Release` download manifest, 174 packages) | [`circuitmaker/`](circuitmaker/) |
| `diptrace_trial_setup.exe` (247 MB) | Novarm DipTrace | QSetup Installation Suite SFX (partially extractable) | [`diptrace/`](diptrace/) |

**Clean-room rules enforced:** no proprietary source code is reproduced.
Reports contain only (a) observed byte-level facts (magic bytes, offsets,
record layouts) and (b) publicly documented format knowledge, restated in
our own words. Any parser written from these notes must be a clean-room
reimplementation.

## Autorouting-relevant findings (summary)

### Altium / CircuitMaker → copperline
- `.CMPcbDoc` / `.PcbDoc` are **CFB/OLE2 compound files** (magic
  `D0 CF 11 E0 A1 B1 1A E1`) with ~40 storages: text-prop streams
  (`|KEY=VALUE|` records in `Board6`/`Nets6`/`Components6`/`Rules6`) and
  binary record streams (`Tracks6`/`Vias6`/`Pads6`/`Arcs6`; records are
  `u8 type + u32 sublen`, type IDs 1..12). Coordinates are int32 in
  **1/10000 mil = 2.54 nm**.
- The manifest proves the built-in **Specctra router flow** (Tds_File
  DSN codec + Routecct `RouteSupport.dll` + Situs config) — the exact
  DSN/SES dialect copperline already targets (`fixtures/golden_demo.dsn`).
- Recommended adapter: `src/cmpcb.cpp` — minimal CFB reader → existing
  `Board` model; router/verifier/SES-export then work unchanged. The
  manifest-driven package list (`fixtures/reverse/circuitmaker/
  packages_of_interest.tsv`) shows the supported import/export matrix
  (Protel 2.8 ASCII, PADS, KiCad, EAGLE, OrCAD, ODB++, Gerber, Drill).

### DipTrace → copperline
- `.dip` board files are **binary** with ASCII magic `DTBOARD` +
  UTF-16LE strings and (observed) packed single-precision floats —
  a dedicated parser is a real project; the ASCII path below is the
  pragmatic route.
- DipTrace exports **Tango/Protel-style ASCII netlists** (classic
  `[comp]` / `(net ref-pin)` blocks — verified on the extracted sample
  `fixtures/reverse/diptrace/tango_1.net`) and **Specctra DSN** for its
  external autorouters (TopoR/Electra/Freerouting), mirroring copperline's
  existing DSN flow. `.dch` schematics and `.eli`/`.lib` libraries are
  also ASCII with a documented structure (see `diptrace/formats.md`).
- Recommended adapter: ASCII netlist import → copperline nets/terminals,
  and DSN round-trip via the existing `dsn.cpp`.

### Working code delivered (clean-room)
- `docs/reverse/diptrace/router_engine.md` + `router_exports.tsv` — the
  built-in "shape-based autorouter" is identified as **Electra**:
  `router.dll` exports `electra_init/sethost/load/dofile/command/abort`
  (ordinals 1–6); it runs as an x86 ATL COM local server
  (`RouteService.exe`, ProgID `RouteService.RouterObject`) driven by a
  x64 host adapter (`RouteLayer.dll`) inside DipTrace; six pass families
  (init_route/init_clean/filter/fanout/final_route/final_clean) with
  costed shape-based expansion; speaks **Specctra DSN in / SES out** —
  the exact dialect copperline already implements (`golden_demo.dsn`).
  Fixtures: `RouteServiceChecker.vbs`, `route_service.rgs`,
  `sample_settings.xml`.
- `tools/diptrace_import.py` — partial native-`.dip` reader → copperline
  board JSON (schema `copperline/board/1`). Decodes the confirmed record
  framing (u16-len UTF-16LE strings + LE floats), component/pin records
  and net names; trace/via payloads are located but not field-decoded
  (`docs/reverse/diptrace/dip_binary_format.md` documents the evidence).
  Output is a routable placement skeleton — `router analyze` accepts it.
- `tests/test_diptrace_import.py` — 10 assertions, validates against the
  extracted `Astable_Flip_Flop.dip` fixture and the router binary.
- `em/` module — see README below.

## Usage

```bash
# EM validation of routed copper (RF + current limits) — see em/README.md
python em/validate_rf.py em/fixtures/rf_board.json --config em/fixtures/em_rf_config.json --json
python tests/test_em.py
```