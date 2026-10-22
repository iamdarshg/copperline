# DipTrace observed structures (clean-room notes)

Companion to `README.md`. These are **observed facts** — byte-level findings
from the installer overlay and the extracted samples, plus the documented
public XML structure summarized from Novarm's own format specs. Nothing here
is decompiled code; it is container/format forensics.

---

## A. QSetup SFX container (observed)

### A.1 PE stub facts

```
e_lfanew = 0x100            ; PE header offset
machine   = 0x14c           ; i386
num_sections = 8            ; CODE DATA BSS .idata .tls .rdata .reloc .rsrc
overlay   = 0x3EC00         ; = last section end (0x3C600 + 0x2600)
security  = 0xF650398 size 13808  ; Authenticode cert trailer
```

The stub is a Borland Delphi PE (resources `DVCLAL`, `PACKAGEINFO` are Delphi
markers). Version resource: `FileVersion 5.3.5.1 / 12.0.0.5`,
`ProductVersion 5.3.5.1 / 12.0.0.0`, `CompanyName Novarm Limited` **and**
`Pantaray Research Ltd.` (the QSetup author), `FileDescription "DipTrace
Setup SFX"`, `Last Compile 17/05/2014`. Linker 2.25, OS 4.0 → Delphi 2007-era
toolchain for the stub.

### A.2 Config/strings block (offset 0x3EC00, ~0x3F002)

Plaintext, 0x01-separated key=value pairs:

```
\xad\x03\x00\x00  \x02  \x01 English \x01 Charset=0 \x01 FontSize=8
\x01 FontName=Tahoma \x01 s10=Downloading DipTrace Files... \x01
s11=<b>INTERNET DOWNLOAD</b>... \x01 s12=No valid Internet Connection... \x01
s0=DipTrace Installation \x01 s1=Extracting Files... \x01 s2=DipTrace Setup
will extract files to your Temp Drive (<DRIVE>)... \x01 s3...s8 (disk/CRC
dialog strings) \x01 BtnText=|&OK|&Cancel|&Yes|&No|&Abort|&Retry|&Ignore|
\x01 Attention=Attention \x01 Confirm=Confirm \x01 Error=Error \x01
\x02 M\x00\x00\x00 |http:|.info|.exe|uitftdjsufkwwivjvuxtxgvejhuxwxxxxtxjvsuxxsxuvwufvexweiij|0|
```

- The leading `\xad\x03\x00\x00 \x02` is a 4-byte count/flag + record marker.
- `sN=` strings correspond exactly to `[Extract]`/`[Split]`/… sections of the
  `English.lng` language file (stream 8) — confirming the config block is the
  SFX's UI string table.
- The trailing `|http:|…|0|` block configures the SFX's split/download
  feature (the stub can download volume files). The long token
  `uitftdjsufkww…` is a single-byte-XOR-resistant obfuscated string (no
  1-byte XOR key produced ASCII) — likely keyed XOR; not needed for
  extraction. `.exe`/`.info`/`http:` are plaintext substrings of it.

### A.3 Payload records (from 0x3F002)

```
loop:
  u32 csize (LE)                       ; compressed size of next zlib stream
  zlib(csize bytes)                    ; DEFLATE (zlib header 0x78 …)
      → "|NNNNN#filename|unixtime|\x00" + file bytes
```

- **1,740 streams** total, 258,013,206 compressed bytes, 2,529,871,647 bytes
  uncompressed, last stream ends at 0xF650348 (records stop 13,888 bytes
  before EOF = certificate + gap).
- Streams 0–8 use header `|filename|unixtime|\x00` (no `NNNNN#` index):
  0 `Engine.exe` (UPX-packed, `UPX0/UPX1/.rsrc` sections visible), 1
  `Setup.txt` (build script), 2 `DipTrace 64.qsp`, 3 `Modern_Setup.bmp`,
  4 `Modern_Icon.bmp`, 5 license, 6 `setup.bmp`, 7 `setup_white.bmp`,
  8 `English.lng`.
- Streams 9+ use `|NNNNN#filename|unixtime|\x00`. The 5-digit NNNNN is a
  **stable sequence number** referenced by `Setup.txt`'s
  `SET_COPY_FILES(NNNNN#name|…)` groups. It is NOT the same as the stream
  index (streams 9.. = seq 0..), and multi-part files (`.00002`, `.00003`
  suffixes) reuse the same seq with different filenames.
- Timestamps are Unix epoch seconds (e.g. `1563726458` ≈ 2019-07-21; some
  legacy files carry 2000-era stamps like `999775488`).

### A.4 The build script (`Setup.txt`) grammar (observed)

```
SET_MEDIA_NAME(diptrace_trial_setup.exe);
SET_PROG_NAME(DipTrace);
SET_PROJECT_NAME(DipTrace 64);
SET_PROG_VERSION(5.3);
SET_COMPOSER_BUILD(12.0.0.5);
SET_PC_STAMP({B83E56A0-…});
SET_MSI_CODES(|{7AF0B347-…}|{241FABBE-…}|);
SET_GROUP_DATA(|Main Group|…|2387143|1432|…|);
SET_SUB_DIR(<Application Folder>);
SET_COPY_FILES(00000#CompEdit.chm|00052#Default.reg|…);
```

`<Application Folder>` = `%ProgramFiles%\DipTrace`, `<Common Folder>` =
`%ProgramData%\DipTrace`, `<Auxiliary Folder>` = `%MyDocuments%\DipTrace`.
1444 seq→path entries mapped. This file doubles as a free file manifest for
any QSetup installer.

### A.5 Signature scan results (whole payload)

| Signature | Hits | Verdict |
|---|---|---|
| `PK\x03\x04` / `PK\x05\x06` / `PK\x01\x02` | 0 | no ZIP |
| `MSCF` | 0 | no CAB |
| `Rar!\x1a\x07` | 0 | no RAR |
| `7z\xbc\xaf\x27\x1c` | 0 | no 7z |
| `MSWIM`, `fd7zXZ`, `NullsoftInst`, `Inno Setup`, OLE2 | 0 | none |
| `\x1f\x8b` (gzip magic, 2-byte) | 4435 | false positives (2-byte) |
| `` `\xea`` (ARJ, 2-byte) | ~1.0M | false positives (2-byte) |
| `BZh` (3-byte) | 16 | ≈ random rate for 258 MiB (258M/16.7M ≈ 15.5) |
| `ZED` (3-byte, in first 64 MiB) | 3 | ≈ random (64M/16.7M ≈ 3.8) |

Conclusion: no standard container inside; the entropy≈8.0 uniformity across
every 64 KiB sample is zlib output, not ciphertext.

---

## B. Native binary formats (observed on samples)

### B.1 Common shell

All four native types share a header idiom:

```
<count_byte> <4 ASCII type chars> <float32> <float32> … <len-prefixed UTF-16LE string> …
```

Observed headers (samples in `fixtures/reverse/diptrace/`):

| File | bytes[0:8] | type |
|---|---|---|
| `Astable_Flip_Flop.dip` | `07 44 54 42 4F 41 52 44` | `\x07DTBOARD` |
| `Astable_Flip_Flop.dch` | `07 44 54 53 43 48 45 4D` | `\x07DTSCHEM` |
| `_to220_p5.50.lib` | `06 44 54 43 4C 49 42` | `\x06DTCLIB` |
| `pp_filters_ferrite.eli` | `06 44 54 45 4C 49 42` | `\x06DTELIB` |

- The `0x06`/`0x07` prefix is likely a record/version tag; type strings are
  `DTBOARD`, `DTSCHEM`, `DTCLIB`, `DTELIB`.
- Data idiom: 4-byte LE floats appear as `3b 9a ca 00`-style values
  (e.g. `0x009aca3b` ≈ 0.0047… in IEEE-754 LE, consistent with mm-scale
  coordinates), and strings are UTF-16LE with a 2-byte length prefix
  (e.g. `0x0038` = 56 chars for the path below).
- The `.dip` header embeds the source schematic path:
  `C:\Program Files\DipTrace\Examples\Astable_Flip_Flop.dch` (UTF-16LE).
- The `.eli` header embeds a display name, e.g. `"P & P Filters Ferrite"`
  then per-part records (`Part 1`, part name `GZ2012D601TF`, …).

These binary formats are undocumented by Novarm and change across versions;
the **XML and DSN/SES formats are the supported interchange paths**.

### B.2 The `tango_1.net` sample (Protel/Tango netlist, ASCII)

```
[
C41
CAP_225
C_225
0,1
]
[
R31
SMD1206
RES1206
47k
]
…
(
NET00028
J7-DATA_
R11-1
)
(
+12V
IC1-18
…
)
```

- Component section: `[` + RefDes / Pattern (package) / Value (+ optional 4th
  field) + `]`.
- Net section: `(` + net name + `refdes-pin` list + `)`.
- CRLF line endings, blank lines inside sections. Imported by DipTrace via
  "Import netlist → Tango/Protel".

### B.3 `outline.dxf` sample (DXF R1009, ASCII)

Standard group-code DXF: `0 SECTION / 2 HEADER / 9 $ACADVER / 1 AC1009` …
This is the board outline as plain DXF.

---

## C. DipTrace XML structure (from Novarm's public specs — summarized)

### C.1 Conventions shared by all four dialects

```
Type:        Int / Real(dot decimal) / Bool("Y"/"N") / Text / Color(0x00BBGGRR) / enum
Units:       root attribute "mm"|"inch"|"mil"; all Reals in those units
Angles:      radians CCW; Orientation enums 0/90/180/270; 3D Rotate in degrees
Frames:      project frame (board/sheet objects) vs object-center frame (library internals)
IDs:         flat lists joined by Id; -1 = none; resolve by Id since 5.3.5.0
Defaults:    omitted from file ⇒ use default
```

Font scalars are *not* project units (internal `FontSize`, negative
`FontWidth` stroke codes); positive `FontWidth` since 5.3.5.0 *is* a
project-unit length.

### C.2 PCB dialect skeleton

```
<Source Type="DipTrace-PCB" Version="5.3.5.0" Units="mm">
  <Library Type="DipTrace-ComponentLibrary" Units="mm">
    <Library Type="DipTrace-PatternLibrary" Units="mm">…footprints…</Library>
    <Components>…design-cache…</Components>          <!-- no <Categories> in 5.3.5.0 -->
  </Library>
  <Board>
    <BoardOutline><Points><Point X Y Arc/></Points></BoardOutline>   <!-- Y stored negated -->
    <Panel …/>
    <SheetSettings …/>
    <Settings …/>
    <ProjectLibs …/>
    <CopperLayers><Lay Id Type="Signal|Plane" PlanePad NetId PlaneRing Color><Name/></Lay></CopperLayers>
    <NonSignals><NonSignal Id Side="None|Top|Bottom" Color><Name/></NonSignal></NonSignals>
    <LayerStackName>…</LayerStackName>
    <LayerStackItems><LayerStackItem Lay="0"><Material Type="Conductor|Plane|Dielectric"
        VariableThickness Thickness Constant TraceWidth><Name/></Material></LayerStackItem></LayerStackItems>
    <HierarchySheets …/>
    <ViaStyles><ViaStyle Id Size HoleSize Lay1 Lay2><Name/></ViaStyle></ViaStyles>
    <NetClasses><NetClass Id UpdateId Type="Normal|Differential Pair" AllLayers …>
        <Name/><LayProperties><LayProperty MinWidth MaxWidth Clearance …/></LayProperties>
        <AllowedVias><ViaStyle>0</ViaStyle></AllowedVias>
        <Autorouting RoutePriority PriorityValue RouteMaxVias MaxViasValue
                     RouteAllLayers RouteLayers …/></NetClass></NetClasses>
    <ClassToClass Enabled><CTC_Cells><Cell NetClass1 NetClass2 Clearance>
        <LayerClearances><Lay Id Clearance/></LayerClearances></Cell></CTC_Cells></ClassToClass>
    <DRC AllLayers CheckClearance … SilkClearance>
      <ShowList/><RealTimeMode/><DRCDone/>
      <LayClearances><LayClearance Lay SameTraceToTrace SameSmdToVia …>
          <ClearanceDetails …/></LayClearance></LayClearances>
      <LaySizes><LaySize Lay MinTrace MinDrill MinRing MaxRing …/></LaySizes>
    </DRC>
    <ConnectivityCheck Traces Shapes CopperPours/>
    <MainLengthRule …/><LengthRules …/>
    <Groups><Group Id Enabled/></Groups>
    <Components><Component Id UpdateId PatternStyle X Y Angle Side="Top|Bottom" …>
        <RefDes/><Name/><Value/>
        <Pads><Pad Id NetId InternalConnection SignalDelay/></Pads>
        <RefDesMarking><Silk Show Align Horz Vert X Y Angle/><Assy …/></RefDesMarking>
        …</Component></Components>
    <Ratlines><Ratline Id Hidden X1 Y1 X2 Y2 Comp1 Pad1 Comp2 Pad2/></Ratlines>
    <Nets><Net Id HiddenId NetClass RouteMode="Ratlines|Correct Traces|Full Reroute|Dont Route">
        <Name/><Pads><Item Comp Pad/></Pads><Traces>…</Traces></Net></Nets>
    <DifferentialPairs><DifferentialPair Id NetClass PosNet NegNet RouteMode …>
        <Name/>…coupled segment geometry…</DifferentialPair></DifferentialPairs>
    <CopperPours><CopperPour …/></CopperPours>
    <Shapes><Shape …/></Shapes>
    <DesignErrors …/><Tables …/><Dimensions …/>
  </Board>
</Source>
```

### C.3 Trace/point model (routing core)

- A `<Trace>` inside a net: `Connected1/Object1/SubObject1/Point1` + same for
  end 2 — endpoint kind `Pad|Trace|Segment|Separate Trace|Free`, object id
  (component id for pad ends), sub-object id (pad id), point index.
- `<Points><Point Id X Y Lay Width ShieldGroup ViaStyle=-1 Arc Meander …/>`
  — per-segment layer/width/via on the **second** point; first point's
  non-geometry attrs ignored on import.
- **Connectivity law:** pad `NetId` is authoritative; net `<Pads>` is a
  derived mirror; ratlines auto-generated; a trace endpoint landing on a pad
  of a different net merges the two nets. This matters for any router that
  writes traces back via XML or SES.

### C.4 Schematic dialect skeleton (brief)

```
<Source Type="DipTrace-Schematic" Version Units>
  <Library Type="DipTrace-ComponentLibrary">…design cache…</Library>
  <Schematic>
    <SheetSettings …/> <Settings …/> <NetClasses …/> <DesignCache/>
    <ERC …/> <Components><Part Id …><Pins><Pin …/></Pins></Part></Components>
    <Nets><Net Id …><Name/><Pins><Item Part Pin/></Pins><Wires>…</Wires></Net></Nets>
    <DifferentialPairs …/> <Buses …/> <Shapes …/> <Tables …/> <Simulator …/> <Groups …/>
  </Schematic>
</Source>
```

### C.5 Exchange / plug-in laws (5.3.5.0)

- Manifest gates per object kind: `<Comp>`, `<Net>`, `<Shape>`, `<Plane>`,
  `<Table>`, `<Dim>`, `<Diff>`, `<Board>`, `<Ratlines>` (old name `<Trace>`
  accepted), `<Layers>` (`None` = stack not exchanged), `<DesignCache>`,
  `<Group>` — values `All|Selected|None`, plus `<ExpMode>All|Partial` and
  `<ImpMode>All|Edit` in `settings.xml`.
- Anti-wipe: empty container never erases a live list; absent block changes
  nothing; merge by Id; delete via `Enabled="N"` (don't remove the node).
- Id rules: keep Id to edit; omit Id to add (never compute max+1); full-file
  generation → dense ascending Ids, position==Id.
- Nested lists (traces, wires, points) are **replaced wholesale**, not
  merged: to append a trace you must re-list every current member with its Id.

---

## D. DSN/SES surface relevant to DipTrace integration

copperline's documented DSN subset (from `include/router/dsn.h`):

```
(pcb <name>
  (unit mm|mil|um|inch)
  (structure
    (layer <name> (type signal|plane))
    (boundary (rect …)) (via <name> <outer> <hole>)
    (rule (width w) (clearance c))
    (plane <net> (polygon …) (layer …)) (copper_pour …)
    (wire|via|place|bend|elongate_keepout …))
  (placement (component <ref> (place <part> x y <front|back> <rot>)))
  (library (image <part> (pin <pad> x y) … (outline …)))
  (network (net <name> (pins <ref-pad> …) (class <class>))
           (class <name> (rule …) (circuit (use_via …)))))
```

SES export writes `(session … (route (library (padstack …)) (network
(net … (wire (path <layer> x y … <w>) (via …))))))` in mm/6-decimals,
re-quantized to integer nm on import.

DipTrace's DSN exporter targets Specctra/Electra/TopoR-class routers, so its
output should fall in the same grammar family; any divergence (e.g. boundary
polygons, component rotations in degrees vs radians, keepout spelling) must
be checked against a real capture — the recommended next fixture step.

---

## E. Verification notes

- All byte offsets/sizes verified programmatically against the actual file
  (PE parse, overlay walk, zlib decompress of all 1,740 streams).
- Extraction produced 1,731/1,740 parseable file headers; 9 headerless
  streams are the SFX-internal files (Engine.exe, Setup.txt, qsp, bitmaps,
  lng) which carry the simpler `|name|ts|` header.
- Samples copied into the repo are byte-identical to the decompressed
  stream bodies (header stripped at the first `\x00` after the timestamp).
