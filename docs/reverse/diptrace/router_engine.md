# DipTrace Router Engine — Electra / Specctra-DSN Internals (Clean-Room Report)

**Binaries analyzed** (static analysis only; never executed):

| File | Size | Machine | Role |
|---|---|---|---|
| `router.dll` | 4,456,448 B | x86 (0x14c) | **Electra shape-based autorouter engine** (the whole standalone app, incl. a Qt GUI) |
| `RouteService.exe` | 140,800 B | x86 (0x14c) | ATL **COM local server** that hosts `router.dll` |
| `RouteLayer.dll` | 109,056 B | x64 (0x8664) | 64-bit **host adapter** inside DipTrace that drives RouteService over COM |
| `RouteServiceChecker.vbs` | 981 B | — | Registration/version check script (`CreateObject("RouteService.RouterObject")`, `/RegServer`, `/UnregServer`) |

`router.dll` compiled timestamp: *Wed Aug 12 14:28:43 2009*. `RouteService.exe` typelib:
*Created by MIDL version 7.00.0555 at Fri Nov 18 09:10:22 2016*. So the engine dates
to ~2009 and the COM wrapper is a 2016 retrofit — DipTrace did not write the engine,
they licensed and wrapped Electra. That is confirmed conclusively below.

---

## 1. Architecture verdict

**DipTrace's built-in "shape-based autorouter" is the Electra router** — the classic
freeware shape-based router whose library API is `electra_init / electra_sethost /
electra_load / electra_dofile / electra_command / electra_abort`. `router.dll` exports
exactly these six functions (ordinals 1–6, see `router_exports.tsv`).

Observed in-string self-identification (all from `router.dll`, offsets in hex):

- `0x2e211c  "Adaptive Shape-based AutoRouter"` — the app's own self-description
- `0x2e213c  "ELECTRA"`, `0x39daf4 "ELECTRA DEMO"`, `0x39ed40 "ELECTRA Version "`,
  `0x39ed54 "ELECTRA DEMO Version "`, `0x39deb8 "Confirm Stopping ELECTRA"`
- `0x2e1ff0  "-design      Open a design file"` (command line), `0x2e22b8 "electra.png"`
- Pass-table column header (the router's own results window):
  `|  #  |   Algo   | Shorts |DRC Errs|Failures|Unwired |  Vias  |  %Red  |  Time   |  Total  |`

### Routing approach: shape-based, costed search with rip-up-less multi-pass strategy

Electra is a **shape-based (polygon-obstacle) autorouter**, not a maze router on a
fixed grid in the classic sense. Observable evidence in `router.dll`:

- Cost-driven expansion: `0x317ce0 "cost <= total_cost"`, `0x39e98e "C@Max cost"`,
  `0x39e9b8 "Using default cost"`, `0x39ee58 "Missing cost argument"`, `0x39f358 "cost"`.
  These belong to a costed search front (A\*-like expansion with a total-cost bound).
- The routing strategy is organized as **pass families** — six named groups, each with
  a runnable count and a `run_*` entry (strings):
  - `init_route_passes`, `run_init_route` behavior behind `autoroute`
  - `init_clean_passes`, `run_init_clean`
  - `filter_passes` (cleaning/subopt filtering)
  - `fanout_passes` (SMD fanout first)
  - `final_route_passes`, `final_clean_passes`
  - UI: `Pass:` label, `Shows the current pass number`, `passLabel`, `setStatusPass(int,int)`,
    `Passes ranges from 1 to 1000`, `Start_pass ranges from 1 to 1000`
- Clean/fanout phases: `0x39f51c "Cleaning..."`, `0x39f364 "Fanout Routing..."`,
  `0x39f4b8 "fanout"`, `0x39f5cc "filter"`, `0x2e81c8 "Fanout "`, `0x2e81e4 "Clean "`,
  `run_fanout`, `run_final_clean`, `run_init_clean`
- Obstacle representation is geometric (traces/pads/vias as shapes): DSN geometry tokens
  (`(wire …`, `(via …`, `(path …`, `(qarc …`, `(rect …`, `(shape …`), DRC
  object-pair types `Wire_to_Via`, `Via_to_Via`, `Pin_to_Via`, `Via_to_Keepout`,
  `bend_keepout`, `via_keepout`, and error string `"Wire to Wire Clearance is too small"`.

### The two integrated autorouters (official docs)

From `Pcb.chm` page **Autorouting** (`autoroute.htm`):

> "DipTrace PCB Layout has two integrated autorouters: 1. High-speed shape-based
> autorouter suitable multi-layer SMD circuits. 2. Entry-level grid-based autorouter
> for simple hobby circuit boards with jumper wires. … Shape Router is selected by
> default."

Page **Review** (`simplerouterreview.htm`) describes autorouter #2:
"Grid Router is a grid-based autorouter with ‘rip-up-and-retry’ technology … three
basic autorouter steps: trace placement with ‘rip-up and retry’ technology, trace
optimization, and trace correction" with Quick/Normal/Good/Best modes; page
**Parameters** (`simplerouterproperties.htm`) adds "First Route Limit, Second Route
Limit — the part of ‘rip-up-and-retry’ technology", jumper wires, "Matrix Shift and
Choice", "Number of Options". So the classic maze + rip-up-and-retry grid router is a
**separate, simpler engine**; the shape-based one (our target) uses pass-strategy +
costed search instead (Electra's documented design).

Page **Interactive Routing Setup** (`interactive_routing_setup.htm`) describes the
manual router's *Walk Around / Push Traces / Push vs Walk Around* engine — a separate
interactive feature of Pcb.exe, not part of router.dll.

---

## 2. Public API surface and interop architecture

### 2.1 router.dll exports (the Electra library API)

| Ordinal | Name | Notes |
|---|---|---|
| 1 | `electra_abort` | stop current run |
| 2 | `electra_command` | execute a router command string |
| 3 | `electra_dofile` | execute a command file |
| 4 | `electra_init` | initialize the engine |
| 5 | `electra_load` | load a design (DSN) |
| 6 | `electra_sethost` | install the host-callback vtable (file transport, display, log, status) |

Full table with RVAs: `router_exports.tsv`.

### 2.2 RouteLayer.dll exports — the Electra host callback set

This is the host-interface mirror of the Electra design (the host must provide,
per Electra's documented architecture: display init, log, mark, path, status, via,
route init, and file transport):

| Ordinal | Name | Purpose (inferred from name + imports) |
|---|---|---|
| 1 | `Abort_Route` | tell engine to abort |
| 2 | `Destroy` | teardown |
| 3 | `Do_File` | run command file |
| 4 | `Init_Host` | register host callbacks |
| 5 | `Init_Route` | init routing subsystem |
| 6 | `Init_display` | display callback hook |
| 7 | `Init_log` | log callback hook |
| 8 | `Init_mark` | marker/point callback hook |
| 9 | `Init_path` | path drawing callback hook |
| 10 | `Init_status` | status callback hook |
| 11 | `Init_via` | via rendering hook |
| 12 | `Load_File` | load design file |
| 13 | `Send_Comm` | send command |
| 14 | `SetTempPath` | **set the temp-file directory used for the host transport** |

Imports: KERNEL32 (file I/O: `CreateFileW`, `SetFilePointerEx`, `WriteFile`,
`FlushFileBuffers`, `CloseHandle`), ole32 (`CoInitializeEx`, `CoCreateInstance`,
`OleRun`), OLEAUT32 (VARIANT/BSTR marshaling). **No router.dll import** — it is not
the in-process host; it is the DipTrace-side COM client adapter. Because it is a
64-bit DLL and router.dll is 32-bit, it cannot load the engine directly; it must
drive the 32-bit RouteService host process through COM/OLE Automation.

### 2.3 RouteService.exe — ATL COM local server hosting router.dll

- COM server machinery imported (typical ATL exe): `CoRegisterClassObject`,
  `CoRevokeClassObject`, `CoResumeClassObjects`, `CoAddRefServerProcess`,
  `CoReleaseServerProcess`, registry ops, typelib register/load
  (`RegisterTypeLib`, `LoadRegTypeLib`), and **RPC NDR stubs** (`NdrStubCall2`,
  `NdrStubForwardingFunction`, `IUnknown_*_Proxy`) for cross-process COM marshaling.
- **Direct imports from router.dll: `electra_init`, `electra_sethost`, `electra_load`,
  `electra_dofile`, `electra_command`, `electra_abort`** — proof that this exe is the
  concrete host that loads the 32-bit engine.
- RTTI strings reveal the host classes: `IElectraHost`, `IElectraHostDerived`,
  `CRouterObject`, `CRouteServiceModule`.
- COM contract (from embedded MIDL/typelib strings and the .rgs resource):
  - ProgID: `RouteService.RouterObject` (v1: `RouteService.RouterObject.1`)
  - CLSID: `{5BDF40FD-F1A5-47C4-A52D-171ABE029342}`; LocalServer32 = `"%MODULE%"`
  - TypeLib: `{E917AEE4-D143-4E20-B930-6D47FE655016}` (`RouteServiceLib 1.0`)
  - Interface `IRouterObject` (dual/idispatch) methods:
    `GetVersion`, `Electra_Init`, `Electra_SetHost`, `Electra_Load`,
    `Electra_DoFile`, `Electra_Command`, `Electra_Abort`, `Napi_version`
    (i.e. a thin IDispatch façade over `electra_*`), plus an events
    interface `_IRouterEvents` with members `layerName` and `completion` —
    the service reports per-layer progress and completion back to DipTrace.
  - AppID entry: `'RouteService.EXE'` with `val AppID = s '%APPID%'`.

The full recovered registration script is in
`fixtures/reverse/diptrace/route_service.rgs` (observed string dump, tiny).

### 2.4 Wire-format transport: temp files + COM

- `RouteLayer.dll` export `SetTempPath` + its file I/O imports, and router.dll's own
  strings `"Opened temporary file %s"`, `"Closed temporary file %s"`,
  `"Failed to create temporary file %s"`, `"Write failed on temporary file --- out of
  disk space?"`, `"Seek failed on temporary file"`, `"Read failed on temporary file"`
  show the Electra **host file transport**: design/session data is exchanged through
  temp files opened by the host and read/written via the host callback layer
  (Electra's classic pfile-style channel), while control/status goes over the COM
  methods. The checker VBS and `SetTempPath` confirm temp dir is host-controlled.

### 2.5 The chain

```
Pcb.exe (64-bit, has DSN/SES menu actions + XML)
  └─ loads RouteLayer.dll (x64, in-process host adapter)
       └─ CoCreateInstance/OleRun  →  RouteService.exe (x86, out-of-proc COM)
            └─ LoadLibrary router.dll (x86)  (Electra engine)
                 └─ electra_init / electra_sethost / electra_load / electra_dofile
                 └─ temp-file transport (host-provided), progress → IRouterEvents
```

The 32/64-bit boundary is exactly the COM boundary: DipTrace is 64-bit, the Electra
engines (router.dll, RouteService.exe) are 32-bit.

### 2.6 External-router interface (independent of the COM path)

**This is the golden path for copperline.** The official Pcb.chm pages:

- **Electra/Specctra integration** (`electraspecctra_integration.htm`):
  "DipTrace allows the user to route DipTrace circuit boards with external Electra and
  Specctra autorouters. You need to export layout into \*.dsn format, then route the
  circuit board in Electra or Specctra autorouter, and import completed layout back
  into DipTrace." Menu path "Route \ Electra/Specctra Interface \ Export Autorouter
  DSN" / "Import Autorouter SES". "Do not make any changes to the design on the stage
  before importing the \*.ses file from external autorouter."
- **Autorouter DSN** (`autorouter_dsn.htm`): "Export a PCB to \*.dsn format, route it
  with Electra or Specctra, and import results in \*.ses format back to DipTrace."
- **Autorouter SES** (`autorouter_ses.htm`): import the SES produced by the external
  router; "keep a source \*.dip file open while autorouting and importing the \*.ses
  file".

So the *external* contract is plain Specctra-DSN-in / Specctra-SES-out file
exchange — no COM required. COM is only how DipTrace drives its *bundled* Electra
engine. copperline can be a drop-in external router by speaking DSN/SES (see §5).

---

## 3. Algorithm internals (observable, clean-room)

### 3.1 Copper representation

Electra builds copper from the DSN geometry: wires, paths (polylines with optional
qarc segments), rects and padstack shapes form the obstacle set on each layer; DRC
uses object-to-object clearance pairs (`Wire_to_Via`, `Via_to_Via`, `Pin_to_Via`,
`Via_to_Keepout`, `bend_keepout`, `via_keepout`). Internal nets use generated names:
`0x2e1c10 "@temp_net%d"` — Electra/Specctra convention of `@`-prefixed temporary nets
(used for fanout "netlets"). Protected nets/wires are tracked (`0x2e1b80 "Net %s is
protected against deletion"`, `0x2e1ba8 "ProtectedFreePad"`, `0x39f098 "Protect All
Wires"`, `0x39f0d0 "UnProtect All Wires"`).

### 3.2 Search & cost

Costed expansion with a total-cost bound (`"cost <= total_cost"`); costs are
configurable via the `cost` command (`"Missing cost argument"`, `"Using default
cost"`, `"Max cost"`). The `SET WIRE_BEND 3` string (DSN-level `set` command) shows
bend-angle constraint is part of the search model.

### 3.3 Strategy passes

Six pass families exist as named count parameters + `run_*` entry points:
`init_route_passes`, `init_clean_passes` (+`run_init_clean`), `filter_passes`,
`fanout_passes` (+`run_fanout`), `final_route_passes`, `final_clean_passes`
(+`run_final_clean`). UI strings suggest fanout runs before main routing
(`Fanout Routing...` dialog) and route/clean alternate; the exact execution order
is not observable from strings. Each family takes a count, 1..1000
(`Passes ranges from 1 to 1000`, `Start_pass ranges from 1 to 1000`).
Menu `&AutoRoute`, `AutoRouteRun(),
AutoRouteStop(), AutoRoutePauseAction_activated()`, `autorouteSetup()` = the
**Autoroute Strategy** dialog (strings `0x3a7658 "Autoroute Strategy"`; via policy
checkbox names `chkViaAtSMD`, `chkViaAtSMDGrid`, `chkSharingVia`, `chkFanout`,
`spinFanout`, `edFanoutMaxLength`, `cbFanoutDir`, `spinInitClean`, `spinFinalClean`,
`chkFinalClean`, `chkInitClean`, `CleanGroup`).

### 3.4 Via insertion policy

Via-rule knobs present in the engine (strings): `max_total_vias`, `limit_bends`,
`limit_bends`/`limit_bends`, `limit_vias`, `via_on_pin`, `via_at_smd`,
`via_at_smd_grid`, `via_at_smd_fit`, `via_share`, `sharing_via`, `via_testpoint`,
`via_to_via_gap`, `smd_to_via_gap`, `non_planar_via_gap`, `buried_via_gap`,
`via_via_same_net`, `smd_via_same_net`, `bend_keepout`, `via_keepout`,
`smd_to_via_gap`. DSN rule emission observed: `rule pcb (via_at_smd on (grid on)
(via_share on) (pin_share on) …)`; the DSN writer formats per-rule `(clearance X
(type a_wire))`/`(type a_via)`/`(type a_smd)`/`(type a_pin)` and same-net forms
`smd_via_same_net`, `via_via_same_net`. `"Unknown via name"`, `"Undefined Via Type"`,
`"Routing with undefined Via"`, `"Undefined padstack for Via"`.

### 3.5 Layer assignment

DSN structure writer emits per-layer `(layer <name> (type signal|power|mixed))`, with
`(direction in)`/`(direction out)` and `(grid on)`/`(grid off)` per layer —
layer direction and grid-enable participate in the search. Stack-up/dielectric
handling is *not* in router.dll (that lives in DipTrace's own data model; see
`layer_stackup.htm`, `net_classes.htm`).

### 3.6 DSN/SES dialect (SES exchange format)

The writer format strings (observed verbatim; `%s/%d` are placeholders):

- SES header: `(session %c%s%c%s` (Electra/"session" opener inside `.ses`)
- `(routes %s` inside the session; `(net %s )`, `(Net %s %s …` net records
- Wires: `(wire ...` (and in SES output, per-net `(wire ... (path ... (qarc ...)))`)
- `(via %s ` padstacks `(padstack %s%s`; `(shape`, `(path %s %s `, `(qarc %s %s %s %s`,
  `(rect %s %s %s %s %s)`
- DSN top-level: `(parser%s`, `(resolution %s %d)`, `(library_out%s`,
  `(structure_out%s`, `(network_out%s`, `(route`s), rules `(rule%s`
- Protected wires written as `%s    (type protect)`
- Fanout nets tagged `%s    (attr fanout)`
- `SET WIRE_BEND 3%s` — bend setting command in the DSN stream
- `rule pcb (via_at_smd on/off …`, `(direction out)`/`(direction in)`, `(grid off)`/
  `(grid on)` (structure/rule fragments)

**The DSN parser lives inside router.dll** (Electra parses `electra_load`'s input
itself; the `(parser…`, `(resolution…`, `(library…`, `(structure…`, `(network…`,
`(rules…` writer strings are all in router.dll). Pcb.exe handles the user-facing
File→Export/Import DSN/SES menu items, but the engine-side parse/write is Electra's.

### 3.7 Status reporting

`status`, `status_file` commands; per-pass progress + final table
(§1); `"Routed Length ="` / `Routed Length` metric; `&Routing Status...`,
`reportRoutingStatus()`. `IRouterEvents.layerName` / `.completion` surface the same
per-pass progress to DipTrace over COM.

---

## 4. RouteLayer.dll responsibilities (summary)

RouteLayer.dll (x64) is the **out-of-process host bridge**: it implements the Electra
host-callback surface (`Init_Host/Init_display/Init_log/Init_mark/Init_path/
Init_status/Init_via/Init_Route/Abort_Route/Load_File/Do_File/Send_Comm/Destroy/
SetTempPath`) on the DipTrace side, activates RouteService via COM
(`CoCreateInstance`+`OleRun`), and provides the temp-file plumbing
(`SetTempPath`, `CreateFileW`/`SetFilePointerEx`/`WriteFile`/`FlushFileBuffers`) for
the router's host file transport. It contains **no** routing logic and **no** DSN/SES
parser parity of its own — strings are minimal (only runtime/lib plumbing). Layer
stack, dielectric, plane and net-class data are DipTrace model concerns, exported
through the DSN `(structure…(layer (type signal|power|mixed)…))` + `(rule…)`
fragments the engine writes (see `layer_stackup.htm`, `net_classes.htm`,
`via_styles.htm` in the CHM).

---

## 5. copperline integration — DSN/SES as a drop-in external router

What the evidence proves: **DipTrace already treats "an external Electra-or-Specctra
autorouter" as a program that takes a DSN file and returns an SES file.**
That is exactly the transport copperline implements (`DsnImporter` / `board_to_ses` /
`ses_merge_into`). copperline therefore needs **zero COM knowledge**:

```
Pcb.exe  ──File/Export → Autorouter DSN──▶  copperline router route board.dsn → routed.ses
Pcb.exe  ◀──File/Import → Autorouter SES──  copperline (as external router)
```

To be a faithful drop-in, copperline's DSN/SES dialect should handle the Electra-era
Specctra constructs observed in this binary set:

1. **DSN input (what Pcb.exe may emit):** `(parser …)`, `(resolution <unit> <n>)`
   (Electra prints `(resolution %s %d)`), `(structure (layer <name> (type signal|
   power|mixed) (direction …) (grid …)) …)`, `(library_out/network_out/structure_out`,
   `(padstack …)`, per-net `(net <name> (network …))`, `(wire …)`), `(rule …)` incl.
   `rule pcb (via_at_smd on/off …)`, `(clearance … (type …))` with the `*_wire`/
   `*_via`/`*_smd`/`*_pin` and `smd_via_same_net`/`via_via_same_net` variants, and
   `(attr fanout)` / `(type protect)` markers. Wiring arcs may appear as `(qarc …)`
   and bounds as `(rect …)`; tolerate `(path …)` polylines.
2. **SES output (what Pcb.exe must consume):** `(session "…" (route (library
   (padstack …)) (network (net … (wire … (path … (qarc …)))))))` plus per-wire
   `(type protect)` for wires to keep, and `@temp_net*` names for any
   fanout/intermediate nets (DipTrace's import is net-name-matched; protected wires
   are honored — Pcb.chm `autorouter_ses.htm` requires *"Do not make any changes
   after you have exported DSN file and have not imported an SES file yet"*, i.e. the
   SES must be a pure routing augmentation of the exported design).
3. **Workflow rule:** DipTrace keeps the `.dip` open and expects the SES back with no
   design edits in between — the same "route this snapshot" contract copperline's DSN
   importer already assumes. Consider exposing per-net route modes
   (Ratlines/Correct Traces/Full/Don't Route from `autoroute.htm`) via the DSN
   net attributes if Pcb.exe emits them.
4. **Optional, higher-fidelity path (no router needed):** DipTrace's XML plugin
   contract (`settings.xml` + exchange file) can carry net classes, via styles, DRC
   and diff pairs that DSN only represents partially — see `README.md` §3.3 in this
   directory. DSN/SES remains the zero-new-work golden path.

### Quirks to verify against a captured DipTrace DSN

- Whether DipTrace's DSN export emits `(boundary …)` polygons vs `(rect …)`,
  keepouts (`via_keepout`/`bend_keepout` imply keepout regions), and per-net
  `(rule …)` — check against a real export from an installed DipTrace before
  claiming full round-trip (no capture available in this offline session).
- Unit handling: DSN resolution vs copperline's integer-nm quantization
  (documented in the existing formats notes, §D).

---

## 6. Evidence cited

**CHM pages (Pcb.chm, extracted):** `autoroute.htm`, `electraspecctra_integration.htm`,
`autorouter_dsn.htm`, `autorouter_ses.htm`, `route.htm`, `routing_steps.htm`,
`simplerouterproperties.htm`, `simplerouterreview.htm`, `interactive_routing_setup.htm`,
`net_classes.htm`, `via_styles.htm`, `layer_stackup.htm`, `routing_diff_pairs.htm`.

**Observed strings:** quotes inline above with `0x…` file offsets (router.dll unless
noted). Export/import tables: `router_exports.tsv` (both DLLs).

**Fixtures copied:** `fixtures/reverse/diptrace/RouteServiceChecker.vbs` (verbatim
COM checker), `fixtures/reverse/diptrace/route_service.rgs` (recovered registration
script, <1 KB), `fixtures/reverse/diptrace/sample_settings.xml` (installer-shipped
example of the XML **plugin manifest** shape — the CompEdit plugin "Convert
Selected Shapes to Dash/Dot"; useful as a template for the higher-fidelity XML
plugin path, not a router config).