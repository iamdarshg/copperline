# DipTrace Autorouter — Module Decomposition (Clean-Room)

> **Scope.** Functional decomposition of the shape-based autorouter engine that
> DipTrace ships as its default "Shape Router". The engine is **Electra**
> (Konekt, 2004–2009), wrapped by DipTrace as a 32-bit COM service
> (`RouteService.exe` → `router.dll`) driven from 64-bit `Pcb.exe` through
> `RouteLayer.dll` (see `router_engine.md` for the interop verdict). This
> document decomposes the engine into re-implementable modules, mapping each to
> observed evidence (strings + hex offsets in router.dll / RouteService.exe /
> RouteLayer.dll / Pcb_full.exe), to Specctra-DSN/SES tokens, and to DipTrace
> CHM documentation. All offsets are file offsets (hex); the binaries are
> static-analysis only, never executed.
>
> Companion evidence table: `autorouter_module_strings.tsv`.

**Confidence legend.** `[confirmed]` = direct observation (export table,
literal string, official CHM quote). `[inferred-from-strings]` = plausible
reading of observed strings/grouping without control-flow proof.
`[docs]` = stated in Pcb.chm (authoritative for user-visible behavior).

**Coverage note.** router.dll bundles a full Qt GUI shell, PNG/JPEG codecs,
and a Qt unit-test harness — those are *host/presentation* concerns, not
router modules, and are excluded. What is decomposed below is the routing
pipeline and its data model.

---

## 0. Module map (one-paragraph architecture)

Data flow inside `router.dll`:

```
DSN file ──► M1 DSN parser ──► Board Model (M2 shapes, M3 net classes, M4 via/layers)
electra_command / electra_dofile ──► command dispatch (script dialect)
autoroute ──► M6 Pass Orchestrator
               ├─ M7 Fanout (SMD) + memory/bus pre-route
               ├─ M5 Costed expansion engine (per net: shape-based search)
               ├─ M9 Clean / Filter / Miter (init_clean, run_filter, run_miter)
               └─ final_route / final_clean
SES writer (M10) ──► session file via host file transport
RouteService/RouteLayer (M11) ──► COM + temp-file transport, _IRouterEvents
```

The second engine — the entry-level **Grid Router** — is a separate, simpler
router embedded in `Pcb.exe` (Delphi) and is decomposed briefly in M12.

### Evidence summary (observed, router.dll)

Self-identification: `0x2e211c "Adaptive Shape-based AutoRouter"`;
`0x39f8b2 " Adaptive Shape-Based AutoRouting Software "`;
`0x39f8df " www.konekt.com "`; `0x39f8f1 " Copyright 2004-2009"`.
Command-line batch mode: `0x2e1fa0 "Usage: <application> [-design <design_file>]
[-do <do_file>] [-w <routes_file>] [-nog] [-quit]"` — the classic Electra
headless form (`-design` open DSN, `-do` command file, `-w` import routes
file, `-nog` no GUI).

---

## 1. DSN parser / board-model loader

**Purpose.** Parse the Specctra-DSN `(pcb …)` dialect delivered by
`electra_load`, and a variant reader for `.ses` (session) and `.rte`
(route-only) files; build the internal board model. The standalone app
filenames confirm three input kinds: `"Design files (*.dsn *.ses)"` (0x2e45c)
and `"Routes Files (*.rte *.RTE)"` (0x2e2b8), with progress strings
`"Loading design file..."` / `"Loading routes file..."` /
`"Loading placement file..."` (0x3b1204/0x3b1234/0x3b1264).

**Inputs.** DSN text stream (or SES/RTE). **Outputs.** Board model object:
layers, board boundary, placements (component → pin → pad), padstack library,
nets (+wires), rules, regions/keepouts, `base_design` reference.

**Key data structures / tokens** (parser keyword table, 0x3b17d8–0x3b1a18):
`property`, `position`, `lock_type` (placement records); `logical_part_mapping`
`component`, `component back`, `component front` (placement side);
`layer_rule`, `terminal`, `grid wiring`, `set_color`, `mirror_first`,
`rotate_first`, `flip_style`, `absolute` (component/terminal attributes);
`padstack`; keepout kinds `place_keepout` `bend_keepout` `wire_keepout`
`via_keepout` + `conductor`; top sections `host_cad`, `resolution`,
`network_out`, `part_library`, `floor_plan`, `placement`, `structure`,
`constant`, `base_design`. Writer-side counterparts confirm the same section
set: `(resolution %s %d)`, `(library_out…`, `(structure_out…`,
`(network_out…` (0x2e8728/0x2e8558/0x2e86f4/0x2e86e0), `(parser…` (0x2e8428).

**Algorithm.** Recursive S-expression tokenizer with line/say-tracking.
Parser diagnostics (0x3b1298–0x3b1cec): `"Opening quote not closed"`,
`"Line too long"`, `"Parsing error at line:"`, `"Coordinate out of boundary"`,
`"Undefined Net in routes section"`, `"Undefined layer: %s at line: %ld"`,
`"Undefined Net in wire section"`, `"Undefined padstack for Via"`,
`"Undefined padstack"`, `"Undefined part"`, `"Undefined pin id"`,
`"Undefined refdes"`, `"Warning reading file on line %ld!"` (tolerant mode).
Refdes/pin `-` delimiter handling: `"No '-' delimiter found"` +
`"If Component name contains '-', type in dash_cmp before reading design"` /
`"…dash_pin…"` (0x3b1479/0x3b14c1) — a parser mode that splits
`refdes-pin` on the first `-`. Placement-less designs tolerated with warning:
`"Warning - No placement data found !"` (0x3b16b0). Net-pair/fromto rule
tokens: `"Wrong syntax for net pair"`, `"Undefined fromto pin pair"`,
`"Undefined net in first net pair"` / `"…second…"` (0x3b16d8–0x3b1798).
Base-design reference: `"Parsing Error: Base Design %s not found"`,
`"base_design"` (0x3b1c28/0x3b1c60); SES files carry
`(base_design %c%s%c)` (0x2e874c).

**Endpoint for copperline.** DSN in / SES out is the drop-in contract; the
parser must accept the `(pcb` sections above plus `(rules` / `(route`s`)
(writer tokens `(rule…`, 0x2e86b8; `(routes %s`, 0x2e8740).

---

## 2. Shape / obstacle representation (copper model)

**Purpose.** Represent every conductor and keepout as a geometric object so
routing and DRC work on shapes instead of a fixed grid. This is Electra's
defining property ("shape-based") and drives both the search (§5) and DRC.

**Inputs/Outputs.** Board model → per-layer obstacle set; DRC object pairs →
clearance checks.

**Key data structures** (geometry tokens, writer 0x2e8260–0x2e83ec):
`(qarc %s %s %s %s` (arcs, 0x2e8260), `(rect %s %s %s %s %s)` (0x2e831c),
`(circ %s %s %s %s)` (0x2e8308), `(poly %s %s ` (0x2e833c),
`(path %s %s ` (0x2e834c), `(wire …`, `(via %s ` (0x2e83ec),
`(shape ` (0x2e8400), `(padstack %s%s` (0x2e8414), ` (terminal wire)`
(0x2e839c), ` (contact %s)` (0x2e83d0), ` (net %s )` (0x2e83e0),
`%s    (connect …` (0x2e83b0), `(tjunction …` (0x2e861c). Via geometry
named `"VIA %s round 1-16"` (0x2e8578) — padstack names carry shape+diameter
suffixes. Parity checks on widths: `"Wire width is too small"`,
`"Wire width is too big"`, `"Wire to Wire Clearance is too small"`
(0x2e1b14/0x2e1b50/0x2e1b2c), `"Shape Outside Board Boundary"`,
`"Found unconnected shape"`, `"Nets shorting at pin"` (0x2e1cb8–0x2e1d38).

**DRC object-pair types** (0x3a0e44–0x3a0ec4, `[confirmed]`):
`Via_to_Keepout`, `Via_to_Via`, `Wire_to_Keepout`, `Wire_to_Via`,
`Wire_to_Wire`, `Pin_to_Keepout`, `Pin_to_Via`, `Pin_to_Wire`,
`Pin_to_Pin`, `Wire_to_Pin`. These are exactly the pairs the class-to-class
clearance matrix (§3) indexes.

**Wire ownership / protection** (SES `(type …)` markers + UI):
`%s    (type protect)` (0x2e82b4), `(type soft)` (0x2e82cc),
`(type route)` (0x2e82e0), `(type fix)` (0x2e82f4), `(attr bus)`
(0x2e828c), `(attr fanout)` (0x2e82a0). Semantics `[inferred-from-strings]`:
*protect* = never touch (UI: `Protect/UnProtect All/Selected Wires`,
`"Net %s is protected against deletion"` 0x2e1b80, `ProtectedFreePad`
0x2e1ba8); *fix* = frozen; *soft* = removable by later passes; *route* =
router-owned wire. `(attr fanout)` marks netlets created by the fanout module.

**Algorithm.** Obstacles are stored as polygons/rects/arcs per layer; DRC
inflates each shape by the paired clearance and tests geometric overlap.
The `add region` command exposes the manual region object:
`"Syntax: add region <layer> <width> <clearance> <xlo> <ylo> <xhi> <yhi>"`
(0x39ef80) with `"Wrong rectangle definition"`, `"Wrong clearance"`,
`"Wrong width"` (0x39efc8–0x39eff4) — region = width+clearance band, matching
the `(clearance X (type %s_area))` rule form (§3).

## 3. Net class & rule resolution

**Purpose.** Resolve every routing decision's numeric parameters: wire width,
clearance per object-pair type, via policy, topology, priority, layer
restrictions — from the DSN `(rule …)` / `(net_class …)` data. DipTrace's own
net-class model (per-layer MinWidth/MaxWidth/clearance, AllowedVias,
RoutePriority, RouteMaxVias, RouteAllLayers, Class-to-Class matrix) is
serialized into these DSN rules and re-parsed here (`[docs]`: net_classes.htm;
Delphi RTTI `FMinWidth`/`FMaxWidth`/`MinWidth`/`MaxWidth` at 0x001e845f/
0x001e842c/0x001e85f1/0x001e858a in Pcb_full.exe).

**Key data structures.** Per-net-class properties; a per-pair-type clearance
table keyed by the object-pair names of §2; via-style policy flags.

**Observed rule/gap keys** (0x3b151c–0x3b1650, `[confirmed]` string table):
`limit_bend` `limit_cross` `limit_via` (search-side limits),
`smd_to_via_gap` `via_to_via_gap` `drill_gap` `antipad_gap`
`non_planar_via_gap` `buried_via_gap` (geometric gaps),
`smd_to_turn_gap` `pin_to_turn_gap` `pad_to_turn_gap` (bend/turn keepouts),
testpoint pairs `testpoint_testpoint` `wire_testpoint` `via_testpoint`
`smd_testpoint` `area_testpoint` `pin_testpoint` (testpad clearances),
same-net relaxations `via_via_same_net` `smd_via_same_net`.

**Rule writer formats** (`[confirmed]`): `(clearance %s (type %s_area))`,
`(type %s_wire)`, `(type %s_via)`, `(type %s_smd)`, `(type %s_pin)`
(0x2e8434–0x2e84c4), same-net `(clearance %s (type smd_via_same_net))` /
`(type via_via_same_net)` (0x2e862c/0x2e865c), bare `(clearance %s)`
(0x2e868c), `(width %s)`, `(rule …` (0x2e86a4/0x2e86b8), and the pcb-rule
prologue `rule pcb (via_at_smd on …` / `…off …` (0x3a0864/0x3a0894) followed
by ` (via_share on)` ` (pin_share on)` ` (direction out/in)` ` (grid on/off)`
` (fit on)` (0x3a0808–0x3a0888). Note Pcb_full.exe also emits its own
`(rule (via_at_smd on …` (0x01539668) — both sides write these rules.

**Command vocabulary** (0x39e8c4–0x39f798, `[confirmed]` strings):
`starburst` `daisy_chain` `term_only` `max_stub` `junction_type`
`joint_type` `tjunction` `max_stagger` (topology), `via_on_pin`
`via_at_smd` (0x39e964/0x39e970), `clearance` (0x39e97c), `max_total_vias`
`limit_bends` `limit_crossings` `limit_vias` `limit_way` (0x39e860–0x39e8b0),
`priority` (0x39f2f8, `"Priority ranges from 1 to 255"` 0x39eedc),
`use_layer` (0x39f2ec), `direction` (0x39f568; values `ORTHOGONAL`/
`VERTICAL`/`HORIZONTAL` 0x39f544–0x39f55c), `side_exit` `off_center`
`off_grid` (0x39f154–0x39f16c), `autopair`, `pin_type`, `via_share`,
`pin_share`, `share_len`, `unprotect`, `components` (0x39f1f8).

**Algorithm.** Class→rule resolution: a net is looked up by class; per-layer
width/clearance fall back to class-wide values; class-to-class clearance
wins over class clearance (`[docs]` net_classes.htm: "class-to-class
clearances have higher priority than regular net class clearances").
`same_net` entries relax gaps between objects of one net; `testpoint` entries
drive via/pad test-point clearances.

## 4. Via style / layer stack management

**Purpose.** Choose and validate padstacks for vias during routing; enforce
the via-insertion policy; represent layer direction/grid participation in the
search.

**Evidence (`[confirmed]` strings).** `"Unknown Via"` `"Unknown via name"`
`"Undefined Via Type"` `"Routing with undefined Via"` `"Undefined padstack
for Via"` (0x3b1664, 0x39f1a0, 0x39eefc, 0x2e8106, 0x3b13d8); via record
`   (via %s ` + `  )` (0x2e83ec); via policy knobs `via_at_smd`
`via_at_smd_fit` `via_at_smd_grid` `via_share` `sharing_via` `sharing_pin`
`share_dist` `share_len` `via_on_pin` `via_testpoint` `max_total_vias`
`non_planar_via_gap` `buried_via_gap` (offsets §3; 0x3b1590/0x3b1618/0x3b162c).
Strategy-dialog controls `chkViaAtSMD`, `chkViaAtSMDGrid`, `bgrpViaAtSMD`,
`chkSharingVia`, `chkSharingPin`, `chkFitInside` (0x3a7768–0x3a77a8,
`[confirmed]` object names).

**Docs mapping.** via_styles.htm: via style = type (through / blind/buried),
layer pair (Lay1/Lay2 for blind/buried), outer (copper ring) and hole
diameter; first style is always "Default", through. The DSN carries only layer
`(type signal|power|mixed)` (0x2e8538/0x2e851c/0x2e8500) + `(direction in/out)`
+ `(grid on/off)` per layer — so the engine's *layers* are the signal/power/
mixed set with a routing direction; dielectric/plane topology stays in the
DipTrace model (`[docs]` layer_stackup.htm). Blind/buried via layer spans are
a per-style property in DipTrace, serialized through the via padstack names
(`VIA %s round 1-16` naming, 0x2e8578) and consumed via `buried_via_gap`.

**Algorithm.** When the search (§5) wants a via on layer pair L↑/L↓, it picks
a via style: `via_at_smd` = allow via directly on SMD pad (on/off rule),
`via_at_smd_fit` = only if pad fits, `via_at_smd_grid` = snap via to routing
grid, `via_share`/`sharing_via`/`share_dist` = reuse an existing via of the
same net within `share_dist` instead of adding one. Error paths (`"Routing
with undefined Via"`) force a defined default style.

## 5. Costed expansion engine (shape-based search)

**Purpose.** Route one net from target/seed through the shape obstacle set,
minimizing a cost function under a total-cost bound. This is the core module;
the pass orchestrator (§6) only chooses *which* nets and *when*.

**Evidence (`[confirmed]` strings).** Cost model: `"cost <= total_cost"`
(0x317ce0, an assertion inside the search — the accumulated cost must not
exceed the total budget), `"Max cost"` (0x39e98e), `"Using default cost"`
(0x39e9b8), `"Missing cost argument"` (0x39ee58), `cost` command keyword
(0x39f358). Bend constraint: `SET WIRE_BEND 3` (0x2e85e8) — wire bend angle
limit emitted into the command stream. Optimization flag `SET OPTIMIZING ON`
(0x2e85d4). Search-side limit knobs: `limit_bends` `limit_crossings`
`limit_vias` `limit_way` (0x39e878–0x39e8b0) and `max_total_vias` (0x39e860).
Swap / virtual-memory support for the search's database:
`"Opened temporary file %s"` `"Closed temporary file %s"` `"Failed to create
temporary file %s"` (0x3652a4/0x3652c0/0x3658e4), `"Write failed on
temporary file --- out of disk space?"` `"Seek failed on temporary file"`
`"Read failed on temporary file"` (0x36586c–0x3658c4), plus legacy
`"Write to XMS failed"` / `"Read from XMS failed"` / EMS variants
(0x3657a8–0x365d74) — the tile/expansion database is file-backed.

**Algorithm (in words; `[inferred-from-strings]`, consistent with the
shape-based family).** From each unrouted pin the engine performs a wave/
expansion over legal shape-to-shape moves; every node carries an accumulated
cost = wire length + bend penalty (bends constrained by WIRE_BEND) + via cost
(vias counted against `limit_vias`/`max_total_vias`, via-at-SMD/sharing
reduces it) + crossing penalty (`limit_crossings`) + layer-direction penalty
(`(direction in/out)` preference, `priority`) . Expansion stops for a node
when its cost exceeds `total_cost` (assert confirms the invariant); the
lowest-cost complete connection is committed, or the net is left unwired for
a later pass (the pass table counts `Failures` / `Unwired`). `sharing_via` /
`sharing_pin` / `share_dist` add a reward for reusing an existing object
within a distance — this is how via sharing reduces expansion cost.
The file-backed store holds the expansion tiles, so huge boards swap
gracefully.

**DSN/SES mapping.** `cost` command; `SET WIRE_BEND`; the per-layer
`(direction …)` / `(grid …)` rule fragments; via-policy rule fragments §4;
net-topology commands `starburst`/`daisy_chain`/`term_only` (§3).

## 6. Pass orchestrator (strategy scheduler)

**Purpose.** Execute the re-route "campaign": which algorithm families run,
how many passes each gets, in what order, and stop/abort handling.

**Evidence (`[confirmed]`).**
- Pass-count knobs (0x39f63c–0x39f698): `init_route_passes`
  `init_clean_passes` `filter_passes` `fanout_passes` `final_route_passes`
  `final_clean_passes`.
- Runners (0x39f734–0x39f798): `run_miter` `run_filter` `run_final_clean`
  `run_final_route` `run_init_clean` `run_init_route` `run_fanout`
  `run_memory`.
- Ranges: `"Passes ranges from 1 to 1000"` (0x39ee20), `"Start_pass ranges
  from 1 to 1000"` (0x39f04c) — pass number offset.
- `autoroute` command keyword (0x39f630); busy states `"Routing..."`
  `"Fanout Routing..."` `"Cleaning..."` `"Filtering..."` `"Checking..."`
  `"Bus Routing..."` `"Chamfering Corners..."` (0x39e31f–0x39f544).
- Pass-table "Algo" column values (0x39eaa8–0x39eb14): `Fanout`, `Recorner`,
  `Bus`, `Clean`, `Filter`, `DRoute`, `Route` (DRoute = diagonal-route family).
  Table header (0x39eb80): `|  #  |   Algo   | Shorts |DRC Errs|Failures|
  Unwired |  Vias  |  %Red  |  Time   |  Total  |` — the per-pass KPI row.
- Strategy dialog (RouterDlg) groups (`[confirmed]` object names,
  0x3a7524–0x3a7910): **Pre-Route** (`grpPreRoute`: `chkMemoryRoute` "Bus
  Router (colinear pins)", `chkFanout`/`spinFanout` "SMD Fanout" with
  `edShareWithinDist` `edFanoutMaxLength` `cbFanoutDir` `chkFitInside`
  `chkViaAtSMDGrid` `chkViaAtSMD`), **Initial** (`InitialGroup`:
  `chkInitRoute`/`spinInitRoute`, `chkInitClean`/`spinInitClean`),
  **Post-Route** (`FinalGroup`: `chkFilter`/`spinFilter` "Filter",
  `chkFinalRoute`/`spinFinalRoute` "Final Route",
  `chkFinalClean`/`spinFinalClean` "Final Clean", `chkMiter`
  "Miter corners to 45", "Recorner"; `CleanGroup`).
- Status/API: `setStatusPass(int,int)` `setStatusConnect(int,int)`
  `setStatusConflict(int)` `setStatusCompletion(double)` (0x3a67a4–0x3a689c);
  IRouterEvents `UpdateStatusChanging(currentPass, totalPass, totalWire,
  succeedWire, conflicts, completion)` (RouteService typelib, §11).

**Algorithm (in words).** The `autoroute` command schedules, in order,
Pre-Route (memory/bus route for colinear-pin networks first — `chkMemoryRoute`
— then SMD fanout), Initial routing (`init_route_passes` loops of
route-family), `init_clean` between route batches, optional `filter`
passes that drop suboptimal alternatives, `final_route_passes` to finish
unwired nets, `final_clean`, and miter (corner chamfer to 45°). `Start_pass`
skips leading passes; every family count is 1..1000. A pass can leave
conflicts/shorts (`Conflicts:`, Shorts column) that downstream passes absorb —
that is the multi-pass contract (see §8: no rip-up).

## 7. Fanout module (SMD fanout pre-router)

**Purpose.** Give every SMD pad of fanout-eligible nets a short stub + via so
the main search can start from a via on an inner layer — and tag those
netlets so the SES writer marks them.

**Evidence (`[confirmed]`).** `run_fanout`, `fanout_passes` (0x39f78c/
0x39f64c); knobs `fanout_dist` `fanout_dir` (0x39f6b8/0x39f6c4);
`"Fanout Routing..."` (0x39f364); Fanout-options dialog `edFanoutMaxLength`
`cbFanoutDir` `chkFitInside` `chkViaAtSMDGrid` `chkViaAtSMD`
"Share Within Dist:" "Max Length:" "Direction:" (0x3a7524–0x3a77a8);
SES/DSN marker `%s    (attr fanout)` (0x2e82a0); temp-net naming
`@temp_net` / `@temp_net%d` (0x2e86d4/0x2e1c10) + `@ALONE_PIN_NETS@`
`@UNUSED_PINS@` (0x2e820c/0x2e8220); `sharing_via` `sharing_pin` `share_dist`
(§4) — fanout reuses neighbor vias within `share_dist`.

**Docs.** fanout.htm: fanout of pattern (SOIC/QUAD, BGA — All Pads, BGA —
By Rows), placement (outside / inside / zigzag), pads (all / one side), Use
Connected Pads Only; fanout of net or single pad.

**Algorithm (in words).** For each SMD pad: emit a short wire to a via placed
per `fanout_dir`/`fanout_dist` (or on-pad when `via_at_smd` on), snapped to
grid when `via_at_smd_grid`; the stub+via become an `@temp_net` netlet marked
`(attr fanout)`; within `edShareWithinDist` an existing net via is shared
instead. The temp-net naming and `(attr fanout)` marker survive into the SES
output so the import side (DipTrace) can recognize fanout artifacts; a
`(route …)`/`(net …)` record group is produced per net.

## 8. Rip-up / reroute strategy

**Verdict: the shape-based engine has NO classic rip-up-and-retry.**
`[confirmed]` by absence: grep across router.dll for rip-up/ripup/retry/
maze produces nothing (only Qt/PNG/JPEG false hits). The strategy is instead
multi-pass: each pass covers a subset of nets, tolerates conflicts
(`Conflicts:` counter; `Shorts`/`DRC Errs`/`Failures`/`Unwired` pass columns;
`"Found Unconnected Wire"` 0x2e87d0), and later passes re-clean/re-route —
with wire state marked `soft`/`route` so those wires may be removed by the
next pass (`(type soft)`, `(type route)`, §2). `limit_way`/`limit_crossings`
bound individual searches so a failing net yields instead of poisoning the
board.

The **grid router** (M12) *does* implement explicit rip-up-and-retry
(`[docs]` simplerouterreview.htm) — a separate engine inside Pcb.exe, not
router.dll (see M12).

## 9. Cleanup / optimization passes

**Purpose.** Post- and inter-pass polish: shorten, straighten, de-via, and
chamfer.

**Evidence (`[confirmed]`).** `run_init_clean` `run_final_clean`
`init_clean_passes` `final_clean_passes` (0x39f74c–0x39f76c, 0x39f684/
0x39f65c); `"Cleaning..."` progress (0x39f51c); `run_filter` `filter_passes`
`"Filtering..."` (0x39f740/0x39f63c/0x39f504); `run_miter` `"Miter corners to
45"` `"Chamfering Corners..."` + `recorner`/`orthogonal` commands
(0x39f734, 0x3a75bc, 0x39f228, 0x39f260/0x39f240); status label `Clean`,
`Filter`, `Recorner` (0x2e81e4/0x2e81ec/0x2e81d0); dialog checkboxes
`chkInitClean`/`chkFinalClean`/`chkFilter`/`chkMiter` + spin counts
(`spinInitClean`/`spinFinalClean`/`spinFilter`, 0x3a77c0–0x3a7858).
`Routed Length =` report metric (0x39ecac); `SET OPTIMIZING ON` (0x2e85d4).

**Algorithm (in words).** *Clean* iterates routed nets: removes redundant
bends, collapses collinear segments, and retries with fewer vias (respects
`limit_vias`), recomputing `Routed Length` and %Reduction (`%Red` column =
reduction vs. previous pass). *Filter* deletes inferior alternatives from the
set of identical-route candidates (multi-candidate wires per net are an
Electra device to allow swap-out). *Miter/recorner* converts 90° corners to
chamfered 45° mitered corners (`OSET`/orthogonal modes), subject to
`limit_bend`.

## 10. SES writer / session output

**Purpose.** Write the routed result as a Specctra session file (`.ses`) that
the host can merge by net name, augmenting the original design — never
restructuring it.

**Evidence (`[confirmed]` writer formats, 0x2e824f–0x2e877c).**
- Header: `(session %c%s%c%s …` (0x2e8764 — quoted session name), then
  ` (base_design %c%s%c)%s` (0x2e874c — the original design file it annotates)
  and ` (routes %s` (0x2e8740).
- Records: ` (net %s )` (0x2e83e0), ` (contact %s)` (0x2e83d0),
  ` (terminal wire)` (0x2e839c), `%s    (connect …` (0x2e83b0),
  `   (wire …` (0x2e83c0), `   (via %s ` + closing `  )` (0x2e83ec),
  ` (shape …` (0x2e8400), `  (padstack %s%s` (0x2e8414),
  `(Net %s %s …` (0x2e86c4), `(routes …` (0x2e8740); geometry
  `(path %s %s ` / `(qarc %s %s %s %s` / `(rect …)` / `(circ …)` / `(poly …`
  (§2); wire attributes `(type protect|soft|route|fix)`, `(attr bus|fanout)`
  (§2).
- Merge semantics: net-name matching (`(Net <name> …`) with per-net wire
  groups; protected wires (`(type protect)`) must pass through unchanged;
  `@temp_net` fanout groups are written for temp nets. `"Found Unconnected
  Wire"` (0x2e87d0) is reported for wires lacking net/side, feeding SES
  validation. Save flow strings: `"Do you want to save to the following
  session file ?"`, `"Saving aborted"`, `"Session Files (*.ses *.SES)"`
  (0x39e408/0x39e246/0x39e258).

**Docs / contract.** autorouter_ses.htm + autorouter_dsn.htm: "Do not make
any changes to the design after you have exported DSN file and have not
imported an SES file yet" — the SES is a pure routing augmentation of the
exported snapshot, keyed by net name; Pcb.exe merges `(wire…)/(via…)` into
nets, honors protected wires, and ignores structure edits.

## 11. COM / transport layer (host bridge)

**Purpose.** Move DSN in, SES out, and progress/redraw events between the
64-bit DipTrace host and the 32-bit engine across a COM boundary, using
temp files for bulk payload.

**Evidence (`[confirmed]`).**
- RouteService.exe (x86, ATL) imports `electra_init` `electra_sethost`
  `electra_load` `electra_dofile` `electra_command` `electra_abort`
  from router.dll and hosts class `CRouterObject`; RTTI
  `IElectraHost`/`IElectraHostDerived`, `CRouteServiceModule` (0x1e5b0+ region
  in routesvc_strings). ProgID `RouteService.RouterObject`, CLSID
  `{5BDF40FD-F1A5-47C4-A52D-171ABE029342}`, TypeLib RouteServiceLib
  `{E917AEE4-D143-4E20-B930-6D47FE655016}` (registration strings
  0x1f632–0x1f898).
- `IRouterObject` (typelib, 0x20224+): `GetVersion`, `Electra_Init`,
  `Electra_SetHost`, `Electra_Load(design_file)`,
  `Electra_DoFile(do_file)`, `Electra_Command`, `Electra_Abort`,
  `Napi_version`, `SetTempPath` — a thin automation façade over the six
  `electra_*` exports.
- `_IRouterEvents` (typelib, 0x1ff18+): `AddViaChanging(netName, pstkName,
  fprotect)`, `AddPathChanging(layerName, hNwidth)`, `AddMarkChanging`,
  `UpdateStatusChanging(currentPass, totalPass, totalWire, succeedWire,
  conflicts, completion)`, `UpdateDisplay_Changing`,
  `DeleteAllVias_Changing`, `DeleteAllPaths_Changing`,
  `DeleteAllMarks_Changing`, `LogMessage_Changing(mType, message)`.
  These mirror Electra's host display callbacks (mark/path/via/status/log)
  and are what DipTrace uses for live redraw + progress
  (`setStatusPass/Connect/Conflict/Completion`, §6).
- RouteLayer.dll (x64): exports `Init_Host Init_display Init_log Init_mark
  Init_path Init_status Init_via Init_Route Abort_Route Load_File Do_File
  Send_Comm SetTempPath Destroy` (export table) — i.e. it *is* the Electra
  host implementation on the DipTrace side, forwarding events to the
  `CRouterEventSink` ATL sink (RTTI `CRouterEventSink` 0x18d58). PDB path
  `C:\Users\sslav\Desktop\Router\_Release\Win64\bin\RouteLayer.pdb`
  (0x15758) — DipTrace-side build. Imports KERNEL32 file I/O
  (`CreateFileW SetFilePointerEx WriteFile FlushFileBuffers CloseHandle`) for
  the temp-file channel and ole32 (`CoCreateInstance OleRun`) to activate
  RouteService.
- Temp-file transport in engine: `Opened/Closed temporary file`,
  `Failed to create temporary file`, `Write/Seek/Read failed on temporary
  file` (0x3652a4–0x3658e4, §5) — the host's file channel (SetTempPath →
  electra_sethost hook) carries design/session payload; control/status goes
  over COM methods + events.

**Data flow.** `Pcb.exe → RouteLayer.dll → [COM] RouteService.exe → router.dll
(electra_*)`; temp files via host path; progress via `_IRouterEvents`.
**Re-implementation note:** copperline needs none of this — the external-router
contract is DSN in / SES out (§1, §10).

## 12. Grid router (second, simpler engine) — brief decomposition

**Where it lives.** Inside `Pcb.exe` itself (Delphi), not router.dll.
`[confirmed]` menu strings at 0x027466f6 / 0x028ed411 ("Grid Router") and
0x0272c147 ("Shape Router") + `N_RunAutorouter` action names
(0x0224336b etc.); the CHM documents it under the same "Autorouting" tree.
The two routers share the *dispatch* (`Route ▸ Current Autorouter`,
`Route ▸ Run Autorouter` — menu strings 0x02246464 "SBAutoRoute" = shape
router id) but have disjoint engines.

**Identity (`[docs]`).** simplerouterreview.htm: "Grid Router is a grid-based
autorouter with 'rip-up-and-retry' technology… three basic autorouter steps:
trace placement with 'rip-up and retry' technology, trace optimization, and
trace correction." Four presets: Quick (default), Normal, Good, Best.

**Suggested modules (doc-derived, `[docs]` unless noted):**
1. *Grid model + trace placer* — maze router on the routing grid with
   `First Route Limit` / `Second Route Limit`: maximum ratio of trace length
   to start-to-end-point spacing (rip-up-and-retry parameters);
   `Maximum Number of Modifications` bounds reroute attempts; `Angles`
   90°/45° or 90° only; `Number of Layers` / `Use All Layers` /
   `Route without Vias`; `Use Priority Layer Directions` (per-layer preferred
   direction, `[inferred]` — mirrors Electra's `direction`/`priority`).
2. *Grid-shift attempt engine* — `Matrix Shift and Choice`: shift the routing
   grid to find the best variant; `Number of Options` = attempts per shift;
   `Via Value` = length-equivalent cost of a via used in trace-length
   accounting.
3. *Optimizer* — `Number of Optimizations` trace-optimization cycles;
   `Correct Angles` (delete sharp angles) / `Correct Traces` (correction step,
   steps per attempt, cycles).
4. *Jumper-wire support* — `Allow Jumper Wires`, `Forbid jumpers under
   Patterns`, `Jumper Wire Direction` (vertical/horizontal); single-layer
   only. jumper_wires.htm: maskless vias at start/end of the jumper segment;
   Pcb strings `Jmp`-related and `Check_Jumpers` (0x00dfdf7f) confirm
   jumper DRC integration.
5. *Result hand-off* — writes traces directly into the DipTrace board model
   (no DSN/SES); DRC auto-verify after routing is optional
   (`autoroute.htm`: "Route ▸ Current Autorouter ▸ Run DRC after
   Autorouting").

---

## Appendix A. String evidence table

See `autorouter_module_strings.tsv`: every string cited above with its
binary, file offset, and the module(s) it evidences. Offsets are file offsets
in the shadow copies listed in `router_exports.tsv`'s header comments.

## Appendix B. Confidence summary

| Module | Base evidence | Confidence |
|---|---|---|
| M1 DSN parser | keyword/error tables, loader strings | confirmed |
| M2 shapes/DRC pairs | geometry writers, pair-name list | confirmed |
| M3 net class/rules | rule writers, gap-key table, docs | confirmed (semantics of `limit_*` inferred) |
| M4 via/layers | via strings, rule fragments, docs | confirmed |
| M5 costed expansion | cost strings, XMS/pfile, WIRE_BEND | confirmed existence; algorithm inferred |
| M6 pass orchestrator | *_passes, run_*, dialog, table | confirmed |
| M7 fanout | run_fanout, attr fanout, dialog, docs | confirmed |
| M8 rip-up | absence scan + grid-router docs | confirmed absence in engine |
| M9 clean/filter/miter | runners, progress strings | confirmed |
| M10 SES writer | writer formats, session header | confirmed |
| M11 COM transport | typelib events/methods, exports | confirmed |
| M12 grid router | CHM docs + menu strings | docs (engine internals inferable only at schematic level) |

## Appendix C. Cross-references

- CHM pages: autoroute.htm, autorouter_dsn.htm, autorouter_ses.htm,
  routing_steps.htm, net_classes.htm, via_styles.htm, layer_stackup.htm,
  routing_diff_pairs.htm, interactive_routing_setup.htm,
  simplerouterproperties.htm, simplerouterreview.htm, fanout.htm,
  route.htm, jumper_wires.htm.
- Sibling docs: router_engine.md (interop + copperline plan),
  router_exports.tsv, autorouter_module_strings.tsv (this set).
- Copperline leverage: M1 (DSN dialect), M10 (SES dialect) are the whole
  external API; M2/M3/M5 give the internal design targets for a clean-room
  shape-based engine; M6/M7/M9 give the pass strategy UI contract DipTrace
  exposes ("Autoroute Strategy" dialog) for feature parity.

