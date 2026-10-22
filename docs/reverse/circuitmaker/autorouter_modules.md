# CircuitMaker Autorouting System — Clean-Room Module Decomposition

**Scope:** the Specctra-family DSN/SES codec (`Tds_File` -> `tds_file.dll`,
`tds_dss.dll`), the external-router bridge (`Routecct` -> `RouteSupport.dll`),
and the in-process Situs topological router (`RouterEntry.dll`,
`IntGeometryEntry.dll`, `Situs.INI`, `route.rul`, `SitusTemplate.Bas`,
`padinit.dat`) as shipped in CircuitMaker 2.2.1.6.

**Purpose:** architecture baseline for a clean-room re-implementation of the
Altium-family autorouting pipeline inside copperline. Written entirely from
public documentation (Altium/Cadence manuals, Wikipedia, KiCad docs,
LayoutEditor, FreeRouting/tscircuit ecosystem) and from byte-level facts
observed in the CircuitMaker installer manifest (`Installation.Release`).

**Date of analysis:** 2026-10-21 · **Analyst:** copperline RE subagent (clean-room)

---

## Confidence legend

| Marker | Meaning |
|---|---|
| [confirmed] | Directly observed in the CircuitMaker installer manifest/PE artifacts (byte-level fact) |
| [public-docs] | Stated in public vendor or third-party documentation (Altium docs, Cadence Specctra Design Language Reference, Wikipedia, KiCad dev-docs, LayoutEditor, FreeRouting docs) |
| [inferred] | Reasonable conclusion from the above; not directly documented; flagged for verification |

---

## 1. Executive summary — which engine does CircuitMaker ship?

**CircuitMaker 2.2.1.6 ships BOTH router runtimes, but only the Situs
topological autorouter is user-facing.** [confirmed for packaging, public-docs
for usage]

Evidence from the installer manifest (package `<files>` lists are
manifest-reported facts, [confirmed]):

| Package | Payload | Role |
|---|---|---|
| `Base_PCBUtils - Tds_File` | `System\tds_file.dll` (7,026,688 B), `System\tds_dss.dll` (6,965,248 B) | Specctra DSN/SES/RTE format codec ("TDS" = The Design System, Cadence Specctra heritage) |
| `Base_PCBUtils - Routecct` | `System\RouteSupport.dll` (14,873,088 B), `System\RouteCct.Do` (1,020 B), `System\RouteSupport.INS` (1,181 B), `System\RouteSupport.OUT` (641 B), `System\routecct.ico` | External Specctra-compatible router integration: DO-command file + integration-server (.INS/.OUT) glue |
| `Base_PCB - Gloss` | `RouterEntry.dll` (6,915,584 B), `IntGeometryEntry.dll` (362,496 B) | Situs in-process router entry point + integer geometry kernel (post-route "gloss" package name) |
| `Base_PCB` | `System\Situs.INI` (2,251 B), `System\SitusTemplate.Bas` (695 B), `System\route.rul` (375 B), `System\padinit.dat` (455,079 B), `System\DefaultRules.rhv`, `System\DefaultPadsPinDecals.c`, plus `Advpcb.dll`, `SpatialIndex.dll`, `rtree.dll`, `OpenMCDF.dll`, `Situs.ico/png` | Core PCB editor + Situs router configuration files |
| `Base_PCB - PcbDlgs` | `System\pcbdlgs.dll` (18,488,320 B), `System\xSignals.dll` | PCB dialogs — the Situs Routing Strategies / Strategy Editor dialogs live here |
| `Base_SchUtils - SchRouter` | (package present) | Schematic-side router extension (schematic routing, not PCB) |

Observations that settle the "Routecct vs Situs" question:

1. CircuitMaker's own documentation describes **its** autorouter as
   topological: *"The CircuitMaker autorouter is topological ... it triangulates
   the space between adjacent obstacles"* and names it the **Situs Autorouter**
   (Tools | AutoRoute sub-menu, Situs Routing Strategies dialog, Situs Strategy
   Editor). [public-docs]
2. The manifest physically ships `Situs.INI` + `SitusTemplate.Bas` + `route.rul`
   (Situs config) in `Base_PCB` and `RouterEntry.dll`/`IntGeometryEntry.dll`
   in the Gloss package — the Situs engine is installed in-process. [confirmed]
3. The manifest ALSO ships the legacy external Specctra flow (`tds_file.dll`
   codec + `RouteCct.Do` DO-file + `RouteSupport.dll`) — the same package trio
   Altium has carried since the Protel days for driving an external
   Specctra-compatible router (Cadence Allegro PCB Router / FreeRouting style)
   via DSN out, SES/RTE in. [confirmed + public-docs]

Conclusion: the Situs topological router is CircuitMaker's shipped and
documented interactive autorouter; the Routecct/Tds_File path is the **external
router interface** (in Altium Designer it is the user-facing "Specctra"
import/export extension feature — File > Import > Specctra Design / File >
Export > Specctra Design). Whether the DSN export command is visible in the
CircuitMaker UI is not confirmed by any CM doc (CM's UI is a curated subset);
the codec and bridge are installed regardless. [inferred]

---

## 2. Two-runtime architecture overview

The Altium-family autorouting pipeline is really two independent pipelines
sharing the same design-rule model:

```
                    ┌─────────────────────────────────────────────┐
                    │            Pcb/Advpcb.dll (PCB editor)      │
                    │                                             │
  ┌─────────────────┴─────────────────────────────────────────────┴────────────┐
  │ PIPELINE A — in-process Situs topological autorouter (user-facing)        │
  │                                                                           │
  │  Board model ──► Topological map builder (triangulation)                  │
  │       │              │                                                    │
  │       │              ▼                                                    │
  │       │      Strategy engine (route.rul / Situs.INI / strategy passes)    │
  │       │              │                                                    │
  │       │              ▼                                                    │
  │       │      Pass engines: Memory, Main, Fanout, Hug, Straighten, …       │
  │       │              │                                                    │
  │       │              ▼                                                    │
  │       │      Post-route cleanup (Gloss / RouterEntry.dll)                 │
  │       └──────────────┼─────────────────────────────────────────────────── │
  │                      ▼                                                    │
  │              Board model with tracks/vias written back                    │
  └────────────────────────────────────────────────────────────────────────────┘
  │ PIPELINE B — external Specctra-compatible router (Tds_File + Routecct)    │
  │                                                                           │
  │  Board model ──► tds_file.dll DSN writer ──► (resolution/unit/…)          │
  │       ▲                                        │ board.dsn                │
  │       │                                        ▼                          │
  │       │                              external router                      │
  │       │                              (Specctra/Electra/FreeRouting)       │
  │       │                                        │ board.ses / board.rte     │
  │       │                                        ▼                          │
  │       └── tds_file.dll SES/RTE reader ── net-matched merge into Board      │
  └────────────────────────────────────────────────────────────────────────────┘
```

Both pipelines consume the same rules: net classes, clearance matrix,
routing width/via style, differential pairs, layer directions, keepouts and
planes. Pipeline A is topological; Pipeline B is whatever the external router
implements (classically CCT's shape-based gridless router with rip-up/retry
and push-and-shove, [public-docs]).

---

## 3. Module decomposition

### M1. DSN codec — parser/writer (`Tds_File`: `tds_file.dll`, `tds_dss.dll`)

**Purpose.** Bi-directional translation between the Altium board model and
the Cadence Specctra **Design Language** (DSN). "TDS" = The Design System —
the historical CCT (Cooper & Chyan Technology) product family name; DSN is the
ASCII S-expression design interchange format standardized by CCT's Specctra
router (1989) and inherited by Cadence Allegro PCB Router, Electra, TopoR,
FreeRouting, DeepPCB. [public-docs] The two DLLs partition the work: the file
codec (`tds_file.dll`) and the design-system session support (`tds_dss.dll`)
[confirmed file split; inferred internal responsibilities].

**Inputs.** Board document (`.PcbDoc`/`.CMPcbDoc` CFB) or an in-memory board;
for import: a `.dsn` text file.

**Outputs.** For export: `(pcb ...)` S-expression text. For import: a board
model (layers, boundary, components/placement, padstacks, nets, rules,
pre-routes/planes/keepouts, optional `wiring`).

**DSN grammar subset Altium emits** (from public DSN examples, KiCad's
documented importer, LayoutEditor notes, tscircuit/specctra-dsn-json;
[public-docs], dialect specifics [inferred]):

```
(pcb "<design name>"
  (parser (string_quote ") (space_in_quoted_tokens on))   ; quoting rules
  (resolution <unit> <digits>)      ; e.g. (resolution mil 10) or (resolution mm 2)
  (unit <unit> [<digits>])          ; older/alternate unit form
  (structure
    (layer "<name>" (type signal|power|mixed|jumper) (direction horizontal|vertical|45|…))
    (boundary (rect <layer> x1 y1 x2 y2) | (polygon <layer> x1 y1 x2 y2 …))
    (via "<via_name>" <diameter> <hole>)          ; via style table
    (rule (width <w>) (clearance <c>) (via "<via_name>") (class "<cls>"))
    (netclass "<cls>" (nets "<n1>" …))
    (plane <layer> …) (copper_pour …) (keepout …) (wiring (wire (path …) …))
    (microvia …) (via_rule …) (tstrike …) (virtual_pin …))
  (placement
    (component "<ref>"
      (place "<footprint_name>" x y front|back <degrees>)) …)
  (library
    (image "<footprint_name>"
      (padstack …) (pin "<n>" x y) (rotate …structure…)) …)
  (network
    (net "<net_name>" (pins "<ref>-<pad>" …) (class "<cls>") (wiring …)) …)
)
```

Structure semantics: `(structure …)` = layer stack + boundary + via padstack
table + global/per-class rules + copper/keepout/plane shapes; `(placement …)`
= component instances with part name, X/Y, side, rotation; `(library …)` =
footprint images with padstacks and pin geometry; `(network …)` = nets with
pin turret lists (`REF-PAD` tokens, split on `-`) and optional pre-routes
(`wiring`). [public-docs]

**The `(resolution …)` descriptor is mandatory to honor.** The
`<resolution_descriptor>` defines the unit AND the resolution (decimal places)
of the file (`(resolution mil 10)` => coordinates are mils at 10 decimal
places; `(resolution um 10)` => 0.1 um grid). Exporters such as KiCad write
`(resolution um 10)`; Altium historically writes mil-based resolutions.
A clean-room importer must scale every numeric literal by
`unit_size * 10^-digits` into its internal coordinate system; the current
copperline importer only reads `(unit …)` and ignores `(resolution …)` —
this is the top interop gap (see formats.md §4, §8). [public-docs + confirmed
gap]

**Exact dialect differences vs canonical Specctra that copperline must
tolerate** ([public-docs] for canonical grammar; Altium specifics [inferred]
from the Altium-exported DSNs other tools report and from Altium's own rule-
mapping documentation):

1. **Quoted layer/footprint/net names.** Altium quotes names: `(layer "Top
   Layer" (type signal))`. Canonical Specctra uses unquoted upper-case
   reserved names (`(layer TOP (type signal))`). A tolerant parser must treat
   `"Top Layer"`, `TopLayer`, `TOP` as the same layer.
2. **Dual unit descriptors.** Altium files commonly carry both
   `(resolution …)` and `(unit …)`; `(unit)` alone appears in some exporters.
   If both exist the `(resolution)` form wins; if coordinates exceed the
   resolution decade the implied scale is still unit-based.
3. **Boundary layer id.** `(boundary (rect pcb x1 y1 x2 y2))` — the reserved
   layer name `pcb` means "the PCB boundary" (the bounding box for storing
   shapes); `signal`/`power` are reserved layer wildcards. Non-rectangular
   outlines appear as `(polygon pcb x1 y1 …)` — copperline must accept polygon
   boundaries (currently rect-only).
4. **Float literals with trailing dots.** Altium grids emit `10.` / `0.` style
   floats; the parser must accept integers, floats, and floats without
   fractional digits.
5. **Rotation on `(place …)` is in degrees; mirrored parts are `back`.** The
   footprint-pin mapping must account for 180 deg rotation + X mirror when the
   part is on the back.
6. **Padstack contents.** `(padstack "<name>" <dia> <hole>)` appears both
   standalone (via table) and inside `(image …)` (component pads); shapes
   (`(shapes (rect …) (circle …) (polygon …) (octagon …))`) are the full form
   Altium exports for non-circular pads. copperline currently treats pads as
   fixed-size squares — must add shape parsing for DRC-correct results.
7. **Net class scoping.** Altium emits `(netclass …)` inside `(structure …)`
   and/or `(class "<cls>")` inside `(network …)`; both must map onto
   copperline's class model. Rules can be repeated per class; a class
   inherits the file-default rule otherwise.
8. **Unknown/extra scopes must be ignored, not fatal.** Canonical Specctra has
   a very large grammar (design attributes, clusters, super-nets, virtual
   pins, supply pins, tstrikes, micro-via rules, groups/from-tos). Altium
   exports a modest subset but a clean-room importer should skip-and-warn on
   any unrecognized scope (copperline already does this for `wiring`
   pre-routes, microvia/blind/buried).
9. **Parser directive block.** `(parser (string_quote ") (space_in_quoted_tokens
   on))` tells the reader that `"` is the quoting character and quoted tokens
   may contain spaces — names it would otherwise split on. A clean-room
   implementation should thread this through its tokenizer (copperline's
   sexpr parser must at minimum not fatal on it).

**Algorithm (own words, clean-room).** Read the file as a token stream
(parens, atoms, quoted strings); build an S-expression tree; walk the tree to
instantiate: layer stack, boundary polygon, via padstack table, rule table
(default + per-class), net class table, component placements, footprint
images (padstacks + pins), and nets (pin turrets resolved to terminals via
placement+library). Writer: serialize the inverse, emitting `(resolution …)`
first, then structure/placement/library/network, quantizing coordinates to
the declared resolution. No design decisions are made in this module — it is
a pure adapter (copperline's `DsnImporter`/`board_to_ses` already follow this
architecture).

**Mapping to copperline.** `DsnImporter::import_text` (src/dsn.cpp) → `Board`,
`BoardDefaults`, `DsnClassRule`, `Terminal`/`Net`, `PlaneZone`, `Keepout`,
`Via`. Gaps: `(resolution …)` scaling, padstack `(shape …)` parsing,
polygon boundaries, quoted-name normalization, microvia/`via_rule` (record
and warn).

### M2. SES/RTE codec — session file parser/writer (`Tds_File`)

**Purpose.** Read back the external router's output. A **S**pecctra **E**lectra
**S**ession file (`.ses`) is the router's incremental result over the base DSN;
the **route file** (`.rte`) is the older/alternative result form. Altium
imports DSN and RTE (".rte = routed result"), exports DSN; FreeRouting and the
other Specctra-family tools write SES. [public-docs] copperline treats `.rte`
as an alias for `.ses`. [inferred, harmless]

**Inputs.** `.ses`/`.rte` text; the board to merge into (net-matched).

**Outputs.** Track/via objects added to the matching nets of the board;
optional updated placement.

**Grammar families** ([public-docs]; FreeRouting dialect per its SES export and
kicad-tools SES-import reports):

```
; Classic router-output SES (Cadence/Electra-family):
(session "<name>" (parser …) (base_design "<dsn name>")
  (unit <unit> [<digits>])
  (route
    (library (image "<footprint>" (padstack …) …))   ; padstacks the router used
    (network (net "<net>" (wires (wire (path <layer> x1 y1 x2 y2 <w>) …) )
                        (vias (via "<padstack>" x y) …)) …)))

; FreeRouting-style SES (same purpose, different nesting):
(session "<name>" (parser (string_quote ") (space_in_quoted_tokens on))
  (base_design "<dsn name>")
  (resolution um 10)
  (structure (boundary (rect …)) …)        ; optional echo of design
  (library_out (image … (padstack …) …))   ; via/pad stack definitions
  (network_out (net "<net>" (wire (path <layer> x1 y1 x2 y2 <w>) (via "<ps>" x y) …)) …))
```

Either way the payload is per-net wires (paths: layer + endpoints + width,
optionally multi-segment) and vias (padstack name + center), plus a padstack
table defining the vias. Nets not mentioned carry no new routing. Session
files may also carry placement data (the CCT guide notes the session file
"contains routing and placement data" [public-docs]).

**Net-matched merge semantics (algorithm, own words).** For each net group in
the session: look up the board net by name; create the missing net if it is
new; for each `wire`: append track segments (or re-route the net's path) on
the given layer with the given width, honoring the file's unit/resolution;
for each `via`: resolve the padstack name from the session library (falling
back to the DSN via table by name), then place a via on the matched net.
Scale all coordinates by `unit_size * 10^-digits` exactly as in M1. Return a
summary (nets touched, wires, vias, unmatched padstack names). copperline's
`ses_merge_into` (src/dsn.cpp) already implements this shape — the gap is
dialect nesting (`network_out` vs `route/network`) and padstack-name
resolution.

**Mapping to copperline.** `board_to_ses` (writer), `SesImporter` /
`ses_merge_into` (reader) → `Board` tracks/vias keyed by net. SES validation
should use FreeRouting (`-de board.dsn`, `-do board.ses`) as an independent
oracle (see §10).

---

### M3. DO/DID command layer (Routecct)

**Purpose.** The **DO-file** (`*.Do`) is a Specctra *command script*: the
classic CCT flow drives the router by commands such as `read dsn`, `route`,
`write ses` — strategies, rule overrides and file I/O are scripted. A
**DID-file** is the execution transcript/log, which after editing can be
reused as a new DO-file. [public-docs] `RouteCct.Do` (1,020 B, [confirmed]) is
the canned command file the Routecct integration feeds to an external
Specctra-compatible router: it is the "route this DSN, write that SES" script.
Do-files historically also specify routing strategies (pass selection,
weightings) for CCT-style routers.

**Inputs.** The DO-file text; the DSN path; output path(s).

**Outputs.** Router commands / a router process invocation; the SES/RTE.

**Key data structures (observed/format-level).** Plain ASCII S-expression-free
command list — one command per line-ish: `read dsn <file>`, `write ses <file>`,
`write routes <file>`, strategy/pass directives. [public-docs; contents of the
1 KB RouteCct.Do not extracted — package not downloaded — so pass/strategy
directives inside it are [inferred]]

**Mapping to copperline.** A `RoutecctRunner` that emits a DO-file and invokes
an external router (or FreeRouting) is an optional orchestrator; copperline
itself needs no DO parsing if it targets SES directly.

---

### M4. External router bridge (`RouteSupport.dll`, `.INS`, `.OUT`)

**Purpose.** Altium's integration-server glue around a Specctra-compatible
router. `.INS` = integration-server info (how DXP/EDP loads the plugin),
`.OUT` = output-generator definition (which file types it can produce); the
DLL provides the route-command plumbing (prepare board, run router, read
result). This is the modern descendant of Protel's "route via Specctra"
feature. [inferred from Altium's .INS/.OUT extension conventions, which the
other CAM packages in the same manifest follow identically: `Gerber.OUT`,
`Drill.OUT`, `Odb.OUT`, `PADS_ASCII.out` — [confirmed pattern]]

**Inputs.** A prepared board + DO-file + rule set.

**Outputs.** A routed board (SES/RTE merged back) or a report.

**Algorithm (own words).** It is an adapter, not an algorithm: convert the
board to DSN (M1), write the DO-file (M3), launch/wait on the external router,
read the SES (M2), apply net-matched merge, refresh DRC. Altium Designer
documents the same conceptual flow as a user feature ("Ensuring PCB Readiness
for Specctra-compatible Routers" — export DSN, route externally, import RTE).

**Mapping to copperline.** copperline's `route --output *.ses` +
`verify --routes *.ses` already implements the data-exchange half of this
module; a future `--route-external <router>` wrapper completes it.

---

### M5. Situs topological engine (`RouterEntry.dll`, `IntGeometryEntry.dll`)

**Purpose.** The in-process autorouter CircuitMaker documents as "the Situs
Autorouter". `RouterEntry.dll` is the router entry/plugin; `IntGeometryEntry.dll`
is the integer-geometry kernel (exact arithmetic on integer coordinates —
the board model's nm grid maps directly). [confirmed packaging; inferred
responsibility split] The Gloss package name indicates this same engine also
owns post-route glossing (see M12).

**Core idea — topological mapping** ([public-docs], Altium resources article +
CM docs): unlike grid routers (uniform grid) or rectilinear-expansion routers
(Racal Redac 1984 "A Method for Gridless Routing of Printed Circuit Boards"),
Situs does not map coordinates. It **triangulates the free space between
adjacent obstacles** and links each obstacle to its neighbours, producing a
graph "like a set of connected spiders' webs". Routing paths are found by
stepping from web strand to web strand between obstacle pairs, start to end.
Because the mapping space and the routing space are separate, paths are
**any-angle** (not restricted to orthogonal) and shape-independent (obstacles
of any shape). This yields high completion on staggered-pin/dense/non-orthogonal
boards and fewer vias (diagonal routes found directly, no post-hoc corner
conversion).

**Algorithm (own words).** Phase 1 — build the topological map: for every
obstacle (pad, track, via, pour, keepout region) compute its clearance-expanded
shape; triangulate free space (Delaunay-style constraint triangulation between
obstacle boundaries); the adjacency graph of triangle edges IS the map. Phase 2
— path search on the map: a graph search (costed) from source to target
treating triangle crossings as steps; costs encode layer direction preference,
via cost, and obstacle proximity. Phase 3 — realize the topological path into
geometry: the chosen web-strand path is converted into actual tracks by the
shape-based push-and-shove router, which physically displaces existing routes
that collide (shove) or tears them up (rip-up/retry) when shoving fails.
Phase 4 — cleanup passes (M12). The "More/Less Vias" slider and "Orthogonal"
option are strategy-level knobs that feed the cost function. [inferred from
the documented behavior of the pass set + the topological description]

**Engines inside Situs** (named in Altium docs, [public-docs]): memory router,
pattern routers, a power-and-ground router, a wavefront router, shape-based
push-and-shove routers, and heuristic routers for BGA fanout.

**Mapping to copperline.** Conceptually maps to `engine`/`astar` (costed graph
search on a spatial index) + `spatial_index` (obstacle queries) + `push_and_shove`
behaviour (copperline's router rips up and reroutes rather than shoving —
topological *mapping* is the new capability a re-implementation would add).

---

### M6. Topological map builder (Situs internals)

**Purpose.** Construct and maintain the triangulated obstacle-adjacency map.

**Inputs.** Board obstacles (pads, tracks, vias, pours, keepouts) per layer,
each expanded by its applicable clearance; layer directions.

**Outputs.** A per-layer triangular tiling of free space with obstacle
neighbour links; the "spiders' web" used by the Main/Completion passes.

**Key data structures ([inferred]).** Constrained triangulation mesh (triangle
cells + adjacency), obstacle-to-mesh link table, per-triangle clearance/flow
costs, region-of-influence lists. Public descriptions only go as far as
"triangulating the space between adjacent obstacles" — the exact mesh form is
not documented and must remain inferred.

**Algorithm (own words).** Insert every clearance-expanded obstacle contour
into a planar constraint set; compute a constrained Delaunay triangulation;
classify triangles as free/blocked; build dual graph; update incrementally
when the router adds/rips tracks.

**Mapping to copperline.** A new `TopoMap` module over `spatial_index`; the
existing `density`/`sparse_graph` machinery is a partial analogue (grid vs
triangle). This is the largest net-new algorithmic work for a clean-room
port of Situs.

### M7. Routing passes (Situs pass library)

**Purpose.** Parameterized routing algorithms the strategy engine assembles
into a strategy. Each pass turns (a subset of) the topological map's virtual
paths into finished tracks. CircuitMaker's Strategy Editor exposes the same
pass library as Altium Designer; the full documented set ([public-docs], CM
and AD docs) is:

| Pass | Type | Behaviour (from vendor docs, own words) |
|---|---|---|
| Main | connection | Uses the topological map to find the path, then push-and-shove to realize it. The workhorse. |
| Multilayer Main | connection | Same as Main, costs tuned for multi-layer boards. |
| Globally Optimised Main | connection | First iteration routes ignoring violations; then reroutes with rising conflict costs until clean. With Orthogonal on, produces tidy patterns (add Recorner to miter). |
| Completion | connection | Like Main but re-costed (vias cheaper, wrong-way dearer) to finish stubborn connections. |
| Memory | connection | Routes two same-net pins on different components that share an X or Y coordinate (the classic "memory" U pattern). |
| Adjacent Memory | connection | U-pattern fanout for adjacent same-net pins. |
| Clean Pad Entries | connection | Reroutes out of each pad along its longest axis; always include after Memory when pads are asymmetric. |
| Fan out Signal | component | SMD fanout to signal layers per the Fanout Control rule (inline/staggered patterns). |
| Fan out to Plane | component | SMD fanout to an internal plane layer. |
| Layer Patterns | connection | Routes only connections matching a layer direction (within tolerance); costed to hug existing routing. |
| Hug | connection | Reroutes each connection hugging existing routing at minimum clearance to free space. Slow. |
| Spread | connection | Reroutes to spread/equalize spacing between fixed objects. Slow. |
| Straighten | connection | Corner reduction: walk to a corner, probe H/V/45up/45down for another point of the same net; keep if it shortens the route. |
| Recorner | connection | Miters corners (used when Orthogonal is on; off by default otherwise). |

Only one Main-family pass per strategy. Passes run in strategy order, top-down;
the same pass may appear multiple times (stage late cleanups).

**Algorithm (own words).** Each pass selects a class of connections (all,
layer-matching, memory-pattern, pad-entry) and applies its well-defined
operation: main-family passes run map-search + shove realization; pattern
passes match geometric templates; cleanup passes walk existing geometry and
apply local transforms (line-probe shortening, corner mitering, spacing
evening) subject to DRC.

**Mapping to copperline.** `Main` ≈ `engine`/`astar` with via/layer costs;
`Memory` ≈ a `memory_route` helper on the connectivity model; `Straighten`,
`Recorner`, `Spread`, `Hug` ≈ passes over `optimizer` + `simplify` output;
`Clean Pad Entries` ≈ a `pad_entry_cleanup` geometry sweep. copperline already
has `parallel` and `optimizer` infrastructure to host multi-pass pipelines.

---

### M8. Strategy engine + strategy files (Situs)

**Purpose.** The "brain" of the router. A strategy file defines the ordered
list of passes and their weightings; the strategy engine executes them and
adapts as routing density changes (empty-board passes early, tight squeeze
passes late). Altium: "the Situs strategy file is one of the most
sophisticated strategy files of any autorouter available today… the strategy
file controls the routing engines, calling them when necessary and weighting
their actions." [public-docs]

**Shipments in CircuitMaker ([confirmed]):** `System\Situs.INI` (2,251 B —
default router options + strategy list), `System\route.rul` (375 B — the
default routing strategy/rule file), `System\SitusTemplate.Bas` (695 B — a
DelphiScript/Basic script template showing how to drive the autorouter from a
script), `System\DefaultRules.rhv`. Contents of these small files are *not*
extracted (packages not downloaded) — their formats are [inferred]: an
INI-style options file, a compact rule/strategy text, and a Basic script
template respectively.

**Default strategies** (documented in AD; the same editor ships in CM —
[public-docs]): Cleanup, Default 2 Layer Board, Default 2 Layer With Edge
Connectors, Default Multi Layer Board, General Orthogonal, Via Miser. Defaults
cannot be deleted/edited; users duplicate them.

**Key data structures ([inferred]).** Strategy record = {name, description,
pass list with per-pass parameters, via slider value, orthogonal on/off};
store = initialized from `Situs.INI`/`route.rul` at first use, persisted to
the user profile on edit.

**Algorithm (own words).** An interpreter: given a strategy, iterate passes in
order; each pass consumes the current board state; after each pass the board
state updates (tracks added, violations recorded). Via slider scales the cost
of adding a via during Main-family search (more vias = faster/looser; fewer =
slower/tighter).

**Mapping to copperline.** A `Strategy` struct + `run_strategy(board, passes)`
over the pass library; `router.md`/docs format for user-defined strategies.

---

### M9. Net class & clearance model

**Purpose.** The rule system both pipelines obey. Altium's docs explicitly
document the translation to Specctra's fixed scoping hierarchy — this is the
authoritative map for a clean-room exporter ([public-docs], "Ensuring PCB
Readiness for Specctra-compatible Routers"):

| Specctra scope | Altium Designer expression | Priority |
|---|---|---|
| PCB design | All | 12 (lowest) |
| Layer | OnLayer('L') | 11 |
| Net class | InNetClass('C') | 10 |
| Net class on layer | InNetClass('C') And OnLayer('L') | 9 |
| Group set | (not supported in AD) | - |
| Net | InNet('N') | 8 |
| Net on layer | InNet('N') And OnLayer('L') | 7 |
| Group | emulated via From-To class: InFromToClass('…') | 6 |
| Group on layer | + OnLayer | 5 |
| FromTo | InFromTo('Net (From:To)') | 4 |
| FromTo on layer | + OnLayer | 3 |
| Class vs Class | (InNetClass vs InNetClass — not supported by exporter) | 2 |
| Region | WithinRoom('R') within WithinRoom('R') | 1 (highest) |

Specctra applies rules by this fixed order; AD applies by user priority — the
exporter must flatten AD rules into Specctra scopes to get equivalent results.
Clearance rules can use primitive-scope modifiers: IsPad, IsThruPin,
IsSMDPad, IsVia, IsTrack, IsFill, IsPolyRegion, IsTestPoint (e.g. `InNet('N1')
AND IsVia vs IsPad` for pad-to-via clearance).

**Rule types feeding the router** (from formats.md Rules6 kinds, [public-docs +
confirmed]): Clearance, Width (Min/Preferred/Max — router uses the Preferred
value [public-docs]), Routing Vias (diameter + hole min/pref/max), Routing
Layers (allowed layers), Routing Priority, SMD Fanout Control, Differential
Pairs Routing, Routing Corners — the router obeys all Electrical and Routing
rules *except* Routing Corners [public-docs]. Via types (blind/buried/micro)
come from the Layer Stack drill-pair definitions and are honored when the
router drops a via [public-docs].

**Mapping to copperline.** `BoardDefaults` + `DsnClassRule` + `rules`
(rules.h) — extend `DsnClassRule` with layer-scoped variants, via-type sets,
min/pref/max triplets, and clearance-class pairs (`via_at_smd` etc.). The
"preferred value" convention should match: width rule = preferred, else min;
via = preferred style, else default.

---

### M10. Obstacle / shape model

**Purpose.** Geometric substrate for mapping + DRC.

**Components ([public-docs] + [confirmed] via formats.md record layouts):**
- **Pads as shapes.** Pads6 records carry per-layer X/Y sizes and shape
  codes (1=circle, 2=rect, 3=octagon, 9=rounded-rect), hole size/shape
  (round/slot), layer span (simple / top-mid-bottom / full stack), paste &
  solder mask expansions. Situs expands pads by their clearance and treats
  the expanded silhouette as the obstacle (undocumented but mathematically
  forced; [inferred]).
- **Tracks/vias** as swept shapes (track = segment with width; via = padstack
  with hole + annular ring + per-layer shapes in the 32-entry per-layer arrays).
- **Thermal relief / polygon connect.** Rules6 `PolygonConnect` record:
  CONNECTSTYLE (relief/solid/none), AIRGAPWIDTH, RELIEFCONDUCTORWIDTH,
  RELIEFENTRIES (spoke count) — the router must honor these when a via/pad
  touches a plane; Situs routes to planes through relief spokes ([inferred]
  from AD plane-connect behavior documented in the pad/via/plane docs).
- **Planes & pours as obstacles.** `(plane …)`/`(copper_pour …)` in DSN;
  Polygons6 in the CFB. Solid/hatched pours are obstacle regions; hatched
  pours are decomposed to track/arc sets ([public-docs]). Situs can route a
  board containing pours but Altium recommends deferring pours until after
  routing ([public-docs]).
- **Keepouts.** `(keepout …)`, `(wire_keepout …)`, `(via_keepout …)`,
  `(place_keepout …)`, `(bend_keepout …)`, `(elongate_keepout …)` (DSN) /
  keepout regions (board) — hard obstacles; Situs requires a closed boundary
  of keepout objects around the board edge [public-docs].

**Mapping to copperline.** `Keepout`, `PlaneZone`, `Via`, `Terminal`, obstacle
construction in `board.h` + `spatial_index`; add shape-aware pad expansion
(rounded-rect → octagon → polygon) and thermal-relief spoke modeling for the
plane-connected-object rule.

---

### M11. Fanout / escape engine (Situs)

**Purpose.** SMD/BGA escape routing: route each pad out past the device edge
for later connection. CircuitMaker exposes the full fanout toolset (All / Power
Plane Nets / Signal Nets / Net / Connection / Component / Selected Components /
Pad). [public-docs]

**Behavior** ([public-docs]): typical fanout = dog-bone (short track + via at
the end), then escape route from the via out past the edge, working through
routing layers until all pads are escaped. Options: fanout pads without nets,
fanout outer 2 rows, include escape routes, blind-via fanout when drill-pairs
exist (BGA escape only), differential-pair-first escape on the same layer/side.
Arrangement selection (inline/staggered) honors the Fanout Control rule
(pattern check + clearance + width + via style) [public-docs].

**Mapping to copperline.** A `fanout` module (component-level) atop the
existing `escape` header — copperline already has `escape.cpp`/`hierarchy`
for BGA-style escape problems; extend with fanout-pattern selection and
blind-via drill-pairs.

---

### M12. Post-route cleanup & optimization ("Gloss")

**Purpose.** Polish pass set that runs after (or between) main passes to raise
route quality. Packaged in CM as `Base_PCB - Gloss` (RouterEntry.dll —
cleanup runs inside the router engine; [confirmed packaging, inferred split]).

**Operations** (passes above + documented behavior, [public-docs]):
- **Straighten** — corner reduction via H/V/45 probes (kept only if length
  drops).
- **Recorner** — mitering of 90-degree corners when Orthogonal strategies.
- **Spread** — even spacing between fixed objects.
- **Hug** — pack routes to boundaries to open channels.
- Clean Pad Entries — re-exit pads along their long axis.
- Rip-up Violations After Routing — post-session DRC-driven rip-up of any
  remaining violations [public-docs].
- Via minimization — the "Via Miser" default strategy + the More/Less Vias
  slider trade speed against via count [public-docs + inferred].

Situs mitered corners and any-angle paths by default, so less post-hoc
corner work than grid routers [public-docs].

**Mapping to copperline.** `optimizer` (length/corner reduction — the
Straighten probe family), `simplify` (arc/corner smoothing ≈ Recorner),
`tuning`/`via_bundle` (via count reduction ≈ Via Miser), `verifier` (rip-up
gate). copperline's existing optimizer+simplify+verifier trio already covers
most of M12's operations.

### M13. Autorouting orchestration (how the PCB editor drives the router)

**Purpose.** Wire the router into the editor: menus, dialogs, rule handoff,
progress/abort, result write-back.

**Situs path (in-process).** The PCB editor (Advpcb.dll core + pcbdlgs.dll
dialogs) exposes Tools | AutoRoute with Setup / All / Net / Net Class /
Connection / Area / Component / Component Class / Connections On Selected
Components / Connections Between Selected Components; fanout sub-menus; and
Stop / Reset / Pause during a run. [public-docs, CM + CS docs] The orchestrator
(void of any batch-mode file protocol in CM — the router operates on the live
board) does: collect enabled rules + layer directions → run the pre-routing
analysis → show the Routing Setup Report (rules and object counts affected,
layer directions, drill layer pairs, warnings incl. layers set to Any) →
execute the chosen strategy in-process → rip-up violations if requested →
write tracks/vias into the board document. [public-docs] DRC tie-in: routing
setup report lists rule violations to fix before routing; pre-existing
violations slow the router and can block routes [public-docs]. The
`SitusTemplate.Bas` script template ([confirmed] file) suggests scripted
drive (execute a strategy from a Basic script) is supported.

**External path (Routecct).** The Routecct integration (RouteSupport.dll +
RouteCct.Do) drives Pipeline B: prepare DSN via Tds_File → run DO commands →
read SES/RTE via Tds_File → merge. In Altium Designer this is exposed as the
Specctra extension feature (File > Import > Specctra Design (.dsn, .rte);
File > Export > Specctra Design (.dsn), also as an OutJob export "Specctra
Design PCB") with a "Setup Specctra Router" dialog [public-docs]. In
CircuitMaker the same binaries ship but no UI path is documented [inferred —
verify on a running install].

**Batch/GUI-less mode.** No documented batch CLI for either engine in CM
(Situs is always interactive; the DO path needs an external router process).
copperline's own `route --output *.ses` + external FreeRouting invocation is
an independent, legitimate re-implementation of Pipeline B's orchestration.

**Mapping to copperline.** `main.cpp` subcommands (`route`, `verify`,
`analyze`, `optimize`) are the orchestrator; add `setup-report` (pre-route
analysis), `fanout`, and `gloss-all` surfaces.

---

### M14. Pad/via default tables (`padinit.dat`, DefaultPadsPinDecals.c,
DefaultRules.rhv)

**Purpose.** Default-value data for pad construction, via styles and rules
when a design under-specifies them ([confirmed] shipped in Base_PCB; exact
formats [inferred]):
- `padinit.dat` (455 KB) — a large pad-parameter database (default pad/via
  size/shape tables, historical Protel-era artifact also consulted by the
  router's rule interpolation).
- `DefaultPadsPinDecals.c` — a C-like text of default pad pin decals
  (footprint primitives used when a library part has no pads).
- `DefaultRules.rhv` — default design-rule values (width/clearance/via)
  seeded into new documents.

**Mapping to copperline.** `BoardDefaults` initialization fallbacks; copperline
already has sane defaults — these files only matter for bit-exact round-trips.

---

## 4. What CircuitMaker specifically exposes (bridge to user)

| Capability | CircuitMaker 2.2.1 | Altium Designer (comparison) |
|---|---|---|
| Autoroute menu (Setup/All/Net/Net Class/Connection/Area/Component/…) | Yes — Situs, documented [public-docs] | Yes — Situs [public-docs] |
| Situs Routing Strategies dialog + Routing Setup Report | Yes [public-docs] | Yes [public-docs] |
| Situs Strategy Editor (passes, Via slider, Orthogonal) | Yes [public-docs] | Yes [public-docs] |
| Fanout menus + Fanout Options (BGA escape, blind-via) | Yes [public-docs] | Yes [public-docs] |
| DRC tie-in (pre-route report, rip-up violations after) | Yes [public-docs] | Yes [public-docs] |
| Specctra DSN export / SES-RTE import (external router flow) | Binaries installed [confirmed]; UI exposure not documented [inferred — likely hidden in CM's curated UI] | Yes — Specctra extension feature: File > Import/Export Specctra Design, OutJob export, Setup Specctra Router dialog [public-docs] |
| Routecct external-router invocation | Installed [confirmed]; not documented in CM UI [inferred] | Legacy; superseded by Situs but the Specctra feature remains [inferred] |

**Bottom line for copperline users:** navigating real CircuitMaker community
boards can assume Situs was the router, i.e. the board on disk contains
finished tracks/vias as ordinary primitives — there is **no autoroute
artifact** to parse (Situs writes back into the .CMPcbDoc directly; no DSN/SES
left behind by default). The DSN/SES formats matter for copperline as the
*exchange* path with external routers (KiCad/FreeRouting style), which is
exactly what copperline already implements.

---

## 5. copperline integration mapping (summary table)

| Altium/CM module | copperline today | Gap / action |
|---|---|---|
| M1 DSN codec (Tds_File) | `DsnImporter::import_text` (src/dsn.cpp) | (resolution …) scaling; padstack (shape …); polygon boundary; quoted-name normalization; microvia/via_rule warn |
| M2 SES codec | `board_to_ses`, `ses_merge_into` | Freerouting `network_out` nesting; padstack-name resolution from library_out; unit/resolution on session |
| M3 DO layer | (none) | optional `RoutecctRunner` (write .Do, launch router) |
| M4 External bridge | `route --output *.ses` + `verify --routes *.ses` | optional `--route-external` wrapper |
| M5 Situs engine | `engine` + `astar` + `spatial_index` | topological mapping (new `TopoMap` module) is the big add |
| M6 Topo map builder | `sparse_graph`/`density` (grid-ish) | constrained triangulation + obstacle web |
| M7 Passes | optimizer/simplify/parallel infra | add Memory, Hug, Spread, Fanout, Straighten pass implementations |
| M8 Strategy engine | (none) | `Strategy` struct + `run_strategy`; inherit defaults (Via Miser, 2-Layer, Multi-Layer, Orthogonal) |
| M9 Rules model | `BoardDefaults`/`DsnClassRule`/rules.h | layer-scoped rules, min/pref/max, via types, clearance modifiers, diff-pair params |
| M10 Obstacles | `Keepout`/`PlaneZone`/`Via`/`Terminal` + spatial_index | shape-aware pad expansion, thermal-relief spokes |
| M11 Fanout | `escape` | fanout patterns + blind-via drill-pairs |
| M12 Gloss | `optimizer`/`simplify`/`tuning`/`verifier` | rip-up-after + pad-entry cleanup sweep |
| M13 Orchestration | main.cpp subcommands | setup-report + fanout + gloss-all surfaces |
| M14 Defaults | `BoardDefaults` | none (only bit-exact round-trip care) |

---

## 6. Clean-room implementation checklist (priority order)

1. **Resolution fix (M1)** — parse `(resolution <u> <d>)` (and `(unit <u>
   <d>)`), scale all literals; unit tests vs KiCad-exported DSNs. Cheap,
   unlocks real CM/AD/FreeRouting files.
2. **Padstack shapes (M1)** — parse `(shapes (circle|rect|octagon|polygon …))`
   on padstacks; drive copperline pad expansion from them.
3. **SES dialect tolerance (M2)** — accept `network_out`/`library_out`
   nesting and `(route (network …))`; resolve padstack names; validate against
   FreeRouting output as the independent oracle.
4. **Quoted-name normalization + polygon boundary (M1)** — treat
   `"Top Layer"` == top; accept `(boundary (polygon …))` and non-rect boards.
5. **Strategy engine + pass library (M7/M8)** — model Situs strategies
   (passes in order, Via slider, Orthogonal) on top of copperline's engine;
   start with Main, Memory, Straighten, Recorner, Clean Pad Entries.
6. **Topological mapping (M5/M6)** — experimental `TopoMap` (constrained
   triangulation over spatial_index) feeding the existing astar; measure vs
   current grid approach on dense BGA fixtures.
7. **M9–M12 extensions** as needed by fixtures (rules kind Parity with
   formats.md Rules6 kinds).

Nothing here requires Altium code: every layout above is from public
documentation (Specctra Design Language Reference — public PDF; Cadence/Allegro
router behavior described in Cadence and third-party docs; Altium's own
published rule-scope mapping; KiCad's documented Specctra support; FreeRouting
docs/issue-tracker reports of its SES dialect) plus installer-manifest byte
facts.

---

## 7. Sources

**Manifest / observed artifacts ([confirmed])**
- CircuitMaker 2.2.1.6 `Installation.Release` (extracted overlay,
  `%TEMP%\opencode\re-cm\re-cm-payload\`) — package `<files>` lists for
  `Base_PCBUtils - Tds_File`, `Base_PCBUtils - Routecct`, `Base_PCB`,
  `Base_PCB - Gloss`, `Base_PCB - PcbDlgs`, `Base_SchUtils - SchRouter`.
- docs/reverse/circuitmaker/README.md (installer RE report), formats.md
  (CFB/text/binary record layouts, rules kinds), packages_of_interest.tsv.

**Public documentation ([public-docs])**
1. Altium CircuitMaker docs — "Autoroute" (Situs topological; pass list;
   Strategy/Strategies dialogs; Fanout): altium.com/documentation/altium-circuitmaker/automated-board-layout-situs-topological-autorouter
2. Altium resources — "Automated PCB Routing With the Situs Topological
   Autorouter" (topological vs grid vs rectilinear-expansion; spiders'-web
   map; engine list; strategy-file brain; default strategies; rule guidance):
   resources.altium.com/p/automated-pcb-routing-with-situs-topological-autorouter
3. Altium Designer docs — "Ensuring PCB Readiness for Specctra-compatible
   Routers" (Specctra scope hierarchy table, rule translation, import/export
   commands, .dsn/.rte): altium.com/documentation/altium-designer/design-tools-interfacing/specctra-router
4. Altium Designer docs — "Interfacing to Other Design Tools" (Specctra
   import .dsn/.rte, export .dsn; extension-based features).
5. Altium CircuitStudio docs — "Autorouting Tools" (Situs pass/routing setup).
6. Cadence — "SPECCTRA Design Language Reference", Product Version 10.0
   (public PDF via hackaday.io / specctra.pdf): `(resolution …)` semantics,
   boundary `(rect pcb …)`, reserved layer names pcb/signal/power, route/
   session/route-file descriptors, virtual pin, supply pin, tstrike.
7. Wikipedia — "Specctra" (CCT 1989, Cooper & Chyan; Cadence 1997 → Allegro
   PCB Router; DSN design-file + Do-files + SES/RTE + Did-files; rip-up-and-
   retry + push-and-shove; Electra/TopoR/FreeRouting/DeepPCB compat).
8. LayoutEditor docs — "DSN" (DSN in / SES out exchange; S-expression
   grammar family; layout editor writer subset).
9. FreeRouting — freerouting.org (Specctra DSN/SES compliance; using-with-
   kicad flow; CLI `-de board.dsn[:ses] -do board.ses|dsn|scr` per its
   command-line docs).
10. KiCad dev-docs — Altium import format reference; KiCad DSN namespace
    reference; KiCad docs on Specctra Session import.
11. KiCad ecosystem — rjwalters/kicad-tools issue #6593 (FreeRouting/fastroute
    SES dialect: `network_out`, `(resolution um 10)`, `(parser …)`,
    `library_out` padstack); GoForFab/pcb-rnd Freerouting write-ups (DSN→SES
    round-trip used in the scripting tests).
12. pcbsync.com — DSN/SES/RTE/DO/DID pair + role descriptions.
13. tscircuit/specctra-dsn-json — `(parser (string_quote ") (space_in_quoted_tokens
    on))`, `(resolution um 10)` samples.

**Clean-room note.** No vendor source code was consulted or reproduced. DO/DID
and Situs.INI/route.rul/padinit.dat *contents* were not extracted (packages
were not downloaded); their described roles are inferred from naming + the
documented behaviors above, and are marked [inferred] in the text.

---

*End of module decomposition. Companion docs: README.md (installer report),
formats.md (board format notes).*