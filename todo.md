\# GLOBAL INSTRUCTIONS FOR ALL FIVE RUNS



You are implementing a production-quality C++20 PCB autorouter intended primarily for autonomous coding/EDA agents.



The authoritative architecture is:



`docs/superpowers/specs/2026-09-13-parallel-pcb-router-design.md`



Read it completely at the beginning of every run.



Continue from the existing repository state. Inspect previous commits and tests instead of recreating working components.



\## Product identity



This is a \*\*CLI-only, headless router\*\*.



Do NOT build:



\- Qt UI

\- web UI

\- interactive PCB viewer

\- desktop application shell

\- graphical editor

\- GUI-only diagnostics



The primary consumers are AI agents, scripts, CI systems and EDA pipelines.



Human-readable CLI output is useful, but every important operation must have a stable machine-readable representation.



Provide:



`router route ...`



`router verify ...`



`router analyze ...`



`router escape ...`



`router benchmark ...`



`router explain-failure ...`



`router capabilities`



Where appropriate support:



`--json`



`--quiet`



`--seed`



`--threads`



`--timeout`



`--max-search-nodes`



`--output`



`--report`



`--config`



All important results must have stable JSON schemas.



stderr is for diagnostics/progress.



stdout in `--json` mode must contain only machine-readable output.



Use documented nonzero process exit codes for:



\- invalid input

\- malformed design rules

\- routing incomplete

\- hard-rule violation

\- internal failure

\- resource/search-budget exhaustion



An agent must be able to determine exactly what happened without parsing prose.



\## Electrical-awareness requirements



Routing legality and priority MUST understand:



\### Current / amperage



Each net may specify:



\- expected continuous current

\- optional peak current

\- minimum trace width

\- preferred trace width

\- copper thickness if known

\- maximum via current / via-class preference if provided



The router must not silently invent current.



If current metadata is absent, use explicit board defaults and report that defaults were used.



Current-aware routing must affect:



\- minimum permissible trace width

\- preferred trace width

\- neckdown legality

\- via choice

\- number of parallel vias where configured

\- route difficulty

\- congestion/resource consumption



The width model must be replaceable.



Implement a `CurrentCapacityModel` abstraction rather than burying one formula inside A\*.



Support at least:



1\. direct explicit width rules

2\. configurable IPC-derived width estimation

3\. user-defined width/current classes



Safety rule:



Explicit design rules always override inferred estimates.



If an inferred current requirement conflicts with available geometry, report the conflict instead of silently narrowing the trace.



\### Voltage



Each net may specify nominal voltage and/or a voltage class.



Clearance resolution must consider \*\*voltage difference between interacting nets\*\*, not merely each individual net's voltage.



Implement a `VoltageClearanceModel`.



Clearance may be resolved from:



\- explicit net-pair rule

\- voltage-class pair

\- voltage-difference table

\- board default



Example concept:



`required\_clearance(net\_A, net\_B)`



must resolve using both nets.



Do not hardcode assumptions about mains, creepage or regulatory safety into the router.



The design/configuration supplies those rules.



The router enforces them exactly.



Voltage-aware clearance must affect:



\- obstacle expansion

\- A\* edge legality

\- via legality

\- corridor capacity

\- congestion estimation

\- verification



\### Pin density



Pin density is a first-class routing signal.



Compute at least:



\- footprint-local pin density

\- endpoint-local density

\- estimated routing-channel density

\- escape-region density



Use density in:



\- fine-pitch detection

\- route-task difficulty

\- tie-breaking

\- congestion estimates

\- probable corridor selection

\- worker scheduling



For BGA/LGA/fine-pitch footprints, however, simple density must NOT replace the centre-out invariant.



Fine-pitch ordering remains structurally:



deepest/most central pads first → perimeter last.



Density augments this ordering; it does not override it.



\## Optimization hierarchy



Global acceptance remains lexicographic:



1\. unconnected required terminals

2\. hard rule violations

3\. unresolved resource overuse

4\. electrically undesirable geometry

5\. via count

6\. total trace length

7\. bend cost

8\. soft congestion



An ugly legal fully-connected board beats a beautiful board with one missing connection.



\## GitHub



At the beginning of each run:



\- inspect `git status`

\- inspect branches

\- inspect recent commits

\- inspect remotes

\- preserve unrelated valid work



Commit frequently.



Never force-push.



Never rewrite published history.



Never delete unrelated branches.



If a writable GitHub remote exists, push tested work.



If no remote exists but authenticated GitHub CLI/API access is available, create an appropriate repository under the authenticated user's account and configure `origin`.



If GitHub authentication is unavailable, complete and commit the work locally and report exactly what prevented the push.



Never fabricate credentials, repository URLs or successful pushes.



\---



\# PROMPT 1 — Core geometry, electrical rules, CLI contract and single-thread router



Implement the foundational router.



Do not move on to BGA meta-search or parallel routing yet.



\## Deliverables



Build a clean C++20 project with CMake.



Implement focused modules for:



\- integer-nanometre geometry

\- board model

\- layer model

\- net model

\- terminals/pads

\- trace and via geometry

\- keepouts and obstacles

\- RouteTree

\- spatial indexing

\- design-rule resolution

\- current-awareness

\- voltage-awareness

\- pin-density analysis

\- BoardVerifier

\- sparse routing graph

\- single-thread A\*

\- CLI shell

\- stable JSON result/report schemas

\- synthetic board fixtures



All geometry legality uses integer units.



Do not use a fine uniform raster as the principal router representation.



\## Rule engine



Create clean abstractions similar to:



```cpp

class RuleResolver;

class CurrentCapacityModel;

class VoltageClearanceModel;

```



The rule engine must be able to answer questions equivalent to:



```cpp

TraceRule traceRule(NetId net, LayerId layer, RegionId region);



Length requiredTraceWidth(

&#x20;   NetId net,

&#x20;   LayerId layer,

&#x20;   const ElectricalContext\& context);



Length requiredClearance(

&#x20;   NetId a,

&#x20;   NetId b,

&#x20;   LayerId layer,

&#x20;   const ElectricalContext\& context);



std::vector<ViaStyle> allowedVias(

&#x20;   NetId net,

&#x20;   LayerSpan span);

```



Exact APIs may vary, but responsibilities must remain separated.



\## Current-aware tests



Test:



\- explicit minimum trace width

\- higher-current net consuming more routing width

\- illegal neckdown rejected

\- legal explicit neckdown accepted

\- via class rejected for current requirements

\- board default applied when current absent

\- explicit board rule overriding inferred width



\## Voltage-aware tests



Test:



\- same-voltage low-clearance pair

\- large voltage-difference pair requiring larger clearance

\- explicit pair rule overriding voltage table

\- A\* avoiding a route legal geometrically but illegal electrically

\- verifier detecting insufficient voltage-dependent clearance



\## Pin-density tests



Implement a spatial density estimator.



Verify a synthetic dense MCU/BGA region scores substantially higher than open board space.



Expose density information through:



`router analyze --json`



The machine-readable analysis should include something like:



\- nets

\- terminal count

\- local densities

\- detected dense footprints

\- current classes

\- voltage classes

\- likely bottlenecks



\## CLI



At minimum implement working versions of:



`router capabilities --json`



`router analyze board --json`



`router verify board --json`



`router route board --json`



Even if routing functionality is incomplete in this phase, schemas and command behavior must already be stable.



\## Sparse A\*



Search state includes:



\- position

\- layer

\- incoming direction/bend state



Cost structure must permit:



\- length

\- bends

\- vias

\- preferred layers

\- congestion later

\- reservations later

\- electrical penalties



Hard electrical legality must never be represented merely as a soft penalty.



\## Testing



Create:



\- open 2-layer board

\- obstacle-detour board

\- voltage-clearance board

\- high-current width board

\- impossible blocked board



BoardVerifier must independently derive connectivity and legality from actual geometry.



Run the full suite.



Do not claim success until it passes.



Commit and push this phase.



At the end report:



\- branch

\- commits

\- build command

\- test command

\- tests passing/failing

\- CLI examples

\- JSON examples

\- electrical-rule behavior demonstrated

\- push status

\- exact work remaining for Prompt 2



\---



\# PROMPT 2 — Fine-pitch detection and strict inside-out escape routing



Continue from Prompt 1.



Read the spec and inspect all current code/tests first.



Do not rewrite working foundations.



Implement the complete fine-pitch escape stage.



\## Core requirement



BGA/LGA/dense-array routing is structurally \*\*centre-out\*\*.



This cannot merely be:



`score += centrality\_bonus`



Instead, deeper pads become eligible before shallower pads.



For a regular pad array, conceptually:



centre/depth 4



then depth 3



then depth 2



then depth 1



then perimeter/depth 0.



A shallower terminal may not be finally committed while a deeper eligible terminal has neither:



\- at least one viable candidate

\- nor an explicit temporary-infeasibility record for the current routing state



Implement this invariant and test it directly.



\## Implement



\- `FinePitchDetector`

\- `CentreDepthAnalyzer`

\- `EscapeBoundary`

\- escape portals

\- local escape-density map

\- legal immediate-exit-sector analysis

\- via-site scarcity analysis

\- K-best constrained A\*

\- candidate route signatures

\- dogbone escape

\- via-first escape

\- legal neckdown escape

\- multilayer escape

\- temporary infeasibility records

\- escape-result JSON diagnostics



\## Pin-density integration



Pin density matters here in two separate ways.



\### Component detection



Use:



\- pitch

\- pad dimensions

\- trace width

\- electrical clearance

\- channel count

\- pad count

\- local pin density

\- legal exit sectors



to determine whether a component deserves dedicated escape handling.



\### Same-ring ordering



Within equal centre depth, prioritize:



1\. fewest legal exits

2\. fewest via opportunities

3\. highest pin/escape density

4\. highest expected corridor contention

5\. highest downstream route difficulty

6\. stable terminal ID



Centre depth always outranks this.



\## Electrical constraints inside footprints



The escape router must honor:



\- amperage-driven trace width

\- legal neckdown rules

\- via/current constraints

\- voltage-dependent clearance

\- net-class layer restrictions



A centre pad requiring a large high-current trace is therefore allowed to consume more escape resources than a low-current signal where the rules demand it.



Do not silently shrink it to escape.



\## K-best requirement



Generate genuinely different choices.



Candidate signature should consider:



\- portal

\- principal direction

\- first via

\- layer strategy

\- bottleneck resources



Tiny geometric perturbations should not count as distinct.



\## Golden fixtures



Add at least:



\- 4×4 BGA

\- 8×8 BGA

\- 8×8 BGA requiring vias from inner rings

\- irregular array

\- dense QFN

\- high-current BGA pad

\- mixed-voltage fine-pitch region

\- greedy outside-first failure case

\- deliberately impossible escape case



Expose useful diagnostics through:



`router escape board --json`



Report:



\- footprint

\- pad

\- centre depth

\- density

\- eligibility order

\- candidate count

\- candidate portals

\- via decisions

\- infeasibility reasons



Run all old and new tests.



Commit and push.



Do not begin global parallel routing until this phase works.



\---



\# PROMPT 3 — Parallel global routing, density-aware scheduling and negotiated congestion



Continue from the working implementation.



Read the specification and previous commits.



Your goal is now to make global routing multicore without introducing nondeterministic board mutation.



\## Implement



\- connection-task creation from RouteTrees

\- route difficulty vectors

\- global pin-density contribution

\- probable route corridors

\- bottleneck estimates

\- task-interference graph

\- deterministic batch scheduler

\- soft route reservations

\- immutable routing epochs

\- worker pool

\- parallel A\* candidate generation

\- candidate conflict graph

\- deterministic central arbiter

\- atomic batch commit

\- present congestion

\- historical congestion

\- routing statistics

\- progress/event JSON



\## Route difficulty



Difficulty must incorporate at least:



\- endpoint pin density

\- local free routing space

\- corridor count

\- corridor scarcity

\- electrical trace width

\- voltage clearance burden

\- layer restrictions

\- via restrictions

\- span/distance

\- previous failures

\- fine-pitch continuation status



A 5 A wide-trace connection through a dense region must normally score as more difficult than an equivalent low-current signal.



A high-voltage-clearance net consuming a narrow channel must similarly reflect its larger geometric resource demand.



\## Parallel model



Workers:



\- operate on an immutable committed snapshot

\- never commit copper themselves

\- return candidate routes only



The arbiter:



\- revalidates every candidate

\- builds conflicts

\- selects a deterministic compatible subset

\- commits them as one epoch



Worker completion order must not change the final commit order.



\## Reservations



Before launching the batch, generate soft reservation costs from probable corridors.



High-density/high-difficulty tasks get stronger reservations.



Reservations are costs, not hard locks.



Never reject the only legal route purely because another task predicted it might use the resource.



\## Pathfinder-style congestion



Track separate:



`present\_cost`



and



`history\_cost`.



Resources repeatedly implicated in conflicts/failures must become progressively less attractive.



Physical illegal overlaps are never committed.



Negotiation affects planning pressure, not final DRC legality.



\## CLI/agent support



Extend JSON reporting so an external agent can observe:



\- epoch number

\- connected terminals

\- remaining terminals

\- active worker count

\- candidate routes

\- accepted routes

\- rejected/conflicting candidates

\- congestion hotspots

\- per-net failures

\- elapsed search work



Support a useful progress format through stderr or newline-delimited JSON when requested.



Do not add a GUI.



\## Tests



Demonstrate:



\- deterministic output at 1, 2, 4 and available higher worker counts

\- parallel workers do not mutate committed state

\- overlapping candidates are handled safely

\- independent routes commit in one epoch

\- density-aware scheduler prioritization

\- current-aware resource estimates

\- voltage-clearance-aware interference

\- narrow-channel contention

\- materially useful multicore execution



Benchmark against single-thread mode.



Do not fake speedups in tests.



Commit and push only after the full suite passes.



\---



\# PROMPT 4 — Rip-up/reroute, chess-engine-style meta-search and completion recovery



Continue from the existing router.



Now implement the machinery that prevents greedy A\* decisions from permanently stranding connections.



\## Implement



\- stall detection

\- A\* rejection/frontier diagnostics

\- blocker attribution

\- blocked-net dependency graph

\- route protection score

\- selective rip-up-set generation

\- rerouting generations

\- global high-impact decision search

\- 128-bit incremental/Zobrist-style state hashing

\- transposition table

\- action/move ordering

\- history heuristic

\- principal-variation reuse

\- iterative deepening/widening

\- parallel speculative high-level branches

\- escalating recovery modes



Do NOT implement minimax merely because these mechanisms are inspired by chess engines.



There is no adversarial player.



Do NOT use chess alpha-beta semantics.



The tree represents alternative routing decisions.



A\* remains responsible for detailed geometry.



Meta-search handles decisions such as:



\- which BGA escape bundle to use

\- which route owns a narrow channel

\- which layer-transition strategy to choose

\- which routes to rip up

\- routing-order changes after stalls



\## Route protection



Fine-pitch escape stubs should initially be more expensive to rip up than normal global traces.



Stable difficult routes should accumulate protection.



But nothing except fixed user copper is absolutely protected in exhaustive recovery if changing it is required to reach connectivity.



\## Recovery modes



Implement:



\### FAST



Weighted A\*, modest alternatives, strong parallelism.



\### RECOVERY



More candidates, larger detours, deeper rip-up, stronger historical congestion.



\### EXHAUSTIVE\_LOCAL\_RECOVERY



For remaining difficult tasks:



\- admissible A\* where practical

\- no unsafe probabilistic pruning

\- expanded via/layer options permitted by rules

\- larger search budgets

\- deeper high-level branching

\- expanded rip-up neighborhood



Do not claim mathematical proof of general unroutability.



Use a result equivalent to:



`SEARCH\_BUDGET\_EXHAUSTED\_WITH\_UNROUTED\_CONNECTIONS`



or:



`UNROUTABLE\_UNDER\_CONFIGURED\_CONSTRAINTS\_AND\_BUDGET`



with detailed evidence.



\## Objective



State comparison must retain strict connectivity priority.



Meta-search must never prefer:



0.1% shorter board + one unconnected pad



over:



longer board + complete legal connectivity.



\## Required test



Construct a board where naive greedy routing blocks a later net.



Show:



1\. initial route reaches a dead end

2\. blocker attribution identifies the obstructing route

3\. router rips appropriate geometry

4\. rerouting creates a different allocation

5\. all nets become connected

6\. verifier passes



Also test a fine-pitch escape bundle requiring reconsideration.



Commit and push this phase.



\---



\# PROMPT 5 — Optimizer, import/export, agent UX, exhaustive validation, benchmarking and GitHub release



Treat this as the integration/release pass.



Read the complete architecture and all source/tests before editing.



Do not paper over unfinished subsystems.



Your task is to produce a credible end-to-end v1.



\## Complete adapters



At minimum make one real PCB workflow fully usable.



Prefer:



\- Specctra DSN input + SES output



and/or the cleanest reliable KiCad-facing route available in this repository.



Keep adapters separate from the routing core.



Preserve:



\- net names

\- pad identities

\- layers

\- net classes

\- widths

\- clearances

\- via rules

\- current metadata where available

\- voltage metadata where available



Because common board formats may not contain current/voltage intent, allow sidecar configuration through JSON/YAML/TOML or equivalent.



Example concepts:



```yaml

nets:

&#x20; VBAT:

&#x20;   current\_a: 8.0

&#x20;   voltage\_v: 16.8

&#x20;   trace\_width\_min\_mm: 1.2

&#x20;   class: HIGH\_CURRENT



&#x20; USB\_D+:

&#x20;   voltage\_v: 3.3

&#x20;   class: SIGNAL



voltage\_clearance:

&#x20; - delta\_v\_min: 0

&#x20;   clearance\_mm: 0.15



&#x20; - delta\_v\_min: 50

&#x20;   clearance\_mm: 0.50

```



Do not treat this exact syntax as mandatory if the project already has a better configuration system.



\## Cleanup optimizer



Only run normal cleanup after complete legal connectivity.



Implement transactional transforms such as:



\- bend removal

\- trace shortening

\- via removal

\- local smoothing

\- preferred-layer improvement

\- congestion reduction



Every transformation must be reverted if it harms:



\- connectivity

\- legality

\- higher-priority electrical requirements



\## Agent-first CLI



Make the CLI exceptionally usable for coding agents.



Document examples such as:



```bash

router analyze board.dsn --config rules.yaml --json

```



```bash

router route board.dsn \\

&#x20; --config rules.yaml \\

&#x20; --threads 16 \\

&#x20; --seed 42 \\

&#x20; --output routed.ses \\

&#x20; --report route-report.json \\

&#x20; --json

```



```bash

router verify board.dsn --routes routed.ses --config rules.yaml --json

```



```bash

router explain-failure route-report.json --json

```



Output useful stable identifiers so an agent can iterate on:



\- exact failed net

\- exact terminal

\- component

\- blockers

\- congestion region

\- violated rule

\- required width

\- required clearance

\- attempted layers

\- attempted via classes

\- search generations



No GUI should be necessary for diagnosis.



\## Failure report



For every unresolved connection include machine-readable:



\- net ID/name

\- source/target component

\- electrical constraints

\- centre depth where relevant

\- pin density

\- failed candidate count

\- top blockers

\- congestion hotspots

\- rip-up attempts

\- route modes attempted

\- best partial state

\- recommended reason category



\## Golden suite



Run end-to-end fixtures including:



\- open 2-layer

\- obstacle routing

\- high-current routing

\- mixed-voltage clearance

\- narrow channel

\- 4×4 BGA

\- 8×8 BGA

\- irregular fine pitch

\- multi-terminal tree

\- forced rip-up

\- dense 4-layer MCU board

\- intentionally impossible design



Every routable golden fixture must reach 100% required connectivity.



Every successful fixture must independently pass BoardVerifier.



The intentionally impossible fixture must NOT falsely report success.



\## Determinism



Run identical jobs multiple times.



For equal:



\- board

\- rules

\- worker count

\- seed

\- router version



verify identical committed-geometry hashes.



Worker timing must not affect output.



\## Benchmarks



Record:



\- wall time

\- CPU utilization where measurable

\- worker utilization

\- A\* expansions

\- accepted/rejected candidates

\- epochs

\- rip-up generations

\- final connected percentage

\- via count

\- route length

\- peak memory



Compare:



\- 1 thread

\- 2 threads

\- 4 threads

\- as many useful hardware threads as available



Report real numbers, not estimated numbers.



\## Repository/release quality



Before declaring v1 complete:



\- remove dead scaffolding

\- run formatting

\- enable compiler warnings

\- run tests

\- run sanitizers if practical

\- document the CLI

\- document electrical rule configuration

\- document architecture

\- document supported/unsupported PCB features

\- document known limitations



Create a clean README aimed primarily at agent and automation users.



Include a copy/paste quick-start.



If GitHub access is authenticated:



\- ensure repository has `origin`

\- push all commits

\- push the working branch

\- if appropriate merge through the normal repository workflow

\- create a v0.1.0 tag only after all release gates pass



Do not force-push.



Do not claim GitHub upload unless the push command actually succeeds.



\## Absolute completion gate



You are not done because the project compiles.



You are not done because the architecture exists.



You are not done because one board routes.



Final report must provide actual evidence for:



\- build success

\- test success

\- golden fixture results

\- current-aware routing

\- voltage-aware clearance

\- pin-density behavior

\- centre-out fine-pitch routing

\- multicore routing

\- dead-end recovery

\- independent verification

\- deterministic results

\- CLI JSON output

\- GitHub push status



If any of these remain broken, continue fixing them within this run as far as available context and tooling permit.



Do not silently downgrade requirements.

