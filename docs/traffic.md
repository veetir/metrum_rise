# Metrum Rise - Traffic Movement

This document owns lane-bound vehicle movement behavior: car following, junction connector
traversal, lane changes, conservative overtaking, render smoothing, and traffic debug output.

It does not own:

- road surface / roadbed geometry; see [`roads.md`](roads.md)
- building entrance planning and exact attach / detach selection; see
  [`entrance_and_exit.md`](entrance_and_exit.md)
- stable constants tables and buffer formats; see [`reference.md`](reference.md)
- future public transport mode design; see `TRANSIT-*` rows in [`roadmap.md`](roadmap.md)

Implementation lives mainly in:

- `rust/src/simulation/economy/agents/tick.rs`
- `rust/src/simulation/economy/agents/data.rs`
- `rust/src/simulation/network/lanes/vehicle_junctions.rs`
- `rust/src/nodes/sim/render/lane_pose.rs`
- `rust/src/nodes/sim/core/snapshot.rs`
- `rust/src/nodes/sim/render/agents.rs`
- `godot/scripts/renderers/agents.gd`

## Current Scope

### Upstream integration acceptance — TRAFFIC-09

#### Conflict-zone implementation and spillback audit — 2026-10-25

The approved local reservation redesign is implemented. This does **not** close TRAFFIC-09:
the original 48-hour world now drains, but a longer continuation develops a closed loop of
full road lanes. The remaining audit must distinguish artificial blockage from genuine demand
exceeding the capacity of these 1+1 yield-controlled junctions; upstream flow parity is not
the acceptance criterion.
No right-of-way, collision, entry or performance threshold was weakened.

**Compiled authority.** Each conflicting connector pair owns one resource per connected
component of its swept-path overlap. Fractional segment/capsule clipping gives entry/exit
stations on both paths, rather than reserving whole sampled segments or connectors. These
resources live with the junction, have stable pair/path order, rebuild only with affected
junctions, and refresh their distance stations after terrain fitting. No overlap geometry
runs during traffic. The existing right-hand/major-road policy, complete stop requirement,
roundabout circulation priority and yielding-cycle arbitration remain authoritative.

**Vehicle claims.** Occupancy buckets seed physical body/tail ownership. A vehicle claims
reachable zones in path order; an opposing path cannot use a zone until the owner's tail
clears it. A same-path platoon retains ordinary following separation and names its rearmost
claimant, whose tail clears last. Wait vertices are individual vehicles, with immediate
following edges and remaining-zone dependencies. Before a claim or platoon-owner transfer,
retained DFS scratch checks that it cannot close a local circular wait. Connector cohorts
are insufficient: a leader must not inherit all of its followers' future waits. Sorted
junction/connector/bucket order and existing ordered handoffs make claims deterministic;
ordinary independent movement remains parallel. Released zones become available in the next
occupancy snapshot, so a parallel step never relies on another car moving first.

Full-crossing readiness still controls approach braking. Statutory permission can also let
a car stage on a clear connector prefix at junction crawl speed; a separate per-vehicle
travel bound stops strictly before an unclaimed zone, including at the maximum UI time step.
Touching an entrance is not occupation. A front strictly beyond entry and a tail strictly
before exit occupies the interval; f64 subtraction avoids f32 tail-clearance cancellation.

**Storage and entries.** Each admitted car also books one complete tail-clearing slot in
the downstream lane's free prefix, after promises to other in-flight cars. Connector exit
consumes that booking. Frontage, lateral and border insertion cannot consume booked storage
or leave their own tail across the mouth. First network entry now uses the existing ordered
gap/claim path; denied cars wait outside the lane without losing their trip, freight event,
target or scheduler state. They remain visible in raw network counts. Format **109** persists
`tail_connector_lane_id` through handoffs, compaction and lane remapping: choosing a guessed
incoming connector at a shared exit is unsafe. Older saves explicitly initialise it to
unknown; such an outgoing tail conservatively excludes new conflicting entry until clear.
The legacy road/upstream version histories remain distinct and supported.

**Bounds.** For one local junction, let C be connectors, Z zone references, J registered cars
and tails, E their remaining wait/following links, q a lane's occupants, B local successors,
and z_v a vehicle's path zones. One vehicle costs O(z_v·(J+E) + log q + B), with empty-opposite
paths skipping DFS. Junction preparation/claims cost O(Z·log q + C·B + E +
Σ_v z_v·(J+E)); statutory Tarjan remains O(C+K) for K yield links. Finding and sorting active
junctions costs O(D·log D) for D occupied lanes. There is no citywide agent/spatial scan in
zone arbitration. Storage is retained O(A+L+Z+J) scratch for A agent-index slots and L lanes;
only active slots reset. Z counts the allocated zone-ID high-water until a full lane rebuild,
as incremental rebuilds preserve remote IDs. Buffers grow before visitation, with no steady-state
vehicle-loop allocation. Geometry compilation costs O(C²·P²) time and O(P²) temporary pair workspace for
P sampled segments. The local DFS is an explicit correctness cost beyond the former coarse
O(conflict degree) gate; equal-work PERF-06 measurements below bound its measured impact.

**Prior-session safety evidence (inspected, not rerun in this takeover).** The agent group passes 162 tests and the save group 64. Independent
body/tail assertions observe actual lane transitions, rather than trusting reservation owners.
Finite all-yield, opposing-left, short-link, roundabout, ramp, pocket and bus-lane fleets drain
without overlapping opposing pair zones, with identical one/eight-worker results. Additional
regressions cover a busy-prefix stage, exact entrance touching, maximum-speed steps, booked
exit storage, height/incremental cache rebuilds, border holds and v109 mid-exit save/remapping.
The existing priority/stop/roundabout/ramp/pocket tests are retained. This is constructed-case
coverage, not a proof against arbitrary saturated road-network cycles.

**Unchanged common initial world, 48 hours.** New zone reservations end with 6,891 employed,
2,836 raw network occupants, raw car speed 1.957435 m/s, 1,176 physical network cars at
4.720480 m/s and 1,660 outside holds; non-midnight hour mean in the final replay is 0.430580 ms. The upgraded
regression fails on unchanged `0cc4a2ab` production behavior: 8,225 raw occupants, 8,208
physical cars and 0.097857 m/s. Its newly generated initial agents and saved-lane rows match
the preserved original exactly. Passing the regression means drainage, at least the game's
minimum junction speed and preserved initial employment; it does not certify parity with
upstream's 944 / 9.112 m/s. That gap remains explicit.

A separate diagnostic build applies **only the border hold** to plain `b75e03c9`: it ends
with 7,019 employed, 1,321 raw network occupants, 1,310 network cars at 6.123514 m/s,
875 physical cars at 9.136034 m/s and 435 outside holds (hour mean 0.219886 ms). This shows
why outside holds explain part of the raw-count difference, but cannot explain the slower
physical traffic under safe junction rules. This build is a causal control, not plain
upstream acceptance or a production alternative.

**Remaining failure with evidence.** The new default day-12 10k snapshot has 2,649 rows with
workplaces, 7,065 raw network occupants and raw car speed 0.561691 m/s (5,768 physical cars,
1,269 holds). A diagnostic continuation to day 14 reaches 8,120 raw occupants, 6,461 physical
cars at 0.285067 m/s and fails the same progress assertion. It contains the directed lane
cycle `732 → 738 → 1572 → 979 → 973 → 1513 → 732`: four 39 m lanes hold 14 cars each and
two 389 m lanes hold 144 each, for **344 cars**. Their head requests are connectors
`10674, 10751, 13807, 13725, 13651, 10605`. All six crossings are clear, all have zero zone
bookings, and every downstream entry capacity is zero. The first cars are at 3.9 m on the
short lanes and about 2.9 m on the long ones; neither leaves the 5.3 m prefix needed to
accept another 2.6 m body with the existing 0.1 m gap and clear its tail. No car can legally
start the transfer that would free the next road. This is road-storage circular wait, not
an occupied junction or a conflict-zone ownership cycle. The unchanged upstream-plus-border
control remains employed at 7,202 on day 12, with 2,644 raw occupants, 1,212 physical cars,
1,417 holds and raw car speed 3.035606 m/s.

These observations establish full downstream storage, not the cause of its accumulation.
The follow-up is bounded to one five-hour window and looks for true blockages: unreleased
claims or holds, missed wait cycles, faulty platoon-owner transfers, clear-zone starvation
and legally permitted moves that fail to proceed. Each confirmed fault needs a root fix
and regression. Genuine saturation must instead be documented with offered demand and
measured junction throughput; signals or larger roads are the realistic capacity answer.
No network-wide admission redesign is required merely to reproduce unsafe upstream flow.
No clearance, priority, fixture or trip/job deletion workaround is shipped. TRAFFIC-09 stays
open pending that audit and the requested final performance-contract rerun.

#### Fresh release acceptance checks for the zone model

The following are completed prior-session measurements, inspected during takeover;
they are not new reruns. All runs below are serial and unprofiled on Apple M2 Pro, Rust 1.98.1 / macOS SDK 26.5,
eight Rayon workers, `METRUM_DEBUG=0`, without concurrent builds/tests/Godot. The final model
is Rust `50e0b60d`; full-contract binary SHA-256 prefixes are plain `b75e03c9`
`c406e5be621f4bea` and merged `bfc70121e0182eb7`. The separate plain-upstream matched-world
driver is `5e68…` (test instrumentation only); its fresh replay reproduces the original
944 / 9.112031 result. `git_head` in tool reports identifies the orchestration checkout,
not the source of an archived binary.

The complete commands, without narrowing phases or shortening windows, are:

```sh
METRUM_DEBUG=0 RAYON_NUM_THREADS=8 ./run.sh --perf-contract \
  --binary "$TEST_BINARY" --artifacts "$ARTIFACTS/final-contract"
METRUM_DEBUG=0 RAYON_NUM_THREADS=8 "$TEST_BINARY" \
  simulation::economy::agents::tick::contract::benchmark_agent_tick_contract \
  --exact --ignored --nocapture --test-threads=1
```

Both full contracts **fail**: upstream has the same five work-scaling failures in **76.50 s**;
merged has two in **97.22 s**. All absolute hour/day limits, locality gates, agent gates and
fiscal churn coverage pass. Upstream failures remain baseline buyer sites 10→369, staffing
trials 49,289→231,170, job-search keys 64→15,448, fiscal buyer sites 716→2,764 and CCH searches
6,771→26,851, against 3.4283×. Merged fails baseline supplier searches 14,614→53,548 and
supplier routes 177,611→741,463, against 3.5442×. The observed excessive work is retained;
no gate or fixture was changed to pass.

| Complete next-day hour mean, ms | Plain upstream | Zone model |
| --- | ---: | ---: |
| Baseline 10k | 1.017743 | 2.463531 |
| Baseline 30k | 3.947025 | 6.582522 |
| Fiscal stress 10k | 7.062749 | 11.902543 |
| Fiscal stress 30k | 29.432408 | 37.583617 |

These unchanged default recipes use the corrected 1e10 treasury, unlike the preserved
1e15 common-world reproducer. Their own warm-ups diverge. At the saved baseline boundary,
resident employment uses the production census diagnostic (not merely non-null job rows):

| Tier / build | Employed | Raw network occupants | Physical network cars | Outside cars | Raw car speed, m/s | Physical car speed, m/s |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 10k upstream | 7,186 | 2,481 | 2,443 | 17 | 4.953905 | 4.988378 |
| 10k zone model | 2,621 | 7,065 | 5,768 | 1,269 | 0.561691 | 0.685267 |
| 30k upstream | 16,715 | 7,951 | 7,680 | 227 | 3.277907 | 3.372985 |
| 30k zone model | 5,186 | 22,467 | 18,601 | 3,838 | 0.360038 | 0.434326 |

After the measured next day, baseline employment is 7,179/3,040 at 10k and 16,674/5,191 at
30k (upstream/model). Fiscal final employment is 67/67 and 201/245. Thus the hour means are
observations of different resulting work, not equal-work acceptance or a claimed speedup.
One-time baseline fixture generation, separately timed, increases **30.68→53.35 s** at 10k
and **95.83→164.20 s** at 30k; fiscal preparation is **13.10→22.83 s** and **35.98→72.20 s**.
Exact saved/straight-through continuation checks pass for all four new fixtures.

PERF-06 repeats five upstream/model pairs, alternating pair order. All 60 records have
matching work dictionaries for their six corresponding modes. Median per-tick ms
(min–max across the five independent invocations):

| Mode | Agents | Upstream | Zone model |
| --- | ---: | ---: | ---: |
| Network | 10k | 0.518 (0.419–0.570) | 0.588 (0.510–0.788) |
| Network | 30k | 1.010 (0.890–1.159) | 1.044 (0.860–1.391) |
| Access egress | 10k | 0.781 (0.752–0.859) | 0.727 (0.702–0.750) |
| Access egress | 30k | 1.683 (1.663–1.745) | 1.712 (1.640–1.777) |
| Access ingress | 10k | 0.472 (0.363–0.555) | 0.414 (0.380–0.543) |
| Access ingress | 30k | 0.672 (0.466–1.053) | 0.696 (0.584–0.827) |

The 30k observed increases are 3.4%, 1.7% and 3.6%; the 10k network median rises 13.4%.
Ranges overlap and macOS has no affinity control here, so these short mixed observations
establish neither a general speedup nor a significant change. PERF-06's long-road fixtures
have no occupied junction conflicts: they check equal-work dispatch/following/access overhead,
not worst-case zone DFS. The dense-world failure therefore remains a performance blocker.
The previously completed PERF-07 matrix remains **historical and unaccepted**, including
missing macOS run-queue statistics; it was not rerun for this follow-up and is not presented
as validation of the new model.

Fresh road chunk and six populated locality comparisons against `0cc4a2ab` pass their
unchanged product/bounds assertions; the new constant-cost differences, separate setup
measurements and rendered inspection are recorded in `roads.md`.

#### Historical controlled diagnosis after legacy migration

The jam already exists on pre-merge `0cc4a2ab`. Its unchanged 10k fixture, admitted through
`PopulatedCity::admit_households` and advanced through the usual 48 warm-up hours, ends with
8,225 network occupants and mean network-car speed 0.097857 m/s. This run happens before
any reload, so the earlier vacancy-reserve continuation failure cannot explain the jam.

A task-owned v95 snapshot immediately after that original admission is the common initial
world for the following runs. All start with 7,396 buildings, 12,242 agents, 10,000 residents,
6,690 employed agents, 2,263 network occupants and seven intersection occupants. Hashes of
nodes, control/physical geometry and directed saved-lane topology match across old, merged,
all mechanism controls and plain upstream. All 446 edges have 1+1 lanes: there are no pockets
and zero lanes requiring turn preparation. ROAD-67 lane choice/pocket-capacity branches
therefore do no work in this fixture; this is not an acceptance claim for a multilane city.

Eight-worker, serial, unprofiled release results after the same 48 hours, Apple M2 Pro,
`METRUM_DEBUG=0`, maximum fixture speed:

| Build / isolated mechanism | Employed | Network occupants | Car speed, m/s | Non-midnight hour mean, ms |
| --- | ---: | ---: | ---: | ---: |
| Original road chain, straight through | 6,937 | 8,225 | 0.097857 | 10.592841 |
| Plain upstream, common initial world | 7,210 | 944 | 9.112031 | 0.245382 |
| Merged, all road rules | 6,957 | 7,216 | 0.192786 | 0.459187 |
| Diagnostic: precedence removed, crossing reservations retained | 6,899 | 7,290 | 0.178700 | 0.413355 |
| Diagnostic: snapshot conflict-occupancy gate bypassed | 7,181 | 2,115 | 3.302306 | 0.440684 |
| Diagnostic: downstream admission gate bypassed | 6,825 | 6,440 | 1.075079 | 0.443910 |
| Diagnostic: connector controls removed | 7,210 | 1,000 | 9.124341 | 0.466623 |

The mechanism variants exist only in the task-owned experimental source archive/test builds;
none is a production fix. Precedence removal clears statutory yield/stop decisions while
retaining crossing occupancy, downstream admission and same-tick conflict claims. Snapshot
bypass leaves precedence, downstream admission and same-tick conflict claims in place. Exit
bypass leaves precedence and crossing checks in place. Ordinary road entry/exit gap and lane
claims remain in every variant. Complete control removal is an intentionally unsafe causal
control, not a permitted repair. These are matched *initial* workloads; their eventual trip
and employment counts differ. The fixture retains the old chain's 1e15 funded treasury;
whole-contract runs below retain upstream's corrected 1e10 fixture. Do not mix their means.

Plain upstream cannot read road v95. For this no-feature grid only, the merged build loads
that snapshot and writes v108; a new task-owned copy advertises v107 for the upstream reader.
The shared SQL/runtime schema is upstream v107, the added road tables are empty/default,
and the initial topology/agent-count hashes above verify this control. Neither the canonical
initial snapshot nor any player save is edited. A preliminary reload-driver attempt remained
paused because production load pauses the clock; it produced no timing observation. The
corrected driver explicitly restores the existing fixture's maximum speed after load.

The responsible mechanism is the road chain's coarse connector conflict reservation, rather
than the new scheduler or the statutory right-hand/major-road ordering alone.
`JunctionAdmission::prepare` marks a movement clear only when **every conflicting connector
bucket is empty**; `LaneClaimContext::connector_allowed` also reserves the entire conflicting
movement during a handoff. Conflict lists are derived once from any overlap of the complete
swept connector trajectories. Thus the intersection reserves whole traversals, rather than
releasing individual conflict zones when a vehicle's tail clears them. Turning/braking and
exit-storage holds extend those reservations; removing precedence alone cannot restore flow.
The two narrower controls show both the occupancy gate's dominant effect and the additional
cost of downstream holding. Disabling the checks would trade the jam for vehicle conflicts.

`TRAFFIC-07` is independently present: before warm-up, 2,182 cars occupy distance zero of
border-entry lane 2040. At hour 48, the merged city still stacks 1,839 there; plain upstream
from the same initial world clears the stack completely. `prepare_lane_entry` still bypasses
the ordinary attach-gap/claim path for first network entry. Overfill supplies unrealistic
pressure, but the unchanged initial stack drains under upstream/control-removal behavior;
it does not by itself account for the road-rule throughput collapse. No entry check was
loosened, no fixture changed and no cars/jobs deleted to obtain these results.

The original handoff stopped at the redesign decision. The user subsequently approved the
conflict-zone implementation above; that earlier stop is superseded. The historical controls
below remain evidence of their original builds, not validation of the new reservation model.

The committed ignored driver `nodes::sim::core::populated_city::traffic09::dense_city_warmup`
records six-hour checkpoints, initial/final SQL snapshots and lane/control geometry without
altering production rules. With `METRUM_TRAFFIC09_INPUT` omitted it constructs the normal
fixture; with it set it loads the common initial snapshot read-only. A fresh replay of the
committed driver reproduces all nine authority checkpoints exactly; its all-target Clippy
check is clean. The timings in the table remain those of the original matched sequence:

```sh
METRUM_DEBUG=0 RAYON_NUM_THREADS=8 METRUM_TRAFFIC09_INPUT="$ARTIFACTS/initial.sqlite" \
  METRUM_TRAFFIC09_OUTPUT="$ARTIFACTS/fresh-run" "$TEST_BINARY" \
  nodes::sim::core::populated_city::traffic09::dense_city_warmup \
  --exact --ignored --nocapture --test-threads=1
```

#### Historical whole-contract rerun after migration

The road/scheduler merge is not accepted for dense-city behavior. Fresh serial, unprofiled
release reruns after the migration use upstream `b75e03c9` and merged Rust `cfcb2bd3`, Apple
M2 Pro, eight Rayon workers and `METRUM_DEBUG=0`, with no concurrent builds/tests/Godot.
Both full `./run.sh --perf-contract` checks still fail work-scaling limits: five on upstream
in 76.44 s, three on merged in 85.94 s. All absolute hour/day limits pass. Revision-checked
copies of the original complete contract caches are retained; no fixture or threshold was
changed. These default day-12/day-16 cities have diverged during their own warm-ups, unlike
the common initial world in the causal experiment above.

| Full-day hour mean, ms | Plain upstream | Merged |
| --- | ---: | ---: |
| Baseline 10k | 0.999110 | 2.500121 |
| Baseline 30k | 4.143611 | 6.753725 |
| Fiscal stress 10k | 6.961491 | 9.433688 |
| Fiscal stress 30k | 29.796770 | 26.580440 |

The pre-merge road binary `0cc4a2ab` fails its 10k snapshot's exact save/reload preparation
check: `vacancy_reserve` changes from 531 to 511 on reload, while the other recorded fields
match. No full economy timing from that rejected snapshot is used. The table therefore
compares plain upstream with merged, not the older road binary with merged.

The prepared day-12 10k saves already diverge before loading: upstream has 7,186 employed
agents and 2,481 network occupants; merged has 716 employed and 9,092 network occupants.
Car speed on network lanes averages 4.954 versus 0.052 m/s. This establishes a dense-traffic
flow/behavior regression during fixture warm-up, not a save-loader loss. The controlled
follow-up above identifies the connector-reservation bottleneck. These day-12 figures remain observations of their original builds, not matched initial
workloads or proof of a repaired city.

Upstream work-limit failures are baseline buyer sites 10 → 369, staffing trials 49,289 →
231,170 and job-search keys 64 → 15,448; fiscal buyer sites 716 → 2,764 and combined CCH
searches 6,771 → 26,851. Their 10k→30k bound is 3.4283×. Merged failures are baseline supplier
routes 144,303 → 530,202 and combined CCH searches 9,465 → 33,953; fiscal job-search keys
7 → 59, against 3.5112×. These failures remain visible; none of the limits were weakened.

At the prepared default baseline boundary, upstream/merged 10k employment is 7,186/716,
network occupancy 2,481/9,092 and car speed 4.953905/0.051891 m/s. At 30k it is
16,729/1,932 employed, 7,951/26,131 network occupants and 3.277907/0.076792 m/s. These
nominal tiers are not matched work, and their hour means cannot certify equal-work performance.
The earlier 88133881 means (1.058/2.541 ms at 10k and 4.040/6.708 at 30k) remain historical
observations; the new measurements above do not establish an improvement in the traffic model.

Reproduce both full runs with `./run.sh --perf-contract --binary "$TEST_BINARY" --artifacts
"$ARTIFACTS/fresh-contract"`, without `--phase`, `--agents` or shortened periods. Retained
binary SHA-256 prefixes are upstream `c406e5be621f4bea` and merged `998cd542b60b40b9`;
report `git_head` denotes the orchestration checkout, not an archived binary's source.

The short `PERF-06` agent contract passes on all three binaries. Upstream and merged have
identical work dictionaries in all six 10k/30k modes; the older road binary also counted the
congestion pass that upstream deliberately removed. Earlier 30k per-tick observations, ms:

| Mode | Road `0cc4a2ab` | Upstream `b75e03c9` | Merged `88133881` |
| --- | ---: | ---: | ---: |
| Network | 0.963 | 0.968 | 1.062 |
| Access egress | 1.595 | 1.644 | 1.750 |
| Access ingress | 0.604 | 0.809 | 0.686 |

These are single short contract invocations, not repeated sustained-tick acceptance or
evidence that the dense-city regression is repaired.

Separately, the production bridge loads untouched populated upstream v107 saves at 10k and
30k with exactly 7,396/11,042 and 20,359/31,597 buildings/agents. Both advance at least a full
hour without a bridge error. This verifies upstream runtime loading, not dense-city throughput
acceptance. The road-chain v88–v95 copied-save/fingerprint probes, restored upstream v57–107 readers and
university bridge now pass; the migration evidence is recorded in the
[road integration checkpoint](roads.md#upstream-integration-checkpoint--2026-10-24).

### Automatic vehicle junction priority — TRAFFIC-08 stage 1a

The road-chain implementation was tracked as `TRAFFIC-06` before upstream integration.
It now uses `TRAFFIC-08`; upstream's `TRAFFIC-06` planning row remains preserved.

Stage 1a adds automatic vehicle right of way to the existing connector reservations. It is a staged
implementation: zebra priority, fixed-time signals, authored control/major-road selection, all-way stop,
control previews/undo and importer setters are still pending. ROAD-62 now supplies explicit roundabout entry priority (below).
The signalised Prisma reference is **not** represented by this stage's priority capture.

At a mixed-hierarchy junction the higher road has priority. Hierarchy is the ordered tuple
(design speed in mm/s, original through-lane count, original carriageway width in mm).
`EdgeClass` describes structures (surface/bridge/tunnel), not an OSM road class, so it does not
set hierarchy. Turn pockets use their original cross-section and cannot promote a side road.
Equal-hierarchy approaches use the Finnish right-hand rule; a left turn yields to conflicting
oncoming straight/right movements. Where trajectories overlap without a statutory ordering,
connector order breaks the tie. Default NET-07 U-turn legality is unchanged.

Lesser approaches yield. A road at most 30 km/h joining a major road at least 60 km/h gets a
stop control: its leading vehicle must be within 5 cm of its lane mouth and below 0.1 m/s in
a completed occupancy snapshot before it can enter. This requires a full stopped tick, not
an arbitrary dwell timer. B1/B5/B6 signing is derived from the identical policy through ROAD-60.
These are explicit game defaults, not inferred import metadata.

Lane rebuilding compiles local swept-trajectory conflicts and directed yielding relations.
Opposing cars exclude each other only while occupying the same compiled conflict zone;
a vehicle tail releases each interval separately, as specified under TRAFFIC-09. Waiting priority
persists while that crossing is occupied, so another minor-road arrival cannot follow the
crossing car ahead of a waiting major approach. A blocked outgoing road does not reserve
precedence for a movement that cannot leave the junction.
Admission also checks the outgoing road's entry slot to
avoid deliberately entering a blocked exit. It reserves the existing connector lane, ordered paired zones and complete downstream
body storage in the stable serial handoff phase.

Before speed updates, immutable lane buckets supply leading approach requests. The lookahead
covers braking distance, a 20 m approach zone and the current tick's travel.
Strongly connected components in the active yielding graph identify a
closed right-hand waiting cycle; its lowest agent index proceeds. A minor movement outside
the cycle cannot acquire priority by the tie-break. Per-agent speed updates and ordinary
movement remain parallel, and IDM brakes against denied admission at the lane mouth.

The admission prepass visits nodes reached from occupied lane buckets and reuses retained
scratch; the zone/vehicle wait-graph bounds and persistence contract are specified above.
No new spatial index is introduced. Full/incremental rebuild and clear own the derived
geometry cache lifecycle. Steady-state arbitration and agent loops allocate nothing.

**Zebra boundary found during implementation.** Vehicle connectors start at the asphalt
junction throat, but zebra centres are `CROSSWALK_INSET` (4 m) farther out on the physical
road lanes. A connector-only pedestrian conflict check misses these crossings. Stage 1a
therefore leaves pedestrian admission unchanged; it does not claim zebra priority. The next
stage must put the stop point before the stripes and reserve the entire incoming/outgoing
crossing portion, retain the existing turn shape, support degree-two direct continuations,
and preserve full/incremental rebuild equivalence and agent/save references. Composed walk
routes must obey the same crossing authority. Do not patch this with a late connector gate.

The priority feature first shipped with road-chain format 91; its policy and scratch remain
derived. The combined writer is format 109. Explicit upstream and road-chain readers preserve
their separate histories, including road-chain v88–95; legacy loading is repaired. Compatible loads apply the derived priority policy
without rewriting their source file; the feature's original captures used fresh blank worlds.

### Roundabout entry priority — ROAD-62

Circulating carriageways have an explicit persisted role, ahead of the existing automatic
(speed, original lanes, original width) hierarchy. Every entering arm yields to conflicting
circulating connectors, including an 80 km/h approach to a 20 km/h ring; circulation never
acquires an obligation to yield to an entering arm. Circulating arms do not emit B1 road-priority
signs. Entering arms emit the B5/D2 compound sign from the same policy. Existing connector
conflicts, downstream entry-slot checks and stable handoff claims apply without a separate
roundabout tick loop. Ordinary roads retain their original hierarchy and right-hand rules.

No new agent fields or runtime allocations are introduced. All added classification occurs
at local lane/sign rebuilding. The tangent-continuous single-lane road ring uses the existing
connector path builder and NET-07 routing; no new U-turn exception is needed. A targeted test
loads six uneven arms with 36 cars taking the long path around the ring and verifies drainage
and bit-identical final edge/distance/speed results with one and four Rayon workers. This
fixture has open exits; it does not promise flow through physically blocked downstream roads.

Geometry/tool contracts, first-slice limits and fresh acceptance evidence live in
[ROAD-62](roads.md#single-lane-roundabout-authoring--road-62-first-slice). Two-lane destination
lane choice, spiral exits and pedestrian crossing priority remain pending.

#### Stage 1a acceptance — 2026-10-20/21

Baseline is ROAD-68 checkpoint `7990a6d6`; candidate is the vehicle-priority working tree.
Apple M2 Pro, Rust 1.98.1, macOS SDK 26.5 and Godot 4.7.2 (gdext 4.5 API).
Preserved unprofiled release artifacts and individual logs live in `$ARTIFACTS`.

| Artifact | Baseline SHA-256 | Candidate SHA-256 |
| --- | --- | --- |
| Library test executable | `d41b15c620baec9088fbaf87ce1a08f3ecca323c29b622b90cb4181246cda452` | `4972ab34cbe199b60cafed4a17f0c8235af3041cce1298cb65ba62d15b865195` |
| GDExtension library | `fd3a90faa20e306e0e19bca47ac34376676f4a5efe8af0e7255d6037aed3d505` | `a52a5eb774b463a7456b502821c3fa56684f373d8b3c0df3256ba6f0c8af6338` |

Fresh correctness commands, from the repository root:

```bash
export SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk
cargo test --manifest-path rust/Cargo.toml --lib
cargo clippy --manifest-path rust/Cargo.toml --all-targets
cargo test --manifest-path rust/Cargo.toml --release --lib --no-run
"$TEST_BINARY" junction_control --nocapture --test-threads=1
"$TEST_BINARY" automatic_priority_yield_and_stop_models_follow_approach_policy --nocapture --test-threads=1
"$TEST_BINARY" test_junction_gate_prevents_stacking --nocapture --test-threads=1
```

The full debug library suite passed **2,611 tests, zero failures, 148 ignored**; all-target
Clippy passed without warnings. Ten new admission tests cover hierarchy independent of agent
storage order, equal-road right-hand yielding, oncoming left turns, cycle draining, priority
retained during an occupied crossing, full stops, occupied/conflicting claims, incremental
cache retirement and pocket hierarchy. The sign test
checks B1/B5/B6 model policy. Existing junction-spacing regressions still check every tick and
now allow up to 500 simulated seconds for the newly yielding queues to drain, stopping early
on success.
A fan-connector regression exposed and fixed braking against a denied representative instead
of an admissible shorter routing candidate.

Each bridge ran individually against the final native release library in a separate isolated
Godot test profile; no player mods/saves were loaded:

```bash
for test in road_junction_control_test road_junction_preview_test road_preview_stream_test \
    road_cross_section_test road_turn_lanes_test road_pockets_test road_pockets_legacy_test \
    road_trajectory_test road_grade_separation_test junction_debug_overlay_test; do
    METRUM_DEBUG=0 RAYON_NUM_THREADS=4 perl -e 'alarm 900; exec @ARGV' \
        godot --headless --path "$ARTIFACTS/profile/project" --script "res://tests/$test.gd"
done
```

All ten passed. The legacy bridge retains all four roadbed golden hashes from `6ec0b858`;
paint and sign transforms are intentionally outside those geometry hashes. The new bridge
builds a divided 2+2 major road, 1+1 minor road and median/kerb pockets through Rust tools,
checks sign poses across the bridge, and runs a car through its pocket route.

PERF-06: **15 alternating baseline/candidate process pairs**, eight Rayon workers,
`METRUM_DEBUG=0`, no concurrent build/capture. Each executable ran:

```bash
RAYON_NUM_THREADS=8 METRUM_DEBUG=0 "$TEST_BINARY" \
    simulation::economy::agents::tick::contract::benchmark_agent_tick_contract \
    --exact --ignored --nocapture --test-threads=1
```

| Workload | Agents | Baseline median ms | Candidate median ms | Paired median delta, 95% interval |
| --- | ---: | ---: | ---: | --- |
| Network | 10,000 | 0.519125 | 0.500292 | -4.93% [-10.99%, +4.37%] |
| Network | 30,000 | 0.888083 | 0.906708 | +0.81% [-7.30%, +9.70%] |
| Access egress | 10,000 | 0.652667 | 0.647625 | +1.80% [-1.64%, +3.08%] |
| Access egress | 30,000 | 1.543791 | 1.569250 | +1.04% [-1.45%, +4.98%] |
| Access ingress | 10,000 | 0.349916 | 0.344125 | -4.22% [-12.24%, +12.18%] |
| Access ingress | 30,000 | 0.511083 | 0.585083 | -7.05% [-10.94%, +28.77%] |

All work counters match, including linear agent counts and 16 fixed frontage lanes. Every
interquartile range overlaps and every paired interval includes zero: no regression beyond
noise is detected on the existing PERF-06 fixtures. Paired intervals use 10,000 bootstrap
resamples (seed 606); the median of pair ratios differs from the ratio of independent medians.
This establishes neither a speedup nor saturated signal/junction throughput. Road rebuild,
locality and visual evidence belong to the [ROAD-60 acceptance](roads.md#road-signs-road-60).

### Restricted ramp movement — ROAD-69 first slice

Ramp stubs use the existing physical lanes, node-to-node routing states and target-lane
reservations. Their explicit fork whitelist connects through lanes by rank, isolates the
opposite carriageway, and dedicates the added outer lane to the one-way stub. The downstream
acceleration taper merges that added lane into its adjacent through lane using the existing
slice B fork/drop mapping and gap checks. `Ramp` nodes have no ordinary right-hand, stop or
priority admission cache and no zebras; ordinary junctions at later ramp terminals keep their
existing automatic controls. No new tick loop, agent field, hot-path allocation or U-turn
exception is added. Source/target lane offsets are still rebuilt, never decided in GDScript.

Fresh verification on the ROAD-69 release: the complete library suite passes 2,624 tests,
including shared blocked-merge serialization. The native ramp bridge drives an exit or entry
plus both through carriageways in all four direction/role combinations. Thirty alternating
PERF-06 process pairs with eight Rayon workers retain identical work counters. The 30k network
short-test paired ratio is 1.117 (95% interval 1.029–1.229), so its timing acceptance remains open.
Three matched sustained production pairs measure 2.890→2.861 ms at 100k and 37.647→38.331 ms
at 1M on-road agents; the latter is a +1.8% observed change within process-pair variation.
Build identities, exact commands, timing tables and the populated-road locality check are recorded in the
[road acceptance](roads.md#first-slice-release-verification--2026-10-21).

The corresponding authoring scope and pending signs, islands, interchange presets and import
mapping are recorded in [roads.md](roads.md#restricted-ramp-stubs--road-69-first-slice).

### Bus-only car exclusion — ROAD-72 lane-use slice

Reserved physical ribbons remain in road geometry but are absent from the car `LaneSystem`.
Existing car spawning, frontage, lane-change and connector queries therefore cannot select
them. Default movement allocation uses the remaining general lanes. Car routing removes
all-bus directional traversals and filters explicit whitelist pairs naming a reserved source
or target; walking is retained. Legal right-turn connectors may cross a reserved ribbon, but
longitudinal bus/right-turn sharing is pending. NET-07's dead-end-only U-turn rule is unchanged.
There is no bus vehicle mode yet; `Edge::lane_use` exposes the saved class permission for its
future lane graph. No agent layout, movement loop or tick allocation changes.

Saved masks and transactional edit behavior are owned by
[the road contract](roads.md#bus-only-lane-authoring--road-72-lane-use-slice).

### Imported road compiler completion — ROAD-73

The ROAD-73 span-frame repair keeps successful existing section geometry and all lane
identities unchanged. Rejected spans may receive a validated nonuniform knot frame; surface
queries and node mouths share its exact retained sections. Overlapping connected junctions
now use one source-constrained grade, with conforming internal span mouths and material
boundaries. Nonroad composition gives connected carriageways consistent visible precedence.
The full-width university rebuild retains 3+3 lanes and both bus lanes, passes exact
preview/commit, drives traffic and survives save/reload. Bus lanes use ordinary Finnish
asphalt with white BUS lettering; the inherited red fill is removed.

No agent layout, movement loop, reservation policy, NET-07 behavior or tick-time allocation
changes. Compiler and renderer measurements are not agent-tick performance claims. Source
exceptions, legacy grade comparisons and fresh acceptance evidence are owned by
[the ROAD-73 road contract](roads.md#imported-junction-geometry--road-73).

### ROAD-72 release traffic verification — 2026-10-21

Baseline is `c81d4477`; final source/build identities and road acceptance are in
[roads.md](roads.md#lane-use-slice-verification--2026-10-21). The agent layout and tick loops
are unchanged. All six PERF-06 mode/tier work dictionaries match exactly across builds and
repeated fresh fixtures; agent work grows 3× from 10k to 30k and lane work remains constant.

The first 15 alternating process pairs show +10.5% network and +5.8% egress at 10k (ratios of
medians). A focused 30-pair repeat on the same binaries does not reproduce a stable positive
increment. Short-test process timing is variable, so neither campaign establishes a speedup.
The repeat below shows process medians in milliseconds and the median paired percent change
with its deterministic 10,000-resample percentile bootstrap interval:

| Mode / agents | Baseline ms | Candidate ms | Paired change %, 95% interval |
|---|---:|---:|---:|
| `network` / 10000 | 0.738 | 0.625 | -13.9 [-28.8, -2.3] |
| `network` / 30000 | 1.417 | 1.335 | -7.8 [-13.6, +0.5] |
| `access_egress` / 10000 | 0.700 | 0.737 | -0.5 [-6.2, +13.9] |
| `access_egress` / 30000 | 1.703 | 1.723 | -1.4 [-8.5, +0.4] |
| `access_ingress` / 10000 | 0.475 | 0.432 | -5.9 [-24.7, +19.4] |
| `access_ingress` / 30000 | 0.887 | 0.830 | -4.4 [-17.3, +2.6] |

To check production cost without test-only counters, the existing Criterion `agent_benchmark`
was freshly built for both sources. Eight workers, 30 samples, 2 s warmup and an 8 s requested
measurement window; construction is outside timed ticks. The first three inherited-policy
process pairs are too variable for timing acceptance: paired ratios span 0.663–2.141 at 100k
and 0.643–1.422 at 1M. One bounded follow-up applies macOS application resource policies
(`taskpolicy -a`) equally to both builds, with three alternating process pairs and no global
system-setting change:

| Agents | Baseline median ms | Candidate median ms | Change | Observed paired ratio range |
|---:|---:|---:|---:|---:|
| 100000 | 3.611 | 3.609 | -0.1% | 0.900–1.001 |
| 1000000 | 44.263 | 42.580 | -3.8% | 0.948–1.034 |

Production benchmark SHA-256: baseline `61aa987b0049284569717a2dfcf484e3094243d64fce95eec3a5b2d1fdfd24ad`, candidate `7f6c801f7cd8c773252e417a576f8b21604b8718e3006bb72e8db49a91367658`.

The controlled medians change −0.1%/−3.8%, within the observed process-pair variation.
This detects no material added production tick cost on these fixtures and is not a speedup
claim. The fixture is one long road, so these results do not establish routing-heavy whole-city
or rendering-FPS acceptance. Earlier ROAD-69 short-test timing remains separately documented.

```sh
METRUM_DEBUG=0 RAYON_NUM_THREADS=8 "$TEST_BINARY" \
  simulation::economy::agents::tick::contract::benchmark_agent_tick_contract \
  --exact --ignored --nocapture --test-threads=1
METRUM_DEBUG=0 RAYON_NUM_THREADS=8 CRITERION_HOME="$ARTIFACTS/criterion" \
  "$BENCH_BINARY" --bench 'AgentSystem::tick/on_road/(100000|1000000)$' \
  --noplot --save-baseline roads-bs-inherited-pair
METRUM_DEBUG=0 RAYON_NUM_THREADS=8 CRITERION_HOME="$ARTIFACTS/application-criterion" \
  taskpolicy -a "$BENCH_BINARY" --bench 'AgentSystem::tick/on_road/(100000|1000000)$' \
  --noplot --save-baseline roads-bs-application-pair
```

### Shared lane occupancy contract (TRAFFIC-02)

All live lane acquisitions use `agents/lane_state.rs`: building egress and ingress rollback,
network entry, connector entry/exit, direct continuations, lane changes, road-edit reattachment,
building-origin recovery, and debug spawning. The shared writer validates the lane ID and travel
mode, finite nonnegative distance, and connector junction ownership before publishing lane,
edge, node, distance, and transit state together. Routing, access legality, and traffic claims
remain the responsibility of the requesting subsystem. Leaving a lane and clearing/replanning
a route remain separate operations.

The contract costs O(1) per acquisition and allocates nothing on success. It uses safe disjoint
field references, including at the existing parallel movement boundary; it adds no unsafe blocks.
A violated runtime invariant panics at the caller with agent/mode, previous occupancy, requested
lane and network revision. It does not silently change travel mode or relocate the agent.
`./run.sh --release` enables the existing crash diagnostics by default, so a rejected transition
is captured with its caller and the simulation flight recorder under `logs/` (see
[`debug.md`](debug.md)). This is a fail-fast invariant, including in release builds.
Save and load share the mode/owner checks. A rejected save retains the previous file through the
existing transactional replacement mechanism.

The `zoned-city-26-days.sqlite` investigation found 14 walkers assigned to vehicle lanes; lane 61
held five walkers ahead of nine cars. Their pedestrian destination plans and saved positions
confirm inconsistent runtime occupancy. Nearby road additions are a possible trigger, but the
original corrupting transition has not been reproduced. This contract refactor does not repair
that already-invalid snapshot or establish the historical root cause.

Performance acceptance (2026-10-14): matched unprofiled release executables, baseline
`abb37aa5` (SHA-256 `eabe4dc799995762e66b84552deb5865133b31fc19d59b8f5dee797da221136d`)
and the contract working tree (`11e08f2a3a59097cca794527c1741edbcbfa2451b6d8d0d53ba0f2678167c3a3`),
each run as `RAYON_NUM_THREADS=8 taskset -c 0,2,4,6,8,10,12,14 "$TEST_BINARY" <test> --exact
--ignored --nocapture --test-threads=1` in alternating order with nothing else running.

- `benchmark_lane_transitions` (five pairs): transition counts match in every scenario. At
  65,536 groups the median deltas are clear +1.3%, merge +1.6%, blocked +1.4%, cruise +2.9% and
  lateral −4.0% (at most +0.08 ms); smaller tiers stay within pair-to-pair noise.
- `benchmark_agent_tick_contract` (`PERF-06`, 25 pairs): work counts are identical. Medians at
  10k/30k: network 0.228→0.237/0.494→0.494 ms, egress 0.478→0.471/1.415→1.381 ms; ingress
  (0.230→0.265/0.443→0.361 ms) has overlapping interquartile ranges in both directions.
- `python3 tools/perf_contract.py --binary "$TEST_BINARY" --artifacts "$ARTIFACTS"` on both
  executables reports the same pre-existing `PERF-03`–`PERF-05` budget and scaling failures,
  with every non-timing field identical. Daily ticks are 6.98→6.62/34.45→34.56 ms
  (baseline) and 22.01→21.72/73.88→70.96 ms (fiscal stress) at 10k/30k.

The shared contract therefore adds no measurable cost beyond about 2% on transition-dominated
microbenchmarks and leaves simulation work unchanged.

Save format 58 stores one identity row per live lane: road edge, direction and local lane index,
or junction node plus incoming/outgoing road lanes. Current and planned agent references resolve
through these identities after graph compaction and lane rebuilds. Tombstoned lanes are excluded.
Loading rebuilds final visible-surface lane heights before restoring distances and references,
and validates each current lane against its owner and travel mode. A live session lifts lanes the
same way without waiting for the renderer: rebuilds only append lanes, so `LaneSystem` keeps a
watermark of lanes already on the visible surface, and `TransitNetwork::sync_new_lane_heights`
lifts the lanes past it at the start of every simulation frame and after each road mesh
precompute (O(new lane points); a frame with nothing pending returns at once). Before this, a lane
rebuilt outside a drawn road chunk, such as a crosswalk connector rebuilt by a building edit, kept
its flat build heights and so a shorter length until the next load, and a session continued
differently after save and reload. Pedestrian lanes remain active
through load, including final frontage legs with an exhausted graph-node path.
An empty network path can also be a pending immediate replan after demolition. With valid
node, edge and destination references, loading preserves its planned target and path cursor
even when retry time and failure count are zero. An agent in network transit at a valid node
stays there and replans from the node whatever its mode, as the running game does: a road edit
that rebuilds the junction lane a walker is crossing leaves it at the node with no edge (found in
`SIM-02-I` part 3, when loading sent such walkers straight into their destination). Delayed
retries retain their backoff; agents without a usable destination or pending retry still
receive orphan repair.

Version 57 saves recover sidewalk direction from the saved road/node and connector identity from
the saved junction, outgoing route and authoritative position (including the pedestrian offset).
Recovery only inspects that edge or junction's indexed lanes and rejects unmatched connector
poses. Equivalent matching route prefixes use stable lane order. New saves use explicit identities.
This work occurs only during save/load; current-lane pose sampling remains allocation-free O(log P)
in the lane's point count, and agent restoration uses local owner lookups rather than city-wide scans.

The current traffic model is intentionally local and deterministic. Cars are independent agents
travelling along lane centerlines with per-lane occupancy buckets used for local gap checks.
There is no global microscopic traffic assignment, signal timing, parking search, multi-car
negotiation, or stochastic driver personality model yet.

Supported live behavior:

- car following with simplified IDM
- road speed limits
- junction connector lanes with tangent circular turns and cubic lateral transitions
- curve-based junction turn speed caps
- acceleration and braking limits
- connector entry spacing so multiple cars can use a junction when safely separated
- planned same-edge lane changes for reaching the correct destination frontage lane
- conservative same-edge overtaking and return-to-cruise-lane behavior
- render-side position and rotation smoothing by stable car render ID
- `./run.sh --debug traffic` logging and visual traffic debug overlays

## Core State

Traffic state is stored in the agent SoA. The traffic-owned continuation fields are:

- `current_lane_id`: the lane currently governing longitudinal movement
- `lane_distance`: metres travelled along `current_lane_id`
- `speed`: current car speed in m/s
- `lane_change_from_lane_id`: source lane for an active S-curve lane change, or `u32::MAX`
- `lane_change_start_d`: lane distance where the active lane change started
- `lane_change_length_m`: longitudinal length of the active lane-change S-curve
- `overtake_blocked_time_s`: time spent held below free-flow speed by lane traffic
- `overtake_cooldown_s`: cooldown before another discretionary overtake / return lane change

Save format 75 preserves speed, `lane_change_*` and `overtake_*`: they affect occupied lanes,
arrival times and later transactions. Active lane-change origins are remapped through saved lane
identities. New benchmark agents may still start with inactive defaults.

## Lane Buckets

The buckets hold `(lane_distance, agent_index)` per lane, sorted by distance with an index
tiebreak. The tick keeps them across ticks: the end of each tick fills them from every live lane
agent on the shared pool (agents grouped by lane range in one pass over 4,096-agent chunks, then
each lane range fills, sorts and overlap-corrects its own lanes in agent order), appended agents
join at the next tick start, and swap-removes patch them in place (`SIM-02-K` part 4,
[simulation.md](simulation.md#sim-02-k-part-4--agent-tick-2026-10-20-first-commit)). A bucket is
sorted again after an overlap clamp, so the snapshot is always the function of the agents' state
that a rebuild gives. These buckets are the hot-path structure for:

- nearest-car-ahead gap lookup
- connector entry slot checks
- lane-change target gap checks
- post-movement overlap correction

The bucket fill reuses retained scratch buffers and allocates nothing per agent once they have
grown. This is required for the 1M-agent scale target; traffic logic must not allocate per agent
in the tick hot path.

Active lane-changing cars are inserted into their target lane bucket and, before movement, also
into their source lane bucket while the S-curve is still active. This lets cars in both lanes react
to the crossing car during the IDM speed pass. Post-movement overlap correction clamps the current
authoritative lane.

Removing the last agent also retires dirty lane occupancy before the
empty-agent tick returns (`AUDIT-01-A5`). Cleanup reuses the existing dirty-lane list and
buffers. Once these are empty, later empty ticks skip the occupancy rebuild. The separate
low-frequency frontage-delay cache continues its existing decay from live agent speeds. Its
once-a-simulated-second update notes each agent's lane in parallel, then sums each lane range's
cars in agent order on the pool, so the delays are bit-identical for any worker count
(`SIM-02-K` part 5).
Save format 75 preserves that smoothed delay and its fractional update clock, restoring lane
values through saved lane identities; see [frontage delay persistence](entrance_and_exit.md#exact-frontage-congestion-penalty-cache).

Claim preparation classifies each agent independently into the retained byte per agent
(`AUDIT-01-A4`, `AUDIT-01-A12`). Large collections use contiguous 4,096-agent Rayon batches; smaller collections
and single-worker execution use the same direct loop. Lane-change selection is shared with
movement (`AUDIT-01-A12`): it inspects the current edge's lanes and sorted local occupancy, with
O(D + log K) work for D local lanes and K occupants. The byte distinguishes ordinary parallel
movement, fixed lateral candidates and dynamic handoffs, so ordinary movement skips a second
lane-change decision. There are no per-agent allocations or additional per-agent buffers.

A lateral move that cannot reach another lane end or frontage handoff this tick reserves its
target in the existing per-lane array. Atomic minimum selects the lowest agent index independently
of worker scheduling; these cars then move in parallel. Dynamic handoffs execute afterward in
stable agent-index order and use the remaining unreserved lanes. Fixed lateral reservations
therefore have priority over connector/frontage arrivals for that tick. Reusing one's own
reservation is allowed, including entry followed by a frontage detach.

Each lane stores one owner index (`usize::MAX` when free), replacing the former boolean. On a
64-bit target this adds seven bytes per retained lane; reset remains O(L) in retained lanes, in
parallel 4,096-lane chunks from 65,536 lanes.
Connector choices retain their existing `stable_index` algorithm and seeds.

## Car Following

Cars use a simplified Intelligent Driver Model in `idm_new_speed()`:

- free-flow speed is the current road edge speed limit, or the connector turn speed in junctions
- gap is bumper-to-bumper distance to the next car in the lane bucket
- the full approach-speed IDM term is not implemented yet because per-agent lead speed is not
  tracked

Current constants:

- `IDM_A_MAX = 6.0 m/s^2`
- `IDM_B = 6.0 m/s^2`
- `IDM_T_HEAD = 0.5 s`
- `IDM_S_MIN = 0.1 m`
- `CAR_LENGTH = 2.6 m`

After IDM proposes a target speed, `limit_speed_change()` clamps the change by acceleration and
comfortable braking. This prevents cars from snapping instantly between stopped and free-flow
speed.

The speed pass writes each result directly to that agent's SoA slot (`AUDIT-01-A7`). This is
independent because the shared lane snapshot contains distances and the current IDM model reads
no other agent's speed. The duplicate speed array and its final serial copy are removed. Non-car
and inactive agents retain their current speed while their existing traffic-timer rules run.

If a car is approaching a blocked connector or a required lane change whose target gap is blocked,
the speed pass computes a braking speed for the remaining distance. This makes the car slow before
the conflict instead of teleporting or stopping only at the exact handoff point.

## Road Speeds

The current urban road presets use `50 km/h`, stored as:

- `DEFAULT_URBAN_ROAD_SPEED_MS = 13.89 m/s`

The simulation stores speeds in m/s. `KMH_TO_MPS = 1 / 3.6`.

Normal road bends do not currently receive a curvature speed cap. A car on a curved road lane
keeps the road speed limit unless traffic or upcoming connector / lane-change constraints slow it.

## Junction Connectors

Vehicle junction movement uses explicit connector lanes generated in
`network/lanes/vehicle_junctions.rs`.

For every allowed inbound-to-outbound vehicle lane pair at a node, the lane system creates a
connection lane with:

- `edge_id = usize::MAX`
- `lane_type = LaneType::Vehicle`
- `node_id = junction node`
- `next_lanes = [target road lane]`
- cubic Bezier geometry from the inbound lane neck to the outbound lane neck
- at most `1 m` chord sampling, clamped to `8..64` steps

At true junctions (three or more vehicle arms) and user-authored vehicle connection nodes, connector control
distance and sampling use the junction footprint span: the larger of the lane-mouth chord, the two
mouth distances through the node center, and the widest incident roadbed span. This prevents
two-lane to four-lane T-junctions and explicit connection nodes from degenerating into tiny lateral
connector shims that cars traverse almost instantly.

Allowed turns are derived from `Node::lane_connections`:

- if the node has no user vehicle connections, all non-U-turn outbound lanes are open
- if the node has any user vehicle connection, the node is in whitelist mode and unspecified turns
  are blocked
- terminal nodes may connect back to the same edge

Strict degree-two corridor splits may skip materializing a vehicle connector and directly link the
aligned incoming physical lane to the outgoing physical lane. This direct pass-through is allowed
only when the node has exactly two incident vehicle arms, no user-authored vehicle lane connections,
their XZ tangents are near-collinear, and the lane mouths are coincident or overlap (on the inside of
a slight bend the outgoing lane starts behind the incoming lane's end, so any connector would run
backwards).

The pass-through degree here counts only arms with vehicle lanes (`TRAFFIC-03`), so a footpath
joining a road leaves it a pass-through for cars. The U-turn rule is the shared
`open_node_allows_u_turn` (`NET-07`), which counts every arm: a road ending in a footpath is not a
dead end, and cars do not turn around there. Counting the footpath once turned such a node into a three-arm junction: the road's
coincident lane ends got a junction-span Bézier connector that ran about 3 m backwards, so cars
turned around for a moment. Saves holding such a connector still load: a saved connector whose
incoming lane now links straight to its outgoing lane is dropped, and agents on it replan. True junctions and all
user-authored vehicle connection nodes must keep explicit connector lanes so junction speed,
spacing, and whitelist semantics still apply.

Cars enter connector lanes through `TRANSIT_INTERSECTION`. Connector lanes are lane-bucketed like
road lanes, so multiple cars may occupy the same connector when they respect separation. Connector
entry also uses a per-tick claim to avoid two cars grabbing the same zero-distance entry slot.
Car exits check outgoing-lane occupancy and acquire the same reservation (`AUDIT-01-A11`). A
blocked car waits at the connector end with its route intact until the outgoing entry clears.

## Junction Speeds

Junction movement uses a cap separate from the parent road speed:

- `CAR_JUNCTION_SPEED_MS = 6.0 m/s`
- `CAR_JUNCTION_LATERAL_ACCEL_MS2 = 2.2 m/s^2`
- `CAR_JUNCTION_MIN_SPEED_MS = 2.0 m/s`

`connector_turn_speed()` estimates turn severity from the angle between the connector's first and
last non-degenerate tangents. For straight or very gentle connectors, the cap is
`CAR_JUNCTION_SPEED_MS`. For tighter connectors, approximate radius is:

```text
radius = connector_length / turn_angle_radians
```

The turn speed cap is:

```text
sqrt(CAR_JUNCTION_LATERAL_ACCEL_MS2 * radius)
```

clamped to `CAR_JUNCTION_MIN_SPEED_MS..CAR_JUNCTION_SPEED_MS`.

`CAR_JUNCTION_MIN_SPEED_MS` is the lower bound for the curvature-derived cap. It is not a rule that
boosts a stopped or slow car up to crawl speed when entering a connector.

Road-lane speed prediction looks ahead to the next connector and brakes early if the connector is
tight or blocked. Once in `TRANSIT_INTERSECTION`, the car remains capped by the connector's
curvature speed until it exits to the target road lane.

## Planned Lane Changes

Planned lane changes serve both the next junction movement (`ROAD-67`) and the exact
`planned_detach_lane_id` for building access. Junction lane use and target pairing are owned by
[`roads.md`](roads.md#default-junction-lane-use--road-67-slice-b-stages-12).

Turn preparation starts within `70 m × (directional lane count − 1) + 30 m` of the road-lane end.
The car reads its next path edge (or destination approach edge), finds the nearest same-direction
sibling with an actual connector/direct continuation to it, and moves one adjacent lane at a
time. Ties use centre-relative lane index then lane ID. The current lane wins if already legal,
preventing discretionary return/overtake from undoing the preparation. Frontage preparation has
priority on the destination road. Whitelists use the same actual-connector query.

Turn changes use the existing speed-scaled S-curve, gap tests and deterministic lane reservation.
The remaining approach distance is divided over the still-required adjacent steps, capping each
maneuver while retaining its 18 m minimum and the existing 12 m road-end buffer. There is no extra
per-agent state or allocation. Both claim classification and movement call the same selector.
Per-tick additional work is O(A × (D + L × C)) in the preparation window, where D is the
local node degree, L the same-direction lane count and C its successor count. A rebuild-derived per-lane byte skips
approaches without lane-dependent movements; outside the window an O(1) guard also returns
before reading path/access columns. No route query or graph-wide lookup occurs during ordinary
lane choice.

A blocked turn-lane gap suppresses discretionary changes but does not stop the car indefinitely.
At the road end, if a sibling can make the desired turn but this lane cannot, the car takes its
first legal onward movement in stable successor order. It keeps its physical connector and
replans on the outgoing road before its next junction. This reuses the ordinary destination
planner, and does not jump sideways, reverse on the approach, or relax a turn restriction.
Freight border trips use the same outgoing-road replan boundary; debug cars without a destination stop
at the end of the diversion road. A directed network without a legal recovery loop can still
lose reachability after a missed turn. Genuine missing turns retain the existing failure report.

A destination turn may enter a different same-direction lane on its road, then use frontage lane
changes to reach the exact detach lane. Inspector route prediction includes these preparation
moves and the same destination-edge connector fallback; occupied gaps can still change the
future route and trigger a redraw.

Ownership split:

- [`entrance_and_exit.md`](entrance_and_exit.md) owns choosing `planned_detach_lane_id` and
  `planned_detach_lane_d`
- this document owns how the car physically changes lanes once that plan requires it

Rules:

- only cars in `TRANSIT_NETWORK` may start a planned lane change
- source and target lanes must be same-edge, same-direction vehicle lanes
- movement advances one adjacent lane at a time toward the final planned detach lane
- the target lane must have a speed-scaled safe gap at the current distance
- the car must own the target lane's reservation before starting the maneuver
- clear target lane means no intentional speed penalty
- blocked target lane is traffic and may force braking before the detach point
- planned lane changes take priority over discretionary overtaking

Lane-change length is speed-scaled:

```text
length = speed * 3.5 s
```

clamped to:

- minimum `18 m`
- maximum `70 m`

The car's authoritative longitudinal lane becomes the target lane when the maneuver starts, while
rendering blends from the source lane to the target lane over the S-curve. This keeps traffic gap
checks and destination detach checks tied to the lane the car is entering, without creating a
temporary lane object per car.

ROAD-67 stages 3–5 use [split transition nodes](roads.md#split-transition-nodes-and-turn-pockets--road-67-slice-b-stages-35).
Pocket lanes retain node-to-node identities and ordinary lane-local arc distances. The wider
road's lane mouths sit at the full-width end of the taper; fork/merge connectors traverse the
taper through existing junction gap checks and reservations. Transition nodes suppress zebras
while retaining pedestrian continuity. This adds no agent state or per-tick allocation.
The final 20 m of a movement-selective approach has solid divider paint. The existing cached
approach flag bounds every planned/frontage/discretionary maneuver before that interval; no new
agent fields or allocations are needed, and the added tick work is O(1). A blocked car continues
legally and replans as described above. Rebuilds can reattach a car directly onto a new taper
connector at its preserved position. Resize/removal/undo reproject moving lane stations, and
save version 90 persists the sparse taper records while lane identities remain derived.
H edits pockets, J/K toggle sides, U/I adjust body length and Shift+U/I taper length; both off
removes it. See the linked road spec for constraints and preview publication.

ROAD-67 connector trajectories are constructed at lane rebuild time. A single turn uses
the largest circular fillet fitting the entry/exit tangent intersection, with straight tangent
leads for unequal mouth distances. Curvature stays zero or one signed reciprocal radius;
it cannot reverse or spike above that required turn radius.
Straight-through and parallel lane gain/drop paths use their actual mouth span, retaining the
necessary lateral S without junction-wide overshoot. Continuous divided bends use tangent
straight leads and one circular arc, with radius limited by lane-to-median clearance. They
retain the median corridor without the old offset frame's endpoint/midpoint heading breaks.
This adds O(1) construction work per connector and no tick-time work, agent state or allocation.
Vehicle pose sampling remains unchanged; the driven lane geometry owns the correction.
Explicitly connected straight splits inset only the physical vehicle lane mouths, retaining
a real connector span instead of a backtracking loop at coincident endpoints. Insets are
bounded by the remaining edge length; graph clips, roadbed and sidewalk geometry stay intact.

### ROAD-67 trajectory follow-up acceptance (2026-10-20)

Fresh default and release suites pass all 2,583 library tests; the default whole suite also
passes all four growth integrations. Sixteen release headless road/traffic bridges and the
windowed trajectory/traffic fixture pass. Sampled connectors reject reverse signed curvature
for single turns, bound curvature by the available tangent-circle radius (including median
clearance), and check endpoint tangency. Coverage includes divided/undivided T and cross
junctions, skewed arms, 1–4 lanes, Prisma left/right pockets, lane gains/drops and 15–90°
divided bends. Explicit straight whitelists must advance without a backtracking loop.

Matched PERF-06 compares the completed stages 3–5 release with this follow-up, using 15
alternating process pairs and eight Rayon workers. All six workloads have matching work counters across builds;
30k agent work remains exactly 3× 10k with fixed lane work. Every paired bootstrap interval
includes zero: no tick regression is detected. These noisy single-tick measurements establish
neither a fixed equivalence bound nor a speedup. Build identities, commands and road/locality
results are in the [road acceptance record](roads.md#split-transition-nodes-and-turn-pockets--road-67-slice-b-stages-35).

| Mode | Agents | Before → after median, ms | Paired median change | Paired bootstrap 95% |
| --- | --- | --- | --- | --- |
| network | 10,000 | 0.499458 → 0.498083 | +2.04% | -5.38% … +6.28% |
| network | 30,000 | 0.970750 → 0.927333 | +0.94% | -7.84% … +8.21% |
| access_egress | 10,000 | 0.646500 → 0.641000 | +2.71% | -6.51% … +4.40% |
| access_egress | 30,000 | 1.592000 → 1.575167 | +0.14% | -2.27% … +4.24% |
| access_ingress | 10,000 | 0.352291 → 0.307708 | -9.41% | -17.34% … +6.43% |
| access_ingress | 30,000 | 0.604625 → 0.556166 | +3.40% | -24.49% … +28.79% |

The stages 3–5 acceptance below describes the earlier, pre-follow-up build.

### ROAD-67 stages 3–5 acceptance (2026-10-19)

Matched unprofiled release runs compare `6ec0b858` with the completed stages 3–5 build on
Apple M2 Pro (12 logical CPUs), Rust 1.98.1, MacOSX26.5 SDK, release `opt-level=3`, `debug=1`.
Fifteen alternating process pairs use eight Rayon workers and identical workloads/counters.
All paired bootstrap intervals include zero; no agent-tick regression is detected. The noisy
single-tick contract does not establish equivalence at a fixed percentage bound or a speedup.

| Mode | Agents | Before → after median, ms | Paired median change | Paired bootstrap 95% |
| --- | --- | --- | --- | --- |
| network | 10,000 | 0.484250 → 0.465709 | -1.44% | -7.01% … +1.26% |
| network | 30,000 | 0.907625 → 0.906542 | -0.63% | -4.25% … +11.95% |
| access_egress | 10,000 | 0.665833 → 0.650125 | +0.16% | -3.69% … +5.19% |
| access_egress | 30,000 | 1.580125 → 1.565625 | -1.76% | -4.17% … +2.80% |
| access_ingress | 10,000 | 0.388417 → 0.330250 | -9.38% | -26.36% … +7.77% |
| access_ingress | 30,000 | 0.614958 → 0.610000 | +0.40% | -8.52% … +20.22% |

Test executable SHA-256: before
`4bc39d10fbe01c7da5f8945dc2a781c4dc9144979a320cd4e99fcc66b914971f`, after
`d3474f6396dc614f1c6671768d1667911ff1761529c7c7faff7ba15f722dc85f`.
Exact command:

```bash
METRUM_DEBUG=0 RAYON_NUM_THREADS=8 "$TEST_BINARY" \
  simulation::economy::agents::tick::contract::benchmark_agent_tick_contract \
  --exact --ignored --nocapture --test-threads=1
```

Three matched pairs also cover lane transitions, claim preparation/reset, incremental lane
rebuilds and lane reattachment. At 65,536 transition groups, clear/merge/blocked medians are
4.805/5.923/5.217 → 4.550/5.536/4.637 ms; lateral is 4.740 → 4.850 ms and cruise
3.233 → 3.226 ms. At 1,048,576 rows, idle/mixed claims are 0.256776/0.620568 →
0.256755/0.640193 ms; lane reset is 0.301536 → 0.304688 ms. Counts/checksums match.
Fresh default/release suites pass 2,579 library tests; all four growth integration tests pass.
Road/traffic bridges, real windowed traffic, connector save/load and agent-safe edit/undo checks
pass. The [road acceptance record](roads.md#split-transition-nodes-and-turn-pockets--road-67-slice-b-stages-35)
owns complete commands, worker settings, auxiliary/road/locality tables, library identities,
legacy geometry proof and capture comparisons. The following section retains stages 1–2 evidence.

### ROAD-67 stages 1–2 acceptance (2026-10-19)

Fresh matched, unprofiled release runs on Apple M2 Pro (12 logical CPUs), Rust 1.98.1,
`opt-level=3`, `debug=1`, MacOSX26.5 SDK; inherited macOS scheduling. Baseline is `ff3f9882`;
no builds, other tests or game captures ran during measurement. Test executable SHA-256:

- baseline: `b8befab53b6b34332e4b66b8904f4c02526313ae01dd919ccf7563640c1bacd0`
- candidate: `4bc39d10fbe01c7da5f8945dc2a781c4dc9144979a320cd4e99fcc66b914971f`

An 11-pair screen had unstable ingress samples, so the final tick comparison uses 51 alternating
process pairs and eight Rayon workers. The table reports medians of process samples; every paired
bootstrap 95% interval includes zero. It establishes no tick regression beyond noise and no
speedup claim. All deterministic work counts match; exercised agent rows grow exactly 3× and
lane rows stay at 16.

| Tick fixture | Agents | Before (ms) | After (ms) | Paired median change, 95% interval |
| --- | ---: | ---: | ---: | --- |
| Network | 10,000 | 0.476500 | 0.487084 | +1.12% [-2.12%, +4.35%] |
| Network | 30,000 | 0.902000 | 0.903625 | -0.86% [-6.23%, +7.22%] |
| Access egress | 10,000 | 0.643209 | 0.645042 | +1.53% [-2.31%, +3.15%] |
| Access egress | 30,000 | 1.568250 | 1.557667 | -0.22% [-1.62%, +1.11%] |
| Access ingress | 10,000 | 0.347042 | 0.342792 | -6.84% [-10.99%, +1.81%] |
| Access ingress | 30,000 | 0.596209 | 0.589458 | -2.43% [-6.25%, +11.64%] |

Five alternating transition pairs use four workers and identical transition counts. Largest tiers:

| Movement case | Agents | Before (ms) | After (ms) |
| --- | ---: | ---: | ---: |
| clear | 131,072 | 4.824958 | 4.678500 |
| merge | 131,072 | 5.986250 | 5.814000 |
| blocked | 196,608 | 4.966208 | 5.142459 |
| lateral | 196,608 | 4.623791 | 4.720750 |
| cruise | 196,608 | 3.089333 | 3.189083 |

The positive movement-only deltas are at most 0.177 ms at these large tiers; they are not claimed
as zero cost. Three matched claim/reset pairs (four workers) retain identical classification
checksums: at 1,048,576 agents idle is 0.256979→0.257073 ms and mixed 0.616021→0.619109 ms;
resetting 1,048,576 empty lanes is 0.300651→0.299719 ms.

The first implementation's 10–17% transition slowdown was rejected. The accepted build keeps
rare route planning and frontage detach geometry outside the small movement loop, keeps the
existing compact lane-change input, and gates the route query with the rebuild-derived lane byte.
No new per-agent column, allocation or spatial index is introduced.

Build each retained executable with `cargo test --release --lib --no-run`. The exact tick command
is `METRUM_DEBUG=0 RAYON_NUM_THREADS=8 "$TEST_BINARY" simulation::economy::agents::tick::contract::benchmark_agent_tick_contract --exact --ignored --nocapture --test-threads=1`.
The other commands use `METRUM_DEBUG=0 RAYON_NUM_THREADS=4 "$TEST_BINARY" <test> --exact --ignored --nocapture --test-threads=1`, with:

- `simulation::economy::agents::tick::movement_pass::tests::benchmark_lane_transitions`
- `simulation::economy::agents::tick::claims::tests::benchmark_claim_preparation`
- `simulation::economy::agents::tick::movement_pass::tests::benchmark_empty_lane_claim_reset`

Raw process rows, earlier rejected results and build identities remain local under `$ARTIFACTS`.
Road/locality measurements and the windowed inspection are recorded in [`roads.md`](roads.md#default-junction-lane-use--road-67-slice-b-stages-12).

## Conservative Overtaking

Overtaking is intentionally conservative and local. It exists to make multi-lane roads feel alive
without adding a full driver AI.

Rules:

- only cars in `TRANSIT_NETWORK`
- only same-edge, same-direction sibling vehicle lanes
- never while a planned lane change is pending
- never while already lane-changing
- never near the current edge end
- never near a planned building detach point
- target lane must have a safe speed-scaled gap at the current distance
- target lane must have meaningfully more space ahead
- one adjacent lane at a time
- cooldown after each discretionary overtake / return

Current constants:

- stuck time before passing: `2.0 s`
- cooldown: `8.0 s`
- minimum speed-gain condition: `2.0 m/s`
- minimum gap gain over current lane: `12 m`
- target ahead gap for passing: `30 m`
- target ahead gap for returning: `20 m`
- edge-end buffer: `12 m`
- detach buffer: `25 m`

Lane direction convention:

- forward lanes use non-negative lane indices; lane `0` is closest to the road center
- backward lanes use negative lane indices; lane `-1` is closest to the road center
- overtaking moves toward the center lane
- return-to-cruise moves outward

This rule intentionally avoids weaving:

- a car must be traffic-blocked before passing
- a car returns only when the outward lane is clear and the overtake cooldown has expired
- planned destination-lane changes override overtaking

## Render Movement

Rust produces car transforms from lane poses in `nodes/sim/core/snapshot.rs`; the former direct
car/pedestrian exporters are removed. `nodes/sim/render/agents.rs` owns path debug overlays only.
The snapshot samples an active car lane-change curve only when source and destination lanes have
matching road, direction and lane type. Invalid/stale sources use the current lane pose. This
reuses the existing allocation-free lane sampler; it adds one bounded source-lane lookup and, for
active changes, source-lane sampling, not a scan over agents or roads. The snapshot regression
`car_snapshot_keeps_lane_change_curve_in_both_directions` checks start/midpoint/end positions,
sloped lane heights, lateral orientation, ownership flags and mismatched-source rejection.

The final audit's `lane_change_snapshot_benchmark` measures 100/1,000/10,000 cars on two sloped
lanes, with fixture setup excluded, ten warmups and 100 recycled snapshots per workload. Release
p50 for inactive versus all-active lane changes is 0.0316/0.0363, 0.1189/0.1507 and 0.8272/0.9872
ms respectively (`RAYON_NUM_THREADS=24`). This isolates the snapshot sampling cost; it is not a
gameplay frame-time benchmark.

Lane pose sampling in `nodes/sim/render/lane_pose.rs`:

- samples position by distance along lane geometry
- samples tangent using a `2 m` look-behind / look-ahead window
- falls back to the local segment tangent when the look window degenerates
- samples active lane-change S-curves by smoothstep blending source-lane and target-lane positions
- derives S-curve tangent from blended lane tangents plus the lateral blend derivative

Render-side car interpolation runs in Rust (`nodes/sim/render/car_visual.rs`, `RENDER-14`).
Each frame `godot/scripts/renderers/agents.gd` passes `get_car_render_data()` to
`smooth_car_render_data(car_data, delta)` and uploads the returned per-bucket buffers:

- cars have stable render IDs from Rust; drawn poses are main-thread presentation state on
  `SimulationNode`, never saved or read by the simulation
- pedestrian and vehicle MultiMeshes use the shared dynamic shadow-caster policy
- origin interpolation smooths per-tick position updates
- basis interpolation smooths rotation (Godot's `Basis.slerp` formula, reimplemented on `glam`;
  godot-rust's `Quaternion::slerp` is an engine call per car)
- large jumps above `80 m` snap instead of interpolating across the map
- a change between lane and off-lane height owners snaps instead of blending
- off-lane cars are grounded by the shared support solve after interpolation; when the core is
  busy they show their validated snapshot pose, never an unsupported interpolated one
- a car absent from one frame is forgotten; degenerate or non-finite snapshot poses pass through
  unchanged and are not remembered
- transform uploads happen every render frame so fast sim multipliers do not quantize cars to the
  simulation tick rate

Current render interpolation constants:

- `INTERPOLATION_RATE = 24.0`
- `ROTATION_INTERPOLATION_RATE = 18.0`
- `SNAP_DISTANCE_M = 80.0`

**Cost (`RENDER-14`, 2026-09-30).** Smoothing is O(visible cars) with one hash lookup and insert
per car. Buckets of at least 2,048 cars interpolate in parallel on the global Rayon pool, the pool
the support solve already used from the main thread. A populated-city run should confirm that the
main thread does not wait on a pool the simulation tick is saturating.

Matched A/B, `godot/tests/vehicle_ground_support_test.gd -- --benchmark-vehicle-support`: 100 timed
`update_swarm` frames with cars advancing every frame. Baseline is `HEAD` `eedd0094` GDScript;
both sides load the same release library (SHA-256 `92a4c47b…94841fda`), whose only change is the
new function. Runs were interleaved, two per side, headless and windowed 640×360, with
`RAYON_NUM_THREADS=4` on an i9-12900K / RX 7900 XTX. Windowed p50 ranges in milliseconds:

| Cars | Lane cars | Off-lane cars |
| --- | --- | --- |
| 1,000 | 1.22 → 0.074 | 1.87–1.96 → 0.60–0.63 |
| 5,000 | 6.06–6.12 → 0.34–0.49 | 8.74–9.28 → 2.23–2.37 |
| 20,000 | 24.9–25.3 → 1.19–1.25 | 34.8–35.3 → 8.22–8.26 |

Headless numbers agree (lane cars at 20,000: 24.6–24.8 → 0.82–0.88 ms). Off-lane cars keep the
unchanged native support solve, about 0.4 µs per car, which is now their dominant cost. Real
off-lane counts are limited to cars in an access phase. The Rust-only kernel
(`car_smoothing_frame_cost`, ignored test) measures 0.045 / 0.17 / 0.66 ms for 1,000 / 5,000 /
20,000 lane cars. After the runs, a clippy cleanup (`as_chunks` in place of `chunks_exact`, with no
behaviour change) produced the deployed `godot/bin` library `7a09021d…bb31156e`. The native tests
and the headless and windowed vehicle grounding test were rerun on it.

**Correctness checks:**
- Native `car_visual` tests cover:
  - a first sighting shows the snapshot pose;
  - origin and heading close their gaps at 24/s and 18/s, and the next frame starts from the drawn
    pose;
  - long jumps and height-owner changes snap;
  - absent cars are forgotten;
  - degenerate and non-finite poses pass through unrecorded;
  - a busy support solve restores only the off-lane snapshot poses.
- The 156 related release vehicle, car and render tests pass. Rustdoc reports no warnings. Clippy
  reports nothing new in the touched files.
- `vehicle_ground_support_test.gd` drives the production `agents.gd` update through the native
  call with real vehicle meshes, and passes headless and windowed. It checks 4,840 poses: no buried
  contact, windowed MultiMesh buffers equal to the drawn pose, and the height-owner snap within the
  snap distance.

### Vehicle models and lamps (`RENDER-26`)

Cars are the Kenney models (`godot/assets/models/vehicles/civilian/` sedan, sports sedan, SUV,
luxury SUV; `freight/` delivery van) with their colormap atlas variants. Generic Finnish models
were tried on this branch and withdrawn as not good enough to ship; they are kept off-tree. The
Kenney meshes carry no lamp data, so `vehicle_lamps.gd` holds a measured table of front and rear
lamp anchors per model (left lamps mirror X); procedural cars derive theirs from the AABB.

At night (sun below 4 to -2 degrees, as the street lights):

- `vehicle_lamps.gd` adds a camera-facing glow quad per
  lamp (`vehicle_lamp_glow.gdshader`) that keeps at least 1.6 pixels, so traffic shows across the
  map; headlights show from ahead, tail lights from behind;
- the 12 drawn cars nearest the camera within 100 m get a real `SpotLight3D` each
  (`vehicle_headlights.gd`, 60 m range, inverse-square decay, 35 degrees with a soft rim, no
  shadows; see below), chosen in Rust by
  `nearest_car_poses` (O(drawn cars · log 12) with a bounded heap). They light the road, trees,
  walls, walkers and the car ahead by their own colour;
- every car carries a light box (`vehicle_headlight_beam.gdshader`) that rebuilds each covered
  pixel from the depth buffer and adds a small red tail glow behind, and for cars from 70-100 m
  out (where the spot lights fade) the same cone the real headlight casts: Godot's SpotLight3D
  falloff with its 60 m range, inverse-square decay, 35° angle and rim attenuation 3, lit as level
  ground. This light is added blind to surface colour, acceptable only at that distance. Near the camera the box shrinks in the vertex shader
  to the tail region; by day and past 450 m it collapses and draws nothing.

Measured 2026-10-09 on the M2 Pro, `no-headlights` save (7 cars), release build, 120 frames,
old (Kenney models, no lamps) and new alternated three times: by day the new models cost under
0.1 ms. At 23:00, with real lights already on for the street lights: saved view +1.15 ms,
eye level +0.8 ms, 32 m overhead +0.4 ms with a 45 m spot range. Without the spot lights the
lamps cost 0.1-0.35 ms; the spot lights' cost follows their reach, not their count (6 or 12 the
same), and a 28 m range saved 0.45 ms in the saved view (about +0.85 ms over old in total), so
the range is 30 m. The 30 m setting itself was checked visually, not re-timed.

Near and far headlights did not match: up close the lighting shrank and dimmed. The real lights
were about 100 times weaker than the far light box on ordinary surfaces (their light is scaled by
albedo, the box's is added blind to it), fell off linearly instead of with the square of distance,
and reached 30 m against the box's 45. `tools/bake_headlight_projector.py` calibrates the
two against each other and bakes the box's own low-beam pattern into a spot-light projector
(`godot/assets/textures/vehicles/headlight_projector.png`). Godot 4.7 applies a projector only to
a shadowed light (`SpotLight3D` warns so; `storage_rd/light_storage.cpp` builds the projector
matrix only for shadowed lights), and four shadowed lights cost 1.9-2.9 ms a frame on the M2 Pro,
two 1.3-2.1 ms; hard shadows or an empty caster mask saved little, so the cost is per-pixel
shading. They are off (`SHADOWED := 0`), and every light is a plain cone: 60 m range,
inverse-square decay, 35 degrees with a steep rim falloff, energy 1200, matching the box's hot
spot near 20 m and overlighting the first few metres. Against the old 30 m lights (same save,
23:00, 120 frames, alternated twice): close view -0.1 ms, saved view +0.4 ms, overhead +0.15 ms.
Shots: `screenshots/2026-10-09-roads/headlights-4-shadowed.png` and
`headlights-shadowless-cone.png`.

Far and near headlights still did not match (2026-10-14): from 70 m out a car lit a wide white
block with straight edges, and close up a narrow soft cone. The box drew its own low-beam pattern,
still strong at the box's front face (45 m) and side faces (16 m), and lit each pixel with a
normal rebuilt from depth derivatives, which broke at every kerb and depth step into bright dashes
and straight-edged patches. The box now draws the real light's cone (above), lights everything as
level ground, and reaches 60 m ahead and 42 m (tan 35° × 60 m) to each side, where that cone is
spent; it fades out below the box's 8 m top. Its energy (62) and a ground tint (1.07, 1.0, 0.85)
were fitted to the mean light the real headlights add to the same views (`light-road` save at
01:00, cars 60 and 110 m away, all-real against all-box: 86-110 % per channel before the tint).
Frame cost was not measurable apart from the street-light change it shipped with (`ROAD-59`).

## Debugging

Run:

```bash
./run.sh --debug traffic
```

or:

```bash
./run.sh --debug-traffic
```

Traffic debug logging goes to stdout through `traffic_log!` on the `traffic` debug channel, each
line prefixed `[DEBUG:traffic]`. The channel can also be switched on in game from the debug
console; see [`debug.md`](debug.md).

Important log markers:

- `[JUNCTION_ENTER]`
- `[JUNCTION_BYPASS]`
- `[JUNCTION_EXIT]`
- `[JUNCTION_WAIT]`
- `[JUNCTION_MISSING_CONN]`
- `[JUNCTION_MISSING_EDGE]`
- `[JUNCTION_MISSING_EXIT]`
- `[LANE_CHANGE_START]`
- `[LANE_CHANGE_WAIT]`
- `[OVERTAKE_START]`
- `[OVERTAKE_RETURN]`
- `[ACCESS_EGRESS_ATTACH]`
- `[ACCESS_INGRESS_DETACH]`
- `[ACCESS_INGRESS_WAIT]`

When traffic debug is enabled, the agent path overlay (`P`) also shows each car's lane and
junction labels and the connector it will take. To see every connector at nearby junctions,
colored by whitelist, turn speed or occupancy, and to log cars stuck on a connector, use the
`junction_connectors` visual mode ([debug.md](debug.md#junction-connectors)).

## Performance Contract

Traffic code is a hot path. New traffic behavior must preserve these rules:

- no allocation inside per-agent tick loops
- reuse existing lane buckets and scratch buffers
- use `rayon` through existing dispatch helpers for independent agent work
- keep per-agent decisions O(1) or bounded by tiny per-edge lane counts
- do not add a new spatial structure when lane buckets, edge lanes, the road graph, or existing
  indices answer the query
- keep route planning and traffic movement separate; local traffic behavior must not trigger
  per-tick CCH pathfinding

### Unmarked pedestrian junction routes — ROAD-74

Zebra authoring no longer gates legal crossing connectivity. Ordinary unmarked junction arms
retain perpendicular pedestrian steps when the junction has no marked arm; marked arms remain
the nearby crossing choice. Unmarked sidewalks reach the solved mouth without zebra inset.
Coincident straight mouths use stationary connectors, and rebuild-only shortest-route trees
never reparent settled mouths through zero-length cycles. LaneCrosswalk carries crossing identity
without requiring paint. Ramp and transition continuity, continuous median barriers and NET-07
vehicle U-turn rules are unchanged. Zebra/turning-vehicle priority and signals remain the pending
TRAFFIC-08 stages; this slice changes route construction, not agent admission. See
[road marking rules](roads.md#authored-zebras-and-selective-lane-arrows--road-74--road-75).

### Short agent-tick contract (`PERF-06`)

Routine traffic changes use the ignored release test
`simulation::economy::agents::tick::contract::benchmark_agent_tick_contract`. It warms each isolated
lane fixture once, then records one setup-free timed tick for the 10k and 30k agent tiers. The
test emits one `PERF_AGENT` JSON line per mode and tier with deterministic row counts for lane
bucket preparation, speed updates, lane claims, movement, the end-of-tick lane fill and frontage-delay
aggregation. The fixture uses one long straight lane, fixed 16 m spacing and no route-planning
work; construction and buffer allocation stay outside the timed tick.

The bounded counts are the contract's first diagnostic. Every emitted phase count must grow by at
most `1.2 × (30k / 10k) = 3.6×`; a zero count at 10k must remain zero at 30k. A failed count is
investigated before any timing comparison.
The one timing sample per tier is a routine regression signal, not a speedup claim. Keep a
release baseline executable beside the current executable and report both build identities when
the contract runs. Matched alternating process pairs remain reserved for milestone comparisons
and claimed speedups. The short contract is run serially with no concurrent build or measurement;
the 30-minute Criterion `agent_benchmark` is no longer a routine gate.

ROAD-74/75 check (2026-10-22), versus queued predecessor `f70c6dbe`: fifteen alternating
unprofiled release process pairs/eight workers match all six work dictionaries and deterministic
scaling checks. Network medians at 10k/30k agents are 0.539/0.947 → 0.524/0.919 ms;
egress 0.656/1.584 → 0.638/1.600 ms; ingress 0.359/0.615 → 0.323/0.607 ms. These mixed short
measurements establish the contract check, not a tick speedup. Agent dispatch and tick-time
allocation policy are unchanged. Exact binary identities and commands are in
[marking-rule acceptance](roads.md#marking-rule-acceptance--2026-10-22).

ROAD-68 completion check (2026-10-22), versus queued predecessor `1a4ea348`: fifteen alternating
release process pairs/eight workers match all six work dictionaries and scaling bounds. All
short timing intervals include zero (10,000 paired bootstrap resamples, seed 68); opposite
10k/30k ingress shifts remain inconclusive. Three alternating sustained Criterion pairs give
100k ticks 2.8647 → 2.8982 ms (+1.17%) and 1M ticks 37.8215 → 38.2545 ms (+1.14%). All three
1M pairs are slightly higher, so this is recorded as a small measured increase, not dismissed
as noise. No agent dispatch, hot Edge fields or per-agent allocation policy changes in this
completion; structures live on compiled span owners. The earlier topology-stage comparison
against `cce4554e` had conflicting short cohorts whose cause remains unknown; a later
predecessor comparison cannot retrospectively resolve it. Commands, exact binary identities,
fixture-setup failures and completed cohorts are recorded in
[ROAD-68 performance acceptance](roads.md#completion-performance-and-perf-06-interpretation--2026-10-22).

Earlier road-chain measurements below used fixed contiguous agent ranges. With the upstream
integration, the dispatch keeps the same ownership guarantees while distributing those ranges:
Movement now owns route-query scratch by shards that each take every shards-th chunk of 256
agents (`SIM-02-K` part 4), retaining the existing
small-city serial threshold and separate ordered dynamic movement. Scoped TLS access restores
the previous scratch on return or unwind; this preserves planner entry points while making
memoization independent of which Rayon worker executes a range. Storage is prepared outside
agent loops and bounded by worker count and graph size. The 2026-10-14 owned-scratch build
(`c7ca3b0bf9514d00a0e89d5c7f02a16b47fc0489da9536418073c08de8a2d66a`) passes the short
agent contract: network 0.205/0.587 ms, egress 0.472/1.334 ms, ingress 0.404/0.427 ms at
10k/30k. Agent work is exactly 3× and lane work stays constant. These single samples are
routine validation, not speedup evidence. See `economy.md` for repeatable whole-day CCH counts.

Fresh infrastructure check (2026-10-13): the release test above passed with eight Rayon
workers and inherited CPU affinity, run serially as
`RAYON_NUM_THREADS=8 "$TEST_BINARY" simulation::economy::agents::tick::contract::benchmark_agent_tick_contract --exact --ignored --nocapture --test-threads=1`.
Executable SHA-256: `b660a61e02aa320874e141994a55a518dcdf78efd60569bdba60694dd111c84e`
(working tree based on `395b810f6575ac8c73eee0ff1cbc1d5c5b2b9ad3`).
Network ticks measured 0.417/1.077 ms at 10k/30k, access egress 0.556/1.368 ms,
and access ingress 0.211/0.316 ms. All exercised agent-row counts grew exactly 3×,
lane counts stayed at 16, and fresh-fixture counter repeats matched. The complete test
took 0.05 seconds including setup. These single samples validate the short runner and
work bounds; they establish no speedup against the retained baseline.

`PERF-06` acceptance (2026-10-14): the complete performance runner passes all six agent samples
and deterministic repeats with eight Rayon workers pinned to CPUs `0,2,4,6,8,10,12,14`.
Current executable SHA-256 is `06e8aa0a4a2899a89cff7ad78bcfea5a6c2eb32ee2bdabbd3f2c33449347074c`;
the retained baseline executable's verified SHA-256 is
`b34a0bedb12609c2b1fb8382d2e4f2cabc5e0102553fcd59fca46944fd88d546`.
Network ticks measure 0.242/0.507 ms, access egress 0.458/1.342 ms and ingress 0.143/0.313 ms
at 10k/30k. Every exercised agent count is exactly 3×; lane counts remain 16. The 36 targeted
agent-tick regression tests pass (eight manual tests ignored). These are routine contract
samples, not a measured speedup against the baseline.

Current known bounded scans:

- adjacent lane lookup scans the current edge's lane list, whose count is tiny for supported road
  presets
- connector selection scans `next_lanes` from the current lane, bounded by junction fan-out
- direct pass-through detection scans only the candidate `next_lanes` of the current lane, bounded
  by degree-two split fan-out
- lane bucket gap checks use sorted vectors and `partition_point`

### Claim Preparation Audit Measurements (2026-09-12)

Five alternating unprofiled release process pairs used eight physical cores (`taskset -c
0,2,4,6,8,10,12,14`, `RAYON_NUM_THREADS=8`); three pairs used CPU 0 and one worker. The isolated
SoA fixture includes idle agents, walking/car egress, normal road travel, imminent lane endings,
stopped cars, invalid lanes and junction travel. One lane is sufficient for these classification
branches; placement, route planning, fixture allocation and checksum validation are outside timing.
Each process warms up three times, then records 21 samples of eight preparations. Every output
flag matches across versions and worker counts.

| Agents | Eight-core idle before → after, ms | Eight-core mixed before → after, ms |
| --- | --- | --- |
| 1,024 | 0.000631 → 0.000693 | 0.001533 → 0.001574 |
| 16,384 | 0.009895 → 0.010659 | 0.025071 → 0.024581 |
| 131,072 | 0.080190 → 0.014952 | 0.200441 → 0.030323 |
| 1,048,576 | 0.648757 → 0.094635 | 1.621333 → 0.225566 |

| Agents | Single-core idle before → after, ms | Single-core mixed before → after, ms |
| --- | --- | --- |
| 1,024 | 0.000631 → 0.000694 | 0.001604 → 0.001548 |
| 16,384 | 0.009894 → 0.010864 | 0.024039 → 0.024992 |
| 131,072 | 0.080474 → 0.089581 | 0.199360 → 0.200038 |
| 1,048,576 | 0.648518 → 0.687894 | 1.607550 → 1.636986 |

The accepted tradeoff is a small serial cost: the largest single-worker idle case adds about
0.039 ms and the mixed case adds 0.029 ms per preparation. Small-city differences remain below
0.001 ms in these runs. These are classification timings, not full-frame or full-movement timings.
The existing lane-claim order, actual movement, legal detach checks and lane-gap rules are unchanged.

Run `BINARY --exact simulation::economy::agents::tick::claims::tests::benchmark_claim_preparation
--ignored --nocapture --test-threads=1` with the affinity/worker settings above. Both hot predicates
explicitly retain their original in-loop execution in the accepted build.

### Speed Update Audit Measurements (2026-09-12)

The isolated speed-phase fixture uses one straight road lane and one straight connector, with
fixed 16 m spacing and the final agent at least 1,000 m from the lane end. It exercises road cars,
a mix of road cars/walkers/inactive agents/junction cars, and an all-inactive case. Placement,
route planning, occupancy construction, allocation and result hashing are outside timing. Each
process performs three warmups and 21 samples of eight updates; the same fixed sequence produces
identical speed, blocked-time and cooldown bits across builds and worker counts.

Five alternating release process pairs use eight physical cores (`taskset -c
0,2,4,6,8,10,12,14`, `RAYON_NUM_THREADS=8`); three pairs use CPU 0 with one worker.

| Agents | Eight-core road before → after, ms | Eight-core mixed before → after, ms |
| --- | --- | --- |
| 1,024 | 0.013677 → 0.012956 | 0.012443 → 0.011602 |
| 16,384 | 0.081637 → 0.068452 | 0.055334 → 0.043820 |
| 131,072 | 0.613947 → 0.551550 | 0.335940 → 0.280316 |
| 1,048,576 | 5.971705 → 6.180697 | 3.360615 → 2.515615 |

At 1,048,576 agents, single-core road timing is 43.944670 → 46.577261 ms and mixed timing is
31.389801 → 24.839429 ms. The fully occupied single-lane stress case therefore has an accepted
3.5% eight-core / 6% single-core cost; the change is not a universal timing improvement. Idle
work retains comparable timings.

The change removes one retained `f32` per speed-buffer slot (4 MiB in the largest fixture) and an
O(A) serial copy. Existing per-agent gap lookup remains O(log K) in that lane's occupancy, with
bounded per-edge lane queries. This fixture intentionally concentrates traffic in very long
lanes; it measures the speed phase rather than gameplay frame time or ordinary city density.

Run `BINARY --exact simulation::economy::agents::tick::speed::tests::benchmark_speed_update
--ignored --nocapture --test-threads=1` with the settings above. The regression
independently reverses agent storage and runs with one/eight workers, comparing every speed
and timer bit after six updates.

### Lane Reservation Audit Measurements (2026-09-12)

`AUDIT-01-A11/A12` closes collisions at connector exits and lateral lane changes. The movement
fixture isolates dispatch, including claim preparation, with 512, 8,192 and 65,536 independent
lane groups. Each connector group has two 10 m connectors entering one 100 m road lane; cases
cover an available exit, competing exits and a stationary blocker. Lateral groups have three
100 m road lanes, competing return/overtake maneuvers into the middle lane and a stopped leader.
Cruising groups have three cars 20 m apart with no maneuver. Cars move at 4 m/s in 0.25 s steps.
Fixture creation, occupancy construction and resets are outside timing. Each process uses three
warmups and 21 single-step samples; no road-end route query occurs in this minimal lane fixture.

Five alternating unprofiled release pairs use eight physical cores (`taskset -c
0,2,4,6,8,10,12,14`, `RAYON_NUM_THREADS=8`); three pairs use CPU 0 and one worker. Source and
binaries remain fixed, with no concurrent builds or other benchmark jobs. Median process medians:

| Workload | Agents or retained lanes | Eight-core before → after, ms | Single-core before → after, ms |
| --- | --- | --- | --- |
| Clear connector exits | 131,072 agents | 4.176683 → 4.231970 | 7.400009 → 7.498916 |
| Competing connector exits | 131,072 agents | 5.973477 → 5.260708 | 7.990074 → 7.279029 |
| Blocked connector exits | 196,608 agents | 6.598414 → 6.290669 | 10.158671 → 9.745280 |
| Competing lateral moves | 196,608 agents | 2.500647 → 3.442970 | 13.851123 → 19.403899 |
| Cruising | 1,536 agents | 0.026580 → 0.055582 | 0.051212 → 0.066385 |
| Cruising | 24,576 agents | 0.216818 → 0.237956 | 0.834715 → 1.090162 |
| Cruising | 196,608 agents | 2.064773 → 2.301867 | 11.618839 → 13.034764 |
| Idle classification | 1,048,576 agents | 0.093443 → 0.078447 | 0.633645 → 0.558090 |
| Mixed classification | 1,048,576 agents | 0.234366 → 0.224720 | 1.670352 → 1.637971 |
| Empty occupancy/claim reset | 1,048,576 lanes | 0.122143 → 0.188869 | 0.122626 → 0.202618 |

The corrected contested cases intentionally admit fewer cars: one winner instead of two, or
zero when blocked. Those timing differences are not equivalent-output throughput improvements.
The runner verifies the expected transition counts separately for each build. Classification
checksums match across builds and worker counts; that fixture has no eligible lateral moves.
Classification/reset timings use 21 samples of eight preparations after three warmups. The
empty-agent reset fixture warms all buffers and isolates the wider owner array; reset is serial
under both worker settings.

The accepted eight-core cost is about 0.24 ms for 196,608 cruising agents and 0.94 ms for the
extreme lateral case, which attempts 131,072 lane changes at once. Small cruising batches show
a 0.029 ms increase; the 24,576-agent single-core case costs 30.6% more. These are movement-phase
measurements, not full-city frame timings. The retained classification buffer stays one byte
per agent. On 64-bit targets, replacing a boolean claim with an owner adds seven bytes per
retained lane (7 MiB at 1,048,576 lanes), with O(L) reset work and no per-agent allocation.

The first correct implementation serialized all lane changes and increased the eight-core
lateral case from 2.601286 to 13.154782 ms; it was rejected. The accepted version reserves fixed
lateral targets in parallel using minimum agent ID, then executes those moves in parallel.
Three classification modes in the existing byte avoid repeating the decision for ordinary
movement. Dynamic handoffs retain stable serial order.

The measured ignored tests are
`tick::movement_pass::tests::benchmark_lane_transitions`,
`tick::claims::tests::benchmark_claim_preparation`, and
`tick::movement_pass::tests::benchmark_empty_lane_claim_reset` under
`simulation::economy::agents`, with `METRUM_DEBUG=0` and the worker settings above. Rust is
1.98.1 (`48a229cea`, 2026-09-01). Baseline binary SHA-256
`ed8ee8df8150244d3637cbebbf74318d1e0676a887e6c2e36ab17df7f280e138`; accepted binary SHA-256
`71402a5b38110a6ce1b15ff553e012d0d9814dc52530fe9a0ba292cac6ca18fc`.

## Known Limits

- normal road bends do not yet use curvature speed limits
- full IDM approach-speed interaction is not implemented because lead-vehicle speed is not tracked
- overtaking has no driver personality, urgency, emergency behavior, or multi-car prediction
- automatic vehicle priority/yield/stop control is implemented in TRAFFIC-08 stage 1a; zebra priority, fixed-time lights,
  authored controls and importer hooks remain pending; explicit single-lane roundabout entry priority is implemented by ROAD-62
- lane changes are centerline S-curves, not full swept-body collision geometry
- parking, driveways, curb queues, and building entrance reservations are not modeled
- connector lanes are generated from lane necks; if road geometry changes materially, lane rebuild
  must keep connector lanes and `next_lanes` in sync
- network entry at a node does not claim the entry slot (`TRAFFIC-07`)
- walkers keep car spacing (2.7 m) in lane buckets, connector entry claims and overlap correction
