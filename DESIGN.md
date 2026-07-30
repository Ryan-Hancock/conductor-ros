# Conductor — Design

*Status: v1.5 — static toolchain + pluggable-transport runtime, an
rmw_zenoh transport verified against live ROS 2 (Lyrical) traffic,
`.msg`/`.srv`/`.action`-to-Go codegen with local REP-2011 hash computation,
lifecycle, parameters, observability with a built-in dashboard, an in-process
test harness, declared environments, single-binary deployment with
graph-derived systemd units, declarative missions, a declared transform tree,
a fleet view that merges a deployment's processes — across robots — into
one graph, a worked Nav2 example that drives a real navigation stack through
its lifecycle, its recovery behaviours and its actions, and discovery that
derives the externals block from a live graph rather than trusting a hand
transcription of it. This document records the vision, the architecture, and
the decisions still open.*

## Thesis

ROS 2 is a runtime pretending to be an application framework. It provides a
comms layer (DDS), a lifecycle spec almost nobody uses because it is tedious,
and a pile of conventions enforced by nothing. Everything a framework like
Encore automates for backend services — wiring, config, environments, tracing,
deployment — exists in ROS only as per-team folklore.

Conductor treats ROS 2 as the runtime it is. Developers describe what the
robot's nodes do in a small amount of declarative Go; the framework derives
the ROS artifacts (interfaces, launch configuration, parameters, QoS,
lifecycle, observability) and — critically — knows the **entire application
graph at build time**.

That last point is the Encore analogy that matters most. Encore's superpower
is static knowledge of the service topology. ROS knows nothing until runtime,
which is why its canonical failure mode is two nodes silently not talking
(QoS mismatch, typo'd topic, wrong type), discovered in the field. Conductor
makes an incomplete or incompatible graph a *build error*.

## Principles

1. **Compatibility over purity.** Generate the boring standard thing — real
   launch files, real DDS/Zenoh traffic, real parameter files — so Conductor
   apps coexist with Nav2, MoveIt, drivers, rviz, and rosbag. Rust dataflow
   frameworks (dora-rs, copper-rs) validate the "ROS is a runtime" thesis but
   abandoned wire compatibility; that discards the ecosystem, which is the
   only reason to be near ROS at all.
2. **The graph is a compile-time artifact.** Every topic, type, QoS pairing,
   handler, and parameter is statically derivable from source. If the checker
   can't prove the graph is wired, the build fails.
3. **Convention over configuration.** Node name = snake_case struct name.
   Handler = `On<Field>`. QoS = named intent profiles, not 12 DDS knobs.
   Escape hatches can come later; defaults come first.
4. **The 20% stays where it is.** Hard-realtime control loops, drivers, and
   perception pipelines remain C++/ros2_control territory. Conductor targets
   the orchestration/behavior/business-logic layer and interops with the rest
   over topics declared as *externals*.

## Why Go

- The target user is the backend developer robotics keeps borrowing. Typed
  channels and goroutines map naturally onto topics and executors.
- **Deployment is the sleeper feature.** `GOOS=linux GOARCH=arm64 go build`
  produces one static binary to copy onto a robot. Compare: shipping a colcon
  workspace plus its apt/rosdep dependency lattice. This alone may be the
  adoption wedge.
- GC pauses (sub-millisecond) are irrelevant for the non-hard-realtime layer
  Conductor owns.

## Architecture

Three layers, deliberately separable:

```
┌─────────────────────────────────────────────────────────┐
│ 1. Application model (source of truth)                  │
│    //conductor:node structs, Sub/Pub/Param/Timer fields │
│    //ros:type message mappings, conductor.json externals│
├─────────────────────────────────────────────────────────┤
│ 2. Static toolchain (cmd/conductor)                     │
│    scan (syntactic) → graph (validate) → gen (emit)     │
│    launch.xml · params.yaml · graph.dot · binary        │
├─────────────────────────────────────────────────────────┤
│ 3. Runtime (package conductor)                          │
│    reflection wiring · per-node single-threaded         │
│    executors · pluggable transport                      │
│    inproc bus (default) │ zenoh (rmw_zenoh-compatible)  │
└─────────────────────────────────────────────────────────┘
```

The scanner is purely syntactic (go/parser, no type checking), which keeps it
fast and dependency-free; the type-level guarantees come from the Go compiler
itself, since `Sub[T]`/`Pub[T]`/handler signatures are checked by `go build`.
The static view and the runtime view are two projections of the same
declarations, so they cannot drift.

### Validation rules (implemented)

| Code | Severity | Rule |
|---|---|---|
| CND001 | error | endpoint missing `topic` tag |
| CND002 | error | unknown QoS profile |
| CND003 | error | handler method missing or wrong arity |
| CND004 | error | invalid timer rate |
| CND005 | error | external role not publisher/subscriber |
| CND008 | warn  | message type lacks `//ros:type` mapping |
| CND010 | error | subscription with no publisher (internal or external) |
| CND011 | warn  | topic published but never consumed |
| CND012 | error | endpoints disagree on message type |
| CND013 | error | QoS request-vs-offered incompatibility (reliability, durability) |
| CND040 | error | mission transition (`next`/`fail`/literal `Goto`) names no such step |
| CND041 | error | mission shape: unknown `start`, steps without a `Mission`, two missions on one node |
| CND042 | warn  | step unreachable from the start step |
| CND043 | error | invalid `timeout`/`retry`/`backoff` on a step |
| CND050 | error | `frame:` tag names a frame the transform tree does not declare |
| CND051 | error | a frame has more than one parent |
| CND052 | error | the transform tree contains a cycle |
| CND053 | error | the transform tree has more than one root |
| CND054 | error | a literal `TF.Lookup` cannot be resolved from static transforms |
| CND055 | warn  | static transforms declared but no node declares `conductor.TF` |
| CND056 | warn/error | endpoints of one topic declare different frames (error if nothing connects them) |
| CND057 | error | `frame:` tag on a message type with no `Header` |

## The transport decision (RESOLVED: Zenoh-native, shipped in v0.2)

The runtime speaks to a `Transport` interface. Three candidate paths to a
live ROS 2 graph were considered:

**A. cgo over rcl (the rclgo approach).** Exact ROS semantics, all RMWs — but
drags the entire ROS build/dependency world into the Go toolchain, killing
the single-binary deployment story. Kept as a possible fallback for DDS-only
fleets.

**B. Zenoh-native (chosen, implemented).** Speak `rmw_zenoh`'s wire protocol
directly. Implemented in v0.2 and verified end-to-end against ROS 2 Lyrical
with rmw_zenoh 0.10: bidirectional pub/sub with ros2 CLI tooling, full graph
visibility (`ros2 node list`, `topic info --verbose` shows correct
type/hash/QoS). The implementation splits into:

- `transport/rmwzenoh` (pure Go): rmw_zenoh's conventions — topic keyexprs
  (`<domain>/<topic>/<dds type>/<RIHS01 hash>`), liveliness/graph tokens
  (`@ros2_lv/...`), QoS keyexpr encoding, and the per-message attachment
  (sequence number, source timestamp, GID). All formats were captured from
  live rmw_zenoh traffic with a wildcard sniffer and locked in as golden
  tests, not inferred from source alone.
- `cdr` (pure Go): XCDR1-LE serialization with the RTPS encapsulation
  header, golden-tested against `rclpy.serialization` output.
- `transport/zenoh` (cgo, `-tags zenoh`): a thin session layer over the
  official zenoh-go bindings + zenoh-c ≥ 1.9. Only this package needs native
  libs; the default build stays dependency-free.

Type hashes (REP 2011) are harvested from the ROS distro's installed type
description JSONs and registered per message type — computing RIHS01 from
scratch is deliberately avoided until `.msg` codegen lands.

**C. Pure-Go DDS.** No mature implementation exists; rejected.

Known transport limitations: no transient-local latching toward late joiners
(rmw_zenoh uses advanced publishers with cache for this), publisher GIDs are
random rather than XXH3-derived from the liveliness keyexpr (cosmetic in
`ros2 topic info` cross-referencing), and a cgo session rather than pure Go
(a pure-Go zenoh client would restore the fully-static binary; zenoh-pico's
protocol subset is a plausible template).

**Interop testing must cover conductor↔conductor, not just conductor↔ROS.**
The zenoh querier's default consolidation mode (LATEST) deduplicates replies
by key expression, and since every reply to a ROS service arrives on the one
service keyexpr, it silently swallowed replies whenever *both* peers were
conductor processes. Conductor↔rclpy worked in both directions throughout,
so the bug survived a full milestone undetected; it only surfaced when an
action client first called an action server that was also conductor.
`.tools/interop.sh` now runs every leg of that matrix, including the
same-implementation ones. The general lesson: when reimplementing a wire
protocol, testing only against the reference implementation leaves the
diagonal untested, and asymmetries in the reference (rmw_zenoh sets
consolidation explicitly, with a comment) are the exact places bugs hide.

## The 80% — repeated ROS patterns and where they land

| Pattern | Today in ROS | Conductor | Milestone |
|---|---|---|---|
| Node boilerplate (init/executor/spin) | hand-written | generated by `Run` | ✅ v0.1 |
| Topic wiring + QoS | runtime folklore | declared, statically checked | ✅ v0.1 |
| Launch files | imperative Python | generated from graph | ✅ v0.1 (basic) |
| Development bringup | shell: start router, `sleep 2`, start sim, `sleep 4`, run, `pkill` | `conductor run`: the environment's declared processes, waited on by condition, torn down by process group; `-split` runs the deployment's process-per-node layout with the fleet view over it; the dashboard is on by default | ✅ v1.3 |
| Parameters + per-env config | YAML sprawl | `Param[T]`, parameter files with environment overlays, ROS parameter services for live updates | ✅ v0.9 |
| Timers/watchdogs | hand-written | `Timer` fields | ✅ v0.1 |
| Message interop | .msg packages + CMake | CDR codec; `.msg` → Go codegen with local RIHS01 hash computation (validated against the full distro corpus) | ✅ v0.3 |
| Live ROS graph | — | Zenoh transport, rmw_zenoh wire-compatible | ✅ v0.2 |
| Lifecycle + startup ordering | unused spec, `wait_for_service` loops | every node is a managed node; hooks by convention; timers/publishers/subscriptions gated on Active; bringup order derived from the graph (runtime, `check`, and generated launch) | ✅ v0.7 |
| Services | boilerplate | `Svc[Req,Res]` / `Client[Req,Res]` fields, graph-validated, rmw_zenoh querier/queryable wire format, `.srv` codegen | ✅ v0.4 |
| Actions (server) | boilerplate | `Action[G,F,R]` fields over the 3-service/2-topic convention; goal state machine, per-goal goroutines, context cancellation; `.action` codegen | ✅ v0.5 |
| Actions (client, e.g. calling Nav2) | boilerplate | `ActionClient[G,F,R]` with goal handles, feedback channels, cancellation | ✅ v0.6 |
| Observability | `/rosout` + prayer | a span per callback with W3C trace context propagated through messages; Prometheus metrics per node/topic/callback; a built-in dashboard (graph, live rates, lifecycle, parameters, traces, missions, frames) served by the app itself | ✅ v0.8, dashboard v1.2 |
| Testing | `launch_testing` | the whole app inside `go test`: in-process transport, deterministic timers, settle-not-sleep, typed publish/record/call | ✅ v1.0 |
| Deployment | colcon + rosdep + apt | `conductor deploy`: cross-compiled binary, release bundle with a manifest, systemd units whose ordering is the graph's bringup order, ssh install, rollback; fleets roll robot by robot, gated on each graph coming up | ✅ v1.1, fleets v1.3 |
| Per-env config (sim/dev/robot-N) | copy-pasted YAML | `environments.json`: per-environment externals, transport, parameter overlays and deploy target, and the robots an environment runs on, each overriding its own calibration and tuning; `-env` and `-robot` everywhere | ✅ v1.1, robots v1.3 |
| Task orchestration (state machines/BTs) | XML/hand-rolled | `Mission`/`Step` fields with tagged transitions; targets checked (including literal `Goto`s), unreachable steps warned, machine drawn in mission.dot, current step observable | ✅ v1.2 |
| TF conventions | `static_transform_publisher` in a launch file | `frames.json` published on tf_static, `TF.Lookup` composition, `frame:` tags that stamp and check headers, tree checked at build time | ✅ v1.2 |
| Driving Nav2 | hand-written action clients, `wait_for_service` loops, a behaviour tree XML for the sequencing, `lifecycle_manager` for the startup | `examples/nav2`: the stack's startup is a mission step, recovery is a `fail:` branch onto Nav2's own behaviours, docking is a checked `Goto`; upstream `nav2_msgs` definitions so the hashes are Nav2's; a stand-in stack so it runs without one | ✅ v1.4 |
| Externals of someone else's graph | hand-transcribed from source | `conductor externals`: read the liveliness graph, roll actions up, derive the block, and diff it against what is declared (`-check` in CI, `-write` to update) | ✅ v1.5 |
| Driving MoveIt | as above, plus planning-group magic strings | the same treatment as Nav2; the largest nested messages in common use, so also a CDR/msggen stress test | v1.6 |
| Bringing a managed stack up | `lifecycle_manager` + `autostart`, ordering in a launch file | a lifecycle *client*: drive transitions and wait for Active in the graph's own order, over the set discovery already found | v1.6 |
| Robot description | URDF + xacro, constants duplicated into code and launch files | `frames.json` derived from the URDF's fixed joints, `/robot_description` published, SRDF planning groups declared and checked like frames | v1.6 |
| Simulation | `use_sim_time` folklore, hand-written spawn scripts | the simulator is a declared `requires`; a clock source behind `Timer` and header stamping so sim time is the same code path as wall time | v1.6 |

## Observability (v0.8, implemented)

The runtime owns every callback invocation, so it wraps all of them.

**Tracing.** Each callback gets a span. The hard part is causal propagation,
and the trick that makes it invisible to users is the executor: a node runs
its callbacks on exactly one goroutine, so the runtime can park the current
span's context on the node while a callback runs, and any `Publish` from
inside that callback picks it up. No `context.Context` threading, no changes
to handler signatures.

Trace context crosses processes in an extension appended to the rmw_zenoh
attachment (magic `CDTR`, 16-byte trace id, 8-byte span id, flags). This is
safe because rmw's deserializer reads its three fields positionally and
ignores trailing bytes — confirmed in its source and then on the wire, with
`ros2 topic echo` reading traced messages unchanged. IDs follow W3C
trace-context so `Traceparent()` drops straight into OpenTelemetry; the
`Exporter` interface keeps the OTel SDK out of the dependency list.

Known gap: action handlers run on a goroutine per goal rather than the
executor, so they start a fresh trace rather than continuing the caller's.

**Metrics.** A small built-in registry exposed in Prometheus text format on
an optional HTTP endpoint — no client library dependency, since the
exposition format is a few lines to write. Message counts, callback
latency sums, service outcomes, and lifecycle state per node, all with no
user code.

**The dashboard (v1.2).** Encore's local development dashboard is the part
developers actually feel, and the same argument applies here: the runtime
wired every endpoint, owns every callback, and already propagates trace
context, so it can *show* the application instead of leaving a developer to
reconstruct it from `ros2 topic hz` in one terminal and `ros2 param get` in
another. `-dashboard :4000` serves the graph in bringup order, a live card
per node, editable parameters, lifecycle controls, the metric table, and —
with `-dashboard-traces N` — recent traces nested as causal chains.

Three decisions worth recording:

- **The runtime describes itself.** Binding already knows each endpoint's
  name, type, QoS and rate, so it records an inventory (`inventory.go`)
  rather than the dashboard re-deriving one. The static view (`conductor
  check`) and this live view are again two projections of one set of
  declarations — but this one reports what a process actually has open.
- **Counters out, rates in the page.** The API exposes absolute counters and
  a server timestamp; the page divides successive snapshots. The runtime
  never has to choose a rate window, and pausing the page pauses the maths.
- **Writes go through the existing paths.** A parameter edit calls the same
  handle `ros2 param set` uses, so type checking is identical; a lifecycle
  button calls the same transition `ros2 lifecycle set` does. The dashboard
  is a view over the runtime, not a second way to change it.

Tracing stays opt-in — a span per callback is not free — and the page is one
embedded self-contained file, because the machine that most needs the view is
a robot with no route to a CDN.

**The fleet view (in progress).** A zenoh deployment is one process per node,
so the honest per-process dashboard is also a partial one: everything beyond
that process is `ROS graph`. `conductor dashboard -env <name>` fans out over
the deployment's processes and merges their state into one graph.

Two decisions carry it. The peers are *derived*: units are generated with a
dashboard port per node (base plus bringup position, the rule the metrics
ports already follow), so the view resolves the same addresses from the same
declarations instead of being handed a list that drifts. And the merge is a
plain HTTP client of the API the processes already serve — no new wire
protocol, no second source of truth, and it runs from a laptop or from the
robot itself.

What the merge is *for* is the class of failures no single process can see: a
subscriber whose publisher is not answering, two processes disagreeing about
a topic's type or QoS (a deploy caught halfway), two robots reporting
different transform trees (one stale calibration), a unit that is simply
down. Those are findings on the page rather than something to notice in four
journals.

**Traces across processes.** The trace context has crossed the wire since
v0.8, which means each process already records its half of every distributed
trace and neither half is legible alone. The collector polls `/api/spans`
with a cursor — only what each process has recorded since the last poll, so
watching a robot costs the new spans rather than the ring — and joins parent
to child by span id, regardless of which process each end came from. What
comes out is a causal chain across units with the handovers marked, and the
gap between a publish and the remote callback it caused is the wire latency,
measured rather than guessed.

Two things the merge refuses to fake. Clocks on separate machines disagree,
so the chain is ordered by causality, not timestamps; a child that appears to
start before its parent is reported (FLEET07) with the size of the
disagreement instead of being drawn as time running backwards. And a span
whose parent has not been collected is marked as an orphan rather than
promoted to a root, because "this is the start of the chain" and "the other
end is missing" are different statements.

**The development loop (v1.3, implemented).** `conductor run -split` runs the
layout the robot has: one process per node, in bringup order, on the ports
the generated units would assign — the same rule, so the fleet view resolves
them without being told. It exists because the default locally is the
in-process bus, which hides every boundary that exists in production; a topic
that never crosses a process in development crosses four of them on the
robot. Splitting an in-process environment is refused for the reason deploy
makes one unit instead of many.

The dashboard follows Encore's lead and is served by *default* for a
development run — at the environment's address, on loopback when it has none
— with a browser opened where there plainly is one and the URL printed either
way. A split run opens the fleet view instead of four pages, which is the
payoff for having built the aggregator first: it is also the only layout in
which cross-process traces have anything to stitch. Deployed units keep the
opposite default: they serve a dashboard only when asked.

**Local bringup (v1.3, implemented).** `conductor run` is the same argument
as the rest of the framework, applied to the thing every robotics repository
keeps in a Makefile: an environment already declares its transport, its
parameters and its calibration, so the only missing declaration was *who
provides the externals here* — a simulator in one environment, a real driver
in another. That is `requires`, and it lives in the environment because it is
what differs between them.

Two details carry it. Readiness is a **condition**, not a duration: a port to
connect to (the endpoint the environment already declares) or a command to
poll. A `sleep 4` is a guess that is too long on a fast machine and too short
on a loaded one, and it is where development bringup flakiness comes from.
And teardown is **by process group**, which is what `pkill -x turtlesim_node`
cannot do: it stops the launcher's children along with it, and it stops this
run's simulator rather than whichever one happens to share the name.

Conductor deliberately learns nothing about how ROS is installed doing this —
what to run is a command line, and whether it is up is a condition — so the
toolchain still builds and tests without a ROS install.

**Fleets (v1.3, implemented).** A fleet turned out not to need a file of its
own: it is an environment that runs on more than one machine, so the robots
are declared in the environment that describes them. Each robot inherits
everything and overrides only what is per-machine — host, calibration,
parameter overlays, router endpoint, dashboard port. Resolving a robot
produces an ordinary resolved application, which is the decision that kept
one code path: deploy, check and the dashboard gained a `-robot` flag and
nothing else.

Two consequences worth recording. **The rollout is gated by the runtime's own
account of itself**: robots are done one at a time, and the next is only
touched once every process on the last one answers and every node reports
Active. That is the fleet view used as a gate rather than a person watching a
dashboard, and it means a bad release reaches one robot instead of ten. **Two
robots are two ROS graphs**, so the merge keeps them apart; joining them
would draw edges between machines that cannot talk, and would turn ordinary
per-robot calibration into a false alarm about diverging transform trees.

Known gap: there is no authentication — the assumption is a robot network,
not the internet.

## Missions and frames (v1.2, implemented)

The two remaining lines of the 80% table were the two places where ROS still
keeps the application's structure somewhere the compiler cannot see it: a
behaviour tree in XML, and a transform tree in launch-file arguments. Both
become declarations.

**A mission is a state machine of fields.** `Mission` names the start step,
`Step` fields name their own transitions (`next`, `fail`, `timeout`, `retry`,
`backoff`), and the handler is `On<Step>(*Task) error`, found by the same
convention as every other callback. Three consequences follow, and they are
the argument for the whole approach:

- **The transitions are checked.** A `next:` naming no step is a build error.
  So is `t.Goto("recharge")` when it is written with a string literal — the
  scanner collects those calls and resolves them against the same step set,
  which is a syntactic scanner earning its keep: the branches taken in code
  are held to the declarations, not just the ones written in tags.
- **The lifecycle owns it.** A mission starts when its node reaches Active and
  is canceled when it leaves, so `ros2 lifecycle set` stops a task in flight,
  and a step's `Task.Context()` is what makes that real. Steps run on the
  mission's own goroutine — long-running by nature, like action handlers —
  with `Task.Do` for the moments they must touch executor-owned state.
- **The view is free.** Current step, attempts, entries and durations are
  metrics and spans; `conductor check` prints the machine, `gen/mission.dot`
  draws it, and the dashboard shows the live position on it.

**The transform tree is configuration, not code.** `frames.json` declares
static links (ours, published on `tf_static`) and dynamic ones (someone
else's, with `by` naming the publisher). Declaring the dynamic links is what
makes the tree whole: it lets the checker distinguish "that frame does not
exist" from "that frame exists, but only a transform published at runtime
reaches it", which is the difference between a typo and a lookup that belongs
in a callback. Both the toolchain and the runtime read the file through the
same loader, so the robot and the checker cannot disagree about where the
lidar is; an environment may name a different file, because calibration is
per-robot.

A `frame:` tag then does on a topic what the QoS tag does for delivery: the
publisher stamps the declared frame (and a timestamp) into the header, and a
subscription verifies incoming ones, counting and naming a peer that sends
another frame — once, not at the topic's rate. Verified against real ROS 2:
`ros2 topic echo /tf_static` reads the tree as `tf2_msgs/msg/TFMessage`, and
`tf2_ros tf2_echo base_link laser` reports the declared offset and yaw.

Known gap: `tf_static` is republished at 1 Hz rather than latched, because
transient-local durability is not implemented on either transport. A late
joiner sees the tree within a second instead of immediately, which is
acceptable for a static tree and not a reason to fake latching in the layer
above.

## Testing (v1.0, implemented)

The framework owns wiring, the executors, and the clock's effect on the
application, which is exactly what makes a ROS test flaky when the framework
does not. So the harness (`conductortest`, over `conductor.TestApp`) takes
all three back:

- **The in-process transport is the same transport.** Nodes are wired,
  brought up through their real lifecycle, and talk over the bus that ships
  in production; only the wire format is absent.
- **Timers do not tick.** `Tick(node)` fires a node's timers once, so a test
  says "one control period passed" rather than sleeping and hoping. Wall
  clock timers remain available for the tests that genuinely want them.
- **Settle, don't sleep.** `Publish`, `Tick` and `Call` return only once the
  graph is quiet: a barrier is queued behind every node's mailbox and the
  round repeats while callbacks are still producing callbacks. A message
  crossing three nodes has arrived before the next assertion runs, with no
  timing constants in the test.
- **Probes.** A test can wire its own endpoints into the running app as an
  extra node (`app.Probe`), which is how action clients — with goal handles,
  feedback channels and cancellation — are driven without special-casing
  actions in the harness.

`conductor test` runs graph validation before the tests, because a wiring
mistake otherwise surfaces as a pile of behavioural failures with no
explanation. That ordering is the same instinct as the rest of the design:
say what is wrong in the vocabulary of the mistake.

## Environments and deployment (v1.1, implemented)

Environments live in `environments.json` beside `conductor.json` — a separate
file, following the Encore split the open question below argued for: the
application model is code, the environments it runs in are configuration. An
environment declares its transport, parameter overlays, the externals present
there (merged over the base set, or dropped with `without`), and a deploy
target.

The payoff is that **the checker becomes environment-aware**. Externals are
what tell the graph checker a subscription has a publisher, so declaring them
per environment means `conductor check -env sim` can report that a service
nothing calls in simulation, or a topic no driver publishes there, is exactly
that — before anything runs, in the vocabulary of the environment.

Deployment is then mostly mechanical, and deliberately so: cross-compile,
stage, tar, copy, run the bundle's `install.sh`. What makes it Conductor's
rather than a shell script is the same static knowledge everything else uses:

- **Units are a projection of the graph.** One unit per node, with `After=`
  taken from the dependency edges that produce the bringup order, and cycle
  edges dropped because systemd breaks ordering cycles arbitrarily.
  Ordering only (`Wants=`, never `Requires=`): a provider restarting must not
  stop its consumers, since the lifecycle already handles a peer that is not
  up. The launch file and the units are two renderings of one order.
- **The transport chooses the process layout.** An `inproc` environment
  deploys as a single unit running every node — its bus does not leave the
  process, so a unit per node would be a set of mutually silent robots. This
  is knowable statically and is therefore refused statically.
- **What cannot work is refused before the build.** An environment on the
  zenoh transport without `-tags zenoh` would exit on the robot with
  "unknown transport"; a cgo cross-build with no `cc` would fail late. Both
  are deploy-time errors with the fix in the message.
- **Releases are versioned and switchable.** `releases/<version>` with a
  `current` symlink, a recorded previous, and `-rollback` as a symlink swap
  plus a restart. The manifest records the graph fingerprint (a hash of every
  topic, type, QoS and parameter), so "what is this robot running?" is
  answerable without trusting timestamps.

The install script is generated with its values already substituted rather
than being a template driven by flags, because the robot is where debugging
is hardest: an operator with no network can read it and run it by hand.

Known limitation: the zenoh transport is cgo, so a release that joins a live
ROS graph is not the fully static binary the pure-Go path produces — it needs
a cross toolchain with zenoh-c for the target. This is the deployment-side
cost of the transport decision above, and the argument for a pure-Go zenoh
client.

## Driving Nav2 (v1.4, implemented)

The thesis has been asserted since the first line of this document: the 20%
stays where it is, and Conductor orchestrates it. `examples/nav2` is the first
place it is tested against something that did not bend to fit — a stack with
its own lifecycle, its own recovery behaviours, its own opinions about QoS,
and interfaces nobody here designed.

The application is two nodes. A **commander** whose mission is the patrol —
bring the stack up, localize, drive to a waypoint, follow it, arrive, recover,
dock, charge — and a **watchdog** that reports on `/diagnostics` what Nav2 is
actually doing, because deciding what should happen and watching what does are
different jobs with different blocking behaviour. Three claims survive the
exercise, and each replaces a piece of ROS convention:

- **The stack's startup belongs to the mission, not to a launch file.** Nav2's
  servers are managed nodes; `navigate_to_pose` does not answer until a
  `lifecycle_manager` has driven them to Active. So `bring_up` is the mission's
  first step, calling `manage_nodes` with `retry:"3" backoff:"2s"` — and the
  `sim` environment launches `nav2_bringup` with `autostart:=False`, because
  the ordering now comes from the graph. This is the third use of the
  lifecycle machinery (server, rollout gate, and now client-of-a-manager), and
  it is what makes the *client* case in the roadmap below worth building
  properly.
- **A recovery branch is a tag.** `Following` declares `fail:"recover"`, and
  the recover step runs Nav2's own `backup` then `spin` and returns to the
  same waypoint, since the route index only advances on arrival. The reason
  travels with the branch: `t.Err()` reads `navigation ABORTED: … (nav2
  error_code 9000)`. In a behaviour tree this is a subtree; here it is one
  tag and a step that says what it did.
- **The diversion is a checked string.** `t.Goto("docking")` on a low battery
  is resolved against the declared steps by `conductor check`, so the branch
  that runs least often is the one the compiler has already looked at.

The interfaces are the upstream `.action` and `.srv` text, vendored into the
example so it builds with no Nav2 installed and hashed locally — which means
the RIHS01 hashes are Nav2's and a real stack answers. Alongside it,
`examples/nav2stub` serves those same interfaces with no navigation behind
them, so the example runs, and fails, anywhere: it refuses goals until
started, and aborts every `fail_every`'th goal, because a stand-in that always
succeeds would leave the recovery branch untested. The pattern generalizes —
the stand-in is the cheapest way to make an interop example runnable by
someone who has not installed the other side.

Four things this found, which is the point of building it:

- **An aborted goal must carry its result.** The action server reported
  ABORTED and dropped the handler's result value, keeping it only for
  cancellation. That is wrong for every interface in common use, because the
  reason for failure lives in the result: `NavigateToPose` puts it in
  `error_code` and `error_msg`. Fixed, with a test — a client that learns only
  "aborted" has been told that something went wrong and not what.
- **Traces do not cross a mission step's publish.** A message published from
  inside a callback carries that callback's trace context, because the
  executor stores it for the duration. A mission step runs off the executor by
  design, so its publishes carry nothing, and the fleet view shows the
  watchdog's handling of a waypoint as a separate root rather than as the
  continuation it is. Two candidate fixes: thread a context explicitly at the
  publish site (an API addition), or have a step publish the trace it is
  already carrying as a fallback the executor's context overrides. The second
  is smaller and mis-attributes an action handler's publishes in a node that
  also runs a mission, which is why this is recorded rather than guessed at.
- **The frames model's axis is ownership, not motion.** Nav2's tree is
  entirely someone else's: `robot_state_publisher` publishes the fixed joints,
  amcl and the odometry publish the moving ones. Conductor's `dynamic` means
  "not ours", so the example declares the whole tree that way and gets the
  frame checks (CND050–057) with no publishing — correct, but it means
  `TF.Lookup` cannot resolve a chain that is fixed in the URDF and merely
  published by someone else. Deriving the tree from the robot description is
  the fix for both halves at once.
- **The externals list is exactly the transcription this project exists to
  remove.** Two entries in `conductor.json` were only right after reading
  Nav2's source: `cmd_vel` is `TwistStamped` now, and `amcl_pose` is a latched
  publisher. Getting either wrong produces silence, which is the failure mode
  the checker is supposed to prevent — and it cannot, because it is checking a
  hand-written claim about someone else's graph. Hence the first item below.

## Reading someone else's graph (v1.5, implemented)

Every other declaration in Conductor is derived from something the toolchain
can see: the graph from the code, the units from the graph, the bringup order
from the dependency edges. The externals block was the exception — a hand
transcription of somebody else's system, believed absolutely by the checker.
The Nav2 example made the cost concrete: two entries were only right after
reading Nav2's source, and being wrong in either would have produced silence
that the checker would have cheerfully validated.

So the last hand-written declaration becomes a derived one. `conductor
externals` queries the graph and reports the difference; `-write` updates
conductor.json, `-check` fails a script.

**Discovery costs nothing new.** rmw_zenoh advertises every entity as a
liveliness token whose key expression already carries the node, the topic, the
DDS type, the RIHS01 hash and the QoS profile — and `transport/rmwzenoh`
already wrote those tokens, byte-for-byte, to be indistinguishable from a C++
node. Reading the graph is that mapping run backwards: a parser in the same
pure-Go package, tested by round-tripping every token the writer can produce
and by decoding one captured from live rmw_zenoh traffic. No ROS install, no
`ros2` CLI to parse, no daemon whose cache might be stale.

Three judgements make the difference between a generator worth using and one
that is only mostly right:

- **An action is one interface.** rmw sees three services and two topics; the
  declaration wants `nav2_msgs/action/NavigateToPose`, recovered from the
  derived types (`..._SendGoal` and friends). Infrastructure every node carries
  — parameter services, lifecycle services, `/rosout`, `/tf` — is filtered by
  *type* rather than by name, because names move with namespaces and types do
  not.
- **A role describes the outside, and we are not outside.** Our subscription
  needs an external publisher; our client needs a server. The application's own
  nodes are excluded by name, since a conductor process on the graph advertises
  everything it publishes, and declaring that external would tell the checker
  its own topics come from somewhere else.
- **Disagreement is reported, never averaged.** Two publishers offering
  different profiles make "the" QoS of a topic a question: the weakest offer is
  declared, because that is what a subscriber must match to hear all of them,
  and the disagreement is named. Two peers advertising different type *hashes*
  for one interface are running different definitions of it — a fault no
  comparison of type names can see. And a declaration the graph cannot confirm
  is reported `absent` and kept: half a stack being up is the normal state of a
  robot under development, and a tool that deleted declarations on that basis
  would be worse than the transcription it replaced.

`-write` also refuses to flatten an environment's `externals`/`without` overlay
into the base file. The list this command sees is the merged one, and writing
it back would quietly make a simulation's stand-in drivers part of every
environment.

The honest limitation: this reads a *zenoh* graph, so the command needs the cgo
transport compiled in — and without it, it says so rather than reporting an
empty graph, because "nothing is running" and "I cannot look" are different
answers. It is also a snapshot: what is up now, not what the robot has when
everything is running, which is why `-check` belongs in the interop matrix
(where the stack is deliberately up) rather than in `conductor check`.

## What comes next (v1.6 and beyond)

The pattern so far is that each milestone took something ROS leaves as
folklore and made it a declaration the toolchain can read. What follows is
the same exercise applied to the places a conductor application still meets
the ecosystem by hand.

### Driving lifecycles directly, and the next big stack

**A lifecycle client.** Nav2's nodes are managed nodes driven by a
`lifecycle_manager`, and the example drives that manager over its
`manage_nodes` service — one service call standing in for the ordering
Conductor already derives. A first-class client (drive a transition, wait for
Active, report which of a set is not up) could replace the manager with
something whose order comes from the graph. The runtime already speaks the
protocol as a server, the fleet rollout already waits for Active, and the
example is now the third caller of the same machinery. Discovery makes it
sharper still: `conductor externals` can already see which lifecycle services
exist and on which nodes, so the set to drive need not be written down either.

**A MoveIt example** is the harder one, and useful for a second reason: its
interfaces (`moveit_msgs/action/MoveGroup`, the planning scene, joint
trajectories) are the largest nested messages in common use, so it is a real
stress test of the CDR codec and of msggen on deeply nested arrays. The
manipulation logic itself — pick, move, place, recover — is a mission, and
planning group names are magic strings today, which is what the SRDF work
below would fix.

### The robot description: URDF and SRDF

`frames.json` and a URDF say overlapping things. The URDF already describes
every link and joint; its **fixed joints are exactly the static transforms**
conductor publishes on tf_static, and its movable joints are exactly the
dynamic links a robot_state_publisher provides. Declaring them twice is
duplication conductor introduced, and the fix is to derive: `conductor frames
-from robot.urdf` emitting the tree, with movable joints marked dynamic and
attributed to whoever publishes them. Then the frame checks (CND050–057)
apply to the robot's real description rather than to a file beside it.

Two related pieces follow. Tools find a robot's model on
**`/robot_description`**, a transient-local topic — which is a second
dependent for the latching gap below, and an argument for closing it. And
**SRDF** names planning groups, named poses and disabled collision pairs; a
MoveIt-driving application refers to those by string today, and they could be
declared and checked exactly as frames are.

The cost to be honest about: URDF is XML with a macro language (xacro) layered
on top, and xacro is Python. Conductor should read plain URDF — a small,
well-specified subset for links and joints — and leave xacro expansion to a
`requires` step or a build command, rather than pretending to reimplement it.

### Simulation and time

Gazebo needs less from conductor than it looks: `conductor run` already
starts a simulator as a declared `requires` with a readiness condition, and
spawning models is another command. What simulation actually needs from the
*runtime* is **time**.

ROS 2 answers this with `/clock` and a `use_sim_time` parameter, and a node
that ignores it computes velocities and timeouts against a clock the world is
not using. Conductor's timers, its header stamping, its mission timeouts and
its watchdogs all read the wall clock today. The design already has the shape
of the answer, though: the test harness proved that time can be a source the
runtime reads rather than a global — `Tick` drives timers deterministically
in `go test`. Simulated time is the third implementation of that idea, behind
the same abstraction: wall clock, test clock, `/clock`. Honouring
`use_sim_time` also matters for interop, since other tools set it on our
nodes and expect it to be obeyed.

With that in place, **scenario tests** become possible in the same shape as
the unit tests: bring up a world, run the application, assert on what it did,
with time under the test's control. The cheaper cousin is **rosbag replay** —
conductor knows exactly which topics an application consumes, so it could
record precisely those and replay them into the harness, giving regression
tests from real robot data without a simulator at all.

### Smaller things worth doing

- **Trace context from off-executor goroutines** (see Nav2 above): a mission
  step's or action handler's publish loses the trace, so a fleet view of a
  mission-driven application shows roots where it should show a chain. The
  choice is between an explicit publish-site context and a per-step fallback.
- **Goal rejection**: the action server accepts every goal and reports refusal
  through the result, because a handler is only called after acceptance. Real
  servers reject — "the stack is not active", "that pose is off the map" —
  before accepting, and `ErrGoalRejected` exists on the client already with
  nothing on the server able to cause it.
- **Transient-local latching**: still the known gap in the runtime. It now has
  two dependents that matter — `tf_static` (republished at 1 Hz as a
  stand-in) and `/robot_description` — which is enough to promote it from
  "noted" to "next".
- **Parameter constraints**: ROS parameter descriptors carry ranges and
  enumerations. `Param[float64]` with `min`/`max` tags could refuse a bad
  `ros2 param set` the way the type system already refuses a bad type, and
  the dashboard could render a slider rather than a text box.
- **A QoS escape hatch**: the non-goal says intent profiles now, raw knobs
  later. A `qos:"custom"` with explicit fields, still checked for
  compatibility, is the shape.
- **Dashboard authentication**: the fleet view assumes a robot network. A
  token, or a bind default of loopback plus an explicit opt-in to a routable
  interface, is the minimum before anyone puts one on a machine with a public
  address.
- **Multi-instance nodes** (open question 3): two cameras is the canonical
  case, and every declared example so far has been a singleton.
- **Zero-copy for large messages**: the in-process bus passes Go values, so it
  is already copy-free within a process; over zenoh, images and point clouds
  pay a CDR round trip. Zenoh's shared memory transport is the obvious
  answer, and the perception boundary is where it would matter.

## Non-goals

- Hard-realtime control (stays in C++/ros2_control; interop via externals).
- Replacing perception/planning stacks — Conductor orchestrates them.
- Supporting every DDS QoS knob; intent profiles with an escape hatch later.
- ROS 1.

## Open questions

1. Zenoh-native vs rclgo-cgo ordering (see transport decision above).
2. `.msg` codegen ergonomics: generate from installed ROS interface packages,
   or vendor common interface definitions so the toolchain needs no ROS
   install on the dev machine? (Leaning: vendor, for the no-ROS-laptop DX.)
3. Namespacing/remapping model — per-instance node names when the same struct
   runs twice (e.g. two cameras). Likely: instance tags in `Run` +
   namespace field in conductor.json.
4. ~~Per-environment externals~~ (RESOLVED in v1.1): a separate
   `environments.json`, as the Encore precedent argued. What remains open is
   how far an environment may reach — it currently overrides externals,
   transport, parameters and the deploy target, but not QoS profiles or node
   membership (a node that only exists on the robot). Node membership in
   particular starts to look like a different application rather than a
   different environment, which is probably the line.
5. Sim-in-CI: the harness covers logic; scenario tests against Gazebo (or a
   rosbag fixture replayed through the zenoh transport) are the missing
   layer, and they need a story for time — likely `/clock` and a simulated
   time source behind the same Tick abstraction. Sharpened in "Simulation and
   time" above: the open part is no longer *whether* time should be a source
   the runtime reads, but whether `use_sim_time` should be a runtime flag, a
   parameter (as ROS has it), or a property of the environment — the last
   being most conductor-shaped and least like what other tools expect.
6. ~~Fleet deployment~~ (RESOLVED in v1.3): robots are declared inside the
   environment they belong to rather than in a fleet file of their own,
   because a fleet is an environment that runs on several machines and a
   robot is that environment's instance on one. The rollout is sequential
   and gated on each robot's graph reaching Active. What remains open is
   rollout policy beyond "one at a time, stop on failure": canarying is
   expressible today as an environment with one robot, but batching,
   automatic rollback of the robots already done, and a bake time before
   moving on are not.
7. Mission composition: a mission is one machine per node, flat. Nested
   missions (a step that runs a sub-machine) and concurrent branches are what
   behaviour trees offer and this does not. Both are expressible today by
   giving the sub-task its own node and driving it over an action — which
   keeps every step observable and lifecycle-managed — but that is a heavier
   answer than a `steps:` tag pointing at another machine. Worth revisiting
   once real missions exist to argue from.
8. Dynamic transforms: `TF` resolves the declared static tree, and a lookup
   that crosses a dynamic link is refused rather than guessed. Consuming
   `/tf` at runtime means a buffer with time interpolation and a policy for
   extrapolation — a real feature, not a small one, and the declared tree is
   what makes it checkable when it arrives.
9. How far to take the robot description. Deriving `frames.json` from a URDF
   is clearly right; the question is whether the URDF then becomes the source
   of truth outright — links, joints, limits, collision geometry, with
   conductor reading it directly — or stays an import step that produces a
   file the checker reads. Import keeps the toolchain free of XML and the
   errors legible; direct reading avoids a generated file that can go stale.
   Leaning: import, with a check that the derived file still matches its
   source, which is the same trick the graph fingerprint plays for releases.
10. ~~Whether `conductor run` should default to serving the dashboard~~
    (RESOLVED in v1.3): on for `run`, off for anything deployed, and bound to
    loopback unless an environment says otherwise. What remains open is the
    same question one level up — whether a *split* run should also default to
    opening the fleet view rather than printing it, which is a stronger
    action than opening one page and is currently taken anyway.
9. Services/actions API shape: `Svc[Req,Res]` with `OnField(Req) (Res,
   error)` is the obvious mirror; actions need goal/feedback/result and
   cancellation — design against Nav2's action servers as the reference
   consumer.
