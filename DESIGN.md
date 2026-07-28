# Conductor — Design

*Status: v0.3 — static toolchain + pluggable-transport runtime, a working
rmw_zenoh transport verified against live ROS 2 (Lyrical) traffic, and
`.msg`-to-Go codegen with local REP-2011 hash computation. This document
records the vision, the architecture, and the decisions still open.*

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
| Parameters + per-env config | YAML sprawl | `Param[T]`, parameter files with environment overlays, ROS parameter services for live updates | ✅ v0.9 |
| Timers/watchdogs | hand-written | `Timer` fields | ✅ v0.1 |
| Message interop | .msg packages + CMake | CDR codec; `.msg` → Go codegen with local RIHS01 hash computation (validated against the full distro corpus) | ✅ v0.3 |
| Live ROS graph | — | Zenoh transport, rmw_zenoh wire-compatible | ✅ v0.2 |
| Lifecycle + startup ordering | unused spec, `wait_for_service` loops | every node is a managed node; hooks by convention; timers/publishers/subscriptions gated on Active; bringup order derived from the graph (runtime, `check`, and generated launch) | ✅ v0.7 |
| Services | boilerplate | `Svc[Req,Res]` / `Client[Req,Res]` fields, graph-validated, rmw_zenoh querier/queryable wire format, `.srv` codegen | ✅ v0.4 |
| Actions (server) | boilerplate | `Action[G,F,R]` fields over the 3-service/2-topic convention; goal state machine, per-goal goroutines, context cancellation; `.action` codegen | ✅ v0.5 |
| Actions (client, e.g. calling Nav2) | boilerplate | `ActionClient[G,F,R]` with goal handles, feedback channels, cancellation | ✅ v0.6 |
| Observability | `/rosout` + prayer | a span per callback with W3C trace context propagated through messages; Prometheus metrics per node/topic/callback | ✅ v0.8 |
| Testing | `launch_testing` | mocked-topic unit tests, rosbag-fixture replay, sim-in-CI scenario runs | v0.5 |
| Deployment | colcon + rosdep + apt | cross-compiled static binary, `conductor deploy` | v0.6 |
| Per-env config (sim/dev/robot-N) | copy-pasted YAML | `-params` files plus `-env` overlays (parameters done; per-environment *externals* still open) | ✅ v0.9 (partial) |
| Task orchestration (state machines/BTs) | XML/hand-rolled | declarative mission layer | v1.0 |
| TF conventions | boilerplate | declared static transforms, frame checks in graph | v1.0 |

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
4. Parameters now have environment overlays (`-params` + `-env`, resolved
   as params.<env>.yaml). The open half is *externals*: should
   `conductor.json` also gain per-environment sections, so a sim
   configuration can declare different external publishers than a robot?
   (Encore precedent: keep the app model in code, environments in config —
   which argues for a separate environments file over conductor.json
   sections.)
5. Services/actions API shape: `Svc[Req,Res]` with `OnField(Req) (Res,
   error)` is the obvious mirror; actions need goal/feedback/result and
   cancellation — design against Nav2's action servers as the reference
   consumer.
