# Conductor

**Build ROS 2 applications, not ROS 2 plumbing.**

Conductor is an opinionated application framework for robotics, in Go, in the
spirit of [Encore](https://encore.dev): you declare *what your robot's nodes
do* in a small amount of typed, declarative code, and the framework derives
everything ROS 2 makes you hand-maintain — the communication graph, QoS
configuration, launch files, parameter files, lifecycle wiring, and
observability.

The core idea: **treat ROS 2 as a runtime, not a framework**, and make the
application graph a compile-time artifact. The canonical ROS failure — two
nodes silently not talking because of a QoS mismatch or a typo'd topic name,
discovered on the robot — becomes a build error.

## What it looks like

```go
//conductor:node
type Navigator struct {
    Pose     conductor.Sub[msgs.PoseStamped] `topic:"amcl_pose" qos:"reliable"`
    Cmd      conductor.Pub[msgs.Twist]       `topic:"cmd_vel"  qos:"reliable"`
    MaxSpeed conductor.Param[float64]        `name:"max_speed" default:"1.5"`
}

func (n *Navigator) OnPose(p msgs.PoseStamped) {
    // steer toward the goal; callbacks are single-threaded per node,
    // so plain struct fields are safe.
    n.Cmd.Publish(...)
}
```

```go
func main() {
    conductor.Run(&Localizer{}, &Navigator{}, &SafetyMonitor{})
}
```

From those declarations the `conductor` CLI statically derives the full topic
graph and checks it before anything runs:

```
$ conductor check examples/patrol
app patrol — 3 node(s), 3 topic(s), 2 external interface(s)
...
topics:
  amcl_pose   geometry_msgs/msg/PoseStamped   localizer -> navigator
  cmd_vel     geometry_msgs/msg/Twist         navigator -> safety_monitor,(external)
  estop       std_msgs/msg/Bool               (external) -> safety_monitor

✓ graph valid: 0 errors, 0 warning(s)
```

Wiring mistakes fail the build with file:line positions:

```
error CND013: topic "battery": subscriber requests reliable delivery but
              publisher offers best-effort (app.go:26)
error CND010: topic "lidar": subscribed by monitor but nothing publishes it
error CND003: node monitor: missing handler method OnEstop (app.go:27)
```

## Try it

```sh
go test ./...                                # runtime + scanner + graph tests
go run ./cmd/conductor check examples/patrol # validate the example app
go run ./cmd/conductor build examples/patrol # compile + emit gen/ artifacts
./examples/patrol/gen/bin/patrol             # run all nodes in-process (Ctrl-C to stop)
./examples/patrol/gen/bin/patrol -node navigator   # run a single node
```

`conductor build` emits, under `examples/patrol/gen/`:

- `patrol.launch.xml` — a ROS 2 launch file, one process per node
- `params.yaml` — ROS-style parameter file from the `Param` declarations
- `graph.dot` — Graphviz view of the topic graph
- `bin/patrol` — the compiled app (one static binary; cross-compile with
  `GOOS`/`GOARCH` like any Go program)

## Concepts

| Declaration | Meaning |
|---|---|
| `//conductor:node` on a struct | The struct is a node (name: snake_case of the type) |
| `conductor.Sub[T]` + `OnField(T)` | Subscription with typed handler |
| `conductor.Pub[T]` | Publisher |
| `conductor.Param[T]` | Node parameter with default |
| `conductor.Timer` + `OnField()` | Periodic callback (`rate:"10hz"` or `rate:"250ms"`) |
| `conductor.Svc[Req,Res]` + `OnField(Req) (Res, error)` | Service server (`service:"name"`) |
| `conductor.Client[Req,Res]` + `.Call(req)` | Service client (`service:"name"`, optional `timeout:"3s"`) |
| `conductor.Action[G,F,R]` + `OnField(*Goal[G,F]) (R, error)` | Action server (`action:"name"`); handler runs one goroutine per goal |
| `conductor.ActionClient[G,F,R]` + `.SendGoal(g)` | Action client (`action:"name"`, optional `timeout:"60s"`) |
| `//ros:type pkg/msg/Name` on a struct | Maps a Go message type to its ROS interface |
| `conductor.json` | App name + topics provided/consumed by external ROS nodes |

QoS is expressed as intent-level profiles (`reliable`, `sensor`, `transient`)
rather than raw DDS knobs; the checker validates every pub/sub pairing.

Callbacks run on a single goroutine per node (mirroring a ROS single-threaded
executor), so node-local state needs no locking.

## Joining a live ROS 2 graph (zenoh transport)

Conductor apps can join a real ROS 2 graph as first-class peers of
`rmw_zenoh` nodes: same key expressions, CDR payloads, attachments, and
liveliness tokens, so `ros2 node list`, `ros2 topic list/info/echo` all see
conductor nodes natively — no bridge, no ROS installation linked into the
binary.

```sh
source .tools/env.sh                        # zenoh-c paths + rmw_zenoh overlay
go build -tags zenoh -o bin/patrol ./examples/patrol
./bin/patrol -transport zenoh               # connects to the router at tcp/127.0.0.1:7447
```

Verified end-to-end against ROS 2 Lyrical with rmw_zenoh 0.10:
`ros2 topic pub /chatter` → conductor subscriber, and conductor's navigator →
`ros2 topic echo /cmd_vel`, with correct type/hash/QoS shown by
`ros2 topic info --verbose`.

The transport is selected at runtime (`-transport inproc|zenoh`, plus
`-zenoh-endpoint`, `-domain`); the zenoh implementation links zenoh-c via
cgo and is gated behind the `zenoh` build tag, so plain `go build` needs no
native dependencies.

## Custom messages (`conductor msggen`)

Define interfaces as plain ROS `.msg` files and generate Go from them —
including the REP-2011 RIHS01 type hash, which conductor computes itself
(validated against all 284 message types of a ROS 2 Lyrical install):

```sh
conductor msggen -out examples/patrol -pkg main -ros-pkg patrol_msgs examples/patrol/msg
```

emits structs with `//ros:type` directives (so `conductor check` sees them)
and `RegisterMessage` calls (so the zenoh transport can wire them). Standard
types referenced by your messages (e.g. `std_msgs/Header`) resolve via
`$AMENT_PREFIX_PATH`; `builtin_interfaces` Time/Duration map to Go
`time.Time`/`time.Duration`. Because the hash is computed locally, a custom
type needs no ROS package, no CMake, and no rebuild dance — `ros2 topic info
--verbose` on a live graph shows conductor's custom types with correct
type names and hashes.

## Services

Servers and clients are declared like everything else, with handlers that
run on the node's executor (state access is lock-free):

```go
Engage conductor.Svc[srvs.SetBoolRequest, srvs.SetBoolResponse] `service:"engage_estop"`

func (s *SafetyMonitor) OnEngage(req srvs.SetBoolRequest) (srvs.SetBoolResponse, error) { ... }
```

`conductor msggen` also parses `.srv` files — request/response structs plus
the service-level RIHS01 hash (which covers rosidl's synthesized `_Event`
type). Over zenoh, services are querier/queryable pairs in rmw_zenoh's wire
format, verified live in both directions: `ros2 service call` against a
conductor server, and a conductor `Client` calling an rclpy server. Graph
validation covers services too: a client whose service nothing serves, two
servers on one name, or a request/response type disagreement all fail
`conductor check`.

## Actions

An action server is one field and one method; conductor implements the full
ROS 2 action protocol (send_goal/cancel_goal/get_result services plus
feedback and status topics under `<name>/_action/`) on top of the transport:

```go
Fib conductor.Action[FibonacciGoal, FibonacciFeedback, FibonacciResult] `action:"fibonacci"`

func (f *FibServer) OnFib(g *conductor.Goal[FibonacciGoal, FibonacciFeedback]) (FibonacciResult, error) {
    for ... {
        select {
        case <-g.Context().Done():   // cancellation
            return partial, g.Context().Err()
        case <-time.After(step):
        }
        g.Feedback(...)
    }
    return result, nil
}
```

Handlers run on a dedicated goroutine per goal (they are long-running by
nature), with cancellation via the goal context; a non-nil error after
cancellation reports CANCELED with the partial result, otherwise ABORTED.
`conductor msggen` parses `.action` files (goal/result/feedback structs, all
derived-type hashes).

Calling an action — a conductor node driving Nav2, say — is the mirror
image:

```go
Fib conductor.ActionClient[FibonacciGoal, FibonacciFeedback, FibonacciResult] `action:"fibonacci" timeout:"60s"`

h, err := m.Fib.SendGoal(FibonacciGoal{Order: 8})   // blocks until accepted
go func() { for fb := range h.Feedback() { ... } }()
h.Cancel()                                          // optional
result, status, err := h.Result()                   // blocks until terminal
```

`SendGoal` returns `ErrGoalRejected` if the server refuses. `Result` reports
the terminal status (SUCCEEDED/CANCELED/ABORTED) alongside whatever result
the server sent, reserving `err` for transport failures. Because these calls
block, drive them from a goroutine — never directly from an executor
callback.

Both directions are covered by [`.tools/interop.sh`](.tools/interop.sh),
which runs conductor against real ROS 2 over rmw_zenoh: `ros2 service call`
and `ros2 action send_goal` into conductor servers, conductor clients into
both a conductor server and an rclpy server. See
[examples/fibonacci](examples/fibonacci/main.go) (server) and
[examples/mission](examples/mission/main.go) (client).

## Status

v0.6 — the static toolchain (scan → validate → generate) works; the runtime
executes nodes over a pluggable transport: in-process bus by default, or
Zenoh/rmw_zenoh to join a live ROS 2 graph (see above). CDR serialization is
pure Go, byte-verified against rclpy; `.msg`/`.srv`/`.action` codegen
computes RIHS01 hashes locally (validated 692/692 against a full distro).
Topics, services, and actions all work in both directions, on both
transports, verified against real ROS 2 by `.tools/interop.sh`. Not yet
done: lifecycle orchestration, transient-local latching, per-environment
config, and OpenTelemetry integration. See [DESIGN.md](DESIGN.md).

## Layout

- [conductor (root package)](run.go) — runtime: node wiring, executors, transport registry
- [cmd/conductor](cmd/conductor/main.go) — CLI: `check`, `graph`, `build`
- [internal/scan](internal/scan/scan.go) — syntactic scanner for directives and declarations
- [internal/graph](internal/graph/graph.go) — topic graph construction + validation rules
- [internal/gen](internal/gen/gen.go) — launch/params/dot generation
- [cdr](cdr/cdr.go) — pure-Go CDR (XCDR1-LE) codec, golden-tested against rclpy
- [transport/rmwzenoh](transport/rmwzenoh/rmwzenoh.go) — rmw_zenoh wire conventions (pure Go, golden-tested against live traffic)
- [transport/zenoh](transport/zenoh/zenoh.go) — the cgo Zenoh transport (`-tags zenoh`)
- [msgs](msgs/msgs.go) — hand-written common ROS message types + registered type hashes
- [examples/patrol](examples/patrol/nodes.go) — example application
- [examples/chatter](examples/chatter/main.go) — minimal ROS-interop example
