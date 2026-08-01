package conductor

import (
	"fmt"
	"reflect"
	"sync"
	"time"
)

// TestApp runs an application in-process for tests: the same runtime Run
// uses, on the in-process transport, without flag parsing or signal
// handling. Timers do not tick on their own unless asked, so a test decides
// when time passes.
//
// This is the low-level handle. Tests normally use the conductortest
// package, which wraps it with typed publish/record/call helpers and
// testing.TB integration.
type TestApp struct {
	a *app

	// Tests drive an app from several goroutines (a probe here, an action
	// client there), so the harness's own state is locked.
	mu     sync.Mutex
	probes []*nodeRuntime
	closed bool
}

// runtimes returns every executor the harness knows about, or nil once the
// app has been closed.
func (ta *TestApp) runtimes() []*nodeRuntime {
	ta.mu.Lock()
	defer ta.mu.Unlock()
	if ta.closed {
		return nil
	}
	return append(append([]*nodeRuntime{}, ta.a.rt.nodes...), ta.probes...)
}

// TestOptions configures NewTestApp. The zero value gives the common case:
// every node wired, brought up to Active, with manual timers.
type TestOptions struct {
	// Params seeds parameter values as a parameter file would, keyed by
	// node name then parameter name, with values in the same textual form.
	Params map[string]map[string]string
	// RealTimers lets timers tick from the wall clock instead of firing
	// only from Tick. Use it for timing-dependent tests; prefer Tick.
	RealTimers bool
	// ManualLifecycle leaves nodes Unconfigured so the test drives the
	// transitions itself. By default NewTestApp brings them up to Active.
	ManualLifecycle bool
	// Only wires just the named node, like Run's -node flag.
	Only string
	// Frames is the transform tree the app runs with, as frames.json would
	// supply at runtime. Use conductor.LoadFrames to read the real one.
	Frames *FrameTree
	// Semantics are the planning groups, as groups.json would supply at
	// runtime. Use conductor.LoadSemantics to read the real ones.
	Semantics *Semantics
	// Description is the robot's URDF text, as -description would supply at
	// runtime; it is published on /robot_description when this application
	// owns the transform tree.
	Description string
}

// NewTestApp wires nodes and (unless ManualLifecycle) brings them up.
// Call Close when done.
func NewTestApp(opts TestOptions, nodes ...any) (*TestApp, error) {
	a, err := newAppOpts(appOptions{
		transport:    "inproc",
		only:         opts.Only,
		params:       opts.Params,
		manualTimers: !opts.RealTimers,
		frames:       opts.Frames,
		semantics:    opts.Semantics,
		description:  opts.Description,
	}, nodes...)
	if err != nil {
		return nil, err
	}
	ta := &TestApp{a: a}
	if !opts.ManualLifecycle {
		if err := a.bringUp(); err != nil {
			ta.Close()
			return nil, err
		}
	}
	return ta, nil
}

// Transport returns the in-process transport the app is wired to, so a test
// can declare its own publishers, subscriptions and service clients against
// it.
func (ta *TestApp) Transport() Transport { return ta.a.rt.transport }

// Nodes returns the names of the wired nodes.
func (ta *TestApp) Nodes() []string {
	names := make([]string, len(ta.a.rt.nodes))
	for i, nr := range ta.a.rt.nodes {
		names[i] = nr.name
	}
	return names
}

func (ta *TestApp) node(name string) (*nodeRuntime, error) {
	for _, nr := range ta.a.rt.nodes {
		if nr.name == name {
			return nr, nil
		}
	}
	return nil, fmt.Errorf("conductor: no node %q in this app (have %v)", name, ta.Nodes())
}

// BindProbe wires ptr — a pointer to a struct with conductor field
// declarations — into the running app under the given name, as if it were
// another node, except that it has no lifecycle: its publishers and handlers
// are live immediately.
//
// It is the escape hatch for endpoints the typed helpers do not cover, most
// of all action clients:
//
//	var probe struct {
//	    Nav conductor.ActionClient[Goal, Feedback, Result] `action:"navigate"`
//	}
//	app.BindProbe("test_driver", &probe)
//	h, err := probe.Nav.SendGoal(Goal{...})
func (ta *TestApp) BindProbe(name string, ptr any) error {
	v := reflect.ValueOf(ptr)
	if v.Kind() != reflect.Pointer || v.Elem().Kind() != reflect.Struct {
		return fmt.Errorf("conductor: probe must be a pointer to a struct, got %T", ptr)
	}
	if err := ta.a.rt.transport.DeclareNode(name); err != nil {
		return err
	}
	nr := newNodeRuntime(name)
	if err := bindFields(ta.a.rt, nr, v); err != nil {
		return fmt.Errorf("probe %s: %w", name, err)
	}
	go nr.run()
	ta.mu.Lock()
	ta.probes = append(ta.probes, nr)
	ta.mu.Unlock()
	return nil
}

// Tick fires every timer of the named node once, on that node's executor,
// and waits for the work it causes to be processed. It reports how many
// timers fired — zero if the node has none, or if it is not Active (an
// inactive node's timers do not fire, exactly as in production).
func (ta *TestApp) Tick(node string) (int, error) {
	nr, err := ta.node(node)
	if err != nil {
		return 0, err
	}
	if !nr.active() {
		return 0, nil
	}
	n := 0
	for _, th := range ta.a.rt.timers {
		if th.node != nr {
			continue
		}
		th.node.enqueue(th.fire)
		n++
	}
	ta.Settle()
	return n, nil
}

// Settle waits until every node executor has drained: it puts a barrier at
// the back of each mailbox, waits for it, and repeats while that round did
// any real work — messages published from a callback land in another node's
// mailbox, so quiescence takes as many rounds as the graph is deep.
//
// Work started outside the executors (an action handler's goroutine, a real
// timer) can still arrive afterwards; that is what Await is for.
func (ta *TestApp) Settle() {
	runtimes := ta.runtimes()
	for round := 0; round < 100; round++ {
		before := uint64(0)
		for _, nr := range runtimes {
			before += nr.processed.Load()
		}
		barriers := 0
		for _, nr := range runtimes {
			done := make(chan struct{})
			if !enqueueBlocking(nr, func() { close(done) }) {
				continue // node is shutting down
			}
			barriers++
			select {
			case <-done:
			case <-nr.done:
			}
		}
		after := uint64(0)
		for _, nr := range runtimes {
			after += nr.processed.Load()
		}
		// Every executed callback counts one, including our barriers; if
		// that is all that ran, nothing is left in flight.
		if after-before <= uint64(barriers) {
			return
		}
	}
}

// enqueueBlocking retries until the mailbox has room, since a test barrier
// must not be dropped the way a message would be.
func enqueueBlocking(nr *nodeRuntime, fn func()) bool {
	for i := 0; i < 1000; i++ {
		select {
		case <-nr.quit:
			return false
		default:
		}
		if nr.enqueue(fn) {
			return true
		}
		time.Sleep(time.Millisecond)
	}
	return false
}

// SetParam updates a parameter the way `ros2 param set` would, taking the
// value in the same textual form parameter files use.
func (ta *TestApp) SetParam(node, name, value string) error {
	nr, err := ta.node(node)
	if err != nil {
		return err
	}
	for _, h := range nr.params {
		if h.name == name {
			return h.set(value)
		}
	}
	return fmt.Errorf("conductor: node %q has no parameter %q", node, name)
}

// Param returns a parameter's current value.
func (ta *TestApp) Param(node, name string) (any, error) {
	nr, err := ta.node(node)
	if err != nil {
		return nil, err
	}
	for _, h := range nr.params {
		if h.name == name {
			return h.get(), nil
		}
	}
	return nil, fmt.Errorf("conductor: node %q has no parameter %q", node, name)
}

// State returns a node's lifecycle state.
func (ta *TestApp) State(node string) (State, error) {
	nr, err := ta.node(node)
	if err != nil {
		return StateUnknown, err
	}
	return nr.lifecycle.State(), nil
}

// Transition drives one lifecycle transition on a node, like
// `ros2 lifecycle set`.
func (ta *TestApp) Transition(node string, t Transition) error {
	nr, err := ta.node(node)
	if err != nil {
		return err
	}
	if ok, err := nr.lifecycle.transition(t); !ok {
		return fmt.Errorf("node %s: %s: %w", node, t, err)
	}
	ta.Settle()
	return nil
}

// Mission returns a node's mission status and current step.
//
// A mission runs on its own goroutine, so unlike Publish and Tick it is not
// covered by Settle: use AwaitMission to wait for it.
func (ta *TestApp) Mission(node string) (MissionStatus, string, error) {
	nr, err := ta.node(node)
	if err != nil {
		return MissionIdle, "", err
	}
	if nr.mission == nil {
		return MissionIdle, "", fmt.Errorf("conductor: node %q declares no mission", node)
	}
	s := nr.mission.snapshot()
	return s.Status, s.Step, nil
}

// AwaitMission waits for a node's mission to reach a status, reporting the
// status and step it was actually in if the timeout expires first.
func (ta *TestApp) AwaitMission(node string, want MissionStatus, timeout time.Duration) error {
	deadline := time.Now().Add(timeout)
	for {
		got, step, err := ta.Mission(node)
		if err != nil {
			return err
		}
		if got == want {
			return nil
		}
		if time.Now().After(deadline) {
			return fmt.Errorf("conductor: mission of %s is %s at step %s after %s, want %s",
				node, got, step, timeout, want)
		}
		time.Sleep(time.Millisecond)
	}
}

// CancelMission stops a node's mission, as leaving the Active state would.
func (ta *TestApp) CancelMission(node string) error {
	nr, err := ta.node(node)
	if err != nil {
		return err
	}
	if nr.mission == nil {
		return fmt.Errorf("conductor: node %q declares no mission", node)
	}
	nr.mission.stop(time.Second)
	return nil
}

// Close shuts the application down: lifecycle hooks run in reverse bringup
// order, then executors and the transport stop.
func (ta *TestApp) Close() {
	ta.mu.Lock()
	if ta.closed {
		ta.mu.Unlock()
		return
	}
	ta.closed = true
	probes := ta.probes
	ta.probes = nil
	ta.mu.Unlock()

	for _, nr := range probes {
		close(nr.quit)
		<-nr.done
	}
	ta.a.stop()
}
