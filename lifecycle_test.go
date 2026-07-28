package conductor

import (
	"errors"
	"os"
	"strings"
	"sync/atomic"
	"testing"
	"time"

	"conductor.dev/conductor/internal/msggen"
)

// Verifies the hardcoded lifecycle wire hashes against the installed ROS
// distro (skipped when none is present).
func TestLifecycleWireHashes(t *testing.T) {
	share := "/opt/ros/lyrical/share"
	if _, err := os.Stat(share); err != nil {
		t.Skip("no ROS distro installed")
	}
	r := msggen.NewResolver([]string{share})
	for name, want := range map[string]string{
		"lifecycle_msgs/msg/TransitionEvent":         transitionEventHash,
		"lifecycle_msgs/srv/ChangeState":             changeStateHash,
		"lifecycle_msgs/srv/GetState":                getStateHash,
		"lifecycle_msgs/srv/GetAvailableStates":      getAvailableStatesHash,
		"lifecycle_msgs/srv/GetAvailableTransitions": getAvailableTransitionsHash,
	} {
		td, err := r.Describe(name)
		if err != nil {
			t.Fatalf("%s: %v", name, err)
		}
		if got := td.Hash(); got != want {
			t.Errorf("%s:\n got %s\nwant hardcoded %s", name, got, want)
		}
	}
}

// TestBringupOrder checks the ordering derived from a small pipeline:
// sensor publishes /scan, filter consumes it and publishes /clean, planner
// consumes /clean and calls the map service that mapper serves.
func TestBringupOrder(t *testing.T) {
	deps := map[string]*nodeDeps{
		"planner": {provides: map[string]bool{}, consumes: map[string]bool{"clean": true, "get_map": true}},
		"filter":  {provides: map[string]bool{"clean": true}, consumes: map[string]bool{"scan": true}},
		"sensor":  {provides: map[string]bool{"scan": true}, consumes: map[string]bool{}},
		"mapper":  {provides: map[string]bool{"get_map": true}, consumes: map[string]bool{}},
	}
	nodes := []string{"planner", "filter", "sensor", "mapper"}
	order, cycles := BringupOrder(nodes, deps)
	if len(cycles) != 0 {
		t.Fatalf("unexpected cycles: %v", cycles)
	}
	pos := map[string]int{}
	for i, n := range order {
		pos[n] = i
	}
	for _, pair := range [][2]string{{"sensor", "filter"}, {"filter", "planner"}, {"mapper", "planner"}} {
		if pos[pair[0]] > pos[pair[1]] {
			t.Errorf("%s must come up before %s, got order %v", pair[0], pair[1], order)
		}
	}
}

// Feedback loops are normal in robotics, so a cycle must not deadlock
// bringup — the nodes involved are reported and started anyway.
func TestBringupOrderTolerateCycle(t *testing.T) {
	deps := map[string]*nodeDeps{
		"a": {provides: map[string]bool{"x": true}, consumes: map[string]bool{"y": true}},
		"b": {provides: map[string]bool{"y": true}, consumes: map[string]bool{"x": true}},
		"c": {provides: map[string]bool{}, consumes: map[string]bool{}},
	}
	order, cycles := BringupOrder([]string{"a", "b", "c"}, deps)
	if len(order) != 3 {
		t.Fatalf("order = %v, want all three nodes", order)
	}
	if len(cycles) != 2 {
		t.Errorf("cycles = %v, want a and b", cycles)
	}
}

type hookRecorder struct {
	Tick Timer `rate:"1000hz"`

	events atomic.Int64
	log    []string
	ticks  atomic.Int64
	failOn string
}

func (h *hookRecorder) OnTick() { h.ticks.Add(1) }

func (h *hookRecorder) record(name string) error {
	h.log = append(h.log, name)
	h.events.Add(1)
	if h.failOn == name {
		return errors.New("hook failed on purpose")
	}
	return nil
}

func (h *hookRecorder) OnConfigure() error  { return h.record("configure") }
func (h *hookRecorder) OnActivate() error   { return h.record("activate") }
func (h *hookRecorder) OnDeactivate() error { return h.record("deactivate") }
func (h *hookRecorder) OnCleanup() error    { return h.record("cleanup") }
func (h *hookRecorder) OnShutdown() error   { return h.record("shutdown") }

func TestLifecycleHooksAndGating(t *testing.T) {
	n := &hookRecorder{}
	a, err := newApp("inproc", TransportOptions{}, "", n)
	if err != nil {
		t.Fatal(err)
	}
	nr := a.rt.nodes[0]

	// Before bringup the node is unconfigured and its timer must not fire.
	if got := nr.lifecycle.State(); got != StateUnconfigured {
		t.Fatalf("initial state = %s, want unconfigured", got)
	}
	time.Sleep(30 * time.Millisecond)
	if n.ticks.Load() != 0 {
		t.Errorf("timer fired %d times while unconfigured", n.ticks.Load())
	}

	if err := a.bringUp(); err != nil {
		t.Fatal(err)
	}
	if got := nr.lifecycle.State(); got != StateActive {
		t.Fatalf("state after bringup = %s, want active", got)
	}
	time.Sleep(30 * time.Millisecond)
	if n.ticks.Load() == 0 {
		t.Error("timer did not fire while active")
	}

	// Deactivating stops the timer again.
	if ok, err := nr.lifecycle.transition(TransitionDeactivate); !ok {
		t.Fatal(err)
	}
	time.Sleep(10 * time.Millisecond)
	before := n.ticks.Load()
	time.Sleep(30 * time.Millisecond)
	if after := n.ticks.Load(); after != before {
		t.Errorf("timer fired %d times while inactive", after-before)
	}

	a.stop()
	if got := strings.Join(n.log, ","); got != "configure,activate,deactivate,shutdown" {
		t.Errorf("hook sequence = %q", got)
	}
}

func TestLifecycleInvalidTransition(t *testing.T) {
	a, err := newApp("inproc", TransportOptions{}, "", &hookRecorder{})
	if err != nil {
		t.Fatal(err)
	}
	defer a.stop()
	// Activate is not valid from Unconfigured.
	if ok, err := a.rt.nodes[0].lifecycle.transition(TransitionActivate); ok || err == nil {
		t.Fatalf("activate from unconfigured should fail, got ok=%v err=%v", ok, err)
	}
}

func TestLifecycleHookFailureAborts(t *testing.T) {
	n := &hookRecorder{failOn: "activate"}
	a, err := newApp("inproc", TransportOptions{}, "", n)
	if err != nil {
		t.Fatal(err)
	}
	defer a.stop()
	if err := a.bringUp(); err == nil {
		t.Fatal("bringup should fail when a hook fails")
	}
	// A failed activation leaves the node in its previous state.
	if got := a.rt.nodes[0].lifecycle.State(); got != StateInactive {
		t.Errorf("state after failed activate = %s, want inactive", got)
	}
}

// The lifecycle services are the ROS-facing surface; drive them the way
// `ros2 lifecycle` does.
func TestLifecycleServices(t *testing.T) {
	a := newTestApp(t, &hookRecorder{})
	tr := a.rt.transport

	getState, err := tr.ServiceClient(ServiceSpec{Service: "hook_recorder/get_state", Node: "test"})
	if err != nil {
		t.Fatal(err)
	}
	changeState, err := tr.ServiceClient(ServiceSpec{Service: "hook_recorder/change_state", Node: "test"})
	if err != nil {
		t.Fatal(err)
	}

	res, err := getState(getStateRequest{}, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	if got := res.(getStateResponse).CurrentState; got.Id != uint8(StateActive) || got.Label != "active" {
		t.Fatalf("get_state = %+v, want active", got)
	}

	out, err := changeState(changeStateRequest{Transition: TransitionDeactivate.msg()}, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	if !out.(changeStateResponse).Success {
		t.Fatal("deactivate via change_state failed")
	}
	res, err = getState(getStateRequest{}, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	if got := res.(getStateResponse).CurrentState.Label; got != "inactive" {
		t.Fatalf("state after deactivate = %s", got)
	}

	// An invalid transition is refused, not fatal.
	out, err = changeState(changeStateRequest{Transition: TransitionDeactivate.msg()}, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	if out.(changeStateResponse).Success {
		t.Error("second deactivate should be refused")
	}
}
