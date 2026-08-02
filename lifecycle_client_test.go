package conductor

import (
	"context"
	"errors"
	"strings"
	"testing"
	"time"
)

// The nodes being managed are deliberately dull: what is under test is the
// protocol between a manager and a managed node, which is the same protocol
// whether the peer is this runtime, rclcpp, or Nav2.
type managedAlpha struct {
	Tick  Timer `rate:"1hz"`
	calls []string
}

func (m *managedAlpha) OnTick()             {}
func (m *managedAlpha) OnConfigure() error  { m.calls = append(m.calls, "configure"); return nil }
func (m *managedAlpha) OnActivate() error   { m.calls = append(m.calls, "activate"); return nil }
func (m *managedAlpha) OnDeactivate() error { m.calls = append(m.calls, "deactivate"); return nil }

type managedBeta struct {
	Tick Timer `rate:"1hz"`
}

func (m *managedBeta) OnTick() {}

// stackManager is what replaces a lifecycle_manager: a declared list, in the
// order it should be brought up.
type stackManager struct {
	Stack Lifecycle `nodes:"managed_alpha,managed_beta" timeout:"2s"`
}

// manualApp wires nodes without bringing them up, so the manager does it.
func manualApp(t *testing.T, nodes ...any) (*TestApp, *stackManager) {
	t.Helper()
	ta, err := NewTestApp(TestOptions{ManualLifecycle: true}, nodes...)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(ta.Close)

	mgr := &stackManager{}
	if err := ta.BindProbe("manager", mgr); err != nil {
		t.Fatal(err)
	}
	return ta, mgr
}

func stateOf(t *testing.T, ta *TestApp, node string) State {
	t.Helper()
	s, err := ta.State(node)
	if err != nil {
		t.Fatal(err)
	}
	return s
}

// BringUp is a lifecycle_manager: configure everything in declared order, then
// activate everything in declared order.
func TestLifecycleClientBringsAStackUp(t *testing.T) {
	alpha := &managedAlpha{}
	ta, mgr := manualApp(t, alpha, &managedBeta{})

	if got := stateOf(t, ta, "managed_alpha"); got != StateUnconfigured {
		t.Fatalf("managed_alpha starts %s, want unconfigured", got)
	}
	if err := mgr.Stack.BringUp(context.Background()); err != nil {
		t.Fatal(err)
	}
	for _, node := range []string{"managed_alpha", "managed_beta"} {
		if got := stateOf(t, ta, node); got != StateActive {
			t.Errorf("%s is %s after BringUp, want active", node, got)
		}
	}
	// The managed node's own hooks ran, in the order the protocol requires.
	if got := strings.Join(alpha.calls, ","); got != "configure,activate" {
		t.Errorf("hooks ran %q, want configure,activate", got)
	}
	if err := mgr.Stack.AwaitActive(context.Background(), time.Second); err != nil {
		t.Errorf("AwaitActive after BringUp: %v", err)
	}
	if left := mgr.Stack.NotActive(); len(left) != 0 {
		t.Errorf("NotActive = %v after a successful bringup", left)
	}
}

// The dashboard asks what the stack is doing, and asking must not mean six
// service calls a second — the client reports what it last saw, and says so
// when it has not looked yet.
func TestLifecycleClientSummarisesTheStack(t *testing.T) {
	_, mgr := manualApp(t, &managedAlpha{}, &managedBeta{})

	if got := mgr.Stack.summary(); got != "2 node(s), not yet asked" {
		t.Errorf("summary before any call = %q", got)
	}
	if err := mgr.Stack.BringUp(context.Background()); err != nil {
		t.Fatal(err)
	}
	// BringUp never calls get_state on a node it drove all the way up, so this
	// is the transition results being remembered, not a fresh query.
	if got := mgr.Stack.summary(); got != "2 node(s), all active" {
		t.Errorf("summary after BringUp = %q", got)
	}
	if err := mgr.Stack.Deactivate("managed_beta"); err != nil {
		t.Fatal(err)
	}
	if got := mgr.Stack.summary(); got != "1 of 2 node(s) active, one is inactive" {
		t.Errorf("summary after one node stood down = %q", got)
	}
}

// A mission step declaring retry:"3" re-runs BringUp after a partial failure,
// so it has to be safe to call again: nodes already up are left alone rather
// than driven through a transition they cannot make.
func TestLifecycleClientBringUpIsRepeatable(t *testing.T) {
	alpha := &managedAlpha{}
	_, mgr := manualApp(t, alpha, &managedBeta{})

	if err := mgr.Stack.BringUp(context.Background()); err != nil {
		t.Fatal(err)
	}
	if err := mgr.Stack.BringUp(context.Background()); err != nil {
		t.Fatalf("second BringUp: %v", err)
	}
	if got := strings.Join(alpha.calls, ","); got != "configure,activate" {
		t.Errorf("hooks ran %q on the second pass, want the node left alone", got)
	}
}

// Teardown runs the other way round, so a provider stops after the nodes that
// depend on it.
func TestLifecycleClientDeactivatesInReverse(t *testing.T) {
	ta, mgr := manualApp(t, &managedAlpha{}, &managedBeta{})
	if err := mgr.Stack.BringUp(context.Background()); err != nil {
		t.Fatal(err)
	}

	if err := mgr.Stack.Deactivate(); err != nil {
		t.Fatal(err)
	}
	for _, node := range []string{"managed_alpha", "managed_beta"} {
		if got := stateOf(t, ta, node); got != StateInactive {
			t.Errorf("%s is %s after Deactivate, want inactive", node, got)
		}
	}

	// Cleanup takes them the rest of the way back.
	if err := mgr.Stack.Cleanup(); err != nil {
		t.Fatal(err)
	}
	if got := stateOf(t, ta, "managed_alpha"); got != StateUnconfigured {
		t.Errorf("managed_alpha is %s after Cleanup, want unconfigured", got)
	}
}

// One node's state is one call, and a state it cannot make is refused by the
// node rather than assumed by the manager.
func TestLifecycleClientTransitionAndState(t *testing.T) {
	_, mgr := manualApp(t, &managedAlpha{}, &managedBeta{})

	if got, err := mgr.Stack.State("managed_alpha"); err != nil || got != StateUnconfigured {
		t.Fatalf("State = %s, %v; want unconfigured", got, err)
	}
	// Activating an unconfigured node is not a legal transition, and the
	// managed node is the one that says so.
	if err := mgr.Stack.Transition("managed_alpha", TransitionActivate); err == nil {
		t.Error("activating an unconfigured node was accepted")
	}
	if err := mgr.Stack.Transition("managed_alpha", TransitionConfigure); err != nil {
		t.Fatal(err)
	}
	if got, _ := mgr.Stack.State("managed_alpha"); got != StateInactive {
		t.Errorf("State = %s after configure, want inactive", got)
	}

	// A node this field does not declare is a mistake worth naming, not a
	// silent no-op.
	if err := mgr.Stack.Transition("nowhere", TransitionConfigure); err == nil ||
		!strings.Contains(err.Error(), "declared nodes") {
		t.Errorf("driving an undeclared node: %v", err)
	}
}

// A node nobody is answering for reports unknown rather than failing the whole
// question, and AwaitActive says which ones are missing — the report that makes
// a bringup failure diagnosable.
func TestLifecycleClientReportsWhoIsNotUp(t *testing.T) {
	ta, err := NewTestApp(TestOptions{ManualLifecycle: true}, &managedAlpha{})
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(ta.Close)

	// managed_beta is declared but not in this application at all.
	mgr := &stackManager{}
	if err := ta.BindProbe("manager", mgr); err != nil {
		t.Fatal(err)
	}

	states := mgr.Stack.States()
	if states["managed_alpha"] != StateUnconfigured {
		t.Errorf("managed_alpha = %s, want unconfigured", states["managed_alpha"])
	}
	if states["managed_beta"] != StateUnknown {
		t.Errorf("managed_beta = %s, want unknown for a node nobody serves", states["managed_beta"])
	}

	err = mgr.Stack.AwaitActive(context.Background(), 50*time.Millisecond)
	if err == nil {
		t.Fatal("AwaitActive succeeded with a node missing")
	}
	var notActive *ErrNotActive
	if !errors.As(err, &notActive) {
		t.Fatalf("error is %T, want *ErrNotActive", err)
	}
	if !strings.Contains(err.Error(), "managed_beta is unknown") {
		t.Errorf("error does not name the missing node: %v", err)
	}
	if left := mgr.Stack.NotActive(); len(left) != 2 {
		t.Errorf("NotActive = %v, want both nodes", left)
	}
}

// The nodes tag is a declaration like any other, so its mistakes are wiring
// errors rather than runtime surprises.
func TestLifecycleClientTagIsValidated(t *testing.T) {
	cases := map[string]any{
		"no nodes tag": &struct{ Stack Lifecycle }{},
		"empty list": &struct {
			Stack Lifecycle `nodes:""`
		}{},
		"empty name": &struct {
			Stack Lifecycle `nodes:"alpha,,beta"`
		}{},
		"listed twice": &struct {
			Stack Lifecycle `nodes:"alpha,alpha"`
		}{},
		"bad timeout": &struct {
			Stack Lifecycle `nodes:"alpha" timeout:"soon"`
		}{},
		"zero timeout": &struct {
			Stack Lifecycle `nodes:"alpha" timeout:"0s"`
		}{},
	}
	for name, probe := range cases {
		ta, err := NewTestApp(TestOptions{ManualLifecycle: true}, &managedAlpha{})
		if err != nil {
			t.Fatal(err)
		}
		if err := ta.BindProbe("manager", probe); err == nil {
			t.Errorf("%s: wired without complaint", name)
		}
		ta.Close()
	}
}
