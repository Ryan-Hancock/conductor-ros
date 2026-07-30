package conductor

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"reflect"
	"sort"
	"strings"
	"sync/atomic"
	"time"
)

// Lifecycle drives *other* managed nodes: it is the third use of the protocol
// this runtime already speaks as a server and already waits on during a fleet
// rollout.
//
// The ROS answer to bringing a stack up is a `lifecycle_manager` — a node whose
// entire job is to hold a list of node names and call change_state on each in
// order, configured by a parameter nobody validates. Nav2 ships one, and
// launches it with `autostart` so the ordering lives in a launch file. That is
// the same shape as every other piece of folklore this framework replaces: a
// list, an order, and no way to check either.
//
// Here the list is a declaration:
//
//	//conductor:node
//	type Commander struct {
//	    Stack conductor.Lifecycle `nodes:"map_server,amcl,controller_server,bt_navigator" timeout:"30s"`
//	}
//
// and bringing the stack up is one call from a mission step:
//
//	func (c *Commander) OnBringUp(t *conductor.Task) error {
//	    return c.Stack.BringUp(t.Context())
//	}
//
// Tags: nodes (required) — the managed nodes to drive, in bringup order;
// timeout (time.ParseDuration; default 10s) — per service call.
//
// Every method blocks on service calls, so drive a Lifecycle from a mission
// step or a goroutine, never from an executor callback.
type Lifecycle struct {
	nodes   []string
	timeout time.Duration

	change map[string]func(any, time.Duration) (any, error)
	state  map[string]func(any, time.Duration) (any, error)

	calls atomic.Uint64
}

// ErrNotActive reports managed nodes that did not reach Active.
type ErrNotActive struct {
	States map[string]State
}

func (e *ErrNotActive) Error() string {
	var parts []string
	for _, node := range sortedNodes(e.States) {
		parts = append(parts, fmt.Sprintf("%s is %s", node, e.States[node]))
	}
	return "not active: " + strings.Join(parts, ", ")
}

// Nodes returns the managed nodes this field drives, in declared order.
func (l *Lifecycle) Nodes() []string { return append([]string(nil), l.nodes...) }

// State asks one managed node what state it is in.
func (l *Lifecycle) State(node string) (State, error) {
	call, ok := l.state[node]
	if !ok {
		return StateUnknown, fmt.Errorf("conductor: %q is not one of this field's declared nodes (%s)",
			node, strings.Join(l.nodes, ", "))
	}
	l.calls.Add(1)
	res, err := call(getStateRequest{}, l.timeout)
	if err != nil {
		return StateUnknown, fmt.Errorf("%s/get_state: %w", node, err)
	}
	return State(res.(getStateResponse).CurrentState.Id), nil
}

// States is the state of every declared node. A node that cannot be reached
// reports StateUnknown rather than failing the whole call: "which of these is
// not up?" is the question being asked, and one silent node is the answer, not
// an error.
func (l *Lifecycle) States() map[string]State {
	out := make(map[string]State, len(l.nodes))
	for _, node := range l.nodes {
		state, err := l.State(node)
		if err != nil {
			slog.Debug("conductor: lifecycle get_state failed", "node", node, "err", err)
			state = StateUnknown
		}
		out[node] = state
	}
	return out
}

// NotActive lists the declared nodes that are not Active, in declared order —
// the report a bringup failure should carry.
func (l *Lifecycle) NotActive() []string {
	var out []string
	for _, node := range l.nodes {
		if state, err := l.State(node); err != nil || state != StateActive {
			out = append(out, node)
		}
	}
	return out
}

// Transition drives one transition on one node. A transition the node refuses
// is an error, and so is one it cannot make from its current state — the
// server decides, exactly as it would for `ros2 lifecycle set`.
func (l *Lifecycle) Transition(node string, t Transition) error {
	call, ok := l.change[node]
	if !ok {
		return fmt.Errorf("conductor: %q is not one of this field's declared nodes (%s)",
			node, strings.Join(l.nodes, ", "))
	}
	l.calls.Add(1)
	res, err := call(changeStateRequest{Transition: t.msg()}, l.timeout)
	if err != nil {
		return fmt.Errorf("%s/change_state %s: %w", node, t, err)
	}
	if !res.(changeStateResponse).Success {
		return fmt.Errorf("%s refused to %s", node, t)
	}
	return nil
}

// Configure configures the named nodes — or every declared node, in declared
// order, when called with none.
func (l *Lifecycle) Configure(nodes ...string) error {
	return l.each(TransitionConfigure, l.orAll(nodes))
}

// Activate activates the named nodes, or every declared node in order.
func (l *Lifecycle) Activate(nodes ...string) error {
	return l.each(TransitionActivate, l.orAll(nodes))
}

// Deactivate deactivates the named nodes, or every declared node in *reverse*
// order: teardown runs the other way round, so a provider stops after the
// nodes depending on it, the same rule the runtime's own shutdown follows.
func (l *Lifecycle) Deactivate(nodes ...string) error {
	return l.each(TransitionDeactivate, reversed(l.orAll(nodes)))
}

// Cleanup returns the named nodes (or all, in reverse order) to Unconfigured.
func (l *Lifecycle) Cleanup(nodes ...string) error {
	return l.each(TransitionCleanup, reversed(l.orAll(nodes)))
}

// Shutdown finalizes the named nodes (or all, in reverse order), choosing the
// shutdown transition valid from each node's current state.
func (l *Lifecycle) Shutdown(nodes ...string) error {
	var errs []error
	for _, node := range reversed(l.orAll(nodes)) {
		state, err := l.State(node)
		if err != nil {
			errs = append(errs, err)
			continue
		}
		t, ok := shutdownFor(state)
		if !ok {
			continue // already finalized, or mid-transition
		}
		if err := l.Transition(node, t); err != nil {
			errs = append(errs, err)
		}
	}
	return errors.Join(errs...)
}

// BringUp is what a lifecycle_manager does, with the order declared rather
// than parameterised: configure every node in turn, then activate every node
// in turn. Both passes run in declared order, and configuring everything
// before activating anything is deliberate — it is how Nav2's manager behaves,
// and it means a node that cannot configure is found before any of its peers
// starts publishing.
//
// A node already in the state a pass would reach is left alone, so BringUp is
// safe to retry: a mission step with `retry:"3"` re-runs it after a partial
// failure without tripping over the nodes that already came up.
func (l *Lifecycle) BringUp(ctx context.Context) error {
	for _, pass := range []struct {
		transition Transition
		from, to   State
	}{
		{TransitionConfigure, StateUnconfigured, StateInactive},
		{TransitionActivate, StateInactive, StateActive},
	} {
		for _, node := range l.nodes {
			if err := ctx.Err(); err != nil {
				return err
			}
			state, err := l.State(node)
			if err != nil {
				return err
			}
			switch state {
			case pass.to, StateActive:
				continue // already there
			case pass.from:
				if err := l.Transition(node, pass.transition); err != nil {
					return err
				}
			default:
				return fmt.Errorf("%s is %s, so it cannot %s", node, state, pass.transition)
			}
		}
	}
	return nil
}

// AwaitActive waits for every declared node to report Active, and says which
// ones did not when it gives up. It exists because a transition returning
// success and a stack being ready are different claims: a managed node may be
// driven by something other than this application.
func (l *Lifecycle) AwaitActive(ctx context.Context, timeout time.Duration) error {
	deadline := time.Now().Add(timeout)
	for {
		states := l.States()
		waiting := false
		for _, state := range states {
			if state != StateActive {
				waiting = true
				break
			}
		}
		if !waiting {
			return nil
		}
		if time.Now().After(deadline) {
			return &ErrNotActive{States: states}
		}
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-time.After(200 * time.Millisecond):
		}
	}
}

func (l *Lifecycle) each(t Transition, nodes []string) error {
	for _, node := range nodes {
		if err := l.Transition(node, t); err != nil {
			return err
		}
	}
	return nil
}

func (l *Lifecycle) orAll(nodes []string) []string {
	if len(nodes) == 0 {
		return l.nodes
	}
	return nodes
}

func (l *Lifecycle) bind(rt *runtimeState, nr *nodeRuntime, field reflect.StructField, ownerPtr reflect.Value) error {
	nodes, err := parseNodeList(field.Tag.Get("nodes"))
	if err != nil {
		return err
	}
	timeout := 10 * time.Second
	if tag := field.Tag.Get("timeout"); tag != "" {
		d, err := time.ParseDuration(tag)
		if err != nil || d <= 0 {
			return fmt.Errorf("invalid timeout %q", tag)
		}
		timeout = d
	}

	l.nodes = nodes
	l.timeout = timeout
	l.change = make(map[string]func(any, time.Duration) (any, error), len(nodes))
	l.state = make(map[string]func(any, time.Duration) (any, error), len(nodes))

	for _, node := range nodes {
		change, err := rt.transport.ServiceClient(ServiceSpec{
			Service: node + "/change_state",
			ReqType: reflect.TypeFor[changeStateRequest](),
			ResType: reflect.TypeFor[changeStateResponse](),
			Node:    nr.name,
			Timeout: timeout,
		})
		if err != nil {
			return err
		}
		state, err := rt.transport.ServiceClient(ServiceSpec{
			Service: node + "/get_state",
			ReqType: reflect.TypeFor[getStateRequest](),
			ResType: reflect.TypeFor[getStateResponse](),
			Node:    nr.name,
			Timeout: timeout,
		})
		if err != nil {
			return err
		}
		l.change[node] = change
		l.state[node] = state
	}

	// One endpoint for the field rather than two per node: the declaration is
	// "this node manages that set", and a dashboard listing twelve lifecycle
	// services would bury it.
	rt.recordEndpoint(Endpoint{Node: nr.name, Kind: EndpointLifecycle, Field: field.Name,
		Name: strings.Join(nodes, ","), Type: "lifecycle_msgs/srv/ChangeState",
		count: countOf(l.calls.Load)})
	return nil
}

// parseNodeList reads the nodes tag: a comma-separated list, in bringup order.
func parseNodeList(tag string) ([]string, error) {
	if strings.TrimSpace(tag) == "" {
		return nil, errors.New(`missing nodes tag (e.g. nodes:"map_server,amcl,bt_navigator")`)
	}
	var out []string
	seen := map[string]bool{}
	for _, raw := range strings.Split(tag, ",") {
		node := strings.TrimSpace(raw)
		if node == "" {
			return nil, fmt.Errorf("empty node name in nodes:%q", tag)
		}
		node = strings.TrimPrefix(node, "/")
		if seen[node] {
			return nil, fmt.Errorf("node %q is listed twice in nodes:%q", node, tag)
		}
		seen[node] = true
		out = append(out, node)
	}
	return out, nil
}

func reversed(in []string) []string {
	out := make([]string, len(in))
	for i, v := range in {
		out[len(in)-1-i] = v
	}
	return out
}

func sortedNodes(states map[string]State) []string {
	out := make([]string, 0, len(states))
	for node := range states {
		out = append(out, node)
	}
	sort.Strings(out)
	return out
}
