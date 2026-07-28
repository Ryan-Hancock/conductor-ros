package conductor

import (
	"fmt"
	"log/slog"
	"reflect"
	"sync"
	"time"
)

// Conductor nodes are ROS 2 managed nodes. Each exposes the standard
// lifecycle services (change_state, get_state, get_available_states,
// get_available_transitions) and the transition_event topic, so `ros2
// lifecycle` and external orchestrators drive them like any other managed
// node.
//
// A node may implement any of these hooks, found by name like other
// callbacks; each runs on the node's executor and aborts the transition if
// it returns an error:
//
//	func (n *Node) OnConfigure() error
//	func (n *Node) OnActivate() error
//	func (n *Node) OnDeactivate() error
//	func (n *Node) OnCleanup() error
//	func (n *Node) OnShutdown() error
//
// While a node is not active its timers do not fire, its publishers drop
// messages, and its subscription handlers are not invoked — the behaviour
// the managed-node design prescribes.

// lifecycleHooks holds the optional transition callbacks a node implements.
type lifecycleHooks struct {
	configure  func() error
	activate   func() error
	deactivate func() error
	cleanup    func() error
	shutdown   func() error
}

func findHooks(ownerPtr reflect.Value) (lifecycleHooks, error) {
	var h lifecycleHooks
	for name, dst := range map[string]*func() error{
		"OnConfigure":  &h.configure,
		"OnActivate":   &h.activate,
		"OnDeactivate": &h.deactivate,
		"OnCleanup":    &h.cleanup,
		"OnShutdown":   &h.shutdown,
	} {
		m := ownerPtr.MethodByName(name)
		if !m.IsValid() {
			continue
		}
		fn, ok := m.Interface().(func() error)
		if !ok {
			return h, fmt.Errorf("lifecycle hook %s must have signature func() error", name)
		}
		*dst = fn
	}
	return h, nil
}

func (h lifecycleHooks) forTransition(t Transition) func() error {
	switch t {
	case TransitionConfigure:
		return h.configure
	case TransitionActivate:
		return h.activate
	case TransitionDeactivate:
		return h.deactivate
	case TransitionCleanup:
		return h.cleanup
	case TransitionUnconfiguredShutdown, TransitionInactiveShutdown, TransitionActiveShutdown:
		return h.shutdown
	}
	return nil
}

// lifecycle is a node's managed-state machine.
type lifecycle struct {
	node  *nodeRuntime
	hooks lifecycleHooks

	mu    sync.Mutex
	state State

	eventPub func(any, Metadata) error
}

func newLifecycle(nr *nodeRuntime, hooks lifecycleHooks) *lifecycle {
	return &lifecycle{node: nr, hooks: hooks, state: StateUnconfigured}
}

// State returns the node's current primary state.
func (l *lifecycle) State() State {
	l.mu.Lock()
	defer l.mu.Unlock()
	return l.state
}

// transition drives one lifecycle transition, running the node's hook on its
// executor. It reports whether the transition succeeded.
func (l *lifecycle) transition(t Transition) (bool, error) {
	l.mu.Lock()
	start := l.state
	goal, ok := transitionTable[struct {
		from State
		t    Transition
	}{start, t}]
	if !ok {
		l.mu.Unlock()
		return false, fmt.Errorf("transition %s is not valid from state %s", t, start)
	}
	l.state = intermediateState[t]
	l.mu.Unlock()

	err := l.runHook(l.hooks.forTransition(t))

	l.mu.Lock()
	if err != nil {
		// A failed transition returns the node to where it came from; the
		// spec's ErrorProcessing state is reported in the event only.
		l.state = start
		l.mu.Unlock()
		l.publishEvent(t, start, stateErrorProcessing)
		return false, err
	}
	l.state = goal
	l.mu.Unlock()
	gauge("conductor_node_lifecycle_state", "node", l.node.name).Store(int64(goal))
	counter("conductor_lifecycle_transitions_total", "node", l.node.name, "transition", t.String()).Add(1)
	l.publishEvent(t, start, goal)
	// Individual transitions are routine; bringUp logs the overall order.
	slog.Debug("conductor: lifecycle transition", "node", l.node.name, "transition", t.String(), "state", goal.String())
	return true, nil
}

// runHook executes fn on the node's executor so hooks can touch node state
// like any other callback.
func (l *lifecycle) runHook(fn func() error) error {
	if fn == nil {
		return nil
	}
	done := make(chan error, 1)
	if !l.node.enqueue(func() { done <- fn() }) {
		return fmt.Errorf("node %s is not accepting work", l.node.name)
	}
	select {
	case err := <-done:
		return err
	case <-l.node.quit:
		return fmt.Errorf("node %s shut down during transition", l.node.name)
	}
}

func (l *lifecycle) publishEvent(t Transition, start, goal State) {
	if l.eventPub == nil {
		return
	}
	if err := l.eventPub(transitionEventMsg{
		Stamp:      time.Now(),
		Transition: t.msg(),
		StartState: start.msg(),
		GoalState:  goal.msg(),
	}, Metadata{}); err != nil {
		slog.Warn("conductor: transition event publish failed", "node", l.node.name, "err", err)
	}
}

// active reports whether callbacks and publishers should run.
func (l *lifecycle) active() bool { return l.State() == StateActive }

// bindLifecycle wires the node's lifecycle services and transition_event
// topic, so external tooling can drive it.
func bindLifecycle(rt *runtimeState, nr *nodeRuntime) error {
	l := nr.lifecycle
	reliableQoS, _ := QoSProfile("reliable")
	transientQoS, _ := QoSProfile("transient")

	pub, err := rt.transport.Publisher(TopicSpec{
		Topic: nr.name + "/transition_event", QoS: reliableQoS,
		Type: reflect.TypeFor[transitionEventMsg](), Node: nr.name,
	})
	if err != nil {
		return err
	}
	l.eventPub = pub
	_ = transientQoS

	serve := func(suffix string, req, res reflect.Type, handle func(any) (any, error)) error {
		return rt.transport.Serve(ServiceSpec{
			Service: nr.name + "/" + suffix, ReqType: req, ResType: res, Node: nr.name,
		}, handle)
	}

	if err := serve("change_state",
		reflect.TypeFor[changeStateRequest](), reflect.TypeFor[changeStateResponse](),
		func(reqAny any) (any, error) {
			req := reqAny.(changeStateRequest)
			ok, err := l.transition(Transition(req.Transition.Id))
			if err != nil {
				slog.Warn("conductor: lifecycle transition refused", "node", nr.name, "err", err)
			}
			return changeStateResponse{Success: ok}, nil
		}); err != nil {
		return err
	}

	if err := serve("get_state",
		reflect.TypeFor[getStateRequest](), reflect.TypeFor[getStateResponse](),
		func(any) (any, error) {
			s := l.State()
			return getStateResponse{CurrentState: s.msg()}, nil
		}); err != nil {
		return err
	}

	if err := serve("get_available_states",
		reflect.TypeFor[getAvailableStatesRequest](), reflect.TypeFor[getAvailableStatesResponse](),
		func(any) (any, error) {
			states := []stateMsg{
				StateUnknown.msg(), StateUnconfigured.msg(), StateInactive.msg(),
				StateActive.msg(), StateFinalized.msg(),
				stateConfiguring.msg(), stateCleaningUp.msg(), stateShuttingDown.msg(),
				stateActivating.msg(), stateDeactivating.msg(), stateErrorProcessing.msg(),
			}
			return getAvailableStatesResponse{AvailableStates: states}, nil
		}); err != nil {
		return err
	}

	return serve("get_available_transitions",
		reflect.TypeFor[getAvailableTransitionsRequest](), reflect.TypeFor[getAvailableTransitionsResponse](),
		func(any) (any, error) {
			current := l.State()
			var out []transitionDescriptionMsg
			for key, goal := range transitionTable {
				if key.from != current {
					continue
				}
				out = append(out, transitionDescriptionMsg{
					Transition: key.t.msg(),
					StartState: key.from.msg(),
					GoalState:  goal.msg(),
				})
			}
			return getAvailableTransitionsResponse{AvailableTransitions: out}, nil
		})
}
