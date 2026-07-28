// Package conductortest runs conductor applications inside `go test`.
//
// A test wires the real nodes on the in-process transport, feeds them
// messages, fires their timers, and asserts on what they publish — no ROS
// install, no launch file, no sleeping-and-hoping:
//
//	func TestNavigator(t *testing.T) {
//	    app := conductortest.Run(t, &Navigator{})
//	    cmd := conductortest.Watch[msgs.Twist](app, "cmd_vel")
//
//	    conductortest.Publish(app, "amcl_pose", msgs.PoseStamped{...})
//
//	    got := cmd.Await(t, time.Second)
//	    if got.Linear.X <= 0 { t.Fatal("expected to drive forward") }
//	}
//
// Timers do not tick on their own: call App.Tick to advance the node by one
// period, so a test is deterministic rather than timing-dependent. Publish
// and Tick both wait for the resulting callbacks to run before returning, so
// most assertions need no waiting at all.
package conductortest

import (
	"reflect"
	"sync"
	"testing"
	"time"

	"conductor.dev/conductor"
)

// Options configures the application under test; see conductor.TestOptions.
type Options = conductor.TestOptions

// App is a running application under test. It embeds *conductor.TestApp, so
// Tick, Settle, SetParam, Transition, State and BindProbe are available on
// it directly; the methods here add testing.TB integration.
type App struct {
	*conductor.TestApp

	tb testing.TB

	mu   sync.Mutex
	pubs map[string]func(any, conductor.Metadata) error
}

// Run wires nodes, brings them up, and registers cleanup with t. Anything
// that goes wrong wiring the app fails the test immediately.
func Run(tb testing.TB, nodes ...any) *App {
	tb.Helper()
	return RunWith(tb, Options{}, nodes...)
}

// RunWith is Run with parameter values, real timers, or manual lifecycle.
func RunWith(tb testing.TB, opts Options, nodes ...any) *App {
	tb.Helper()
	ta, err := conductor.NewTestApp(opts, nodes...)
	if err != nil {
		tb.Fatalf("conductortest: wiring the app: %v", err)
	}
	tb.Cleanup(ta.Close)
	return &App{TestApp: ta, tb: tb, pubs: map[string]func(any, conductor.Metadata) error{}}
}

// Tick fires every timer of the named node once and waits for the callbacks
// it causes. It fails the test if there is no such node.
func (a *App) Tick(node string) int {
	a.tb.Helper()
	n, err := a.TestApp.Tick(node)
	if err != nil {
		a.tb.Fatalf("conductortest: %v", err)
	}
	return n
}

// TickN fires the node's timers n times.
func (a *App) TickN(node string, n int) {
	a.tb.Helper()
	for i := 0; i < n; i++ {
		a.Tick(node)
	}
}

// SetParam updates a parameter, as `ros2 param set` would.
func (a *App) SetParam(node, name, value string) {
	a.tb.Helper()
	if err := a.TestApp.SetParam(node, name, value); err != nil {
		a.tb.Fatalf("conductortest: %v", err)
	}
}

// Param returns a parameter's current value.
func (a *App) Param(node, name string) any {
	a.tb.Helper()
	v, err := a.TestApp.Param(node, name)
	if err != nil {
		a.tb.Fatalf("conductortest: %v", err)
	}
	return v
}

// Transition drives a lifecycle transition, as `ros2 lifecycle set` would.
func (a *App) Transition(node string, t conductor.Transition) {
	a.tb.Helper()
	if err := a.TestApp.Transition(node, t); err != nil {
		a.tb.Fatalf("conductortest: %v", err)
	}
}

// State returns a node's lifecycle state.
func (a *App) State(node string) conductor.State {
	a.tb.Helper()
	s, err := a.TestApp.State(node)
	if err != nil {
		a.tb.Fatalf("conductortest: %v", err)
	}
	return s
}

// Probe wires extra conductor endpoints into the app under a node name of
// its own — the way to drive an action server, or anything else the helpers
// below do not cover. See conductor.TestApp.BindProbe.
func (a *App) Probe(name string, ptr any) {
	a.tb.Helper()
	if err := a.BindProbe(name, ptr); err != nil {
		a.tb.Fatalf("conductortest: %v", err)
	}
}

// Publish sends msg on topic as an outside publisher would, then waits for
// the subscribers' callbacks to run.
func Publish[T any](app *App, topic string, msg T) {
	app.tb.Helper()
	key := topic + "\x00" + reflect.TypeFor[T]().String()
	app.mu.Lock()
	publish, ok := app.pubs[key]
	if !ok {
		var err error
		publish, err = app.Transport().Publisher(conductor.TopicSpec{
			Topic: topic,
			Type:  reflect.TypeFor[T](),
			Node:  "conductortest",
		})
		if err != nil {
			app.mu.Unlock()
			app.tb.Fatalf("conductortest: publisher for %q: %v", topic, err)
		}
		app.pubs[key] = publish
	}
	app.mu.Unlock()

	if err := publish(msg, conductor.Metadata{}); err != nil {
		app.tb.Fatalf("conductortest: publishing to %q: %v", topic, err)
	}
	app.Settle()
}

// Call invokes a service the app serves and returns its response, or the
// error the handler returned.
func Call[Req, Res any](app *App, service string, req Req) (Res, error) {
	app.tb.Helper()
	var zero Res
	call, err := app.Transport().ServiceClient(conductor.ServiceSpec{
		Service: service,
		ReqType: reflect.TypeFor[Req](),
		ResType: reflect.TypeFor[Res](),
		Node:    "conductortest",
		Timeout: 5 * time.Second,
	})
	if err != nil {
		app.tb.Fatalf("conductortest: service client for %q: %v", service, err)
	}
	res, err := call(req, 5*time.Second)
	if err != nil {
		return zero, err
	}
	app.Settle()
	return res.(Res), nil
}

// Watch records every message published on a topic from the moment it is
// called. Start watching before the messages you care about are published.
func Watch[T any](app *App, topic string) *Recorder[T] {
	app.tb.Helper()
	r := &Recorder[T]{topic: topic, ch: make(chan T, 256)}
	err := app.Transport().Subscribe(conductor.TopicSpec{
		Topic: topic,
		Type:  reflect.TypeFor[T](),
		Node:  "conductortest",
	}, func(msg any, _ conductor.Metadata) {
		m, ok := msg.(T)
		if !ok {
			return // another type on the same topic; not this recorder's
		}
		r.mu.Lock()
		r.msgs = append(r.msgs, m)
		r.mu.Unlock()
		select {
		case r.ch <- m:
		default: // Await is not keeping up; All still has everything
		}
	})
	if err != nil {
		app.tb.Fatalf("conductortest: watching %q: %v", topic, err)
	}
	return r
}

// Recorder collects the messages published on one topic.
type Recorder[T any] struct {
	topic string
	ch    chan T

	mu   sync.Mutex
	msgs []T
}

// Len returns how many messages have been recorded.
func (r *Recorder[T]) Len() int {
	r.mu.Lock()
	defer r.mu.Unlock()
	return len(r.msgs)
}

// All returns every recorded message, oldest first.
func (r *Recorder[T]) All() []T {
	r.mu.Lock()
	defer r.mu.Unlock()
	return append([]T{}, r.msgs...)
}

// Last returns the most recent message, and whether there was one.
func (r *Recorder[T]) Last() (T, bool) {
	r.mu.Lock()
	defer r.mu.Unlock()
	if len(r.msgs) == 0 {
		var zero T
		return zero, false
	}
	return r.msgs[len(r.msgs)-1], true
}

// Await returns the next message not yet returned by Await, waiting up to
// timeout. It fails the test if none arrives — use Len or All to assert that
// nothing was published.
func (r *Recorder[T]) Await(tb testing.TB, timeout time.Duration) T {
	tb.Helper()
	select {
	case m := <-r.ch:
		return m
	case <-time.After(timeout):
		var zero T
		tb.Fatalf("conductortest: no message on %q within %s (%d recorded in total)", r.topic, timeout, r.Len())
		return zero
	}
}

// AwaitN returns the next n messages, waiting up to timeout in total.
func (r *Recorder[T]) AwaitN(tb testing.TB, n int, timeout time.Duration) []T {
	tb.Helper()
	out := make([]T, 0, n)
	deadline := time.After(timeout)
	for len(out) < n {
		select {
		case m := <-r.ch:
			out = append(out, m)
		case <-deadline:
			tb.Fatalf("conductortest: wanted %d messages on %q within %s, got %d", n, r.topic, timeout, len(out))
		}
	}
	return out
}

// Reset forgets everything recorded so far.
func (r *Recorder[T]) Reset() {
	r.mu.Lock()
	r.msgs = nil
	r.mu.Unlock()
	for {
		select {
		case <-r.ch:
		default:
			return
		}
	}
}
