package conductor

import (
	"sync/atomic"
	"time"
)

// nodeRuntime is the executor for one node: a mailbox drained by a single
// goroutine, so node callbacks never run concurrently and node-local state
// needs no locking. A full mailbox drops the message (counted), mirroring
// QoS depth semantics rather than applying backpressure to publishers.
type nodeRuntime struct {
	name      string
	mailbox   chan func()
	quit      chan struct{}
	done      chan struct{}
	processed atomic.Uint64
	dropped   atomic.Uint64
	lifecycle *lifecycle
	params    []*paramHandle

	// mission and steps are the node's declared task machine, assembled
	// after binding (the Mission field and its Steps may be declared in any
	// order). onActive/onInactive are what the lifecycle starts and stops.
	mission    *missionRunner
	steps      []*stepDef
	onActive   []func()
	onInactive []func()

	// currentTrace is the trace context of the callback the executor is
	// running, so messages published from inside a callback become children
	// of it. Written only by the executor, but read by publishers on other
	// goroutines (an action handler, a mission step), so it is atomic.
	currentTrace atomic.Pointer[TraceContext]
}

// runInstrumented executes a callback on the executor with a span and
// metrics around it, exposing its trace context to anything the callback
// publishes.
func (nr *nodeRuntime) runInstrumented(kind SpanKind, name string, parent TraceContext, fn func()) {
	start := time.Now()
	var span *Span
	if tracingEnabled() {
		span = startSpan(nr.name, kind, name, parent)
		nr.currentTrace.Store(&span.Context)
	}
	fn()
	nr.currentTrace.Store(nil)
	span.finish(nil)
	observe("conductor_callback_duration", time.Since(start),
		"node", nr.name, "kind", string(kind), "name", name)
}

func newNodeRuntime(name string) *nodeRuntime {
	return &nodeRuntime{
		name:    name,
		mailbox: make(chan func(), 64),
		quit:    make(chan struct{}),
		done:    make(chan struct{}),
	}
}

// active reports whether the node is in the Active lifecycle state, and so
// whether its timers, publishers, and subscription handlers should run.
func (nr *nodeRuntime) active() bool {
	return nr.lifecycle == nil || nr.lifecycle.active()
}

// enqueue submits fn to the node's executor, reporting whether it was
// accepted (false: the node is shutting down or its mailbox is full).
func (nr *nodeRuntime) enqueue(fn func()) bool {
	select {
	case <-nr.quit:
		return false
	case nr.mailbox <- fn:
		return true
	default:
		nr.dropped.Add(1)
		return false
	}
}

func (nr *nodeRuntime) run() {
	defer close(nr.done)
	for {
		select {
		case <-nr.quit:
			return
		case fn := <-nr.mailbox:
			fn()
			nr.processed.Add(1)
		}
	}
}
