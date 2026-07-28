package conductor

import "sync/atomic"

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
}

func newNodeRuntime(name string) *nodeRuntime {
	return &nodeRuntime{
		name:    name,
		mailbox: make(chan func(), 64),
		quit:    make(chan struct{}),
		done:    make(chan struct{}),
	}
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
