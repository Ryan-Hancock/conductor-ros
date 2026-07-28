package conductor

import (
	"fmt"
	"log/slog"
	"sync"
	"time"
)

func init() {
	RegisterTransport("inproc", func(TransportOptions) (Transport, error) {
		return newInproc(), nil
	})
}

// inproc is the default transport: an in-process bus that hands each
// published message to every subscriber's deliver function (which enqueues
// onto that node's executor). Messages are passed by value, never serialized.
type inproc struct {
	mu      sync.RWMutex
	subs    map[string][]func(any)
	pubs    map[string][]string
	servers map[string]func(any) (any, error)
}

func newInproc() *inproc {
	return &inproc{
		subs:    map[string][]func(any){},
		pubs:    map[string][]string{},
		servers: map[string]func(any) (any, error){},
	}
}

func (b *inproc) DeclareNode(string) error { return nil }

func (b *inproc) Publisher(spec TopicSpec) (func(any) error, error) {
	b.mu.Lock()
	b.pubs[spec.Topic] = append(b.pubs[spec.Topic], spec.Node)
	b.mu.Unlock()
	return func(msg any) error {
		b.mu.RLock()
		subs := b.subs[spec.Topic]
		b.mu.RUnlock()
		for _, deliver := range subs {
			deliver(msg)
		}
		return nil
	}, nil
}

func (b *inproc) Subscribe(spec TopicSpec, deliver func(any)) error {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.subs[spec.Topic] = append(b.subs[spec.Topic], deliver)
	return nil
}

func (b *inproc) Serve(spec ServiceSpec, handle func(any) (any, error)) error {
	b.mu.Lock()
	defer b.mu.Unlock()
	if _, taken := b.servers[spec.Service]; taken {
		return fmt.Errorf("service %q already has a server", spec.Service)
	}
	b.servers[spec.Service] = handle
	return nil
}

func (b *inproc) ServiceClient(spec ServiceSpec) (func(any, time.Duration) (any, error), error) {
	// The server is looked up per call, so declaration order does not matter.
	return func(req any, timeout time.Duration) (any, error) {
		b.mu.RLock()
		handle := b.servers[spec.Service]
		b.mu.RUnlock()
		if handle == nil {
			return nil, fmt.Errorf("service %q has no in-process server", spec.Service)
		}
		type result struct {
			res any
			err error
		}
		ch := make(chan result, 1)
		go func() {
			res, err := handle(req)
			ch <- result{res, err}
		}()
		select {
		case r := <-ch:
			return r.res, r.err
		case <-time.After(timeout):
			return nil, fmt.Errorf("service %q: no response within %s", spec.Service, timeout)
		}
	}, nil
}

// Start warns about subscriptions nothing publishes to — the same check the
// CLI performs statically, as a runtime safety net. Only meaningful in-proc:
// on a networked transport the publisher may live in another process.
func (b *inproc) Start() error {
	b.mu.RLock()
	defer b.mu.RUnlock()
	for topic := range b.subs {
		if len(b.pubs[topic]) == 0 {
			slog.Warn("conductor: subscription has no in-process publisher (external node? run with -transport zenoh to join a live ROS graph)", "topic", topic)
		}
	}
	return nil
}

func (b *inproc) Close() error { return nil }
