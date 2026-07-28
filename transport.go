package conductor

import (
	"fmt"
	"reflect"
	"sort"
	"strings"
	"sync"
	"time"
)

// TopicSpec describes one endpoint of a topic as declared by a node.
type TopicSpec struct {
	Topic string // topic name as declared, without leading slash
	QoS   QoS
	Type  reflect.Type // message struct type
	Node  string       // declaring node's name
}

// ServiceSpec describes a service endpoint (server or client) as declared
// by a node.
type ServiceSpec struct {
	Service string // service name as declared, without leading slash
	ReqType reflect.Type
	ResType reflect.Type
	Node    string // declaring node's name
	// Timeout is the longest a call on this client may take. Transports that
	// impose their own deadline at declaration time (zenoh queriers default
	// to 10s) must honour it; zero means the transport default. Ignored for
	// servers.
	Timeout time.Duration
}

// Metadata travels alongside a message. It carries the trace context that
// links a publish to the callbacks it causes, so a trace spans the whole
// path of a message through the graph. Transports that cannot carry it may
// drop it; traces then start fresh at each hop.
type Metadata struct {
	Trace TraceContext
}

// Transport routes messages between nodes. "inproc" (the default) delivers
// over an internal bus; other transports register themselves via
// RegisterTransport from their package's init — e.g. importing
// conductor.dev/conductor/transport/zenoh (built with -tags zenoh) registers
// "zenoh", which joins a live ROS 2 graph.
type Transport interface {
	// DeclareNode is called once per node, before any of its endpoints.
	DeclareNode(name string) error
	// Publisher declares a publisher and returns its publish function.
	Publisher(spec TopicSpec) (func(msg any, md Metadata) error, error)
	// Subscribe declares a subscription. deliver may be called from any
	// goroutine; the runtime routes it onto the node's executor.
	Subscribe(spec TopicSpec, deliver func(msg any, md Metadata)) error
	// Serve declares a service server. handle may be called from any
	// goroutine and blocks until the response is ready (the runtime routes
	// execution onto the node's executor).
	Serve(spec ServiceSpec, handle func(req any) (any, error)) error
	// ServiceClient declares a service client and returns its call function.
	ServiceClient(spec ServiceSpec) (func(req any, timeout time.Duration) (any, error), error)
	// Start is called after all declarations, before the app runs.
	Start() error
	Close() error
}

// TransportOptions carries transport configuration from Run's flags.
type TransportOptions struct {
	Endpoint string // transport-specific endpoint (zenoh: router endpoint)
	Domain   int    // ROS domain id
}

var (
	transportMu        sync.RWMutex
	transportFactories = map[string]func(TransportOptions) (Transport, error){}
)

// RegisterTransport makes a transport selectable via the -transport flag.
func RegisterTransport(name string, factory func(TransportOptions) (Transport, error)) {
	transportMu.Lock()
	defer transportMu.Unlock()
	transportFactories[name] = factory
}

func newTransport(name string, opts TransportOptions) (Transport, error) {
	transportMu.RLock()
	factory, ok := transportFactories[name]
	var known []string
	for n := range transportFactories {
		known = append(known, n)
	}
	transportMu.RUnlock()
	if !ok {
		sort.Strings(known)
		return nil, fmt.Errorf("unknown transport %q (registered: %s); the zenoh transport requires importing conductor.dev/conductor/transport/zenoh and building with -tags zenoh",
			name, strings.Join(known, ", "))
	}
	return factory(opts)
}
