package conductor

import (
	"crypto/rand"
	"encoding/hex"
	"log/slog"
	"sync"
	"time"
)

// Distributed tracing for robots. A ROS graph is a causal chain — a sensor
// message triggers a filter callback, which publishes, which triggers a
// planner — but nothing in the ecosystem records that chain, so "why did the
// robot stop 200ms after that lidar frame?" is answered by correlating log
// timestamps by hand.
//
// Conductor gives every callback a span, and propagates trace context along
// the messages themselves, so a trace covers the full path of a message
// across nodes and processes. Propagation is automatic: a message published
// from inside a callback is a child of that callback's span.

// TraceContext identifies one trace and the span within it that caused the
// current work. It matches the W3C trace-context fields.
type TraceContext struct {
	TraceID [16]byte
	SpanID  [8]byte
	Sampled bool
}

// Valid reports whether the context refers to a real trace.
func (tc TraceContext) Valid() bool { return tc.TraceID != [16]byte{} }

// TraceIDString returns the trace id as hex, the form to grep logs by.
func (tc TraceContext) TraceIDString() string { return hex.EncodeToString(tc.TraceID[:]) }

// SpanIDString returns the span id as hex.
func (tc TraceContext) SpanIDString() string { return hex.EncodeToString(tc.SpanID[:]) }

// Traceparent renders the context in W3C traceparent form, which OpenTelemetry
// and most tracing backends accept directly.
func (tc TraceContext) Traceparent() string {
	flags := "00"
	if tc.Sampled {
		flags = "01"
	}
	return "00-" + tc.TraceIDString() + "-" + tc.SpanIDString() + "-" + flags
}

func newTraceID() (id [16]byte) {
	rand.Read(id[:])
	return id
}

func newSpanID() (id [8]byte) {
	rand.Read(id[:])
	return id
}

// SpanKind describes what produced a span.
type SpanKind string

const (
	SpanSubscription SpanKind = "subscription"
	SpanTimer        SpanKind = "timer"
	SpanService      SpanKind = "service"
	SpanAction       SpanKind = "action"
	SpanLifecycle    SpanKind = "lifecycle"
	SpanStep         SpanKind = "step"
)

// Span is one unit of traced work: a single callback invocation.
type Span struct {
	Context  TraceContext
	ParentID [8]byte
	Node     string
	Kind     SpanKind
	Name     string // topic, service, or action name
	Start    time.Time
	Duration time.Duration
	Err      error
}

// Exporter receives finished spans. Exporters must be safe for concurrent
// use and must not block: spans are exported from callback goroutines.
type Exporter interface {
	Export(Span)
}

var (
	traceMu   sync.RWMutex
	exporters []Exporter
)

// AddExporter registers a span exporter. With no exporters registered,
// tracing costs one atomic check per callback.
func AddExporter(e Exporter) {
	traceMu.Lock()
	defer traceMu.Unlock()
	exporters = append(exporters, e)
}

func tracingEnabled() bool {
	traceMu.RLock()
	defer traceMu.RUnlock()
	return len(exporters) > 0
}

func exportSpan(s Span) {
	traceMu.RLock()
	defer traceMu.RUnlock()
	for _, e := range exporters {
		e.Export(s)
	}
}

// startSpan begins a span for a callback. parent is the context the incoming
// message carried, if any; when it is invalid a new trace is started.
func startSpan(node string, kind SpanKind, name string, parent TraceContext) *Span {
	s := &Span{Node: node, Kind: kind, Name: name, Start: time.Now()}
	if parent.Valid() {
		s.Context.TraceID = parent.TraceID
		s.ParentID = parent.SpanID
		s.Context.Sampled = parent.Sampled
	} else {
		s.Context.TraceID = newTraceID()
		s.Context.Sampled = true
	}
	s.Context.SpanID = newSpanID()
	return s
}

func (s *Span) finish(err error) {
	if s == nil {
		return
	}
	s.Duration = time.Since(s.Start)
	s.Err = err
	exportSpan(*s)
}

// LogExporter writes finished spans to the structured log, including the
// W3C traceparent, so traces are greppable without a tracing backend. It is
// what -trace enables.
type LogExporter struct{}

func (LogExporter) Export(s Span) {
	slog.Info("conductor: span",
		"trace", s.Context.TraceIDString(),
		"span", s.Context.SpanIDString(),
		"parent", hex.EncodeToString(s.ParentID[:]),
		"node", s.Node,
		"kind", string(s.Kind),
		"name", s.Name,
		"duration", s.Duration.String(),
		"traceparent", s.Context.Traceparent(),
	)
}

// SpanRecorder is an Exporter that keeps spans in memory, for tests and for
// the trace view of small applications.
type SpanRecorder struct {
	mu    sync.Mutex
	spans []Span
}

func (r *SpanRecorder) Export(s Span) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.spans = append(r.spans, s)
}

// Spans returns a copy of the recorded spans.
func (r *SpanRecorder) Spans() []Span {
	r.mu.Lock()
	defer r.mu.Unlock()
	out := make([]Span, len(r.spans))
	copy(out, r.spans)
	return out
}

// Reset discards recorded spans.
func (r *SpanRecorder) Reset() {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.spans = nil
}
