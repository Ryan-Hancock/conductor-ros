package conductor

import (
	"strings"
	"sync/atomic"
	"testing"
	"time"
)

// A three-stage pipeline: a timer publishes on "raw", the middle node
// republishes on "cooked", the last consumes it. All of that work should end
// up in one trace, because trace context rides along with the messages.
type traceSource struct {
	Tick Timer     `rate:"200hz"`
	Out  Pub[ping] `topic:"raw"`
}

func (t *traceSource) OnTick() { t.Out.Publish(ping{V: 1}) }

type traceMiddle struct {
	In  Sub[ping] `topic:"raw"`
	Out Pub[ping] `topic:"cooked"`
}

func (t *traceMiddle) OnIn(p ping) { t.Out.Publish(ping{V: p.V + 1}) }

type traceSink struct {
	In Sub[ping] `topic:"cooked"`

	got atomic.Int64
}

func (t *traceSink) OnIn(p ping) { t.got.Add(1) }

func TestTracePropagation(t *testing.T) {
	rec := &SpanRecorder{}
	AddExporter(rec)
	t.Cleanup(func() {
		traceMu.Lock()
		exporters = nil
		traceMu.Unlock()
	})

	sink := &traceSink{}
	newTestApp(t, &traceSource{}, &traceMiddle{}, sink)
	time.Sleep(150 * time.Millisecond)
	if sink.got.Load() == 0 {
		t.Fatal("pipeline produced nothing")
	}

	spans := rec.Spans()
	byKind := map[SpanKind][]Span{}
	for _, s := range spans {
		byKind[s.Kind] = append(byKind[s.Kind], s)
	}
	if len(byKind[SpanTimer]) == 0 || len(byKind[SpanSubscription]) == 0 {
		t.Fatalf("missing spans: %d timer, %d subscription", len(byKind[SpanTimer]), len(byKind[SpanSubscription]))
	}

	// Find a subscription span on "cooked" and walk back to the timer span
	// that started it: same trace, chained parents, three distinct nodes.
	var sinkSpan *Span
	for i := range spans {
		if spans[i].Kind == SpanSubscription && spans[i].Name == "cooked" {
			sinkSpan = &spans[i]
			break
		}
	}
	if sinkSpan == nil {
		t.Fatal("no subscription span for the cooked topic")
	}
	byID := map[[8]byte]Span{}
	for _, s := range spans {
		byID[s.Context.SpanID] = s
	}
	parent, ok := byID[sinkSpan.ParentID]
	if !ok {
		t.Fatal("sink span's parent was not recorded")
	}
	if parent.Node != "trace_middle" || parent.Name != "raw" {
		t.Errorf("parent span = %s/%s, want trace_middle/raw", parent.Node, parent.Name)
	}
	grandparent, ok := byID[parent.ParentID]
	if !ok {
		t.Fatal("middle span's parent was not recorded")
	}
	if grandparent.Node != "trace_source" || grandparent.Kind != SpanTimer {
		t.Errorf("grandparent span = %s/%s, want the trace_source timer", grandparent.Node, grandparent.Kind)
	}
	// The whole chain shares one trace id.
	if sinkSpan.Context.TraceID != grandparent.Context.TraceID {
		t.Error("spans in one causal chain must share a trace id")
	}
	if len(sinkSpan.Context.Traceparent()) != 55 {
		t.Errorf("traceparent %q is not W3C-shaped", sinkSpan.Context.Traceparent())
	}
}

func TestMetricsExposition(t *testing.T) {
	sink := &traceSink{}
	newTestApp(t, &traceSource{}, &traceMiddle{}, sink)
	time.Sleep(100 * time.Millisecond)

	text := MetricsText()
	for _, want := range []string{
		`conductor_messages_published_total{node="trace_source",topic="raw"}`,
		`conductor_messages_received_total{node="trace_middle",topic="raw"}`,
		`conductor_callback_duration_count{`,
		`conductor_callback_duration_sum_seconds{`,
	} {
		if !strings.Contains(text, want) {
			t.Errorf("metrics missing %s\n---\n%s", want, text)
		}
	}
	// Prometheus text format: every line is "name value" or "name{labels} value".
	for _, line := range strings.Split(strings.TrimSpace(text), "\n") {
		if strings.Count(line, " ") == 0 {
			t.Errorf("malformed metrics line: %q", line)
		}
	}
}

func TestTraceContextTraceparent(t *testing.T) {
	tc := TraceContext{TraceID: [16]byte{1}, SpanID: [8]byte{2}, Sampled: true}
	if got := tc.Traceparent(); got != "00-01000000000000000000000000000000-0200000000000000-01" {
		t.Errorf("traceparent = %s", got)
	}
	if (TraceContext{}).Valid() {
		t.Error("zero trace context must not be valid")
	}
}
